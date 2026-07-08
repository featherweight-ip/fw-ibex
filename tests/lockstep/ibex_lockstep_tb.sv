// ======================================================================
// P5 RVFI lockstep: Ibex SPL model vs. the lowRISC Ibex RTL.
//
// Both cores execute the SAME program image out of their own memories and emit
// an RVFI retirement stream. The SPL stream feeds a `ref` collector directly
// (its rvfi_if port); the RTL stream is sampled off ibex_top's rvfi_* pins by
// the fw-proto-rvfi monitor xtor, bridged into a `dut` collector. The
// fwvip-rvfi scoreboard then diffs the two streams field-by-field, pairing by
// `order`. This is the primary correctness gate for the model.
//
// Scope: straight-line RV32IMC -- RV32IM ALU/mem/branch/jump plus an RV32C block
// (compressed ALU + SP-relative load/store, incl. a word-straddling 32-bit insn).
// No CSR accesses and no traps/interrupts yet; those arrive in P5.3 once the trap
// RVFI conventions are reconciled against the RTL.
//
// Run:  dfm run fw-ibex.lockstep      (expect: [ibex_lockstep] PASS)
// ======================================================================
`include "fw_hdl_macros.svh"

module ibex_lockstep_tb;
    import fw_hdl_pkg::*;
    import fw_std_pkg::*;
    import fw_mem_pkg::*;
    import fw_proto_rvfi_pkg::*;    // rvfi_if + rvfi_meta_t + monitor bridge
    import fwvip_rvfi_pkg::*;       // rvfi_stream_collector + rvfi_scoreboard
    import ibex_spl_pkg::*;
    `include "fw_proto_rvfi_macros.svh"   // RVFI_WIRES / RVFI_CONNECT

    typedef logic [31:0] addr_t;
    typedef logic [31:0] data_t;
    typedef logic [3:0]  strb_t;

    // Ibex reset PC = {boot_addr_i[31:8], 8'h80}; with boot=0 the first fetch
    // (and the program image) sits at 0x80. Data + tohost live below 0x800 so
    // x0-relative (signed 12-bit) stores can reach them.
    localparam addr_t BOOT     = 32'h0000_0000;
    localparam addr_t RESET_PC = 32'h0000_0080;
    localparam addr_t SCRATCH  = 32'h0000_0400;
    localparam addr_t TOHOST   = 32'h0000_07F0;

    // ------------------------------------------------------------------
    // SPL environment: the core, its memory, and the reference collector.
    // The static assembler + program builder live here so the RTL side can
    // load the identical image (single source of truth).
    // ------------------------------------------------------------------
    class lockstep_env extends fw_component;
        ibex_core                                  core;
        fw_mem_flat #(addr_t, data_t, strb_t)      mem;
        rvfi_stream_collector                      ref_c;

        function new(string n, fw_component p); super.new(n, p); endfunction

        // ---- assembler ------------------------------------------------
        static function data_t r_type(logic [6:0] op, logic [2:0] f3, logic [6:0] f7,
                                      int rd, int rs1, int rs2);
            return {f7, rs2[4:0], rs1[4:0], f3, rd[4:0], op};
        endfunction
        static function data_t i_type(logic [6:0] op, logic [2:0] f3,
                                      int rd, int rs1, logic signed [31:0] imm);
            return {imm[11:0], rs1[4:0], f3, rd[4:0], op};
        endfunction
        static function data_t s_type(logic [6:0] op, logic [2:0] f3,
                                      int rs1, int rs2, logic signed [31:0] imm);
            return {imm[11:5], rs2[4:0], rs1[4:0], f3, imm[4:0], op};
        endfunction
        static function data_t b_type(logic [2:0] f3, int rs1, int rs2,
                                      logic signed [31:0] imm);
            return {imm[12], imm[10:5], rs2[4:0], rs1[4:0], f3,
                    imm[4:1], imm[11], 7'h63};
        endfunction
        static function data_t u_type(logic [6:0] op, int rd, logic [31:0] imm20);
            return {imm20[19:0], rd[4:0], op};
        endfunction
        static function data_t j_type(int rd, logic signed [31:0] imm);
            return {imm[20], imm[10:1], imm[11], imm[19:12], rd[4:0], 7'h6F};
        endfunction

        static function data_t ADDI(int rd,int rs1,int im); return i_type(7'h13,3'h0,rd,rs1,im); endfunction
        static function data_t ADD (int rd,int rs1,int rs2);return r_type(7'h33,3'h0,7'h00,rd,rs1,rs2); endfunction
        static function data_t SUB (int rd,int rs1,int rs2);return r_type(7'h33,3'h0,7'h20,rd,rs1,rs2); endfunction
        static function data_t XOR (int rd,int rs1,int rs2);return r_type(7'h33,3'h4,7'h00,rd,rs1,rs2); endfunction
        static function data_t OR  (int rd,int rs1,int rs2);return r_type(7'h33,3'h6,7'h00,rd,rs1,rs2); endfunction
        static function data_t AND_(int rd,int rs1,int rs2);return r_type(7'h33,3'h7,7'h00,rd,rs1,rs2); endfunction
        static function data_t SLT (int rd,int rs1,int rs2);return r_type(7'h33,3'h2,7'h00,rd,rs1,rs2); endfunction
        static function data_t SLTU(int rd,int rs1,int rs2);return r_type(7'h33,3'h3,7'h00,rd,rs1,rs2); endfunction
        static function data_t SLLI(int rd,int rs1,int sh); return i_type(7'h13,3'h1,rd,rs1,sh & 32'h1f); endfunction
        static function data_t SRLI(int rd,int rs1,int sh); return i_type(7'h13,3'h5,rd,rs1,sh & 32'h1f); endfunction
        static function data_t SRAI(int rd,int rs1,int sh); return i_type(7'h13,3'h5,rd,rs1,32'h400 | (sh & 32'h1f)); endfunction
        static function data_t LUI  (int rd,logic [31:0] im);return u_type(7'h37,rd,im); endfunction
        static function data_t AUIPC(int rd,logic [31:0] im);return u_type(7'h17,rd,im); endfunction
        static function data_t MUL (int rd,int rs1,int rs2);return r_type(7'h33,3'h0,7'h01,rd,rs1,rs2); endfunction
        static function data_t DIV (int rd,int rs1,int rs2);return r_type(7'h33,3'h4,7'h01,rd,rs1,rs2); endfunction
        static function data_t REM (int rd,int rs1,int rs2);return r_type(7'h33,3'h6,7'h01,rd,rs1,rs2); endfunction
        static function data_t SW  (int rs2,int rs1,int im);return s_type(7'h23,3'h2,rs1,rs2,im); endfunction
        static function data_t LW  (int rd,int rs1,int im); return i_type(7'h03,3'h2,rd,rs1,im); endfunction
        static function data_t BEQ (int rs1,int rs2,int im);return b_type(3'h0,rs1,rs2,im); endfunction
        static function data_t BNE (int rs1,int rs2,int im);return b_type(3'h1,rs1,rs2,im); endfunction
        static function data_t JAL (int rd,int im);         return j_type(rd,im); endfunction
        static function data_t ECALL();                     return 32'h0000_0073; endfunction

        // ---- compressed (RV32C / Zca) encoders -> 16-bit ------------------------
        static function logic [15:0] CLI  (int rd, logic signed [31:0] im);
            return {3'b010, im[5], rd[4:0], im[4:0], 2'b01};
        endfunction
        static function logic [15:0] CADDI(int rd, logic signed [31:0] im);
            return {3'b000, im[5], rd[4:0], im[4:0], 2'b01};
        endfunction
        static function logic [15:0] CMV  (int rd, int rs2);
            return {4'b1000, rd[4:0], rs2[4:0], 2'b10};
        endfunction
        static function logic [15:0] CADD (int rd, int rs2);
            return {4'b1001, rd[4:0], rs2[4:0], 2'b10};
        endfunction
        static function logic [15:0] CSLLI(int rd, int sh);
            return {3'b000, 1'b0, rd[4:0], sh[4:0], 2'b10};   // RV32: shamt[5]=0
        endfunction
        // SP-relative (x2) word store/load; uimm is a *4-scaled byte offset.
        static function logic [15:0] CSWSP(int rs2, logic [7:0] uimm);
            return {3'b110, uimm[5:2], uimm[7:6], rs2[4:0], 2'b10};
        endfunction
        static function logic [15:0] CLWSP(int rd, logic [7:0] uimm);
            return {3'b010, uimm[5], rd[4:0], uimm[4:2], uimm[7:6], 2'b10};
        endfunction

        // Emit onto a 16-bit halfword stream (memory is little-endian).
        static function void e16(ref logic [15:0] hw[$], input logic [15:0] c);
            hw.push_back(c);
        endfunction
        static function void e32(ref logic [15:0] hw[$], input data_t w);
            hw.push_back(w[15:0]);
            hw.push_back(w[31:16]);
        endfunction

        // Deterministic RV32IMC program: the RV32IM straight-line core (identical
        // coverage to P5.1) followed by an RV32C block (compressed ALU + SP-relative
        // load/store), so the stream exercises compressed retirement -- where the
        // RTL reports rvfi_insn as the 16-bit encoding, not the expanded form. Built
        // as a halfword stream and packed into 32-bit words: the odd-length compressed
        // block makes the terminating 32-bit store straddle a word boundary (both the
        // SPL fetch buffer and the RTL prefetch handle that).
        static function void build_prog(ref data_t p[$]);
            logic [15:0] hw[$];
            // ---- RV32IM core (branch/jump offsets are self-contained here) ----
            e32(hw, ADDI(1,0,5));
            e32(hw, ADDI(2,0,37));
            e32(hw, ADD (3,1,2));
            e32(hw, SUB (4,2,1));
            e32(hw, XOR (5,1,2));
            e32(hw, OR  (6,1,2));
            e32(hw, AND_(7,1,2));
            e32(hw, SLLI(8,1,3));
            e32(hw, SRLI(9,2,1));
            e32(hw, SRAI(10,2,1));
            e32(hw, SLT (11,1,2));
            e32(hw, SLTU(12,2,1));
            e32(hw, LUI (13,32'h12345));
            e32(hw, AUIPC(14,32'h1));
            e32(hw, MUL (15,1,2));
            e32(hw, DIV (16,2,1));
            e32(hw, REM (17,2,1));
            e32(hw, SW  (3,0,SCRATCH));      // [0x400]=42
            e32(hw, LW  (18,0,SCRATCH));     // x18=42
            e32(hw, BEQ (1,2,8));            // not taken
            e32(hw, BNE (1,2,8));            // taken -> skip next
            e32(hw, ADDI(19,0,1));           // skipped
            e32(hw, JAL (20,8));             // -> skip next (link x20)
            e32(hw, ADDI(21,0,1));           // skipped
            e32(hw, ADDI(22,0,1));           // x22=1 (tohost value); JAL landing
            // ---- RV32C block (x2 re-based to an aligned scratch for C.SWSP/LWSP) --
            e32(hw, ADDI(2,0,SCRATCH+16));   // x2 = 0x410 (word-aligned SP base)
            e16(hw, CLI  (8, 5));            // x8 = 5
            e16(hw, CADDI(8, 1));            // x8 = 6
            e16(hw, CLI  (9, 9));            // x9 = 9
            e16(hw, CMV  (10, 8));           // x10 = 6
            e16(hw, CADD (10, 9));           // x10 = 15
            e16(hw, CSLLI(10, 2));           // x10 = 60
            e16(hw, CMV  (14, 10));          // x14 = 60  (odd count -> next straddles)
            e16(hw, CSWSP(8, 8'd0));         // [x2+0]=6  (compressed store)
            e16(hw, CLWSP(12, 8'd0));        // x12 = 6   (compressed load)
            // ---- terminate via a synchronous exception ----
            e32(hw, ECALL());                // ECALL(M) -> trap to mtvec base (0x0)
            e32(hw, JAL (0,0));              // not reached (trap redirects); pad

            // Pack the halfword stream into 32-bit words (pad a trailing halfword).
            if (hw.size() % 2 == 1) hw.push_back(16'h0000);
            p = {};
            for (int k = 0; k < hw.size(); k += 2)
                p.push_back({hw[k+1], hw[k]});
        endfunction

        // Trap handler at the mtvec base (0x0): store the pass value (x22) to
        // TOHOST -- which halts the SPL and is retired by the RTL -- then park on
        // a self-loop. The ECALL that vectors here is a synchronous exception, so
        // both cores retire it with rvfi_trap=1 at the ECALL's own pc.
        static function void build_handler(ref data_t p[$]);
            logic [15:0] hw[$];
            e32(hw, SW  (22,0,TOHOST));
            e32(hw, JAL (0,0));
            if (hw.size() % 2 == 1) hw.push_back(16'h0000);
            p = {};
            for (int k = 0; k < hw.size(); k += 2)
                p.push_back({hw[k+1], hw[k]});
        endfunction

        function void build();
            data_t prog[$], hnd[$];
            core  = new("core", this, RESET_PC, BOOT, 32'h0, 32);
            mem   = new("mem",  this);
            ref_c = new("ref");
            build_prog(prog);
            build_handler(hnd);
            mem.load(RESET_PC, prog);   // main at reset PC (0x80)
            mem.load(BOOT,     hnd);    // handler at mtvec base (0x0)
        endfunction

        function void connect();
            core.hart.imem.connect(mem.mem_if);
            core.hart.dmem.connect(mem.mem_if);
            core.hart.rvfi.connect(ref_c.ap);
            core.hart.set_tohost(TOHOST);
        endfunction
    endclass

    // ------------------------------------------------------------------
    // Clock / reset. `reset` is active-high (fw_root + monitor convention);
    // the RTL takes active-low rst_ni. run_clk gates the clock so it can be
    // frozen the instant the RTL stream reaches N records (prevents a
    // post-tohost illegal fetch from retiring an extra record).
    // ------------------------------------------------------------------
    logic clock     = 1'b0;
    logic reset     = 1'b1;    // fw_root + rvfi monitor (active high)
    logic rtl_rst_n = 1'b1;    // Ibex reset (active low)
    bit   run_clk   = 1'b1;
    wire  rst_ni    = rtl_rst_n;
    always begin #5ns; if (run_clk) clock = ~clock; end

    // ---- RTL pin nets --------------------------------------------------
    wire        instr_req, instr_gnt, instr_rvalid, instr_err;
    wire [31:0] instr_addr, instr_rdata;
    wire        data_req, data_gnt, data_rvalid, data_we, data_err;
    wire [3:0]  data_be;
    wire [31:0] data_addr, data_wdata, data_rdata;

    `RVFI_WIRES(rvfi_)

    // Ibex drives rvfi_mem_rmask/wmask (and rvfi_mem_addr) for EVERY retiring
    // instruction from the LSU byte-type decode + ALU adder result -- it does
    // NOT gate them by whether the instruction actually accesses memory
    // (ibex_core.sv:1652, lsu_type defaults to 00 -> mask 0xF). The RVFI spec
    // (and the fw-proto-rvfi monitor's kind-selection) expect these zero for
    // non-memory ops, so gate the masks by the real opcode before the monitor.
    // rvfi_insn carries the ORIGINAL encoding, so this must recognize both the
    // 32-bit and the compressed (RV32C) load/store forms.
    function automatic bit rvfi_is_load(input logic [31:0] iw);
        if (iw[1:0] == 2'b11) return (iw[6:0] == 7'b000_0011);            // 32b LW/LB/...
        if (iw[1:0] == 2'b00 && iw[15:13] == 3'b010) return 1'b1;         // C.LW
        if (iw[1:0] == 2'b10 && iw[15:13] == 3'b010) return 1'b1;         // C.LWSP
        return 1'b0;
    endfunction
    function automatic bit rvfi_is_store(input logic [31:0] iw);
        if (iw[1:0] == 2'b11) return (iw[6:0] == 7'b010_0011);            // 32b SW/SB/...
        if (iw[1:0] == 2'b00 && iw[15:13] == 3'b110) return 1'b1;         // C.SW
        if (iw[1:0] == 2'b10 && iw[15:13] == 3'b110) return 1'b1;         // C.SWSP
        return 1'b0;
    endfunction

    wire [3:0] rvfi_mem_rmask_raw;
    wire [3:0] rvfi_mem_wmask_raw;
    assign rvfi_mem_rmask = rvfi_is_load(rvfi_insn)  ? rvfi_mem_rmask_raw : 4'b0;
    assign rvfi_mem_wmask = rvfi_is_store(rvfi_insn) ? rvfi_mem_wmask_raw : 4'b0;

    // ---- Ibex RTL ------------------------------------------------------
    ibex_top #(
        .RV32E  (1'b0),
        .RV32M  (ibex_pkg::RV32MFast),
        .RV32B  (ibex_pkg::RV32BNone),
        .RV32ZC (ibex_pkg::RV32Zca),   // base RV32C only (matches the SPL decompressor)
        .RegFile(ibex_pkg::RegFileFF)
    ) u_rtl (
        .clk_i (clock),
        .rst_ni(rst_ni),

        .test_en_i             (1'b0),
        .ram_cfg_icache_tag_i  ('0),
        .ram_cfg_icache_tag_o  (),
        .ram_cfg_icache_data_i ('0),
        .ram_cfg_icache_data_o (),

        .hart_id_i  (32'h0),
        .boot_addr_i(BOOT),

        .instr_req_o       (instr_req),
        .instr_gnt_i       (instr_gnt),
        .instr_rvalid_i    (instr_rvalid),
        .instr_addr_o      (instr_addr),
        .instr_rdata_i     (instr_rdata),
        .instr_rdata_intg_i(7'h0),
        .instr_err_i       (instr_err),

        .data_req_o       (data_req),
        .data_gnt_i       (data_gnt),
        .data_rvalid_i    (data_rvalid),
        .data_we_o        (data_we),
        .data_be_o        (data_be),
        .data_addr_o      (data_addr),
        .data_wdata_o     (data_wdata),
        .data_wdata_intg_o(),
        .data_rdata_i     (data_rdata),
        .data_rdata_intg_i(7'h0),
        .data_err_i       (data_err),

        .irq_software_i(1'b0),
        .irq_timer_i   (1'b0),
        .irq_external_i(1'b0),
        .irq_fast_i    (15'h0),
        .irq_nm_i      (1'b0),

        .scramble_key_valid_i(1'b0),
        .scramble_key_i      (128'h0),
        .scramble_nonce_i    (64'h0),
        .scramble_req_o      (),

        .debug_req_i        (1'b0),
        .crash_dump_o       (),
        .double_fault_seen_o(),

        .rvfi_valid    (rvfi_valid),
        .rvfi_order    (rvfi_order),
        .rvfi_insn     (rvfi_insn),
        .rvfi_trap     (rvfi_trap),
        .rvfi_halt     (rvfi_halt),
        .rvfi_intr     (rvfi_intr),
        .rvfi_mode     (rvfi_mode),
        .rvfi_ixl      (),
        .rvfi_rs1_addr (rvfi_rs1_addr),
        .rvfi_rs2_addr (rvfi_rs2_addr),
        .rvfi_rs3_addr (),
        .rvfi_rs1_rdata(rvfi_rs1_rdata),
        .rvfi_rs2_rdata(rvfi_rs2_rdata),
        .rvfi_rs3_rdata(),
        .rvfi_rd_addr  (rvfi_rd_addr),
        .rvfi_rd_wdata (rvfi_rd_wdata),
        .rvfi_pc_rdata (rvfi_pc_rdata),
        .rvfi_pc_wdata (rvfi_pc_wdata),
        .rvfi_mem_addr (rvfi_mem_addr),
        .rvfi_mem_rmask(rvfi_mem_rmask_raw),
        .rvfi_mem_wmask(rvfi_mem_wmask_raw),
        .rvfi_mem_rdata(rvfi_mem_rdata),
        .rvfi_mem_wdata(rvfi_mem_wdata),
        .rvfi_ext_pre_mip            (),
        .rvfi_ext_post_mip           (),
        .rvfi_ext_nmi               (),
        .rvfi_ext_nmi_int           (),
        .rvfi_ext_debug_req         (),
        .rvfi_ext_debug_mode        (),
        .rvfi_ext_rf_wr_suppress    (),
        .rvfi_ext_mcycle            (),
        .rvfi_ext_mhpmcounters      (),
        .rvfi_ext_mhpmcountersh     (),
        .rvfi_ext_ic_scr_key_valid  (),
        .rvfi_ext_irq_valid         (),
        .rvfi_ext_expanded_insn_valid(),
        .rvfi_ext_expanded_insn     (),
        .rvfi_ext_expanded_insn_last(),

        .fetch_enable_i       (ibex_pkg::IbexMuBiOn),
        .mcounteren_writable_i(ibex_pkg::IbexMuBiOff),
        .alert_minor_o         (),
        .alert_major_internal_o(),
        .alert_major_bus_o     (),
        .core_sleep_o          (),

        .scan_rst_ni(1'b1),

        .lockstep_cmp_en_o(),

        .data_req_shadow_o       (),
        .data_we_shadow_o        (),
        .data_be_shadow_o        (),
        .data_addr_shadow_o      (),
        .data_wdata_shadow_o     (),
        .data_wdata_intg_shadow_o(),
        .instr_req_shadow_o      (),
        .instr_addr_shadow_o     ()
    );

    // ---- RTL memory (shares the program image with the SPL fw_mem_flat) --
    ibex_pin_mem u_rtlmem (
        .clk_i (clock),
        .rst_ni(rst_ni),
        .instr_req_i   (instr_req),
        .instr_gnt_o   (instr_gnt),
        .instr_rvalid_o(instr_rvalid),
        .instr_addr_i  (instr_addr),
        .instr_rdata_o (instr_rdata),
        .instr_err_o   (instr_err),
        .data_req_i    (data_req),
        .data_gnt_o    (data_gnt),
        .data_rvalid_o (data_rvalid),
        .data_we_i     (data_we),
        .data_be_i     (data_be),
        .data_addr_i   (data_addr),
        .data_wdata_i  (data_wdata),
        .data_rdata_o  (data_rdata),
        .data_err_o    (data_err)
    );

    // ---- RTL rvfi monitor xtor (pins -> u_if -> bridge -> dut collector) --
    rvfi_monitor_xtor mon (
        .clock(clock), .reset(reset),
        `RVFI_CONNECT(rvfi_, rvfi_)
    );

    // ---- SPL root ------------------------------------------------------
    `fw_root_begin(lockstep_env, u_root, clock, reset)
    `fw_root_end

    // ---- dut side (RTL stream) + scoreboard ----------------------------
    rvfi_stream_collector   dut_c;
    rvfi_monitor_xtor_bridge mbr;
    rvfi_scoreboard          sb;

    initial begin
        lockstep_env e;
        data_t       prog[$];
        data_t       hnd[$];
        int          N;
        bit          pass;

        // Back-door load the RTL memory with the identical program image:
        // main at the reset PC (0x80) and the trap handler at the mtvec base (0x0).
        lockstep_env::build_prog(prog);
        foreach (prog[i])
            u_rtlmem.mem[(RESET_PC >> 2) + i] = prog[i];
        lockstep_env::build_handler(hnd);
        foreach (hnd[i])
            u_rtlmem.mem[(BOOT >> 2) + i] = hnd[i];

        // Bring up the RTL rvfi monitor bridge before releasing reset.
        dut_c = new("dut");
        mbr   = new(mon.u_if, dut_c.ap);
        mbr.start();

        // Assert reset. rtl_rst_n starts high, so driving it low here gives the
        // Ibex async-reset flops a real negedge -- without it, Verilator never
        // applies reset values that differ from zero-init (e.g. priv_lvl_q, which
        // resets to PRIV_LVL_M), and the core would run in U-mode.
        rtl_rst_n = 1'b0;
        reset     = 1'b1;
        repeat (6) @(posedge clock);
        reset     = 1'b0;
        rtl_rst_n = 1'b1;

        while (u_root.root == null) @(posedge clock);
        e = u_root.root;

        // SPL runs to completion (clockless service loop). Then wait for the
        // RTL stream to reach the same record count and check IMMEDIATELY --
        // freezing the clock so no post-tohost fetch can retire an extra one.
        while (e.core.hart.halt_code == HALT_NONE) @(posedge clock);
        N = e.ref_c.q.size();

        while (dut_c.q.size() < N) @(posedge clock);
        run_clk = 1'b0;   // freeze: no post-store retirement can be sampled now

        // Safety: the RTL parks on a self-loop after the tohost store, so its
        // first N records are exactly the shared program. Drop any trailing
        // park-loop retirements that slipped in before the freeze took effect.
        while (dut_c.q.size() > N) void'(dut_c.q.pop_back());

        // Convention reconciliation: Ibex numbers rvfi_order from 1 (its order
        // counter pre-increments, ibex_core.sv:1455), while the SPL model and the
        // RISC-V Formal spec number from 0. `order` is only the scoreboard's
        // pairing key, so re-base the reference stream onto Ibex's numbering.
        foreach (e.ref_c.q[k]) e.ref_c.q[k].order = e.ref_c.q[k].order + 1;

        sb   = new(e.ref_c, dut_c);
        pass = sb.check();

        if (pass && e.ref_c.q.size() == N && dut_c.q.size() == N)
            $display("[ibex_lockstep] PASS (%0d records)", N);
        else
            $display("[ibex_lockstep] FAIL (ref=%0d dut=%0d)",
                     e.ref_c.q.size(), dut_c.q.size());
        $finish;
    end

    // Watchdog.
    initial begin
        #2ms;
        $fatal(1, "[ibex_lockstep] TIMEOUT (ref=%0d dut=%0d)",
               (u_root.root == null) ? -1 : u_root.root.ref_c.q.size(),
               (dut_c == null) ? -1 : dut_c.q.size());
    end
endmodule

