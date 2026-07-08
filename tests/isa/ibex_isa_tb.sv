// ======================================================================
// Phase-3 ISA directed test: M extension + C extension.
//
// Three DUTs (each = ibex_core + fw_mem_flat RAM):
//   d_m    : RV32M mul/div/rem, incl. div-by-zero and signed-overflow edge cases.
//            Results are stored to a results area; the TB checks them via mem.peek.
//   d_c    : RV32C compressed instructions, built as a HALFWORD stream so a 32-bit
//            instruction lands at a 2-mod-4 address (word-straddling fetch).
//   d_cill : an illegal compressed encoding -> Illegal-Instruction trap; the handler
//            halts and the TB checks mcause / mtval (= zero-extended 16-bit).
//
// Run:  dfm run fw-ibex.isa      (expect: [ibex_isa] PASS)
// ======================================================================
`include "fw_hdl_macros.svh"

module ibex_isa_tb;
    import fw_hdl_pkg::*;
    import fw_std_pkg::*;
    import fw_mem_pkg::*;
    import ibex_spl_pkg::*;

    typedef logic [31:0] addr_t;
    typedef logic [31:0] data_t;
    typedef logic [3:0]  strb_t;

    localparam addr_t HANDLER = 32'h0000_0040;
    localparam addr_t TOHOST  = 32'h0000_0400;
    localparam addr_t RES     = 32'h0000_0100;   // results base (m test)

    // ---- 32-bit assembler ----------------------------------------------------
    function automatic data_t r_type(logic [6:0] op, logic [2:0] f3, logic [6:0] f7,
                                     logic [4:0] rd, logic [4:0] rs1, logic [4:0] rs2);
        return {f7, rs2, rs1, f3, rd, op};
    endfunction
    function automatic data_t i_type(logic [6:0] op, logic [2:0] f3,
                                     logic [4:0] rd, logic [4:0] rs1, logic signed [31:0] imm);
        return {imm[11:0], rs1, f3, rd, op};
    endfunction
    function automatic data_t s_type(logic [6:0] op, logic [2:0] f3,
                                     logic [4:0] rs1, logic [4:0] rs2, logic signed [31:0] imm);
        return {imm[11:5], rs2, rs1, f3, imm[4:0], op};
    endfunction
    function automatic data_t ADDI(int rd, int rs1, int imm);
        return i_type(7'h13, 3'b000, rd[4:0], rs1[4:0], imm);
    endfunction
    function automatic data_t LUI(int rd, int imm20);
        return {imm20[19:0], rd[4:0], 7'h37};
    endfunction
    function automatic data_t SW(int rs2, int rs1, int imm);
        return s_type(7'h23, 3'b010, rs1[4:0], rs2[4:0], imm);
    endfunction
    // RV32M: R-type, funct7 = 0000001.
    function automatic data_t M(logic [2:0] f3, int rd, int rs1, int rs2);
        return r_type(7'h33, f3, 7'b0000001, rd[4:0], rs1[4:0], rs2[4:0]);
    endfunction

    // ---- 16-bit (compressed) assembler ---------------------------------------
    function automatic logic [15:0] C_LI(int rd, int imm6);
        return {3'b010, imm6[5], rd[4:0], imm6[4:0], 2'b01};
    endfunction
    function automatic logic [15:0] C_ADDI(int rd, int imm6);
        return {3'b000, imm6[5], rd[4:0], imm6[4:0], 2'b01};
    endfunction
    function automatic logic [15:0] C_ADD(int rd, int rs2);
        return {3'b100, 1'b1, rd[4:0], rs2[4:0], 2'b10};
    endfunction
    function automatic logic [15:0] C_NOP();
        return 16'h0001;   // c.addi x0, 0
    endfunction

    // Pack a halfword stream into 32-bit little-endian words (low half first).
    function automatic void pack(ref data_t q[$], input logic [15:0] hs[$]);
        logic [15:0] h[$] = hs;
        q = {};
        if (h.size() % 2 == 1) h.push_back(C_NOP());   // pad to a whole word
        for (int i = 0; i < h.size(); i += 2)
            q.push_back({h[i+1], h[i]});
    endfunction

    // Push a 32-bit instruction into a halfword stream as two halves (low, high).
    function automatic void push32(ref logic [15:0] hs[$], input data_t insn);
        hs.push_back(insn[15:0]);
        hs.push_back(insn[31:16]);
    endfunction

    function automatic void compose(ref data_t q[$], input data_t main[$], input data_t handler[$]);
        q = {};
        foreach (main[i])    q.push_back(main[i]);
        while (q.size() < 16) q.push_back(ADDI(0, 0, 0));
        foreach (handler[i]) q.push_back(handler[i]);
    endfunction

    // ---- DUT (core + RAM) ----------------------------------------------------
    class dut extends fw_component;
        data_t     prog[$];
        ibex_core  core;
        fw_mem_flat #(addr_t, data_t, strb_t) mem;
        function new(string n, fw_component p); super.new(n, p); endfunction
        function void build();
            core = new("core", this, 32'h0, HANDLER, 32'h0, 32);
            mem  = new("mem",  this);
            mem.load(32'h0, prog);
        endfunction
        function void connect();
            core.hart.imem.connect(mem.mem_if);
            core.hart.dmem.connect(mem.mem_if);
            core.hart.set_tohost(TOHOST);
        endfunction
    endclass

    // ---- environment ---------------------------------------------------------
    class isa_env extends fw_component;
        dut d_m, d_c, d_cill;
        function new(string n, fw_component p); super.new(n, p); endfunction

        function void build();
            data_t   mprog[$];
            logic [15:0] hs[$];
            data_t   hh[$];

            d_m    = new("m",    this);
            d_c    = new("c",    this);
            d_cill = new("cill", this);

            // ---- M program: compute results, store to RES + i*4, halt -----------
            mprog = {};
            // MUL 7*6 = 42
            mprog = {mprog, ADDI(1,0,7), ADDI(2,0,6), M(3'b000,3,1,2), SW(3,0,RES+0)};
            // DIV -20/3 = -6 ; REM -20%3 = -2
            mprog = {mprog, ADDI(1,0,-20), ADDI(2,0,3),
                            M(3'b100,3,1,2), SW(3,0,RES+4),
                            M(3'b110,3,1,2), SW(3,0,RES+8)};
            // DIV 5/0 = -1 ; REMU 5/0 = 5
            mprog = {mprog, ADDI(1,0,5), ADDI(2,0,0),
                            M(3'b100,3,1,2), SW(3,0,RES+12),
                            M(3'b111,3,1,2), SW(3,0,RES+16)};
            // overflow: 0x80000000 / -1 = 0x80000000 ; % -1 = 0
            mprog = {mprog, LUI(1,32'h80000), ADDI(2,0,-1),
                            M(3'b100,3,1,2), SW(3,0,RES+20),
                            M(3'b110,3,1,2), SW(3,0,RES+24)};
            // MULHU(-1,-1) = 0xFFFFFFFE ; MULHSU(-1, 2) = 0xFFFFFFFF
            mprog = {mprog, ADDI(1,0,-1), M(3'b011,3,1,1), SW(3,0,RES+28),
                            ADDI(2,0,2),  M(3'b010,3,1,2), SW(3,0,RES+32)};
            // halt
            mprog = {mprog, ADDI(5,0,1), SW(5,0,TOHOST)};
            d_m.prog = mprog;

            // ---- C program: compressed ops + a straddling 32-bit store ----------
            // x8 = 5; x9 = 7; x8 += x9 (=12) in 3 halfwords -> next addr is 2-mod-4,
            // so the following 32-bit SW straddles a word boundary.
            hs = {};
            hs.push_back(C_LI(8, 5));         // @0x00
            hs.push_back(C_LI(9, 7));         // @0x02
            hs.push_back(C_ADD(8, 9));        // @0x04  x8 = 12
            push32(hs, SW(8, 0, RES+0));      // @0x06  STRADDLES 0x04/0x08
            hs.push_back(C_ADDI(8, 1));       // @0x0a  x8 = 13
            push32(hs, SW(8, 0, RES+4));      // @0x0c  aligned
            hs.push_back(C_LI(5, 1));         // @0x10
            push32(hs, SW(5, 0, TOHOST));     // @0x12  STRADDLES 0x10/0x14 -> halt
            pack(d_c.prog, hs);

            // ---- illegal compressed: half at 0x00 = 0x0000 (reserved C.ADDI4SPN) -
            hh = {ADDI(31,0,1), SW(31,0,TOHOST)};
            compose(d_cill.prog, '{32'h0000_0000}, hh);
        endfunction
    endclass

    // ---- clock/reset + root --------------------------------------------------
    logic clock = 1'b0;
    logic reset = 1'b1;
    always #5ns clock = ~clock;

    `fw_root_begin(isa_env, u_root, clock, reset)
    `fw_root_end

    int errors = 0;
    task automatic expect_eq(input string what, input longint got, input longint exp);
        if (got !== exp) begin
            errors++;
            $display("[ibex_isa] FAIL: %s = 0x%0h (exp 0x%0h)", what, got, exp);
        end
    endtask

    initial begin
        isa_env e;
        reset = 1'b1;
        repeat (4) @(posedge clock);
        reset = 1'b0;
        while (u_root.root == null) @(posedge clock);
        e = u_root.root;

        // ---- M ----
        while (e.d_m.core.hart.halt_code == HALT_NONE) @(posedge clock);
        expect_eq("m.mul",       e.d_m.mem.peek(RES+0),  42);
        expect_eq("m.div",       e.d_m.mem.peek(RES+4),  32'hFFFF_FFFA);   // -6
        expect_eq("m.rem",       e.d_m.mem.peek(RES+8),  32'hFFFF_FFFE);   // -2
        expect_eq("m.div0",      e.d_m.mem.peek(RES+12), 32'hFFFF_FFFF);   // 5/0
        expect_eq("m.remu0",     e.d_m.mem.peek(RES+16), 5);               // 5%0
        expect_eq("m.div_ovf",   e.d_m.mem.peek(RES+20), 32'h8000_0000);
        expect_eq("m.rem_ovf",   e.d_m.mem.peek(RES+24), 0);
        expect_eq("m.mulhu",     e.d_m.mem.peek(RES+28), 32'hFFFF_FFFE);
        expect_eq("m.mulhsu",    e.d_m.mem.peek(RES+32), 32'hFFFF_FFFF);

        // ---- C ----
        while (e.d_c.core.hart.halt_code == HALT_NONE) @(posedge clock);
        expect_eq("c.straddle_sw", e.d_c.mem.peek(RES+0), 12);
        expect_eq("c.addi",        e.d_c.mem.peek(RES+4), 13);

        // ---- illegal compressed ----
        while (e.d_cill.core.hart.halt_code == HALT_NONE) @(posedge clock);
        expect_eq("cill.mcause", e.d_cill.core.csrs.regs.mcause.read().code, 31'(EXC_ILLEGAL_INSN));
        expect_eq("cill.mtval",  e.d_cill.core.csrs.regs.mtval.read(),  32'h0);
        expect_eq("cill.trapc",  e.d_cill.core.hart.trap_count, 1);

        if (errors == 0) $display("[ibex_isa] PASS");
        else             $display("[ibex_isa] FAIL (%0d errors)", errors);
        $finish;
    end

    initial begin
        #500us;
        $fatal(1, "[ibex_isa] TIMEOUT");
    end
endmodule
