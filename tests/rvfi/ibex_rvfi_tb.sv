// ======================================================================
// Phase-4 RVFI emission test for the Ibex SPL model.
//
// Binds an rvfi_if sink (stamped with FW_RVFI_IMP, from fw-proto-rvfi) to the
// core's rvfi port, runs a short program that exercises each retirement KIND
// (plain retire, load, store, trap), and checks the recorded record stream:
// order, insn, rd/rs fields, memory address/mask/data, and the trap.
//
// Run:  dfm run fw-ibex.rvfi      (expect: [ibex_rvfi] PASS)
// ======================================================================
`include "fw_hdl_macros.svh"

module ibex_rvfi_tb;
    import fw_hdl_pkg::*;
    import fw_std_pkg::*;
    import fw_mem_pkg::*;
    import fw_proto_rvfi_pkg::*;    // rvfi_if + rvfi_meta_t
    import ibex_spl_pkg::*;
    `include "fw_proto_rvfi_macros.svh"   // FW_RVFI_IMP

    typedef logic [31:0] addr_t;
    typedef logic [31:0] data_t;
    typedef logic [3:0]  strb_t;

    localparam addr_t HANDLER = 32'h0000_0040;
    localparam addr_t TOHOST  = 32'h0000_0400;
    localparam addr_t MEM     = 32'h0000_0100;

    // ---- assembler -----------------------------------------------------------
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
    function automatic data_t ADD(int rd, int rs1, int rs2);
        return r_type(7'h33, 3'b000, 7'b0000000, rd[4:0], rs1[4:0], rs2[4:0]);
    endfunction
    function automatic data_t SW(int rs2, int rs1, int imm);
        return s_type(7'h23, 3'b010, rs1[4:0], rs2[4:0], imm);
    endfunction
    function automatic data_t LW(int rd, int rs1, int imm);
        return i_type(7'h03, 3'b010, rd[4:0], rs1[4:0], imm);
    endfunction
    function automatic data_t ECALL(); return 32'h0000_0073; endfunction

    // ---- recorded RVFI record ------------------------------------------------
    typedef struct {
        int          kind;     // 0=retire, 1=load, 2=store, 3=trap
        bit          intr;
        rvfi_meta_t  m;
        logic [31:0] mem_addr, mem_data;
        logic [3:0]  mem_mask;
    } rvfi_rec_t;

    // ---- rvfi sink (provides rvfi_if) ----------------------------------------
    class rvfi_sink extends fw_component;
        rvfi_rec_t recs[$];
        `FW_RVFI_IMP(rvfi_sink, s);

        function new(string n, fw_component p); super.new(n, p); endfunction
        function void build(); s = new(this); endfunction

        virtual function void s_retire(input rvfi_meta_t m);
            recs.push_back('{kind:0, intr:0, m:m, mem_addr:0, mem_data:0, mem_mask:0});
        endfunction
        virtual function void s_retire_load(input rvfi_meta_t m, input logic [31:0] a,
                input logic [3:0] rmask, input logic [31:0] rdata);
            recs.push_back('{kind:1, intr:0, m:m, mem_addr:a, mem_data:rdata, mem_mask:rmask});
        endfunction
        virtual function void s_retire_store(input rvfi_meta_t m, input logic [31:0] a,
                input logic [3:0] wmask, input logic [31:0] wdata);
            recs.push_back('{kind:2, intr:0, m:m, mem_addr:a, mem_data:wdata, mem_mask:wmask});
        endfunction
        virtual function void s_trap(input rvfi_meta_t m, input bit intr);
            recs.push_back('{kind:3, intr:intr, m:m, mem_addr:0, mem_data:0, mem_mask:0});
        endfunction
    endclass

    // ---- environment ---------------------------------------------------------
    class rvfi_env extends fw_component;
        ibex_core core;
        fw_mem_flat #(addr_t, data_t, strb_t) mem;
        rvfi_sink  sink;

        function new(string n, fw_component p); super.new(n, p); endfunction

        function void build();
            data_t prog[$];
            core = new("core", this, 32'h0, HANDLER, 32'h0, 32);
            mem  = new("mem",  this);
            sink = new("sink", this);
            prog.push_back(ADDI(1,0,5));         // 0x00 retire
            prog.push_back(ADDI(2,0,42));        // 0x04 retire
            prog.push_back(SW  (2,0,MEM));       // 0x08 retire_store
            prog.push_back(LW  (3,0,MEM));       // 0x0c retire_load
            prog.push_back(ADD (4,1,2));         // 0x10 retire
            prog.push_back(ECALL());             // 0x14 trap
            while (prog.size() < 16) prog.push_back(ADDI(0,0,0));
            prog.push_back(ADDI(31,0,1));        // 0x40 handler
            prog.push_back(SW  (31,0,TOHOST));   // 0x44 halt
            mem.load(32'h0, prog);
        endfunction

        function void connect();
            core.hart.imem.connect(mem.mem_if);
            core.hart.dmem.connect(mem.mem_if);
            core.hart.rvfi.connect(sink.s);
            core.hart.set_tohost(TOHOST);
        endfunction
    endclass

    // ---- clock/reset + root --------------------------------------------------
    logic clock = 1'b0;
    logic reset = 1'b1;
    always #5ns clock = ~clock;

    `fw_root_begin(rvfi_env, u_root, clock, reset)
    `fw_root_end

    int errors = 0;
    task automatic eq(input string what, input longint got, input longint exp);
        if (got !== exp) begin
            errors++;
            $display("[ibex_rvfi] FAIL: %s = 0x%0h (exp 0x%0h)", what, got, exp);
        end
    endtask

    initial begin
        rvfi_env e;
        rvfi_rec_t r;
        reset = 1'b1;
        repeat (4) @(posedge clock);
        reset = 1'b0;
        while (u_root.root == null) @(posedge clock);
        e = u_root.root;
        while (e.core.hart.halt_code == HALT_NONE) @(posedge clock);

        // Expect at least 6 records (5 retires + 1 trap) before the handler runs.
        if (e.sink.recs.size() < 6) begin
            errors++;
            $display("[ibex_rvfi] FAIL: only %0d records", e.sink.recs.size());
        end else begin
            // rec0: ADDI x1,x0,5
            r = e.sink.recs[0];
            eq("r0.kind", r.kind, 0); eq("r0.order", r.m.order, 0);
            eq("r0.rd_addr", r.m.rd_addr, 1); eq("r0.rd_wdata", r.m.rd_wdata, 5);
            eq("r0.pc_rdata", r.m.pc_rdata, 32'h0); eq("r0.pc_wdata", r.m.pc_wdata, 32'h4);
            eq("r0.mode", r.m.mode, 2'b11);
            // rec1: ADDI x2,x0,42
            r = e.sink.recs[1];
            eq("r1.rd_addr", r.m.rd_addr, 2); eq("r1.rd_wdata", r.m.rd_wdata, 42);
            // rec2: SW x2 -> [0x100]
            r = e.sink.recs[2];
            eq("r2.kind", r.kind, 2); eq("r2.mem_addr", r.mem_addr, MEM);
            eq("r2.mem_mask", r.mem_mask, 4'hF); eq("r2.mem_data", r.mem_data, 42);
            eq("r2.rs2_addr", r.m.rs2_addr, 2); eq("r2.rs2_rdata", r.m.rs2_rdata, 42);
            eq("r2.rd_addr", r.m.rd_addr, 0);
            // rec3: LW x3 <- [0x100]
            r = e.sink.recs[3];
            eq("r3.kind", r.kind, 1); eq("r3.mem_addr", r.mem_addr, MEM);
            eq("r3.mem_mask", r.mem_mask, 4'hF); eq("r3.mem_data", r.mem_data, 42);
            eq("r3.rd_addr", r.m.rd_addr, 3); eq("r3.rd_wdata", r.m.rd_wdata, 42);
            // rec4: ADD x4,x1,x2
            r = e.sink.recs[4];
            eq("r4.kind", r.kind, 0); eq("r4.rd_addr", r.m.rd_addr, 4);
            eq("r4.rd_wdata", r.m.rd_wdata, 47);
            eq("r4.rs1_addr", r.m.rs1_addr, 1); eq("r4.rs1_rdata", r.m.rs1_rdata, 5);
            eq("r4.rs2_addr", r.m.rs2_addr, 2); eq("r4.rs2_rdata", r.m.rs2_rdata, 42);
            // rec5: ECALL -> trap. A trapping instruction reports pc_wdata as the
            // SEQUENTIAL next pc (0x14 + 4), not the trap vector -- the trap is
            // flagged by kind=trap; the vector shows up as the handler's pc_rdata.
            // (Matches Ibex; verified by the P5.3 RVFI lockstep.)
            r = e.sink.recs[5];
            eq("r5.kind", r.kind, 3); eq("r5.intr", r.intr, 0);
            eq("r5.insn", r.m.insn, 32'h0000_0073);
            eq("r5.pc_rdata", r.m.pc_rdata, 32'h14); eq("r5.pc_wdata", r.m.pc_wdata, 32'h18);
            // orders strictly increment
            eq("orders_monotonic", e.sink.recs[5].m.order, 5);
        end

        if (errors == 0) $display("[ibex_rvfi] PASS (%0d records)", e.sink.recs.size());
        else             $display("[ibex_rvfi] FAIL (%0d errors)", errors);
        $finish;
    end

    initial begin
        #500us;
        $fatal(1, "[ibex_rvfi] TIMEOUT");
    end
endmodule
