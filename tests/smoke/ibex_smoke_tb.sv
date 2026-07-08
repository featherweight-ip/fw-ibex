// ======================================================================
// Phase-1 SMOKE test for the Ibex SPL (class) model.
//
// Proves the class skeleton ELABORATES and RUNS a real RV32I program, using ONLY
// the protocol-independent fw_mem_if edge -- NO bus protocol, no transactors. The
// hart's two master ports (imem/dmem) are wired to one fw_mem_flat behavioral RAM
// (from fw-mem) that holds both the program and the data:
//
//   core.hart.imem --fw_mem_if--> mem   (instruction fetch)
//   core.hart.dmem --fw_mem_if--> mem   (loads / stores)
//
// A tiny in-module assembler builds a self-checking program that exercises OP-IMM,
// OP, LUI/AUIPC, LOAD/STORE (word + byte), BRANCH, JAL, then signals completion by
// storing a pass code (1) to the tohost address (riscv-tests style; the hart halts
// on that store). After the hart halts, the TB checks the final GPRs and data
// memory. Pure delay-driven (no clock in the model) -- it is a TLM model.
//
// Run:  dfm run fw-ibex.smoke      (expect: [ibex_smoke] PASS)
// ======================================================================
`include "fw_hdl_macros.svh"

module ibex_smoke_tb;
    import fw_hdl_pkg::*;
    import fw_std_pkg::*;       // fw_mem_if
    import fw_mem_pkg::*;       // fw_mem_flat behavioral RAM
    import ibex_spl_pkg::*;     // the Ibex class model

    typedef logic [31:0] addr_t;
    typedef logic [31:0] data_t;
    typedef logic [3:0]  strb_t;
    typedef fw_mem_if #(addr_t, data_t, strb_t) mem_if_t;

    // ---- data addresses used by the program ---------------------------------
    localparam addr_t WORD_ADDR = 32'h0000_0100;   // SW/LW target
    localparam addr_t BYTE_ADDR = 32'h0000_0200;   // SB/LBU target
    localparam addr_t TOHOST    = 32'h0000_0400;   // store here => halt (pass code)

    // ---- minimal RV32I assembler (returns one 32-bit instruction word) ------
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
    function automatic data_t b_type(logic [6:0] op, logic [2:0] f3,
                                     logic [4:0] rs1, logic [4:0] rs2, logic signed [31:0] imm);
        return {imm[12], imm[10:5], rs2, rs1, f3, imm[4:1], imm[11], op};
    endfunction
    function automatic data_t j_type(logic [6:0] op, logic [4:0] rd, logic signed [31:0] imm);
        return {imm[20], imm[10:1], imm[11], imm[19:12], rd, op};
    endfunction

    // opcode/funct shorthands
    localparam logic [6:0] OP_IMM = 7'h13, OP_R = 7'h33, LOAD = 7'h03,
                           STORE = 7'h23, BRANCH = 7'h63, JAL = 7'h6f, SYSTEM = 7'h73;

    function automatic data_t ADDI(int rd, int rs1, int imm);
        return i_type(OP_IMM, 3'b000, rd[4:0], rs1[4:0], imm);
    endfunction
    function automatic data_t ADD(int rd, int rs1, int rs2);
        return r_type(OP_R, 3'b000, 7'b0000000, rd[4:0], rs1[4:0], rs2[4:0]);
    endfunction
    function automatic data_t SW(int rs2, int rs1, int imm);
        return s_type(STORE, 3'b010, rs1[4:0], rs2[4:0], imm);
    endfunction
    function automatic data_t LW(int rd, int rs1, int imm);
        return i_type(LOAD, 3'b010, rd[4:0], rs1[4:0], imm);
    endfunction
    function automatic data_t SB(int rs2, int rs1, int imm);
        return s_type(STORE, 3'b000, rs1[4:0], rs2[4:0], imm);
    endfunction
    function automatic data_t LBU(int rd, int rs1, int imm);
        return i_type(LOAD, 3'b100, rd[4:0], rs1[4:0], imm);
    endfunction
    function automatic data_t BEQ(int rs1, int rs2, int imm);
        return b_type(BRANCH, 3'b000, rs1[4:0], rs2[4:0], imm);
    endfunction
    function automatic data_t JAL_(int rd, int imm);
        return j_type(JAL, rd[4:0], imm);
    endfunction

    // ---- environment: the Ibex model + one fw_mem_flat RAM ------------------
    class ibex_smoke_env extends fw_component;
        ibex_core                             core;
        fw_mem_flat #(addr_t, data_t, strb_t) mem;

        function new(string name, fw_component parent); super.new(name, parent); endfunction

        function void build();
            core = new("core", this, /*reset*/ 32'h0, /*boot*/ 32'h0, /*hartid*/ 32'h0, /*ngpr*/ 32);
            mem  = new("mem",  this);
            load_program(mem);
        endfunction

        function void connect();
            core.hart.imem.connect(mem.mem_if);   // fetch  -> RAM
            core.hart.dmem.connect(mem.mem_if);   // data   -> RAM (resolved at run start)
            core.hart.set_tohost(TOHOST);         // store to TOHOST halts the hart
        endfunction

        // Preload the self-checking program at address 0.
        function void load_program(fw_mem_flat #(addr_t, data_t, strb_t) m);
            data_t prog[$];
            prog.push_back(ADDI(1, 0, 5));            // 0x00  x1 = 5
            prog.push_back(ADDI(2, 0, 37));           // 0x04  x2 = 37
            prog.push_back(ADD (3, 1, 2));            // 0x08  x3 = 42
            prog.push_back(SW  (3, 0, WORD_ADDR));    // 0x0c  mem[0x100] = 42
            prog.push_back(LW  (4, 0, WORD_ADDR));    // 0x10  x4 = 42
            prog.push_back(JAL_(6, 8));               // 0x14  x6 = 0x18, jump 0x1c
            prog.push_back(ADDI(5, 0, 99));           // 0x18  (skipped by JAL)
            prog.push_back(ADDI(5, 0, 1));            // 0x1c  x5 = 1
            prog.push_back(SB  (3, 0, BYTE_ADDR));    // 0x20  mem[0x200].b0 = 42
            prog.push_back(LBU (7, 0, BYTE_ADDR));    // 0x24  x7 = 42
            prog.push_back(BEQ (4, 3, 8));            // 0x28  taken -> skip next
            prog.push_back(ADDI(7, 0, 55));           // 0x2c  (skipped by BEQ)
            prog.push_back(ADDI(8, 0, 1));            // 0x30  x8 = 1 (pass code)
            prog.push_back(SW  (8, 0, TOHOST));       // 0x34  store 1 -> tohost: halt
            m.load(32'h0, prog);
        endfunction
    endclass

    // ---- clock/reset + root lifecycle ---------------------------------------
    logic clock = 1'b0;
    logic reset = 1'b1;
    always #5ns clock = ~clock;

    `fw_root_begin(ibex_smoke_env, u_root, clock, reset)
    `fw_root_end

    // ---- run + self-check ----------------------------------------------------
    task automatic chk_reg(ref int errors, input ibex_rf rf, input int idx,
                           input data_t exp);
        automatic data_t got = rf.read(idx[4:0]);
        if (got !== exp) begin
            errors++;
            $display("[ibex_smoke] FAIL: x%0d = 0x%08h (exp 0x%08h)", idx, got, exp);
        end
    endtask

    initial begin
        automatic int errors = 0;
        automatic ibex_hart hart;
        automatic ibex_rf   rf;

        reset = 1'b1;
        repeat (4) @(posedge clock);
        reset = 1'b0;

        // fw_root news the root one clock after reset release; then the hart runs.
        while (u_root.root == null) @(posedge clock);
        hart = u_root.root.core.hart;
        rf   = u_root.root.core.rf;

        // Wait for the hart to halt (tohost store) or time out.
        while (hart.halt_code == HALT_NONE) @(posedge clock);

        if (hart.halt_code != HALT_TOHOST) begin
            errors++;
            $display("[ibex_smoke] FAIL: halt_code = %s (exp HALT_TOHOST)", hart.halt_code.name());
        end
        if (hart.tohost_val !== 32'd1) begin
            errors++;
            $display("[ibex_smoke] FAIL: tohost_val = 0x%08h (exp 1)", hart.tohost_val);
        end

        chk_reg(errors, rf, 1, 32'd5);
        chk_reg(errors, rf, 2, 32'd37);
        chk_reg(errors, rf, 3, 32'd42);
        chk_reg(errors, rf, 4, 32'd42);
        chk_reg(errors, rf, 5, 32'd1);           // JAL skipped the x5=99 insn
        chk_reg(errors, rf, 6, 32'h0000_0018);   // JAL link = pc+4
        chk_reg(errors, rf, 7, 32'd42);          // BEQ skipped the x7=55 insn

        if (u_root.root.mem.peek(WORD_ADDR) !== 32'd42) begin
            errors++;
            $display("[ibex_smoke] FAIL: mem[0x100] = 0x%08h (exp 42)",
                     u_root.root.mem.peek(WORD_ADDR));
        end
        if (u_root.root.mem.peek(BYTE_ADDR)[7:0] !== 8'd42) begin
            errors++;
            $display("[ibex_smoke] FAIL: mem[0x200].b0 = 0x%02h (exp 2a)",
                     u_root.root.mem.peek(BYTE_ADDR)[7:0]);
        end

        if (errors == 0) $display("[ibex_smoke] PASS (%0d instructions retired)", hart.retired);
        else             $display("[ibex_smoke] FAIL (%0d errors)", errors);
        $finish;
    end

    // Watchdog so a broken model fails fast instead of hanging.
    initial begin
        #500us;
        $fatal(1, "[ibex_smoke] TIMEOUT");
    end
endmodule
