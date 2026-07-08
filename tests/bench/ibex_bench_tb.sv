// ======================================================================
// Throughput benchmark for the Ibex SPL model (model only, no RTL).
//
// Runs a tight counter loop for ~5M retired instructions and reports the count.
// The engine is clockless -- run() executes the whole program in ZERO simulation
// time -- so the SimRun task's wall-clock duration IS the model's execution time.
// Instructions/sec = retired / wall_time; the model ticks one "cycle" per
// retired instruction (csrs.tick_cycle per loop iteration), so cycles/sec is the
// same figure. RVFI is left unconnected so this measures raw execution.
//
// Run:  dfm run fw-ibex.bench      (read: retired count + the SimRun wall time)
// ======================================================================
`include "fw_hdl_macros.svh"

module ibex_bench_tb;
    import fw_hdl_pkg::*;
    import fw_std_pkg::*;
    import fw_mem_pkg::*;
    import ibex_spl_pkg::*;

    typedef logic [31:0] addr_t;
    typedef logic [31:0] data_t;
    typedef logic [3:0]  strb_t;

    localparam addr_t TOHOST = 32'h0000_0400;
    localparam int    LIMIT  = 2_500_000;   // loop iterations (2 insns each -> ~5M)

    class bench_env extends fw_component;
        ibex_core                             core;
        fw_mem_flat #(addr_t, data_t, strb_t) mem;

        function new(string n, fw_component p); super.new(n, p); endfunction

        function automatic data_t i_type(logic [6:0] op, logic [2:0] f3,
                                         int rd, int rs1, logic signed [31:0] imm);
            return {imm[11:0], rs1[4:0], f3, rd[4:0], op};
        endfunction
        function automatic data_t ADDI(int rd,int rs1,int im);
            return i_type(7'h13, 3'h0, rd, rs1, im);
        endfunction
        function automatic data_t LUI(int rd, logic [31:0] im20);
            return {im20[19:0], rd[4:0], 7'h37};
        endfunction
        function automatic data_t SW(int rs2,int rs1,logic signed [31:0] im);
            return {im[11:5], rs2[4:0], rs1[4:0], 3'h2, im[4:0], 7'h23};
        endfunction
        function automatic data_t BNE(int rs1,int rs2,logic signed [31:0] im);
            return {im[12], im[10:5], rs2[4:0], rs1[4:0], 3'h1, im[4:1], im[11], 7'h63};
        endfunction

        function void build();
            data_t prog[$];
            core = new("core", this, 32'h0, 32'h0, 32'h0, 32);   // reset PC = 0
            mem  = new("mem",  this);
            // x2 = LIMIT (0x2625A0 for 2.5M) via LUI + ADDI
            prog.push_back(LUI (2, 32'(LIMIT) >> 12));            // 0x00
            prog.push_back(ADDI(2, 2, 32'(LIMIT) & 32'hFFF));     // 0x04
            prog.push_back(ADDI(1, 0, 1));                        // 0x08  tohost value
            prog.push_back(ADDI(3, 0, 0));                        // 0x0c  counter = 0
            prog.push_back(ADDI(3, 3, 1));                        // 0x10  counter++
            prog.push_back(BNE (3, 2, -4));                       // 0x14  loop -> 0x10
            prog.push_back(SW  (1, 0, TOHOST));                   // 0x18  halt
            mem.load(32'h0, prog);
        endfunction

        function void connect();
            core.hart.imem.connect(mem.mem_if);
            core.hart.dmem.connect(mem.mem_if);
            core.hart.set_tohost(TOHOST);
        endfunction
    endclass

    logic clock = 1'b0;
    logic reset = 1'b1;
    always #5ns clock = ~clock;

    `fw_root_begin(bench_env, u_root, clock, reset)
    `fw_root_end

    initial begin
        bench_env e;
        reset = 1'b1;
        repeat (4) @(posedge clock);
        reset = 1'b0;
        while (u_root.root == null) @(posedge clock);
        e = u_root.root;
        while (e.core.hart.halt_code == HALT_NONE) @(posedge clock);
        $display("[ibex_bench] retired=%0d halt=%s tohost=0x%08h",
                 e.core.hart.retired, e.core.hart.halt_code.name(), e.core.hart.tohost_val);
        $finish;
    end
endmodule
