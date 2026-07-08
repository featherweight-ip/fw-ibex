// ======================================================================
// Phase-2 TRAP / INTERRUPT test for the Ibex SPL model.
//
// Seven independent DUTs (each = ibex_core + fw_mem_flat RAM + an interrupt
// driver), each running a directed program that triggers one trap scenario. Every
// program installs a trap handler at the mtvec base (boot_addr = 0x40); a handler
// either (a) stores a pass code to the tohost address to halt, or (b) bumps mepc and
// MRETs back. Because the handler halts *inside* the trap, the TB inspects the frozen
// CSR state (mcause / mepc / mtval) directly through core.csrs and checks it.
//
// Scenarios: ECALL+MRET round trip, illegal instruction, EBREAK, load access fault,
// timer interrupt (MIE gating positive), NMI (ignores MIE), MIE-gated (negative).
//
// Run:  dfm run fw-ibex.trap      (expect: [ibex_trap] PASS)
// ======================================================================
`include "fw_hdl_macros.svh"

module ibex_trap_tb;
    import fw_hdl_pkg::*;
    import fw_std_pkg::*;
    import fw_mem_pkg::*;
    import ibex_spl_pkg::*;

    typedef logic [31:0] addr_t;
    typedef logic [31:0] data_t;
    typedef logic [3:0]  strb_t;

    localparam addr_t HANDLER = 32'h0000_0040;   // mtvec base (boot_addr)
    // tohost must be reachable as SW rs2, TOHOST(x0): the S-type immediate is signed
    // 12-bit, so TOHOST must be in [0, 0x7ff] to encode as a positive offset.
    localparam addr_t TOHOST  = 32'h0000_0400;   // store here => halt

    // CSR addresses used by handlers.
    localparam logic [11:0] MEPC = 12'h341, MIE = 12'h304, MSTATUS = 12'h300;

    // ---- minimal assembler ---------------------------------------------------
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
    function automatic data_t SW(int rs2, int rs1, int imm);
        return s_type(7'h23, 3'b010, rs1[4:0], rs2[4:0], imm);
    endfunction
    function automatic data_t LW(int rd, int rs1, int imm);
        return i_type(7'h03, 3'b010, rd[4:0], rs1[4:0], imm);
    endfunction
    function automatic data_t LUI(int rd, int imm20);
        return {imm20[19:0], rd[4:0], 7'h37};
    endfunction
    function automatic data_t CSRRW(int rd, int csr, int rs1);
        return {csr[11:0], rs1[4:0], 3'b001, rd[4:0], 7'h73};
    endfunction
    function automatic data_t CSRRS(int rd, int csr, int rs1);
        return {csr[11:0], rs1[4:0], 3'b010, rd[4:0], 7'h73};
    endfunction
    function automatic data_t ECALL();  return 32'h0000_0073; endfunction
    function automatic data_t EBREAK(); return 32'h0010_0073; endfunction
    function automatic data_t MRET();   return 32'h3020_0073; endfunction
    function automatic data_t WFI();    return 32'h1050_0073; endfunction

    // Compose a program: main[] at 0x00, padded to 0x40, then handler[].
    function automatic void compose(ref data_t q[$], input data_t main[$], input data_t handler[$]);
        q = {};
        foreach (main[i])    q.push_back(main[i]);
        while (q.size() < 16) q.push_back(ADDI(0, 0, 0));   // NOP pad to 0x40
        foreach (handler[i]) q.push_back(handler[i]);
    endfunction

    // A handler that halts the sim by storing `code` to tohost.
    function automatic void handler_halt(ref data_t h[$], input int code);
        h = {ADDI(31, 0, code), SW(31, 0, TOHOST)};
    endfunction

    // ---- interrupt driver (provides ibex_irq_if) -----------------------------
    class irq_drv extends fw_component;
        ibex_irqs_t  lines;
        fw_event_set monitors[$];
        `FW_IBEX_IRQ_IMP(irq_drv, o);

        function new(string n, fw_component p); super.new(n, p); endfunction
        function void build(); o = new(this); lines = '0; endfunction

        virtual function ibex_irqs_t o_pending();               return lines;             endfunction
        virtual function void o_produce_to(fw_event_set s);     monitors.push_back(s);    endfunction
        function void notify_all();  foreach (monitors[i]) monitors[i].notify();          endfunction
        function void set_timer(bit v); lines.timer = v; notify_all(); endfunction
        function void set_nmi(bit v);   lines.nmi   = v; notify_all(); endfunction
    endclass

    // ---- one device under test: core + RAM + irq driver ----------------------
    class dut extends fw_component;
        data_t             prog[$];      // set by the env before build()
        longint unsigned   mem_bound = 0;
        ibex_core          core;
        fw_mem_flat #(addr_t, data_t, strb_t) mem;
        irq_drv            irq;

        function new(string n, fw_component p); super.new(n, p); endfunction

        function void build();
            core = new("core", this, 32'h0 /*reset*/, HANDLER /*boot=mtvec*/, 32'h0, 32);
            mem  = new("mem",  this);
            irq  = new("irq",  this);
            if (mem_bound != 0) mem.set_size(mem_bound);
            mem.load(32'h0, prog);
        endfunction

        function void connect();
            core.hart.imem.connect(mem.mem_if);
            core.hart.dmem.connect(mem.mem_if);
            core.hart.irq.connect(irq.o);
            core.hart.set_tohost(TOHOST);
        endfunction
    endclass

    // ---- environment: build the seven scenarios ------------------------------
    class trap_env extends fw_component;
        dut d_ecall, d_illegal, d_ebreak, d_loadflt, d_timer, d_nmi, d_gated;

        function new(string n, fw_component p); super.new(n, p); endfunction

        function void build();
            data_t hh[$], he[$];

            d_ecall   = new("ecall",   this);
            d_illegal = new("illegal", this);
            d_ebreak  = new("ebreak",  this);
            d_loadflt = new("loadflt", this);
            d_timer   = new("timer",   this);
            d_nmi     = new("nmi",     this);
            d_gated   = new("gated",   this);

            // 1) ECALL + MRET round trip: handler bumps mepc past ECALL and returns;
            //    main then stores 7 to tohost, proving the return path.
            he = {CSRRS(6, MEPC, 0), ADDI(6, 6, 4), CSRRW(0, MEPC, 6), MRET()};
            compose(d_ecall.prog,
                    '{ECALL(), ADDI(5, 0, 7), SW(5, 0, TOHOST)}, he);

            // 2) Illegal instruction (all-ones), handler halts.
            handler_halt(hh, 1);
            compose(d_illegal.prog, '{32'hFFFF_FFFF}, hh);

            // 3) EBREAK.
            handler_halt(hh, 1);
            compose(d_ebreak.prog, '{EBREAK()}, hh);

            // 4) Load access fault: load from 0x10000 with the RAM bounded to 4 KiB.
            handler_halt(hh, 1);
            compose(d_loadflt.prog, '{LUI(2, 32'h10), LW(1, 2, 0)}, hh);
            d_loadflt.mem_bound = 32'h0000_1000;

            // 5) Timer interrupt: enable MTIE + MIE, then WFI. Driver asserts timer.
            handler_halt(hh, 1);
            compose(d_timer.prog,
                    '{ADDI(1, 0, 32'h80), CSRRW(0, MIE, 1),
                      ADDI(2, 0, 8),      CSRRS(0, MSTATUS, 2), WFI()}, hh);

            // 6) NMI: MIE stays 0, WFI. Driver asserts NMI (ignored MIE => taken).
            handler_halt(hh, 1);
            compose(d_nmi.prog, '{WFI()}, hh);

            // 7) MIE-gated (negative): enable MTIE but leave MIE=0, WFI. Driver
            //    asserts timer -> WFI wakes but NO trap; main halts with code 1.
            handler_halt(hh, 2);   // handler (unreached) would store 2
            compose(d_gated.prog,
                    '{ADDI(1, 0, 32'h80), CSRRW(0, MIE, 1), WFI(),
                      ADDI(31, 0, 1), SW(31, 0, TOHOST)}, hh);
        endfunction
    endclass

    // ---- clock/reset + root --------------------------------------------------
    logic clock = 1'b0;
    logic reset = 1'b1;
    always #5ns clock = ~clock;

    `fw_root_begin(trap_env, u_root, clock, reset)
    `fw_root_end

    // ---- checks --------------------------------------------------------------
    int errors = 0;
    task automatic expect_eq(input string what, input longint got, input longint exp);
        if (got !== exp) begin
            errors++;
            $display("[ibex_trap] FAIL: %s = 0x%0h (exp 0x%0h)", what, got, exp);
        end
    endtask

    initial begin
        trap_env e;
        reset = 1'b1;
        repeat (4) @(posedge clock);
        reset = 1'b0;
        while (u_root.root == null) @(posedge clock);
        e = u_root.root;

        // Let the combinational DUTs run and the WFI DUTs reach their sleep, then
        // drive the interrupt lines.
        repeat (20) @(posedge clock);
        e.d_timer.irq.set_timer(1'b1);
        e.d_nmi.irq.set_nmi(1'b1);
        e.d_gated.irq.set_timer(1'b1);

        // 1) ECALL + MRET
        while (e.d_ecall.core.hart.halt_code == HALT_NONE) @(posedge clock);
        expect_eq("ecall.tohost",     e.d_ecall.core.hart.tohost_val,          7);
        expect_eq("ecall.mcause.int", e.d_ecall.core.csrs.regs.mcause.read().interrupt, 0);
        expect_eq("ecall.mcause",     e.d_ecall.core.csrs.regs.mcause.read().code, 31'(EXC_ECALL_M));
        expect_eq("ecall.trap_count", e.d_ecall.core.hart.trap_count,           1);

        // 2) Illegal
        while (e.d_illegal.core.hart.halt_code == HALT_NONE) @(posedge clock);
        expect_eq("illegal.mcause", e.d_illegal.core.csrs.regs.mcause.read().code, 31'(EXC_ILLEGAL_INSN));
        expect_eq("illegal.mtval",  e.d_illegal.core.csrs.regs.mtval.read(),  32'hFFFF_FFFF);
        expect_eq("illegal.mepc",   e.d_illegal.core.csrs.regs.mepc.read(),   32'h0);

        // 3) EBREAK
        while (e.d_ebreak.core.hart.halt_code == HALT_NONE) @(posedge clock);
        expect_eq("ebreak.mcause", e.d_ebreak.core.csrs.regs.mcause.read().code, 31'(EXC_BREAKPOINT));

        // 4) Load access fault
        while (e.d_loadflt.core.hart.halt_code == HALT_NONE) @(posedge clock);
        expect_eq("loadflt.mcause", e.d_loadflt.core.csrs.regs.mcause.read().code, 31'(EXC_LOAD_ACCESS_FAULT));
        expect_eq("loadflt.mtval",  e.d_loadflt.core.csrs.regs.mtval.read(),  32'h0001_0000);
        expect_eq("loadflt.mepc",   e.d_loadflt.core.csrs.regs.mepc.read(),   32'h4);

        // 5) Timer interrupt
        while (e.d_timer.core.hart.halt_code == HALT_NONE) @(posedge clock);
        expect_eq("timer.mcause.int", e.d_timer.core.csrs.regs.mcause.read().interrupt, 1);
        expect_eq("timer.mcause",     e.d_timer.core.csrs.regs.mcause.read().code, 31'(IRQ_M_TIMER));
        expect_eq("timer.trap_count", e.d_timer.core.hart.trap_count, 1);

        // 6) NMI (taken despite MIE=0)
        while (e.d_nmi.core.hart.halt_code == HALT_NONE) @(posedge clock);
        expect_eq("nmi.mcause.int", e.d_nmi.core.csrs.regs.mcause.read().interrupt, 1);
        expect_eq("nmi.mcause",     e.d_nmi.core.csrs.regs.mcause.read().code, 31'(IRQ_NMI));

        // 7) MIE-gated: WFI woke but no interrupt was taken.
        while (e.d_gated.core.hart.halt_code == HALT_NONE) @(posedge clock);
        expect_eq("gated.trap_count", e.d_gated.core.hart.trap_count, 0);
        expect_eq("gated.tohost",     e.d_gated.core.hart.tohost_val, 1);

        if (errors == 0) $display("[ibex_trap] PASS");
        else             $display("[ibex_trap] FAIL (%0d errors)", errors);
        $finish;
    end

    initial begin
        #500us;
        $fatal(1, "[ibex_trap] TIMEOUT");
    end
endmodule
