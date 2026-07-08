// Top behavioral component of the Ibex SPL model. A pure component: it owns the GPR
// and CSR files and instances the hart engine, but knows nothing about signal-level
// transactors or pins -- the testbench (and the future ibex_spl pin wrapper) bind
// the engine's edge endpoints. Binding endpoints:
//   hart.imem -- instruction fetch master (fw_mem_if)
//   hart.dmem -- data load/store master  (fw_mem_if)
//   hart.irq  -- interrupt input          (ibex_irq_if)
// (hart.rvfi is added in P4.)
//
// The GPR and CSR files are plain classes (ibex_rf / ibex_csr), constructed here and
// shared with the hart -- mirroring how wb_dma passes its register file to the engine.
`ifndef INCLUDED_IBEX_CORE_SVH
`define INCLUDED_IBEX_CORE_SVH

class ibex_core extends fw_component;
    // configuration
    int unsigned  num_gpr;      // 32 = RV32I, 16 = RV32E
    ibex_addr_t   reset_addr;   // first fetch address out of reset
    ibex_addr_t   boot_addr;    // seeds mtvec
    ibex_word_t   hart_id;      // mhartid value

    // children
    ibex_rf       rf;           // GPR file (plain class); shared with the hart
    ibex_csr      csrs;         // CSR file (plain class); shared with the hart
    ibex_hart     hart;         // the runnable engine

    function new(string name, fw_component parent,
                 ibex_addr_t reset_addr = 32'h0000_0000,
                 ibex_addr_t boot_addr  = 32'h0000_0000,
                 ibex_word_t hart_id    = 32'h0,
                 int unsigned num_gpr   = 32);
        super.new(name, parent);
        this.reset_addr = reset_addr;
        this.boot_addr  = boot_addr;
        this.hart_id    = hart_id;
        this.num_gpr    = num_gpr;
        // Plain state constructed before build(); the hart ctor captures the handles
        // (do_build recurses top-down, so ordering is safe). RV32E when num_gpr < 32.
        rf   = new(num_gpr);
        csrs = new(hart_id, (num_gpr < 32));
    endfunction

    function void build();
        hart = new("hart", this, rf, csrs, reset_addr, boot_addr, num_gpr);
    endfunction
endclass

`endif // INCLUDED_IBEX_CORE_SVH
