// Behavioral (class-based) model of the lowRISC Ibex RISC-V core -- the "spl"
// model. One package holds the whole class layer: the shared types, the model-
// specific APIs, and the model components (one .svh per class, included below). The
// signal-level pin wrapper (ibex_spl, ports matching ibex_top) will live in a
// separate compilation unit and bring in the memory + RVFI transactors; this core-
// model package does NOT depend on any bus protocol.
//
// Memory access is protocol-independent: the engine speaks fw_mem_if (from fw-hdl's
// fw_std_pkg); a req/gnt/rvalid bus is reached only via transactors + fw_mem_if
// adapters at the ibex_spl boundary (a later phase). See doc/ibex_spl_design.md.
//
// Package dependencies (P1): fw_hdl_pkg (modeling-library kernel) and fw_std_pkg
// (the std protocol layer, which provides fw_mem_if). Both ship in fw-hdl. The RVFI
// trace port (fw_proto_rvfi_pkg) is added at P4.
`include "fw_hdl_macros.svh"
`include "ibex_spl_macros.svh"     // also pulls in fw_std_macros.svh (FW_MEM_IMP)

package ibex_spl_pkg;
    import fw_hdl_pkg::*;
    import fw_std_pkg::*;           // fw_mem_if (protocol-independent memory API)
    export fw_std_pkg::*;           // re-export so consumers see fw_mem_if too
    import fw_proto_rvfi_pkg::*;    // rvfi_if + rvfi_meta_t (retirement trace API)
    export fw_proto_rvfi_pkg::*;    // re-export so consumers see rvfi_if too

    // ---- shared types --------------------------------------------------------
    `include "ibex_spl_types.svh"   // opcodes, ibex_op_e, decoded_instr_t, CSR/irq

    // ---- model-specific edge APIs --------------------------------------------
    `include "ibex_irq_if.svh"      // interrupt-input API (extends fw_awaitable_if)

    // ---- pure helpers (no state, no ports) -----------------------------------
    `include "ibex_alu.svh"         // integer ALU + branch comparators
    `include "ibex_muldiv.svh"      // M-extension mul/div
    `include "ibex_decoder.svh"     // bytes -> decoded_instr_t
    `include "ibex_rf.svh"          // GPR file (plain class, NUM_GPR)
    `include "ibex_csr.svh"         // CSR file + trap helpers (register model)

    // ---- model components (one class per file) -------------------------------
    `include "ibex_hart.svh"        // the runnable engine (fetch/decode/execute/trap)
    `include "ibex_core.svh"        // top component (rf + csrs + hart)
endpackage
