`ifndef INCLUDED_IBEX_SPL_MACROS_SVH
`define INCLUDED_IBEX_SPL_MACROS_SVH

// Provider (fw_export-stamping) macros for the Ibex SPL model's model-specific
// APIs, following the fw-api-kit recipe used by wb_dma_spl_macros.svh. The memory
// API provider macro (`FW_MEM_IMP) comes from fw-hdl's std layer -- pulled in here
// so one include of this file makes every provider macro the model needs visible.
//
// `FW_MEM_IMP comes from fw-hdl's std layer (the testbench memory provides fw_mem_if
// with it). `FW_IBEX_IRQ_IMP (below) stamps an interrupt-line provider.
`include "fw_std_macros.svh"

// `FW_IBEX_IRQ_IMP(IMP, NAME) -- stamp an interrupt-input provider inside a
//   component. The driver implements pending() as NAME_pending() and, being an
//   awaitable source, produce_to() as NAME_produce_to(set) -- it pushes a wake into
//   `set` whenever a line rises (used by the hart's WFI wait). Follows the
//   FW_WB_DMA_HS_IMP recipe (set_imp(this) after super.new -- passing `this` into
//   super.new segfaults Verilator 5.041).
`define FW_IBEX_IRQ_IMP(IMP, NAME) \
    class NAME``_imp_t extends fw_export #(ibex_irq_if) \
            implements ibex_irq_if; \
        local IMP m_imp; \
        function new(IMP imp); \
            super.new(`"NAME`", imp, null); \
            set_imp(this); \
            m_imp = imp; \
        endfunction \
        virtual function ibex_irqs_t pending(); \
            return m_imp.NAME``_pending(); \
        endfunction \
        virtual function void produce_to(fw_event_set s); \
            m_imp.NAME``_produce_to(s); \
        endfunction \
    endclass \
    NAME``_imp_t NAME

`endif // INCLUDED_IBEX_SPL_MACROS_SVH
