// Interrupt-input API -- the model's contract with whatever drives its interrupt
// lines (a testbench, an interrupt controller, or the future pin wrapper). The hart
// CONSUMES it (holds an fw_port); the driver PROVIDES it (via `FW_IBEX_IRQ_IMP).
//
// It extends fw_awaitable_if so an interrupt line assertion is one of the
// heterogeneous things the engine waits on: for WFI the hart builds an fw_event_set
// over this port, and a rising line PUSHES a wake into that set (inherited
// produce_to). pending() is the level view the engine samples each instruction
// boundary to decide whether to take an interrupt. This mirrors how wb_dma_hs_if is
// both an awaitable source (dma_req wakes the engine) and a polled level.
`ifndef INCLUDED_IBEX_IRQ_IF_SVH
`define INCLUDED_IBEX_IRQ_IF_SVH

interface class ibex_irq_if extends fw_awaitable_if;
    // Current level of every interrupt line (software/timer/external/fast/nmi).
    pure virtual function ibex_irqs_t pending();
    // produce_to(set) is inherited from fw_awaitable_if: the provider wires a line
    // assertion to notify `set`, so the engine's WFI wait wakes on any change.
endclass

`endif // INCLUDED_IBEX_IRQ_IF_SVH
