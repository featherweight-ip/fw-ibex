// General-purpose register file (the x-registers). A plain class -- no ports, no
// process: the hart engine owns it and mutates it directly (mirrors how the wb_dma
// engine owns its channels' working state). Parameterized by NUM_GPR: 32 for RV32I,
// 16 for RV32E. x0 reads as zero and ignores writes.
//
// The RV32E index-range check (a register index >= NUM_GPR is illegal) is enforced
// in the decoder, not here -- this file just clamps x0 and stores the rest.
`ifndef INCLUDED_IBEX_RF_SVH
`define INCLUDED_IBEX_RF_SVH

class ibex_rf;
    // Number of architectural GPRs (32 = RV32I, 16 = RV32E).
    local int unsigned    m_num;
    // Backing storage. x[0] is kept but never returned/written (x0 hard-wired 0).
    local ibex_word_t     m_x[];

    function new(int unsigned num_gpr = 32);
        m_num = num_gpr;
        m_x   = new[num_gpr];
        foreach (m_x[i]) m_x[i] = '0;
    endfunction

    function int unsigned num();
        return m_num;
    endfunction

    // Read x[a]; x0 always reads 0.
    function ibex_word_t read(ibex_regid_t a);
        if (a == 0) return '0;
        return m_x[a];
    endfunction

    // Write x[a] <- v; writes to x0 are discarded.
    function void write(ibex_regid_t a, ibex_word_t v);
        if (a == 0) return;
        m_x[a] = v;
    endfunction
endclass

`endif // INCLUDED_IBEX_RF_SVH
