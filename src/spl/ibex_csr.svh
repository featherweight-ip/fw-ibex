// CSR file + trap architecture. A plain class (like ibex_rf) the hart owns and
// drives -- no ports, no process. The stateful CSRs are typed fw_reg #(T) held in a
// fw_reg_block #(32) placed at their 12-bit CSR addresses, so software access
// (CSR instructions) routes through the block's offset decode and the register
// model applies the sw write masks, while the trap logic reads/writes named fields
// (mstatus.read().mie) with no hand-maintained bit offsets. Read-only/computed CSRs
// (misa, mip, mhartid, mvendorid, ...) are NOT stored -- they are derived on read,
// like the DMA model's INT_SRC views.
//
// The engine calls: reset(), read()/write() (CSR instructions, with priv/RO/exist
// checks that flag illegal), enter_trap()/mret() (the trap/return sequences),
// tick_cycle()/tick_instret() (the functional counters). Interrupt evaluation reads
// mstatus()/mie_reg()/priv().
`ifndef INCLUDED_IBEX_CSR_SVH
`define INCLUDED_IBEX_CSR_SVH

// The stored (read/write) CSRs, placed by CSR address. Only bits the model supports
// are software-writable (sw_wmask); the rest read as their reset value.
class ibex_regs extends fw_reg_block #(32);
    fw_reg #(mstatus_t) mstatus;
    fw_reg #(mtvec_t)   mtvec;
    fw_reg #(mie_t)     mie;
    fw_reg #(bit[31:0]) mscratch;
    fw_reg #(bit[31:0]) mepc;
    fw_reg #(mcause_t)  mcause;
    fw_reg #(bit[31:0]) mtval;
    fw_reg #(bit[31:0]) mcounteren;
    fw_reg #(bit[31:0]) mcountinhibit;
    fw_reg #(bit[31:0]) mcycle, mcycleh, minstret, minstreth;

    // Writable-bit masks (as struct literals cast to 32 bits).
    localparam bit [31:0] MSTATUS_WMASK = 32'h0022_1888; // TW(21) MPRV(17) MPP(12:11) MPIE(7) MIE(3)
    localparam bit [31:0] MIE_WMASK     = 32'h7FFF_0888; // fast(30:16), MEIE, MTIE, MSIE
    localparam bit [31:0] MEPC_WMASK    = 32'hFFFF_FFFE; // bit0 = 0 (IALIGN=16 with C)
    localparam bit [31:0] MTVEC_WMASK   = 32'hFFFF_FFFD; // base + mode bit0 (direct/vectored)
    localparam bit [31:0] CINH_WMASK    = 32'h0000_0005; // IR(bit2), CY(bit0)

    function new();
        super.new("csr");
        // offset == CSR address (sparse; register lookup is exact-match by offset)
        mstatus       = new("mstatus",       this, CSR_MSTATUS,       '0, MSTATUS_WMASK);
        mtvec         = new("mtvec",         this, CSR_MTVEC,         '0, MTVEC_WMASK);
        mie           = new("mie",           this, CSR_MIE,           '0, MIE_WMASK);
        mscratch      = new("mscratch",      this, CSR_MSCRATCH);
        mepc          = new("mepc",          this, CSR_MEPC,          '0, MEPC_WMASK);
        mcause        = new("mcause",        this, CSR_MCAUSE);
        mtval         = new("mtval",         this, CSR_MTVAL);
        mcounteren    = new("mcounteren",    this, CSR_MCOUNTEREN);
        mcountinhibit = new("mcountinhibit", this, CSR_MCOUNTINHIBIT, '0, CINH_WMASK);
        mcycle        = new("mcycle",        this, CSR_MCYCLE);
        mcycleh       = new("mcycleh",       this, CSR_MCYCLEH);
        minstret      = new("minstret",      this, CSR_MINSTRET);
        minstreth     = new("minstreth",     this, CSR_MINSTRETH);
    endfunction
endclass

class ibex_csr;
    ibex_regs    regs;
    // configuration
    local bit         m_rv32e;
    local ibex_word_t m_hart_id;
    // dynamic state
    local ibex_priv_e m_priv;       // current privilege
    local ibex_irqs_t m_irqs;       // latest interrupt-line levels (for mip)
    local bit         m_nmi_active;  // NMI taken, not yet returned-from (masks NMI)

    function new(ibex_word_t hart_id = '0, bit rv32e = 1'b0);
        regs         = new();
        m_hart_id    = hart_id;
        m_rv32e      = rv32e;
        m_priv       = PRIV_M;
        m_irqs       = '0;
        m_nmi_active = 1'b0;
    endfunction

    // ---- reset -----------------------------------------------------------------
    // Return to machine mode, clear stored CSRs, seed mtvec from boot_addr (direct).
    // A task (not a function): writing mtvec goes through the register model's
    // write(), which is a task.
    task reset(ibex_addr_t boot_addr);
        mtvec_t tv;
        regs.mstatus.reset();
        regs.mtvec.reset();
        regs.mie.reset();
        regs.mscratch.reset();
        regs.mepc.reset();
        regs.mcause.reset();
        regs.mtval.reset();
        regs.mcounteren.reset();
        regs.mcountinhibit.reset();
        regs.mcycle.reset();      regs.mcycleh.reset();
        regs.minstret.reset();    regs.minstreth.reset();
        tv.base = boot_addr[31:2];
        tv.mode = 2'b00;          // direct
        regs.mtvec.write(tv);
        m_priv       = PRIV_M;
        m_nmi_active = 1'b0;
    endtask

    // ---- introspection used by the engine's interrupt evaluation ---------------
    function ibex_priv_e priv();      return m_priv;             endfunction
    function mstatus_t   mstatus();   return regs.mstatus.read(); endfunction
    function mie_t       mie_reg();   return regs.mie.read();     endfunction
    function void        set_irqs(ibex_irqs_t irqs); m_irqs = irqs; endfunction

    // ---- CSR instruction access (with checks) ----------------------------------
    // Read CSR `addr`. Sets `illegal` on a nonexistent CSR or insufficient
    // privilege. Computed CSRs are derived here; stored CSRs route through the block.
    function ibex_word_t read(logic [11:0] addr, output bit illegal);
        illegal = 1'b0;
        if (!accessible(addr)) begin illegal = 1'b1; return '0; end
        return read_raw(addr);
    endfunction

    // Write `val` to CSR `addr`. Sets `illegal` on nonexistent / insufficient
    // privilege / write to a read-only CSR (addr[11:10] == 2'b11). A task: the
    // register model's write_val() is a task.
    task write(logic [11:0] addr, ibex_word_t val, output bit illegal);
        illegal = 1'b0;
        if (!accessible(addr) || addr[11:10] == 2'b11) begin illegal = 1'b1; return; end
        regs.write_val(addr, val);   // masked by the register's sw_wmask
    endtask

    // Exists AND the current privilege may access it (addr[9:8] = lowest priv).
    local function bit accessible(logic [11:0] addr);
        if (!exists(addr)) return 1'b0;
        // machine CSRs (addr[9:8]==11) require M; user CSRs require >= U (always ok)
        if (int'(m_priv) < int'(addr[9:8])) return 1'b0;
        return 1'b1;
    endfunction

    // Is `addr` an implemented CSR (stored or computed)?
    local function bit exists(logic [11:0] addr);
        if (regs.lookup(addr) != null) return 1'b1;
        case (addr)
            CSR_MISA, CSR_MIP, CSR_MHARTID, CSR_MVENDORID,
            CSR_MARCHID, CSR_MIMPID, CSR_MCONFIGPTR: return 1'b1;
            default: return 1'b0;
        endcase
    endfunction

    // Raw value of an existing CSR (no checks).
    local function ibex_word_t read_raw(logic [11:0] addr);
        case (addr)
            CSR_MISA:       return misa();
            CSR_MIP:        return mip();
            CSR_MHARTID:    return m_hart_id;
            CSR_MVENDORID:  return 32'h0;
            CSR_MARCHID:    return 32'h0000_0016;   // Ibex
            CSR_MIMPID:     return 32'h0;
            CSR_MCONFIGPTR: return 32'h0;
            default:        return regs.read_val(addr);   // stored CSR
        endcase
    endfunction

    // misa: MXL=1 (RV32) + implemented extensions (I or E, M, C, U).
    local function ibex_word_t misa();
        ibex_word_t v = '0;
        v[31:30] = 2'b01;             // MXL = 32
        if (m_rv32e) v[4] = 1'b1;     // E
        else         v[8] = 1'b1;     // I
        v[12] = 1'b1;                 // M
        v[2]  = 1'b1;                 // C
        v[20] = 1'b1;                 // U
        return v;
    endfunction

    // mip: a derived read of the current interrupt lines (not stored state).
    local function ibex_word_t mip();
        mie_t p = '0;
        p.msie = m_irqs.software;
        p.mtie = m_irqs.timer;
        p.meie = m_irqs.external;
        p.fast = m_irqs.fast;
        return 32'(p);
    endfunction

    // ---- trap architecture -----------------------------------------------------
    // Enter a trap: save state and return (via new_pc) the handler entry PC.
    // `is_int` selects interrupt vs exception; for a vectored mtvec, interrupts add
    // 4*code. A task (writes several CSRs through the register model).
    task enter_trap(bit is_int, logic [30:0] code, ibex_word_t tval, ibex_addr_t epc,
                    output ibex_addr_t new_pc);
        mstatus_t s = regs.mstatus.read();
        mcause_t  c;
        mtvec_t   tv = regs.mtvec.read();
        ibex_addr_t base = {tv.base, 2'b00};
        s.mpie = s.mie;
        s.mie  = 1'b0;
        s.mpp  = m_priv;
        regs.mstatus.write(s);
        regs.mepc.write(epc);
        c.interrupt = is_int;
        c.code      = code;
        regs.mcause.write(c);
        regs.mtval.write(tval);
        m_priv = PRIV_M;
        // NMI ignores MIE, so trap entry must latch it masked until MRET -- else a
        // still-asserted NMI line re-traps every loop iteration before the handler
        // runs (an infinite zero-time loop).
        if (is_int && code == 31'(IRQ_NMI)) m_nmi_active = 1'b1;
        new_pc = (is_int && tv.mode == 2'b01) ? base + (ibex_addr_t'(code) << 2) : base;
    endtask

    // Is an NMI currently being handled (masks further NMIs until MRET)?
    function bit nmi_active();  return m_nmi_active;  endfunction

    // MRET: restore MIE from MPIE, drop to MPP, return the resume PC (mepc).
    task mret(output ibex_addr_t new_pc);
        mstatus_t   s        = regs.mstatus.read();
        ibex_priv_e new_priv = ibex_priv_e'(s.mpp);
        s.mie  = s.mpie;
        s.mpie = 1'b1;
        s.mpp  = PRIV_U;          // least privilege
        regs.mstatus.write(s);
        m_priv       = new_priv;
        m_nmi_active = 1'b0;      // returning from the handler unmasks NMI
        new_pc       = regs.mepc.read();
    endtask

    // ---- functional counters (design sec 11) -----------------------------------
    // mcycle ticks once per loop iteration; minstret once per retired instruction;
    // each gated by mcountinhibit (CY=bit0, IR=bit2). Tasks (register writes).
    task tick_cycle();
        if (!regs.mcountinhibit.read()[0]) inc64(regs.mcycle, regs.mcycleh);
    endtask
    task tick_instret();
        if (!regs.mcountinhibit.read()[2]) inc64(regs.minstret, regs.minstreth);
    endtask

    local task inc64(fw_reg #(bit[31:0]) lo, fw_reg #(bit[31:0]) hi);
        bit [31:0] l = lo.read();
        if (l == 32'hFFFF_FFFF) hi.write(hi.read() + 1);
        lo.write(l + 1);
    endtask
endclass

`endif // INCLUDED_IBEX_CSR_SVH
