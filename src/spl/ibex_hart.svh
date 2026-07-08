// The hart execution engine -- the one runnable in the model. Its run() is the
// whole core: a forever loop that (1) evaluates pending interrupts, (2) fetches,
// decodes, executes, and retires one instruction, taking a precise trap on any
// fault, speaking only fw_mem_if for memory (design sec 4/9). It owns the edge
// ports (imem/dmem masters, irq input) and mutates the GPR file and CSR file
// directly. Same shape as wb_dma_de::run() -- a service loop, no clock in the model.
//
// Phase P2 adds the full trap architecture: CSR instructions, ECALL/EBREAK/illegal/
// bus-fault exceptions, asynchronous interrupts with MIE gating + NMI, MRET, WFI
// (sleep on an fw_event_set over the irq port), and the functional counters. The
// M/C extensions still HALT with HALT_UNIMPL (executed in P3).
//
// Test termination uses a tohost convention (riscv-tests style): a store to the
// configured tohost address halts the engine with the written value in tohost_val
// (bit0 = pass). This replaces P1's ECALL-halt, since ECALL is now a real trap.
`ifndef INCLUDED_IBEX_HART_SVH
`define INCLUDED_IBEX_HART_SVH

// Why the engine's run() returned.
typedef enum {
    HALT_NONE,    // still running / not halted
    HALT_TOHOST,  // a store hit the tohost address (test termination; see tohost_val)
    HALT_UNIMPL,  // decoded but not executable this phase (M/C extension)
    HALT_RUNAWAY  // safety backstop: too many iterations with no progress (see pc)
} ibex_halt_e;

// Safety backstop: the model has no clock, so a control-flow bug that neither
// retires-to-halt nor blocks would spin in zero simulation time (a wall-clock hang
// the sim-time watchdog can never catch). Cap loop iterations so such a bug halts
// with HALT_RUNAWAY instead. Far above any directed/trap test; P4 compliance runs
// can raise it if a long program legitimately needs more.
localparam longint unsigned IBEX_RUNAWAY_LIMIT = 10_000_000;

class ibex_hart extends fw_component implements fw_runnable;
    typedef fw_mem_if #(ibex_addr_t, ibex_word_t, logic [3:0]) mem_if_t;

    // ---- configuration / collaborators (set by ibex_core) ----------------------
    local int unsigned  m_num_gpr;
    ibex_addr_t         reset_addr;
    ibex_addr_t         boot_addr;
    ibex_rf             rf;        // shared GPR file (owned by ibex_core)
    ibex_csr            csrs;      // shared CSR file  (owned by ibex_core)
    ibex_decoder        dec;       // pure decoder (owned here)

    // ---- edge ports ------------------------------------------------------------
    fw_port #(mem_if_t)    imem;   // instruction fetch master
    fw_port #(mem_if_t)    dmem;   // data load/store master
    fw_port #(ibex_irq_if) irq;    // interrupt input (level + WFI wake source)
    fw_port #(rvfi_if)     rvfi;   // retirement trace sink (optional)

    // ---- architectural / status state ------------------------------------------
    ibex_addr_t         pc;
    ibex_halt_e         halt_code; // why run() stopped (HALT_NONE while running)
    longint unsigned    retired;   // retired-instruction count (tests / RVFI order)
    longint unsigned    trap_count;// traps taken (exceptions + interrupts)

    // ---- tohost test-termination hook ------------------------------------------
    local ibex_addr_t   m_tohost_addr;
    local bit           m_tohost_en;
    ibex_word_t         tohost_val;

    // ---- WFI wait --------------------------------------------------------------
    local fw_event_set  m_wake;
    local bit           m_irq_conn;

    // ---- RVFI trace ------------------------------------------------------------
    local bit           m_rvfi_conn;
    local longint       m_order;      // monotonic RVFI retirement index

    function new(string name, fw_component parent, ibex_rf rf, ibex_csr csrs,
                 ibex_addr_t reset_addr = 32'h0, ibex_addr_t boot_addr = 32'h0,
                 int unsigned num_gpr = 32);
        super.new(name, parent);
        this.rf         = rf;
        this.csrs       = csrs;
        this.reset_addr = reset_addr;
        this.boot_addr  = boot_addr;
        this.m_num_gpr  = num_gpr;
        this.dec        = new(num_gpr);
        this.m_tohost_en = 1'b0;
        parent.add_runnable(this);
    endfunction

    function void build();
        imem = new("imem", this);
        dmem = new("dmem", this);
        irq  = new("irq",  this);
        rvfi = new("rvfi", this);
    endfunction

    // Configure the tohost termination address (a store here halts the engine).
    function void set_tohost(ibex_addr_t a);
        m_tohost_addr = a;
        m_tohost_en   = 1'b1;
    endfunction

    // ---- service loop ----------------------------------------------------------
    virtual task run();
        automatic decoded_instr_t d;
        automatic ibex_word_t     insn;
        automatic bit             err;
        automatic ibex_irqs_t     irqs;
        automatic logic [30:0]    icode;

        csrs.reset(boot_addr);
        pc         = reset_addr;
        halt_code  = HALT_NONE;
        retired    = 0;
        trap_count = 0;

        m_irq_conn  = irq.is_connected();
        m_rvfi_conn = rvfi.is_connected();
        m_order     = 0;
        m_wake      = new();
        if (m_irq_conn) m_wake.add(irq.t);   // WFI wakes on any line change

        forever begin
            if (retired + trap_count > IBEX_RUNAWAY_LIMIT) begin
                $display("[ibex_hart] RUNAWAY at pc=0x%08h", pc);
                halt_code = HALT_RUNAWAY;
                return;
            end
            csrs.tick_cycle();               // one "cycle" per loop iteration

            // (1) asynchronous interrupts, evaluated before fetch
            if (m_irq_conn) begin
                irqs = irq.t.pending();
                csrs.set_irqs(irqs);          // keep mip's derived view current
                if (irq_take(irqs, icode)) begin
                    automatic ibex_addr_t np;
                    automatic logic [1:0] mode0 = (csrs.priv() == PRIV_M) ? 2'b11 : 2'b00;
                    automatic ibex_addr_t epc = pc;
                    csrs.enter_trap(1'b1, icode, '0, pc, np);
                    pc = np;
                    trap_count++;
                    rvfi_trap(32'h0, 1'b1, epc, np, mode0);   // async interrupt: insn=0
                    continue;                 // handler entry; nothing retires
                end
            end

            // (2) fetch: 16/32-bit, word-straddle aware; compressed -> expanded
            begin
                automatic logic [2:0]  flen;
                automatic bit           acc_err, bad;
                automatic ibex_word_t   mtval_i;
                fetch(insn, flen, acc_err, bad, mtval_i);
                if (acc_err) begin take_exc(EXC_INSN_ACCESS_FAULT, pc, 32'h0, pc); continue; end
                d       = dec.decode(insn);
                d.len   = flen;
                if (bad || d.illegal) begin
                    take_exc(EXC_ILLEGAL_INSN, mtval_i, mtval_i, pc + 32'(flen)); continue;
                end
                // `insn` is the (possibly decompressed) 32-bit form used to decode
                // and execute; `mtval_i` is the ORIGINAL fetched instruction word --
                // the 32-bit encoding, or {16'b0, chalf} for a compressed insn. RVFI
                // reports the original bits (RISC-V Formal / Ibex convention), so pass
                // mtval_i as the RVFI instruction word.
                step(d, insn, mtval_i);
                if (halt_code != HALT_NONE) return;
            end
        end
    endtask

    // ---- interrupt evaluation (design sec 9.2) ---------------------------------
    // Highest-priority pending+enabled interrupt: NMI > external > software > timer
    // > fast (Ibex order). NMI ignores MIE; others gate on (priv==U || mstatus.MIE).
    local function bit irq_take(ibex_irqs_t irqs, output logic [30:0] code);
        mstatus_t s    = csrs.mstatus();
        mie_t     en   = csrs.mie_reg();
        bit       glob = (csrs.priv() == PRIV_U) || s.mie;
        code = '0;
        if (irqs.nmi && !csrs.nmi_active()) begin code = 31'(IRQ_NMI); return 1'b1; end
        if (!glob) return 1'b0;
        if (irqs.external && en.meie) begin code = 31'(IRQ_M_EXTERNAL); return 1'b1; end
        if (irqs.software && en.msie) begin code = 31'(IRQ_M_SOFTWARE); return 1'b1; end
        if (irqs.timer    && en.mtie) begin code = 31'(IRQ_M_TIMER);    return 1'b1; end
        for (int i = 0; i < 15; i++)
            if (irqs.fast[i] && en.fast[i]) begin code = 31'(16 + i); return 1'b1; end
        return 1'b0;
    endfunction

    // Take a synchronous exception at the current instruction (epc = pc). `insn` is
    // the faulting instruction word (0 if not yet fetched) for the RVFI trap record.
    // `seq_pc` is the SEQUENTIAL next pc (epc + instruction length): RVFI reports a
    // trapping instruction's pc_wdata as if it had fallen through (the trap itself is
    // flagged by rvfi_trap=1, and the vector appears as the handler's pc_rdata) --
    // this matches Ibex (ibex_core.sv:1651 captures pc_if, not the redirect target).
    local task take_exc(ibex_exc_e cause, ibex_word_t tval, ibex_word_t insn,
                        ibex_addr_t seq_pc);
        automatic ibex_addr_t np;
        automatic logic [1:0] mode0 = (csrs.priv() == PRIV_M) ? 2'b11 : 2'b00;
        automatic ibex_addr_t epc = pc;
        csrs.enter_trap(1'b0, 31'(cause), tval, pc, np);
        pc = np;                       // architectural redirect to the trap vector
        trap_count++;
        rvfi_trap(insn, 1'b0, epc, seq_pc, mode0);   // pc_wdata = sequential (not np)
    endtask

    // Emit an RVFI trap record (exception or interrupt). No architectural
    // instruction retired; pc_rdata = faulting/interrupted pc. pc_wdata is the
    // sequential next pc for a synchronous exception (Ibex convention).
    local function void rvfi_trap(ibex_word_t insn, bit intr,
                                  ibex_addr_t epc, ibex_addr_t newpc, logic [1:0] mode);
        rvfi_meta_t m;
        if (!m_rvfi_conn) return;
        m.order     = m_order++;
        m.insn      = insn;
        m.mode      = mode;
        m.rs1_addr  = '0; m.rs2_addr = '0; m.rd_addr = '0;
        m.rs1_rdata = '0; m.rs2_rdata = '0; m.rd_wdata = '0;
        m.pc_rdata  = epc;
        m.pc_wdata  = newpc;
        rvfi.t.trap(m, intr);
    endfunction

    // Fetch buffer (design sec 6): deliver a 16- or 32-bit instruction at any 2-byte
    // aligned pc. Read the aligned word, extract the halfword at pc[1]; if its low 2
    // bits != 11 it is compressed -> expand to 32 bits (len=2); else it is a 32-bit
    // instruction, and if it straddles the word boundary (pc[1] set) read the next
    // word and concatenate (len=4). `bad`/`mtval_i` carry a compressed-illegal result;
    // `acc_err` an instruction-access fault. `insn` is always the 32-bit form to decode.
    local task automatic fetch(output ibex_word_t insn, output logic [2:0] len,
                               output bit acc_err, output bit bad, output ibex_word_t mtval_i);
        automatic ibex_addr_t base = pc & ~32'h3;
        automatic ibex_word_t w0, w1;
        automatic bit         e0, e1 = 1'b0;
        automatic logic [15:0] half;
        automatic bit          ill;

        acc_err = 1'b0; bad = 1'b0; mtval_i = '0; insn = '0; len = 3'd4;
        imem.t.read(w0, e0, base);
        if (e0) begin acc_err = 1'b1; return; end

        half = pc[1] ? w0[31:16] : w0[15:0];
        if (half[1:0] != 2'b11) begin
            insn    = dec.decompress(half, ill);
            len     = 3'd2;
            bad     = ill;
            mtval_i = {16'b0, half};       // spec: mtval = original 16-bit pattern
        end else begin
            if (pc[1]) begin
                imem.t.read(w1, e1, base + 4);
                if (e1) begin acc_err = 1'b1; return; end
                insn = {w1[15:0], w0[31:16]};
            end else begin
                insn = w0;
            end
            len     = 3'd4;
            mtval_i = insn;
        end
    endtask

    // Execute one decoded instruction. Normal completion retires (updates counters
    // and pc); a fault calls take_exc() and returns without retiring; a tohost store
    // sets halt_code and returns.
    // `insn` is the decode/execute word (32-bit, decompressed if needed); `rvfi_iw`
    // is the original fetched instruction word reported over RVFI (compressed insns
    // carry their 16-bit encoding in the low half, per the RISC-V Formal / Ibex
    // convention). They are equal for 32-bit instructions.
    local task automatic step(decoded_instr_t d, ibex_word_t insn, ibex_word_t rvfi_iw);
        automatic ibex_word_t a, b, res, src, old, neu;
        automatic ibex_addr_t next_pc = pc + d.len;   // sequential (2 or 4 bytes)
        automatic ibex_addr_t addr;
        automatic bit         err, illc, do_write;
        // RVFI capture (raw register reads before any writeback; priv at execution)
        automatic ibex_word_t rs1v = rf.read(d.rs1);
        automatic ibex_word_t rs2v = rf.read(d.rs2);
        automatic logic [1:0] mode0 = (csrs.priv() == PRIV_M) ? 2'b11 : 2'b00;
        automatic ibex_word_t wb_val = '0;                 // value written to rd
        automatic mem_op_e    memk   = MEM_NONE;           // load/store kind retired
        automatic ibex_addr_t mem_a  = '0;
        automatic logic [3:0] mem_m  = '0;
        automatic ibex_word_t mem_d  = '0;

        a = (d.a_sel == A_PC)   ? pc :
            (d.a_sel == A_ZERO) ? '0 : rs1v;
        b = (d.b_sel == B_RS2)  ? rs2v : d.imm;

        case (d.op)
            OP_ADD, OP_SUB, OP_SLL, OP_SLT, OP_SLTU,
            OP_XOR, OP_SRL, OP_SRA, OP_OR, OP_AND: begin
                res = ibex_alu::exec(d.op, a, b);
                wb_val = res; if (d.rf_we) rf.write(d.rd, res);
            end

            OP_LUI:   begin wb_val = d.imm;      if (d.rf_we) rf.write(d.rd, d.imm);      end
            OP_AUIPC: begin wb_val = pc + d.imm; if (d.rf_we) rf.write(d.rd, pc + d.imm); end

            OP_JAL: begin
                wb_val = pc + d.len; if (d.rf_we) rf.write(d.rd, pc + d.len);  // link
                next_pc = pc + d.imm;
            end
            OP_JALR: begin
                wb_val = pc + d.len; if (d.rf_we) rf.write(d.rd, pc + d.len);  // link
                next_pc = (rs1v + d.imm) & ~32'h1;
            end

            OP_BEQ, OP_BNE, OP_BLT, OP_BGE, OP_BLTU, OP_BGEU:
                if (ibex_alu::branch_taken(d.op, rs1v, rs2v))
                    next_pc = pc + d.imm;

            OP_LOAD: begin
                addr = rs1v + d.imm;
                do_load(d.mem, addr, res, err);
                if (err) begin take_exc(EXC_LOAD_ACCESS_FAULT, addr, rvfi_iw, next_pc); return; end
                wb_val = res; if (d.rf_we) rf.write(d.rd, res);
                memk = d.mem; mem_lanes(d.mem, addr, res, mem_a, mem_m, mem_d);
            end
            OP_STORE: begin
                addr = rs1v + d.imm;
                do_store(d.mem, addr, rs2v, err);
                if (err) begin take_exc(EXC_STORE_ACCESS_FAULT, addr, rvfi_iw, next_pc); return; end
                memk = d.mem; mem_lanes(d.mem, addr, rs2v, mem_a, mem_m, mem_d);
                // Test-termination hook: a store to the tohost address halts the
                // engine -- but only AFTER this store architecturally commits and
                // retires (it is a real retirement in RTL, so it must be one here
                // too for RVFI lockstep to line up). halt_code is honored by run()
                // once step() returns.
                if (m_tohost_en && (addr & ~32'h3) == (m_tohost_addr & ~32'h3)) begin
                    tohost_val = rs2v;               // test termination
                    halt_code  = HALT_TOHOST;
                end
            end

            OP_FENCE: ;   // ordering no-op (single hart, no cache)

            OP_CSR: begin
                old = csrs.read(d.csr_addr, illc);
                if (illc) begin take_exc(EXC_ILLEGAL_INSN, rvfi_iw, rvfi_iw, next_pc); return; end
                src = d.csr_use_imm ? d.imm : rs1v;
                case (d.csr_op)
                    CSR_RW: begin neu = src;         do_write = 1'b1; end
                    CSR_RS: begin neu = old | src;   do_write = has_src(d); end
                    CSR_RC: begin neu = old & ~src;  do_write = has_src(d); end
                    default: do_write = 1'b0;
                endcase
                if (do_write) begin
                    csrs.write(d.csr_addr, neu, illc);
                    if (illc) begin take_exc(EXC_ILLEGAL_INSN, rvfi_iw, rvfi_iw, next_pc); return; end
                end
                wb_val = old; if (d.rf_we) rf.write(d.rd, old);
            end

            OP_ECALL:  begin
                take_exc((csrs.priv() == PRIV_U) ? EXC_ECALL_U : EXC_ECALL_M, 32'h0, rvfi_iw, next_pc);
                return;
            end
            OP_EBREAK: begin take_exc(EXC_BREAKPOINT, pc, rvfi_iw, next_pc); return; end
            OP_MRET: begin
                if (csrs.priv() != PRIV_M) begin take_exc(EXC_ILLEGAL_INSN, rvfi_iw, rvfi_iw, next_pc); return; end
                csrs.mret(next_pc);
            end
            OP_WFI: begin
                // Sleep until a line changes, unless one is already asserted.
                if (m_irq_conn && irq.t.pending() == '0) m_wake.wait_any();
            end

            // M extension (mul/div).
            OP_MUL, OP_MULH, OP_MULHSU, OP_MULHU,
            OP_DIV, OP_DIVU, OP_REM, OP_REMU: begin
                res = ibex_muldiv::exec(d.op, rs1v, rs2v);
                wb_val = res; if (d.rf_we) rf.write(d.rd, res);
            end

            default: begin take_exc(EXC_ILLEGAL_INSN, rvfi_iw, rvfi_iw, next_pc); return; end
        endcase

        // retire (+ RVFI trace). rvfi_iw (original fetched bits) is reported as the
        // RVFI instruction word so compressed insns match the RTL (16-bit encoding).
        retired++;
        csrs.tick_instret();
        rvfi_retire(d, rvfi_iw, mode0, rs1v, rs2v, wb_val, next_pc, memk, mem_a, mem_m, mem_d);
        pc = next_pc;
    endtask

    // Assemble the rvfi_meta_t from data already computed this step and dispatch to
    // exactly one rvfi_if callback by kind (design sec 12). rs1/rs2 addresses are 0
    // when the instruction does not read them; rd is 0 for x0 / no-writeback.
    local function void rvfi_retire(decoded_instr_t d, ibex_word_t insn, logic [1:0] mode,
                                    ibex_word_t rs1v, ibex_word_t rs2v, ibex_word_t wb_val,
                                    ibex_addr_t next_pc, mem_op_e memk,
                                    ibex_addr_t mem_a, logic [3:0] mem_m, ibex_word_t mem_d);
        rvfi_meta_t m;
        automatic bit r1 = dec.uses_rs1(d);
        automatic bit r2 = dec.uses_rs2(d);
        if (!m_rvfi_conn) return;
        m.order     = m_order++;
        m.insn      = insn;
        m.mode      = mode;
        m.rs1_addr  = r1 ? d.rs1 : 5'd0;   m.rs1_rdata = r1 ? rs1v : 32'h0;
        m.rs2_addr  = r2 ? d.rs2 : 5'd0;   m.rs2_rdata = r2 ? rs2v : 32'h0;
        m.rd_addr   = (d.rf_we && d.rd != 0) ? d.rd : 5'd0;
        m.rd_wdata  = (m.rd_addr != 0) ? wb_val : 32'h0;
        m.pc_rdata  = pc;
        m.pc_wdata  = next_pc;
        if (memk inside {MEM_LB, MEM_LH, MEM_LW, MEM_LBU, MEM_LHU})
            rvfi.t.retire_load(m, mem_a, mem_m, mem_d);
        else if (memk inside {MEM_SB, MEM_SH, MEM_SW})
            rvfi.t.retire_store(m, mem_a, mem_m, mem_d);
        else
            rvfi.t.retire(m);
    endfunction

    // Byte-lane view of a memory access for RVFI: effective address, 4-bit lane mask,
    // and the accessed bytes positioned in their word lanes (within-word accesses).
    local function void mem_lanes(mem_op_e m, ibex_addr_t addr, ibex_word_t val,
                                  output ibex_addr_t maddr, output logic [3:0] mask,
                                  output ibex_word_t lanes);
        automatic int         nb  = mem_nbytes(m);
        automatic int         off = addr & 32'h3;
        automatic logic [31:0] bmask = (nb == 4) ? 32'hFFFF_FFFF : ((32'h1 << (nb*8)) - 1);
        maddr = addr;
        mask  = 4'(((32'h1 << nb) - 1) << off);
        lanes = (val & bmask) << (off * 8);
    endfunction

    local function int mem_nbytes(mem_op_e m);
        case (m)
            MEM_LB, MEM_LBU, MEM_SB: return 1;
            MEM_LH, MEM_LHU, MEM_SH: return 2;
            default:                 return 4;
        endcase
    endfunction

    // CSRRS/CSRRC (and immediate forms) write only when the source operand is
    // nonzero (rs1 != x0, or a nonzero uimm) -- else they are pure reads.
    local function bit has_src(decoded_instr_t d);
        return d.csr_use_imm ? (d.imm != 0) : (d.rs1 != 0);
    endfunction

    // ---- memory helpers (byte-granular; a sub-word/straddling access splits into
    //      at most two aligned fw_mem_if transactions, matching Ibex, design sec 5)
    local function void load_shape(mem_op_e m, output int nbytes, output bit is_signed);
        case (m)
            MEM_LB:  begin nbytes = 1; is_signed = 1'b1; end
            MEM_LBU: begin nbytes = 1; is_signed = 1'b0; end
            MEM_LH:  begin nbytes = 2; is_signed = 1'b1; end
            MEM_LHU: begin nbytes = 2; is_signed = 1'b0; end
            default: begin nbytes = 4; is_signed = 1'b0; end   // MEM_LW
        endcase
    endfunction

    local task automatic do_load(mem_op_e m, ibex_addr_t addr,
                                 output ibex_word_t data, output bit err);
        automatic ibex_addr_t base = addr & ~32'h3;
        automatic int         off  = addr & 32'h3;
        automatic int         nbytes;
        automatic bit         is_signed;
        automatic ibex_word_t w0, w1 = '0;
        automatic bit         e0, e1 = 1'b0;
        automatic logic [63:0] pair;
        load_shape(m, nbytes, is_signed);

        dmem.t.read(w0, e0, base);
        if (off + nbytes > 4) dmem.t.read(w1, e1, base + 4);
        err = e0 | e1;
        if (err) begin data = '0; return; end

        pair = {w1, w0};
        data = '0;
        for (int i = 0; i < nbytes; i++)
            data[i*8 +: 8] = pair[(off + i)*8 +: 8];
        if (is_signed && nbytes < 4 && data[nbytes*8 - 1])
            for (int i = nbytes; i < 4; i++) data[i*8 +: 8] = 8'hff;
    endtask

    local function void store_shape(mem_op_e m, output int nbytes);
        case (m)
            MEM_SB:  nbytes = 1;
            MEM_SH:  nbytes = 2;
            default: nbytes = 4;   // MEM_SW
        endcase
    endfunction

    local task automatic do_store(mem_op_e m, ibex_addr_t addr,
                                  ibex_word_t sdata, output bit err);
        automatic ibex_addr_t base = addr & ~32'h3;
        automatic int         off  = addr & 32'h3;
        automatic int         nbytes;
        automatic logic [3:0] s0 = '0, s1 = '0;
        automatic ibex_word_t d0 = '0, d1 = '0;
        automatic bit         e0, e1 = 1'b0;
        store_shape(m, nbytes);

        for (int i = 0; i < nbytes; i++) begin
            automatic int lane = off + i;
            if (lane < 4) begin s0[lane]     = 1'b1; d0[lane*8 +: 8]     = sdata[i*8 +: 8]; end
            else          begin s1[lane - 4] = 1'b1; d1[(lane-4)*8 +: 8] = sdata[i*8 +: 8]; end
        end

        dmem.t.write(e0, base, d0, s0);
        if (|s1) dmem.t.write(e1, base + 4, d1, s1);
        err = e0 | e1;
    endtask

endclass

`endif // INCLUDED_IBEX_HART_SVH

