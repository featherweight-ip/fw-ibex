// Shared types for the Ibex SPL model. Following the wb_dma_spl convention there is
// no dedicated types package -- the behavioral model has no synthesizable-sharing
// constraint, so the few shared types live here and are `include`d into
// ibex_spl_pkg before any class that uses them.
//
// Phase P1 populates the instruction-decode types (opcodes, the semantic op enum,
// operand selects, and the decoded-instruction record). The CSR field structs and
// the interrupt-line struct are added in P2 (see design doc sec 8/9/10).
`ifndef INCLUDED_IBEX_SPL_TYPES_SVH
`define INCLUDED_IBEX_SPL_TYPES_SVH

// ---- machine word --------------------------------------------------------------
// RV32: XLEN = 32. Kept as typedefs so a future RV64 variant is a localized change.
typedef logic [31:0] ibex_word_t;
typedef logic [31:0] ibex_addr_t;
typedef logic [4:0]  ibex_regid_t;

// ---- RV32 base opcodes (insn[6:0], the low field of every 32-bit instruction) --
// Names + encodings follow the RISC-V unprivileged spec (and ibex_pkg opcode_e).
typedef enum logic [6:0] {
    OPCODE_LOAD     = 7'h03,
    OPCODE_MISC_MEM = 7'h0f,   // FENCE / FENCE.I
    OPCODE_OP_IMM   = 7'h13,
    OPCODE_AUIPC    = 7'h17,
    OPCODE_STORE    = 7'h23,
    OPCODE_OP       = 7'h33,
    OPCODE_LUI      = 7'h37,
    OPCODE_BRANCH   = 7'h63,
    OPCODE_JALR     = 7'h67,
    OPCODE_JAL      = 7'h6f,
    OPCODE_SYSTEM   = 7'h73
} ibex_opcode_e;

// ---- semantic operation -------------------------------------------------------
// Model-owned semantic op (NOT ibex_pkg's internal alu_op_e): the decoder lowers an
// instruction to one of these, and the engine's execute() switches on it. Keeping
// the enum ours decouples the model from RTL naming and keeps execute() readable.
// A documented correspondence to ibex_pkg lives in the design doc (sec 7).
typedef enum {
    // integer register-register / register-immediate ALU
    OP_ADD, OP_SUB, OP_SLL, OP_SLT, OP_SLTU, OP_XOR, OP_SRL, OP_SRA, OP_OR, OP_AND,
    // upper-immediate
    OP_LUI, OP_AUIPC,
    // control transfer
    OP_JAL, OP_JALR,
    OP_BEQ, OP_BNE, OP_BLT, OP_BGE, OP_BLTU, OP_BGEU,
    // memory (width/sign carried in mem_op_e)
    OP_LOAD, OP_STORE,
    // system / CSR (executed in P2; decoded now)
    OP_CSR, OP_ECALL, OP_EBREAK, OP_MRET, OP_WFI, OP_FENCE,
    // M extension (executed in P3; decoded now so illegal-detection is stable)
    OP_MUL, OP_MULH, OP_MULHSU, OP_MULHU, OP_DIV, OP_DIVU, OP_REM, OP_REMU,
    // undecodable / unsupported
    OP_ILLEGAL
} ibex_op_e;

// Operand-A source mux (feeds ALU input a / address base).
typedef enum { A_RS1, A_PC, A_ZERO } a_sel_e;
// Operand-B source mux (feeds ALU input b).
typedef enum { B_RS2, B_IMM } b_sel_e;

// Memory access shape for OP_LOAD / OP_STORE. Encodes width + sign-extension; NONE
// for non-memory ops. LB/LH/LW and their unsigned loads, SB/SH/SW.
typedef enum {
    MEM_NONE,
    MEM_LB, MEM_LH, MEM_LW, MEM_LBU, MEM_LHU,   // loads (sign/zero per suffix)
    MEM_SB, MEM_SH, MEM_SW                        // stores
} mem_op_e;

// CSR access kind (RV32 Zicsr). NONE for non-CSR instructions.
typedef enum {
    CSR_NONE, CSR_RW, CSR_RS, CSR_RC
} csr_op_e;

// ---- decoded instruction ------------------------------------------------------
// The pure output of ibex_decoder::decode(). Carries everything execute() needs and
// nothing architectural -- decode touches no state, so this record is also the
// source of the RVFI `insn` breakout (design sec 12).
typedef struct {
    ibex_op_e     op;          // semantic operation
    ibex_regid_t  rs1, rs2, rd;
    ibex_word_t   imm;         // sign/zero-extended per format (I/S/B/U/J)
    a_sel_e       a_sel;       // operand-A mux
    b_sel_e       b_sel;       // operand-B mux
    bit           rf_we;       // writes rd?
    mem_op_e      mem;         // memory shape (MEM_NONE if not a load/store)
    csr_op_e      csr_op;      // CSR access kind
    logic [11:0]  csr_addr;    // CSR address (valid when csr_op != CSR_NONE)
    bit           csr_use_imm; // CSRRWI/RSI/RCI: use zimm (rs1 field) instead of x[rs1]
    bit           illegal;     // undecodable / unsupported encoding
    logic [2:0]   len;         // instruction length in bytes (4 for base; 2 set by fetch on C)
} decoded_instr_t;

// ================================================================================
// P2 additions: privilege, CSR field layouts, interrupt lines, trap causes.
// ================================================================================

// ---- privilege modes (mstatus.MPP / current priv encoding) ---------------------
typedef enum logic [1:0] {
    PRIV_U = 2'b00,
    PRIV_M = 2'b11
} ibex_priv_e;

// ---- interrupt input lines (level), delivered through ibex_irq_if ---------------
// Not a memory-mapped register -- a plain value type carrying the current level of
// every interrupt source. `fast` are Ibex's 15 fast local interrupts.
typedef struct packed {
    bit        nmi;        // non-maskable (ignores mstatus.MIE)
    bit        external;   // machine external interrupt (MEI)
    bit        software;   // machine software interrupt (MSI)
    bit        timer;      // machine timer interrupt (MTI)
    bit [14:0] fast;       // 15 fast local interrupts (Ibex)
} ibex_irqs_t;

// ---- CSR field layouts (2-state packed structs, full 32-bit, MSB-first) ---------
// Only the bits the model implements are named; the rest are reserved fillers so
// the struct is exactly 32 bits and field access (mstatus.read().mie) needs no
// hand-maintained offsets. Bit positions follow the RISC-V privileged spec.

// mstatus (0x300): MIE(3) MPIE(7) MPP(12:11) MPRV(17) TW(21)
typedef struct packed {
    bit [9:0] rsvd4;   // 31:22
    bit       tw;      // 21
    bit [2:0] rsvd3;   // 20:18
    bit       mprv;    // 17
    bit [3:0] rsvd2;   // 16:13
    bit [1:0] mpp;     // 12:11
    bit [2:0] rsvd1;   // 10:8
    bit       mpie;    // 7
    bit [2:0] rsvd0b;  // 6:4
    bit       mie;     // 3
    bit [2:0] rsvd0a;  // 2:0
} mstatus_t;

// mtvec (0x305): base[31:2], mode[1:0] (0=direct, 1=vectored)
typedef struct packed {
    bit [29:0] base;   // 31:2
    bit [1:0]  mode;   // 1:0
} mtvec_t;

// mcause (0x342): interrupt bit + exception/interrupt code
typedef struct packed {
    bit        interrupt;  // 31
    bit [30:0] code;       // 30:0
} mcause_t;

// mie (0x304) / mip (0x344): MSIE/MSIP(3) MTIE/MTIP(7) MEIE/MEIP(11) + fast(30:16)
typedef struct packed {
    bit        rsvd3;   // 31
    bit [14:0] fast;    // 30:16
    bit [3:0]  rsvd2;   // 15:12
    bit        meie;    // 11
    bit [2:0]  rsvd1;   // 10:8
    bit        mtie;    // 7
    bit [2:0]  rsvd0b;  // 6:4
    bit        msie;    // 3
    bit [2:0]  rsvd0a;  // 2:0
} mie_t;

// ---- CSR addresses (12-bit) -----------------------------------------------------
typedef enum logic [11:0] {
    CSR_MSTATUS       = 12'h300,
    CSR_MISA          = 12'h301,
    CSR_MIE           = 12'h304,
    CSR_MTVEC         = 12'h305,
    CSR_MCOUNTEREN    = 12'h306,
    CSR_MSCRATCH      = 12'h340,
    CSR_MEPC          = 12'h341,
    CSR_MCAUSE        = 12'h342,
    CSR_MTVAL         = 12'h343,
    CSR_MIP           = 12'h344,
    CSR_MCOUNTINHIBIT = 12'h320,
    CSR_MCYCLE        = 12'hB00,
    CSR_MINSTRET      = 12'hB02,
    CSR_MCYCLEH       = 12'hB80,
    CSR_MINSTRETH     = 12'hB82,
    CSR_MVENDORID     = 12'hF11,
    CSR_MARCHID       = 12'hF12,
    CSR_MIMPID        = 12'hF13,
    CSR_MHARTID       = 12'hF14,
    CSR_MCONFIGPTR    = 12'hF15
} ibex_csr_addr_e;

// ---- trap causes ----------------------------------------------------------------
// Exception codes (mcause.interrupt = 0).
typedef enum logic [30:0] {
    EXC_INSN_ADDR_MISALIGNED = 31'd0,
    EXC_INSN_ACCESS_FAULT    = 31'd1,
    EXC_ILLEGAL_INSN         = 31'd2,
    EXC_BREAKPOINT           = 31'd3,
    EXC_LOAD_ACCESS_FAULT    = 31'd5,
    EXC_STORE_ACCESS_FAULT   = 31'd7,
    EXC_ECALL_U              = 31'd8,
    EXC_ECALL_M              = 31'd11
} ibex_exc_e;

// Interrupt codes (mcause.interrupt = 1). Fast interrupts occupy 16..30; NMI = 31.
typedef enum logic [30:0] {
    IRQ_M_SOFTWARE = 31'd3,
    IRQ_M_TIMER    = 31'd7,
    IRQ_M_EXTERNAL = 31'd11,
    IRQ_NMI        = 31'd31
} ibex_irq_code_e;

`endif // INCLUDED_IBEX_SPL_TYPES_SVH
