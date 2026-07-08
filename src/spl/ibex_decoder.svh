// Pure instruction decoder. `decode()` maps a 32-bit instruction word to a
// decoded_instr_t: it switches on the base opcode (insn[6:0]), extracts register
// indices and the format-specific immediate, and selects the semantic op. It
// touches NO architectural state, so it is a pure function -- trivially unit-
// testable and reusable to build the RVFI `insn` breakout (design sec 7).
//
// Phase P1 decodes the RV32I base set (OP / OP-IMM / LUI / AUIPC / LOAD / STORE /
// BRANCH / JAL / JALR / SYSTEM / MISC_MEM). SYSTEM/CSR fields are decoded here but
// executed in P2; M-extension encodings are recognized (so illegal-detection is
// stable) and executed in P3. Compressed (16-bit) expansion is added in P3 via
// decompress(); base decode always sets len = 4.
//
// Unsupported/undecodable encodings set illegal = 1 (op = OP_ILLEGAL); the engine
// turns that into an Illegal-Instruction trap (P2). RV32E out-of-range register
// indices (>= NUM_GPR) also raise illegal.
`ifndef INCLUDED_IBEX_DECODER_SVH
`define INCLUDED_IBEX_DECODER_SVH

class ibex_decoder;
    // Register-index bound: an index >= m_num_gpr is illegal (RV32E => 16).
    local int unsigned m_num_gpr;

    function new(int unsigned num_gpr = 32);
        m_num_gpr = num_gpr;
    endfunction

    // ---- immediate extractors (RISC-V formats, sign-extended where signed) ------
    local function ibex_word_t imm_i(logic [31:0] i);   // I-type: i[31:20]
        return {{20{i[31]}}, i[31:20]};
    endfunction
    local function ibex_word_t imm_s(logic [31:0] i);   // S-type: i[31:25|11:7]
        return {{20{i[31]}}, i[31:25], i[11:7]};
    endfunction
    local function ibex_word_t imm_b(logic [31:0] i);   // B-type: 13-bit, bit0=0
        return {{19{i[31]}}, i[31], i[7], i[30:25], i[11:8], 1'b0};
    endfunction
    local function ibex_word_t imm_u(logic [31:0] i);   // U-type: i[31:12]<<12
        return {i[31:12], 12'b0};
    endfunction
    local function ibex_word_t imm_j(logic [31:0] i);   // J-type: 21-bit, bit0=0
        return {{11{i[31]}}, i[31], i[19:12], i[20], i[30:21], 1'b0};
    endfunction

    // ---- main decode ------------------------------------------------------------
    function decoded_instr_t decode(logic [31:0] insn);
        decoded_instr_t d;
        logic [6:0] opcode = insn[6:0];
        logic [2:0] funct3 = insn[14:12];
        logic [6:0] funct7 = insn[31:25];

        // defaults: illegal, no writeback, no memory/CSR, 4-byte length. Each field
        // is set explicitly (an aggregate '{default:'0} can't init the enum members
        // under strict enum checking).
        d.op          = OP_ILLEGAL;
        d.rs1         = insn[19:15];
        d.rs2         = insn[24:20];
        d.rd          = insn[11:7];
        d.imm         = '0;
        d.a_sel       = A_RS1;
        d.b_sel       = B_IMM;
        d.rf_we       = 1'b0;
        d.mem         = MEM_NONE;
        d.csr_op      = CSR_NONE;
        d.csr_addr    = '0;
        d.csr_use_imm = 1'b0;
        d.illegal     = 1'b0;
        d.len         = 3'd4;

        case (opcode)
            // -------- register/immediate ALU -------------------------------------
            OPCODE_OP_IMM: begin
                d.a_sel = A_RS1; d.b_sel = B_IMM; d.rf_we = 1'b1;
                d.imm   = imm_i(insn);
                case (funct3)
                    3'b000: d.op = OP_ADD;                 // ADDI
                    3'b010: d.op = OP_SLT;                 // SLTI
                    3'b011: d.op = OP_SLTU;                // SLTIU
                    3'b100: d.op = OP_XOR;                 // XORI
                    3'b110: d.op = OP_OR;                  // ORI
                    3'b111: d.op = OP_AND;                 // ANDI
                    3'b001: begin                          // SLLI
                        if (funct7 == 7'b0000000) d.op = OP_SLL; else illegal(d);
                    end
                    3'b101: begin                          // SRLI / SRAI
                        if      (funct7 == 7'b0000000) d.op = OP_SRL;
                        else if (funct7 == 7'b0100000) d.op = OP_SRA;
                        else illegal(d);
                    end
                    default: illegal(d);
                endcase
            end

            // -------- register/register ALU + M extension ------------------------
            OPCODE_OP: begin
                d.a_sel = A_RS1; d.b_sel = B_RS2; d.rf_we = 1'b1;
                if (funct7 == 7'b0000001) begin             // M extension
                    case (funct3)
                        3'b000: d.op = OP_MUL;
                        3'b001: d.op = OP_MULH;
                        3'b010: d.op = OP_MULHSU;
                        3'b011: d.op = OP_MULHU;
                        3'b100: d.op = OP_DIV;
                        3'b101: d.op = OP_DIVU;
                        3'b110: d.op = OP_REM;
                        3'b111: d.op = OP_REMU;
                        default: illegal(d);
                    endcase
                end else begin
                    case (funct3)
                        3'b000: d.op = (funct7 == 7'b0100000) ? OP_SUB :
                                       (funct7 == 7'b0000000) ? OP_ADD : OP_ILLEGAL;
                        3'b001: d.op = (funct7 == 7'b0000000) ? OP_SLL  : OP_ILLEGAL;
                        3'b010: d.op = (funct7 == 7'b0000000) ? OP_SLT  : OP_ILLEGAL;
                        3'b011: d.op = (funct7 == 7'b0000000) ? OP_SLTU : OP_ILLEGAL;
                        3'b100: d.op = (funct7 == 7'b0000000) ? OP_XOR  : OP_ILLEGAL;
                        3'b101: d.op = (funct7 == 7'b0000000) ? OP_SRL  :
                                       (funct7 == 7'b0100000) ? OP_SRA  : OP_ILLEGAL;
                        3'b110: d.op = (funct7 == 7'b0000000) ? OP_OR   : OP_ILLEGAL;
                        3'b111: d.op = (funct7 == 7'b0000000) ? OP_AND  : OP_ILLEGAL;
                        default: ;
                    endcase
                    if (d.op == OP_ILLEGAL) illegal(d);
                end
            end

            // -------- upper immediate --------------------------------------------
            OPCODE_LUI: begin
                d.op = OP_LUI; d.rf_we = 1'b1; d.imm = imm_u(insn);
                d.a_sel = A_ZERO; d.b_sel = B_IMM;
            end
            OPCODE_AUIPC: begin
                d.op = OP_AUIPC; d.rf_we = 1'b1; d.imm = imm_u(insn);
                d.a_sel = A_PC; d.b_sel = B_IMM;
            end

            // -------- jumps ------------------------------------------------------
            OPCODE_JAL: begin
                d.op = OP_JAL; d.rf_we = 1'b1; d.imm = imm_j(insn);
            end
            OPCODE_JALR: begin
                if (funct3 == 3'b000) begin
                    d.op = OP_JALR; d.rf_we = 1'b1; d.imm = imm_i(insn);
                end else illegal(d);
            end

            // -------- branches ---------------------------------------------------
            OPCODE_BRANCH: begin
                d.imm = imm_b(insn);
                case (funct3)
                    3'b000: d.op = OP_BEQ;
                    3'b001: d.op = OP_BNE;
                    3'b100: d.op = OP_BLT;
                    3'b101: d.op = OP_BGE;
                    3'b110: d.op = OP_BLTU;
                    3'b111: d.op = OP_BGEU;
                    default: illegal(d);
                endcase
            end

            // -------- loads ------------------------------------------------------
            OPCODE_LOAD: begin
                d.op = OP_LOAD; d.rf_we = 1'b1; d.imm = imm_i(insn);
                d.a_sel = A_RS1; d.b_sel = B_IMM;
                case (funct3)
                    3'b000: d.mem = MEM_LB;
                    3'b001: d.mem = MEM_LH;
                    3'b010: d.mem = MEM_LW;
                    3'b100: d.mem = MEM_LBU;
                    3'b101: d.mem = MEM_LHU;
                    default: illegal(d);
                endcase
            end

            // -------- stores -----------------------------------------------------
            OPCODE_STORE: begin
                d.op = OP_STORE; d.imm = imm_s(insn);
                d.a_sel = A_RS1; d.b_sel = B_IMM;
                case (funct3)
                    3'b000: d.mem = MEM_SB;
                    3'b001: d.mem = MEM_SH;
                    3'b010: d.mem = MEM_SW;
                    default: illegal(d);
                endcase
            end

            // -------- fence (ordering no-op for a single-hart model) -------------
            OPCODE_MISC_MEM: begin
                // FENCE (funct3=000) / FENCE.I (funct3=001): no cache/reorder in the
                // model, so both are ordering no-ops. Other funct3 => illegal.
                if (funct3 == 3'b000 || funct3 == 3'b001) d.op = OP_FENCE;
                else illegal(d);
            end

            // -------- system / CSR ----------------------------------------------
            OPCODE_SYSTEM: begin
                if (funct3 == 3'b000) begin
                    // privileged: ECALL / EBREAK / MRET / WFI (by imm/funct7)
                    case (insn[31:20])
                        12'h000: d.op = OP_ECALL;
                        12'h001: d.op = OP_EBREAK;
                        12'h302: d.op = OP_MRET;
                        12'h105: d.op = OP_WFI;
                        default: illegal(d);
                    endcase
                end else begin
                    // Zicsr: CSRRW/S/C and immediate forms
                    d.op       = OP_CSR;
                    d.rf_we    = 1'b1;               // rd written with old CSR value
                    d.csr_addr = insn[31:20];
                    d.csr_use_imm = funct3[2];       // funct3[2] => immediate variant
                    case (funct3[1:0])
                        2'b01: d.csr_op = CSR_RW;     // CSRRW / CSRRWI
                        2'b10: d.csr_op = CSR_RS;     // CSRRS / CSRRSI
                        2'b11: d.csr_op = CSR_RC;     // CSRRC / CSRRCI
                        default: illegal(d);
                    endcase
                    // imm variants carry a 5-bit zero-extended immediate in rs1 field
                    d.imm = {27'b0, insn[19:15]};
                end
            end

            default: illegal(d);
        endcase

        // RV32E: any *used* register index >= NUM_GPR is illegal. Only fields the op
        // actually reads/writes matter, but checking all three is conservative and
        // matches Ibex (which flags on the decoded, in-use fields).
        if (!d.illegal && m_num_gpr < 32) begin
            if (uses_rs1(d) && int'(d.rs1) >= int'(m_num_gpr)) illegal(d);
            if (uses_rs2(d) && int'(d.rs2) >= int'(m_num_gpr)) illegal(d);
            if (d.rf_we    && int'(d.rd)  >= int'(m_num_gpr)) illegal(d);
        end

        return d;
    endfunction

    // ================================================================================
    // Compressed (RV32C) expansion. decompress() maps a 16-bit compressed instruction
    // to its equivalent 32-bit base instruction, which the fetch buffer then feeds
    // through decode() -- so all execution semantics are reused, and only the encoding
    // is undone here (the same strategy as ibex_compressed_decoder.sv). `illegal` is
    // set for reserved/unsupported (F/D/RV64) encodings; the caller raises the trap
    // with mtval = the original 16-bit pattern (zero-extended).
    // ================================================================================

    // 32-bit base-instruction field encoders (mirror the RISC-V formats).
    local function logic [31:0] enc_r(logic [6:0] op, logic [2:0] f3, logic [6:0] f7,
                                      logic [4:0] rd, logic [4:0] rs1, logic [4:0] rs2);
        return {f7, rs2, rs1, f3, rd, op};
    endfunction
    local function logic [31:0] enc_i(logic [6:0] op, logic [2:0] f3,
                                      logic [4:0] rd, logic [4:0] rs1, logic [31:0] imm);
        return {imm[11:0], rs1, f3, rd, op};
    endfunction
    local function logic [31:0] enc_s(logic [6:0] op, logic [2:0] f3,
                                      logic [4:0] rs1, logic [4:0] rs2, logic [31:0] imm);
        return {imm[11:5], rs2, rs1, f3, imm[4:0], op};
    endfunction
    local function logic [31:0] enc_b(logic [6:0] op, logic [2:0] f3,
                                      logic [4:0] rs1, logic [4:0] rs2, logic [31:0] imm);
        return {imm[12], imm[10:5], rs2, rs1, f3, imm[4:1], imm[11], op};
    endfunction
    local function logic [31:0] enc_u(logic [6:0] op, logic [4:0] rd, logic [31:0] imm);
        return {imm[31:12], rd, op};
    endfunction
    local function logic [31:0] enc_j(logic [6:0] op, logic [4:0] rd, logic [31:0] imm);
        return {imm[20], imm[10:1], imm[11], imm[19:12], rd, op};
    endfunction

    // 3-bit compressed register field -> x8..x15.
    local function logic [4:0] regp(logic [2:0] r); return {2'b01, r}; endfunction

    localparam logic [6:0] OPC_OP_IMM = 7'h13, OPC_OP = 7'h33, OPC_LOAD = 7'h03,
                           OPC_STORE  = 7'h23, OPC_LUI = 7'h37, OPC_BRANCH = 7'h63,
                           OPC_JAL    = 7'h6f, OPC_JALR = 7'h67;

    function logic [31:0] decompress(logic [15:0] c, output bit illegal);
        logic [1:0] quad   = c[1:0];
        logic [2:0] funct3 = c[15:13];
        // reconstructed immediates (sign-extended where signed)
        logic [31:0] cj  = {{20{c[12]}}, c[12], c[8], c[10:9], c[6], c[7], c[2], c[11], c[5:3], 1'b0};
        logic [31:0] cb  = {{23{c[12]}}, c[12], c[6:5], c[2], c[11:10], c[4:3], 1'b0};
        logic [31:0] i6  = {{26{c[12]}}, c[12], c[6:2]};   // 6-bit signed (ADDI/LI/ANDI/SRxI shamt sign)
        logic [4:0]  shamt = {c[6:2]};                      // RV32 shift amount (c[12] must be 0)
        illegal = 1'b0;

        case (quad)
            // -------- quadrant 0 --------------------------------------------------
            2'b00: case (funct3)
                3'b000: begin   // C.ADDI4SPN -> addi rd', x2, nzuimm
                    logic [31:0] nz = {22'b0, c[10:7], c[12:11], c[5], c[6], 2'b00};
                    if (nz == 0) illegal = 1'b1;   // reserved
                    return enc_i(OPC_OP_IMM, 3'b000, regp(c[4:2]), 5'd2, nz);
                end
                3'b010: begin   // C.LW -> lw rd', off(rs1')
                    logic [31:0] off = {25'b0, c[5], c[12:10], c[6], 2'b00};
                    return enc_i(OPC_LOAD, 3'b010, regp(c[4:2]), regp(c[9:7]), off);
                end
                3'b110: begin   // C.SW -> sw rs2', off(rs1')
                    logic [31:0] off = {25'b0, c[5], c[12:10], c[6], 2'b00};
                    return enc_s(OPC_STORE, 3'b010, regp(c[9:7]), regp(c[4:2]), off);
                end
                default: illegal = 1'b1;   // FLD/FLW/FSD/FSW/reserved: no F/D in RV32IMC
            endcase

            // -------- quadrant 1 --------------------------------------------------
            2'b01: case (funct3)
                3'b000: return enc_i(OPC_OP_IMM, 3'b000, c[11:7], c[11:7], i6);   // C.ADDI / C.NOP
                3'b001: return enc_j(OPC_JAL, 5'd1, cj);                          // C.JAL (RV32)
                3'b010: return enc_i(OPC_OP_IMM, 3'b000, c[11:7], 5'd0, i6);      // C.LI
                3'b011: begin
                    if (c[11:7] == 5'd2) begin   // C.ADDI16SP -> addi x2, x2, nzimm
                        logic [31:0] nz = {{22{c[12]}}, c[12], c[4:3], c[5], c[2], c[6], 4'b0};
                        if (nz == 0) illegal = 1'b1;
                        return enc_i(OPC_OP_IMM, 3'b000, 5'd2, 5'd2, nz);
                    end else begin               // C.LUI -> lui rd, nzimm
                        logic [31:0] nz = {{14{c[12]}}, c[12], c[6:2], 12'b0};
                        if (nz == 0 || c[11:7] == 5'd0) illegal = 1'b1;   // reserved / hint
                        return enc_u(OPC_LUI, c[11:7], nz);
                    end
                end
                3'b100: begin   // MISC-ALU
                    case (c[11:10])
                        2'b00: begin   // C.SRLI
                            if (c[12]) illegal = 1'b1;   // RV32: shamt[5] must be 0
                            return enc_i(OPC_OP_IMM, 3'b101, regp(c[9:7]), regp(c[9:7]), {27'b0, shamt});
                        end
                        2'b01: begin   // C.SRAI  (imm[11:5] = 0100000)
                            if (c[12]) illegal = 1'b1;
                            return enc_i(OPC_OP_IMM, 3'b101, regp(c[9:7]), regp(c[9:7]), {20'b0, 7'b0100000, shamt});
                        end
                        2'b10:         // C.ANDI
                            return enc_i(OPC_OP_IMM, 3'b111, regp(c[9:7]), regp(c[9:7]), i6);
                        2'b11: begin   // C.SUB/XOR/OR/AND (c[12]=0); SUBW/ADDW (c[12]=1) illegal RV32
                            if (c[12]) begin illegal = 1'b1; return 32'h0; end
                            case (c[6:5])
                                2'b00: return enc_r(OPC_OP, 3'b000, 7'b0100000, regp(c[9:7]), regp(c[9:7]), regp(c[4:2])); // SUB
                                2'b01: return enc_r(OPC_OP, 3'b100, 7'b0000000, regp(c[9:7]), regp(c[9:7]), regp(c[4:2])); // XOR
                                2'b10: return enc_r(OPC_OP, 3'b110, 7'b0000000, regp(c[9:7]), regp(c[9:7]), regp(c[4:2])); // OR
                                2'b11: return enc_r(OPC_OP, 3'b111, 7'b0000000, regp(c[9:7]), regp(c[9:7]), regp(c[4:2])); // AND
                                default: illegal = 1'b1;
                            endcase
                        end
                        default: illegal = 1'b1;
                    endcase
                end
                3'b101: return enc_j(OPC_JAL, 5'd0, cj);                               // C.J
                3'b110: return enc_b(OPC_BRANCH, 3'b000, regp(c[9:7]), 5'd0, cb);      // C.BEQZ
                3'b111: return enc_b(OPC_BRANCH, 3'b001, regp(c[9:7]), 5'd0, cb);      // C.BNEZ
                default: illegal = 1'b1;
            endcase

            // -------- quadrant 2 --------------------------------------------------
            2'b10: case (funct3)
                3'b000: begin   // C.SLLI
                    if (c[12]) illegal = 1'b1;   // RV32: shamt[5] must be 0
                    return enc_i(OPC_OP_IMM, 3'b001, c[11:7], c[11:7], {27'b0, shamt});
                end
                3'b010: begin   // C.LWSP -> lw rd, off(x2)
                    logic [31:0] off = {24'b0, c[3:2], c[12], c[6:4], 2'b00};
                    if (c[11:7] == 5'd0) illegal = 1'b1;   // reserved
                    return enc_i(OPC_LOAD, 3'b010, c[11:7], 5'd2, off);
                end
                3'b100: begin   // C.JR / C.MV / C.EBREAK / C.JALR / C.ADD
                    if (c[12] == 1'b0) begin
                        if (c[6:2] == 5'd0) begin   // C.JR
                            if (c[11:7] == 5'd0) illegal = 1'b1;   // reserved
                            return enc_i(OPC_JALR, 3'b000, 5'd0, c[11:7], 32'h0);
                        end else                    // C.MV -> add rd, x0, rs2
                            return enc_r(OPC_OP, 3'b000, 7'b0000000, c[11:7], 5'd0, c[6:2]);
                    end else begin
                        if (c[11:7] == 5'd0 && c[6:2] == 5'd0)      // C.EBREAK
                            return 32'h0010_0073;
                        else if (c[6:2] == 5'd0)                    // C.JALR
                            return enc_i(OPC_JALR, 3'b000, 5'd1, c[11:7], 32'h0);
                        else                                        // C.ADD -> add rd, rd, rs2
                            return enc_r(OPC_OP, 3'b000, 7'b0000000, c[11:7], c[11:7], c[6:2]);
                    end
                end
                3'b110: begin   // C.SWSP -> sw rs2, off(x2)
                    logic [31:0] off = {24'b0, c[8:7], c[12:9], 2'b00};
                    return enc_s(OPC_STORE, 3'b010, 5'd2, c[6:2], off);
                end
                default: illegal = 1'b1;   // C.FLDSP/FLWSP/FSDSP/FSWSP: no F/D
            endcase

            default: illegal = 1'b1;   // quad==11 is not compressed (caller filters)
        endcase

        if (illegal) return 32'h0;
        return 32'h0;   // unreachable (each arm returns), keeps the linter happy
    endfunction

    // ---- helpers ----------------------------------------------------------------
    local function void illegal(ref decoded_instr_t d);
        d.op      = OP_ILLEGAL;
        d.illegal = 1'b1;
        d.rf_we   = 1'b0;
        d.mem     = MEM_NONE;
        d.csr_op  = CSR_NONE;
    endfunction

    // Does this op read rs1 / rs2? Used for the RV32E range check and to populate
    // the RVFI rs1_addr/rs2_addr fields (0 when the register is not actually read).
    function bit uses_rs1(decoded_instr_t d);
        // CSR immediate variants do not read rs1 (the field is a zimm).
        if (d.csr_op != CSR_NONE) return !d.csr_use_imm;
        case (d.op)
            OP_LUI, OP_AUIPC, OP_JAL,
            OP_ECALL, OP_EBREAK, OP_MRET, OP_WFI, OP_FENCE, OP_ILLEGAL: return 1'b0;
            default: return 1'b1;
        endcase
    endfunction
    function bit uses_rs2(decoded_instr_t d);
        case (d.op)
            OP_STORE,
            OP_BEQ, OP_BNE, OP_BLT, OP_BGE, OP_BLTU, OP_BGEU: return 1'b1;
            OP_ADD, OP_SUB, OP_SLL, OP_SLT, OP_SLTU, OP_XOR, OP_SRL, OP_SRA, OP_OR, OP_AND:
                return (d.b_sel == B_RS2);
            OP_MUL, OP_MULH, OP_MULHSU, OP_MULHU,
            OP_DIV, OP_DIVU, OP_REM, OP_REMU: return 1'b1;
            default: return 1'b0;
        endcase
    endfunction

endclass

`endif // INCLUDED_IBEX_DECODER_SVH
