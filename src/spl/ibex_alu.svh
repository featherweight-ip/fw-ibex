// Pure ALU for the Ibex SPL model. No state, no ports, no process -- a bag of
// static functions the engine calls to compute a result from two 32-bit operands.
// Being pure it is trivially unit-testable and carries no clock/side effects.
//
// Covers the RV32I integer ALU (ADD/SUB, shifts, logicals, set-less-than) plus the
// branch comparators. The M-extension ops live in ibex_muldiv (P3).
`ifndef INCLUDED_IBEX_ALU_SVH
`define INCLUDED_IBEX_ALU_SVH

class ibex_alu;

    // Compute a register/immediate ALU result for the integer ops. `op` must be one
    // of the OP_ADD..OP_AND set; other ops return 'x (a decode/dispatch bug).
    static function ibex_word_t exec(ibex_op_e op, ibex_word_t a, ibex_word_t b);
        logic [4:0] shamt = b[4:0];              // RV32 shift amount is b[4:0]
        case (op)
            OP_ADD:  return a + b;
            OP_SUB:  return a - b;
            OP_SLL:  return a << shamt;
            OP_SRL:  return a >> shamt;
            OP_SRA:  return $signed(a) >>> shamt;
            OP_AND:  return a & b;
            OP_OR:   return a | b;
            OP_XOR:  return a ^ b;
            OP_SLT:  return ($signed(a) < $signed(b)) ? 32'd1 : 32'd0;
            OP_SLTU: return (a < b) ? 32'd1 : 32'd0;
            default: return 'x;
        endcase
    endfunction

    // Evaluate a branch condition. `op` is one of OP_BEQ..OP_BGEU; returns 1 when
    // the branch is taken.
    static function bit branch_taken(ibex_op_e op, ibex_word_t a, ibex_word_t b);
        case (op)
            OP_BEQ:  return (a === b);
            OP_BNE:  return (a !== b);
            OP_BLT:  return ($signed(a) <  $signed(b));
            OP_BGE:  return ($signed(a) >= $signed(b));
            OP_BLTU: return (a <  b);
            OP_BGEU: return (a >= b);
            default: return 1'b0;
        endcase
    endfunction

endclass

`endif // INCLUDED_IBEX_ALU_SVH
