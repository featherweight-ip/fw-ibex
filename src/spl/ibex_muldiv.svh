// Pure M-extension (mul/div) operations. Like ibex_alu, a bag of static functions
// the engine calls -- no state, no ports. Implements the RV32M semantics exactly,
// including the specified div-by-zero and signed-overflow results (which never trap
// in RISC-V; they return defined values).
`ifndef INCLUDED_IBEX_MULDIV_SVH
`define INCLUDED_IBEX_MULDIV_SVH

class ibex_muldiv;

    // Low 32 bits of the product (sign-agnostic).
    static function ibex_word_t mul(ibex_word_t a, ibex_word_t b);
        return ibex_word_t'(a * b);
    endfunction

    // High 32 bits of signed(a) * signed(b).
    static function ibex_word_t mulh(ibex_word_t a, ibex_word_t b);
        logic signed [63:0] sa = $signed(a);   // sign-extend 32 -> 64
        logic signed [63:0] sb = $signed(b);
        logic signed [63:0] p  = sa * sb;
        return p[63:32];
    endfunction

    // High 32 bits of signed(a) * unsigned(b).
    static function ibex_word_t mulhsu(ibex_word_t a, ibex_word_t b);
        logic signed [63:0] sa = $signed(a);           // signed
        logic signed [63:0] ub = signed'({32'b0, b});  // b as a non-negative 64-bit
        logic signed [63:0] p  = sa * ub;
        return p[63:32];
    endfunction

    // High 32 bits of unsigned(a) * unsigned(b).
    static function ibex_word_t mulhu(ibex_word_t a, ibex_word_t b);
        logic [63:0] ua = a;   // zero-extend
        logic [63:0] ub = b;
        logic [63:0] p  = ua * ub;
        return p[63:32];
    endfunction

    // Signed divide. Div-by-zero => -1; signed overflow (INT_MIN / -1) => INT_MIN.
    static function ibex_word_t div(ibex_word_t a, ibex_word_t b);
        if (b == 32'h0)                                return 32'hFFFF_FFFF;
        if (a == 32'h8000_0000 && b == 32'hFFFF_FFFF)  return 32'h8000_0000;
        return ibex_word_t'($signed(a) / $signed(b));
    endfunction

    // Unsigned divide. Div-by-zero => all ones.
    static function ibex_word_t divu(ibex_word_t a, ibex_word_t b);
        if (b == 32'h0) return 32'hFFFF_FFFF;
        return a / b;
    endfunction

    // Signed remainder. Rem-by-zero => dividend; signed overflow => 0.
    static function ibex_word_t rem(ibex_word_t a, ibex_word_t b);
        if (b == 32'h0)                                return a;
        if (a == 32'h8000_0000 && b == 32'hFFFF_FFFF)  return 32'h0;
        return ibex_word_t'($signed(a) % $signed(b));
    endfunction

    // Unsigned remainder. Rem-by-zero => dividend.
    static function ibex_word_t remu(ibex_word_t a, ibex_word_t b);
        if (b == 32'h0) return a;
        return a % b;
    endfunction

    // Dispatch by semantic op (OP_MUL .. OP_REMU).
    static function ibex_word_t exec(ibex_op_e op, ibex_word_t a, ibex_word_t b);
        case (op)
            OP_MUL:    return mul(a, b);
            OP_MULH:   return mulh(a, b);
            OP_MULHSU: return mulhsu(a, b);
            OP_MULHU:  return mulhu(a, b);
            OP_DIV:    return div(a, b);
            OP_DIVU:   return divu(a, b);
            OP_REM:    return rem(a, b);
            OP_REMU:   return remu(a, b);
            default:   return 'x;
        endcase
    endfunction

endclass

`endif // INCLUDED_IBEX_MULDIV_SVH
