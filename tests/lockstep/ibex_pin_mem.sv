// ======================================================================
// ibex_pin_mem -- a trivial 0-wait-state memory for the Ibex RTL side of the
// RVFI lockstep. It speaks Ibex's OBI-like instruction/data protocol
// (req/gnt/rvalid), which the SPL model's fw_mem_if abstracts away.
//
// Protocol (matches Ibex's "zero wait state" expectation, e.g. simple_system):
//   - grant is combinational (gnt = req): every request is accepted the cycle
//     it is presented.
//   - the response (rvalid + rdata / write-ack) follows a granted request by
//     exactly one clock. One outstanding transaction, which is all Ibex needs
//     when gnt is immediate.
//
// A single word array backs BOTH ports (unified instr+data space, like the real
// program image). It is marked public so the testbench can back-door load the
// program (identical bytes to the SPL model's fw_mem_flat) during reset.
// ======================================================================
module ibex_pin_mem #(
    parameter int unsigned WORDS = 8192,   // 32 KiB; covers [0, 0x7FFF]
    parameter int unsigned AW    = 32
) (
    input  logic          clk_i,
    input  logic          rst_ni,

    // Instruction port (read-only)
    input  logic          instr_req_i,
    output logic          instr_gnt_o,
    output logic          instr_rvalid_o,
    input  logic [AW-1:0] instr_addr_i,
    output logic [31:0]   instr_rdata_o,
    output logic          instr_err_o,

    // Data port (read/write, byte-enabled)
    input  logic          data_req_i,
    output logic          data_gnt_o,
    output logic          data_rvalid_o,
    input  logic          data_we_i,
    input  logic [3:0]    data_be_i,
    input  logic [AW-1:0] data_addr_i,
    input  logic [31:0]   data_wdata_i,
    output logic [31:0]   data_rdata_o,
    output logic          data_err_o
);
    // Word-addressed backing store, public for back-door program load.
    logic [31:0] mem [0:WORDS-1] /*verilator public_flat_rw*/;

    // Word index from a byte address.
    function automatic int unsigned idx(logic [AW-1:0] a);
        return a[$clog2(WORDS)+1:2];
    endfunction

    // Grants are immediate (zero wait state); this memory never errors in range.
    // The core can therefore retire up to one instruction per cycle -- the
    // fw-proto-rvfi monitor sustains that drain rate.
    assign instr_gnt_o = instr_req_i;
    assign data_gnt_o  = data_req_i;
    assign instr_err_o = 1'b0;
    assign data_err_o  = 1'b0;

    // Instruction response: one-cycle latency, read-only.
    logic        instr_rvalid_q;
    logic [31:0] instr_rdata_q;
    always_ff @(posedge clk_i or negedge rst_ni) begin
        if (!rst_ni) begin
            instr_rvalid_q <= 1'b0;
            instr_rdata_q  <= '0;
        end else begin
            instr_rvalid_q <= instr_req_i & instr_gnt_o;
            if (instr_req_i & instr_gnt_o)
                instr_rdata_q <= mem[idx(instr_addr_i)];
        end
    end
    assign instr_rvalid_o = instr_rvalid_q;
    assign instr_rdata_o  = instr_rdata_q;

    // Data response: one-cycle latency; byte-enabled writes commit on grant.
    logic        data_rvalid_q;
    logic [31:0] data_rdata_q;
    always_ff @(posedge clk_i or negedge rst_ni) begin
        if (!rst_ni) begin
            data_rvalid_q <= 1'b0;
            data_rdata_q  <= '0;
        end else begin
            data_rvalid_q <= data_req_i & data_gnt_o;
            if (data_req_i & data_gnt_o) begin
                if (data_we_i) begin
                    for (int b = 0; b < 4; b++)
                        if (data_be_i[b])
                            mem[idx(data_addr_i)][b*8 +: 8] <= data_wdata_i[b*8 +: 8];
                end else begin
                    data_rdata_q <= mem[idx(data_addr_i)];
                end
            end
        end
    end
    assign data_rvalid_o = data_rvalid_q;
    assign data_rdata_o  = data_rdata_q;
endmodule
