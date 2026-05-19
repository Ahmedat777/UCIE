module sb_error_handler (
    input  logic        clk,
    input  logic        rst_n,

    // -------------------------------------------------------------------------
    // Sideband packet from RX path (latched after serial deserialisation)
    // -------------------------------------------------------------------------
    input  logic        i_pkt_valid,    // header word is valid and complete
    input  logic [63:0] i_header,       // 64-bit sideband header flit
    input  logic [63:0] i_data,         // 64-bit data flit (ignored when !i_has_data)
    input  logic        i_has_data,     // 1 = a data flit accompanies this header

    // -------------------------------------------------------------------------
    // Outputs
    // -------------------------------------------------------------------------

    // Sideband parity error flags (combinational, for visibility / debug)
    output logic        o_cp_error,     // CP mismatch on current packet
    output logic        o_dp_error      // DP mismatch on current packet
);

    // =========================================================================
    // CP verification  
    // =========================================================================
    // CP = even_parity(header with CP and DP bit positions both zeroed).
    // Received CP is header[62]; received DP placeholder is header[63].

    logic [63:0] w_hdr_for_cp;
    logic        w_cp_calc;

    always_comb begin
        w_hdr_for_cp      = i_header;
        w_hdr_for_cp[62]  = 1'b0;   // zero out the CP field itself
        w_hdr_for_cp[63]  = 1'b0;   // zero out the DP field (excluded from CP)
        w_cp_calc         = ^w_hdr_for_cp; // even parity = XOR reduce
    end

    // CP mismatch: calculated parity must equal received CP bit
    assign o_cp_error = i_pkt_valid && (w_cp_calc != i_header[62]);

    // =========================================================================
    // DP verification  
    // =========================================================================
    // DP = even_parity(data payload). Zero when no data payload.
    // Received DP is header[63].

    logic w_dp_calc;
    assign w_dp_calc = i_has_data ? (^i_data) : 1'b0;

    // DP mismatch: only meaningful when a data flit is present
    assign o_dp_error = i_pkt_valid && i_has_data && (w_dp_calc != i_header[63]);

endmodule : sb_error_handler