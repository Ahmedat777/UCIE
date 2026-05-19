module SB_PACKET_ENCODER_MUX (
    // ── SBINIT pattern generator ──────────────────────────────────────────────
    input  logic [63:0] i_pattern,            // 64-bit alternating pattern word
    input  logic        i_pattern_valid,      // pattern word valid this cycle

    // ── ltsm_encoder (header flit + has_data flag) ────────────────────────────
    input  logic [63:0] i_enc_header,         // fully encoded 64-bit header flit
    input  logic        i_enc_has_data,       // 1 = this message has a data flit
    input  logic        i_encoded_pkt_valid,  // encoder outputs valid this cycle

    // ── Selected output ───────────────────────────────────────────────────────
    output logic [63:0] o_final_packet,       // chosen word → SB_TX_FIFO
    output logic        o_final_valid,        
    output logic        o_final_has_data      // wrapper must queue data flit next
);

    always_comb begin
        o_final_packet   = 64'd0;
        o_final_valid    = 1'b0;
        o_final_has_data = 1'b0;

        // Priority 1 – SBINIT / PSTP: no flow-control check (§4.1.5.2)
        if (i_pattern_valid) begin
            o_final_packet   = i_pattern;
            o_final_valid    = 1'b1;
            o_final_has_data = 1'b0;   // pattern is always a single 64-bit word

        // Priority 2 – encoded sideband header from ltsm_encoder
        end else if (i_encoded_pkt_valid) begin
            o_final_packet   = i_enc_header;
            o_final_valid    = 1'b1;
            o_final_has_data = i_enc_has_data;

        // Idle – "SB Transmitters continue to be held Low" (§4.1.5)
        end
    end

endmodule : SB_PACKET_ENCODER_MUX