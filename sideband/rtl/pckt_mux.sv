// =============================================================================
// SB_PACKET_ENCODER_MUX  –  UCIe Sideband Packet Output Multiplexer
//
// Spec reference: UCIe Specification Rev 3.0 Ver 1.0 (August 2025)
//   §4.1.5.2  Priority Sideband Packet Transfer (PSTP): "must always be
//             accepted and make forward progress – there is no flow control
//             check at the Transmitter for them."
//   §7.1.1    Opcode 11110b / 11111b = Priority Packet (no credits checked)
//
// Input sources:
//   i_pattern       – start-pattern word from tx_pattern_generator (SBINIT)
//                     OR any PSTP priority word
//   i_encoded_packet – fully encoded 64-bit packet from ltsm_encoder
//                     (o_ltsm_header or o_ltsm_data; ltsm_encoder builds the
//                      complete packet including CP/DP in one clock cycle —
//                      there is no separate framing stage)
//
// Mux priority (highest to lowest):
//   1. Priority pattern / PSTP  (i_pattern_valid)
//   2. Encoded sideband packet  (i_encoded_pkt_valid)
//   3. Drive-low when idle (spec §4.1.5: "SB Transmitters held Low")
//
// If both valids arrive simultaneously, pattern takes precedence
// (consistent with PSTP priority rule, spec §4.1.5.2).
// =============================================================================

module SB_PACKET_ENCODER_MUX (
    input  [63:0]  i_pattern,             // start-pattern / PSTP word
    input  [63:0]  i_encoded_packet,      // ltsm_encoder output (header or data)
    input          i_pattern_valid,       // i_pattern should be output
    input          i_encoded_pkt_valid,   // i_encoded_packet should be output
    output reg [63:0] o_final_packet      // muxed output to serialiser
);

    always_comb begin
        // Priority 1: start-pattern / PSTP (no flow control, spec §4.1.5.2)
        if (i_pattern_valid)
            o_final_packet = i_pattern;

        // Priority 2: encoded sideband packet from ltsm_encoder
        else if (i_encoded_pkt_valid)
            o_final_packet = i_encoded_packet;

        // Idle: spec §4.1.5 – "SB Transmitters continue to be held Low"
        else
            o_final_packet = 64'd0;
    end

endmodule : SB_PACKET_ENCODER_MUX