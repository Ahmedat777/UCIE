// =============================================================================
// ltsm_decoder  –  UCIe Sideband Packet Decoder
//
// Spec reference: UCIe Specification Rev 3.0 Ver 1.0 (August 2025)
//   §7.1.2   Sideband packet formats; field layout
//
// Role in the receive path:
//   SerDes → SB_RX_FIFO → ltsm_decoder → SB_RX_FSM → TIME_OUT_COUNTER
//                       → sb_error_handler
//
//   Parity is NOT checked here. CP/DP verification is handled exclusively
//   by sb_error_handler, which reads the header and data directly from the
//   RX FIFO. The decoder does not forward raw parity bits.
//
// Header bit layout (matches ltsm_encoder.sv / packet spec Phase0/Phase1):
//   Phase0 (header[31:0]):
//     [4:0]   opcode[4:0]
//     [23:16] msgcode[7:0]
//     [31:29] srcid[2:0]
//   Phase1 (header[63:32]):
//     [39:32] msgsubcode[7:0]
//     [55:40] MsgInfo[15:0]
//     [58:56] dstid[2:0]
//     [62]    CP
//     [63]    DP
//
// Timeout-controller interface:
//   o_timeout_stop – pulses when ANY valid message is decoded. Any RX traffic
//                    proves the sub-state is progressing → stop the 8-ms timer.
//                    Replaces the old separate resp/stall signals; the timeout
//                    counter no longer distinguishes REQ from RESP.
// =============================================================================

import pckg::*;

module ltsm_decoder (
    input  logic        clk,
    input  logic        rst,

    input  logic        i_pckt_valid,   // SB_RX_FSM o_msg_valid
    input  logic [63:0] i_header,       // received 64-bit header
    input  logic [63:0] i_data,         // received 64-bit data flit (0 if no data)

    output logic        o_valid,        // decoded message found in SB_MSG_LUT
    output sb_msg_id    o_msg_id,       // identified message enum value
    output logic [15:0] o_msginfo,      // MsgInfo field [55:40]
    output logic [63:0] o_data,         // data flit (valid when has_data)

    // Timeout-controller interface (TIME_OUT_COUNTER)
    output logic        o_timeout_stop  // any valid RX message → stop 8-ms timer
);

    logic [7:0]  msgcode;
    logic [7:0]  msgsubcode;
    integer      i;
    logic        found;

    always_ff @(posedge clk or posedge rst) begin

        if (rst) begin
            o_valid        <= 1'b0;
            o_msg_id       <= sb_msg_id'(0);
            o_msginfo      <= 16'b0;
            o_data         <= 64'b0;
            o_timeout_stop <= 1'b0;
        end

        else begin
            o_valid        <= 1'b0;
            o_timeout_stop <= 1'b0;

            if (i_pckt_valid) begin

                /* ----------------------------
                 * FIELD EXTRACTION
                 * Phase0/Phase1 layout per ltsm_encoder.sv
                 * ---------------------------- */
                msgcode      = i_header[23:16];     // msgcode   at Phase0[23:16]
                msgsubcode   = i_header[39:32];     // msgsubcode at Phase1[39:32]
                o_msginfo   <= i_header[55:40];     // MsgInfo   at Phase1[55:40]

                /* ----------------------------
                 * FIND MESSAGE ID
                 * ---------------------------- */
                found = 1'b0;

                for (i = 0; i < $size(SB_MSG_LUT); i++) begin
                    if (SB_MSG_LUT[i].msgcode    == msgcode &&
                        SB_MSG_LUT[i].msgsubcode == msgsubcode) begin
                        o_msg_id = sb_msg_id'(i);
                        found    = 1'b1;
                    end
                end

                /* ----------------------------
                 * DATA OUTPUT
                 * ---------------------------- */
                if (found && SB_MSG_LUT[o_msg_id].has_data)
                    o_data <= i_data;
                else
                    o_data <= 64'b0;

                o_valid <= found;

                /* ----------------------------
                 * TIMEOUT STOP
                 * Any valid decoded message stops the 8-ms timer — no
                 * distinction between REQ and RESP needed here.
                 * ---------------------------- */
                if (found)
                    o_timeout_stop <= 1'b1;

            end
        end
    end

endmodule : ltsm_decoder