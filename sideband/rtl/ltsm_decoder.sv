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