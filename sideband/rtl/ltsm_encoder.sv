import pckg::*;

module ltsm_encoder ( input rst,
                    input clk,
                    input reg i_ltsm_valid,
                    input sb_msg_id i_msg_id,
                    input reg [63:0] ltsm_data,
                    input reg [15:0] ltsm_msginfo,
                    output reg o_ltsmpckt_valid,
                    output reg [63:0] o_ltsm_header,
                    output reg [63:0] o_ltsm_data,
                    // Timeout counter interface
                    output reg        o_timeout_start,  // pulse when packet built → start timer
                    output sb_msg_id  o_timeout_msg_id  // which msg was sent → exclude check
);


function automatic logic even_parity(input logic [63:0] v);
  even_parity = ^v; // XOR reduction → even parity
endfunction

always_ff @(posedge clk or posedge rst) begin
  if (rst) begin
    o_ltsmpckt_valid  <= 1'b0;
    o_ltsm_header     <= 64'b0;
    o_ltsm_data       <= 64'b0;
    o_timeout_start   <= 1'b0;
    o_timeout_msg_id  <= sb_msg_id'(0);
  end else begin
    o_ltsmpckt_valid  <= 1'b0;
    o_timeout_start   <= 1'b0;

    if (i_ltsm_valid) begin
      logic [63:0] header;
      logic        dp_bit;
      logic        cp_bit;
      logic        has_data;
      logic [63:0] header_cp_calc;


      has_data = SB_MSG_LUT[i_msg_id].has_data;

      /* ----------------------------
       * DATA PARITY (DP)
       * ---------------------------- */
      if (has_data)
        dp_bit = even_parity(ltsm_data);
      else
        dp_bit = 1'b0;

      /* ----------------------------
       * BUILD HEADER
       * ---------------------------- */
      header = 64'b0;

      // --- Phase0 (header[31:0]) ---
      header[4:0]   = SB_MSG_LUT[i_msg_id].opcode;       // opcode[4:0]
      header[23:16] = SB_MSG_LUT[i_msg_id].msgcode;      // msgcode[7:0]
      header[31:29] = 3'b010;                             // srcid[2:0]

      // --- Phase1 (header[63:32]) ---
      header[39:32] = SB_MSG_LUT[i_msg_id].msgsubcode;   // MsgSubcode[7:0]
      header[55:40] = ltsm_msginfo;                       // MsgInfo[15:0]
      header[58:56] = 3'b110;                             // dstid[2:0]
      header[62]    = 1'b0;                               // CP placeholder
      header[63]    = dp_bit;                             // DP

      /* ----------------------------
       * CONTROL PARITY (CP)
       * Exclude DP bit [63]
       * ---------------------------- */
      header_cp_calc     = header;
      header_cp_calc[63] = 1'b0;                          // exclude DP
      cp_bit             = even_parity(header_cp_calc);

      header[62] = cp_bit;

      /* ----------------------------
       * OUTPUT
       * ---------------------------- */
      o_ltsm_header <= header;

      if (has_data)
        o_ltsm_data <= ltsm_data;
      else
        o_ltsm_data <= 64'b0;

      o_ltsmpckt_valid  <= 1'b1;
      o_timeout_start   <= 1'b1;   // tell timeout counter a message was sent
      o_timeout_msg_id  <= i_msg_id; // pass msg_id for exclusion check
    end
  end
end

endmodule: ltsm_encoder