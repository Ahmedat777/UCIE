module RX_initiated_D2C_wrapper (
  
  /* ================= Inputs ================= */
  input  wire                 i_clk,
  input  wire                 i_en_d2c_rx,
  input  wire                 i_rst_n,
  input  wire                 i_datavref_or_valvref,
  input  wire                 i_pattern_finished,
  input  wire                 i_SB_Busy,
  input  wire                 i_busy_negedge_detected,
  input  wire                 i_message_valid,
  input		wire	[3:0]			        i_decoded_sb_msg,
  input  wire [15:0]          i_per_lane_error,

  
  /* ================= Outputs ================= */
  output reg		[3:0]			        o_encoded_sb_msg,
  output wire [15:0]          o_data_bus,
  output reg  								        o_msg_valid,
  output wire                 o_data_valid,
  output	reg									         o_ack_rx,
  output wire [15:0]          o_per_lane_error,
  output wire                 o_val_pattern_en,
  output wire [1:0]           o_generator_mode_cw,
  output wire                 o_comparison_valid_en,
  output wire [1:0]           o_comparator_mode_cw
  );
  
  wire       msg_valid_tx , msg_valid_rx;
  wire [3:0] encoded_sb_msg_tx , encoded_sb_msg_rx;
  wire       ack_tx , ack_rx;
  wire       sb_data_pattern;
  wire       sb_burst_count;
  wire       sb_comparison_mode;
  wire [1:0] clock_phase;
  wire       o_data_valid_tx;
  wire       o_val_pattern_en_tx;
  wire [1:0] o_generator_mode_cw_tx;
  wire       o_comparison_valid_en_rx;
  wire [1:0] o_comparator_mode_cw_rx;
  
  
  assign o_data_bus = {11'b0, sb_comparison_mode, sb_burst_count, clock_phase, sb_data_pattern};
  //assign o_ack_rx   = ack_tx & ack_rx;
  assign o_per_lane_error = i_per_lane_error;
  //assign o_msg_valid = msg_valid_tx | msg_valid_rx;
  assign o_data_valid = o_data_valid_tx;
  assign o_val_pattern_en = o_val_pattern_en_tx;
  assign o_generator_mode_cw = o_generator_mode_cw_tx;
  assign o_comparison_valid_en = o_comparison_valid_en_rx;
  assign o_comparator_mode_cw = o_comparator_mode_cw_rx;
  
   /* ================= TX ================= */
  RX_initiated_D2C_tx u_tx (
    .i_clk                   (i_clk),
    .i_rst_n                 (i_rst_n),
    .i_en_d2c_rx             (i_en_d2c_rx),
    .i_rx_valid              (msg_valid_rx),
    .i_busy_negedge_detected (i_busy_negedge_detected),
    .i_datavref_or_valvref   (i_datavref_or_valvref),
    .i_pattern_finished      (i_pattern_finished),
    .i_message_valid         (i_message_valid),
    .i_decoded_sb_msg        (i_decoded_sb_msg),

    .o_encoded_sb_msg        (encoded_sb_msg_tx),
    .o_sb_data_pattern       (sb_data_pattern),
    .o_sb_burst_count        (sb_burst_count),
    .o_sb_comparison_mode    (sb_comparison_mode),
    .o_clock_phase           (clock_phase),
    .o_data_valid            (o_data_valid_tx),
    .o_ack_rx_tx             (ack_tx),
    .o_msg_valid_tx          (msg_valid_tx),
    .o_val_pattern_en        (o_val_pattern_en_tx),
    .o_generator_mode_cw     (o_generator_mode_cw_tx)
  );

  /* ================= RX ================= */
  RX_initiated_D2C_rx u_rx (
    .i_clk                   (i_clk),
    .i_rst_n                 (i_rst_n),
    .i_en_d2c_rx             (i_en_d2c_rx),
    .i_tx_valid              (msg_valid_tx),
    .i_busy_negedge_detected (i_busy_negedge_detected),
    .i_SB_Busy               (i_SB_Busy),
    .i_datavref_or_valvref   (i_datavref_or_valvref),
    .i_message_valid         (i_message_valid),
    .i_decoded_sb_msg        (i_decoded_sb_msg),

    .o_encoded_sb_msg        (encoded_sb_msg_rx),
    .o_ack_rx_rx             (ack_rx),
    .o_msg_valid_rx          (msg_valid_rx),
    .o_comparison_valid_en   (o_comparison_valid_en_rx),
    .o_comparator_mode_cw    (o_comparator_mode_cw_rx)
  );

  /* ================= Wrapper arbitration ================= */
  always @(*) begin
    /* defaults */
    o_encoded_sb_msg = 4'b0000;
    o_msg_valid      = 1'b0;
    o_ack_rx         = 1'b0;

    /* TX has priority */
    if (msg_valid_tx) begin
      o_encoded_sb_msg = encoded_sb_msg_tx;
      o_msg_valid      = 1'b1;
    end
    else if (msg_valid_rx) begin
      o_encoded_sb_msg = encoded_sb_msg_rx;
      o_msg_valid      = 1'b1;
    end

    /* ACK only when both sides are done */
    o_ack_rx = ack_tx & ack_rx;
  end

endmodule
  
  
  
  
  
