module TX_initiated_D2C_wrapper #(
    parameter NUM_LANE = 16
)(
    //=====================
    // Global
    //=====================
    input  wire                 i_clk,
    input  wire                 i_rst_n,
    input  wire                 i_en,

    //=====================
    // Sideband from outside
    //=====================
    input  wire [3:0]           i_sideband_message,
    input  wire [NUM_LANE-1:0]  i_sideband_data,
    input  wire                 i_sideband_message_valid,
    input  wire                 i_busy_negedge_detected,

    //=====================
    // Test configuration
    //=====================
    input  wire                 i_mainband_or_valtrain_test,
    input  wire                 i_lfsr_or_perlane,
    input  wire                 i_pattern_finished,

    //=====================
    // Comparator results
    //=====================
    input  wire [NUM_LANE-1:0]  i_per_lane_error,

    //=====================
    // Unified sideband outputs
    //=====================
    output wire [3:0]           o_sideband_message,
    output wire                 o_message_valid,
    output wire [NUM_LANE-1:0]  o_sideband_data,
    output wire                 o_data_valid,

    //=====================
    // Generator & Comparator control
    //=====================
    output wire                 o_val_pattern_en,
    output wire [1:0]           o_generator_mode_cw,
    output wire [1:0]           o_comparator_mode_cw,
    output wire                 o_comparison_valid_en,

    //=====================
    // Test done
    //=====================
    output wire                 o_test_ack,
    output wire [NUM_LANE-1:0]  o_mainband_lanes_result
);

    //====================================================
    // TX <-> RX interconnect signals
    //====================================================
    wire [3:0]          sb_msg_tx, sb_msg_rx;
    wire                valid_tx,  valid_rx;
    wire [NUM_LANE-1:0] sb_data_tx, sb_data_rx;
    wire                data_valid_tx, data_valid_rx;
    wire                ack_tx, ack_rx;

    //====================================================
    // TX instance
    //====================================================
    TX_initiated_D2C_tx #(
        .NUM_LANE(NUM_LANE)
    ) u_tx (
        .i_clk                      (i_clk),
        .i_rst_n                    (i_rst_n),
        .i_en_d2c_tx                (i_en),

        .i_mainband_or_valtrain_test(i_mainband_or_valtrain_test),
        .i_lfsr_or_perlane          (i_lfsr_or_perlane),
        .i_pattern_finished         (i_pattern_finished),

        .i_sideband_message         (i_sideband_message),
        .i_sideband_message_valid   (i_sideband_message_valid),
        .i_busy_negedge_detected    (i_busy_negedge_detected),
        .i_valid_rx                 (valid_rx),

        .o_sideband_message         (sb_msg_tx),
        .o_message_valid            (valid_tx),
        .o_sideband_data            (sb_data_tx),
        .o_data_valid               (data_valid_tx),

        .o_val_pattern_en           (o_val_pattern_en),
        .o_generator_mode_cw        (o_generator_mode_cw),
        .o_test_ack_tx              (ack_tx)
    );

    //====================================================
    // RX instance
    //====================================================
    TX_initiated_D2C_rx #(
        .NUM_LANE(NUM_LANE)
    ) u_rx (
        .i_clk                      (i_clk),
        .i_rst_n                    (i_rst_n),
        .i_en_d2c_tx                (i_en),

        .i_valid_tx                 (valid_tx),
        .i_busy_negedge_detected    (i_busy_negedge_detected),

        .i_mainband_or_valtrain_test(i_mainband_or_valtrain_test),
        .i_lfsr_or_perlane          (i_lfsr_or_perlane),

        .i_sideband_message         (i_sideband_message),
        .i_sideband_message_valid   (i_sideband_message_valid),

        .i_per_lane_error           (i_per_lane_error),

        .o_sideband_message         (sb_msg_rx),
        .o_message_valid            (valid_rx),
        .o_sideband_data            (sb_data_rx),
        .o_data_valid               (data_valid_rx),

        .o_comparator_mode_cw       (o_comparator_mode_cw),
        .o_comparison_valid_en      (o_comparison_valid_en),
        .o_test_ack_rx              (ack_rx)
    );

    //====================================================
    // Unified outputs (TX priority)
    //====================================================
    //assign o_sideband_message = valid_tx ? sb_msg_tx : sb_msg_rx;
    assign o_message_valid    = valid_tx | valid_rx;

    //assign o_sideband_data    = data_valid_tx ? sb_data_tx : sb_data_rx;
    assign o_data_valid       = data_valid_tx | data_valid_rx;

    assign o_test_ack         = ack_tx & ack_rx;

    // Reference behavior
    assign o_mainband_lanes_result = i_sideband_data;
    
    reg [3:0] o_sideband_message_r;

always @(*) begin
    if (!valid_tx && !valid_rx)
        o_sideband_message_r = 4'b0000;
    else if (valid_tx && !valid_rx)
        o_sideband_message_r = sb_msg_tx;
    else
        o_sideband_message_r = sb_msg_rx; // covers rx only & both active
end

assign o_sideband_message = o_sideband_message_r;

reg [NUM_LANE-1:0] o_sideband_data_r;

always @(*) begin
    if (!data_valid_tx && !data_valid_rx)
        o_sideband_data_r = {NUM_LANE{1'b0}};
    else if (data_valid_tx && !data_valid_rx)
        o_sideband_data_r = sb_data_tx;
    else
        o_sideband_data_r = sb_data_rx; // rx only or both
end

assign o_sideband_data = o_sideband_data_r;

endmodule