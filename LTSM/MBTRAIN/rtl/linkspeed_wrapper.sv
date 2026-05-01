module mbtrain_linkspeed_wrapper # ( parameter SB_MSG_WIDTH = 4, parameter LANE_STATUS_WIDTH = 2)
 (
// inputs
    input          i_clk,
    input          i_rst_n,
    input          i_enable,

    // communication with Sideband
    input   [SB_MSG_WIDTH-1:0]  i_sb_msg,
    input                       i_sb_valid,
    input                       i_falling_edge_busy,

    // Test / Comparator Results 
    input          i_point_test_ack,
    input   [15:0] i_lane_results,   // 1 = the lane pass threshold, 0 = fail

    // from mb train controller
    input          i_phy_in_retrain_flag,    
    input          i_tx_first_8_lanes_ok_mbinit ,i_tx_second_8_lanes_ok_mbinit ,  
    
    // from reg file
    input          i_runtime_config_change,  // Runtime Link Test Control change (some configs saved in the reg file)
 
 // outputs 
    output reg         o_point_test_en,

    // communication with Sideband 
    output reg  [SB_MSG_WIDTH-1:0]  o_sb_msg,
    output reg                      o_valid, 
    output reg                      o_timeout_disable, 
    // to phyretrain block
    output reg  [LANE_STATUS_WIDTH-1:0]    o_linkspeed_lanes_status,   // 0h: IDLE , 1h: No Lane errors, 2h: Lane errors & faulty Lanes are repairable, 3h: Lane errors & faulty Lanes cannot be repaired
    // communication with the controller
    output reg                      o_tx_first_8_lanes_ok_linkspeed , o_tx_second_8_lanes_ok_linkspeed,
    output reg                      o_clear_phy_in_retrain_flag,
    output reg                      o_exit_phyretrain,
    output reg                      o_exit_repair,
    output reg                      o_exit_speedidle,
    output reg                      o_ack

 );

///////////////////////// TX signals //////////////////////////////

wire                       o_valid_tx;
wire  [SB_MSG_WIDTH-1:0]   o_sb_msg_tx;
wire                       o_point_test_en_tx;
wire                       o_ack_tx;

///////////////////////// RX signals //////////////////////////////

wire                       o_valid_rx;
wire  [SB_MSG_WIDTH-1:0]   o_sb_msg_rx;
wire                       o_point_test_en_rx;
wire                       o_ack_rx;


//////////////////// Combining TX and RX outputs ////////////////////

assign o_valid = o_valid_tx || o_valid_rx;
assign o_point_test_en = o_point_test_en_tx && o_point_test_en_rx;
assign o_ack = o_ack_tx && o_ack_rx;


// sideband message
always @(*) begin
   case({o_valid_tx,o_valid_rx})
     2'b00 : o_sb_msg = 'b0;
     2'b10 : o_sb_msg = o_sb_msg_tx;
     2'b01 : o_sb_msg = o_sb_msg_rx;
     2'b11 : o_sb_msg = o_sb_msg_rx;
   endcase
end

 mbtrain_linkspeed_tx #(.SB_MSG_WIDTH(SB_MSG_WIDTH), .LANE_STATUS_WIDTH(LANE_STATUS_WIDTH)) linkspeed_tx_u0 (
    .i_clk(i_clk),
    .i_rst_n(i_rst_n),
    .i_enable(i_enable),

    .i_valid_rx(o_valid_rx),
    .i_msg_rx(o_sb_msg_rx),

    .i_sb_msg(i_sb_msg),
    .i_sb_valid(i_sb_valid),
    .i_falling_edge_busy(i_falling_edge_busy),

    .i_point_test_ack(i_point_test_ack),
    .i_lane_results(i_lane_results),

    .i_tx_first_8_lanes_ok_mbinit(i_tx_first_8_lanes_ok_mbinit),
    .i_tx_second_8_lanes_ok_mbinit(i_tx_second_8_lanes_ok_mbinit),

    .i_phy_in_retrain_flag(i_phy_in_retrain_flag),
    .i_runtime_config_change(i_runtime_config_change),

    .o_point_test_en(o_point_test_en_tx),

    .o_sb_msg(o_sb_msg_tx),
    .o_valid_tx(o_valid_tx), 
    .o_timeout_disable(o_timeout_disable), 
    .o_linkspeed_lanes_status(o_linkspeed_lanes_status),   
    .o_ack(o_ack_tx),
    .o_clear_phy_in_retrain_flag(o_clear_phy_in_retrain_flag),
    .o_exit_phyretrain(o_exit_phyretrain),
    .o_exit_repair(o_exit_repair),
    .o_exit_speedidle(o_exit_speedidle),
    .o_tx_first_8_lanes_ok_linkspeed(o_tx_first_8_lanes_ok_linkspeed),
    .o_tx_second_8_lanes_ok_linkspeed(o_tx_second_8_lanes_ok_linkspeed)
 );

 mbtrain_linkspeed_rx #(.SB_MSG_WIDTH(SB_MSG_WIDTH)) linkspeed_rx_u0 (
    .i_clk(i_clk),
    .i_rst_n(i_rst_n),
    .i_enable(i_enable),

    .i_sb_msg(i_sb_msg),
    .i_sb_valid(i_sb_valid),
    .i_falling_edge_busy(i_falling_edge_busy),

    .i_msg_tx(o_sb_msg_tx),
    .i_valid_tx(o_valid_tx), 

    .i_point_test_ack(i_point_test_ack),

    .o_point_test_en(o_point_test_en_rx),
    .o_sb_msg(o_sb_msg_rx),
    .o_valid_rx(o_valid_rx),
    .o_ack(o_ack_rx)
 );

 endmodule