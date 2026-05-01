module Vref_cal_wrapper # ( parameter SB_MSG_WIDTH = 4) 
(

 //inputs
	input        i_clk,    
	input        i_rst_n,  
	input        i_enable, 
      //sideband signals
	input [SB_MSG_WIDTH-1:0]  i_decoded_sideband_msg ,
	input                     i_sideband_valid,
  input                     i_falling_edge_busy,
      //point test block acknowledgement
	input 	     i_test_ack, 
		
   //outputs
      //sideband signals
	output reg [SB_MSG_WIDTH-1:0] o_sideband_msg,
	output                        o_valid,
      //point test and eye width sweep block signals 
	output        o_point_test_en,
	output        o_eye_width_sweep_en,
      //main controller signal
	output        o_test_ack
                                                                                                                                                                                                                                                                                                                      
);

//assign  o_data_or_valid_test = i_data_or_valid_test;  let it out from the controller (the generator of the signal) directly to the point test blocks.

///////////////////////// TX signals //////////////////////////////

wire                       o_valid_tx;
wire  [SB_MSG_WIDTH-1:0]   o_sideband_msg_tx;
wire                       o_point_test_en_tx;
wire                       o_eye_width_sweep_en_tx;
wire                       o_test_ack_tx;

///////////////////////// RX signals //////////////////////////////

wire                       o_valid_rx;
wire  [SB_MSG_WIDTH-1:0]   o_sideband_msg_rx;
wire                       o_point_test_en_rx;
wire                       o_eye_width_sweep_en_rx;
wire                       o_test_ack_rx;

//////////////////// Combining TX and RX outputs ////////////////////

assign o_valid = o_valid_tx || o_valid_rx;
assign o_point_test_en = o_point_test_en_tx && o_point_test_en_rx;
assign o_eye_width_sweep_en = o_eye_width_sweep_en_tx && o_eye_width_sweep_en_rx;
assign o_test_ack = o_test_ack_tx && o_test_ack_rx;


// sideband message
always @(*) begin
   case({o_valid_tx,o_valid_rx})
     2'b00 : o_sideband_msg = 'b0;
     2'b10 : o_sideband_msg = o_sideband_msg_tx;
     2'b01 : o_sideband_msg = o_sideband_msg_rx;
     2'b11 : o_sideband_msg = o_sideband_msg_rx;
   endcase
end

///////////////////////// TX instantiation ////////////////////////

vref_cal_tx #(.SB_MSG_WIDTH(SB_MSG_WIDTH)) vref_cal_tx_u0 (
  //inputs
   .i_clk(i_clk),
   .i_rst_n(i_rst_n),
   .i_enable(i_enable),
     //sideband signals
   .i_decoded_sideband_msg(i_decoded_sideband_msg),
   .i_sideband_valid(i_sideband_valid),
   .i_falling_edge_busy(i_falling_edge_busy),
     //point test block acknowledgement
   .i_test_ack(i_test_ack),
  //outputs
     //sideband signals
   .o_sideband_msg(o_sideband_msg_tx),
   .o_valid_tx(o_valid_tx),
     //point test and eye width sweep block signals
   .o_point_test_en(o_point_test_en_tx),
   .o_eye_width_sweep_en(o_eye_width_sweep_en_tx),
     //main controller signal
   .o_test_ack(o_test_ack_tx)
);

///////////////////////// RX instantiation ////////////////////////

vref_cal_rx #(.SB_MSG_WIDTH(SB_MSG_WIDTH)) vref_cal_rx_u0 (
  //inputs
   .i_clk(i_clk),
   .i_rst_n(i_rst_n),
   .i_enable(i_enable),
     //sideband signals
   .i_decoded_sideband_msg(i_decoded_sideband_msg),
   .i_sideband_valid(i_sideband_valid),
   .i_falling_edge_busy(i_falling_edge_busy),
     //point test block acknowledgement
   .i_test_ack(i_test_ack),
  //outputs
     //sideband signals
   .o_sideband_msg(o_sideband_msg_rx),
   .o_valid_rx(o_valid_rx),
     //point test and eye width sweep block signals
   .o_point_test_en(o_point_test_en_rx),
   .o_eye_width_sweep_en(o_eye_width_sweep_en_rx),
     //main controller signal
   .o_test_ack(o_test_ack_rx)
);

endmodule