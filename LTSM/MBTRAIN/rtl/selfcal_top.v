module selfcal_top (
////////////////////////////**main inputs**//////////////////////////////////
		input clk,    
		input rst_n,  
		input i_en, 
////////////////////////////**sideband inputs**//////////////////////////////
		input [3:0]  i_decoded_sideband_message ,
		input 		 i_sideband_valid,
		input 		 i_falling_edge_busy,
	//outputs 
////////////////////////////**sideband outputs**////////////////////////////
			output  [3:0] o_sideband_message ,
			output  o_valid,
			output  o_test_ack

);
/*------------------------------------------------------------------------------
/////////////////////////////**tx signals ////////////////////////////////    
------------------------------------------------------------------------------*/
		wire [3:0] o_sideband_message_tx;
		wire       o_valid_tx;
		wire       o_test_ack_tx;
/*------------------------------------------------------------------------------
/////////////////////////////**rx signals////////////////////////////////    
------------------------------------------------------------------------------*/
		wire [3:0] o_sideband_message_rx;
		wire       o_valid_rx;
		wire       o_test_ack_rx;
/*------------------------------------------------------------------------------
/////////////////////////////**assign statements////////////////////////////////    
------------------------------------------------------------------------------*/		
assign o_test_ack=o_test_ack_rx && o_test_ack_tx;
assign o_valid=o_valid_rx || o_valid_tx;
/*------------------------------------------------------------------------------
/////////////////////////////**tx instantiations ////////////////////////////////    
------------------------------------------------------------------------------*/
selfcal_tx_tx selfcal_tx_inst( 
		 .clk(clk),    
		 .rst_n(rst_n),  
		 .i_en(i_en),
		.i_decoded_sideband_message(i_decoded_sideband_message) ,
		.i_busy_negedge_detected(i_falling_edge_busy),.i_valid_rx(o_valid_rx),
		.i_sideband_valid          (i_sideband_valid), 
		.o_sideband_message(o_sideband_message_tx),
		.o_valid_tx(o_valid_tx),
		.o_test_ack(o_test_ack_tx)
);
/*------------------------------------------------------------------------------
/////////////////////////////**rx instantiations////////////////////////////////    
------------------------------------------------------------------------------*/
selfcal_tx_rx  selfcal_rx_inst(
    .clk(clk),    
    .rst_n(rst_n),  
    .i_en(i_en),
    .i_decoded_sideband_message(i_decoded_sideband_message),
    .i_busy_negedge_detected(i_falling_edge_busy),
    .i_valid_tx(o_valid_tx),
    .i_sideband_valid(i_sideband_valid), 
    .o_sideband_message(o_sideband_message_rx),
    .o_valid_rx(o_valid_rx),
    .o_test_ack(o_test_ack_rx)
);
/*------------------------------------------------------------------------------
/////////////////////////////**handling massages////////////////////////////////    
------------------------------------------------------------------------------*/
assign o_sideband_message = (o_valid_tx) ? o_sideband_message_tx : 
                            (o_valid_rx) ? o_sideband_message_rx : 4'b0000;
endmodule 