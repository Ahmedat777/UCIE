module repair_wrapper (
////////////////////////////**main inputs**//////////////////////////////////
			input clk,    
			input i_en, 
			input rst_n, 
////////////////////////////**sideband inputs**////////////////////////////// 
		    input [3:0] i_sideband_message,
		    input i_busy,
			input i_falling_edge_busy,
			input [2:0]i_sideband_data_lanes_encoding,
			input i_sideband_valid,
////////////////////////////**communicating with partner**////////////////////////////// 			
			input i_first_8_lanes_are_functional , i_second_8_lanes_are_functional,
	//outputs 
////////////////////////////**communicating with sideband**//////////////////////////////	
			output   o_valid,
			output [2:0] o_sideband_data_lanes_encoding ,
			output [3:0] o_sideband_message,
////////////////////////////**communicating with mbtrain**//////////////////////////////			
			output o_remote_partner_first_8_lanes_result , o_remote_partner_second_8_lanes_result,
////////////////////////////**ack_controler**//////////////////////////////
			output o_test_ack 

);

/*------------------------------------------------------------------------------
/////////////////////////////**tx signals ////////////////////////////////    
------------------------------------------------------------------------------*/ 
	    wire  [3:0] o_sideband_message_tx;
	    wire  o_valid_tx ;
	    wire o_test_ack_tx;
/*------------------------------------------------------------------------------
/////////////////////////////**rx signals////////////////////////////////    
------------------------------------------------------------------------------*/ 
	    	wire [3:0] o_sideband_message_rx;
	    	wire o_valid_rx;
	    	wire o_test_ack_rx;
/*------------------------------------------------------------------------------
/////////////////////////////**assign statements////////////////////////////////    
------------------------------------------------------------------------------*/
	assign o_valid= o_valid_tx || o_valid_rx;
	assign o_test_ack=o_test_ack_rx && o_test_ack_tx;
/*------------------------------------------------------------------------------
/////////////////////////////**tx instantiations ////////////////////////////////    
------------------------------------------------------------------------------*/
repair_tx repair_tx_instance(  
			.clk(clk),
			.rst_n(rst_n),
			.i_en(i_en), 
			.i_sideband_message(i_sideband_message),
			.i_busy_negedge_detected(i_falling_edge_busy),
			.i_sideband_valid(i_sideband_valid),
		    .i_first_8_lanes_are_functional(i_first_8_lanes_are_functional) ,
		    .i_second_8_lanes_are_functional(i_second_8_lanes_are_functional),
		    .i_valid_rx(o_valid_rx),
		    .o_sideband_message(o_sideband_message_tx),
		    .o_valid_tx(o_valid_tx),
		   .o_sideband_data_lanes_encoding(o_sideband_data_lanes_encoding),
		    .o_test_ack(o_test_ack_tx)
);
/*------------------------------------------------------------------------------
/////////////////////////////**rx instantiations////////////////////////////////    
------------------------------------------------------------------------------*/
repair_rx repair_rx_instance(
			.clk(clk),
			.rst_n(rst_n),
			.i_en(i_en),
		    .i_sideband_message(i_sideband_message),
		    .i_sideband_data_lanes_encoding(i_sideband_data_lanes_encoding),
		    .i_busy_negedge_detected(i_falling_edge_busy),
		    .i_valid_tx(o_valid_tx),
		     .o_sideband_message(o_sideband_message_rx),
		     .o_valid_rx(o_valid_rx),
		    .o_test_ack(o_test_ack_rx),
		   .o_remote_partner_first_8_lanes_result(o_remote_partner_first_8_lanes_result) ,
		    .o_remote_partner_second_8_lanes_result(o_remote_partner_second_8_lanes_result) 
);
/*------------------------------------------------------------------------------
/////////////////////////////**handling massages////////////////////////////////    
------------------------------------------------------------------------------*/
assign o_sideband_message = (o_valid_tx) ? o_sideband_message_tx : 
                            (o_valid_rx) ? o_sideband_message_rx : 4'b0000;
                                                        

endmodule 