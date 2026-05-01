module vref_cal_rx  # ( parameter SB_MSG_WIDTH = 4) 
(
 //inputs 
	 input        i_clk,    
	 input        i_rst_n,  
	 input        i_enable,
     //sideband signals 
     input [SB_MSG_WIDTH-1:0]              i_decoded_sideband_msg,
	 input 	                               i_sideband_valid,
     input                                 i_falling_edge_busy,
     //point test block acknowledgement
     input 	        i_test_ack, 
 
 //output 
	 //sideband signals
	 output reg [SB_MSG_WIDTH-1:0]              o_sideband_msg,
	 output reg                                 o_valid_rx,
	 ////point test and eye width sweep block signals 
	 output reg       o_point_test_en,
     output reg       o_eye_width_sweep_en,
	 //main controller signal 
	 output reg       o_test_ack
);

///////////////////////// local Parameters ////////////////////////

//states (Grey encoding)
typedef enum logic [2:0] {
  IDLE               = 3'b000,
  WAIT_START_REQ     = 3'b001,
  SEND_START_RESPOND = 3'b011,
  POINT_TEST         = 3'b111,
  WAIT_END_REQ       = 3'b110,
  SEND_END_RESPOND   = 3'b100,
  TEST_END           = 3'b101
} rx_state;

rx_state current_state_rx, next_state_rx;

//i_decoded_sideband_msg
localparam  START_RESPOND  = 4'b0010,
            END_RESPOND    = 4'b0100;

//o_sideband_msg
localparam  START_REQUEST  = 4'b0001,
            END_REQUEST    = 4'b0011;

/////////////////// current state logic /////////////////

always @(posedge i_clk or negedge i_rst_n) begin
  
  if (!i_rst_n)
   current_state_rx <= IDLE;
  else
   current_state_rx <= next_state_rx;

end

//////////////////// next state logic ////////////////////

always @(*) begin

  case (current_state_rx) 
     
    IDLE : next_state_rx = (i_enable)? WAIT_START_REQ  : IDLE;

    WAIT_START_REQ : begin
        if(!i_enable)
            next_state_rx = IDLE;
        else if(i_decoded_sideband_msg == START_REQUEST && i_sideband_valid) //startt request
            next_state_rx = SEND_START_RESPOND;
        else
            next_state_rx = WAIT_START_REQ;
    end  
    
    SEND_START_RESPOND : begin
        if(!i_enable)
            next_state_rx = IDLE;
        else if(i_falling_edge_busy)
            next_state_rx = POINT_TEST;
        else
            next_state_rx = SEND_START_RESPOND;
    end


    POINT_TEST : begin
        if(!i_enable)
            next_state_rx = IDLE;
        else if(i_test_ack)
            next_state_rx = WAIT_END_REQ;
        else
            next_state_rx = POINT_TEST;
    end  

    WAIT_END_REQ : begin
        if(!i_enable)
            next_state_rx = IDLE;
        else if(i_decoded_sideband_msg == END_REQUEST && i_sideband_valid) //endd request
            next_state_rx = SEND_END_RESPOND;
        else
            next_state_rx = WAIT_END_REQ;
    end 

    SEND_END_RESPOND : begin
        if(!i_enable)
            next_state_rx = IDLE;
        else if(i_falling_edge_busy)
            next_state_rx = TEST_END;
        else
            next_state_rx = SEND_END_RESPOND;
    end 

    TEST_END : next_state_rx = (!i_enable)? IDLE : TEST_END;

    default :  next_state_rx = IDLE ;
 
  endcase 

end

//////////////////////// output logic //////////////////////////////


always @(posedge i_clk or negedge i_rst_n) begin

  if (!i_rst_n) begin
     o_sideband_msg       <= 4'b0;
     o_point_test_en      <= 1'b0;
     o_eye_width_sweep_en <= 1'b0;
     o_test_ack           <= 1'b0;
  end
  else begin           
     o_sideband_msg       <= 4'b0;
     o_point_test_en      <= 1'b0;
     o_eye_width_sweep_en <= 1'b0;
     o_test_ack           <= 1'b0;
   case (next_state_rx) 
    
    IDLE : begin
    end

    WAIT_START_REQ : begin
    end

    SEND_START_RESPOND : begin
        o_sideband_msg <= START_RESPOND; // startt respond 
    end  

    POINT_TEST : begin
        o_point_test_en      <= 1'b1;
        o_eye_width_sweep_en <= 1'b1;
    end  

    WAIT_END_REQ : begin
    end

    SEND_END_RESPOND : begin
        o_sideband_msg <= END_RESPOND; // endd respond
    end  
    
    TEST_END : o_test_ack    <= 1'b1;
    
    default : begin
        o_sideband_msg       <= 4'b0;
        o_point_test_en      <= 1'b0;
        o_eye_width_sweep_en <= 1'b0;
        o_test_ack           <= 1'b0;
    end

   endcase 
  end
end

//////////////////////////// Valid output ///////////////////////////////
always @(posedge i_clk or negedge i_rst_n) begin
    if(!i_rst_n)
      o_valid_rx <= 1'b0;
    else if((current_state_rx != next_state_rx) && (next_state_rx == SEND_START_RESPOND || next_state_rx == SEND_END_RESPOND ))
      o_valid_rx <= 1'b1;
    else if(i_falling_edge_busy)
      o_valid_rx<= 1'b0;
end

endmodule