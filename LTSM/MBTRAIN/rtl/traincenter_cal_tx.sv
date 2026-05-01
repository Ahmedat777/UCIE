module traincenter_cal_tx  # ( parameter SB_MSG_WIDTH = 4)

  (
   //inputs
	input        i_clk, 
	input        i_rst_n, 
	input        i_enable,
      //sideband signals
	input  [SB_MSG_WIDTH-1:0]             i_decoded_sideband_msg,
	input                                 i_sideband_valid,
  input                                 i_falling_edge_busy,
      //point test block acknowledgement
	input 	     i_test_ack,
		
   //outputs
      //sideband signals
	output reg  [SB_MSG_WIDTH-1:0]            o_sideband_msg,
	output reg                                o_valid_tx,
      //point test and eye width sweep block signals
	output reg       o_point_test_en,
	output reg       o_eye_width_sweep_en,
      //main controller signal
	output reg       o_test_ack
  );
///////////////////// local Parameters ////////////////////////

//states (Grey encoding)
typedef enum logic [2:0] {
  IDLE           = 3'b000,
  SEND_START_REQ = 3'b001,
  POINT_TEST     = 3'b011,
  SEND_END_REQ   = 3'b111,
  TEST_END       = 3'b110
} tx_state;

tx_state current_state_tx, next_state_tx;

//i_decoded_sideband_msg
localparam  START_RESPOND  = 4'b0010,
            END_RESPOND    = 4'b0100;

//o_sideband_msg
localparam  START_REQUEST  = 4'b0001,
            END_REQUEST    = 4'b0011;

/////////////////// current state logic /////////////////

always @(posedge i_clk or negedge i_rst_n) begin
  
  if (!i_rst_n)
   current_state_tx <= IDLE;
  else
   current_state_tx <= next_state_tx;

end

//////////////////// next state logic ////////////////////

always @(*) begin

  case (current_state_tx) 
     
    IDLE : next_state_tx = (i_enable)? SEND_START_REQ : IDLE;

    SEND_START_REQ : begin
        if(!i_enable)
            next_state_tx = IDLE;
        else if(i_decoded_sideband_msg == START_RESPOND && i_sideband_valid) //startt respond
            next_state_tx = POINT_TEST;
        else
            next_state_tx = SEND_START_REQ;
    end  

    POINT_TEST : begin
        if(!i_enable)
            next_state_tx = IDLE;
        else if(i_test_ack)
            next_state_tx = SEND_END_REQ;
        else
            next_state_tx = POINT_TEST;
    end  

    SEND_END_REQ : begin
        if(!i_enable)
            next_state_tx = IDLE;
        else if(i_decoded_sideband_msg == END_RESPOND  && i_sideband_valid) //endd respond
            next_state_tx = TEST_END;
        else
            next_state_tx = SEND_END_REQ;
    end  

    TEST_END : next_state_tx = (!i_enable)? IDLE : TEST_END;

    default :  next_state_tx = IDLE ;
 
  endcase 

end

//////////////////////// output logic //////////////////////////////

always @(posedge i_clk or negedge i_rst_n) begin
     o_sideband_msg       <= 4'b0;
     o_point_test_en      <= 1'b0;
     o_eye_width_sweep_en <= 1'b0;
     o_test_ack           <= 1'b0;

  if (!i_rst_n) begin
     o_sideband_msg       <= 4'b0;
     o_point_test_en      <= 1'b0;
     o_eye_width_sweep_en <= 1'b0;
     o_test_ack           <= 1'b0;
  end
  else begin
  case (next_state_tx)

    IDLE : begin
    end
     
    SEND_START_REQ : begin
        o_sideband_msg       <= START_REQUEST; // startt request
    end 

    POINT_TEST : begin
        o_point_test_en      <= 1'b1;
        o_eye_width_sweep_en <= 1'b1;
    end

    SEND_END_REQ :  begin
        o_sideband_msg       <= END_REQUEST; // endd request
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
      o_valid_tx <= 1'b0;
    else if((current_state_tx != next_state_tx) && (next_state_tx == SEND_START_REQ || next_state_tx == SEND_END_REQ ))
      o_valid_tx <= 1'b1;
    else if(i_falling_edge_busy)
      o_valid_tx <= 1'b0;
end
 
endmodule

