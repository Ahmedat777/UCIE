module RX_initiated_D2C_rx (
  
  /* ================= Inputs ================= */
  input  wire                 i_clk,
  input  wire                 i_en_d2c_rx,
  input  wire                 i_rst_n,
  input  wire                 i_tx_valid,
  input  wire                 i_busy_negedge_detected,
  input  wire                 i_SB_Busy,
  input  wire                 i_datavref_or_valvref,
  input  wire                 i_message_valid,
  input		wire	[3:0]			        i_decoded_sb_msg, 
  
  /* ================= Outputs ================= */
  output reg		[3:0]			        o_encoded_sb_msg, 
  output	reg									         o_ack_rx_rx,
  output reg 								         o_msg_valid_rx,
  output reg                  o_comparison_valid_en,
  output reg  [1:0]           o_comparator_mode_cw
  );
  
  reg [3:0] current_state, next_state;
  reg       o_msg_valid_rx_reg;
  wire      valid_negedge_detected;
  reg       pending_valid;
  //reg       save_msg_valid;
  
  
// FSM states
  localparam IDLE                = 4'd0;
  localparam WAIT_START_REQ      = 4'd1;
  localparam START_RESP          = 4'd2;
  localparam WAIT_LFSR_CLEAR_REQ = 4'd3;
  localparam LFSR_CLEAR_RESP     = 4'd4;
  localparam WAIT_COUNT_DONE_REQ = 4'd5;
  localparam COUNT_DONE_RSP      = 4'd6;
  localparam WAIT_SEND_END_REQ   = 4'd7;
  localparam SEND_END_RSP        = 4'd8;
  localparam TEST_DONE           = 4'd9;
  
  
  
  wire valid_cond = (next_state == START_RESP && !i_SB_Busy) || (next_state == LFSR_CLEAR_RESP && !i_SB_Busy) || (next_state == COUNT_DONE_RSP && !i_SB_Busy) || (next_state == SEND_END_RSP && !i_SB_Busy);
  
  always @(posedge i_clk or negedge i_rst_n) begin
  if (!i_rst_n) begin
    pending_valid <= 1'b0;
  end else begin
    if ( (next_state == START_RESP && i_tx_valid) || (next_state == LFSR_CLEAR_RESP && i_tx_valid) || (next_state == COUNT_DONE_RSP && i_tx_valid) || (next_state == SEND_END_RSP && i_tx_valid) ) begin
      pending_valid <= 1'b1;
    end
    else if (o_msg_valid_rx) begin
      pending_valid <= 1'b0;
    end
  end
  end 
  
  
  always @(posedge i_clk or negedge i_rst_n) begin
  if (!i_rst_n) begin
    o_msg_valid_rx <= 1'b0;
    //save_msg_valid <= 1'b0;
  end else begin
    //save_msg_valid <= o_msg_valid_tx;

    // SB consumed the message
    if (i_busy_negedge_detected) begin
      o_msg_valid_rx <= 1'b0;
    end
    // normal raise OR delayed raise after TX busy
    else if (valid_cond || (pending_valid && !i_tx_valid)) begin
      o_msg_valid_rx <= 1'b1;
    end
  end
end

  
  assign valid_negedge_detected = ~o_msg_valid_rx && (o_msg_valid_rx_reg != o_msg_valid_rx);
    
    always@(posedge i_clk or negedge i_rst_n) begin
        if (!i_rst_n) 
          o_msg_valid_rx_reg <= 1'b0;
        else
          o_msg_valid_rx_reg <= o_msg_valid_rx;
    end
  
  // FSM state register
  always @(posedge i_clk or negedge i_rst_n) begin
    if(!i_rst_n)
      current_state <= IDLE;
    else
      current_state <= next_state;
  end

/* ================= Next State Logic ================= */
always @(*) begin
    next_state = current_state;
    case(current_state)
      IDLE: begin
        if(i_en_d2c_rx) begin
          next_state = WAIT_START_REQ;
        end
        else begin
          next_state = IDLE;
        end
      end
      
      WAIT_START_REQ: begin
        if(!i_en_d2c_rx) begin
          next_state = IDLE;
        end
        else if(i_decoded_sb_msg == 4'b0001 && i_message_valid) begin
          next_state = START_RESP;
        end
      end
      
      START_RESP: begin
        if(!i_en_d2c_rx) begin
          next_state = IDLE;
        end
        //else if(i_en_d2c_rx && i_busy_negedge_detected) begin
        else if(valid_negedge_detected) begin
          next_state = WAIT_LFSR_CLEAR_REQ;
        end
      end
      
      WAIT_LFSR_CLEAR_REQ: begin
        if(!i_en_d2c_rx) begin
          next_state = IDLE;
        end
        else if(i_decoded_sb_msg == 4'b0011 && i_message_valid) begin
          next_state = LFSR_CLEAR_RESP;
        end
      end
      
      LFSR_CLEAR_RESP: begin
        if(!i_en_d2c_rx) begin
          next_state = IDLE;
        end
        //else if(i_en_d2c_rx && i_busy_negedge_detected) begin
        else if(valid_negedge_detected) begin
          next_state = WAIT_COUNT_DONE_REQ;
        end
      end
      
      WAIT_COUNT_DONE_REQ: begin
        if(!i_en_d2c_rx) begin
          next_state = IDLE;
        end
        else if(i_decoded_sb_msg == 4'b0101 && i_message_valid) begin
          next_state = COUNT_DONE_RSP;
        end
      end
      
      COUNT_DONE_RSP: begin
        if(!i_en_d2c_rx) begin
          next_state = IDLE;
        end
        //else if(i_en_d2c_rx && i_busy_negedge_detected) begin
        else if(valid_negedge_detected) begin
          next_state = WAIT_SEND_END_REQ;
        end
      end
      
      WAIT_SEND_END_REQ: begin
        if(!i_en_d2c_rx) begin
          next_state = IDLE;
        end
        else if(i_decoded_sb_msg == 4'b0111 && i_message_valid) begin
          next_state = SEND_END_RSP;
        end
      end
      
      SEND_END_RSP: begin
        if(!i_en_d2c_rx) begin
          next_state = IDLE;
        end
        //else if(i_en_d2c_rx && i_busy_negedge_detected) begin
        else if(valid_negedge_detected) begin
          next_state = TEST_DONE;
        end
      end
      
      TEST_DONE: begin
        if(!i_en_d2c_rx) 
         next_state = IDLE;
      end
      
      default: next_state = IDLE;
    endcase
  end
          
  
/* ================= Output Logic ================= */      
always @(posedge i_clk or negedge i_rst_n) begin
  if(!i_rst_n) begin
    o_encoded_sb_msg      <= 4'b0000;
    o_ack_rx_rx           <= 1'b0;
    o_comparison_valid_en <= 1'b0;
    o_comparator_mode_cw  <= 2'b00;
  end
  else begin
    case (current_state)

    /* ================= IDLE ================= */
    IDLE: begin
      o_encoded_sb_msg      <= 4'b0000;
      o_ack_rx_rx           <= 1'b0;
      o_comparison_valid_en <= 1'b0;
      o_comparator_mode_cw  <= 2'b00;
    end
    
    WAIT_START_REQ: begin
      if(next_state == START_RESP) begin
        o_encoded_sb_msg      <= 4'b0010;
      end
    end
    
    WAIT_LFSR_CLEAR_REQ: begin
      if(next_state == LFSR_CLEAR_RESP) begin
        o_encoded_sb_msg      <= 4'b0100;
        if(i_datavref_or_valvref) begin
          o_comparison_valid_en <= 1'b1;
        end else begin
          o_comparator_mode_cw  <= 2'b01; //clear LFSR
        end
      end
    end
    
    LFSR_CLEAR_RESP: begin
      if(next_state == WAIT_COUNT_DONE_REQ && i_datavref_or_valvref) begin
        o_comparator_mode_cw  <= 2'b10;
      end
    end
    
    WAIT_COUNT_DONE_REQ: begin
      if(next_state == COUNT_DONE_RSP) begin
        o_encoded_sb_msg      <= 4'b0110;
        o_comparison_valid_en <= 1'b0;
        o_comparator_mode_cw  <= 2'b00;
      end
    end
    
    WAIT_SEND_END_REQ: begin
      if(next_state == SEND_END_RSP) begin
        o_encoded_sb_msg      <= 4'b1000;
      end
    end
    
    SEND_END_RSP: begin
      if(next_state == TEST_DONE) begin
        o_ack_rx_rx         <= 1'b1;
      end
    end
    
    TEST_DONE: begin
      //o_ack_rx_rx         <= 1'b1;
    end 
  endcase
  
 end
end
endmodule




