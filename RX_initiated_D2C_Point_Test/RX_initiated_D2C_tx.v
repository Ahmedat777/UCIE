module RX_initiated_D2C_tx (
  
   /* ================= Inputs ================= */
  input  wire                 i_clk,
  input  wire                 i_en_d2c_rx,
  input  wire                 i_rst_n,
  input  wire                 i_rx_valid,
  input  wire                 i_busy_negedge_detected,
  input  wire                 i_datavref_or_valvref,
  input  wire                 i_pattern_finished,
  input  wire                 i_message_valid,
  input		wire	[3:0]			        i_decoded_sb_msg, 
  
  /* ================= Outputs ================= */
  output reg		[3:0]			        o_encoded_sb_msg, 
  output reg                  o_sb_data_pattern, 
  output reg                  o_sb_burst_count,
  output reg                  o_sb_comparison_mode,
  output reg  [1:0]           o_clock_phase,
  output reg                  o_data_valid,
  output	reg									         o_ack_rx_tx,
  output reg 								         o_msg_valid_tx,
  output reg                  o_val_pattern_en,
  output reg  [1:0]           o_generator_mode_cw
  );
  
  // FSM states
  localparam IDLE                = 3'd0;
  localparam WAIT_START_REQ      = 3'd1;
  localparam SETUP_AND_START_REQ = 3'd2;
  localparam LFSR_CLEAR_REQ      = 3'd3;
  localparam PATTERN_TRANSMIT    = 3'd4;
  localparam COUNT_DONE_REQ      = 3'd5;
  localparam SEND_END_REQ        = 3'd6;
  localparam TEST_DONE           = 3'd7;
  
  reg [2:0] current_state, next_state;
  
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
        
        if(i_en_d2c_rx && i_decoded_sb_msg != 4'b0001)begin
          next_state = SETUP_AND_START_REQ;
        end 
        else if(i_en_d2c_rx && i_decoded_sb_msg == 4'b0001 && i_message_valid) begin
          next_state = WAIT_START_REQ;
        end 
        else begin
          next_state = IDLE;
        end
      end
        
      
      WAIT_START_REQ: begin
        if(!i_en_d2c_rx)begin
          next_state = IDLE;
        end else begin 
          if(i_busy_negedge_detected && i_rx_valid)
             next_state = SETUP_AND_START_REQ;
        end
      end
      
      SETUP_AND_START_REQ: begin
        if(!i_en_d2c_rx)begin
          next_state = IDLE;
        end else begin 
          if(i_message_valid && i_decoded_sb_msg == 4'b0010)
            next_state = LFSR_CLEAR_REQ;
        end
      end
      
      LFSR_CLEAR_REQ: begin
        if(!i_en_d2c_rx)begin
          next_state = IDLE;
        end else begin 
          if(i_message_valid && i_decoded_sb_msg == 4'b0100)
            next_state = PATTERN_TRANSMIT;
        end
      end
      
      
      PATTERN_TRANSMIT: begin
        if(!i_en_d2c_rx)begin
          next_state = IDLE;
        end else begin 
          if(i_pattern_finished)
            next_state = COUNT_DONE_REQ;
        end
      end
      
      COUNT_DONE_REQ: begin
        if(!i_en_d2c_rx)begin
          next_state = IDLE;
        end else begin 
          if(i_message_valid && i_decoded_sb_msg == 4'b0110)
            next_state = SEND_END_REQ;
        end
      end
      
      SEND_END_REQ: begin
        if(!i_en_d2c_rx)begin
          next_state = IDLE;
        end else begin 
          if(i_message_valid && i_decoded_sb_msg == 4'b1000)
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
    o_encoded_sb_msg     <= 4'b0000;
    o_sb_data_pattern    <= 1'b0;
    o_sb_burst_count     <= 1'b0;
    o_sb_comparison_mode <= 1'b0;
    o_clock_phase        <= 2'b00;
    o_data_valid         <= 1'b0;
    o_ack_rx_tx          <= 1'b0;
    o_msg_valid_tx       <= 1'b0;
    o_val_pattern_en     <= 1'b0;
    o_generator_mode_cw  <= 2'b00;
  end
  else begin
    /* defaults */
    o_data_valid   <= o_data_valid;
    //o_ack_rx_tx    <= 1'b0;
    o_msg_valid_tx <= 1'b0;

    case (current_state)

    /* ================= IDLE ================= */
    IDLE: begin
      o_generator_mode_cw <= 2'b00;
      o_val_pattern_en    <= 1'b0;
      o_ack_rx_tx         <= 1'b0;

      if (next_state == SETUP_AND_START_REQ) begin
        o_encoded_sb_msg     <= 4'b0001; // START_RX_D2C_PT_REQ
        o_sb_data_pattern    <= 1'b0;
        o_sb_comparison_mode <= 1'b0;
        o_clock_phase        <= 2'b00;
        o_sb_burst_count     <= (i_datavref_or_valvref) ? 1'b0 : 1'b1;
        o_msg_valid_tx       <= 1'b1;
        o_data_valid         <= 1'b1;   
      end
    end

    /* ================= WAIT_START_REQ ================= */
    WAIT_START_REQ: begin
      if (next_state == SETUP_AND_START_REQ) begin
        o_encoded_sb_msg     <= 4'b0001; // START_RX_D2C_PT_REQ
        o_sb_data_pattern    <= 1'b0;
        o_sb_comparison_mode <= 1'b0;
        o_clock_phase        <= 2'b00;
        o_sb_burst_count     <= (i_datavref_or_valvref) ? 1'b0 : 1'b1;
        o_msg_valid_tx       <= 1'b1;
        o_data_valid         <= 1'b1;
      end
    end

    /* ================= SETUP_AND_START_REQ ================= */
    SETUP_AND_START_REQ: begin
      if (next_state == LFSR_CLEAR_REQ) begin
        o_encoded_sb_msg    <= 4'b0011; // LFSR_CLR_ERROR_REQ
        o_generator_mode_cw <= 2'b01;   // CLEAR_LFSR
        o_msg_valid_tx      <= 1'b1;
      end
    end

    /* ================= LFSR_CLEAR_REQ ================= */
    LFSR_CLEAR_REQ: begin
      if (next_state == PATTERN_TRANSMIT) begin
        if (i_datavref_or_valvref == 1'b0)
          o_generator_mode_cw <= 2'b10; // LFSR
        else
          o_val_pattern_en <= 1'b1;
      end
    end

    /* ================= PATTERN_TRANSMIT ================= */
    PATTERN_TRANSMIT: begin
      if (next_state == COUNT_DONE_REQ) begin
        o_encoded_sb_msg    <= 4'b0101; // COUNT_DONE_REQ
        o_generator_mode_cw <= 2'b00;
        o_val_pattern_en    <= 1'b0;
        o_msg_valid_tx      <= 1'b1;
      end
    end

    /* ================= COUNT_DONE_REQ ================= */
    COUNT_DONE_REQ: begin
      if (next_state == SEND_END_REQ) begin
        o_encoded_sb_msg <= 4'b0111; // END_RX_D2C_PT_REQ
        o_msg_valid_tx   <= 1'b1;
      end
    end

    /* ================= SEND_END_REQ ================= */
    SEND_END_REQ: begin
      if (next_state == TEST_DONE) begin
        o_ack_rx_tx   <= 1'b1; // done pulse
        o_data_valid  <= 1'b0;
      end
    end

    /* ================= TEST_DONE ================= */
    TEST_DONE: begin
      // hold state, wait for i_en_d2c_rx to deassert
    end

    default: begin
      //o_data_valid   <= o_data_valid;
    end
    endcase

    /* ================= valid drop ================= */
    if (i_busy_negedge_detected && !i_rx_valid)
      o_msg_valid_tx <= 1'b0;

  end
end

endmodule
      
          
        
          
