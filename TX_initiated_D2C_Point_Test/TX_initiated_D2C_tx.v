module TX_initiated_D2C_tx #(
  parameter NUM_LANE = 16
)(
  input  wire                 i_clk,
  input  wire                 i_en_d2c_tx,
  input  wire                 i_rst_n,
  input  wire                 i_mainband_or_valtrain_test, //0 mainband, 1 validation pattern
  input  wire                 i_lfsr_or_perlane,           //0 LFSR, 1 per-lane
  input  wire                 i_pattern_finished,
  input  wire [3:0]           i_sideband_message,
  input  wire                 i_sideband_message_valid,
  input  wire                 i_busy_negedge_detected,
  input  wire                 i_valid_rx,

  output reg  [3:0]           o_sideband_message,
  output reg                  o_message_valid,
  output reg  [NUM_LANE-1:0]  o_sideband_data,
  output reg                  o_data_valid,
  output reg                  o_val_pattern_en,
  output reg  [1:0]           o_generator_mode_cw,
  output reg                  o_test_ack_tx
);

  // FSM states
  localparam IDLE                  = 3'd5;
  localparam SEND_START_REQ        = 3'd0;
  localparam SEND_LFSR_CLEAR_REQ   = 3'd1;
  localparam SEND_PATTERN          = 3'd2;
  localparam REQUEST_RESULTS       = 3'd3;
  localparam SEND_END_REQ          = 3'd4;
  localparam TEST_FINISHED         = 3'd6;

  // generator control words
  localparam GEN_OFF        = 2'b00;
  localparam GEN_CLEAR_LFSR = 2'b01;
  localparam GEN_LFSR       = 2'b10;
  localparam GEN_PER_LANE   = 2'b11;

  // sideband flags
  reg sb_data_pattern;
  reg sb_burst_count;
  reg sb_comparison_mode;

  reg [2:0] current_state, next_state;

  // sideband data
  always @(*) begin
    o_sideband_data = {11'h000, sb_comparison_mode, sb_burst_count, 3'b000, sb_data_pattern};
  end

  // FSM state register
  always @(posedge i_clk or negedge i_rst_n) begin
    if(!i_rst_n)
      current_state <= IDLE;
    else
      current_state <= next_state;
  end

  // next-state logic
  always @(*) begin
    next_state = current_state;
    case(current_state)
      IDLE: begin
        if(i_en_d2c_tx) 
         next_state = SEND_START_REQ;
      end
      SEND_START_REQ: begin
        if(!i_en_d2c_tx)
          next_state = IDLE;
        else if(i_sideband_message_valid && i_sideband_message == 4'b0010)
          next_state = SEND_LFSR_CLEAR_REQ;
        else  
          next_state = SEND_START_REQ;
      end
      SEND_LFSR_CLEAR_REQ: begin
        if(!i_en_d2c_tx) 
          next_state = IDLE;
        else if(i_sideband_message_valid && i_sideband_message == 4'b0100)
          next_state = SEND_PATTERN;
        else  
          next_state = SEND_LFSR_CLEAR_REQ;
      end
      SEND_PATTERN: begin
        if(!i_en_d2c_tx) 
         next_state = IDLE;
        else if(i_pattern_finished)
         next_state = REQUEST_RESULTS;
        else   
         next_state = SEND_PATTERN;
      end
      REQUEST_RESULTS: begin
        if(!i_en_d2c_tx) 
          next_state = IDLE;
        else if(i_sideband_message_valid && i_sideband_message == 4'b0110)
          next_state = SEND_END_REQ;
        else   
          next_state = REQUEST_RESULTS;
      end
      SEND_END_REQ: begin
        if(!i_en_d2c_tx) 
          next_state = IDLE;
        else if(i_sideband_message_valid && i_sideband_message == 4'b1000)
          next_state = TEST_FINISHED;
        else   
          next_state = SEND_END_REQ;
      end
      TEST_FINISHED: begin
        if(!i_en_d2c_tx) 
         next_state = IDLE;
        else   
         next_state = TEST_FINISHED;
      end
      default: next_state = IDLE;
    endcase
  end

wire enter_send_pattern;

assign enter_send_pattern = (current_state != SEND_PATTERN) && (next_state == SEND_PATTERN) && (i_mainband_or_valtrain_test == 1);


//wire enter_test_finished;

//assign enter_test_finished = (current_state != TEST_FINISHED) && (next_state    == TEST_FINISHED);
       
 
always @(posedge i_clk or negedge i_rst_n) begin
  if(!i_rst_n)
    o_val_pattern_en <= 1'b0;
  else
    o_val_pattern_en <= enter_send_pattern;
end
/*
always @(posedge i_clk or negedge i_rst_n) begin
  if(!i_rst_n)
    o_test_ack_tx <= 1'b0;
  else
    o_test_ack_tx <= enter_test_finished;
end      
       */
wire enter_send_start_req;
wire enter_lfsr_clear_req;
wire enter_request_results;
wire enter_send_end_req;

assign enter_send_start_req      = (current_state != SEND_START_REQ)      && (next_state == SEND_START_REQ);
assign enter_lfsr_clear_req      = (current_state != SEND_LFSR_CLEAR_REQ)  && (next_state == SEND_LFSR_CLEAR_REQ);
assign enter_request_results     = (current_state != REQUEST_RESULTS)     && (next_state == REQUEST_RESULTS);
assign enter_send_end_req         = (current_state != SEND_END_REQ)        && (next_state == SEND_END_REQ);

always @(posedge i_clk or negedge i_rst_n) begin
    if(!i_rst_n)
        o_message_valid <= 1'b0;
    else if(i_busy_negedge_detected && ~i_valid_rx)
        o_message_valid <= 1'b0;
    else
        o_message_valid <= enter_send_start_req | enter_lfsr_clear_req |enter_request_results | enter_send_end_req;
end

reg data_valid_pulse;

always @(posedge i_clk or negedge i_rst_n) begin
    if(!i_rst_n)
        data_valid_pulse <= 0;
    else
        
        data_valid_pulse <= (~(current_state == SEND_START_REQ) && (next_state == SEND_START_REQ)) ? 1'b1 : 1'b0;
end

assign o_data_valid_next = data_valid_pulse;

always @(posedge i_clk or negedge i_rst_n) begin
    if(!i_rst_n)
        o_data_valid <= 0;
    else if(i_busy_negedge_detected && ~i_valid_rx)
        o_data_valid <= 0;
    else
        o_data_valid <= data_valid_pulse; 
end


  // output logic
  always @(posedge i_clk or negedge i_rst_n) begin
    if(!i_rst_n) begin
      o_sideband_message  <= 4'b0000;
      //o_data_valid        <= 1'b0;
      //o_val_pattern_en    <= 1'b0;
      o_generator_mode_cw <= GEN_OFF;
      o_test_ack_tx       <= 1'b0;
      sb_data_pattern     <= 1'b0;
      sb_burst_count      <= 1'b0;
      sb_comparison_mode  <= 1'b0;
    end 
    /*
    else begin
      // defaults
      o_sideband_message  <= 4'b0000;
      //o_data_valid        <= 1'b0;
      o_val_pattern_en    <= 1'b0;
      o_generator_mode_cw <= GEN_OFF;
      o_test_ack_tx       <= 1'b0;
        */
      case(current_state)
        IDLE: begin
          o_sideband_message  <= 4'b0000;
          //o_val_pattern_en    <= 1'b0;
          o_generator_mode_cw <= GEN_OFF;
          o_test_ack_tx       <= 1'b0; 
          sb_data_pattern     <= 1'b0;
          sb_burst_count      <= 1'b0;
          sb_comparison_mode  <= 1'b0;
          
          if(next_state == SEND_START_REQ) 
            o_sideband_message <= 4'b0001;
            
        end
        SEND_START_REQ: begin
          sb_data_pattern    <= i_mainband_or_valtrain_test;
          sb_burst_count     <= i_mainband_or_valtrain_test;
          sb_comparison_mode <= 1'b0;
          //o_sideband_message <= 4'b0001;
          if(next_state == SEND_LFSR_CLEAR_REQ) 
            o_sideband_message <= 4'b0011;
        end
        SEND_LFSR_CLEAR_REQ: begin
          //o_sideband_message  <= 4'b0010;
          o_generator_mode_cw <= GEN_CLEAR_LFSR;
        end
        SEND_PATTERN: begin
          //o_sideband_message <= 4'b0011;
          //if(i_mainband_or_valtrain_test)
            //o_val_pattern_en <= enter_send_pattern;
          //else begin
            case({i_mainband_or_valtrain_test, i_lfsr_or_perlane})
              2'b00:begin
               o_generator_mode_cw <= GEN_LFSR;
               //o_val_pattern_en    <= 1'b0;
              end
              2'b01:begin
               o_generator_mode_cw <= GEN_PER_LANE;
               //o_val_pattern_en    <= 1'b0;
              end
              default:begin
               //o_val_pattern_en <= enter_send_pattern;
               o_generator_mode_cw <= GEN_OFF;
              end
            endcase
          //end
          
          if(next_state == REQUEST_RESULTS) 
                o_sideband_message <= 4'b0101;
        end
        
        REQUEST_RESULTS: begin
          //o_val_pattern_en <= 1'b0;
          o_generator_mode_cw <= GEN_OFF;
          //o_sideband_message <= 4'b0101;
          if(next_state == SEND_END_REQ) 
            o_sideband_message <= 4'b0111;
        end
        
        SEND_END_REQ: begin
          //o_sideband_message <= 4'b0111;
          //o_test_ack_tx      <= 1'b1;
          if(next_state == TEST_FINISHED) begin
            o_sideband_message <= 4'b0000;
          end
        end
        
        TEST_FINISHED: begin
          o_test_ack_tx      <= 1'b1;
        end
        
    endcase
  end

endmodule