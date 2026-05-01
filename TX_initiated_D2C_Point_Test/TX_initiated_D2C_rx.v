module TX_initiated_D2C_rx #(
    parameter NUM_LANE = 16
)(
    input  wire                  i_clk,
    input  wire                  i_rst_n,
    input  wire                  i_en_d2c_tx,

    /* handshake with TX */
    input  wire                  i_busy_negedge_detected,
    input  wire                  i_valid_tx,

    /* configuration */
    input  wire                  i_mainband_or_valtrain_test,
    input  wire                  i_lfsr_or_perlane,

    /* sideband from TX */
    input  wire [3:0]            i_sideband_message,
    input  wire                  i_sideband_message_valid,

    /* results */
    input  wire [NUM_LANE-1:0]   i_per_lane_error,

    /* outputs to TX */
    output reg  [3:0]            o_sideband_message,
    output reg                   o_message_valid,
    output reg  [NUM_LANE-1:0]   o_sideband_data,
    output reg                   o_data_valid,

    /* comparator control */
    output reg  [1:0]            o_comparator_mode_cw,
    output reg                   o_comparison_valid_en,

    /* test status */
    output reg                   o_test_ack_rx
);

    /*=============================
      FSM States 
    ==============================*/
    localparam IDLE                 = 3'd0;
    localparam WAIT_START_REQ       = 3'd1;
    localparam WAIT_LFSR_CLEAR_REQ  = 3'd2;
    localparam CLEAR_LFSR           = 3'd3;
    localparam WAIT_RESULT_REQ      = 3'd4;
    localparam WAIT_END_REQ         = 3'd5;
    localparam END_RESP             = 3'd6;
    localparam TEST_FINISHED        = 3'd7;

    reg [2:0] current_state, next_state;

    /*=============================
      State Register
    ==============================*/
    always @(posedge i_clk or negedge i_rst_n) begin
        if (!i_rst_n)
            current_state <= IDLE;
        else
            current_state <= next_state;
    end
    
    // ==========================================================//
    
    wire valid_negedge_detected;
    reg o_message_valid_reg;
    //wire valid_cond = (current_state!= next_state) && ((next_state == WAIT_LFSR_CLEAR_REQ) || (next_state == CLEAR_LFSR) || (next_state == WAIT_END_REQ) || (next_state == END_RESP));
    wire valid_cond = (current_state!= next_state) && (next_state != IDLE) && (next_state != WAIT_START_REQ) && (next_state != WAIT_RESULT_REQ) && (next_state != TEST_FINISHED);
    
    
    assign valid_negedge_detected = ~o_message_valid && o_message_valid_reg;
    
    always@(posedge i_clk or negedge i_rst_n) begin
        if (!i_rst_n) 
          o_message_valid_reg <= 1'b0;
        else
          o_message_valid_reg <= o_message_valid;
    end
   
    
// ==========================================================//
   
 
reg pending_valid;
/*
always @(posedge i_clk or negedge i_rst_n) begin
  if (!i_rst_n) begin
    o_message_valid   <= 0;
    pending_valid     <= 0;
  end else begin

    // detect event that requires valid
    if (valid_cond)
      pending_valid <= 1;

    // generate valid when TX is free
    if (pending_valid && !i_valid_tx) begin
      o_message_valid <= 1;
      pending_valid   <= 0;   // consume the request
    end else if (i_busy_negedge_detected) begin
      o_message_valid <= 0;      // end pulse
    end

  end
end



always @(posedge i_clk or negedge i_rst_n) begin
  if (!i_rst_n) begin
    o_message_valid <= 1'b0;
    pending_valid   <= 1'b0;
  end else begin

    // latch request if TX is busy
    if (valid_cond && i_valid_tx)
      pending_valid <= 1'b1;

    // raise valid when TX becomes free
    if ((valid_cond || pending_valid) && !i_valid_tx)
      o_message_valid <= 1'b1;

    // clear valid only on busy negedge
    if (i_busy_negedge_detected) begin
      o_message_valid <= 1'b0;
    else if(i_busy_negedge_detected && !i_valid_tx) 
      pending_valid   <= 1'b0;
    end

  end
end
*/

always @(posedge i_clk or negedge i_rst_n) begin
  if (!i_rst_n) begin
    o_message_valid <= 1'b0;
    pending_valid   <= 1'b0;

  end else begin
    // default: pulse
    o_message_valid <= 1'b0;

    // latch request if TX is busy
    if (valid_cond && i_valid_tx)
      pending_valid <= 1'b1;

    // fire ONE pulse when TX becomes free
    if ((valid_cond || pending_valid) && !i_valid_tx) begin
      o_message_valid <= 1'b1;
      pending_valid   <= 1'b0;
    end
  end
end

// ==========================================================//

/*
reg pending_data_valid;

always @(posedge i_clk or negedge i_rst_n) begin
  if (!i_rst_n) begin
    o_data_valid        <= 0;
    pending_data_valid  <= 0;
  end else begin

    // detect event that requires data valid
    if (valid_cond && next_state == WAIT_END_REQ)
      pending_data_valid <= 1;

    // generate data valid pulse when TX is free
    if (pending_data_valid && !i_valid_tx) begin
      o_data_valid       <= 1;
      pending_data_valid <= 0;  // consume the request
    end else if (i_busy_negedge_detected) begin
      o_data_valid <= 0;         // end pulse
    end

  end
end
*/

always @(posedge i_clk or negedge i_rst_n) begin
  if (!i_rst_n) begin
    o_data_valid  <= 1'b0;
  end else begin
    // default: pulse
    o_data_valid <= 1'b0;

    // fire data valid exactly when data is driven
    if (!i_valid_tx && current_state == WAIT_RESULT_REQ && next_state == WAIT_END_REQ) begin
      o_data_valid <= 1'b1;
    end
  end
end

// ==========================================================//
/*
always @(posedge i_clk or negedge i_rst_n) begin
    if (!i_rst_n) begin
        o_test_ack_rx <= 1'b0;
    end else begin
        // 1-cycle pulse exactly on entry to TEST_FINISHED
        if (current_state == END_RESP && next_state == TEST_FINISHED)
            o_test_ack_rx <= 1'b1;
        else
            o_test_ack_rx <= 1'b0;
    end
end
*/
    /*=============================
      Next State Logic
    ==============================*/
    always @(*) begin
        next_state = current_state;
        case (current_state)

            IDLE:begin
              if (i_en_d2c_tx)
                next_state = WAIT_START_REQ;
            end
            
            WAIT_START_REQ:begin
                if (!i_en_d2c_tx)
                    next_state = IDLE;
                else if (i_sideband_message_valid && i_sideband_message == 4'b0001)
                    next_state = WAIT_LFSR_CLEAR_REQ;
            end
            
            WAIT_LFSR_CLEAR_REQ:begin
                if (!i_en_d2c_tx)
                    next_state = IDLE;
                else if (i_sideband_message_valid && i_sideband_message == 4'b0011)
                    next_state = CLEAR_LFSR;
            end
            
            CLEAR_LFSR:begin
                if (!i_en_d2c_tx)
                    next_state = IDLE;
                else if (valid_negedge_detected) // Move to result request only after current valid data has finished (detect 1->0 edge of o_message_valid) 
                    next_state = WAIT_RESULT_REQ;
            end
            
            WAIT_RESULT_REQ:begin
                if (!i_en_d2c_tx)
                    next_state = IDLE;
                else if (i_sideband_message_valid && i_sideband_message == 4'b0101)
                    next_state = WAIT_END_REQ;
            end
            
            WAIT_END_REQ :begin
                if (!i_en_d2c_tx)
                    next_state = IDLE;
                else if (i_sideband_message_valid && i_sideband_message == 4'b0111)
                    next_state = END_RESP;
            end
            
            END_RESP:begin
                if (!i_en_d2c_tx)
                  next_state = IDLE;
                else if (valid_negedge_detected) // Move to finish the test only after current valid data has finished (detect 1->0 edge of o_message_valid) 
                  next_state = TEST_FINISHED;
            end  
                  
            TEST_FINISHED:begin
              if (!i_en_d2c_tx)
                next_state = IDLE;
            end
            
        endcase
    end
    

    /*=============================
      Output Logic (clean pulses)
    ==============================*/
    always @(posedge i_clk or negedge i_rst_n) begin
        if (!i_rst_n) begin
            o_sideband_message    <= 4'b0;
            //o_message_valid       <= 1'b0;
            o_sideband_data       <= {NUM_LANE{1'b0}};
            //o_data_valid          <= 1'b0;
            o_comparator_mode_cw  <= 2'b00;
            o_comparison_valid_en <= 1'b0;
            o_test_ack_rx         <= 1'b0;
        end else begin
            /* defaults every cycle */
            o_sideband_message    <= o_sideband_message;
            //o_message_valid       <= 1'b0;
            o_sideband_data       <= o_sideband_data;
            //o_data_valid          <= 1'b0;
            o_comparison_valid_en <= 1'b0;
            //o_test_ack_rx         <= 1'b0;

            case (current_state)
                IDLE:begin
                  o_sideband_message    <= 4'b0000;
                  //o_message_valid       <= 1'b0;
                  //o_sideband_data       <= {NUM_LANE{1'b0}};
                  //o_data_valid          <= 1'b0;
                  o_comparison_valid_en <= 1'b0;
                  o_test_ack_rx         <= 1'b0;
                end

                WAIT_START_REQ: begin
                  if (next_state == WAIT_LFSR_CLEAR_REQ) 
                      o_sideband_message <= 4'b0010; // start resp
                        //o_message_valid    <= 1'b1;
                end

                WAIT_LFSR_CLEAR_REQ: begin
                  if (next_state == CLEAR_LFSR) begin
                      o_sideband_message <= 4'b0100; // clear LFSR resp
                        //o_message_valid    <= 1'b1;
                  end
                end

                CLEAR_LFSR: begin
                  if(!i_mainband_or_valtrain_test)
                    o_comparator_mode_cw<= 2'b01;
                    /*
                    if (next_state == WAIT_RESULT_REQ) begin
                        o_sideband_message <= 4'b0100; // clear resp
                        //o_message_valid    <= 1'b1;
                    end */
                end

                WAIT_RESULT_REQ: begin
                  case ({i_mainband_or_valtrain_test, i_lfsr_or_perlane})
                    2'b00:begin
                      o_comparator_mode_cw<=2'b10; //lfsr pattern generation
								      o_comparison_valid_en<=0;
							      end
							      2'b01:begin
								      o_comparator_mode_cw<=2'b11;//per lane id 
								      o_comparison_valid_en<=0;
							      end
							      default : begin
								      o_comparison_valid_en<=1;
								      o_comparator_mode_cw<=2'b00;
							      end
					       endcase
					       if (next_state == WAIT_END_REQ) begin
                        o_sideband_message <= 4'b0110; // result resp
                        //o_message_valid    <= 1'b1;
                        o_sideband_data    <= i_per_lane_error;
                        //o_data_valid       <= 1'b1;
                 end
                end

                WAIT_END_REQ: begin
                  
                  if (next_state == END_RESP) 
                        o_sideband_message <= 4'b1000; // result resp
                        //o_message_valid    <= 1'b1;
                end
                
                END_RESP:begin
                  /*
                  if (next_state == TEST_FINISHED)
                    o_test_ack_rx <= 1'b1;
                    */
                end

                TEST_FINISHED: begin
                    o_test_ack_rx <= 1'b1;
                end
                
                default:begin
                  o_sideband_message    <= o_sideband_message;
                  //o_message_valid       <= 1'b0;
                  o_sideband_data       <= o_sideband_data;
                  //o_data_valid          <= 1'b0;
                  o_comparator_mode_cw  <= 2'b00;
                  o_comparison_valid_en <= 1'b0;
                  //o_test_ack_rx         <= 1'b0;
                end
            endcase
        end
    end

endmodule

 