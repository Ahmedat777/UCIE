`timescale 1ns/1ps

module TX_initiated_D2C_wrapper_tb;

  localparam CLK_PERIOD = 10;
  localparam NUM_LANE   = 16;

  // ================= CLOCK =================
  reg i_clk;
  initial i_clk = 0;
  always #(CLK_PERIOD/2) i_clk = ~i_clk;

  // ================= INPUTS =================
  reg                  i_rst_n;
  reg                  i_en;

  reg  [3:0]           i_sideband_message;
  reg  [NUM_LANE-1:0]  i_sideband_data;
  reg                  i_sideband_message_valid;
  reg                  i_busy_negedge_detected;

  reg                  i_mainband_or_valtrain_test;
  reg                  i_lfsr_or_perlane;
  reg                  i_pattern_finished;

  reg  [NUM_LANE-1:0]  i_per_lane_error;

  // ================= OUTPUTS =================
  wire [3:0]           o_sideband_message;
  wire                 o_message_valid;
  wire [NUM_LANE-1:0]  o_sideband_data;
  wire                 o_data_valid;

  wire                 o_val_pattern_en;
  wire [1:0]           o_generator_mode_cw;
  wire [1:0]           o_comparator_mode_cw;
  wire                 o_comparison_valid_en;

  wire                 o_test_ack;
  wire [NUM_LANE-1:0]  o_mainband_lanes_result;

  // ================= DUT =================
  TX_initiated_D2C_wrapper #(
    .NUM_LANE(NUM_LANE)
  ) DUT (
    .i_clk(i_clk),
    .i_rst_n(i_rst_n),
    .i_en(i_en),

    .i_sideband_message(i_sideband_message),
    .i_sideband_data(i_sideband_data),
    .i_sideband_message_valid(i_sideband_message_valid),
    .i_busy_negedge_detected(i_busy_negedge_detected),

    .i_mainband_or_valtrain_test(i_mainband_or_valtrain_test),
    .i_lfsr_or_perlane(i_lfsr_or_perlane),
    .i_pattern_finished(i_pattern_finished),

    .i_per_lane_error(i_per_lane_error),

    .o_sideband_message(o_sideband_message),
    .o_message_valid(o_message_valid),
    .o_sideband_data(o_sideband_data),
    .o_data_valid(o_data_valid),

    .o_val_pattern_en(o_val_pattern_en),
    .o_generator_mode_cw(o_generator_mode_cw),
    .o_comparator_mode_cw(o_comparator_mode_cw),
    .o_comparison_valid_en(o_comparison_valid_en),

    .o_test_ack(o_test_ack),
    .o_mainband_lanes_result(o_mainband_lanes_result)
  );

  // ================= TASKS =================
  task clear_sideband;
    begin
      #(CLK_PERIOD);
      i_sideband_message       = 4'b0000;
      i_sideband_message_valid = 1'b0;
    end
  endtask

  task busy_pulse;
    begin
      @(posedge i_clk);
      i_busy_negedge_detected = 1'b1;
      @(posedge i_clk);
      i_busy_negedge_detected = 1'b0;
    end
  endtask

  // ================= MAIN TEST =================
  initial begin
    $display("========================================");
    $display(" TX INITIATED D2C WRAPPER FULL TB ");
    $display("========================================");

    // -------- init --------
    i_rst_n  = 0;
    i_en     = 0;

    i_sideband_message = 0;
    i_sideband_message_valid = 0;
    i_sideband_data = 0;
    i_busy_negedge_detected = 0;

    i_mainband_or_valtrain_test = 0;
    i_lfsr_or_perlane = 0;
    i_pattern_finished = 0;

    i_per_lane_error = 16'h00F0;

    #(2*CLK_PERIOD);

    // -------- reset release --------
    i_rst_n = 1;
    $display("[%0t] Reset released", $time);
    #(2*CLK_PERIOD);

    // -------- enable --------
    i_en = 1;
    $display("[%0t] Wrapper enabled", $time);

    // =====================================================
    // START REQ (from wrapper)
    // =====================================================
    @(posedge i_clk);
    wait(o_message_valid && o_sideband_message == 4'b0001);
    $display("[WRAPPER] START_REQ observed");
    /*
    @(posedge i_clk);
    wait(o_message_valid && o_sideband_message == 4'b0010);
    $display("[WRAPPER] WAIT_START_REQ observed");
    */
    @(posedge i_clk);
    i_sideband_message       = 4'b0001;
    i_sideband_message_valid = 1'b1;
    $display("[TB] START_REQ observed to RX");
    
    wait(o_message_valid && o_sideband_message == 4'b0010);
    $display("[WRAPPER] WAIT_LFSR_CLEAR_REQ observed");
    busy_pulse();
    clear_sideband();
    
    /*
    wait(o_message_valid && o_sideband_message == 4'b0010);
    $display("[WRAPPER] WAIT_LFSR_CLEAR_REQ observed");
*/

    @(posedge i_clk);
    // START_RESP from outside
    i_sideband_message       = 4'b0010;
    i_sideband_message_valid = 1'b1;
    $display("[TB] START_RSP observed");
    wait(o_message_valid && o_sideband_message == 4'b0011);
    $display("[WRAPPER] LFSR_CLEAR_REQ observed");
    busy_pulse();
    clear_sideband();

    // =====================================================
    // LFSR CLEAR
    // =====================================================
    //@(posedge i_clk);
    /*
    wait(o_message_valid && o_sideband_message == 4'b0011);
    $display("[WRAPPER] LFSR_CLEAR_REQ observed");
    */
    
    @(posedge i_clk);
    i_sideband_message       = 4'b0011;
    i_sideband_message_valid = 1'b1;
    $display("[TB] LFSR_CLEAR_REQ observed to RX");
    @(posedge i_clk);
    wait(o_message_valid && o_sideband_message == 4'b0100);
    $display("[WRAPPER] LFSR_CLEAR observed to RX DONE");
    busy_pulse();
    clear_sideband();
    /*
    //@(posedge i_clk);
    wait(o_message_valid && o_sideband_message == 4'b0010);
    $display("[WRAPPER] LFSR_CLEAR_REQ observed to RX DONE");
    */
    //@(posedge i_clk);
    /*
    wait(o_message_valid && o_sideband_message == 4'b0010);
    $display("[WRAPPER] LFSR_CLEAR_REQ observed to RX DONE");
    */
    
    @(posedge i_clk);
    i_sideband_message       = 4'b0100;
    i_sideband_message_valid = 1'b1;
    busy_pulse();
    clear_sideband();

    // =====================================================
    // PATTERN
    // =====================================================
    #(4*CLK_PERIOD);
    i_pattern_finished = 1;
    $display("[TB] Pattern finished");
    #CLK_PERIOD;
    i_pattern_finished = 0;

    // =====================================================
    // RESULT REQUEST
    // =====================================================
    //@(posedge i_clk);
    wait(o_message_valid && o_sideband_message == 4'b0101);
    $display("[WRAPPER] RESULT_REQ observed");
    
    @(posedge i_clk);
    i_sideband_message       = 4'b0101;
    i_sideband_message_valid = 1'b1;
    $display("[TB] RESULT_REQ observed to RX");
    
    wait(o_message_valid && o_sideband_message == 4'b0110);
    $display("[WRAPPER] RESULT_REQ observed to RX DONE");
    
    busy_pulse();
    clear_sideband();
    /*
    //@(posedge i_clk);
    wait(o_message_valid && o_sideband_message == 4'b0100);
    $display("[WRAPPER] RESULT_REQ observed to RX DONE");
    */
    @(posedge i_clk);
    i_sideband_message       = 4'b0110;
    i_sideband_message_valid = 1'b1;
    
    wait(o_message_valid && o_sideband_message == 4'b0111);
    $display("[WRAPPER] END_REQ observed");
    
    busy_pulse();
    clear_sideband();

    // =====================================================
    // END
    // =====================================================
    //@(posedge i_clk);
    /*
    wait(o_message_valid && o_sideband_message == 4'b0111);
    $display("[WRAPPER] END_REQ observed");
    */
    @(posedge i_clk);
    i_sideband_message       = 4'b0111;
    i_sideband_message_valid = 1'b1;
    $display("[TB] END_REQ observed to RX");
    
    wait(o_message_valid && o_sideband_message == 4'b1000);
    $display("[WRAPPER] END_REQ observed to RX DONE");
    
    busy_pulse();
    clear_sideband();
    /*
    //@(posedge i_clk);
    wait(o_message_valid && o_sideband_message == 4'b1000);
    $display("[WRAPPER] END_REQ observed to RX DONE");
    */
    @(posedge i_clk);
    i_sideband_message       = 4'b1000;
    i_sideband_message_valid = 1'b1;
    busy_pulse();
    clear_sideband();

    // =====================================================
    // DONE
    // =====================================================
    if (o_test_ack)
      $display("[%0t] [PASS] TEST DONE ACKED", $time);
    else
      $display("[%0t] [FAIL] TEST ACK MISSING", $time);

    #(3*CLK_PERIOD);
    i_en = 0;

    $display("========================================");
    $display(" ALL TX INITIATED WRAPPER TESTS DONE ");
    $display("========================================");
    $stop;
  end
  /*
  initial begin
  $monitor("[%0t] msg_valid=%b, sb_msg=%b", $time, o_message_valid, o_sideband_message);
  end
  */
endmodule