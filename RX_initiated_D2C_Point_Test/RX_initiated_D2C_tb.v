`timescale 1ns/1ps

module RX_initiated_D2C_wrapper_tb;

  localparam CLK_PERIOD = 10;

  // ---------------- Inputs ----------------
  reg i_clk;
  reg i_rst_n;
  reg i_en_d2c_rx;
  reg i_datavref_or_valvref;
  reg i_pattern_finished;
  reg i_busy_negedge_detected;
  reg i_SB_Busy;

  reg [3:0] i_decoded_sb_msg;
  reg       i_message_valid;
  reg [15:0] i_per_lane_error;

  // ---------------- Outputs ----------------
  wire [3:0] o_encoded_sb_msg;
  wire       o_msg_valid;
  wire       o_data_valid;
  wire       o_ack_rx;
  wire [15:0] o_data_bus;
  wire [15:0] o_per_lane_error;
  wire       o_val_pattern_en;
  wire [1:0] o_generator_mode_cw;
  wire       o_comparison_valid_en;
  wire [1:0] o_comparator_mode_cw;

  // ---------------- Instantiate Wrapper ----------------
  RX_initiated_D2C_wrapper DUT (
    .i_clk(i_clk),
    .i_rst_n(i_rst_n),
    .i_en_d2c_rx(i_en_d2c_rx),
    .i_datavref_or_valvref(i_datavref_or_valvref),
    .i_pattern_finished(i_pattern_finished),
    .i_SB_Busy(i_SB_Busy),
    .i_busy_negedge_detected(i_busy_negedge_detected),
    .i_decoded_sb_msg(i_decoded_sb_msg),
    .i_message_valid(i_message_valid),
    .i_per_lane_error(i_per_lane_error),
    .o_encoded_sb_msg(o_encoded_sb_msg),
    .o_msg_valid(o_msg_valid),
    .o_data_valid(o_data_valid),
    .o_ack_rx(o_ack_rx),
    .o_data_bus(o_data_bus),
    .o_per_lane_error(o_per_lane_error),
    .o_val_pattern_en(o_val_pattern_en),
    .o_generator_mode_cw(o_generator_mode_cw),
    .o_comparison_valid_en(o_comparison_valid_en),
    .o_comparator_mode_cw(o_comparator_mode_cw)
  );

  // ---------------- Clock ----------------
  initial i_clk = 0;
  always #(CLK_PERIOD/2) i_clk = ~i_clk;

  // ---------------- Utility tasks ----------------
  task clear_sideband;
    begin
      #(CLK_PERIOD);
      i_decoded_sb_msg = 4'b0000;
      i_message_valid  = 1'b0;
    end
  endtask

  task busy_pulse;
    begin
      i_busy_negedge_detected = 1'b1;
      #CLK_PERIOD;
      i_busy_negedge_detected = 1'b0;
    end
  endtask
 /* 
  // ---------------- Continuous signal printing ----------------
initial begin
  wait(i_rst_n == 1 && i_en_d2c_rx == 1); 
  repeat (200) @(posedge i_clk) begin
    $display("[%0t] o_encoded_sb_msg=%b, o_msg_valid=%b, i_decoded_sb_msg=%b, i_message_valid=%b",
             $time, o_encoded_sb_msg, o_msg_valid, i_decoded_sb_msg, i_message_valid);
  end
end
  */

  // ---------------- Main Test ----------------
  initial begin
    $display("=========================================");
    $display(" RX INITIATED D2C WRAPPER FULL TB ");
    $display("=========================================");

    // ---------------- Init ----------------
    i_rst_n = 0;
    i_en_d2c_rx = 0;
    i_datavref_or_valvref = 0;
    i_pattern_finished = 0;
    i_busy_negedge_detected = 0;
    i_SB_Busy = 0;
    i_decoded_sb_msg = 0;
    i_message_valid = 0;
    i_per_lane_error = 16'h0;

    // Wait a little to avoid HIZ at start
    #(2*CLK_PERIOD);

    // ---------------- Reset Release ----------------
    i_rst_n = 1;
    $display("[%0t] [INFO] Reset released", $time);

    #(2*CLK_PERIOD);

    // ---------------- Enable Wrapper ----------------
    i_en_d2c_rx = 1;
    $display("[%0t] [INFO] RX Initiated D2C enabled", $time);

    @(posedge i_clk);

    // ---------------- START ----------------
    wait(o_msg_valid && o_encoded_sb_msg == 4'b0001);
    $display("[Wrapper] START_REQ observed");
    /*
    i_decoded_sb_msg = 4'b0010;
    i_message_valid  = 1'b1;
    //send_msg(4'b0010); // START_RESP to TX
    $display("[TB ] Inject START_RESPOND");
    busy_pulse();
    clear_sideband();
    */
    #(2*CLK_PERIOD);
    i_decoded_sb_msg = 4'b0001;
    i_message_valid  = 1'b1;
    //send_msg(4'b0001); // START_REQ to RX
    $display("[TB ] Inject START_REQUEST to RX");
    wait(o_msg_valid && o_encoded_sb_msg == 4'b0010);
    $display("[Wrapper] START_RESP observed");
    @(posedge i_clk);
    
    i_decoded_sb_msg = 4'b0010;
    i_message_valid  = 1'b1;
    //send_msg(4'b0010); // START_RESP to TX
    $display("[TB ] Inject START_RESPOND");
    busy_pulse();
    clear_sideband();
    
    // ---------------- LFSR CLEAR ----------------
    wait(o_msg_valid && o_encoded_sb_msg == 4'b0011);
    $display("[Wrapper] LFSR CLEAR REQ observed");
    i_decoded_sb_msg = 4'b0100;
    i_message_valid  = 1'b1;
    busy_pulse();
    clear_sideband();
    $display("[TB ] Inject LFSR CLEAR");
    
    i_decoded_sb_msg = 4'b0011;// LFSR_CLEAR_REQ to RX
    i_message_valid  = 1'b1;
    busy_pulse();
    clear_sideband();
    $display("[TB ] Inject LFSR CLEAR to RX");
    wait(o_msg_valid && o_encoded_sb_msg == 4'b0100);
    $display("[Wrapper] LFSR_CLEAR_RESP observed");
    
    // ---------------- PATTERN TRANSMIT ----------------
    #(2*CLK_PERIOD);
    i_pattern_finished = 1;
    $display("[TB] Pattern finished");
    #CLK_PERIOD;
    i_pattern_finished = 0;

    // ---------------- COUNT DONE ----------------
    wait(o_msg_valid && o_encoded_sb_msg == 4'b0101);
    $display("[Wrapper] COUNT_DONE_REQ observed");
    /*
    i_decoded_sb_msg = 4'b0110;
    i_message_valid  = 1'b1;
    busy_pulse();
    clear_sideband();
    $display("[TB ] Inject COUNT_DONE_RSP");
    */
    #(2*CLK_PERIOD);
    i_decoded_sb_msg = 4'b0101;
    i_message_valid  = 1'b1;
    busy_pulse();
    clear_sideband();
    $display("[TB ] Inject COUNT_DONE_REQ to RX");
    wait(o_msg_valid && o_encoded_sb_msg == 4'b0110);
    $display("[Wrapper] COUNT_DONE_RESP observed");
    @(posedge i_clk);
    
    i_decoded_sb_msg = 4'b0110;
    i_message_valid  = 1'b1;
    busy_pulse();
    clear_sideband();
    $display("[TB ] Inject COUNT_DONE_RSP");

    // ---------------- END ----------------
    wait(o_msg_valid && o_encoded_sb_msg == 4'b0111);
    i_decoded_sb_msg = 4'b0110;
    i_message_valid  = 1'b1;
    busy_pulse();
    clear_sideband();
    $display("[TB ] Inject SEND_END_RSP");
    
    #(2*CLK_PERIOD);
    i_decoded_sb_msg = 4'b0111;
    i_message_valid  = 1'b1;
    busy_pulse();
    clear_sideband();
    $display("[TB ] Inject END_REQ to RX");
    wait(o_msg_valid && o_encoded_sb_msg == 4'b1000);
    $display("[Wrapper] END_RESP observed");
    
    i_decoded_sb_msg = 4'b1000;
    i_message_valid  = 1'b1;
    busy_pulse();
    clear_sideband();
    $display("[TB ] Inject END_RESP observed");

    // ---------------- Test Done ----------------//
    if(o_ack_rx)
      $display("[%0t] [PASS] TEST_DONE acknowledged by Wrapper", $time);
    else
      $display("[%0t] [FAIL] TEST_DONE missing", $time);

    #(CLK_PERIOD);

    // ---------------- disable Wrapper ----------------
    i_en_d2c_rx = 0;
    #(4*CLK_PERIOD);
    $display("==============================================");
    $display(" ALL PROTOCOL TESTS COMPLETED ");
    $display("==============================================");
    $stop;
  end

endmodule