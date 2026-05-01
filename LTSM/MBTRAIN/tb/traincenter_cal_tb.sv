`timescale 1ns/1ps

module traincenter_cal_tb;
//tb_traincenter_cal_wrapper
  localparam SB_MSG_WIDTH = 4;

  // DUT inputs
  reg i_clk;
  reg i_rst_n;
  reg i_enable;
  reg [SB_MSG_WIDTH-1:0] i_decoded_sideband_msg;
  reg i_sideband_valid;
  reg i_falling_edge_busy;
  reg i_test_ack;

  // DUT outputs
  wire [SB_MSG_WIDTH-1:0] o_sideband_msg;
  wire o_valid;
  wire o_point_test_en;
  wire o_eye_width_sweep_en;
  wire o_test_ack;

  // Sideband encoding
  localparam START_REQUEST = 4'b0001;
  localparam START_RESPOND = 4'b0010;
  localparam END_REQUEST   = 4'b0011;
  localparam END_RESPOND   = 4'b0100;

  // DUT
  Traincenter_cal_wrapper #(.SB_MSG_WIDTH(SB_MSG_WIDTH)) DUT (
    .i_clk(i_clk),
    .i_rst_n(i_rst_n),
    .i_enable(i_enable),
    .i_decoded_sideband_msg(i_decoded_sideband_msg),
    .i_sideband_valid(i_sideband_valid),
    .i_falling_edge_busy(i_falling_edge_busy),
    .i_test_ack(i_test_ack),
    .o_sideband_msg(o_sideband_msg),
    .o_valid(o_valid),
    .o_point_test_en(o_point_test_en),
    .o_eye_width_sweep_en(o_eye_width_sweep_en),
    .o_test_ack(o_test_ack)
  );

  // Clock
  always #5 i_clk = ~i_clk;

  // -------------------------------
  // Utility tasks
  // -------------------------------
  task clear_sideband;
    begin
      i_decoded_sideband_msg = 0;
      i_sideband_valid = 0;
    end
  endtask

  task busy_pulse;
    begin
          i_falling_edge_busy = 1;
      #10 i_falling_edge_busy = 0;
    end
  endtask

  // -------------------------------
  // MAIN TEST
  // -------------------------------
  initial begin
    $display("=========================================");
    $display(" TRAINCENTER CAL WRAPPER FULL PROTOCOL TB ");
    $display("=========================================");

    // Init
    i_clk = 0;
    i_rst_n = 0;
    i_enable = 0;
    i_decoded_sideband_msg = 0;
    i_sideband_valid = 0;
    i_falling_edge_busy = 0;
    i_test_ack = 0;

    // Reset
    #20;
    i_rst_n = 1;
    $display("[PASS] Reset released");

    // Enable
    #10;
    i_enable = 1;
    $display("[INFO] Enable asserted");

    // =====================================================
    // TX → START_REQUEST
    // =====================================================
    wait (o_valid && o_sideband_msg == START_REQUEST);
    $display("[TX ] START_REQUEST observed");

    // TB responds
    #10;
    i_decoded_sideband_msg = START_RESPOND;
    i_sideband_valid = 1;
    $display("[TB ] Inject START_RESPOND");
    busy_pulse();
    clear_sideband();

    #20;
    i_decoded_sideband_msg = START_REQUEST;
    i_sideband_valid = 1;
    $display("[TB ] Inject START_REQUEST to RX");wait (o_valid && o_sideband_msg == START_RESPOND);
    $display("[RX ] START_RESPOND observed");

    #10
    busy_pulse();
    clear_sideband();

    // =====================================================
    // POINT TEST phase (TX)
    // =====================================================
    #20;
    if (o_point_test_en && o_eye_width_sweep_en)
      $display("[PASS] POINT_TEST enabled on TX");
    else
      $display("[FAIL] POINT_TEST not asserted");

    // End point test
    #10;
    i_test_ack = 1;
    #10;
    i_test_ack = 0;
    $display("[TB ] Point test ACK sent");

    // =====================================================
    // TX → END_REQUEST
    // =====================================================
    wait (o_valid && o_sideband_msg == END_REQUEST);
    $display("[TX ] END_REQUEST observed");

    #10;
    i_decoded_sideband_msg = END_RESPOND;
    i_sideband_valid = 1;
    $display("[TB ] Inject END_RESPOND");
    busy_pulse();
    clear_sideband();

     #10;
    i_decoded_sideband_msg = END_REQUEST;
    i_sideband_valid = 1;
    $display("[TB ] Inject END_REQUEST to RX");
    wait (o_valid && o_sideband_msg == END_RESPOND);
    $display("[RX ] END_RESPOND observed");
    #10
    busy_pulse();
    clear_sideband();  

    // =====================================================
    // TEST_END
    // =====================================================
    #20;
    if (o_test_ack)
      $display("[PASS] TEST_END acknowledged by TX & RX");
    else
      $display("[FAIL] TEST_END missing");

    // =====================================================
    // ENDING TESTS
    // =====================================================
    
    i_enable = 1'b0;
     $display("[INFO] Enable deasserted");
    

    // =====================================================
    // DONE
    // =====================================================
    #30;
    $display("=========================================");
    $display(" ALL PROTOCOL TESTS COMPLETED ");
    $display("=========================================");
    $stop;
  end

endmodule
