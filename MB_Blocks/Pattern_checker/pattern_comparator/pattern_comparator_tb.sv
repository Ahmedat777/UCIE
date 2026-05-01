`timescale 1ns/1ps
module tb_pattern_comparator;

  parameter LANE_WIDTH = 32;
  parameter NUM_LANE  = 16;

  reg clk, rst_n;
  reg en_comp;
  reg en_buffer;           
  reg comp_type;
  reg [1:0] comparator_mode;

  reg [LANE_WIDTH-1:0] gen_data [0:NUM_LANE-1];
  reg [LANE_WIDTH-1:0] recv_data[0:NUM_LANE-1];

  reg [11:0] per_lane_thresh;
  reg [15:0] agg_thresh;

  wire [NUM_LANE-1:0] per_lane_error;
  wire module_error;
  wire [15:0] aggregate_counter;
  wire aggregate_exceeded;

  // Instantiate DUT
  pattern_comparator #(
    .LANE_WIDTH(LANE_WIDTH),
    .NUM_LANE(NUM_LANE)
  ) DUT (
    .i_clk(clk),
    .i_rst_n(rst_n),
    .i_en_comparator(en_comp),
    .i_enable_buffer(en_buffer),
    .i_comp_type(comp_type),
    .i_comparator_mode(comparator_mode),
    .i_gen_data(gen_data),
    .i_data(recv_data),
    .i_per_lane_Threshold(per_lane_thresh),
    .i_aggregate_Threshold(agg_thresh),
    .o_per_lane_error(per_lane_error),
    .o_module_error(module_error),
    .o_aggregate_counter(aggregate_counter),
    .o_aggregate_exceeded(aggregate_exceeded)
  );

  // Clock generation
  initial clk = 0;
  always #5 clk = ~clk; // 100 MHz

  integer i, lane, frame;
  initial begin
    // Reset and init
    rst_n = 0;
    en_comp = 1;
    en_buffer = 1;           
    comp_type = 0;           
    comparator_mode = 2'b00;
    per_lane_thresh = 2;
    agg_thresh = 3;          

    for(i=0;i<NUM_LANE;i=i+1) begin
      gen_data[i]  = 32'hA5A5A5A5;
      recv_data[i] = 32'hA5A5A5A5;
    end

    #10 rst_n = 1;

    // ==========================
    // Per-lane comparison tests
    // ==========================
    comp_type = 1;           

    // Test 1: no errors
    comparator_mode = 2'b10; #10;  
    #10;                            
    comparator_mode = 2'b11; #10;  
    #10;                            
    $display("==== TEST 1 ? NO ERRORS ====");
    $display("per_lane_error = %b", per_lane_error);

    // Test 2: errors in lanes 0,3,4
    recv_data[0] = 32'hFFFFFFFF;
    recv_data[3] = 32'h00000000;
    recv_data[4] = 32'h12345678;
    comparator_mode = 2'b10; #10;
    #10;
    comparator_mode = 2'b11; #10;
    #10;
    $display("==== TEST 2 ? ERRORS IN LANE 0,3,4 ====");
    $display("per_lane_error = %b", per_lane_error);

    // ==========================
    // Aggregate comparison tests
    // ==========================
    comp_type = 0;           

    // Reset patterns to no error
    for(i=0;i<NUM_LANE;i=i+1) recv_data[i] = gen_data[i];
    comparator_mode = 2'b10; #10;  #10;
    comparator_mode = 2'b11; #10;  #10;
    $display("==== TEST 3 ? NO AGGREGATE ERROR ====");
    $display("module_error = %b, aggregate_exceeded = %b", module_error, aggregate_exceeded);

    // Clear once before threshold test
    comparator_mode = 2'b01; // CLEAR_LFSR
    @(posedge clk);
    
    // Introduce randomized errors to trigger module_error and aggregate_exceeded
    for(frame=0; frame<5; frame=frame+1) begin
      for(lane=0; lane<NUM_LANE; lane=lane+1) begin
        if ($urandom % 2) 
          recv_data[lane] = gen_data[lane] ^ $urandom;
        else
          recv_data[lane] = gen_data[lane];
      end
      comparator_mode = 2'b10; #10; #10;
      comparator_mode = 2'b11; #10; #10;
      $display("Frame %0d: module_error=%b, aggregate_exceeded=%b", frame, module_error, aggregate_exceeded);
    end

    $display("Simulation finished!");
    $stop;
  end

endmodule