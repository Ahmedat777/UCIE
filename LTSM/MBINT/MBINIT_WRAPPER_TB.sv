`timescale 1ns/1ps

import pckg::*;
module MBINIT_WRAPPER_TB;

////////////////////////////////////////////////////////////
// Parameters
////////////////////////////////////////////////////////////
parameter CLK_PERIOD   = 10;

////////////////////////////////////////////////////////////
// Clock & Reset (SHARED)
////////////////////////////////////////////////////////////
reg i_clk;
reg i_rst_n;

////////////////////////////////////////////////////////////
// Test Summary Counters
////////////////////////////////////////////////////////////
integer total_tests;
integer passed_tests;
integer failed_tests;

// State test tracking (each state has a pass/fail flag)
// Index mapping:
//  0  = PARAM.config_req
//  1  = PARAM.config_resp
//  2  = CAL.done_req
//  3  = CAL.done_resp
//  4  = REPAIRCLK.init_req
//  5  = REPAIRCLK.init_resp
//  6  = REPAIRCLK.result_req
//  7  = REPAIRCLK.result_resp
//  8  = REPAIRCLK.done_req
//  9  = REPAIRCLK.done_resp
//  10 = REPAIRVAL.init_req
//  11 = REPAIRVAL.init_resp
//  12 = REPAIRVAL.result_req
//  13 = REPAIRVAL.result_resp
//  14 = REPAIRVAL.done_req
//  15 = REPAIRVAL.done_resp
//  16 = REVERSALMB.init_req
//  17 = REVERSALMB.init_resp
//  18 = REVERSALMB.clear_error_req
//  19 = REVERSALMB.clear_error_resp
//  20 = REVERSALMB.result_req
//  21 = REVERSALMB.result_resp
//  22 = REVERSALMB.done_req
//  23 = REVERSALMB.done_resp
//  24 = REPAIRMB.start_req
//  25 = REPAIRMB.start_resp
//  26 = REPAIRMB.apply_degrade_req
//  27 = REPAIRMB.apply_degrade_resp
//  28 = REPAIRMB.end_req
//  29 = REPAIRMB.end_resp

localparam NUM_STATES = 30;
reg [NUM_STATES-1:0] state_pass_A;   // per-state pass flag for Instance A
reg [NUM_STATES-1:0] state_pass_B;   // per-state pass flag for Instance B
reg [NUM_STATES-1:0] state_tested;   // which states were exercised

reg [8*40-1:0] state_names [0:NUM_STATES-1]; // 40-char names (packed)

////////////////////////////////////////////////////////////
// ==================== INSTANCE A ========================
////////////////////////////////////////////////////////////

// Inputs
reg                          i_MBINIT_en_A;
reg                          i_sb_busy_A;
reg                          i_falling_edge_busy_A;
sb_msg_id                    i_decoded_sb_msg_A;
reg                          i_sb_valid_A;
reg  [2:0]                   i_msg_info_A;
reg  [15:0]                  i_data_bus_A;

reg  [3:0]                   i_rf_data_rate_A;
reg  [4:0]                   i_rf_vswing_A;
reg                          i_rf_clk_mode_A;
reg                          i_rf_clk_phase_A;
reg  [1:0]                   i_rf_module_id_A;
reg                          i_rf_ucie_sx8_A;
reg                          i_rf_sfes_A;
reg                          i_rf_tarr_A;

reg                          i_clk_ptrn_done_A;
reg  [2:0]                   i_logged_results_COMP_A;

reg                          i_VAL_Pattern_done_A;
reg                          i_VAL_Result_logged_COMB_A;

reg                          i_REVERSAL_done_A;
reg                          i_LaneID_Pattern_done_A;
reg  [15:0]                  i_REVERSAL_Result_logged_A;

reg                          i_d2c_tx_ack_A;
reg  [15:0]                  i_lanes_results_tx_A;

// Outputs
wire sb_msg_id               o_encoded_SB_msg_A;
wire                         o_msg_valid_A;
wire [15:0]                  o_data_bus_A;
wire [2:0]                   o_msg_info_A;
wire                         o_MBINIT_END_A;
wire                         o_error_req_A;
wire                         o_valid_datafield_A;
wire                         o_clk_ptrn_en_A;      

////////////////////////////////////////////////////////////
// ==================== INSTANCE B ========================
////////////////////////////////////////////////////////////

// Inputs
reg                          i_MBINIT_en_B;
reg                          i_sb_busy_B;
reg                          i_falling_edge_busy_B;
sb_msg_id                    i_decoded_sb_msg_B;
reg                          i_sb_valid_B;
reg  [2:0]                   i_msg_info_B;
reg  [15:0]                  i_data_bus_B;

reg  [3:0]                   i_rf_data_rate_B;
reg  [4:0]                   i_rf_vswing_B;
reg                          i_rf_clk_mode_B;
reg                          i_rf_clk_phase_B;
reg  [1:0]                   i_rf_module_id_B;
reg                          i_rf_ucie_sx8_B;
reg                          i_rf_sfes_B;
reg                          i_rf_tarr_B;

reg                          i_clk_ptrn_done_B;
reg  [2:0]                   i_logged_results_COMP_B;

reg                          i_VAL_Pattern_done_B;
reg                          i_VAL_Result_logged_COMB_B;

reg                          i_REVERSAL_done_B;
reg                          i_LaneID_Pattern_done_B;
reg  [15:0]                  i_REVERSAL_Result_logged_B;

reg                          i_d2c_tx_ack_B;
reg  [15:0]                  i_lanes_results_tx_B;

// Outputs
wire sb_msg_id               o_encoded_SB_msg_B;
wire                         o_msg_valid_B;
wire [15:0]                  o_data_bus_B;
wire [2:0]                   o_msg_info_B;
wire                         o_MBINIT_END_B;
wire                         o_error_req_B;
wire                         o_valid_datafield_B;
wire                         o_lane_A;
wire                         o_lane_B;
wire                         o_clk_ptrn_en_B; 

////////////////////////////////////////////////////////////
// Clock Generation
////////////////////////////////////////////////////////////
initial begin
    i_clk = 0;
    forever #(CLK_PERIOD/2) i_clk = ~i_clk;
end

////////////////////////////////////////////////////////////
// Default Initialization
////////////////////////////////////////////////////////////
initial begin
    total_tests  = 0;
    passed_tests = 0;
    failed_tests = 0;
    state_pass_A  = 0;
    state_pass_B  = 0;
    state_tested  = 0;

    // Instance A defaults
    i_MBINIT_en_A = 0;
    i_sb_busy_A = 0;
    i_falling_edge_busy_A = 0;
    i_decoded_sb_msg_A = SB_MBINIT_CAL_DONE_REQ;
    i_sb_valid_A = 0;
    i_msg_info_A = 0;
    i_data_bus_A = 0;

    i_rf_data_rate_A = 8;
    i_rf_vswing_A = 0;
    i_rf_clk_mode_A = 0;
    i_rf_clk_phase_A = 0;
    i_rf_module_id_A = 0;
    i_rf_ucie_sx8_A = 0;
    i_rf_sfes_A = 0;
    i_rf_tarr_A = 0;

    i_clk_ptrn_done_A = 0;
    i_logged_results_COMP_A = 0;

    i_VAL_Pattern_done_A = 0;
    i_VAL_Result_logged_COMB_A = 0;

    i_REVERSAL_done_A = 0;
    i_LaneID_Pattern_done_A = 0;
    i_REVERSAL_Result_logged_A = 0;

    i_d2c_tx_ack_A = 0;
    i_lanes_results_tx_A = 0;

    // Instance B defaults
    i_MBINIT_en_B = 0;
    i_sb_busy_B = 0;
    i_falling_edge_busy_B = 0;
    i_decoded_sb_msg_B = SB_MBINIT_CAL_DONE_REQ;
    i_sb_valid_B = 0;
    i_msg_info_B = 0;
    i_data_bus_B = 0;

    i_rf_data_rate_B = 10;
    i_rf_vswing_B = 0;
    i_rf_clk_mode_B = 0;
    i_rf_clk_phase_B = 0;
    i_rf_module_id_B = 0;
    i_rf_ucie_sx8_B = 0;
    i_rf_sfes_B = 0;
    i_rf_tarr_B = 0;

    i_clk_ptrn_done_B = 0;
    i_logged_results_COMP_B = 0;

    i_VAL_Pattern_done_B = 0;
    i_VAL_Result_logged_COMB_B = 0;

    i_REVERSAL_done_B = 0;
    i_LaneID_Pattern_done_B = 0;
    i_REVERSAL_Result_logged_B = 0;

    i_d2c_tx_ack_B = 0;
    i_lanes_results_tx_B = 0;
end

////////////////////////////////////////////////////////////
// DUT Instantiation A
////////////////////////////////////////////////////////////
MBINIT_WRAPPER DUT_A (
    .i_clk(i_clk),
    .i_rst_n(i_rst_n),
    .i_MBINIT_en(i_MBINIT_en_A),
    .i_sb_busy(i_sb_busy_A),
    .i_falling_edge_busy(i_falling_edge_busy_A),
    .i_decoded_sb_msg(o_encoded_SB_msg_B),
    .i_sb_valid(o_msg_valid_B),
    .i_msg_info(o_msg_info_B),
    .i_data_bus(o_data_bus_B),

    .i_rf_data_rate(i_rf_data_rate_A),
    .i_rf_vswing(i_rf_vswing_A),
    .i_rf_clk_mode(i_rf_clk_mode_A),
    .i_rf_clk_phase(i_rf_clk_phase_A),
    .i_rf_module_id(i_rf_module_id_A),
    .i_rf_ucie_sx8(i_rf_ucie_sx8_A),
    .i_rf_sfes(i_rf_sfes_A),
    .i_rf_tarr(i_rf_tarr_A),

    .i_clk_ptrn_done(i_clk_ptrn_done_A),
    .i_logged_results_COMP(i_logged_results_COMP_A),

    .i_VAL_Pattern_done(i_VAL_Pattern_done_A),
    .i_VAL_Result_logged_COMB(i_VAL_Result_logged_COMB_A),

    .i_REVERSAL_done(i_REVERSAL_done_A),
    .i_LaneID_Pattern_done(i_LaneID_Pattern_done_A),
    .i_REVERSAL_Result_logged(i_REVERSAL_Result_logged_A),

    .i_d2c_tx_ack(i_d2c_tx_ack_A),
    .i_lanes_results_tx(i_lanes_results_tx_A),

    .o_encoded_SB_msg(o_encoded_SB_msg_A),
    .o_msg_valid(o_msg_valid_A),
    .o_data_bus(o_data_bus_A),
    .o_msg_info(o_msg_info_A),
    .o_MBINIT_END(o_MBINIT_END_A),
    .o_error_req(o_error_req_A),
    .o_valid_datafield(o_valid_datafield_A),
    .o_clk_ptrn_en(o_clk_ptrn_en_A)
);

////////////////////////////////////////////////////////////
// DUT Instantiation B
////////////////////////////////////////////////////////////
MBINIT_WRAPPER DUT_B (
    .i_clk(i_clk),
    .i_rst_n(i_rst_n),
    .i_MBINIT_en(i_MBINIT_en_B),
    .i_sb_busy(i_sb_busy_B),
    .i_falling_edge_busy(i_falling_edge_busy_B),
    .i_decoded_sb_msg(o_encoded_SB_msg_A),
    .i_sb_valid(o_msg_valid_A),
    .i_msg_info(o_msg_info_A),
    .i_data_bus(o_data_bus_A),

    .i_rf_data_rate(i_rf_data_rate_B),
    .i_rf_vswing(i_rf_vswing_B),
    .i_rf_clk_mode(i_rf_clk_mode_B),
    .i_rf_clk_phase(i_rf_clk_phase_B),
    .i_rf_module_id(i_rf_module_id_B),
    .i_rf_ucie_sx8(i_rf_ucie_sx8_B),
    .i_rf_sfes(i_rf_sfes_B),
    .i_rf_tarr(i_rf_tarr_B),

    .i_clk_ptrn_done(i_clk_ptrn_done_B),
    .i_logged_results_COMP(i_logged_results_COMP_B),

    .i_VAL_Pattern_done(i_VAL_Pattern_done_B),
    .i_VAL_Result_logged_COMB(i_VAL_Result_logged_COMB_B),

    .i_REVERSAL_done(i_REVERSAL_done_B),
    .i_LaneID_Pattern_done(i_LaneID_Pattern_done_B),
    .i_REVERSAL_Result_logged(i_REVERSAL_Result_logged_B),

    .i_d2c_tx_ack(i_d2c_tx_ack_B),
    .i_lanes_results_tx(i_lanes_results_tx_B),

    .o_encoded_SB_msg(o_encoded_SB_msg_B),
    .o_msg_valid(o_msg_valid_B),
    .o_data_bus(o_data_bus_B),
    .o_msg_info(o_msg_info_B),
    .o_MBINIT_END(o_MBINIT_END_B),
    .o_error_req(o_error_req_B),
    .o_valid_datafield(o_valid_datafield_B),
    .o_clk_ptrn_en(o_clk_ptrn_en_B)
);

////////////////////////////////////////////////////////////
// NEGATIVE EDGE DETECTORS INSTANCES
////////////////////////////////////////////////////////////
negedge_detector negedge_det_A (
    .clk   (i_clk),
    .rst_n (i_rst_n),
    .i_busy(i_sb_busy_A),
    .o_falling_edge_busy(i_falling_edge_busy_A)
);

negedge_detector negedge_det_B (
    .clk   (i_clk),
    .rst_n (i_rst_n),
    .i_busy(i_sb_busy_B),
    .o_falling_edge_busy(i_falling_edge_busy_B)
);

////////////////////////////////////////////////////////////
// REPAIRCLK GENERATOR
////////////////////////////////////////////////////////////
REPAIRCLK_PTRN_GEN u_ptrn_gen_A (
    .i_clk   (i_clk),
    .i_rst_n (i_rst_n),
    .i_en    (o_clk_ptrn_en_A),
    .o_lane  (o_lane_A),
    .o_done  (i_clk_ptrn_done_A)
);

REPAIRCLK_PTRN_GEN u_ptrn_gen_B (
    .i_clk   (i_clk),
    .i_rst_n (i_rst_n),
    .i_en    (o_clk_ptrn_en_B),
    .o_lane  (o_lane_B),
    .o_done  (i_clk_ptrn_done_B)
);

///////////////////////////////////////////////////////////
// TASKS
////////////////////////////////////////////////////////////

// ---- drive_busy ----
task automatic drive_busy_A(input int cycles = 2);
begin
    i_sb_busy_A = 1'b1;
    repeat (cycles) @(posedge i_clk);
    i_sb_busy_A = 1'b0;
    @(posedge i_clk);
end
endtask

task automatic drive_busy_B(input int cycles = 2);
begin
    i_sb_busy_B = 1'b1;
    repeat (cycles) @(posedge i_clk);
    i_sb_busy_B = 1'b0;
    @(posedge i_clk);
end
endtask

// ---- check_output_A ----
// state_idx  : index into the state tracking arrays
// state_name : human-readable label printed in the log
task automatic check_output_A(
    input sb_msg_id          expected_msg,
    input                    expected_valid,
    input integer            state_idx,
    input [8*60-1:0]         state_name
);
    reg test_ok;
begin
    test_ok = 1;
    $display("\n[A][OUTPUT CHECK] State: %0s | Expect: VALID=%0b MSG=0x%0h  @%0t",
             state_name, expected_valid, expected_msg, $time);

    @(negedge i_clk);

    total_tests = total_tests + 1;
    state_tested[state_idx] = 1;

    if (o_msg_valid_A !== expected_valid) begin
        $error("[A][OUTPUT][FAIL] State: %0s | VALID mismatch: expected=%0b got=%0b @%0t",
               state_name, expected_valid, o_msg_valid_A, $time);
        test_ok = 0;
    end

    if (expected_valid && (o_encoded_SB_msg_A !== expected_msg)) begin
        $error("[A][OUTPUT][FAIL] State: %0s | MSG mismatch: expected=0x%0h got=0x%0h @%0t",
               state_name, expected_msg, o_encoded_SB_msg_A, $time);
        test_ok = 0;
    end

    if (test_ok) begin
        $display("[A][OUTPUT][PASS] State: %0s | All checks passed @%0t", state_name, $time);
        passed_tests = passed_tests + 1;
        state_pass_A[state_idx] = 1;
    end else begin
        failed_tests = failed_tests + 1;
        state_pass_A[state_idx] = 0;
    end
end
endtask

// ---- check_output_B ----
task automatic check_output_B(
    input sb_msg_id          expected_msg,
    input                    expected_valid,
    input integer            state_idx,
    input [8*60-1:0]         state_name
);
    reg test_ok;
begin
    test_ok = 1;
    $display("\n[B][OUTPUT CHECK] State: %0s | Expect: VALID=%0b MSG=0x%0h  @%0t",
             state_name, expected_valid, expected_msg, $time);

    @(negedge i_clk);

    total_tests = total_tests + 1;
    state_tested[state_idx] = 1;

    if (o_msg_valid_B !== expected_valid) begin
        $error("[B][OUTPUT][FAIL] State: %0s | VALID mismatch: expected=%0b got=%0b @%0t",
               state_name, expected_valid, o_msg_valid_B, $time);
        test_ok = 0;
    end

    if (expected_valid && (o_encoded_SB_msg_B !== expected_msg)) begin
        $error("[B][OUTPUT][FAIL] State: %0s | MSG mismatch: expected=0x%0h got=0x%0h @%0t",
               state_name, expected_msg, o_encoded_SB_msg_B, $time);
        test_ok = 0;
    end

    if (test_ok) begin
        $display("[B][OUTPUT][PASS] State: %0s | All checks passed @%0t", state_name, $time);
        passed_tests = passed_tests + 1;
        state_pass_B[state_idx] = 1;
    end else begin
        failed_tests = failed_tests + 1;
        state_pass_B[state_idx] = 0;
    end
end
endtask

// ---- print_summary ----
task automatic print_summary;
    integer i;
begin
    $display("\n");
    $display("╔══════════════════════════════════════════════════════════════════════════╗");
    $display("║                         MBINIT TESTBENCH SUMMARY                        ║");
    $display("╠══════════════════════════════════════════════════════════════════════════╣");
    $display("║  Total Tests   : %-4d                                                   ║", total_tests);
    $display("║  Passed        : %-4d                                                   ║", passed_tests);
    $display("║  Failed        : %-4d                                                   ║", failed_tests);
    $display("╠══════════════════════════════════════════════════════════════════════════╣");
    $display("║  State-by-State Results (A = Instance A, B = Instance B)                ║");
    $display("╠═══════════════════════════════════════════════╦══════════╦══════════════╣");
    $display("║  State Name                                   ║  Inst A  ║  Inst B      ║");
    $display("╠═══════════════════════════════════════════════╬══════════╬══════════════╣");

    //  0
    $display("║  PARAM     | configuration_req                ║  %-8s║  %-12s║",
        state_tested[0] ? (state_pass_A[0] ? "PASS" : "FAIL") : "---",
        state_tested[0] ? (state_pass_B[0] ? "PASS" : "FAIL") : "---");
    //  1
    $display("║  PARAM     | configuration_resp               ║  %-8s║  %-12s║",
        state_tested[1] ? (state_pass_A[1] ? "PASS" : "FAIL") : "---",
        state_tested[1] ? (state_pass_B[1] ? "PASS" : "FAIL") : "---");
    //  2
    $display("║  CAL       | done_req                         ║  %-8s║  %-12s║",
        state_tested[2] ? (state_pass_A[2] ? "PASS" : "FAIL") : "---",
        state_tested[2] ? (state_pass_B[2] ? "PASS" : "FAIL") : "---");
    //  3
    $display("║  CAL       | done_resp                        ║  %-8s║  %-12s║",
        state_tested[3] ? (state_pass_A[3] ? "PASS" : "FAIL") : "---",
        state_tested[3] ? (state_pass_B[3] ? "PASS" : "FAIL") : "---");
    //  4
    $display("║  REPAIRCLK | init_req                         ║  %-8s║  %-12s║",
        state_tested[4] ? (state_pass_A[4] ? "PASS" : "FAIL") : "---",
        state_tested[4] ? (state_pass_B[4] ? "PASS" : "FAIL") : "---");
    //  5
    $display("║  REPAIRCLK | init_resp                        ║  %-8s║  %-12s║",
        state_tested[5] ? (state_pass_A[5] ? "PASS" : "FAIL") : "---",
        state_tested[5] ? (state_pass_B[5] ? "PASS" : "FAIL") : "---");
    //  6
    $display("║  REPAIRCLK | result_req                       ║  %-8s║  %-12s║",
        state_tested[6] ? (state_pass_A[6] ? "PASS" : "FAIL") : "---",
        state_tested[6] ? (state_pass_B[6] ? "PASS" : "FAIL") : "---");
    //  7
    $display("║  REPAIRCLK | result_resp                      ║  %-8s║  %-12s║",
        state_tested[7] ? (state_pass_A[7] ? "PASS" : "FAIL") : "---",
        state_tested[7] ? (state_pass_B[7] ? "PASS" : "FAIL") : "---");
    //  8
    $display("║  REPAIRCLK | done_req                         ║  %-8s║  %-12s║",
        state_tested[8] ? (state_pass_A[8] ? "PASS" : "FAIL") : "---",
        state_tested[8] ? (state_pass_B[8] ? "PASS" : "FAIL") : "---");
    //  9
    $display("║  REPAIRCLK | done_resp                        ║  %-8s║  %-12s║",
        state_tested[9] ? (state_pass_A[9] ? "PASS" : "FAIL") : "---",
        state_tested[9] ? (state_pass_B[9] ? "PASS" : "FAIL") : "---");
    // 10
    $display("║  REPAIRVAL | init_req                         ║  %-8s║  %-12s║",
        state_tested[10] ? (state_pass_A[10] ? "PASS" : "FAIL") : "---",
        state_tested[10] ? (state_pass_B[10] ? "PASS" : "FAIL") : "---");
    // 11
    $display("║  REPAIRVAL | init_resp                        ║  %-8s║  %-12s║",
        state_tested[11] ? (state_pass_A[11] ? "PASS" : "FAIL") : "---",
        state_tested[11] ? (state_pass_B[11] ? "PASS" : "FAIL") : "---");
    // 12
    $display("║  REPAIRVAL | result_req                       ║  %-8s║  %-12s║",
        state_tested[12] ? (state_pass_A[12] ? "PASS" : "FAIL") : "---",
        state_tested[12] ? (state_pass_B[12] ? "PASS" : "FAIL") : "---");
    // 13
    $display("║  REPAIRVAL | result_resp                      ║  %-8s║  %-12s║",
        state_tested[13] ? (state_pass_A[13] ? "PASS" : "FAIL") : "---",
        state_tested[13] ? (state_pass_B[13] ? "PASS" : "FAIL") : "---");
    // 14
    $display("║  REPAIRVAL | done_req                         ║  %-8s║  %-12s║",
        state_tested[14] ? (state_pass_A[14] ? "PASS" : "FAIL") : "---",
        state_tested[14] ? (state_pass_B[14] ? "PASS" : "FAIL") : "---");
    // 15
    $display("║  REPAIRVAL | done_resp                        ║  %-8s║  %-12s║",
        state_tested[15] ? (state_pass_A[15] ? "PASS" : "FAIL") : "---",
        state_tested[15] ? (state_pass_B[15] ? "PASS" : "FAIL") : "---");
    // 16
    $display("║  REVERSALMB| init_req                         ║  %-8s║  %-12s║",
        state_tested[16] ? (state_pass_A[16] ? "PASS" : "FAIL") : "---",
        state_tested[16] ? (state_pass_B[16] ? "PASS" : "FAIL") : "---");
    // 17
    $display("║  REVERSALMB| init_resp                        ║  %-8s║  %-12s║",
        state_tested[17] ? (state_pass_A[17] ? "PASS" : "FAIL") : "---",
        state_tested[17] ? (state_pass_B[17] ? "PASS" : "FAIL") : "---");
    // 18
    $display("║  REVERSALMB| clear_error_req                  ║  %-8s║  %-12s║",
        state_tested[18] ? (state_pass_A[18] ? "PASS" : "FAIL") : "---",
        state_tested[18] ? (state_pass_B[18] ? "PASS" : "FAIL") : "---");
    // 19
    $display("║  REVERSALMB| clear_error_resp                 ║  %-8s║  %-12s║",
        state_tested[19] ? (state_pass_A[19] ? "PASS" : "FAIL") : "---",
        state_tested[19] ? (state_pass_B[19] ? "PASS" : "FAIL") : "---");
    // 20
    $display("║  REVERSALMB| result_req                       ║  %-8s║  %-12s║",
        state_tested[20] ? (state_pass_A[20] ? "PASS" : "FAIL") : "---",
        state_tested[20] ? (state_pass_B[20] ? "PASS" : "FAIL") : "---");
    // 21
    $display("║  REVERSALMB| result_resp                      ║  %-8s║  %-12s║",
        state_tested[21] ? (state_pass_A[21] ? "PASS" : "FAIL") : "---",
        state_tested[21] ? (state_pass_B[21] ? "PASS" : "FAIL") : "---");
    // 22
    $display("║  REVERSALMB| done_req                         ║  %-8s║  %-12s║",
        state_tested[22] ? (state_pass_A[22] ? "PASS" : "FAIL") : "---",
        state_tested[22] ? (state_pass_B[22] ? "PASS" : "FAIL") : "---");
    // 23
    $display("║  REVERSALMB| done_resp                        ║  %-8s║  %-12s║",
        state_tested[23] ? (state_pass_A[23] ? "PASS" : "FAIL") : "---",
        state_tested[23] ? (state_pass_B[23] ? "PASS" : "FAIL") : "---");
    // 24
    $display("║  REPAIRMB  | start_req                        ║  %-8s║  %-12s║",
        state_tested[24] ? (state_pass_A[24] ? "PASS" : "FAIL") : "---",
        state_tested[24] ? (state_pass_B[24] ? "PASS" : "FAIL") : "---");
    // 25
    $display("║  REPAIRMB  | start_resp                       ║  %-8s║  %-12s║",
        state_tested[25] ? (state_pass_A[25] ? "PASS" : "FAIL") : "---",
        state_tested[25] ? (state_pass_B[25] ? "PASS" : "FAIL") : "---");
    // 26
    $display("║  REPAIRMB  | apply_degrade_req                ║  %-8s║  %-12s║",
        state_tested[26] ? (state_pass_A[26] ? "PASS" : "FAIL") : "---",
        state_tested[26] ? (state_pass_B[26] ? "PASS" : "FAIL") : "---");
    // 27
    $display("║  REPAIRMB  | apply_degrade_resp               ║  %-8s║  %-12s║",
        state_tested[27] ? (state_pass_A[27] ? "PASS" : "FAIL") : "---",
        state_tested[27] ? (state_pass_B[27] ? "PASS" : "FAIL") : "---");
    // 28
    $display("║  REPAIRMB  | end_req                          ║  %-8s║  %-12s║",
        state_tested[28] ? (state_pass_A[28] ? "PASS" : "FAIL") : "---",
        state_tested[28] ? (state_pass_B[28] ? "PASS" : "FAIL") : "---");
    // 29
    $display("║  REPAIRMB  | end_resp                         ║  %-8s║  %-12s║",
        state_tested[29] ? (state_pass_A[29] ? "PASS" : "FAIL") : "---",
        state_tested[29] ? (state_pass_B[29] ? "PASS" : "FAIL") : "---");

    $display("╚═══════════════════════════════════════════════╩══════════╩══════════════╝");

    if (failed_tests == 0)
        $display("\n  *** ALL TESTS PASSED *** \n");
    else
        $display("\n  *** %0d TEST(S) FAILED — REVIEW ERRORS ABOVE *** \n", failed_tests);
end
endtask

///////////////////////////////////////////////////////////
// TEST SEQUENCE
////////////////////////////////////////////////////////////
initial begin
    i_rst_n = 1'b0;
    repeat (2) @(posedge i_clk);
    i_rst_n = 1'b1;
    i_MBINIT_en_A = 1;
    i_MBINIT_en_B = 1;

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//      MBINIT.PARAM TEST
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    $display("\n");
    $display("╔══════════════════════════════════════════════════════════════════════════╗");
    $display("║                     STARTING MBINIT.PARAM TEST                          ║");
    $display("╚══════════════════════════════════════════════════════════════════════════╝");

    @(posedge i_clk);
    fork
        begin drive_busy_A(1); end
        begin drive_busy_B(1); end
        begin check_output_A(SB_MBINIT_PARAM_CONFIG_REQ,  1, 0, "PARAM | configuration_req "); end
        begin check_output_B(SB_MBINIT_PARAM_CONFIG_REQ,  1, 0, "PARAM | configuration_req "); end
    join

    repeat (1) @(posedge i_clk);
    wait(o_msg_valid_B || o_msg_valid_A);

    fork
        begin drive_busy_A(1); end
        begin drive_busy_B(1); end
        begin check_output_A(SB_MBINIT_PARAM_CONFIG_RESP, 1, 1, "PARAM | configuration_resp"); end
        begin check_output_B(SB_MBINIT_PARAM_CONFIG_RESP, 1, 1, "PARAM | configuration_resp"); end
    join

    wait (DUT_A.dut_param.o_PARAM_END == 1 && DUT_B.dut_param.o_PARAM_END == 1);
    $display("\n  [INFO] MBINIT.PARAM completed successfully.\n");

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//      MBINIT.CAL TEST
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    $display("\n");
    $display("╔══════════════════════════════════════════════════════════════════════════╗");
    $display("║                      STARTING MBINIT.CAL TEST                           ║");
    $display("╚══════════════════════════════════════════════════════════════════════════╝");

    @(posedge i_clk);
    fork
        begin drive_busy_A(1); end
        begin drive_busy_B(1); end
        begin check_output_A(SB_MBINIT_CAL_DONE_REQ,  1, 2, "CAL | done_req            "); end
        begin check_output_B(SB_MBINIT_CAL_DONE_REQ,  1, 2, "CAL | done_req            "); end
    join

    @(posedge i_clk); wait(o_msg_valid_B || o_msg_valid_A);
    fork
        begin drive_busy_A(1); end
        begin drive_busy_B(1); end
        begin check_output_A(SB_MBINIT_CAL_DONE_RESP, 1, 3, "CAL | done_resp           "); end
        begin check_output_B(SB_MBINIT_CAL_DONE_RESP, 1, 3, "CAL | done_resp           "); end
    join

    wait (DUT_A.dut_cal.o_MBINIT_CAL_end == 1 && DUT_B.dut_cal.o_MBINIT_CAL_end);
    $display("\n  [INFO] MBINIT.CAL completed successfully.\n");

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//      MBINIT.REPAIRCLK TEST
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    $display("\n");
    $display("╔══════════════════════════════════════════════════════════════════════════╗");
    $display("║                   STARTING MBINIT.REPAIRCLK TEST                        ║");
    $display("╚══════════════════════════════════════════════════════════════════════════╝");

    @(posedge i_clk);
    fork
        begin drive_busy_A(1); end
        begin drive_busy_B(1); end
        begin check_output_A(SB_MBINIT_REPAIRCLK_INIT_REQ,     1, 4, "REPAIRCLK | init_req      "); end
        begin check_output_B(SB_MBINIT_REPAIRCLK_INIT_REQ,     1, 4, "REPAIRCLK | init_req      "); end
    join

    repeat (1) @(posedge i_clk);
    wait(o_msg_valid_B || o_msg_valid_A);

    fork
        begin drive_busy_A(1); end
        begin drive_busy_B(1); end
        begin check_output_A(SB_MBINIT_REPAIRCLK_INIT_RESP,   1, 5, "REPAIRCLK | init_resp     "); end
        begin check_output_B(SB_MBINIT_REPAIRCLK_INIT_RESP,   1, 5, "REPAIRCLK | init_resp     "); end
    join

    wait (DUT_A.dut_repairclk.o_clk_ptrn_en == 1 && DUT_B.dut_repairclk.o_clk_ptrn_en == 1);
    $display("  [INFO] REPAIRCLK pattern generators enabled.");

    wait(i_clk_ptrn_done_A && i_clk_ptrn_done_B);
    i_logged_results_COMP_A = 3'b111;
    i_logged_results_COMP_B = 3'b111;
    $display("  [INFO] REPAIRCLK pattern generation complete. Results logged.");

    repeat(2) @(posedge i_clk);
    fork
        begin drive_busy_A(1); end
        begin drive_busy_B(1); end
        begin check_output_A(SB_MBINIT_REPAIRCLK_RESULT_REQ,  1, 6, "REPAIRCLK | result_req    "); end
        begin check_output_B(SB_MBINIT_REPAIRCLK_RESULT_REQ,  1, 6, "REPAIRCLK | result_req    "); end
    join

    repeat (1) @(posedge i_clk);
    wait(o_msg_valid_B || o_msg_valid_A);

    fork
        begin drive_busy_A(1); end
        begin drive_busy_B(1); end
        begin check_output_A(SB_MBINIT_REPAIRCLK_RESULT_RESP, 1, 7, "REPAIRCLK | result_resp   "); end
        begin check_output_B(SB_MBINIT_REPAIRCLK_RESULT_RESP, 1, 7, "REPAIRCLK | result_resp   "); end
    join

    @(posedge i_clk);
    fork
        begin drive_busy_A(1); end
        begin drive_busy_B(1); end
        begin check_output_A(SB_MBINIT_REPAIRCLK_DONE_REQ,    1, 8, "REPAIRCLK | done_req      "); end
        begin check_output_B(SB_MBINIT_REPAIRCLK_DONE_REQ,    1, 8, "REPAIRCLK | done_req      "); end
    join

    repeat (1) @(posedge i_clk);
    wait(o_msg_valid_B || o_msg_valid_A);

    fork
        begin drive_busy_A(1); end
        begin drive_busy_B(1); end
        begin check_output_A(SB_MBINIT_REPAIRCLK_DONE_RESP,   1, 9, "REPAIRCLK | done_resp     "); end
        begin check_output_B(SB_MBINIT_REPAIRCLK_DONE_RESP,   1, 9, "REPAIRCLK | done_resp     "); end
    join

    wait (DUT_A.dut_repairclk.o_MBINIT_REPAIRCLK_end == 1 && DUT_B.dut_repairclk.o_MBINIT_REPAIRCLK_end == 1);
    $display("\n  [INFO] MBINIT.REPAIRCLK completed successfully.\n");

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//      MBINIT.REPAIRVAL TEST
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    $display("\n");
    $display("╔══════════════════════════════════════════════════════════════════════════╗");
    $display("║                   STARTING MBINIT.REPAIRVAL TEST                        ║");
    $display("╚══════════════════════════════════════════════════════════════════════════╝");

    @(posedge i_clk);
    fork
        begin drive_busy_A(1); end
        begin drive_busy_B(1); end
        begin check_output_A(SB_MBINIT_REPAIRVAL_INIT_REQ,      1, 10, "REPAIRVAL | init_req      "); end
        begin check_output_B(SB_MBINIT_REPAIRVAL_INIT_REQ,      1, 10, "REPAIRVAL | init_req      "); end
    join

    repeat (1) @(posedge i_clk);
    wait(o_msg_valid_B || o_msg_valid_A);

    fork
        begin drive_busy_A(1); end
        begin drive_busy_B(1); end
        begin check_output_A(SB_MBINIT_REPAIRVAL_INIT_RESP,    1, 11, "REPAIRVAL | init_resp     "); end
        begin check_output_B(SB_MBINIT_REPAIRVAL_INIT_RESP,    1, 11, "REPAIRVAL | init_resp     "); end
    join

    wait (DUT_A.dut_repairval.o_MBINIT_REPAIRVAL_Pattern_En == 1 && DUT_B.dut_repairval.o_MBINIT_REPAIRVAL_Pattern_En == 1);
    $display("  [INFO] REPAIRVAL pattern generators enabled.");

    i_VAL_Pattern_done_A = 1;
    i_VAL_Pattern_done_B = 1;
    i_VAL_Result_logged_COMB_A = 1;
    i_VAL_Result_logged_COMB_B = 1;
    $display("  [INFO] REPAIRVAL pattern generation complete. Results logged.");

    repeat(2) @(posedge i_clk);
    fork
        begin drive_busy_A(1); end
        begin drive_busy_B(1); end
        begin check_output_A(SB_MBINIT_REPAIRVAL_RESULT_REQ,   1, 12, "REPAIRVAL | result_req    "); end
        begin check_output_B(SB_MBINIT_REPAIRVAL_RESULT_REQ,   1, 12, "REPAIRVAL | result_req    "); end
    join

    repeat (1) @(posedge i_clk);
    wait(o_msg_valid_B || o_msg_valid_A);

    fork
        begin drive_busy_A(1); end
        begin drive_busy_B(1); end
        begin check_output_A(SB_MBINIT_REPAIRVAL_RESULT_RESP,  1, 13, "REPAIRVAL | result_resp   "); end
        begin check_output_B(SB_MBINIT_REPAIRVAL_RESULT_RESP,  1, 13, "REPAIRVAL | result_resp   "); end
    join

    @(posedge i_clk);
    fork
        begin drive_busy_A(1); end
        begin drive_busy_B(1); end
        begin check_output_A(SB_MBINIT_REPAIRVAL_DONE_REQ,     1, 14, "REPAIRVAL | done_req      "); end
        begin check_output_B(SB_MBINIT_REPAIRVAL_DONE_REQ,     1, 14, "REPAIRVAL | done_req      "); end
    join

    repeat (1) @(posedge i_clk);
    wait(o_msg_valid_B || o_msg_valid_A);

    fork
        begin drive_busy_A(1); end
        begin drive_busy_B(1); end
        begin check_output_A(SB_MBINIT_REPAIRVAL_DONE_RESP,    1, 15, "REPAIRVAL | done_resp     "); end
        begin check_output_B(SB_MBINIT_REPAIRVAL_DONE_RESP,    1, 15, "REPAIRVAL | done_resp     "); end
    join

    wait (DUT_A.dut_repairval.o_MBINIT_REPAIRVAL_end == 1 && DUT_B.dut_repairval.o_MBINIT_REPAIRVAL_end == 1);
    $display("\n  [INFO] MBINIT.REPAIRVAL completed successfully.\n");

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//      MBINIT.REVERSALMB TEST
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    $display("\n");
    $display("╔══════════════════════════════════════════════════════════════════════════╗");
    $display("║                   STARTING MBINIT.REVERSALMB TEST                       ║");
    $display("╚══════════════════════════════════════════════════════════════════════════╝");

    @(posedge i_clk);
    fork
        begin drive_busy_A(1); end
        begin drive_busy_B(1); end
        begin check_output_A(SB_MBINIT_REVERSALMB_INIT_REQ,         1, 16, "REVERSALMB | init_req       "); end
        begin check_output_B(SB_MBINIT_REVERSALMB_INIT_REQ,         1, 16, "REVERSALMB | init_req       "); end
    join

    repeat (1) @(posedge i_clk);
    wait(o_msg_valid_B || o_msg_valid_A);

    fork
        begin drive_busy_A(1); end
        begin drive_busy_B(1); end
        begin check_output_A(SB_MBINIT_REVERSALMB_INIT_RESP,        1, 17, "REVERSALMB | init_resp      "); end
        begin check_output_B(SB_MBINIT_REVERSALMB_INIT_RESP,        1, 17, "REVERSALMB | init_resp      "); end
    join

    repeat (1) @(posedge i_clk);
    wait(o_msg_valid_B || o_msg_valid_A);

    fork
        begin drive_busy_A(1); end
        begin drive_busy_B(1); end
        begin check_output_A(SB_MBINIT_REVERSALMB_CLEAR_ERROR_REQ,  1, 18, "REVERSALMB | clear_error_req"); end
        begin check_output_B(SB_MBINIT_REVERSALMB_CLEAR_ERROR_REQ,  1, 18, "REVERSALMB | clear_error_req"); end
    join

    repeat (1) @(posedge i_clk);
    wait(o_msg_valid_B || o_msg_valid_A);

    fork
        begin drive_busy_A(1); end
        begin drive_busy_B(1); end
        begin check_output_A(SB_MBINIT_REVERSALMB_CLEAR_ERROR_RESP, 1, 19, "REVERSALMB | clear_error_resp"); end
        begin check_output_B(SB_MBINIT_REVERSALMB_CLEAR_ERROR_RESP, 1, 19, "REVERSALMB | clear_error_resp"); end
    join

    wait (DUT_A.dut_reversalmb.o_MBINIT_REVERSALMB_LaneID_Pattern_En == 3 && DUT_B.dut_reversalmb.o_MBINIT_REVERSALMB_LaneID_Pattern_En == 3);
    $display("  [INFO] REVERSALMB LaneID pattern generators enabled.");

    repeat(2) @(posedge i_clk);
    i_LaneID_Pattern_done_A      = 1;
    i_LaneID_Pattern_done_B      = 1;
    i_REVERSAL_Result_logged_A   = 'hffff;
    i_REVERSAL_Result_logged_B   = 'hffff;
    $display("  [INFO] REVERSALMB LaneID pattern generation complete. Results logged.");

    repeat (2) @(posedge i_clk);
    fork
        begin drive_busy_A(1); end
        begin drive_busy_B(1); end
        begin check_output_A(SB_MBINIT_REVERSALMB_RESULT_REQ,       1, 20, "REVERSALMB | result_req     "); end
        begin check_output_B(SB_MBINIT_REVERSALMB_RESULT_REQ,       1, 20, "REVERSALMB | result_req     "); end
    join

    repeat (1) @(posedge i_clk);
    wait(o_msg_valid_B || o_msg_valid_A);

    fork
        begin drive_busy_A(1); end
        begin drive_busy_B(1); end
        begin check_output_A(SB_MBINIT_REVERSALMB_RESULT_RESP,      1, 21, "REVERSALMB | result_resp    "); end
        begin check_output_B(SB_MBINIT_REVERSALMB_RESULT_RESP,      1, 21, "REVERSALMB | result_resp    "); end
    join

    @(posedge i_clk);
    wait(o_msg_valid_B || o_msg_valid_A);

    fork
        begin drive_busy_A(1); end
        begin drive_busy_B(1); end
        begin check_output_A(SB_MBINIT_REVERSALMB_DONE_REQ,         1, 22, "REVERSALMB | done_req       "); end
        begin check_output_B(SB_MBINIT_REVERSALMB_DONE_REQ,         1, 22, "REVERSALMB | done_req       "); end
    join

    repeat (1) @(posedge i_clk);
    wait(o_msg_valid_B || o_msg_valid_A);

    fork
        begin drive_busy_A(1); end
        begin drive_busy_B(1); end
        begin check_output_A(SB_MBINIT_REVERSALMB_DONE_RESP,        1, 23, "REVERSALMB | done_resp      "); end
        begin check_output_B(SB_MBINIT_REVERSALMB_DONE_RESP,        1, 23, "REVERSALMB | done_resp      "); end
    join

    wait (DUT_A.dut_reversalmb.o_MBINIT_REVERSALMB_end == 1 && DUT_B.dut_reversalmb.o_MBINIT_REVERSALMB_end == 1);
    $display("\n  [INFO] MBINIT.REVERSALMB completed successfully.\n");

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//      MBINIT.REPAIRMB TEST
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    $display("\n");
    $display("╔══════════════════════════════════════════════════════════════════════════╗");
    $display("║                   STARTING MBINIT.REPAIRMB TEST                         ║");
    $display("╚══════════════════════════════════════════════════════════════════════════╝");

    repeat (1) @(posedge i_clk);
    fork
        begin drive_busy_A(1); end
        begin drive_busy_B(1); end
        begin check_output_A(SB_MBINIT_REPAIRMB_START_REQ,         1, 24, "REPAIRMB | start_req        "); end
        begin check_output_B(SB_MBINIT_REPAIRMB_START_REQ,         1, 24, "REPAIRMB | start_req        "); end
    join

    repeat (1) @(posedge i_clk);
    fork
        begin drive_busy_A(1); end
        begin drive_busy_B(1); end
        begin check_output_A(SB_MBINIT_REPAIRMB_START_RESP,        1, 25, "REPAIRMB | start_resp       "); end
        begin check_output_B(SB_MBINIT_REPAIRMB_START_RESP,        1, 25, "REPAIRMB | start_resp       "); end
    join

    wait (DUT_A.dut_repairmb.o_Transmitter_initiated_D2C_en &&
          DUT_B.dut_repairmb.o_Transmitter_initiated_D2C_en &&
          DUT_A.dut_repairmb.o_perlane_Transmitter_initiated_D2C &&
          DUT_B.dut_repairmb.o_perlane_Transmitter_initiated_D2C);
    $display("  [INFO] REPAIRMB D2C transmitter enabled per lane.");

    repeat(1) @(posedge i_clk);
    i_d2c_tx_ack_A       = 1;
    i_d2c_tx_ack_B       = 1;
    i_lanes_results_tx_A = 'hffff;
    i_lanes_results_tx_B = 'hffff;
    $display("  [INFO] REPAIRMB D2C acknowledged. Lane results loaded.");

    wait(o_msg_valid_B || o_msg_valid_A);
    fork
        begin drive_busy_A(1); end
        begin drive_busy_B(1); end
        begin check_output_A(SB_MBINIT_REPAIRMB_APPLY_DEGRADE_REQ,  1, 26, "REPAIRMB | apply_degrade_req"); end
        begin check_output_B(SB_MBINIT_REPAIRMB_APPLY_DEGRADE_REQ,  1, 26, "REPAIRMB | apply_degrade_req"); end
    join

    repeat (1) @(posedge i_clk);
    wait(o_msg_valid_B || o_msg_valid_A);

    fork
        begin drive_busy_A(1); end
        begin drive_busy_B(1); end
        begin check_output_A(SB_MBINIT_REPAIRMB_APPLY_DEGRADE_RESP, 1, 27, "REPAIRMB | apply_degrade_resp"); end
        begin check_output_B(SB_MBINIT_REPAIRMB_APPLY_DEGRADE_RESP, 1, 27, "REPAIRMB | apply_degrade_resp"); end
    join

    wait(o_msg_valid_B || o_msg_valid_A);
    @(posedge i_clk);
    fork
        begin drive_busy_A(1); end
        begin drive_busy_B(1); end
        begin check_output_A(SB_MBINIT_REPAIRMB_END_REQ,            1, 28, "REPAIRMB | end_req          "); end
        begin check_output_B(SB_MBINIT_REPAIRMB_END_REQ,            1, 28, "REPAIRMB | end_req          "); end
    join

    wait(o_msg_valid_B || o_msg_valid_A);
    repeat (1) @(posedge i_clk);
    fork
        begin drive_busy_A(1); end
        begin drive_busy_B(1); end
        begin check_output_A(SB_MBINIT_REPAIRMB_END_RESP,           1, 29, "REPAIRMB | end_resp         "); end
        begin check_output_B(SB_MBINIT_REPAIRMB_END_RESP,           1, 29, "REPAIRMB | end_resp         "); end
    join

    wait (DUT_A.dut_repairmb.o_MBINIT_REPAIRMB_end == 1 && DUT_B.dut_repairmb.o_MBINIT_REPAIRMB_end == 1);
    $display("\n  [INFO] MBINIT.REPAIRMB completed successfully.");
    $display("  [INFO] Full MBINIT sequence completed.\n");

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//      PRINT TEST SUMMARY
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    repeat (2) @(posedge i_clk);
    print_summary;

    repeat (10) @(posedge i_clk);
    $stop;
end

endmodule