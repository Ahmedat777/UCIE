`timescale 1ns/1ps

module MBINIT_WRAPPER_TB;

////////////////////////////////////////////////////////////
// Parameters
////////////////////////////////////////////////////////////
parameter SB_MSG_Width = 4;
parameter CLK_PERIOD   = 10;

////////////////////////////////////////////////////////////
// Clock & Reset (SHARED)
////////////////////////////////////////////////////////////
reg i_clk;
reg i_rst_n;

////////////////////////////////////////////////////////////
// ==================== INSTANCE A ========================
////////////////////////////////////////////////////////////

// Inputs
reg                          i_MBINIT_en_A;
reg                          i_sb_busy_A;
reg                          i_falling_edge_busy_A;
reg  [SB_MSG_Width-1:0]      i_decoded_sb_msg_A;
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
wire [SB_MSG_Width-1:0]      o_encoded_SB_msg_A;
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
reg  [SB_MSG_Width-1:0]      i_decoded_sb_msg_B;
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
wire [SB_MSG_Width-1:0]      o_encoded_SB_msg_B;
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
    // Instance A defaults
    i_MBINIT_en_A = 0;
    i_sb_busy_A = 0;
    i_falling_edge_busy_A = 0;
    i_decoded_sb_msg_A = 0;
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
    i_decoded_sb_msg_B = 0;
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
// SIDEBAND SIGNAL"S ENCODING
////////////////////////////////////////////////////////////
localparam MBINIT_PARAM_configuration_req = 4'b0001;
localparam MBINIT_PARAM_configuration_resp = 4'b0010;

localparam  MSG_CAL_DONE_REQ  = 4'b0001;
localparam  MSG_CAL_DONE_RESP = 4'b0010;

localparam MBINI_REPAIRCLK_init_req     = 4'b0001;
localparam MBINIT_REPAIRCLK_init_resp   = 4'b0010;
localparam MBINIT_REPAIRCLK_result_req  = 4'b0011;
localparam MBINIT_REPAIRCLK_result_resp = 4'b0100;
localparam MBINIT_REPAIRCLK_done_req    = 4'b0101;
localparam MBINIT_REPAIRCLK_done_resp   = 4'b0110;

localparam MBINI_REPAIRVAL_init_req     = 4'b0001;
localparam MBINIT_REPAIRVAL_init_resp   = 4'b0010;
localparam MBINIT_REPAIRVAL_result_req  = 4'b0011;
localparam MBINIT_REPAIRVAL_result_resp = 4'b0100;
localparam MBINIT_REPAIRVAL_done_req    = 4'b0101;
localparam MBINIT_REPAIRVAL_done_resp   = 4'b0110;

localparam MBINIT_REVERSALMB_init_req           = 4'b0001;//1
localparam MBINIT_REVERSALMB_init_resp          = 4'b0010;//2
localparam MBINIT_REVERSALMB_clear_error_req    = 4'b0011;//3
localparam MBINIT_REVERSALMB_clear_error_resp   = 4'b0100;//4
localparam MBINIT_REVERSALMB_result_req         = 4'b0101;//5
localparam MBINIT_REVERSALMB_result_resp        = 4'b0110;//6
localparam MBINIT_REVERSALMB_done_req           = 4'b0111;//7
localparam MBINIT_REVERSALMB_done_resp          = 4'b1000;//8

localparam [3:0] MBINIT_REPAIRMB_start_req          = 4'b0001;//1
localparam [3:0] MBINIT_REPAIRMB_start_resp         = 4'b0010;//2
localparam [3:0] MBINIT_REPAIRMB_apply_degrade_req  = 4'b0101;//5
localparam [3:0] MBINIT_REPAIRMB_apply_degrade_resp = 4'b0110;//6
localparam [3:0] MBINIT_REPAIRMB_end_req            = 4'b0011;//3
localparam [3:0] MBINIT_REPAIRMB_end_resp           = 4'b0100;//4

////////////////////////////////////////////////////////////
// DUT Instantiation A
////////////////////////////////////////////////////////////
MBINIT_WRAPPER #(.SB_MSG_Width(SB_MSG_Width)) DUT_A (
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
MBINIT_WRAPPER #(.SB_MSG_Width(SB_MSG_Width)) DUT_B (
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
task automatic drive_busy_A(input int cycles = 2);
begin
    i_sb_busy_A = 1'b1;
    repeat (cycles) @(posedge i_clk);

    i_sb_busy_A = 1'b0;
    @(posedge i_clk); // allow falling edge detection
end
endtask

task automatic check_output_A(
    input [SB_MSG_Width-1:0] expected_msg,
    input                     expected_valid
);
begin
    // Declaration / intent
    $display("\n[A][OUTPUT CHECK] Expect: VALID=%0b, MSG=%0h @%0t",
              expected_valid, expected_msg, $time);

    @(negedge i_clk);

    // VALID check
    if (o_msg_valid_A !== expected_valid) begin
        $error("[A][OUTPUT][FAIL] VALID mismatch: expected=%0b, got=%0b @%0t",
               expected_valid, o_msg_valid_A, $time);
    end

    // MSG check (only if valid)
    if (expected_valid) begin
        if (o_encoded_SB_msg_A !== expected_msg) begin
            $error("[A][OUTPUT][FAIL] MSG mismatch: expected=%0h, got=%0h @%0t",
                   expected_msg, o_encoded_SB_msg_A, $time);
        end
        else begin
            $display("[A][OUTPUT][PASS] VALID & MSG matched @%0t", $time);
        end
    end
    else begin
        $display("[A][OUTPUT][PASS] VALID matched (MSG ignored) @%0t", $time);
    end
end
endtask

task automatic check_input_A(
    input [SB_MSG_Width-1:0] expected_msg,
    input                     expected_valid
);
begin
    $display("\n[A][INPUT CHECK] Expect: VALID=%0b, MSG=%0h @%0t",
              expected_valid, expected_msg, $time);

    @(negedge i_clk);

    if (i_sb_valid_A !== expected_valid) begin
        $error("[A][INPUT][FAIL] VALID mismatch: expected=%0b, got=%0b @%0t",
               expected_valid, i_sb_valid_A, $time);
    end

    if (expected_valid) begin
        if (i_decoded_sb_msg_A !== expected_msg) begin
            $error("[A][INPUT][FAIL] MSG mismatch: expected=%0h, got=%0h @%0t",
                   expected_msg, i_decoded_sb_msg_A, $time);
        end
        else begin
            $display("[A][INPUT][PASS] VALID & MSG matched @%0t", $time);
        end
    end
    else begin
        $display("[A][INPUT][PASS] VALID matched (MSG ignored) @%0t", $time);
    end
end
endtask











task automatic drive_busy_B(input int cycles = 2);
begin
    i_sb_busy_B = 1'b1;
    repeat (cycles) @(posedge i_clk);

    i_sb_busy_B = 1'b0;
    @(posedge i_clk); // allow falling edge detection
end
endtask

task automatic check_output_B(
    input [SB_MSG_Width-1:0] expected_msg,
    input                     expected_valid
);
begin
    $display("\n[B][OUTPUT CHECK] Expect: VALID=%0b, MSG=%0h @%0t",
              expected_valid, expected_msg, $time);

    @(negedge i_clk);

    if (o_msg_valid_B !== expected_valid) begin
        $error("[B][OUTPUT][FAIL] VALID mismatch: expected=%0b, got=%0b @%0t",
               expected_valid, o_msg_valid_B, $time);
    end

    if (expected_valid) begin
        if (o_encoded_SB_msg_B !== expected_msg) begin
            $error("[B][OUTPUT][FAIL] MSG mismatch: expected=%0h, got=%0h @%0t",
                   expected_msg, o_encoded_SB_msg_B, $time);
        end
        else begin
            $display("[B][OUTPUT][PASS] VALID & MSG matched @%0t", $time);
        end
    end
    else begin
        $display("[B][OUTPUT][PASS] VALID matched (MSG ignored) @%0t", $time);
    end
end
endtask

task automatic check_input_B(
    input [SB_MSG_Width-1:0] expected_msg,
    input                     expected_valid
);
begin
    $display("\n[B][INPUT CHECK] Expect: VALID=%0b, MSG=%0h @%0t",
              expected_valid, expected_msg, $time);

    @(negedge i_clk);

    if (i_sb_valid_B !== expected_valid) begin
        $error("[B][INPUT][FAIL] VALID mismatch: expected=%0b, got=%0b @%0t",
               expected_valid, i_sb_valid_B, $time);
    end

    if (expected_valid) begin
        if (i_decoded_sb_msg_B !== expected_msg) begin
            $error("[B][INPUT][FAIL] MSG mismatch: expected=%0h, got=%0h @%0t",
                   expected_msg, i_decoded_sb_msg_B, $time);
        end
        else begin
            $display("[B][INPUT][PASS] VALID & MSG matched @%0t", $time);
        end
    end
    else begin
        $display("[B][INPUT][PASS] VALID matched (MSG ignored) @%0t", $time);
    end
end
endtask

///////////////////////////////////////////////////////////
// TEST SEQUENCE
////////////////////////////////////////////////////////////
initial begin
    i_rst_n = 1'b0;                // assert reset
    repeat (2) @(posedge i_clk);   // hold for 2 cycles
    i_rst_n = 1'b1;                // release reset
    i_MBINIT_en_A = 1 ;
    i_MBINIT_en_B = 1 ;
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//      MBINIT.PARAM TEST
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////    
    @(posedge i_clk);
    fork
       begin
            drive_busy_A(1) ;
       end

       begin
            drive_busy_B(1) ; 
       end 

       begin
            check_output_A(MBINIT_PARAM_configuration_req , 1) ;
       end

       begin
            check_output_B(MBINIT_PARAM_configuration_req , 1) ; 
       end 

    join
    repeat (1) @(posedge i_clk);
    wait(o_msg_valid_B || o_msg_valid_A) ;

    fork
       begin
            drive_busy_A(1) ;
       end

       begin
            drive_busy_B(1) ; 
       end 

       begin
            check_output_A(MBINIT_PARAM_configuration_resp , 1) ;
       end

       begin
            check_output_B(MBINIT_PARAM_configuration_resp , 1) ; 
       end 

    join
wait (DUT_A.dut_param.o_PARAM_END == 1 && DUT_B.dut_param.o_PARAM_END == 1);
$display("MBINIT.PARAM is finished") ;  // CAL_STATE is initiated
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//      MBINIT.CAL TEST
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
@ (posedge i_clk) ;
fork
       begin
            drive_busy_A(1) ;
       end

       begin
            drive_busy_B(1) ; 
       end 

       begin
            check_output_A(MSG_CAL_DONE_REQ , 1) ;
       end

       begin
            check_output_B(MSG_CAL_DONE_REQ , 1) ; 
       end 

    join
@(posedge i_clk) wait(o_msg_valid_B || o_msg_valid_A) ;
    fork
       begin
            drive_busy_A(1) ;
       end

       begin
            drive_busy_B(1) ; 
       end 

       begin
            check_output_A(MSG_CAL_DONE_RESP , 1) ;
       end

       begin
            check_output_B(MSG_CAL_DONE_RESP , 1) ; 
       end 

    join

wait (DUT_A.dut_cal.o_MBINIT_CAL_end == 1 && DUT_B.dut_cal.o_MBINIT_CAL_end);
$display("MBINIT.CAL is finished") ;  // RE{AIRCLK_STATE is initiated
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//      MBINIT.REAPAIRCLK TEST
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
@(posedge i_clk);
    fork
       begin
            drive_busy_A(1) ;
       end

       begin
            drive_busy_B(1) ; 
       end 

       begin
            check_output_A(MBINI_REPAIRCLK_init_req , 1) ;
       end

       begin
            check_output_B(MBINI_REPAIRCLK_init_req , 1) ; 
       end 

    join
    repeat (1) @(posedge i_clk);
    wait(o_msg_valid_B || o_msg_valid_A) ;

    fork
       begin
            drive_busy_A(1) ;
       end

       begin
            drive_busy_B(1) ; 
       end 

       begin
            check_output_A(MBINIT_REPAIRCLK_init_resp , 1) ;
       end

       begin
            check_output_B(MBINIT_REPAIRCLK_init_resp , 1) ; 
       end 

    join
wait (DUT_A.dut_repairclk.o_clk_ptrn_en == 1 && DUT_B.dut_repairclk.o_clk_ptrn_en ==1);
$display("pattern generators are on") ;  // RE{AIRCLK_STATE is initiated
 wait(i_clk_ptrn_done_A && i_clk_ptrn_done_B) ;
 i_logged_results_COMP_A = 3'b111 ;
 i_logged_results_COMP_B = 3'b111 ;
$display("pattern generator is done") ;

repeat(2) @(posedge i_clk);
    fork
       begin
            drive_busy_A(1) ;
       end

       begin
            drive_busy_B(1) ; 
       end 

       begin
            check_output_A(MBINIT_REPAIRCLK_result_req , 1) ;
       end

       begin
            check_output_B(MBINIT_REPAIRCLK_result_req , 1) ; 
       end 

    join
    repeat (1) @(posedge i_clk);
    wait(o_msg_valid_B || o_msg_valid_A) ;

    fork
       begin
            drive_busy_A(1) ;
       end

       begin
            drive_busy_B(1) ; 
       end 

       begin
            check_output_A(MBINIT_REPAIRCLK_result_resp , 1) ;
       end

       begin
            check_output_B(MBINIT_REPAIRCLK_result_resp , 1) ; 
       end 

    join

    @(posedge i_clk);
    fork
       begin
            drive_busy_A(1) ;
       end

       begin
            drive_busy_B(1) ; 
       end 

       begin
            check_output_A(MBINIT_REPAIRCLK_done_req , 1) ;
       end

       begin
            check_output_B(MBINIT_REPAIRCLK_done_req , 1) ; 
       end 

    join
    repeat (1) @(posedge i_clk);
    wait(o_msg_valid_B || o_msg_valid_A) ;

    fork
       begin
            drive_busy_A(1) ;
       end

       begin
            drive_busy_B(1) ; 
       end 

       begin
            check_output_A(MBINIT_REPAIRCLK_done_resp , 1) ;
       end

       begin
            check_output_B(MBINIT_REPAIRCLK_done_resp , 1) ; 
       end 

    join

    wait (DUT_A.dut_repairclk.o_MBINIT_REPAIRCLK_end == 1 && DUT_B.dut_repairclk.o_MBINIT_REPAIRCLK_end ==1);
    $display("MBINIT.REPAIRCLK is finished");
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//      MBINIT.REPAIRVAL TEST
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    @(posedge i_clk);
    fork
       begin
            drive_busy_A(1) ;
       end

       begin
            drive_busy_B(1) ; 
       end 

       begin
            check_output_A(MBINI_REPAIRVAL_init_req  , 1) ;
       end

       begin
            check_output_B(MBINI_REPAIRVAL_init_req  , 1) ; 
       end 

    join
    repeat (1) @(posedge i_clk);
    wait(o_msg_valid_B || o_msg_valid_A) ;

    fork
       begin
            drive_busy_A(1) ;
       end

       begin
            drive_busy_B(1) ; 
       end 

       begin
            check_output_A(MBINIT_REPAIRVAL_init_resp , 1) ;
       end

       begin
            check_output_B(MBINIT_REPAIRVAL_init_resp , 1) ; 
       end 

    join
wait (DUT_A.dut_repairval.o_MBINIT_REPAIRVAL_Pattern_En == 1 && DUT_B.dut_repairval.o_MBINIT_REPAIRVAL_Pattern_En ==1);
$display("pattern generators are on") ;  
i_VAL_Pattern_done_A = 1 ;
i_VAL_Pattern_done_B = 1 ;
i_VAL_Result_logged_COMB_A = 1 ;
i_VAL_Result_logged_COMB_B = 1 ;
$display("pattern generator is done") ;

repeat(2) @(posedge i_clk);
    fork
       begin
            drive_busy_A(1) ;
       end

       begin
            drive_busy_B(1) ; 
       end 

       begin
            check_output_A(MBINIT_REPAIRVAL_result_req , 1) ;
       end

       begin
            check_output_B(MBINIT_REPAIRVAL_result_req , 1) ; 
       end 

    join
    repeat (1) @(posedge i_clk);
    wait(o_msg_valid_B || o_msg_valid_A) ;

    fork
       begin
            drive_busy_A(1) ;
       end

       begin
            drive_busy_B(1) ; 
       end 

       begin
            check_output_A(MBINIT_REPAIRVAL_result_resp , 1) ;
       end

       begin
            check_output_B(MBINIT_REPAIRVAL_result_resp , 1) ; 
       end 

    join

    @(posedge i_clk);
    fork
       begin
            drive_busy_A(1) ;
       end

       begin
            drive_busy_B(1) ; 
       end 

       begin
            check_output_A(MBINIT_REPAIRVAL_done_req , 1) ;
       end

       begin
            check_output_B(MBINIT_REPAIRVAL_done_req , 1) ; 
       end 

    join
    repeat (1) @(posedge i_clk);
    wait(o_msg_valid_B || o_msg_valid_A) ;

    fork
       begin
            drive_busy_A(1) ;
       end

       begin
            drive_busy_B(1) ; 
       end 

       begin
            check_output_A(MBINIT_REPAIRVAL_done_resp , 1) ;
       end

       begin
            check_output_B(MBINIT_REPAIRVAL_done_resp , 1) ; 
       end 

    join

    wait (DUT_A.dut_repairval.o_MBINIT_REPAIRVAL_end == 1 && DUT_B.dut_repairval.o_MBINIT_REPAIRVAL_end ==1);
    $display("MBINIT.REPAIRVAL is finished");
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//      MBINIT.REVERSALMB TEST
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

@(posedge i_clk);
    fork
       begin
            drive_busy_A(1) ;
       end

       begin
            drive_busy_B(1) ; 
       end 

       begin
            check_output_A(MBINIT_REVERSALMB_init_req  , 1) ;
       end

       begin
            check_output_B(MBINIT_REVERSALMB_init_req  , 1) ; 
       end 

    join
    repeat (1) @(posedge i_clk);
    wait(o_msg_valid_B || o_msg_valid_A) ;

    fork
       begin
            drive_busy_A(1) ;
       end

       begin
            drive_busy_B(1) ; 
       end 

       begin
            check_output_A(MBINIT_REVERSALMB_init_resp , 1) ;
       end

       begin
            check_output_B(MBINIT_REVERSALMB_init_resp , 1) ; 
       end 

    join
repeat (1) @(posedge i_clk);
wait(o_msg_valid_B || o_msg_valid_A) ;
    fork
       begin
            drive_busy_A(1) ;
       end

       begin
            drive_busy_B(1) ; 
       end 

       begin
            check_output_A(MBINIT_REVERSALMB_clear_error_req , 1) ;
       end

       begin
            check_output_B(MBINIT_REVERSALMB_clear_error_req , 1) ; 
       end 

    join

repeat (1) @(posedge i_clk);
wait(o_msg_valid_B || o_msg_valid_A) ;
    fork
       begin
            drive_busy_A(1) ;
       end

       begin
            drive_busy_B(1) ; 
       end 

       begin
            check_output_A(MBINIT_REVERSALMB_clear_error_resp , 1) ;
       end

       begin
            check_output_B(MBINIT_REVERSALMB_clear_error_resp , 1) ; 
       end 

    join

wait (DUT_A.dut_reversalmb.o_MBINIT_REVERSALMB_LaneID_Pattern_En == 3 && DUT_B.dut_reversalmb.o_MBINIT_REVERSALMB_LaneID_Pattern_En ==3);
$display("pattern generators are on") ; 
repeat(2) @(posedge i_clk); 
i_LaneID_Pattern_done_A = 1 ;
i_LaneID_Pattern_done_B = 1 ;
i_REVERSAL_Result_logged_A = 'hffff ;
i_REVERSAL_Result_logged_B = 'hffff ;
$display("pattern generator is done") ;

    repeat (2) @(posedge i_clk);
    
    fork
       begin
            drive_busy_A(1) ;
       end

       begin
            drive_busy_B(1) ; 
       end 

       begin
            check_output_A(MBINIT_REVERSALMB_result_req , 1) ;
       end

       begin
            check_output_B(MBINIT_REVERSALMB_result_req , 1) ; 
       end 

    join
    repeat (1) @(posedge i_clk);
    wait(o_msg_valid_B || o_msg_valid_A) ;

    fork
       begin
            drive_busy_A(1) ;
       end

       begin
            drive_busy_B(1) ; 
       end 

       begin
            check_output_A(MBINIT_REVERSALMB_result_resp , 1) ;
       end

       begin
            check_output_B(MBINIT_REVERSALMB_result_resp , 1) ; 
       end 

    join
     @(posedge i_clk);
    wait(o_msg_valid_B || o_msg_valid_A) ;
    fork
       begin
            drive_busy_A(1) ;
       end

       begin
            drive_busy_B(1) ; 
       end 

       begin
            check_output_A(MBINIT_REVERSALMB_done_req , 1) ;
       end

       begin
            check_output_B(MBINIT_REVERSALMB_done_req , 1) ; 
       end 

    join
    repeat (1) @(posedge i_clk);
    wait(o_msg_valid_B || o_msg_valid_A) ;

    fork
       begin
            drive_busy_A(1) ;
       end

       begin
            drive_busy_B(1) ; 
       end 

       begin
            check_output_A(MBINIT_REVERSALMB_done_resp , 1) ;
       end

       begin
            check_output_B(MBINIT_REVERSALMB_done_resp , 1) ; 
       end 

    join

    wait (DUT_A.dut_reversalmb.o_MBINIT_REVERSALMB_end == 1 && DUT_B.dut_reversalmb.o_MBINIT_REVERSALMB_end==1);
    $display("MBINIT.REVERSALMB is finished");

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//      MBINIT.REPAIRMB TEST
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////    

repeat (1) @(posedge i_clk);
    fork
       begin
            drive_busy_A(1) ;
       end

       begin
            drive_busy_B(1) ; 
       end 

       begin
            check_output_A(MBINIT_REPAIRMB_start_req , 1) ;
       end

       begin
            check_output_B(MBINIT_REPAIRMB_start_req , 1) ; 
       end 

    join
    repeat (1) @(posedge i_clk);
    //wait(o_msg_valid_B || o_msg_valid_A) ;

    fork
       begin
            drive_busy_A(1) ;
       end

       begin
            drive_busy_B(1) ; 
       end 

       begin
            check_output_A( MBINIT_REPAIRMB_start_resp , 1) ;
       end

       begin
            check_output_B( MBINIT_REPAIRMB_start_resp, 1) ; 
       end 

    join
wait (DUT_A.dut_repairmb.o_Transmitter_initiated_D2C_en && DUT_B.dut_repairmb.o_Transmitter_initiated_D2C_en && DUT_A.dut_repairmb.o_perlane_Transmitter_initiated_D2C && DUT_B.dut_repairmb.o_perlane_Transmitter_initiated_D2C);
$display("pattern generators are on") ; 
repeat(1) @(posedge i_clk);
i_d2c_tx_ack_A = 1 ;
i_d2c_tx_ack_B = 1 ;
 i_lanes_results_tx_A = 'hffff ;
 i_lanes_results_tx_B = 'hffff ;
$display("pattern generator is done") ;
wait(o_msg_valid_B || o_msg_valid_A) ;

//repeat(2) @(posedge i_clk);
    fork
       begin
            drive_busy_A(1) ;
       end

       begin
            drive_busy_B(1) ; 
       end 

       begin
            check_output_A(MBINIT_REPAIRMB_apply_degrade_req , 1) ;
       end

       begin
            check_output_B(MBINIT_REPAIRMB_apply_degrade_req , 1) ; 
       end 

    join
    repeat (1) @(posedge i_clk);
    wait(o_msg_valid_B || o_msg_valid_A) ;

    fork
       begin
            drive_busy_A(1) ;
       end

       begin
            drive_busy_B(1) ; 
       end 

       begin
            check_output_A(MBINIT_REPAIRMB_apply_degrade_resp , 1) ;
       end

       begin
            check_output_B(MBINIT_REPAIRMB_apply_degrade_resp , 1) ; 
       end 

    join
     wait(o_msg_valid_B || o_msg_valid_A) ;
    @(posedge i_clk);
    fork
       begin
            drive_busy_A(1) ;
       end

       begin
            drive_busy_B(1) ; 
       end 

       begin
            check_output_A(MBINIT_REPAIRMB_end_req , 1) ;
       end

       begin
            check_output_B(MBINIT_REPAIRMB_end_req , 1) ; 
       end 

    join
    
    wait(o_msg_valid_B || o_msg_valid_A) ;
     repeat (1) @(posedge i_clk);
    fork
       begin
            drive_busy_A(1) ;
       end

       begin
            drive_busy_B(1) ; 
       end 

       begin
            check_output_A(MBINIT_REPAIRMB_end_resp , 1) ;
       end

       begin
            check_output_B(MBINIT_REPAIRMB_end_resp , 1) ; 
       end 

    join

    wait (DUT_A.dut_repairmb.o_MBINIT_REPAIRMB_end == 1 && DUT_B.dut_repairmb.o_MBINIT_REPAIRMB_end ==1);
    $display("MBINIT.REPAIRMB is finished");
    $display("MBINIT is finished");

    repeat (10) @(posedge i_clk); 
    $stop ;

end

endmodule