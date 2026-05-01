`timescale 1ns / 1ps

module tb_REPAIRMB_Wrapper;

parameter CLK_PERIOD = 10;

// DUT1 signals (Transmitter/Module side)
reg         dut1_CLK, dut1_rst_n, dut1_MBINIT_REVERSALMB_end;
reg         dut1_i_d2c_tx_ack;
reg  [15:0] dut1_i_lanes_results_tx;
reg  [1:0]  dut1_i_Functional_Lanes;
wire [3:0]  dut1_i_RX_SbMessage;
wire        dut1_i_msg_valid, dut1_i_falling_edge_busy;
wire [3:0]  dut1_o_TX_SbMessage;
wire        dut1_o_MBINIT_REPAIRMB_end, dut1_o_tx_data_valid_repair;
wire [1:0]  dut1_o_Functional_Lanes_out_tx, dut1_o_Functional_Lanes_out_rx;
wire        dut1_o_Transmitter_initiated_D2C_en;
wire        dut1_o_perlane_Transmitter_initiated_D2C;
wire        dut1_o_mainband_Transmitter_initiated_D2C;
wire        dut1_o_train_error;
wire [2:0]  dut1_o_msg_info_repairmb;

// DUT2 signals (Partner/Receiver side)
reg         dut2_CLK, dut2_rst_n, dut2_MBINIT_REVERSALMB_end;
reg         dut2_i_d2c_tx_ack;
reg  [15:0] dut2_i_lanes_results_tx;
reg  [1:0]  dut2_i_Functional_Lanes;
wire [3:0]  dut2_i_RX_SbMessage;
wire        dut2_i_msg_valid, dut2_i_falling_edge_busy;
wire [3:0]  dut2_o_TX_SbMessage;
wire        dut2_o_MBINIT_REPAIRMB_end, dut2_o_tx_data_valid_repair;
wire [1:0]  dut2_o_Functional_Lanes_out_tx, dut2_o_Functional_Lanes_out_rx;
wire        dut2_o_Transmitter_initiated_D2C_en;
wire        dut2_o_perlane_Transmitter_initiated_D2C;
wire        dut2_o_mainband_Transmitter_initiated_D2C;
wire        dut2_o_train_error;
wire [2:0]  dut2_o_msg_info_repairmb;

// Message definitions
localparam [3:0] MBINIT_REPAIRMB_start_req          = 4'b0001;
localparam [3:0] MBINIT_REPAIRMB_start_resp         = 4'b0010;
localparam [3:0] MBINIT_REPAIRMB_apply_degrade_req  = 4'b0101;
localparam [3:0] MBINIT_REPAIRMB_apply_degrade_resp = 4'b0110;
localparam [3:0] MBINIT_REPAIRMB_end_req            = 4'b0011;
localparam [3:0] MBINIT_REPAIRMB_end_resp           = 4'b0100;

// Cross-connect messages
assign dut1_i_RX_SbMessage = dut2_o_TX_SbMessage;
assign dut2_i_RX_SbMessage = dut1_o_TX_SbMessage;

// Falling edge busy generation: pulse 2-3 cycles AFTER tx_data_valid rises
reg [2:0] dut1_valid_counter, dut2_valid_counter;
reg dut1_prev_ValidOut, dut2_prev_ValidOut;

always @(posedge dut1_CLK or negedge dut1_rst_n) begin
    if (!dut1_rst_n) begin
        dut1_valid_counter <= 3'd0;
        dut1_prev_ValidOut <= 1'b0;
    end else begin
        dut1_prev_ValidOut <= dut1_o_tx_data_valid_repair;
        
        // Detect rising edge of tx_data_valid_repair
        if (dut1_o_tx_data_valid_repair && !dut1_prev_ValidOut) begin
            dut1_valid_counter <= 3'd3; // Start counting down from 3
        end else if (dut1_valid_counter > 0) begin
            dut1_valid_counter <= dut1_valid_counter - 1;
        end
    end
end

always @(posedge dut2_CLK or negedge dut2_rst_n) begin
    if (!dut2_rst_n) begin
        dut2_valid_counter <= 3'd0;
        dut2_prev_ValidOut <= 1'b0;
    end else begin
        dut2_prev_ValidOut <= dut2_o_tx_data_valid_repair;
        
        // Detect rising edge of tx_data_valid_repair
        if (dut2_o_tx_data_valid_repair && !dut2_prev_ValidOut) begin
            dut2_valid_counter <= 3'd3; // Start counting down from 3
        end else if (dut2_valid_counter > 0) begin
            dut2_valid_counter <= dut2_valid_counter - 1;
        end
    end
end

assign dut1_i_falling_edge_busy = 1'b1;
assign dut2_i_falling_edge_busy = 1'b1;

// Message valid generation
reg [3:0] dut1_prev_rx_msg, dut2_prev_rx_msg;

always @(posedge dut1_CLK or negedge dut1_rst_n) begin
    if (!dut1_rst_n)
        dut1_prev_rx_msg <= 4'b0000;
    else if (dut2_o_tx_data_valid_repair)
        dut1_prev_rx_msg <= dut1_i_RX_SbMessage;
end

always @(posedge dut2_CLK or negedge dut2_rst_n) begin
    if (!dut2_rst_n)
        dut2_prev_rx_msg <= 4'b0000;
    else if (dut1_o_tx_data_valid_repair)
        dut2_prev_rx_msg <= dut2_i_RX_SbMessage;
end

assign dut1_i_msg_valid = 1'b1;
assign dut2_i_msg_valid = 1'b1;

// Clocks
initial begin dut1_CLK = 0; forever #(CLK_PERIOD/2) dut1_CLK = ~dut1_CLK; end
initial begin dut2_CLK = 0; forever #(CLK_PERIOD/2) dut2_CLK = ~dut2_CLK; end

// Automatic D2C acknowledgment - responds immediately after D2C test is initiated
reg dut1_prev_d2c_en, dut2_prev_d2c_en;

always @(posedge dut1_CLK or negedge dut1_rst_n) begin
    if (!dut1_rst_n) begin
        dut1_i_d2c_tx_ack <= 1'b0;
        dut1_prev_d2c_en <= 1'b0;
    end else begin
        dut1_prev_d2c_en <= dut1_o_Transmitter_initiated_D2C_en && dut1_o_perlane_Transmitter_initiated_D2C;
        
        // Generate ack on the rising edge of D2C enable (when it wasn't high before but is high now)
        if (!dut1_prev_d2c_en && dut1_o_Transmitter_initiated_D2C_en && dut1_o_perlane_Transmitter_initiated_D2C)
            dut1_i_d2c_tx_ack <= 1'b1;
        else
            dut1_i_d2c_tx_ack <= 1'b0;
    end
end

always @(posedge dut2_CLK or negedge dut2_rst_n) begin
    if (!dut2_rst_n) begin
        dut2_i_d2c_tx_ack <= 1'b0;
        dut2_prev_d2c_en <= 1'b0;
    end else begin
        dut2_prev_d2c_en <= dut2_o_Transmitter_initiated_D2C_en && dut2_o_perlane_Transmitter_initiated_D2C;
        
        // Generate ack on the rising edge of D2C enable (when it wasn't high before but is high now)
        if (!dut2_prev_d2c_en && dut2_o_Transmitter_initiated_D2C_en && dut2_o_perlane_Transmitter_initiated_D2C)
            dut2_i_d2c_tx_ack <= 1'b1;
        else
            dut2_i_d2c_tx_ack <= 1'b0;
    end
end

// DUT instances
REPAIRMB_Wrapper dut1 (
    .CLK(dut1_CLK),
    .rst_n(dut1_rst_n),
    .MBINIT_REVERSALMB_end(dut1_MBINIT_REVERSALMB_end),
    .i_RX_SbMessage(dut1_i_RX_SbMessage),
    .i_falling_edge_busy(dut1_i_falling_edge_busy),
    .i_d2c_tx_ack(dut1_i_d2c_tx_ack),
    .i_lanes_results_tx(dut1_i_lanes_results_tx),
    .i_Functional_Lanes(dut1_i_Functional_Lanes),
    .i_msg_valid(dut1_i_msg_valid),
    .o_TX_SbMessage(dut1_o_TX_SbMessage),
    .o_MBINIT_REPAIRMB_end(dut1_o_MBINIT_REPAIRMB_end),
    .o_tx_data_valid_repair(dut1_o_tx_data_valid_repair),
    .o_Functional_Lanes_out_tx(dut1_o_Functional_Lanes_out_tx),
    .o_Functional_Lanes_out_rx(dut1_o_Functional_Lanes_out_rx),
    .o_Transmitter_initiated_D2C_en(dut1_o_Transmitter_initiated_D2C_en),
    .o_perlane_Transmitter_initiated_D2C(dut1_o_perlane_Transmitter_initiated_D2C),
    .o_mainband_Transmitter_initiated_D2C(dut1_o_mainband_Transmitter_initiated_D2C),
    .o_train_error(dut1_o_train_error),
    .o_msg_info_repairmb(dut1_o_msg_info_repairmb)
);

REPAIRMB_Wrapper dut2 (
    .CLK(dut2_CLK),
    .rst_n(dut2_rst_n),
    .MBINIT_REVERSALMB_end(dut2_MBINIT_REVERSALMB_end),
    .i_RX_SbMessage(dut2_i_RX_SbMessage),
    .i_falling_edge_busy(dut2_i_falling_edge_busy),
    .i_d2c_tx_ack(dut2_i_d2c_tx_ack),
    .i_lanes_results_tx(dut2_i_lanes_results_tx),
    .i_Functional_Lanes(dut2_i_Functional_Lanes),
    .i_msg_valid(dut2_i_msg_valid),
    .o_TX_SbMessage(dut2_o_TX_SbMessage),
    .o_MBINIT_REPAIRMB_end(dut2_o_MBINIT_REPAIRMB_end),
    .o_tx_data_valid_repair(dut2_o_tx_data_valid_repair),
    .o_Functional_Lanes_out_tx(dut2_o_Functional_Lanes_out_tx),
    .o_Functional_Lanes_out_rx(dut2_o_Functional_Lanes_out_rx),
    .o_Transmitter_initiated_D2C_en(dut2_o_Transmitter_initiated_D2C_en),
    .o_perlane_Transmitter_initiated_D2C(dut2_o_perlane_Transmitter_initiated_D2C),
    .o_mainband_Transmitter_initiated_D2C(dut2_o_mainband_Transmitter_initiated_D2C),
    .o_train_error(dut2_o_train_error),
    .o_msg_info_repairmb(dut2_o_msg_info_repairmb)
);

// State monitors
wire [3:0] dut1_mod_st = dut1.REPAIRMB_Module_inst.CS;
wire [3:0] dut1_par_st = dut1.REPAIRMB_Module_Partner_inst.CS;
wire [3:0] dut2_mod_st = dut2.REPAIRMB_Module_inst.CS;
wire [3:0] dut2_par_st = dut2.REPAIRMB_Module_Partner_inst.CS;

// Debug monitors
always @(posedge dut1_CLK) begin
    if (dut1_i_falling_edge_busy) 
        $display("[%0t] DUT1: FALLING_EDGE_BUSY pulse", $time);
    if (dut1_o_tx_data_valid_repair && !dut1_prev_ValidOut) begin
        $display("[%0t] DUT1: TX rising, msg=%b (%s)", $time, dut1_o_TX_SbMessage, 
                 dut1_o_TX_SbMessage == MBINIT_REPAIRMB_start_req ? "start_req" :
                 dut1_o_TX_SbMessage == MBINIT_REPAIRMB_start_resp ? "start_resp" :
                 dut1_o_TX_SbMessage == MBINIT_REPAIRMB_apply_degrade_req ? "degrade_req" :
                 dut1_o_TX_SbMessage == MBINIT_REPAIRMB_apply_degrade_resp ? "degrade_resp" :
                 dut1_o_TX_SbMessage == MBINIT_REPAIRMB_end_req ? "end_req" :
                 dut1_o_TX_SbMessage == MBINIT_REPAIRMB_end_resp ? "end_resp" : "unknown");
    end
    if (dut1_i_msg_valid) begin
        $display("[%0t] DUT1: RX msg_valid=%b (%s)", $time, dut1_i_RX_SbMessage,
                 dut1_i_RX_SbMessage == MBINIT_REPAIRMB_start_req ? "start_req" :
                 dut1_i_RX_SbMessage == MBINIT_REPAIRMB_start_resp ? "start_resp" :
                 dut1_i_RX_SbMessage == MBINIT_REPAIRMB_apply_degrade_req ? "degrade_req" :
                 dut1_i_RX_SbMessage == MBINIT_REPAIRMB_apply_degrade_resp ? "degrade_resp" :
                 dut1_i_RX_SbMessage == MBINIT_REPAIRMB_end_req ? "end_req" :
                 dut1_i_RX_SbMessage == MBINIT_REPAIRMB_end_resp ? "end_resp" : "unknown");
    end
    if (dut1_mod_st != dut1.REPAIRMB_Module_inst.NS) 
        $display("[%0t] DUT1.MOD: %0d->%0d", $time, dut1_mod_st, dut1.REPAIRMB_Module_inst.NS);
    if (dut1_par_st != dut1.REPAIRMB_Module_Partner_inst.NS) 
        $display("[%0t] DUT1.PAR: %0d->%0d", $time, dut1_par_st, dut1.REPAIRMB_Module_Partner_inst.NS);
    if (dut1_o_Transmitter_initiated_D2C_en && !dut1.REPAIRMB_Module_inst.o_Transmitter_initiated_D2C_en)
        $display("[%0t] DUT1: D2C Test Enabled", $time);
    if (dut1_i_d2c_tx_ack)
        $display("[%0t] DUT1: D2C ACK (auto-generated)", $time);
end

always @(posedge dut2_CLK) begin
    if (dut2_i_falling_edge_busy) 
        $display("[%0t] DUT2: FALLING_EDGE_BUSY pulse", $time);
    if (dut2_o_tx_data_valid_repair && !dut2_prev_ValidOut) begin
        $display("[%0t] DUT2: TX rising, msg=%b (%s)", $time, dut2_o_TX_SbMessage,
                 dut2_o_TX_SbMessage == MBINIT_REPAIRMB_start_req ? "start_req" :
                 dut2_o_TX_SbMessage == MBINIT_REPAIRMB_start_resp ? "start_resp" :
                 dut2_o_TX_SbMessage == MBINIT_REPAIRMB_apply_degrade_req ? "degrade_req" :
                 dut2_o_TX_SbMessage == MBINIT_REPAIRMB_apply_degrade_resp ? "degrade_resp" :
                 dut2_o_TX_SbMessage == MBINIT_REPAIRMB_end_req ? "end_req" :
                 dut2_o_TX_SbMessage == MBINIT_REPAIRMB_end_resp ? "end_resp" : "unknown");
    end
    if (dut2_i_msg_valid) begin
        $display("[%0t] DUT2: RX msg_valid=%b (%s)", $time, dut2_i_RX_SbMessage,
                 dut2_i_RX_SbMessage == MBINIT_REPAIRMB_start_req ? "start_req" :
                 dut2_i_RX_SbMessage == MBINIT_REPAIRMB_start_resp ? "start_resp" :
                 dut2_i_RX_SbMessage == MBINIT_REPAIRMB_apply_degrade_req ? "degrade_req" :
                 dut2_i_RX_SbMessage == MBINIT_REPAIRMB_apply_degrade_resp ? "degrade_resp" :
                 dut2_i_RX_SbMessage == MBINIT_REPAIRMB_end_req ? "end_req" :
                 dut2_i_RX_SbMessage == MBINIT_REPAIRMB_end_resp ? "end_resp" : "unknown");
    end
    if (dut2_mod_st != dut2.REPAIRMB_Module_inst.NS) 
        $display("[%0t] DUT2.MOD: %0d->%0d", $time, dut2_mod_st, dut2.REPAIRMB_Module_inst.NS);
    if (dut2_par_st != dut2.REPAIRMB_Module_Partner_inst.NS) 
        $display("[%0t] DUT2.PAR: %0d->%0d", $time, dut2_par_st, dut2.REPAIRMB_Module_Partner_inst.NS);
    if (dut2_o_Transmitter_initiated_D2C_en && !dut2.REPAIRMB_Module_inst.o_Transmitter_initiated_D2C_en)
        $display("[%0t] DUT2: D2C Test Enabled", $time);
    if (dut2_i_d2c_tx_ack)
        $display("[%0t] DUT2: D2C ACK (auto-generated)", $time);
end

// Test scenarios
// NOTE: DUT1 = Transmitter/Module (initiates the repair)
//       DUT2 = Partner/Receiver (responds to repair requests)
task test_scenario_all_lanes_good;
    begin
        $display("\n=== TEST SCENARIO: All Lanes Good (2'b11) ===");
        $display("DUT1 (Module/Transmitter) initiates, DUT2 (Partner) responds");
        
        // Both sides start with all lanes functional
        dut1_i_lanes_results_tx = 16'hFFFF;
        dut2_i_lanes_results_tx = 16'hFFFF;
        dut2_i_Functional_Lanes = 2'b11;
        
        #(CLK_PERIOD*2); 
        dut1_MBINIT_REVERSALMB_end = 1;
        dut2_MBINIT_REVERSALMB_end = 1;
        $display("[%0t] REVERSALMB_end asserted\n", $time);
        
        // Wait for Module to send start_req
        wait(dut1_o_TX_SbMessage == MBINIT_REPAIRMB_start_req);
        $display("[%0t] DUT1 (Module): Sent start_req", $time);
        
        // Wait for Partner to respond with start_resp
        wait(dut2_o_TX_SbMessage == MBINIT_REPAIRMB_start_resp);
        $display("[%0t] DUT2 (Partner): Sent start_resp", $time);
        
        // Wait for D2C test initiation from DUT1
        @(posedge dut1_o_Transmitter_initiated_D2C_en);
        $display("[%0t] DUT1: D2C test initiated (ack auto-generated)", $time);
        
        // Wait for completion
        #(CLK_PERIOD*200);
    end
endtask

task test_scenario_lane_degradation;
    begin
        $display("\n=== TEST SCENARIO: Lane Degradation (2'b11 -> 2'b10) ===");
        
        // DUT1 starts with only upper lanes working
        dut1_i_lanes_results_tx = 16'hFF00;
        dut2_i_lanes_results_tx = 16'hFFFF;
        dut2_i_Functional_Lanes = 2'b10;
        dut1_i_Functional_Lanes = 2'b11;
        #(CLK_PERIOD*2); 
        dut1_MBINIT_REVERSALMB_end = 1;
        dut2_MBINIT_REVERSALMB_end = 1;
        $display("[%0t] REVERSALMB_end asserted\n", $time);
        
        // Wait for D2C test initiation
        @(posedge dut1_o_Transmitter_initiated_D2C_en);
        $display("[%0t] DUT1: D2C test initiated (ack auto-generated)", $time);
        
        // Wait for completion
        #(CLK_PERIOD*300);
    end
endtask

task test_scenario_single_lane;
    begin
        $display("\n=== TEST SCENARIO: Single Lane (2'b01) ===");
        
        // DUT1 starts with only lower lanes working
        dut1_i_lanes_results_tx = 16'h00FF;
        dut2_i_lanes_results_tx = 16'hFFFF;
        dut2_i_Functional_Lanes = 2'b01;
        dut1_i_Functional_Lanes = 2'b11;

        #(CLK_PERIOD*2); 
        dut1_MBINIT_REVERSALMB_end = 1;
        dut2_MBINIT_REVERSALMB_end = 1;
        $display("[%0t] REVERSALMB_end asserted\n", $time);
        
        // Wait for D2C test initiation
        @(posedge dut1_o_Transmitter_initiated_D2C_en);
        $display("[%0t] DUT1: D2C test initiated (ack auto-generated)", $time);
        
        // Wait for completion
        #(CLK_PERIOD*300);
    end
endtask

task test_scenario_no_lanes;
    begin
        $display("\n=== TEST SCENARIO: No Lanes (2'b00 - Error Expected) ===");
        
        // DUT1 starts with no lanes working
        dut1_i_lanes_results_tx = 16'h0000;
        dut2_i_lanes_results_tx = 16'hFFFF;
        dut2_i_Functional_Lanes = 2'b00;
          dut1_i_Functional_Lanes = 2'b11;

        #(CLK_PERIOD*2); 
        dut1_MBINIT_REVERSALMB_end = 1;
        dut2_MBINIT_REVERSALMB_end = 1;
        $display("[%0t] REVERSALMB_end asserted\n", $time);
        
        // Wait for D2C test initiation
        @(posedge dut1_o_Transmitter_initiated_D2C_en);
        $display("[%0t] DUT1: D2C test initiated (ack auto-generated)", $time);
        
        // Wait for error
        #(CLK_PERIOD*100);
        
        if (dut2_o_train_error)
            $display("[%0t] *** Train Error Detected as Expected ***", $time);
    end
endtask

// Main test
initial begin
    // Initialize all signals
    dut1_rst_n = 0; dut1_MBINIT_REVERSALMB_end = 0;
    dut1_i_lanes_results_tx = 16'h0000;
    dut1_i_Functional_Lanes = 2'b11;
    
    dut2_rst_n = 0; dut2_MBINIT_REVERSALMB_end = 0;
    dut2_i_lanes_results_tx = 16'h0000;
    dut2_i_Functional_Lanes = 2'b11;
    
    #(CLK_PERIOD*5); 
    dut1_rst_n = 1; 
    dut2_rst_n = 1;
    #(CLK_PERIOD*2);
    
    $display("\n=== REPAIRMB WRAPPER TEST START ===");
    $display("NOTE: tx_data_valid_repair stays HIGH while in SEND states");
    $display("      falling_edge_busy PULSES 2 cycles after valid rises");
    $display("      i_d2c_tx_ack is AUTO-GENERATED when D2C test is enabled");
    $display("      This simulates sideband bus acknowledging the transmission\n");
    
    // Select test scenario (uncomment one)
    //test_scenario_all_lanes_good;
   // test_scenario_lane_degradation;
    //test_scenario_single_lane;
    test_scenario_no_lanes;
    
    $display("\n=== RESULTS ===");
    $display("DUT1: Mod_st=%0d, Par_st=%0d, End=%b, Lanes_TX=%b, Train_Error=%b", 
             dut1_mod_st, dut1_par_st, dut1_o_MBINIT_REPAIRMB_end, 
             dut1_o_Functional_Lanes_out_tx, dut1_o_train_error);
    $display("DUT2: Mod_st=%0d, Par_st=%0d, End=%b, Lanes_RX=%b, Train_Error=%b", 
             dut2_mod_st, dut2_par_st, dut2_o_MBINIT_REPAIRMB_end,
             dut2_o_Functional_Lanes_out_rx, dut2_o_train_error);
    
    if (dut1_o_MBINIT_REPAIRMB_end && dut2_o_MBINIT_REPAIRMB_end && !dut1_o_train_error && !dut2_o_train_error)
        $display("\n*** PASS ***\n");
    else if (dut1_o_train_error || dut2_o_train_error)
        $display("\n*** ERROR DETECTED (may be expected) ***\n");
    else
        $display("\n*** FAIL/INCOMPLETE ***\n");
    
    #(CLK_PERIOD*10); 
    $finish;
end

initial begin 
    $dumpfile("tb_REPAIRMB_Wrapper.vcd"); 
    $dumpvars(0, tb_REPAIRMB_Wrapper); 
end

initial begin 
    #(CLK_PERIOD*3000); 
    $display("\n*** TIMEOUT ***\n"); 
    $finish; 
end

endmodule