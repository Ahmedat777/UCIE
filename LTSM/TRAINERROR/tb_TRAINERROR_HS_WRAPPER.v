`timescale 1ns / 1ps

module tb_TRAINERROR_HS_WRAPPER;

parameter CLK_PERIOD = 10;

// DUT1 signals (Transmitter/Module side)
reg         dut1_CLK, dut1_rst_n;
reg         dut1_i_trainerror_en;
wire [3:0]  dut1_i_Rx_SbMessage;
wire        dut1_i_msg_valid, dut1_i_falling_edge_busy;
wire [3:0]  dut1_o_TX_SbMessage;
wire        dut1_o_TRAINERROR_HS_end;
wire        dut1_o_tx_msg_valid;

// DUT2 signals (Partner/Receiver side)
reg         dut2_CLK, dut2_rst_n;
reg         dut2_i_trainerror_en;
wire [3:0]  dut2_i_Rx_SbMessage;
wire        dut2_i_msg_valid, dut2_i_falling_edge_busy;
wire [3:0]  dut2_o_TX_SbMessage;
wire        dut2_o_TRAINERROR_HS_end;
wire        dut2_o_tx_msg_valid;

// Message definitions (from TRAINERROR modules)
localparam [3:0] TRAINERROR_ENTRY_REQ_MSG  = 15;
localparam [3:0] TRAINERROR_ENTRY_RESP_MSG = 14;

// Cross-connect messages between DUT1 and DUT2
assign dut1_i_Rx_SbMessage = dut2_o_TX_SbMessage;
assign dut2_i_Rx_SbMessage = dut1_o_TX_SbMessage;

// Falling edge busy generation: pulse 2-3 cycles AFTER tx_msg_valid rises
reg [2:0] dut1_valid_counter, dut2_valid_counter;
reg dut1_prev_ValidOut, dut2_prev_ValidOut;

always @(posedge dut1_CLK or negedge dut1_rst_n) begin
    if (!dut1_rst_n) begin
        dut1_valid_counter <= 3'd0;
        dut1_prev_ValidOut <= 1'b0;
    end else begin
        dut1_prev_ValidOut <= dut1_o_tx_msg_valid;
        
        // Detect rising edge of tx_msg_valid
        if (dut1_o_tx_msg_valid && !dut1_prev_ValidOut) begin
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
        dut2_prev_ValidOut <= dut2_o_tx_msg_valid;
        
        // Detect rising edge of tx_msg_valid
        if (dut2_o_tx_msg_valid && !dut2_prev_ValidOut) begin
            dut2_valid_counter <= 3'd3; // Start counting down from 3
        end else if (dut2_valid_counter > 0) begin
            dut2_valid_counter <= dut2_valid_counter - 1;
        end
    end
end

assign dut1_i_falling_edge_busy = (dut1_valid_counter == 1);
assign dut2_i_falling_edge_busy = (dut2_valid_counter == 1);

// Message valid generation
reg [3:0] dut1_prev_rx_msg, dut2_prev_rx_msg;

always @(posedge dut1_CLK or negedge dut1_rst_n) begin
    if (!dut1_rst_n)
        dut1_prev_rx_msg <= 4'b0000;
    else if (dut2_o_tx_msg_valid)
        dut1_prev_rx_msg <= dut1_i_Rx_SbMessage;
end

always @(posedge dut2_CLK or negedge dut2_rst_n) begin
    if (!dut2_rst_n)
        dut2_prev_rx_msg <= 4'b0000;
    else if (dut1_o_tx_msg_valid)
        dut2_prev_rx_msg <= dut2_i_Rx_SbMessage;
end

assign dut1_i_msg_valid = dut2_o_tx_msg_valid;
assign dut2_i_msg_valid = dut1_o_tx_msg_valid;

// Clocks
initial begin dut1_CLK = 0; forever #(CLK_PERIOD/2) dut1_CLK = ~dut1_CLK; end
initial begin dut2_CLK = 0; forever #(CLK_PERIOD/2) dut2_CLK = ~dut2_CLK; end

// DUT instances
TRAINERROR_HS_WRAPPER #(
    .SB_MSG_WIDTH(4)
) dut1 (
    .i_clk(dut1_CLK),
    .i_rst_n(dut1_rst_n),
    .i_trainerror_en(dut1_i_trainerror_en),
    .i_msg_valid(dut1_i_msg_valid),
    .i_falling_edge_busy(dut1_i_falling_edge_busy),
    .i_Rx_SbMessage(dut1_i_Rx_SbMessage),
    .o_TX_SbMessage(dut1_o_TX_SbMessage),
    .o_TRAINERROR_HS_end(dut1_o_TRAINERROR_HS_end),
    .o_tx_msg_valid(dut1_o_tx_msg_valid)
);

TRAINERROR_HS_WRAPPER #(
    .SB_MSG_WIDTH(4)
) dut2 (
    .i_clk(dut2_CLK),
    .i_rst_n(dut2_rst_n),
    .i_trainerror_en(dut2_i_trainerror_en),
    .i_msg_valid(dut2_i_msg_valid),
    .i_falling_edge_busy(dut2_i_falling_edge_busy),
    .i_Rx_SbMessage(dut2_i_Rx_SbMessage),
    .o_TX_SbMessage(dut2_o_TX_SbMessage),
    .o_TRAINERROR_HS_end(dut2_o_TRAINERROR_HS_end),
    .o_tx_msg_valid(dut2_o_tx_msg_valid)
);

// State monitors
wire [2:0] dut1_mod_st = dut1.u1.CS;
wire [2:0] dut1_par_st = dut1.u2.CS;
wire [2:0] dut2_mod_st = dut2.u1.CS;
wire [2:0] dut2_par_st = dut2.u2.CS;

// Debug monitors
always @(posedge dut1_CLK) begin
    if (dut1_i_falling_edge_busy) 
        $display("[%0t] DUT1: FALLING_EDGE_BUSY pulse", $time);
    if (dut1_o_tx_msg_valid && !dut1_prev_ValidOut) begin
        $display("[%0t] DUT1: TX rising, msg=%b (%s)", $time, dut1_o_TX_SbMessage, 
                 dut1_o_TX_SbMessage == TRAINERROR_ENTRY_REQ_MSG ? "ENTRY_REQ" :
                 dut1_o_TX_SbMessage == TRAINERROR_ENTRY_RESP_MSG ? "ENTRY_RESP" : "unknown");
    end
    if (dut1_i_msg_valid) begin
        $display("[%0t] DUT1: RX msg_valid=%b (%s)", $time, dut1_i_Rx_SbMessage,
                 dut1_i_Rx_SbMessage == TRAINERROR_ENTRY_REQ_MSG ? "ENTRY_REQ" :
                 dut1_i_Rx_SbMessage == TRAINERROR_ENTRY_RESP_MSG ? "ENTRY_RESP" : "unknown");
    end
    if (dut1_mod_st != dut1.u1.NS) 
        $display("[%0t] DUT1.MOD: %0d->%0d", $time, dut1_mod_st, dut1.u1.NS);
    if (dut1_par_st != dut1.u2.NS) 
        $display("[%0t] DUT1.PAR: %0d->%0d", $time, dut1_par_st, dut1.u2.NS);
    if (dut1_o_TRAINERROR_HS_end)
        $display("[%0t] DUT1: TRAINERROR_HS_end asserted", $time);
end

always @(posedge dut2_CLK) begin
    if (dut2_i_falling_edge_busy) 
        $display("[%0t] DUT2: FALLING_EDGE_BUSY pulse", $time);
    if (dut2_o_tx_msg_valid && !dut2_prev_ValidOut) begin
        $display("[%0t] DUT2: TX rising, msg=%b (%s)", $time, dut2_o_TX_SbMessage,
                 dut2_o_TX_SbMessage == TRAINERROR_ENTRY_REQ_MSG ? "ENTRY_REQ" :
                 dut2_o_TX_SbMessage == TRAINERROR_ENTRY_RESP_MSG ? "ENTRY_RESP" : "unknown");
    end
    if (dut2_i_msg_valid) begin
        $display("[%0t] DUT2: RX msg_valid=%b (%s)", $time, dut2_i_Rx_SbMessage,
                 dut2_i_Rx_SbMessage == TRAINERROR_ENTRY_REQ_MSG ? "ENTRY_REQ" :
                 dut2_i_Rx_SbMessage == TRAINERROR_ENTRY_RESP_MSG ? "ENTRY_RESP" : "unknown");
    end
    if (dut2_mod_st != dut2.u1.NS) 
        $display("[%0t] DUT2.MOD: %0d->%0d", $time, dut2_mod_st, dut2.u1.NS);
    if (dut2_par_st != dut2.u2.NS) 
        $display("[%0t] DUT2.PAR: %0d->%0d", $time, dut2_par_st, dut2.u2.NS);
    if (dut2_o_TRAINERROR_HS_end)
        $display("[%0t] DUT2: TRAINERROR_HS_end asserted", $time);
end

// Test scenarios
// NOTE: DUT1 = Module (initiates the train error handshake)
//       DUT2 = Partner (responds to train error requests)

task test_scenario_normal_handshake;
begin
    $display("\n=== TEST SCENARIO: Normal TRAINERROR Handshake ===");
    $display("DUT1 (Module) initiates, DUT2 (Partner) responds");
    
    #(CLK_PERIOD*2); 
    dut1_i_trainerror_en = 1;
    dut2_i_trainerror_en = 1;
    $display("[%0t] trainerror_en asserted on both sides\n", $time);
    
    // Wait for Module to send ENTRY_REQ
    wait(dut1_o_TX_SbMessage == TRAINERROR_ENTRY_REQ_MSG && dut1_o_tx_msg_valid);
    $display("[%0t] DUT1 (Module): Sent ENTRY_REQ", $time);
    
    // Wait for Partner to respond with ENTRY_RESP
    wait(dut2_o_TX_SbMessage == TRAINERROR_ENTRY_RESP_MSG && dut2_o_tx_msg_valid);
    $display("[%0t] DUT2 (Partner): Sent ENTRY_RESP", $time);
    
    // Wait for completion
    wait(dut1_o_TRAINERROR_HS_end && dut2_o_TRAINERROR_HS_end);
    $display("[%0t] Both modules completed successfully", $time);
    
    #(CLK_PERIOD*10);
    
    // Deassert enable
    dut1_i_trainerror_en = 0;
    dut2_i_trainerror_en = 0;
    #(CLK_PERIOD*5);
end
endtask

task test_scenario_partner_initiates_first;
begin
    $display("\n=== TEST SCENARIO: Partner Receives REQ Before Initiating ===");
    $display("DUT2 receives ENTRY_REQ before it would initiate");
    
    #(CLK_PERIOD*2); 
    // Enable DUT1 first, then DUT2 slightly later
    dut1_i_trainerror_en = 1;
    $display("[%0t] DUT1 trainerror_en asserted\n", $time);
    
    #(CLK_PERIOD*3);
    dut2_i_trainerror_en = 1;
    $display("[%0t] DUT2 trainerror_en asserted (delayed)\n", $time);
    
    // Wait for handshake completion
    wait(dut1_o_TRAINERROR_HS_end && dut2_o_TRAINERROR_HS_end);
    $display("[%0t] Both modules completed successfully", $time);
    
    #(CLK_PERIOD*10);
    
    // Deassert enable
    dut1_i_trainerror_en = 0;
    dut2_i_trainerror_en = 0;
    #(CLK_PERIOD*5);
end
endtask

task test_scenario_simultaneous_enable;
begin
    $display("\n=== TEST SCENARIO: Simultaneous Enable ===");
    $display("Both sides enable simultaneously");
    
    #(CLK_PERIOD*2); 
    dut1_i_trainerror_en = 1;
    dut2_i_trainerror_en = 1;
    $display("[%0t] Both sides trainerror_en asserted simultaneously\n", $time);
    
    // Wait for completion
    wait(dut1_o_TRAINERROR_HS_end && dut2_o_TRAINERROR_HS_end);
    $display("[%0t] Both modules completed successfully", $time);
    
    #(CLK_PERIOD*10);
    
    // Deassert enable
    dut1_i_trainerror_en = 0;
    dut2_i_trainerror_en = 0;
    #(CLK_PERIOD*5);
end
endtask

task test_scenario_early_disable;
begin
    $display("\n=== TEST SCENARIO: Early Disable During Handshake ===");
    $display("Testing abort by disabling trainerror_en mid-handshake");
    
    #(CLK_PERIOD*2); 
    dut1_i_trainerror_en = 1;
    dut2_i_trainerror_en = 1;
    $display("[%0t] trainerror_en asserted on both sides\n", $time);
    
    // Wait for initial request
    wait(dut1_o_TX_SbMessage == TRAINERROR_ENTRY_REQ_MSG && dut1_o_tx_msg_valid);
    $display("[%0t] DUT1 sent ENTRY_REQ", $time);
    
    // Disable before completion
    #(CLK_PERIOD*5);
    dut1_i_trainerror_en = 0;
    dut2_i_trainerror_en = 0;
    $display("[%0t] trainerror_en deasserted (abort test)\n", $time);
    
    #(CLK_PERIOD*10);
    
    // Check that modules returned to IDLE
    if (dut1_mod_st == 0 && dut1_par_st == 0 && 
        dut2_mod_st == 0 && dut2_par_st == 0)
        $display("[%0t] Both modules returned to IDLE as expected", $time);
    
    #(CLK_PERIOD*5);
end
endtask

task test_scenario_multiple_handshakes;
integer i;
begin
    $display("\n=== TEST SCENARIO: Multiple Sequential Handshakes ===");
    
    for (i = 1; i <= 3; i = i + 1) begin
        $display("\n--- Handshake #%0d ---", i);
        
        #(CLK_PERIOD*2); 
        dut1_i_trainerror_en = 1;
        dut2_i_trainerror_en = 1;
        $display("[%0t] trainerror_en asserted\n", $time);
        
        // Wait for completion
        wait(dut1_o_TRAINERROR_HS_end && dut2_o_TRAINERROR_HS_end);
        $display("[%0t] Handshake #%0d completed", $time, i);
        
        #(CLK_PERIOD*5);
        
        // Deassert enable
        dut1_i_trainerror_en = 0;
        dut2_i_trainerror_en = 0;
        $display("[%0t] trainerror_en deasserted\n", $time);
        
        #(CLK_PERIOD*10);
    end
end
endtask

// Main test
initial begin
    // Initialize all signals
    dut1_rst_n = 0; 
    dut1_i_trainerror_en = 0;
    
    dut2_rst_n = 0; 
    dut2_i_trainerror_en = 0;
    
    #(CLK_PERIOD*5); 
    dut1_rst_n = 1; 
    dut2_rst_n = 1;
    #(CLK_PERIOD*2);
    
    $display("\n=== TRAINERROR_HS_WRAPPER TEST START ===");
    $display("NOTE: tx_msg_valid stays HIGH while in SEND states");
    $display("      falling_edge_busy PULSES 2-3 cycles after valid rises");
    $display("      This simulates sideband bus acknowledging the transmission\n");
    
    // Select test scenario (uncomment desired test)
    test_scenario_normal_handshake;
    //test_scenario_partner_initiates_first;
    //test_scenario_simultaneous_enable;
    //test_scenario_early_disable;
    //test_scenario_multiple_handshakes;
    
    $display("\n=== RESULTS ===");
    $display("DUT1: Mod_st=%0d, Par_st=%0d, HS_End=%b", 
             dut1_mod_st, dut1_par_st, dut1_o_TRAINERROR_HS_end);
    $display("DUT2: Mod_st=%0d, Par_st=%0d, HS_End=%b", 
             dut2_mod_st, dut2_par_st, dut2_o_TRAINERROR_HS_end);
    
    if (dut1_o_TRAINERROR_HS_end && dut2_o_TRAINERROR_HS_end)
        $display("\n*** PASS: Both modules completed handshake ***\n");
    else if (dut1_mod_st == 0 && dut2_mod_st == 0)
        $display("\n*** Test completed (modules in IDLE) ***\n");
    else
        $display("\n*** FAIL/INCOMPLETE ***\n");
    
    #(CLK_PERIOD*10); 
    $finish;
end

initial begin 
    $dumpfile("tb_TRAINERROR_HS_WRAPPER.vcd"); 
    $dumpvars(0, tb_TRAINERROR_HS_WRAPPER); 
end

initial begin 
    #(CLK_PERIOD*1000); 
    $display("\n*** TIMEOUT ***\n"); 
    $finish; 
end

endmodule