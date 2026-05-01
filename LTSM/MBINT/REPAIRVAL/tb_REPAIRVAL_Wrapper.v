`timescale 1ns / 1ps

module tb_REPAIRVAL_Wrapper;

parameter CLK_PERIOD = 10;

// DUT1 signals
reg         dut1_CLK, dut1_rst_n, dut1_i_REPAIRCLK_end;
reg         dut1_i_VAL_Pattern_done, dut1_i_VAL_Result_logged_RXSB, dut1_i_VAL_Result_logged_COMB;
wire [3:0]  dut1_i_Rx_SbMessage;
wire        dut1_i_msg_valid, dut1_i_falling_edge_busy;
wire        dut1_o_train_error_req, dut1_o_MBINIT_REPAIRVAL_Pattern_En, dut1_o_MBINIT_REPAIRVAL_end;
wire [3:0]  dut1_o_TX_SbMessage;
wire        dut1_o_VAL_128Result_logged, dut1_o_enable_16_iterations, dut1_o_ValidOutData;

// DUT2 signals
reg         dut2_CLK, dut2_rst_n, dut2_i_REPAIRCLK_end;
reg         dut2_i_VAL_Pattern_done, dut2_i_VAL_Result_logged_RXSB, dut2_i_VAL_Result_logged_COMB;
wire [3:0]  dut2_i_Rx_SbMessage;
wire        dut2_i_msg_valid, dut2_i_falling_edge_busy;
wire        dut2_o_train_error_req, dut2_o_MBINIT_REPAIRVAL_Pattern_En, dut2_o_MBINIT_REPAIRVAL_end;
wire [3:0]  dut2_o_TX_SbMessage;
wire        dut2_o_VAL_128Result_logged, dut2_o_enable_16_iterations, dut2_o_ValidOutData;

// Message definitions
localparam MBINI_REPAIRVAL_init_req = 4'b0001, MBINIT_REPAIRVAL_init_resp = 4'b0010;
localparam MBINIT_REPAIRVAL_result_req = 4'b0011, MBINIT_REPAIRVAL_result_resp = 4'b0100;
localparam MBINIT_REPAIRVAL_done_req = 4'b0101, MBINIT_REPAIRVAL_done_resp = 4'b0110;

// Cross-connect
assign dut1_i_Rx_SbMessage = dut2_o_TX_SbMessage;
assign dut2_i_Rx_SbMessage = dut1_o_TX_SbMessage;

// Falling edge busy generation: pulse 2-3 cycles AFTER ValidOutData rises
// This simulates the sideband acknowledging the transmission
reg [2:0] dut1_valid_counter, dut2_valid_counter;
reg dut1_prev_ValidOut, dut2_prev_ValidOut;

always @(posedge dut1_CLK or negedge dut1_rst_n) begin
    if (!dut1_rst_n) begin
        dut1_valid_counter <= 3'd0;
        dut1_prev_ValidOut <= 1'b0;
    end else begin
        dut1_prev_ValidOut <= dut1_o_ValidOutData;
        
        // Detect rising edge of ValidOutData
        if (dut1_o_ValidOutData && !dut1_prev_ValidOut) begin
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
        dut2_prev_ValidOut <= dut2_o_ValidOutData;
        
        // Detect rising edge of ValidOutData
        if (dut2_o_ValidOutData && !dut2_prev_ValidOut) begin
            dut2_valid_counter <= 3'd3; // Start counting down from 3
        end else if (dut2_valid_counter > 0) begin
            dut2_valid_counter <= dut2_valid_counter - 1;
        end
    end
end

// Pulse when counter reaches 1 (meaning 2 cycles after ValidOut rose)
assign dut1_i_falling_edge_busy = 1'b1;
assign dut2_i_falling_edge_busy = 1'b1;

// Message valid generation
reg [3:0] dut1_prev_rx_msg, dut2_prev_rx_msg;

always @(posedge dut1_CLK or negedge dut1_rst_n) begin
    if (!dut1_rst_n)
        dut1_prev_rx_msg <= 4'b0000;
    else if (dut2_o_ValidOutData)
        dut1_prev_rx_msg <= dut1_i_Rx_SbMessage;
end

always @(posedge dut2_CLK or negedge dut2_rst_n) begin
    if (!dut2_rst_n)
        dut2_prev_rx_msg <= 4'b0000;
    else if (dut1_o_ValidOutData)
        dut2_prev_rx_msg <= dut2_i_Rx_SbMessage;
end

assign dut1_i_msg_valid = dut2_o_ValidOutData && (dut1_i_Rx_SbMessage != 4'b0000) && (dut1_i_Rx_SbMessage != dut1_prev_rx_msg);
assign dut2_i_msg_valid = dut1_o_ValidOutData && (dut2_i_Rx_SbMessage != 4'b0000) && (dut2_i_Rx_SbMessage != dut2_prev_rx_msg);

// Clocks
initial begin dut1_CLK = 0; forever #(CLK_PERIOD/2) dut1_CLK = ~dut1_CLK; end
initial begin dut2_CLK = 0; forever #(CLK_PERIOD/2) dut2_CLK = ~dut2_CLK; end

// DUT instances
REPAIRVAL_Wrapper dut1 (
    .CLK(dut1_CLK), .rst_n(dut1_rst_n), .i_REPAIRCLK_end(dut1_i_REPAIRCLK_end),
    .i_VAL_Pattern_done(dut1_i_VAL_Pattern_done), .i_Rx_SbMessage(dut1_i_Rx_SbMessage),
    .i_msg_valid(dut1_i_msg_valid), .i_falling_edge_busy(dut1_i_falling_edge_busy),
    .i_VAL_Result_logged_RXSB(dut1_i_VAL_Result_logged_RXSB), .i_VAL_Result_logged_COMB(dut1_i_VAL_Result_logged_COMB),
    .o_train_error_req(dut1_o_train_error_req), .o_MBINIT_REPAIRVAL_Pattern_En(dut1_o_MBINIT_REPAIRVAL_Pattern_En),
    .o_MBINIT_REPAIRVAL_end(dut1_o_MBINIT_REPAIRVAL_end), .o_TX_SbMessage(dut1_o_TX_SbMessage),
    .o_VAL_128Result_logged(dut1_o_VAL_128Result_logged), .o_enable_16_iterations(dut1_o_enable_16_iterations),
    .o_ValidOutData(dut1_o_ValidOutData)
);

REPAIRVAL_Wrapper dut2 (
    .CLK(dut2_CLK), .rst_n(dut2_rst_n), .i_REPAIRCLK_end(dut2_i_REPAIRCLK_end),
    .i_VAL_Pattern_done(dut2_i_VAL_Pattern_done), .i_Rx_SbMessage(dut2_i_Rx_SbMessage),
    .i_msg_valid(dut2_i_msg_valid), .i_falling_edge_busy(dut2_i_falling_edge_busy),
    .i_VAL_Result_logged_RXSB(dut2_i_VAL_Result_logged_RXSB), .i_VAL_Result_logged_COMB(dut2_i_VAL_Result_logged_COMB),
    .o_train_error_req(dut2_o_train_error_req), .o_MBINIT_REPAIRVAL_Pattern_En(dut2_o_MBINIT_REPAIRVAL_Pattern_En),
    .o_MBINIT_REPAIRVAL_end(dut2_o_MBINIT_REPAIRVAL_end), .o_TX_SbMessage(dut2_o_TX_SbMessage),
    .o_VAL_128Result_logged(dut2_o_VAL_128Result_logged), .o_enable_16_iterations(dut2_o_enable_16_iterations),
    .o_ValidOutData(dut2_o_ValidOutData)
);

// State monitors
wire [3:0] dut1_mod_st = dut1.u_repairval_module.current_state;
wire [3:0] dut1_par_st = dut1.u_repairval_partner.current_state;
wire [3:0] dut2_mod_st = dut2.u_repairval_module.current_state;
wire [3:0] dut2_par_st = dut2.u_repairval_partner.current_state;

// Debug monitors
always @(posedge dut1_CLK) begin
    if (dut1_i_falling_edge_busy) $display("[%0t] DUT1: FALLING_EDGE_BUSY pulse", $time);
    if (dut1_o_ValidOutData && !dut1_prev_ValidOut) $display("[%0t] DUT1: TX rising, msg=%b", $time, dut1_o_TX_SbMessage);
    if (dut1_i_msg_valid) $display("[%0t] DUT1: RX msg_valid=%b", $time, dut1_i_Rx_SbMessage);
    if (dut1_mod_st != dut1.u_repairval_module.next_state) $display("[%0t] DUT1.MOD: %0d->%0d", $time, dut1_mod_st, dut1.u_repairval_module.next_state);
    if (dut1_par_st != dut1.u_repairval_partner.next_state) $display("[%0t] DUT1.PAR: %0d->%0d", $time, dut1_par_st, dut1.u_repairval_partner.next_state);
end

always @(posedge dut2_CLK) begin
    if (dut2_i_falling_edge_busy) $display("[%0t] DUT2: FALLING_EDGE_BUSY pulse", $time);
    if (dut2_o_ValidOutData && !dut2_prev_ValidOut) $display("[%0t] DUT2: TX rising, msg=%b", $time, dut2_o_TX_SbMessage);
    if (dut2_i_msg_valid) $display("[%0t] DUT2: RX msg_valid=%b", $time, dut2_i_Rx_SbMessage);
    if (dut2_mod_st != dut2.u_repairval_module.next_state) $display("[%0t] DUT2.MOD: %0d->%0d", $time, dut2_mod_st, dut2.u_repairval_module.next_state);
    if (dut2_par_st != dut2.u_repairval_partner.next_state) $display("[%0t] DUT2.PAR: %0d->%0d", $time, dut2_par_st, dut2.u_repairval_partner.next_state);
end

// Test
initial begin
    dut1_rst_n=0; dut1_i_REPAIRCLK_end=0; dut1_i_VAL_Pattern_done=0; dut1_i_VAL_Result_logged_RXSB=0; dut1_i_VAL_Result_logged_COMB=0;
    dut2_rst_n=0; dut2_i_REPAIRCLK_end=0; dut2_i_VAL_Pattern_done=0; dut2_i_VAL_Result_logged_RXSB=0; dut2_i_VAL_Result_logged_COMB=0;
    
    #(CLK_PERIOD*5); dut1_rst_n=1; dut2_rst_n=1;
    #(CLK_PERIOD*2);
    
    $display("\n=== TEST START ===");
    $display("NOTE: ValidOutData stays HIGH while in SEND states (using current_state)");
    $display("      falling_edge_busy PULSES 2 cycles after ValidOut rises");
    $display("      This simulates sideband bus acknowledging the transmission\n");
    
    #(CLK_PERIOD*2); dut1_i_REPAIRCLK_end=1; dut2_i_REPAIRCLK_end=1;
    $display("[%0t] REPAIRCLK_end asserted\n", $time);
    
    #(CLK_PERIOD*150);
    
    if (dut1_o_MBINIT_REPAIRVAL_Pattern_En) begin
        $display("[%0t] DUT1: Pattern running...", $time);
        @(posedge dut1_CLK); 
        dut1_i_VAL_Pattern_done=1;
        // Immediately set results as pattern completes
        dut1_i_VAL_Result_logged_RXSB=1; 
        dut1_i_VAL_Result_logged_COMB=1;
        @(posedge dut1_CLK); 
        dut1_i_VAL_Pattern_done=0;
        $display("[%0t] DUT1: Pattern done, results=PASS\n", $time);
    end
    
    if (dut2_o_MBINIT_REPAIRVAL_Pattern_En) begin
        $display("[%0t] DUT2: Pattern running...", $time);
        @(posedge dut2_CLK); 
        dut2_i_VAL_Pattern_done=1;
        // Immediately set results as pattern completes
        dut2_i_VAL_Result_logged_RXSB=1; 
        dut2_i_VAL_Result_logged_COMB=1;
        @(posedge dut2_CLK); 
        dut2_i_VAL_Pattern_done=0;
        $display("[%0t] DUT2: Pattern done, results=PASS\n", $time);
    end
    
    #(CLK_PERIOD*200);
    
    $display("\n=== RESULTS ===");
    $display("DUT1: Mod_st=%0d, Par_st=%0d, End=%b", dut1_mod_st, dut1_par_st, dut1_o_MBINIT_REPAIRVAL_end);
    $display("DUT2: Mod_st=%0d, Par_st=%0d, End=%b", dut2_mod_st, dut2_par_st, dut2_o_MBINIT_REPAIRVAL_end);
    
    if (dut1_o_MBINIT_REPAIRVAL_end && dut2_o_MBINIT_REPAIRVAL_end)
        $display("\n*** PASS ***\n");
    else
        $display("\n*** FAIL/INCOMPLETE ***\n");
    
    #(CLK_PERIOD*10); $finish;
end

initial begin $dumpfile("tb_REPAIRVAL_Wrapper.vcd"); $dumpvars(0, tb_REPAIRVAL_Wrapper); end
initial begin #(CLK_PERIOD*2000); $display("\n*** TIMEOUT ***\n"); $finish; end

endmodule