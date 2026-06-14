`timescale 1ns / 1ps

module repair_wrapper_tb;

    // Parameters
    parameter CLK_PERIOD = 10;
    
    // Sideband Messages (Copy from design)
    parameter INIT_REQUEST           = 4'b0001;
    parameter INIT_RESPONSE          = 4'b0010;
    parameter APPLY_DEGRADE_REQUEST  = 4'b0011;
    parameter APPLY_DEGRADE_RESPONSE = 4'b0100;
    parameter END_REQUEST            = 4'b0101;
    parameter END_RESPONSE           = 4'b0110;

    // Inputs to Wrapper
    reg clk;
    reg rst_n;
    reg i_en;
    reg [3:0] i_sideband_message;
    reg i_busy;
    reg i_falling_edge_busy;
    reg [2:0] i_sideband_data_lanes_encoding;
    reg i_sideband_valid;
    reg i_first_8_lanes_are_functional;
    reg i_second_8_lanes_are_functional;

    // Outputs from Wrapper
    wire o_valid;
    wire [2:0] o_sideband_data_lanes_encoding;
    wire [3:0] o_sideband_message;
    wire o_remote_partner_first_8_lanes_result;
    wire o_remote_partner_second_8_lanes_result;
    wire o_test_ack;

    // Instantiate the Unit Under Test (UUT)
    repair_wrapper uut (
        .clk(clk),
        .rst_n(rst_n),
        .i_en(i_en),
        .i_sideband_message(i_sideband_message),
        .i_busy(i_busy),
        .i_falling_edge_busy(i_falling_edge_busy),
        .i_sideband_data_lanes_encoding(i_sideband_data_lanes_encoding),
        .i_sideband_valid(i_sideband_valid),
        .i_first_8_lanes_are_functional(i_first_8_lanes_are_functional),
        .i_second_8_lanes_are_functional(i_second_8_lanes_are_functional),
        .o_valid(o_valid),
        .o_sideband_data_lanes_encoding(o_sideband_data_lanes_encoding),
        .o_sideband_message(o_sideband_message),
        .o_remote_partner_first_8_lanes_result(o_remote_partner_first_8_lanes_result),
        .o_remote_partner_second_8_lanes_result(o_remote_partner_second_8_lanes_result),
        .o_test_ack(o_test_ack)
    );

    // Clock generation
    initial clk = 0;
    always #(CLK_PERIOD/2) clk = ~clk;

    // Stimulus logic
    initial begin
        // 1. Initial Reset
        rst_n = 0;
        i_en = 0;
        i_sideband_message = 0;
        i_busy = 0;
        i_falling_edge_busy = 0;
        i_sideband_data_lanes_encoding = 0;
        i_sideband_valid = 0;
        i_first_8_lanes_are_functional = 1; // Assume all lanes ok
        i_second_8_lanes_are_functional = 1;
        
        #(CLK_PERIOD * 5);
        rst_n = 1;
        #(CLK_PERIOD * 2);

        // 2. Start Repair Procedure
        $display("--- Starting Repair Procedure ---");
        i_en = 1;

        // 3. Wait for INIT_REQUEST and Send INIT_RESPONSE
        wait(o_valid && o_sideband_message == INIT_REQUEST);
        #(CLK_PERIOD * 2); 
        $display("TX sent INIT_REQUEST. Sending INIT_RESPONSE...");
        i_sideband_message = INIT_RESPONSE;
        i_sideband_valid = 1;
        #(CLK_PERIOD);
        i_sideband_valid = 0; // Pulse valid

        // 4. Wait for APPLY_DEGRADE_REQUEST and Send APPLY_DEGRADE_RESPONSE
        wait(o_valid && o_sideband_message == APPLY_DEGRADE_REQUEST);
        $display("TX sent APPLY_DEGRADE_REQUEST. Data Encoding: %b", o_sideband_data_lanes_encoding);
        #(CLK_PERIOD * 2);
        i_sideband_message = APPLY_DEGRADE_RESPONSE;
        i_sideband_data_lanes_encoding = 3'b011; // Remote partner says ok
        i_sideband_valid = 1;
        #(CLK_PERIOD);
        i_sideband_valid = 0;

        // 5. Wait for END_REQUEST and Send END_RESPONSE
        wait(o_valid && o_sideband_message == END_REQUEST);
        $display("TX sent END_REQUEST. Sending END_RESPONSE...");
        #(CLK_PERIOD * 2);
        i_sideband_message = END_RESPONSE;
        i_sideband_valid = 1;
        #(CLK_PERIOD);
        i_sideband_valid = 0;

        // 6. Check for Finish
        wait(o_test_ack);
        $display("--- Repair Procedure Finished Successfully! ---");
        
        #(CLK_PERIOD * 10);
        i_en = 0;
        $finish;
    end

    // Monitor Busy Logic
    initial begin
        #(CLK_PERIOD * 50); // Optional: test busy mid-way
        // i_falling_edge_busy = 1;
        // #(CLK_PERIOD);
        // i_falling_edge_busy = 0;
    end

endmodule