`timescale 1ns / 1ps

module rx_cal_Top_tb();

    reg clk;
    reg rst_n;
    reg i_en;
    reg [3:0] i_decoded_sideband_message;
    reg i_sideband_valid;
    reg i_falling_edge_busy;

    wire [3:0] o_sideband_message;
    wire o_valid;
    wire o_test_ack;

    always #5 clk = (clk === 1'b0); // Clock generation

    rx_cal_Top uut (
        .clk(clk), .rst_n(rst_n), .i_en(i_en),
        .i_decoded_sideband_message(i_decoded_sideband_message),
        .i_sideband_valid(i_sideband_valid),
        .i_falling_edge_busy(i_falling_edge_busy),
        .o_sideband_message(o_sideband_message),
        .o_valid(o_valid), .o_test_ack(o_test_ack)
    );

    initial begin
       
        clk = 0; rst_n = 0; i_en = 0;
        i_decoded_sideband_message = 0;
        i_sideband_valid = 0;
        i_falling_edge_busy = 0;

        
        #50 rst_n = 1;
        #50 i_en = 1;
        $display("[%t] System Started...", $time);

        
        #100;
        i_decoded_sideband_message = 4'b0001; 
        i_sideband_valid = 1;
        #10 i_sideband_valid = 0;
        $display("[%t] START_REQ simulated.", $time);

        #100;
        i_decoded_sideband_message = 4'b0010; 
        i_sideband_valid = 1;
        #10 i_sideband_valid = 0;
        $display("[%t] START_RESP simulated.", $time);

        
        #200; 
        i_decoded_sideband_message = 4'b0011; 
        i_sideband_valid = 1;
        #10 i_sideband_valid = 0;
        $display("[%t] DONE_REQ simulated.", $time);

        #100;
        i_decoded_sideband_message = 4'b0100; 
        i_sideband_valid = 1;
        #10 i_sideband_valid = 0;
        $display("[%t] DONE_RESP simulated.", $time);

        
        #100;
        i_falling_edge_busy = 1; 
        #50;
        i_falling_edge_busy = 0; // Falling Edge detected!
        $display("[%t] Busy Falling Edge detected. Waiting for Ack...", $time);

        
        #200;
        if (o_test_ack) 
            $display("[%t] SUCCESS! Ack is High.", $time);
        else 
            $display("[%t] FAILED! Ack is still Low.", $time);
        
        #100;
        $finish;
    end
endmodule