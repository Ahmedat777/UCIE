`timescale 1ns / 1ps

module selfcal_top_tb();

    
    reg clk;
    reg rst_n;
    reg i_en;
    reg [3:0] i_decoded_sideband_message;
    reg i_sideband_valid;
    reg i_falling_edge_busy;

    
    wire [3:0] o_sideband_message;
    wire o_valid;
    wire o_test_ack;

    
    always #5 clk = ~clk;

    
    selfcal_top uut (
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
        $display("System Started...");

        #100; 
        i_decoded_sideband_message = 4'b0001; 
        #50;  
        i_sideband_valid = 1;                 
        #100; 
        i_sideband_valid = 0;                 
        $display("RX received Done_req...");

        
        #100; 
        i_decoded_sideband_message = 4'b0010; 
        #50;
        i_sideband_valid = 1;
        #50;
        i_sideband_valid = 0;
        $display("TX received Done_resp...");

        
        #50;
        
       i_falling_edge_busy = 1; 
        #50;
        
        i_falling_edge_busy = 0; 
        $display("Busy falling edge done!");

        #500;
        if (o_test_ack) $display("SUCCESS! Ack is High.");
        else            $display("FAILED! Check the Waveform.");
        
        
                #200; 
                i_falling_edge_busy = 1; 
                #100;
                i_falling_edge_busy = 0; 
                
                #200;
                if (o_test_ack) 
                    $display("SUCCESS: Test Finished and Ack is High!");
                else 
                    $display("WAIT: Ack is still low, check falling edge logic.");*/
        $finish;
    end

endmodule