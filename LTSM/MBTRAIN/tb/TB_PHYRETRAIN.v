`timescale 1ns/1ps

module TB_PHYRETRAIN();

    // Inputs to Wrapper
    reg         clk;
    reg         rst_n;
    reg         i_phyretrain_en;
    reg         i_enter_from_active_or_mbtrain;
    reg  [1:0]  i_linkspeed_lanes_status;
    reg         i_clear_resolved_state;
    reg         i_SB_Busy;
    reg         i_falling_edge_busy;
    reg         i_rx_msg_valid;
    reg  [2:0]  i_rx_msg_info;
    reg  [3:0]  i_decoded_SB_msg;

    // Outputs from Wrapper
    wire [3:0]  o_encoded_SB_msg;
    wire        o_tx_msg_valid;
    wire [2:0]  o_tx_msg_info;
    wire        o_PHYRETRAIN_end;
    wire [1:0]  o_resolved_state;

    // Instantiate the Wrapper
    PHYRETRAIN_WRAPPER dut (
        .i_clk(clk),
        .i_rst_n(rst_n),
        .i_phyretrain_en(i_phyretrain_en),
        .i_enter_from_active_or_mbtrain(i_enter_from_active_or_mbtrain),
        .i_linkspeed_lanes_status(i_linkspeed_lanes_status),
        .i_clear_resolved_state(i_clear_resolved_state),
        .i_SB_Busy(i_SB_Busy),
        .i_falling_edge_busy(i_falling_edge_busy),
        .i_rx_msg_valid(i_rx_msg_valid),
        .i_rx_msg_info(i_rx_msg_info),
        .i_decoded_SB_msg(i_decoded_SB_msg),
        .o_encoded_SB_msg(o_encoded_SB_msg),
        .o_tx_msg_valid(o_tx_msg_valid),
        .o_tx_msg_info(o_tx_msg_info),
        .o_PHYRETRAIN_end(o_PHYRETRAIN_end),
        .o_resolved_state(o_resolved_state)
    );

    // Clock Generation
    initial clk = 0;
    always #5 clk = ~clk; 

 
    initial begin
        // Reset and init
        initialize_signals();
        reset_system();

        $display("--- Starting Scenario 1: Local Device Initiate ---");
        i_phyretrain_en = 1;
        i_enter_from_active_or_mbtrain = 0; // Active state
        
        
        wait(o_tx_msg_valid && o_encoded_SB_msg == 4'b0001);
        $display("TX sent REQ at time %t", $time);
        
      
        repeat(5) @(posedge clk);
        i_decoded_SB_msg = 4'b0010; // RESP
        i_rx_msg_valid   = 1'b1;
        i_rx_msg_info    = 3'b001;  // Partner wants TXSELFCAL
        $display("Partner sent RESP at time %t", $time);


        wait(dut.wp_rx_valid == 1'b1);
        $display("DUT RX is now in RESP state at time %t", $time);
        
   
        repeat(3) @(posedge clk);
        i_falling_edge_busy = 1'b1;
        @(posedge clk);
        i_falling_edge_busy = 1'b0;
        $display("Falling edge busy pulse sent.");

        // 5. 
        wait(o_PHYRETRAIN_end);
        $display("--- SUCCESS: Retraining Finished! ---");
        $display("Resolved State = %b", o_resolved_state);
        
        #100;
        $stop;
    end

    // --- Tasks ---
    task initialize_signals();
        begin
            i_phyretrain_en = 0;
            i_enter_from_active_or_mbtrain = 0;
            i_linkspeed_lanes_status = 0;
            i_clear_resolved_state = 0;
            i_SB_Busy = 0;
            i_falling_edge_busy = 0;
            i_rx_msg_valid = 0;
            i_rx_msg_info = 0;
            i_decoded_SB_msg = 0;
        end
    endtask

    task reset_system();
        begin
            rst_n = 0;
            #20 rst_n = 1;
            #10;
        end
    endtask

endmodule