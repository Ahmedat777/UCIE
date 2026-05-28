`timescale 1ns/1ps
import pckg::*;
module SBINIT_tb;

    // Parameters
    parameter SB_MSG_Width = 4;
    parameter CLK_PERIOD = 10;
    
    //==========================================================================
    // Signal Declarations
    //==========================================================================
    
    // Clock and Reset
    logic                       clk;
    logic                       rst_n;
    
    // Control Signals
    logic                       sbinit_en;
    logic                       pattern_done;
    
    // Sideband Interface - Receive
    logic                       rx_msg_valid;
    sb_msg_id    decoded_sb_msg;
    
    // Sideband Interface - Busy Management
    logic                       sb_busy;
    logic                       falling_edge_busy;
    
    // SBINIT Outputs
    wire                        start_pattern_req;
    wire                        msg_valid;
   wire sb_msg_id     encoded_SB_msg;
    wire                        SBINIT_end;
    
    //==========================================================================
    // SBINIT Module Instantiation
    //==========================================================================
    
    SBINIT #(
        .SB_MSG_Width(SB_MSG_Width)
    ) u_sbinit (
        // Clock and Reset
        .i_clk                  (clk),
        .i_rst_n                (rst_n),
        
        // Control
        .i_sbinit_en            (sbinit_en),
        .i_pattern_done         (pattern_done),
        
        // Receive Interface
        .i_sb_valid         (rx_msg_valid),
        .i_decoded_sb_msg       (decoded_sb_msg),
        
        // Busy Management
        .i_sb_busy              (sb_busy),
        .i_falling_edge_busy    (falling_edge_busy),
        
        // Outputs
        .o_start_pattern_req    (start_pattern_req),
        .o_msg_valid            (msg_valid),
        .o_encoded_SB_msg       (encoded_SB_msg),
        .o_SBINIT_end           (SBINIT_end)
    );
    
    //==========================================================================
    // Clock Generation
    //==========================================================================
    
    always #(CLK_PERIOD/2) clk = ~clk;

    //============================================================================
    // =================== message encodings  ===================================
    //==========================================================================
    /*localparam [SB_MSG_Width-1:0]   SBINIT_Out_of_Reset_msg = 4'd3 ,
                                    SBINIT_done_req_msg     = 4'd1 ,
                                    SBINIT_done_resp_msg    = 4'd2 ;*/
    
    //===========================================================================
    //              TASKS
    //==========================================================================
    task reset;
        rst_n = 0;
        sbinit_en = 0;
        pattern_done = 0;
        rx_msg_valid = 0;
        decoded_sb_msg = sb_msg_id'(0);
        sb_busy = 0;
        falling_edge_busy = 0;
        
        // Reset sequence
        #(CLK_PERIOD );
        rst_n = 1;
    endtask

    task automatic wait_for_msg;
    input   sb_msg_id expected_msg;
    input  string                  msg_name;      // optional for display
    
    bit timeout_flag;
    bit condition_flag;
    begin
    @(posedge clk);    
    fork
        // Timeout counter (3 clock cycles)
        begin
            timeout_flag = 0;
            repeat (3) @(posedge clk);
            timeout_flag = 1;
        end
        // Wait for the expected message
        begin
            condition_flag = 0;
            wait (encoded_SB_msg == expected_msg && msg_valid);
            condition_flag = 1;
        end
    join_any
    disable fork; // Stop the other branch
    if (condition_flag) begin
        $display("[TIME %0t] PASS: %s sent (0x%0h)", $time, msg_name, expected_msg);
        
    end
    else if (timeout_flag) begin
        $display("[TIME %0t] ERROR: Timeout waiting for %s (0x%0h)", $time, msg_name, expected_msg);
       
    end
end
endtask

task complete_tx_();
        begin
            sb_busy = 1;
            #(CLK_PERIOD);
            falling_edge_busy = 1;
            sb_busy = 0;
            $display("[TIME: %0t]falling edge busy is high",$time);
            #CLK_PERIOD;
            falling_edge_busy = 0;   
        end
    endtask
    

    
    initial begin
        // Initialize signals
        clk = 0;
       
        reset();
    //=========================================================
    //                  SCENARIO 1
    //=========================================================

        #(CLK_PERIOD);
        $display("scenario 1 : done req is received while sending the same msg and OOR is received late ");
        sbinit_en = 1;
        
        // Wait for pattern request
        wait(start_pattern_req);
        $display("[%0t] Pattern request detected", $time);
        
        // Simulate pattern completion
        #(CLK_PERIOD * 5);
        $display("[%0t] Pattern done", $time);
        pattern_done = 1;
        #(CLK_PERIOD);
        pattern_done = 0;
        

        wait_for_msg (SB_SBINIT_OUT_OF_RESET , "OOR") ;
        #(CLK_PERIOD * 2);
        complete_tx_();
        // Simulate receiving OOR message from peer
        #(CLK_PERIOD * 5);
        $display("[%0t] SB_SBINIT_OUT_OF_RESET is received", $time);
        rx_msg_valid = 1;
        decoded_sb_msg = SB_SBINIT_OUT_OF_RESET ; // OOR message
        #(CLK_PERIOD);
        rx_msg_valid = 0;
        fork
           begin
            wait_for_msg (SB_SBINIT_DONE_REQ , "DONE_REQ") ;
           end 
            begin 
                complete_tx_();
            end
            begin
                $display("[%0t] SB_SBINIT_DONE_REQ is received", $time);
                rx_msg_valid = 1;
                decoded_sb_msg = SB_SBINIT_DONE_REQ ; // done req message
                #(CLK_PERIOD);
                rx_msg_valid = 0;
            end
        join_any
        
        wait_for_msg (SB_SBINIT_DONE_RESP , "DONE_RESP") ;
        complete_tx_();

        $display("[%0t] SB_SBINIT_DONE_RESP is received", $time);
        rx_msg_valid = 1;
        decoded_sb_msg = SB_SBINIT_DONE_RESP ; // done resp message
        #(CLK_PERIOD);
        rx_msg_valid = 0;

        // Wait for SBINIT completion
        wait(SBINIT_end);
        $display("[%0t] SBINIT completed successfully", $time);
        sbinit_en = 0 ;
        #(CLK_PERIOD * 5);

    //=========================================================
    //                  SCENARIO 2
    //=========================================================

        #(CLK_PERIOD);
        $display("scenario 2 : while waiting for done resp in the end state and oor received early");
        sbinit_en = 1;
        
        // Wait for pattern request
        wait(start_pattern_req);
        $display("[%0t] Pattern request detected", $time);
        
        // Simulate pattern completion
        #(CLK_PERIOD * 5);
        $display("[%0t] Pattern done", $time);
        pattern_done = 1;
        #(CLK_PERIOD);
        pattern_done = 0;
        
        $display("[%0t] SB_SBINIT_OUT_OF_RESET is received", $time);
        rx_msg_valid = 1;
        decoded_sb_msg = SB_SBINIT_OUT_OF_RESET ; // OOR message
        #(CLK_PERIOD);
        rx_msg_valid = 0;

        wait_for_msg (SB_SBINIT_OUT_OF_RESET , "OOR") ;
        #(CLK_PERIOD * 2);
        complete_tx_();
        // Simulate receiving OOR message from peer
        
        
        wait_for_msg (SB_SBINIT_DONE_REQ , "DONE_REQ") ;
        complete_tx_();

        $display("[%0t] SB_SBINIT_DONE_RESP is received", $time);
        rx_msg_valid = 1;
        decoded_sb_msg = SB_SBINIT_DONE_RESP ; // done resp message
        #(CLK_PERIOD);
        rx_msg_valid = 0;

        #(CLK_PERIOD *2 );
        $display("[%0t] SB_SBINIT_DONE_REQ is received", $time);
                rx_msg_valid = 1;
                decoded_sb_msg = SB_SBINIT_DONE_REQ ; // done req message
                #(CLK_PERIOD);
                rx_msg_valid = 0;
            wait_for_msg (SB_SBINIT_DONE_RESP , "DONE_RESP") ;
            complete_tx_();
    
        
        // Wait for SBINIT completion
        wait(SBINIT_end);  $display("[%0t] SBINIT completed successfully", $time);
         #(CLK_PERIOD );
        sbinit_en = 0 ;
        #(CLK_PERIOD * 5);

    //=========================================================
    //                  SCENARIO3 
    //=========================================================

        #(CLK_PERIOD);
        $display("scenario 3 : done req is received right after oor ");
        sbinit_en = 1;
        
        // Wait for pattern request
        wait(start_pattern_req);
        $display("[%0t] Pattern request detected", $time);
        
        // Simulate pattern completion
        #(CLK_PERIOD * 5);
        $display("[%0t] Pattern done", $time);
        pattern_done = 1;
        #(CLK_PERIOD);
        pattern_done = 0;

        $display("[%0t] SB_SBINIT_OUT_OF_RESET is received", $time);
        rx_msg_valid = 1;
        decoded_sb_msg = SB_SBINIT_OUT_OF_RESET ; // OOR message
        #(CLK_PERIOD);
        rx_msg_valid = 0;
        fork
            begin
                wait_for_msg (SB_SBINIT_OUT_OF_RESET , "OOR") ;
                complete_tx_();
            end
            begin
              complete_tx_();  
            end
           begin
            $display("[%0t] SB_SBINIT_DONE_REQ is received", $time);
                rx_msg_valid = 1;
                decoded_sb_msg = SB_SBINIT_DONE_REQ ; // done req message
                #(CLK_PERIOD);
                rx_msg_valid = 0;
            wait_for_msg (SB_SBINIT_DONE_RESP , "DONE_RESP") ;
            complete_tx_();
           end  
        join
        
        // Simulate receiving OOR message from peer
        
        
        wait_for_msg (SB_SBINIT_DONE_REQ , "DONE_REQ") ;
        complete_tx_();

        $display("[%0t] SB_SBINIT_DONE_RESP is received", $time);
        rx_msg_valid = 1;
        decoded_sb_msg = SB_SBINIT_DONE_RESP ; // done resp message
        #(CLK_PERIOD);
        rx_msg_valid = 0;
        
        // Wait for SBINIT completion
        wait(SBINIT_end);  $display("[%0t] SBINIT completed successfully", $time);
         #(CLK_PERIOD );
        sbinit_en = 0 ;
        #(CLK_PERIOD * 5);



        $stop;
    end

    /*initial begin
        #500000000 ;
        $display("TIMEOUT") ;
        $stop ;
    end*/

endmodule

