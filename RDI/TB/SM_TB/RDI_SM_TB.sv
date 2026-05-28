`timescale 1ms/1ns
module RDI_SM_TB;

import pckg::*;
/*
typedef enum logic [3:0] {
    STS_RDI_RESET = 4'b0000,
    STS_RDI_ACTIVE = 4'b0001,
    STS_LINK_RESET = 4'b1001 ,
    STS_RDI_RETRAIN = 4'b1011,
    STS_LINK_ERROR = 4'b1010,
    STS_RDI_DISABLED = 4'b1100
} rdi_state_sts;

typedef enum logic [3:0] {
    REQ_NOP = 4'b0000,
    REQ_RDI_ACTIVE = 4'b0001,
    REQ_LINK_RESET = 4'b1001 ,
    REQ_RDI_RETRAIN = 4'b1011,
    REQ_RDI_DISABLED = 4'b1100
} rdi_state_req;
*/
logic RDI_clk_TB;
logic rstn_TB;
logic active_handshake_done_TB ;      
logic retrain_req_TB;
logic internal_error_hold_TB;
logic errortime_done_TB;
logic stall_handshake_done_TB;
sb_msg_id sb_msg_id_rdi;

RDI_if#(.NBYTES(64), .NC(32)) RDI_if_TB(
    .clk(RDI_clk_TB),
    .rstn(rstn_TB)
);

RDI_SM #(.NBYTES(64), .NC(32)) RDI_SM_TB(
    .active_handshake_done(active_handshake_done_TB),         
    .retrain_req(retrain_req_TB), 
    .internal_error_hold(internal_error_hold_TB),                   
    .errortime_done(errortime_done_TB),
    .stall_handshake_done(stall_handshake_done_TB),
    .lphy(RDI_if_TB.lphy)
);

initial begin
    RDI_clk_TB = 0;
    forever #0.5 RDI_clk_TB = ~RDI_clk_TB; // 100MHz clock
end

initial begin
    $monitor("Time: %0t, pl_state_sts: %b, lp_state_req: %b", $time, RDI_if_TB.pl_state_sts, RDI_if_TB.lp_state_req);
    rstn_TB = 0;
    RDI_if_TB.lp_state_req = REQ_NOP;
    #1step;
    $display("[%0t] Reset Asserted", $time);
    $display("pl_state_sts", RDI_if_TB.pl_state_sts);
    active_handshake_done_TB=0;
    retrain_req_TB=0;
    internal_error_hold_TB=0;
    @(posedge RDI_clk_TB);
    rstn_TB = 1; 
    #1step;
    $display("[%0t] Reset deasserted", $time);
    @(posedge RDI_clk_TB);
    @(posedge RDI_clk_TB);
    // testing the state transition from NOP to RDI_ACTIVE
    active_handshake_done_TB = 1;
    RDI_if_TB.lp_state_req = REQ_RDI_ACTIVE; // Transition from NOP to RDI_ACTIVE
    @(posedge RDI_clk_TB);
    #1step;
    $display("[%0t] lp_state_req set to REQ_RDI_ACTIVE", $time);
    $display("[%0t] pl_state_req: %b", $time, RDI_if_TB.lp_state_req);
    $display("[%0t] pl_state_sts: %b", $time, RDI_if_TB.pl_state_sts);
    if (RDI_if_TB.pl_state_sts == STS_RDI_ACTIVE) begin
        $display("Transition to STS_RDI_ACTIVE successful");
    end else begin
        $display("Transition to STS_RDI_ACTIVE failed");
    end
    $display("//////////////////////////////////////////////////");
    $display(" testing the state transition from RDI_ACTIVE to LINK_RESET");
    
    RDI_if_TB.lp_state_req = REQ_NOP;
    active_handshake_done_TB = 0;
    @(posedge RDI_clk_TB);
    RDI_if_TB.lp_state_req = REQ_LINK_RESET ;
    active_handshake_done_TB = 1'b1; // Simulate active handshake completion
    @(posedge RDI_clk_TB);
    #1step;
    $display("[%0t] lp_state_req set to REQ_LINK_RESET", $time);
    $display("[%0t] pl_state_req: %b", $time, RDI_if_TB.lp_state_req);
    $display("[%0t] pl_state_sts: %b", $time, RDI_if_TB.pl_state_sts);
    if (RDI_if_TB.pl_state_sts == STS_RDI_ACTIVE) begin
        $display("Transition to STS_RDI_ACTIVE successful");
    end else begin
        $display("Transition to STS_RDI_ACTIVE failed");
    end
    $display("//////////////////////////////////////////////////");
    stall_handshake_done_TB = 1'b1; // Simulate stall handshake completion
    RDI_if_TB.lp_state_req =REQ_LINK_RESET;
    @(posedge RDI_clk_TB);
    #1step;
    if (RDI_if_TB.pl_state_sts == STS_LINK_RESET) begin
        $display("Transition to STS_LINK_RESET successful");
    end else begin
        $display("Transition to STS_LINK_RESET failed");
    end
    $display("//////////////////////////////////////////////////");
   /* $display(" testing the state transition from RDI_ACTIVE to RDI_LINK_RESET");
    RDI_if_TB.lp_state_req = REQ_NOP;
    active_handshake_done_TB = 0;
    @(posedge RDI_clk_TB);
    RDI_if_TB.lp_state_req = REQ_LINK_RESET ;
    active_handshake_done_TB = 1'b1; // Simulate active handshake completion
    @(posedge RDI_clk_TB);
    #1step;
    $display("[%0t] lp_state_req set to REQ_LINK_RESET", $time);
    $display("[%0t] pl_state_req: %b", $time, RDI_if_TB.lp_state_req);
    $display("[%0t] pl_state_sts: %b", $time, RDI_if_TB.pl_state_sts);
    if (RDI_if_TB.pl_state_sts == STS_RDI_ACTIVE) begin
        $display("Transition to STS_RDI_ACTIVE successful");
    end else begin
        $display("Transition to STS_RDI_ACTIVE failed");
    end*/
    /*active_handshake_done_TB = 1;
    @(posedge RDI_clk_TB);
    #1step;
    $display("[%0t] lp_state_req set to REQ_RDI_ACTIVE", $time);
    $display("[%0t] pl_state_req: %b", $time, RDI_if_TB.lp_state_req);
    $display("[%0t] pl_state_sts: %b", $time, RDI_if_TB.pl_state_sts);
    if (RDI_if_TB.pl_state_sts == STS_LINK_RESET) begin
        $display("Transition to STS_LINK_RESET successful");
    end else begin
        $display("Transition to STS_LINK_RESET failed");
    end*/
    $display("//////////////////////////////////////////////////");
    @(posedge RDI_clk_TB);
    RDI_if_TB.lp_state_req = REQ_RDI_ACTIVE;
    stall_handshake_done_TB = 1'b0; // Ensure stall handshake is not done to test active handshake path
    @(posedge RDI_clk_TB);
    #1step;
    $display("[%0t]  pl_state_req: %b", $time, RDI_if_TB.lp_state_req);
    $display("[%0t] pl_state_sts: %b", $time, RDI_if_TB.pl_state_sts);
    RDI_if_TB.lp_state_req = REQ_NOP;
    @(posedge RDI_clk_TB);
    $display("[%0t] pl_state_req: %b", $time, RDI_if_TB.lp_state_req);
    $display("[%0t] pl_state_sts: %b", $time, RDI_if_TB.pl_state_sts);
    active_handshake_done_TB = 0;
    @(posedge RDI_clk_TB);
    RDI_if_TB.lp_state_req = REQ_RDI_DISABLED ;
    active_handshake_done_TB = 1'b1;
    @(posedge RDI_clk_TB);
    #1step;
    $display("[%0t] lp_state_req set to REQ_RDI_DISABLED", $time);
    $display("[%0t] pl_state_req: %b", $time, RDI_if_TB.lp_state_req);
    $display("[%0t] pl_state_sts: %b", $time, RDI_if_TB.pl_state_sts);
    if (RDI_if_TB.pl_state_sts == STS_RDI_ACTIVE) begin
        $display(" Transition to STS_RDI_ACTIVE successful");
    end else begin
        $display("Transition to STS_RDI_ACTIVE failed");
    end
    stall_handshake_done_TB = 1'b1; // Simulate stall handshake completion
    @(posedge RDI_clk_TB);
    #1step;
    if (RDI_if_TB.pl_state_sts == STS_RDI_DISABLED) begin
        $display("Transition to STS_RDI_DISABLED successful");
    end else begin
        $display("Transition to STS_RDI_DISABLED failed");
    end
    @(posedge RDI_clk_TB);
    RDI_if_TB.lp_state_req = REQ_RDI_ACTIVE;
    @(posedge RDI_clk_TB);
    #1step;
    $display("[%0t] pl_state_req: %b", $time, RDI_if_TB.lp_state_req);
    $display("[%0t] pl_state_sts: %b", $time, RDI_if_TB.pl_state_sts);

    $display("**************//////////////////////////////////////////////////*************");
    @(posedge RDI_clk_TB);
    RDI_if_TB.lp_state_req = REQ_RDI_ACTIVE;
    active_handshake_done_TB = 0;
    @(posedge RDI_clk_TB);
    RDI_if_TB.lp_state_req = REQ_NOP;
    @(posedge RDI_clk_TB);
    $display("[%0t] pl_state_req: %b", $time, RDI_if_TB.lp_state_req);
    $display("[%0t] pl_state_sts: %b", $time, RDI_if_TB.pl_state_sts);
    @(posedge RDI_clk_TB);
    RDI_if_TB.lp_state_req = REQ_RDI_ACTIVE ;
    active_handshake_done_TB = 1'b1;
    @(posedge RDI_clk_TB);
    #1step;
    $display("[%0t] lp_state_req set to REQ_RDI_RETRAIN", $time);
    $display("[%0t] pl_state_req: %b", $time, RDI_if_TB.lp_state_req);
    $display("[%0t] pl_state_sts: %b", $time, RDI_if_TB.pl_state_sts);
    if (RDI_if_TB.pl_state_sts == STS_RDI_ACTIVE) begin
        $display("hereeeee Transition to STS_RDI_ACTIVE successful");
    end else begin
        $display("Transition to STS_RDI_ACTIVE failed");
    end
    RDI_if_TB.lp_state_req = REQ_RDI_RETRAIN;
    @(posedge RDI_clk_TB);
    #1step;
    $display("[%0t] pl_state_req: %b", $time, RDI_if_TB.lp_state_req);
    $display("[%0t] pl_state_sts: %b", $time, RDI_if_TB.pl_state_sts);
    if (RDI_if_TB.pl_state_sts == STS_RDI_RETRAIN) begin
        $display("Transition to STS_RDI_RETRAIN successful");
    end else begin
        $display("Transition to STS_RDI_RETRAIN failed");
    end
    $display("//////////////////////////////////////////////////");
    $display(" testing the state transition from RDI_ACTIVE to RDI_RETRAIN");
    RDI_if_TB.pl_state_sts = STS_RDI_ACTIVE;
    RDI_if_TB.lp_state_req = REQ_RDI_RETRAIN;
    @(posedge RDI_clk_TB);
    #1step;
    $display("[%0t] pl_state_req: %b", $time, RDI_if_TB.lp_state_req);
    $display("[%0t] pl_state_sts: %b", $time, RDI_if_TB.pl_state_sts);
    if (RDI_if_TB.pl_state_sts == STS_RDI_RETRAIN) begin
        $display("Transition to STS_RDI_RETRAIN successful");
    end else begin
        $display("Transition to STS_RDI_RETRAIN failed");
    end
    $display("//////////////////////////////////////////////////");
    @(posedge RDI_clk_TB);
    RDI_if_TB.pl_state_sts = STS_RDI_RETRAIN;
    RDI_if_TB.lp_state_req = REQ_NOP;
    @(posedge RDI_clk_TB);
    RDI_if_TB.lp_state_req = REQ_RDI_ACTIVE;
    active_handshake_done_TB = 1;
    @(posedge RDI_clk_TB);
    #1step;
    $display("[%0t] pl_state_req: %b", $time, RDI_if_TB.lp_state_req);
    $display("[%0t] pl_state_sts: %b", $time, RDI_if_TB.pl_state_sts);
    if (RDI_if_TB.pl_state_sts == STS_RDI_ACTIVE)
    $display("Transition to STS_RDI_ACTIVE successful");
    else
    $display("Transition to STS_RDI_ACTIVE failed");
    
    $display("//////////////////////////////////////////////////");
    @(posedge RDI_clk_TB);
    stall_handshake_done_TB = 1'b1;
    RDI_if_TB.lp_state_req = REQ_RDI_RETRAIN;
    @(posedge RDI_clk_TB);
    #1step;
    $display("[%0t] pl_state_req: %b", $time, RDI_if_TB.lp_state_req);
    $display("[%0t] pl_state_sts: %b", $time, RDI_if_TB.pl_state_sts);
    if (RDI_if_TB.pl_state_sts == STS_RDI_RETRAIN) begin
        $display("Transition to STS_RDI_RETRAIN successful");
    end else begin
        $display("Transition to STS_RDI_RETRAIN failed");
    end
    $display("//////////////////////////////////////////////////");
    @(posedge RDI_clk_TB);
    RDI_if_TB.lp_state_req = REQ_LINK_RESET;
    stall_handshake_done_TB = 1'b1; // Ensure handshake is not done to test link error path
    @(posedge RDI_clk_TB);
    stall_handshake_done_TB = 1'b1; // Simulate stall handshake completion
    sb_msg_id_rdi = SB_RDI_REQ_LINKRESET; // Simulate sideband message for link reset
    @(posedge RDI_clk_TB);
    #1step;
    $display("[%0t] pl_state_req: %b", $time, RDI_if_TB.lp_state_req);
    $display("[%0t] pl_state_sts: %b", $time, RDI_if_TB.pl_state_sts);
    if (RDI_if_TB.pl_state_sts == STS_LINK_RESET) begin
        $display("Transition to STS_LINK_RESET successful");
    end else begin
        $display("Transition to STS_LINK_RESET failed");
    end
    $display("//////////////////////////////////////////////////");
    @(posedge RDI_clk_TB);
    RDI_if_TB.pl_state_sts = STS_RDI_DISABLED;
    RDI_if_TB.lp_state_req = REQ_RDI_ACTIVE;
    @(posedge RDI_clk_TB);
    #1step;
    $display("[%0t] pl_state_req: %b", $time, RDI_if_TB.lp_state_req);
    $display("[%0t] pl_state_sts: %b", $time, RDI_if_TB.pl_state_sts);
    if (RDI_if_TB.pl_state_sts == STS_RDI_RESET) begin
        $display("Transition to STS_RDI_RESET successful");
    end else begin
        $display("Transition to STS_RDI_RESET failed");
    end
    $display("//////////////////////////////////////////////////");
    @(posedge RDI_clk_TB);
    RDI_if_TB.lp_linkerror = 1'b1;
    @(posedge RDI_clk_TB);
    #1step;
    if (RDI_if_TB.pl_state_sts == STS_LINK_ERROR) begin
        $display("Transition to STS_LINK_ERROR successful");
    end else begin
        $display("Transition to STS_LINK_ERROR failed");
    end
    $display("//////////////////////////////////////////////////");
    @(posedge RDI_clk_TB);
    RDI_if_TB.lp_state_req = REQ_RDI_ACTIVE;
    internal_error_hold_TB = 1'b0;
    RDI_if_TB.lp_linkerror = 1'b0;
    repeat(20) @(posedge RDI_clk_TB); 
    @(posedge RDI_clk_TB);
    #1step;
    $display("[%0t] pl_state_req: %b", $time, RDI_if_TB.lp_state_req);
    $display("[%0t] pl_state_sts: %b", $time, RDI_if_TB.pl_state_sts);
    $display("[%0t] error time done: %b", $time, errortime_done_TB);
    if (RDI_if_TB.pl_state_sts == STS_RDI_RESET) begin
        $display("Transition to STS_RDI_RESET successful");
    end else begin
        $display("Transition to STS_RDI_RESET failed");
    end
    
end
endmodule 