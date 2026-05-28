module tb_stall_phy;

    // Parameters
    localparam CLK_PERIOD = 10; // 100 MHz for simulation
    // RDI State Encodings from Table 10-1
    typedef enum logic [3:0] {
    RDI_RESET = 4'b0000,
    RDI_ACTIVE = 4'b0001,
    LINK_RESET = 4'b1001 ,
    RDI_RETRAIN = 4'b1011,
    LINK_ERROR = 4'b1010
    } rdi_state_e;

    // Signals
    logic tb_rdi_clk;
    logic free_running_clk;
    logic tb_rstn;
    logic tb_handshake_done;

RDI_if #(.NBYTES(64),.NC(32)) RDI_if_inst (
    .clk(tb_rdi_clk),
    .rstn(tb_rstn)
);

PHY_RDI_SM #() dut (
    .RDI_clk(free_running_clk),
    .SM(RDI_if_inst.lphy), // PHY uses lphy modport
    .handshake_done(tb_handshake_done)
);



initial begin
    free_running_clk = 0;
    forever #(CLK_PERIOD/2) free_running_clk = ~free_running_clk;
end

initial begin
    tb_rstn = 0;
    RDI_if_inst.lp_stallack = 0;
    RDI_if_inst.pl_state_sts = RDI_RESET;
    repeat(2) @(posedge free_running_clk);
    tb_rstn = 1;
    $display("[%0t] Reset De-asserted", $time);
    // --- TEST CASE 1: Trigger Stall Request in Active State ---
    @(posedge free_running_clk);
    RDI_if_inst.pl_state_sts = RDI_ACTIVE;
    RDI_if_inst.lp_state_req = RDI_RETRAIN; // Trigger condition for stall
    @(posedge free_running_clk);
    #1step;
    if (RDI_if_inst.pl_stallreq !== 1 || tb_handshake_done !== 0)
        $error($time, "Test case 1 faied: pl_req = 0, expected 1");
    else
        $display($time, "Test case 1 done, pl_stallreq = %b , pl_trdy = %b, handshake_done = %b"
        ,RDI_if_inst.pl_stallreq, RDI_if_inst.pl_trdy, tb_handshake_done);

// test case 2 : test recieving stallack 
    @(posedge free_running_clk);
    RDI_if_inst.lp_stallack = 1; // Acknowledge the stall request
    @(posedge free_running_clk);
    #1step;
    if (RDI_if_inst.pl_trdy !== 0 || tb_handshake_done !== 0)
        $error($time, "Test case 2 failed: pl_trdy = 1, expected 0");
    else
        $display($time, "Test case 2 done, pl_stallreq = %b , pl_trdy = %b, handshake_done = %b"
        ,RDI_if_inst.pl_stallreq, RDI_if_inst.pl_trdy, tb_handshake_done);
    // test case 3 : test reqdeassert state
    @(posedge free_running_clk);
    #1step;
    if (RDI_if_inst.pl_stallreq !== 0 || tb_handshake_done !== 0)
        $error($time, "Test case 3 failed: pl_stallreq = 1, expected 0");
    else
    begin 
        $display($time, "Test case 3 done, pl_stallreq = %b , pl_trdy = %b, handshake_done = %b"
        ,RDI_if_inst.pl_stallreq, RDI_if_inst.pl_trdy, tb_handshake_done);
    end
// test case 4 : test deasserting stallreq
    @(posedge free_running_clk);
    RDI_if_inst.lp_stallack = 0; // De-assert the stall acknowledge
    @(posedge free_running_clk);
    #1step;
    if (RDI_if_inst.pl_stallreq !== 0 || tb_handshake_done !== 1)
        $error($time, "Test case 3 failed: pl_stallreq = 1, expected 0");
    else
    begin 
        $display($time, "Test case 3 done, pl_stallreq = %b , pl_trdy = %b, handshake_done = %b"
        ,RDI_if_inst.pl_stallreq, RDI_if_inst.pl_trdy, tb_handshake_done);
    end
    $display("[%0t] Testbench completed", $time);
end


endmodule