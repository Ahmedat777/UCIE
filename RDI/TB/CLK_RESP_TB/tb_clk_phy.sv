module tb_clk_phy;
parameter NC = 32;
logic tb_clk;
logic tb_rstn;
logic tb_ltsm_training_done    ;
logic tb_internal_sb_pending   ;
logic tb_internal_link_error_req;
logic [4*NC-1:0] tb_internal_sb_data;

RDI_if rdi_if_inst(tb_clk, tb_rstn);

clk_phy dut (
    .clk(tb_clk),
    .internal_sb_data(tb_internal_sb_data),
    .internal_sb_pending(tb_internal_sb_pending),
    .internal_link_error_req(tb_internal_link_error_req),
    .ltsm_training_done(tb_ltsm_training_done),
    .inst(rdi_if_inst.lphy)
);

typedef enum logic [3:0] {
    RDI_RESET = 4'b0000,
    RDI_ACTIVE = 4'b0001,
    LINK_RESET = 4'b1001 ,
    RDI_RETRAIN = 4'b1011,
    LINK_ERROR = 4'b1010
    } rdi_state_e;


// Clock Generation
initial begin
    tb_clk = 0;
    forever #5 tb_clk = ~tb_clk; // 100MHz clock
end

// Test Stimulus
initial begin
    // --- Initialization ---
    tb_rstn = 0;
    tb_ltsm_training_done = 0;
    tb_internal_sb_pending = 0;
    tb_internal_link_error_req = 0;
    tb_internal_sb_data = 32'hDEADBEEF; // Example data
    rdi_if_inst.pl_state_sts = RDI_RESET; // Initial state;


    repeat(5) @(posedge tb_clk);
    tb_rstn = 1;
    $display("[%0t] Reset De-asserted", $time);

    // --- Test Case 1: Triggering clk_req ---
    @(posedge tb_clk);
    rdi_if_inst.pl_state_sts = RDI_ACTIVE;
    @(posedge tb_clk);

    if (rdi_if_inst.pl_clk_req !== 1) 
        $error("Expected pl_clk_req to be 1, but got %b", rdi_if_inst.pl_clk_req);
    else
        $display("[%0t] pl_clk_req correctly asserted", $time);
    // Trigger clk_req by setting internal_sb_pending
    $display("[%0t] Triggering clk_req with internal_sb_pending", $time);

    tb_internal_sb_pending = 1;
    @(posedge tb_clk);
    rdi_if_inst.lp_clk_ack = 1;
    @(posedge tb_clk);

    
   

    #1step; // Small delay to allow signals to propagate
    if (rdi_if_inst.pl_cfg == tb_internal_sb_data[NC-1:0] && rdi_if_inst.pl_cfg_vld == 1'b1) begin
        $display("[%0t] pl_cfg and pl_cfg_vld correctly set for internal_sb_pending", $time);
    end else begin
        $error("pl_cfg or pl_cfg_vld not set correctly for internal_sb_pending. pl_cfg: %h, pl_cfg_vld: %b", $time, rdi_if_inst.pl_cfg, rdi_if_inst.pl_cfg_vld);
    end
    @(posedge tb_clk);
    #1step; // Small delay to allow signals to propagate
    if (rdi_if_inst.pl_cfg == tb_internal_sb_data[2*NC-1:NC] && rdi_if_inst.pl_cfg_vld == 1'b1) begin
        $display("[%0t] pl_cfg and pl_cfg_vld correctly set for internal_sb_pending", $time);
    end else begin
        $error("pl_cfg or pl_cfg_vld not set correctly for internal_sb_pending. pl_cfg: %h, pl_cfg_vld: %b", $time, rdi_if_inst.pl_cfg, rdi_if_inst.pl_cfg_vld);
    end
    @(posedge tb_clk);
    #1step; // Small delay to allow signals to propagate
    if (rdi_if_inst.pl_cfg == tb_internal_sb_data[3*NC-1:2*NC] && rdi_if_inst.pl_cfg_vld == 1'b1) begin
        $display("[%0t] pl_cfg and pl_cfg_vld correctly set for internal_sb_pending", $time);
    end else begin
        $error("pl_cfg or pl_cfg_vld not set correctly for internal_sb_pending. pl_cfg: %h, pl_cfg_vld: %b", $time, rdi_if_inst.pl_cfg, rdi_if_inst.pl_cfg_vld);
    end

     @(posedge tb_clk);
    // testing pl_inband_pres
    @(posedge tb_clk);
    tb_ltsm_training_done = 1'b1;
    @(posedge tb_clk);
    #1step; // Small delay to allow signals to propagate
    if (rdi_if_inst.pl_inband_pres !== 1) 
        $error("Expected pl_inband_pres to be 1 after clk_ack, but got %b", rdi_if_inst.pl_inband_pres);
    else
        $display("[%0t] pl_inband_pres correctly asserted after clk_ack", $time);

    @(posedge tb_clk);
    // test clk_req de-assertion
    
    tb_ltsm_training_done = 0;
    tb_internal_sb_pending = 0;
    @(posedge tb_clk);
    rdi_if_inst.pl_state_sts = RDI_RESET;
    #1step; // Small delay to allow signals to propagate
    if (rdi_if_inst.pl_clk_req !== 0) 
        $error("Expected pl_clk_req to be 0 after de-assertion, but got %b", rdi_if_inst.pl_clk_req);
    else
        $display("[%0t] pl_clk_req correctly de-asserted", $time);
    @(posedge tb_clk);
    #1step; // Small delay to allow signals to propagate
     if (rdi_if_inst.pl_inband_pres !== 0) 
        $error("Expected pl_inband_pres to be 0 after de-assertion, but got %b", rdi_if_inst.pl_inband_pres);
    else
        $display("[%0t] pl_inband_pres correctly de-asserted", $time);

end
    


endmodule 