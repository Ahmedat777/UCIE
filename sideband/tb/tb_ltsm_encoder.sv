`timescale 1ps/1ps
import pckg::*;

module tb_ltsm_encoder;

  logic clk;
  logic rst;
  logic i_ltsm_valid;
  sb_msg_id i_msg_id;
  logic [63:0] ltsm_data;
  logic [15:0] ltsm_msginfo;

  logic o_ltsmpckt_valid;
  logic [63:0] o_ltsm_header;
  logic [63:0] o_ltsm_data;

  ltsm_encoder dut (
    .clk(clk),
    .rst(rst),
    .i_ltsm_valid(i_ltsm_valid),
    .i_msg_id(i_msg_id),
    .ltsm_data(ltsm_data),
    .ltsm_msginfo(ltsm_msginfo),
    .o_ltsmpckt_valid(o_ltsmpckt_valid),
    .o_ltsm_header(o_ltsm_header),
    .o_ltsm_data(o_ltsm_data)
  );

  // 800 MHz clock → 1.25 ns period
  always #625 clk = ~clk;

  function automatic logic even_parity(input logic [63:0] v);
    even_parity = ^v;
  endfunction

  task automatic check_pkt(
    input string name,
    input sb_msg_id id,
    input logic [63:0] exp_data
  );
    logic dp, cp;
    logic [63:0] h;

    dp = SB_MSG_LUT[id].has_data ? even_parity(exp_data) : 1'b0;

    h = o_ltsm_header;
    h[56] = 1'b0;  // Clear DP bit
    h[57] = 1'b0;  // Clear CP bit (FIX: must clear CP before calculating expected CP)
    cp = even_parity(h);

    assert(o_ltsmpckt_valid) else $error("%s: VALID=0", name);
    assert(o_ltsm_header[63:61]==3'b010) else $error("%s: SRCID", name);
    assert(o_ltsm_header[60:58]==3'b110) else $error("%s: DSTID", name);
    assert(o_ltsm_header[56]==dp) else $error("%s: DP", name);
    assert(o_ltsm_header[57]==cp) else $error("%s: CP", name);

    if (SB_MSG_LUT[id].has_data)
      assert(o_ltsm_data==exp_data) else $error("%s: DATA", name);
    else
      assert(o_ltsm_data==0) else $error("%s: DATA NOT ZERO", name);

    $display("%s PASS", name);
  endtask

  initial begin
    clk=0; rst=1; i_ltsm_valid=0;
    #20000 rst=0;

    // TEST 1: RESET
    assert(o_ltsmpckt_valid==0);
    assert(o_ltsm_header==0);
    assert(o_ltsm_data==0);
    $display("RESET PASS");

    // TEST 2: WITH DATA
    @(posedge clk);
    i_ltsm_valid=1;
    i_msg_id=SB_MBINIT_PARAM_CONFIG_REQ;
    ltsm_data=64'h1234;
    ltsm_msginfo=16'hAAAA;

    @(posedge clk);
    i_ltsm_valid=0;
    @(posedge clk);
    check_pkt("WITH DATA 1", SB_MBINIT_PARAM_CONFIG_REQ, 64'h1234);

    // TEST 3: WITH DATA
    @(posedge clk);
    i_ltsm_valid=1;
    i_msg_id=SB_MBTRAIN_REPAIR_APPLY_REPAIR_REQ;
    ltsm_data=64'hFFFF;
    ltsm_msginfo=16'hBBBB;

    @(posedge clk);
    i_ltsm_valid=0;
    @(posedge clk);
    check_pkt("WITH DATA 2", SB_MBTRAIN_REPAIR_APPLY_REPAIR_REQ, 64'hFFFF);

    // TEST 4: NO DATA
    @(posedge clk);
    i_ltsm_valid=1;
    i_msg_id=SB_SBINIT_OUT_OF_RESET;
    ltsm_data=64'hDEAD;
    ltsm_msginfo=16'h1111;

    @(posedge clk);
    i_ltsm_valid=0;
    @(posedge clk);
    check_pkt("NO DATA 1", SB_SBINIT_OUT_OF_RESET, 64'b0);

    // TEST 5: NO DATA
    @(posedge clk);
    i_ltsm_valid=1;
    i_msg_id=SB_MBTRAIN_LINKSPEED_DONE_REQ;
    ltsm_data=64'hBEEF;
    ltsm_msginfo=16'h2222;

    @(posedge clk);
    i_ltsm_valid=0;
    @(posedge clk);
    check_pkt("NO DATA 2", SB_MBTRAIN_LINKSPEED_DONE_REQ, 64'b0);

    $display("ALL TESTS PASSED");
    $finish;
  end

endmodule