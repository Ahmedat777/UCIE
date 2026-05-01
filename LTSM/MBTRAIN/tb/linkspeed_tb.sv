`timescale 1ns/1ps

module tb_linkspeed_cal;

parameter SB_MSG_WIDTH = 4;

//////////////// ENUM FOR MESSAGES //////////////////

typedef enum logic [3:0] {

START_REQ                  = 1,
START_RESP                 = 2,
ERROR_REQ                  = 3,
ERROR_RESP                 = 4,
EXIT_TO_REPAIR_REQ         = 5,
EXIT_TO_REPAIR_RESP        = 6,
EXIT_TO_SPEED_DEGRADE_REQ  = 7,
EXIT_TO_SPEED_DEGRADE_RESP = 8,
DONE_REQ                   = 9,
DONE_RESP                  = 10,
EXIT_TO_PHYRETRAIN_REQ     = 11,
EXIT_TO_PHYRETRAIN_RESP    = 12

} msg_e;

//////////////// CLOCK //////////////////

reg i_clk = 0;
always #5 i_clk = ~i_clk;

//////////////// DUT SIGNALS //////////////////

reg i_rst_n;
reg i_enable;

reg [SB_MSG_WIDTH-1:0] i_sb_msg;
reg i_sb_valid;
reg i_falling_edge_busy;

reg i_point_test_ack;
reg [15:0] i_lane_results;

reg i_phy_in_retrain_flag;
reg i_runtime_config_change;

reg i_tx_first_8_lanes_ok_mbinit;
reg i_tx_second_8_lanes_ok_mbinit;

//////////// outputs ////////////

wire o_point_test_en;

wire [SB_MSG_WIDTH-1:0] o_sb_msg;
wire o_valid;
wire o_timeout_disable;

wire [1:0] o_linkspeed_lanes_status;

wire o_tx_first_8_lanes_ok_linkspeed;
wire o_tx_second_8_lanes_ok_linkspeed;

wire o_clear_phy_in_retrain_flag;
wire o_exit_phyretrain;
wire o_exit_repair;
wire o_exit_speedidle;

wire o_ack;


//////////////// DUT //////////////////

mbtrain_linkspeed_wrapper dut (.*);

//////////////// MESSAGE NAME //////////////////

function string msg_name(input [3:0] msg);

case(msg)

1  : msg_name="START_REQ";
2  : msg_name="START_RESP";
3  : msg_name="ERROR_REQ";
4  : msg_name="ERROR_RESP";
5  : msg_name="EXIT_TO_REPAIR_REQ";
6  : msg_name="EXIT_TO_REPAIR_RESP";
7  : msg_name="EXIT_TO_SPEED_DEGRADE_REQ";
8  : msg_name="EXIT_TO_SPEED_DEGRADE_RESP";
9  : msg_name="DONE_REQ";
10 : msg_name="DONE_RESP";
11 : msg_name="EXIT_TO_PHYRETRAIN_REQ";
12 : msg_name="EXIT_TO_PHYRETRAIN_RESP";

default : msg_name="UNKNOWN";


endcase
endfunction

//////////////// MONITOR //////////////////

always @(posedge i_clk)

if(o_valid)

$display("[%0t] DUT SENT -> %s",$time,msg_name(o_sb_msg));

//////////////// CHECK //////////////////

task check;

input cond;
input string text;

begin

if(cond)
$display("PASS : %s",text);
else
$display("FAIL : %s",text);

end

endtask

//////////////// PARTNER SEND //////////////////

task partner_send(input msg_e msg);

begin

@(posedge i_clk);

i_sb_msg = msg;
i_sb_valid = 1;

$display("[%0t] PARTNER SENT -> %s",$time,msg_name(msg));

@(posedge i_clk);

i_sb_valid = 0;

end

endtask
//////////////////////////////////////////////////////////
//////////////////////// TEST ////////////////////////////
//////////////////////////////////////////////////////////

initial begin

//////////////// RESET //////////////////


i_rst_n = 0;
i_enable = 0;

i_sb_valid = 0;
i_sb_msg = 0;
i_falling_edge_busy = 0;

i_lane_results = 16'h0;

i_point_test_ack = 0;

i_phy_in_retrain_flag = 0;
i_runtime_config_change = 0;

i_tx_first_8_lanes_ok_mbinit = 0;
i_tx_second_8_lanes_ok_mbinit = 0; 

#30;

i_rst_n = 1;
i_enable = 1;




$display("\n========= TEST START =========\n");

//////////////////////////////////////////////////////////
//////////// MODULE DECIDES PHY RETRAIN /////////////////
//////////////////////////////////////////////////////////

$display("\n========= MODULE DECIDES PHY RETRAIN =========\n");
i_lane_results = 16'hffff; // but we will trigger phy retrain to check priority
partner_send(START_REQ);
#10;
partner_send(START_RESP);
i_phy_in_retrain_flag = 1;
i_runtime_config_change = 1;
i_falling_edge_busy = 1;
#10;
i_point_test_ack = 1;
i_lane_results = 16'h00ff; // half lanes ok
partner_send(ERROR_REQ);
#10;
partner_send(EXIT_TO_PHYRETRAIN_REQ);
#10;
partner_send(EXIT_TO_PHYRETRAIN_RESP);
#10;
check(o_exit_phyretrain,"Exit PHY Retrain output flag asserted");
partner_send(DONE_REQ);
#10;
partner_send(DONE_RESP);
#10;
check(o_ack,"ACK asserted");
#10;
i_enable = 0;

#20;

check(dut.linkspeed_tx_u0.current_state_tx==0,
"Returned to IDLE");



//////////////////////////////////////////////////////////
//////////// partner force speedidle /////////////////
//////////////////////////////////////////////////////////
$display("\n========= partner force speedidle =========\n");
i_rst_n = 0;
i_enable = 0;

i_sb_valid = 0;
i_sb_msg = 0;
i_falling_edge_busy = 0;

i_lane_results = 16'h0;

i_point_test_ack = 0;

i_phy_in_retrain_flag = 0;
i_runtime_config_change = 0;
i_tx_first_8_lanes_ok_mbinit = 0;
i_tx_second_8_lanes_ok_mbinit = 0; 
#30;

i_rst_n = 1;
i_enable = 1;

i_tx_first_8_lanes_ok_mbinit = 1;
i_tx_second_8_lanes_ok_mbinit = 1; 
#10;

partner_send(START_REQ);
#10;
partner_send(START_RESP);
i_falling_edge_busy = 1;
#10;
i_point_test_ack = 1;
i_lane_results = 16'b 1111111100000000; // 8 lanes not ok, 8 lanes ok (repairable condition)
#20;
if(o_linkspeed_lanes_status == 2'b10)
$display("PASS : Linkspeed lanes status is correct showing 8 lanes not ok so repairable ");
else
partner_send(ERROR_REQ);
#20;
partner_send(ERROR_RESP);
#10;
partner_send(EXIT_TO_SPEED_DEGRADE_REQ);
#10;
partner_send(EXIT_TO_SPEED_DEGRADE_RESP);
#10;
check(o_exit_speedidle,"Exit Speedidle output flag asserted");
if (o_tx_first_8_lanes_ok_linkspeed && o_tx_second_8_lanes_ok_linkspeed)
$display("PASS : Both first and second 8 lanes took the value from mbinit as speedidle");

partner_send(DONE_REQ);
#10;
partner_send(DONE_RESP);
#10;
check(o_ack,"ACK asserted");
#10;
i_enable = 0;

#20;

check(dut.linkspeed_tx_u0.current_state_tx==0,
"Returned to IDLE");


//////////////////////////////////////////////////////////
//////////// both repair/////////////////
//////////////////////////////////////////////////////////
$display("\n========= both repair =========\n");
i_rst_n = 0;
i_enable = 0;

i_sb_valid = 0;
i_sb_msg = 0;
i_falling_edge_busy = 0;

i_lane_results = 16'h0;

i_point_test_ack = 0;

i_phy_in_retrain_flag = 0;
i_runtime_config_change = 0;
i_tx_first_8_lanes_ok_mbinit = 0;
i_tx_second_8_lanes_ok_mbinit = 0; 
#30;

i_rst_n = 1;
i_enable = 1;

i_tx_first_8_lanes_ok_mbinit = 1;
i_tx_second_8_lanes_ok_mbinit = 1; 
#10;

partner_send(START_REQ);
#10;
partner_send(START_RESP);
i_falling_edge_busy = 1;
#10;
i_point_test_ack = 1;
i_lane_results = 16'b 1111111100000000; // 8 lanes not ok, 8 lanes ok (repairable condition)
#20;
if(o_linkspeed_lanes_status == 2'b10)
$display("PASS : Linkspeed lanes status is correct showing 8 lanes not ok so repairable ");
else
partner_send(ERROR_REQ);
#20;
partner_send(ERROR_RESP);
#10;
partner_send(EXIT_TO_REPAIR_REQ);
#10;
partner_send(EXIT_TO_REPAIR_RESP);
#10;
check(o_exit_repair,"Exit Repair output flag asserted");
if (!o_tx_first_8_lanes_ok_linkspeed && o_tx_second_8_lanes_ok_linkspeed)
$display("PASS : Both first and second 8 lanes took the value from LANE RESULTS as repair");

partner_send(DONE_REQ);
#10;
partner_send(DONE_RESP);
#10;
check(o_ack,"ACK asserted");
#10;
i_enable = 0;

#20;

check(dut.linkspeed_tx_u0.current_state_tx==0,
"Returned to IDLE");

//////////////////////////////////////////////////////////
//////////// testing done/////////////////
//////////////////////////////////////////////////////////
$display("\n========= testing done =========\n");
i_rst_n = 0;
i_enable = 0;

i_sb_valid = 0;
i_sb_msg = 0;
i_falling_edge_busy = 0;

i_lane_results = 16'h0;

i_point_test_ack = 0;

i_phy_in_retrain_flag = 0;
i_runtime_config_change = 0;
i_tx_first_8_lanes_ok_mbinit = 0;
i_tx_second_8_lanes_ok_mbinit = 0; 
#30;

i_rst_n = 1;
i_enable = 1;

i_tx_first_8_lanes_ok_mbinit = 1;
i_tx_second_8_lanes_ok_mbinit = 1; 
#10;

partner_send(START_REQ);
#10;
partner_send(START_RESP);
i_falling_edge_busy = 1;
#10;
i_point_test_ack = 1;
i_lane_results = 16'h ffff; // all ok condition
#20;
if(o_linkspeed_lanes_status == 2'b01)
$display("PASS : Linkspeed lanes status is correct showing all 16 lanes ok ");
#10;
partner_send(DONE_REQ);
#10;
partner_send(DONE_RESP);
#10;
if (o_tx_first_8_lanes_ok_linkspeed && o_tx_second_8_lanes_ok_linkspeed)
$display("PASS : Both first and second 8 lanes took the value from LANE RESULTS as DONE");

check(o_ack,"ACK asserted");
#10;
i_enable = 0;


#20;

check(dut.linkspeed_tx_u0.current_state_tx==0,
"Returned to IDLE");
$display("\n========= TEST FINISHED =========\n");

$stop;

end

endmodule