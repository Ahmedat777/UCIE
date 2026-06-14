module TX_PHYRETRAIN (
////////////////////////////**main inputs**////////////////////////////
input clk,
input rst_n,
input i_en,
////////////////////////////**sideband inputs**////////////////////////////
input [3:0] i_sideband_message,
input i_sideband_valid,
input i_busy_negedge_detected,
////////////////////////////**rx input**//////////////////////////// 
input i_valid_rx,
////////////////////////////**mb input**////////////////////////////
input i_enter_from_active_or_mbtrain, // 0b:  ACTIVE state , 1b:  MBTRAIN from  Runtime Link Test Status Register
input[1:0]i_linkspeed_lanes_status,   // from linkspeed  0h: IDLE , 1h: No Lane errors, 2h: Lane errors & faulty Lanes are repairable, 3h: Lane errors & faulty Lanes cannot be repaired
////////////////////////////**sideband outputs**////////////////////////////
output reg [3:0] o_sideband_message,
output reg[2:0] o_sideband_encoding,
output reg o_valid_tx,
////////////////////////////**mbtrain output**////////////////////////////
output reg o_test_ack);

/*------------------------------------------------------------------------------
////////////////////////////**FSM States////////////////////////////   
------------------------------------------------------------------------------*/
localparam [2:0] IDLE                 = 3'b000;
localparam [2:0] WAIT_FOR_RX_TO_RESP  = 3'b001;
localparam [2:0] SEND_PHYRETRAIN_REQ  = 3'b010;
localparam [2:0] TEST_FINISHED        = 3'b011;

/*------------------------------------------------------------------------------
////////////////////////////**Sideband Messages////////////////////////////   
------------------------------------------------------------------------------*/
localparam PHYRETRAIN_START_REQ  = 4'b0001;
localparam PHYRETRAIN_START_RESP = 4'b0010;
/*------------------------------------------------------------------------------
////////////////////////////**Variables Declaration////////////////////////////  
------------------------------------------------------------------------------*/
reg [2:0] CS, NS;
/*------------------------------------------------------------------------------
-- State Register (Sequential)
------------------------------------------------------------------------------*/
always @(posedge clk or negedge rst_n) begin
    if (~rst_n) CS <= IDLE;
    else        CS <= NS;
end
/*------------------------------------------------------------------------------
///////////////////**combinational always for ns calculations///////////////////
------------------------------------------------------------------------------*/
always @ (*) begin
    case (CS) 
        IDLE: 
            NS = (!i_en) ? IDLE : 
                 (i_sideband_message == PHYRETRAIN_START_REQ && i_sideband_valid) ? WAIT_FOR_RX_TO_RESP : SEND_PHYRETRAIN_REQ;

        WAIT_FOR_RX_TO_RESP: 
            NS = (!i_en) ? IDLE : 
                 (i_busy_negedge_detected && i_valid_rx) ? SEND_PHYRETRAIN_REQ : WAIT_FOR_RX_TO_RESP;

        SEND_PHYRETRAIN_REQ: 
            NS = (!i_en) ? IDLE : 
                 (i_sideband_message == PHYRETRAIN_START_RESP && i_sideband_valid) ? TEST_FINISHED : SEND_PHYRETRAIN_REQ;

        TEST_FINISHED: 
            NS = (!i_en) ? IDLE : TEST_FINISHED;

        default: 
            NS = IDLE;
    endcase
end
/*------------------------------------------------------------------------------
-- Output Logic (Sequential - Registered Outputs)
------------------------------------------------------------------------------*/

always @(posedge clk or negedge rst_n) begin 
    if (!rst_n) begin
        o_sideband_message  <= 4'b0000;
        o_test_ack          <= 1'b0;
        o_sideband_encoding <= 3'b000;
        o_valid_tx          <= 1'b0; 
    end
    else begin
        case (CS)
            IDLE: begin
                o_sideband_message  <= 4'b0000;
                o_test_ack          <= 1'b0;
                o_sideband_encoding <= 3'b000;
                o_valid_tx          <= 1'b0;
            end

            WAIT_FOR_RX_TO_RESP: begin
                // في الحالة دي إحنا بنستقبل، فمش بنبعت حاجة لسه
                o_sideband_message  <= 4'b0000;
                o_test_ack          <= 1'b0;
                o_sideband_encoding <= 3'b000;
                o_valid_tx          <= 1'b0;
            end

            SEND_PHYRETRAIN_REQ: begin
                o_valid_tx         <= 1'b1;
                o_sideband_message <= PHYRETRAIN_START_REQ;
                o_test_ack         <= 1'b0;

                // تحديد الـ Encoding بناءً على المصدر ونتيجة الـ Lanes
                if (i_enter_from_active_or_mbtrain == 0) begin
                    o_sideband_encoding <= 3'b001; // TXSELFCAL
                end 
                else begin
                    if (i_linkspeed_lanes_status == 2'b01)
                        o_sideband_encoding <= 3'b001; // TXSELFCAL
                    else if (i_linkspeed_lanes_status == 2'b10)
                        o_sideband_encoding <= 3'b100; // REPAIR
                    else if (i_linkspeed_lanes_status == 2'b11)
                        o_sideband_encoding <= 3'b010; // SPEEDIDLE
                    else
                        o_sideband_encoding <= 3'b000;
                end
            end

            TEST_FINISHED: begin
                o_valid_tx          <= 1'b0;
                o_sideband_message  <= 4'b0000;
                o_test_ack          <= 1'b1; 
            end

            default: begin
                o_valid_tx          <= 1'b0;
                o_sideband_message  <= 4'b0000;
                o_test_ack          <= 1'b0;
                o_sideband_encoding <= 3'b000;
            end
        endcase
/*------------------------------------------------------------------------------
/////////////////////////////**handling valid////////////////////////////////    
------------------------------------------------------------------------------*/
        if (i_busy_negedge_detected && !i_valid_rx) begin
            o_valid_tx <= 1'b0;
        end	
    end
end
endmodule

