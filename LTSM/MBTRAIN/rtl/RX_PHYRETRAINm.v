module RX_PHYRETRAIN (
////////////////////////////**main inputs**////////////////////////////
input clk,
input rst_n,
input i_en,
////////////////////////////**tx input**//////////////////////////// 
input i_valid_tx,			
input i_busy_negedge_detected,
input[2:0]i_local_retrain_encoding,
input i_clear_resolved_state,
input i_sideband_valid,
input [2:0] i_retrain_encoding_partner,
input  [3:0]  i_sideband_message,

output reg [3:0] o_sideband_message,
//output reg[2:0] o_sideband_encoding,
output reg  o_valid_rx ,
output reg o_test_ack,
output reg  [1:0] o_resolved_state
);

/*------------------------------------------------------------------------------
////////////////////////////**FSM States////////////////////////////   
------------------------------------------------------------------------------*/
localparam [2:0] IDLE                     = 3'b000;
localparam [2:0] WAIT_FOR_PHYRETRAIN_REQ  = 3'b001;
localparam [2:0] SEND_PHYRETRAIN_RESP 	  = 3'b010;
localparam [2:0] TEST_FINISHED            = 3'b011;
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
            NS = (!i_en) ? IDLE : WAIT_FOR_PHYRETRAIN_REQ;

        WAIT_FOR_PHYRETRAIN_REQ: 
            NS = (!i_en) ? IDLE : 
                 (i_sideband_message == PHYRETRAIN_START_REQ && i_sideband_valid) ? SEND_PHYRETRAIN_RESP : WAIT_FOR_PHYRETRAIN_REQ;

        SEND_PHYRETRAIN_RESP: 
            NS = (!i_en) ? IDLE : 
                 (i_busy_negedge_detected) ? TEST_FINISHED : SEND_PHYRETRAIN_RESP;

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
        o_test_ack <= 1'b0;
       // o_sideband_message <= 4'b0000;
        o_resolved_state    <= 2'b00;
        o_valid_rx          <= 1'b0;
    end
    else begin
        case (CS)
            IDLE: begin
                o_test_ack <= 1'b0;
                o_sideband_message <= 4'b0000;
                o_valid_rx          <= 1'b0;
                // بنصفر الـ resolved state فقط لو جالي أمر مسح، غير كدة بنحتفظ بآخر قيمة
                if (i_clear_resolved_state) 
                    o_resolved_state <= 2'b00;
            end

            WAIT_FOR_PHYRETRAIN_REQ: begin
                o_test_ack <= 1'b0;
                o_sideband_message <= 4'b0000;
                o_valid_rx          <= 1'b0;
                
                if (i_clear_resolved_state) 
                    o_resolved_state <= 2'b00;
            end

            SEND_PHYRETRAIN_RESP: begin
                o_valid_rx          <= 1'b1; // بنرفع الـ valid عشان نرد على الشريك
                o_sideband_message <= PHYRETRAIN_START_RESP;
                o_test_ack <= 1'b0;
				
				/*o_sideband_encoding <= (o_resolved_state == 2'h1) ? 3'b001 : // TXSELFCAL
                       (o_resolved_state == 2'h2) ? 3'b100 : // REPAIR
                       (o_resolved_state == 2'h3) ? 3'b010 : 3'b000;*/

                // تنفيذ الـ Resolution Logic مرة واحدة عند الانتقال للحالة دي أو طول ما إحنا فيها
                case ({i_local_retrain_encoding, i_retrain_encoding_partner, i_sideband_valid})
                    7'b001_001_1: o_resolved_state <= 2'h1; // TXSELFCAL
                    7'b001_100_1: o_resolved_state <= 2'h2; // REPAIR
                    7'b001_010_1: o_resolved_state <= 2'h3; // SPEEDIDLE
                    7'b100_001_1: o_resolved_state <= 2'h2; // REPAIR
                    7'b100_100_1: o_resolved_state <= 2'h2; // REPAIR
                    7'b100_010_1: o_resolved_state <= 2'h3; // SPEEDIDLE
                    7'b010_001_1: o_resolved_state <= 2'h3; // SPEEDIDLE
                    7'b010_100_1: o_resolved_state <= 2'h3; // SPEEDIDLE
                    7'b010_010_1: o_resolved_state <= 2'h3; // SPEEDIDLE
                    default     : o_resolved_state <= o_resolved_state; // حافظ على القيمة الحالية
                endcase
            end

            TEST_FINISHED: begin
                o_valid_rx          <= 1'b0; // نزل الـ valid لأننا خلصنا بعت
                o_sideband_message <= 4'b0000;
                o_test_ack <= 1'b1; // رفع إشارة النهاية للـ LTSM
            end

            default: begin
                o_test_ack <= 1'b0;
                o_sideband_message <= 4'b0000;
                o_valid_rx          <= 1'b0;
            end
        endcase
        
        // أولوية المسح (Clear) للـ Resolved State تكون أعلى من الـ Assignment
        if (i_clear_resolved_state) begin
            o_resolved_state <= 2'b00;
        end
    end
end