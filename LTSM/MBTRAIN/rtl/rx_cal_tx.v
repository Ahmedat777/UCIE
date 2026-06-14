module rx_cal_tx (
    ////////////////////////////**main inputs**//////////////////////////////////
    input clk,    
    input rst_n,  
    input i_en, 
    ////////////////////////////**sideband inputs**//////////////////////////////
    input i_sideband_valid,
    input [3:0]  i_decoded_sideband_message,
    input i_busy_negedge_detected,
    ////////////////////////////**rx input**////////////////////////////////////		
    input i_valid_rx, 

    ////////////////////////////**sideband outputs**////////////////////////////
    output reg [3:0] o_sideband_message,
    ////////////////////////////**rx output**////////////////////////////////////
    output reg       o_valid_tx,
    ////////////////////////////**mbtrain output**//////////////////////////////
    output reg       o_test_ack
);
/*------------------------------------------------------------------------------
-- Sideband Messages Parameters
------------------------------------------------------------------------------*/
parameter START_REQ_MSG = 4'b0001; 
parameter START_RESP    = 4'b0010; 
parameter DONE_REQ_MSG  = 4'b0011; 
parameter DONE_RESP     = 4'b0100; 

/*------------------------------------------------------------------------------
-- FSM States
------------------------------------------------------------------------------*/
parameter IDLE          = 3'b000;
parameter START_REQ     = 3'b001;
parameter APPLY_CAL      = 3'b010;
parameter END_REQ       = 3'b011;
parameter TEST_FINISHED = 3'b100;

/*------------------------------------------------------------------------------
-- Variables Declaration
------------------------------------------------------------------------------*/
reg [2:0] cs, ns;

/*------------------------------------------------------------------------------
-- Next State Logic (Combinational)
------------------------------------------------------------------------------*/
always @(*) begin
    case (cs)
        IDLE:          ns = i_en ? START_REQ : IDLE;
        
        START_REQ:     ns = (i_decoded_sideband_message == START_RESP && i_sideband_valid) ? APPLY_CAL : START_REQ;
        
        APPLY_CAL:      ns = END_REQ; 
        
        END_REQ:       ns = (i_decoded_sideband_message == DONE_RESP && i_sideband_valid) ? TEST_FINISHED : END_REQ;
        
        TEST_FINISHED: ns = (~i_en) ? IDLE : TEST_FINISHED;
        
        default:       ns = IDLE;
    endcase
end

/*------------------------------------------------------------------------------
-- State Register (Sequential)
------------------------------------------------------------------------------*/
always @(posedge clk or negedge rst_n) begin
    if (~rst_n) cs <= IDLE;
    else        cs <= ns;
end

/*------------------------------------------------------------------------------
-- Output Logic Calculations (Sequential based on NS)
------------------------------------------------------------------------------*/
always @(posedge clk or negedge rst_n) begin 
    if(~rst_n) begin
        o_sideband_message <= 4'b0000;
        o_valid_tx         <= 1'b0;
        o_test_ack         <= 1'b0;
    end else begin
        case (ns) 
            IDLE: begin
                o_sideband_message <= 4'b0000;
                o_valid_tx         <= 1'b0;
                o_test_ack         <= 1'b0;
            end
            
            START_REQ: begin
                o_sideband_message <= START_REQ_MSG;
                o_valid_tx         <= 1'b1;
                o_test_ack         <= 1'b0;
            end
            
            APPLY_CAL: begin
                o_sideband_message <= 4'b0000; 
                o_valid_tx         <= 1'b0;
                o_test_ack         <= 1'b0;
            end
            
            END_REQ: begin
                o_sideband_message <= DONE_REQ_MSG;
                o_valid_tx         <= 1'b1;
                o_test_ack         <= 1'b0;
            end
            
            TEST_FINISHED: begin
                o_sideband_message <= 4'b0000;
                o_valid_tx         <= 1'b0;
                o_test_ack         <= 1'b1;
            end
            
            default: begin
                o_sideband_message <= 4'b0000;
                o_valid_tx         <= 1'b0;
                o_test_ack         <= 1'b0;
            end
        endcase
/*------------------------------------------------------------------------------
/////////////////////////////**handling valid////////////////////////////////    
------------------------------------------------------------------------------*/		
		 if (i_busy_negedge_detected && ~i_valid_rx) begin
            o_valid_tx <= 0;
        end
    end
end