module rx_cal_rx (
    ////////////////////////////**Main Inputs**//////////////////////////////////
    input clk,    
    input rst_n,  
    input i_en, 
    ////////////////////////////**Sideband Inputs**//////////////////////////////
    input [3:0]  i_decoded_sideband_message,
    input        i_sideband_valid,
    input        i_busy_negedge_detected,
    ////////////////////////////**TX Status Input**//////////////////////////////        
    input        i_valid_tx,
    ////////////////////////////**Sideband Outputs**////////////////////////////
    output reg [3:0] o_sideband_message,
	////////////////////////////**tx output**////////////////////////////////////
    output reg       o_valid_rx,
    ////////////////////////////**FSM Output Ack**//////////////////////////////
    output reg       o_test_ack
);

/*------------------------------------------------------------------------------
-- Sideband Messages Parameters
------------------------------------------------------------------------------*/
parameter START_REQ_MSG  = 4'b0001; 
parameter START_RESP_MSG = 4'b0010; 
parameter DONE_REQ_MSG   = 4'b0011; 
parameter DONE_RESP_MSG  = 4'b0100; 

/*------------------------------------------------------------------------------
-- FSM States
------------------------------------------------------------------------------*/
parameter IDLE               = 3'b000;
parameter WAIT_FOR_START_REQ = 3'b001;
parameter APPLY_CAL          = 3'b010;
parameter WAIT_FOR_END_REQ   = 3'b011;
parameter SEND_END_RESPONSE  = 3'b100;
parameter TEST_FINISHED      = 3'b101;

/*------------------------------------------------------------------------------
-- Variables Declaration
------------------------------------------------------------------------------*/
reg [2:0] cs, ns;

/*------------------------------------------------------------------------------
-- Next State Logic (Combinational)
------------------------------------------------------------------------------*/
always @(*) begin
    case (cs)
        IDLE:               ns = i_en ? WAIT_FOR_START_REQ : IDLE;

        WAIT_FOR_START_REQ: ns = (i_decoded_sideband_message == START_REQ_MSG && i_sideband_valid) ? APPLY_CAL : WAIT_FOR_START_REQ;

        APPLY_CAL:          ns = WAIT_FOR_END_REQ; 

        WAIT_FOR_END_REQ:   ns = (i_decoded_sideband_message == DONE_REQ_MSG && i_sideband_valid) ? SEND_END_RESPONSE : WAIT_FOR_END_REQ;

        
        SEND_END_RESPONSE:  ns = i_busy_negedge_detected ? TEST_FINISHED : SEND_END_RESPONSE;

        TEST_FINISHED:      ns = (~i_en) ? IDLE : TEST_FINISHED;

        default:            ns = IDLE;
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
-- Output Logic Calculations (Based on Next State)
------------------------------------------------------------------------------*/
always @(posedge clk or negedge rst_n) begin 
    if(~rst_n) begin
        o_sideband_message <= 4'b0000;
        o_valid_rx         <= 0;
        o_test_ack         <= 0;
    end else begin
        case (ns) 
            IDLE, WAIT_FOR_START_REQ: begin
                o_sideband_message <= 4'b0000;
                o_valid_rx         <= 0;
                o_test_ack         <= 0;
            end
            
            APPLY_CAL: begin
                
                o_sideband_message <= START_RESP_MSG;
                o_valid_rx         <= 1; 
                o_test_ack         <= 0;
            end

            WAIT_FOR_END_REQ: begin
                o_sideband_message <= 4'b0000;
                o_valid_rx         <= 0;
                o_test_ack         <= 0;
            end

            SEND_END_RESPONSE: begin
                o_sideband_message <= DONE_RESP_MSG;
                o_valid_rx         <= 1;
                o_test_ack         <= 0;
            end

            TEST_FINISHED: begin
                o_sideband_message <= 4'b0000;
                o_valid_rx         <= 0;
                o_test_ack         <= 1;
            end

            default: begin
                o_sideband_message <= 4'b0000;
                o_valid_rx         <= 0;
                o_test_ack         <= 0;
            end
        endcase
/*------------------------------------------------------------------------------
/////////////////////////////**handling valid////////////////////////////////    
------------------------------------------------------------------------------*/     
        if (i_busy_negedge_detected) begin
            o_valid_rx <= 0;
        end
    end 
end

endmodule