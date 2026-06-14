module repair_rx (  
////////////////////////////**main inputs**//////////////////////////////////
input clk,
input rst_n,
input i_en,
////////////////////////////**sideband inputs**/////////////////////////////
input [3:0] i_sideband_message,
input [2:0] i_sideband_data_lanes_encoding,
input i_busy_negedge_detected,
input i_sideband_valid,
////////////////////////////**tx input**//////////////////////////////////// 
input i_valid_tx,
////////////////////////////**sideband outputs**////////////////////////////
output reg [3:0] o_sideband_message,
output reg o_valid_rx,
////////////////////////////**mbtrain output**////////////////////////////
output reg o_test_ack,
output reg o_remote_partner_first_8_lanes_result, o_remote_partner_second_8_lanes_result
);

/*----------------------------------------------------------------------------
-- Sideband Messages Parameters
----------------------------------------------------------------------------*/
parameter INIT_REQUEST           = 4'b0001;
parameter INIT_RESPONSE          = 4'b0010;
parameter APPLY_DEGRADE_REQUEST  = 4'b0011;
parameter APPLY_DEGRADE_RESPONSE = 4'b0100;
parameter END_REQUEST            = 4'b0101;
parameter END_RESPONSE           = 4'b0110;

/*------------------------------------------------------------------------------
-- FSM States
------------------------------------------------------------------------------*/
parameter IDLE                          = 3'b000;
parameter REPAIR_INIT_RESPONSE          = 3'b001;
parameter REPAIR_APPLY_DEGRADE_RESPONSE = 3'b010;
parameter REPAIR_END_RESPONSE           = 3'b011;
parameter SEND_END_RESPONSE             = 3'b100;
parameter TEST_FINISH                   = 3'b101;

/*------------------------------------------------------------------------------
-- Variables Declaration
------------------------------------------------------------------------------*/
reg [2:0] cs, ns;

/*------------------------------------------------------------------------------
-- Combinational Next State Logic (Using Ternary Operators)
------------------------------------------------------------------------------*/
always @(*) begin
    case (cs)
        IDLE: 
            ns = i_en ? REPAIR_INIT_RESPONSE : IDLE;

        REPAIR_INIT_RESPONSE: 
            ns = (!i_en) ? IDLE : 
                 (i_sideband_message == INIT_REQUEST && i_sideband_valid) ? REPAIR_APPLY_DEGRADE_RESPONSE : REPAIR_INIT_RESPONSE;

        REPAIR_APPLY_DEGRADE_RESPONSE: 
            ns = (!i_en) ? IDLE : 
                 (i_sideband_message == APPLY_DEGRADE_REQUEST && i_sideband_valid) ? REPAIR_END_RESPONSE : REPAIR_APPLY_DEGRADE_RESPONSE;

        REPAIR_END_RESPONSE: 
            ns = (!i_en) ? IDLE : 
                 (i_sideband_message == END_REQUEST && i_sideband_valid) ? SEND_END_RESPONSE : REPAIR_END_RESPONSE;

        SEND_END_RESPONSE: 
            ns = (!i_en) ? IDLE : TEST_FINISH; 

        TEST_FINISH: 
            ns = (!i_en) ? IDLE : TEST_FINISH;

        default: 
            ns = IDLE;
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
-- Output Logic (Sequential - Registered) 
------------------------------------------------------------------------------*/
always @(posedge clk or negedge rst_n) begin
    if (~rst_n) begin
        o_sideband_message <= 4'b0000;
        o_valid_rx         <= 1'b0;
        o_test_ack         <= 1'b0;
        o_remote_partner_first_8_lanes_result  <= 1'b0;
        o_remote_partner_second_8_lanes_result <= 1'b0;
    end
    else begin
        case (cs)
            IDLE: begin
                o_sideband_message <= 4'b0000;
                o_valid_rx         <= 1'b0;
                o_test_ack         <= 1'b0;
            end

            REPAIR_INIT_RESPONSE: begin
                o_sideband_message <= INIT_RESPONSE;
                o_valid_rx         <= 1'b1;
            end

            REPAIR_APPLY_DEGRADE_RESPONSE: begin
                o_sideband_message <= APPLY_DEGRADE_RESPONSE;
                o_valid_rx         <= 1'b1;
                o_remote_partner_first_8_lanes_result  <= (i_sideband_data_lanes_encoding == 3'b011 || i_sideband_data_lanes_encoding == 3'b001);
                o_remote_partner_second_8_lanes_result <= (i_sideband_data_lanes_encoding == 3'b011 || i_sideband_data_lanes_encoding == 3'b010);
            end

            REPAIR_END_RESPONSE: begin
                o_sideband_message <= END_RESPONSE;
                o_valid_rx         <= 1'b1;
            end

            SEND_END_RESPONSE: begin
                o_sideband_message <= 4'b0000;
                o_valid_rx         <= 1'b0;
                o_test_ack         <= 1'b1;
            end

            TEST_FINISH: begin
                o_test_ack <= 1'b1;
                o_valid_rx <= 1'b0;
            end

            default: begin
                o_sideband_message <= 4'b0000;
                o_valid_rx         <= 1'b0;
                o_test_ack         <= 1'b0;
            end
        endcase
/*------------------------------------------------------------------------------
/////////////////////////////**handling valid////////////////////////////////    
------------------------------------------------------------------------------*/
        if (i_busy_negedge_detected ) begin
            o_valid_rx <= 1'b0;
        end
    end
end

endmodule