module TRAINERROR__HS_Module #(
    parameter SB_MSG_WIDTH = 4
) (
    input                           i_clk,
    input                           i_rst_n,
    input                           i_trainerror_en,
    input                           i_msg_valid,
    input                           i_falling_edge_busy,
    input                           i_partner_valid,
    input  [SB_MSG_WIDTH-1:0]       i_Rx_SbMessage,
    output reg [SB_MSG_WIDTH-1:0]   o_TX_SbMessage,
    output reg                      o_trainerror_end_Module,
    output reg                      o_valid_Module
);  

reg [2:0] CS, NS;

////////////////////////////////////////////////////////////////////////////////
// Sideband messages
////////////////////////////////////////////////////////////////////////////////

localparam TRAINERROR_ENTRY_REQ_MSG  = 15;//f
localparam TRAINERROR_ENTRY_RESP_MSG = 14;//e

////////////////////////////////////////////////////////////////////////////////
// State machine states
////////////////////////////////////////////////////////////////////////////////

localparam IDLE                          = 0;
localparam WAIT_FOR_PARTNER_RESPONSE     = 1;
localparam SEND_TRAINERROR_REQUEST       = 2;
localparam TRAINERROR_TEST_COMPLETE      = 3;

////////////////////////////////////////////////////////////////////////////////
// State machine Transition for the TRAINERROR_Module
////////////////////////////////////////////////////////////////////////////////

always @(posedge i_clk or negedge i_rst_n) begin
    if (!i_rst_n) begin
        CS <= IDLE;
    end
    else begin
        CS <= NS;
    end
end

////////////////////////////////////////////////////////////////////////////////
// Next state logic for the TRAINERROR_Module
////////////////////////////////////////////////////////////////////////////////

always @(*) begin
    NS = CS;
    case (CS) 
        IDLE: begin
            if (i_trainerror_en && ~i_partner_valid) begin 
                NS = SEND_TRAINERROR_REQUEST; 
            end 
          
        end
        
        WAIT_FOR_PARTNER_RESPONSE: begin
            if (~i_trainerror_en) NS = IDLE;
            else if (i_Rx_SbMessage == TRAINERROR_ENTRY_RESP_MSG && i_msg_valid) begin
                NS = TRAINERROR_TEST_COMPLETE;
            end
        end
        
        SEND_TRAINERROR_REQUEST: begin
            if (~i_trainerror_en) NS = IDLE;
            else if (i_falling_edge_busy) NS = WAIT_FOR_PARTNER_RESPONSE;

           
        end
        
        TRAINERROR_TEST_COMPLETE: begin
            if (~i_trainerror_en) NS = IDLE;
        end
        
        default: begin
            NS = IDLE;
        end
    endcase
end

////////////////////////////////////////////////////////////////////////////////
// Registered output logic for the TRAINERROR_Module
////////////////////////////////////////////////////////////////////////////////

always @(posedge i_clk or negedge i_rst_n) begin
    if (!i_rst_n) begin
        o_TX_SbMessage          <= 4'b0000; 
        o_trainerror_end_Module <= 0;
        o_valid_Module          <= 0;
    end
    else begin
        o_TX_SbMessage          <= 4'b0000;
        o_trainerror_end_Module <= 0;
        o_valid_Module          <= 0;
        
        case (NS)
            SEND_TRAINERROR_REQUEST: begin
                o_TX_SbMessage <= TRAINERROR_ENTRY_REQ_MSG;
                o_valid_Module <= 1'b1;
            end
            
            TRAINERROR_TEST_COMPLETE: begin
                o_trainerror_end_Module <= 1'b1;
            end
            
            default: begin
                o_TX_SbMessage          <= 4'b0000; 
                o_trainerror_end_Module <= 0;
                o_valid_Module          <= 0;
            end
        endcase
    end 
end

endmodule