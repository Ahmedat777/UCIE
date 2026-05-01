module TRAINERROR_HS_ModulePartner #(
    parameter SB_MSG_WIDTH = 4
) (
    input                           i_clk,
    input                           i_rst_n, 
    input                           i_trainerror_en,
    input                           i_msg_valid,
    input                           i_falling_edge_busy,
    input                           i_module_valid,
    input  [SB_MSG_WIDTH-1:0]       i_Rx_SbMessage,
    output reg [SB_MSG_WIDTH-1:0]   o_TX_SbMessage,
    output reg                      o_trainerror_end_Module_Partner,
    output reg                      o_valid_Module_Partner
);

reg [2:0] CS, NS;

////////////////////////////////////////////////////////////////////////////////
// Sideband messages
////////////////////////////////////////////////////////////////////////////////

localparam TRAINERROR_ENTRY_REQ_MSG  = 15;
localparam TRAINERROR_ENTRY_RESP_MSG = 14;

////////////////////////////////////////////////////////////////////////////////
// State machine states ModulePartner
////////////////////////////////////////////////////////////////////////////////

localparam IDLE                              = 0;
localparam WAIT_FOR_TRAINERROR_REQUEST       = 1;
localparam WAIT_BUSY_CLEAR_FOR_RESP          = 2;
localparam SEND_TRAINERROR_RESPONSE          = 3;
localparam TRAINERROR_TEST_COMPLETE          = 4;

////////////////////////////////////////////////////////////////////////////////
// State machine Transition for the TRAINERROR_ModulePartner
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
// Next state logic for the TRAINERROR_ModulePartner
////////////////////////////////////////////////////////////////////////////////

always @(*) begin
    NS = CS;
    case (CS) 
        IDLE: begin
            if (i_trainerror_en) begin
                NS = WAIT_FOR_TRAINERROR_REQUEST;
            end
        end
        
        WAIT_FOR_TRAINERROR_REQUEST: begin 
            if (~i_trainerror_en) NS = IDLE;
            else if (i_Rx_SbMessage == TRAINERROR_ENTRY_REQ_MSG && i_msg_valid) begin 
                NS = WAIT_BUSY_CLEAR_FOR_RESP;
            end
        end
        
        WAIT_BUSY_CLEAR_FOR_RESP: begin
            if (~i_trainerror_en) NS = IDLE;
            else if (~i_module_valid) NS = SEND_TRAINERROR_RESPONSE;
        end
        
        SEND_TRAINERROR_RESPONSE: begin
            if (~i_trainerror_en) NS = IDLE;
            else if (i_falling_edge_busy) begin
                NS = TRAINERROR_TEST_COMPLETE;
            end
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
// Registered output logic for the TRAINERROR_ModulePartner
////////////////////////////////////////////////////////////////////////////////

always @(posedge i_clk or negedge i_rst_n) begin
    if (!i_rst_n) begin
        o_trainerror_end_Module_Partner <= 0;
        o_TX_SbMessage                  <= 4'b0000;
        o_valid_Module_Partner          <= 0;
    end
    else begin
        o_trainerror_end_Module_Partner <= 0;
        o_TX_SbMessage                  <= 4'b0000;
        o_valid_Module_Partner          <= 0;
        
        case (NS)
            SEND_TRAINERROR_RESPONSE: begin
                o_TX_SbMessage         <= TRAINERROR_ENTRY_RESP_MSG;
                o_valid_Module_Partner <= 1'b1;
            end
            
            TRAINERROR_TEST_COMPLETE: begin
                o_trainerror_end_Module_Partner <= 1'b1;
            end
            
            default: begin
                o_trainerror_end_Module_Partner <= 0;
                o_TX_SbMessage                  <= 4'b0000;
                o_valid_Module_Partner          <= 0;
            end
        endcase
    end
end

endmodule