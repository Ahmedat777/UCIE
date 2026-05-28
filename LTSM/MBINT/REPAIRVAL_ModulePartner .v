import pckg::*;

module REPAIRVAL_ModulePartner (
    input  wire         CLK,
    input  wire         rst_n,
    input  wire         i_REPAIRCLK_end,
    input  wire         i_VAL_Result_logged,// patern compartor
    input  wire sb_msg_id i_Rx_SbMessage,
    input  wire         i_falling_edge_busy,
    input  wire         i_Busy_SideBand,
    input  wire         i_msg_valid,
    
    output reg          o_VAL_128Result_logged,
    output reg sb_msg_id o_TX_SbMessage,
    output reg          o_MBINIT_REPAIRVAL_ModulePartner_end,
    output reg          o_ValidOutDatat_ModulePartner,
    output reg          o_enable_16_iterations
);

////////////////////////////////////////////////////////////////////////////////
// State Machine Definitions
////////////////////////////////////////////////////////////////////////////////
localparam IDLE                      = 4'd0;
localparam WAIT_INIT_REQUEST         = 4'd1;
localparam SEND_INIT_RESPONSE        = 4'd2;
localparam SEND_RESULT_RESPONSE      = 4'd3;
localparam SEND_DONE_RESPONSE        = 4'd4;
localparam SEQUENCE_COMPLETE         = 4'd5;
localparam WAIT_FOR_REQUEST          = 4'd6;
localparam WAIT_BUSY_CLEAR_INIT      = 4'd7;
localparam WAIT_BUSY_CLEAR_RESULT    = 4'd8;
localparam WAIT_BUSY_CLEAR_DONE      = 4'd9;

////////////////////////////////////////////////////////////////////////////////
// State Registers
////////////////////////////////////////////////////////////////////////////////
reg [3:0] current_state, next_state;
reg val_result_latched;

////////////////////////////////////////////////////////////////////////////////
// State Register Update
////////////////////////////////////////////////////////////////////////////////
always @(posedge CLK or negedge rst_n) begin
    if (!rst_n)
        current_state <= IDLE;
    else
        current_state <= next_state;
end

////////////////////////////////////////////////////////////////////////////////
// Latch i_VAL_Result_logged
////////////////////////////////////////////////////////////////////////////////
always @(posedge CLK or negedge rst_n) begin
    if (!rst_n)
        val_result_latched <= 1'b0;
    else if (current_state == WAIT_BUSY_CLEAR_RESULT && ~i_Busy_SideBand)
        val_result_latched <= i_VAL_Result_logged;
end

////////////////////////////////////////////////////////////////////////////////
// Next State Logic
////////////////////////////////////////////////////////////////////////////////
always @(*) begin
    next_state = current_state; // Default

    case(current_state)
        IDLE: begin
            if (i_REPAIRCLK_end)
                next_state = WAIT_INIT_REQUEST;
        end

        WAIT_INIT_REQUEST: begin
            if (~i_REPAIRCLK_end)
                next_state = IDLE;
            else if (i_Rx_SbMessage == SB_MBINIT_REPAIRVAL_INIT_REQ && i_msg_valid)
                next_state = WAIT_BUSY_CLEAR_INIT;
        end

        WAIT_BUSY_CLEAR_INIT: begin
            if (~i_REPAIRCLK_end)
                next_state = IDLE;
            else if (~i_Busy_SideBand)
                next_state = SEND_INIT_RESPONSE;
        end

        SEND_INIT_RESPONSE: begin
            if (~i_REPAIRCLK_end)
                next_state = IDLE;
            else if (i_falling_edge_busy)
                next_state = WAIT_FOR_REQUEST;
        end

        WAIT_FOR_REQUEST: begin
            if (~i_REPAIRCLK_end)
                next_state = IDLE;
            else if (i_Rx_SbMessage == SB_MBINIT_REPAIRVAL_RESULT_REQ && i_msg_valid)
                next_state = WAIT_BUSY_CLEAR_RESULT;
            else if (i_Rx_SbMessage == SB_MBINIT_REPAIRVAL_DONE_REQ && i_msg_valid)
                next_state = WAIT_BUSY_CLEAR_DONE;
        end

        WAIT_BUSY_CLEAR_RESULT: begin
            if (~i_REPAIRCLK_end)
                next_state = IDLE;
            else if (~i_Busy_SideBand)
                next_state = SEND_RESULT_RESPONSE;
        end

        SEND_RESULT_RESPONSE: begin
            if (~i_REPAIRCLK_end)
                next_state = IDLE;
            else if (i_falling_edge_busy)
                next_state = WAIT_FOR_REQUEST;
        end

        WAIT_BUSY_CLEAR_DONE: begin
            if (~i_REPAIRCLK_end)
                next_state = IDLE;
            else if (~i_Busy_SideBand)
                next_state = SEND_DONE_RESPONSE;
        end

        SEND_DONE_RESPONSE: begin
            if (~i_REPAIRCLK_end)
                next_state = IDLE;
            else if (i_falling_edge_busy)
                next_state = SEQUENCE_COMPLETE;
        end

        SEQUENCE_COMPLETE: begin
            if (~i_REPAIRCLK_end)
                next_state = IDLE;
        end

        default: next_state = IDLE;
    endcase
end

////////////////////////////////////////////////////////////////////////////////
// Registered Output Logic
////////////////////////////////////////////////////////////////////////////////
always @(posedge CLK or negedge rst_n) begin
    if (!rst_n) begin
        o_ValidOutDatat_ModulePartner        <= 1'b0;
        o_VAL_128Result_logged               <= 1'b0;
        o_TX_SbMessage                       <= sb_msg_id'(0);
        o_MBINIT_REPAIRVAL_ModulePartner_end <= 1'b0;
        o_enable_16_iterations               <= 1'b0;
    end else begin
        // Defaults
        o_ValidOutDatat_ModulePartner        <= 1'b0;
        o_VAL_128Result_logged               <= 1'b0;
        o_TX_SbMessage                       <= sb_msg_id'(0);
        o_MBINIT_REPAIRVAL_ModulePartner_end <= 1'b0;
        o_enable_16_iterations               <= 1'b1;

        case(next_state)
            SEND_INIT_RESPONSE: begin
                o_ValidOutDatat_ModulePartner <= 1'b1;
                o_TX_SbMessage                <= SB_MBINIT_REPAIRVAL_INIT_RESP;
            end

            SEND_RESULT_RESPONSE: begin
                o_ValidOutDatat_ModulePartner <= 1'b1;
                o_TX_SbMessage                <= SB_MBINIT_REPAIRVAL_RESULT_RESP;
                o_VAL_128Result_logged        <= val_result_latched; 
            end

            SEND_DONE_RESPONSE: begin
                o_ValidOutDatat_ModulePartner <= 1'b1;
                o_TX_SbMessage                <= SB_MBINIT_REPAIRVAL_DONE_RESP;
            end

            SEQUENCE_COMPLETE: begin
                o_MBINIT_REPAIRVAL_ModulePartner_end <= 1'b1;
            end
        endcase
    end
end

endmodule