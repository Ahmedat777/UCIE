module REPAIRVAL_Module (
    input               CLK,
    input               rst_n,
    input               i_REPAIRCLK_end,
    input               i_VAL_Pattern_done,
    input [3:0]         i_Rx_SbMessage,
    input               i_Busy_SideBand,
    input               i_falling_edge_busy,
    input               i_VAL_Result_logged,// from sb
    input               i_msg_valid,

    output reg          o_train_error_req,
    output reg          o_MBINIT_REPAIRVAL_Pattern_En,
    output reg          o_MBINIT_REPAIRVAL_Module_end,
    output reg [3:0]    o_TX_SbMessage,
    output reg          o_ValidOutDatat_Module
);

////////////////////////////////////////////////////////////////////////////////
// Sideband Message Definitions
////////////////////////////////////////////////////////////////////////////////
localparam MBINI_REPAIRVAL_init_req     = 4'b0001;
localparam MBINIT_REPAIRVAL_init_resp   = 4'b0010;
localparam MBINIT_REPAIRVAL_result_req  = 4'b0011;
localparam MBINIT_REPAIRVAL_result_resp = 4'b0100;
localparam MBINIT_REPAIRVAL_done_req    = 4'b0101;
localparam MBINIT_REPAIRVAL_done_resp   = 4'b0110;

////////////////////////////////////////////////////////////////////////////////
// FSM States
////////////////////////////////////////////////////////////////////////////////
localparam IDLE                   = 4'd0;
localparam SEND_INIT_REQUEST      = 4'd1;
localparam WAIT_BUSY_CLEAR_INIT  = 4'd2;
localparam WAIT_FOR_RESPONSE     = 4'd3;
localparam RUN_VALIDATION_PATTERN= 4'd4;
localparam WAIT_BUSY_CLEAR_RESULT= 4'd5;
localparam SEND_RESULT_REQUEST   = 4'd6;
localparam VERIFY_RESULT         = 4'd7;
localparam WAIT_BUSY_CLEAR_DONE  = 4'd8;
localparam SEND_DONE_REQUEST     = 4'd9;
localparam SEQUENCE_COMPLETE     = 4'd10;

reg [3:0] current_state,next_state;

////////////////////////////////////////////////////////////////////////////////
// State Register
////////////////////////////////////////////////////////////////////////////////
always @(posedge CLK or negedge rst_n)
    if(!rst_n)
        current_state <= IDLE;
    else
        current_state <= next_state;

////////////////////////////////////////////////////////////////////////////////
// Next State Logic
////////////////////////////////////////////////////////////////////////////////
always @(*) begin
    next_state = current_state;

    case(current_state)

    IDLE:
        if(i_REPAIRCLK_end && ~i_Busy_SideBand)
            next_state = SEND_INIT_REQUEST;

    SEND_INIT_REQUEST:
        if(i_falling_edge_busy)
            next_state = WAIT_BUSY_CLEAR_INIT;

    WAIT_BUSY_CLEAR_INIT:
        if(~i_Busy_SideBand)
            next_state = WAIT_FOR_RESPONSE;

    WAIT_FOR_RESPONSE:
        if(i_Rx_SbMessage==MBINIT_REPAIRVAL_init_resp && i_msg_valid)
            next_state = RUN_VALIDATION_PATTERN;
        else if(i_Rx_SbMessage==MBINIT_REPAIRVAL_result_resp && i_msg_valid)
            next_state = VERIFY_RESULT;
        else if(i_Rx_SbMessage==MBINIT_REPAIRVAL_done_resp && i_msg_valid)
            next_state = SEQUENCE_COMPLETE;

    RUN_VALIDATION_PATTERN:
        if(i_VAL_Pattern_done)
            next_state = WAIT_BUSY_CLEAR_RESULT;

    WAIT_BUSY_CLEAR_RESULT:
        if(~i_Busy_SideBand)
            next_state = SEND_RESULT_REQUEST;

    SEND_RESULT_REQUEST:
        if(i_falling_edge_busy)
            next_state = WAIT_FOR_RESPONSE;

    VERIFY_RESULT:
        if(~i_VAL_Result_logged)
            next_state = IDLE;
        else
            next_state = WAIT_BUSY_CLEAR_DONE;

    WAIT_BUSY_CLEAR_DONE:
        if(~i_Busy_SideBand)
            next_state = SEND_DONE_REQUEST;

    SEND_DONE_REQUEST:
        if(i_falling_edge_busy)
            next_state = WAIT_FOR_RESPONSE;

    SEQUENCE_COMPLETE:
        if(~i_REPAIRCLK_end)
            next_state = IDLE;

    default:
        next_state = IDLE;

    endcase
end

////////////////////////////////////////////////////////////////////////////////
// Outputs
////////////////////////////////////////////////////////////////////////////////
always @(posedge CLK or negedge rst_n) begin
    if(!rst_n) begin
        o_train_error_req             <= 0;
        o_MBINIT_REPAIRVAL_Pattern_En <= 0;
        o_MBINIT_REPAIRVAL_Module_end <= 0;
        o_TX_SbMessage                <= 0;
        o_ValidOutDatat_Module        <= 0;
    end else begin

        o_train_error_req             <= 0;
        o_MBINIT_REPAIRVAL_Pattern_En <= 0;
        o_MBINIT_REPAIRVAL_Module_end <= 0;
        o_TX_SbMessage                <= 0;
        o_ValidOutDatat_Module        <= 0;

        case(next_state)

        SEND_INIT_REQUEST: begin
            o_ValidOutDatat_Module <= 1;
            o_TX_SbMessage         <= MBINI_REPAIRVAL_init_req;
        end

        RUN_VALIDATION_PATTERN:
            o_MBINIT_REPAIRVAL_Pattern_En <= 1;

        SEND_RESULT_REQUEST: begin
            o_ValidOutDatat_Module <= 1;
            o_TX_SbMessage         <= MBINIT_REPAIRVAL_result_req;
        end

        VERIFY_RESULT:
            if(~i_VAL_Result_logged)
                o_train_error_req <= 1;

        SEND_DONE_REQUEST: begin
            o_ValidOutDatat_Module <= 1;
            o_TX_SbMessage         <= MBINIT_REPAIRVAL_done_req;
        end

        SEQUENCE_COMPLETE:
            o_MBINIT_REPAIRVAL_Module_end <= 1;
        default:
        o_ValidOutDatat_Module        <= 0;
        endcase
    end
end

endmodule
