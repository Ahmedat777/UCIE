////////////////////////////////////////////////////////////////////////////////
// CAL_Module - Initiator/Master side of MBINIT_CAL handshake protocol
// Sends CAL_Done request and waits for response
////////////////////////////////////////////////////////////////////////////////
module CAL_Module #(
    parameter SB_MSG_WIDTH = 4
) (
    input                           CLK,
    input                           rst_n,
    input                           i_MBINIT_PARAM_end,
    input                           i_falling_edge_busy,
    input                           i_Busy_SideBand,
    input      [SB_MSG_WIDTH-1:0]   i_RX_SbMessage,
    input                           i_msg_valid,
    
    output reg [SB_MSG_WIDTH-1:0]   o_TX_SbMessage,
    output reg                      o_ValidOutDatat_Module,
    output reg                      o_MBINIT_CAL_Module_end
);

////////////////////////////////////////////////////////////////////////////////
// State encoding
////////////////////////////////////////////////////////////////////////////////
localparam STATE_WIDTH = 2;
localparam [STATE_WIDTH-1:0] IDLE            = 2'd0;
localparam [STATE_WIDTH-1:0] SEND_REQUEST    = 2'd1;
localparam [STATE_WIDTH-1:0] WAIT_RESPONSE   = 2'd2;
localparam [STATE_WIDTH-1:0] DONE            = 2'd3;

////////////////////////////////////////////////////////////////////////////////
// Sideband message definitions
////////////////////////////////////////////////////////////////////////////////
localparam [SB_MSG_WIDTH-1:0] MSG_CAL_DONE_REQ  = 4'b0001;
localparam [SB_MSG_WIDTH-1:0] MSG_CAL_DONE_RESP = 4'b0010;

////////////////////////////////////////////////////////////////////////////////
// State registers
////////////////////////////////////////////////////////////////////////////////
reg [STATE_WIDTH-1:0] current_state, next_state;

////////////////////////////////////////////////////////////////////////////////
// State machine: Sequential logic
////////////////////////////////////////////////////////////////////////////////
always @(posedge CLK or negedge rst_n) begin
    if (!rst_n) begin
        current_state <= IDLE;
    end else begin
        current_state <= next_state;
    end
end

////////////////////////////////////////////////////////////////////////////////
// State machine: Next state logic
////////////////////////////////////////////////////////////////////////////////
always @(*) begin
    // Default: stay in current state
    next_state = current_state;
    
    case (current_state)
        IDLE: begin
            // Wait until PARAM phase ends and bus is not busy
            if (i_MBINIT_PARAM_end && !i_Busy_SideBand) begin
                next_state = SEND_REQUEST;
            end
        end
        
        SEND_REQUEST: begin
            // Return to IDLE if PARAM phase ends prematurely
            if (!i_MBINIT_PARAM_end) begin
                next_state = IDLE;
            end 
            // Wait for falling edge of busy (request acknowledged)
            else if (i_falling_edge_busy) begin
                next_state = WAIT_RESPONSE;
            end
        end
        
        WAIT_RESPONSE: begin
            // Return to IDLE if PARAM phase ends prematurely
            if (!i_MBINIT_PARAM_end) begin
                next_state = IDLE;
            end 
            // Check for valid response message
            else if (i_msg_valid && (i_RX_SbMessage == MSG_CAL_DONE_RESP)) begin
                next_state = DONE;
            end
        end
        
        DONE: begin
            // Stay in DONE until PARAM phase ends
            if (!i_MBINIT_PARAM_end) begin
                next_state = IDLE;
            end
        end
        
        default: begin
            next_state = IDLE;
        end
    endcase
end

////////////////////////////////////////////////////////////////////////////////
// Output logic: Registered outputs based on next state
////////////////////////////////////////////////////////////////////////////////
always @(posedge CLK or negedge rst_n) begin
    if (!rst_n) begin
        o_TX_SbMessage          <= 4'b0000;
        o_MBINIT_CAL_Module_end <= 1'b0;
        o_ValidOutDatat_Module  <= 1'b0;
    end else begin
        // Default values
        o_TX_SbMessage          <= 4'b0000;
        o_MBINIT_CAL_Module_end <= 1'b0;
        o_ValidOutDatat_Module  <= 1'b0;
        
        // Override defaults based on next state
        if (next_state == SEND_REQUEST) begin
            o_TX_SbMessage         <= MSG_CAL_DONE_REQ;
            o_ValidOutDatat_Module <= 1'b1;
        end 
        else if (next_state == DONE) begin
            o_MBINIT_CAL_Module_end <= 1'b1;
        end
    end
end

endmodule