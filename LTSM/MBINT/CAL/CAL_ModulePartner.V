////////////////////////////////////////////////////////////////////////////////
// CAL_ModulePartner - Responder/Slave side of MBINIT_CAL handshake protocol
// Receives CAL_Done request and sends response
////////////////////////////////////////////////////////////////////////////////
module CAL_ModulePartner (
    input               CLK,
    input               rst_n,
    input               i_MBINIT_PARAM_end,
    input [3:0]         i_RX_SbMessage,
    input               i_msg_valid,
    input               i_Busy_SideBand,
    input               i_falling_edge_busy,

    output reg          o_MBINIT_CAL_ModulePartner_end,
    output reg          o_ValidOutDatat_ModulePartner,
    output reg [3:0]    o_TX_SbMessage
);

////////////////////////////////////////////////////////////////////////////////
// State encoding
////////////////////////////////////////////////////////////////////////////////
localparam STATE_WIDTH = 3;
localparam [STATE_WIDTH-1:0] IDLE            = 3'd0;
localparam [STATE_WIDTH-1:0] WAIT_REQUEST    = 3'd1;
localparam [STATE_WIDTH-1:0] WAIT_BUS_FREE   = 3'd2;
localparam [STATE_WIDTH-1:0] SEND_RESPONSE   = 3'd3;
localparam [STATE_WIDTH-1:0] DONE            = 3'd4;

////////////////////////////////////////////////////////////////////////////////
// Sideband message definitions
////////////////////////////////////////////////////////////////////////////////
localparam [3:0] MSG_CAL_DONE_REQ  = 4'b0001;
localparam [3:0] MSG_CAL_DONE_RESP = 4'b0010;

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
            // Wait for PARAM phase to start
            if (i_MBINIT_PARAM_end) begin
                next_state = WAIT_REQUEST;
            end
        end
        
        WAIT_REQUEST: begin
            // Return to IDLE if PARAM phase ends prematurely
            if (!i_MBINIT_PARAM_end) begin
                next_state = IDLE;
            end 
            // Check for valid request message
            else if (i_msg_valid && (i_RX_SbMessage == MSG_CAL_DONE_REQ)) begin
                next_state = WAIT_BUS_FREE;
            end
        end
        
        WAIT_BUS_FREE: begin
            // Return to IDLE if PARAM phase ends prematurely
            if (!i_MBINIT_PARAM_end) begin
                next_state = IDLE;
            end 
            // Wait until bus is free before sending response
            else if (!i_Busy_SideBand) begin
                next_state = SEND_RESPONSE;
            end
        end
        
        SEND_RESPONSE: begin
            // Return to IDLE if PARAM phase ends prematurely
            if (!i_MBINIT_PARAM_end) begin
                next_state = IDLE;
            end 
            // Wait for falling edge of busy (response acknowledged)
            else if (i_falling_edge_busy) begin
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
        o_MBINIT_CAL_ModulePartner_end  <= 1'b0;
        o_TX_SbMessage                  <= 4'b0000;
        o_ValidOutDatat_ModulePartner   <= 1'b0;
    end else begin
        // Default values
        o_MBINIT_CAL_ModulePartner_end  <= 1'b0;
        o_TX_SbMessage                  <= 4'b0000;
        o_ValidOutDatat_ModulePartner   <= 1'b0;
        
        // Override defaults based on next state
        if (next_state == SEND_RESPONSE) begin
            o_TX_SbMessage                <= MSG_CAL_DONE_RESP;
            o_ValidOutDatat_ModulePartner <= 1'b1;
        end 
        else if (next_state == DONE) begin
            o_MBINIT_CAL_ModulePartner_end <= 1'b1;
        end
    end
end

endmodule