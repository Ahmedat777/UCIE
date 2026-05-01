module REVERSALMB_ModulePartner (
    input wire              CLK,
    input wire              rst_n,
    input wire              i_MBINIT_REPAIRVAL_end,
    input wire [15:0]       i_REVERSAL_Result_logged,// pattern comparator
    input wire [3:0]        i_Rx_SbMessage,
    input wire              i_falling_edge_busy,
    input wire              i_Busy_SideBand,
    input                   i_msg_valid,
    input wire              apply_repeater,
    input wire              i_Second_Clear_Error_Req,

    output reg [15:0]       o_REVERSAL_Pattern_Result_logged, // to sb 
    output reg [3:0]        o_TX_SbMessage,
    output reg [1:0]        o_Clear_Pattern_Comparator,    // i_state at pattern_comparator  00 IDLE  01 CLEAR_LFSR 10 PATTERN_LFSR 11 PER_LANE_IDE
    output reg              o_MBINIT_REVERSALMB_ModulePartner_end,
    output reg              o_tx_data_valid_reversal,
    output reg              o_Start_Repeater,
    output reg              o_ValidDataFieldParameters_modulePartner
);


reg [3:0] CS, NS;   // CS current state, NS next state

////////////////////////////////////////////////////////////////////////////////
// clear_error_req counter
// Counts how many times SEND_CLEAR_ERROR_RESPONSE is entered.
// On the 2nd occurrence, o_Start_Repeater is asserted.
////////////////////////////////////////////////////////////////////////////////
reg [1:0] clear_error_count;  // 2-bit counter, we only need to track up to 2


////////////////////////////////////////////////////////////////////////////////
// Sideband messages
////////////////////////////////////////////////////////////////////////////////

localparam MBINIT_REVERSALMB_init_req            = 4'b0001;
localparam MBINIT_REVERSALMB_init_resp          = 4'b0010;
localparam MBINIT_REVERSALMB_clear_error_req    = 4'b0011;
localparam MBINIT_REVERSALMB_clear_error_resp   = 4'b0100;
localparam MBINIT_REVERSALMB_result_req         = 4'b0101;
localparam MBINIT_REVERSALMB_result_resp        = 4'b0110;
localparam MBINIT_REVERSALMB_done_req           = 4'b0111;
localparam MBINIT_REVERSALMB_done_resp          = 4'b1000;


////////////////////////////////////////////////////////////////////////////////
// State machine states ModulePartner
////////////////////////////////////////////////////////////////////////////////
localparam IDLE                                     = 0;
localparam WAIT_FOR_INIT_REQUEST                    = 1;
localparam WAIT_BUSY_CLEAR_FOR_INIT_RESP            = 2;
localparam SEND_INIT_RESPONSE                       = 3;
localparam WAIT_FOR_REQUEST                         = 4;
localparam WAIT_BUSY_CLEAR_FOR_ERROR_RESP           = 5;
localparam SEND_CLEAR_ERROR_RESPONSE                = 6;
localparam WAIT_BUSY_CLEAR_FOR_RESULT_RESP          = 7;
localparam SEND_RESULT_RESPONSE                     = 8;
localparam WAIT_BUSY_CLEAR_FOR_DONE_RESP            = 9;
localparam SEND_DONE_RESPONSE                       = 10;
localparam REVERSAL_PARTNER_COMPLETE                = 11;
localparam WAIT_FOR_SECONDE_REQ                     = 12;

wire entering_clear_error_resp = (NS == WAIT_BUSY_CLEAR_FOR_ERROR_RESP) && (CS != WAIT_BUSY_CLEAR_FOR_ERROR_RESP); // 6 = SEND_CLEAR_ERROR_RESPONSE
always @(posedge CLK or negedge rst_n) begin
    if (!rst_n) begin
        clear_error_count <= 2'd0;
        o_Start_Repeater  <= 1'b0;
    end else begin
        if (~i_MBINIT_REPAIRVAL_end) begin
            // Reset counter when handshake deasserted
            clear_error_count <= 2'd0;
            o_Start_Repeater  <= 1'b0;
        end else if (entering_clear_error_resp) begin
            if (clear_error_count < 2'd2)
                clear_error_count <= clear_error_count + 1'b1;
            // On the 2nd clear_error_req -> assert o_Start_Repeater
            if (clear_error_count == 2'd1)   // about to become 2
                o_Start_Repeater <= 1'b1;
        end
    end
end
////////////////////////////////////////////////////////////////////////////////
// State machine Transition for the REVERSALMB_ModulePartner
////////////////////////////////////////////////////////////////////////////////
    always @(posedge CLK or negedge rst_n) begin
        if (!rst_n) begin
            CS <= IDLE;
        end else begin
            CS <= NS;
        end
    end


////////////////////////////////////////////////////////////////////////////////
// Next state logic for the REVERSALMB_ModulePartner
////////////////////////////////////////////////////////////////////////////////
    always @(*) begin
        NS = CS;
        case (CS)
            IDLE: begin
                if (i_MBINIT_REPAIRVAL_end) NS = WAIT_FOR_INIT_REQUEST;
            end
            WAIT_FOR_INIT_REQUEST: begin
                if (~i_MBINIT_REPAIRVAL_end) NS = IDLE;
                else if (i_Rx_SbMessage == MBINI_REVERSALMB_init_req && i_msg_valid) NS = WAIT_BUSY_CLEAR_FOR_INIT_RESP;
            end
            WAIT_BUSY_CLEAR_FOR_INIT_RESP: begin
                if (~i_MBINIT_REPAIRVAL_end) NS = IDLE;
                else if (~i_Busy_SideBand) NS = SEND_INIT_RESPONSE;
            end
            SEND_INIT_RESPONSE: begin
                if (~i_MBINIT_REPAIRVAL_end) NS = IDLE;
                else if (i_falling_edge_busy) NS = WAIT_FOR_REQUEST;
            end
            WAIT_FOR_REQUEST: begin
                if (~i_MBINIT_REPAIRVAL_end) NS = IDLE;
                else if (apply_repeater) NS = WAIT_FOR_SECONDE_REQ;
                else if (i_Rx_SbMessage == MBINIT_REVERSALMB_clear_error_req && i_msg_valid && ~i_Second_Clear_Error_Req ) NS = WAIT_BUSY_CLEAR_FOR_ERROR_RESP;
                else if (i_Rx_SbMessage == MBINIT_REVERSALMB_result_req && i_msg_valid) NS = WAIT_BUSY_CLEAR_FOR_RESULT_RESP;
                else if (i_Rx_SbMessage == MBINIT_REVERSALMB_done_req && i_msg_valid && ~apply_repeater) NS = WAIT_BUSY_CLEAR_FOR_DONE_RESP;
            end
            WAIT_FOR_SECONDE_REQ: begin
                if (~i_MBINIT_REPAIRVAL_end) NS = IDLE;
                else if (i_Rx_SbMessage == MBINIT_REVERSALMB_clear_error_req && i_msg_valid) NS = WAIT_BUSY_CLEAR_FOR_ERROR_RESP;
            end
            WAIT_BUSY_CLEAR_FOR_ERROR_RESP: begin
                if (~i_MBINIT_REPAIRVAL_end) NS = IDLE;
                else if (~i_Busy_SideBand) NS = SEND_CLEAR_ERROR_RESPONSE;
            end
            SEND_CLEAR_ERROR_RESPONSE: begin
                if (~i_MBINIT_REPAIRVAL_end) NS = IDLE;
                else if (i_falling_edge_busy) NS = WAIT_FOR_REQUEST;
            end
            WAIT_BUSY_CLEAR_FOR_RESULT_RESP: begin
                if (~i_MBINIT_REPAIRVAL_end) NS = IDLE;
                else if (~i_Busy_SideBand) NS = SEND_RESULT_RESPONSE;
            end
            SEND_RESULT_RESPONSE: begin
                if (~i_MBINIT_REPAIRVAL_end) NS = IDLE;
                else if (i_falling_edge_busy && apply_repeater) NS = WAIT_FOR_SECONDE_REQ;
                else if (i_falling_edge_busy) NS = WAIT_FOR_REQUEST;
            end
            WAIT_BUSY_CLEAR_FOR_DONE_RESP: begin
                if (~i_MBINIT_REPAIRVAL_end) NS = IDLE;
                else if (~i_Busy_SideBand) NS = SEND_DONE_RESPONSE;
            end
            SEND_DONE_RESPONSE: begin
                if (~i_MBINIT_REPAIRVAL_end) NS = IDLE;
                else if (i_falling_edge_busy) NS = REVERSAL_PARTNER_COMPLETE;
            end
            REVERSAL_PARTNER_COMPLETE: begin
                if (~i_MBINIT_REPAIRVAL_end) NS = IDLE;
            end
            default: NS = IDLE;
        endcase
    end

    ////////////////////////////////////////////////////////////////////////////////
    // Registered output logic for the REVERSALMB_ModulePartner
    ////////////////////////////////////////////////////////////////////////////////
    always @(posedge CLK or negedge rst_n) begin
        if (!rst_n) begin
            o_REVERSAL_Pattern_Result_logged        <= 16'b0;
            o_TX_SbMessage                          <= 4'b0000;
            o_Clear_Pattern_Comparator              <= 2'b11;
            o_MBINIT_REVERSALMB_ModulePartner_end   <= 0;
            o_tx_data_valid_reversal                <= 0;
            o_ValidDataFieldParameters_modulePartner<= 0;
        end else begin
            o_tx_data_valid_reversal                 <= 1'b0;
            o_TX_SbMessage                           <= 4'b0000;
            o_ValidDataFieldParameters_modulePartner <= 1'b0;
            o_MBINIT_REVERSALMB_ModulePartner_end    <= 1'b0;
            case (NS)
                IDLE: begin
                    o_Clear_Pattern_Comparator  <= 2'b00;
                end
                SEND_INIT_RESPONSE: begin
                    o_tx_data_valid_reversal <= 1'b1;
                    o_TX_SbMessage <= MBINIT_REVERSALMB_init_resp;
                end
                SEND_CLEAR_ERROR_RESPONSE: begin
                    o_tx_data_valid_reversal   <= 1'b1;
                    o_TX_SbMessage             <= MBINIT_REVERSALMB_clear_error_resp;
                    o_Clear_Pattern_Comparator <= 2'b01;
                end
                SEND_RESULT_RESPONSE: begin
                    o_tx_data_valid_reversal                 <= 1'b1;
                    o_TX_SbMessage                           <= MBINIT_REVERSALMB_result_resp;
                    o_ValidDataFieldParameters_modulePartner <= 1'b1;
                    o_REVERSAL_Pattern_Result_logged         <= i_REVERSAL_Result_logged;
                end
                SEND_DONE_RESPONSE: begin
                    o_tx_data_valid_reversal <= 1'b1;
                    o_TX_SbMessage           <= MBINIT_REVERSALMB_done_resp;
                end
                REVERSAL_PARTNER_COMPLETE: begin
                    o_ValidDataFieldParameters_modulePartner <= 0;
                    o_tx_data_valid_reversal                 <= 1'b0;
                    o_MBINIT_REVERSALMB_ModulePartner_end    <= 1;
                end
                default: begin
                    o_REVERSAL_Pattern_Result_logged        <= 16'b0;
                    o_TX_SbMessage                          <= 4'b0000;
                    o_Clear_Pattern_Comparator              <= 2'b11;
                    o_MBINIT_REVERSALMB_ModulePartner_end   <= 0;
                    o_tx_data_valid_reversal                <= 0;
                    o_ValidDataFieldParameters_modulePartner<= 0;
                end
            endcase
        end
    end

endmodule