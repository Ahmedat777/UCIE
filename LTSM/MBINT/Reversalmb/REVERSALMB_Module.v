module REVERSALMB_Module (   
    input               CLK,
    input               rst_n,
    input               i_MBINIT_REPAIRVAL_end,
    input               i_REVERSAL_done, // from lfsr
    input [3:0]         i_Rx_SbMessage,
    input               i_Busy_SideBand, // from partner
    input               i_msg_valid,
    input               i_LaneID_Pattern_done,// from lfsr
    input               i_falling_edge_busy, 
    input [15:0]        i_REVERSAL_Result_SB, //from rx_sb when it responed with resp on result on i_Rx_SbMessage
    input               i_Start_Repeater,//from partner


    output reg [1:0]    o_MBINIT_REVERSALMB_LaneID_Pattern_En,// i_state at lfsr State input (IDLE, Clear_lfsr, PATTERN_LFSR, PER_LANE_IDE)
                                                              // here 11 to make PATTERN_LFSR
    output reg          o_MBINIT_ApplyReversal_En,       
    output reg          o_MBINIT_REVERSALMB_Module_end,
    output reg [3:0]    o_TX_SbMessage,
    output reg          o_tx_data_valid_reversal,
    output              o_train_error_req_reversalmb,
    output reg          o_Second_Clear_Error_Req,
    output reg          apply_repeater
);

integer  i;
reg [4:0] one_count;
reg DONE_CHECK;
reg [3:0] CS, NS;   // CS current state, NS next state
reg handle_error_req;
reg [1:0] clear_req_count;


////////////////////////////////////////////////////////////////////////////////
// Sideband messages
////////////////////////////////////////////////////////////////////////////////

localparam MBINIT_REVERSALMB_init_req           = 4'b0001;//1
localparam MBINIT_REVERSALMB_init_resp          = 4'b0010;//2
localparam MBINIT_REVERSALMB_clear_error_req    = 4'b0011;//3
localparam MBINIT_REVERSALMB_clear_error_resp   = 4'b0100;//4
localparam MBINIT_REVERSALMB_result_req         = 4'b0101;//5
localparam MBINIT_REVERSALMB_result_resp        = 4'b0110;//6
localparam MBINIT_REVERSALMB_done_req           = 4'b0111;//7
localparam MBINIT_REVERSALMB_done_resp          = 4'b1000;//8

////////////////////////////////////////////////////////////////////////////////
// State machine states
////////////////////////////////////////////////////////////////////////////////
localparam IDLE                                     = 0;
localparam SEND_REVERSAL_INIT_REQUEST               = 1;
localparam SEND_CLEAR_ERROR_REQUEST                 = 2;
localparam GENERATE_LANEID_PATTERN                  = 3;
localparam SEND_RESULT_REQUEST                      = 4;
localparam EVALUATE_REVERSAL_RESULT                 = 5;
localparam APPLY_LANE_REVERSAL                      = 6;
localparam SEND_DONE_REQUEST                        = 7;
localparam REVERSAL_COMPLETE                        = 8;
localparam WAIT_FOR_RESPONSE                        = 9;
localparam WAIT_BUSY_CLEAR_FOR_ERROR_REQ            = 10;
localparam WAIT_BUSY_CLEAR_FOR_RESULT_REQ           = 11;
localparam WAIT_BUSY_CLEAR_FOR_DONE_REQ             = 12;

assign o_train_error_req_reversalmb = (CS == EVALUATE_REVERSAL_RESULT && one_count < 8 && DONE_CHECK && handle_error_req);

wire entering_clear_error_req = (NS == SEND_CLEAR_ERROR_REQUEST) && (CS != SEND_CLEAR_ERROR_REQUEST);


always @(posedge CLK or negedge rst_n) begin
    if (!rst_n) begin
        clear_req_count        <= 2'd0;
        o_Second_Clear_Error_Req <= 1'b0;
    end else begin
        if (~i_MBINIT_REPAIRVAL_end) begin
            clear_req_count        <= 2'd0;
            o_Second_Clear_Error_Req <= 1'b0;
        end else if (entering_clear_error_req) begin
            if (clear_req_count < 2'd2)
                clear_req_count <= clear_req_count + 1'b1;

            // When count is 1 (about to become 2) -> this is the 2nd req
            if (clear_req_count == 2'd1)
                o_Second_Clear_Error_Req <= 1'b1;
            else
                o_Second_Clear_Error_Req <= 1'b0;
        end 
    end
end

////////////////////////////////////////////////////////////////////////////////
// State machine logic for the REVERSALMB_Module
////////////////////////////////////////////////////////////////////////////////
always @(posedge CLK or negedge rst_n) begin
    if (!rst_n) begin
        CS <= IDLE;
    end else begin
        CS <= NS;
    end
end



////////////////////////////////////////////////////////////////////////////////
// Next state logic for the REVERSALMB_Module
////////////////////////////////////////////////////////////////////////////////
always @(*) begin
    NS = CS; // Default to hold state
    case (CS)
        IDLE: begin
            if (i_MBINIT_REPAIRVAL_end && ~i_Busy_SideBand) NS = SEND_REVERSAL_INIT_REQUEST;
        end
        SEND_REVERSAL_INIT_REQUEST: begin
            if (~i_MBINIT_REPAIRVAL_end) NS = IDLE;
            else if (i_falling_edge_busy) NS = WAIT_FOR_RESPONSE;
        end
        WAIT_FOR_RESPONSE: begin
            if (~i_MBINIT_REPAIRVAL_end) NS = IDLE;
            else if (i_Rx_SbMessage == MBINIT_REVERSALMB_init_resp && i_msg_valid  || (i_Start_Repeater && ~o_Second_Clear_Error_Req)) NS = WAIT_BUSY_CLEAR_FOR_ERROR_REQ;
            else if (i_Rx_SbMessage == MBINIT_REVERSALMB_clear_error_resp && i_msg_valid) NS = GENERATE_LANEID_PATTERN;            
            else if (i_Rx_SbMessage == MBINIT_REVERSALMB_result_resp && i_msg_valid) NS = EVALUATE_REVERSAL_RESULT;
            else if (i_Rx_SbMessage == MBINIT_REVERSALMB_done_resp && i_msg_valid)  NS = REVERSAL_COMPLETE;
        end

        WAIT_BUSY_CLEAR_FOR_ERROR_REQ: begin
            if (~i_MBINIT_REPAIRVAL_end) NS = IDLE;
            else if (~i_Busy_SideBand) NS = SEND_CLEAR_ERROR_REQUEST;
        end
        
        SEND_CLEAR_ERROR_REQUEST: begin
            if (~i_MBINIT_REPAIRVAL_end) NS = IDLE;
            else if (i_falling_edge_busy) NS = WAIT_FOR_RESPONSE;
        end

        GENERATE_LANEID_PATTERN: begin
            if (~i_MBINIT_REPAIRVAL_end) NS = IDLE;
            else if (i_LaneID_Pattern_done) NS = WAIT_BUSY_CLEAR_FOR_RESULT_REQ;            
        end
        
        WAIT_BUSY_CLEAR_FOR_RESULT_REQ : begin
            if (~i_MBINIT_REPAIRVAL_end) NS = IDLE;
            else if (~i_Busy_SideBand) NS = SEND_RESULT_REQUEST;
        end
        
        SEND_RESULT_REQUEST: begin
            if (~i_MBINIT_REPAIRVAL_end) NS = IDLE;
            else if (i_falling_edge_busy) NS = WAIT_FOR_RESPONSE;     
        end

        EVALUATE_REVERSAL_RESULT: begin
            if (~i_MBINIT_REPAIRVAL_end) NS = IDLE; 
            else if (DONE_CHECK) begin
                if (one_count > 8) 
                    NS = WAIT_BUSY_CLEAR_FOR_DONE_REQ;
                else if (!handle_error_req) 
                    NS = APPLY_LANE_REVERSAL;
                else
                 NS = WAIT_BUSY_CLEAR_FOR_DONE_REQ;

            end
        end

        WAIT_BUSY_CLEAR_FOR_DONE_REQ: begin
            if (~i_MBINIT_REPAIRVAL_end) NS = IDLE;
            else if (~i_Busy_SideBand) NS = SEND_DONE_REQUEST;
        end
        
        APPLY_LANE_REVERSAL: begin
            if (~i_MBINIT_REPAIRVAL_end) NS = IDLE;
            else if (i_REVERSAL_done) NS = WAIT_BUSY_CLEAR_FOR_ERROR_REQ;
        end

        SEND_DONE_REQUEST: begin
            if (~i_MBINIT_REPAIRVAL_end) NS = IDLE;
            else if (i_falling_edge_busy) NS = WAIT_FOR_RESPONSE; 
        end

        REVERSAL_COMPLETE: begin
            if (~i_MBINIT_REPAIRVAL_end) NS = IDLE;
        end

        default: begin
            NS = IDLE;
        end
    endcase
end

always @(*) begin
    one_count = 0;  //initialize count variable.
    for (i = 0; i < 16; i = i + 1) begin
        //for all the bits.
        one_count = one_count + i_REVERSAL_Result_SB[i]; //Add the bit to the count. 
    end
    DONE_CHECK = 1; // Always done after loop completes
end

////////////////////////////////////////////////////////////////////////////////
// Registered output logic for the REVERSALMB_Module
////////////////////////////////////////////////////////////////////////////////
always @(posedge CLK or negedge rst_n) begin
    if (!rst_n) begin
        o_MBINIT_REVERSALMB_LaneID_Pattern_En <= 0;
        o_MBINIT_ApplyReversal_En  <= 0;
        o_MBINIT_REVERSALMB_Module_end        <= 0;
        o_TX_SbMessage                        <= 4'b0000;
        o_tx_data_valid_reversal                <= 0;
        handle_error_req                      <= 0;
        apply_repeater <= 0; 
    end else begin
        o_MBINIT_REVERSALMB_LaneID_Pattern_En <= 0;
        o_MBINIT_ApplyReversal_En  <= 0;
        o_MBINIT_REVERSALMB_Module_end        <= 0;
        o_TX_SbMessage                        <= 4'b0000;
        o_tx_data_valid_reversal                <= 0;
        apply_repeater <=0;

        case (NS)
            SEND_REVERSAL_INIT_REQUEST: begin
                o_tx_data_valid_reversal <= 1'b1;
                o_TX_SbMessage <= MBINI_REVERSALMB_init_req;
            end
            SEND_CLEAR_ERROR_REQUEST: begin
                o_tx_data_valid_reversal <= 1'b1;
                o_TX_SbMessage <= MBINIT_REVERSALMB_clear_error_req;
            end       
            GENERATE_LANEID_PATTERN: begin
                o_MBINIT_REVERSALMB_LaneID_Pattern_En <= 2'b11; // PERLANE
            end
            SEND_RESULT_REQUEST: begin
                o_tx_data_valid_reversal <= 1'b1;
                o_TX_SbMessage <= MBINIT_REVERSALMB_result_req;
            end
            APPLY_LANE_REVERSAL: begin
                o_MBINIT_ApplyReversal_En <= 1'b1;
                handle_error_req <= 1;
                apply_repeater <=1;
            end
            SEND_DONE_REQUEST: begin
                o_tx_data_valid_reversal <= 1'b1;
                o_TX_SbMessage <= MBINIT_REVERSALMB_done_req;
            end
            REVERSAL_COMPLETE: begin
                o_MBINIT_REVERSALMB_Module_end <= 1;
            end
            default: begin
                o_MBINIT_REVERSALMB_LaneID_Pattern_En <= 0;
                o_MBINIT_ApplyReversal_En  <= 0;
                o_MBINIT_REVERSALMB_Module_end        <= 0;
                o_TX_SbMessage                        <= 4'b0000;
                o_tx_data_valid_reversal                <= 0;   
                apply_repeater <=0;
 
            end
        endcase
    end
end
endmodule