module REPAIRMB_Module_Partner (
    input wire                  CLK,
    input wire                  rst_n,
    input wire                  MBINIT_REVERSALMB_end,
    input wire                  i_Busy_SideBand,
    input wire                  i_falling_edge_busy,
    input wire [3:0]            i_RX_SbMessage,
    input wire                  i_msg_valid,
    input wire [1:0]            i_Functional_Lanes,// from rx side band in msginfo
    input wire                  i_Done_Repeater,
    input wire                  i_Transmitter_initiated_D2C_en,//from module
    
    output reg                  o_Start_Repeater,
    output wire                 o_train_error,
    output reg                  o_MBINIT_REPAIRMB_Module_Partner_end,
    output reg                  o_tx_data_valid_repair_partner,
    output reg [3:0]            o_TX_SbMessage,
    output reg                  apply_repeater,
    output reg [1:0]            o_Functional_Lanes
);

    ////////////////////////////////////////////////////////////////////////////////
    // Sideband Messages
    ////////////////////////////////////////////////////////////////////////////////
    localparam [3:0] MBINIT_REPAIRMB_start_req           = 4'b0001;
    localparam [3:0] MBINIT_REPAIRMB_start_resp          = 4'b0010;
    localparam [3:0] MBINIT_REPAIRMB_apply_degrade_req   = 4'b0101;
    localparam [3:0] MBINIT_REPAIRMB_apply_degrade_resp  = 4'b0110;
    localparam [3:0] MBINIT_REPAIRMB_end_req             = 4'b0011;
    localparam [3:0] MBINIT_REPAIRMB_end_resp            = 4'b0100;

    ////////////////////////////////////////////////////////////////////////////////
    // States 
    ////////////////////////////////////////////////////////////////////////////////
    localparam [3:0] 
    IDLE                  = 4'd0,
    WAIT_START_REQ        = 4'd1,
    WAIT_SIDEBAND_FREE    = 4'd2,
    SEND_START_RESP       = 4'd3,
    WAIT_FOR_REQUEST      = 4'd4,
    VALIDATE_LANE_CONFIG  = 4'd5,
    PERFORM_REPEATER      = 4'd6,
    WAIT_SIDEBAND_DEGRADE = 4'd7,
    SEND_DEGRADE_RESP     = 4'd8,
    WAIT_SIDEBAND_END     = 4'd9,
    SEND_END_RESP         = 4'd10,
    REPAIR_COMPLETE       = 4'd11;

    ////////////////////////////////////////////////////////////////////////////////
    // Internal Signals
    ////////////////////////////////////////////////////////////////////////////////
    reg [3:0] CS, NS;
    reg       i_start_check;
    reg       i_second_check;
    reg       continue_flag;
    reg       end_req_received;  // ** Track if end_req was received**

    wire      o_done_check;
    wire      o_go_to_repeat;
    wire      o_continue;

    ////////////////////////////////////////////////////////////////////////////////
    // Checker Instantiation
    ////////////////////////////////////////////////////////////////////////////////
    CHECKER_REPAIRMB_Module_Partner u_checker (
        .CLK                            (CLK),
        .rst_n                          (rst_n),
        .i_start_check                  (i_start_check),
        .i_second_check                 (i_second_check),
        .i_Functional_Lanes             (i_Functional_Lanes),
        .i_Transmitter_initiated_D2C_en (i_Transmitter_initiated_D2C_en),
        .o_done_check                   (o_done_check),
        .o_go_to_repeat                 (o_go_to_repeat),
        .o_go_to_train_error            (o_train_error),
        .o_continue                     (o_continue)
    );

    ////////////////////////////////////////////////////////////////////////////////
    // State Register
    ////////////////////////////////////////////////////////////////////////////////
    always @(posedge CLK or negedge rst_n) begin
        if (!rst_n)
            CS <= IDLE;
        else
            CS <= NS;
    end

    ////////////////////////////////////////////////////////////////////////////////
    // Capture end_req if it arrives while busy
    ////////////////////////////////////////////////////////////////////////////////
    always @(posedge CLK or negedge rst_n) begin
        if (!rst_n)
            end_req_received <= 1'b0;
        else if (i_RX_SbMessage == MBINIT_REPAIRMB_end_req && i_msg_valid && 
                 (CS == SEND_DEGRADE_RESP || CS == WAIT_SIDEBAND_DEGRADE))
            end_req_received <= 1'b1;
        else if (NS == WAIT_SIDEBAND_END)
            end_req_received <= 1'b0;
    end

    ////////////////////////////////////////////////////////////////////////////////
    // Next State Logic
    ////////////////////////////////////////////////////////////////////////////////
    always @(*) begin
        NS = CS;
        case (CS)
            IDLE:
                if (MBINIT_REVERSALMB_end)
                    NS = WAIT_START_REQ;

            WAIT_START_REQ:
                if (!MBINIT_REVERSALMB_end)
                    NS = IDLE;
                else if (i_RX_SbMessage == MBINIT_REPAIRMB_start_req && i_msg_valid)
                    NS = WAIT_SIDEBAND_FREE;

            WAIT_SIDEBAND_FREE:
                if (!MBINIT_REVERSALMB_end)
                    NS = IDLE;
                else if (!i_Busy_SideBand)
                    NS = SEND_START_RESP;

            SEND_START_RESP:
                if (!MBINIT_REVERSALMB_end)
                    NS = IDLE;
                else if (i_falling_edge_busy)
                    NS = WAIT_FOR_REQUEST;

            WAIT_FOR_REQUEST: begin
                if (!MBINIT_REVERSALMB_end)
                    NS = IDLE;
                else if ((i_RX_SbMessage == MBINIT_REPAIRMB_end_req && i_msg_valid && continue_flag) ||
                         (end_req_received && continue_flag))  // ** Check stored flag**
                    NS = WAIT_SIDEBAND_END;
                else if (i_RX_SbMessage == MBINIT_REPAIRMB_apply_degrade_req &&
                         i_msg_valid && !i_Transmitter_initiated_D2C_en)
                    NS = VALIDATE_LANE_CONFIG;
                else if (apply_repeater)
                    NS = PERFORM_REPEATER;
                else if (i_RX_SbMessage == MBINIT_REPAIRMB_apply_degrade_req &&
                         i_msg_valid && i_second_check)
                    NS = VALIDATE_LANE_CONFIG;
            end

            VALIDATE_LANE_CONFIG:
                if (!MBINIT_REVERSALMB_end)
                    NS = IDLE;
                else if (o_done_check) begin
                    if (o_train_error)
                        NS = IDLE;
                    else
                        NS = WAIT_SIDEBAND_DEGRADE;
                end

            PERFORM_REPEATER:
                if (!MBINIT_REVERSALMB_end)
                    NS = IDLE;
                else if (i_Done_Repeater)
                    NS = WAIT_FOR_REQUEST;

            WAIT_SIDEBAND_DEGRADE:
                if (!MBINIT_REVERSALMB_end)
                    NS = IDLE;
                else if (!i_Busy_SideBand)
                    NS = SEND_DEGRADE_RESP;

            SEND_DEGRADE_RESP:
                if (!MBINIT_REVERSALMB_end)
                    NS = IDLE;
                else if (i_falling_edge_busy) begin
                    // **Check if end_req already arrived**
                    if (end_req_received && continue_flag)
                        NS = WAIT_SIDEBAND_END;
                    else
                        NS = WAIT_FOR_REQUEST;
                end

            WAIT_SIDEBAND_END:
                if (!MBINIT_REVERSALMB_end)
                    NS = IDLE;
                else if (!i_Busy_SideBand)
                    NS = SEND_END_RESP;

            SEND_END_RESP:
                if (!MBINIT_REVERSALMB_end)
                    NS = IDLE;
                else if (i_falling_edge_busy)
                    NS = REPAIR_COMPLETE;

            REPAIR_COMPLETE:
                if (!MBINIT_REVERSALMB_end)
                    NS = IDLE;
        endcase
    end

    ////////////////////////////////////////////////////////////////////////////////
    // Second Check Flag
    ////////////////////////////////////////////////////////////////////////////////
    always @(posedge CLK or negedge rst_n) begin
        if (!rst_n)
            i_second_check <= 1'b0;
        else if (i_Done_Repeater)
            i_second_check <= 1'b1;
        else if (NS == SEND_END_RESP)
            i_second_check <= 1'b0;
    end

    ////////////////////////////////////////////////////////////////////////////////
    // Output Logic
    ////////////////////////////////////////////////////////////////////////////////
    always @(posedge CLK or negedge rst_n) begin
        if (!rst_n) begin
            o_TX_SbMessage                       <= 4'b0;
            o_MBINIT_REPAIRMB_Module_Partner_end <= 1'b0;
            o_tx_data_valid_repair_partner       <= 1'b0;
            o_Functional_Lanes                   <= 2'b11;
            o_Start_Repeater                     <= 1'b0;
            i_start_check                        <= 1'b0;
            continue_flag                        <= 1'b0;
            apply_repeater                       <= 1'b0;
        end else begin
            o_TX_SbMessage                       <= 4'b0;
            o_MBINIT_REPAIRMB_Module_Partner_end <= 1'b0;
            o_tx_data_valid_repair_partner       <= 1'b0;
            o_Start_Repeater                     <= 1'b0;
            i_start_check                        <= 1'b0;

            if (NS == WAIT_SIDEBAND_DEGRADE) begin
                if (o_go_to_repeat) begin
                    o_Functional_Lanes <= i_Functional_Lanes;
                    apply_repeater     <= 1'b1;
                    continue_flag      <= 1'b0;
                end else if (o_continue) begin
                    continue_flag  <= 1'b1;
                    apply_repeater <= 1'b0;
                end
            end

            case (NS)
                SEND_START_RESP: begin
                    o_tx_data_valid_repair_partner <= 1'b1;
                    o_TX_SbMessage                 <= MBINIT_REPAIRMB_start_resp;
                end

                VALIDATE_LANE_CONFIG:
                    i_start_check <= 1'b1;

                PERFORM_REPEATER: begin
                    o_Start_Repeater <= 1'b1;
                    apply_repeater   <= 1'b0;
                    continue_flag    <= 1'b0; 
                end

                SEND_DEGRADE_RESP: begin
                    o_tx_data_valid_repair_partner <= 1'b1;
                    o_TX_SbMessage                 <= MBINIT_REPAIRMB_apply_degrade_resp;
                end

                SEND_END_RESP: begin
                    o_tx_data_valid_repair_partner <= 1'b1;
                    o_TX_SbMessage                 <= MBINIT_REPAIRMB_end_resp;
                    continue_flag                  <= 1'b0;
                end

                REPAIR_COMPLETE:
                    o_MBINIT_REPAIRMB_Module_Partner_end <= 1'b1;
            endcase
        end
    end

endmodule