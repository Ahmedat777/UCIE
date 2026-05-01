module REPAIRMB_Module (
    input wire                  CLK,
    input wire                  rst_n,
    input wire                  MBINIT_REVERSALMB_end,
    input wire [3:0]            i_RX_SbMessage,
    input wire                  i_Busy_SideBand,
    input wire                  i_falling_edge_busy,
    input wire                  i_msg_valid,
    input wire                  i_Start_Repeater,//from partner
    input wire                  apply_repeater,
    input wire                  i_d2c_tx_ack,
    input wire [15:0]           i_lanes_results_tx,
    
    output reg [3:0]            o_TX_SbMessage,
    output reg                  o_Done_Repeater,
    output reg                  o_MBINIT_REPAIRMB_Module_end,
    output reg                  o_tx_data_valid_repair,
    output reg [1:0]            o_Functional_Lanes,
    output reg                  o_Transmitter_initiated_D2C_en,
    output reg                  o_perlane_Transmitter_initiated_D2C,
    output reg                  o_mainband_Transmitter_initiated_D2C,
    output reg [2:0]            o_msg_info_repairmb
);

    // Sideband messages
    localparam [3:0] MBINIT_REPAIRMB_start_req          = 4'b0001;//1
    localparam [3:0] MBINIT_REPAIRMB_start_resp         = 4'b0010;//2
    localparam [3:0] MBINIT_REPAIRMB_apply_degrade_req  = 4'b0101;//5
    localparam [3:0] MBINIT_REPAIRMB_apply_degrade_resp = 4'b0110;//6
    localparam [3:0] MBINIT_REPAIRMB_end_req            = 4'b0011;//3
    localparam [3:0] MBINIT_REPAIRMB_end_resp           = 4'b0100;//4

    // States
    localparam [3:0] IDLE                = 4'd0,
                     SEND_START_REQ      = 4'd1,
                     WAIT_FOR_RESPONSE   = 4'd2,
                     PERFORM_D2C_TEST    = 4'd3,
                     ANALYZE_LANE_STATUS = 4'd4,
                     WAIT_SIDEBAND_READY = 4'd5,
                     SEND_DEGRADE_REQ    = 4'd6,
                     WAIT_SIDEBAND_END   = 4'd7,
                     SEND_END_REQ        = 4'd8,
                     REPAIR_COMPLETE     = 4'd9;

    reg [3:0] CS, NS;
    reg start_setup, valid_done, Go_to_done;

    wire done_setup;
    wire [1:0] w_Functional_Lanes;

    Functional_Lane_Setup u_lane (
        .CLK(CLK),
        .rst_n(rst_n),
        .start_setup(start_setup),
        .i_lanes_results_tx(i_lanes_results_tx),
        .done_setup(done_setup),
        .o_Functional_Lanes(w_Functional_Lanes)
    );

    always @(posedge CLK or negedge rst_n)
        if (!rst_n) CS <= IDLE;
        else CS <= NS;

    always @(*) begin
        NS = CS;
        case (CS)
            IDLE:
                if (MBINIT_REVERSALMB_end && !i_Busy_SideBand)
                    NS = SEND_START_REQ;

            SEND_START_REQ:
                if (!MBINIT_REVERSALMB_end) NS = IDLE;
                else if (i_falling_edge_busy) NS = WAIT_FOR_RESPONSE;

            WAIT_FOR_RESPONSE: begin
                if (!MBINIT_REVERSALMB_end) NS = IDLE;
                else if ((i_RX_SbMessage == MBINIT_REPAIRMB_start_resp && i_msg_valid) || i_Start_Repeater)
                    NS = PERFORM_D2C_TEST;
                else if (i_RX_SbMessage == MBINIT_REPAIRMB_apply_degrade_resp && i_msg_valid && !apply_repeater) begin
                    if (w_Functional_Lanes == 2'b11 || valid_done || Go_to_done)
                        NS = WAIT_SIDEBAND_END;
                    else
                        NS = PERFORM_D2C_TEST;
                end
                else if (i_RX_SbMessage == MBINIT_REPAIRMB_end_resp && i_msg_valid)
                    NS = REPAIR_COMPLETE;
            end

            PERFORM_D2C_TEST:
                if (!MBINIT_REVERSALMB_end) NS = IDLE;
                else if (i_d2c_tx_ack) NS = ANALYZE_LANE_STATUS;

                
            ANALYZE_LANE_STATUS:
                 if (!MBINIT_REVERSALMB_end) NS = IDLE;
                else if (done_setup) NS =WAIT_SIDEBAND_READY;
             
            WAIT_SIDEBAND_READY:
                if (!i_Busy_SideBand) NS = SEND_DEGRADE_REQ;

            SEND_DEGRADE_REQ:
                if (i_falling_edge_busy) NS = WAIT_FOR_RESPONSE;

            WAIT_SIDEBAND_END:
                if (!i_Busy_SideBand) NS = SEND_END_REQ;

            SEND_END_REQ:
                 if (i_Busy_SideBand) NS =  SEND_END_REQ;
               else if (i_falling_edge_busy) NS = WAIT_FOR_RESPONSE;
               
            REPAIR_COMPLETE:
                if (!MBINIT_REVERSALMB_end) NS = IDLE;
        endcase
    end

    always @(posedge CLK or negedge rst_n) begin
        if (!rst_n) begin
            o_TX_SbMessage <= 0;
            o_MBINIT_REPAIRMB_Module_end <= 0;
            o_tx_data_valid_repair <= 0;
            o_Functional_Lanes <= 2'b11;
            o_Transmitter_initiated_D2C_en <= 0;
            o_perlane_Transmitter_initiated_D2C <= 0;
            o_mainband_Transmitter_initiated_D2C <= 0;
            start_setup <= 0;
            o_Done_Repeater <= 0;
            o_msg_info_repairmb <= 0;
            valid_done <= 0;
            Go_to_done <= 0;
        end else begin
           o_TX_SbMessage <= 0;
            o_MBINIT_REPAIRMB_Module_end <= 0;
            o_tx_data_valid_repair <= 0;
            o_Transmitter_initiated_D2C_en <= 0;
            o_perlane_Transmitter_initiated_D2C <= 0;
            o_mainband_Transmitter_initiated_D2C <= 0;
            start_setup <= 0;
            o_Done_Repeater <= 0;
            o_msg_info_repairmb <= 0;

            if (i_Start_Repeater) valid_done <= 1'b1;
           if (i_RX_SbMessage == MBINIT_REPAIRMB_apply_degrade_resp && 
            (w_Functional_Lanes == 2'b10 || w_Functional_Lanes == 2'b01 || valid_done) && 
            ~apply_repeater && i_msg_valid) begin
            Go_to_done <= 1;
            end
            if(i_Start_Repeater)o_Done_Repeater <= 1;

            case (NS)
                SEND_START_REQ: begin
                    o_tx_data_valid_repair <= 1;
                    o_TX_SbMessage <= MBINIT_REPAIRMB_start_req;
                end

                PERFORM_D2C_TEST: begin
                    o_Transmitter_initiated_D2C_en <= 1;
                    o_perlane_Transmitter_initiated_D2C <= 1;
                end

                ANALYZE_LANE_STATUS:
                    start_setup <= 1;

                SEND_DEGRADE_REQ: begin
                    o_tx_data_valid_repair <= 1;
                    o_TX_SbMessage <= MBINIT_REPAIRMB_apply_degrade_req;
                    o_Functional_Lanes <= w_Functional_Lanes;
                    o_msg_info_repairmb <= {1'b0, w_Functional_Lanes};
                end

                SEND_END_REQ: begin
                    o_tx_data_valid_repair <= 1;
                    o_TX_SbMessage <= MBINIT_REPAIRMB_end_req;
                    valid_done <= 0;
                    Go_to_done <= 0;
                end

                REPAIR_COMPLETE:
                    o_MBINIT_REPAIRMB_Module_end <= 1;
            endcase
        end
    end
endmodule
