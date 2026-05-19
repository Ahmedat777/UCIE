import pckg::*;

module TIME_OUT_COUNTER
(
    input  logic      i_clk,
    input  logic      i_rst_n,
    input  logic      i_start,                   // start counting (msg sent by encoder)
    input  logic      i_stop,                    // any RX message received (from decoder)
    input  sb_msg_id  i_msg_id,                  // TX msg_id — exclude non-timed states
    input  sb_msg_id  i_rx_msg_id,               // RX msg_id — stall condition 2 check
    input  logic [15:0] i_rx_msginfo,            // RX MsgInfo — stall condition 1 & 2 check

    input  logic      i_start_pattern_req,       // SBINIT pattern phase entered → arm counter
    input  logic      i_rx_sb_pattern_samp_done, // partner pattern locked        → stop counter
    input  logic      i_pattern_time_out,        // TX-side 8 ms expired          → pass-through

    output logic      o_time_out
);
    // ------------------------------------------------------------
    // 8 ms @ 800 MHz
    // ------------------------------------------------------------
    localparam TIMEOUT_CYCLES = 6_400_000;
    localparam CNT_WIDTH = $clog2(TIMEOUT_CYCLES);

    logic [CNT_WIDTH-1:0] counter;
    logic counting;

    // ------------------------------------------------------------
    // Detect timeout-excluded states
    // ------------------------------------------------------------
    function automatic logic is_excluded(input sb_msg_id id);
        case (id)
            SB_TRAINERROR_ENTRY_REQ,
            SB_TRAINERROR_ENTRY_RESP,
            SB_RDI_REQ_ACTIVE,
            SB_RDI_RSP_ACTIVE,
            SB_RDI_REQ_L1,
            SB_RDI_RSP_L1,
            SB_RDI_REQ_L2,
            SB_RDI_RSP_L2:   return 1'b1;
            default:          return 1'b0;
        endcase
    endfunction

    wire tx_excluded = is_excluded(i_msg_id);
    wire rx_excluded = is_excluded(i_rx_msg_id);

    // ------------------------------------------------------------
    // Stall detection (evaluated from RX inputs, spec §4.5)
    //
    // Condition 1: any message with MsgInfo == 16'hFFFF
    // Condition 2: SB_RECAL_TRACK_TX_ADJUST_RESP with MsgInfo[1:0] == 2'b11
    //              [15:2] reserved; [1:0] = 11b encodes Stall
    //
    // Only meaningful when i_stop is asserted (a message was received).
    // ------------------------------------------------------------
    logic w_stall;
    always_comb begin
        w_stall = 1'b0;
        if (i_stop && !rx_excluded) begin
            if (i_rx_msginfo == 16'hFFFF)
                w_stall = 1'b1;
            else if (i_rx_msg_id  == SB_RECAL_TRACK_TX_ADJUST_RESP &&
                     i_rx_msginfo[1:0] == 2'b11)
                w_stall = 1'b1;
        end
    end

    // ------------------------------------------------------------
    // Start / stop / stall logic
    // ------------------------------------------------------------
    always_ff @(posedge i_clk or negedge i_rst_n) begin
        if (!i_rst_n)
            counting <= 1'b0;
        else if ((i_stop && !w_stall && !rx_excluded) || i_rx_sb_pattern_samp_done)
            counting <= 1'b0;
        else if ((i_start && !tx_excluded) || i_start_pattern_req)
            counting <= 1'b1;
        else if (o_time_out)
            counting <= 1'b0;
        // w_stall: counting unchanged — counter resets below
    end

    // ------------------------------------------------------------
    // Counter
    // ------------------------------------------------------------
    always_ff @(posedge i_clk or negedge i_rst_n) begin
        if (!i_rst_n)
            counter <= '0;
        else if (!counting)
            counter <= '0;
        else if (w_stall)
            counter <= '0;
        else if (counter < TIMEOUT_CYCLES)
            counter <= counter + 1'b1;
    end

    // ------------------------------------------------------------
    // Timeout generation
    // ------------------------------------------------------------
    always_ff @(posedge i_clk or negedge i_rst_n) begin
        if (!i_rst_n)
            o_time_out <= 1'b0;
        else if (i_pattern_time_out)
            o_time_out <= 1'b1;
        else if (counter == TIMEOUT_CYCLES-1)
            o_time_out <= 1'b1;
        else
            o_time_out <= 1'b0;
    end

endmodule : TIME_OUT_COUNTER