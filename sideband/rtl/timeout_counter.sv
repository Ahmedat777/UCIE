// ================================================================
// TIME_OUT_COUNTER
//
// 8 ms sideband timeout (UCIe §4.5)
//
// System clock = 800 MHz
// Timeout cycles = 6,400,000
//
// Pattern generator additions (§4.5.3.2):
//   i_start_pattern_req       – arms counter when SBINIT pattern phase
//                               starts; no REQ message is sent in this
//                               phase so i_start never fires for it
//   i_rx_sb_pattern_samp_done – stops counter when partner pattern is
//                               locked (128 UI confirmed by RX FSM)
//   i_pattern_time_out        – TX-side: SB_PATTERN_GEN fires this after
//                               8 ms if partner never detects our pattern;
//                               passed straight through to o_time_out
//
// Stall detection (§4.5 / §7.1.3.1) — evaluated internally from RX inputs:
//   Condition 1: i_rx_msginfo == 16'hFFFF
//                applies to any received message
//   Condition 2: i_rx_msg_id == SB_RECAL_TRACK_TX_ADJUST_RESP
//                AND i_rx_msginfo[1:0] == 2'b11
//                ([15:2] reserved, [1:0]=11b means Stall per RECAL spec)
//
//   On stall: reset counter to zero, keep counting (fresh 8-ms window).
//   On normal stop (i_stop asserted, no stall): stop counting.
//   i_stop and stall are mutually exclusive per the logic below:
//   stall is checked first and takes priority.
// ================================================================
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

    // Pattern generator signals (SBINIT §4.5.3.2)
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
    //
    // Applied to both TX (i_msg_id) and RX (i_rx_msg_id).
    // If either direction carries an excluded state the timer
    // must not start / must not keep running.
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
    //
    //   i_stop && !w_stall  → stop counting (normal message received)
    //   i_stop &&  w_stall  → keep counting (stall: counter resets below)
    //   i_start             → start counting (encoder fired a packet)
    //   i_start_pattern_req → start counting (SBINIT pattern phase)
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
    //
    // w_stall resets counter to zero, keeps counting → fresh 8-ms
    // window for the partner (spec §4.5: "reset if Stall received").
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
    //
    //   counter == TIMEOUT_CYCLES-1 → normal 8 ms expiry
    //   i_pattern_time_out          → SBINIT TX-side expiry pass-through
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