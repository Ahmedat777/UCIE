// =============================================================================
// SB_FSM  –  UCIe Sideband Transmit FSM
//
// Spec reference: UCIe Specification Rev 3.0 Ver 1.0 (August 2025)
//   §4.5     "Timeout: Every state except RESET, Active, L1/L2, and TRAINERROR
//             has a residency timeout of 8 ms. The timeout counters must be
//             reset if a sideband message with Stall encoding is received."
//   §4.5.3.2 SBINIT – start-pattern sending (64-UI alternating, 1ms on/off,
//             timeout after 8 ms)
//   §7.1.2   Sideband packet formats (header + optional data flit)
//   §7.1.3.1 8-ms sideband response timeout per REQ/RESP handshake
//
// Encoding model (ltsm_encoder.sv):
//   ltsm_encoder takes i_msg_id + ltsm_data + ltsm_msginfo and in ONE clock
//   cycle produces BOTH o_ltsm_header (64-bit, with CP/DP computed) AND
//   o_ltsm_data together under a single i_ltsm_valid enable. There is only
//   ONE encoder — no separate header encoder and data encoder. The FSM
//   asserts o_encoder_enable (→ i_ltsm_valid) once; the encoder handles
//   both fields internally and signals completion via o_ltsmpckt_valid.
//
// 8-ms timeout is managed externally by TIME_OUT_COUNTER.
//
// Flow:
//   IDLE        → PATTERN_GEN  when i_start_pattern_req  (SBINIT phase)
//   IDLE        → ENCODING     when i_msg_valid
//   ENCODING    → WAIT_RSP     when i_ltsm_pckt_valid (encoder done, 1 cycle)
//   WAIT_RSP    → END_MESSAGE  when i_sb_ltsm_resp_rcvd OR i_time_out
//   END_MESSAGE → IDLE         after 2-cycle drain
// =============================================================================

import pckg::*;

module SB_FSM (
    input           i_clk,
    input           i_rst_n,

    // ── Control ───────────────────────────────────────────────────────────────
    input           i_start_pattern_req,    // SBINIT: begin start-pattern TX
    input           i_msg_valid,            // new SB message ready to encode

    // ── Encoder interface (ltsm_encoder) ──────────────────────────────────────
    // There is ONE encoder. o_encoder_enable maps to ltsm_encoder i_ltsm_valid.
    // The encoder produces o_ltsm_header AND o_ltsm_data in the same clock
    // cycle under that single enable — no separate data-path enable exists.
    // i_ltsm_pckt_valid = ltsm_encoder o_ltsmpckt_valid: packet fully built.
    input           i_ltsm_pckt_valid,      // encoder output valid (both header and data ready)

    // ── Response / timeout ────────────────────────────────────────────────────
    // i_sb_ltsm_resp_rcvd: SB_RX_FSM o_sb_ltsm_resp_rcvd
    //   Genuine LTSM RESP from remote die (msgcode[3:0]==4'hA or ==8'h02).
    //   Exits WAIT_RSP: transaction complete.
    // i_time_out: TIME_OUT_COUNTER o_time_out
    //   8 ms elapsed with no RESP. Exits WAIT_RSP: error path → TRAINERROR.
    input           i_sb_ltsm_resp_rcvd,
    input           i_time_out,

    input           i_start_pattern_done,   // pattern generator finished

    // ── Enable outputs ────────────────────────────────────────────────────────
    // o_encoder_enable: asserted for one cycle to trigger ltsm_encoder.
    // Maps to ltsm_encoder i_ltsm_valid. The encoder builds header AND data
    // internally under this single enable.
    output reg      o_encoder_enable,       // → ltsm_encoder i_ltsm_valid

    // ── Status ────────────────────────────────────────────────────────────────
    output reg      o_busy,
    output reg      o_rsp_timeout           // latched timeout → drive LTSM to TRAINERROR
);

    // ── State encoding ─────────────────────────────────────────────────────────
    localparam [2:0]
        IDLE        = 3'd0,
        PATTERN_GEN = 3'd1,   // SBINIT start-pattern phase
        ENCODING    = 3'd2,   // ltsm_encoder building the packet (1 cycle)
        WAIT_RSP    = 3'd3,   // waiting for remote RESP or 8-ms timeout
        END_MESSAGE = 3'd4;   // 2-cycle drain before accepting next message

    // ── State register ─────────────────────────────────────────────────────────
    reg [2:0] cs, ns;

    always_ff @(posedge i_clk or negedge i_rst_n) begin
        if (!i_rst_n) cs <= IDLE;
        else          cs <= ns;
    end

    // ── 2-cycle drain counter (END_MESSAGE) ────────────────────────────────────
    reg [1:0] drain_cnt;

    always_ff @(posedge i_clk or negedge i_rst_n) begin
        if (!i_rst_n)
            drain_cnt <= 2'b00;
        else if (cs == WAIT_RSP && ns == END_MESSAGE)
            drain_cnt <= 2'b00;
        else if (cs == END_MESSAGE)
            drain_cnt <= drain_cnt + 2'b01;
    end

    // ── Timeout flag (sticky; cleared in IDLE) ─────────────────────────────────
    always_ff @(posedge i_clk or negedge i_rst_n) begin
        if (!i_rst_n)
            o_rsp_timeout <= 1'b0;
        else if (cs == IDLE)
            o_rsp_timeout <= 1'b0;
        else if (i_time_out)
            o_rsp_timeout <= 1'b1;
    end

    // ── Next-state logic ───────────────────────────────────────────────────────
    always_comb begin
        ns = cs;

        case (cs)
            IDLE: begin
                if (i_start_pattern_req)
                    ns = PATTERN_GEN;
                else if (i_msg_valid)
                    ns = ENCODING;
            end

            PATTERN_GEN: begin
                if (i_start_pattern_done)
                    ns = IDLE;
            end

            ENCODING: begin
                // ltsm_encoder builds the full packet in one clock cycle.
                // i_ltsm_pckt_valid is o_ltsmpckt_valid from ltsm_encoder.
                if (i_ltsm_pckt_valid)
                    ns = WAIT_RSP;
            end

            WAIT_RSP: begin
                // TIME_OUT_COUNTER handles the 8-ms window and Stall resets
                if (i_sb_ltsm_resp_rcvd || i_time_out)
                    ns = END_MESSAGE;
            end

            END_MESSAGE: begin
                if (&drain_cnt)
                    ns = IDLE;
            end

            default: ns = IDLE;
        endcase
    end

    // ── Output logic ──────────────────────────────────────────────────────────
    always_comb begin
        o_encoder_enable = 1'b0;
        o_busy           = 1'b0;

        case (cs)
            IDLE: begin
                // Pre-arm encoder one cycle early when a message is waiting
                if (ns == ENCODING) begin
                    o_encoder_enable = 1'b1;
                end
            end

            PATTERN_GEN:
                o_busy = 1'b1;

            ENCODING: begin
                // Keep encoder enabled while it works (it completes in 1 cycle)
                o_encoder_enable = 1'b1;
                o_busy           = 1'b1;
            end

            WAIT_RSP:
                o_busy = 1'b1;

            END_MESSAGE:
                o_busy = ~(&drain_cnt);

            default: ;
        endcase
    end

endmodule : SB_FSM