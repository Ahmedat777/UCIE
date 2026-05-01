// =============================================================================
// SB_RX_FSM  –  UCIe Sideband Receive FSM
//
// Spec reference: UCIe Specification Rev 3.0 Ver 1.0 (August 2025)
//   §4.1.5   Sideband transmission
//   §4.5.3.2 SBINIT – start-pattern detection (128 UI = 2 × 64-bit words)
//   §7.1.2   Sideband packet formats
//   §7.1.3   Flow control and data integrity
//
// Architecture position:
//   SerDes  →  SB_RX_FIFO  →  ltsm_decoder  →  LTSM
//                    ↑               ↓
//                SB_RX_FSM    sb_error_handler
//                    ↓
//              TIME_OUT_COUNTER
//
// ── Responsibility ────────────────────────────────────────────────────────────
//   This FSM is CONTROL ONLY. It sequences the RX datapath without ever
//   inspecting header or data payloads. Specifically it handles:
//
//     1. Start-pattern detection  – counts i_de_ser_done pulses and checks
//                                   i_fifo_is_pattern (supplied by RX FIFO);
//                                   two consecutive matches = 128 UI locked.
//     2. Flit sequencing          – drives the RX FIFO write-side controls
//                                   (header phase, data phase, commit pulse)
//                                   and the read-enable to the decoder.
//     3. has_data sequencing      – uses i_fifo_has_data from the RX FIFO
//                                   to decide whether a data flit follows.
//     4. Reset detection          – observes i_msg_id / i_msg_id_valid from
//                                   ltsm_decoder; any reset-class message
//                                   returns the FSM to IDLE.
//
//   The FSM does NOT perform parity checks (CP or DP). All parity
//   verification is handled exclusively by sb_error_handler, which reads
//   i_header and i_data directly from the SB_RX_FIFO front entry.
//   The FSM does NOT hold or inspect any 64-bit data word.
//
// ── Pattern detection ─────────────────────────────────────────────────────────
//   The RX FIFO exposes i_fifo_is_pattern — a combinational flag that is HIGH
//   when the word currently in the FIFO assembly register equals
//   64'h5555_5555_5555_5555 (spec §4.5.3.2). The FSM relies on this flag
//   instead of decoding the raw word itself.
//
// ── Reset detection ───────────────────────────────────────────────────────────
//   i_msg_id / i_msg_id_valid come from ltsm_decoder (o_msg_id / o_valid).
//   Reset-class messages: SB_SBINIT_OUT_OF_RESET, SB_RDI_REQ_LINKRESET,
//   SB_RDI_RSP_LINKRESET.  Any of these forces the FSM back to IDLE.
// =============================================================================

import pckg::*;

module SB_RX_FSM (
    input  logic        i_clk,
    input  logic        i_rst_n,

    // ── Deserialiser interface ────────────────────────────────────────────────
    // Pulse only — raw 64-bit word goes directly into the RX FIFO write port.
    input  logic        i_de_ser_done,              // pulse: new 64-bit word captured by FIFO

    // ── RX FIFO status / pattern flag ────────────────────────────────────────
    input  logic        i_fifo_is_pattern,          // HIGH when FIFO asm-reg == SB_START_PATTERN
    input  logic        i_fifo_has_data,            // front entry carries a data flit
    input  logic        i_fifo_valid,               // RX FIFO has a committed entry ready

    // ── RX FIFO write-side controls ──────────────────────────────────────────
    output logic        o_rx_header_valid,          // phase-1: latch incoming word as header
    output logic        o_rx_data_valid,            // phase-2: latch incoming word as data flit
    output logic        o_write_commit,             // pulse: push assembled entry into FIFO

    // ── RX FIFO read-side control ─────────────────────────────────────────────
    output logic        o_fifo_read_enable,         // pop front entry (decoder ready to consume)

    // ── ltsm_decoder feedback (reset detection) ──────────────────────────────
    input  logic [10:0] i_msg_id,                   // ltsm_decoder o_msg_id (11-bit enum)
    input  logic        i_msg_id_valid,             // ltsm_decoder o_valid

    // ── Outputs to upper layers ───────────────────────────────────────────────
    output logic        o_rx_sb_start_pattern,      // request TX to send start-pattern (SBINIT)
    output logic        o_rx_sb_pattern_samp_done,  // 128 UI pattern locked
    output logic        o_msg_valid,                // complete packet in RX FIFO, ready for decoder
    output logic        o_de_ser_done_sampled        // registered ack to deserialiser
);

    // -------------------------------------------------------------------------
    // FSM state encoding
    // -------------------------------------------------------------------------
    localparam logic [2:0]
        IDLE           = 3'd0,   // waiting for first pattern word
        PATTERN_DETECT = 3'd1,   // first pattern seen, waiting for confirmation
        GENERAL_DECODE = 3'd2,   // pattern locked, waiting for next packet word
        HEADER_LATCH   = 3'd3,   // header word latched, decide next phase
        DATA_LATCH     = 3'd4;   // data flit expected from SerDes

    logic [2:0] cs, ns;

    // -------------------------------------------------------------------------
    // Reset-class message detection (drives return to IDLE)
    // -------------------------------------------------------------------------
    function automatic logic is_reset_msg(input logic [10:0] id);
        case (id)
            sb_msg_id'(SB_SBINIT_OUT_OF_RESET),
            sb_msg_id'(SB_RDI_REQ_LINKRESET),
            sb_msg_id'(SB_RDI_RSP_LINKRESET): return 1'b1;
            default:                           return 1'b0;
        endcase
    endfunction

    logic w_reset_msg;
    assign w_reset_msg = i_msg_id_valid && is_reset_msg(i_msg_id);

    // -------------------------------------------------------------------------
    // Pattern lock: track whether the previous word was also a pattern.
    // Two consecutive pattern words = 128 UI (spec §4.5.3.2).
    // -------------------------------------------------------------------------
    logic r_prev_pattern;

    always_ff @(posedge i_clk or negedge i_rst_n) begin
        if (!i_rst_n)
            r_prev_pattern <= 1'b0;
        else if (cs == IDLE)
            r_prev_pattern <= 1'b0;
        else if (i_de_ser_done)
            r_prev_pattern <= i_fifo_is_pattern;
    end

    // -------------------------------------------------------------------------
    // Deserialiser ack (registered)
    // -------------------------------------------------------------------------
    always_ff @(posedge i_clk or negedge i_rst_n) begin
        if (!i_rst_n) o_de_ser_done_sampled <= 1'b0;
        else          o_de_ser_done_sampled <= i_de_ser_done;
    end

    // -------------------------------------------------------------------------
    // FSM state register
    // -------------------------------------------------------------------------
    always_ff @(posedge i_clk or negedge i_rst_n) begin
        if (!i_rst_n) cs <= IDLE;
        else          cs <= ns;
    end

    // -------------------------------------------------------------------------
    // Next-state logic  (pure control — no payload inspection)
    // -------------------------------------------------------------------------
    always_comb begin
        ns = cs;

        case (cs)

            IDLE: begin
                if (i_de_ser_done && i_fifo_is_pattern)
                    ns = PATTERN_DETECT;
            end

            PATTERN_DETECT: begin
                if (w_reset_msg)
                    ns = IDLE;
                else if (i_de_ser_done) begin
                    if (i_fifo_is_pattern && r_prev_pattern)
                        ns = GENERAL_DECODE;   // 128 UI confirmed
                    else if (!i_fifo_is_pattern)
                        ns = IDLE;             // spurious word, restart
                end
            end

            GENERAL_DECODE: begin
                if (w_reset_msg)
                    ns = IDLE;
                else if (i_de_ser_done) begin
                    if (i_fifo_is_pattern)
                        ns = GENERAL_DECODE;   // keep-alive pattern; stay
                    else
                        ns = HEADER_LATCH;     // non-pattern word = header flit
                end
            end

            HEADER_LATCH: begin
                if (w_reset_msg)
                    ns = IDLE;
                else if (i_fifo_has_data)
                    ns = DATA_LATCH;           // data flit expected next
                else
                    ns = GENERAL_DECODE;       // header-only: commit and loop
            end

            DATA_LATCH: begin
                if (w_reset_msg)
                    ns = IDLE;
                else if (i_de_ser_done)
                    ns = GENERAL_DECODE;       // data captured; commit and loop
            end

            default: ns = IDLE;
        endcase
    end

    // -------------------------------------------------------------------------
    // Output logic  (combinational, registered below)
    // -------------------------------------------------------------------------
    logic r_start_pat, r_pat_done, r_msg_valid;
    logic r_hdr_valid, r_data_valid, r_commit, r_fifo_rd;

    always_comb begin
        r_start_pat  = 1'b0;
        r_pat_done   = 1'b0;
        r_msg_valid  = 1'b0;
        r_hdr_valid  = 1'b0;
        r_data_valid = 1'b0;
        r_commit     = 1'b0;
        r_fifo_rd    = 1'b0;

        case (cs)

            IDLE: begin
                if (ns == PATTERN_DETECT)
                    r_start_pat = 1'b1;
            end

            PATTERN_DETECT: begin
                if (ns == GENERAL_DECODE)
                    r_pat_done = 1'b1;
            end

            GENERAL_DECODE: begin
                // Non-pattern word arrived → tell FIFO to latch it as header
                if (i_de_ser_done && !i_fifo_is_pattern)
                    r_hdr_valid = 1'b1;
            end

            HEADER_LATCH: begin
                if (!i_fifo_has_data) begin
                    // Header-only packet: commit entry and notify decoder
                    r_commit    = 1'b1;
                    r_fifo_rd   = 1'b1;
                    r_msg_valid = 1'b1;
                end
                // has_data case: wait for DATA_LATCH before committing
            end

            DATA_LATCH: begin
                if (i_de_ser_done) begin
                    // Data flit arrived: latch, commit, notify decoder
                    r_data_valid = 1'b1;
                    r_commit     = 1'b1;
                    r_fifo_rd    = 1'b1;
                    r_msg_valid  = 1'b1;
                end
            end

            default: ;
        endcase
    end

    // -------------------------------------------------------------------------
    // Register all outputs
    // -------------------------------------------------------------------------
    always_ff @(posedge i_clk or negedge i_rst_n) begin
        if (!i_rst_n) begin
            o_rx_sb_start_pattern     <= 1'b0;
            o_rx_sb_pattern_samp_done <= 1'b0;
            o_msg_valid               <= 1'b0;
            o_rx_header_valid         <= 1'b0;
            o_rx_data_valid           <= 1'b0;
            o_write_commit            <= 1'b0;
            o_fifo_read_enable        <= 1'b0;
        end else begin
            o_rx_sb_start_pattern     <= r_start_pat;
            o_rx_sb_pattern_samp_done <= r_pat_done;
            o_msg_valid               <= r_msg_valid;
            o_rx_header_valid         <= r_hdr_valid;
            o_rx_data_valid           <= r_data_valid;
            o_write_commit            <= r_commit;
            o_fifo_read_enable        <= r_fifo_rd;
        end
    end

endmodule : SB_RX_FSM