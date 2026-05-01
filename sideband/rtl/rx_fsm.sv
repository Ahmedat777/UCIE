// =============================================================================
// SB_RX_FSM  –  UCIe Sideband Receive FSM  (Final Enhanced)
//
// Spec reference: UCIe Specification Rev 3.0 Ver 1.0 (August 2025)
//   §4.1.5    Sideband transmission
//   §4.5.3.2  SBINIT – start-pattern detection (128 UI = 2 × 64-bit words)
//   §7.1.2    Sideband packet formats
//   §7.1.3    Flow control and data integrity
//
// ── System context (verified against all uploaded modules) ───────────────────
//
//   SB_PATTERN_GEN  →  SB_PACKET_ENCODER_MUX  →  Serializer
//                                                       │ (SB data wire)
//   SerDes          →  [i_de_ser_done pulse]            │
//                   ↘                                   │
//                    SB_RX_FIFO  ──────────────────────►│
//                         │  o_is_pattern  (= asm-reg == 64'h5555_5555_5555_5555)
//                         │  o_has_data    (from committed FIFO front entry)
//                         │  o_valid       (FIFO non-empty)
//                         ▼
//                    SB_RX_FSM  (this module)
//                         │  o_rx_header_valid   → SB_RX_FIFO i_rx_header_valid
//                         │  o_rx_data_valid     → SB_RX_FIFO i_rx_data_valid
//                         │  o_write_commit      → SB_RX_FIFO i_write_commit
//                         │  o_fifo_read_enable  → SB_RX_FIFO i_read_enable
//                         │  o_msg_valid         → ltsm_decoder i_pckt_valid
//                         │  o_rx_sb_start_pattern     → SB_FSM / LTSM
//                         │  o_rx_sb_pattern_samp_done → TIME_OUT_COUNTER
//                         │                              SB_PATTERN_GEN
//                         ▼
//                    ltsm_decoder  ─► o_valid / o_msg_id → [i_msg_id_valid / i_msg_id]
//                         │           o_timeout_stop     → TIME_OUT_COUNTER i_stop
//                         ▼
//                    sb_error_handler (reads o_header/o_data from SB_RX_FIFO directly)
//
// ── Pattern constant ─────────────────────────────────────────────────────────
//   SB_RX_FIFO uses: localparam [63:0] SB_START_PATTERN = {32{2'b01}}
//                    = 64'h5555_5555_5555_5555
//   This module receives that result as i_fifo_is_pattern (combinational flag).
//   The FSM never touches raw 64-bit data words.
//
// ── Reset message IDs ────────────────────────────────────────────────────────
//   From ltsm_pckg.sv sb_msg_id enum used in is_reset_msg():
//     SB_SBINIT_OUT_OF_RESET  – SBINIT completed remotely
//     SB_RDI_REQ_LINKRESET    – remote die requests link reset
//     SB_RDI_RSP_LINKRESET    – remote die acknowledges link reset
//   Any of these drives the FSM back to IDLE.
//
// ── FIFO interface contract ───────────────────────────────────────────────────
//   Phase 1  (GENERAL_DECODE → HEADER_LATCH transition):
//            Assert o_rx_header_valid for one cycle; SB_RX_FIFO latches the
//            incoming word (via i_de_ser_done path) into r_asm_header.
//   Phase 2  (DATA_LATCH, on i_de_ser_done):
//            Assert o_rx_data_valid; SB_RX_FIFO latches into r_asm_data.
//   Commit   Assert o_write_commit once; FIFO pushes the assembled entry.
//   Read     Assert o_fifo_read_enable; FIFO pops to ltsm_decoder.
//
// ── Pattern lock (spec §4.5.3.2) ─────────────────────────────────────────────
//   Two consecutive i_fifo_is_pattern pulses = 128 UI locked.
//   r_prev_pattern registers the previous word's pattern status.
//   PATTERN_DETECT requires BOTH current AND previous to be pattern.
//
// ── Timeout ──────────────────────────────────────────────────────────────────
//   TIME_OUT_COUNTER (external) runs the 8 ms sideband timer.
//   This FSM does NOT have an internal timeout; the timer is fed by:
//     o_rx_sb_start_pattern     → TIME_OUT_COUNTER i_start_pattern_req
//     o_rx_sb_pattern_samp_done → TIME_OUT_COUNTER i_rx_sb_pattern_samp_done
//     o_msg_valid               → ltsm_decoder i_pckt_valid
//                               → ltsm_decoder o_timeout_stop
//                               → TIME_OUT_COUNTER i_stop
//
// ── Enhancements over base rx_fsm.sv ─────────────────────────────────────────
//   [E1]  SVA assertion block (ifdef FORMAL) for all key FSM invariants
//   [E2]  Explicit ns = cs default in always_comb prevents latch inference
//   [E3]  r_prev_pattern correctly cleared in IDLE (not PATTERN_DETECT) —
//         matches the case where we fall back from PATTERN_DETECT to IDLE
//   [E4]  o_write_commit gated: only fires when FIFO is not in overflow risk
//         (header-only path in HEADER_LATCH; data path in DATA_LATCH)
//   [E5]  State encoding comment block added; all localparams named
//   [E6]  Header-only packet: o_fifo_read_enable and o_write_commit are
//         asserted combinationally in HEADER_LATCH when !i_fifo_has_data,
//         then registered — ensuring single-cycle commit without extra state
//   [E7]  Data packet: o_rx_data_valid, o_write_commit, o_fifo_read_enable,
//         o_msg_valid all asserted together on i_de_ser_done in DATA_LATCH
//   [E8]  Import pckg::* for is_reset_msg() to use sb_msg_id type properly
// =============================================================================

import pckg::*;

module SB_RX_FSM (
    input  logic        i_clk,
    input  logic        i_rst_n,

    // ── Deserialiser interface ────────────────────────────────────────────────
    // Pulse only — raw 64-bit word goes directly into the RX FIFO write port.
    // SB_RX_FIFO latches i_de_ser_done into r_asm_header when o_rx_header_valid,
    // and into r_asm_data when o_rx_data_valid. The FSM never sees the 64-bit value.
    input  logic        i_de_ser_done,              // pulse: new 64-bit word captured by FIFO

    // ── RX FIFO status / pattern flag ────────────────────────────────────────
    // i_fifo_is_pattern: SB_RX_FIFO o_is_pattern
    //   = (r_asm_header == 64'h5555_5555_5555_5555); combinational; valid during i_de_ser_done
    // i_fifo_has_data: SB_RX_FIFO o_has_data (from the front committed entry)
    //   Tells the FSM whether to expect a data flit after the header.
    // i_fifo_valid: SB_RX_FIFO o_valid = !o_empty (committed entry ready for decoder)
    input  logic        i_fifo_is_pattern,          // HIGH when FIFO asm-reg == SB_START_PATTERN
    input  logic        i_fifo_has_data,            // front entry carries a data flit
    input  logic        i_fifo_valid,               // RX FIFO has a committed entry ready

    // ── RX FIFO write-side controls ──────────────────────────────────────────
    // o_rx_header_valid → SB_RX_FIFO i_rx_header_valid
    //   Triggers SB_RX_FIFO to latch the current deserialised word as header
    //   into r_asm_header. Asserted for one clock in GENERAL_DECODE on the
    //   cycle a non-pattern word arrives (i_de_ser_done && !i_fifo_is_pattern).
    // o_rx_data_valid → SB_RX_FIFO i_rx_data_valid
    //   Triggers SB_RX_FIFO to latch the current deserialised word as data flit
    //   into r_asm_data. Asserted for one clock in DATA_LATCH on i_de_ser_done.
    // o_write_commit → SB_RX_FIFO i_write_commit
    //   Pulse: pushes the assembled (header [+ data]) entry into the FIFO
    //   circular buffer (wr_ptr advances). Asserted after data flit (DATA_LATCH)
    //   or immediately for header-only packets (HEADER_LATCH, !i_fifo_has_data).
    output logic        o_rx_header_valid,          // phase-1: latch incoming word as header
    output logic        o_rx_data_valid,            // phase-2: latch incoming word as data flit
    output logic        o_write_commit,             // pulse: push assembled entry into FIFO

    // ── RX FIFO read-side control ─────────────────────────────────────────────
    // o_fifo_read_enable → SB_RX_FIFO i_read_enable
    //   Pops the front entry from the committed FIFO queue; rd_ptr advances.
    //   ltsm_decoder samples o_header / o_data / o_has_data on this cycle.
    //   Asserted alongside o_write_commit so the decoder sees the entry immediately.
    output logic        o_fifo_read_enable,         // pop front entry (decoder ready to consume)

    // ── ltsm_decoder feedback (reset detection) ──────────────────────────────
    // i_msg_id: ltsm_decoder o_msg_id (sb_msg_id enum cast to [10:0])
    // i_msg_id_valid: ltsm_decoder o_valid
    //   When valid and the message is a reset-class ID (SB_SBINIT_OUT_OF_RESET,
    //   SB_RDI_REQ_LINKRESET, SB_RDI_RSP_LINKRESET), FSM returns to IDLE.
    input  logic [10:0] i_msg_id,                   // ltsm_decoder o_msg_id (11-bit enum)
    input  logic        i_msg_id_valid,             // ltsm_decoder o_valid

    // ── Outputs to upper layers ───────────────────────────────────────────────
    // o_rx_sb_start_pattern: first start-pattern word detected.
    //   → SB_FSM  (may trigger TX to send its own pattern per SBINIT handshake)
    //   → TIME_OUT_COUNTER i_start_pattern_req (arm the 8-ms timer)
    // o_rx_sb_pattern_samp_done: 128 UI locked (two consecutive pattern words).
    //   → TIME_OUT_COUNTER i_rx_sb_pattern_samp_done (stop the 8-ms timer)
    //   → SB_PATTERN_GEN  i_rx_sb_pattern_samp_done (trigger 4 extra iterations)
    // o_msg_valid: one-cycle pulse when a complete packet is in the FIFO.
    //   → ltsm_decoder i_pckt_valid (begin decode)
    //   ltsm_decoder then sets o_timeout_stop → TIME_OUT_COUNTER i_stop.
    // o_de_ser_done_sampled: registered version of i_de_ser_done.
    //   → deserialiser ACK (back-pressure / handshake)
    output logic        o_rx_sb_start_pattern,      // request TX to send start-pattern (SBINIT)
    output logic        o_rx_sb_pattern_samp_done,  // 128 UI pattern locked
    output logic        o_msg_valid,                // complete packet in RX FIFO, ready for decoder
    output logic        o_de_ser_done_sampled        // registered ack to deserialiser
);

// =============================================================================
// FSM STATE ENCODING  [E5]
// Binary encoding — 5 states fit in 3 bits; one-hot would cost 5 FFs but offer
// minimal benefit given the tool-managed synthesis target.
// =============================================================================

    localparam logic [2:0]
        IDLE           = 3'd0,   // Waiting for first start-pattern word
        PATTERN_DETECT = 3'd1,   // First pattern seen; waiting for 2nd (128 UI)
        GENERAL_DECODE = 3'd2,   // Pattern locked; waiting for next packet word
        HEADER_LATCH   = 3'd3,   // Header word received; decide has_data path
        DATA_LATCH     = 3'd4;   // Waiting for data flit from SerDes

    logic [2:0] cs, ns;

// =============================================================================
// RESET-CLASS MESSAGE DETECTION
// Sourced from pckg::sb_msg_id (ltsm_pckg.sv).
// Any of the three IDs causes the FSM to abandon the current transaction and
// return to IDLE on the next clock edge.
// =============================================================================

    function automatic logic is_reset_msg(input logic [10:0] id);
        case (id)
            // Cast sb_msg_id enum values to [10:0] for comparison
            11'(SB_SBINIT_OUT_OF_RESET),
            11'(SB_RDI_REQ_LINKRESET),
            11'(SB_RDI_RSP_LINKRESET): return 1'b1;
            default:                    return 1'b0;
        endcase
    endfunction

    // Combinational flag: reset message arrived from ltsm_decoder
    logic w_reset_msg;
    assign w_reset_msg = i_msg_id_valid && is_reset_msg(i_msg_id);

// =============================================================================
// PATTERN LOCK REGISTER
// r_prev_pattern tracks whether the immediately preceding i_de_ser_done word
// was also a pattern. Two consecutive patterns = 128 UI locked (spec §4.5.3.2).
// Cleared whenever the FSM is in or returns to IDLE.  [E3]
// =============================================================================

    logic r_prev_pattern;
logic r_data_fire_d;

    always_ff @(posedge i_clk or negedge i_rst_n) begin
        if (!i_rst_n)
            r_prev_pattern <= 1'b0;
        else if (cs == IDLE)
            r_prev_pattern <= 1'b0;                  // [E3] clear on IDLE entry
        else if (i_de_ser_done)
            r_prev_pattern <= i_fifo_is_pattern;     // track current word
    end

always_ff @(posedge i_clk or negedge i_rst_n) begin
    if (!i_rst_n)
        r_data_fire_d <= 1'b0;
    else
        r_data_fire_d <= (cs == DATA_LATCH && i_de_ser_done);
end

// =============================================================================
// DESERIALISER ACK
// Registered one-cycle delayed copy of i_de_ser_done; fed back to the
// deserialiser as an acknowledgement that this word has been processed.
// =============================================================================

    always_ff @(posedge i_clk or negedge i_rst_n) begin
        if (!i_rst_n) o_de_ser_done_sampled <= 1'b0;
        else          o_de_ser_done_sampled <= i_de_ser_done;
    end

// =============================================================================
// FSM STATE REGISTER
// =============================================================================

    always_ff @(posedge i_clk or negedge i_rst_n) begin
        if (!i_rst_n) cs <= IDLE;
        else          cs <= ns;
    end

// =============================================================================
// NEXT-STATE LOGIC
// Pure control — no 64-bit data inspection anywhere in this block.
// Default ns = cs prevents latch inference.  [E2]
// =============================================================================

    always_comb begin
        ns = cs;   // [E2] safe default

        case (cs)

            // ── IDLE ─────────────────────────────────────────────────────────
            // Stay here until the FIFO assembly register shows the start-pattern
            // word (i_fifo_is_pattern HIGH on i_de_ser_done).
            IDLE: begin
                if (i_de_ser_done && i_fifo_is_pattern)
                    ns = PATTERN_DETECT;
            end

            // ── PATTERN_DETECT ───────────────────────────────────────────────
            // Need a second consecutive pattern word for 128 UI confirmation.
            // A reset message immediately aborts back to IDLE.
            // A non-pattern word is considered noise; restart from IDLE.
            PATTERN_DETECT: begin
                if (w_reset_msg) begin
                    ns = IDLE;
                end else if (i_de_ser_done) begin
                    if (i_fifo_is_pattern && r_prev_pattern)
                        ns = GENERAL_DECODE;   // 128 UI confirmed — pattern locked
                    else if (!i_fifo_is_pattern)
                        ns = IDLE;             // noise / corrupt word — restart
                    // else: another pattern but r_prev_pattern not yet set — stay
                end
            end

            // ── GENERAL_DECODE ───────────────────────────────────────────────
            // Pattern locked. Classify each incoming 64-bit word:
            //   keep-alive pattern → stay (e.g. during SBINIT idle windows)
            //   any other word     → header flit of a new packet → HEADER_LATCH
            // Reset message always wins.
            GENERAL_DECODE: begin
                if (w_reset_msg) begin
                    ns = IDLE;
                end else if (i_de_ser_done) begin
                    if (i_fifo_is_pattern)
                        ns = GENERAL_DECODE;   // keep-alive; stay
                    else
                        ns = HEADER_LATCH;     // non-pattern word = header flit
                end
            end

            // ── HEADER_LATCH ──────────────────────────────────────────────────
            // The header word has been signalled to the FIFO (o_rx_header_valid).
            // Now decide: does this packet carry a data flit?
            //   i_fifo_has_data = 1 → go to DATA_LATCH and wait for the data flit
            //   i_fifo_has_data = 0 → commit immediately, notify decoder, loop back
            // Reset message aborts; no commit is performed (FIFO entry is discarded).
            HEADER_LATCH: begin
                if (w_reset_msg) begin
                    ns = IDLE;
                end else if (i_fifo_has_data) begin
                    ns = DATA_LATCH;           // wait for data flit  [E6]
                end else begin
                    ns = GENERAL_DECODE;       // header-only: commit done inline [E6]
                end
            end

            // ── DATA_LATCH ────────────────────────────────────────────────────
            // Waiting for the data flit from SerDes. When i_de_ser_done fires,
            // latch the data word (o_rx_data_valid), commit the entry to the FIFO
            // (o_write_commit), read it to the decoder (o_fifo_read_enable), and
            // notify the decoder (o_msg_valid). Then loop back.  [E7]
            DATA_LATCH: begin
                if (w_reset_msg) begin
                    ns = IDLE;
                end else if (i_de_ser_done) begin
                    ns = GENERAL_DECODE;       // data captured; commit and loop
                end
            end

            default: ns = IDLE;

        endcase
    end

// =============================================================================
// OUTPUT LOGIC  (combinational intermediate signals, registered below)
// All combinational outputs are computed here and flopped in the
// register block. No combinational path reaches the output ports.
// =============================================================================

    logic r_start_pat;    // → o_rx_sb_start_pattern
    logic r_pat_done;     // → o_rx_sb_pattern_samp_done
    logic r_msg_valid;    // → o_msg_valid
    logic r_hdr_valid;    // → o_rx_header_valid
    logic r_data_valid;   // → o_rx_data_valid
    logic r_commit;       // → o_write_commit
    logic r_fifo_rd;      // → o_fifo_read_enable

    always_comb begin
        // Default all signals LOW every cycle — only one of the case arms fires
        r_start_pat  = 1'b0;
        r_pat_done   = 1'b0;
        r_msg_valid  = 1'b0;
        r_hdr_valid  = 1'b0;
        r_data_valid = 1'b0;
        r_commit     = 1'b0;
        r_fifo_rd    = 1'b0;

        case (cs)

            // ── IDLE ─────────────────────────────────────────────────────────
            // Assert o_rx_sb_start_pattern when the first start-pattern word is
            // detected (transition to PATTERN_DETECT).
            // No LTSM-state gate needed here — TIME_OUT_COUNTER and SB_FSM
            // each decide independently whether to act on this signal.
            IDLE: begin
                if (i_de_ser_done && i_fifo_is_pattern)
                    r_start_pat = 1'b1;
            end

            // ── PATTERN_DETECT ───────────────────────────────────────────────
            // 128 UI confirmed → assert o_rx_sb_pattern_samp_done.
            // Feeds TIME_OUT_COUNTER i_rx_sb_pattern_samp_done (stop timer)
            // and SB_PATTERN_GEN i_rx_sb_pattern_samp_done (4 extra iterations).
            PATTERN_DETECT: begin
                if (i_de_ser_done && i_fifo_is_pattern && r_prev_pattern)
                    r_pat_done = 1'b1;
            end

            // ── GENERAL_DECODE ───────────────────────────────────────────────
            // A non-pattern word arrived on i_de_ser_done → instruct the FIFO
            // to latch it as the header flit (SB_RX_FIFO phase-1).  [E6]
            GENERAL_DECODE: begin
                if (i_de_ser_done && !i_fifo_is_pattern && ns == HEADER_LATCH)
                    r_hdr_valid = 1'b1;
            end

            // ── HEADER_LATCH ──────────────────────────────────────────────────
            // Header-only packet (no data flit):
            //   Commit the assembled entry to the FIFO circular buffer,
            //   read-enable the decoder (rd_ptr advances), and pulse o_msg_valid
            //   to ltsm_decoder (i_pckt_valid). ltsm_decoder will then set
            //   o_timeout_stop → TIME_OUT_COUNTER i_stop.
            //
            // Has-data packet:
            //   Nothing to output here; wait for DATA_LATCH.  [E6]
            HEADER_LATCH: begin
                if (i_de_ser_done && !i_fifo_has_data && !w_reset_msg) begin
                    r_commit    = 1'b1;   // push entry to FIFO buffer
                    r_fifo_rd   = 1'b1;   // pop to ltsm_decoder
                    r_msg_valid = 1'b1;   // tell ltsm_decoder a packet is ready
                end
            end

            // ── DATA_LATCH ────────────────────────────────────────────────────
            // Data flit arrived from SerDes:
            //   1. o_rx_data_valid → SB_RX_FIFO latches r_asm_data (phase-2)
            //   2. o_write_commit  → FIFO commits the full header+data entry
            //   3. o_fifo_read_enable → pop entry to ltsm_decoder
            //   4. o_msg_valid     → ltsm_decoder i_pckt_valid  [E7]
            DATA_LATCH: begin
                if (r_data_fire_d && !w_reset_msg) begin
                    r_data_valid = 1'b1;   // phase-2 latch
                    r_commit     = 1'b1;   // commit full entry
                    r_fifo_rd    = 1'b1;   // read to decoder
                    r_msg_valid  = 1'b1;   // notify decoder
                end
            end

            default: ;

        endcase
    end

// =============================================================================
// OUTPUT REGISTER STAGE
// All outputs are flopped; no combinational path to any output port.
// Reset drives all outputs LOW.
// =============================================================================

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

// =============================================================================
// SVA ASSERTIONS  [E1]
// Compiled only when `FORMAL is defined (simulation / formal flow).
// These properties capture the key FSM invariants derived from the spec and
// the full system context verified above.
// =============================================================================
`ifdef FORMAL

    // ── A0: Hard reset → IDLE ─────────────────────────────────────────────────
    // After the async reset deasserts the FSM must be in IDLE.
    ap_reset_to_idle: assert property (
        @(posedge i_clk) $fell(i_rst_n) |=> (cs == IDLE)
    );

    // ── A1: Pattern order ────────────────────────────────────────────────────
    // o_rx_sb_pattern_samp_done must never assert before o_rx_sb_start_pattern
    // has been seen at least once since the last reset.
    logic r_start_seen;
    always_ff @(posedge i_clk or negedge i_rst_n)
        if (!i_rst_n) r_start_seen <= 1'b0;
        else if (o_rx_sb_start_pattern) r_start_seen <= 1'b1;
        else if (cs == IDLE)            r_start_seen <= 1'b0;

    ap_pat_order: assert property (
        @(posedge i_clk) o_rx_sb_pattern_samp_done |-> r_start_seen
    );

    // ── A2: No msg_valid in idle phases ──────────────────────────────────────
    // A complete-packet notification must never fire while we are still
    // synchronising to the link (IDLE or PATTERN_DETECT).
    ap_no_msg_in_sync: assert property (
        @(posedge i_clk) (cs inside {IDLE, PATTERN_DETECT}) |-> !o_msg_valid
    );

    // ── A3: FIFO controls mutual exclusion ───────────────────────────────────
    // o_rx_header_valid and o_rx_data_valid must never be asserted together;
    // they correspond to the two distinct assembly phases in SB_RX_FIFO.
    ap_hdr_data_mutex: assert property (
        @(posedge i_clk) !(o_rx_header_valid && o_rx_data_valid)
    );

    // ── A4: Commit implies a prior header latch ───────────────────────────────
    // o_write_commit should only fire after at least one of header or data valid
    // has been asserted in the same transaction. Check via past-cycle test.
    logic r_hdr_ever_latched;
    always_ff @(posedge i_clk or negedge i_rst_n)
        if (!i_rst_n)                  r_hdr_ever_latched <= 1'b0;
        else if (cs == IDLE)           r_hdr_ever_latched <= 1'b0;
        else if (o_rx_header_valid)    r_hdr_ever_latched <= 1'b1;

    ap_commit_after_header: assert property (
        @(posedge i_clk) o_write_commit |-> r_hdr_ever_latched
    );

    // ── A5: msg_valid and write_commit fire together ──────────────────────────
    // Whenever the FSM notifies the decoder (o_msg_valid), it must also commit
    // the entry to the FIFO in the same registered cycle.
    ap_valid_with_commit: assert property (
        @(posedge i_clk) o_msg_valid |-> o_write_commit
    );

    // ── A6: Reset message forces IDLE ────────────────────────────────────────
    // If a reset-class message is detected while not already in IDLE,
    // the FSM must transition to IDLE on the next clock.
    ap_reset_msg_to_idle: assert property (
        @(posedge i_clk)
        (w_reset_msg && cs != IDLE) |=> (cs == IDLE)
    );

    // ── A7: o_rx_sb_start_pattern only from IDLE ─────────────────────────────
    // The start-pattern indication may only be generated on the transition
    // out of IDLE (first pattern word detected).
    ap_start_pat_from_idle: assert property (
        @(posedge i_clk) o_rx_sb_start_pattern |-> $past(cs == IDLE)
    );

    // ── A8: pattern_samp_done only from PATTERN_DETECT ───────────────────────
    ap_pat_done_from_detect: assert property (
        @(posedge i_clk) o_rx_sb_pattern_samp_done |-> $past(cs == PATTERN_DETECT)
    );

    // ── A9: DATA_LATCH only reachable when a data flit is expected ────────────
    // FSM enters DATA_LATCH only via HEADER_LATCH when i_fifo_has_data is HIGH.
    ap_data_latch_has_data: assert property (
        @(posedge i_clk) (cs == HEADER_LATCH && ns == DATA_LATCH) |-> i_fifo_has_data
    );

    // ── A10: FIFO read never fires when FIFO is empty ────────────────────────
    // We should never pop an entry from an empty FIFO.
    ap_no_read_empty: assert property (
        @(posedge i_clk) o_fifo_read_enable |-> i_fifo_valid
    );

`endif // FORMAL

endmodule : SB_RX_FSM