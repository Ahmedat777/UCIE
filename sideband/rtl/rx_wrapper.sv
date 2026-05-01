// =============================================================================
// SB_RX_WRAPPER  –  UCIe Sideband Receive Wrapper
//
// Spec reference: UCIe Specification Rev 3.0 Ver 1.0 (August 2025)
//   §4.1.5   Sideband reception (64-bit serial packets)
//   §4.5.3.2 SBINIT start-pattern detection (128 UI)
//   §7.1.2   Sideband packet formats
//   §7.1.3.2 Parity error → fatal UIE
//
// ── Modules instantiated ──────────────────────────────────────────────────────
//
//   SB_RX_FIFO      – buffers received header+data+has_data entries (depth 4)
//   SB_RX_FSM       – control-only FSM: pattern detection, flit sequencing,
//                     FIFO write/read control, reset detection
//   ltsm_decoder    – decodes msgcode/msgsubcode → sb_msg_id, extracts MsgInfo
//   sb_error_handler– CP/DP parity verification (sole owner of parity checks)
//
// ── RX datapath ───────────────────────────────────────────────────────────────
//
//   SerDes (i_de_ser_done / i_rx_word)
//         │
//         ├──► SB_RX_FIFO (write port: i_rx_header / i_rx_data)
//         │         │  driven by SB_RX_FSM write-side controls
//         │         │
//         │    SB_RX_FSM ◄── i_fifo_is_pattern / i_fifo_has_data / i_fifo_valid
//         │         │    ◄── ltsm_decoder o_msg_id / o_valid  (reset detection)
//         │         │
//         │    SB_RX_FIFO (read port: o_header / o_data / o_has_data)
//         │         │
//         │         ├──► ltsm_decoder  →  LTSM  (o_msg_id, o_msginfo, o_data)
//         │         └──► sb_error_handler  →  LTSM  (o_cp_error, o_dp_error)
//         │
//         └── o_rx_word routed to both FIFO write ports (header and data phases)
//
// ── Wrapper boundary ports ────────────────────────────────────────────────────
//
//   Clocks / resets:
//     i_clk, i_rst_n
//
//   From SerDes (deserialiser):
//     i_de_ser_done   – pulse when a new 64-bit word is ready
//     i_rx_word [63:0]– the received 64-bit word (goes to FIFO write port)
//
//   To LTSM — decoded message:
//     o_msg_valid     – complete packet ready (from SB_RX_FSM)
//     o_msg_id        – decoded message enum  (from ltsm_decoder)
//     o_msginfo [15:0]– MsgInfo field          (from ltsm_decoder)
//     o_rx_data [63:0]– data flit              (from ltsm_decoder)
//     o_decoder_valid – ltsm_decoder o_valid
//
//   To LTSM — error flags:
//     o_cp_error      – Control Parity mismatch → UIE
//     o_dp_error      – Data Parity mismatch    → UIE
//
//   To TX wrapper / LTSM — SBINIT:
//     o_rx_sb_start_pattern    – request TX to start sending pattern
//     o_rx_sb_pattern_samp_done– 128 UI pattern locked
//
//   To TIME_OUT_COUNTER:
//     o_timeout_stop  – any valid decoded message → stop 8-ms timer
//     o_timeout_rx_msg_id   – decoded msg_id for stall-condition check
//     o_timeout_rx_msginfo  – MsgInfo for stall-condition check
//
//   Debug:
//     o_fifo_overflow – RX FIFO wrote while full
//     o_de_ser_done_sampled – registered ack back to SerDes
// =============================================================================

import pckg::*;

module SB_RX_WRAPPER #(
    parameter int unsigned FIFO_DEPTH = 4
)(
    // ── Clocks / resets ───────────────────────────────────────────────────────
    input  logic        i_clk,                // 800 MHz SB clock
    input  logic        i_rst_n,              // active-low asynchronous reset

    // ── From SerDes (deserialiser) ────────────────────────────────────────────
    input  logic        i_de_ser_done,        // pulse: new 64-bit word is ready
    input  logic [63:0] i_rx_word,            // received 64-bit word from SerDes

    // ── To LTSM — decoded message ─────────────────────────────────────────────
    output logic        o_msg_valid,          // complete packet committed to FIFO and ready
    output sb_msg_id    o_msg_id,             // decoded message enum value
    output logic [15:0] o_msginfo,            // MsgInfo field [55:40]
    output logic [63:0] o_rx_data,            // data flit (valid when decoder has_data)
    output logic        o_decoder_valid,      // ltsm_decoder o_valid (msg found in LUT)

    // ── To LTSM — parity errors ───────────────────────────────────────────────
    output logic        o_cp_error,           // CP mismatch → fatal UIE
    output logic        o_dp_error,           // DP mismatch → fatal UIE

    // ── To TX wrapper / LTSM — SBINIT ────────────────────────────────────────
    output logic        o_rx_sb_start_pattern,      // request TX to send start-pattern
    output logic        o_rx_sb_pattern_samp_done,  // 128 UI pattern locked

    // ── To TIME_OUT_COUNTER ───────────────────────────────────────────────────
    output logic        o_timeout_stop,             // any valid decoded msg → stop timer
    output sb_msg_id    o_timeout_rx_msg_id,        // RX msg_id (stall condition 2)
    output logic [15:0] o_timeout_rx_msginfo,       // RX MsgInfo (stall conditions 1 & 2)

    // ── Debug / status ────────────────────────────────────────────────────────
    output logic        o_fifo_overflow,            // RX FIFO overflow (should never fire)
    output logic        o_de_ser_done_sampled        // registered ack to SerDes
);

    // =========================================================================
    // Internal wires
    // =========================================================================

    // ── SB_RX_FSM → SB_RX_FIFO write-side ───────────────────────────────────
    logic        w_rx_header_valid;
    logic        w_rx_data_valid;
    logic        w_write_commit;
    logic        w_has_data_to_fifo;    // driven by FSM from i_fifo_has_data

    // ── SB_RX_FSM → SB_RX_FIFO read-side ────────────────────────────────────
    logic        w_fifo_read_enable;

    // ── SB_RX_FIFO → SB_RX_FSM status ───────────────────────────────────────
    logic        w_fifo_is_pattern;
    logic        w_fifo_has_data;
    logic        w_fifo_valid;
    logic        w_fifo_empty;
    logic        w_fifo_full;

    // ── SB_RX_FIFO read outputs → ltsm_decoder / sb_error_handler ───────────
    logic [63:0] w_fifo_header;
    logic [63:0] w_fifo_data;
    logic        w_fifo_has_data_out;

    // ── SB_RX_FSM → ltsm_decoder / sb_error_handler ─────────────────────────
    logic        w_msg_valid;           // packet committed to FIFO, ready for processing

    // ── ltsm_decoder → SB_RX_FSM (reset detection) ──────────────────────────
    logic [10:0] w_dec_msg_id_raw;      // raw 11-bit cast of sb_msg_id enum
    logic        w_dec_valid;

    // ── ltsm_decoder outputs ─────────────────────────────────────────────────
    sb_msg_id    w_dec_msg_id;

    // =========================================================================
    // SB_RX_FIFO  –  receive packet buffer
    //
    // Write port: raw SerDes word arrives on both header and data phases.
    // The FSM controls which phase is active via o_rx_header_valid /
    // o_rx_data_valid, so the same i_rx_word bus feeds both inputs.
    // =========================================================================
    SB_RX_FIFO #(
        .DEPTH (FIFO_DEPTH)
    ) u_rx_fifo (
        .i_clk             (i_clk),
        .i_rst_n           (i_rst_n),
        // Write port — raw word from SerDes; FSM controls which phase is active
        .i_rx_header_valid (w_rx_header_valid),
        .i_rx_header       (i_rx_word),
        .i_rx_data_valid   (w_rx_data_valid),
        .i_rx_data         (i_rx_word),
        .i_has_data        (w_fifo_has_data),    // fed back from FIFO's own o_has_data
        .i_write_commit    (w_write_commit),
        // Read port — to ltsm_decoder / sb_error_handler
        .i_read_enable     (w_fifo_read_enable),
        .o_header          (w_fifo_header),
        .o_data            (w_fifo_data),
        .o_has_data        (w_fifo_has_data_out),
        .o_valid           (w_fifo_valid),
        // Pattern flag — to SB_RX_FSM (no data inspection in FSM)
        .o_is_pattern      (w_fifo_is_pattern),
        // Status
        .o_empty           (w_fifo_empty),
        .o_full            (w_fifo_full),
        .o_overflow        (o_fifo_overflow)
    );

    assign w_fifo_has_data = w_fifo_has_data_out;

    // =========================================================================
    // SB_RX_FSM  –  control-only receive FSM
    //
    // Never sees any 64-bit word. Drives FIFO write-side, detects pattern lock
    // via i_fifo_is_pattern, sequences has_data flit, and resets to IDLE on
    // reset-class msg_id from ltsm_decoder.
    // =========================================================================
    SB_RX_FSM u_rx_fsm (
        .i_clk                    (i_clk),
        .i_rst_n                  (i_rst_n),
        // Deserialiser pulse only
        .i_de_ser_done            (i_de_ser_done),
        // FIFO status flags
        .i_fifo_is_pattern        (w_fifo_is_pattern),
        .i_fifo_has_data          (w_fifo_has_data),
        .i_fifo_valid             (w_fifo_valid),
        // FIFO write-side controls
        .o_rx_header_valid        (w_rx_header_valid),
        .o_rx_data_valid          (w_rx_data_valid),
        .o_write_commit           (w_write_commit),
        // FIFO read-side control
        .o_fifo_read_enable       (w_fifo_read_enable),
        // ltsm_decoder feedback for reset detection
        .i_msg_id                 (w_dec_msg_id_raw),
        .i_msg_id_valid           (w_dec_valid),
        // Outputs to upper layers
        .o_rx_sb_start_pattern    (o_rx_sb_start_pattern),
        .o_rx_sb_pattern_samp_done(o_rx_sb_pattern_samp_done),
        .o_msg_valid              (w_msg_valid),
        .o_de_ser_done_sampled    (o_de_ser_done_sampled)
    );

    // Cast sb_msg_id enum to raw logic for FSM port (11-bit)
    assign w_dec_msg_id_raw = 11'(w_dec_msg_id);

    // =========================================================================
    // ltsm_decoder  –  message identification
    //
    // Reads header and data from the RX FIFO front entry.
    // Outputs msg_id, msginfo, data to LTSM and o_timeout_stop to the counter.
    // o_msg_id also feeds back to SB_RX_FSM for reset-class detection.
    // =========================================================================
    ltsm_decoder u_ltsm_decoder (
        .clk           (i_clk),
        .rst           (~i_rst_n),        // decoder uses active-high reset
        .i_pckt_valid  (w_msg_valid),
        .i_header      (w_fifo_header),
        .i_data        (w_fifo_data),
        .o_valid       (w_dec_valid),
        .o_msg_id      (w_dec_msg_id),
        .o_msginfo     (o_msginfo),
        .o_data        (o_rx_data),
        .o_timeout_stop(o_timeout_stop)
    );

    // Route decoder outputs to wrapper ports
    assign o_decoder_valid       = w_dec_valid;
    assign o_msg_id              = w_dec_msg_id;
    assign o_timeout_rx_msg_id   = w_dec_msg_id;
    assign o_timeout_rx_msginfo  = o_msginfo;
    assign o_msg_valid           = w_msg_valid;

    // =========================================================================
    // sb_error_handler  –  CP / DP parity verification
    //
    // Reads header and data from the RX FIFO front entry (same source as
    // ltsm_decoder). Sole owner of parity checking in the receive path.
    // =========================================================================
    sb_error_handler u_error_handler (
        .clk        (i_clk),
        .rst_n      (i_rst_n),
        .i_pkt_valid(w_msg_valid),
        .i_header   (w_fifo_header),
        .i_data     (w_fifo_data),
        .i_has_data (w_fifo_has_data_out),
        .o_cp_error (o_cp_error),
        .o_dp_error (o_dp_error)
    );

endmodule : SB_RX_WRAPPER