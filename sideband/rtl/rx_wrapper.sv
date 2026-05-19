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

    // ── Debug / status ────────────────────────────────────────────────────────
    output logic        o_de_ser_done_sampled,       // registered ack to SerDes

    // ── To TX wrapper (same die, direct cross-path — NOT through LTSM) ───────
    output logic        o_sb_ltsm_resp_rcvd          // RESP decoded → release TX WAIT_RSP
);

    // =========================================================================
    // Internal wires
    // =========================================================================

    // ── SB_RX_FSM → SB_RX_FIFO read-side ────────────────────────────────────
    logic        w_fifo_read_enable;
	logic        w_commit;

    // ── SB_RX_FIFO → SB_RX_FSM status ───────────────────────────────────────
    logic        w_fifo_is_pattern;
    logic        w_fifo_has_data;
    //logic        w_fifo_valid;
    logic        w_fifo_empty;
    logic        w_fifo_full;

    // ── SB_RX_FIFO read outputs → ltsm_decoder / sb_error_handler ───────────
    logic [63:0] w_fifo_header;
    logic [63:0] w_fifo_data;

    // ── SB_RX_FSM → ltsm_decoder / sb_error_handler ─────────────────────────
    logic        w_msg_valid;           // packet committed to FIFO, ready for processing

    // ── ltsm_decoder → SB_RX_FSM (reset detection) ──────────────────────────
    logic [10:0] w_dec_msg_id_raw;      // raw 11-bit cast of sb_msg_id enum
    logic        w_dec_valid;

    // ── ltsm_decoder outputs ─────────────────────────────────────────────────
    sb_msg_id    w_dec_msg_id;

    // =========================================================================
    // SB_RX_FIFO  –  receive packet buffer
    // =========================================================================
    SB_RX_FIFO #(
        .DEPTH (FIFO_DEPTH)
    ) u_rx_fifo (
        .i_clk             (i_clk),
        .i_rst_n           (i_rst_n),
    // Write port — raw word from SerDes; write_enable comes directly from the
    // remote partner (i_de_ser_done). FSM controls phase sequencing internally
    // via r_hdr_received inside the FIFO.
        .i_write_enable    (i_de_ser_done),   // remote partner pulse
        .i_rx_word         (i_rx_word),
        // Read port — to ltsm_decoder / sb_error_handler
        .i_read_enable     (w_fifo_read_enable),
		.i_write_commit    (w_commit),
        .o_header          (w_fifo_header),
        .o_data            (w_fifo_data),
        .o_has_data        (w_fifo_has_data),
        // Pattern flag — to SB_RX_FSM (no data inspection in FSM)
        .o_is_pattern      (w_fifo_is_pattern),
        // Status
        .o_empty           (w_fifo_empty),
        .o_full            (w_fifo_full)
    );


    // =========================================================================
    // SB_RX_FSM  –  control-only receive FSM
    // =========================================================================
    SB_RX_FSM u_rx_fsm (
        .i_clk                    (i_clk),
        .i_rst_n                  (i_rst_n),
        // Deserialiser pulse only
        .i_de_ser_done            (i_de_ser_done),
        // FIFO status flags
        .i_fifo_is_pattern        (w_fifo_is_pattern),
        .i_fifo_has_data          (w_fifo_has_data),
        //.i_fifo_valid             (w_fifo_valid),
        .i_fifo_full              (w_fifo_full),
        .i_fifo_empty             (w_fifo_empty),
        // FIFO control
        .o_fifo_read_enable       (w_fifo_read_enable),
		.o_write_commit           (w_commit),
        // ltsm_decoder feedback for reset detection
        .i_msg_id                 (w_dec_msg_id_raw),
        .i_msg_id_valid           (w_dec_valid),
        // Outputs to upper layers
        .o_rx_sb_start_pattern    (o_rx_sb_start_pattern),
        .o_rx_sb_pattern_samp_done(o_rx_sb_pattern_samp_done),
        .o_msg_valid              (w_msg_valid),
        .o_de_ser_done_sampled    (o_de_ser_done_sampled),
        .o_sb_ltsm_resp_rcvd      (o_sb_ltsm_resp_rcvd)
    );

    // Cast sb_msg_id enum to raw logic for FSM port (11-bit)
    assign w_dec_msg_id_raw = 11'(w_dec_msg_id);

    // =========================================================================
    // ltsm_decoder  –  message identification
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

    // =========================================================================
    // sb_error_handler  –  CP / DP parity verification
    // =========================================================================
    sb_error_handler u_error_handler (
        .clk        (i_clk),
        .rst_n      (i_rst_n),
        .i_pkt_valid(w_msg_valid),
        .i_header   (w_fifo_header),
        .i_data     (w_fifo_data),
        .i_has_data (w_fifo_has_data),
        .o_cp_error (o_cp_error),
        .o_dp_error (o_dp_error)
    );

endmodule : SB_RX_WRAPPER