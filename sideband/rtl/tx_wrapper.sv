// =============================================================================
// SB_TX_WRAPPER  –  UCIe Sideband Transmit Wrapper
//
// Spec reference: UCIe Specification Rev 3.0 Ver 1.0 (August 2025)
//   §4.1.5   Sideband transmission (64-bit serial packets, 32-UI dead-time)
//   §4.5.3.2 SBINIT start-pattern generation
//   §7.1.2   Sideband packet formats
//
// ── Modules instantiated ──────────────────────────────────────────────────────
//
//   ltsm_encoder        – builds 64-bit header + data with CP/DP in one cycle
//   SB_PACKET_ENCODER_MUX – selects pattern vs encoded packet; two-phase output
//   SB_TX_FIFO          – stores header+data+has_data entries (depth 4)
//   SB_FSM              – high-level TX sequencer (ENCODING→WAIT_RSP handshake)
//   SB_TX_FSM_Modelling – serialiser-level controller (header/data phase split)
//   SB_CLOCK_CONTROLLER – generates gated 800 MHz TXCKSB and timing pulses
//   SB_PATTERN_GEN      – SBINIT 64-UI alternating pattern generator
//
// ── TX datapath ───────────────────────────────────────────────────────────────
//
//   LTSM  →  ltsm_encoder
//                │  o_ltsm_header / o_ltsm_data / o_ltsmpckt_valid
//                ▼
//         SB_PACKET_ENCODER_MUX  ◄──  SB_PATTERN_GEN
//                │  o_final_packet / o_final_valid / o_final_has_data
//                ▼
//           SB_TX_FIFO
//                │  o_header / o_data / o_has_data / o_empty
//                ▼
//       SB_TX_FSM_Modelling  ──►  SB_CLOCK_CONTROLLER  ──►  TXCKSB
//
//   SB_FSM controls ltsm_encoder (o_encoder_enable → i_ltsm_valid)
//
// ── Wrapper boundary ports ────────────────────────────────────────────────────
//
//   Clocks / resets:
//     i_clk, i_rst_n
//
//   From LTSM (message requests):
//     i_msg_valid, i_msg_id, i_ltsm_data, i_ltsm_msginfo
//     i_start_pattern_req
//     i_sb_ltsm_resp_rcvd   – RESP received on RX side, routed here by LTSM
//     i_time_out            – TIME_OUT_COUNTER o_time_out relayed by LTSM
//
//   From RX wrapper (cross-path signals):
//     i_rx_sb_pattern_samp_done – partner pattern locked (from SB_RX_FSM)
//
//   Status to LTSM:
//     o_busy, o_rsp_timeout
//
//   Physical output:
//     TXCKSB
// =============================================================================

import pckg::*;

module SB_TX_WRAPPER #(
    parameter int unsigned FIFO_DEPTH          = 4,
    parameter int unsigned SB_CLK_CYCLES_PER_MS = 800_000
)(
    // ── Clocks / resets ───────────────────────────────────────────────────────
    input  logic        i_clk,               // 800 MHz SB / PLL clock
    input  logic        i_rst_n,             // active-low asynchronous reset

    // ── From LTSM — message request ───────────────────────────────────────────
    input  logic        i_msg_valid,         // new SB message ready to send
    input  sb_msg_id    i_msg_id,            // message identifier (SB_MSG_LUT key)
    input  logic [63:0] i_ltsm_data,         // data payload (used when has_data)
    input  logic [15:0] i_ltsm_msginfo,      // MsgInfo field [55:40]

    // ── From LTSM — control ───────────────────────────────────────────────────
    input  logic        i_start_pattern_req, // enter SBINIT pattern phase
    input  logic        i_sb_ltsm_resp_rcvd, // RESP received by RX path → exit WAIT_RSP
    input  logic        i_time_out,          // 8 ms timeout from TIME_OUT_COUNTER via LTSM

    // ── From RX wrapper ───────────────────────────────────────────────────────
    input  logic        i_rx_sb_pattern_samp_done, // partner 128 UI pattern locked

    // ── Status to LTSM ────────────────────────────────────────────────────────
    output logic        o_busy,              // TX path is actively processing
    output logic        o_rsp_timeout,       // latched: 8 ms elapsed without RESP → TRAINERROR

    // ── To TIME_OUT_COUNTER ───────────────────────────────────────────────────
    output logic        o_timeout_start,     // ltsm_encoder fired a packet → start timer
    output sb_msg_id    o_timeout_msg_id,    // which msg was sent (for exclusion check)
    output logic        o_pattern_time_out,  // SB_PATTERN_GEN 8 ms TX-side expiry

    // ── Physical sideband clock output ────────────────────────────────────────
    output logic        TXCKSB               // gated sideband clock to remote die
);

    // =========================================================================
    // Internal wires
    // =========================================================================

    // ── ltsm_encoder → SB_PACKET_ENCODER_MUX ────────────────────────────────
    logic        w_enc_pkt_valid;
    logic [63:0] w_enc_header;
    logic [63:0] w_enc_data;
    logic        w_enc_has_data;   // derived from SB_MSG_LUT inside encoder

    // ── SB_FSM → ltsm_encoder ────────────────────────────────────────────────
    logic        w_encoder_enable;

    // ── SB_PATTERN_GEN → SB_PACKET_ENCODER_MUX ──────────────────────────────
    logic [63:0] w_pattern;
    logic        w_pattern_valid;
    logic        w_start_pattern_done;

    // ── SB_PACKET_ENCODER_MUX → SB_TX_FIFO ──────────────────────────────────
    logic [63:0] w_mux_packet;
    logic        w_mux_valid;
    logic        w_mux_has_data;

    // ── SB_TX_FIFO → SB_TX_FSM_Modelling ────────────────────────────────────
    logic [63:0] w_fifo_header;
    logic [63:0] w_fifo_data;
    logic        w_fifo_has_data;
    logic        w_fifo_empty;
    logic        w_fifo_full;
    logic        w_fifo_overflow;
    logic        w_fifo_ser_done_sampled;

    // ── SB_TX_FSM_Modelling → SB_TX_FIFO ────────────────────────────────────
    logic        w_read_enable;
    logic        w_read_data_enable;

    // ── SB_TX_FSM_Modelling → SB_CLOCK_CONTROLLER ───────────────────────────
    logic        w_clk_en;

    // ── SB_CLOCK_CONTROLLER outputs ──────────────────────────────────────────
    logic        w_ser_done;
    logic        w_dead_time_done;

    // =========================================================================
    // SB_FSM  –  high-level TX sequencer
    // =========================================================================
    SB_FSM u_sb_fsm (
        .i_clk                 (i_clk),
        .i_rst_n               (i_rst_n),
        .i_start_pattern_req   (i_start_pattern_req),
        .i_msg_valid           (i_msg_valid),
        .i_ltsm_pckt_valid     (w_enc_pkt_valid),
        .i_sb_ltsm_resp_rcvd   (i_sb_ltsm_resp_rcvd),
        .i_time_out            (i_time_out),
        .i_start_pattern_done  (w_start_pattern_done),
        .o_encoder_enable      (w_encoder_enable),
        .o_busy                (o_busy),
        .o_rsp_timeout         (o_rsp_timeout)
    );

    // =========================================================================
    // ltsm_encoder  –  builds header + data with CP/DP in one clock cycle
    // =========================================================================
    ltsm_encoder u_ltsm_encoder (
        .rst               (~i_rst_n),           // encoder uses active-high reset
        .clk               (i_clk),
        .i_ltsm_valid      (w_encoder_enable),
        .i_msg_id          (i_msg_id),
        .ltsm_data         (i_ltsm_data),
        .ltsm_msginfo      (i_ltsm_msginfo),
        .o_ltsmpckt_valid  (w_enc_pkt_valid),
        .o_ltsm_header     (w_enc_header),
        .o_ltsm_data       (w_enc_data),
        .o_timeout_start   (o_timeout_start),
        .o_timeout_msg_id  (o_timeout_msg_id)
    );

    // has_data is part of SB_MSG_LUT; mirror it combinationally from the
    // encoded header opcode so the MUX and TX FIFO know when a data phase follows.
    // ltsm_encoder already sets o_ltsm_data=0 for no-data messages, but the
    // FIFO needs the explicit flag.  We extract it from the opcode field.
    localparam logic [4:0] SB_OPCODE_DATA = 5'b11011;
    assign w_enc_has_data = (w_enc_header[4:0] == SB_OPCODE_DATA);

    // =========================================================================
    // SB_PATTERN_GEN  –  SBINIT 64-UI alternating clock pattern
    // =========================================================================
    SB_PATTERN_GEN #(
        .SB_CLK_CYCLES_PER_MS (SB_CLK_CYCLES_PER_MS)
    ) u_pattern_gen (
        .i_clk                    (i_clk),
        .i_rst_n                  (i_rst_n),
        .i_start_pattern_req      (i_start_pattern_req),
        .i_rx_sb_pattern_samp_done(i_rx_sb_pattern_samp_done),
        .i_ser_done               (w_ser_done),
        .o_start_pattern_done     (w_start_pattern_done),
        .o_pattern_time_out       (o_pattern_time_out),
        .o_pattern                (w_pattern),
        .o_pattern_valid          (w_pattern_valid)
    );

    // =========================================================================
    // SB_PACKET_ENCODER_MUX  –  pattern vs encoded-packet arbitration
    //
    // Priority: pattern (PSTP) > encoded packet > idle (drive 0).
    // The MUX releases the encoded packet in two phases: the TX FIFO stores
    // both header and data as a single entry keyed by has_data.
    // =========================================================================
    SB_PACKET_ENCODER_MUX u_pkt_mux (
        .i_pattern          (w_pattern),
        .i_encoded_packet   (w_enc_pkt_valid ? w_enc_header : w_enc_data),
        .i_pattern_valid    (w_pattern_valid),
        .i_encoded_pkt_valid(w_enc_pkt_valid),
        .o_final_packet     (w_mux_packet)
    );

    // Additional control signals produced by the wrapper for the TX FIFO
    assign w_mux_valid    = w_pattern_valid | w_enc_pkt_valid;
    assign w_mux_has_data = w_enc_pkt_valid ? w_enc_has_data : 1'b0;

    // =========================================================================
    // SB_TX_FIFO  –  packet queue (header + data + has_data per entry)
    // =========================================================================
    SB_TX_FIFO #(
        .DEPTH (FIFO_DEPTH)
    ) u_tx_fifo (
        .i_clk              (i_clk),
        .i_rst_n            (i_rst_n),
        // Write port — from MUX
        .i_write_enable     (w_mux_valid),
        .i_header           (w_mux_packet),
        .i_data             (w_enc_data),
        .i_has_data         (w_mux_has_data),
        // Read port — to SB_TX_FSM_Modelling
        .i_read_enable      (w_read_enable),
        .o_header           (w_fifo_header),
        .o_data             (w_fifo_data),
        .o_has_data         (w_fifo_has_data),
        // Status
        .o_empty            (w_fifo_empty),
        .o_full             (w_fifo_full),
        .o_overflow         (w_fifo_overflow),
        // Serialiser ack
        .o_ser_done_sampled (w_fifo_ser_done_sampled)
    );

    // =========================================================================
    // SB_TX_FSM_Modelling  –  serialiser-level controller
    // Controls the clock controller one 64-bit flit at a time;
    // splits has_data messages into header phase then data phase.
    // =========================================================================
    SB_TX_FSM_Modelling u_tx_fsm_mod (
        .i_clk                 (i_clk),
        .i_rst_n               (i_rst_n),
        .i_ser_done            (w_ser_done),
        .i_empty               (w_fifo_empty),
        .i_dead_time_done      (w_dead_time_done),
        .i_read_enable_sampled (w_fifo_ser_done_sampled),
        .i_has_data            (w_fifo_has_data),
        .o_read_enable         (w_read_enable),
        .o_read_data_enable    (w_read_data_enable),
        .o_clk_en              (w_clk_en)
    );

    // =========================================================================
    // SB_CLOCK_CONTROLLER  –  gated 800 MHz clock and per-packet timing
    // =========================================================================
    SB_CLOCK_CONTROLLER u_clk_ctrl (
        .i_pll_clk       (i_clk),
        .i_rst_n         (i_rst_n),
        .i_enable        (w_clk_en),
        .o_dead_time_done(w_dead_time_done),
        .o_ser_done      (w_ser_done),
        .TXCKSB          (TXCKSB)
    );

endmodule : SB_TX_WRAPPER