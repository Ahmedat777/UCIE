import pckg::*;

module SB_TX_WRAPPER #(
    parameter int unsigned FIFO_DEPTH           = 4,
    parameter int unsigned SB_CLK_CYCLES_PER_MS = 800_000
)(
    // ── Clocks / resets ───────────────────────────────────────────────────────
    input  logic        i_clk,
    input  logic        i_rst_n,

    // ── From LTSM — message request ───────────────────────────────────────────
    input  logic        i_msg_valid,
    input  sb_msg_id    i_msg_id,
    input  logic [63:0] i_ltsm_data,
    input  logic [15:0] i_ltsm_msginfo,

    // ── From LTSM — control ───────────────────────────────────────────────────
    input  logic        i_start_pattern_req,      // SBINIT: begin start-pattern TX
    input  logic        i_sb_ltsm_resp_rcvd,
    input  logic        i_time_out,

    // ── From RX wrapper ───────────────────────────────────────────────────────
    input  logic        i_rx_sb_pattern_samp_done,

    // ── Status to LTSM ────────────────────────────────────────────────────────
    output logic        o_busy,
    output logic        o_rsp_timeout,

    // ── To TIME_OUT_COUNTER ───────────────────────────────────────────────────
    output logic        o_timeout_start,
    output sb_msg_id    o_timeout_msg_id,
    output logic        o_pattern_time_out,

    // ── Physical sideband outputs ─────────────────────────────────────────────
    output logic        TXCKSB,          // gated sideband clock to remote die
    output logic [63:0] o_tx_packet      // final 64-bit packet word to remote die
);

    // =========================================================================
    // Internal wires
    // =========================================================================

    // ── ltsm_encoder outputs ─────────────────────────────────────────────────
    logic        w_enc_pkt_valid;
    logic [63:0] w_enc_header;
    logic [63:0] w_enc_data;

    // ── SB_FSM → ltsm_encoder ────────────────────────────────────────────────
    logic        w_encoder_enable;

    // ── SB_PATTERN_GEN outputs ────────────────────────────────────────────────
    logic [63:0] w_pattern;
    logic        w_pattern_valid;
    logic        w_start_pattern_done;

    // ── has_data flag from encoder LUT (combinational from header opcode) ─────
    // SB_MSG_LUT opcode for messages with data = SB_OPCODE_DATA (5'b11011)
    localparam logic [4:0] SB_OPCODE_DATA = 5'b11011;
    logic w_enc_has_data;
    assign w_enc_has_data = (w_enc_header[4:0] == SB_OPCODE_DATA);

    // ── SB_PACKET_ENCODER_MUX outputs ────────────────────────────────────────
    logic [63:0] w_mux_packet;
    logic        w_mux_valid;
    logic        w_mux_has_data;

    // ── SB_TX_FIFO status ─────────────────────────────────────────────────────
    logic        w_fifo_empty;
    logic        w_fifo_full;
    logic        w_fifo_ser_done_sampled;

    // ── SB_TX_FSM_Modelling outputs ───────────────────────────────────────────
    logic        w_read_enable;
    logic        w_clk_en;

    // ── SB_CLOCK_CONTROLLER outputs ───────────────────────────────────────────
    logic        w_ser_done;
    logic        w_dead_time_done;

    // =========================================================================
    // SB_FSM  –  high-level TX sequencer
    // =========================================================================
    SB_FSM u_sb_fsm (
        .i_clk                (i_clk),
        .i_rst_n              (i_rst_n),
        .i_start_pattern_req  (i_start_pattern_req),
        .i_msg_valid          (i_msg_valid),
        .i_ltsm_pckt_valid    (w_enc_pkt_valid),
        .i_sb_ltsm_resp_rcvd  (i_sb_ltsm_resp_rcvd),
        .i_time_out           (i_time_out),
        .i_start_pattern_done (w_start_pattern_done),
        .o_encoder_enable     (w_encoder_enable),
        .o_busy               (o_busy),
        .o_rsp_timeout        (o_rsp_timeout)
    );

    // =========================================================================
    // ltsm_encoder  –  builds header + data with CP/DP in one clock cycle
    // =========================================================================
    ltsm_encoder u_ltsm_encoder (
        .rst              (~i_rst_n),
        .clk              (i_clk),
        .i_ltsm_valid     (w_encoder_enable),
        .i_msg_id         (i_msg_id),
        .ltsm_data        (i_ltsm_data),
        .ltsm_msginfo     (i_ltsm_msginfo),
        .o_ltsmpckt_valid (w_enc_pkt_valid),
        .o_ltsm_header    (w_enc_header),
        .o_ltsm_data      (w_enc_data),
        .o_timeout_start  (o_timeout_start),
        .o_timeout_msg_id (o_timeout_msg_id)
    );

    // =========================================================================
    // SB_PATTERN_GEN  –  SBINIT 64-UI alternating clock pattern
    // =========================================================================
    SB_PATTERN_GEN #(
        .SB_CLK_CYCLES_PER_MS (SB_CLK_CYCLES_PER_MS)
    ) u_pattern_gen (
        .i_clk                     (i_clk),
        .i_rst_n                   (i_rst_n),
        .i_start_pattern_req       (i_start_pattern_req),   // from SBINIT / LTSM
        .i_rx_sb_pattern_samp_done (i_rx_sb_pattern_samp_done),
        .i_ser_done                (w_ser_done),
        .o_start_pattern_done      (w_start_pattern_done),
        .o_pattern_time_out        (o_pattern_time_out),
        .o_pattern                 (w_pattern),
        .o_pattern_valid           (w_pattern_valid)
    );

    // =========================================================================
    // SB_PACKET_ENCODER_MUX  –  pattern vs encoded-packet arbitration
    // =========================================================================
    SB_PACKET_ENCODER_MUX u_pkt_mux (
        .i_pattern           (w_pattern),
        .i_pattern_valid     (w_pattern_valid),
        .i_enc_header        (w_enc_header),
        .i_enc_has_data      (w_enc_has_data),
        .i_encoded_pkt_valid (w_enc_pkt_valid),
        .o_final_packet      (w_mux_packet),
        .o_final_valid       (w_mux_valid),
        .o_final_has_data    (w_mux_has_data)
    );

    // =========================================================================
    // SB_TX_FIFO  –  packet queue; one 64-bit word per entry
    // =========================================================================

    logic r_data_pending;

    always_ff @(posedge i_clk or negedge i_rst_n) begin
        if (!i_rst_n)
            r_data_pending <= 1'b0;
        else if (r_data_pending && !w_fifo_full)
            r_data_pending <= 1'b0;   // data flit accepted, clear
        else if (w_mux_valid && w_mux_has_data && !w_fifo_full)
            r_data_pending <= 1'b1;   // header accepted; data flit pending
    end

    // FIFO write-enable and data word mux
    logic        w_fifo_wr_en;
    logic [63:0] w_fifo_wr_data;

    always_comb begin
        if (r_data_pending && !w_fifo_full) begin
            // Cycle after header: push the data flit
            w_fifo_wr_en   = 1'b1;
            w_fifo_wr_data = w_enc_data;
        end else if (w_mux_valid && !w_fifo_full) begin
            // Normal push: header flit or pattern word from MUX
            w_fifo_wr_en   = 1'b1;
            w_fifo_wr_data = w_mux_packet;
        end else begin
            w_fifo_wr_en   = 1'b0;
            w_fifo_wr_data = 64'd0;
        end
    end

    // FIFO output wire (single final packet word) → drives wrapper output port
    logic [63:0] w_fifo_packet;
    assign o_tx_packet = w_fifo_packet;

    SB_TX_FIFO #(
        .DEPTH (FIFO_DEPTH)
    ) u_tx_fifo (
        .i_clk             (i_clk),
        .i_rst_n           (i_rst_n),
        // Write port
        .i_write_enable    (w_fifo_wr_en),
        .i_packet          (w_fifo_wr_data),
        // Read port → serialiser
        .i_read_enable     (w_read_enable),
        .o_packet          (w_fifo_packet),
        // Status
        .o_empty           (w_fifo_empty),
        .o_full            (w_fifo_full),
        // Ack
        .o_ser_done_sampled(w_fifo_ser_done_sampled)
    );

    // =========================================================================
    // SB_TX_FSM_Modelling  –  serialiser-level controller
    // =========================================================================
    SB_TX_FSM_Modelling u_tx_fsm_mod (
        .i_clk                (i_clk),
        .i_rst_n              (i_rst_n),
        .i_ser_done           (w_ser_done),
        .i_empty              (w_fifo_empty),
        .i_dead_time_done     (w_dead_time_done),
        .i_read_enable_sampled(w_fifo_ser_done_sampled),
        .o_read_enable        (w_read_enable),
        .o_clk_en             (w_clk_en)
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