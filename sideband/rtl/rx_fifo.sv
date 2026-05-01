// =============================================================================
// SB_RX_FIFO  –  UCIe Sideband Receive FIFO
//
// Spec reference: UCIe Specification Rev 3.0 Ver 1.0 (August 2025)
//   §4.1.5   Each SB serial packet is 64 bits.  After SerDes reception the
//            deserialiser presents one 64-bit word (header or data flit).
//            The FIFO buffers both flits as a paired entry for the decoder.
//   §7.1.3.1 "The Receiver must guarantee unconditional sinking for any
//              Register Access Completion packets."
//            → The FIFO must never silently drop a packet; overflow is fatal.
//
// Architecture position:
//   SerDes  →  SB_RX_FIFO  →  ltsm_decoder  →  SB_RX_FSM
//                                            →  sb_error_handler
//
// Each FIFO entry stores:
//   • 64-bit header   (always present)
//   • 64-bit data     (valid when has_data == 1)
//   • 1-bit  has_data (mirrors SB_PACKET_ENCODER_MUX / ltsm_encoder flag)
//
// The write side receives flits in two phases from the RX deserialiser:
//   Phase 1 – header flit  (i_rx_header_valid asserted)
//   Phase 2 – data flit    (i_rx_data_valid asserted, only when has_data)
// Both arrive before i_write_commit pulses to commit the complete entry.
//
// The read side presents the full entry to ltsm_decoder:
//   o_header    – 64-bit header flit
//   o_data      – 64-bit data flit (meaningful when o_has_data)
//   o_has_data  – indicates whether a data flit is present
//   o_valid     – entry is ready for the decoder
//
// Implementation: circular buffer, depth = DEPTH (default 4, power-of-2).
// 3-bit pointers (MSB = wrap bit, [1:0] = FIFO index) give unambiguous
// full/empty detection — same scheme as SB_TX_FIFO.
//
// o_overflow is a sticky-pulse debug flag cleared on reset.
// =============================================================================

module SB_RX_FIFO #(
    parameter int unsigned DEPTH = 4    // must be power of 2, max 4 for 2-bit index
)(
    input  logic        i_clk,
    input  logic        i_rst_n,

    // ── Write port – from RX deserialiser (via SerDes) ───────────────────────
    // Flits arrive in two phases; the entry is committed only when both are ready.
    input  logic        i_rx_header_valid,  // phase 1: header flit available
    input  logic [63:0] i_rx_header,        // 64-bit received header flit
    input  logic        i_rx_data_valid,    // phase 2: data flit available
    input  logic [63:0] i_rx_data,          // 64-bit received data flit
    input  logic        i_has_data,         // 1 = this packet carries a data flit
    input  logic        i_write_commit,     // pulse: commit assembled entry to FIFO

    // ── Read port – to ltsm_decoder ──────────────────────────────────────────
    input  logic        i_read_enable,      // pop one entry (consume front)
    output logic [63:0] o_header,           // header of the front entry
    output logic [63:0] o_data,             // data flit of the front entry
    output logic        o_has_data,         // 1 = front entry has a data flit
    output logic        o_valid,            // entry ready for decoder (!o_empty)

    // ── Pattern flag (for SB_RX_FSM — no data inspection in FSM) ─────────────
    // Combinational: HIGH when the assembly register holds the 64-UI start-pattern
    // word 64'h5555_5555_5555_5555 (spec §4.5.3.2). SB_RX_FSM uses this flag
    // instead of reading the raw 64-bit word itself.
    output logic        o_is_pattern,       // asm-reg word == SB_START_PATTERN

    // ── Status ────────────────────────────────────────────────────────────────
    output logic        o_empty,            // FIFO is empty
    output logic        o_full,             // FIFO is full
    output logic        o_overflow          // write attempted while full (debug)
);

    // ─────────────────────────────────────────────────────────────────────────
    // Storage arrays (one entry = header + data + has_data flag)
    // ─────────────────────────────────────────────────────────────────────────
    logic [63:0] mem_header   [0:DEPTH-1];
    logic [63:0] mem_data     [0:DEPTH-1];
    logic        mem_has_data [0:DEPTH-1];

    // ─────────────────────────────────────────────────────────────────────────
    // 3-bit pointers: MSB = wrap bit, [1:0] = FIFO index (matches SB_TX_FIFO)
    // ─────────────────────────────────────────────────────────────────────────
    logic [2:0] wr_ptr;
    logic [2:0] rd_ptr;

    // ─────────────────────────────────────────────────────────────────────────
    // Assembly registers – hold flits between phase-1 and phase-2 arrival
    // ─────────────────────────────────────────────────────────────────────────
    logic [63:0] r_asm_header;
    logic [63:0] r_asm_data;
    logic        r_asm_has_data;
    logic        r_hdr_received;    // header phase complete, waiting for data

    // ─────────────────────────────────────────────────────────────────────────
    // Full / empty (combinational) – same arithmetic as SB_TX_FIFO
    // ─────────────────────────────────────────────────────────────────────────
    wire w_empty = (wr_ptr == rd_ptr);
    wire w_full  = (wr_ptr[1:0] == rd_ptr[1:0]) && (wr_ptr[2] != rd_ptr[2]);

    always_ff @(posedge i_clk or negedge i_rst_n) begin
        if (!i_rst_n) o_empty <= 1'b1;
        else          o_empty <= w_empty;
    end

    localparam logic [63:0] SB_START_PATTERN = {32{2'b01}};  // 64'h5555_5555_5555_5555

    assign o_full       = w_full;
    assign o_valid      = !o_empty;
    assign o_is_pattern = (r_asm_header == SB_START_PATTERN);  // combinational; valid when i_de_ser_done

    // ─────────────────────────────────────────────────────────────────────────
    // Write-side assembly:
    //   Phase 1 – latch header flit when i_rx_header_valid
    //   Phase 2 – latch data  flit when i_rx_data_valid (has_data packets only)
    //   Commit  – push assembled entry into circular buffer when i_write_commit
    // ─────────────────────────────────────────────────────────────────────────
    always_ff @(posedge i_clk or negedge i_rst_n) begin
        if (!i_rst_n) begin
            r_asm_header   <= 64'b0;
            r_asm_data     <= 64'b0;
            r_asm_has_data <= 1'b0;
            r_hdr_received <= 1'b0;
            wr_ptr         <= 3'b000;
            o_overflow     <= 1'b0;
        end else begin
            o_overflow <= 1'b0;

            // Phase 1: latch incoming header flit
            if (i_rx_header_valid) begin
                r_asm_header   <= i_rx_header;
                r_asm_has_data <= i_has_data;
                r_hdr_received <= 1'b1;
            end

            // Phase 2: latch incoming data flit (only relevant for has_data packets)
            if (i_rx_data_valid && r_hdr_received) begin
                r_asm_data <= i_rx_data;
            end

            // Commit: push assembled entry when signalled
            if (i_write_commit) begin
                if (!w_full) begin
                    mem_header  [wr_ptr[1:0]] <= r_asm_header;
                    mem_data    [wr_ptr[1:0]] <= r_asm_has_data ? r_asm_data : 64'b0;
                    mem_has_data[wr_ptr[1:0]] <= r_asm_has_data;
                    wr_ptr         <= wr_ptr + 3'b001;
                    r_hdr_received <= 1'b0;
                end else begin
                    // Should never happen; flag for debug
                    o_overflow <= 1'b1;
                end
            end
        end
    end

    // ─────────────────────────────────────────────────────────────────────────
    // Read port – present front entry to ltsm_decoder; advance on i_read_enable
    // ─────────────────────────────────────────────────────────────────────────
    always_ff @(posedge i_clk or negedge i_rst_n) begin
        if (!i_rst_n) begin
            rd_ptr     <= 3'b000;
            o_header   <= 64'b0;
            o_data     <= 64'b0;
            o_has_data <= 1'b0;
        end else begin
            if (i_read_enable && !w_empty) begin
                o_header   <= mem_header  [rd_ptr[1:0]];
                o_data     <= mem_data    [rd_ptr[1:0]];
                o_has_data <= mem_has_data[rd_ptr[1:0]];
                rd_ptr     <= rd_ptr + 3'b001;
            end
        end
    end

endmodule : SB_RX_FIFO