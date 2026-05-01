// =============================================================================
// SB_TX_FIFO  –  UCIe Sideband Transmit FIFO
//
// Stores encoded sideband packets (header + optional data flit) produced by
// ltsm_encoder and feeds them to SB_TX_FSM_Modelling for serialisation.
//
// Each FIFO entry holds:
//   • 64-bit header   (always present)
//   • 64-bit data     (present when has_data == 1)
//   • 1-bit has_data  (indicates whether a data flit follows the header)
//
// Interface matches SB_TX_WRAPPER instantiation:
//   Write side  ← ltsm_encoder  (i_write_enable, i_header, i_data, i_has_data)
//   Read side   → SB_TX_FSM_Modelling (i_read_enable, o_header, o_data, o_has_data)
//   Status      → tx_wrapper debug (o_empty, o_full, o_overflow)
//   Ack         → SB_TX_FSM_Modelling (o_ser_done_sampled)
//
// Implementation: circular buffer, depth = DEPTH (default 4, power-of-2).
// Gray-code-style 3-bit pointers (MSB = wrap bit, [1:0] = index) give
// unambiguous full/empty detection — same scheme as the reference code.
// =============================================================================

module SB_TX_FIFO #(
    parameter int unsigned DEPTH = 4    // must be power of 2, max 4 for 2-bit index
)(
    input  logic        i_clk,
    input  logic        i_rst_n,

    // ── Write port (from ltsm_encoder) ───────────────────────────────────────
    input  logic        i_write_enable,
    input  logic [63:0] i_header,       // encoded 64-bit packet header
    input  logic [63:0] i_data,         // data flit (valid when i_has_data)
    input  logic        i_has_data,     // 1 = this entry carries a data flit

    // ── Read port (to SB_TX_FSM_Modelling) ───────────────────────────────────
    input  logic        i_read_enable,
    output logic [63:0] o_header,       // header of the front entry
    output logic [63:0] o_data,         // data flit of the front entry
    output logic        o_has_data,     // 1 = front entry has a data flit

    // ── Status ────────────────────────────────────────────────────────────────
    output logic        o_empty,        // FIFO is empty
    output logic        o_full,         // FIFO is full
    output logic        o_overflow,     // write attempted while full (sticky pulse)

    // ── Serialiser ack ────────────────────────────────────────────────────────
    output logic        o_ser_done_sampled  // registered copy of i_read_enable
                                            // used by SB_TX_FSM_Modelling as ack
);

    // ─────────────────────────────────────────────────────────────────────────
    // Storage
    // ─────────────────────────────────────────────────────────────────────────
    logic [63:0] mem_header   [0:DEPTH-1];
    logic [63:0] mem_data     [0:DEPTH-1];
    logic        mem_has_data [0:DEPTH-1];

    // ─────────────────────────────────────────────────────────────────────────
    // 3-bit pointers: MSB = wrap bit, [1:0] = FIFO index
    // Same scheme as reference code — works for DEPTH = 4.
    // ─────────────────────────────────────────────────────────────────────────
    logic [2:0] wr_ptr;
    logic [2:0] rd_ptr;

    // ─────────────────────────────────────────────────────────────────────────
    // Full / empty (combinational)
    // empty: pointers identical
    // full:  lower bits identical, wrap bits differ
    // ─────────────────────────────────────────────────────────────────────────
    wire w_empty = (wr_ptr == rd_ptr);
    wire w_full  = (wr_ptr[1:0] == rd_ptr[1:0]) && (wr_ptr[2] != rd_ptr[2]);

    assign o_empty = w_empty;
    assign o_full  = w_full;

    // ─────────────────────────────────────────────────────────────────────────
    // Write
    // ─────────────────────────────────────────────────────────────────────────
    always_ff @(posedge i_clk or negedge i_rst_n) begin
        if (!i_rst_n) begin
            wr_ptr     <= 3'b000;
            o_overflow <= 1'b0;
        end else begin
            o_overflow <= 1'b0;
            if (i_write_enable) begin
                if (!w_full) begin
                    mem_header  [wr_ptr[1:0]] <= i_header;
                    mem_data    [wr_ptr[1:0]] <= i_data;
                    mem_has_data[wr_ptr[1:0]] <= i_has_data;
                    wr_ptr <= wr_ptr + 3'b001;
                end else begin
                    o_overflow <= 1'b1;     // write dropped — FIFO full
                end
            end
        end
    end

    // ─────────────────────────────────────────────────────────────────────────
    // Read
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

    // ─────────────────────────────────────────────────────────────────────────
    // Serialiser ack — registered read_enable
    // SB_TX_FSM_Modelling uses this to confirm the read was accepted
    // ─────────────────────────────────────────────────────────────────────────
    always_ff @(posedge i_clk or negedge i_rst_n) begin
        if (!i_rst_n) o_ser_done_sampled <= 1'b0;
        else          o_ser_done_sampled <= i_read_enable;
    end

endmodule : SB_TX_FIFO