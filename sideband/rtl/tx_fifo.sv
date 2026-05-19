module SB_TX_FIFO #(
    parameter int unsigned DEPTH = 4    // must be power of 2; max 4 for 2-bit index
)(
    input  logic        i_clk,
    input  logic        i_rst_n,

    // ── Write port (from SB_PACKET_ENCODER_MUX via wrapper) ──────────────────
    input  logic        i_write_enable,   // push i_packet into FIFO
    input  logic [63:0] i_packet,         // complete 64-bit word (header, data, or pattern)

    // ── Read port (to SB_TX_FSM_Modelling → serialiser) ──────────────────────
    input  logic        i_read_enable,    // pop front entry
    output logic [63:0] o_packet,         // front-of-queue 64-bit word to serialiser

    // ── Status ────────────────────────────────────────────────────────────────
    output logic        o_empty,          // FIFO is empty  (no words queued)
    output logic        o_full,           // FIFO is full   (back-pressure to wrapper)

    // ── Serialiser ack ────────────────────────────────────────────────────────
    output logic        o_ser_done_sampled   // registered i_read_enable;
                                             // SB_TX_FSM_Modelling uses this to
                                             // confirm the pop was accepted
);

    // ─────────────────────────────────────────────────────────────────────────
    // Storage – one 64-bit word per slot
    // ─────────────────────────────────────────────────────────────────────────
    logic [63:0] mem [0:DEPTH-1];

    // ─────────────────────────────────────────────────────────────────────────
    // 3-bit pointers: [2] = wrap bit, [1:0] = slot index
    // ─────────────────────────────────────────────────────────────────────────
    logic [2:0] wr_ptr;
    logic [2:0] rd_ptr;

    // ─────────────────────────────────────────────────────────────────────────
    // Full / empty 
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
            wr_ptr <= 3'b000;
        end else if (i_write_enable && !w_full) begin
            mem[wr_ptr[1:0]] <= i_packet;
            wr_ptr           <= wr_ptr + 3'b001;
        end
    end

    // ─────────────────────────────────────────────────────────────────────────
    // Read 
    // ─────────────────────────────────────────────────────────────────────────
    always_ff @(posedge i_clk or negedge i_rst_n) begin
        if (!i_rst_n) begin
            rd_ptr   <= 3'b000;
            o_packet <= 64'b0;
        end else if (i_read_enable && !w_empty) begin
            o_packet <= mem[rd_ptr[1:0]];
            rd_ptr   <= rd_ptr + 3'b001;
        end
    end

    // ─────────────────────────────────────────────────────────────────────────
    // Serialiser ack – registered read_enable used by SB_TX_FSM_Modelling
    // ─────────────────────────────────────────────────────────────────────────
    always_ff @(posedge i_clk or negedge i_rst_n) begin
        if (!i_rst_n) o_ser_done_sampled <= 1'b0;
        else          o_ser_done_sampled <= i_read_enable;
    end

endmodule : SB_TX_FIFO