module SB_RX_FIFO #(
    parameter int unsigned DEPTH = 4    // must be power of 2, max 4 for 2-bit index
)(
    input  logic        i_clk,
    input  logic        i_rst_n,

    // ── Write port – from RX deserialiser (via SerDes) ───────────────────────
    input  logic        i_write_enable,     // pulse: current flit on i_rx_word is valid
    input  logic [63:0] i_rx_word,          // 64-bit received flit (header or data, time-multiplexed)
    input  logic        i_write_commit,     // pulse: commit assembled entry to FIFO

    // ── Read port – to ltsm_decoder ──────────────────────────────────────────
    input  logic        i_read_enable,      // pop one entry (consume front)
    output logic [63:0] o_header,           // header of the front entry
    output logic [63:0] o_data,             // data flit of the front entry
    output logic        o_has_data,         // 1 = front entry has a data flit
    //output logic        o_valid,            // entry ready for decoder (!o_empty)

    // ── Pattern flag (for SB_RX_FSM) ─────────────────────────────────────────
    output logic        o_is_pattern,       // asm-reg word == SB_START_PATTERN

    // ── Status ────────────────────────────────────────────────────────────────
    output logic        o_empty,
    output logic        o_full
);

    // ─────────────────────────────────────────────────────────────────────────
    // Opcode definitions – bits [4:0] of the header flit
    // ─────────────────────────────────────────────────────────────────────────
    localparam logic [4:0] SB_OPCODE_NO_DATA = 5'b10010;
    localparam logic [4:0] SB_OPCODE_DATA    = 5'b11011;

    // ─────────────────────────────────────────────────────────────────────────
    // Storage arrays
    // ─────────────────────────────────────────────────────────────────────────
    logic [63:0] mem_header   [0:DEPTH-1];
    logic [63:0] mem_data     [0:DEPTH-1];
    logic        mem_has_data [0:DEPTH-1];

    // ─────────────────────────────────────────────────────────────────────────
    // 3-bit pointers: MSB = wrap bit, [1:0] = FIFO index
    // ─────────────────────────────────────────────────────────────────────────
    logic [2:0] wr_ptr;
    logic [2:0] rd_ptr;

    // ─────────────────────────────────────────────────────────────────────────
    // Assembly registers
    // ─────────────────────────────────────────────────────────────────────────
    logic [63:0] r_asm_header;
    logic [63:0] r_asm_data;
    logic        r_asm_has_data;    // decoded internally from opcode on phase 1

    // ─────────────────────────────────────────────────────────────────────────
    // Opcode decode – combinational, only meaningful when phase-1 flit arrives.
    // Checks bits [4:0] of the incoming word against SB_OPCODE_DATA.
    // ─────────────────────────────────────────────────────────────────────────
	
	logic w_flit_has_data;
	
	always_ff @(posedge i_clk or negedge i_rst_n) begin
	    if (!i_rst_n)
		    w_flit_has_data = 1'b0;
	    if (i_rx_word[4:0] == SB_OPCODE_DATA)
            w_flit_has_data = 1'b1;
	    else if (i_rx_word[4:0] == 5'b10010)
			w_flit_has_data = 1'b0;
    end
	
    // ─────────────────────────────────────────────────────────────────────────
    // Flit-phase tracker
    // ─────────────────────────────────────────────────────────────────────────
    logic r_hdr_received;

    always_ff @(posedge i_clk or negedge i_rst_n) begin
        if (!i_rst_n)
            r_hdr_received <= 1'b0;
        else if (!w_flit_has_data)
            r_hdr_received = 1'b0;                          // entry committed → reset phase
        else if (!r_hdr_received && w_flit_has_data)
            r_hdr_received <= 1'b1;                          // data-bearing header → expect data flit next
    end

    // ─────────────────────────────────────────────────────────────────────────
    // Full / empty
    // ─────────────────────────────────────────────────────────────────────────
    wire w_empty = (wr_ptr == rd_ptr);
    wire w_full  = (wr_ptr[1:0] == rd_ptr[1:0]) && (wr_ptr[2] != rd_ptr[2]);

    always_ff @(posedge i_clk or negedge i_rst_n) begin
        if (!i_rst_n) o_empty <= 1'b1;
        else          o_empty <= w_empty;
    end

    localparam logic [63:0] SB_START_PATTERN = {32{2'b01}};  // 64'h5555_5555_5555_5555

    assign o_full       = w_full;
    //assign o_valid      = !o_empty;
    assign o_is_pattern = (i_rx_word == SB_START_PATTERN);

    // ─────────────────────────────────────────────────────────────────────────
    // Write-side assembly// ─────────────────────────────────────────────────────────────────────────
    always_ff @(posedge i_clk or negedge i_rst_n) begin
        if (!i_rst_n) begin
            r_asm_header   <= 64'b0;
            r_asm_data     <= 64'b0;
            r_asm_has_data <= 1'b0;
            wr_ptr         <= 3'b000;
        end else begin
            if (i_write_enable && i_rx_word != SB_START_PATTERN) begin
                if (!r_hdr_received && i_rx_word != 1'b0) begin
                    // Phase 1: header flit – decode opcode here
                    r_asm_header   <= i_rx_word;
                    r_asm_has_data <= w_flit_has_data;   // 1 if DATA opcode, 0 if NO_DATA
                end else if (w_flit_has_data && r_asm_header != i_rx_word && i_rx_word != 1'b0) begin
                    // Phase 2: data flit
                    r_asm_data <= i_rx_word;
                end
            end

            if (i_write_commit && !o_is_pattern && !w_full) begin
                mem_header  [wr_ptr[1:0]] <= r_asm_header;
                mem_data    [wr_ptr[1:0]] <= r_asm_has_data ? r_asm_data : 64'b0;
                mem_has_data[wr_ptr[1:0]] <= r_asm_has_data;
                wr_ptr <= wr_ptr + 3'b001;
            end
        end
    end

    // ─────────────────────────────────────────────────────────────────────────
    // Read port
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