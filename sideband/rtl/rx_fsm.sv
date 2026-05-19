import pckg::*;

module SB_RX_FSM (
    input  logic        i_clk,
    input  logic        i_rst_n,

    // ── Deserialiser interface ────────────────────────────────────────────────
    
    input  logic        i_de_ser_done,              // pulse: new 64-bit word captured by FIFO

    // ── RX FIFO status / pattern flag ────────────────────────────────────────
    
    input  logic        i_fifo_is_pattern,          // HIGH when FIFO asm-reg == SB_START_PATTERN
    input  logic        i_fifo_has_data,            // front entry carries a data flit
    input  logic        i_fifo_full,                // RX FIFO is full  — suppress writes
    input  logic        i_fifo_empty,               // RX FIFO is empty — suppress reads

    // ── RX FIFO write-side controls ──────────────────────────────────────────
    
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
    output logic        o_de_ser_done_sampled,       // registered ack to deserialiser
    // ── To TX wrapper (same die, direct cross-path) ───────────────────────────

    output logic        o_sb_ltsm_resp_rcvd         // RESP decoded → release TX WAIT_RSP
);

// =============================================================================
// FSM STATE ENCODING
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
// =============================================================================

    function automatic logic is_reset_msg(input logic [10:0] id);
        case (id)
            // Cast sb_msg_id enum values to [10:0] for comparison
            11'(SB_RDI_REQ_LINKRESET),
            11'(SB_RDI_RSP_LINKRESET): return 1'b1;
            default:                    return 1'b0;
        endcase
    endfunction

    // Combinational flag: reset message arrived from ltsm_decoder
    logic w_reset_msg;
    assign w_reset_msg = is_reset_msg(i_msg_id);

// =============================================================================
// RESP MESSAGE DETECTION
// =============================================================================

    function automatic logic is_resp_msg(input logic [10:0] id);
        logic [7:0] mc;
        mc = SB_MSG_LUT[sb_msg_id'(id)].msgcode;
        return (mc[3:0] == 4'hA) || (mc == 8'h02);
    endfunction

    logic w_resp_msg;
    assign w_resp_msg = i_msg_id_valid && is_resp_msg(i_msg_id);

// =============================================================================
// PATTERN LOCK REGISTER
// =============================================================================

    logic [1:0] r_pattern_cnt;
logic       r_prev_pattern;

always_ff @(posedge i_clk or negedge i_rst_n) begin
    if (!i_rst_n) begin
        r_pattern_cnt  <= 2'd0;
        r_prev_pattern <= 1'b0;

    end else if (i_de_ser_done) begin

        // consecutive pattern counter
        if (i_fifo_is_pattern) begin

            // saturate at 2
            if (r_pattern_cnt != 2'd2)
                r_pattern_cnt <= r_pattern_cnt + 2'd1;

        end else begin
            r_pattern_cnt <= 2'd0;
        end

        // assert after 2 consecutive patterns
        r_prev_pattern <= (r_pattern_cnt >= 2'd1) && i_fifo_is_pattern;
    end
end

// =============================================================================
// DESERIALISER ACK
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
// =============================================================================

    always_comb begin
        ns = cs;   

        case (cs)

            IDLE: begin
                if (i_de_ser_done && i_fifo_is_pattern)
                    ns = PATTERN_DETECT;
            end

            PATTERN_DETECT: begin
                if (w_reset_msg) begin
                    ns = IDLE;
                end else if (i_de_ser_done) begin  
				if (r_prev_pattern)
                        ns = GENERAL_DECODE;   // 128 UI confirmed — pattern locked
                else
				        ns = IDLE;
			    end
			end

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

            HEADER_LATCH: begin
                if (w_reset_msg) begin
                    ns = IDLE;
                end else if (i_fifo_has_data) begin
                    ns = DATA_LATCH;           // wait for data flit  [E6]
                end else begin
                    ns = GENERAL_DECODE;       // header-only: commit done inline [E6]
                end
            end

            DATA_LATCH: begin
                if (w_reset_msg) begin
                    ns = IDLE;
                end else if (i_de_ser_done) begin
                    ns = GENERAL_DECODE;       // data captured; commit and loop
                end
            end

        endcase
    end

// =============================================================================
// OUTPUT LOGIC  
// =============================================================================

    logic r_start_pat;    // → o_rx_sb_start_pattern
    logic r_pat_done;     // → o_rx_sb_pattern_samp_done
    logic r_msg_valid;    // → o_msg_valid
    logic r_commit;       // → o_write_commit
    logic r_fifo_rd;      // → o_fifo_read_enable
    logic r_resp_rcvd;    // → o_sb_ltsm_resp_rcvd

    always_comb begin
        r_start_pat  = 1'b0;
        r_pat_done   = 1'b0;
        r_msg_valid  = 1'b0;
        r_commit     = 1'b0;
        r_fifo_rd    = 1'b0;
        r_resp_rcvd  = w_resp_msg;  // pulses any cycle a RESP is decoded
		
		if (i_de_ser_done && !w_reset_msg && !i_fifo_full)
                    r_commit = 1'b1;

        case (cs)

            IDLE: begin 
                if (i_de_ser_done && i_fifo_is_pattern)
                    r_start_pat = 1'b1;
            end

            PATTERN_DETECT: begin
                if (i_de_ser_done && i_fifo_is_pattern && r_prev_pattern)
                    r_pat_done = 1'b1;
            end

            GENERAL_DECODE: begin
                // write_enable is now i_de_ser_done directly (remote partner);
                // FSM only drives fifo_rd and msg_valid for header-only path
                if (i_de_ser_done && !i_fifo_is_pattern && ns == HEADER_LATCH && !i_fifo_empty) begin
					r_fifo_rd   = 1'b1;   // pop to ltsm_decoder
                    r_msg_valid = 1'b1;
				end
            end
			
            HEADER_LATCH: begin
                // Gate commit on !i_fifo_full; gate read on !i_fifo_empty
                if (i_de_ser_done && !i_fifo_has_data && !w_reset_msg
                        && !i_fifo_empty) begin
                    r_fifo_rd   = 1'b1;   // pop to ltsm_decoder
                    r_msg_valid = 1'b1;   // tell ltsm_decoder a packet is ready
                end
            end

            DATA_LATCH: begin
                if (i_de_ser_done && !w_reset_msg && !i_fifo_empty) begin
                    r_fifo_rd    = 1'b1;   // read to decoder
                    r_msg_valid  = 1'b1;   // notify decoder
                end
            end

            default: ;

        endcase
    end

// =============================================================================
// OUTPUT REGISTER STAGE
// =============================================================================

    always_ff @(posedge i_clk or negedge i_rst_n) begin
        if (!i_rst_n) begin
            o_rx_sb_start_pattern     <= 1'b0;
            o_rx_sb_pattern_samp_done <= 1'b0;
            o_msg_valid               <= 1'b0;
            o_write_commit            <= 1'b0;
            o_fifo_read_enable        <= 1'b0;
            o_sb_ltsm_resp_rcvd       <= 1'b0;
        end else begin
		
            o_rx_sb_start_pattern     <= r_start_pat;
            o_rx_sb_pattern_samp_done <= r_pat_done;
            o_msg_valid               <= r_msg_valid;
            o_write_commit            <= r_commit;
            o_fifo_read_enable        <= r_fifo_rd;
            o_sb_ltsm_resp_rcvd       <= r_resp_rcvd;
        end
    end

endmodule : SB_RX_FSM