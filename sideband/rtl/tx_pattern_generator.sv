module SB_PATTERN_GEN #(
    parameter int unsigned SB_CLK_CYCLES_PER_MS = 800_000
)(
    input           i_clk,
    input           i_rst_n,
    input           i_start_pattern_req,        // LTSM enters SBINIT
    input           i_rx_sb_pattern_samp_done,  // partner detected our pattern
    input           i_ser_done,                 // one 64-bit word serialised
    output reg      o_start_pattern_done,       // pattern phase complete
    output reg      o_pattern_time_out,         // 8-ms timeout (→ TRAINERROR)
    output reg [63:0] o_pattern,                // pattern word to serializer
    output reg      o_pattern_valid             // o_pattern is valid this cycle
);

    // -----------------------------------------------------------------------
    // Pattern constant: bit[0]=1, bit[1]=0 … = 0x5555_5555_5555_5555
    // -----------------------------------------------------------------------
    localparam [63:0] SB_CLK_PATTERN = {32{2'b01}};   // 64'h5555_5555_5555_5555

    // -----------------------------------------------------------------------
    // Counter widths derived from parameter
    // -----------------------------------------------------------------------
    localparam int unsigned MS_CNT_W = $clog2(SB_CLK_CYCLES_PER_MS) + 1;

    // -----------------------------------------------------------------------
    // Registers
    // -----------------------------------------------------------------------
    reg [2:0]            more_four_ittr_cntr;       // extra 4 iterations after detection
    reg                  more_four_ittr_cntr_en;
    reg [MS_CNT_W-1:0]   one_ms_counter;
    reg                  one_ms_counter_en;
    reg                  send_pattern;              // 1→send, 0→hold low this 1-ms slot
    reg [2:0]            eight_ms_counter;

    // -----------------------------------------------------------------------
    // Post-detection: send 4 more iterations (spec §4.5.3.2 step 4)
    // -----------------------------------------------------------------------
    always_ff @(posedge i_clk or negedge i_rst_n) begin
        if (!i_rst_n) begin
            more_four_ittr_cntr    <= 3'd0;
            more_four_ittr_cntr_en <= 1'b0;
            o_start_pattern_done   <= 1'b0;
        end

        // Enter extra-iterations mode on detection
        else if (i_rx_sb_pattern_samp_done && !more_four_ittr_cntr_en) begin
            more_four_ittr_cntr_en <= 1'b1;
            more_four_ittr_cntr    <= 3'd0;
        end

        // Count extra iterations (each i_ser_done = one 64-bit word)
        else if (more_four_ittr_cntr_en && i_ser_done && more_four_ittr_cntr < 3'd3) begin
            more_four_ittr_cntr <= more_four_ittr_cntr + 3'd1;
        end

        // Done after 4 iterations
        else if (more_four_ittr_cntr_en && i_ser_done && more_four_ittr_cntr == 3'd3) begin
            o_start_pattern_done   <= 1'b1;
            more_four_ittr_cntr    <= 3'd0;
            more_four_ittr_cntr_en <= 1'b0;
        end

        else begin
            o_start_pattern_done <= 1'b0;
        end
    end

    // -----------------------------------------------------------------------
    // 1 ms / 1 ms alternating send/sleep counters and 8-ms timeout
    //  holding low for 1 ms, for a total of 8 ms")
    // -----------------------------------------------------------------------
    always_ff @(posedge i_clk or negedge i_rst_n) begin
        if (!i_rst_n) begin
            one_ms_counter    <= '0;
            one_ms_counter_en <= 1'b0;
            send_pattern      <= 1'b0;
            eight_ms_counter  <= 3'd0;
            o_pattern_time_out<= 1'b0;
        end

        // Stop all timing once extra iterations complete
        else if (more_four_ittr_cntr == 3'd3) begin
            one_ms_counter    <= '0;
            send_pattern      <= 1'b0;
            one_ms_counter_en <= 1'b0;
            eight_ms_counter  <= 3'd0;
        end

        // Start counting on pattern request (first 1-ms = send)
        else if (i_start_pattern_req && !one_ms_counter_en) begin
            one_ms_counter_en <= 1'b1;
            send_pattern      <= 1'b1;
        end

        else if (one_ms_counter_en) begin
            if (one_ms_counter < MS_CNT_W'(SB_CLK_CYCLES_PER_MS - 1)) begin
                one_ms_counter <= one_ms_counter + 1'b1;
            end else begin
                // One millisecond elapsed
                one_ms_counter   <= '0;
                eight_ms_counter <= eight_ms_counter + 3'd1;

                if (eight_ms_counter == 3'd7) begin
                    // 8 ms expired → timeout (spec §4.5.3.2)
                    o_pattern_time_out <= 1'b1;
                    send_pattern       <= 1'b0;
                    one_ms_counter_en  <= 1'b0;
                    eight_ms_counter   <= 3'd0;
                end else begin
                    // Toggle send/sleep for next 1-ms slot
                    send_pattern <= ~send_pattern;
                end
            end
        end

        else begin
            o_pattern_time_out <= 1'b0;
            one_ms_counter     <= '0;
            send_pattern       <= 1'b0;
            one_ms_counter_en  <= 1'b0;
            eight_ms_counter   <= 3'd0;
        end
    end

    // -----------------------------------------------------------------------
    // Pattern and valid output
    // Emit one pattern word per i_ser_done pulse while in send phase.
    // -----------------------------------------------------------------------
    always_ff @(posedge i_clk or negedge i_rst_n) begin
        if (!i_rst_n) begin
            o_pattern       <= 64'd0;
            o_pattern_valid <= 1'b0;
        end
        else if ((send_pattern || more_four_ittr_cntr_en) && i_ser_done) begin
            o_pattern       <= SB_CLK_PATTERN;  // 64'h5555_5555_5555_5555
            o_pattern_valid <= 1'b1;
        end
        else begin
            o_pattern_valid <= 1'b0;
        end
    end

endmodule : SB_PATTERN_GEN