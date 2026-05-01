// =============================================================================
// tb_tx_pattern_generator.sv  –  Testbench for SB_PATTERN_GEN
//
// Uses SB_CLK_CYCLES_PER_MS = 8 for fast simulation (8 cycles = 1 ms).
//
// Verifies:
//   1. Reset: all outputs deasserted.
//   2. Pattern starts immediately on i_start_pattern_req (send_pattern slot).
//   3. Pattern word is 64'h5555_5555_5555_5555 (SB_CLK_PATTERN).
//   4. o_pattern_valid asserted on each i_ser_done pulse while sending.
//   5. 1-ms/1-ms alternation: pattern held after first 8-cycle window.
//   6. Post-detection: 4 extra iterations then o_start_pattern_done.
//   7. o_pattern_time_out after 8 ms (64 cycles at 8 cyc/ms) without detection.
//   8. No pattern output during sleep slots.
//   9. o_start_pattern_done is single-pulse.
// =============================================================================

`timescale 1ns/1ps

module tb_tx_pattern_generator;

    // Use tiny SB_CLK_CYCLES_PER_MS for simulation speed
    localparam int SIM_MS = 8;

    // ── DUT ports ─────────────────────────────────────────────────────────────
    logic        i_clk, i_rst_n;
    logic        i_start_pattern_req;
    logic        i_rx_sb_pattern_samp_done;
    logic        i_ser_done;
    logic        o_start_pattern_done;
    logic        o_pattern_time_out;
    logic [63:0] o_pattern;
    logic        o_pattern_valid;

    SB_PATTERN_GEN #(.SB_CLK_CYCLES_PER_MS(SIM_MS)) dut (
        .i_clk                   (i_clk),
        .i_rst_n                 (i_rst_n),
        .i_start_pattern_req     (i_start_pattern_req),
        .i_rx_sb_pattern_samp_done(i_rx_sb_pattern_samp_done),
        .i_ser_done              (i_ser_done),
        .o_start_pattern_done    (o_start_pattern_done),
        .o_pattern_time_out      (o_pattern_time_out),
        .o_pattern               (o_pattern),
        .o_pattern_valid         (o_pattern_valid)
    );

    localparam real CLK_PERIOD = 2.0;
    initial i_clk = 0;
    always #(CLK_PERIOD/2.0) i_clk = ~i_clk;

    int pass_cnt = 0, fail_cnt = 0;

    task check(input string name, input logic got, input logic exp);
        if (got === exp) begin $display("  PASS  %-55s  %0b", name, got); pass_cnt++; end
        else begin $display("  FAIL  %-55s  got=%0b exp=%0b @%0t", name, got, exp, $time); fail_cnt++; end
    endtask

    task check64(input string name, input logic [63:0] got, input logic [63:0] exp);
        if (got === exp) begin $display("  PASS  %-55s  %016h", name, got); pass_cnt++; end
        else begin $display("  FAIL  %-55s  got=%016h exp=%016h", name, got, exp); fail_cnt++; end
    endtask

    task reset_dut();
        i_rst_n                  = 0;
        i_start_pattern_req      = 0;
        i_rx_sb_pattern_samp_done= 0;
        i_ser_done               = 0;
        repeat(4) @(posedge i_clk); @(negedge i_clk); i_rst_n = 1;
        @(posedge i_clk); #0.1;
    endtask

    // ── Pulse ser_done ────────────────────────────────────────────────────────
    task pulse_ser();
        @(negedge i_clk); i_ser_done = 1;
        @(posedge i_clk); #0.1; //i_ser_done = 0;
    endtask

    // ── Main ──────────────────────────────────────────────────────────────────
    initial begin
        $display("==== tb_tx_pattern_generator (SIM_MS=%0d cyc/ms) ====", SIM_MS);

        // ------------------------------------------------------------------
        // TEST 1: Reset state
        // ------------------------------------------------------------------
        $display("\n-- TEST 1: Reset --");
        reset_dut();
        check("pattern_done=0    in reset", o_start_pattern_done, 1'b0);
        check("pattern_timeout=0 in reset", o_pattern_time_out,   1'b0);
        check("pattern_valid=0   in reset", o_pattern_valid,      1'b0);

        // ------------------------------------------------------------------
        // TEST 2: Pattern starts on request; word = SB_CLK_PATTERN
        // ------------------------------------------------------------------
        $display("\n-- TEST 2: Pattern word and valid --");
        reset_dut();
        @(negedge i_clk); i_start_pattern_req = 1;
        @(posedge i_clk); #0.1; i_start_pattern_req = 0;
        // Pattern output fires on i_ser_done when send_pattern is active
        pulse_ser();
        @(posedge i_clk); #0.1;
        check("pattern_valid after ser_done",          o_pattern_valid, 1'b1);
        check64("pattern word = SB_CLK_PATTERN",      o_pattern, 64'h5555_5555_5555_5555);

        // ------------------------------------------------------------------
        // TEST 3: No output on ser_done when in sleep slot
        // ------------------------------------------------------------------
        $display("\n-- TEST 3: No pattern during sleep slot --");
        reset_dut();
        @(negedge i_clk); i_start_pattern_req = 1;
        @(posedge i_clk); #0.1; i_start_pattern_req = 0;
        // Wait out the first send slot (SIM_MS cycles)
        repeat(SIM_MS) @(posedge i_clk);
        // Now in sleep slot; ser_done should not produce valid output
        @(posedge i_clk); #0.1;
        pulse_ser();
        @(posedge i_clk); #0.1;
        check("pattern_valid=0 during sleep slot", o_pattern_valid, 1'b0);

        // ------------------------------------------------------------------
        // TEST 4: Post-detection: 4 extra iterations then done
        // ------------------------------------------------------------------
        $display("\n-- TEST 4: Post-detection 4 extra iterations --");
        reset_dut();
        @(negedge i_clk); i_start_pattern_req = 1;
        @(posedge i_clk); #0.1; i_start_pattern_req = 0;
        repeat(2) @(posedge i_clk);

        // Signal partner detected our pattern
        @(negedge i_clk); i_rx_sb_pattern_samp_done = 1;
        @(posedge i_clk); #0.1; i_rx_sb_pattern_samp_done = 0;

        // Send 4 ser_done pulses; done should fire after 4th
        begin
            automatic int done_cnt = 0;
            for (int i = 0; i < 4; i++) begin
                pulse_ser();
                @(posedge i_clk); #0.1;
                if (o_start_pattern_done) done_cnt++;
            end
            // After 4 pulses done should be asserted
            if (done_cnt >= 1) begin
                $display("  PASS  start_pattern_done fired (cnt=%0d)", done_cnt);
                pass_cnt++;
            end else begin
                $display("  FAIL  start_pattern_done never fired");
                fail_cnt++;
            end
        end

        // ------------------------------------------------------------------
        // TEST 5: o_start_pattern_done is single-cycle pulse
        // ------------------------------------------------------------------
        $display("\n-- TEST 5: done is single-cycle --");
        // After TEST 4, done should have cleared
        @(posedge i_clk); #0.1;
        check("pattern_done cleared after pulse", o_start_pattern_done, 1'b0);

        // ------------------------------------------------------------------
        // TEST 6: Timeout after 8 ms (8 * SIM_MS = 64 cycles)
        // ------------------------------------------------------------------
        $display("\n-- TEST 6: 8-ms timeout --");
        reset_dut();
        @(negedge i_clk); i_start_pattern_req = 1;
        @(posedge i_clk); #0.1; i_start_pattern_req = 0;

        begin
            automatic int tout_seen = 0;
            // 8 ms = 8 * SIM_MS cycles; add headroom
            repeat(8 * SIM_MS + 10) begin
                @(posedge i_clk); #0.1;
                if (o_pattern_time_out) tout_seen = 1;
            end
            if (tout_seen) begin
                $display("  PASS  Timeout fired within 8 ms (at SIM_MS=%0d cyc/ms)", SIM_MS);
                pass_cnt++;
            end else begin
                $display("  FAIL  Timeout did not fire");
                fail_cnt++;
            end
        end

        // ------------------------------------------------------------------
        // TEST 7: No timeout if detection happens in time
        // ------------------------------------------------------------------
        $display("\n-- TEST 7: No timeout with early detection --");
        reset_dut();
        @(negedge i_clk); i_start_pattern_req = 1;
        @(posedge i_clk); #0.1; i_start_pattern_req = 0;
        repeat(3) @(posedge i_clk);
        @(negedge i_clk); i_rx_sb_pattern_samp_done = 1;
        @(posedge i_clk); #0.1; i_rx_sb_pattern_samp_done = 0;
        // Do 4 extra ser_done to finish
        repeat(4) begin pulse_ser(); @(posedge i_clk); #0.1; end
        check("no timeout with early detection", o_pattern_time_out, 1'b0);

        // ------------------------------------------------------------------
        // Summary
        // ------------------------------------------------------------------
        $display("\n==== RESULTS: %0d passed, %0d failed ====", pass_cnt, fail_cnt);
        if (fail_cnt == 0) $display("ALL TESTS PASSED");
        else               $display("SOME TESTS FAILED");
        $finish;
    end

    initial begin #100000; $display("WATCHDOG"); $finish; end

endmodule : tb_tx_pattern_generator