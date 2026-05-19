`timescale 1ns/1ps

import pckg::*;

module tb_timeout_counter;

    // =========================================================================
    // DUT ports
    // =========================================================================
    logic        clk;
    logic        rst_n;
    logic        i_start;
    logic        i_stop;
    sb_msg_id    i_msg_id;
    sb_msg_id    i_rx_msg_id;
    logic [15:0] i_rx_msginfo;
    logic        i_start_pattern_req;
    logic        i_rx_sb_pattern_samp_done;
    logic        i_pattern_time_out;
    logic        o_time_out;

    // =========================================================================
    // DUT instantiation
    // =========================================================================
    TIME_OUT_COUNTER dut (
        .i_clk                    (clk),
        .i_rst_n                  (rst_n),
        .i_start                  (i_start),
        .i_stop                   (i_stop),
        .i_msg_id                 (i_msg_id),
        .i_rx_msg_id              (i_rx_msg_id),
        .i_rx_msginfo             (i_rx_msginfo),
        .i_start_pattern_req      (i_start_pattern_req),
        .i_rx_sb_pattern_samp_done(i_rx_sb_pattern_samp_done),
        .i_pattern_time_out       (i_pattern_time_out),
        .o_time_out               (o_time_out)
    );

    // =========================================================================
    // Clock  (800 MHz → 1.25 ns period)
    // =========================================================================
    initial clk = 0;
    always #0.5 clk = ~clk;

    // =========================================================================
    // Scoreboard / pass-fail counters
    // =========================================================================
    int pass_cnt = 0;
    int fail_cnt = 0;

    // =========================================================================
    // Helper macros
    // =========================================================================
    `define CHECK(sig, exp, msg) \
        if ((sig) === (exp)) begin \
            $display("  PASS  %s  (got %0h) %0t", msg, sig, $time); \
            pass_cnt++; \
        end else begin \
            $display("  FAIL  %s  expected=%0h  got=%0h %0t", msg, exp, sig, $time); \
            fail_cnt++; \
        end

    // =========================================================================
    // Helper: pulse i_start for one cycle with the given msg_id
    // =========================================================================
    task automatic start_count(input sb_msg_id msg);
        @(negedge clk);
        i_start  = 1'b1;
        i_msg_id = msg;
        @(negedge clk);
        i_start  = 1'b0;
    endtask

    // =========================================================================
    // Helper: pulse i_stop for one cycle with the given rx_msg_id / msginfo
    // =========================================================================
    task automatic stop_count(input sb_msg_id rx_msg, input logic [15:0] msginfo);
        @(negedge clk);
        i_stop       = 1'b1;
        i_rx_msg_id  = rx_msg;
        i_rx_msginfo = msginfo;
        @(negedge clk);
        i_stop       = 1'b0;
        i_rx_msginfo = 16'h0;
    endtask

    // =========================================================================
    // Test stimulus
    // =========================================================================
    initial begin
        // Initialise all inputs
        rst_n                    = 1'b0;
        i_start                  = 1'b0;
        i_stop                   = 1'b0;
        i_msg_id                 = SB_LFSR_CLEAR_ERROR_REQ;
        i_rx_msg_id              = SB_LFSR_CLEAR_ERROR_REQ;
        i_rx_msginfo             = 16'h0;
        i_start_pattern_req      = 1'b0;
        i_rx_sb_pattern_samp_done= 1'b0;
        i_pattern_time_out       = 1'b0;

        $display("========================================================");
        $display("  TIME_OUT_COUNTER Testbench");
        $display("========================================================");

        // ─────────────────────────────────────────────────────────────────────
        // TC1 – Reset behaviour
        // ─────────────────────────────────────────────────────────────────────
        $display("\n--- TC1: Reset ---");
        repeat(4) @(negedge clk);
        `CHECK(o_time_out, 1'b0, "o_time_out deasserted in reset")

        // Release reset
        @(negedge clk);
        rst_n = 1'b1;
        repeat(2) @(negedge clk);

        // ─────────────────────────────────────────────────────────────────────
        // TC2 – Start / stop (normal): counter armed then stopped cleanly
        // ─────────────────────────────────────────────────────────────────────
        $display("\n--- TC2: Start / stop (normal) ---");
        begin
            start_count(SB_LFSR_CLEAR_ERROR_REQ);
            repeat(10) @(posedge clk); #0.1;
            `CHECK(o_time_out, 1'b0, "no timeout after 10 cycles (far from limit)")

            stop_count(SB_LFSR_CLEAR_ERROR_REQ, 16'h1234);
            repeat(5) @(posedge clk); #0.1;
            `CHECK(o_time_out, 1'b0, "no timeout after normal stop")
        end

        // ─────────────────────────────────────────────────────────────────────
        // TC3 – i_pattern_time_out → o_time_out pulse (direct passthrough)
        // ─────────────────────────────────────────────────────────────────────
        $display("\n--- TC3: pattern_time_out passthrough ---");
        begin
            fork
                begin
                    @(negedge clk);
                    i_pattern_time_out = 1'b1;
                    @(negedge clk);
                    i_pattern_time_out = 1'b0;
                end
                begin
                    @(posedge clk); #0.1;   // sample during the pulse
                    `CHECK(o_time_out, 1'b1, "o_time_out asserts on i_pattern_time_out")
                end
            join
            @(posedge clk); #0.1;
            `CHECK(o_time_out, 1'b0, "o_time_out clears after pattern_time_out deasserts")
        end

        // ─────────────────────────────────────────────────────────────────────
        // TC4 – Excluded TX state: i_start with excluded msg_id does NOT arm
        // ─────────────────────────────────────────────────────────────────────
        $display("\n--- TC4: Excluded TX state ---");
        begin
            start_count(SB_TRAINERROR_ENTRY_REQ);   // excluded msg_id
            repeat(5) @(posedge clk); #0.1;
            `CHECK(o_time_out, 1'b0, "no timeout with excluded TX msg_id")
        end

        // ─────────────────────────────────────────────────────────────────────
        // TC5 – Excluded RX state: i_stop with excluded rx_msg_id is ignored
        // ─────────────────────────────────────────────────────────────────────
        $display("\n--- TC5: Excluded RX state (stop ignored) ---");
        begin
            // Arm with a valid msg
            start_count(SB_LFSR_CLEAR_ERROR_REQ);
            repeat(5) @(posedge clk); #0.1;

            // Attempt stop with excluded rx_msg_id — should be ignored
            stop_count(SB_RDI_REQ_ACTIVE, 16'h0);
            repeat(3) @(posedge clk); #0.1;
            `CHECK(o_time_out, 1'b0, "no spurious timeout (excluded RX stop ignored)")

            // Clean up: real stop
            stop_count(SB_LFSR_CLEAR_ERROR_REQ, 16'h0);
            repeat(3) @(posedge clk); #0.1;
        end

        // ─────────────────────────────────────────────────────────────────────
        // TC6 – Stall condition 1: rx_msginfo==16'hFFFF resets counter
        //        but keeps counting; no timeout should fire yet
        // ─────────────────────────────────────────────────────────────────────
        $display("\n--- TC6: Stall condition 1 (msginfo=0xFFFF) ---");
        begin
            start_count(SB_LFSR_CLEAR_ERROR_REQ);
            repeat(5) @(posedge clk); #0.1;

            stop_count(SB_LFSR_CLEAR_ERROR_REQ, 16'hFFFF);   // stall: reset counter

            repeat(5) @(posedge clk); #0.1;
            `CHECK(o_time_out, 1'b0, "no timeout after stall reset (counter restarted)")

            // Clean up
            stop_count(SB_LFSR_CLEAR_ERROR_REQ, 16'h0);
            repeat(3) @(posedge clk); #0.1;
        end

        // ─────────────────────────────────────────────────────────────────────
        // TC7 – Stall condition 2: RECAL_RESP + msginfo[1:0]==2'b11
        //        resets counter but keeps counting
        // ─────────────────────────────────────────────────────────────────────
        $display("\n--- TC7: Stall condition 2 (RECAL stall) ---");
        begin
            start_count(SB_LFSR_CLEAR_ERROR_REQ);
            repeat(3) @(posedge clk); #0.1;

            stop_count(SB_RECAL_TRACK_TX_ADJUST_RESP, 16'h0003);   // [1:0]=2'b11

            repeat(5) @(posedge clk); #0.1;
            `CHECK(o_time_out, 1'b0, "no timeout after RECAL stall reset")

            // Clean up
            stop_count(SB_LFSR_CLEAR_ERROR_REQ, 16'h0);
            repeat(3) @(posedge clk); #0.1;
        end

        // ─────────────────────────────────────────────────────────────────────
        // TC8 – i_start_pattern_req arms counter;
        //        i_rx_sb_pattern_samp_done stops it cleanly
        // ─────────────────────────────────────────────────────────────────────
        $display("\n--- TC8: start_pattern_req / pattern_samp_done ---");
        begin
            @(negedge clk);
            i_start_pattern_req = 1'b1;
            @(negedge clk);
            i_start_pattern_req = 1'b0;

            repeat(3) @(posedge clk); #0.1;

            @(negedge clk);
            i_rx_sb_pattern_samp_done = 1'b1;
            @(negedge clk);
            i_rx_sb_pattern_samp_done = 1'b0;

            repeat(3) @(posedge clk); #0.1;
            `CHECK(o_time_out, 1'b0, "no timeout (stopped by pattern_samp_done)")
        end

        // ─────────────────────────────────────────────────────────────────────
        // TC9 – Normal stop: non-stall msginfo stops counting
        // ─────────────────────────────────────────────────────────────────────
        $display("\n--- TC9: Normal stop (non-stall msginfo) ---");
        begin
            start_count(SB_LFSR_CLEAR_ERROR_REQ);
            repeat(3) @(posedge clk); #0.1;

            stop_count(SB_LFSR_CLEAR_ERROR_REQ, 16'h1234);   // non-stall msginfo

            repeat(5) @(posedge clk); #0.1;
            `CHECK(o_time_out, 1'b0, "no timeout after normal stop")
        end

        // ─────────────────────────────────────────────────────────────────────
        // Summary
        // ─────────────────────────────────────────────────────────────────────
        repeat(4) @(posedge clk);
        $display("\n========================================================");
        $display("  RESULTS:  PASS=%0d   FAIL=%0d", pass_cnt, fail_cnt);
        $display("========================================================\n");

        if (fail_cnt == 0)
            $display("ALL TESTS PASSED");
        else
            $display("SOME TESTS FAILED – review log above");

        $finish;
    end

    // =========================================================================
    // Timeout watchdog  (prevent infinite hang)
    // =========================================================================
    initial begin
        #500_000;
        $display("WATCHDOG: simulation exceeded 500 µs – aborting");
        $finish;
    end

    // =========================================================================
    // Optional waveform dump
    // =========================================================================
    initial begin
        $dumpfile("tb_timeout_counter.vcd");
        $dumpvars(0, tb_timeout_counter);
    end

endmodule : tb_timeout_counter