// =============================================================================
// tb_tx_wrapper.sv  –  Simple testbench for SB_TX_WRAPPER
//
// Tests:
//   1. Reset behaviour
//   2. No-data message send  (SB_SBINIT_DONE_REQ)
//   3. Data-bearing message  (SB_MBINIT_REPAIRVAL_APPLY_REPAIR_REQ)
//   4. SBINIT pattern request
//   5. Timeout / rsp_timeout flag
// =============================================================================

`timescale 1ns/1ps

import pckg::*;

module tb_tx_wrapper;

    // -------------------------------------------------------------------------
    // Parameters – use a tiny CLK_PER_MS so timeouts fire quickly in sim
    // -------------------------------------------------------------------------
    localparam int CLK_PERIOD_NS    = 1;          // 1 GHz ≈ 800 MHz stub
    localparam int CLK_PER_MS       = 100;        // shrink enormously for sim

    // -------------------------------------------------------------------------
    // DUT signals
    // -------------------------------------------------------------------------
    logic        i_clk;
    logic        i_rst_n;
    logic        i_msg_valid;
    sb_msg_id    i_msg_id;
    logic [63:0] i_ltsm_data;
    logic [15:0] i_ltsm_msginfo;
    logic        i_start_pattern_req;
    logic        i_sb_ltsm_resp_rcvd;
    logic        i_time_out;
    logic        i_rx_sb_pattern_samp_done;

    logic        o_busy;
    logic        o_rsp_timeout;
    logic        o_timeout_start;
    sb_msg_id    o_timeout_msg_id;
    logic        o_pattern_time_out;
    logic        TXCKSB;

    // -------------------------------------------------------------------------
    // DUT instantiation
    // -------------------------------------------------------------------------
    SB_TX_WRAPPER #(
        .FIFO_DEPTH           (4),
        .SB_CLK_CYCLES_PER_MS (CLK_PER_MS)
    ) dut (
        .i_clk                    (i_clk),
        .i_rst_n                  (i_rst_n),
        .i_msg_valid              (i_msg_valid),
        .i_msg_id                 (i_msg_id),
        .i_ltsm_data              (i_ltsm_data),
        .i_ltsm_msginfo           (i_ltsm_msginfo),
        .i_start_pattern_req      (i_start_pattern_req),
        .i_sb_ltsm_resp_rcvd      (i_sb_ltsm_resp_rcvd),
        .i_time_out               (i_time_out),
        .i_rx_sb_pattern_samp_done(i_rx_sb_pattern_samp_done),
        .o_busy                   (o_busy),
        .o_rsp_timeout            (o_rsp_timeout),
        .o_timeout_start          (o_timeout_start),
        .o_timeout_msg_id         (o_timeout_msg_id),
        .o_pattern_time_out       (o_pattern_time_out),
        .TXCKSB                   (TXCKSB)
    );

    // -------------------------------------------------------------------------
    // Clock generation – 1 GHz (1 ns period)
    // -------------------------------------------------------------------------
    initial i_clk = 0;
    always #(CLK_PERIOD_NS / 2.0) i_clk = ~i_clk;

    // -------------------------------------------------------------------------
    // Helper tasks
    // -------------------------------------------------------------------------

    // Wait for N clock rising edges
    task automatic wait_clk(input int n);
        repeat (n) @(posedge i_clk);
    endtask

    // Apply reset
    task automatic apply_reset();
        i_rst_n               = 0;
        i_msg_valid           = 0;
        i_msg_id              = SB_SBINIT_DONE_REQ;
        i_ltsm_data           = '0;
        i_ltsm_msginfo        = '0;
        i_start_pattern_req   = 0;
        i_sb_ltsm_resp_rcvd   = 0;
        i_time_out            = 0;
        i_rx_sb_pattern_samp_done = 0;
        wait_clk(5);
        i_rst_n = 1;
        wait_clk(2);
        $display("[%0t] Reset released", $time);
    endtask

    // Send a single-cycle message request
    task automatic send_msg(
        input sb_msg_id    msg,
        input logic [63:0] data    = '0,
        input logic [15:0] msginfo = '0
    );
        @(posedge i_clk);
        i_msg_valid    = 1;
        i_msg_id       = msg;
        i_ltsm_data    = data;
        i_ltsm_msginfo = msginfo;
        @(posedge i_clk);
        i_msg_valid    = 0;
        $display("[%0t] Sent msg_id=%0s  data=%0h", $time, msg.name(), data);
    endtask

    // -------------------------------------------------------------------------
    // Scoreboard / checker
    // -------------------------------------------------------------------------
    int pass_cnt = 0;
    int fail_cnt = 0;

    task automatic check(
        input string label,
        input logic  got,
        input logic  expected
    );
        if (got === expected) begin
            $display("  PASS  %s", label);
            pass_cnt++;
        end else begin
            $display("  FAIL  %s  got=%0b expected=%0b", label, got, expected);
            fail_cnt++;
        end
    endtask

    // -------------------------------------------------------------------------
    // Main stimulus
    // -------------------------------------------------------------------------
    initial begin
        $dumpfile("tb_tx_wrapper.vcd");
        $dumpvars(0, tb_tx_wrapper);

        // ── TEST 1: Reset ──────────────────────────────────────────────────
        $display("\n=== TEST 1: Reset behaviour ===");
        apply_reset();
        wait_clk(2);
        check("o_busy low after reset",       o_busy,       1'b0);
        check("o_rsp_timeout low after reset", o_rsp_timeout, 1'b0);
        check("TXCKSB low after reset",       TXCKSB,       1'b0);

        // ── TEST 2: No-data message ────────────────────────────────────────
        $display("\n=== TEST 2: No-data message (SB_SBINIT_DONE_REQ) ===");
        send_msg(SB_SBINIT_DONE_REQ);
        // Give FSM time to pick up the message and go busy
        wait_clk(10);
        check("o_busy asserted after msg", o_busy, 1'b1);

        // Simulate RESP received from RX side → FSM exits WAIT_RSP
        @(posedge i_clk);
        i_sb_ltsm_resp_rcvd = 1;
        @(posedge i_clk);
        i_sb_ltsm_resp_rcvd = 0;
        wait_clk(5);
        check("o_busy cleared after resp", o_busy, 1'b0);

        // ── TEST 3: Data-bearing message ───────────────────────────────────
        $display("\n=== TEST 3: Data message (SB_MBINIT_REPAIRVAL_APPLY_REPAIR_REQ) ===");
        send_msg(SB_MBINIT_REPAIRVAL_APPLY_REPAIR_REQ,
                 64'hDEAD_BEEF_1234_5678,
                 16'hABCD);
        wait_clk(10);
        check("o_busy asserted (data msg)", o_busy, 1'b1);
        @(posedge i_clk); i_sb_ltsm_resp_rcvd = 1;
        @(posedge i_clk); i_sb_ltsm_resp_rcvd = 0;
        wait_clk(5);
        check("o_busy cleared (data msg)", o_busy, 1'b0);

        // ── TEST 4: SBINIT pattern request ────────────────────────────────
        $display("\n=== TEST 4: SBINIT pattern request ===");
        @(posedge i_clk);
        i_start_pattern_req = 1;
        @(posedge i_clk);
        i_start_pattern_req = 0;
        wait_clk(5);
        // Signal that RX side has sampled the pattern (128 UI locked)
        @(posedge i_clk);
        i_rx_sb_pattern_samp_done = 1;
        @(posedge i_clk);
        i_rx_sb_pattern_samp_done = 0;
        wait_clk(CLK_PER_MS * 2 + 20);   // wait enough for pattern gen to finish
        check("o_pattern_time_out NOT set (normal exit)", o_pattern_time_out, 1'b0);

        // ── TEST 5: Timeout flag ───────────────────────────────────────────
        $display("\n=== TEST 5: RSP timeout ===");
        send_msg(SB_SBINIT_DONE_RESP);
        wait_clk(5);
        // Assert timeout without giving resp_rcvd
        @(posedge i_clk);
        i_time_out = 1;
        @(posedge i_clk);
        i_time_out = 0;
        wait_clk(5);
        check("o_rsp_timeout set on timeout", o_rsp_timeout, 1'b1);

        // ── Summary ───────────────────────────────────────────────────────
        $display("\n==============================================");
        $display("  TX WRAPPER TB  –  PASS: %0d   FAIL: %0d", pass_cnt, fail_cnt);
        $display("==============================================\n");

        $finish;
    end

    // Safety watchdog
    initial begin
        #500_000;
        $display("WATCHDOG: simulation timeout");
        $finish;
    end

endmodule : tb_tx_wrapper