`timescale 1ns/1ps

import pckg::*;

module tb_tx_wrapper;

    // =========================================================================
    // Parameters
    // =========================================================================
    localparam int CLK_HALF_NS = 1;       // 500 MHz
    localparam int CLK_PER_MS  = 100;     // shrunken for simulation

    // =========================================================================
    // DUT ports
    // =========================================================================
    logic        clk;
    logic        rst_n;
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
    logic [63:0] o_tx_packet;

    // =========================================================================
    // DUT instantiation
    // =========================================================================
    SB_TX_WRAPPER #(
        .FIFO_DEPTH           (4),
        .SB_CLK_CYCLES_PER_MS (CLK_PER_MS)
    ) dut (
        .i_clk                    (clk),
        .i_rst_n                  (rst_n),
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
        .TXCKSB                   (TXCKSB),
        .o_tx_packet              (o_tx_packet)
    );

    // =========================================================================
    // Clock  (500 MHz → 2 ns period)
    // =========================================================================
    initial clk = 0;
    always #CLK_HALF_NS clk = ~clk;

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
    // Helper: assert i_msg_valid for exactly one clock cycle
    // =========================================================================
    task automatic send_msg(
        input sb_msg_id    msg,
        input logic [63:0] data    = '0,
        input logic [15:0] msginfo = '0
    );
        @(negedge clk);
        i_msg_valid    = 1'b1;
        i_msg_id       = msg;
        i_ltsm_data    = data;
        i_ltsm_msginfo = msginfo;
        @(negedge clk);
        i_msg_valid    = 1'b0;
        $display("[%0t] Sent msg_id=%0s  data=%016h  msginfo=%04h",
                 $time, msg.name(), data, msginfo);
    endtask

    // =========================================================================
    // Helper: pulse i_sb_ltsm_resp_rcvd for one cycle (simulate remote RESP)
    // =========================================================================
    task automatic send_resp();
        @(negedge clk);
        i_sb_ltsm_resp_rcvd = 1'b1;
        @(negedge clk);
        i_sb_ltsm_resp_rcvd = 1'b0;
    endtask

    // =========================================================================
    // Test parameters
    // =========================================================================
    localparam logic [63:0] SB_START_PATTERN = 64'h5555_5555_5555_5555;

    // =========================================================================
    // Test stimulus
    // =========================================================================
    initial begin
        // Initialise all inputs
        rst_n                     = 1'b0;
        i_msg_valid               = 1'b0;
        i_msg_id                  = SB_SBINIT_DONE_REQ;
        i_ltsm_data               = '0;
        i_ltsm_msginfo            = '0;
        i_start_pattern_req       = 1'b0;
        i_sb_ltsm_resp_rcvd       = 1'b0;
        i_time_out                = 1'b0;
        i_rx_sb_pattern_samp_done = 1'b0;

        $display("========================================================");
        $display("  SB_TX_WRAPPER Testbench");
        $display("========================================================");

        // ─────────────────────────────────────────────────────────────────────
        // TC1 – Reset behaviour
        // ─────────────────────────────────────────────────────────────────────
        $display("\n--- TC1: Reset ---");
        repeat(4) @(negedge clk);
        `CHECK(o_busy,            1'b0,  "o_busy deasserted in reset")
        `CHECK(o_rsp_timeout,     1'b0,  "o_rsp_timeout deasserted in reset")
        `CHECK(o_timeout_start,   1'b0,  "o_timeout_start deasserted in reset")
        `CHECK(o_pattern_time_out,1'b0,  "o_pattern_time_out deasserted in reset")
        `CHECK(TXCKSB,            1'b0,  "TXCKSB deasserted in reset")
        `CHECK(o_tx_packet,       64'h0, "o_tx_packet=0 in reset")

        // Release reset
        @(negedge clk);
        rst_n = 1'b1;
        repeat(2) @(negedge clk);

        // ─────────────────────────────────────────────────────────────────────
        // TC2 – No-data message: SB_SBINIT_DONE_REQ
        // ─────────────────────────────────────────────────────────────────────
        $display("\n--- TC2: No-data message (SB_SBINIT_DONE_REQ) ---");
        begin
            logic [63:0] pkt2;

            send_msg(SB_SBINIT_DONE_REQ);
            repeat(5) @(posedge clk); #0.1;
            `CHECK(o_busy, 1'b1, "o_busy asserted after SB_SBINIT_DONE_REQ")

            // Sample o_tx_packet in a fork so the check runs concurrently
            // with the clock-controller sending the 64-UI word
            fork
                begin
                    @(posedge clk); #0.1;
                    pkt2 = o_tx_packet;
                end
            join
            `CHECK(|pkt2, 1'b1, "o_tx_packet non-zero (header flit transmitted)")

            send_resp();
            repeat(10) @(posedge clk); #0.1;
            `CHECK(o_busy, 1'b0, "o_busy cleared after RESP")
        end

        // ─────────────────────────────────────────────────────────────────────
        // TC3 – Data-bearing message: SB_MBINIT_REPAIRVAL_APPLY_REPAIR_REQ
        // ─────────────────────────────────────────────────────────────────────
        $display("\n--- TC3: Data message (SB_MBINIT_REPAIRVAL_APPLY_REPAIR_REQ) ---");
        begin
            logic [63:0] pkt3_hdr, pkt3_dat;

            send_msg(SB_MBINIT_REPAIRVAL_APPLY_REPAIR_REQ,
                     64'hDEAD_BEEF_1234_5678,
                     16'hABCD);
            repeat(5) @(posedge clk); #0.1;
            `CHECK(o_busy, 1'b1, "o_busy asserted after data message")

            // Catch header flit
            fork
                begin
                    @(posedge clk); #0.1;
                    pkt3_hdr = o_tx_packet;
                end
            join
            `CHECK(|pkt3_hdr, 1'b1, "Word 1 (header flit) non-zero")

            // Wait for full 96-UI frame (64 active + 32 dead-time) then
            // sample data flit on the next pop
            repeat(100) @(posedge clk); #0.1;
            pkt3_dat = o_tx_packet;
            `CHECK(|pkt3_dat,            1'b1, "Word 2 (data flit) non-zero")
            `CHECK(pkt3_dat !== pkt3_hdr,1'b1, "Word 2 distinct from Word 1")

            send_resp();
            repeat(10) @(posedge clk); #0.1;
            `CHECK(o_busy, 1'b0, "o_busy cleared after RESP (data msg)")
        end

        // ─────────────────────────────────────────────────────────────────────
        // TC4 – SBINIT pattern request
        // ─────────────────────────────────────────────────────────────────────
        $display("\n--- TC4: SBINIT pattern request ---");
        begin
            logic [63:0] pkt4;

            @(negedge clk);
            i_start_pattern_req = 1'b1;
            @(negedge clk);
            i_start_pattern_req = 1'b0;

            // Wait for TC3 to fully drain and pattern gen to start driving,
            // then sample o_tx_packet for the pattern word
            repeat(150) @(posedge clk); #0.1;
            pkt4 = o_tx_packet;
            `CHECK(pkt4, SB_START_PATTERN, "o_tx_packet = SB_START_PATTERN")

            // Signal RX side locked → pattern gen exits normally
            @(negedge clk);
            i_rx_sb_pattern_samp_done = 1'b1;
            @(negedge clk);
            i_rx_sb_pattern_samp_done = 1'b0;

            repeat(CLK_PER_MS * 2 + 20) @(posedge clk); #0.1;
            `CHECK(o_pattern_time_out, 1'b0, "o_pattern_time_out NOT set (normal exit)")
        end

        // ─────────────────────────────────────────────────────────────────────
        // TC5 – RSP timeout: i_time_out → o_rsp_timeout
        // ─────────────────────────────────────────────────────────────────────
        $display("\n--- TC5: RSP timeout ---");
        begin
            send_msg(SB_SBINIT_DONE_REQ);

            // Let FSM reach WAIT_RSP then fire timeout
            repeat(5) @(negedge clk);
            @(negedge clk); i_time_out = 1'b1;
            @(negedge clk); i_time_out = 1'b0;

            repeat(5) @(posedge clk); #0.1;
            `CHECK(o_rsp_timeout, 1'b1, "o_rsp_timeout asserts on i_time_out")
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
        $dumpfile("tb_tx_wrapper.vcd");
        $dumpvars(0, tb_tx_wrapper);
    end

endmodule : tb_tx_wrapper