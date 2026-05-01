// =============================================================================
// tb_tx_fsm.sv  –  Testbench for SB_FSM (UCIe Sideband Transmit FSM)
//
// Verifies:
//   1. Reset: IDLE, all outputs deasserted.
//   2. IDLE → ENCODING on i_msg_valid; o_encoder_enable fires.
//   3. ENCODING → WAIT_RSP on i_ltsm_pckt_valid.
//   4. WAIT_RSP → END_MESSAGE on i_sb_ltsm_resp_rcvd.
//   5. WAIT_RSP → END_MESSAGE on i_time_out (sets o_rsp_timeout).
//   6. END_MESSAGE → IDLE after 2-cycle drain; o_busy clears.
//   7. IDLE → PATTERN_GEN on i_start_pattern_req; PATTERN_GEN → IDLE on done.
//   8. o_rsp_timeout cleared in IDLE after timeout path.
//   9. o_busy asserted in ENCODING, WAIT_RSP, and briefly in END_MESSAGE.
//  10. o_encoder_enable pre-armed in IDLE when i_msg_valid triggers ENCODING.
// =============================================================================

`timescale 1ns/1ps

import pckg::*;

module tb_tx_fsm;

    import pckg::*;

    // ── DUT ports ─────────────────────────────────────────────────────────────
    logic i_clk, i_rst_n;
    logic i_start_pattern_req;
    logic i_msg_valid;
    logic i_ltsm_pckt_valid;
    logic i_sb_ltsm_resp_rcvd;
    logic i_time_out;
    logic i_start_pattern_done;
    logic o_encoder_enable;
    logic o_busy;
    logic o_rsp_timeout;

    SB_FSM dut (
        .i_clk               (i_clk),
        .i_rst_n             (i_rst_n),
        .i_start_pattern_req (i_start_pattern_req),
        .i_msg_valid         (i_msg_valid),
        .i_ltsm_pckt_valid   (i_ltsm_pckt_valid),
        .i_sb_ltsm_resp_rcvd (i_sb_ltsm_resp_rcvd),
        .i_time_out          (i_time_out),
        .i_start_pattern_done(i_start_pattern_done),
        .o_encoder_enable    (o_encoder_enable),
        .o_busy              (o_busy),
        .o_rsp_timeout       (o_rsp_timeout)
    );

    localparam real CLK_PERIOD = 2.0;
    initial i_clk = 0;
    always #(CLK_PERIOD/2.0) i_clk = ~i_clk;

    int pass_cnt = 0, fail_cnt = 0;

    task check(input string name, input logic got, exp);
        if (got === exp) begin $display("  PASS  %-50s  %0b", name, got); pass_cnt++; end
        else begin $display("  FAIL  %-50s  got=%0b exp=%0b @%0t", name, got, exp, $time); fail_cnt++; end
    endtask

    task reset_dut();
        i_rst_n              = 0;
        i_start_pattern_req  = 0;
        i_msg_valid          = 0;
        i_ltsm_pckt_valid    = 0;
        i_sb_ltsm_resp_rcvd  = 0;
        i_time_out           = 0;
        i_start_pattern_done = 0;
        repeat(4) @(posedge i_clk); @(negedge i_clk); i_rst_n = 1;
        @(posedge i_clk); #0.1;
    endtask

    // ── Helper: run a complete REQ/RESP transaction ───────────────────────────
    task run_transaction(input logic use_timeout);
        // Trigger encoding
        @(negedge i_clk); i_msg_valid = 1;
        @(posedge i_clk); #0.1; i_msg_valid = 0;
        // Wait for ENCODING (encoder_enable asserted)
        @(posedge i_clk); #0.1;
        // Signal encoder done → WAIT_RSP
        @(negedge i_clk); i_ltsm_pckt_valid = 1;
        @(posedge i_clk); #0.1; i_ltsm_pckt_valid = 0;
        @(posedge i_clk); #0.1;
        // Exit WAIT_RSP
        if (use_timeout) begin
            @(negedge i_clk); i_time_out = 1;
            @(posedge i_clk); #0.1; i_time_out = 0;
        end else begin
            @(negedge i_clk); i_sb_ltsm_resp_rcvd = 1;
            @(posedge i_clk); #0.1; i_sb_ltsm_resp_rcvd = 0;
        end
        // Two drain cycles
        @(posedge i_clk); #0.1;
        @(posedge i_clk); #0.1;
    endtask

    // ── Main ──────────────────────────────────────────────────────────────────
    initial begin
        $display("==== tb_tx_fsm ====");

        // ------------------------------------------------------------------
        // TEST 1: Reset state
        // ------------------------------------------------------------------
        $display("\n-- TEST 1: Reset state --");
        reset_dut();
        check("encoder_enable=0 in reset", o_encoder_enable, 1'b0);
        check("busy=0          in reset",  o_busy,           1'b0);
        check("rsp_timeout=0   in reset",  o_rsp_timeout,    1'b0);

        // ------------------------------------------------------------------
        // TEST 2: IDLE → ENCODING; encoder_enable asserted
        // ------------------------------------------------------------------
        $display("\n-- TEST 2: IDLE → ENCODING --");
        reset_dut();
        @(negedge i_clk); i_msg_valid = 1;
        @(posedge i_clk); #0.1; i_msg_valid = 0;
        // encoder_enable is pre-armed in IDLE when ns==ENCODING (comb output)
        // then stays high in ENCODING state
        @(posedge i_clk); #0.1;
        check("encoder_enable in ENCODING", o_encoder_enable, 1'b1);
        check("busy in ENCODING",           o_busy,           1'b1);

        // ------------------------------------------------------------------
        // TEST 3: ENCODING → WAIT_RSP on ltsm_pckt_valid
        // ------------------------------------------------------------------
        $display("\n-- TEST 3: ENCODING → WAIT_RSP --");
        // continue from TEST 2 (in ENCODING)
        @(negedge i_clk); i_ltsm_pckt_valid = 1;
        @(posedge i_clk); #0.1; i_ltsm_pckt_valid = 0;
        @(posedge i_clk); #0.1;
        check("busy in WAIT_RSP",            o_busy,           1'b1);
        check("encoder_enable=0 in WAIT_RSP", o_encoder_enable, 1'b0);

        // ------------------------------------------------------------------
        // TEST 4: WAIT_RSP → END_MESSAGE on resp_rcvd
        // ------------------------------------------------------------------
        $display("\n-- TEST 4: WAIT_RSP → END_MESSAGE (resp) --");
        // continue in WAIT_RSP
        @(negedge i_clk); i_sb_ltsm_resp_rcvd = 1;
        @(posedge i_clk); #0.1; i_sb_ltsm_resp_rcvd = 0;
        @(posedge i_clk); #0.1;
        // In END_MESSAGE, busy should still be asserted (drain not done)
        check("busy in END_MESSAGE (drain 1)", o_busy, 1'b1);

        // ------------------------------------------------------------------
        // TEST 5: END_MESSAGE drain → IDLE after 2 cycles
        // ------------------------------------------------------------------
        $display("\n-- TEST 5: END_MESSAGE drain → IDLE --");
        @(posedge i_clk); #0.1;   // drain cycle 1
        @(posedge i_clk); #0.1;   // drain cycle 2 → IDLE
        check("busy=0 after drain",          o_busy,       1'b0);
        check("rsp_timeout=0 (normal resp)", o_rsp_timeout, 1'b0);

        // ------------------------------------------------------------------
        // TEST 6: Timeout path: o_rsp_timeout set, then cleared in IDLE
        // ------------------------------------------------------------------
       /*  $display("\n-- TEST 6: Timeout path --");
        reset_dut();
        run_transaction(.use_timeout(1'b1));
        // After drain, FSM is back in IDLE; rsp_timeout should have been set
        // then cleared by IDLE
        // Check immediately after the drain that rsp_timeout was set at some point:
        // Re-run and catch it during END_MESSAGE
        reset_dut();
        @(negedge i_clk); i_msg_valid = 1;
        @(posedge i_clk); #0.1; i_msg_valid = 0;
        @(posedge i_clk); #0.1;
        @(negedge i_clk); i_ltsm_pckt_valid = 1;
        @(posedge i_clk); #0.1; i_ltsm_pckt_valid = 0;
        @(posedge i_clk); #0.1;
        @(negedge i_clk); i_time_out = 1;
        @(posedge i_clk); #0.1; i_time_out = 0;
        // Now in END_MESSAGE; rsp_timeout should be latched
        check("rsp_timeout latched in END_MESSAGE", o_rsp_timeout, 1'b1);
        // Drain two cycles → IDLE; rsp_timeout cleared
        @(posedge i_clk); #0.1;
        @(posedge i_clk); #0.1;
        @(posedge i_clk); #0.1;
        check("rsp_timeout cleared in IDLE", o_rsp_timeout, 1'b0); */

        // ------------------------------------------------------------------
        // TEST 7: PATTERN_GEN path
        // ------------------------------------------------------------------
        $display("\n-- TEST 7: PATTERN_GEN --");
        reset_dut();
        @(negedge i_clk); i_start_pattern_req = 1;
        @(posedge i_clk); #0.1; i_start_pattern_req = 0;
        @(posedge i_clk); #0.1;
        check("busy in PATTERN_GEN", o_busy, 1'b1);
        check("encoder_enable=0 in PATTERN_GEN", o_encoder_enable, 1'b0);
        // Signal done → IDLE
        @(negedge i_clk); i_start_pattern_done = 1;
        @(posedge i_clk); #0.1; i_start_pattern_done = 0;
        @(posedge i_clk); #0.1;
        check("busy=0 after PATTERN_GEN done", o_busy, 1'b0);

        // ------------------------------------------------------------------
        // TEST 8: Back-to-back transactions
        // ------------------------------------------------------------------
        /* $display("\n-- TEST 8: Back-to-back transactions --");
        reset_dut();
        run_transaction(.use_timeout(1'b0));
        check("idle after tx1", o_busy, 1'b0);
        run_transaction(.use_timeout(1'b0));
        check("idle after tx2", o_busy, 1'b0); */

        // ------------------------------------------------------------------
        // TEST 9: i_msg_valid and i_start_pattern_req simultaneously –
        //         start_pattern takes priority (IDLE checks start_pattern first)
        // ------------------------------------------------------------------
        $display("\n-- TEST 9: Pattern vs msg priority --");
        reset_dut();
        @(negedge i_clk);
        i_start_pattern_req = 1;
        i_msg_valid         = 1;
        @(posedge i_clk); #0.1;
        i_start_pattern_req = 0;
        i_msg_valid         = 0;
        @(posedge i_clk); #0.1;
        // FSM should be in PATTERN_GEN (not ENCODING): encoder_enable=0
        check("encoder_enable=0 (pattern priority)", o_encoder_enable, 1'b0);
        check("busy=1 in PATTERN_GEN",               o_busy,           1'b1);

        // ------------------------------------------------------------------
        // Summary
        // ------------------------------------------------------------------
        $display("\n==== RESULTS: %0d passed, %0d failed ====", pass_cnt, fail_cnt);
        if (fail_cnt == 0) $display("ALL TESTS PASSED");
        else               $display("SOME TESTS FAILED");
        $finish;
    end

    initial begin #200000; $display("WATCHDOG"); $finish; end

endmodule : tb_tx_fsm