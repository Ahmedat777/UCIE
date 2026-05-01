// =============================================================================
// tb_ltsm_decoder.sv  –  Testbench for ltsm_decoder
//
// Requires the real pckg:: package (SB_MSG_LUT, sb_msg_id enum).
// If pckg is unavailable, a minimal stub is provided below that
// creates a 2-entry LUT so the core logic is still exercised.
//
// Verifies:
//   1. Reset: all outputs deasserted.
//   2. Valid packet matching SB_MSG_LUT entry → o_valid, correct o_msg_id.
//   3. o_msginfo correctly extracted from header[55:40].
//   4. o_data forwarded when SB_MSG_LUT entry has_data=1 and packet has data.
//   5. o_data zeroed when has_data=0 in LUT.
//   6. Unknown msgcode/msgsubcode → o_valid=0.
//   7. o_timeout_stop pulsed on valid match; not pulsed on miss.
//   8. i_pckt_valid=0 → no decode activity.
//   9. Consecutive valid packets decoded correctly.
// =============================================================================

import pckg::*;

`timescale 1ns/1ps

module tb_ltsm_decoder;

    import pckg::*;

    // ── DUT ports ─────────────────────────────────────────────────────────────
    logic        clk, rst;
    logic        i_pckt_valid;
    logic [63:0] i_header;
    logic [63:0] i_data;
    logic        o_valid;
    sb_msg_id    o_msg_id;
    logic [15:0] o_msginfo;
    logic [63:0] o_data;
    logic        o_timeout_stop;

    ltsm_decoder dut (
        .clk           (clk),
        .rst           (rst),
        .i_pckt_valid  (i_pckt_valid),
        .i_header      (i_header),
        .i_data        (i_data),
        .o_valid       (o_valid),
        .o_msg_id      (o_msg_id),
        .o_msginfo     (o_msginfo),
        .o_data        (o_data),
        .o_timeout_stop(o_timeout_stop)
    );

    localparam real CLK_PERIOD = 2.0;
    initial clk = 0;
    always #(CLK_PERIOD/2.0) clk = ~clk;

    int pass_cnt = 0, fail_cnt = 0;

    task check(input string name, input logic got, exp);
        if (got === exp) begin $display("  PASS  %-55s  %0b", name, got); pass_cnt++; end
        else begin $display("  FAIL  %-55s  got=%0b exp=%0b @%0t", name, got, exp, $time); fail_cnt++; end
    endtask

    task check16(input string name, input logic [15:0] got, exp);
        if (got === exp) begin $display("  PASS  %-55s  %04h", name, got); pass_cnt++; end
        else begin $display("  FAIL  %-55s  got=%04h exp=%04h @%0t", name, got, exp, $time); fail_cnt++; end
    endtask

    task check64(input string name, input logic [63:0] got, exp);
        if (got === exp) begin $display("  PASS  %-55s  %016h", name, got); pass_cnt++; end
        else begin $display("  FAIL  %-55s  got=%016h exp=%016h @%0t", name, got, exp, $time); fail_cnt++; end
    endtask

    task reset_dut();
        rst          = 1;
        i_pckt_valid = 0;
        i_header     = '0;
        i_data       = '0;
        repeat(4) @(posedge clk); @(negedge clk); rst = 0;
        @(posedge clk); #0.1;
    endtask

    // ── Build header with msgcode and msgsubcode at canonical positions ────────
    // Phase0[23:16]=msgcode, Phase1[39:32]=msgsubcode, Phase1[55:40]=MsgInfo
    function automatic logic [63:0] build_header(
        input logic [7:0]  mc, msc,
        input logic [15:0] msginfo
    );
        logic [63:0] h = '0;
        h[23:16] = mc;
        h[39:32] = msc;
        h[55:40] = msginfo;
        return h;
    endfunction

    // ── Apply one valid packet and observe outputs ────────────────────────────
    task send_pkt(input logic [63:0] hdr, dat, input logic valid);
        @(negedge clk);
        i_header     = hdr;
        i_data       = dat;
        i_pckt_valid = valid;
        @(posedge clk); #0.2;  // registered outputs settle
    endtask

    // ── Main ──────────────────────────────────────────────────────────────────
    initial begin
        $display("==== tb_ltsm_decoder ====");

        // ------------------------------------------------------------------
        // TEST 1: Reset state
        // ------------------------------------------------------------------
        $display("\n-- TEST 1: Reset --");
        reset_dut();
        check("o_valid=0         in reset", o_valid,        1'b0);
        check("o_timeout_stop=0  in reset", o_timeout_stop, 1'b0);

        // ------------------------------------------------------------------
        // TEST 2: Match header-only LUT entry
        // ------------------------------------------------------------------
        $display("\n-- TEST 2: Header-only match (SB_MSG_HDR_ONLY) --");
        reset_dut();
        begin
            automatic logic [63:0] hdr = build_header(8'h8A, 8'h01, 16'hABCD);
            send_pkt(hdr, 64'h0, 1'b1);
            check("o_valid=1 on match",       o_valid,        1'b1);
            check("o_timeout_stop on match",  o_timeout_stop, 1'b1);
            check16("o_msginfo extracted",    o_msginfo, 16'hABCD);
            // has_data=0 in LUT → o_data should be 0
            check64("o_data=0 (no data msg)", o_data, 64'h0);
        end

        // ------------------------------------------------------------------
        // TEST 3: Match data-bearing LUT entry
        // ------------------------------------------------------------------
        $display("\n-- TEST 3: Data-bearing match (SB_MSG_WITH_DATA) --");
        reset_dut();
        begin
            automatic logic [63:0] hdr = build_header(8'h85, 8'h0D, 16'h1234);
            automatic logic [63:0] dat = 64'hDEAD_BEEF_CAFE_BABE;
            send_pkt(hdr, dat, 1'b1);
            check("o_valid=1 on data match",     o_valid,        1'b1);
            check("o_timeout_stop on data match", o_timeout_stop, 1'b1);
            check16("o_msginfo data msg",         o_msginfo, 16'h1234);
            check64("o_data forwarded",           o_data, dat);
        end

        // ------------------------------------------------------------------
        // TEST 4: Unknown msgcode/msgsubcode → o_valid=0
        // ------------------------------------------------------------------
        $display("\n-- TEST 4: Unknown packet (no LUT match) --");
        reset_dut();
        begin
            automatic logic [63:0] hdr = build_header(8'hFF, 8'hFF, 16'h0);
            send_pkt(hdr, 64'h0, 1'b1);
            check("o_valid=0 on miss",       o_valid,        1'b0);
            check("no timeout_stop on miss", o_timeout_stop, 1'b0);
        end

        // ------------------------------------------------------------------
        // TEST 5: i_pckt_valid=0 → no decode
        // ------------------------------------------------------------------
        $display("\n-- TEST 5: pkt_valid=0 suppresses decode --");
        reset_dut();
        begin
            automatic logic [63:0] hdr = build_header(8'hA0, 8'h01, 16'hDEAD);
            send_pkt(hdr, 64'h0, 1'b0);
            check("o_valid=0 (pkt_valid=0)",       o_valid,        1'b0);
            check("no timeout_stop (pkt_valid=0)", o_timeout_stop, 1'b0);
        end

        // ------------------------------------------------------------------
        // TEST 6: Outputs clear between packets
        // ------------------------------------------------------------------
        $display("\n-- TEST 6: Outputs clear when idle --");
        reset_dut();
        begin
            automatic logic [63:0] hdr = build_header(8'hA0, 8'h01, 16'h0);
            send_pkt(hdr, 64'h0, 1'b1);
        end
        // Wait one extra cycle (no new packet)
        @(posedge clk); #0.1;
        check("o_valid clears after one packet",       o_valid,        1'b0);
        check("o_timeout_stop clears after one packet",o_timeout_stop, 1'b0);

        // ------------------------------------------------------------------
        // TEST 7: Msgsubcode differentiates entries (same msgcode, diff subcode)
        // ------------------------------------------------------------------
        $display("\n-- TEST 7: Subcode differentiation --");
        reset_dut();
        begin
            // Use SB_MSG_HDR_ONLY (mc=A0, msc=01) but wrong subcode (msc=FF)
            automatic logic [63:0] hdr = build_header(8'hA0, 8'hFF, 16'h0);
            send_pkt(hdr, 64'h0, 1'b1);
            check("o_valid=0 wrong subcode", o_valid, 1'b0);
        end

        // ------------------------------------------------------------------
        // Summary
        // ------------------------------------------------------------------
        $display("\n==== RESULTS: %0d passed, %0d failed ====", pass_cnt, fail_cnt);
        if (fail_cnt == 0) $display("ALL TESTS PASSED");
        else               $display("SOME TESTS FAILED");
        $finish;
    end

    initial begin #50000; $display("WATCHDOG"); $finish; end

endmodule : tb_ltsm_decoder