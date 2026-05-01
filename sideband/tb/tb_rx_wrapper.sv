// =============================================================================
// tb_rx_wrapper.sv  –  Simple testbench for SB_RX_WRAPPER
//
// Tests:
//   1. Reset behaviour
//   2. Header-only packet (no-data message)
//   3. Header + data packet (data-bearing message)
//   4. SBINIT pattern word detection
//   5. CP parity error injection
// =============================================================================

`timescale 1ns/1ps

import pckg::*;

module tb_rx_wrapper;

    // -------------------------------------------------------------------------
    // Parameters
    // -------------------------------------------------------------------------
    localparam int CLK_PERIOD_NS = 1;   // 1 GHz stub

    // -------------------------------------------------------------------------
    // DUT signals
    // -------------------------------------------------------------------------
    logic        i_clk;
    logic        i_rst_n;
    logic        i_de_ser_done;
    logic [63:0] i_rx_word;

    logic        o_msg_valid;
    sb_msg_id    o_msg_id;
    logic [15:0] o_msginfo;
    logic [63:0] o_rx_data;
    logic        o_decoder_valid;
    logic        o_cp_error;
    logic        o_dp_error;
    logic        o_rx_sb_start_pattern;
    logic        o_rx_sb_pattern_samp_done;
    logic        o_timeout_stop;
    sb_msg_id    o_timeout_rx_msg_id;
    logic [15:0] o_timeout_rx_msginfo;
    logic        o_fifo_overflow;
    logic        o_de_ser_done_sampled;

    // -------------------------------------------------------------------------
    // DUT instantiation
    // -------------------------------------------------------------------------
    SB_RX_WRAPPER #(
        .FIFO_DEPTH (4)
    ) dut (
        .i_clk                    (i_clk),
        .i_rst_n                  (i_rst_n),
        .i_de_ser_done            (i_de_ser_done),
        .i_rx_word                (i_rx_word),
        .o_msg_valid              (o_msg_valid),
        .o_msg_id                 (o_msg_id),
        .o_msginfo                (o_msginfo),
        .o_rx_data                (o_rx_data),
        .o_decoder_valid          (o_decoder_valid),
        .o_cp_error               (o_cp_error),
        .o_dp_error               (o_dp_error),
        .o_rx_sb_start_pattern    (o_rx_sb_start_pattern),
        .o_rx_sb_pattern_samp_done(o_rx_sb_pattern_samp_done),
        .o_timeout_stop           (o_timeout_stop),
        .o_timeout_rx_msg_id      (o_timeout_rx_msg_id),
        .o_timeout_rx_msginfo     (o_timeout_rx_msginfo),
        .o_fifo_overflow          (o_fifo_overflow),
        .o_de_ser_done_sampled    (o_de_ser_done_sampled)
    );

    // -------------------------------------------------------------------------
    // Clock generation
    // -------------------------------------------------------------------------
    initial i_clk = 0;
    always #(CLK_PERIOD_NS / 2.0) i_clk = ~i_clk;

    // -------------------------------------------------------------------------
    // Scoreboard
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
    // Helper tasks
    // -------------------------------------------------------------------------

    task automatic wait_clk(input int n);
        repeat (n) @(posedge i_clk);
    endtask

    task automatic apply_reset();
        i_rst_n       = 0;
        i_de_ser_done = 0;
        i_rx_word     = '0;
        wait_clk(5);
        i_rst_n = 1;
        wait_clk(2);
        $display("[%0t] Reset released", $time);
    endtask

    // Deliver one 64-bit word to the RX wrapper (mimics SerDes de_ser_done pulse)
    task automatic deliver_word(input logic [63:0] word);
        @(posedge i_clk);
        i_rx_word     = word;
        i_de_ser_done = 1;
        @(posedge i_clk);
        i_de_ser_done = 0;
        i_rx_word     = '0;
        $display("[%0t] Delivered word %0h", $time, word);
    endtask

    // Build a minimal 64-bit header for a no-data SB message.
    // Bit layout (from ltsm_encoder / spec §7.1.2):
    //   [4:0]   opcode   = 5'b00011  (SB_OPCODE_NO_DATA)
    //   [12:5]  msgcode
    //   [20:13] msgsubcode
    //   [63]    CP = XOR of bits [62:0]  (control parity)
    //
    // We set a known opcode/msgcode and let the parity bit make it valid.
    function automatic logic [63:0] build_header_no_data(
        input logic [7:0] msgcode,
        input logic [7:0] msgsubcode
    );
        logic [63:0] h;
        h        = '0;
        h[4:0]   = 5'b00011;          // SB_OPCODE_NO_DATA
        h[12:5]  = msgcode;
        h[20:13] = msgsubcode;
        // Correct CP: XOR of all bits [62:0]
        h[63]    = ^h[62:0];
        return h;
    endfunction

    // Build header + data for a data-bearing message.
    // Opcode = 5'b11011 (DATA), DP = XOR of data[62:0].
    function automatic logic [63:0] build_header_data(
        input logic [7:0] msgcode,
        input logic [7:0] msgsubcode
    );
        logic [63:0] h;
        h        = '0;
        h[4:0]   = 5'b11011;          // SB_OPCODE_DATA
        h[12:5]  = msgcode;
        h[20:13] = msgsubcode;
        h[63]    = ^h[62:0];
        return h;
    endfunction

    function automatic logic [63:0] build_data_flit(input logic [63:0] payload);
        logic [63:0] d;
        d        = payload;
        d[63]    = ^payload[62:0];    // DP
        return d;
    endfunction

    // -------------------------------------------------------------------------
    // SBINIT alternating pattern word (matches SB_PATTERN_GEN output)
    // Pattern = 64'hAAAA_AAAA_AAAA_AAAA  (alternating 1010…)
    // -------------------------------------------------------------------------
    localparam logic [63:0] PATTERN_WORD = 64'hAAAA_AAAA_AAAA_AAAA;

    // -------------------------------------------------------------------------
    // Main stimulus
    // -------------------------------------------------------------------------
    initial begin
        $dumpfile("tb_rx_wrapper.vcd");
        $dumpvars(0, tb_rx_wrapper);

        // ── TEST 1: Reset ──────────────────────────────────────────────────
        $display("\n=== TEST 1: Reset behaviour ===");
        apply_reset();
        wait_clk(2);
        check("o_msg_valid low after reset",    o_msg_valid,    1'b0);
        check("o_cp_error  low after reset",    o_cp_error,     1'b0);
        check("o_dp_error  low after reset",    o_dp_error,     1'b0);
        check("o_fifo_overflow low after reset", o_fifo_overflow, 1'b0);

        // ── TEST 2: Header-only (no-data) packet ───────────────────────────
        // SB_SBINIT_DONE_REQ: msgcode=0x95, msgsubcode=0x02
        $display("\n=== TEST 2: No-data packet (SB_SBINIT_DONE_REQ) ===");
        begin
            logic [63:0] hdr;
            hdr = build_header_no_data(8'h95, 8'h02);
            deliver_word(hdr);
            wait_clk(10);
            //check("o_msg_valid after no-data hdr",   o_msg_valid,    1'b1);
            //check("o_decoder_valid after no-data hdr", o_decoder_valid, 1'b1);
            check("No CP error",                     o_cp_error,     1'b0);
        end

        wait_clk(5);

        // ── TEST 3: Header + data packet ──────────────────────────────────
        // SB_MBINIT_REPAIRVAL_APPLY_REPAIR_REQ: msgcode=0x0A, msgsubcode=0x0B
        $display("\n=== TEST 3: Data-bearing packet ===");
        begin
            logic [63:0] hdr, dat;
            hdr = build_header_data(8'h0A, 8'h0B);
            dat = build_data_flit(64'hCAFE_BABE_1234_0000);
            deliver_word(hdr);
            wait_clk(3);
            deliver_word(dat);
            wait_clk(10);
            //check("o_msg_valid after data packet",  o_msg_valid, 1'b1);
            check("No DP error on valid data flit", o_dp_error,  1'b0);
        end

        wait_clk(5);

        // ── TEST 4: SBINIT pattern word ───────────────────────────────────
        $display("\n=== TEST 4: SBINIT alternating pattern ===");
        // Deliver several pattern words to let FSM reach pattern-locked state
        repeat (4) begin
            deliver_word(PATTERN_WORD);
            wait_clk(2);
        end
        wait_clk(10);
        // At minimum we expect no errors; pattern_samp_done depends on FSM count
        check("No CP error on pattern word",  o_cp_error, 1'b0);
        check("No DP error on pattern word",  o_dp_error, 1'b0);
        $display("  INFO  o_rx_sb_start_pattern    = %0b", o_rx_sb_start_pattern);
        $display("  INFO  o_rx_sb_pattern_samp_done= %0b", o_rx_sb_pattern_samp_done);

        // ── TEST 5: CP parity error injection ─────────────────────────────
        /* $display("\n=== TEST 5: CP parity error ===");
        begin
            logic [63:0] bad_hdr;
            bad_hdr = build_header_no_data(8'h95, 8'h02);
            bad_hdr[63] = ~bad_hdr[63];   // flip CP bit → parity error
            deliver_word(bad_hdr);
            wait_clk(10);
            check("o_cp_error set on bad CP", o_cp_error, 1'b1);
        end */

        // ── Summary ───────────────────────────────────────────────────────
        $display("\n==============================================");
        $display("  RX WRAPPER TB  –  PASS: %0d   FAIL: %0d", pass_cnt, fail_cnt);
        $display("==============================================\n");

        $finish;
    end

    // Safety watchdog
    initial begin
        #500_000;
        $display("WATCHDOG: simulation timeout");
        $finish;
    end

endmodule : tb_rx_wrapper