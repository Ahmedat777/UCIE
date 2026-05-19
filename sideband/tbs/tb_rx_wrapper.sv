`timescale 1ns/1ps

import pckg::*;

module tb_rx_wrapper;

    // =========================================================================
    // DUT ports
    // =========================================================================
    logic        clk;
    logic        rst_n;
    logic        de_ser_done;
    logic [63:0] rx_word;

    pckg::sb_msg_id  o_msg_id;
    logic [15:0] o_msginfo;
    logic [63:0] o_rx_data;
    logic        o_decoder_valid;
    logic        o_cp_error;
    logic        o_dp_error;
    logic        o_rx_sb_start_pattern;
    logic        o_rx_sb_pattern_samp_done;
    logic        o_timeout_stop;
    logic        o_de_ser_done_sampled;
    logic        o_sb_ltsm_resp_rcvd;

    // =========================================================================
    // DUT instantiation
    // =========================================================================
    SB_RX_WRAPPER #(.FIFO_DEPTH(4)) dut (
        .i_clk                      (clk),
        .i_rst_n                    (rst_n),
        .i_de_ser_done              (de_ser_done),
        .i_rx_word                  (rx_word),
        .o_msg_id                   (o_msg_id),
        .o_msginfo                  (o_msginfo),
        .o_rx_data                  (o_rx_data),
        .o_decoder_valid            (o_decoder_valid),
        .o_cp_error                 (o_cp_error),
        .o_dp_error                 (o_dp_error),
        .o_rx_sb_start_pattern      (o_rx_sb_start_pattern),
        .o_rx_sb_pattern_samp_done  (o_rx_sb_pattern_samp_done),
        .o_timeout_stop             (o_timeout_stop),
        .o_de_ser_done_sampled      (o_de_ser_done_sampled),
        .o_sb_ltsm_resp_rcvd        (o_sb_ltsm_resp_rcvd)
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
    // Helper: build a 64-bit header with correct CP/DP parity
    //
    // Field positions per ltsm_decoder.sv / error_handler.sv:
    //   opcode      [4:0]   → header[4:0]
    //   msgcode     [7:0]   → header[23:16]
    //   msgsubcode  [7:0]   → header[39:32]
    //   msginfo     [15:0]  → header[55:40]
    //   CP  [62]  = XOR of header with [62:63] zeroed
    //   DP  [63]  = XOR of data payload; 0 if no data
    // =========================================================================
    function automatic logic [63:0] build_header(
        input logic [4:0]  opcode,
        input logic [7:0]  msgcode,
        input logic [7:0]  msgsubcode,
        input logic [15:0] msginfo,
        input logic [63:0] data_word,
        input logic        has_data,
        input logic        corrupt_cp,
        input logic        corrupt_dp
    );
        logic [63:0] hdr;
        logic        cp, dp;

        hdr        = 64'h0;
        hdr[4:0]   = opcode;
        hdr[23:16] = msgcode;
        hdr[39:32] = msgsubcode;
        hdr[55:40] = msginfo;
        cp = ^hdr;
        dp = has_data ? (^data_word) : 1'b0;

        if (corrupt_cp) cp = ~cp;
        if (corrupt_dp) dp = ~dp;

        hdr[62] = cp;
        hdr[63] = dp;

        return hdr;
    endfunction

    // =========================================================================
    // Helper: drive one de_ser_done pulse with rx_word
    // =========================================================================
    task automatic send_word(input logic [63:0] word);
        @(negedge clk);
        rx_word     = word;
        de_ser_done = 1'b1;
        repeat(5) @(negedge clk);         // FIX-A: hold for one cycle then deassert
        //de_ser_done = 1'b0;
		rx_word     = 64'h0;
		//repeat(2) @(negedge clk);
    endtask

    // =========================================================================
    // Helper: send two consecutive pattern words to lock 128 UI
    // =========================================================================
    localparam logic [63:0] PATTERN_WORD = 64'h5555_5555_5555_5555;

    task automatic send_pattern_lock();
        send_word(PATTERN_WORD);            // IDLE → PATTERN_DETECT
        repeat(1) @(posedge clk);          
        send_word(PATTERN_WORD);            // PATTERN_DETECT → GENERAL_DECODE
    endtask

    // =========================================================================
    // Helper: send a no-data packet (header only)
    // =========================================================================
    task automatic send_no_data_pkt(
        input logic [4:0]  opcode,
        input logic [7:0]  msgcode,
        input logic [7:0]  msgsubcode,
        input logic [15:0] msginfo,
        input logic        corrupt_cp,
        input logic        corrupt_dp
    );
        logic [63:0] hdr;
        hdr = build_header(opcode, msgcode, msgsubcode, msginfo,
                           64'h0, 1'b0, corrupt_cp, corrupt_dp);
        send_word(hdr);
    endtask

    // =========================================================================
    // Helper: send a data packet (header then data word)
    // =========================================================================
    task automatic send_data_pkt(
        input logic [4:0]  opcode,
        input logic [7:0]  msgcode,
        input logic [7:0]  msgsubcode,
        input logic [15:0] msginfo,
        input logic [63:0] data_word,
        input logic        corrupt_cp,
        input logic        corrupt_dp
    );
        logic [63:0] hdr;
        hdr = build_header(opcode, msgcode, msgsubcode, msginfo,
                           data_word, 1'b1, corrupt_cp, corrupt_dp);
        send_word(hdr);
        send_word(data_word);
    endtask

    // =========================================================================
    // Helper: wait up to N cycles for a signal to go high
    // =========================================================================
    task automatic wait_for(ref logic sig, input int max_cycles, output logic timedout);
        int cnt;
        timedout = 1'b0;
        for (cnt = 0; cnt < max_cycles; cnt++) begin
            @(posedge clk);
            if (sig) return;
        end
        timedout = 1'b1;
    endtask

    // =========================================================================
    // Test parameters — pulled directly from ltsm_pckg.sv SB_MSG_LUT
    // =========================================================================
    // Opcodes (ltsm_pckg.sv):
    localparam logic [4:0] OP_NO_DATA = 5'b10010;   // SB_OPCODE_NO_DATA
    localparam logic [4:0] OP_DATA    = 5'b11011;   // SB_OPCODE_DATA

    // SB_SBINIT_DONE_REQ  : opcode=NO_DATA, msgcode=0x95, msgsubcode=0x01
    localparam logic [7:0] MC_SBINIT_REQ  = 8'h95;
    localparam logic [7:0] MSC_SBINIT_REQ = 8'h01;

    // SB_SBINIT_DONE_RESP : opcode=NO_DATA, msgcode=0x9A, msgsubcode=0x01
    localparam logic [7:0] MC_SBINIT_RESP  = 8'h9A;
    localparam logic [7:0] MSC_SBINIT_RESP = 8'h01;

    // SB_TXINIT_D2C_POINT_TEST_START_REQ : opcode=DATA, msgcode=0x85, msgsubcode=0x0D
    localparam logic [7:0] MC_DATA_PKT  = 8'h85;
    localparam logic [7:0] MSC_DATA_PKT = 8'h0D;

    // =========================================================================
    // Test stimulus
    // =========================================================================
    initial begin
        rst_n       = 1'b0;
        de_ser_done = 1'b0;
        rx_word     = 64'h0;

        $display("========================================================");
        $display("  SB_RX_WRAPPER Testbench");
        $display("========================================================");

        // ─────────────────────────────────────────────────────────────────────
        // TC1 – Reset behaviour
        // ─────────────────────────────────────────────────────────────────────
        $display("\n--- TC1: Reset ---");
        repeat(4) @(negedge clk);
        `CHECK(o_decoder_valid,          1'b0, "o_decoder_valid deasserted in reset")
        `CHECK(o_cp_error,               1'b0, "o_cp_error deasserted in reset")
        `CHECK(o_dp_error,               1'b0, "o_dp_error deasserted in reset")
        `CHECK(o_rx_sb_start_pattern,    1'b0, "o_rx_sb_start_pattern deasserted in reset")
        `CHECK(o_rx_sb_pattern_samp_done,1'b0, "o_rx_sb_pattern_samp_done deasserted in reset")
        `CHECK(o_timeout_stop,           1'b0, "o_timeout_stop deasserted in reset")
        `CHECK(o_sb_ltsm_resp_rcvd,      1'b0, "o_sb_ltsm_resp_rcvd deasserted in reset")

        // Release reset
        @(negedge clk);
        rst_n = 1'b1;
        repeat(2) @(negedge clk);

        // ─────────────────────────────────────────────────────────────────────
        // TC2 – SBINIT start pattern detection
       // ─────────────────────────────────────────────────────────────────────
        $display("\n--- TC2: SBINIT start pattern ---");
        begin
            send_word(PATTERN_WORD);            // word 1: IDLE → PATTERN_DETECT
            `CHECK(o_rx_sb_start_pattern, 1'b1, "o_rx_sb_start_pattern asserted after 1st pattern word")
            `CHECK(o_de_ser_done_sampled, 1'b1, "o_de_ser_done_sampled registered")

            send_word(PATTERN_WORD);            // word 2: PATTERN_DETECT → GENERAL_DECODE

            `CHECK(o_rx_sb_pattern_samp_done,1'b1, "o_rx_sb_pattern_samp_done asserted after 2nd pattern word")
        end

        // ─────────────────────────────────────────────────────────────────────
        // TC3 – No-data packet: SBINIT_DONE_REQ
        // ─────────────────────────────────────────────────────────────────────
        $display("\n--- TC3: No-data packet (SBINIT_DONE_REQ) ---");
        begin
            send_no_data_pkt(OP_NO_DATA, MC_SBINIT_REQ, MSC_SBINIT_REQ,
                             16'hCAFE, 1'b0, 1'b0);

            `CHECK(o_decoder_valid,1'b1,                      "o_decoder_valid asserted")
            `CHECK(o_msg_id,       pckg::SB_SBINIT_DONE_REQ,  "o_msg_id = SB_SBINIT_DONE_REQ")
            `CHECK(o_msginfo,      16'hCAFE,                  "o_msginfo = 0xCAFE")
            `CHECK(o_timeout_stop, 1'b1,                      "o_timeout_stop asserted")
            `CHECK(o_cp_error,     1'b0,                      "o_cp_error stays 0 (good parity)")
            `CHECK(o_dp_error,     1'b0,                      "o_dp_error stays 0 (no data)")
        end

        // ─────────────────────────────────────────────────────────────────────
        // TC4 – Data packet: TXINIT_D2C_POINT_TEST_START_REQ
        // ─────────────────────────────────────────────────────────────────────
        $display("\n--- TC4: Data packet (TXINIT_D2C_POINT_TEST_START_REQ) ---");
        begin
            logic [63:0] test_data;
            test_data = 64'hDEAD_BEEF_1234_5678;
            send_pattern_lock();

            send_data_pkt(OP_DATA, MC_DATA_PKT, MSC_DATA_PKT,
                          16'h0000, test_data, 1'b0, 1'b0);

            `CHECK(o_decoder_valid,1'b1,                                      "o_decoder_valid asserted (data pkt)")
            `CHECK(o_msg_id,       pckg::SB_TXINIT_D2C_POINT_TEST_START_REQ, "o_msg_id = SB_TXINIT_D2C_POINT_TEST_START_REQ")
//		   `CHECK(o_rx_data,      test_data,                                 "o_rx_data = payload")
            `CHECK(o_timeout_stop, 1'b1,                                      "o_timeout_stop asserted")
            `CHECK(o_cp_error,     1'b0,                                      "o_cp_error stays 0")
            `CHECK(o_dp_error,     1'b0,                                      "o_dp_error stays 0")
        end

        // ─────────────────────────────────────────────────────────────────────
        // TC5 – Correct CP/DP parity (SBINIT_DONE_REQ again)
        // ─────────────────────────────────────────────────────────────────────
        $display("\n--- TC5: Good parity (sanity check) ---");
        begin
            send_pattern_lock();
            send_no_data_pkt(OP_NO_DATA, MC_SBINIT_REQ, MSC_SBINIT_REQ,
                             16'h0000, 1'b0, 1'b0);
            `CHECK(o_cp_error, 1'b0, "o_cp_error = 0 with correct parity")
            `CHECK(o_dp_error, 1'b0, "o_dp_error = 0 with no data")
        end

        // ─────────────────────────────────────────────────────────────────────
        // TC6 – Corrupt CP parity
        // ─────────────────────────────────────────────────────────────────────
        $display("\n--- TC6: Corrupt CP parity ---");
        begin
            send_pattern_lock();
            send_no_data_pkt(OP_NO_DATA, MC_SBINIT_REQ, MSC_SBINIT_REQ,
                             16'h0000, 1'b1 , 1'b0);
            `CHECK(o_cp_error, 1'b1, "o_cp_error asserts on corrupted CP")
        end

        // ─────────────────────────────────────────────────────────────────────
        // TC7 – Corrupt DP parity
        // ─────────────────────────────────────────────────────────────────────
        $display("\n--- TC7: Corrupt DP parity ---");
        begin
            logic [63:0] bad_data;
            bad_data = 64'hBAD0_DAAA_0000_0001;
            send_pattern_lock();
            send_data_pkt(OP_DATA, MC_DATA_PKT, MSC_DATA_PKT,
                          16'h0000, bad_data, 1'b0, 1'b1 /*corrupt_dp*/);
			wait(o_dp_error == 1'b1);
            `CHECK(o_dp_error, 1'b1, "o_dp_error asserts on corrupted DP")
        end

        // ─────────────────────────────────────────────────────────────────────
        // TC8 – RESP message drives o_sb_ltsm_resp_rcvd
        // ─────────────────────────────────────────────────────────────────────
        $display("\n--- TC8: RESP message → o_sb_ltsm_resp_rcvd ---");
        begin
            send_pattern_lock();
            send_no_data_pkt(OP_NO_DATA, MC_SBINIT_RESP, MSC_SBINIT_RESP,
                             16'h0000, 1'b0, 1'b0);
		     repeat(5) @(posedge clk);
			 wait(o_sb_ltsm_resp_rcvd == 1'b1);
            `CHECK(o_sb_ltsm_resp_rcvd, 1'b1, "o_sb_ltsm_resp_rcvd asserted on RESP message")
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
        #10000;
        $display("WATCHDOG: simulation exceeded 10 µs – aborting");
        $finish;
    end

    // =========================================================================
    // Optional waveform dump
    // =========================================================================
    initial begin
        $dumpfile("tb_rx_wrapper.vcd");
        $dumpvars(0, tb_rx_wrapper);
    end

endmodule : tb_rx_wrapper