// =============================================================================
// tb_sb_rx_fsm.sv  –  Testbench for SB_RX_FSM (UCIe Sideband Receive FSM)
//
// DUT:  rxfsmtst.sv  (SB_RX_FSM)
// Pkg:  ltsm_pckg.sv (pckg::*)
//
// Test Plan
// ─────────────────────────────────────────────────────────────────────────────
//  TC01  Reset behaviour          – all outputs LOW after async reset
//  TC02  Single pattern → PATTERN_DETECT
//  TC03  Two consecutive patterns → GENERAL_DECODE (128 UI lock)
//  TC04  Noise word in PATTERN_DETECT → back to IDLE
//  TC05  Keep-alive pattern in GENERAL_DECODE → stay in GENERAL_DECODE
//  TC06  Header-only packet (i_fifo_has_data=0) full flow
//  TC07  Header + data packet    (i_fifo_has_data=1) full flow
//  TC08  Reset message (SB_SBINIT_OUT_OF_RESET)  in GENERAL_DECODE → IDLE
//  TC09  Reset message (SB_RDI_REQ_LINKRESET)    in HEADER_LATCH   → IDLE
//  TC10  Reset message (SB_RDI_RSP_LINKRESET)    in DATA_LATCH     → IDLE
//  TC11  Non-reset msg_id_valid does NOT force IDLE
//  TC12  Multiple back-to-back header-only packets
//  TC13  o_de_ser_done_sampled is a one-cycle delayed copy of i_de_ser_done
//  TC14  o_rx_header_valid and o_rx_data_valid never asserted together
//  TC15  o_write_commit never fires before a header has been latched
//  TC16  o_rx_sb_start_pattern asserted exactly once per lock sequence
//  TC17  o_rx_sb_pattern_samp_done asserted exactly once per lock sequence
// =============================================================================

`timescale 1ns/1ps

import pckg::*;

module tb_sb_rx_fsm;

// ─────────────────────────────────────────────────────────────────────────────
// Clock / reset
// ─────────────────────────────────────────────────────────────────────────────
    localparam int CLK_PERIOD = 10;   // 10 ns → 100 MHz

    logic clk;
    logic rst_n;

    initial clk = 1'b0;
    always #(CLK_PERIOD/2) clk = ~clk;

// ─────────────────────────────────────────────────────────────────────────────
// DUT signals
// ─────────────────────────────────────────────────────────────────────────────
    logic        de_ser_done;
    logic        fifo_is_pattern;
    logic        fifo_has_data;
    logic        fifo_valid;
    logic [10:0] msg_id;
    logic        msg_id_valid;

    logic        rx_header_valid;
    logic        rx_data_valid;
    logic        write_commit;
    logic        fifo_read_enable;
    logic        rx_sb_start_pattern;
    logic        rx_sb_pattern_samp_done;
    logic        msg_valid;
    logic        de_ser_done_sampled;

// ─────────────────────────────────────────────────────────────────────────────
// DUT instantiation
// ─────────────────────────────────────────────────────────────────────────────
    SB_RX_FSM dut (
        .i_clk                     (clk),
        .i_rst_n                   (rst_n),
        .i_de_ser_done             (de_ser_done),
        .i_fifo_is_pattern         (fifo_is_pattern),
        .i_fifo_has_data           (fifo_has_data),
        .i_fifo_valid              (fifo_valid),
        .i_msg_id                  (msg_id),
        .i_msg_id_valid            (msg_id_valid),
        .o_rx_header_valid         (rx_header_valid),
        .o_rx_data_valid           (rx_data_valid),
        .o_write_commit            (write_commit),
        .o_fifo_read_enable        (fifo_read_enable),
        .o_rx_sb_start_pattern     (rx_sb_start_pattern),
        .o_rx_sb_pattern_samp_done (rx_sb_pattern_samp_done),
        .o_msg_valid               (msg_valid),
        .o_de_ser_done_sampled     (de_ser_done_sampled)
    );

// ─────────────────────────────────────────────────────────────────────────────
// Scoreboard / checker helpers
// ─────────────────────────────────────────────────────────────────────────────
    int pass_count = 0;
    int fail_count = 0;

    // Convenience – check a named condition, print PASS / FAIL
    task automatic chk(input string name, input logic cond);
        if (cond) begin
            $display("  [PASS] %s", name);
            pass_count++;
        end else begin
            $display("  [FAIL] %s  (at time %0t)", name, $time);
            fail_count++;
        end
    endtask

    // Wait for rising edge then sample outputs one delta later
    task automatic clk_tick(input int n = 1);
        repeat (n) @(posedge clk);
        #1;
    endtask

    // Drive a one-cycle de_ser_done pulse with associated side-band flags
    task automatic send_word(input logic is_pat, input logic has_data,
                             input logic fifo_vld = 1'b1);
        @(posedge clk); #1;
        de_ser_done     = 1'b1;
        fifo_is_pattern = is_pat;
        fifo_has_data   = has_data;
        fifo_valid      = fifo_vld;
        //@(posedge clk); #1;
        //de_ser_done     = 1'b0;
        //fifo_is_pattern = 1'b0;
        // has_data / fifo_valid intentionally kept so HEADER_LATCH can sample them
    endtask

    // Drive a reset-class message for one cycle
    task automatic send_reset_msg(input pckg::sb_msg_id id);
        @(posedge clk); #1;
        msg_id       = 11'(id);
        msg_id_valid = 1'b1;
        //@(posedge clk); #1;
        //msg_id_valid = 1'b0;
        //msg_id       = '0;
    endtask

    // Lock the link: drive two consecutive pattern words, return in GENERAL_DECODE
    task automatic do_pattern_lock();
        send_word(1'b1, 1'b0);   // 1st pattern → PATTERN_DETECT
        clk_tick(1);             // settle r_prev_pattern
        send_word(1'b1, 1'b0);   // 2nd pattern → GENERAL_DECODE
        clk_tick(2);
    endtask

    // Apply and release async reset
    task automatic do_reset(input int hold_cycles = 3);
        rst_n = 1'b0;
        repeat (hold_cycles) @(posedge clk);
        #1;
        rst_n = 1'b1;
        #1;
    endtask

// ─────────────────────────────────────────────────────────────────────────────
// Continuous mutual-exclusion monitor  (runs throughout simulation)
// ─────────────────────────────────────────────────────────────────────────────
    always @(posedge clk) begin
        if (rx_header_valid && rx_data_valid) begin
            $display("[MONITOR-FAIL] o_rx_header_valid and o_rx_data_valid both HIGH at %0t", $time);
            fail_count++;
        end
    end

// ─────────────────────────────────────────────────────────────────────────────
// Main test sequence
// ─────────────────────────────────────────────────────────────────────────────
    initial begin
        // ── initialise all inputs ──────────────────────────────────────────────
        rst_n           = 1'b1;
        de_ser_done     = 1'b0;
        fifo_is_pattern = 1'b0;
        fifo_has_data   = 1'b0;
        fifo_valid      = 1'b0;
        msg_id          = '0;
        msg_id_valid    = 1'b0;

        // ═══════════════════════════════════════════════════════════════════════
        // TC01 – Reset: all outputs must be LOW after rst_n deasserts
        // ═══════════════════════════════════════════════════════════════════════
        $display("\n--- TC01: Reset behaviour ---");
        do_reset();
        clk_tick(2);
        chk("rx_header_valid    == 0 after reset", rx_header_valid         === 1'b0);
        chk("rx_data_valid      == 0 after reset", rx_data_valid           === 1'b0);
        chk("write_commit       == 0 after reset", write_commit            === 1'b0);
        chk("fifo_read_enable   == 0 after reset", fifo_read_enable        === 1'b0);
        chk("rx_sb_start_pattern== 0 after reset", rx_sb_start_pattern     === 1'b0);
        chk("rx_sb_pattern_done == 0 after reset", rx_sb_pattern_samp_done === 1'b0);
        chk("msg_valid          == 0 after reset", msg_valid               === 1'b0);
        chk("de_ser_done_sampled== 0 after reset", de_ser_done_sampled     === 1'b0);

        // ═══════════════════════════════════════════════════════════════════════
        // TC02 – First pattern word → o_rx_sb_start_pattern asserted
        // ═══════════════════════════════════════════════════════════════════════
        $display("\n--- TC02: Single pattern word → PATTERN_DETECT ---");
        do_reset();
        send_word(1'b1, 1'b0);          // 1st pattern
        clk_tick(1);
        chk("start_pattern raised after 1st pattern",  rx_sb_start_pattern     === 1'b1);
        chk("pattern_samp_done NOT raised yet",        rx_sb_pattern_samp_done === 1'b0);
        clk_tick(1);
        chk("start_pattern returns LOW (one-cycle pulse)", rx_sb_start_pattern  === 1'b0);

        // ═══════════════════════════════════════════════════════════════════════
        // TC03 – Two consecutive patterns → 128 UI lock → GENERAL_DECODE
       /*  // ═══════════════════════════════════════════════════════════════════════
        $display("\n--- TC03: Two consecutive patterns → GENERAL_DECODE ---");
        do_reset();
        send_word(1'b1, 1'b0);          // 1st pattern
        clk_tick(1);                    // r_prev_pattern latches
        send_word(1'b1, 1'b0);          // 2nd pattern
        clk_tick(3);
        chk("pattern_samp_done asserted after 2nd pattern", rx_sb_pattern_samp_done === 1'b1);
        chk("start_pattern NOT re-asserted",                 rx_sb_start_pattern     === 1'b0);
        clk_tick(1);
        chk("pattern_samp_done returns LOW",                 rx_sb_pattern_samp_done === 1'b0); */

        // ═══════════════════════════════════════════════════════════════════════
        // TC04 – Noise word in PATTERN_DETECT → back to IDLE
        // ═══════════════════════════════════════════════════════════════════════
        /* $display("\n--- TC04: Noise in PATTERN_DETECT → IDLE ---");
        do_reset();
        send_word(1'b1, 1'b0);          // 1st pattern → PATTERN_DETECT
        clk_tick(1);
        send_word(1'b0, 1'b0);          // non-pattern noise → IDLE
        clk_tick(2);
        // After noise FSM returns to IDLE; confirm by checking a new pattern
        // can restart the sequence (start_pattern fires again).
        send_word(1'b1, 1'b0);          // fresh 1st pattern
        clk_tick(3);
        chk("start_pattern fires after noise-induced IDLE return", rx_sb_start_pattern === 1'b1); */

        // ═══════════════════════════════════════════════════════════════════════
        // TC05 – Keep-alive pattern in GENERAL_DECODE stays in GENERAL_DECODE
        // ═══════════════════════════════════════════════════════════════════════
        $display("\n--- TC05: Keep-alive pattern in GENERAL_DECODE ---");
        do_reset();
        do_pattern_lock();              // → GENERAL_DECODE
        // Send several keep-alive patterns — none should trigger header_valid
        repeat (4) begin
            send_word(1'b1, 1'b0);
            clk_tick(1);
        end
        chk("rx_header_valid NOT raised during keep-alive", rx_header_valid === 1'b0);
        chk("msg_valid NOT raised during keep-alive",       msg_valid       === 1'b0);

        // ═══════════════════════════════════════════════════════════════════════
        // TC06 – Header-only packet full flow
        // ═══════════════════════════════════════════════════════════════════════
        $display("\n--- TC06: Header-only packet flow ---");
        do_reset();
        do_pattern_lock();              // → GENERAL_DECODE

        fifo_valid    = 1'b1;
        fifo_has_data = 1'b0;          // no data flit expected

        // Phase-1: send non-pattern word → HEADER_LATCH
        send_word(1'b0, 1'b0);         // header word (is_pat=0, has_data=0)
        clk_tick(1);
        chk("rx_header_valid pulsed",   rx_header_valid === 1'b1);
        clk_tick(1);                    // HEADER_LATCH evaluates !has_data
        chk("write_commit asserted (header-only)",   write_commit      === 1'b1);
        chk("fifo_read_enable asserted",             fifo_read_enable  === 1'b1);
        chk("msg_valid asserted",                    msg_valid         === 1'b1);
        chk("rx_data_valid NOT asserted",            rx_data_valid     === 1'b0);
        clk_tick(1);
        chk("write_commit returns LOW",              write_commit      === 1'b0);
        chk("msg_valid returns LOW",                 msg_valid         === 1'b0);

        // ═══════════════════════════════════════════════════════════════════════
        // TC07 – Header + data packet full flow
        // ═══════════════════════════════════════════════════════════════════════
        /* $display("\n--- TC07: Header+data packet flow ---");
        do_reset();
        do_pattern_lock();

        fifo_valid    = 1'b1;
        fifo_has_data = 1'b1;          // data flit expected after header

        // Header word
        send_word(1'b0, 1'b1);
        clk_tick(2);
        chk("rx_header_valid pulsed (has_data pkt)", rx_header_valid === 1'b1);
        clk_tick(1);                    // HEADER_LATCH → DATA_LATCH
        chk("write_commit NOT asserted in HEADER_LATCH (has_data)", write_commit === 1'b0);
        chk("msg_valid NOT asserted yet",                            msg_valid    === 1'b0);

        // Data flit
        fifo_has_data = 1'b0;          // clear for DATA_LATCH (not used there)
        send_word(1'b0, 1'b0);         // data word
        clk_tick(3);
        chk("rx_data_valid asserted",   rx_data_valid    === 1'b1);
        chk("write_commit asserted",    write_commit     === 1'b1);
        chk("fifo_read_enable asserted",fifo_read_enable === 1'b1);
        chk("msg_valid asserted",       msg_valid        === 1'b1);
        clk_tick(1);
        chk("rx_data_valid returns LOW",rx_data_valid    === 1'b0);
        chk("write_commit returns LOW", write_commit     === 1'b0); */

        // ═══════════════════════════════════════════════════════════════════════
        // TC08 – Reset message in GENERAL_DECODE → IDLE
        // ═══════════════════════════════════════════════════════════════════════
        $display("\n--- TC08: SB_SBINIT_OUT_OF_RESET in GENERAL_DECODE → IDLE ---");
        do_reset();
        do_pattern_lock();             // → GENERAL_DECODE
        send_reset_msg(SB_SBINIT_OUT_OF_RESET);
        clk_tick(2);
        // Confirm we are back in IDLE: a fresh pattern should fire start_pattern
        send_word(1'b1, 1'b0);
        clk_tick(1);
        chk("start_pattern fires after SBINIT_OUT_OF_RESET → IDLE", rx_sb_start_pattern === 1'b1);

        // ═══════════════════════════════════════════════════════════════════════
        // TC09 – Reset message in HEADER_LATCH → IDLE, no commit
        // ═══════════════════════════════════════════════════════════════════════
        $display("\n--- TC09: SB_RDI_REQ_LINKRESET in HEADER_LATCH → IDLE ---");
        do_reset();
        do_pattern_lock();

        fifo_valid    = 1'b1;
        fifo_has_data = 1'b1;
        send_word(1'b0, 1'b1);         // header → HEADER_LATCH
        clk_tick(1);
        // Now inject reset msg while in HEADER_LATCH
        send_reset_msg(SB_RDI_REQ_LINKRESET);
        clk_tick(2);
        chk("write_commit NOT fired on reset abort", write_commit === 1'b0);
        chk("msg_valid NOT fired on reset abort",    msg_valid    === 1'b0);
        // Confirm IDLE: fresh pattern fires start_pattern
        fifo_has_data = 1'b0;
        send_word(1'b1, 1'b0);
        clk_tick(1);
        chk("start_pattern fires after abort → IDLE", rx_sb_start_pattern === 1'b1);

        // ═══════════════════════════════════════════════════════════════════════
        // TC10 – Reset message in DATA_LATCH → IDLE, no commit
        // ═══════════════════════════════════════════════════════════════════════
        $display("\n--- TC10: SB_RDI_RSP_LINKRESET in DATA_LATCH → IDLE ---");
        do_reset();
        do_pattern_lock();

        fifo_valid    = 1'b1;
        fifo_has_data = 1'b1;
        send_word(1'b0, 1'b1);         // header → HEADER_LATCH
        clk_tick(2);                   // → DATA_LATCH

        // Inject reset msg in DATA_LATCH before de_ser_done
        send_reset_msg(SB_RDI_RSP_LINKRESET);
        clk_tick(2);
        chk("write_commit NOT fired (DATA_LATCH abort)", write_commit === 1'b0);
        chk("msg_valid NOT fired (DATA_LATCH abort)",    msg_valid    === 1'b0);

        fifo_has_data = 1'b0;
        send_word(1'b1, 1'b0);
        clk_tick(1);
        chk("start_pattern fires after DATA_LATCH abort", rx_sb_start_pattern === 1'b1);

        // ═══════════════════════════════════════════════════════════════════════
        // TC11 – Non-reset msg_id_valid does NOT force IDLE while locked
        // ═══════════════════════════════════════════════════════════════════════
        $display("\n--- TC11: Non-reset msg_id does NOT force IDLE ---");
        do_reset();
        do_pattern_lock();             // → GENERAL_DECODE

        // SB_SBINIT_DONE_REQ is not a reset message
        @(posedge clk); #1;
        msg_id       = 11'(SB_SBINIT_DONE_REQ);
        msg_id_valid = 1'b1;
        @(posedge clk); #1;
        msg_id_valid = 1'b0;
        clk_tick(2);

        // FSM should still be in GENERAL_DECODE; send keep-alive, no unexpected outputs
        send_word(1'b1, 1'b0);
        clk_tick(1);
        chk("no spurious start_pattern from non-reset msg",   rx_sb_start_pattern === 1'b0);
        chk("no spurious msg_valid from non-reset msg",       msg_valid           === 1'b0);

        // ═══════════════════════════════════════════════════════════════════════
        // TC12 – Back-to-back header-only packets
        // ═══════════════════════════════════════════════════════════════════════
        $display("\n--- TC12: Multiple back-to-back header-only packets ---");
        do_reset();
        do_pattern_lock();

        fifo_valid    = 1'b1;
        fifo_has_data = 1'b0;
        begin : tc12_blk
            int commit_count = 0;
            int msg_count    = 0;
            // Send 4 header-only packets in quick succession
            repeat (4) begin
    send_word(1'b0, 1'b0);

    // observe over 2 cycles
    repeat (2) begin
        @(posedge clk); #1;
        if (write_commit) commit_count++;
        if (msg_valid)    msg_count++;
    end
end
            chk("4 commits for 4 header-only pkts", commit_count == 4);
            chk("4 msg_valid for 4 header-only pkts", msg_count  == 4);
        end

        // ═══════════════════════════════════════════════════════════════════════
        // TC13 – de_ser_done_sampled is exactly one clock delayed
        // ═══════════════════════════════════════════════════════════════════════
        $display("\n--- TC13: de_ser_done_sampled is 1-cycle delayed ---");
        do_reset();
        clk_tick(2);
        @(posedge clk); #1;
        de_ser_done = 1'b1;
        @(posedge clk); #1;            // one cycle after assertion
        chk("de_ser_done_sampled HIGH one cycle after de_ser_done", de_ser_done_sampled === 1'b1);
        de_ser_done = 1'b0;
        @(posedge clk); #1;
        chk("de_ser_done_sampled LOW one cycle after deassertion",  de_ser_done_sampled === 1'b0);

        // ═══════════════════════════════════════════════════════════════════════
        // TC14 – Mutual exclusion: rx_header_valid and rx_data_valid never together
        //         (Continuous monitor handles this; just run a combined transaction)
        // ═══════════════════════════════════════════════════════════════════════
        $display("\n--- TC14: Mutual exclusion header_valid / data_valid ---");
        do_reset();
        do_pattern_lock();
        fifo_valid    = 1'b1;
        fifo_has_data = 1'b1;
        send_word(1'b0, 1'b1);         // header
        clk_tick(2);
        send_word(1'b0, 1'b0);         // data
        clk_tick(2);
        // Continuous monitor would have flagged any simultaneous assertion.
        chk("mutual exclusion: no simultaneous header+data valid (monitor clean)", 1'b1);

        // ═══════════════════════════════════════════════════════════════════════
        // TC15 – write_commit never fires before a header has been latched
        //         (Verified by SVA A4; exercise here to see no spurious commit)
        // ═══════════════════════════════════════════════════════════════════════
        $display("\n--- TC15: write_commit never fires before header_valid ---");
        do_reset();
        // Don't lock; sit in IDLE / PATTERN_DETECT and confirm no commit
        repeat (5) begin
            send_word(1'b0, 1'b0);     // noise words before lock
            clk_tick(1);
        end
        chk("write_commit NOT fired before lock", write_commit === 1'b0);

        // ═══════════════════════════════════════════════════════════════════════
        // TC16 – start_pattern fires exactly once per lock sequence
        // ═══════════════════════════════════════════════════════════════════════
        $display("\n--- TC16: start_pattern fires exactly once per lock ---");
        do_reset();
        begin : tc16_blk
            int sp_count = 0;
            // Monitor start_pattern during lock sequence
            fork
                begin
                    // stimulate
                    send_word(1'b1, 1'b0);
                    clk_tick(1);
                    send_word(1'b1, 1'b0);
                    clk_tick(5);
                end
                begin
                    // count
                    repeat (20) begin
                        @(posedge clk); #1;
                        if (rx_sb_start_pattern) sp_count++;
                    end
                end
            join
            chk("start_pattern asserted exactly once per lock",  sp_count == 1);
        end

        // ═══════════════════════════════════════════════════════════════════════
        // TC17 – pattern_samp_done fires exactly once per lock sequence
        // ═══════════════════════════════════════════════════════════════════════
        $display("\n--- TC17: pattern_samp_done fires exactly once per lock ---");
        do_reset();
        begin : tc17_blk
            int pd_count = 0;
            fork
                begin
                    send_word(1'b1, 1'b0);
                    clk_tick(1);
                    send_word(1'b1, 1'b0);
                    clk_tick(5);
                end
                begin
                    repeat (20) begin
                        @(posedge clk); #1;
                        if (rx_sb_pattern_samp_done) pd_count++;
                    end
                end
            join
            chk("pattern_samp_done asserted exactly once per lock", pd_count == 1);
        end

        // ═══════════════════════════════════════════════════════════════════════
        // Summary
        // ═══════════════════════════════════════════════════════════════════════
        $display("\n======================================================");
        $display("  Simulation complete: %0d PASS  /  %0d FAIL", pass_count, fail_count);
        $display("======================================================\n");
        if (fail_count > 0)
            $display("RESULT: ** SOME TESTS FAILED **");
        else
            $display("RESULT: ALL TESTS PASSED");

        $finish;
    end

// ─────────────────────────────────────────────────────────────────────────────
// Optional VCD dump
// ─────────────────────────────────────────────────────────────────────────────
    initial begin
        $dumpfile("tb_sb_rx_fsm.vcd");
        $dumpvars(0, tb_sb_rx_fsm);
    end

endmodule : tb_sb_rx_fsm