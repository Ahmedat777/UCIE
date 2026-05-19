`timescale 1ns/1ps

import pckg::*;

module tb_sb_rx_fsm;

    // =========================================================================
    // Clock / reset
    // =========================================================================
    localparam int CLK_PERIOD = 10;   // 10 ns → 100 MHz

    logic clk;
    logic rst_n;

    initial clk = 1'b0;
    always #(CLK_PERIOD/2) clk = ~clk;

    // =========================================================================
    // DUT ports
    // =========================================================================
    // Inputs
    logic        de_ser_done;
    logic        fifo_is_pattern;
    logic        fifo_has_data;
    logic        fifo_full;
    logic        fifo_empty;
    logic [10:0] msg_id;
    logic        msg_id_valid;

    // Outputs
    logic        write_commit;
    logic        fifo_read_enable;
    logic        rx_sb_start_pattern;
    logic        rx_sb_pattern_samp_done;
    logic        msg_valid;
    logic        de_ser_done_sampled;
    logic        sb_ltsm_resp_rcvd;

    // =========================================================================
    // DUT instantiation
    // =========================================================================
    SB_RX_FSM dut (
        .i_clk                     (clk),
        .i_rst_n                   (rst_n),
        // Deserialiser
        .i_de_ser_done             (de_ser_done),
        // RX FIFO status
        .i_fifo_is_pattern         (fifo_is_pattern),
        .i_fifo_has_data           (fifo_has_data),
        //.i_fifo_valid              (fifo_valid),
        .i_fifo_full               (fifo_full),
        .i_fifo_empty              (fifo_empty),
        // ltsm_decoder feedback
        .i_msg_id                  (msg_id),
        .i_msg_id_valid            (msg_id_valid),
        // Write-side controls
        // NOTE: o_write_enable removed; write_enable is now i_de_ser_done
        //       (remote partner) connected directly in the wrapper.
        .o_write_commit            (write_commit),       // push assembled entry
        // Read-side control
        .o_fifo_read_enable        (fifo_read_enable),
        // Upper-layer outputs
        .o_rx_sb_start_pattern     (rx_sb_start_pattern),
        .o_rx_sb_pattern_samp_done (rx_sb_pattern_samp_done),
        .o_msg_valid               (msg_valid),
        .o_de_ser_done_sampled     (de_ser_done_sampled),
        .o_sb_ltsm_resp_rcvd       (sb_ltsm_resp_rcvd)
    );

    // =========================================================================
    // Scoreboard / pass-fail counters
    // =========================================================================
    int pass_count = 0;
    int fail_count = 0;

    // =========================================================================
    // Helper: print PASS / FAIL for a named condition
    // =========================================================================
    task automatic chk(input string name, input logic cond);
        if (cond) begin
            $display("  [PASS] %s", name);
            pass_count++;
        end else begin
            $display("  [FAIL] %s  (at time %0t)", name, $time);
            fail_count++;
        end
    endtask

    // =========================================================================
    // Helper: advance n rising edges then sample outputs one delta later
    // =========================================================================
    task automatic clk_tick(input int n = 1);
        repeat (n) @(posedge clk);
        #1;
    endtask

    // =========================================================================
    // Helper: send_word
    // =========================================================================
    task automatic send_word(input logic is_pat,
                             input logic has_data);
        @(posedge clk); #1;
        de_ser_done     = 1'b1;
        fifo_is_pattern = is_pat;
        fifo_has_data   = has_data;
        //fifo_valid      = fifo_vld;
        /* @(posedge clk); #1;       // hold for one clock so FF sampling is clean
        de_ser_done     = 1'b0;
        fifo_is_pattern = 1'b0; */
        // fifo_has_data and fifo_valid are left for the caller to manage
    endtask

    // =========================================================================
    // Helper: send_reset_msg
    // =========================================================================
    task automatic send_reset_msg(input pckg::sb_msg_id id);
        @(posedge clk); #1;
        msg_id       = 11'(id);
        msg_id_valid = 1'b1;
        @(posedge clk); #1;
        //msg_id_valid = 1'b0;
        msg_id       = '0;
    endtask

    // =========================================================================
    // Helper: do_pattern_lock
    // =========================================================================
    task automatic do_pattern_lock();
        msg_id       = 11'(SB_SBINIT_DONE_REQ);   // non-reset message
        msg_id_valid = 1'b1;
        send_word(1'b1, 1'b0);   // 1st pattern → IDLE transitions to PATTERN_DETECT
        send_word(1'b1, 1'b0);   // 2nd pattern → PATTERN_DETECT → GENERAL_DECODE
        clk_tick(2);             // let output register settle
        //msg_id_valid = 1'b0;
        msg_id       = '0;
    endtask

    // =========================================================================
    // Helper: do_reset
    // Apply async reset for hold_cycles then deassert; return when stable.
    // =========================================================================
    task automatic do_reset(input int hold_cycles = 3);
        rst_n        = 1'b0;
        // Clear all stimulus so no spurious inputs arrive during reset
        de_ser_done     = 1'b0;
        fifo_is_pattern = 1'b0;
        fifo_has_data   = 1'b0;
        fifo_full       = 1'b0;
        fifo_empty      = 1'b0;
        msg_id          = '0;
        msg_id_valid    = 1'b0;
        repeat (hold_cycles) @(posedge clk);
        #1;
        rst_n = 1'b1;
        #1;
    endtask

    // =========================================================================
    // Continuous monitors  (run throughout the entire simulation)
    // =========================================================================
    logic mon_header_seen;
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            mon_header_seen <= 1'b0;
        else if (de_ser_done)
            mon_header_seen <= 1'b1;
        else if (write_commit)
            mon_header_seen <= 1'b0;   // reset after commit
    end

    always @(posedge clk) begin
        if (write_commit && !mon_header_seen) begin
            $display("[MONITOR-FAIL] o_write_commit fired without prior o_write_enable at %0t", $time);
            fail_count++;
        end
    end

    // =========================================================================
    // Test stimulus
    // =========================================================================
    initial begin
        // Initialise all inputs
        rst_n           = 1'b1;
        de_ser_done     = 1'b0;
        fifo_is_pattern = 1'b0;
        fifo_has_data   = 1'b0;
        //fifo_valid      = 1'b0;
        fifo_full       = 1'b0;
        fifo_empty      = 1'b0;
        msg_id          = '0;
        msg_id_valid    = 1'b0;

        $display("========================================================");
        $display("  tb_sb_rx_fsm Testbench");
        $display("========================================================");

        // ─────────────────────────────────────────────────────────────────────
        // TC01 – Reset: all outputs must be LOW immediately after rst_n deasserts
        // ─────────────────────────────────────────────────────────────────────
        $display("\n--- TC01: Reset behaviour ---");
        do_reset();
        clk_tick(2);
        // DUT no longer exposes o_write_enable (now remote partner signal)
        chk("write_commit        == 0 after reset", write_commit            === 1'b0);
        chk("fifo_read_enable    == 0 after reset", fifo_read_enable        === 1'b0);
        chk("rx_sb_start_pattern == 0 after reset", rx_sb_start_pattern     === 1'b0);
        chk("rx_sb_pattern_done  == 0 after reset", rx_sb_pattern_samp_done === 1'b0);
        chk("msg_valid           == 0 after reset", msg_valid               === 1'b0);
        chk("de_ser_done_sampled == 0 after reset", de_ser_done_sampled     === 1'b0);
        chk("sb_ltsm_resp_rcvd   == 0 after reset", sb_ltsm_resp_rcvd       === 1'b0);

        // ─────────────────────────────────────────────────────────────────────
        // TC02 – First pattern word → o_rx_sb_start_pattern one-cycle pulse
        // ─────────────────────────────────────────────────────────────────────
        $display("\n--- TC02: Single pattern word → PATTERN_DETECT ---");
        do_reset();
        // Keep w_reset_msg LOW so PATTERN_DETECT is not immediately aborted
        msg_id       = 11'(SB_SBINIT_DONE_REQ);
        msg_id_valid = 1'b1;
        send_word(1'b1, 1'b0);          // 1st pattern word

        wait(rx_sb_start_pattern     == 1'b1);
        chk("start_pattern raised after 1st pattern",  rx_sb_start_pattern     === 1'b1);
        chk("pattern_samp_done NOT raised yet",        rx_sb_pattern_samp_done === 1'b0);
        clk_tick(1);
        chk("start_pattern returns LOW (one-cycle pulse)", rx_sb_start_pattern === 1'b0);
        msg_id_valid = 1'b0;
        msg_id       = '0;

        // ─────────────────────────────────────────────────────────────────────
        // TC03 – Two consecutive patterns → 128 UI lock → GENERAL_DECODE
        // ─────────────────────────────────────────────────────────────────────
        $display("\n--- TC03: Two consecutive patterns → GENERAL_DECODE ---");
        do_reset();
        msg_id       = 11'(SB_SBINIT_DONE_REQ);
        msg_id_valid = 1'b1;
        send_word(1'b1, 1'b0);   // 1st pattern → PATTERN_DETECT; r_prev_pattern latches
        send_word(1'b1, 1'b0);   // 2nd pattern → GENERAL_DECODE; r_pat_done fires

        wait(rx_sb_pattern_samp_done == 1'b1);
        chk("pattern_samp_done asserted after 2nd pattern", rx_sb_pattern_samp_done === 1'b1);
        chk("start_pattern NOT re-asserted on 2nd word",    rx_sb_start_pattern     === 1'b0);
        clk_tick(1);
        chk("pattern_samp_done returns LOW",                rx_sb_pattern_samp_done === 1'b0);
        msg_id_valid = 1'b0;
        msg_id       = '0;

        // ─────────────────────────────────────────────────────────────────────
        // TC04 – Noise word in PATTERN_DETECT → back to IDLE; restart works
        // ─────────────────────────────────────────────────────────────────────
        $display("\n--- TC04: Noise in PATTERN_DETECT → IDLE ---");
        do_reset();
        msg_id       = 11'(SB_SBINIT_DONE_REQ);
        msg_id_valid = 1'b1;
        send_word(1'b1, 1'b0);   // 1st pattern → PATTERN_DETECT
        send_word(1'b0, 1'b0);   // non-pattern noise → FSM returns to IDLE
        clk_tick(1);
        // Now send a fresh pattern to confirm IDLE: start_pattern must fire again
        send_word(1'b1, 1'b0);
        chk("start_pattern fires after noise-induced IDLE return", rx_sb_start_pattern === 1'b1);
        clk_tick(2);
        chk("start_pattern returns LOW after noise-restart",       rx_sb_start_pattern === 1'b0);
        msg_id_valid = 1'b0;
        msg_id       = '0;

        // ─────────────────────────────────────────────────────────────────────
        // TC05 – Keep-alive pattern in GENERAL_DECODE stays in GENERAL_DECODE
        // ─────────────────────────────────────────────────────────────────────
        $display("\n--- TC05: Keep-alive pattern in GENERAL_DECODE ---");
        do_reset();
        do_pattern_lock();       // → GENERAL_DECODE; msg_id_valid cleared inside
        // Drive a non-reset valid msg_id to keep w_reset_msg LOW
        msg_id       = 11'(SB_SBINIT_DONE_REQ);
        msg_id_valid = 1'b1;

        repeat (4) begin
            send_word(1'b1, 1'b0);
            // check after each word
            chk("write_commit NOT raised during keep-alive", write_commit === 1'b0);
            chk("msg_valid NOT raised during keep-alive",    msg_valid    === 1'b0);
        end
        msg_id_valid = 1'b0;
        msg_id       = '0;

        // ─────────────────────────────────────────────────────────────────────
        // TC06 – Header-only packet (i_fifo_has_data=0) full flow
        // ─────────────────────────────────────────────────────────────────────
        $display("\n--- TC06: Header-only packet flow ---");
        do_reset();
        do_pattern_lock();       // → GENERAL_DECODE

        //fifo_valid    = 1'b1;
        fifo_has_data = 1'b0;   // no data flit
        msg_id        = 11'(SB_SBINIT_DONE_REQ);
        msg_id_valid  = 1'b1;

        // Phase 1: header word → GENERAL_DECODE, FSM moves to HEADER_LATCH
        // write_enable (i_de_ser_done) pulses at the FIFO directly; FSM does not.
        send_word(1'b0, 1'b0);          // non-pattern word; cs moves to HEADER_LATCH
        clk_tick(1);

        // Phase 2: HEADER_LATCH with no data → commit immediately
        send_word(1'b0, 1'b0);
        chk("write_commit asserted (header-only)",  write_commit     === 1'b1);
        chk("fifo_read_enable asserted",            fifo_read_enable === 1'b1);
        chk("msg_valid asserted",                   msg_valid        === 1'b1);
        clk_tick(2);
        chk("write_commit returns LOW",             write_commit     === 1'b0);
        chk("msg_valid returns LOW",                msg_valid        === 1'b0);

        msg_id_valid = 1'b0;
        msg_id       = '0;

        // ─────────────────────────────────────────────────────────────────────
        // TC07 – Header + data packet full flow
        // ─────────────────────────────────────────────────────────────────────
        $display("\n--- TC07: Header+data packet flow ---");
        do_reset();
        do_pattern_lock();

        //fifo_valid    = 1'b1;
        fifo_has_data = 1'b1;   // data flit expected
        msg_id        = 11'(SB_SBINIT_DONE_REQ);
        msg_id_valid  = 1'b1;

        // Header word
        send_word(1'b0, 1'b1);          // non-pattern, has_data=1
        chk("write_commit NOT asserted in HEADER_LATCH",     write_commit  === 1'b0);
        chk("msg_valid NOT asserted yet",                    msg_valid     === 1'b0);
        clk_tick(1);

        // Data flit
        fifo_has_data = 1'b0;           // data word carries no further data flag
        send_word(1'b0, 1'b0);          // data flit → DATA_LATCH output fires
        chk("write_commit asserted (data pkt)",  write_commit     === 1'b1);
        chk("fifo_read_enable asserted",         fifo_read_enable === 1'b1);
        chk("msg_valid asserted",                msg_valid        === 1'b1);
        clk_tick(1);
        chk("write_commit returns LOW",          write_commit     === 1'b0);
        chk("msg_valid returns LOW",             msg_valid        === 1'b0);

        msg_id_valid = 1'b0;
        msg_id       = '0;

        // ─────────────────────────────────────────────────────────────────────
        // TC08 – Reset message (SB_SBINIT_OUT_OF_RESET) in GENERAL_DECODE → IDLE
        // ─────────────────────────────────────────────────────────────────────
        $display("\n--- TC08: SB_SBINIT_OUT_OF_RESET in GENERAL_DECODE → IDLE ---");
        do_reset();
        do_pattern_lock();              // → GENERAL_DECODE

        send_reset_msg(SB_SBINIT_OUT_OF_RESET);   // valid reset-class ID → IDLE
        clk_tick(2);
        // Confirm IDLE: a fresh pattern must fire start_pattern
        msg_id       = 11'(SB_SBINIT_DONE_REQ);
        msg_id_valid = 1'b1;
        send_word(1'b1, 1'b0);
        chk("start_pattern fires after SBINIT_OUT_OF_RESET → IDLE", rx_sb_start_pattern === 1'b1);
        msg_id_valid = 1'b0;
        msg_id       = '0;

        // ─────────────────────────────────────────────────────────────────────
        // TC09 – Reset message (SB_RDI_REQ_LINKRESET) in HEADER_LATCH → IDLE
        // ─────────────────────────────────────────────────────────────────────
        $display("\n--- TC09: SB_RDI_REQ_LINKRESET in HEADER_LATCH → IDLE ---");
        do_reset();
        do_pattern_lock();

        //fifo_valid    = 1'b1;
        fifo_has_data = 1'b1;
        msg_id        = 11'(SB_SBINIT_DONE_REQ);
        msg_id_valid  = 1'b1;

        send_word(1'b0, 1'b1);          // header → HEADER_LATCH
        clk_tick(1);                    // settle in HEADER_LATCH

        // Now inject a reset message (overrides the non-reset ID above)
        send_reset_msg(SB_RDI_REQ_LINKRESET);
        clk_tick(2);
        chk("write_commit NOT fired on HEADER_LATCH abort", write_commit === 1'b0);
        chk("msg_valid NOT fired on HEADER_LATCH abort",    msg_valid    === 1'b0);

        // Confirm IDLE
        fifo_has_data = 1'b0;
        msg_id        = 11'(SB_SBINIT_DONE_REQ);
        msg_id_valid  = 1'b1;
        send_word(1'b1, 1'b0);
        chk("start_pattern fires after HEADER_LATCH abort → IDLE", rx_sb_start_pattern === 1'b1);
        msg_id_valid = 1'b0;
        msg_id       = '0;

        // ─────────────────────────────────────────────────────────────────────
        // TC10 – Reset message (SB_RDI_RSP_LINKRESET) in DATA_LATCH → IDLE
        // ─────────────────────────────────────────────────────────────────────
        $display("\n--- TC10: SB_RDI_RSP_LINKRESET in DATA_LATCH → IDLE ---");
        do_reset();
        do_pattern_lock();

        //fifo_valid    = 1'b1;
        fifo_has_data = 1'b1;
        msg_id        = 11'(SB_SBINIT_DONE_REQ);
        msg_id_valid  = 1'b1;

        send_word(1'b0, 1'b1);          // header → HEADER_LATCH
        clk_tick(1);                    // HEADER_LATCH sees has_data=1 → DATA_LATCH
        clk_tick(1);                    // settle in DATA_LATCH (no de_ser_done yet)

        // Inject reset before the data flit arrives
        send_reset_msg(SB_RDI_RSP_LINKRESET);
        clk_tick(2);
        chk("write_commit NOT fired (DATA_LATCH abort)", write_commit === 1'b0);
        chk("msg_valid NOT fired (DATA_LATCH abort)",    msg_valid    === 1'b0);

        // Confirm IDLE
        fifo_has_data = 1'b0;
        msg_id        = 11'(SB_SBINIT_DONE_REQ);
        msg_id_valid  = 1'b1;
        send_word(1'b1, 1'b0);
        chk("start_pattern fires after DATA_LATCH abort → IDLE", rx_sb_start_pattern === 1'b1);
        msg_id_valid = 1'b0;
        msg_id       = '0;

        // ─────────────────────────────────────────────────────────────────────
        // TC11 – Non-reset msg_id_valid does NOT force IDLE while locked
        // ─────────────────────────────────────────────────────────────────────
        $display("\n--- TC11: Non-reset msg_id does NOT force IDLE ---");
        do_reset();
        do_pattern_lock();              // → GENERAL_DECODE

        // Drive a non-reset REQ — must hold i_msg_id_valid=1
        @(posedge clk); #1;
        msg_id       = 11'(SB_SBINIT_DONE_REQ);
        msg_id_valid = 1'b1;
        @(posedge clk); #1;
        msg_id_valid = 1'b0;
        clk_tick(2);

        msg_id       = 11'(SB_SBINIT_DONE_REQ);
        msg_id_valid = 1'b1;
        send_word(1'b1, 1'b0);
        chk("no spurious start_pattern from non-reset msg",   rx_sb_start_pattern === 1'b0);
        chk("no spurious msg_valid from non-reset msg",       msg_valid           === 1'b0);
        msg_id_valid = 1'b0;
        msg_id       = '0;

        // ─────────────────────────────────────────────────────────────────────
        // TC12 – Multiple back-to-back header-only packets
        // ─────────────────────────────────────────────────────────────────────
        $display("\n--- TC12: Multiple back-to-back header-only packets ---");
        do_reset();
        do_pattern_lock();

        //fifo_valid    = 1'b1;
        fifo_has_data = 1'b0;
        msg_id        = 11'(SB_SBINIT_DONE_REQ);
        msg_id_valid  = 1'b1;

        begin : tc12_blk
            int commit_count = 0;
            int msg_count    = 0;

            repeat (4) begin
                send_word(1'b0, 1'b0);  // header flit → GENERAL_DECODE; write_enable fires
                send_word(1'b0, 1'b0);  // next de_ser_done in HEADER_LATCH → commit
                // outputs are registered; sample after send_word's internal clock step
                if (write_commit) commit_count++;
                if (msg_valid)    msg_count++;
                clk_tick(1);            // drain pipeline before next packet
            end
            chk("4 commits for 4 header-only pkts",   commit_count == 4);
            chk("4 msg_valid for 4 header-only pkts",  msg_count   == 4);
        end

        msg_id_valid = 1'b0;
        msg_id       = '0;

        // ─────────────────────────────────────────────────────────────────────
        // TC13 – o_de_ser_done_sampled is exactly one clock delayed
        // ─────────────────────────────────────────────────────────────────────
        $display("\n--- TC13: de_ser_done_sampled is 1-cycle delayed ---");
        do_reset();
        clk_tick(2);
        @(posedge clk); #1;
        de_ser_done = 1'b1;
        // o_de_ser_done_sampled is registered: appears one clock later
        @(posedge clk); #1;
        chk("de_ser_done_sampled HIGH one cycle after de_ser_done assertion",
             de_ser_done_sampled === 1'b1);
        de_ser_done = 1'b0;
        @(posedge clk); #1;
        chk("de_ser_done_sampled LOW one cycle after de_ser_done deassertion",
             de_ser_done_sampled === 1'b0);

        // ─────────────────────────────────────────────────────────────────────
        // TC14 – write_commit never asserted simultaneously with a spurious
        // ─────────────────────────────────────────────────────────────────────
        $display("\n--- TC14: write_commit fires once per packet (no double commit) ---");
        do_reset();
        do_pattern_lock();

        //fifo_valid    = 1'b1;
        fifo_has_data = 1'b1;
        msg_id        = 11'(SB_SBINIT_DONE_REQ);
        msg_id_valid  = 1'b1;

        send_word(1'b0, 1'b1);  // header
        clk_tick(1);
        fifo_has_data = 1'b0;
        send_word(1'b0, 1'b0);  // data
        clk_tick(2);
        // Monitor B would have flagged any commit without prior de_ser_done.
        chk("no double commit or spurious commit (monitor clean)", 1'b1);

        msg_id_valid = 1'b0;
        msg_id       = '0;

        // ─────────────────────────────────────────────────────────────────────
        // TC15 – write_commit never fires before a de_ser_done has been seen
        // ─────────────────────────────────────────────────────────────────────
        $display("\n--- TC15: write_commit never fires before a flit is received ---");
        do_reset();
        // Keep msg_id_valid=1 non-reset so w_reset_msg stays LOW in PATTERN_DETECT
        msg_id       = 11'(SB_SBINIT_DONE_REQ);
        msg_id_valid = 1'b1;
        // Send noise words before a lock is established
        repeat (5) begin
            send_word(1'b0, 1'b0);   // non-pattern; FSM stays in IDLE
            clk_tick(1);
        end
        chk("write_commit NOT fired before lock",  write_commit  === 1'b0);
        msg_id_valid = 1'b0;
        msg_id       = '0;

        // ─────────────────────────────────────────────────────────────────────
        // TC16 – o_rx_sb_start_pattern fires exactly once per lock sequence
        // ─────────────────────────────────────────────────────────────────────
        $display("\n--- TC16: start_pattern fires exactly once per lock ---");
        do_reset();
        begin : tc16_blk
            int sp_count = 0;
            fork
                begin : tc16_stim
                    msg_id       = 11'(SB_SBINIT_DONE_REQ);
                    msg_id_valid = 1'b1;
                    send_word(1'b1, 1'b0);   // 1st pattern
                    send_word(1'b1, 1'b0);   // 2nd pattern → lock
                    clk_tick(5);
                    msg_id_valid = 1'b0;
                    msg_id       = '0;
                end
                begin : tc16_mon
                    repeat (20) begin
                        @(posedge clk); #1;
                        if (rx_sb_start_pattern) sp_count++;
                    end
                end
            join
            chk("start_pattern asserted exactly once per lock",  sp_count == 1);
        end

        // ─────────────────────────────────────────────────────────────────────
        // TC17 – o_rx_sb_pattern_samp_done fires exactly once per lock sequence
        // ─────────────────────────────────────────────────────────────────────
        $display("\n--- TC17: pattern_samp_done fires exactly once per lock ---");
        do_reset();
        begin : tc17_blk
            int pd_count = 0;
            fork
                begin : tc17_stim
                    msg_id       = 11'(SB_SBINIT_DONE_REQ);
                    msg_id_valid = 1'b1;
                    send_word(1'b1, 1'b0);
                    send_word(1'b1, 1'b0);
                    clk_tick(5);
                    msg_id_valid = 1'b0;
                    msg_id       = '0;
                end
                begin : tc17_mon
                    repeat (20) begin
                        @(posedge clk); #1;
                        if (rx_sb_pattern_samp_done) pd_count++;
                    end
                end
            join
            chk("pattern_samp_done asserted exactly once per lock", pd_count == 1);
        end

        // ─────────────────────────────────────────────────────────────────────
        // TC18 – o_sb_ltsm_resp_rcvd pulses when a RESP msg_id is decoded
        // ─────────────────────────────────────────────────────────────────────
        $display("\n--- TC18: o_sb_ltsm_resp_rcvd on RESP msg_id ---");
        do_reset();
        do_pattern_lock();

        // Inject a RESP-class msg_id
        @(posedge clk); #1;
        msg_id       = 11'(SB_SBINIT_DONE_RESP);
        msg_id_valid = 1'b1;
        @(posedge clk); #1;
        // sb_ltsm_resp_rcvd is registered: appears one clock after i_msg_id_valid
        chk("sb_ltsm_resp_rcvd HIGH on RESP msg_id",         sb_ltsm_resp_rcvd === 1'b1);
        msg_id_valid = 1'b0;
        msg_id       = '0;
        @(posedge clk); #1;
        chk("sb_ltsm_resp_rcvd returns LOW after RESP pulse", sb_ltsm_resp_rcvd === 1'b0);

        // Inject a REQ-class msg_id — must NOT assert sb_ltsm_resp_rcvd
        @(posedge clk); #1;
        msg_id       = 11'(SB_SBINIT_DONE_REQ);
        msg_id_valid = 1'b1;
        @(posedge clk); #1;
        chk("sb_ltsm_resp_rcvd LOW on REQ msg_id",           sb_ltsm_resp_rcvd === 1'b0);
        msg_id_valid = 1'b0;
        msg_id       = '0;

        // ─────────────────────────────────────────────────────────────────────
        // Summary
        // ─────────────────────────────────────────────────────────────────────
        $display("\n========================================================");
        $display("  RESULTS:  PASS=%0d   FAIL=%0d", pass_count, fail_count);
        $display("========================================================\n");

        if (fail_count > 0)
            $display("SOME TESTS FAILED");
        else
            $display("ALL TESTS PASSED");

        $finish;
    end

    // =========================================================================
    // Optional waveform dump
    // =========================================================================
    initial begin
        $dumpfile("tb_sb_rx_fsm.vcd");
        $dumpvars(0, tb_sb_rx_fsm);
    end

endmodule : tb_sb_rx_fsm