`timescale 1ns/1ps

module tb_tx_fsm_modeling;

    // =========================================================================
    // DUT ports
    // =========================================================================
    logic i_clk, i_rst_n;
    logic i_ser_done;
    logic i_empty;
    logic i_dead_time_done;
    logic i_read_enable_sampled;
    logic o_read_enable;
    logic o_clk_en;

    // =========================================================================
    // DUT instantiation
    // =========================================================================
    SB_TX_FSM_Modelling dut (
        .i_clk                (i_clk),
        .i_rst_n              (i_rst_n),
        .i_ser_done           (i_ser_done),
        .i_empty              (i_empty),
        .i_dead_time_done     (i_dead_time_done),
        .i_read_enable_sampled(i_read_enable_sampled),
        .o_read_enable        (o_read_enable),
        .o_clk_en             (o_clk_en)
    );

    // =========================================================================
    // Clock  (500 MHz → 2.0 ns period)
    // =========================================================================
    localparam real CLK_PERIOD = 2.0;
    initial i_clk = 0;
    always #(CLK_PERIOD/2.0) i_clk = ~i_clk;

    // =========================================================================
    // Scoreboard / pass-fail counters
    // =========================================================================
    int pass_cnt = 0, fail_cnt = 0;

    // =========================================================================
    // Helper: check 1-bit signal
    // =========================================================================
    task check(input string name, input logic got, exp);
        if (got === exp) begin
            $display("  PASS  %-55s  %0b", name, got);
            pass_cnt++;
        end else begin
            $display("  FAIL  %-55s  got=%0b exp=%0b @%0t", name, got, exp, $time);
            fail_cnt++;
        end
    endtask

    // =========================================================================
    // Helper: reset DUT
    // =========================================================================
    task reset_dut();
        i_rst_n               = 0;
        i_ser_done            = 0;
        i_empty               = 1;
        i_dead_time_done      = 0;
        i_read_enable_sampled = 0;
        repeat(4) @(posedge i_clk); @(negedge i_clk); i_rst_n = 1;
        @(posedge i_clk); #0.1;
    endtask

    // =========================================================================
    // Simulate FIFO ack: i_read_enable_sampled is o_read_enable delayed 1 cycle
    // =========================================================================
    always_ff @(posedge i_clk)
        i_read_enable_sampled <= o_read_enable;

    // =========================================================================
    // Test stimulus
    // =========================================================================
    initial begin
        $display("========================================================");
        $display("  tb_tx_fsm_modeling Testbench");
        $display("========================================================");

        // ─────────────────────────────────────────────────────────────────────
        // TEST 1: Reset state
        // ─────────────────────────────────────────────────────────────────────
        $display("\n--- TEST 1: Reset ---");
        reset_dut();
        check("read_enable=0 in reset", o_read_enable, 1'b0);
        check("clk_en=0      in reset", o_clk_en,      1'b0);

        // ─────────────────────────────────────────────────────────────────────
        // TEST 2: Single word (header-only or pattern)
        // ─────────────────────────────────────────────────────────────────────
        $display("\n--- TEST 2: Single word ---");
        reset_dut();

        // Put a word in the FIFO (i_empty=0) → FSM should move to SENDING_PACK
        @(negedge i_clk); i_empty = 0;
        @(posedge i_clk); #0.1;   // IDLE → SENDING_PACK; read_enable fires

        // Allow registered outputs to settle
        @(posedge i_clk); #0.1;
        check("read_enable asserted (pop)", o_read_enable, 1'b1);

        // ack propagates → clk_en
        @(posedge i_clk); #0.1;
        check("clk_en high after ack",      o_clk_en,      1'b1);

        // Signal serialisation done → SLEEPING
        @(negedge i_clk); i_ser_done = 1;
        @(posedge i_clk); #0.1; i_ser_done = 0;
        @(posedge i_clk); #0.1;
        check("clk_en=0 in SLEEPING",       o_clk_en,      1'b0);

        // Dead-time done; FIFO now empty → back to IDLE
        @(negedge i_clk); i_dead_time_done = 1; i_empty = 1;
        @(posedge i_clk); #0.1; i_dead_time_done = 0;
        @(posedge i_clk); #0.1;
        check("read_enable=0 in IDLE",      o_read_enable, 1'b0);
        check("clk_en=0 in IDLE",           o_clk_en,      1'b0);

        // ─────────────────────────────────────────────────────────────────────
        // TEST 3: Two consecutive words (simulates header + data flit)
        //   IDLE → SENDING_PACK (pop 1) → SLEEPING →
        //   SENDING_PACK (pop 2) → SLEEPING → IDLE
        // ─────────────────────────────────────────────────────────────────────
        $display("\n--- TEST 3: Two words (header + data flit) ---");
        reset_dut();

        // Two words available
        @(negedge i_clk); i_empty = 0;

        // First word (header)
        @(posedge i_clk); #0.1;   // IDLE → SENDING_PACK
        @(posedge i_clk); #0.1;
        check("read_enable for word 1",     o_read_enable, 1'b1);
        @(posedge i_clk); #0.1;
        check("clk_en for word 1",          o_clk_en,      1'b1);

        @(negedge i_clk); i_ser_done = 1;
        @(posedge i_clk); #0.1; i_ser_done = 0;
        @(posedge i_clk); #0.1;   // SLEEPING
        check("clk_en=0 after word 1",      o_clk_en,      1'b0);

        // dead_time_done; FIFO still has word 2 → SENDING_PACK again
        @(negedge i_clk); i_dead_time_done = 1;
        @(posedge i_clk); #0.1; i_dead_time_done = 0;
        @(posedge i_clk); #0.1;
        check("read_enable for word 2",     o_read_enable, 1'b1);
        @(posedge i_clk); #0.1;
        check("clk_en for word 2",          o_clk_en,      1'b1);

        // Second word (data flit)
        @(negedge i_clk); i_ser_done = 1;
        @(posedge i_clk); #0.1; i_ser_done = 0;
        @(posedge i_clk); #0.1;   // SLEEPING
        check("clk_en=0 after word 2",      o_clk_en,      1'b0);

        // dead_time_done; FIFO now empty → IDLE
        @(negedge i_clk); i_dead_time_done = 1; i_empty = 1;
        @(posedge i_clk); #0.1; i_dead_time_done = 0;
        @(posedge i_clk); #0.1;
        check("read_enable=0 back in IDLE", o_read_enable, 1'b0);
        check("clk_en=0 back in IDLE",      o_clk_en,      1'b0);

        // ─────────────────────────────────────────────────────────────────────
        // TEST 4: Back-to-back messages (FIFO always non-empty)
        //   SLEEPING → SENDING_PACK without returning to IDLE
        // ─────────────────────────────────────────────────────────────────────
        $display("\n--- TEST 4: Back-to-back messages ---");
        reset_dut();
        i_empty = 0;   // always non-empty

        // First packet
        @(posedge i_clk); #0.1;   // IDLE → SENDING_PACK
        @(posedge i_clk); #0.1;
        @(posedge i_clk); #0.1;   // ack → clk_en

        @(negedge i_clk); i_ser_done = 1;
        @(posedge i_clk); #0.1; i_ser_done = 0;
        @(posedge i_clk); #0.1;   // SLEEPING

        @(negedge i_clk); i_dead_time_done = 1;
        @(posedge i_clk); #0.1; i_dead_time_done = 0;
        @(posedge i_clk); #0.1;   // → SENDING_PACK (back-to-back, not IDLE)
        check("read_enable fires on back-to-back", o_read_enable, 1'b1);

        // ─────────────────────────────────────────────────────────────────────
        // TEST 5: IDLE stays idle when FIFO empty
        // ─────────────────────────────────────────────────────────────────────
        $display("\n--- TEST 5: Idle while FIFO empty ---");
        reset_dut();
        i_empty = 1;
        repeat(10) @(posedge i_clk); #0.1;
        check("read_enable=0 while empty", o_read_enable, 1'b0);
        check("clk_en=0      while empty", o_clk_en,      1'b0);

        // ─────────────────────────────────────────────────────────────────────
        // Summary
        // ─────────────────────────────────────────────────────────────────────
        $display("\n========================================================");
        $display("  RESULTS:  PASS=%0d   FAIL=%0d", pass_cnt, fail_cnt);
        $display("========================================================\n");

        if (fail_cnt == 0)
            $display("ALL TESTS PASSED");
        else
            $display("SOME TESTS FAILED");

        $finish;
    end

    // =========================================================================
    // Timeout watchdog  (prevent infinite hang)
    // =========================================================================
    initial begin
        #200_000;
        $display("WATCHDOG: simulation exceeded limit – aborting");
        $finish;
    end

endmodule : tb_tx_fsm_modeling