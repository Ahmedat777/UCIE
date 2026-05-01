// =============================================================================
// tb_tx_fsm_modeling.sv  –  Testbench for SB_TX_FSM_Modelling
//
// Verifies:
//   1. Reset: IDLE, all outputs deasserted.
//   2. Header-only packet:
//        IDLE → SENDING_PACK → SLEEPING → IDLE
//        o_read_enable fires on entry to SENDING_PACK.
//        o_clk_en active during SENDING_PACK once read is acked.
//        o_read_data_enable never asserted.
//   3. Message with data flit (has_data=1):
//        IDLE → SENDING_PACK (header) → SLEEPING →
//        SENDING_PACK (data) → SLEEPING → IDLE
//        o_read_enable for header, o_read_data_enable for data flit.
//   4. SLEEPING returns to IDLE when FIFO empty after header-only.
//   5. Back-to-back messages: SLEEPING → SENDING_PACK without returning to IDLE.
//   6. o_clk_en deasserts after dead_time_done.
// =============================================================================

`timescale 1ns/1ps

module tb_tx_fsm_modeling;

    // ── DUT ports ─────────────────────────────────────────────────────────────
    logic i_clk, i_rst_n;
    logic i_ser_done;
    logic i_empty;
    logic i_dead_time_done;
    logic i_read_enable_sampled;
    logic i_has_data;
    logic o_read_enable;
    logic o_read_data_enable;
    logic o_clk_en;

    SB_TX_FSM_Modelling dut (
        .i_clk               (i_clk),
        .i_rst_n             (i_rst_n),
        .i_ser_done          (i_ser_done),
        .i_empty             (i_empty),
        .i_dead_time_done    (i_dead_time_done),
        .i_read_enable_sampled(i_read_enable_sampled),
        .i_has_data          (i_has_data),
        .o_read_enable       (o_read_enable),
        .o_read_data_enable  (o_read_data_enable),
        .o_clk_en            (o_clk_en)
    );

    localparam real CLK_PERIOD = 2.0;
    initial i_clk = 0;
    always #(CLK_PERIOD/2.0) i_clk = ~i_clk;

    int pass_cnt = 0, fail_cnt = 0;

    task check(input string name, input logic got, exp);
        if (got === exp) begin $display("  PASS  %-55s  %0b", name, got); pass_cnt++; end
        else begin $display("  FAIL  %-55s  got=%0b exp=%0b @%0t", name, got, exp, $time); fail_cnt++; end
    endtask

    task reset_dut();
        i_rst_n              = 0;
        i_ser_done           = 0;
        i_empty              = 1;
        i_dead_time_done     = 0;
        i_read_enable_sampled= 0;
        i_has_data           = 0;
        repeat(4) @(posedge i_clk); @(negedge i_clk); i_rst_n = 1;
        @(posedge i_clk); #0.1;
    endtask

    // ── Simulate FIFO ack (1-cycle delayed read_enable_sampled) ───────────────
    always_ff @(posedge i_clk) begin
        i_read_enable_sampled <= o_read_enable | o_read_data_enable;
    end

    // ── Main ──────────────────────────────────────────────────────────────────
    initial begin
        $display("==== tb_tx_fsm_modeling ====");

        // ------------------------------------------------------------------
        // TEST 1: Reset state
        // ------------------------------------------------------------------
        $display("\n-- TEST 1: Reset --");
        reset_dut();
        check("read_enable=0      in reset", o_read_enable,      1'b0);
        check("read_data_enable=0 in reset", o_read_data_enable, 1'b0);
        check("clk_en=0           in reset", o_clk_en,           1'b0);

        // ------------------------------------------------------------------
        // TEST 2: Header-only packet (has_data=0)
        // ------------------------------------------------------------------
        $display("\n-- TEST 2: Header-only packet --");
        reset_dut();
        i_has_data = 0;
        // Put something in FIFO (i_empty=0)
        @(negedge i_clk); i_empty = 0;
        @(posedge i_clk); #0.1;   // IDLE → SENDING_PACK; read_enable fires

        // Allow 2 cycles for registered outputs
        @(posedge i_clk); #0.1;
        check("read_enable asserted (header pop)", o_read_enable, 1'b1);

        // Let ack propagate (i_read_enable_sampled goes high → clk_en)
        @(posedge i_clk); #0.1;
        check("clk_en high after ack",             o_clk_en,     1'b1);
        check("read_data_enable=0 (no data)",      o_read_data_enable, 1'b0);

        // Signal serialisation done → SLEEPING
        @(negedge i_clk); i_ser_done = 1;
        @(posedge i_clk); #0.1; i_ser_done = 0;
        @(posedge i_clk); #0.1;
        check("clk_en=0 in SLEEPING", o_clk_en, 1'b0);

        // Dead-time done; FIFO empty → IDLE
        @(negedge i_clk); i_dead_time_done = 1; i_empty = 1;
        @(posedge i_clk); #0.1; i_dead_time_done = 0;
        @(posedge i_clk); #0.1;
        check("read_enable=0 back in IDLE",      o_read_enable,      1'b0);
        check("clk_en=0 back in IDLE",           o_clk_en,           1'b0);
        check("read_data_enable=0 back in IDLE", o_read_data_enable, 1'b0);

        // ------------------------------------------------------------------
        // TEST 3: Message with data flit (has_data=1)
        // ------------------------------------------------------------------
        $display("\n-- TEST 3: Header + data flit --");
        reset_dut();
        i_has_data = 1;

        // Start packet
        @(negedge i_clk); i_empty = 0;
        @(posedge i_clk); #0.1;
        @(posedge i_clk); #0.1;
        check("read_enable (header)",         o_read_enable,      1'b1);
        check("no read_data_enable (header)", o_read_data_enable, 1'b0);

        // ser_done → SLEEPING (after header)
        @(posedge i_clk); #0.1;
        @(negedge i_clk); i_ser_done = 1;
        @(posedge i_clk); #0.1; i_ser_done = 0;
        @(posedge i_clk); #0.1;

        // dead_time_done → SENDING_PACK (data flit phase)
        @(negedge i_clk); i_dead_time_done = 1;
        @(posedge i_clk); #0.1; i_dead_time_done = 0;
        @(posedge i_clk); #0.1;
        check("read_data_enable (data flit)",   o_read_data_enable, 1'b1);
        check("read_enable=0 in data phase",    o_read_enable,      1'b0);

        // ser_done → SLEEPING (after data)
        @(posedge i_clk); #0.1;
        @(negedge i_clk); i_ser_done = 1;
        @(posedge i_clk); #0.1; i_ser_done = 0;
        @(posedge i_clk); #0.1;

        // dead_time_done → IDLE (data done, FIFO empty)
        @(negedge i_clk); i_dead_time_done = 1; i_empty = 1;
        @(posedge i_clk); #0.1; i_dead_time_done = 0;
        @(posedge i_clk); #0.1;
        check("all outputs low after data msg", o_read_enable,      1'b0);
        check("clk_en=0 after data msg",        o_clk_en,           1'b0);
        check("read_data_enable=0 after msg",   o_read_data_enable, 1'b0);

        // ------------------------------------------------------------------
        // TEST 4: Back-to-back messages (FIFO non-empty after dead-time)
        // ------------------------------------------------------------------
        $display("\n-- TEST 4: Back-to-back (no-data) --");
        reset_dut();
        i_has_data = 0;
        i_empty    = 0;   // always non-empty

        // First packet
        @(posedge i_clk); #0.1;   // IDLE → SENDING_PACK
        @(posedge i_clk); #0.1;
        @(posedge i_clk); #0.1;   // ack → clk_en

        @(negedge i_clk); i_ser_done = 1;
        @(posedge i_clk); #0.1; i_ser_done = 0;
        @(posedge i_clk); #0.1;   // SLEEPING

        @(negedge i_clk); i_dead_time_done = 1;
        @(posedge i_clk); #0.1; i_dead_time_done = 0;
        @(posedge i_clk); #0.1;   // → SENDING_PACK again (back-to-back)
        check("read_enable fires on back-to-back", o_read_enable, 1'b1);

        // ------------------------------------------------------------------
        // TEST 5: IDLE stays idle when empty
        // ------------------------------------------------------------------
        $display("\n-- TEST 5: Idle when FIFO empty --");
        reset_dut();
        i_empty = 1;
        repeat(10) @(posedge i_clk); #0.1;
        check("read_enable=0 while empty",      o_read_enable,      1'b0);
        check("clk_en=0      while empty",      o_clk_en,           1'b0);
        check("read_data=0   while empty",      o_read_data_enable, 1'b0);

        // ------------------------------------------------------------------
        // Summary
        // ------------------------------------------------------------------
        $display("\n==== RESULTS: %0d passed, %0d failed ====", pass_cnt, fail_cnt);
        if (fail_cnt == 0) $display("ALL TESTS PASSED");
        else               $display("SOME TESTS FAILED");
        $finish;
    end

    initial begin #200000; $display("WATCHDOG"); $finish; end

endmodule : tb_tx_fsm_modeling