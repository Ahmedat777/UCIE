// =============================================================================
// tb_clk_control.sv  –  Testbench for SB_CLOCK_CONTROLLER
//
// Tests:
//   TC1 – Reset behaviour: all outputs deasserted after reset
//   TC2 – Single packet: assert i_enable for 1 cycle, observe
//          TXCKSB active for 64 UI then gated, o_ser_done at cycle 63→64,
//          o_dead_time_done at cycle 95
//   TC3 – Enable held low: no activity (idle check)
// =============================================================================

`timescale 1ns/1ps

module tb_clk_control;

    // -------------------------------------------------------------------------
    // DUT signals
    // -------------------------------------------------------------------------
    logic       i_pll_clk;
    logic       i_rst_n;
    logic       i_enable;

    logic       o_dead_time_done;
    logic       o_ser_done;
    logic       TXCKSB;

    // -------------------------------------------------------------------------
    // Instantiation
    // -------------------------------------------------------------------------
    SB_CLOCK_CONTROLLER dut (
        .i_pll_clk       (i_pll_clk),
        .i_rst_n         (i_rst_n),
        .i_enable        (i_enable),
        .o_dead_time_done(o_dead_time_done),
        .o_ser_done      (o_ser_done),
        .TXCKSB          (TXCKSB)
    );

    // -------------------------------------------------------------------------
    // Clock: 800 MHz → period = 1.25 ns
    // -------------------------------------------------------------------------
    initial i_pll_clk = 0;
    always #0.625 i_pll_clk = ~i_pll_clk;

    // -------------------------------------------------------------------------
    // Helpers
    // -------------------------------------------------------------------------
    int ser_done_count;
    int dead_time_count;
    int txcksb_high_count;

    task apply_reset();
        i_rst_n  = 0;
        i_enable = 0;
        repeat(4) @(posedge i_pll_clk);
        @(negedge i_pll_clk);
        i_rst_n = 1;
        @(posedge i_pll_clk);
    endtask

    task pulse_enable();
        @(negedge i_pll_clk);
        i_enable = 1;
        @(posedge i_pll_clk);
        @(negedge i_pll_clk);
        i_enable = 0;
    endtask

    // -------------------------------------------------------------------------
    // TC1 – Reset check
    // -------------------------------------------------------------------------
    task tc1_reset();
        $display("\n[TC1] Reset behaviour");
        i_rst_n  = 0;
        i_enable = 0;
        repeat(6) @(posedge i_pll_clk);
        if (o_dead_time_done === 0 && o_ser_done === 0 && TXCKSB === 0)
            $display("  PASS: all outputs deasserted during reset");
        else
            $display("  FAIL: unexpected output during reset  dtd=%0b ser=%0b TXCKSB=%0b",
                     o_dead_time_done, o_ser_done, TXCKSB);
        @(negedge i_pll_clk);
        i_rst_n = 1;
        @(posedge i_pll_clk);
    endtask

    // -------------------------------------------------------------------------
    // TC2 – Single packet: count TXCKSB cycles, check ser_done and dead_time_done
    // -------------------------------------------------------------------------
    task tc2_single_packet();
        $display("\n[TC2] Single packet (96-cycle frame)");
        ser_done_count   = 0;
        dead_time_count  = 0;
        txcksb_high_count= 0;

        pulse_enable();

        // Monitor 100 cycles
        repeat(100) begin
            @(posedge i_pll_clk);
            if (TXCKSB)          txcksb_high_count++;
            if (o_ser_done)      ser_done_count++;
            if (o_dead_time_done) dead_time_count++;
        end

        $display("  TXCKSB high cycles    = %0d (expect ~64)", txcksb_high_count);
        $display("  o_ser_done pulses     = %0d (expect  1)",  ser_done_count);
        $display("  o_dead_time_done pulses = %0d (expect 1)", dead_time_count);

        if (ser_done_count == 1)
            $display("  PASS: o_ser_done fired once");
        else
            $display("  FAIL: o_ser_done count = %0d", ser_done_count);

        if (dead_time_count == 1)
            $display("  PASS: o_dead_time_done fired once");
        else
            $display("  FAIL: o_dead_time_done count = %0d", dead_time_count);

        if (txcksb_high_count >= 60 && txcksb_high_count <= 68)
            $display("  PASS: TXCKSB active ~64 cycles");
        else
            $display("  FAIL: TXCKSB active %0d cycles", txcksb_high_count);
    endtask

    // -------------------------------------------------------------------------
    // TC3 – Idle: no enable, TXCKSB must stay low
    // -------------------------------------------------------------------------
    task tc3_idle();
        $display("\n[TC3] Idle – no enable");
        i_enable = 0;
        txcksb_high_count = 0;
        repeat(20) begin
            @(posedge i_pll_clk);
            if (TXCKSB) txcksb_high_count++;
        end
        if (txcksb_high_count == 0)
            $display("  PASS: TXCKSB stayed low during idle");
        else
            $display("  FAIL: TXCKSB went high %0d times while idle", txcksb_high_count);
    endtask

    // -------------------------------------------------------------------------
    // Main
    // -------------------------------------------------------------------------
    initial begin
        $display("===== TB_CLK_CONTROL =====");
        apply_reset();
        tc1_reset();
        apply_reset();
        tc2_single_packet();
        apply_reset();
        tc3_idle();
        $display("\n===== DONE =====");
        $finish;
    end

endmodule : tb_clk_control