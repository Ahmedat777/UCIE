`timescale 1ns/1ps

module tb_tx_fifo;

    // =========================================================================
    // DUT ports
    // =========================================================================
    logic        i_clk;
    logic        i_rst_n;
    logic        i_write_enable;
    logic [63:0] i_packet;
    logic        i_read_enable;
    logic [63:0] o_packet;
    logic        o_empty;
    logic        o_full;
    logic        o_ser_done_sampled;

    // =========================================================================
    // DUT instantiation
    // =========================================================================
    SB_TX_FIFO #(.DEPTH(4)) dut (
        .i_clk             (i_clk),
        .i_rst_n           (i_rst_n),
        .i_write_enable    (i_write_enable),
        .i_packet          (i_packet),
        .i_read_enable     (i_read_enable),
        .o_packet          (o_packet),
        .o_empty           (o_empty),
        .o_full            (o_full),
        .o_ser_done_sampled(o_ser_done_sampled)
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
    int pass_cnt = 0;
    int fail_cnt = 0;

    // =========================================================================
    // Helper: check 1-bit signal
    // =========================================================================
    task check1(input string name, input logic got, input logic exp);
        if (got === exp) begin
            $display("  PASS  %-55s  %0b", name, got);
            pass_cnt++;
        end else begin
            $display("  FAIL  %-55s  got=%0b exp=%0b @%0t", name, got, exp, $time);
            fail_cnt++;
        end
    endtask

    // =========================================================================
    // Helper: check 64-bit signal
    // =========================================================================
    task check64(input string name, input logic [63:0] got, input logic [63:0] exp);
        if (got === exp) begin
            $display("  PASS  %-55s  %016h", name, got);
            pass_cnt++;
        end else begin
            $display("  FAIL  %-55s  got=%016h exp=%016h @%0t", name, got, exp, $time);
            fail_cnt++;
        end
    endtask

    // =========================================================================
    // Helper: reset DUT
    // =========================================================================
    task reset_dut();
        i_rst_n       = 0;
        i_write_enable= 0;
        i_read_enable = 0;
        i_packet      = '0;
        repeat(4) @(posedge i_clk);
        @(negedge i_clk);
        i_rst_n = 1;
        @(posedge i_clk); #0.1;
    endtask

    // =========================================================================
    // Helper: write one 64-bit word
    // =========================================================================
    task write_word(input logic [63:0] pkt);
        @(negedge i_clk);
        i_packet       = pkt;
        i_write_enable = 1;
        @(posedge i_clk); #0.1;
        i_write_enable = 0;
    endtask

    // =========================================================================
    // Helper: read one 64-bit word
    // =========================================================================
    task read_word(output logic [63:0] pkt);
        @(negedge i_clk);
        i_read_enable = 1;
        @(posedge i_clk); #0.1;
        pkt = o_packet;
        i_read_enable = 0;
    endtask

    // =========================================================================
    // Test stimulus
    // =========================================================================
    initial begin
        $display("========================================================");
        $display("  tb_tx_fifo Testbench");
        $display("========================================================");

        // ─────────────────────────────────────────────────────────────────────
        // TEST 1: Reset state
        // ─────────────────────────────────────────────────────────────────────
        $display("\n--- TEST 1: Reset state ---");
        reset_dut();
        check1 ("empty=1 after reset",  o_empty,  1'b1);
        check1 ("full=0  after reset",  o_full,   1'b0);
        check64("packet=0 after reset", o_packet, 64'h0);

        // ─────────────────────────────────────────────────────────────────────
        // TEST 2: Single write / read – data integrity
        // ─────────────────────────────────────────────────────────────────────
        $display("\n--- TEST 2: Single write/read ---");
        reset_dut();
        write_word(64'hDEAD_BEEF_CAFE_BABE);
        check1("not empty after write", o_empty, 1'b0);
        begin
            automatic logic [63:0] got;
            read_word(got);
            check64("packet integrity", got, 64'hDEAD_BEEF_CAFE_BABE);
        end
        @(posedge i_clk); #0.1;
        check1("empty=1 after drain", o_empty, 1'b1);

        // ─────────────────────────────────────────────────────────────────────
        // TEST 3: Two entries (header flit then data flit) – correct order
        // ─────────────────────────────────────────────────────────────────────
        $display("\n--- TEST 3: Two-entry sequence (header then data flit) ---");
        reset_dut();
        write_word(64'hAAAA_1111_BBBB_2222);   // header flit
        write_word(64'hCCCC_3333_DDDD_4444);   // data  flit
        begin
            automatic logic [63:0] g0, g1;
            read_word(g0);
            read_word(g1);
            check64("entry 0 = header flit", g0, 64'hAAAA_1111_BBBB_2222);
            check64("entry 1 = data  flit",  g1, 64'hCCCC_3333_DDDD_4444);
        end
        @(posedge i_clk); #0.1;
        check1("empty after two-entry drain", o_empty, 1'b1);

        // ─────────────────────────────────────────────────────────────────────
        // TEST 4: Fill to depth 4 → full flag
        // ─────────────────────────────────────────────────────────────────────
        $display("\n--- TEST 4: Fill to DEPTH=4 ---");
        reset_dut();
        for (int i = 0; i < 4; i++)
            write_word(64'(i + 1));
        @(posedge i_clk); #0.1;
        check1("full=1 at depth 4", o_full,  1'b1);
        check1("empty=0 when full", o_empty, 1'b0);

        // ─────────────────────────────────────────────────────────────────────
        // TEST 5: Drain from full → empty, correct order
        // ─────────────────────────────────────────────────────────────────────
        $display("\n--- TEST 5: Drain from full ---");
        // FIFO still holds 4 entries (values 1..4) from TEST 4
        begin
            automatic int ok = 0;
            for (int i = 0; i < 4; i++) begin
                automatic logic [63:0] got;
                read_word(got);
                if (got === 64'(i + 1)) ok++;
                else $display("  FAIL  pop %0d: got=%016h exp=%016h", i, got, 64'(i+1));
            end
            if (ok == 4) begin $display("  PASS  All 4 pops correct");    pass_cnt++; end
            else         begin $display("  FAIL  %0d/4 pops wrong", 4-ok); fail_cnt++; end
        end
        @(posedge i_clk); #0.1;
        check1("empty=1 after full drain", o_empty, 1'b1);

        // ─────────────────────────────────────────────────────────────────────
        // TEST 6: Write ignored when full (back-pressure)
        // ─────────────────────────────────────────────────────────────────────
        $display("\n--- TEST 6: Write ignored when full ---");
        reset_dut();
        for (int i = 0; i < 4; i++) write_word(64'(i + 10));
        // Attempt one more write — should be silently dropped
        write_word(64'hFFFF_FFFF_FFFF_FFFF);
        check1("still full after excess write", o_full, 1'b1);
        // Drain and verify the extra word is not present
        begin
            automatic int ok = 0;
            for (int i = 0; i < 4; i++) begin
                automatic logic [63:0] got;
                read_word(got);
                if (got === 64'(i + 10)) ok++;
            end
            @(posedge i_clk); #0.1;
            check1("empty after drain (no phantom entry)", o_empty, 1'b1);
            if (ok == 4) begin $display("  PASS  Excess write correctly dropped"); pass_cnt++; end
            else         begin $display("  FAIL  Excess write not dropped");        fail_cnt++; end
        end

        // ─────────────────────────────────────────────────────────────────────
        // TEST 7: Circular wrap-around
        //   Write 2, read 2, write 4, read 4 – forces pointer wrap
        // ─────────────────────────────────────────────────────────────────────
        $display("\n--- TEST 7: Circular wrap-around ---");
        reset_dut();
        for (int i = 0; i < 2; i++) write_word(64'(i + 200));
        begin automatic logic [63:0] g; repeat(2) read_word(g); end
        for (int i = 0; i < 4; i++) write_word(64'(i + 400));
        begin
            automatic int ok = 0;
            for (int i = 0; i < 4; i++) begin
                automatic logic [63:0] got;
                read_word(got);
                if (got === 64'(i + 400)) ok++;
            end
            if (ok == 4) begin $display("  PASS  Wrap-around all 4 correct"); pass_cnt++; end
            else         begin $display("  FAIL  Wrap-around %0d/4 correct", ok); fail_cnt++; end
        end

        // ─────────────────────────────────────────────────────────────────────
        // TEST 8: o_ser_done_sampled is registered i_read_enable
        // ─────────────────────────────────────────────────────────────────────
        $display("\n--- TEST 8: ser_done_sampled ---");
        reset_dut();
        write_word(64'hABCD_EF01_2345_6789);
        @(negedge i_clk);
        i_read_enable = 1;
        @(posedge i_clk); #0.1;
        i_read_enable = 0;
        check1("ser_done_sampled high after read",   o_ser_done_sampled, 1'b1);
        @(posedge i_clk); #0.1;
        check1("ser_done_sampled clears next cycle", o_ser_done_sampled, 1'b0);

        // ─────────────────────────────────────────────────────────────────────
        // TEST 9: Simultaneous write + read – net count stays constant
        // ─────────────────────────────────────────────────────────────────────
        $display("\n--- TEST 9: Simultaneous write + read ---");
        reset_dut();
        write_word(64'h0000_0001);   // pre-load 1 entry
        @(negedge i_clk);
        i_packet       = 64'h0000_0002;
        i_write_enable = 1;
        i_read_enable  = 1;
        @(posedge i_clk); #0.1;
        i_write_enable = 0;
        i_read_enable  = 0;
        // After simultaneous wr+rd, FIFO should still hold 1 entry (the new write)
        check1("not empty after sim wr+rd", o_empty, 1'b0);
        begin
            automatic logic [63:0] got;
            read_word(got);
            check64("new entry = 0x2", got, 64'h0000_0002);
        end

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

endmodule : tb_tx_fifo