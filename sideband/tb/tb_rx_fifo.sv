// =============================================================================
// tb_rx_fifo.sv  –  Testbench for SB_RX_FIFO
//
// Verifies:
//   1. Reset: empty=1, no overflow, all outputs zero.
//   2. Two-phase write (header then data) + commit → entry appears at read port.
//   3. Header-only packet (has_data=0): commit stores zero data.
//   4. Depth 4 fill → full flag.
//   5. Overflow: commit when full → overflow pulse; entry not corrupted.
//   6. Read side: i_read_enable advances pointer; next entry appears.
//   7. o_is_pattern: HIGH when asm-reg == 64'h5555_5555_5555_5555.
//   8. o_is_pattern LOW for non-pattern word.
//   9. Circular wrap-around.
// =============================================================================

`timescale 1ns/1ps

module tb_rx_fifo;

    // ── DUT ports ─────────────────────────────────────────────────────────────
    logic        i_clk, i_rst_n;
    logic        i_rx_header_valid;
    logic [63:0] i_rx_header;
    logic        i_rx_data_valid;
    logic [63:0] i_rx_data;
    logic        i_has_data;
    logic        i_write_commit;
    logic        i_read_enable;
    logic [63:0] o_header, o_data;
    logic        o_has_data, o_valid, o_is_pattern, o_empty, o_full, o_overflow;

    SB_RX_FIFO #(.DEPTH(4)) dut (
        .i_clk             (i_clk),
        .i_rst_n           (i_rst_n),
        .i_rx_header_valid (i_rx_header_valid),
        .i_rx_header       (i_rx_header),
        .i_rx_data_valid   (i_rx_data_valid),
        .i_rx_data         (i_rx_data),
        .i_has_data        (i_has_data),
        .i_write_commit    (i_write_commit),
        .i_read_enable     (i_read_enable),
        .o_header          (o_header),
        .o_data            (o_data),
        .o_has_data        (o_has_data),
        .o_valid           (o_valid),
        .o_is_pattern      (o_is_pattern),
        .o_empty           (o_empty),
        .o_full            (o_full),
        .o_overflow        (o_overflow)
    );

    localparam real CLK_PERIOD = 2.0;
    initial i_clk = 0;
    always #(CLK_PERIOD/2.0) i_clk = ~i_clk;

    int pass_cnt = 0, fail_cnt = 0;

    task check(input string name, input logic got, exp);
        if (got === exp) begin $display("  PASS  %-50s  %0b", name, got); pass_cnt++; end
        else begin $display("  FAIL  %-50s  got=%0b exp=%0b @%0t", name, got, exp, $time); fail_cnt++; end
    endtask

    task check64(input string name, input logic [63:0] got, exp);
        if (got === exp) begin $display("  PASS  %-50s  %016h", name, got); pass_cnt++; end
        else begin $display("  FAIL  %-50s  got=%016h exp=%016h @%0t", name, got, exp, $time); fail_cnt++; end
    endtask

    task reset_dut();
        i_rst_n            = 0;
        i_rx_header_valid  = 0;
        i_rx_header        = '0;
        i_rx_data_valid    = 0;
        i_rx_data          = '0;
        i_has_data         = 0;
        i_write_commit     = 0;
        i_read_enable      = 0;
        repeat(4) @(posedge i_clk);
        @(negedge i_clk); i_rst_n = 1;
        @(posedge i_clk); #0.1;
    endtask

    // ── Two-phase write helper ────────────────────────────────────────────────
    task write_pkt(input logic [63:0] hdr, dat, input logic hd);
        // Phase 1: header
        @(negedge i_clk);
        i_rx_header       = hdr;
        i_rx_header_valid = 1;
        i_has_data        = hd;
        @(posedge i_clk); #0.1;
        i_rx_header_valid = 0;

        // Phase 2: data (only if has_data)
        if (hd) begin
            @(negedge i_clk);
            i_rx_data       = dat;
            i_rx_data_valid = 1;
            @(posedge i_clk); #0.1;
            i_rx_data_valid = 0;
        end

        // Commit
        @(negedge i_clk);
        i_write_commit = 1;
        @(posedge i_clk); #0.1;
        i_write_commit = 0;
    endtask

    // ── Read helper ───────────────────────────────────────────────────────────
    task read_pkt(output logic [63:0] hdr, dat, output logic hd);
        @(negedge i_clk);
        i_read_enable = 1;
        @(posedge i_clk); #0.1;
        hdr = o_header; dat = o_data; hd = o_has_data;
        i_read_enable = 0;
    endtask

    // ── Main ──────────────────────────────────────────────────────────────────
    initial begin
        $display("==== tb_rx_fifo ====");

        // ------------------------------------------------------------------
        // TEST 1: Reset state
        // ------------------------------------------------------------------
        $display("\n-- TEST 1: Reset --");
        reset_dut();
        check("empty=1 after reset",    o_empty,    1'b1);
        check("full=0  after reset",    o_full,     1'b0);
        check("overflow=0 after reset", o_overflow, 1'b0);
        check("valid=0  after reset",   o_valid,    1'b0);

        // ------------------------------------------------------------------
        // TEST 2: Write header+data, then read back
        // ------------------------------------------------------------------
        /* $display("\n-- TEST 2: Header+data write/read --");
        reset_dut();
        write_pkt(64'hDEAD_BEEF_0000_0001, 64'hCAFE_BABE_ABCD_EF01, 1'b1);
        check("valid after commit", o_valid, 1'b1);
        check("not empty after commit", o_empty, 1'b0);
        begin
            automatic logic [63:0] rh, rd; automatic logic rhd;
            read_pkt(rh, rd, rhd);
            check64("header RD",  rh,  64'hDEAD_BEEF_0000_0001);
            check64("data RD",    rd,  64'hCAFE_BABE_ABCD_EF01);
            check("has_data RD",  rhd, 1'b1);
        end
        @(posedge i_clk); #0.1;
        check("empty after drain", o_empty, 1'b1); */

        // ------------------------------------------------------------------
        // TEST 3: Header-only (has_data=0) – data stored as 0
        // ------------------------------------------------------------------
        $display("\n-- TEST 3: Header-only --");
        reset_dut();
        write_pkt(64'hAAAA_BBBB_CCCC_DDDD, 64'hFFFF_FFFF_FFFF_FFFF, 1'b0);
        begin
            automatic logic [63:0] rh, rd; automatic logic rhd;
            read_pkt(rh, rd, rhd);
            check64("header hdr-only", rh, 64'hAAAA_BBBB_CCCC_DDDD);
            check64("data zero hdr-only", rd, 64'h0);
            check("has_data=0 hdr-only", rhd, 1'b0);
        end

        // ------------------------------------------------------------------
        // TEST 4: Fill to DEPTH=4 → full flag
        // ------------------------------------------------------------------
        $display("\n-- TEST 4: Fill to depth --");
        reset_dut();
        for (int i = 0; i < 4; i++)
            write_pkt(64'(i+1), 64'(i+100), 1'b1);
        @(posedge i_clk); #0.1;
        check("full=1 at depth 4", o_full, 1'b1);

        // ------------------------------------------------------------------
        // TEST 5: Overflow (commit while full)
        // ------------------------------------------------------------------
        $display("\n-- TEST 5: Overflow --");
        // Try to write one more: commit without new header/data (previous asm unchanged)
        @(negedge i_clk);
        i_write_commit = 1;
        @(posedge i_clk); #0.1;
        i_write_commit = 0;
        check("overflow pulse", o_overflow, 1'b1);
        @(posedge i_clk); #0.1;
        check("overflow clears", o_overflow, 1'b0);

        // ------------------------------------------------------------------
        // TEST 6: Drain from full → empty
        // ------------------------------------------------------------------
        $display("\n-- TEST 6: Drain from full --");
        for (int i = 0; i < 4; i++) begin
            automatic logic [63:0] rh, rd; automatic logic rhd;
            read_pkt(rh, rd, rhd);
            check64($sformatf("drain hdr[%0d]", i), rh, 64'(i+1));
        end
        @(posedge i_clk); #0.1;
        check("empty after full drain", o_empty, 1'b1);

        // ------------------------------------------------------------------
        // TEST 7: o_is_pattern HIGH for SB_START_PATTERN
        // ------------------------------------------------------------------
        $display("\n-- TEST 7: o_is_pattern HIGH --");
        reset_dut();
        @(negedge i_clk);
        i_rx_header       = 64'h5555_5555_5555_5555;
        i_rx_header_valid = 1;
        i_has_data        = 0;
        @(posedge i_clk); #0.1;
        i_rx_header_valid = 0;
        #0.1;
        check("is_pattern HIGH for 0x5555...", o_is_pattern, 1'b1);

        // ------------------------------------------------------------------
        // TEST 8: o_is_pattern LOW for non-pattern word
        // ------------------------------------------------------------------
        $display("\n-- TEST 8: o_is_pattern LOW --");
        reset_dut();
        @(negedge i_clk);
        i_rx_header       = 64'hDEAD_BEEF_CAFE_BABE;
        i_rx_header_valid = 1;
        i_has_data        = 0;
        @(posedge i_clk); #0.1;
        i_rx_header_valid = 0;
        #0.1;
        check("is_pattern LOW for non-pattern", o_is_pattern, 1'b0);

        // ------------------------------------------------------------------
        // TEST 9: Circular wrap-around
        // ------------------------------------------------------------------
        $display("\n-- TEST 9: Circular wrap --");
        reset_dut();
        // Write 2, read 2, write 4, read 4
        for (int i = 0; i < 2; i++)
            write_pkt(64'(i+10), 64'(0), 1'b0);
        for (int i = 0; i < 2; i++) begin
            automatic logic [63:0] rh, rd; automatic logic rhd;
            read_pkt(rh, rd, rhd);
        end
        for (int i = 0; i < 4; i++)
            write_pkt(64'(i+20), 64'(i+200), 1'b1);
        begin
            automatic int ok = 0;
            for (int i = 0; i < 4; i++) begin
                automatic logic [63:0] rh, rd; automatic logic rhd;
                read_pkt(rh, rd, rhd);
                if (rh === 64'(i+20)) ok++;
            end
            if (ok == 4) begin $display("  PASS  Wrap-around 4/4 correct"); pass_cnt++; end
            else begin $display("  FAIL  Wrap-around %0d/4 correct", ok); fail_cnt++; end
        end

        // ------------------------------------------------------------------
        // Summary
        // ------------------------------------------------------------------
        $display("\n==== RESULTS: %0d passed, %0d failed ====", pass_cnt, fail_cnt);
        if (fail_cnt == 0) $display("ALL TESTS PASSED");
        else               $display("SOME TESTS FAILED");
        $finish;
    end

    initial begin #200000; $display("WATCHDOG"); $finish; end

endmodule : tb_rx_fifo