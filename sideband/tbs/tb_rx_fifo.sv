`timescale 1ns/1ps

module tb_rx_fifo;

    // =========================================================================
    // Opcode constants (must match DUT)
    // =========================================================================
    localparam logic [4:0] SB_OPCODE_NO_DATA = 5'b10010;
    localparam logic [4:0] SB_OPCODE_DATA    = 5'b11011;

    // =========================================================================
    // DUT ports
    // =========================================================================
    logic        i_clk, i_rst_n;
    logic        i_write_enable;
    logic [63:0] i_rx_word;
    logic        i_write_commit;
    logic        i_read_enable;
    logic [63:0] o_header, o_data;
    logic        o_has_data, o_valid, o_is_pattern, o_empty, o_full;

    // =========================================================================
    // DUT instantiation
    // =========================================================================
    SB_RX_FIFO #(.DEPTH(4)) dut (
        .i_clk          (i_clk),
        .i_rst_n        (i_rst_n),
        .i_write_enable (i_write_enable),
        .i_rx_word      (i_rx_word),
        .i_write_commit (i_write_commit),
        .i_read_enable  (i_read_enable),
        .o_header       (o_header),
        .o_data         (o_data),
        .o_has_data     (o_has_data),
        .o_valid        (o_valid),
        .o_is_pattern   (o_is_pattern),
        .o_empty        (o_empty),
        .o_full         (o_full)
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
        if (got === exp)
            $display("  PASS  %-55s  %0b", name, got);
        else
            $display("  FAIL  %-55s  got=%0b exp=%0b @%0t", name, got, exp, $time);
        if (got === exp) pass_cnt++; else fail_cnt++;
    endtask

    // =========================================================================
    // Helper: check 64-bit signal
    // =========================================================================
    task check64(input string name, input logic [63:0] got, exp);
        if (got === exp)
            $display("  PASS  %-55s  %016h", name, got);
        else
            $display("  FAIL  %-55s  got=%016h exp=%016h @%0t", name, got, exp, $time);
        if (got === exp) pass_cnt++; else fail_cnt++;
    endtask

    // =========================================================================
    // Helper: reset DUT
    // =========================================================================
    task reset_dut();
        i_rst_n        = 0;
        i_write_enable = 0;
        i_rx_word      = '0;
        i_write_commit = 0;
        i_read_enable  = 0;
        repeat(4) @(posedge i_clk);
        @(negedge i_clk); i_rst_n = 1;
        @(posedge i_clk); #0.1;
    endtask

    // =========================================================================
    // Helper: write packet (header + optional data flit, then commit)
    // =========================================================================
    task write_pkt(input logic [63:0] hdr, dat, input logic hd);
        // Phase 1: header flit – opcode embedded in hdr[4:0]
        @(negedge i_clk);
        i_rx_word      = hdr;
        i_write_enable = 1;
        @(posedge i_clk); #0.1;
        i_write_enable = 0;

        // Phase 2: data flit – only when opcode == SB_OPCODE_DATA
        if (hd) begin
            @(negedge i_clk);
            i_rx_word      = dat;
            i_write_enable = 1;
            @(posedge i_clk); #0.1;
            i_write_enable = 0;
        end

        // Commit
        @(negedge i_clk);
        i_write_commit = 1;
        @(posedge i_clk); #0.1;
        i_write_commit = 0;
    endtask

    // =========================================================================
    // Helper: read one packet
    // =========================================================================
    task read_pkt(output logic [63:0] hdr, dat, output logic hd);
        @(negedge i_clk);
        i_read_enable = 1;
        @(posedge i_clk); #0.1;
        hdr = o_header; dat = o_data; hd = o_has_data;
        i_read_enable = 0;
    endtask

    // =========================================================================
    // Helper: build header word with opcode in [4:0]
    // Upper 59 bits carry the user payload; lower 5 bits = opcode.
    // =========================================================================
    function automatic logic [63:0] make_hdr(input logic [58:0] payload,
                                             input logic         has_data);
        return {payload, has_data ? SB_OPCODE_DATA : SB_OPCODE_NO_DATA};
    endfunction

    // =========================================================================
    // Test stimulus
    // =========================================================================
    initial begin
        $display("========================================================");
        $display("  tb_rx_fifo Testbench");
        $display("========================================================");

        // ─────────────────────────────────────────────────────────────────────
        // TEST 1: Reset state
        // ─────────────────────────────────────────────────────────────────────
        $display("\n--- TEST 1: Reset state ---");
        reset_dut();
        check("empty=1    after reset",   o_empty,    1'b1);
        check("full=0     after reset",   o_full,     1'b0);
        check("valid=0    after reset",   o_valid,    1'b0);

        // ─────────────────────────────────────────────────────────────────────
        // TEST 2: DATA opcode – header + data flit, then read back
        // ─────────────────────────────────────────────────────────────────────
        $display("\n--- TEST 2: DATA opcode write/read ---");
        reset_dut();
        // Header payload 59'h1, opcode = SB_OPCODE_DATA → hdr[4:0] = 5'b11011
        begin
            automatic logic [63:0] hdr = make_hdr(59'h1, 1'b1);
            automatic logic [63:0] dat = 64'hCAFE_BABE_ABCD_EF01;
            write_pkt(hdr, dat, 1'b1);
            @(posedge i_clk); #1;
            check("valid after DATA commit",     o_valid, 1'b1);
            check("not empty after DATA commit", o_empty, 1'b0);
            begin
                automatic logic [63:0] rh, rd; automatic logic rhd;
                read_pkt(rh, rd, rhd);
                check64("header read back",   rh,  hdr);
                check64("data   read back",   rd,  dat);
                check("has_data=1 read back", rhd, 1'b1);
            end
            @(posedge i_clk); #0.1;
            check("empty after drain", o_empty, 1'b1);
        end

        // ─────────────────────────────────────────────────────────────────────
        // TEST 3: NO_DATA opcode – header only, data stored as zero
        // ─────────────────────────────────────────────────────────────────────
        $display("\n--- TEST 3: NO_DATA opcode (header-only) ---");
        reset_dut();
        begin
            automatic logic [63:0] hdr = make_hdr(59'h2, 1'b0);  // opcode = SB_OPCODE_NO_DATA
            write_pkt(hdr, 64'hFFFF_FFFF_FFFF_FFFF, 1'b0);        // dat ignored by DUT
            begin
                automatic logic [63:0] rh, rd; automatic logic rhd;
                read_pkt(rh, rd, rhd);
                check64("header no-data",        rh,  hdr);
                check64("data zero for no-data", rd,  64'h0);
                check("has_data=0 no-data",      rhd, 1'b0);
            end
        end

        // ─────────────────────────────────────────────────────────────────────
        // TEST 4: Fill to DEPTH=4 → full flag
        // ─────────────────────────────────────────────────────────────────────
        $display("\n--- TEST 4: Fill to depth ---");
        reset_dut();
        for (int i = 0; i < 4; i++)
            write_pkt(make_hdr(59'(i+1), 1'b1), 64'(i+100), 1'b1);
        @(posedge i_clk); #0.1;
        check("full=1 at depth 4", o_full, 1'b1);

        // ─────────────────────────────────────────────────────────────────────
        // TEST 6: Drain from full → empty; check all headers in order
        // ─────────────────────────────────────────────────────────────────────
        $display("\n--- TEST 6: Drain from full ---");
        for (int i = 0; i < 4; i++) begin
            automatic logic [63:0] rh, rd; automatic logic rhd;
            automatic logic [63:0] exp_hdr = make_hdr(59'(i+1), 1'b1);
            read_pkt(rh, rd, rhd);
            check64($sformatf("drain hdr[%0d]", i), rh, exp_hdr);
            check64($sformatf("drain dat[%0d]", i), rd, 64'(i+100));
        end
        @(posedge i_clk); #0.1;
        check("empty after full drain", o_empty, 1'b1);

        // ─────────────────────────────────────────────────────────────────────
        // TEST 7: o_is_pattern HIGH for SB_START_PATTERN in asm register
        // ─────────────────────────────────────────────────────────────────────
        $display("\n--- TEST 7: o_is_pattern HIGH ---");
        reset_dut();
        @(negedge i_clk);
        i_rx_word      = 64'h5555_5555_5555_5555;
        i_write_enable = 1;
        @(posedge i_clk); #0.1;
        i_write_enable = 0;
        #0.1;
        check("is_pattern HIGH for 0x5555...", o_is_pattern, 1'b1);

        // ─────────────────────────────────────────────────────────────────────
        // TEST 8: o_is_pattern LOW for non-pattern word
        // ─────────────────────────────────────────────────────────────────────
        $display("\n--- TEST 8: o_is_pattern LOW ---");
        reset_dut();
        @(negedge i_clk);
        i_rx_word      = 64'hDEAD_BEEF_CAFE_BABE;
        i_write_enable = 1;
        @(posedge i_clk); #0.1;
        i_write_enable = 0;
        #0.1;
        check("is_pattern LOW for non-pattern", o_is_pattern, 1'b0);

        // ─────────────────────────────────────────────────────────────────────
        // TEST 9: Circular wrap-around
        // ─────────────────────────────────────────────────────────────────────
        $display("\n--- TEST 9: Circular wrap ---");
        reset_dut();
        // Write 2, read 2 (pointers advance past index 0/1)
        for (int i = 0; i < 2; i++)
            write_pkt(make_hdr(59'(i+10), 1'b0), 64'h0, 1'b0);
        for (int i = 0; i < 2; i++) begin
            automatic logic [63:0] rh, rd; automatic logic rhd;
            read_pkt(rh, rd, rhd);
        end
        // Write 4 more (wraps around circular buffer)
        for (int i = 0; i < 4; i++)
            write_pkt(make_hdr(59'(i+20), 1'b1), 64'(i+200), 1'b1);
        begin
            automatic int ok = 0;
            for (int i = 0; i < 4; i++) begin
                automatic logic [63:0] rh, rd;   automatic logic rhd;
                automatic logic [63:0] exp_hdr = make_hdr(59'(i+20), 1'b1);
                automatic logic [63:0] exp_dat = 64'(i+200);
                read_pkt(rh, rd, rhd);
                if (rh === exp_hdr && rd === exp_dat) ok++;
                else $display("  FAIL  wrap[%0d] hdr got=%016h exp=%016h dat got=%016h exp=%016h",
                               i, rh, exp_hdr, rd, exp_dat);
            end
            if (ok == 4) begin $display("  PASS  Wrap-around 4/4 correct"); pass_cnt++; end
            else         begin $display("  FAIL  Wrap-around %0d/4 correct", ok); fail_cnt++; end
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
        #200000;
        $display("WATCHDOG: simulation exceeded limit – aborting");
        $finish;
    end

endmodule : tb_rx_fifo