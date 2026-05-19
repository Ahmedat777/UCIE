`timescale 1ns/1ps

module tb_error_handler;

    // =========================================================================
    // DUT ports
    // =========================================================================
    logic        clk;
    logic        rst_n;
    logic        i_pkt_valid;
    logic [63:0] i_header;
    logic [63:0] i_data;
    logic        i_has_data;
    logic        o_cp_error;
    logic        o_dp_error;

    // =========================================================================
    // DUT instantiation
    // =========================================================================
    sb_error_handler dut (
        .clk        (clk),
        .rst_n      (rst_n),
        .i_pkt_valid(i_pkt_valid),
        .i_header   (i_header),
        .i_data     (i_data),
        .i_has_data (i_has_data),
        .o_cp_error (o_cp_error),
        .o_dp_error (o_dp_error)
    );

    // =========================================================================
    // Clock  (500 MHz → 2.0 ns period)
    // =========================================================================
    localparam real CLK_PERIOD = 2.0;
    initial clk = 0;
    always #(CLK_PERIOD/2.0) clk = ~clk;

    // =========================================================================
    // Scoreboard / pass-fail counters
    // =========================================================================
    int pass_cnt = 0;
    int fail_cnt = 0;

    // =========================================================================
    // Helper: check CP and DP simultaneously
    // =========================================================================
    task check2(input string name,
                input logic got_cp, input logic exp_cp,
                input logic got_dp, input logic exp_dp);
        if (got_cp === exp_cp && got_dp === exp_dp) begin
            $display("  PASS  %-50s  cp=%0b dp=%0b", name, got_cp, got_dp);
            pass_cnt++;
        end else begin
            $display("  FAIL  %-50s  cp: got=%0b exp=%0b  dp: got=%0b exp=%0b  @%0t",
                     name, got_cp, exp_cp, got_dp, exp_dp, $time);
            fail_cnt++;
        end
    endtask

    // =========================================================================
    // Helper: build a header with correct CP (and optional DP)
    // =========================================================================
    function automatic logic [63:0] make_header(
        input logic [63:0] raw,   // header body (bits[62:63] must be 0)
        input logic [63:0] data,
        input logic        has_data
    );
        logic [63:0] h;
        logic        cp_val, dp_val;

        h        = raw;
        h[62]    = 1'b0;   // CP placeholder
        h[63]    = 1'b0;   // DP placeholder

        cp_val   = ^h;               // even parity over all 64 bits (62&63 zeroed)
        dp_val   = has_data ? (^data) : 1'b0;

        h[62]    = cp_val;
        h[63]    = dp_val;
        return h;
    endfunction

    // =========================================================================
    // Helper: apply stimulus and sample outputs (combinational DUT)
    // =========================================================================
    task apply(
        input logic [63:0] hdr,
        input logic [63:0] dat,
        input logic        has_d,
        input logic        valid
    );
        @(negedge clk);
        i_header    = hdr;
        i_data      = dat;
        i_has_data  = has_d;
        i_pkt_valid = valid;
        #0.2;  // let combinational settle
    endtask

    // =========================================================================
    // Test stimulus
    // =========================================================================
    initial begin
        $display("========================================================");
        $display("  tb_error_handler Testbench");
        $display("========================================================");

        // Reset
        rst_n       = 0;
        i_pkt_valid = 0;
        i_header    = '0;
        i_data      = '0;
        i_has_data  = 0;
        repeat(4) @(posedge clk);
        rst_n = 1;
        @(posedge clk); #0.1;

        // ─────────────────────────────────────────────────────────────────────
        // TEST 1: Correct header-only packet → no errors
        // ─────────────────────────────────────────────────────────────────────
        $display("\n--- TEST 1: Correct header-only ---");
        begin
            automatic logic [63:0] body = 64'hDEAD_BEEF_CAFE_0001;
            automatic logic [63:0] h    = make_header(body, 64'h0, 1'b0);
            apply(h, 64'h0, 1'b0, 1'b1);
            check2("Correct header-only → no errors", o_cp_error, 1'b0, o_dp_error, 1'b0);
        end

        // ─────────────────────────────────────────────────────────────────────
        // TEST 2: Correct header+data packet → no errors
        // ─────────────────────────────────────────────────────────────────────
        $display("\n--- TEST 2: Correct header+data ---");
        begin
            automatic logic [63:0] body = 64'h1234_5678_ABCD_EF01;
            automatic logic [63:0] dat  = 64'hFEED_FACE_DEAD_BEEF;
            automatic logic [63:0] h    = make_header(body, dat, 1'b1);
            apply(h, dat, 1'b1, 1'b1);
            check2("Correct header+data → no errors", o_cp_error, 1'b0, o_dp_error, 1'b0);
        end

        // ─────────────────────────────────────────────────────────────────────
        // TEST 3: CP bit flipped → o_cp_error
        // ─────────────────────────────────────────────────────────────────────
        $display("\n--- TEST 3: CP bit flipped ---");
        begin
            automatic logic [63:0] body = 64'hAAAA_BBBB_CCCC_0000;
            automatic logic [63:0] h    = make_header(body, 64'h0, 1'b0);
            h[62] = ~h[62];  // corrupt CP
            apply(h, 64'h0, 1'b0, 1'b1);
            check2("Flipped CP → cp_error=1, dp_error=0", o_cp_error, 1'b1, o_dp_error, 1'b0);
        end

        // ─────────────────────────────────────────────────────────────────────
        // TEST 4: Data bit flipped → o_dp_error (has_data=1)
        // ─────────────────────────────────────────────────────────────────────
        $display("\n--- TEST 4: Data bit flipped (has_data=1) ---");
        begin
            automatic logic [63:0] body = 64'h0000_1111_2222_3333;
            automatic logic [63:0] dat  = 64'hCAFE_BABE_1234_5678;
            automatic logic [63:0] h    = make_header(body, dat, 1'b1);
            dat[0] = ~dat[0];   // corrupt one data bit
            apply(h, dat, 1'b1, 1'b1);
            check2("Flipped data bit → cp_error=0, dp_error=1", o_cp_error, 1'b0, o_dp_error, 1'b1);
        end

        // ─────────────────────────────────────────────────────────────────────
        // TEST 5: DP bit flipped but has_data=0 → no dp_error
        // ─────────────────────────────────────────────────────────────────────
        $display("\n--- TEST 5: DP bit irrelevant when has_data=0 ---");
        begin
            automatic logic [63:0] body = 64'h0011_2233_4455_6677;
            automatic logic [63:0] h    = make_header(body, 64'h0, 1'b0);
            h[63] = ~h[63];  // corrupt DP field; but has_data=0
            // Note: flipping bit[63] also changes CP, re-compute CP
            begin
                automatic logic [63:0] tmp = h;
                tmp[62] = 1'b0;
                tmp[63] = h[63];   // keep corrupted DP
                h[62] = ^tmp;      // recalculate CP with new DP bit
            end
            apply(h, 64'h0, 1'b0, 1'b1);
            // dp_error should be 0 because has_data=0
            if (o_dp_error === 1'b0)
                $display("  PASS  DP not checked when has_data=0  dp=%0b", o_dp_error);
            else begin
                $display("  FAIL  DP error when has_data=0  dp=%0b @%0t", o_dp_error, $time);
                fail_cnt++;
            end
        end

        // ─────────────────────────────────────────────────────────────────────
        // TEST 6: Both CP and DP errors simultaneously
        // ─────────────────────────────────────────────────────────────────────
        $display("\n--- TEST 6: Both CP and DP errors ---");
        begin
            automatic logic [63:0] body = 64'hFFFF_0000_FFFF_0000;
            automatic logic [63:0] dat  = 64'h1111_2222_3333_4444;
            automatic logic [63:0] h    = make_header(body, dat, 1'b1);
            h[62] = ~h[62];   // corrupt CP
            dat[5] = ~dat[5]; // corrupt data
            apply(h, dat, 1'b1, 1'b1);
            check2("Both CP+DP errors", o_cp_error, 1'b1, o_dp_error, 1'b1);
        end

        // ─────────────────────────────────────────────────────────────────────
        // TEST 7: i_pkt_valid=0 → no errors regardless of parity
        // ─────────────────────────────────────────────────────────────────────
        $display("\n--- TEST 7: pkt_valid=0 suppresses errors ---");
        begin
            // Fully corrupt header and data
            apply(64'hFFFF_FFFF_FFFF_FFFF, 64'hFFFF_FFFF_FFFF_FFFF, 1'b1, 1'b0);
            check2("pkt_valid=0 → no errors", o_cp_error, 1'b0, o_dp_error, 1'b0);
        end

        // ─────────────────────────────────────────────────────────────────────
        // TEST 8: All-zeros header (special edge case)
        // ─────────────────────────────────────────────────────────────────────
        $display("\n--- TEST 8: All-zeros header ---");
        begin
            // All zeros: CP=even parity of 64 zeros=0, DP=0 (no data)
            apply(64'h0, 64'h0, 1'b0, 1'b1);
            check2("All-zeros header → no errors", o_cp_error, 1'b0, o_dp_error, 1'b0);
        end

        // ─────────────────────────────────────────────────────────────────────
        // TEST 9: All-ones header body (stress)
        // ─────────────────────────────────────────────────────────────────────
        $display("\n--- TEST 9: All-ones header body ---");
        begin
            automatic logic [63:0] body = 64'hFFFF_FFFF_FFFF_FFFF;
            automatic logic [63:0] dat  = 64'hFFFF_FFFF_FFFF_FFFF;
            automatic logic [63:0] h    = make_header(body, dat, 1'b1);
            apply(h, dat, 1'b1, 1'b1);
            check2("All-ones → correct parity → no errors", o_cp_error, 1'b0, o_dp_error, 1'b0);
        end

        // ─────────────────────────────────────────────────────────────────────
        // TEST 10: Verify CP field isolation (bits[62:63] zeroed before XOR)
        // ─────────────────────────────────────────────────────────────────────
        $display("\n--- TEST 10: CP field isolation ---");
        begin
            // A header where bit[62]=1, bit[63]=0 for CP and DP respectively,
            // and all other bits 0. XOR of 64 bits with bit[62]=0 and [63]=0 = 0.
            // So CP should be 0. Set [62]=1 to create a mismatch.
            automatic logic [63:0] h = 64'h0;
            h[62] = 1'b1;   // wrong CP (should be 0 for all-zero body)
            apply(h, 64'h0, 1'b0, 1'b1);
            check2("Isolated CP field error detected", o_cp_error, 1'b1, o_dp_error, 1'b0);
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
        #50000;
        $display("WATCHDOG: simulation exceeded 50 µs – aborting");
        $finish;
    end

endmodule : tb_error_handler