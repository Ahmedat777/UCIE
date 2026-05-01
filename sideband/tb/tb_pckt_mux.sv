// =============================================================================
// tb_pckt_mux.sv  –  Testbench for SB_PACKET_ENCODER_MUX
//
// Verifies:
//   1. Idle (both valids low)  → o_final_packet = 64'h0.
//   2. Only encoded valid      → o_final_packet = i_encoded_packet.
//   3. Only pattern valid      → o_final_packet = i_pattern.
//   4. Both valids high        → i_pattern wins (priority).
//   5. Valid de-assertion      → output returns to 0.
//   6. Randomised stimulus     → priority rule always holds.
// =============================================================================

`timescale 1ns/1ps

module tb_pckt_mux;

    // ── DUT ports ─────────────────────────────────────────────────────────────
    logic [63:0] i_pattern;
    logic [63:0] i_encoded_packet;
    logic        i_pattern_valid;
    logic        i_encoded_pkt_valid;
    logic [63:0] o_final_packet;

    // ── DUT ───────────────────────────────────────────────────────────────────
    SB_PACKET_ENCODER_MUX dut (
        .i_pattern          (i_pattern),
        .i_encoded_packet   (i_encoded_packet),
        .i_pattern_valid    (i_pattern_valid),
        .i_encoded_pkt_valid(i_encoded_pkt_valid),
        .o_final_packet     (o_final_packet)
    );

    // ── Counters ──────────────────────────────────────────────────────────────
    int pass_cnt = 0;
    int fail_cnt = 0;

    task check64(input string name, input logic [63:0] got, exp);
        if (got === exp) begin
            $display("  PASS  %-45s  got=%016h", name, got);
            pass_cnt++;
        end else begin
            $display("  FAIL  %-45s  got=%016h exp=%016h", name, got, exp);
            fail_cnt++;
        end
    endtask

    // ── Combinational apply helper ─────────────────────────────────────────
    task apply(
        input logic [63:0] pat, enc,
        input logic        pv, ev
    );
        i_pattern          = pat;
        i_encoded_packet   = enc;
        i_pattern_valid    = pv;
        i_encoded_pkt_valid = ev;
        #1;  // propagation delay
    endtask

    // ── Main ──────────────────────────────────────────────────────────────────
    initial begin
        $display("==== tb_pckt_mux ====");

        // ------------------------------------------------------------------
        // TEST 1: Idle – both valids low
        // ------------------------------------------------------------------
        $display("\n-- TEST 1: Idle --");
        apply(64'hAAAA_BBBB_CCCC_DDDD, 64'h1111_2222_3333_4444, 0, 0);
        check64("Idle → 0", o_final_packet, 64'h0);

        // ------------------------------------------------------------------
        // TEST 2: Only encoded packet valid
        // ------------------------------------------------------------------
        $display("\n-- TEST 2: Encoded only --");
        apply(64'hAAAA_BBBB_CCCC_DDDD, 64'h1111_2222_3333_4444, 0, 1);
        check64("Encoded only → encoded_packet", o_final_packet, 64'h1111_2222_3333_4444);

        // ------------------------------------------------------------------
        // TEST 3: Only pattern valid
        // ------------------------------------------------------------------
        $display("\n-- TEST 3: Pattern only --");
        apply(64'h5555_5555_5555_5555, 64'h1111_2222_3333_4444, 1, 0);
        check64("Pattern only → pattern", o_final_packet, 64'h5555_5555_5555_5555);

        // ------------------------------------------------------------------
        // TEST 4: Both valids – pattern has priority
        // ------------------------------------------------------------------
        $display("\n-- TEST 4: Both valid → pattern wins --");
        apply(64'hDEAD_BEEF_CAFE_BABE, 64'h1234_5678_9ABC_DEF0, 1, 1);
        check64("Both valid → pattern", o_final_packet, 64'hDEAD_BEEF_CAFE_BABE);

        // ------------------------------------------------------------------
        // TEST 5: De-assert valid → output back to 0
        // ------------------------------------------------------------------
        $display("\n-- TEST 5: De-assert → idle --");
        apply(64'hDEAD_BEEF_CAFE_BABE, 64'h1234_5678_9ABC_DEF0, 0, 0);
        check64("De-assert → 0", o_final_packet, 64'h0);

        // ------------------------------------------------------------------
        // TEST 6: Sweep random combinations (100 iterations)
        // ------------------------------------------------------------------
        $display("\n-- TEST 6: Random combinations --");
        begin
            automatic int fail_r = 0;
            for (int i = 0; i < 100; i++) begin
                automatic logic [63:0] rp  = $urandom_range(0, {32{1'b1}});
                automatic logic [63:0] rp2 = {rp, ~rp};
                automatic logic [63:0] re  = $urandom_range(0, {32{1'b1}});
                automatic logic [63:0] re2 = {re, ~re};
                automatic logic        pv  = $urandom_range(0, 1);
                automatic logic        ev  = $urandom_range(0, 1);
                automatic logic [63:0] expected;

                apply(rp2, re2, pv, ev);

                if      (pv) expected = rp2;
                else if (ev) expected = re2;
                else         expected = 64'h0;

                if (o_final_packet !== expected) begin
                    $display("  FAIL  iter=%0d pv=%0b ev=%0b pat=%016h enc=%016h got=%016h exp=%016h",
                             i, pv, ev, rp2, re2, o_final_packet, expected);
                    fail_r++;
                end
            end
            if (fail_r == 0) begin
                $display("  PASS  All 100 random iterations correct");
                pass_cnt++;
            end else begin
                $display("  FAIL  %0d of 100 random iterations wrong", fail_r);
                fail_cnt++;
            end
        end

        // ------------------------------------------------------------------
        // TEST 7: Check pattern value = SB_START_PATTERN constant
        // ------------------------------------------------------------------
        $display("\n-- TEST 7: SB_START_PATTERN passthrough --");
        apply(64'h5555_5555_5555_5555, 64'h0, 1, 0);
        check64("SB_START_PATTERN passes through", o_final_packet, 64'h5555_5555_5555_5555);

        // ------------------------------------------------------------------
        // Summary
        // ------------------------------------------------------------------
        $display("\n==== RESULTS: %0d passed, %0d failed ====", pass_cnt, fail_cnt);
        if (fail_cnt == 0) $display("ALL TESTS PASSED");
        else               $display("SOME TESTS FAILED");
        $finish;
    end

endmodule : tb_pckt_mux