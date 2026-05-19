`timescale 1ns/1ps

module tb_pckt_mux;

    // =========================================================================
    // DUT ports
    // =========================================================================
    logic [63:0] i_pattern;
    logic        i_pattern_valid;
    logic [63:0] i_enc_header;
    logic        i_enc_has_data;
    logic        i_encoded_pkt_valid;
    logic [63:0] o_final_packet;
    logic        o_final_valid;
    logic        o_final_has_data;

    // =========================================================================
    // DUT instantiation
    // =========================================================================
    SB_PACKET_ENCODER_MUX dut (
        .i_pattern           (i_pattern),
        .i_pattern_valid     (i_pattern_valid),
        .i_enc_header        (i_enc_header),
        .i_enc_has_data      (i_enc_has_data),
        .i_encoded_pkt_valid (i_encoded_pkt_valid),
        .o_final_packet      (o_final_packet),
        .o_final_valid       (o_final_valid),
        .o_final_has_data    (o_final_has_data)
    );

    // =========================================================================
    // Scoreboard / pass-fail counters
    // =========================================================================
    int pass_cnt = 0;
    int fail_cnt = 0;

    // =========================================================================
    // Helper: check 64-bit signal
    // =========================================================================
    task check64(input string name, input logic [63:0] got, exp);
        if (got === exp) begin
            $display("  PASS  %-50s  got=%016h", name, got);
            pass_cnt++;
        end else begin
            $display("  FAIL  %-50s  got=%016h exp=%016h", name, got, exp);
            fail_cnt++;
        end
    endtask

    // =========================================================================
    // Helper: check 1-bit signal
    // =========================================================================
    task check1(input string name, input logic got, exp);
        if (got === exp) begin
            $display("  PASS  %-50s  got=%0b", name, got);
            pass_cnt++;
        end else begin
            $display("  FAIL  %-50s  got=%0b exp=%0b", name, got, exp);
            fail_cnt++;
        end
    endtask

    // =========================================================================
    // Helper: set all inputs then wait 1 ns for combinational settle
    // =========================================================================
    task apply(
        input logic [63:0] pat,
        input logic        pv,
        input logic [63:0] enc_hdr,
        input logic        enc_hd,
        input logic        ev
    );
        i_pattern          = pat;
        i_pattern_valid    = pv;
        i_enc_header       = enc_hdr;
        i_enc_has_data     = enc_hd;
        i_encoded_pkt_valid = ev;
        #1;
    endtask

    // =========================================================================
    // Test stimulus
    // =========================================================================
    initial begin
        $display("========================================================");
        $display("  tb_pckt_mux Testbench");
        $display("========================================================");

        // ─────────────────────────────────────────────────────────────────────
        // TEST 1: Idle – both valids low
        // ─────────────────────────────────────────────────────────────────────
        $display("\n--- TEST 1: Idle (both valids low) ---");
        apply(64'hAAAA_BBBB_CCCC_DDDD, 0, 64'h1111_2222_3333_4444, 0, 0);
        check64("Idle → packet=0",    o_final_packet,   64'h0);
        check1 ("Idle → valid=0",     o_final_valid,    1'b0);
        check1 ("Idle → has_data=0",  o_final_has_data, 1'b0);

        // ─────────────────────────────────────────────────────────────────────
        // TEST 2: Encoded only, no data flit (has_data=0)
        // ─────────────────────────────────────────────────────────────────────
        $display("\n--- TEST 2: Encoded only, has_data=0 ---");
        apply(64'hAAAA_BBBB_CCCC_DDDD, 0, 64'h1111_2222_3333_4444, 0, 1);
        check64("Enc only → header",      o_final_packet,   64'h1111_2222_3333_4444);
        check1 ("Enc only → valid=1",     o_final_valid,    1'b1);
        check1 ("Enc only → has_data=0",  o_final_has_data, 1'b0);

        // ─────────────────────────────────────────────────────────────────────
        // TEST 3: Encoded only, with data flit (has_data=1)
        // ─────────────────────────────────────────────────────────────────────
        $display("\n--- TEST 3: Encoded only, has_data=1 ---");
        apply(64'hAAAA_BBBB_CCCC_DDDD, 0, 64'hDEAD_BEEF_CAFE_BABE, 1, 1);
        check64("Enc+data → header",      o_final_packet,   64'hDEAD_BEEF_CAFE_BABE);
        check1 ("Enc+data → valid=1",     o_final_valid,    1'b1);
        check1 ("Enc+data → has_data=1",  o_final_has_data, 1'b1);

        // ─────────────────────────────────────────────────────────────────────
        // TEST 4: Pattern only
        // ─────────────────────────────────────────────────────────────────────
        $display("\n--- TEST 4: Pattern only ---");
        apply(64'h5555_5555_5555_5555, 1, 64'h1111_2222_3333_4444, 0, 0);
        check64("Pattern only → pattern",    o_final_packet,   64'h5555_5555_5555_5555);
        check1 ("Pattern only → valid=1",    o_final_valid,    1'b1);
        check1 ("Pattern only → has_data=0", o_final_has_data, 1'b0);

        // ─────────────────────────────────────────────────────────────────────
        // TEST 5: Both valid – pattern wins; has_data must be 0
        // ─────────────────────────────────────────────────────────────────────
        $display("\n--- TEST 5: Both valid → pattern wins ---");
        apply(64'hDEAD_BEEF_CAFE_BABE, 1, 64'h1234_5678_9ABC_DEF0, 1, 1);
        check64("Both → pattern wins",       o_final_packet,   64'hDEAD_BEEF_CAFE_BABE);
        check1 ("Both → valid=1",            o_final_valid,    1'b1);
        check1 ("Both → has_data=0 (pat)",   o_final_has_data, 1'b0);

        // ─────────────────────────────────────────────────────────────────────
        // TEST 6: De-assert both → output returns to idle
        // ─────────────────────────────────────────────────────────────────────
        $display("\n--- TEST 6: De-assert → idle ---");
        apply(64'hDEAD_BEEF_CAFE_BABE, 0, 64'h1234_5678_9ABC_DEF0, 1, 0);
        check64("De-assert → packet=0",  o_final_packet,   64'h0);
        check1 ("De-assert → valid=0",   o_final_valid,    1'b0);
        check1 ("De-assert → has_data=0",o_final_has_data, 1'b0);

        // ─────────────────────────────────────────────────────────────────────
        // TEST 7: Pattern overrides has_data=1 from encoder
        //         (has_data must be 0 whenever pattern wins)
        // ─────────────────────────────────────────────────────────────────────
        $display("\n--- TEST 7: Pattern forces has_data=0 ---");
        apply(64'h5555_5555_5555_5555, 1, 64'hFFFF_FFFF_FFFF_FFFF, 1, 1);
        check1("Pattern wins → has_data forced 0", o_final_has_data, 1'b0);
        check1("Pattern wins → valid=1",            o_final_valid,    1'b1);

        // ─────────────────────────────────────────────────────────────────────
        // TEST 8: Randomised stimulus – priority and has_data rules
        // ─────────────────────────────────────────────────────────────────────
        $display("\n--- TEST 8: Random combinations (100 iterations) ---");
        begin
            automatic int fail_r = 0;
            for (int i = 0; i < 100; i++) begin
                automatic logic [63:0] rp  = {$urandom, $urandom};
                automatic logic [63:0] re  = {$urandom, $urandom};
                automatic logic        pv  = $urandom_range(0, 1);
                automatic logic        ev  = $urandom_range(0, 1);
                automatic logic        ehd = $urandom_range(0, 1);
                automatic logic [63:0] exp_pkt;
                automatic logic        exp_valid;
                automatic logic        exp_hd;

                apply(rp, pv, re, ehd, ev);

                if (pv) begin
                    exp_pkt   = rp;
                    exp_valid = 1'b1;
                    exp_hd    = 1'b0;   // pattern always clears has_data
                end else if (ev) begin
                    exp_pkt   = re;
                    exp_valid = 1'b1;
                    exp_hd    = ehd;
                end else begin
                    exp_pkt   = 64'h0;
                    exp_valid = 1'b0;
                    exp_hd    = 1'b0;
                end

                if (o_final_packet !== exp_pkt || o_final_valid !== exp_valid
                                               || o_final_has_data !== exp_hd) begin
                    $display("  FAIL  iter=%0d pv=%0b ev=%0b ehd=%0b | pkt=%016h(exp=%016h) valid=%0b(exp=%0b) hd=%0b(exp=%0b)",
                             i, pv, ev, ehd,
                             o_final_packet, exp_pkt,
                             o_final_valid,  exp_valid,
                             o_final_has_data, exp_hd);
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

        // ─────────────────────────────────────────────────────────────────────
        // TEST 9: SB_START_PATTERN constant passthrough
        // ─────────────────────────────────────────────────────────────────────
        $display("\n--- TEST 9: SB_START_PATTERN constant passthrough ---");
        apply(64'h5555_5555_5555_5555, 1, 64'h0, 0, 0);
        check64("SB_START_PATTERN passes through", o_final_packet, 64'h5555_5555_5555_5555);
        check1 ("SB_START_PATTERN → valid=1",      o_final_valid,  1'b1);

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

endmodule : tb_pckt_mux