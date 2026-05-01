`timescale 1ns/1ps
import pckg::*;

module tb_sb_wrapper_pair;

    // ---------------------------------------------------------------------
    // Clock & Reset
    // ---------------------------------------------------------------------
    logic clk = 0;
    logic rst_n = 0;
    always #1 clk = ~clk; // 500 MHz

    // ---------------------------------------------------------------------
    // TX Interface
    // ---------------------------------------------------------------------
    logic        tx_msg_valid = 0;
    sb_msg_id    tx_msg_id    = SB_SBINIT_OUT_OF_RESET;
    logic [63:0] tx_ltsm_data = '0;
    logic [15:0] tx_ltsm_msginfo = '0;
    logic        tx_start_pattern_req = 0;
    logic        tx_sb_ltsm_resp_rcvd = 0;
    logic        tx_time_out = 0;

    logic        tx_busy, tx_rsp_timeout, tx_timeout_start;
    sb_msg_id    tx_timeout_msg_id;
    logic        tx_pattern_time_out;
    logic        TXCKSB;

    // ---------------------------------------------------------------------
    // RX Interface
    // ---------------------------------------------------------------------
    logic        rx_de_ser_done = 0;
    logic [63:0] rx_rx_word = '0;

    logic        rx_msg_valid;
    sb_msg_id    rx_msg_id;
    logic [15:0] rx_msginfo;
    logic [63:0] rx_rx_data;
    logic        rx_decoder_valid;
    logic        rx_cp_error, rx_dp_error;
    logic        rx_sb_start_pattern, rx_sb_pattern_samp_done;
    logic        rx_timeout_stop;
    sb_msg_id    rx_timeout_rx_msg_id;
    logic [15:0] rx_timeout_rx_msginfo;
    logic        rx_fifo_overflow;
    logic        rx_de_ser_done_sampled;

    // ---------------------------------------------------------------------
    // DUTs
    // ---------------------------------------------------------------------
    SB_TX_WRAPPER u_tx (
        .i_clk                    (clk),
        .i_rst_n                  (rst_n),

        .i_msg_valid              (tx_msg_valid),
        .i_msg_id                 (tx_msg_id),
        .i_ltsm_data              (tx_ltsm_data),
        .i_ltsm_msginfo           (tx_ltsm_msginfo),

        .i_start_pattern_req      (tx_start_pattern_req),
        .i_sb_ltsm_resp_rcvd      (tx_sb_ltsm_resp_rcvd),
        .i_time_out               (tx_time_out),

        .i_rx_sb_pattern_samp_done(rx_sb_pattern_samp_done),

        .o_busy                   (tx_busy),
        .o_rsp_timeout            (tx_rsp_timeout),
        .o_timeout_start          (tx_timeout_start),
        .o_timeout_msg_id         (tx_timeout_msg_id),
        .o_pattern_time_out       (tx_pattern_time_out),

        .TXCKSB                   (TXCKSB)
    );

    SB_RX_WRAPPER u_rx (
        .i_clk                    (clk),
        .i_rst_n                  (rst_n),

        .i_de_ser_done            (rx_de_ser_done),
        .i_rx_word                (rx_rx_word),

        .o_msg_valid              (rx_msg_valid),
        .o_msg_id                 (rx_msg_id),
        .o_msginfo                (rx_msginfo),
        .o_rx_data                (rx_rx_data),
        .o_decoder_valid          (rx_decoder_valid),

        .o_cp_error               (rx_cp_error),
        .o_dp_error               (rx_dp_error),

        .o_rx_sb_start_pattern    (rx_sb_start_pattern),
        .o_rx_sb_pattern_samp_done(rx_sb_pattern_samp_done),

        .o_timeout_stop           (rx_timeout_stop),
        .o_timeout_rx_msg_id      (rx_timeout_rx_msg_id),
        .o_timeout_rx_msginfo     (rx_timeout_rx_msginfo),

        .o_fifo_overflow          (rx_fifo_overflow),
        .o_de_ser_done_sampled    (rx_de_ser_done_sampled)
    );

    // ---------------------------------------------------------------------
    // ✅ FIX 1: SERDES MODEL — Separate header/data capture lanes
    //
    //    OLD BUG: both w_read_enable and w_read_data_enable wrote to the
    //    same serdes_word_d register. When both fire in the same cycle the
    //    data word silently overwrites the header before it is consumed.
    //
    //    FIX: keep two independent registers (hdr_word_d / dat_word_d) and
    //    a sticky flag (dat_valid_d) that tracks which lane is active.
    //    The MUX selects the correct word when ser_done_d qualifies the
    //    output, so header and data are never aliased.
    // ---------------------------------------------------------------------
    logic [63:0] hdr_word_d;
    logic [63:0] dat_word_d;
    logic        dat_valid_d;     // high when last capture was a data word
    logic        ser_done_d;
    logic        inject_err = 0;

    always_ff @(posedge clk) begin
        // Lane 1 – header
        if (u_tx.w_read_enable)
            hdr_word_d <= u_tx.w_fifo_header;

        // Lane 2 – data (also updates the lane-select flag)
        if (u_tx.w_read_data_enable) begin
            dat_word_d  <= u_tx.w_fifo_data;
            dat_valid_d <= 1'b1;
        end else if (u_tx.w_read_enable)
            dat_valid_d <= 1'b0;   // new header clears the data-valid flag

        // Pipeline the ser_done strobe by one cycle so it aligns with the
        // captured word (both lanes settle one cycle before ser_done fires)
        ser_done_d <= u_tx.w_ser_done;

        // Drive RX inputs
        rx_de_ser_done <= ser_done_d;
        if (ser_done_d) begin
            automatic logic [63:0] muxed = dat_valid_d ? dat_word_d
                                                       : hdr_word_d;
            rx_rx_word <= inject_err ? (muxed ^ 64'h1) : muxed;
        end
    end

    // ---------------------------------------------------------------------
    // PASS / FAIL
    // ---------------------------------------------------------------------
    int passes = 0, fails = 0;

    task automatic check(input string msg, input logic cond);
        if (cond) begin
            $display("[PASS] %s", msg); passes++;
        end else begin
            $display("[FAIL] %s", msg); fails++;
        end
    endtask

    // ---------------------------------------------------------------------
    // ✅ FIX 2: wait_sig — unchanged (was correct)
    // ---------------------------------------------------------------------
    task automatic wait_sig(input logic sig, input int N, input string msg);
        int i;
        for (i = 0; i < N; i++) begin
            @(posedge clk);
            if (sig) return;
        end
        $display("[TIMEOUT] %s", msg);
        fails++;
    endtask

    // ---------------------------------------------------------------------
    // ✅ FIX 2: wait_pulse — sample prev AFTER the first clock edge
    //
    //    OLD BUG: `logic prev = sig` captures the combinational value of
    //    sig at the moment the task is *called*, before any clock edge.
    //    If sig is already high (or pulses on the very first edge) the
    //    rising-edge detector never fires and the task times out.
    //
    //    FIX: advance one @(posedge clk) first, then latch prev. That way
    //    prev always reflects a sampled, stable value and the rising-edge
    //    check (!prev && sig) can never miss the very first pulse.
    // ---------------------------------------------------------------------
    task automatic wait_pulse(input logic sig, input int N, input string msg);
        logic prev;
        @(posedge clk);
        prev = sig;                // first sample at a real clock edge
        for (int i = 0; i < N-1; i++) begin
            @(posedge clk);
            if (!prev && sig) return;   // rising edge detected
            prev = sig;
        end
        $display("[TIMEOUT] %s", msg);
        fails++;
    endtask

    // ---------------------------------------------------------------------
    // ✅ FIX 5: send_msg — guard against a missed rx_msg_valid pulse
    //
    //    OLD BUG: wait_pulse always waits for a *rising* edge. If the RX
    //    path is very short, rx_msg_valid can already be high by the time
    //    send_msg calls wait_pulse, so the rising edge is in the past and
    //    the task stalls until timeout.
    //
    //    FIX: sample rx_msg_valid immediately after deasserting tx_msg_valid.
    //    If it is already high we skip wait_pulse entirely. Otherwise we
    //    wait for the rising edge as before.
    // ---------------------------------------------------------------------
    task automatic send_msg(input sb_msg_id id,
                            input logic [63:0] data = '0,
                            input logic [15:0] info = '0);

        tx_msg_valid    = 1;
        tx_msg_id       = id;
        tx_ltsm_data    = data;
        tx_ltsm_msginfo = info;

        @(posedge clk);
        tx_msg_valid = 0;

        // Level-check first — pulse may already have arrived
        if (!rx_msg_valid)
            wait_pulse(rx_msg_valid, 5000, "rx_msg_valid");

        tx_sb_ltsm_resp_rcvd = 1;
        @(posedge clk);
        tx_sb_ltsm_resp_rcvd = 0;

        repeat (5) @(posedge clk);
    endtask

    // ---------------------------------------------------------------------
    // WAVES + WATCHDOG
    // ---------------------------------------------------------------------
    initial begin
        $dumpfile("tb.vcd");
        $dumpvars(0, tb_sb_wrapper_pair);
    end

    initial begin
        #10_000_000;
        $display("[WATCHDOG] timeout");
        $finish(1);
    end

    // ---------------------------------------------------------------------
    // TEST SEQUENCE
    // ---------------------------------------------------------------------
    initial begin
        $display("=== SB TX/RX Wrapper Testbench ===");

        rst_n = 0;
        repeat (20) @(posedge clk);
        rst_n = 1;
        repeat (20) @(posedge clk);

        // ------------------------------------------------------------------
        // TC1: Pattern Handshake
        // ------------------------------------------------------------------
        $display("\n-- TC1: Pattern Handshake --");
        tx_start_pattern_req = 1;
        @(posedge clk);
        tx_start_pattern_req = 0;

        wait_sig(rx_sb_pattern_samp_done, 2000, "pattern lock");
        check("TC1 pattern locked", rx_sb_pattern_samp_done);

        wait_sig(u_tx.w_start_pattern_done, 2000, "tx pattern done");
        check("TC1 tx pattern done", u_tx.w_start_pattern_done);

        // ------------------------------------------------------------------
        // TC2: SBINIT_OUT_OF_RESET message
        // ------------------------------------------------------------------
        $display("\n-- TC2: SBINIT_OUT_OF_RESET --");
        send_msg(SB_SBINIT_OUT_OF_RESET);
        check("TC2 msg id", rx_msg_id == SB_SBINIT_OUT_OF_RESET);

        // ------------------------------------------------------------------
        // TC3: SBINIT_DONE_REQ message
        // ------------------------------------------------------------------
        $display("\n-- TC3: SBINIT_DONE_REQ --");
        send_msg(SB_SBINIT_DONE_REQ);
        check("TC3 msg id", rx_msg_id == SB_SBINIT_DONE_REQ);

        // ------------------------------------------------------------------
        // TC4: Message with payload — REPAIRCLK_APPLY_REPAIR_REQ
        // ------------------------------------------------------------------
        $display("\n-- TC4: REPAIRCLK_APPLY_REPAIR_REQ with data --");
        send_msg(SB_MBINIT_REPAIRCLK_APPLY_REPAIR_REQ,
                 64'hDEAD_BEEF_CAFE_1234);
        check("TC4 data", rx_rx_data == 64'hDEAD_BEEF_CAFE_1234);

        // ------------------------------------------------------------------
        // TC5: Message with payload — MBTRAIN_REPAIR_APPLY_REPAIR_REQ
        // ------------------------------------------------------------------
        $display("\n-- TC5: MBTRAIN_REPAIR_APPLY_REPAIR_REQ with data --");
        send_msg(SB_MBTRAIN_REPAIR_APPLY_REPAIR_REQ,
                 64'h1234_5678_9ABC_DEF0);
        check("TC5 data", rx_rx_data == 64'h1234_5678_9ABC_DEF0);

        // ------------------------------------------------------------------
        // TC6: RDI state messages
        // ------------------------------------------------------------------
        $display("\n-- TC6: RDI messages --");
        send_msg(SB_RDI_REQ_ACTIVE);
        check("TC6a RDI_ACTIVE", rx_msg_id == SB_RDI_REQ_ACTIVE);

        send_msg(SB_RDI_REQ_LINKRESET);
        check("TC6b RDI_LINKRESET", rx_msg_id == SB_RDI_REQ_LINKRESET);

        // ------------------------------------------------------------------
        // TC7: Response timeout
        //
        //    ✅ FIX 3: OLD code waited a blind 20-cycle gap after tx_busy,
        //    which has no relationship to when the TX FSM actually enters its
        //    wait window.  The TX wrapper already drives o_timeout_start
        //    (= tx_timeout_start here) precisely when the FSM begins waiting
        //    for a response.  We wait for that signal instead, then assert
        //    tx_time_out a couple of cycles later so the FSM is guaranteed
        //    to be in the right state to latch it.
        // ------------------------------------------------------------------
        $display("\n-- TC7: Response timeout --");
        tx_msg_valid = 1;
        tx_msg_id    = SB_SBINIT_DONE_RESP;
        @(posedge clk);
        tx_msg_valid = 0;

        wait_sig(tx_busy, 1000, "TC7 tx_busy");

        // Wait for FSM to signal it is now in its response-wait window
        wait_sig(tx_timeout_start, 1000, "TC7 timeout_start");
        repeat (2) @(posedge clk);   // small settling margin

        tx_time_out = 1;
        @(posedge clk);
        tx_time_out = 0;

        wait_sig(tx_rsp_timeout, 200, "TC7 rsp_timeout");
        check("TC7 timeout", tx_rsp_timeout);

        // ------------------------------------------------------------------
        // TC8: Parity / CRC error injection
        //
        //    ✅ FIX 4: OLD code checked rx_cp_error/rx_dp_error immediately
        //    after send_msg returned.  Both flags are registered outputs in
        //    the RX decoder and are only valid one or more cycles after
        //    rx_msg_valid is seen.  We wait for a rising edge on either flag
        //    before sampling.
        // ------------------------------------------------------------------
        $display("\n-- TC8: Parity error injection --");
        inject_err = 1;
        send_msg(SB_MBINIT_PARAM_CONFIG_REQ);

        // Wait for the registered error flag to pulse
        wait_pulse(rx_cp_error | rx_dp_error, 20, "TC8 parity error pulse");
        check("TC8 parity error", rx_cp_error | rx_dp_error);
        inject_err = 0;

        // ------------------------------------------------------------------
        // Results
        // ------------------------------------------------------------------
        repeat (10) @(posedge clk);

        $display("\n=== RESULTS: %0d PASS / %0d FAIL ===", passes, fails);

        if (fails == 0) $finish(0);
        else            $finish(1);
    end

endmodule