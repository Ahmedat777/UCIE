`timescale 1ns / 1ps

module tb_REVERSALMB_Wrapper;

parameter CLK_PERIOD = 10;

// DUT1 signals (Transmitter/Module side)
reg         dut1_CLK, dut1_rst_n;
reg         dut1_i_MBINIT_REPAIRVAL_end;
reg         dut1_i_REVERSAL_done;
reg         dut1_i_LaneID_Pattern_done;
reg  [15:0] dut1_i_REVERSAL_Result_SB;
reg  [15:0] dut1_i_REVERSAL_Result_logged;
wire [3:0]  dut1_i_Rx_SbMessage;
wire        dut1_i_msg_valid, dut1_i_falling_edge_busy;
wire [3:0]  dut1_o_TX_SbMessage;
wire [1:0]  dut1_o_MBINIT_REVERSALMB_LaneID_Pattern_En;
wire        dut1_o_MBINIT_ApplyReversal_En;
wire        dut1_o_MBINIT_REVERSALMB_end;
wire [1:0]  dut1_o_Clear_Pattern_Comparator;
wire [15:0] dut1_o_REVERSAL_Pattern_Result_logged;
wire        dut1_o_ValidOutDatatREVERSALMB;
wire        dut1_o_ValidDataFieldParameters;
wire        dut1_o_train_error_req_reversalmb;

// DUT2 signals (Partner/Receiver side)
reg         dut2_CLK, dut2_rst_n;
reg         dut2_i_MBINIT_REPAIRVAL_end;
reg         dut2_i_REVERSAL_done;
reg         dut2_i_LaneID_Pattern_done;
reg  [15:0] dut2_i_REVERSAL_Result_SB;
reg  [15:0] dut2_i_REVERSAL_Result_logged;
wire [3:0]  dut2_i_Rx_SbMessage;
wire        dut2_i_msg_valid, dut2_i_falling_edge_busy;
wire [3:0]  dut2_o_TX_SbMessage;
wire [1:0]  dut2_o_MBINIT_REVERSALMB_LaneID_Pattern_En;
wire        dut2_o_MBINIT_ApplyReversal_En;
wire        dut2_o_MBINIT_REVERSALMB_end;
wire [1:0]  dut2_o_Clear_Pattern_Comparator;
wire [15:0] dut2_o_REVERSAL_Pattern_Result_logged;
wire        dut2_o_ValidOutDatatREVERSALMB;
wire        dut2_o_ValidDataFieldParameters;
wire        dut2_o_train_error_req_reversalmb;

// Message definitions
localparam [3:0] MBINI_REVERSALMB_init_req            = 4'b0001;
localparam [3:0] MBINIT_REVERSALMB_init_resp          = 4'b0010;
localparam [3:0] MBINIT_REVERSALMB_clear_error_req    = 4'b0011;
localparam [3:0] MBINIT_REVERSALMB_clear_error_resp   = 4'b0100;
localparam [3:0] MBINIT_REVERSALMB_result_req         = 4'b0101;
localparam [3:0] MBINIT_REVERSALMB_result_resp        = 4'b0110;
localparam [3:0] MBINIT_REVERSALMB_done_req           = 4'b0111;
localparam [3:0] MBINIT_REVERSALMB_done_resp          = 4'b1000;

// Cross-connect messages
// DUT1 receives what DUT2 transmits, and vice versa
assign dut1_i_Rx_SbMessage = dut2_o_TX_SbMessage;
assign dut2_i_Rx_SbMessage = dut1_o_TX_SbMessage;

////////////////////////////////////////////////////////////////////////////////
// Falling edge busy generation: pulse 2-3 cycles AFTER ValidOut rises
////////////////////////////////////////////////////////////////////////////////
reg [2:0] dut1_valid_counter, dut2_valid_counter;
reg dut1_prev_ValidOut, dut2_prev_ValidOut;

always @(posedge dut1_CLK or negedge dut1_rst_n) begin
    if (!dut1_rst_n) begin
        dut1_valid_counter <= 3'd0;
        dut1_prev_ValidOut <= 1'b0;
    end else begin
        dut1_prev_ValidOut <= dut1_o_ValidOutDatatREVERSALMB;
        if (dut1_o_ValidOutDatatREVERSALMB && !dut1_prev_ValidOut)
            dut1_valid_counter <= 3'd3;
        else if (dut1_valid_counter > 0)
            dut1_valid_counter <= dut1_valid_counter - 1;
    end
end

always @(posedge dut2_CLK or negedge dut2_rst_n) begin
    if (!dut2_rst_n) begin
        dut2_valid_counter <= 3'd0;
        dut2_prev_ValidOut <= 1'b0;
    end else begin
        dut2_prev_ValidOut <= dut2_o_ValidOutDatatREVERSALMB;
        if (dut2_o_ValidOutDatatREVERSALMB && !dut2_prev_ValidOut)
            dut2_valid_counter <= 3'd3;
        else if (dut2_valid_counter > 0)
            dut2_valid_counter <= dut2_valid_counter - 1;
    end
end

assign dut1_i_falling_edge_busy = (dut1_valid_counter == 1);
assign dut2_i_falling_edge_busy = (dut2_valid_counter == 1);

////////////////////////////////////////////////////////////////////////////////
// Message valid generation
////////////////////////////////////////////////////////////////////////////////
reg [3:0] dut1_prev_rx_msg, dut2_prev_rx_msg;

always @(posedge dut1_CLK or negedge dut1_rst_n) begin
    if (!dut1_rst_n)
        dut1_prev_rx_msg <= 4'b0000;
    else if (dut2_o_ValidOutDatatREVERSALMB)
        dut1_prev_rx_msg <= dut1_i_Rx_SbMessage;
end

always @(posedge dut2_CLK or negedge dut2_rst_n) begin
    if (!dut2_rst_n)
        dut2_prev_rx_msg <= 4'b0000;
    else if (dut1_o_ValidOutDatatREVERSALMB)
        dut2_prev_rx_msg <= dut2_i_Rx_SbMessage;
end

assign dut1_i_msg_valid = dut2_o_ValidOutDatatREVERSALMB;
assign dut2_i_msg_valid = dut1_o_ValidOutDatatREVERSALMB;

////////////////////////////////////////////////////////////////////////////////
// Clocks
////////////////////////////////////////////////////////////////////////////////
initial begin dut1_CLK = 0; forever #(CLK_PERIOD/2) dut1_CLK = ~dut1_CLK; end
initial begin dut2_CLK = 0; forever #(CLK_PERIOD/2) dut2_CLK = ~dut2_CLK; end

////////////////////////////////////////////////////////////////////////////////
// Automatic Pattern Generation Done - responds after pattern enable
////////////////////////////////////////////////////////////////////////////////
reg dut1_prev_pattern_en, dut2_prev_pattern_en;

always @(posedge dut1_CLK or negedge dut1_rst_n) begin
    if (!dut1_rst_n) begin
        dut1_i_LaneID_Pattern_done <= 1'b0;
        dut1_prev_pattern_en       <= 1'b0;
    end else begin
        dut1_prev_pattern_en <= (dut1_o_MBINIT_REVERSALMB_LaneID_Pattern_En == 2'b11);
        if (!dut1_prev_pattern_en && (dut1_o_MBINIT_REVERSALMB_LaneID_Pattern_En == 2'b11))
            dut1_i_LaneID_Pattern_done <= 1'b1;
        else
            dut1_i_LaneID_Pattern_done <= 1'b0;
    end
end

always @(posedge dut2_CLK or negedge dut2_rst_n) begin
    if (!dut2_rst_n) begin
        dut2_i_LaneID_Pattern_done <= 1'b0;
        dut2_prev_pattern_en       <= 1'b0;
    end else begin
        dut2_prev_pattern_en <= (dut2_o_MBINIT_REVERSALMB_LaneID_Pattern_En == 2'b11);
        if (!dut2_prev_pattern_en && (dut2_o_MBINIT_REVERSALMB_LaneID_Pattern_En == 2'b11))
            dut2_i_LaneID_Pattern_done <= 1'b1;
        else
            dut2_i_LaneID_Pattern_done <= 1'b0;
    end
end

////////////////////////////////////////////////////////////////////////////////
// Automatic Reversal Done - responds after reversal enable
////////////////////////////////////////////////////////////////////////////////
reg dut1_prev_reversal_en, dut2_prev_reversal_en;

always @(posedge dut1_CLK or negedge dut1_rst_n) begin
    if (!dut1_rst_n) begin
        dut1_i_REVERSAL_done    <= 1'b0;
        dut1_prev_reversal_en   <= 1'b0;
    end else begin
        dut1_prev_reversal_en <= dut1_o_MBINIT_ApplyReversal_En;
        if (!dut1_prev_reversal_en && dut1_o_MBINIT_ApplyReversal_En)
            dut1_i_REVERSAL_done <= 1'b1;
        else
            dut1_i_REVERSAL_done <= 1'b0;
    end
end

always @(posedge dut2_CLK or negedge dut2_rst_n) begin
    if (!dut2_rst_n) begin
        dut2_i_REVERSAL_done    <= 1'b0;
        dut2_prev_reversal_en   <= 1'b0;
    end else begin
        dut2_prev_reversal_en <= dut2_o_MBINIT_ApplyReversal_En;
        if (!dut2_prev_reversal_en && dut2_o_MBINIT_ApplyReversal_En)
            dut2_i_REVERSAL_done <= 1'b1;
        else
            dut2_i_REVERSAL_done <= 1'b0;
    end
end

////////////////////////////////////////////////////////////////////////////////
// DUT instances
// NOTE: apply_repeater is INTERNAL to REVERSALMB_Wrapper (u1 -> u2 inside wrapper)
//       It does NOT appear as a top-level port of REVERSALMB_Wrapper
////////////////////////////////////////////////////////////////////////////////
REVERSALMB_Wrapper dut1 (
    .CLK                                (dut1_CLK),
    .rst_n                              (dut1_rst_n),
    .i_MBINIT_REPAIRVAL_end             (dut1_i_MBINIT_REPAIRVAL_end),
    .i_REVERSAL_done                    (dut1_i_REVERSAL_done),
    .i_LaneID_Pattern_done              (dut1_i_LaneID_Pattern_done),
    .i_falling_edge_busy                (dut1_i_falling_edge_busy),
    .i_Rx_SbMessage                     (dut1_i_Rx_SbMessage),
    .i_msg_valid                        (dut1_i_msg_valid),
    .i_REVERSAL_Result_SB               (dut1_i_REVERSAL_Result_SB),
    .i_REVERSAL_Result_logged           (dut1_i_REVERSAL_Result_logged),
    .o_MBINIT_REVERSALMB_LaneID_Pattern_En (dut1_o_MBINIT_REVERSALMB_LaneID_Pattern_En),
    .o_MBINIT_ApplyReversal_En          (dut1_o_MBINIT_ApplyReversal_En),
    .o_MBINIT_REVERSALMB_end            (dut1_o_MBINIT_REVERSALMB_end),
    .o_TX_SbMessage                     (dut1_o_TX_SbMessage),
    .o_Clear_Pattern_Comparator         (dut1_o_Clear_Pattern_Comparator),
    .o_REVERSAL_Pattern_Result_logged   (dut1_o_REVERSAL_Pattern_Result_logged),
    .o_ValidOutDatatREVERSALMB          (dut1_o_ValidOutDatatREVERSALMB),
    .o_ValidDataFieldParameters         (dut1_o_ValidDataFieldParameters),
    .o_train_error_req_reversalmb       (dut1_o_train_error_req_reversalmb)
);

REVERSALMB_Wrapper dut2 (
    .CLK                                (dut2_CLK),
    .rst_n                              (dut2_rst_n),
    .i_MBINIT_REPAIRVAL_end             (dut2_i_MBINIT_REPAIRVAL_end),
    .i_REVERSAL_done                    (dut2_i_REVERSAL_done),
    .i_LaneID_Pattern_done              (dut2_i_LaneID_Pattern_done),
    .i_falling_edge_busy                (dut2_i_falling_edge_busy),
    .i_Rx_SbMessage                     (dut2_i_Rx_SbMessage),
    .i_msg_valid                        (dut2_i_msg_valid),
    .i_REVERSAL_Result_SB               (dut2_i_REVERSAL_Result_SB),
    .i_REVERSAL_Result_logged           (dut2_i_REVERSAL_Result_logged),
    .o_MBINIT_REVERSALMB_LaneID_Pattern_En (dut2_o_MBINIT_REVERSALMB_LaneID_Pattern_En),
    .o_MBINIT_ApplyReversal_En          (dut2_o_MBINIT_ApplyReversal_En),
    .o_MBINIT_REVERSALMB_end            (dut2_o_MBINIT_REVERSALMB_end),
    .o_TX_SbMessage                     (dut2_o_TX_SbMessage),
    .o_Clear_Pattern_Comparator         (dut2_o_Clear_Pattern_Comparator),
    .o_REVERSAL_Pattern_Result_logged   (dut2_o_REVERSAL_Pattern_Result_logged),
    .o_ValidOutDatatREVERSALMB          (dut2_o_ValidOutDatatREVERSALMB),
    .o_ValidDataFieldParameters         (dut2_o_ValidDataFieldParameters),
    .o_train_error_req_reversalmb       (dut2_o_train_error_req_reversalmb)
);

////////////////////////////////////////////////////////////////////////////////
// State monitors (hierarchical access into wrapper internals)
////////////////////////////////////////////////////////////////////////////////
wire [3:0] dut1_mod_st = dut1.u1.CS;
wire [3:0] dut1_par_st = dut1.u2.CS;
wire [3:0] dut2_mod_st = dut2.u1.CS;
wire [3:0] dut2_par_st = dut2.u2.CS;

////////////////////////////////////////////////////////////////////////////////
// Debug monitors
////////////////////////////////////////////////////////////////////////////////
always @(posedge dut1_CLK) begin
    if (dut1_i_falling_edge_busy)
        $display("[%0t] DUT1: FALLING_EDGE_BUSY pulse", $time);
    if (dut1_o_ValidOutDatatREVERSALMB && !dut1_prev_ValidOut)
        $display("[%0t] DUT1: TX rising, msg=%b (%s)", $time, dut1_o_TX_SbMessage,
                 dut1_o_TX_SbMessage == MBINI_REVERSALMB_init_req         ? "init_req"         :
                 dut1_o_TX_SbMessage == MBINIT_REVERSALMB_init_resp        ? "init_resp"        :
                 dut1_o_TX_SbMessage == MBINIT_REVERSALMB_clear_error_req  ? "clear_error_req"  :
                 dut1_o_TX_SbMessage == MBINIT_REVERSALMB_clear_error_resp ? "clear_error_resp" :
                 dut1_o_TX_SbMessage == MBINIT_REVERSALMB_result_req       ? "result_req"       :
                 dut1_o_TX_SbMessage == MBINIT_REVERSALMB_result_resp      ? "result_resp"      :
                 dut1_o_TX_SbMessage == MBINIT_REVERSALMB_done_req         ? "done_req"         :
                 dut1_o_TX_SbMessage == MBINIT_REVERSALMB_done_resp        ? "done_resp"        : "unknown");
    if (dut1_i_msg_valid)
        $display("[%0t] DUT1: RX msg_valid, msg=%b (%s)", $time, dut1_i_Rx_SbMessage,
                 dut1_i_Rx_SbMessage == MBINI_REVERSALMB_init_req         ? "init_req"         :
                 dut1_i_Rx_SbMessage == MBINIT_REVERSALMB_init_resp        ? "init_resp"        :
                 dut1_i_Rx_SbMessage == MBINIT_REVERSALMB_clear_error_req  ? "clear_error_req"  :
                 dut1_i_Rx_SbMessage == MBINIT_REVERSALMB_clear_error_resp ? "clear_error_resp" :
                 dut1_i_Rx_SbMessage == MBINIT_REVERSALMB_result_req       ? "result_req"       :
                 dut1_i_Rx_SbMessage == MBINIT_REVERSALMB_result_resp      ? "result_resp"      :
                 dut1_i_Rx_SbMessage == MBINIT_REVERSALMB_done_req         ? "done_req"         :
                 dut1_i_Rx_SbMessage == MBINIT_REVERSALMB_done_resp        ? "done_resp"        : "unknown");
    if (dut1_mod_st !== dut1.u1.NS)
        $display("[%0t] DUT1.MOD: state %0d -> %0d", $time, dut1_mod_st, dut1.u1.NS);
    if (dut1_par_st !== dut1.u2.NS)
        $display("[%0t] DUT1.PAR: state %0d -> %0d", $time, dut1_par_st, dut1.u2.NS);
    if (dut1_o_MBINIT_REVERSALMB_LaneID_Pattern_En == 2'b11 && !dut1_prev_pattern_en)
        $display("[%0t] DUT1: Pattern Generation Enabled", $time);
    if (dut1_i_LaneID_Pattern_done)
        $display("[%0t] DUT1: Pattern Generation Done (auto-generated)", $time);
    if (dut1_o_MBINIT_ApplyReversal_En && !dut1_prev_reversal_en)
        $display("[%0t] DUT1: Apply Reversal Enabled", $time);
    if (dut1_i_REVERSAL_done)
        $display("[%0t] DUT1: Reversal Done (auto-generated)", $time);
end

always @(posedge dut2_CLK) begin
    if (dut2_i_falling_edge_busy)
        $display("[%0t] DUT2: FALLING_EDGE_BUSY pulse", $time);
    if (dut2_o_ValidOutDatatREVERSALMB && !dut2_prev_ValidOut)
        $display("[%0t] DUT2: TX rising, msg=%b (%s)", $time, dut2_o_TX_SbMessage,
                 dut2_o_TX_SbMessage == MBINI_REVERSALMB_init_req         ? "init_req"         :
                 dut2_o_TX_SbMessage == MBINIT_REVERSALMB_init_resp        ? "init_resp"        :
                 dut2_o_TX_SbMessage == MBINIT_REVERSALMB_clear_error_req  ? "clear_error_req"  :
                 dut2_o_TX_SbMessage == MBINIT_REVERSALMB_clear_error_resp ? "clear_error_resp" :
                 dut2_o_TX_SbMessage == MBINIT_REVERSALMB_result_req       ? "result_req"       :
                 dut2_o_TX_SbMessage == MBINIT_REVERSALMB_result_resp      ? "result_resp"      :
                 dut2_o_TX_SbMessage == MBINIT_REVERSALMB_done_req         ? "done_req"         :
                 dut2_o_TX_SbMessage == MBINIT_REVERSALMB_done_resp        ? "done_resp"        : "unknown");
    if (dut2_i_msg_valid)
        $display("[%0t] DUT2: RX msg_valid, msg=%b (%s)", $time, dut2_i_Rx_SbMessage,
                 dut2_i_Rx_SbMessage == MBINI_REVERSALMB_init_req         ? "init_req"         :
                 dut2_i_Rx_SbMessage == MBINIT_REVERSALMB_init_resp        ? "init_resp"        :
                 dut2_i_Rx_SbMessage == MBINIT_REVERSALMB_clear_error_req  ? "clear_error_req"  :
                 dut2_i_Rx_SbMessage == MBINIT_REVERSALMB_clear_error_resp ? "clear_error_resp" :
                 dut2_i_Rx_SbMessage == MBINIT_REVERSALMB_result_req       ? "result_req"       :
                 dut2_i_Rx_SbMessage == MBINIT_REVERSALMB_result_resp      ? "result_resp"      :
                 dut2_i_Rx_SbMessage == MBINIT_REVERSALMB_done_req         ? "done_req"         :
                 dut2_i_Rx_SbMessage == MBINIT_REVERSALMB_done_resp        ? "done_resp"        : "unknown");
    if (dut2_mod_st !== dut2.u1.NS)
        $display("[%0t] DUT2.MOD: state %0d -> %0d", $time, dut2_mod_st, dut2.u1.NS);
    if (dut2_par_st !== dut2.u2.NS)
        $display("[%0t] DUT2.PAR: state %0d -> %0d", $time, dut2_par_st, dut2.u2.NS);
    if (dut2_o_MBINIT_REVERSALMB_LaneID_Pattern_En == 2'b11 && !dut2_prev_pattern_en)
        $display("[%0t] DUT2: Pattern Generation Enabled", $time);
    if (dut2_i_LaneID_Pattern_done)
        $display("[%0t] DUT2: Pattern Generation Done (auto-generated)", $time);
    if (dut2_o_MBINIT_ApplyReversal_En && !dut2_prev_reversal_en)
        $display("[%0t] DUT2: Apply Reversal Enabled", $time);
    if (dut2_i_REVERSAL_done)
        $display("[%0t] DUT2: Reversal Done (auto-generated)", $time);
end

////////////////////////////////////////////////////////////////////////////////
// Test Tasks
////////////////////////////////////////////////////////////////////////////////

// Helper task: reset both DUTs cleanly between scenarios
task reset_duts;
    begin
        dut1_i_MBINIT_REPAIRVAL_end = 0;
        dut2_i_MBINIT_REPAIRVAL_end = 0;
        dut1_rst_n = 0;
        dut2_rst_n = 0;
        #(CLK_PERIOD*5);
        dut1_rst_n = 1;
        dut2_rst_n = 1;
        #(CLK_PERIOD*2);
    end
endtask

//--------------------------------------------------------------------------
// SCENARIO 1: Good Reversal Result (>8 ones) -> no reversal needed
//--------------------------------------------------------------------------
task test_scenario_reversal_good_result;
    begin
        $display("\n=== TEST SCENARIO 1: Good Reversal Result (>8 ones) ===");
        $display("DUT1 (Module/Transmitter) initiates, DUT2 (Partner) responds");

        dut1_i_REVERSAL_Result_SB     = 16'hFFFF; // 16 ones  > 8  -> good
        dut2_i_REVERSAL_Result_logged = 16'hFFFF;
        dut2_i_REVERSAL_Result_SB     = 16'hFFFF;
        dut1_i_REVERSAL_Result_logged = 16'hFFFF;

        #(CLK_PERIOD*2);
        dut1_i_MBINIT_REPAIRVAL_end = 1;
        dut2_i_MBINIT_REPAIRVAL_end = 1;
        $display("[%0t] MBINIT_REPAIRVAL_end asserted", $time);

        wait(dut1_o_TX_SbMessage == MBINI_REVERSALMB_init_req);
        $display("[%0t] DUT1 (Module): Sent init_req", $time);

        wait(dut2_o_TX_SbMessage == MBINIT_REVERSALMB_init_resp);
        $display("[%0t] DUT2 (Partner): Sent init_resp", $time);

        wait(dut1_o_MBINIT_REVERSALMB_end && dut2_o_MBINIT_REVERSALMB_end);
        $display("[%0t] Both modules completed successfully", $time);

        #(CLK_PERIOD*10);
    end
endtask

//--------------------------------------------------------------------------
// SCENARIO 2: Bad Reversal Result (<=8 ones) -> apply reversal
//--------------------------------------------------------------------------
task test_scenario_reversal_bad_result;
    begin
        $display("\n=== TEST SCENARIO 2: Bad Reversal Result (<=8 ones) - Apply Reversal ===");

        dut1_i_REVERSAL_Result_SB     = 16'h00FF; // 8 ones  -> needs reversal
        dut2_i_REVERSAL_Result_logged = 16'h00FF;
        dut2_i_REVERSAL_Result_SB     = 16'hFFFF;
        dut1_i_REVERSAL_Result_logged = 16'hFFFF;

        #(CLK_PERIOD*2);
        dut1_i_MBINIT_REPAIRVAL_end = 1;
        dut2_i_MBINIT_REPAIRVAL_end = 1;
        $display("[%0t] MBINIT_REPAIRVAL_end asserted", $time);

        wait(dut1_o_MBINIT_ApplyReversal_En);
        $display("[%0t] DUT1: Reversal application started", $time);

        wait(dut1_o_MBINIT_REVERSALMB_end && dut2_o_MBINIT_REVERSALMB_end);
        $display("[%0t] Both modules completed after reversal", $time);

        #(CLK_PERIOD*10);
    end
endtask

//--------------------------------------------------------------------------
// SCENARIO 3: Very Bad Result (<8 ones) after reversal -> error expected
//--------------------------------------------------------------------------
task test_scenario_reversal_very_bad;
    begin
        $display("\n=== TEST SCENARIO 3: Very Bad Result (<8 ones) - Error Expected ===");

        dut1_i_REVERSAL_Result_SB     = 16'h0007; // 3 ones  < 8  -> error
        dut2_i_REVERSAL_Result_logged = 16'h0007;
        dut2_i_REVERSAL_Result_SB     = 16'hFFFF;
        dut1_i_REVERSAL_Result_logged = 16'hFFFF;

        #(CLK_PERIOD*2);
        dut1_i_MBINIT_REPAIRVAL_end = 1;
        dut2_i_MBINIT_REPAIRVAL_end = 1;
        $display("[%0t] MBINIT_REPAIRVAL_end asserted", $time);

        #(CLK_PERIOD*100);

        if (dut1_o_train_error_req_reversalmb)
            $display("[%0t] *** Train Error Detected as Expected ***", $time);
        else
            $display("[%0t] WARNING: Expected train error not seen after 100 cycles", $time);
    end
endtask

//--------------------------------------------------------------------------
// SCENARIO 4: Mixed Results (DUT1 good, DUT2 good)
//--------------------------------------------------------------------------
task test_scenario_mixed_results;
    begin
        $display("\n=== TEST SCENARIO 4: Mixed Results ===");

        dut1_i_REVERSAL_Result_SB     = 16'hFFF0; // 12 ones -> good
        dut2_i_REVERSAL_Result_logged = 16'hFFF0;
        dut2_i_REVERSAL_Result_SB     = 16'hFFFF;
        dut1_i_REVERSAL_Result_logged = 16'hFFFF;

        #(CLK_PERIOD*2);
        dut1_i_MBINIT_REPAIRVAL_end = 1;
        dut2_i_MBINIT_REPAIRVAL_end = 1;
        $display("[%0t] MBINIT_REPAIRVAL_end asserted", $time);

        wait(dut1_o_MBINIT_REVERSALMB_end && dut2_o_MBINIT_REVERSALMB_end);
        $display("[%0t] Both modules completed", $time);

        #(CLK_PERIOD*10);
    end
endtask

////////////////////////////////////////////////////////////////////////////////
// Main test sequence
////////////////////////////////////////////////////////////////////////////////
initial begin
    // Initialize all signals
    dut1_rst_n                    = 0;
    dut1_i_MBINIT_REPAIRVAL_end   = 0;
    dut1_i_REVERSAL_Result_SB     = 16'h0000;
    dut1_i_REVERSAL_Result_logged = 16'h0000;

    dut2_rst_n                    = 0;
    dut2_i_MBINIT_REPAIRVAL_end   = 0;
    dut2_i_REVERSAL_Result_SB     = 16'h0000;
    dut2_i_REVERSAL_Result_logged = 16'h0000;

    #(CLK_PERIOD*5);
    dut1_rst_n = 1;
    dut2_rst_n = 1;
    #(CLK_PERIOD*2);

    $display("\n=== REVERSALMB WRAPPER TEST START ===");
    $display("NOTE: apply_repeater is INTERNAL to REVERSALMB_Wrapper (not a top-level port)");
    $display("      ValidOutDatatREVERSALMB stays HIGH while in SEND states");
    $display("      falling_edge_busy PULSES 2 cycles after valid rises");
    $display("      Pattern and Reversal done signals are AUTO-GENERATED\n");

    // ---- Select / run test scenarios ----
    // Uncomment the ones you want to run; use reset_duts between consecutive runs.

    test_scenario_reversal_bad_result;
    // reset_duts; test_scenario_reversal_good_result;
    // reset_duts; test_scenario_reversal_very_bad;
    // reset_duts; test_scenario_mixed_results;

    $display("\n=== FINAL RESULTS ===");
    $display("DUT1: Mod_st=%0d  Par_st=%0d  End=%b  Train_Error=%b",
             dut1_mod_st, dut1_par_st,
             dut1_o_MBINIT_REVERSALMB_end,
             dut1_o_train_error_req_reversalmb);
    $display("DUT2: Mod_st=%0d  Par_st=%0d  End=%b  Train_Error=%b",
             dut2_mod_st, dut2_par_st,
             dut2_o_MBINIT_REVERSALMB_end,
             dut2_o_train_error_req_reversalmb);

    if (dut1_o_MBINIT_REVERSALMB_end && dut2_o_MBINIT_REVERSALMB_end &&
        !dut1_o_train_error_req_reversalmb && !dut2_o_train_error_req_reversalmb)
        $display("\n*** PASS ***\n");
    else if (dut1_o_train_error_req_reversalmb || dut2_o_train_error_req_reversalmb)
        $display("\n*** ERROR DETECTED (may be expected for scenario 3) ***\n");
    else
        $display("\n*** FAIL / INCOMPLETE ***\n");

    #(CLK_PERIOD*10);
    $finish;
end

////////////////////////////////////////////////////////////////////////////////
// Waveform dump
////////////////////////////////////////////////////////////////////////////////
initial begin
    $dumpfile("tb_REVERSALMB_Wrapper.vcd");
    $dumpvars(0, tb_REVERSALMB_Wrapper);
end

////////////////////////////////////////////////////////////////////////////////
// Global timeout
////////////////////////////////////////////////////////////////////////////////
initial begin
    #(CLK_PERIOD*3000);
    $display("\n*** TIMEOUT - Simulation exceeded 3000 cycles ***\n");
    $finish;
end

endmodule