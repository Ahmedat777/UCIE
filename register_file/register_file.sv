// -----------------------------------------------------------------------------
// ucie_link_capability_reg
// -----------------------------------------------------------------------------
module ucie_link_capability_reg #(
    parameter logic [2:0] MAX_LINK_WIDTH        = 3'd0,
    parameter logic       APMW                  = 1'b0,
    parameter logic       SPMW                  = 1'b0,

    parameter logic RAW_FORMAT_SUPPORTED        = 1'b0,
    parameter logic RETIMER_PRESENT             = 1'b0,
    parameter logic MULTI_PROTOCOL              = 1'b0,
    parameter logic ADVANCED_PACKAGING          = 1'b0,
    parameter logic FLIT_68B_STREAM             = 1'b0,
    parameter logic STREAM_END_HDR256           = 1'b0,
    parameter logic STREAM_START_HDR256         = 1'b0,
    parameter logic LAT_OPT_256_NO_OPT          = 1'b0,
    parameter logic LAT_OPT_256_OPT             = 1'b0,
    parameter logic ENH_MULTI_PROTOCOL          = 1'b0,
    parameter logic PCIE_START_HDR              = 1'b0,
    parameter logic PCIE_LAT_OPT               = 1'b0,
    parameter logic RUNTIME_PARITY_ERR          = 1'b0,
    parameter logic X32_IN_X64_SUPPORTED        = 1'b0,
    parameter logic SIDEBAND_PMO                = 1'b0,
    parameter logic PSPT_SUPPORTED              = 1'b0,
    parameter logic L2SPD_SUPPORTED             = 1'b0
)(
    input  logic        clk,
    input  logic        rst_n,
    output logic [31:0] rd_data
);

    logic [2:0] max_link_width_q;
    logic [3:0] max_link_speed_q;
    logic       apmw_q;
    logic       spmw_q;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            max_link_width_q <= MAX_LINK_WIDTH;
            max_link_speed_q <= 4'b0;
            apmw_q           <= APMW;
            spmw_q           <= SPMW;
        end
    end

    always_comb begin
        rd_data        = 32'b0;
        rd_data[0]     = RAW_FORMAT_SUPPORTED;
        rd_data[3:1]   = max_link_width_q;
        rd_data[7:4]   = max_link_speed_q;
        rd_data[8]     = RETIMER_PRESENT;
        rd_data[9]     = MULTI_PROTOCOL;
        rd_data[10]    = ADVANCED_PACKAGING;
        rd_data[11]    = FLIT_68B_STREAM;
        rd_data[12]    = STREAM_END_HDR256;
        rd_data[13]    = STREAM_START_HDR256;
        rd_data[14]    = LAT_OPT_256_NO_OPT;
        rd_data[15]    = LAT_OPT_256_OPT;
        rd_data[16]    = ENH_MULTI_PROTOCOL;
        rd_data[17]    = PCIE_START_HDR;
        rd_data[18]    = PCIE_LAT_OPT;
        rd_data[19]    = RUNTIME_PARITY_ERR;
        rd_data[20]    = apmw_q;
        rd_data[21]    = X32_IN_X64_SUPPORTED;
        rd_data[22]    = spmw_q;
        rd_data[23]    = SIDEBAND_PMO;
        rd_data[24]    = PSPT_SUPPORTED;
        rd_data[25]    = L2SPD_SUPPORTED;
        // [31:26] Reserved
    end

endmodule


// -----------------------------------------------------------------------------
// ucie_link_control_reg
// -----------------------------------------------------------------------------
module ucie_link_control_reg (
    input  logic        clk,
    input  logic        rst_n,

    input  logic        wr_en,
    input  logic [31:0] wr_data,

    input  logic [31:0] cap_reg,

    input  logic        training_done,
    input  logic        retrain_done,
    input  logic        link_up,

    output logic [31:0] rd_data
);

    logic       raw_format_en;
    logic       multi_protocol_en;
    logic [3:0] target_width;
    logic [3:0] target_speed;
    logic       start_training_bit;
    logic       retrain_bit;
    logic       flit68_en;
    logic       stream_end_en;
    logic       stream_start_en;
    logic       latopt_noopt_en;
    logic       latopt_opt_en;
    logic       enh_multi_en;
    logic       pcie_start_en;
    logic       pcie_latopt_en;
    logic       sb_pmo_en;
    logic       pspt_en;
    logic       l2spd_en;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            raw_format_en      <= 1'b0;
            multi_protocol_en  <= cap_reg[9];
            target_width       <= cap_reg[3:1];
            target_speed       <= cap_reg[7:4];
            start_training_bit <= 1'b0;
            retrain_bit        <= 1'b0;
            // capability-gated defaults
            flit68_en          <= cap_reg[11];
            stream_end_en      <= cap_reg[12];
            stream_start_en    <= cap_reg[13];
            latopt_noopt_en    <= cap_reg[14];
            latopt_opt_en      <= cap_reg[15];
            enh_multi_en       <= cap_reg[16];
            pcie_start_en      <= cap_reg[17];
            pcie_latopt_en     <= cap_reg[18];
            sb_pmo_en          <= cap_reg[23];
            pspt_en            <= cap_reg[24];
            l2spd_en           <= cap_reg[25];
        end
        else begin
            if (wr_en) begin
                raw_format_en      <= wr_data[0];
                multi_protocol_en  <= wr_data[1];
                target_width       <= wr_data[5:2];
                target_speed       <= wr_data[9:6];
                start_training_bit <= wr_data[10];
                retrain_bit        <= wr_data[11];
                // capability masking applied on write
                flit68_en          <= wr_data[13] & cap_reg[11];
                stream_end_en      <= wr_data[14] & cap_reg[12];
                stream_start_en    <= wr_data[15] & cap_reg[13];
                latopt_noopt_en    <= wr_data[16] & cap_reg[14];
                latopt_opt_en      <= wr_data[17] & cap_reg[15];
                enh_multi_en       <= wr_data[18] & cap_reg[16];
                pcie_start_en      <= wr_data[19] & cap_reg[17];
                pcie_latopt_en     <= wr_data[20] & cap_reg[18];
                sb_pmo_en          <= wr_data[21] & cap_reg[23];
                pspt_en            <= wr_data[22] & cap_reg[24];
                l2spd_en           <= wr_data[23] & cap_reg[25];
            end

            // auto-clear
            if (training_done) start_training_bit <= 1'b0;
            if (retrain_done)  retrain_bit        <= 1'b0;
        end
    end

    always_comb begin
        rd_data        = 32'b0;
        rd_data[0]     = raw_format_en;
        rd_data[1]     = multi_protocol_en;
        rd_data[5:2]   = target_width;
        rd_data[9:6]   = target_speed;
        rd_data[10]    = start_training_bit;
        rd_data[11]    = retrain_bit;
        // [12] Reserved
        rd_data[13]    = flit68_en;
        rd_data[14]    = stream_end_en;
        rd_data[15]    = stream_start_en;
        rd_data[16]    = latopt_noopt_en;
        rd_data[17]    = latopt_opt_en;
        rd_data[18]    = enh_multi_en;
        rd_data[19]    = pcie_start_en;
        rd_data[20]    = pcie_latopt_en;
        rd_data[21]    = sb_pmo_en;
        rd_data[22]    = pspt_en;
        rd_data[23]    = l2spd_en;
        // [31:24] Reserved
    end

endmodule


// -----------------------------------------------------------------------------
// ucie_link_status_reg
// -----------------------------------------------------------------------------
module ucie_link_status_reg #(
    parameter logic        LINK_UP                    = 1'b0,
    parameter logic        RAW_FORMAT_EN              = 1'b0,
    parameter logic        MULTI_PROTOCOL_EN          = 1'b0,
    parameter logic        ENHANCED_MULTI_PROTOCOL_EN = 1'b0,
    parameter logic        X32_MODE                   = 1'b0,
    parameter logic [3:0]  FLIT_FORMAT                = 4'h0,
    parameter logic        SB_PMO                     = 1'b0,
    parameter logic        PSPT                       = 1'b0,
    parameter logic        L2SPD                      = 1'b0
)(
    input  logic        clk,
    input  logic        rst_n,

    // HW set events
    input  logic        correctable_err_set,   // correctable error from error_handler
    input  logic        uncorrect_nf_set,      // non-fatal error from error_handler
    input  logic        uncorrect_fatal_set,   // fatal/training error from error_handler

    // SW clear interface
    input  logic        wr_en,
    input  logic [31:0] wr_data,

    output logic [31:0] rd_data
);

    logic corr_err;
    logic unc_nf_err;
    logic unc_fatal_err;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            corr_err      <= 1'b0;
            unc_nf_err    <= 1'b0;
            unc_fatal_err <= 1'b0;
        end
        else begin
            if (correctable_err_set)  corr_err      <= 1'b1;
            if (uncorrect_nf_set)     unc_nf_err    <= 1'b1;
            if (uncorrect_fatal_set)  unc_fatal_err <= 1'b1;

            // RW1C: SW writes 1 to clear
            if (wr_en) begin
                if (wr_data[19]) corr_err      <= 1'b0;
                if (wr_data[20]) unc_nf_err    <= 1'b0;
                if (wr_data[21]) unc_fatal_err <= 1'b0;
            end
        end
    end

    assign rd_data = {
        3'b000,                     // [31:29] RsvdZ
        L2SPD,                      // [28]
        PSPT,                       // [27]
        SB_PMO,                     // [26]
        FLIT_FORMAT,                // [25:22]
        unc_fatal_err,              // [21]
        unc_nf_err,                 // [20]
        corr_err,                   // [19]
        2'b00,                      // [18:17]
        1'b0,                       // [16] RsvdZ
        LINK_UP,                    // [15]
        4'b0000,                    // [14:11] RsvdZ
        4'b0000,                    // [10:7]  RsvdZ
        3'b000,                     // [6:4]
        X32_MODE,                   // [3]
        ENHANCED_MULTI_PROTOCOL_EN, // [2]
        MULTI_PROTOCOL_EN,          // [1]
        RAW_FORMAT_EN               // [0]
    };

endmodule


// -----------------------------------------------------------------------------
// phy_capability_reg
// -----------------------------------------------------------------------------
module phy_capability_reg #(
    parameter TERMINATED_LINK    = 1'b1,
    parameter TXEQ_SUPPORTED     = 1'b1,
    parameter TX_VSWING_ENCODING = 5'h08,
    parameter RX_CLK_MODE_SUP    = 2'b00,
    parameter RX_CLK_PHASE_SUP   = 2'b01,
    parameter PACKAGE_TYPE       = 1'b0,
    parameter TCM_SUPPORTED      = 1'b1,
    parameter TARR_SUPPORTED     = 1'b1
)(
    output logic [31:0] rd_data
);

    assign rd_data = {
        14'b0,               // [31:18] RsvdP
        TARR_SUPPORTED,      // [17]
        TCM_SUPPORTED,       // [16]
        PACKAGE_TYPE,        // [15]
        RX_CLK_PHASE_SUP,    // [14:13]
        RX_CLK_MODE_SUP,     // [12:11]
        1'b0,                // [10] Reserved
        TX_VSWING_ENCODING,  // [9:5]
        TXEQ_SUPPORTED,      // [4]
        TERMINATED_LINK,     // [3]
        3'b000               // [2:0] Reserved
    };

endmodule


// -----------------------------------------------------------------------------
// phy_control_reg
// -----------------------------------------------------------------------------
module phy_control_reg #(
    parameter CAP_TERMINATED_LINK  = 1'b1,
    parameter CAP_TXEQ_SUPPORTED   = 1'b1,
    parameter CAP_RX_CLKMODE_SUP   = 2'b00,
    parameter CAP_RX_CLKPHASE_SUP  = 2'b01,
    parameter CAP_PACKAGE_TYPE     = 1'b0
)(
    input  logic        clk,
    input  logic        rst_n,

    input  logic        wr_en,
    input  logic [31:0] wr_data,

    output logic [31:0] rd_data
);

    logic [31:0] ctrl_reg;

    localparam logic DEFAULT_CLKMODE =
        (CAP_RX_CLKMODE_SUP == 2'b10) ? 1'b1 : 1'b0;

    // Helper functions for capability enforcement
    // Applied at write time so stored value is already the effective value
    function automatic logic enforce_rx_term(input logic val);
        return (CAP_PACKAGE_TYPE == 1'b0) ? 1'b0 : val;
    endfunction

    function automatic logic enforce_txeq(input logic val);
        return CAP_TXEQ_SUPPORTED ? val : 1'b0;
    endfunction

    function automatic logic enforce_clkmode(input logic val);
        return (CAP_RX_CLKMODE_SUP == 2'b10) ? 1'b1 : val;
    endfunction

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            ctrl_reg      <= '0;
            ctrl_reg[3]   <= enforce_rx_term(CAP_TERMINATED_LINK);
            ctrl_reg[4]   <= enforce_txeq(1'b0);
            ctrl_reg[5]   <= DEFAULT_CLKMODE;
            ctrl_reg[6]   <= 1'b0;
        end
        else if (wr_en) begin
            // capability enforcement applied at write time
            ctrl_reg[3]     <= enforce_rx_term(wr_data[3]);
            ctrl_reg[4]     <= enforce_txeq(wr_data[4]);
            ctrl_reg[5]     <= enforce_clkmode(wr_data[5]);
            ctrl_reg[6]     <= wr_data[6];
            ctrl_reg[7]     <= wr_data[7];
            ctrl_reg[8]     <= wr_data[8];
            ctrl_reg[9]     <= wr_data[9];
            ctrl_reg[15:10] <= wr_data[15:10];
            ctrl_reg[16]    <= wr_data[16];
            ctrl_reg[20:17] <= wr_data[20:17];
            ctrl_reg[21]    <= wr_data[21];
            // [2:0] and [31:22] remain zero
        end
    end

    assign rd_data = {
        10'b0,           // [31:22] Reserved
        ctrl_reg[21],    // tarr_enable
        ctrl_reg[20:17], // txeq_force_setting
        ctrl_reg[16],    // txeq_force_en
        ctrl_reg[15:10], // iq_force_param
        ctrl_reg[9],     // iq_force_en
        ctrl_reg[8],     // force_x8
        ctrl_reg[7],     // force_x32
        ctrl_reg[6],     // clk_phase_sel
        ctrl_reg[5],     // clk_mode_sel (enforced)
        ctrl_reg[4],     // tx_eq_en     (enforced)
        ctrl_reg[3],     // rx_term_en   (enforced)
        3'b000           // [2:0] Reserved
    };

endmodule


// -----------------------------------------------------------------------------
// phy_status_reg
// -----------------------------------------------------------------------------
module phy_status_reg (
    input  logic        rx_termination_en,
    input  logic        tx_eq_en,
    input  logic        clk_mode,
    input  logic        clk_phase,
    input  logic        lane_reversal,
    input  logic [5:0]  iq_corr_param,
    input  logic [4:0]  eq_preset,
    input  logic        tarr_operational,

    output logic [31:0] rd_data
);

    assign rd_data = {
        13'b0,             // [31:19] RsvdP
        tarr_operational,  // [18]
        eq_preset,         // [17:14]
        iq_corr_param,     // [13:8]
        lane_reversal,     // [7]
        clk_phase,         // [6]
        clk_mode,          // [5]
        tx_eq_en,          // [4]
        rx_termination_en, // [3]
        3'b000             // [2:0] Reserved
    };

endmodule


// -----------------------------------------------------------------------------
// training_setup1_reg
// -----------------------------------------------------------------------------
module training_setup1_reg (
    input  logic        clk,
    input  logic        rst_n,

    input  logic        wr_en,
    input  logic [31:0] wr_data,
    output logic [31:0] rd_data
);

    logic [31:0] reg_q;

    localparam [31:0] RESET_VALUE = {
        5'b0,      // [31:27] reserved
        16'h0004,  // [26:11] burst_count default = 4
        1'b0,      // [10]    training_mode
        4'h0,      // [9:6]   clock_phase
        3'b000,    // [5:3]   valid_pattern
        3'b000     // [2:0]   data_pattern
    };

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            reg_q <= RESET_VALUE;
        else if (wr_en) begin
            reg_q[26:0]  <= wr_data[26:0];
            reg_q[31:27] <= '0; // reserved forced zero
        end
    end

    assign rd_data = reg_q;

endmodule


// -----------------------------------------------------------------------------
// training_setup2_reg
// -----------------------------------------------------------------------------
module training_setup2_reg (
    input  logic        clk,
    input  logic        rst_n,

    input  logic        wr_en,
    input  logic [31:0] wr_data,
    output logic [31:0] rd_data
);

    logic [31:0] reg_q;

    localparam [31:0] RESET_VALUE = {16'h0004, 16'h0004};

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            reg_q <= RESET_VALUE;
        else if (wr_en)
            reg_q <= wr_data;
    end

    assign rd_data = reg_q;

endmodule


// -----------------------------------------------------------------------------
// training_setup3_reg
// -----------------------------------------------------------------------------
module training_setup3_reg (
    input  logic        clk,
    input  logic        rst_n,

    input  logic        wr_en,
    input  logic [63:0] wr_data,
    output logic [63:0] rd_data
);

    logic [63:0] reg_q;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            reg_q <= '0;
        else if (wr_en)
            reg_q <= wr_data;
    end

    assign rd_data = reg_q;

endmodule


// -----------------------------------------------------------------------------
// training_setup4_reg
// -----------------------------------------------------------------------------
module training_setup4_reg (
    input  logic        clk,
    input  logic        rst_n,

    input  logic        wr_en,
    input  logic [31:0] wr_data,
    output logic [31:0] rd_data
);

    logic [31:0] reg_q;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            reg_q <= '0;
        else if (wr_en)
            reg_q <= wr_data;
    end

    assign rd_data = reg_q;

endmodule


// -----------------------------------------------------------------------------
// error_log1_reg
// -----------------------------------------------------------------------------
module error_log1_reg (
    input  logic        clk,
    input  logic        rst_n,

    input  logic        wr_en,
    input  logic [31:0] wr_data,

    input  logic [7:0]  state_n3_hw,
    input  logic        state_timeout_hw,
    input  logic        sb_timeout_hw,
    input  logic        remote_linkerr_hw,
    input  logic        internal_err_hw,

    output logic [31:0] rd_data
);

    // Sticky RW1CS bits [11:8]
    logic [3:0] sticky;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            sticky <= '0;
        else begin
            // HW sets
            if (state_timeout_hw)  sticky[0] <= 1'b1;
            if (sb_timeout_hw)     sticky[1] <= 1'b1;
            if (remote_linkerr_hw) sticky[2] <= 1'b1;
            if (internal_err_hw)   sticky[3] <= 1'b1;

            // SW clears (RW1C)
            if (wr_en) begin
                if (wr_data[8])  sticky[0] <= 1'b0;
                if (wr_data[9])  sticky[1] <= 1'b0;
                if (wr_data[10]) sticky[2] <= 1'b0;
                if (wr_data[11]) sticky[3] <= 1'b0;
            end
        end
    end

    assign rd_data = {
        20'b0,       // [31:12] Reserved
        sticky[3],   // [11] internal_err
        sticky[2],   // [10] remote_linkerr
        sticky[1],   // [9]  sb_timeout
        sticky[0],   // [8]  state_timeout
        state_n3_hw  // [7:0] live ROS field
    };

endmodule


// -----------------------------------------------------------------------------
// runtime_link_test_ctrl_reg
// -----------------------------------------------------------------------------
module runtime_link_test_ctrl_reg (
    input  logic        clk,
    input  logic        rst_n,

    input  logic        wr_en,
    input  logic [63:0] wr_data,

    input  logic        busy_set_hw,

    output logic [63:0] rd_data
);

    logic [63:0] reg_q;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            reg_q <= '0;
        else begin
            if (wr_en) begin
                reg_q[5:0]   <= wr_data[5:0];
                reg_q[6]     <= wr_data[6];
                reg_q[7]     <= wr_data[7];
                reg_q[14:8]  <= wr_data[14:8];
                reg_q[21:15] <= wr_data[21:15];
                reg_q[28:22] <= wr_data[28:22];
                reg_q[35:29] <= wr_data[35:29];
                reg_q[63:36] <= '0; // reserved
            end

            // HW clears start_cmd once accepted
            if (busy_set_hw)
                reg_q[6] <= 1'b0;
        end
    end

    assign rd_data = reg_q;

endmodule


// -----------------------------------------------------------------------------
// runtime_link_test_status_reg
// -----------------------------------------------------------------------------
module runtime_link_test_status_reg (
    input  logic        clk,
    input  logic        rst_n,

    input  logic        busy_hw,

    output logic [31:0] rd_data
);

    logic busy_q;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            busy_q <= 1'b0;
        else
            busy_q <= busy_hw;
    end

    assign rd_data = {31'b0, busy_q};

endmodule