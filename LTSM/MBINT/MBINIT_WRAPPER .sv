import pckg::*;

module MBINIT_WRAPPER
(   ////////////////////////////////////////////////////////////
    // inputs
    ///////////////////////////////////////////////////////////
    input                               i_clk ,
    input                               i_rst_n ,
    input                               i_MBINIT_en ,
    input                               i_sb_busy ,
    input                               i_falling_edge_busy ,
    input   sb_msg_id                   i_decoded_sb_msg ,
    input                               i_sb_valid ,
    input   [2:0]                       i_msg_info ,
    input   [15:0]                      i_data_bus ,

    // -- Register-file inputs (PARAM) --------------------
    input   wire [3:0]                  i_rf_data_rate,
    input   wire [4:0]                  i_rf_vswing,
    input   wire                        i_rf_clk_mode,
    input   wire                        i_rf_clk_phase,
    input   wire [1:0]                  i_rf_module_id,
    input   wire                        i_rf_ucie_sx8,
    input   wire                        i_rf_sfes,
    input   wire                        i_rf_tarr,

    //------ REPAIRCLK_INPUTS ------------------------
    input                               i_clk_ptrn_done,
    input   [2:0]                       i_logged_results_COMP,

    //------ REPAIRVAL_INPUTS ------------------------
    input   wire                          i_VAL_Pattern_done,
    input   wire                          i_VAL_Result_logged_COMB,

    //------ REVERSALMB_INPUTS ------------------------
    input   wire                          i_REVERSAL_done,
    input                                 i_LaneID_Pattern_done,
    input   wire [15:0]                   i_REVERSAL_Result_logged,

     //------ REPAIRMB_INPUTS ------------------------
    input  wire                             i_d2c_tx_ack,
    input   wire [15:0]                     i_lanes_results_tx,

    ////////////////////////////////////////////////////////////
    // outputs
    ///////////////////////////////////////////////////////////
    output  reg  sb_msg_id              o_encoded_SB_msg ,
    output  reg                         o_msg_valid ,
    output  reg  [15:0]                 o_data_bus ,
    output  reg  [2:0]                  o_msg_info ,
    output  reg                         o_MBINIT_END ,
    output  reg                        o_error_req ,
    output  reg                        o_valid_datafield ,

    // -- Register-file outputs (PARAM) --------------------
    output  wire [3:0]                  o_final_max_data_rate,       //  resolved data rate (tx)
    output  wire [4:0]                  o_module_vswing,                // partner TX vswing (set Vref)
    output  wire                        o_module_clk_mode,              // partner TX clock mode
    output  wire                        o_module_clk_phase,             // partner TX clock phase

    //------ REPAIRCLK_IOUTPUTS ------------------------
    output  wire                        o_clk_ptrn_en,
    output  wire                        o_clear_log_clk ,
    //------ REPAIRVAL_OUTPUTS ------------------------
    output   wire                       o_MBINIT_REPAIRVAL_Pattern_En,
    output   wire                       o_enable_16_iterations,
    //------ REVERSALMB_OUTPUTS ------------------------
    output   wire [1:0]                 o_MBINIT_REVERSALMB_LaneID_Pattern_En,
    output   wire                       o_MBINIT_ApplyReversal_En,
    output   wire [1:0]                 o_Clear_Pattern_Comparator_reversal,
    //------ REPAIRMB_OUTPUTS ------------------------
    output  wire  [1:0]                 o_Functional_Lanes_out_tx,
    output  wire  [1:0]                 o_Functional_Lanes_out_rx,
    output  wire                        o_Transmitter_initiated_D2C_en,
    output  wire                        o_perlane_Transmitter_initiated_D2C,
    output  wire                        o_mainband_Transmitter_initiated_D2C

);

/////////////////////////////////////////////////////////////////////////////
// internal signals
/////////////////////////////////////////////////////////////////////////////

// -- PARAM_WRAPPER outputs / inter-module signals --
sb_msg_id                    encoded_SB_msg_param;
logic                         msg_valid_param;
logic  [15:0]                 data_bus_param;
logic                         data_valid_param;
logic  [3:0]                  final_max_data_rate_tx;
logic  [3:0]                  final_max_data_rate_rx;
logic                         error_req_param;
logic                         PARAM_END;

// -- CAL_Module outputs / inter-module signals --
sb_msg_id                    encoded_SB_msg_cal;
logic                         msg_valid_cal;
logic                         CAL_END;

// -- REPAIRCLK_WRAPPER outputs / inter-module signals --
sb_msg_id                    encoded_sb_msg_repairclk;
logic                         msg_valid_repairclk;
logic  [2:0]                  msg_info_repairclk;
logic                         error_req_repairclk;
logic                         REPAIRCLK_END;

// -- REPAIRVAL_Wrapper outputs / inter-module signals --
sb_msg_id                    encoded_sb_msg_repairval;
logic                         msg_valid_repairval;
logic                         error_req_repairval;
logic                         msg_info_repairval;
logic                         REPAIRVAL_END;

// -- REVERSALMB_Wrapper outputs / inter-module signals --
sb_msg_id                    encoded_sb_msg_reversalmb;
logic                         msg_valid_reversalmb;
logic  [15:0]                 data_bus_reversalmb;
logic                         data_valid_reversal;
logic                         error_req_reversalmb;
logic                         REVERSAL_END;

// -- REPAIRMB_Wrapper outputs / inter-module signals --
sb_msg_id                    encoded_sb_msg_repairmb;
logic                         msg_valid_repairmb;
logic                         data_valid_repairmb;
logic                         error_req_repairmb;
logic  [2:0]                  msg_info_repairmb;
logic                         REPAIRMB_END;
/////////////////////////////////////////////////////////////////////////////
// instances
/////////////////////////////////////////////////////////////////////////////
PARAM_WRAPPER_fin dut_param
(
    .i_clk(i_clk),
    .i_rst_n(i_rst_n),
    .i_PARAM_en(i_MBINIT_en),         
    .i_sb_busy(i_sb_busy),
    .i_falling_edge_busy(i_falling_edge_busy),
    .i_sb_valid(i_sb_valid),
    .i_decoded_sb_msg(i_decoded_sb_msg),
    .i_parameters(i_data_bus),  

    .i_rf_data_rate(i_rf_data_rate),
    .i_rf_vswing(i_rf_vswing),
    .i_rf_clk_mode(i_rf_clk_mode),
    .i_rf_clk_phase(i_rf_clk_phase),
    .i_rf_module_id(i_rf_module_id),
    .i_rf_ucie_sx8(i_rf_ucie_sx8),
    .i_rf_sfes(i_rf_sfes),
    .i_rf_tarr(i_rf_tarr),

    .o_encoded_SB_msg(encoded_SB_msg_param),
    .o_msg_valid(msg_valid_param),
    .o_parameters(data_bus_param), 
    .o_data_valid (data_valid_param),
    .o_final_max_data_rate_tx(final_max_data_rate_tx),
    .o_error_req(error_req_param),            

    .o_module_vswing(o_module_vswing),        
    .o_module_clk_mode(o_module_clk_mode),     
    .o_module_clk_phase(o_module_clk_phase),     
    .o_final_max_data_rate_rx(final_max_data_rate_rx),

    .o_PARAM_END(PARAM_END) 
);

CAL_ModuleWrapper dut_cal
(
    .CLK(i_clk),
    .rst_n(i_rst_n),
    .i_MBINIT_PARAM_end(PARAM_END),
    .i_falling_edge_busy(i_falling_edge_busy),
    //.i_Busy_SideBand(i_sb_busy),
    .i_RX_SbMessage(i_decoded_sb_msg),
    .i_msg_valid(i_sb_valid),
    
    .o_TX_SbMessage(encoded_SB_msg_cal),
    .o_ValidOutDatatCAL(msg_valid_cal),
    .o_MBINIT_CAL_end(CAL_END)
);

REPAIRCLK_WRAPPER dut_repairclk
(
    .i_clk(i_clk),
    .i_rst_n(i_rst_n),
    .i_mbinit_rpairclk_en(CAL_END),
    .i_clk_ptrn_done(i_clk_ptrn_done),
    .i_decoded_sb_msg(i_decoded_sb_msg),
    .i_sb_busy (i_sb_busy),
    .i_falling_edge_busy(i_falling_edge_busy) ,
    .i_sb_valid(i_sb_valid),   
    .i_logged_results_SB(i_msg_info), 
    .i_logged_results_COMP(i_logged_results_COMP), 

    .o_error_req(error_req_repairclk),
    .o_clk_ptrn_en(o_clk_ptrn_en),
    .o_MBINIT_REPAIRCLK_end(REPAIRCLK_END),
    .o_encoded_sb_msg(encoded_sb_msg_repairclk),
    .o_logged_rx (msg_info_repairclk),
    .o_clear_log (o_clear_log_clk),
    .o_msg_valid(msg_valid_repairclk)
);

REPAIRVAL_Wrapper dut_repairval
(
    .CLK(i_clk),
    .rst_n(i_rst_n),
    .i_REPAIRCLK_end(REPAIRCLK_END),
    .i_VAL_Pattern_done(i_VAL_Pattern_done),
    .i_Rx_SbMessage(i_decoded_sb_msg),
    .i_msg_valid(i_sb_valid),
    .i_falling_edge_busy(i_falling_edge_busy),
    .i_VAL_Result_logged_RXSB(i_msg_info[0]),
    .i_VAL_Result_logged_COMB(i_VAL_Result_logged_COMB),
    
    .o_train_error_req(error_req_repairval),
    .o_MBINIT_REPAIRVAL_Pattern_En(o_MBINIT_REPAIRVAL_Pattern_En),
    .o_MBINIT_REPAIRVAL_end(REPAIRVAL_END),
    .o_TX_SbMessage(encoded_sb_msg_repairval),
    .o_VAL_128Result_logged(msg_info_repairval),
    .o_enable_16_iterations(o_enable_16_iterations),
    .o_ValidOutData(msg_valid_repairval)
);

 REVERSALMB_Wrapper dut_reversalmb
 (
    .CLK(i_clk),
    .rst_n(i_rst_n),
    .i_MBINIT_REPAIRVAL_end(REPAIRVAL_END),
    .i_REVERSAL_done(i_REVERSAL_done),
    .i_LaneID_Pattern_done(i_LaneID_Pattern_done),
    .i_falling_edge_busy(i_falling_edge_busy),
    .i_Rx_SbMessage(i_decoded_sb_msg),
    .i_msg_valid(i_sb_valid),
    .i_REVERSAL_Result_SB(i_data_bus),
    .i_REVERSAL_Result_logged(i_REVERSAL_Result_logged),

    .o_MBINIT_REVERSALMB_LaneID_Pattern_En(o_MBINIT_REVERSALMB_LaneID_Pattern_En),
    .o_MBINIT_ApplyReversal_En(o_MBINIT_ApplyReversal_En),
    .o_MBINIT_REVERSALMB_end(REVERSAL_END),
    .o_TX_SbMessage(encoded_sb_msg_reversalmb),
    .o_Clear_Pattern_Comparator(o_Clear_Pattern_Comparator_reversal),
    .o_REVERSAL_Pattern_Result_logged(data_bus_reversalmb),
    .o_ValidOutDatatREVERSALMB(msg_valid_reversalmb),
    .o_ValidDataFieldParameters(data_valid_reversal),
    .o_train_error_req_reversalmb(error_req_reversalmb)

);

REPAIRMB_Wrapper dut_repairmb
(
    .CLK(i_clk),
    .rst_n(i_rst_n),
    .MBINIT_REVERSALMB_end(REVERSAL_END),
    .i_RX_SbMessage(i_decoded_sb_msg),
    .i_falling_edge_busy(i_falling_edge_busy),
    .i_d2c_tx_ack(i_d2c_tx_ack),
    .i_lanes_results_tx(i_lanes_results_tx),
    .i_Functional_Lanes(i_msg_info[1:0]),
    .i_msg_valid(i_sb_valid),
    
    .o_TX_SbMessage(encoded_sb_msg_repairmb),
    .o_MBINIT_REPAIRMB_end(REPAIRMB_END),
    .o_tx_data_valid_repair(msg_valid_repairmb),
    .o_Functional_Lanes_out_tx(o_Functional_Lanes_out_tx),
    .o_Functional_Lanes_out_rx(o_Functional_Lanes_out_rx),
    .o_Transmitter_initiated_D2C_en(o_Transmitter_initiated_D2C_en),
    .o_perlane_Transmitter_initiated_D2C(o_perlane_Transmitter_initiated_D2C),
    .o_mainband_Transmitter_initiated_D2C(o_mainband_Transmitter_initiated_D2C),
    .o_train_error(error_req_repairmb),
    .o_msg_info_repairmb(msg_info_repairmb)
);

/////////////////////////////////////////////////////////////////////////////
// output mux - combinational
/////////////////////////////////////////////////////////////////////////////

always @(*) 
begin
    // default : zero
    o_encoded_SB_msg  = sb_msg_id'(0);
    o_msg_valid       = '0;
    o_data_bus        = '0;
    o_msg_info        = '0;
    o_valid_datafield = '0;
    o_error_req       = '0;

    if (!i_MBINIT_en || REPAIRMB_END) begin
        o_encoded_SB_msg  = sb_msg_id'(0);
        o_msg_valid       = '0;
        o_data_bus        = '0;
        o_msg_info        = '0;
        o_valid_datafield = '0;
        o_error_req       = '0;
        if(REPAIRMB_END)    o_MBINIT_END      = 1'b1;
        else                o_MBINIT_END      = 1'b0;
    end
    else if (REVERSAL_END) begin          // REPAIRMB is active
        o_encoded_SB_msg  = encoded_sb_msg_repairmb;
        o_msg_valid       = msg_valid_repairmb;
        o_data_bus        = '0;
        o_msg_info        = msg_info_repairmb;
        o_valid_datafield = 0;
        o_error_req       = error_req_repairmb;
        o_MBINIT_END      = 1'b0;
    end
    else if (REPAIRVAL_END) begin         // REVERSALMB is active
        o_encoded_SB_msg  = encoded_sb_msg_reversalmb;
        o_msg_valid       = msg_valid_reversalmb;
        o_data_bus        = data_bus_reversalmb;
        o_msg_info        = '0;
        o_valid_datafield = data_valid_reversal;
        o_error_req       = error_req_reversalmb;
        o_MBINIT_END      = 1'b0;
    end
    else if (REPAIRCLK_END) begin         // REPAIRVAL is active
        o_encoded_SB_msg  = encoded_sb_msg_repairval;
        o_msg_valid       = msg_valid_repairval;
        o_data_bus        = '0;
        o_msg_info        = {2'b00, msg_info_repairval};
        o_valid_datafield = '0;
        o_error_req       = error_req_repairval;
        o_MBINIT_END      = 1'b0;
    end
    else if (CAL_END) begin               // REPAIRCLK is active
        o_encoded_SB_msg  = encoded_sb_msg_repairclk;
        o_msg_valid       = msg_valid_repairclk;
        o_data_bus        = '0;
        o_msg_info        = msg_info_repairclk;
        o_valid_datafield = '0;
        o_error_req       = error_req_repairclk;
        o_MBINIT_END      = 1'b0;
    end
    else if (PARAM_END) begin             // CAL is active
        o_encoded_SB_msg  = encoded_SB_msg_cal;
        o_msg_valid       = msg_valid_cal;
        o_data_bus        = '0;
        o_msg_info        = '0;
        o_valid_datafield = '0;
        o_error_req       = '0;
        o_MBINIT_END      = 1'b0;
    end
    else begin                            // PARAM is active
        o_encoded_SB_msg  = encoded_SB_msg_param;
        o_msg_valid       = msg_valid_param;
        o_data_bus        = data_bus_param;
        o_msg_info        = '0;
        o_valid_datafield = data_valid_param;
        o_error_req       = error_req_param;
        o_MBINIT_END      = 1'b0;
    end
end
endmodule