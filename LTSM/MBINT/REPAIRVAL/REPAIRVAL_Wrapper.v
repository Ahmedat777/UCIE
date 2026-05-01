module REPAIRVAL_Wrapper (
    input wire          CLK,
    input wire          rst_n,
    input wire          i_REPAIRCLK_end,
    input wire          i_VAL_Pattern_done,
    input wire [3:0]    i_Rx_SbMessage,
    input wire          i_msg_valid,
    input wire          i_falling_edge_busy,
    input wire          i_VAL_Result_logged_RXSB,
    input wire          i_VAL_Result_logged_COMB,
    
    output wire         o_train_error_req,
    output wire         o_MBINIT_REPAIRVAL_Pattern_En,
    output wire         o_MBINIT_REPAIRVAL_end,
    output wire [3:0]   o_TX_SbMessage,
    output wire         o_VAL_128Result_logged,
    output wire         o_enable_16_iterations,
    output wire         o_ValidOutData
);

////////////////////////////////////////////////////////////////////////////////
// Internal Signals from REPAIRVAL_Module
////////////////////////////////////////////////////////////////////////////////
wire module_train_error_req;
wire module_pattern_enable;
wire o_MBINIT_REPAIRVAL_Module_end;
wire module_valid_tx_data;
wire [3:0] module_tx_message;

////////////////////////////////////////////////////////////////////////////////
// Internal Signals from REPAIRVAL_ModulePartner
////////////////////////////////////////////////////////////////////////////////
wire partner_validation_result_status;
wire o_MBINIT_REPAIRVAL_ModulePartner_end;
wire partner_valid_tx_data;
wire partner_enable_16_iterations;
wire [3:0] partner_tx_message;

////////////////////////////////////////////////////////////////////////////////
// Instantiate REPAIRVAL_Module (Initiator)
////////////////////////////////////////////////////////////////////////////////
REPAIRVAL_Module u_repairval_module (
    .CLK                            (CLK),
    .rst_n                          (rst_n),
    .i_REPAIRCLK_end                (i_REPAIRCLK_end),
    .i_VAL_Pattern_done             (i_VAL_Pattern_done),
    .i_Rx_SbMessage                 (i_Rx_SbMessage),
    .i_Busy_SideBand                (partner_valid_tx_data),
    .i_msg_valid                    (i_msg_valid),
    .i_falling_edge_busy            (i_falling_edge_busy),
    .i_VAL_Result_logged            (i_VAL_Result_logged_RXSB),
    
    .o_train_error_req              (module_train_error_req),
    .o_MBINIT_REPAIRVAL_Pattern_En  (module_pattern_enable),
    .o_MBINIT_REPAIRVAL_Module_end  (o_MBINIT_REPAIRVAL_Module_end),
    .o_TX_SbMessage                 (module_tx_message),
    .o_ValidOutDatat_Module         (module_valid_tx_data)
);

////////////////////////////////////////////////////////////////////////////////
// Instantiate REPAIRVAL_ModulePartner (Responder)
////////////////////////////////////////////////////////////////////////////////
REPAIRVAL_ModulePartner u_repairval_partner (
    .CLK                                    (CLK),
    .rst_n                                  (rst_n),
    .i_REPAIRCLK_end                        (i_REPAIRCLK_end),
    .i_VAL_Result_logged                    (i_VAL_Result_logged_COMB),
    .i_Rx_SbMessage                         (i_Rx_SbMessage),
    .i_msg_valid                            (i_msg_valid),
    .i_falling_edge_busy                    (i_falling_edge_busy),
    .i_Busy_SideBand                        (module_valid_tx_data),
    .o_VAL_128Result_logged                 (partner_validation_result_status),
    .o_TX_SbMessage                         (partner_tx_message),
    .o_MBINIT_REPAIRVAL_ModulePartner_end   (o_MBINIT_REPAIRVAL_ModulePartner_end),
    .o_ValidOutDatat_ModulePartner          (partner_valid_tx_data),
    .o_enable_16_iterations                 (partner_enable_16_iterations)
);

////////////////////////////////////////////////////////////////////////////////
// Wrapper Output Assignments
////////////////////////////////////////////////////////////////////////////////

// Multiplexed TX message - prioritize partner response over module request
assign o_TX_SbMessage = partner_valid_tx_data ? partner_tx_message :
                        module_valid_tx_data  ? module_tx_message  : 4'b0000;

// Overall sequence complete when both modules complete
assign o_MBINIT_REPAIRVAL_end = o_MBINIT_REPAIRVAL_Module_end && o_MBINIT_REPAIRVAL_ModulePartner_end;

// Direct pass-through outputs
assign o_train_error_req              = module_train_error_req;
assign o_MBINIT_REPAIRVAL_Pattern_En  = module_pattern_enable;
assign o_VAL_128Result_logged         = partner_validation_result_status;
assign o_enable_16_iterations         = partner_enable_16_iterations;

// Valid data when either module or partner has valid data
assign o_ValidOutData                 = partner_valid_tx_data || module_valid_tx_data;

endmodule