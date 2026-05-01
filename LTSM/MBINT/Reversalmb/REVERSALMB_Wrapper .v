module REVERSALMB_Wrapper (
    input wire          CLK,
    input wire          rst_n,
    input wire          i_MBINIT_REPAIRVAL_end,
    input wire          i_REVERSAL_done,
    input               i_LaneID_Pattern_done,
    input wire          i_falling_edge_busy,
    input wire [3:0]    i_Rx_SbMessage,
    input               i_msg_valid,
    input wire [15:0]   i_REVERSAL_Result_SB,
    input wire [15:0]   i_REVERSAL_Result_logged,

    output   [1:0]    o_MBINIT_REVERSALMB_LaneID_Pattern_En,
    output            o_MBINIT_ApplyReversal_En,
    output            o_MBINIT_REVERSALMB_end,
    output   [3:0]    o_TX_SbMessage,
    output   [1:0]    o_Clear_Pattern_Comparator,
    output   [15:0]   o_REVERSAL_Pattern_Result_logged,
    output            o_ValidOutDatatREVERSALMB,
    output            o_ValidDataFieldParameters,
    output            o_train_error_req_reversalmb

);

wire ValidOutDatat_Module;
wire ValidOutDatat_ModulePartner;
wire ValidDataFieldParameters_modulePartner;
wire [3:0] TX_SbMessage_Module;
wire [3:0] TX_SbMessage_ModulePartner;
wire MBINIT_REVERSALMB_Module_end;
wire MBINIT_REVERSALMB_ModulePartner_end;
wire [1:0] MBINIT_REVERSALMB_LaneID_Pattern_En;
wire MBINIT_ApplyReversal_En;
wire [15:0] REVERSAL_Pattern_Result_logged;
wire [1:0]  Clear_Pattern_Comparator;
wire        apply_repeater;
wire        Start_Repeater;
wire   Second_Clear_Error_Req;

// Instantiate REVERSALMB_Module
REVERSALMB_Module u1 (
    .CLK(CLK),
    .rst_n(rst_n),
    .i_MBINIT_REPAIRVAL_end(i_MBINIT_REPAIRVAL_end),
    .i_REVERSAL_done(i_REVERSAL_done),
    .i_Rx_SbMessage(i_Rx_SbMessage),
    .i_Busy_SideBand(ValidOutDatat_ModulePartner),
    .i_LaneID_Pattern_done(i_LaneID_Pattern_done),
    .i_falling_edge_busy(i_falling_edge_busy),
    .i_REVERSAL_Result_SB(i_REVERSAL_Result_SB),
    .i_msg_valid(i_msg_valid),
    .apply_repeater                             (apply_repeater),
    .i_Start_Repeater                           (Start_Repeater),


    .o_MBINIT_REVERSALMB_LaneID_Pattern_En(MBINIT_REVERSALMB_LaneID_Pattern_En),
    .o_MBINIT_ApplyReversal_En(MBINIT_ApplyReversal_En),
    .o_MBINIT_REVERSALMB_Module_end(MBINIT_REVERSALMB_Module_end),
    .o_TX_SbMessage(TX_SbMessage_Module),
    .o_tx_data_valid_reversal(ValidOutDatat_Module),
    .o_Second_Clear_Error_Req(Second_Clear_Error_Req),
    .o_train_error_req_reversalmb(o_train_error_req_reversalmb)
);

// Instantiate REVERSALMB_ModulePartner
REVERSALMB_ModulePartner u2 (
    .CLK(CLK),
    .rst_n(rst_n),
    .i_MBINIT_REPAIRVAL_end(i_MBINIT_REPAIRVAL_end),
    .i_REVERSAL_Result_logged(i_REVERSAL_Result_logged),
    .i_Rx_SbMessage(i_Rx_SbMessage),
    .i_falling_edge_busy(i_falling_edge_busy),
    .i_Busy_SideBand(ValidOutDatat_Module),
    .i_msg_valid(i_msg_valid),
    .i_Second_Clear_Error_Req(Second_Clear_Error_Req),
    .o_REVERSAL_Pattern_Result_logged(REVERSAL_Pattern_Result_logged),
    .o_TX_SbMessage(TX_SbMessage_ModulePartner),
    .o_Start_Repeater  (Start_Repeater),
    .o_Clear_Pattern_Comparator(Clear_Pattern_Comparator), 
    .o_MBINIT_REVERSALMB_ModulePartner_end(MBINIT_REVERSALMB_ModulePartner_end),
    .o_tx_data_valid_reversal(ValidOutDatat_ModulePartner),
    .o_ValidDataFieldParameters_modulePartner(ValidDataFieldParameters_modulePartner),
    .apply_repeater (apply_repeater)

);

// Combinational output logic
assign o_TX_SbMessage                          = ValidOutDatat_ModulePartner ? TX_SbMessage_ModulePartner :
                                                 ValidOutDatat_Module ? TX_SbMessage_Module : 4'b0000;
assign o_MBINIT_REVERSALMB_end                 = MBINIT_REVERSALMB_Module_end && MBINIT_REVERSALMB_ModulePartner_end;
assign o_ValidOutDatatREVERSALMB               = ValidOutDatat_ModulePartner || ValidOutDatat_Module;
assign o_ValidDataFieldParameters              = ValidDataFieldParameters_modulePartner;
assign o_MBINIT_REVERSALMB_LaneID_Pattern_En   = MBINIT_REVERSALMB_LaneID_Pattern_En;
assign o_MBINIT_ApplyReversal_En               = MBINIT_ApplyReversal_En;
assign o_Clear_Pattern_Comparator              = Clear_Pattern_Comparator  ;
assign o_REVERSAL_Pattern_Result_logged        = REVERSAL_Pattern_Result_logged;
endmodule