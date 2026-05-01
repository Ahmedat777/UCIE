////////////////////////////////////////////////////////////////////////////////
// CAL_ModuleWrapper - Top-level wrapper for CAL handshake protocol
// Instantiates both initiator and responder modules
////////////////////////////////////////////////////////////////////////////////
module CAL_ModuleWrapper (
    input               CLK,
    input               rst_n,
    input               i_MBINIT_PARAM_end,
    input [3:0]         i_RX_SbMessage,
    input               i_falling_edge_busy,
    input               i_msg_valid,
    
    output [3:0]        o_TX_SbMessage,
    output              o_MBINIT_CAL_end,
    output              o_ValidOutDatatCAL
);

////////////////////////////////////////////////////////////////////////////////
// Internal wires - Module (Initiator) signals
////////////////////////////////////////////////////////////////////////////////
wire [3:0]  w_TX_SbMessage_Module;
wire        w_MBINIT_CAL_Module_end;
wire        w_ValidOutDatat_Module;

////////////////////////////////////////////////////////////////////////////////
// Internal wires - ModulePartner (Responder) signals
////////////////////////////////////////////////////////////////////////////////
wire [3:0]  w_TX_SbMessage_ModulePartner;
wire        w_MBINIT_CAL_ModulePartner_end;
wire        w_ValidOutDatat_ModulePartner;

////////////////////////////////////////////////////////////////////////////////
// Instantiate CAL_Module (Initiator/Master)
////////////////////////////////////////////////////////////////////////////////
CAL_Module #(
    .SB_MSG_WIDTH(4)
) u_cal_module (
    .CLK                        (CLK),
    .rst_n                      (rst_n),
    .i_MBINIT_PARAM_end         (i_MBINIT_PARAM_end),
    .i_RX_SbMessage             (i_RX_SbMessage),
    .i_msg_valid                (i_msg_valid),
    .i_Busy_SideBand            (w_ValidOutDatat_ModulePartner),
    .i_falling_edge_busy        (i_falling_edge_busy),
    .o_TX_SbMessage             (w_TX_SbMessage_Module),
    .o_ValidOutDatat_Module     (w_ValidOutDatat_Module),
    .o_MBINIT_CAL_Module_end    (w_MBINIT_CAL_Module_end)
);

////////////////////////////////////////////////////////////////////////////////
// Instantiate CAL_ModulePartner (Responder/Slave)
////////////////////////////////////////////////////////////////////////////////
CAL_ModulePartner u_cal_module_partner (
    .CLK                                (CLK),
    .rst_n                              (rst_n),
    .i_MBINIT_PARAM_end                 (i_MBINIT_PARAM_end),
    .i_RX_SbMessage                     (i_RX_SbMessage),
    .i_msg_valid                        (i_msg_valid),
    .i_Busy_SideBand                    (o_ValidOutDatatCAL),
    .i_falling_edge_busy                (i_falling_edge_busy),
    .o_TX_SbMessage                     (w_TX_SbMessage_ModulePartner),
    .o_ValidOutDatat_ModulePartner      (w_ValidOutDatat_ModulePartner),
    .o_MBINIT_CAL_ModulePartner_end     (w_MBINIT_CAL_ModulePartner_end)
);

////////////////////////////////////////////////////////////////////////////////
// Output arbitration logic
// Priority: Partner (Responder) has priority over Module (Initiator)
////////////////////////////////////////////////////////////////////////////////
assign o_TX_SbMessage = w_ValidOutDatat_ModulePartner ? w_TX_SbMessage_ModulePartner :
                        w_ValidOutDatat_Module        ? w_TX_SbMessage_Module : 
                        4'b0000;

assign o_MBINIT_CAL_end = w_MBINIT_CAL_Module_end && w_MBINIT_CAL_ModulePartner_end;

assign o_ValidOutDatatCAL = w_ValidOutDatat_ModulePartner || w_ValidOutDatat_Module;

endmodule