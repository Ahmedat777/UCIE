////////////////////////////////////////////////////////////////////////////////
// REPAIRMB_Wrapper - Improved Version
// Top-level wrapper connecting transmitter and partner modules
////////////////////////////////////////////////////////////////////////////////
module REPAIRMB_Wrapper (
    input wire                  CLK,
    input wire                  rst_n,
    input wire                  MBINIT_REVERSALMB_end,
    input wire [3:0]            i_RX_SbMessage,
    input wire                  i_falling_edge_busy,
    input wire                  i_d2c_tx_ack,
    input wire [15:0]           i_lanes_results_tx,
    input wire [1:0]            i_Functional_Lanes,
    input wire                  i_msg_valid,
    
    output wire [3:0]           o_TX_SbMessage,
    output wire                 o_MBINIT_REPAIRMB_end,
    output wire                 o_tx_data_valid_repair,
    output wire [1:0]           o_Functional_Lanes_out_tx,
    output wire [1:0]           o_Functional_Lanes_out_rx,
    output wire                 o_Transmitter_initiated_D2C_en,
    output wire                 o_perlane_Transmitter_initiated_D2C,
    output wire                 o_mainband_Transmitter_initiated_D2C,
    output wire                 o_train_error,
    output wire [2:0]           o_msg_info_repairmb
);

    ////////////////////////////////////////////////////////////////////////////////
    // Internal Wires
    ////////////////////////////////////////////////////////////////////////////////
    wire [3:0]  TX_SbMessage_Module;
    wire        MBINIT_REPAIRMB_Module_end;
    wire        tx_data_valid_Module;

    wire [3:0]  TX_SbMessage_ModulePartner;
    wire        MBINIT_REPAIRMB_ModulePartner_end;
    wire        tx_data_valid_ModulePartner;

    wire        Start_Repeater;
    wire        Done_Repeater;
    wire        apply_repeater;

    ////////////////////////////////////////////////////////////////////////////////
    // Module Instantiations
    ////////////////////////////////////////////////////////////////////////////////
    REPAIRMB_Module REPAIRMB_Module_inst (
        .CLK                                        (CLK),
        .rst_n                                      (rst_n),
        .MBINIT_REVERSALMB_end                      (MBINIT_REVERSALMB_end),
        .i_RX_SbMessage                             (i_RX_SbMessage),
        .i_Busy_SideBand                            (tx_data_valid_ModulePartner),
        .i_falling_edge_busy                        (i_falling_edge_busy),
        .i_msg_valid                                (i_msg_valid),
        .i_Start_Repeater                           (Start_Repeater),
        .i_d2c_tx_ack                               (i_d2c_tx_ack),
        .i_lanes_results_tx                         (i_lanes_results_tx),
        .apply_repeater                             (apply_repeater),
        .o_TX_SbMessage                             (TX_SbMessage_Module),
        .o_Done_Repeater                            (Done_Repeater),
        .o_MBINIT_REPAIRMB_Module_end               (MBINIT_REPAIRMB_Module_end),
        .o_tx_data_valid_repair                     (tx_data_valid_Module),
        .o_Functional_Lanes                         (o_Functional_Lanes_out_tx),
        .o_Transmitter_initiated_D2C_en             (o_Transmitter_initiated_D2C_en),
        .o_perlane_Transmitter_initiated_D2C        (o_perlane_Transmitter_initiated_D2C),
        .o_mainband_Transmitter_initiated_D2C       (o_mainband_Transmitter_initiated_D2C),
        .o_msg_info_repairmb                        (o_msg_info_repairmb)
    );

    REPAIRMB_Module_Partner REPAIRMB_Module_Partner_inst (
        .CLK                                        (CLK),
        .rst_n                                      (rst_n),
        .MBINIT_REVERSALMB_end                      (MBINIT_REVERSALMB_end),
        .i_Busy_SideBand                            (tx_data_valid_Module),
        .i_falling_edge_busy                        (i_falling_edge_busy),
        .i_RX_SbMessage                             (i_RX_SbMessage),
        .i_msg_valid                                (i_msg_valid),
        .i_Functional_Lanes                         (i_Functional_Lanes),
        .i_Transmitter_initiated_D2C_en             (o_Transmitter_initiated_D2C_en),
        .i_Done_Repeater                            (Done_Repeater),
        .o_Start_Repeater                           (Start_Repeater),
        .o_train_error                              (o_train_error),
        .o_MBINIT_REPAIRMB_Module_Partner_end       (MBINIT_REPAIRMB_ModulePartner_end),
        .o_tx_data_valid_repair_partner             (tx_data_valid_ModulePartner),
        .o_TX_SbMessage                             (TX_SbMessage_ModulePartner),
        .o_Functional_Lanes                         (o_Functional_Lanes_out_rx),
        .apply_repeater                             (apply_repeater)
    );

    ////////////////////////////////////////////////////////////////////////////////
    // Output Assignments
    ////////////////////////////////////////////////////////////////////////////////
    assign o_TX_SbMessage = tx_data_valid_ModulePartner ? TX_SbMessage_ModulePartner : 
                            tx_data_valid_Module        ? TX_SbMessage_Module : 
                                                          4'b0000;
    
    assign o_MBINIT_REPAIRMB_end     = MBINIT_REPAIRMB_Module_end && MBINIT_REPAIRMB_ModulePartner_end;
    assign o_tx_data_valid_repair    = tx_data_valid_ModulePartner || tx_data_valid_Module;

endmodule