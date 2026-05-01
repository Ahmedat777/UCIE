module TRAINERROR_HS_WRAPPER #(
    parameter SB_MSG_WIDTH = 4
) (
    input                           i_clk,
    input                           i_rst_n,
    input                           i_trainerror_en,
    input                           i_msg_valid,
    input                           i_falling_edge_busy,
    input  [SB_MSG_WIDTH-1:0]       i_Rx_SbMessage,
    output [SB_MSG_WIDTH-1:0]       o_TX_SbMessage,
    output                          o_TRAINERROR_HS_end,
    output                          o_tx_msg_valid
);

wire                        valid_Module;
wire                        valid_Module_Partner;
wire [SB_MSG_WIDTH-1:0]     TX_SbMessage_Module;
wire [SB_MSG_WIDTH-1:0]     TX_SbMessage_ModulePartner;
wire                        trainerror_end_Module;
wire                        trainerror_end_Module_Partner;

// Registers to latch end signals
reg module_done;
reg partner_done;

// Instantiate TRAINERROR_Module
TRAINERROR__HS_Module #(
    .SB_MSG_WIDTH(SB_MSG_WIDTH)
) u1 (
    .i_clk                      (i_clk),
    .i_rst_n                    (i_rst_n),
    .i_trainerror_en            (i_trainerror_en),
    .i_msg_valid                (i_msg_valid),
    .i_falling_edge_busy        (i_falling_edge_busy),
    .i_partner_valid            (valid_Module_Partner),
    .i_Rx_SbMessage             (i_Rx_SbMessage),
    .o_TX_SbMessage             (TX_SbMessage_Module),
    .o_trainerror_end_Module    (trainerror_end_Module),
    .o_valid_Module             (valid_Module)
);

// Instantiate TRAINERROR_ModulePartner
TRAINERROR_HS_ModulePartner #(
    .SB_MSG_WIDTH(SB_MSG_WIDTH)
) u2 (
    .i_clk                              (i_clk),
    .i_rst_n                            (i_rst_n),
    .i_trainerror_en                    (i_trainerror_en),
    .i_msg_valid                        (i_msg_valid),
    .i_falling_edge_busy                (i_falling_edge_busy),
    .i_module_valid                     (valid_Module),
    .i_Rx_SbMessage                     (i_Rx_SbMessage),
    .o_TX_SbMessage                     (TX_SbMessage_ModulePartner),
    .o_trainerror_end_Module_Partner    (trainerror_end_Module_Partner),
    .o_valid_Module_Partner             (valid_Module_Partner)
);

// Latch end signals
always @(posedge i_clk or negedge i_rst_n) begin
    if (!i_rst_n) begin
        module_done  <= 0;
        partner_done <= 0;
    end else begin
        if (trainerror_end_Module)        module_done  <= 1;
        if (trainerror_end_Module_Partner) partner_done <= 1;
    end
end

// Combinational output logic
assign o_TX_SbMessage       = valid_Module_Partner ? TX_SbMessage_ModulePartner :
                              valid_Module ? TX_SbMessage_Module : 4'b0000;

// o_TRAINERROR_HS_end goes 1 once both modules are done, even if at different clocks
assign o_TRAINERROR_HS_end  = module_done && partner_done;
assign o_tx_msg_valid       = valid_Module_Partner || valid_Module;

endmodule

