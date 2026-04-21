interface  RDI_if#(
    parameter  NBYTES = 64,
    parameter NC = 32
)(
    input logic clk,
    input logic rstn
);
    
logic lp_irdy;
logic [7:0] lp_data[NBYTES-1:0];
logic lp_valid;
logic lp_retimer_crd;
logic [3:0] lp_state_req;
logic lp_linkerror;
logic [NC-1:0] lp_cfg;
logic lp_cfg_vld;
logic lp_cfg_crd;
logic lp_stallack;
logic lp_wake_req;
logic lp_clk_ack;

logic pl_trdy;
logic pl_valid;
logic [7:0] pl_data[NBYTES-1:0];
logic pl_retimer_crd;
logic pl_inband_pres;
logic pl_error;
logic pl_cerror;
logic pl_nferror;
logic pl_trainerror;
logic [3:0] pl_state_sts;
logic [NC-1:0] pl_cfg;
logic pl_cfg_vld;
logic pl_cfg_crd;
logic pl_phyinrecenter;
logic pl_stallreq;
logic [2:0] pl_speedmode;
logic pl_max_speedmode;
logic [2:0] pl_lnk_cfg;
logic pl_wake_ack;
logic pl_clk_req;


modport adapter (
    input clk, rstn, pl_trdy, pl_valid, pl_data, pl_retimer_crd, pl_inband_pres,
    pl_error, pl_cerror, pl_nferror, pl_trainerror, pl_state_sts, pl_cfg, pl_cfg_vld, pl_cfg_crd,
    pl_phyinrecenter, pl_stallreq, pl_speedmode, pl_max_speedmode, pl_lnk_cfg, pl_wake_ack, pl_clk_req,

    output lp_irdy, lp_data, lp_valid, lp_retimer_crd, lp_state_req, lp_linkerror, lp_stallack, lp_wake_req, lp_clk_ack
);

modport lphy (
    input clk, rstn, lp_irdy, lp_data, lp_valid, lp_retimer_crd, lp_state_req, lp_linkerror, lp_cfg, lp_cfg_vld, lp_cfg_crd, 
    lp_stallack, lp_wake_req, lp_clk_ack,
    
    output pl_trdy, pl_valid, pl_data, pl_retimer_crd, pl_inband_pres,
    pl_error, pl_cerror, pl_nferror, pl_trainerror, pl_state_sts, pl_phyinrecenter, pl_stallreq, pl_speedmode, pl_max_speedmode,
    pl_lnk_cfg, pl_wake_ack, pl_clk_req
);

modport sideband (
    input lp_cfg, lp_cfg_vld, lp_cfg_crd,
    output pl_cfg, pl_cfg_vld, pl_cfg_crd
);
endinterface