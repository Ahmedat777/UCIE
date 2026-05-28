import pckg::*;

module REPAIRCLK_WRAPPER
(
    input   i_clk,
    input   i_rst_n,
    input   i_mbinit_rpairclk_en,
    input   i_clk_ptrn_done,
    input  sb_msg_id             i_decoded_sb_msg,
    input   i_sb_busy ,
    input   i_falling_edge_busy ,
    input   i_sb_valid,   
    input [2:0]   i_logged_results_SB, // from sideband
    input [2:0]   i_logged_results_COMP, // from comparator
    // input wire          i_Valid_Clock_track_result_logged, //from comparator when valid result is available (sending _the_result)
    output  o_error_req,
    output  o_clk_ptrn_en,
    output reg o_MBINIT_REPAIRCLK_end,
    output reg sb_msg_id         o_encoded_sb_msg ,
    // output reg          o_MBINIT_RepairCLK_Detection_GetResult,
    output  [2:0]    o_logged_rx ,
    output           o_clear_log ,
    output   reg        o_msg_valid
);

/////////////////// internal signals ////////////////////////////
wire w_valid_RX , w_valid_TX ;
wire w_tx_end , w_rx_end ;
wire sb_msg_id w_tx_msg , w_rx_msg ;
//////////////// TX instance //////////////////////////
REPAIRCLK_TX TX_inst
(
    .i_clk (i_clk) ,
    .i_rst_n(i_rst_n) ,
    .i_mbinit_rpairclk_en(i_mbinit_rpairclk_en) ,
    .i_sb_busy (i_sb_busy),
    .i_falling_edge_busy(i_falling_edge_busy) ,
    .i_decoded_sb_msg (i_decoded_sb_msg) ,
    .i_sb_valid (i_sb_valid),
    .i_clk_ptrn_done (i_clk_ptrn_done) ,
    .i_logged_results(i_logged_results_SB) ,
    .i_valid_partner(w_valid_RX) ,

    .o_encoded_sb_msg (w_tx_msg) ,
    .o_msg_valid(w_valid_TX) ,
    .o_clk_ptrn_en(o_clk_ptrn_en) ,
    .o_error_req (o_error_req),
    .o_TX_end(w_tx_end)    
);

//////////////// RX instance //////////////////////////
REPAIRCLK_RX RX_inst
(
    .i_clk (i_clk) ,
    .i_rst_n(i_rst_n) ,
    .i_mbinit_rpairclk_en(i_mbinit_rpairclk_en) ,
    .i_sb_busy (i_sb_busy),
    .i_falling_edge_busy(i_falling_edge_busy) ,
    .i_decoded_sb_msg (i_decoded_sb_msg) ,
    .i_sb_valid (i_sb_valid),
    .i_logged_results(i_logged_results_COMP) ,
    
    .o_encoded_sb_msg (w_rx_msg) ,
    .o_msg_valid(w_valid_RX) ,
    .o_logged_results(o_logged_rx),
    .o_clear_log(o_clear_log),
    .o_RX_end(w_rx_end)    
);
always @(*)
    begin
      o_msg_valid = w_valid_TX || w_valid_RX ;

      if  (w_valid_RX)      o_encoded_sb_msg =  w_rx_msg ;
      else if (w_valid_TX)  o_encoded_sb_msg =  w_tx_msg ;
      else                  o_encoded_sb_msg =  sb_msg_id'(0) ;

      o_MBINIT_REPAIRCLK_end = w_rx_end && w_tx_end ;
    end
 
 
endmodule