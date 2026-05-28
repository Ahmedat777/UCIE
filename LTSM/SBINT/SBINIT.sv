//------------------------------------------------------------------------------
// Module Name : SBINIT
//
// Description:
//   Implements the UCIe Sideband Initialization (SBINIT) control logic.
//   Manages pattern request, Out-of-Reset (OOR) exchange, and DoneReq/DoneResp
//   handshake between two link partners using a centralized FSM and protocol
//   tracking flags. Sideband message transactions are synchronized using the falling-edge busy indication.
//
// Interface:
// Inputs:
//   i_clk              : System clock.
//   i_rst_n            : Active-low asynchronous reset.
//   i_sbinit_en        : Enables SBINIT operation; deassertion clears state.
//   i_pattern_done     : Indicates completion of sideband pattern transmission.
//   i_sb_valid     : Valid pulse for received sideband message.
//   i_decoded_sb_msg   : Decoded received sideband message (OOR, DoneReq, DoneResp).
//   i_sb_busy          : Sideband interface busy indicator.
//   i_falling_edge_busy: Pulse indicating completion of a sideband transaction.
//
// Outputs:
//   o_start_pattern_req: Requests start of sideband pattern transmission.
//   o_msg_valid        : valid pulse for sideband message transmit.
//   o_encoded_SB_msg   : Encoded sideband message (valid with o_msg_valid).
//   o_SBINIT_end       : Asserted when SBINIT protocol completes.
//
// Internal Signals:
//   CS, NS             : Current and next FSM states.
//   oor_sent_once      : Indicates at least one OOR message transmitted.
//   oor_received       : Indicates receipt of peer OOR message.
//   done_req_sent      : Indicates DoneReq message transmitted.
//   done_req_received  : Indicates receipt of DoneReq message.
//   done_resp_sent     : Indicates DoneResp message transmitted.
//   done_resp_received : Indicates receipt of DoneResp message.
//
// Notes:
//   - Message transmission is synchronized using 
//     i_falling_edge_busy.
//   - DoneResp transmission is prioritized upon receipt of DoneReq to ensure
//     protocol forward progress.
//------------------------------------------------------------------------------
// Author  : Tayseer Magdy Goda
// Version : 1.0
//------------------------------------------------------------------------------

import pckg::*;

module SBINIT #(parameter SB_MSG_Width = 4)
(
    input   wire    i_clk , i_rst_n ,
    input   wire    i_sbinit_en ,
    input   wire    i_pattern_done ,
    input   wire    i_sb_valid ,
    input   sb_msg_id    i_decoded_sb_msg ,
    input   wire    i_sb_busy ,
    input   wire    i_falling_edge_busy ,

    output  reg     o_start_pattern_req ,
    output  reg     o_msg_valid ,
    output    sb_msg_id  o_encoded_SB_msg ,
    output  reg     o_SBINIT_end        
);

//states
typedef enum logic [2:0] {
    IDLE,
    PATTERN_REQ,
    WAIT_PATTERN_DONE,
    SEND_OOR,
    SEND_DONE_REQ,
    SEND_DONE_RESP,
    SBINIT_END
} state_t;
state_t CS, NS;

/*// ---- message encodings  ----
    localparam [SB_MSG_Width-1:0]   SBINIT_Out_of_Reset_msg = 4'd3 ,
                                    SBINIT_done_req_msg     = 4'd1 ,
                                    SBINIT_done_resp_msg    = 4'd2 ;*/

//internal signals
reg    oor_sent_once ;
reg    oor_received ;
reg    done_req_sent ;
reg    done_req_received ;
reg    done_resp_sent ;
reg    done_resp_received ;

//state transition block
always @(posedge i_clk or negedge i_rst_n)
    begin
        if(!i_rst_n)
            CS <= IDLE ;
        else
            CS <= NS ;    
    end

//next state logic
always @(*)
    begin
        if (!i_sbinit_en)
            begin
                NS = IDLE ;
            end
        else    begin    
        case(CS)
            IDLE:
                begin
                    NS = (i_sbinit_en) ?    PATTERN_REQ : IDLE ;    
                end
            PATTERN_REQ:
                begin
                    NS = WAIT_PATTERN_DONE ;
                end
            WAIT_PATTERN_DONE:
                begin
                    NS = (i_pattern_done) ?     SEND_OOR : WAIT_PATTERN_DONE ;
                end
            SEND_OOR:
                begin
                    if (oor_sent_once && oor_received)
                        begin
                            if (done_req_received)
                                NS = SEND_DONE_RESP ;
                            else
                                NS = SEND_DONE_REQ ;    
                        end
                    else
                        begin
                            NS = SEND_OOR ;
                        end
                end
            SEND_DONE_REQ:
                begin
                    if (i_falling_edge_busy)
                        begin
                            if (done_req_received && !done_resp_sent)
                                begin
                                    NS = SEND_DONE_RESP ;
                                end
                            else
                                begin
                                    NS = SBINIT_END ;
                                end     
                        end
                    else
                        begin
                            NS = SEND_DONE_REQ ;
                        end    
                end
            SEND_DONE_RESP:
                begin
                    if (i_falling_edge_busy)
                        begin
                            if(done_req_sent)
                                begin
                                    NS = SBINIT_END ;
                                end
                            else
                                begin
                                    NS = SEND_DONE_REQ ;
                                end     
                        end
                    else
                        NS = SEND_DONE_RESP ;    
                end            
            SBINIT_END:
                begin
                    if (done_req_received && !done_resp_sent)
                                begin
                                    NS = SEND_DONE_RESP ;
                                end
                    else            
                    NS = (!i_sbinit_en) ? IDLE : SBINIT_END ;
                end
            default:
                begin
                    NS = IDLE ;
                end    

        endcase
        end
    end    
// output logic
always @(posedge i_clk or negedge i_rst_n)
    begin
        if (!i_rst_n) 
            begin
                o_start_pattern_req <= 1'b0;
                o_msg_valid         <= 1'b0;
                o_encoded_SB_msg    <= sb_msg_id'(0);
                o_SBINIT_end        <= 1'b0;
                oor_sent_once   <= 0 ;
                oor_received   <= 0 ;
                done_req_sent   <= 0 ;
                done_req_received   <= 0 ;
                done_resp_sent   <= 0 ;
                done_resp_received   <= 0 ;
            end
        /*else if (!i_SBINIT_en)
            begin
                o_start_pattern_req <= 1'b0;
                o_msg_valid         <= 1'b0;
                o_encoded_SB_msg    <= '0;
                o_SBINIT_end        <= 1'b0;
                oor_sent_once   <= 0 ;
                oor_received   <= 0 ;
                done_req_sent   <= 0 ;
                done_req_received   <= 0 ;
                done_resp_sent   <= 0 ;
                done_resp_received   <= 0 ;                                
            end */    
        else
            begin
                if (i_decoded_sb_msg== SB_SBINIT_DONE_RESP && i_sb_valid)
                    begin
                        done_resp_received <= 1'b1 ;
                    end
                if (i_decoded_sb_msg== SB_SBINIT_DONE_REQ && i_sb_valid)
                    begin
                        done_req_received <= 1'b1 ;
                    end    
                o_start_pattern_req <= 1'b0 ;
                o_msg_valid         <= 1'b0 ;
                o_encoded_SB_msg    <= sb_msg_id'(0);
                o_SBINIT_end        <= 1'b0 ;    
                case (NS)
                    IDLE:
                        begin
                            oor_sent_once   <= 0 ;
                            oor_received   <= 0 ;
                            done_req_sent   <= 0 ;
                            done_req_received   <= 0 ;
                            done_resp_sent   <= 0 ;
                            done_resp_received   <= 0 ;    
                        end
                    PATTERN_REQ:
                        begin
                            o_start_pattern_req <= 1'b1 ;
                        end
                    WAIT_PATTERN_DONE:
                        begin
                    
                        end
                    SEND_OOR:
                        begin
                            o_msg_valid         <= 1'b1 ;
                            o_encoded_SB_msg    <= SB_SBINIT_OUT_OF_RESET ;
                            if(i_falling_edge_busy)
                                begin
                                    oor_sent_once <= 1'b1 ;
                                end
                            if (i_decoded_sb_msg== SB_SBINIT_OUT_OF_RESET && i_sb_valid)
                                begin
                                    oor_received <= 1'b1 ;
                                    if (oor_sent_once)
                                        begin
                                           o_msg_valid         <= 1'b0 ; 
                                        end
                                end

                        end
                    SEND_DONE_REQ:
                        begin
                            o_msg_valid         <= 1'b1 ;
                            o_encoded_SB_msg    <= SB_SBINIT_DONE_REQ ;
                            if(i_falling_edge_busy)
                                begin
                                    done_req_sent <= 1'b1 ;
                                    o_msg_valid         <= 1'b0 ;
                                end
                        end
                    SEND_DONE_RESP:
                        begin
                           o_msg_valid         <= 1'b1 ;
                            o_encoded_SB_msg    <= SB_SBINIT_DONE_RESP ;
                            if(i_falling_edge_busy)
                                begin
                                    done_resp_sent <= 1'b1 ;
                                    o_msg_valid         <= 1'b0 ;
                                end 
                        end
                    SBINIT_END:
                        begin
                            if ( done_resp_sent && done_resp_received)
                                begin
                                    o_SBINIT_end <= 'b1 ;
                                end 
                        end 
                    default:
                        begin
                            o_start_pattern_req <= 1'b0;
                            o_msg_valid         <= 1'b0;
                            o_encoded_SB_msg    <= sb_msg_id'(0);
                            o_SBINIT_end        <= 1'b0;
                        end                               
                endcase
            end
        
    end

endmodule