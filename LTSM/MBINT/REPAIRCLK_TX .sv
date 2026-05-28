import pckg::*;

module REPAIRCLK_TX
(
    input   i_clk ,
    input   i_rst_n ,
    input   i_mbinit_rpairclk_en ,
    input   i_sb_busy ,
    input   i_falling_edge_busy ,
    input   sb_msg_id            i_decoded_sb_msg  ,
    input   i_sb_valid ,
    input   i_clk_ptrn_done ,
    input   [2:0]   i_logged_results ,
    input   i_valid_partner ,

    output  reg  sb_msg_id       o_encoded_sb_msg ,
    output  reg  o_msg_valid ,
    output  reg  o_clk_ptrn_en ,
    output  reg  o_error_req ,
    output  reg  o_TX_end    
);

/////////////////////// states /////////////////////////////////

typedef enum logic [3:0] {
    IDLE,
    SEND_INIT_REQ,
    WAIT_RESPS,
    PATTERN_EN,
    CHECK_BUSY_RES,
    SEND_RES_REQ ,
    CHECK_RES,
    CHECK_BUSY_DONE ,
    SEND_DONE_REQ ,
    TX_END
} states;

states CS, NS ;

////////////////////// STATE TRANSITION BLOCK //////////////////////////////
always @ (posedge i_clk or negedge i_rst_n)
    begin
        if(!i_rst_n)
            begin
                CS <= IDLE ;
            end
        else
            begin
                CS <= NS ;
            end    
    end

////////////////////////// NEXT STATE BLOCK ////////////////////////////
always @ (*)
    begin
        if (!i_mbinit_rpairclk_en)
            begin
                NS = IDLE ;
            end
        else
            begin
                case(CS)
                    IDLE :
                        begin
                            if (i_mbinit_rpairclk_en && !i_sb_busy)
                                begin
                                    NS = SEND_INIT_REQ ;
                                end
                            else
                                begin
                                    NS = IDLE ;
                                end    
                        end
                    SEND_INIT_REQ:
                        begin
                            if(i_falling_edge_busy)
                                begin
                                    NS = WAIT_RESPS ;
                                end
                            else
                                begin
                                    NS = SEND_INIT_REQ ;
                                end    
                        end
                    WAIT_RESPS:
                        begin
                            if (i_decoded_sb_msg == SB_MBINIT_REPAIRCLK_INIT_RESP && i_sb_valid )      NS = PATTERN_EN ;
                            else if (i_decoded_sb_msg == SB_MBINIT_REPAIRCLK_RESULT_RESP && i_sb_valid)    NS = CHECK_RES ;
                            else if (i_decoded_sb_msg == SB_MBINIT_REPAIRCLK_DONE_RESP && i_sb_valid)    NS = TX_END ;
                            else    NS = WAIT_RESPS ;
                        end
                    PATTERN_EN:
                        begin
                            if(i_clk_ptrn_done)     NS = CHECK_BUSY_RES ;
                            else                    NS = PATTERN_EN ;
                        end
                    CHECK_BUSY_RES:
                        begin
                            if(!i_sb_busy && !i_valid_partner)      NS = SEND_RES_REQ ;
                            else                                    NS = CHECK_BUSY_RES ; 
                        end
                    SEND_RES_REQ:
                        begin
                            if(i_falling_edge_busy)     NS = WAIT_RESPS ;
                            else                        NS = SEND_RES_REQ ;
                        end
                    CHECK_RES :
                        begin
                            if(i_logged_results != 3'b111)     NS = IDLE ;
                            else                                NS = CHECK_BUSY_DONE ;
                        end
                    CHECK_BUSY_DONE :
                        begin
                          if(!i_sb_busy && !i_valid_partner)      NS = SEND_DONE_REQ ;
                          else                                    NS = CHECK_BUSY_DONE ;  
                        end
                    SEND_DONE_REQ :
                        begin
                           if(i_falling_edge_busy)     NS = WAIT_RESPS ;
                            else                        NS = SEND_DONE_REQ ; 
                        end
                    TX_END :
                        begin
                            if (i_mbinit_rpairclk_en)       NS = TX_END ;
                            else                            NS = IDLE ;
                        end
                    default:
                        begin
                            NS = IDLE ;
                        end                            
                endcase
            end 
    end

////////////////////////// OUTPUT BLOCK /////////////////////////////

always @ (posedge i_clk or negedge i_rst_n)
    begin
        if (!i_rst_n)
            begin
                o_encoded_sb_msg <= sb_msg_id'(0) ;
                o_msg_valid <= 'd0 ;
                o_clk_ptrn_en <= 'd0 ;
                o_error_req <= 'd0 ;
                o_TX_end <= 'd0 ;    
            end
        else
            begin
                o_encoded_sb_msg <= sb_msg_id'(0) ;
                o_msg_valid <= 'd0 ;
                o_clk_ptrn_en <= 'd0 ;
                //o_error_req <= 'd0 ;
                o_TX_end <= 'd0 ;
                case(NS)
                    IDLE:
                        begin
                          o_error_req <= 'd0 ;  
                        end
                    SEND_INIT_REQ:
                        begin
                            o_encoded_sb_msg <= SB_MBINIT_REPAIRCLK_INIT_REQ ;
                            o_msg_valid <= 'd1 ;
                        end
                    WAIT_RESPS:
                        begin
                            
                        end
                    PATTERN_EN:
                        begin
                            o_clk_ptrn_en <= 'd1 ;
                        end
                    CHECK_BUSY_RES:
                        begin
                            
                        end
                    SEND_RES_REQ :
                        begin
                            o_encoded_sb_msg <= SB_MBINIT_REPAIRCLK_RESULT_REQ ;
                            o_msg_valid <= 'd1 ;
                        end
                    CHECK_RES:
                        begin
                            if(i_logged_results != 3'b111)     o_error_req <= 'd1 ;
                        end
                    CHECK_BUSY_DONE :
                        begin
                            
                        end
                    SEND_DONE_REQ :
                        begin
                            o_encoded_sb_msg <= SB_MBINIT_REPAIRCLK_DONE_REQ ;
                            o_msg_valid <= 'd1 ;
                        end
                    TX_END:
                        begin
                           o_TX_end <= 'd1 ; 
                        end
                    default:
                        begin
                            o_encoded_sb_msg <= sb_msg_id'(0) ;
                            o_msg_valid <= 'd0 ;
                            o_clk_ptrn_en <= 'd0 ;
                            o_error_req <= 'd0 ;
                            o_TX_end <= 'd0 ;
                        end        
                endcase                 
            end     
    end

/*always @(posedge i_clk or negedge i_rst_n)
    begin
        if(!i_rst_n)
            begin
              o_msg_valid <= 'd0 ;  
            end
        else
            begin
                if((CS != NS) && (NS==SEND_INIT_REQ || NS==SEND_RES_REQ || NS == SEND_DONE_REQ))
                    begin
                        o_msg_valid <= 'd1 ;
                    end
                else if (i_falling_edge_busy)
                    begin
                       o_msg_valid <= 'd1 ; 
                    end
                else
                    begin
                        o_msg_valid <= 'd0 ;
                    end        
            end    
    end*/    

endmodule