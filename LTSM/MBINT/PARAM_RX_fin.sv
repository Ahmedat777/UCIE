import pckg::*;

module PARAM_RX_fin
(
    input                            i_clk , 
    input                            i_rst_n , 
    input                            i_MBINIT_en ,
    input                            i_sb_busy , i_falling_edge_busy ,
    input                            i_sb_valid ,
    input   sb_msg_id                i_decoded_sb_msg ,
    input   [15:0]                   i_module_parameters ,
    //--------------- same location as tx module ----------------------
    input  wire [3:0]                i_rf_data_rate,
    input  wire                      i_rf_sfes,
    input  wire                      i_rf_tarr,

    output  reg     sb_msg_id           o_encoded_SB_msg ,
    output  reg                         o_msg_valid ,
    output  reg                         o_data_valid ,
    output  reg  [15:0]                 o_parameters ,
    output  reg  [4:0]                  o_module_vswing ,
    output  reg                         o_module_clck_mode , 
    output  reg                         o_module_clck_phase ,
    output  reg  [3:0]                  o_final_max_data_rate ,     // maximum common data rate  
    output  reg                         o_PARAM_rx_end  
);

reg [15:0] saved_tx_parameters ;
reg [3:0] resolved_param ;
reg   data_reg , finish ;

// State machine states
typedef enum logic [2:0] {
    IDLE  ,
    WAIT_REQ  ,
    CHECK_PARAM  ,
    CHECK_SB  ,
    SEND_RESP ,
    PARAM_END  } state ;
state CS , NS ;
//transition logic
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
//next state logic
always @ (*)
    begin
        if (!i_MBINIT_en)
            begin
                NS = IDLE ;
            end
        else
            begin
                case (CS)
                    IDLE :
                        begin
                            if (i_MBINIT_en)
                                begin
                                    NS = WAIT_REQ ;
                                end
                            else
                                begin
                                    NS = IDLE ;
                                end    
                        end

                    WAIT_REQ :
                    begin
                        if (i_decoded_sb_msg == SB_MBINIT_PARAM_CONFIG_REQ && i_sb_valid)
                            begin
                                NS = CHECK_PARAM ;
                            end
                        else
                            begin
                                NS = WAIT_REQ ;
                            end    
                    end

                    CHECK_PARAM :
                        begin
                           if(finish) 
                                begin
                                    NS = CHECK_SB ;
                                end
                            else
                                begin
                                    NS = CHECK_PARAM ;
                                end
                        end

                    CHECK_SB :
                        begin
                            if(!i_sb_busy)
                                begin
                                    NS = SEND_RESP ;
                                end
                            else
                                begin
                                    NS = CHECK_SB ;
                                end    
                        end 

                    SEND_RESP :
                        begin
                            if(i_falling_edge_busy)
                                begin
                                    NS = PARAM_END ;
                                end
                            else
                                begin
                                    NS = SEND_RESP ;
                                end    
                        end 

                    PARAM_END :
                        begin
                            if (i_MBINIT_en)       NS = PARAM_END ;
                            else                            NS = IDLE ;
                        end          
                endcase
            end
    end

//output logic
always @ (posedge i_clk or negedge i_rst_n)
    begin
        if (!i_rst_n)
            begin
                o_encoded_SB_msg <= sb_msg_id'(0) ;
                o_msg_valid <= 'd0 ;
                o_PARAM_rx_end  <='d0 ;
                o_parameters <= 'd0 ;
                o_module_vswing <= 'd0 ;
                o_module_clck_mode <= 'd0 ; 
                o_module_clck_phase <= 'd0 ;
                o_final_max_data_rate <= 'd0 ; 
                finish <= 'd0 ;
                resolved_param <= 'd0 ;
                o_data_valid <= 'd0 ; 
                
            end
        else
            begin
                o_encoded_SB_msg <= sb_msg_id'(0) ;
                o_msg_valid <= 'd0 ;
                o_PARAM_rx_end  <='d0 ;
                o_parameters <= 'd0 ;
                o_module_vswing <= 'd0 ;
                o_module_clck_mode <= 'd0 ; 
                o_module_clck_phase <= 'd0 ;
                o_final_max_data_rate <= 'd0 ;
                o_data_valid <= 'd0 ;
                

                case (NS)
                    IDLE :
                        begin
                           resolved_param <= 'd0 ;
                           finish <= 'd0 ;  
                        end
                    WAIT_REQ :
                        begin
                           
                        end

                    CHECK_PARAM : 
                        begin
                            if(data_reg )
                                    begin
                                        if (saved_tx_parameters[3:0] <= i_rf_data_rate)
                                            begin
                                                resolved_param <= saved_tx_parameters[3:0] ;  
                                            end
                                        else
                                            begin
                                                resolved_param <= i_rf_data_rate ; 
                                            end

                                        finish <= 'd1 ;
                                    end         
                        end 

                    CHECK_SB :
                        begin
                          
                        end

                    SEND_RESP :
                        begin
                            o_encoded_SB_msg <= SB_MBINIT_PARAM_CONFIG_RESP ;
                            o_parameters <= {i_rf_tarr,i_rf_sfes,3'b000,saved_tx_parameters[10],saved_tx_parameters[9],5'b00000,resolved_param} ;
                            o_msg_valid <= 'd1 ;
                            o_data_valid <= 'd1 ;
                            
                        end

                    PARAM_END:
                        begin
                            o_module_vswing <= saved_tx_parameters[8:4] ;
                            o_module_clck_mode <= saved_tx_parameters[9] ; 
                            o_module_clck_phase <= saved_tx_parameters[10] ;
                            o_final_max_data_rate <= resolved_param ;
                            o_PARAM_rx_end <= 1'b1 ;
                        end
                endcase         
            end    
    end

always @(posedge i_clk or negedge i_rst_n)
    begin
        if(!i_rst_n || CS == IDLE)   
            begin
                saved_tx_parameters <= 'd0 ;
                data_reg <= 'd0 ;
            end 
        else
            begin                 
                if(i_decoded_sb_msg == SB_MBINIT_PARAM_CONFIG_REQ && i_sb_valid)
                    begin
                        saved_tx_parameters <= i_module_parameters ;
                        data_reg <= 'd1 ;
                    end                   
            end    
    end     
endmodule