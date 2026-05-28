import pckg::*;

module PARAM_TX_fin
(
    input                            i_clk , 
    input                            i_rst_n , 
    input                            i_MBINIT_en ,
    input                            i_sb_busy , i_falling_edge_busy ,
    input                            i_sb_valid ,
    input   sb_msg_id                i_decoded_sb_msg ,
    input   [15:0]                   i_resolved_parameters ,

    //----------------- my paramters from RF -----------------------
    input  wire [3:0]                i_rf_data_rate,        // the module maximum data rate > location : 9.5.1.4 UCIe Link DVSEC - UCIe Link Capability (Offset Ch) (bit[7:4])
    input  wire [4:0]                i_rf_vswing,           // my module > tx_voltage_swing > location: 9.5.3.22 PHY Capability (Offset 1000h) (bit[9:5])
    input  wire                      i_rf_clk_mode,         // the clock mode that my rx wants > location : 9.5.3.23 PHY Control (Offset 1004h) (bit [5])
    input  wire                      i_rf_clk_phase,        // the clock phase that my rx wants > location : 9.5.3.23 PHY Control (Offset 1004h) (bit [6])
    input  wire [1:0]                i_rf_module_id,        // setted to zero
    input  wire                      i_rf_ucie_sx8,         // 9.5.1.4 UCIe Link DVSEC - UCIe Link Capability (Offset Ch)(bit[22]) || 9.5.3.23 PHY Control (Offset 1004h)(bit[8])
    input  wire                      i_rf_sfes,             // setted to zero
    input  wire                      i_rf_tarr,             // location : 9.5.3.23 PHY Control (Offset 1004h) (bit[21])


    output  reg     sb_msg_id           o_encoded_SB_msg ,
    output  reg                         o_msg_valid ,
    output  reg                         o_data_valid ,
    output  reg  [15:0]                 o_parameters ,
    output  reg  [3:0]                  o_final_max_data_rate ,     // maximum common data rate     
    output  reg                         o_PARAM_tx_end  ,  
    output  reg                            error_req 
);
reg [15:0] saved_rx_parameters ;
reg pass , data_reg , finish ;

// State machine states
typedef enum logic [2:0]    {IDLE ,
                    PARAM_REQ  ,
                    WAIT_RESP  ,
                    CHECK_RESP  ,
                    PARAM_END  
                    } state ;
state CS , NS ;                    

//assign error_req = (finish && ~pass) ;
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
always @(*)
    begin
        if (!i_MBINIT_en || error_req)
            begin
                NS = IDLE ;
            end
        else
            begin
               case(CS)
                    IDLE:
                        begin
                            if(i_MBINIT_en && !i_sb_busy)
                                begin
                                    NS = PARAM_REQ ;
                                end
                            else
                                begin
                                    NS = IDLE ;
                                end    
                        end

                    PARAM_REQ:
                        begin
                            if (i_falling_edge_busy)
                                begin
                                    NS = WAIT_RESP ;
                                end
                            else
                                begin
                                    NS = PARAM_REQ ;
                                end    
                        end

                    WAIT_RESP:
                        begin
                            if(i_decoded_sb_msg == SB_MBINIT_PARAM_CONFIG_RESP && i_sb_valid)
                                begin
                                    NS = CHECK_RESP ;
                                end
                            else
                                begin
                                    NS = WAIT_RESP ;
                                end    
                        end

                    CHECK_RESP:
                        begin
                          if(pass && finish)  NS = PARAM_END ;
                          else      NS = CHECK_RESP ;

                        end
                        

                    PARAM_END:
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
        if(!i_rst_n)
            begin
                o_encoded_SB_msg <= sb_msg_id'(0) ;
                o_msg_valid <= 'd0 ;
                o_PARAM_tx_end  <='d0 ; 
                o_parameters <= 'd0 ;
                pass <= 'd0 ; 
                finish <= 'd0 ;
                o_final_max_data_rate <= 'd0 ;
                o_data_valid <= 'd0 ;
                
            end
        else
            begin
                o_encoded_SB_msg <= sb_msg_id'(0) ;
                o_msg_valid <= 'd0 ;
                o_PARAM_tx_end  <='d0 ; 
                o_parameters <= 'd0 ; 
                o_final_max_data_rate <= 'd0 ;
                o_data_valid <= 'd0 ;
                case (NS)
                        IDLE:
                            begin
                              pass <= 'd0 ;
                              finish <= 'd0 ;
                            end
                        PARAM_REQ:
                            begin
                                o_encoded_SB_msg <= SB_MBINIT_PARAM_CONFIG_REQ ;
                                o_parameters <= {i_rf_tarr,i_rf_sfes,i_rf_ucie_sx8,i_rf_module_id,i_rf_clk_phase,i_rf_clk_mode,i_rf_vswing,i_rf_data_rate} ;
                                o_msg_valid <= 'd1 ;
                                o_data_valid <= 'd1 ;
                               
                            end

                        WAIT_RESP:
                            begin
                                       
                            end 

                        CHECK_RESP:
                              begin

                                if(data_reg )
                                    begin
                                        pass <= (saved_rx_parameters[10] == i_rf_clk_phase && saved_rx_parameters[9] == i_rf_clk_mode && saved_rx_parameters[3:0] <= i_rf_data_rate ) ;
                                        finish <= 'd1 ;
                                    end
                                      
                              end

                        PARAM_END :
                            begin
                                o_final_max_data_rate <= saved_rx_parameters[3:0] ; 
                                o_PARAM_tx_end <= 1 ;
                            end              
                endcase     
            end    
    end


always @(posedge i_clk or negedge i_rst_n)
    begin
        if(!i_rst_n || CS == IDLE)   
            begin
                saved_rx_parameters <= 'd0 ;
                data_reg <= 'd0 ;
                error_req <= 'd0 ;
            end 
        else
            begin                 
                if(i_decoded_sb_msg == SB_MBINIT_PARAM_CONFIG_RESP && i_sb_valid)
                    begin
                        saved_rx_parameters <= i_resolved_parameters ;
                        data_reg <= 'd1 ;
                    end 
                if(finish && ~pass)     error_req <= 'd1 ;
                else                    error_req <= 'd0 ;                      
            end    
    end 

endmodule