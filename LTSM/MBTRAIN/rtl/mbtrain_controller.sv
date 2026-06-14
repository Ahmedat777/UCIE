module mbtrain_controller  #parameter (SUBSTATE_SIZE = 5);

(
// inputs
  input          i_clk,
  input          i_rst_n,
  input          i_enable,   

  //input signal from mbinit 
	input [3:0] i_highest_common_speed,
	input       i_tx_first_8_lanes_ok_mbinit , i_tx_second_8_lanes_ok_mbinit,
	input	     	i_rx_first_8_lanes_ok_mbinit , i_rx_second_8_lanes_ok_mbinit,

  //comming from linkspeed
	input       i_tx_first_8_lanes_ok_linkspeed , i_tx_second_8_lanes_ok_linkspeed,
  input       i_clear_phy_in_retrain_flag,
	input       i_exit_phyretrain,
  input       i_exit_repair,
  input       i_exit_speedidle,

  //comming from  repair
  input	    	i_rx_first_8_lanes_ok_repair , i_rx_second_8_lanes_ok_repair,
  
  
  // acks from the substates blocks
  input          i_valvref_ack,
  input          i_datavref_ack,
  input          i_speedidle_ack,
  input          i_txselfcal_ack,
  input          i_rxclkcal_ack,
  input          i_valtraincenter_ack,
  input          i_valtrainvref_ack,
  input          i_datatraincenter1_ack,
  input          i_datatrainvref_ack,
  input          i_rxdeskew_ack,
  input          i_datatraincenter2_ack,
  input          i_linkspeed_ack,
  input          i_repair_ack,

// outputs 
  // enables to the substates blocks
  output  reg        o_valvref_en,
  output  reg        o_datavref_en,
  output  reg        o_speedidle_en,
  output  reg        o_txselfcal_en,
  output  reg        o_rxclkcal_en,
  output  reg        o_valtraincenter_en,
  output  reg        o_valtrainvref_en,
  output  reg        o_datatraincenter1_en,
  output  reg        o_datatrainvref_en,
  output  reg        o_rxdeskew_en,
  output  reg        o_datatraincenter2_en,
  output  reg        o_linkspeed_en,
  output  reg        o_repair_en,
  
  output  reg        o_mbtrain_end, // indicating the mbtrain state is done
  
  // going to the point test block (tx point test)
  output  reg        o_tx_data_or_valid_test,  // 0 = data test, 1 = valid test
  output  reg        o_tx_scrumbeld_or_perlane, // 0 = scrumbeld (LFSR) test, 1 = perlane test 
  
  //
  output reg        o_tx_first_8_lanes_ok , o_tx_second_8_lanes_ok,
	output reg	    	o_rx_first_8_lanes_ok , o_rx_second_8_lanes_ok,

  // going to the sideband
  output  reg        o_substate

  // going to linkspeed
  output  reg        o_phy_in_retrain_flag,
  
  //
  output reg [3:0] o_curret_operating_speed 

);
///////////////////// states ////////////////////////

//states
typedef enum logic [SUBSTATE_SIZE - 1:0] {
  IDLE                = 0,
  VALVREF             = 1,
  DATAVREF            = 3,
  SPEEDIDLE           = 4,
  TXSELFCAL           = 6,
  RXCLKCAL            = 8,
  VALTRAINCENTER      = 10,
  VALTRAINVREF        = 12,
  DATATRAINCENTER1    = 14,
  DATATRAINVREF       = 16, 
  RXDESKEW            = 18,
  DATATRAINCENTER2    = 20,
  LINKSPEED           = 21,
  REPAIR              = 22, 
  MBTRAIN_END         = 23
} controller_state; 
 
controller_state current_state, next_state;

always @(posedge i_clk or negedge i_rst_n) begin
  if (!i_rst_n)
   current_state <= IDLE;
  else
   current_state <= next_state;
end
/////////////////// current state logic /////////////////

always @(posedge i_clk or negedge i_rst_n) begin
  
  if (!i_rst_n)
   o_phy_in_retrain_flag <= 1'b0;
  else if (current_state == PHYRETRAIN && next_state != PHYRETRAIN) // if we are exiting phy retrain state then clear the flag
   o_phy_in_retrain_flag <= 1'b1;
  else if (i_clear_phy_in_retrain_flag)
   o_phy_in_retrain_flag <= 1'b0;

end

//////////////////// next state logic ////////////////////

always @(*) begin

 case (current_state)

  IDLE : next_state = (i_enable)? VALVREF : IDLE; 

  VALVREF : if(!i_enable)
              next_state = IDLE;
            else if(i_valvref_ack)
              next_state = DATAVREF;
            else
              next_state = VALVREF;

  DATAVREF : if(!i_enable)
              next_state = IDLE;
            else if(i_datavref_ack)
              next_state = SPEEDIDLE;
            else
              next_state = DATAVREF;

  SPEEDIDLE : if(!i_enable)
              next_state = IDLE;
            else if(i_speedidle_ack)
              next_state = TXSELFCAL;
            else
              next_state = SPEEDIDLE;

  TXSELFCAL : if(!i_enable)
              next_state = IDLE;
            else if(i_txselfcal_ack)
              next_state = RXCLKCAL;
            else
              next_state = TXSELFCAL;

  RXCLKCAL : if(!i_enable)
              next_state = IDLE;
            else if(i_rxclkcal_ack)
              next_state = VALTRAINCENTER;
            else
              next_state = RXCLKCAL;

  VALTRAINCENTER : if(!i_enable)
                      next_state = IDLE;
                   else if(i_valtraincenter_ack)
                      next_state = VALTRAINVREF;
                   else
                      next_state = RXCLKCAL;

  VALTRAINVREF :  if(!i_enable)
                      next_state = IDLE;
                   else if(i_valtrainvref_ack)
                      next_state = DATATRAINCENTER1;
                   else
                      next_state = VALTRAINVREF;

  DATATRAINCENTER1 :  if(!i_enable)
                          next_state = IDLE;
                      else if(i_datatraincenter1_ack)
                          next_state = DATATRAINVREF;
                      else
                          next_state = DATATRAINCENTER1;    

  DATATRAINVREF :     if(!i_enable)
                          next_state = IDLE;
                      else if(i_datatrainvref_ack)
                          next_state = RXDESKEW;
                      else
                          next_state = DATATRAINVREF;
 
  RXDESKEW :  if(!i_enable)
                  next_state = IDLE;
              else if(i_rxdeskew_ack)
                  next_state = DATATRAINCENTER2;
              else
                  next_state = RXDESKEW;

  DATATRAINCENTER2 :  if(!i_enable)
                          next_state = IDLE;
                      else if(i_rxdeskew_ack)
                          next_state = LINKSPEED;
                      else
                          next_state = DATATRAINCENTER2;     

  LINKSPEED :         if(!i_enable)
                          next_state = IDLE;

                      else if(i_linkspeed_ack) begin 
                           if(i_exit_phyretrain)
                              next_state = PHYRETRAIN;
                           else if(i_exit_speedidle)
                              next_state = SPEEDIDLE;
                           else if(i_exit_repair)
                              next_state = REPAIR;
                           else
                              next_state = MBTRAIN_END;
                      end
                      else
                          next_state = LINKSPEED;

  REPAIR :            if(!i_enable)
                          next_state = IDLE;
                      else if(i_repair_ack)
                          next_state = TXSELFCAL; // go back to txselfcal to redo the training with repaired lanes
                      else
                          next_state = REPAIR;

  MBTRAIN_END : next_state = (!i_enable)? IDLE : MBTRAIN_END;

  default : next_state = IDLE;

 endcase
end

//////////////////////// output logic //////////////////////////////

always @(posedge i_clk or negedge i_rst_n) begin

  if (!i_rst_n) begin
     o_tx_data_or_valid_test <= 1'b0;
     o_tx_scrumbeld_or_perlane <= 1'b0;
     o_valvref_en          <= 1'b0;
     o_datavref_en         <= 1'b0;
     o_speedidle_en        <= 1'b0;
     o_txselfcal_en        <= 1'b0;
     o_rxclkcal_en         <= 1'b0;
     o_valtraincenter_en   <= 1'b0;
     o_valtrainvref_en     <= 1'b0;
     o_datatraincenter1_en <= 1'b0;
     o_datatrainvref_en    <= 1'b0;
     o_rxdeskew_en         <= 1'b0;
     o_datatraincenter2_en <= 1'b0;
     o_linkspeed_en        <= 1'b0;
     o_repair_en           <= 1'b0;
     o_mbtrain_end         <= 1'b0;
     o_data_or_valid_test  <= 1'b0;
     o_substate            <= 'd0;
  end
 
  else begin
     o_tx_data_or_valid_test <= 1'b0;
     o_tx_scrumbeld_or_perlane <= 1'b0;
     o_valvref_en          <= 1'b0;
     o_datavref_en         <= 1'b0;
     o_speedidle_en        <= 1'b0;
     o_txselfcal_en        <= 1'b0;
     o_rxclkcal_en         <= 1'b0;
     o_valtraincenter_en   <= 1'b0;
     o_valtrainvref_en     <= 1'b0;
     o_datatraincenter1_en <= 1'b0;
     o_datatrainvref_en    <= 1'b0;
     o_rxdeskew_en         <= 1'b0;
     o_datatraincenter2_en <= 1'b0;
     o_linkspeed_en        <= 1'b0;
     o_repair_en           <= 1'b0;
     o_mbtrain_end         <= 1'b0;
     o_data_or_valid_test  <= 1'b0;
     o_substate            <= 'd0;
   case (next_state)
    
    VALVREF : begin
        o_valvref_en          <= 1'b1;
        o_data_or_valid_test  <= 1'b1;
        o_substate            <= VALVREF; 
        o_tx_data_or_valid_test <= 1'b1;
        
    end

    DATAVREF : begin        
        o_datavref_en         <= 1'b1; 
        o_data_or_valid_test  <= 1'b0;
        o_substate            <= DATAVREF;
        o_tx_data_or_valid_test <= 1'b0;
        o_tx_scrumbeld_or_perlane <= 1'b1;
    end
    
    SPEEDIDLE : begin
        o_speedidle_en        <= 1'b1;
        o_substate            <= SPEEDIDLE;
    end
    
    TXSELFCAL : begin      
        o_txselfcal_en        <= 1'b1;
        o_substate            <= TXSELFCAL;
    end
    
    RXCLKCAL : begin
        o_rxclkcal_en         <= 1'b1;
        o_substate            <= RXCLKCAL;
    end
    
    VALTRAINCENTER : begin  
        o_valtraincenter_en   <= 1'b1;
        o_data_or_valid_test  <= 1'b1;
        o_substate            <= VALTRAINCENTER;
        o_tx_data_or_valid_test <= 1'b1;
         
    end
    VALTRAINVREF : begin    
        o_valtrainvref_en     <= 1'b1;
        o_data_or_valid_test  <= 1'b1;
        o_substate            <= VALTRAINVREF;
        o_tx_data_or_valid_test <= 1'b1;
     
    end
    DATATRAINCENTER1 : begin 
        o_datatraincenter1_en <= 1'b1;
        o_data_or_valid_test  <= 1'b0;
        o_substate            <= DATATRAINCENTER1;
        o_tx_data_or_valid_test <= 1'b0;
        o_tx_scrumbeld_or_perlane <= 1'b0;
    end
    DATATRAINVREF : begin
        o_datatrainvref_en    <= 1'b1;
        o_data_or_valid_test  <= 1'b0;
        o_substate            <= DATATRAINVREF;
        o_tx_data_or_valid_test <= 1'b0;
        o_tx_scrumbeld_or_perlane <= 1'b0;
    end
    RXDESKEW : begin
        o_rxdeskew_en         <= 1'b1;
        o_substate            <= RXDESKEW;
    end
    
    DATATRAINCENTER2 : begin 
        o_datatraincenter2_en <= 1'b1;
        o_data_or_valid_test  <= 1'b0;
        o_substate            <= DATATRAINCENTER2;
        o_tx_data_or_valid_test <= 1'b0;
        o_tx_scrumbeld_or_perlane <= 1'b0;
    end
    
    LINKSPEED : begin
        o_linkspeed_en        <= 1'b1;
        o_substate            <= LINKSPEED;
        o_tx_data_or_valid_test <= 1'b0;
        o_tx_scrumbeld_or_perlane <= 1'b0;
    end
    
    REPAIR : begin
        o_repair_en           <= 1'b1;
        o_substate            <= REPAIR;
    end
    
    MBTRAIN_END :    o_mbtrain_end     <= 1'b1;

   endcase
end


/////////////////////////////////// handling operating speed ///////////////////////////////

always @(posedge i_clk or negedge i_rst_n) begin 
	if(!i_rst_n) begin
		o_curret_operating_speed <= 0;
	end 
  else if (current_state==IDLE && next_state == VALVREF) begin
		o_curret_operating_speed <= 0; // first speed which is the lowest one 4GT/S 
  end 
  else if (current_state==DATAVREF && next_state == SPEED_IDLE) begin
		o_curret_operating_speed <= i_highest_common_speed; 
	end 
  else if ( (current_state == LINKSPEED || current_state == PHYRETRAIN) && next_state==SPEED_IDLE) begin
		o_curret_operating_speed<= o_curret_operating_speed-1;
	end 
end

///////////////////////////////////////////// tx and rx lane ok logic ////////////////////////////////////////////
always @(posedge i_clk or negedge i_rst_n) begin 
	if(!i_rst_n) begin
		o_tx_first_8_lanes_ok  <=0;
		o_tx_second_8_lanes_ok <=0;
		o_rx_first_8_lanes_ok  <=0;
		o_rx_second_8_lanes_ok <=0;

	end else if (current_state==IDLE && next_state == VALVREF) begin
		o_tx_first_8_lanes_ok  <=i_tx_first_8_lanes_ok_mbinit;
		o_tx_second_8_lanes_ok <=i_tx_second_8_lanes_ok_mbinit;
		o_rx_first_8_lanes_ok  <=i_rx_first_8_lanes_ok_mbinit;
		o_rx_second_8_lanes_ok <=i_rx_second_8_lanes_ok_mbinit;
	end else if(current_state==LINKSPEED && next_state!=LINKSPEED && (i_tx_first_8_lanes_ok_linkspeed || i_tx_second_8_lanes_ok_linkspeed)) begin 
    // we use the or part cus if found unrepairable and went to speedidle then stay with values of the mbinit.
	
			o_tx_first_8_lanes_ok  <=i_tx_first_8_lanes_ok_linkspeed;
		  o_tx_second_8_lanes_ok <=i_tx_second_8_lanes_ok_linkspeed;
	
  end else if(current_state==REPAIR && next_state!=REPAIR) begin
		  o_rx_first_8_lanes_ok  <=i_rx_first_8_lanes_ok_repair;
		  o_rx_second_8_lanes_ok <=i_rx_second_8_lanes_ok_repair;
	end
end