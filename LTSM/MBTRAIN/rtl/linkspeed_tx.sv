module mbtrain_linkspeed_tx # ( parameter SB_MSG_WIDTH = 4, parameter LANE_STATUS_WIDTH = 2)
 (
    input          i_clk,
    input          i_rst_n,
    input          i_enable,

    // communication with Sideband
    input   [SB_MSG_WIDTH-1:0]  i_sb_msg,
    input                       i_sb_valid,
    input                       i_falling_edge_busy,

    // communicating with the rx
    input                        i_valid_rx,
    input   [SB_MSG_WIDTH-1:0]   i_msg_rx,
    // Test / Comparator Results 
    input          i_point_test_ack,
    input   [15:0] i_lane_results,   // 1 = the lane pass threshold, 0 = fail

    // from mb train controller
    input          i_phy_in_retrain_flag,    
    input          i_tx_first_8_lanes_ok_mbinit ,i_tx_second_8_lanes_ok_mbinit ,  
    
    // from reg file
    input          i_runtime_config_change,  // Runtime Link Test Control change (some configs saved in the reg file)

    // outputs 
    output reg         o_point_test_en,

    // communication with Sideband 
    output reg  [SB_MSG_WIDTH-1:0]  o_sb_msg,
    output reg                      o_valid_tx, 
    output reg                      o_timeout_disable, 
    // to phyretrain block
    output reg  [LANE_STATUS_WIDTH-1:0]    o_linkspeed_lanes_status,   // 0h: IDLE , 1h: No Lane errors, 2h: Lane errors & faulty Lanes are repairable, 3h: Lane errors & faulty Lanes cannot be repaired
    // communication with the controller
    output reg                      o_tx_first_8_lanes_ok_linkspeed , o_tx_second_8_lanes_ok_linkspeed,
    output reg                      o_ack,
    output reg                      o_clear_phy_in_retrain_flag,
    output reg                      o_exit_phyretrain,
    output reg                      o_exit_repair,
    output reg                      o_exit_speedidle
    
);

    /* -------- Sideband Messages -------- */
    
    localparam  [SB_MSG_WIDTH-1:0] START_REQ                   = 1;
    localparam  [SB_MSG_WIDTH-1:0] START_RESP                  = 2;
    localparam  [SB_MSG_WIDTH-1:0] ERROR_REQ                   = 3;
    localparam  [SB_MSG_WIDTH-1:0] ERROR_RESP                  = 4;
    localparam  [SB_MSG_WIDTH-1:0] EXIT_TO_REPAIR_REQ          = 5;
    localparam  [SB_MSG_WIDTH-1:0] EXIT_TO_REPAIR_RESP         = 6;
    localparam  [SB_MSG_WIDTH-1:0] EXIT_TO_SPEED_DEGRADE_REQ   = 7;
    localparam  [SB_MSG_WIDTH-1:0] EXIT_TO_SPEED_DEGRADE_RESP  = 8;
    localparam  [SB_MSG_WIDTH-1:0] DONE_REQ                    = 9;
    localparam  [SB_MSG_WIDTH-1:0] DONE_RESP                   = 10;
    localparam  [SB_MSG_WIDTH-1:0] EXIT_TO_PHYRETRAIN_REQ      = 11;
    localparam  [SB_MSG_WIDTH-1:0] EXIT_TO_PHYRETRAIN_RESP     = 12;

    ////////// FSM States /////////////
    
    typedef enum logic [3:0] {
    IDLE ,
    START ,
    POINT_TEST ,
    ANALYSIS ,
    ERROR_HANDLING ,
    EXIT_PHY_RETRAIN ,
    EXIT_REPAIR ,
    EXIT_SPEED_DEGRADE , 
    DONE ,
    TEST_END
    } tx_state;

    tx_state current_state_tx, next_state_tx;


    ///////////// Lanes calculations //////////////
    wire first8_ok ;
    wire second8_ok ;
    wire all_ok ;
    wire any_ok ;

    assign first8_ok  = &i_lane_results[7:0];
    assign second8_ok = &i_lane_results[15:8];
    assign all_ok     = first8_ok & second8_ok;
    assign any_ok     = first8_ok || second8_ok;

    

    /////////////// valid decisions ///////////////
    wire    valid_1, valid_2, valid_3, valid_4, valid_5, valid_6;

    assign valid_1 = next_state_tx == START;
    assign valid_2 = (next_state_tx == ERROR_HANDLING && (!i_sb_valid  && i_sb_msg != EXIT_TO_PHYRETRAIN_REQ)) ; 
    assign valid_3 = next_state_tx == EXIT_PHY_RETRAIN;
    assign valid_4 = (next_state_tx == EXIT_REPAIR && (!i_sb_valid  && i_sb_msg != EXIT_TO_SPEED_DEGRADE_REQ)) ;
    assign valid_5 = next_state_tx == EXIT_SPEED_DEGRADE;
    assign valid_6 = next_state_tx == DONE;

    /* -------- State Register -------- */
    always @(posedge i_clk or negedge i_rst_n) begin
        if(!i_rst_n)
            current_state_tx <= IDLE;
        else
            current_state_tx <= next_state_tx;
    end

    /* -------- Next State Logic -------- */
    always @(*) begin
        
        case(current_state_tx)

            IDLE : next_state_tx = (i_enable)? START : IDLE;

            START: begin
                if(!i_enable)
                    next_state_tx = IDLE; 
                else if(i_sb_msg == START_RESP && i_sb_valid)
                    next_state_tx = POINT_TEST;
                else
                    next_state_tx = START;
                 
            end

            POINT_TEST: begin
                if(!i_enable)
                    next_state_tx = IDLE; 
                else if(i_point_test_ack)
                    next_state_tx = ANALYSIS;
                else
                    next_state_tx = POINT_TEST;
            end

            ANALYSIS: begin                                         
                
                if(!i_enable)
                    next_state_tx = IDLE;
                else if(i_phy_in_retrain_flag && i_runtime_config_change)
                    next_state_tx = EXIT_PHY_RETRAIN;

                else if(!all_ok)
                    next_state_tx = ERROR_HANDLING;

                else if(all_ok)
                    next_state_tx = DONE;

                else
                    next_state_tx = ANALYSIS;
            end

            ERROR_HANDLING: begin
                if(!i_enable)
                    next_state_tx = IDLE;
                /* partner forces phy retrain */
                else if(i_sb_valid && i_sb_msg == EXIT_TO_PHYRETRAIN_REQ) //////////////////
                    next_state_tx = EXIT_PHY_RETRAIN;

                /* normal error resp path */
                else if(i_sb_valid && i_sb_msg == ERROR_RESP) begin
                    if(any_ok)
                        next_state_tx = EXIT_REPAIR;
                    else if(!all_ok)
                        next_state_tx = EXIT_SPEED_DEGRADE;
                end
                else
                    next_state_tx = ERROR_HANDLING;
            end
           
            EXIT_PHY_RETRAIN: begin
                if(!i_enable)
                    next_state_tx = IDLE;
                else if(i_sb_valid && i_sb_msg == EXIT_TO_PHYRETRAIN_RESP) // the partner respond to our request.
                    next_state_tx = DONE;
                else 
                    next_state_tx = EXIT_PHY_RETRAIN;
            end

            EXIT_REPAIR: begin
                if(!i_enable)
                    next_state_tx = IDLE;
                /* partner forces speed degradation */
                if(i_sb_valid && i_sb_msg == EXIT_TO_SPEED_DEGRADE_REQ)
                    next_state_tx = EXIT_SPEED_DEGRADE;
                else if(i_sb_valid && i_sb_msg == EXIT_TO_REPAIR_RESP)
                    next_state_tx = DONE;
                else
                    next_state_tx = EXIT_REPAIR;
            end

            EXIT_SPEED_DEGRADE: begin
                if(!i_enable) 
                    next_state_tx = IDLE;
                else if(i_sb_valid && i_sb_msg == EXIT_TO_SPEED_DEGRADE_RESP) // the partner respond to our request.
                    next_state_tx = DONE;
                else
                    next_state_tx = EXIT_SPEED_DEGRADE;
            end
            
            DONE: begin
                if(!i_enable) 
                    next_state_tx = IDLE;
                else if(i_sb_valid && i_sb_msg == DONE_RESP)
                    next_state_tx = TEST_END;
                else
                    next_state_tx = DONE;
            end
            
            TEST_END: begin
                next_state_tx = (!i_enable)? IDLE : TEST_END;
            end

            default: next_state_tx = IDLE;
        endcase
    end

    /* -------- Output Logic -------- */
   
    always @(posedge i_clk or negedge i_rst_n) begin
        if(!i_rst_n) begin
            o_sb_msg <= 0;
            o_point_test_en <= 0;
            o_timeout_disable <= 0;
            o_ack <= 0;
        end
        
        else begin
            o_sb_msg <= 0;
            o_point_test_en <= 0;
            o_timeout_disable <= 0;
            o_ack <= 0;

            case(next_state_tx)

                START: begin
                    o_sb_msg   <= START_REQ;
                    
                end

                POINT_TEST: begin
                    o_point_test_en <= 1;
                end

                ANALYSIS: begin
                end

                ERROR_HANDLING: begin
                    
                    if(i_sb_valid && i_sb_msg == EXIT_TO_PHYRETRAIN_REQ) begin
                        o_timeout_disable <= 1; 
                        o_sb_msg <= 0;
                    end
                    else
                        o_sb_msg <= ERROR_REQ;
                end

                EXIT_PHY_RETRAIN: begin
                    o_sb_msg   <= EXIT_TO_PHYRETRAIN_REQ;
                end

                EXIT_REPAIR: begin
                    o_sb_msg   <= EXIT_TO_REPAIR_REQ;
                    if(i_sb_valid && i_sb_msg == EXIT_TO_SPEED_DEGRADE_REQ) begin
                        o_timeout_disable <= 1; 
                        o_sb_msg   <= 0;
                    end
                end

                EXIT_SPEED_DEGRADE: begin
                    
                    o_sb_msg   <= EXIT_TO_SPEED_DEGRADE_REQ;
                    
                end

                DONE: begin
                    o_sb_msg   <= DONE_REQ;
                end
                
                TEST_END: begin
                    o_ack <= 1;
                end

            endcase
        end
    end

//////////////////////// handeling output flags ///////////////////////////

always @(posedge i_clk or negedge i_rst_n) begin
    if(!i_rst_n) begin
        o_clear_phy_in_retrain_flag <= 0;
    end
    else if((i_sb_valid && i_sb_msg == ERROR_RESP) || (i_valid_rx && i_msg_rx == ERROR_RESP)) begin
        o_clear_phy_in_retrain_flag <= 1;
    end
end

always @(posedge i_clk or negedge i_rst_n) begin
    if(!i_rst_n) begin
        o_exit_phyretrain <= 0;
    end
    else if((i_sb_valid && i_sb_msg == EXIT_TO_PHYRETRAIN_RESP) || (i_valid_rx && i_msg_rx == EXIT_TO_PHYRETRAIN_RESP)) begin // the partner or the rx responded(depending on which one requested).
        o_exit_phyretrain <= 1;
    end
end

always @(posedge i_clk or negedge i_rst_n) begin
    if(!i_rst_n) begin
        o_exit_repair <= 0;
    end
    else if((i_sb_valid && i_sb_msg == EXIT_TO_REPAIR_RESP) || (i_valid_rx && i_msg_rx == EXIT_TO_REPAIR_RESP)) begin
        o_exit_repair <= 1;
    end
end

always @(posedge i_clk or negedge i_rst_n) begin
    if(!i_rst_n) begin
        o_exit_speedidle <= 0;
    end
    else if((i_sb_valid && i_sb_msg == EXIT_TO_SPEED_DEGRADE_RESP) || (i_valid_rx && i_msg_rx == EXIT_TO_SPEED_DEGRADE_RESP)) begin
        o_exit_speedidle <= 1;
    end
end


/////////////////////// handeling the lanes status output to phyretrain block ///////////////////////

always @(posedge i_clk or negedge i_rst_n) begin
    if(!i_rst_n)
            o_linkspeed_lanes_status <= 'b00;
    else if(all_ok)
            o_linkspeed_lanes_status <= 'b01;
    else if(any_ok)
            o_linkspeed_lanes_status <= 'b10;
    else if(!all_ok)
            o_linkspeed_lanes_status <= 'b11;
end

///////////////////////////////////////////////////////////////////

always @(posedge i_clk or negedge i_rst_n) begin
    
    if (!i_rst_n) begin
        o_tx_first_8_lanes_ok_linkspeed <= 0;
        o_tx_second_8_lanes_ok_linkspeed <= 0;
    end
    
    else if (i_sb_valid && i_sb_msg == EXIT_TO_SPEED_DEGRADE_RESP) begin // since we are entering speed degrade mode then we will stay on the results from
       // the mbinit as no degradation in the lanes itself, just changing speeds for better results.
        o_tx_first_8_lanes_ok_linkspeed <= i_tx_first_8_lanes_ok_mbinit;
        o_tx_second_8_lanes_ok_linkspeed <= i_tx_second_8_lanes_ok_mbinit;
    end

    else if(current_state_tx == ANALYSIS && next_state_tx == DONE) begin 
        o_tx_first_8_lanes_ok_linkspeed <= first8_ok;
        o_tx_second_8_lanes_ok_linkspeed <= second8_ok;
    end
    else if(i_sb_valid && (i_sb_msg == EXIT_TO_PHYRETRAIN_RESP || i_sb_msg == EXIT_TO_REPAIR_RESP )) begin // now after the analysis state and we depend on 
      //the tests results to decide which lanes are good for tx.
        o_tx_first_8_lanes_ok_linkspeed <= first8_ok;
        o_tx_second_8_lanes_ok_linkspeed <= second8_ok;
    end
        
end
/////////////////////// Valid handeling /////////////////////////

always @(posedge i_clk or negedge i_rst_n) begin
    if(!i_rst_n)
      o_valid_tx <= 1'b0;
    else if((current_state_tx != next_state_tx) && (valid_1 || valid_2 || valid_3 || valid_4 || valid_5 || valid_6))
      o_valid_tx <= 1'b1;
    else if(i_falling_edge_busy || o_sb_msg == 0) // in case the message was cleared due to a partner forced transition, we need to clear the valid as well.
      o_valid_tx <= 1'b0;
end

endmodule