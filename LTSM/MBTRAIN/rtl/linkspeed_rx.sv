module mbtrain_linkspeed_rx # ( parameter SB_MSG_WIDTH = 4)
 ( 
  // inputs
    input          i_clk,
    input          i_rst_n,
    input          i_enable,

    // communication with Sideband
    input   [SB_MSG_WIDTH-1:0]  i_sb_msg,
    input                       i_sb_valid,
    input                       i_falling_edge_busy,

    // communicating with the tx linkspeed block
     input   [SB_MSG_WIDTH-1:0]  i_msg_tx,
     input                       i_valid_tx,
   
    // point Test
    input          i_point_test_ack,

  // outputs 
    output reg         o_point_test_en,

    // communication with Sideband 
    output reg  [SB_MSG_WIDTH-1:0]  o_sb_msg,
    output reg                      o_valid_rx, 
 
    // communication with the controller
    output reg                      o_ack
    
    

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
    WAIT_START ,
    SEND_START ,
    POINT_TEST ,
    WAIT_FOR_REQ ,
    ERROR_HANDLING ,
    EXIT_PHY_RETRAIN ,
    EXIT_REPAIR ,
    EXIT_SPEED_DEGRADE , 
    DONE ,
    TEST_END
    } rx_state;
      rx_state current_state_rx, next_state_rx;
    
    /////////////// valid decisions ///////////////
    wire    valid_1, valid_2, valid_3, valid_4, valid_5, valid_6;

    assign valid_1 = next_state_rx == SEND_START;
    assign valid_2 = (next_state_rx == ERROR_HANDLING &&  i_msg_tx != EXIT_TO_PHYRETRAIN_REQ); 
    assign valid_3 = next_state_rx == EXIT_PHY_RETRAIN;
    assign valid_4 = (next_state_rx == EXIT_REPAIR && (i_msg_tx != EXIT_TO_SPEED_DEGRADE_REQ) && (i_msg_tx != EXIT_TO_PHYRETRAIN_REQ)) ;
    assign valid_5 = (next_state_rx == EXIT_SPEED_DEGRADE && i_msg_tx != EXIT_TO_PHYRETRAIN_REQ);
    assign valid_6 = next_state_rx == DONE;

    /* -------- State Register -------- */
    always @(posedge i_clk or negedge i_rst_n) begin
        if(!i_rst_n)
            current_state_rx <= IDLE;
        else
            current_state_rx <= next_state_rx;
    end

    /* -------- Next State Logic -------- */
    always @(*) begin
        
        case(current_state_rx)

            IDLE : next_state_rx = (i_enable)? WAIT_START : IDLE;

            WAIT_START: begin
                if(!i_enable)
                    next_state_rx = IDLE; 
                else if(i_sb_msg == START_REQ && i_sb_valid)
                    next_state_rx = SEND_START;
                else
                    next_state_rx = WAIT_START;
                 
            end

             SEND_START: begin
                if(!i_enable)
                    next_state_rx = IDLE; 
                else if(i_falling_edge_busy) 
                    next_state_rx = POINT_TEST;
                else
                    next_state_rx = SEND_START;
                 
            end

            POINT_TEST: begin
                if(!i_enable)
                    next_state_rx = IDLE; 
                else if(i_point_test_ack)
                    next_state_rx = WAIT_FOR_REQ;
                else
                    next_state_rx = POINT_TEST;
            end

            WAIT_FOR_REQ: begin                                         
                
                if(!i_enable)
                    next_state_rx = IDLE;
                else if(i_sb_valid && i_sb_msg == EXIT_TO_PHYRETRAIN_REQ)
                    next_state_rx = EXIT_PHY_RETRAIN;
                else if(i_sb_valid && i_sb_msg == ERROR_REQ)
                    next_state_rx = ERROR_HANDLING;
                else if(i_sb_valid && i_sb_msg == DONE_REQ)
                    next_state_rx = DONE;
                else
                    next_state_rx = WAIT_FOR_REQ;
            end

            ERROR_HANDLING: begin
              
                if(!i_enable)
                    next_state_rx = IDLE;
                // making sure no forcing phyretrain by our module
                else if(i_sb_valid && i_sb_msg == EXIT_TO_PHYRETRAIN_REQ)
                    next_state_rx = EXIT_PHY_RETRAIN;
                else if(i_sb_valid && i_sb_msg == EXIT_TO_SPEED_DEGRADE_REQ)
                    next_state_rx = EXIT_SPEED_DEGRADE;
                else if(i_sb_valid && i_sb_msg == EXIT_TO_REPAIR_REQ)
                    next_state_rx = EXIT_REPAIR;
                else
                    next_state_rx = ERROR_HANDLING;    
                
            end

            EXIT_PHY_RETRAIN: begin
                if(!i_enable)
                    next_state_rx = IDLE;
                else if(i_sb_valid && i_sb_msg == DONE_REQ) 
                    next_state_rx = DONE;
                else 
                    next_state_rx = EXIT_PHY_RETRAIN;
            end

            EXIT_REPAIR: begin
                if(!i_enable)
                    next_state_rx = IDLE;
                /* our module forces phyretrain */
                else if(i_sb_valid && i_sb_msg == EXIT_TO_PHYRETRAIN_REQ)
                    next_state_rx = EXIT_PHY_RETRAIN;
                /* our module forces speed degradation */
                else if(i_sb_valid && i_sb_msg == EXIT_TO_SPEED_DEGRADE_REQ)
                    next_state_rx = EXIT_SPEED_DEGRADE;
                
                else if(i_sb_valid && i_sb_msg == DONE_REQ)
                    next_state_rx = DONE;
                else
                    next_state_rx = EXIT_REPAIR;
            end

            EXIT_SPEED_DEGRADE: begin
                if(!i_enable) 
                    next_state_rx = IDLE;
                /* our module forces phyretrain */
                else if(i_sb_valid && i_sb_msg == EXIT_TO_PHYRETRAIN_REQ)
                    next_state_rx = EXIT_PHY_RETRAIN;
                else if(i_sb_valid && i_sb_msg == DONE_REQ)
                    next_state_rx = DONE;
                else
                    next_state_rx = EXIT_SPEED_DEGRADE;
            end
            
            DONE: begin
                if(!i_enable) 
                    next_state_rx = IDLE;
                else if(i_falling_edge_busy)
                    next_state_rx = TEST_END;
                else
                    next_state_rx = DONE;
            end
            
            TEST_END: begin
                next_state_rx = (!i_enable)? IDLE : TEST_END;
            end
            
                default: next_state_rx = IDLE;
        endcase
    end

    /* -------- Output Logic -------- */
   
    always @(posedge i_clk or negedge i_rst_n) begin
        if(!i_rst_n) begin
            o_sb_msg <= 0;
            o_point_test_en <= 0;
            o_ack <= 0;
        end
        
        else begin
            o_sb_msg <= 0;
            o_point_test_en <= 0;
            o_ack <= 0;

            case(next_state_rx)

                SEND_START: begin
                    o_sb_msg   <= START_RESP;    
                end

                POINT_TEST: begin
                    o_point_test_en <= 1;
                end

                ERROR_HANDLING: begin

                    if(i_sb_valid && i_sb_msg == EXIT_TO_PHYRETRAIN_REQ) begin
                        o_sb_msg   <= 0;
                    end
                    else
                    o_sb_msg   <= ERROR_RESP;  
                
                end
             
                EXIT_PHY_RETRAIN: begin
                    o_sb_msg   <= EXIT_TO_PHYRETRAIN_RESP;
                end

                EXIT_REPAIR: begin
                    if(i_sb_valid && (i_sb_msg == EXIT_TO_SPEED_DEGRADE_REQ || i_sb_msg == EXIT_TO_PHYRETRAIN_REQ))
                        o_sb_msg   <= 0;
                    else
                        o_sb_msg   <= EXIT_TO_REPAIR_RESP;
                end

                EXIT_SPEED_DEGRADE: begin

                    if(i_sb_valid && i_sb_msg == EXIT_TO_PHYRETRAIN_REQ) begin
                        o_sb_msg   <= 0;
                    end
                    else
                        o_sb_msg   <= EXIT_TO_SPEED_DEGRADE_RESP;

                end

                DONE: begin
                    o_sb_msg   <= DONE_RESP;
                end
                
                TEST_END: begin
                    o_ack <= 1;
                end

            endcase
        end
    end

/////////////////////// Valid handeling /////////////////////////

always @(posedge i_clk or negedge i_rst_n) begin
    if(!i_rst_n)
      o_valid_rx <= 1'b0;
    else if((current_state_rx != next_state_rx) && (valid_1 || valid_2 || valid_3 || valid_4 || valid_5 || valid_6))
      o_valid_rx <= 1'b1;
    else if(i_falling_edge_busy) 
      o_valid_rx <= 1'b0;
end

endmodule