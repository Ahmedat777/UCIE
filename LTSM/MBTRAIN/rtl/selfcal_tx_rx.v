module selfcal_tx_rx (
    ////////////////////////////**main inputs**//////////////////////////////////
    input clk,    
    input rst_n,  
    input i_en, 
    ////////////////////////////**sideband inputs**//////////////////////////////
    input [3:0]  i_decoded_sideband_message,
    input        i_busy_negedge_detected,
    input        i_sideband_valid,
    ////////////////////////////**tx input**////////////////////////////////////        
    input        i_valid_tx,
    ////////////////////////////**sideband outputs**////////////////////////////
    output reg [3:0] o_sideband_message,
    ////////////////////////////**tx output**////////////////////////////////////
    output reg       o_valid_rx,
    ////////////////////////////**mbtrain output**//////////////////////////////
    output reg       o_test_ack
);

/*------------------------------------------------------------------------------
/////////////////////////////**sideband messages////////////////////////////////    
------------------------------------------------------------------------------*/
parameter Done_req  = 4'b0001;
parameter Done_resp = 4'b0010;

/*------------------------------------------------------------------------------
//////////////////////////////////**FSM States//////////////////////////////////
------------------------------------------------------------------------------*/
parameter IDLE              = 2'b00;
parameter WAIT_FOR_END_REQ  = 2'b01;
parameter SEND_END_RESPONSE = 2'b10;
parameter TEST_FINISHED     = 2'b11;

/*------------------------------------------------------------------------------
////////////////////////////**Variables Declaration////////////////////////////  
------------------------------------------------------------------------------*/
reg [1:0] cs, ns;

/*------------------------------------------------------------------------------
///////////////////**combinational always for ns calculations///////////////////
------------------------------------------------------------------------------*/
always @(*) begin
    case (cs)
        IDLE:              ns = i_en ? WAIT_FOR_END_REQ : IDLE;
        
        
        WAIT_FOR_END_REQ:  ns = (i_decoded_sideband_message == Done_req && i_sideband_valid) ? SEND_END_RESPONSE : WAIT_FOR_END_REQ;
        
        
        SEND_END_RESPONSE: ns = i_busy_negedge_detected ? TEST_FINISHED : SEND_END_RESPONSE;
        
        TEST_FINISHED:     ns = (~i_en) ? IDLE : TEST_FINISHED;
        
        default:           ns = IDLE;
    endcase
end

/*------------------------------------------------------------------------------
///////////////////////////**State Register//////////////////////////
------------------------------------------------------------------------------*/
always @(posedge clk or negedge rst_n) begin
    if (~rst_n) cs <= IDLE;
    else        cs <= ns;
end

/*------------------------------------------------------------------------------
///////////////////////////**output logic calculations//////////////////////////
------------------------------------------------------------------------------*/
always @(posedge clk or negedge rst_n) begin 
    if(~rst_n) begin
        o_sideband_message <= 4'b0000;
        o_test_ack         <= 0;
        o_valid_rx         <= 0;
    end else begin
        case (ns) 
            IDLE: begin
                o_sideband_message <= 4'b0000;
                o_test_ack         <= 0;
                o_valid_rx         <= 0;
            end
            
            WAIT_FOR_END_REQ: begin
                o_sideband_message <= 4'b0000;
                o_valid_rx         <= 0;
                o_test_ack         <= 0;
            end
            
            SEND_END_RESPONSE: begin
                o_sideband_message <= Done_resp;
                o_valid_rx         <= 1'b1;
                o_test_ack         <= 0;
            end
            
            TEST_FINISHED: begin
                o_sideband_message <= 4'b0000;
                o_valid_rx         <= 0;
                o_test_ack         <= 1'b1; 
            end
            
            default: begin
                o_sideband_message <= 4'b0000;
                o_test_ack         <= 0;
                o_valid_rx         <= 0;
            end
        endcase
/*------------------------------------------------------------------------------
/////////////////////////////**handling valid////////////////////////////////    
------------------------------------------------------------------------------*/        
        if (i_busy_negedge_detected ) begin
            o_valid_rx <= 0;
        end		
    end
end

endmodule