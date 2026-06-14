module repair_tx ( 
////////////////////////////**main inputs**////////////////////////////
input clk,
input rst_n,
input i_en,
////////////////////////////**sideband inputs**////////////////////////////
input [3:0] i_sideband_message,
input i_sideband_valid,
input i_busy_negedge_detected,
////////////////////////////**mbtrain input**////////////////////////////
input i_first_8_lanes_are_functional , i_second_8_lanes_are_functional,
////////////////////////////**rx input**//////////////////////////// 
input i_valid_rx,
////////////////////////////**sideband outputs**////////////////////////////
output reg [3:0] o_sideband_message,
output reg[2:0] o_sideband_data_lanes_encoding,
output reg o_valid_tx,
////////////////////////////**mbtrain output**////////////////////////////
output reg o_test_ack
);

/*------------------------------------------------------------------------------
/////////////////////////////**sideband messages////////////////////////////////    
------------------------------------------------------------------------------*/
parameter INIT_REQUEST = 4'b0001;
parameter INIT_RESPONSE = 4'b0010;
parameter APPLY_DEGRADE_REQUEST = 4'b0011;
parameter APPLY_DEGRADE_RESPONSE = 4'b0100;
parameter END_REQUEST = 4'b0101;
parameter END_RESPONSE = 4'b0110;


/*------------------------------------------------------------------------------
////////////////////////////**FSM States////////////////////////////   
------------------------------------------------------------------------------*/
parameter IDLE = 3'b000;
parameter REPAIR_INIT_REQUEST = 3'b001;
parameter REPAIR_APPLY_DEGRADE_REQUEST = 3'b010;
parameter REPAIR_END_REQUEST = 3'b011;
parameter TEST_FINISH=3'b100;
/*------------------------------------------------------------------------------
////////////////////////////**Variables Declaration////////////////////////////  
------------------------------------------------------------------------------*/
reg [2:0] cs, ns;
/*------------------------------------------------------------------------------
///////////////////**combinational always for ns calculations///////////////////
------------------------------------------------------------------------------*/
always @(*) begin
    case (cs)
        IDLE: 
            ns = i_en ? REPAIR_INIT_REQUEST : IDLE;

        REPAIR_INIT_REQUEST: 
            ns = (!i_en) ? IDLE : 
                 (i_sideband_message == INIT_RESPONSE && i_sideband_valid) ? REPAIR_APPLY_DEGRADE_REQUEST : REPAIR_INIT_REQUEST;

        REPAIR_APPLY_DEGRADE_REQUEST: 
            ns = (!i_en) ? IDLE : 
                 (i_sideband_message == APPLY_DEGRADE_RESPONSE && i_sideband_valid) ? REPAIR_END_REQUEST : REPAIR_APPLY_DEGRADE_REQUEST;

        REPAIR_END_REQUEST: 
            ns = (!i_en) ? IDLE : 
                 (i_sideband_message == END_RESPONSE && i_sideband_valid) ? TEST_FINISH : REPAIR_END_REQUEST;

        TEST_FINISH: 
            ns = (!i_en) ? IDLE : TEST_FINISH;

        default: 
            ns = IDLE;
    endcase
end
/*------------------------------------------------------------------------------
-- State Register (Sequential)
------------------------------------------------------------------------------*/
always @(posedge clk or negedge rst_n) begin
    if (~rst_n) cs <= IDLE;
    else        cs <= ns;
end

/*------------------------------------------------------------------------------
///////////////////////////**output logic calculations//////////////////////////
------------------------------------------------------------------------------*/


/*always @(posedge clk or negedge rst_n) begin 
if (~rst_n) begin
//cs <= IDLE;
o_valid_tx <= 1'b0;
o_sideband_message <= 0;
o_test_ack <= 0;
o_sideband_data_lanes_encoding<=0;
end
else begin
//cs <= ns;
case (cs)
IDLE:begin
o_sideband_message<=0;
o_valid_tx <= 1'b0;
o_test_ack<=0;
o_sideband_data_lanes_encoding<=0;
if(ns==REPAIR_INIT_REQUEST) begin
o_sideband_message<=INIT_REQUEST;
o_test_ack <= 0;
end
end
REPAIR_INIT_REQUEST:begin
if(ns==REPAIR_APPLY_DEGRADE_REQUEST) begin
o_sideband_message<=APPLY_DEGRADE_REQUEST;
o_valid_tx <= 1'b1;
if(i_first_8_lanes_are_functional && i_second_8_lanes_are_functional)
o_sideband_data_lanes_encoding<=3'b011;
else if(i_first_8_lanes_are_functional) 
o_sideband_data_lanes_encoding<=3'b001;
else if(i_second_8_lanes_are_functional)
o_sideband_data_lanes_encoding<=3'b010;
end
end
REPAIR_APPLY_DEGRADE_REQUEST:begin
if(ns==REPAIR_END_REQUEST) begin
o_sideband_message<=END_REQUEST;
o_valid_tx <= 1'b1;
end
end
REPAIR_END_REQUEST:begin
if(ns==TEST_FINISH) begin
o_sideband_message<=4'b0000;
o_valid_tx <= 1'b0;
o_test_ack<=1;
end
end
TEST_FINISH:begin
if(ns==IDLE) begin
o_test_ack<=0;
end
end
default: begin
o_sideband_message <= 0;
o_test_ack <= 0;
o_valid_tx <= 1'b0;
o_sideband_data_lanes_encoding<=0;
end
endcase
if (i_busy_negedge_detected && !i_valid_rx) begin
o_valid_tx <= 1'b0;
end
end
end
/*------------------------------------------------------------------------------
/////////////////////////////////**calc for valid///////////////////////////////
------------------------------------------------------------------------------*/
/*always @(posedge clk or negedge rst_n) begin
if (~rst_n) begin
    o_valid_tx <= 1'b0;
end 
else begin
if (cs != ns) begin
if (ns == REPAIR_INIT_REQUEST || ns == REPAIR_APPLY_DEGRADE_REQUEST || ns == REPAIR_END_REQUEST) begin
o_valid_tx <= 1'b1;
end
else begin
o_valid_tx <= 1'b0;
end
end
else if (i_busy_negedge_detected && !i_valid_rx) begin
o_valid_tx <= 1'b0;
end
end
end*/
/*always @(posedge clk or negedge rst_n) begin 
    if (~rst_n) begin
        o_valid_tx <= 1'b0;
        o_sideband_message <= 4'b0000;
        o_test_ack <= 1'b0;
        o_sideband_data_lanes_encoding <= 3'b000;
    end
    else begin
        case (cs)
            IDLE: begin
                o_test_ack <= 1'b0;
                o_sideband_data_lanes_encoding <= 3'b000;
                if (ns == REPAIR_INIT_REQUEST) begin
                    o_sideband_message <= INIT_REQUEST;
                    o_valid_tx <= 1'b1; 
                end else begin
                    o_valid_tx <= 1'b0;
                    o_sideband_message <= 4'b0000;
                end
            end

            REPAIR_INIT_REQUEST: begin
                if (ns == REPAIR_APPLY_DEGRADE_REQUEST) begin
                    o_sideband_message <= APPLY_DEGRADE_REQUEST;
                    o_valid_tx <= 1'b1;
                    // تحديد تشفير المسارات (Lanes Encoding)
                    if (i_first_8_lanes_are_functional && i_second_8_lanes_are_functional)
                        o_sideband_data_lanes_encoding <= 3'b011;
                    else if (i_first_8_lanes_are_functional) 
                        o_sideband_data_lanes_encoding <= 3'b001;
                    else if (i_second_8_lanes_are_functional)
                        o_sideband_data_lanes_encoding <= 3'b010;
                end
            end

            REPAIR_APPLY_DEGRADE_REQUEST: begin
                if (ns == REPAIR_END_REQUEST) begin
                    o_sideband_message <= END_REQUEST;
                    o_valid_tx <= 1'b1;
                end
            end

            REPAIR_END_REQUEST: begin
                if (ns == TEST_FINISH) begin
                    o_sideband_message <= 4'b0000;
                    o_valid_tx <= 1'b0;
                    o_test_ack <= 1'b1; 
                end
            end

            TEST_FINISH: begin
                if (ns == IDLE) begin
                    o_test_ack <= 1'b0;
                end
            end

            default: begin
                o_valid_tx <= 1'b0;
                o_sideband_message <= 4'b0000;
                o_test_ack <= 1'b0;
                o_sideband_data_lanes_encoding <= 3'b000;
            end
        endcase

        // منطق الحماية (Override) في حال اكتشاف Busy
        if (i_busy_negedge_detected && !i_valid_rx) begin
            o_valid_tx <= 1'b0;
        end
    end
end
endmodule */
/*------------------------------------------------------------------------------
-- Output Logic (Sequential - Registered Outputs)
------------------------------------------------------------------------------*/
always @(posedge clk or negedge rst_n) begin 
    if (~rst_n) begin
        o_valid_tx <= 1'b0;
        o_sideband_message <= 4'b0000;
        o_test_ack <= 1'b0;
        o_sideband_data_lanes_encoding <= 3'b000;
    end
    else begin
        case (cs)
            IDLE: begin
                o_valid_tx     <= 1'b0;
                o_sideband_message <= 4'b0000;
                o_test_ack     <= 1'b0;
                o_sideband_data_lanes_encoding <= 3'b000;
            end

            REPAIR_INIT_REQUEST: begin
                o_valid_tx     <= 1'b1;
                o_sideband_message <= INIT_REQUEST;
                o_test_ack     <= 1'b0;
                o_sideband_data_lanes_encoding <= 3'b000;
            end

            REPAIR_APPLY_DEGRADE_REQUEST: begin
                o_valid_tx     <= 1'b1;
                o_sideband_message <= APPLY_DEGRADE_REQUEST;
                
                
                if (i_first_8_lanes_are_functional && i_second_8_lanes_are_functional)
                    o_sideband_data_lanes_encoding <= 3'b011;
                else if (i_first_8_lanes_are_functional) 
                    o_sideband_data_lanes_encoding <= 3'b001;
                else if (i_second_8_lanes_are_functional)
                    o_sideband_data_lanes_encoding <= 3'b010;
                else
                    o_sideband_data_lanes_encoding <= 3'b000;
            end

            REPAIR_END_REQUEST: begin
                o_valid_tx     <= 1'b1;
                o_sideband_message <= END_REQUEST;
                o_test_ack     <= 1'b0;
            end

            TEST_FINISH: begin
                o_valid_tx     <= 1'b0;
                o_sideband_message <= 4'b0000;
                o_test_ack     <= 1'b1; 
            end

            default: begin
                o_valid_tx     <= 1'b0;
                o_sideband_message <= 4'b0000;
                o_test_ack     <= 1'b0;
                o_sideband_data_lanes_encoding <= 3'b000;
            end
        endcase
/*------------------------------------------------------------------------------
/////////////////////////////**handling valid////////////////////////////////    
------------------------------------------------------------------------------*/
        if (i_busy_negedge_detected && !i_valid_rx) begin
            o_valid_tx <= 1'b0;
        end
    end
end
endmodule