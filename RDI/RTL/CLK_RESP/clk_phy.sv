module clk_phy#( parameter NC=32)(
    input wire clk,
    RDI_if.lphy inst,
    input logic ltsm_training_done,       // High when Stage 2 training is complete
    input logic internal_sb_pending,     // PHY needs to send pl_cfg
    input logic internal_link_error_req, // PHY entering LinkError autonomously
    input logic [4*NC-1:0] internal_sb_data   // Data to drive on pl_cfg

);

logic clk_req_trigger;
logic gating_prohibit;
logic [4*NC-1:0] temp_internal_sb_data;
assign clk_req_trigger = internal_sb_pending || internal_link_error_req || ltsm_training_done ;

typedef enum logic [3:0] {
    RDI_RESET = 4'b0000,
    RDI_ACTIVE = 4'b0001,
    LINK_RESET = 4'b1001 ,
    RDI_RETRAIN = 4'b1011,
    LINK_ERROR = 4'b1010
    } rdi_state_e;


 typedef enum logic [2:0] {
        IDLE,
        WAIT_FOR_ACK,
        ACTION_WINDOW_zero,
        SUB_ACTION_WINDOW_one,
        SUB_ACTION_WINDOW_two,
        SUB_ACTION_WINDOW_three,
        RELEASE_REQ
    } clk_fsm_e;

    clk_fsm_e current_state, next_state;




assign gating_prohibit = (inst.pl_state_sts == RDI_ACTIVE || inst.pl_state_sts == LINK_ERROR || inst.pl_state_sts == RDI_RETRAIN);

always_ff @(posedge clk or negedge inst.rstn) begin
    if (!inst.rstn) begin
        current_state <= IDLE;
        temp_internal_sb_data <= {4*NC{1'b0}};
    end
    else begin
        current_state <= next_state;
        if (current_state == IDLE && clk_req_trigger)
            temp_internal_sb_data <= internal_sb_data;
    end
end
    always_comb begin
        case (current_state)
            IDLE: begin
                if (clk_req_trigger )
                    next_state = WAIT_FOR_ACK;
                else
                    next_state = IDLE;
            end
            WAIT_FOR_ACK: begin
               if(inst.lp_clk_ack)
                    next_state = ACTION_WINDOW_zero;
                else
                    next_state = WAIT_FOR_ACK;
            end
            ACTION_WINDOW_zero: begin               
                    next_state = SUB_ACTION_WINDOW_one;
            end
            SUB_ACTION_WINDOW_one:begin
                next_state = SUB_ACTION_WINDOW_two;
            end
            SUB_ACTION_WINDOW_two: begin
                next_state = SUB_ACTION_WINDOW_three;
            end
            SUB_ACTION_WINDOW_three: begin
                    next_state = RELEASE_REQ;
            end
            RELEASE_REQ: begin
                        if (!inst.lp_clk_ack) 
                        next_state = IDLE;
                        else 
                        next_state = RELEASE_REQ;
                    end

            default: next_state = IDLE;
        endcase
    end


always_comb begin
    inst.pl_clk_req = 1'b0;
    inst.pl_cfg = 32'b0;
    inst.pl_cfg_vld = 1'b0;   
    case (current_state)
            
        IDLE: begin
            if (gating_prohibit)
                inst.pl_clk_req = 1'b1; // Force clk_req high if gating is prohibited, even in IDLE
            else
            inst.pl_clk_req = 1'b0;

            inst.pl_cfg = 32'b0;
            inst.pl_cfg_vld = 1'b0;
        end
        WAIT_FOR_ACK: begin
            inst.pl_clk_req = 1'b1;
            inst.pl_cfg = 32'b0;
            inst.pl_cfg_vld = 1'b0;
        end
        ACTION_WINDOW_zero: begin
                inst.pl_clk_req = 1'b1;
                inst.pl_cfg = temp_internal_sb_data[NC-1:0];
                inst.pl_cfg_vld = 1'b1;
        end
        SUB_ACTION_WINDOW_one: begin
            inst.pl_clk_req = 1'b1; // Keep clk_req high to maintain the Action Window until we can release it
            inst.pl_cfg = temp_internal_sb_data[2*NC-1:NC];
            inst.pl_cfg_vld = 1'b1;
        end
        SUB_ACTION_WINDOW_two: begin
            inst.pl_clk_req = 1'b1; // Keep clk_req high to maintain the Action Window until we can release it
            inst.pl_cfg = temp_internal_sb_data[3*NC-1:2*NC];
            inst.pl_cfg_vld = 1'b1;
        end
        SUB_ACTION_WINDOW_three: begin
            inst.pl_clk_req = 1'b1; // Keep clk_req high to maintain the Action Window until we can release it
            inst.pl_cfg = temp_internal_sb_data[4*NC-1:3*NC];
            inst.pl_cfg_vld = 1'b1;
        end
        RELEASE_REQ: begin
                if (gating_prohibit)
                inst.pl_clk_req = 1'b1; // Force clk_req high if gating is prohibited, even in IDLE
            else begin
                inst.pl_clk_req = 1'b0;
                inst.pl_cfg = 32'b0;
                inst.pl_cfg_vld = 1'b0;
            end
        end
        default: begin 
            if (gating_prohibit)
                inst.pl_clk_req = 1'b1; // Force clk_req high if gating is prohibited, even in IDLE
            else
            inst.pl_clk_req = 1'b0;
            inst.pl_cfg = 32'b0;
            inst.pl_cfg_vld = 1'b0;
        end 
    endcase
end
always_ff @(posedge clk or negedge inst.rstn) begin
    if (!inst.rstn) begin
        inst.pl_inband_pres <= 1'b0;
    end else begin
        // Latch high once training is done in the Action Window
        if (current_state == ACTION_WINDOW_zero && ltsm_training_done)
            inst.pl_inband_pres <= 1'b1;
        // Only de-assert if the link status moves to Reset or LinkError [Table 10-1]
        else if (inst.pl_state_sts == 4'b0000 || inst.pl_state_sts == 4'b1010)
            inst.pl_inband_pres <= 1'b0;
    end
end


endmodule