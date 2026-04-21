import pckg::*;
module  RDI_SM #( parameter int NBYTES = 64,        // these parameters are like those in RDI_if and can be used for array sizing and ther purposes within the state machine as needed.o
          parameter int NC     = 32) (
    input logic active_handshake_done,         
    input logic retrain_req, 
    input logic internal_error_hold,                   
    RDI_if.lphy lphy    
);

typedef enum logic [3:0] {
    STS_RDI_RESET = 4'b0000,
    STS_RDI_ACTIVE = 4'b0001,
    STS_LINK_RESET = 4'b1001 ,
    STS_RDI_RETRAIN = 4'b1011,
    STS_LINK_ERROR = 4'b1010,
    STS_RDI_DISABLED = 4'b1100
} rdi_state_sts;

typedef enum logic [3:0] {
    REQ_NOP = 4'b0000,
    REQ_RDI_ACTIVE = 4'b0001,
    REQ_LINK_RESET = 4'b1001 ,
    REQ_RDI_RETRAIN = 4'b1011,
    REQ_RDI_DISABLED = 4'b1100
} rdi_state_req;

rdi_state_sts next_state;
sb_msg_id sb_msg_id_rdi;
rdi_state_req prev_lp_state_req;
logic nop_to_active;
logic nop_to_linkreset;
logic nop_to_disable;
logic [31:0] linkerror_counter;
logic errortime_done;

    


always_ff @(posedge lphy.clk or negedge lphy.rstn) begin
    if (!lphy.rstn)
        linkerror_counter <= 32'b0;
    else if (lphy.pl_state_sts == STS_LINK_ERROR)
        linkerror_counter <= linkerror_counter + 1;
        else 
        linkerror_counter <= 32'b0;     
    end

assign errortime_done = (linkerror_counter >= 16); // note that we need linkerror_counter to be => 16 ms so take care while working wit the naumber and its unit during verification



always_ff @( posedge lphy.clk or negedge lphy.rstn ) begin 
    if (!lphy.rstn) begin
        lphy.pl_state_sts <= STS_RDI_RESET;
        lphy.pl_trdy <= 1'b0;
        lphy.pl_valid <= 1'b0;
        foreach (lphy.pl_data[i]) begin
            lphy.pl_data[i] <= 8'b0;
        end
        lphy.pl_retimer_crd <= 1'b0;
        lphy.pl_inband_pres <= 1'b0;
        lphy.pl_error <= 1'b0;
        lphy.pl_cerror <= 1'b0;
        lphy.pl_nferror <= 1'b0;
        lphy.pl_trainerror <= 1'b0;
        lphy.pl_phyinrecenter <= 1'b0;
        lphy.pl_stallreq <= 1'b0;
        lphy.pl_speedmode <= 3'b0;
        lphy.pl_max_speedmode <= 3'b0;
        lphy.pl_lnk_cfg <= 32'b0;
        lphy.pl_wake_ack <= 1'b0;
        lphy.pl_clk_req <= 1'b0;
        prev_lp_state_req <= REQ_NOP;
    end
    else begin
        lphy.pl_state_sts <= next_state;
        prev_lp_state_req <= lphy.lp_state_req;
    end
end

assign nop_to_active = (prev_lp_state_req == REQ_NOP) && (lphy.lp_state_req == REQ_RDI_ACTIVE); 
assign nop_to_linkreset = (prev_lp_state_req == REQ_NOP) && (lphy.lp_state_req == REQ_LINK_RESET); 
assign nop_to_disable = (prev_lp_state_req == REQ_NOP) && (lphy.lp_state_req == REQ_RDI_DISABLED); 

always_comb begin
    $monitor("Time: %0t, nop_to_active: %b, nop_to_linkreset: %b, nop_to_disable: %b", $time, nop_to_active, nop_to_linkreset, nop_to_disable);
    next_state = STS_RDI_RESET;
    case(lphy.pl_state_sts) 
        STS_RDI_RESET: begin
            if(nop_to_active && (active_handshake_done)) // && active hanshake done through sideband message
                next_state = STS_RDI_ACTIVE;
            else if (nop_to_linkreset && (lphy.lp_state_req == REQ_LINK_RESET || sb_msg_id_rdi == SB_RDI_REQ_LINKRESET)) // || when requested by remote Link partner through the relevant sideband message.
            begin
                next_state = STS_RDI_ACTIVE;
            if (active_handshake_done)
                next_state = STS_LINK_RESET;
            end
            else if (nop_to_disable && (lphy.lp_state_req == REQ_RDI_DISABLED || sb_msg_id_rdi == SB_RDI_REQ_DISABLE)) // || when requested by remote Link partner through the relevant sideband message.
            begin
                next_state = STS_RDI_ACTIVE;
            if (active_handshake_done)
                next_state = STS_RDI_DISABLED;
            end
            else if(lphy.lp_state_req == REQ_RDI_RETRAIN || lphy.lp_linkerror) //his transition is permitted if requested by the remote Link partner through the relevant sideband message.
                next_state = STS_RDI_RETRAIN;
        end
        STS_RDI_ACTIVE: begin
            if(lphy.lp_state_req == REQ_RDI_RETRAIN || retrain_req) // || internal request to retrain the Link 
            next_state = STS_RDI_RETRAIN;           
        end
        STS_RDI_RETRAIN: begin
            if(nop_to_active) begin 
                if(active_handshake_done)
                next_state = STS_RDI_ACTIVE;
            end
            if (lphy.lp_state_req == REQ_LINK_RESET || lphy.lp_state_req == REQ_RDI_DISABLED) begin
                if(active_handshake_done)
                next_state = STS_RDI_ACTIVE;
            end
            end
        STS_LINK_RESET: begin
            if (!lphy.rstn || lphy.lp_state_req == REQ_RDI_ACTIVE)
                next_state = STS_RDI_RESET;
            if (lphy.lp_state_req == REQ_RDI_DISABLED)
                next_state = STS_RDI_DISABLED;
            if(lphy.lp_linkerror) // || when the Link detects an error condition that requires a transition to this state.
                next_state = STS_LINK_ERROR;
            end
        STS_RDI_DISABLED: begin
            if (!lphy.rstn || lphy.lp_state_req == REQ_RDI_ACTIVE)
                next_state = STS_RDI_RESET;
            if(lphy.lp_linkerror) // || when the Link detects an error condition that requires a transition to this state.
                next_state = STS_LINK_ERROR;
        end
        STS_LINK_ERROR: begin
            if (!lphy.rstn || (lphy.lp_state_req == REQ_RDI_ACTIVE && lphy.lp_linkerror==0 && errortime_done && !internal_error_hold))
                next_state = STS_RDI_RESET;
        end

      
    endcase
end
endmodule
