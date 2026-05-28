module PHY_RDI_SM#()(
    input logic RDI_clk,
    output logic handshake_done,
     
    RDI_if.lphy SM
);

wire stall_trigger;
typedef enum logic [3:0] {
    RDI_RESET = 4'b0000,
    RDI_ACTIVE = 4'b0001,
    LINK_RESET = 4'b1001 ,
    RDI_RETRAIN = 4'b1011,
    LINK_ERROR = 4'b1010,
    RDI_DISABLED = 4'b1100
    } rdi_state_e;

typedef enum {
    idle,   
    stallreq,
    ackrecieved,
    reqdeassert
} handshake_sts;

handshake_sts current_state, next_state;

assign stall_trigger = (SM.pl_state_sts == RDI_ACTIVE) &&
                      (SM.lp_state_req == RDI_RETRAIN || SM.lp_state_req == LINK_RESET || SM.lp_state_req == RDI_DISABLED);

always_ff @(posedge RDI_clk or negedge SM.rstn) begin
    if(!SM.rstn) begin
        current_state <= idle;
         SM.pl_state_sts <= RDI_RESET;
    end
    else begin
        current_state <= next_state;
        if (current_state == ackrecieved) begin
                SM.pl_state_sts <= SM.lp_state_req;
            end
    end
end

always_comb begin
    next_state = current_state;
    case(current_state)
        idle:
            if(stall_trigger && !SM.lp_stallack)
                next_state = stallreq;
        stallreq:
            if(SM.lp_stallack)
                next_state = ackrecieved;
        ackrecieved:
                next_state = reqdeassert;
        reqdeassert:
            if(!SM.lp_stallack)
                next_state = idle;
    endcase
end

always_comb begin
    SM.pl_stallreq = 1'b0;
    handshake_done = 1'b0;
    SM.pl_trdy = 1'b1;
    case(current_state)
        idle:begin
            SM.pl_stallreq = 1'b0;
            SM.pl_trdy = 1'b1;
            handshake_done = 1'b1;
        end
        stallreq: begin
            SM.pl_stallreq = 1'b1;
            SM.pl_trdy = 1'b1;
        end
        ackrecieved: begin
            SM.pl_stallreq = 1'b1;
            SM.pl_trdy = 1'b0;        
        end   
        reqdeassert: begin
            SM.pl_trdy = 1'b0;
            SM.pl_stallreq = 1'b0;
            
        end
        default: begin
            SM.pl_trdy = 1'b0;
            SM.pl_stallreq = 1'b0;
            handshake_done = 1'b0;
        end
    endcase
end

endmodule