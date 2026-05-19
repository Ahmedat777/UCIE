module SB_TX_FSM_Modelling (
    input  logic i_clk,                  // 800 MHz SB clock
    input  logic i_rst_n,                // asynchronous active-low reset

    input  logic i_ser_done,             // pulse: 64 bits shifted out
    input  logic i_empty,                // TX FIFO empty
    input  logic i_dead_time_done,       // SB_CLOCK_CONTROLLER: 32-UI dead-time complete
    input  logic i_read_enable_sampled,  // TX FIFO ack: pop was accepted

    output logic o_read_enable,          // pop next word from TX FIFO
    output logic o_clk_en                
);

    // ─────────────────────────────────────────────────────────────────────────
    // State encoding
    // ─────────────────────────────────────────────────────────────────────────
    localparam logic [1:0]
        IDLE         = 2'b00,
        SENDING_PACK = 2'b01,
        SLEEPING     = 2'b10;

    logic [1:0] cs, ns;

    // ─────────────────────────────────────────────────────────────────────────
    // State register
    // ─────────────────────────────────────────────────────────────────────────
    always_ff @(posedge i_clk or negedge i_rst_n) begin
        if (!i_rst_n) cs <= IDLE;
        else          cs <= ns;
    end

    // ─────────────────────────────────────────────────────────────────────────
    // Next-state logic
    // ─────────────────────────────────────────────────────────────────────────
    always_comb begin
        ns = cs;
        case (cs)
            IDLE: begin
                // Start as soon as there is a word in the FIFO
                if (!i_empty)
                    ns = SENDING_PACK;
            end

            SENDING_PACK: begin
                // Wait for the 64-bit serialisation to finish
                if (i_ser_done)
                    ns = SLEEPING;
            end

            SLEEPING: begin
                // SB_CLOCK_CONTROLLER signals end of 32-UI dead-time
                if (i_dead_time_done) begin
                    if (!i_empty)
                        ns = SENDING_PACK;   // next word ready (back-to-back)
                    else
                        ns = IDLE;
                end
            end

            default: ns = IDLE;
        endcase
    end

    always_ff @(posedge i_clk or negedge i_rst_n) begin
        if (!i_rst_n)
            o_clk_en <= 1'b0;
        else if (cs == SENDING_PACK && i_read_enable_sampled)
            o_clk_en <= 1'b1;
        else
            o_clk_en <= 1'b0;
    end

    always_ff @(posedge i_clk or negedge i_rst_n) begin
        if (!i_rst_n) begin
            o_read_enable <= 1'b0;
        end else begin
            if ((cs == IDLE || cs == SLEEPING) && ns == SENDING_PACK)
                o_read_enable <= 1'b1;
            else if (i_read_enable_sampled)
                o_read_enable <= 1'b0;
        end
    end

endmodule : SB_TX_FSM_Modelling