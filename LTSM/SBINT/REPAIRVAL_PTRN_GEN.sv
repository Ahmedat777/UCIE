module REPAIRVAL_PTRN_GEN (
    input  logic i_clk,
    input  logic i_rst_n,
    input  logic i_en,
    output logic o_lane,
    output logic o_done
);

    localparam int HIGH_CYCLES  = 4;   // High phase
    localparam int LOW_CYCLES   = 4;    // flat-low phase
    localparam int ITER_CYCLES  = HIGH_CYCLES + LOW_CYCLES; // 24
    localparam int TOTAL_ITERS  = 128;

    localparam int CYCLE_W = $clog2(ITER_CYCLES);  // 5 bits
    localparam int ITER_W  = $clog2(TOTAL_ITERS);  // 7 bits

    logic [CYCLE_W-1:0] cycle_cnt;
    logic [ITER_W-1:0]  iter_cnt;
    logic               done;

    always_ff @(posedge i_clk or negedge i_rst_n) begin
        if (!i_rst_n) begin
            cycle_cnt <= '0;
            iter_cnt  <= '0;
            done      <= 1'b0;
        end else if (!i_en) begin
            cycle_cnt <= '0;
            iter_cnt  <= '0;
            done      <= 1'b0;
        end else begin
            done <= 1'b0;
            if (iter_cnt == TOTAL_ITERS - 1 && cycle_cnt == ITER_CYCLES - 1) 
            begin
                done <= 1'b1;   // burst complete — freeze until i_en drops
            end 
            else
             begin
                if (cycle_cnt == ITER_CYCLES - 1) 
                begin
                    cycle_cnt <= '0;
                    iter_cnt  <= iter_cnt + 1'b1;
                end 
                else 
                begin
                    cycle_cnt <= cycle_cnt + 1'b1;
                end
            end
        end
    end


    assign o_lane = i_en
                 && (iter_cnt < TOTAL_ITERS)
                 && (cycle_cnt < HIGH_CYCLES)
                 

    assign o_done = done;

endmodule