module Functional_Lane_Setup (
    input wire        CLK,
    input wire        rst_n,
    input wire        start_setup,
    input wire [15:0] i_lanes_results_tx,

    output reg [1:0]  o_Functional_Lanes,
    output reg        done_setup
);

    always @(posedge CLK or negedge rst_n) begin
        if (!rst_n) begin
            o_Functional_Lanes <= 2'b11;
            done_setup <= 0;
        end else begin
            done_setup <= 0;
            if (start_setup) begin
                if (&i_lanes_results_tx)
                    o_Functional_Lanes <= 2'b11;
                else if (&i_lanes_results_tx[15:8])
                    o_Functional_Lanes <= 2'b10;
                else if (&i_lanes_results_tx[7:0])
                    o_Functional_Lanes <= 2'b01;
                else
                    o_Functional_Lanes <= 2'b00;
                done_setup <= 1;
            end
        end
    end
endmodule
