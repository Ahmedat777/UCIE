module negedge_detector (
    input  wire  clk,
    input  wire  rst_n,
    input  wire  i_busy,
    output wire  o_falling_edge_busy
);
    reg busy_delayed;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            busy_delayed <= 1'b0;
        else
            busy_delayed <= i_busy;
    end

    assign o_falling_edge_busy = busy_delayed & ~i_busy;

endmodule