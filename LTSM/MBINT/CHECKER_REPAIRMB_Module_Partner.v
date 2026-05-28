module CHECKER_REPAIRMB_Module_Partner (
    input wire        CLK,
    input wire        rst_n,
    input wire        i_start_check,
    input wire        i_second_check,
    input wire [1:0]  i_Functional_Lanes,//from sb
    input wire        i_Transmitter_initiated_D2C_en,//from module

    output reg        o_done_check,
    output reg        o_go_to_repeat,
    output reg        o_go_to_train_error,
    output reg        o_continue
);

    reg [1:0] prev_Functional_Lanes;

    always @(posedge CLK or negedge rst_n)
        if (!rst_n)
            prev_Functional_Lanes <= 2'b00;
        else if (i_start_check && !i_second_check)
            prev_Functional_Lanes <= i_Functional_Lanes;

    always @(posedge CLK or negedge rst_n) begin
        if (!rst_n) begin
            o_done_check <= 0;
            o_go_to_repeat <= 0;
            o_go_to_train_error <= 0;
            o_continue <= 0;
        end 
        else if (i_start_check && !i_second_check) begin
            // First check (not during repeater flow)
            if (!i_Transmitter_initiated_D2C_en) begin
                case (i_Functional_Lanes)
                    2'b00: o_go_to_train_error <= 1;
                    2'b01, 2'b10: o_go_to_repeat <= 1;
                    2'b11: o_continue <= 1;
                endcase
                o_done_check <= 1;
            end
        end 
        else if (i_start_check && i_second_check) begin
            // Second check (after repeater completes)
            if (i_Functional_Lanes != prev_Functional_Lanes)
                o_go_to_train_error <= 1;
            else
                o_continue <= 1;
                o_done_check <= 1;  
        end 
        else begin
            o_done_check <= 0;
            o_go_to_repeat <= 0;
            o_go_to_train_error <= 0;
            o_continue <= 0;
        end
    end
endmodule