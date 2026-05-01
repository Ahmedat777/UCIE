module pattern_comparator #(
    parameter LANE_WIDTH = 32,
    parameter NUM_LANE   = 16
)(
    input  wire                      i_clk,
    input  wire                      i_rst_n,
    input  wire                      i_en_comparator,
    input  wire                      i_enable_buffer,     // from deserializer
    input  wire                      i_comp_type,         // 1 = per-lane , 0 = aggregate
    input  wire [1:0]                i_comparator_mode,   // FSM state select

    // Expected(Generated) Pattern
    input  wire [LANE_WIDTH-1:0]     i_gen_data [0:NUM_LANE-1],
    
    // Received Pattern
    input  wire [LANE_WIDTH-1:0]     i_data     [0:NUM_LANE-1],
    
    // Thresholds
    input  wire [11:0]               i_per_lane_Threshold,
    input  wire [15:0]               i_aggregate_Threshold,
    
    // Outputs
    output reg  [NUM_LANE-1:0]       o_per_lane_error,
    output reg                        o_module_error,
    output reg  [15:0]               o_aggregate_counter,
    output reg                        o_aggregate_exceeded
);

    //------------------------------------------------------------
    // FSM States
    //------------------------------------------------------------
    localparam IDLE         = 2'b00;
    localparam CLEAR_LFSR   = 2'b01;
    localparam PATTERN_LFSR = 2'b10;
    localparam PER_LANE_ID  = 2'b11;

    reg [1:0] state;

    always @(posedge i_clk or negedge i_rst_n) begin
        if (!i_rst_n)
            state <= IDLE;
        else
            state <= i_comparator_mode;
    end

    //------------------------------------------------------------
    // Internal registers
    //------------------------------------------------------------
    reg [11:0] per_lane_mismatch_acc [0:NUM_LANE-1];
    reg [11:0] lane_bit_error_count_temp [0:NUM_LANE-1];
    reg [NUM_LANE-1:0] lane_error_bits_temp;
    reg [NUM_LANE-1:0] lane_error_bits;  // sticky per-lane inside PATTERN phase

    integer lane_i, bit_i;

    //------------------------------------------------------------
    // Combinational per-UI mismatch detection
    //------------------------------------------------------------
    always @(*) begin
        lane_error_bits_temp = {NUM_LANE{1'b0}};
        for (lane_i = 0; lane_i < NUM_LANE; lane_i = lane_i + 1)
            lane_bit_error_count_temp[lane_i] = 12'd0;

        if (i_en_comparator && i_enable_buffer) begin
            for (lane_i = 0; lane_i < NUM_LANE; lane_i = lane_i + 1) begin
                for (bit_i = 0; bit_i < LANE_WIDTH; bit_i = bit_i + 1) begin
                    if (i_data[lane_i][bit_i] ^ i_gen_data[lane_i][bit_i]) begin
                        lane_error_bits_temp[lane_i] = 1'b1;
                        lane_bit_error_count_temp[lane_i] = lane_bit_error_count_temp[lane_i] + 12'd1;
                    end
                end
            end
        end
    end

    //------------------------------------------------------------
    // Sequential logic
    //------------------------------------------------------------
    always @(posedge i_clk or negedge i_rst_n) begin
        if (!i_rst_n) begin
            // Reset all
            o_aggregate_counter  <= 16'd0;
            o_per_lane_error     <= {NUM_LANE{1'b0}};
            o_module_error       <= 1'b0;
            o_aggregate_exceeded <= 1'b0;
            lane_error_bits      <= {NUM_LANE{1'b0}};
            for (lane_i = 0; lane_i < NUM_LANE; lane_i = lane_i + 1)
                per_lane_mismatch_acc[lane_i] <= 12'd0;
        end
        else begin
            case (state)

                //--------------------------------------------
                IDLE:
                //--------------------------------------------
                begin
                    o_module_error       <= 1'b0;
                    o_aggregate_exceeded <= 1'b0;
                    lane_error_bits      <= {NUM_LANE{1'b0}};
                    o_per_lane_error     <= {NUM_LANE{1'b0}};
                end

                //--------------------------------------------
                CLEAR_LFSR:
                //--------------------------------------------
                begin
                    o_aggregate_counter  <= 16'd0;
                    o_per_lane_error     <= {NUM_LANE{1'b0}};
                    o_module_error       <= 1'b0;
                    o_aggregate_exceeded <= 1'b0;
                    lane_error_bits      <= {NUM_LANE{1'b0}};
                    for (lane_i = 0; lane_i < NUM_LANE; lane_i = lane_i + 1)
                        per_lane_mismatch_acc[lane_i] <= 12'd0;
                end

                //--------------------------------------------
                PATTERN_LFSR:
                //--------------------------------------------
                begin
                    if (i_en_comparator && i_enable_buffer) begin
                        // Update per-lane accumulative mismatch
                        for (lane_i = 0; lane_i < NUM_LANE; lane_i = lane_i + 1)
                            per_lane_mismatch_acc[lane_i] <= per_lane_mismatch_acc[lane_i] + lane_bit_error_count_temp[lane_i];

                        // Sticky lane error bits inside PATTERN phase
                        lane_error_bits <= lane_error_bits | lane_error_bits_temp;

                        // Aggregate counter increment
                        if (|lane_error_bits_temp)
                            o_aggregate_counter <= o_aggregate_counter + 16'd1;
                    end
                end

                //--------------------------------------------
                PER_LANE_ID:
                //--------------------------------------------
                begin
                    if (i_comp_type) begin
                        // Sticky per-lane errors
                        for (lane_i = 0; lane_i < NUM_LANE; lane_i = lane_i + 1) begin
                            if (per_lane_mismatch_acc[lane_i] > i_per_lane_Threshold)
                                o_per_lane_error[lane_i] <= 1'b1;
                        end
                    end
                    else begin
                        o_per_lane_error     <= {NUM_LANE{1'b0}};
                        o_module_error       <= |lane_error_bits;
                        o_aggregate_exceeded <= (o_aggregate_counter > i_aggregate_Threshold);
                    end
                end

            endcase
        end
    end

endmodule