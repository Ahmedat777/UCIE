// =============================================================================
// Module  : ms_timer
// Purpose : Generic millisecond timer. 
//           Counts clock cycles equivalent to PERIOD_MS milliseconds
//           based on the provided CLK_FREQ_HZ parameter.
//
// Parameters:
//   CLK_FREQ_HZ  - Clock frequency in Hz  (e.g. 100_000_000 for 100 MHz)
//   PERIOD_MS    - Desired period in ms    (e.g. 4 for 4ms)
//
// How COUNT_MAX is derived:
//   COUNT_MAX = (CLK_FREQ_HZ / 1000) * PERIOD_MS  - 1
// =============================================================================

module ms_timer #(
    parameter CLK_FREQ_HZ = 100_000_000, // Default: 100 MHz
    parameter PERIOD_MS   = 4            // Default: 4 ms
)(
    input  wire  i_clk,
    input  wire  i_rst_n,
    input  wire  i_enable,       // start/enable counting (was start_reset_counter)
    output reg   o_at_target     // stays HIGH at COUNT_MAX-1 so FSM can act one cycle before reset
);

    // -------------------------------------------------------------------
    // Derived parameter: number of clock cycles for PERIOD_MS milliseconds
    // -------------------------------------------------------------------
    localparam integer COUNT_MAX = (CLK_FREQ_HZ / 1000) * PERIOD_MS; 

    // Width is calculated to fit COUNT_MAX
    localparam integer COUNTER_WIDTH = $clog2(COUNT_MAX + 1);

    reg [COUNTER_WIDTH-1 : 0] counter;

    // -------------------------------------------------------------------
    // Counter logic
    // -------------------------------------------------------------------
    always @(posedge i_clk or negedge i_rst_n) begin
        if (!i_rst_n) begin
            counter    <= 0;
            o_at_target <= 0;
        end else begin
            o_at_target <= 0;
            if (i_enable) 
            begin
                if (counter == COUNT_MAX - 1) 
                begin
                    o_at_target <= 1;           
                end
                else 
                begin
                    counter <= counter + 1;
                end
            end 
            else 
            begin
                counter <= 0; // reset counter when not enabled
            end
        end
    end

endmodule