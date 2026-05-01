module SB_CLOCK_CONTROLLER (
    input           i_pll_clk,        // 800 MHz PLL clock
    input           i_rst_n,          // active-low asynchronous reset
    input           i_enable,         // request to transmit one 64-bit packet

    output reg      o_dead_time_done, // HIGH for the last cycle of 32-UI dead-time
                                      //   → SB_TX_FSM_Modelling: ready for next packet
    output reg      o_ser_done,       // single-cycle pulse when 64 bits shifted out
    output          TXCKSB            // gated sideband clock output
);

    // -----------------------------------------------------------------------
    // Internal registers
    // -----------------------------------------------------------------------
    reg [6:0] counter;       // counts 0..95  (7 bits covers 0–127)
    reg       clock_enable;  // latched "a packet is in progress"
    reg       ser_en_lat;    // ICG latch output (avoids glitch on TXCKSB)

    // -----------------------------------------------------------------------
    // ser_en: active during the 64-UI transmission window (cycles 0..63)
    // Clock is enabled when a packet starts or is already in progress AND
    // we are still within the first 64 UI.
    // -----------------------------------------------------------------------
    wire ser_en = (i_enable || clock_enable) && (counter < 7'd64);

    // -----------------------------------------------------------------------
    // ICG latch – transparent when i_pll_clk is LOW
    // Prevents glitches on TXCKSB by ensuring ser_en_lat only changes during
    // the low phase of i_pll_clk (standard ICG practice).
    // -----------------------------------------------------------------------
    always_latch begin
        if (!i_pll_clk)
            ser_en_lat = ser_en;
    end

    // -----------------------------------------------------------------------
    // Gated clock output (spec §4.5.3.2: TXCKSB gated during dead-time)
    // -----------------------------------------------------------------------
    assign TXCKSB = i_pll_clk & ser_en_lat;

    // -----------------------------------------------------------------------
    // Main counter and control
    // -----------------------------------------------------------------------
    always_ff @(posedge i_pll_clk or negedge i_rst_n) begin
        if (!i_rst_n) begin
            counter         <= 7'd0;
            clock_enable    <= 1'b0;
            o_dead_time_done<= 1'b0;
        end else begin
            if (i_enable || clock_enable) begin
                clock_enable <= 1'b1;

                if (counter == 7'd95) begin
                    // -----------------------------------------------------------
                    // End of 96-UI frame (64 UI active + 32 UI dead-time).
                    // Pulse o_dead_time_done for exactly one PLL cycle here
                    // so SB_TX_FSM_Modelling can immediately request the next
                    // 64-bit packet (header or data flit).
                    // -----------------------------------------------------------
                    counter         <= 7'd0;
                    o_dead_time_done<= 1'b1;   // one-cycle pulse on frame end
                    clock_enable    <= 1'b0;
                end else begin
                    counter         <= counter + 7'd1;
                    o_dead_time_done<= 1'b0;
                end
            end else begin
                // Idle: all outputs cleared
                o_dead_time_done <= 1'b0;
            end
        end
    end

    // -----------------------------------------------------------------------
    // o_ser_done: single-cycle pulse when 64 bits have been shifted out.
    // Fires on the cycle that completes bit[63] (counter 63 → 64 transition).
    // Used by the serialiser/framer to know the full 64-bit word is on the wire.
    // -----------------------------------------------------------------------
    always_ff @(posedge i_pll_clk or negedge i_rst_n) begin
        if (!i_rst_n)
            o_ser_done <= 1'b0;
        else
            o_ser_done <= (counter == 7'd63) && clock_enable;
    end

endmodule : SB_CLOCK_CONTROLLER