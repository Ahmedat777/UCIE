// =============================================================================
// SB_TX_FSM_Modelling  –  UCIe Sideband TX Serialiser Controller
//
// Spec reference: UCIe Specification Rev 3.0 Ver 1.0 (August 2025)
//   §4.1.5  "A 64-bit serial packet is defined on the I/O interface to the
//            remote die… Two sideband serial packets are separated by a minimum
//            of 32 bits low."
//           "A sideband message with data would be transferred as a 64-bit
//            header followed by 32 bits of low followed by 64-bit data
//            followed by 32 bits of low."
//
// This FSM controls SB_CLOCK_CONTROLLER (one 64-bit packet at a time) and the
// TX FIFO read pointer. It operates at the granularity of one 64-bit serial
// packet per trip through SENDING_PACK → SLEEPING.
//
// Packet sequencing per spec §4.1.5:
//   Header-only message:
//     IDLE → SENDING_PACK (header, 64 UI) → SLEEPING (32 UI dead-time) → IDLE
//
//   Message with data flit (i_has_data == 1):
//     IDLE → SENDING_PACK (header, 64 UI) → SLEEPING (32 UI dead-time)
//          → SENDING_PACK (data,   64 UI) → SLEEPING (32 UI dead-time) → IDLE
//
//   SB_CLOCK_CONTROLLER handles exactly ONE 64-bit packet per i_enable
//   assertion (64 UI active + 32 UI dead-time). This FSM asserts i_enable
//   (via o_clk_en) twice for data messages: once for the header flit and
//   once for the data flit. The SB_CLOCK_CONTROLLER output i_dead_time_done
//   (formerly o_pack_finished) signals the end of each 32-UI dead-time,
//   enabling this FSM to proceed to the next phase.
//
// States:
//   IDLE         – wait for TX FIFO to become non-empty
//   SENDING_PACK – clock enabled; serialiser active (64 UI)
//   SLEEPING     – clock gated; SB_CLOCK_CONTROLLER enforcing dead-time (≥32 UI)
//
// r_data_phase tracks which flit we are currently sending:
//   0 → header phase
//   1 → data flit phase  (only valid when i_has_data == 1)
// =============================================================================

module SB_TX_FSM_Modelling (
    input       i_clk,                  // 800 MHz SB clock
    input       i_rst_n,                // asynchronous active-low reset
    input       i_ser_done,             // serialisation done (64 bits shifted out)
    input       i_empty,                // TX FIFO empty
    input       i_dead_time_done,       // SB_CLOCK_CONTROLLER: 32-UI dead-time complete
                                        //   (was o_pack_finished; renamed to match
                                        //    clk_control's o_dead_time_done output)
    input       i_read_enable_sampled,  // TX FIFO ack: read data captured
    input       i_has_data,             // current message includes a data flit
    output reg  o_read_enable,          // pop header from TX FIFO
    output reg  o_read_data_enable,     // pop data flit from TX FIFO (second read)
    output reg  o_clk_en                // enable → SB_CLOCK_CONTROLLER i_enable
);

    // -----------------------------------------------------------------------
    // State encoding
    // -----------------------------------------------------------------------
    localparam [1:0]
        IDLE         = 2'b00,
        SENDING_PACK = 2'b01,
        SLEEPING     = 2'b10;

    reg [1:0] cs, ns;

    // r_data_phase: 0 = header phase, 1 = data flit phase
    reg r_data_phase;

    // -----------------------------------------------------------------------
    // State register + data-phase tracker
    // -----------------------------------------------------------------------
    always_ff @(posedge i_clk or negedge i_rst_n) begin
        if (!i_rst_n) begin
            cs           <= IDLE;
            r_data_phase <= 1'b0;
        end else begin
            cs <= ns;

            // Enter data-flit phase: SLEEPING (header dead-time) → SENDING_PACK
            // only when i_has_data == 1 and we have not yet sent the data flit
            if (cs == SLEEPING && ns == SENDING_PACK && i_has_data && !r_data_phase)
                r_data_phase <= 1'b1;
            // Clear on return to IDLE (message fully sent)
            else if (cs == SLEEPING && ns == IDLE)
                r_data_phase <= 1'b0;
            else if (cs == IDLE)
                r_data_phase <= 1'b0;
        end
    end

    // -----------------------------------------------------------------------
    // Next-state logic
    // -----------------------------------------------------------------------
    always_comb begin
        ns = cs;

        case (cs)
            IDLE: begin
                // Start sending when TX FIFO has something
                if (!i_empty)
                    ns = SENDING_PACK;
            end

            SENDING_PACK: begin
                // Wait for the 64-bit serialisation to complete
                if (i_ser_done)
                    ns = SLEEPING;
            end

            SLEEPING: begin
                // i_dead_time_done: SB_CLOCK_CONTROLLER signals end of 32-UI dead-time.
                // Decide next phase based on spec §4.1.5:
                //   After header dead-time and message HAS data → send data flit
                //   After data dead-time  OR  no-data message   → return to IDLE (or
                //                                                  next message if FIFO non-empty)
                if (i_dead_time_done) begin
                    if (i_has_data && !r_data_phase)
                        ns = SENDING_PACK;   // proceed to data flit
                    else if (!i_empty)
                        ns = SENDING_PACK;   // back-to-back message (PMO disabled)
                    else
                        ns = IDLE;
                end
            end

            default: ns = IDLE;
        endcase
    end

    // -----------------------------------------------------------------------
    // Clock enable output (drives SB_CLOCK_CONTROLLER i_enable)
    // Active during SENDING_PACK; SB_CLOCK_CONTROLLER self-manages the 32-UI gap.
    // -----------------------------------------------------------------------
    always_ff @(posedge i_clk or negedge i_rst_n) begin
        if (!i_rst_n)
            o_clk_en <= 1'b0;
        else if (cs == SENDING_PACK && i_read_enable_sampled)
            o_clk_en <= 1'b1;
        else
            o_clk_en <= 1'b0;
    end

    // -----------------------------------------------------------------------
    // FIFO read-enable outputs
    //   o_read_enable:      pop HEADER from TX FIFO
    //   o_read_data_enable: pop DATA FLIT from TX FIFO (second read for has_data msgs)
    // -----------------------------------------------------------------------
    always_ff @(posedge i_clk or negedge i_rst_n) begin
        if (!i_rst_n) begin
            o_read_enable      <= 1'b0;
            o_read_data_enable <= 1'b0;
        end else begin
            // Header pop: transition IDLE → SENDING_PACK
            //             OR SLEEPING (data-phase complete / no-data) → SENDING_PACK
            if ((cs == IDLE
                || (cs == SLEEPING && (!i_has_data || r_data_phase)))
                && ns == SENDING_PACK) begin
                o_read_enable <= 1'b1;
            end else if (i_read_enable_sampled) begin
                o_read_enable <= 1'b0;
            end

            // Data flit pop: transition SLEEPING (header done, i_has_data) → SENDING_PACK
            if (cs == SLEEPING && ns == SENDING_PACK && i_has_data && !r_data_phase) begin
                o_read_data_enable <= 1'b1;
            end else if (i_read_enable_sampled) begin
                o_read_data_enable <= 1'b0;
            end
        end
    end

endmodule : SB_TX_FSM_Modelling