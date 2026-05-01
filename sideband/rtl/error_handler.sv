// =============================================================================
// sb_error_handler  –  UCIe Sideband Error Handler
//
// Spec reference: UCIe Specification Rev 3.0 Ver 1.0 (August 2025)
//
// ── Scope ─────────────────────────────────────────────────────────────────────
//
// This module handles STANDARD sideband packets only:
//   - Register Access packets      (§7.1.2.1 — CFG/MMIO reads and writes)
//   - Link Management Messages     (§7.1.2 messages without data, Table 7-8/7-9)
//
// NOT in scope for this module:
//   - Management Port Messages (MPM, §7.1.2.4 / §7.1.2.5)
//   - Priority Sideband Traffic Packets (PSTP, §7.1.2.6)
//   - Vendor-defined messages (§7.1.2 Table 7-8 / Table 7-10)
//
// Both packet types in scope share the same 64-bit header + optional 64-bit
// data format (§7.1.2), so a single CP/DP parity check path covers both.
//
// ── What this module checks ──────────────────────────────────────────────────
//
// Parity errors on sideband packets received over FDI. Sideband packets
// are serialised 64-bit transfers carried on the dedicated sideband clock/data
// pin pair (§4.1.5), NOT on the mainband data lanes.
//
// ── Sideband parity error rules (§7.1.3.2) ───────────────────────────────────
//
// §7.1.3.2 (UCIe sideband link between dies):
//   "The BER of the sideband Link is 1e-27 or better. Hence, no retry
//    mechanism is provided for the sideband packets. Receivers of sideband
//    packets must check for Data or Control parity errors, and any of these
//    errors is mapped to a fatal UIE."
//
// ── What does NOT belong here ─────────────────────────────────────────────────
//
// The per-lane comparison and aggregate error counters described in §4.4
// ("Data to Clock Training and Test Modes") apply exclusively to MAINBAND
// lane training performed by the Physical Layer during MBTRAIN states.
// Those counters live in the PHY training logic and are read out by the
// Adapter over sideband messages — they must never be driven by sideband
// packet parity checks.
//
// ── CP / DP field positions (§7.1.2 Table 7-5 / Table 7-7) ──────────────────
//
// CP (Control Parity):
//   Even parity of all header bits excluding DP.
//   Transmitted in the header word at bit [57]; DP is at bit [56].
//   To verify: zero out bits [57] and [56] of the received header, then
//   XOR-reduce all 64 bits — result must equal header[57].
//
// DP (Data Parity):
//   Even parity of all bits in the data payload.
//   "If there is no data payload, this bit is set to 0b." (§7.1.2 Table 7-5)
//   Transmitted in the header word at bit [56].
//   Only checked when i_has_data == 1.
// =============================================================================

module sb_error_handler (
    input  logic        clk,
    input  logic        rst_n,

    // -------------------------------------------------------------------------
    // Sideband packet from RX path (latched after serial deserialisation)
    // -------------------------------------------------------------------------
    input  logic        i_pkt_valid,    // header word is valid and complete
    input  logic [63:0] i_header,       // 64-bit sideband header flit
    input  logic [63:0] i_data,         // 64-bit data flit (ignored when !i_has_data)
    input  logic        i_has_data,     // 1 = a data flit accompanies this header

    // -------------------------------------------------------------------------
    // Outputs
    // -------------------------------------------------------------------------

    // Sideband parity error flags (combinational, for visibility / debug)
    output logic        o_cp_error,     // CP mismatch on current packet
    output logic        o_dp_error      // DP mismatch on current packet
);

    // =========================================================================
    // CP verification  (§7.1.2 Table 7-5)
    // =========================================================================
    // CP = even_parity(header with CP and DP bit positions both zeroed).
    // Received CP is header[57]; received DP placeholder is header[56].

    logic [63:0] w_hdr_for_cp;
    logic        w_cp_calc;

    always_comb begin
        w_hdr_for_cp      = i_header;
        w_hdr_for_cp[57]  = 1'b0;   // zero out the CP field itself
        w_hdr_for_cp[56]  = 1'b0;   // zero out the DP field (excluded from CP)
        w_cp_calc         = ^w_hdr_for_cp; // even parity = XOR reduce
    end

    // CP mismatch: calculated parity must equal received CP bit
    assign o_cp_error = i_pkt_valid && (w_cp_calc != i_header[57]);

    // =========================================================================
    // DP verification  (§7.1.2 Table 7-5)
    // =========================================================================
    // DP = even_parity(data payload). Zero when no data payload.
    // Received DP is header[56].

    logic w_dp_calc;
    assign w_dp_calc = i_has_data ? (^i_data) : 1'b0;

    // DP mismatch: only meaningful when a data flit is present
    assign o_dp_error = i_pkt_valid && i_has_data && (w_dp_calc != i_header[56]);

endmodule : sb_error_handler