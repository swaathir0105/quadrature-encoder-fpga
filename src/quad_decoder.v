// quad_decoder.v  (improved — Stage 2, Sol 5 added)
//
// Decodes quadrature encoder Channel A & B signals.
//   valid      : pulses HIGH for one cycle when a valid transition is detected
//   Y          : direction (1 = anticlockwise, 0 = clockwise)
//
// Added — Sol 5 : Illegal state detector
//   A simultaneous change on both A and B in the same clock cycle is illegal
//   in Grey-code quadrature encoding (covers all 8 illegal combinations).
//   fault_flag : latches HIGH on any illegal transition
//   fault_clear: 1-cycle HIGH pulse clears fault_flag
//
//   On a fault:
//     - valid is NOT pulsed  →  counter in top module is frozen
//     - Y holds its last valid value
//     - fault_flag latches HIGH until cleared

`timescale 1ns / 1ps

module quad_decoder (
    input  wire clk,
    input  wire rst_n,          // Active-low synchronous reset
    input  wire A,              // Filtered, synchronised channel A
    input  wire B,              // Filtered, synchronised channel B
    input  wire fault_clear,    // Pulse HIGH for 1 cycle to clear fault_flag
    output reg  Y,              // Direction: 1 = anticlockwise, 0 = clockwise
    output reg  valid,          // Pulses HIGH for 1 cycle on valid transition
    output reg  fault_flag      // Latches HIGH on illegal state
);

    reg qA, qB;

    // ------------------------------------------------------------------
    // fault_detect : registered — captures the fault condition cleanly.
    //
    // fault_pulse is purely combinational: (A^qA)&(B^qB).
    // Because qA/qB are registered, they hold the PREVIOUS cycle's values
    // at the time the always block evaluates.  The expression is therefore
    // stable throughout the clock cycle and resolves to 1'b1 whenever
    // both inputs changed simultaneously since the last posedge.
    //
    // fault_flag priority: latch wins over clear (if both arrive together,
    // the fault is real and must not be silently erased).
    // ------------------------------------------------------------------

    always @(posedge clk) begin
        if (!rst_n) begin
            qA         <= 1'b0;
            qB         <= 1'b0;
            Y          <= 1'b0;
            valid      <= 1'b0;
            fault_flag <= 1'b0;

        end else begin

            // ---- Default: deassert valid every cycle ----
            valid <= 1'b0;

            // ---- Fault detection ----
            // Both A and B changed since last cycle → illegal Grey-code transition
            if ((A ^ qA) & (B ^ qB)) begin
                fault_flag <= 1'b1;     // Latch fault — valid stays 0, count frozen
            end else begin
                // ---- Normal (legal) transition ----
                if ({A, B} != {qA, qB}) begin
                    // K-map direction decode (unchanged from original Sarkar design)
                    Y <= ( (~qA & ~qB & ~A &  B) |
                           ( qA & ~qB & ~A & ~B) |
                           ( qA &  qB &  A & ~B) |
                           (~qA &  qB &  A &  B) );
                    valid <= 1'b1;
                end

                // Clear fault only when no new fault this cycle
                if (fault_clear)
                    fault_flag <= 1'b0;
            end

            // ---- Update history registers every cycle ----
            qA <= A;
            qB <= B;
        end
    end

endmodule
