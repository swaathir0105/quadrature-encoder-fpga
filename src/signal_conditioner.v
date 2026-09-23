// signal_conditioner.v
// Stage 1 — Signal Conditioning
//
// Sol 2 : 2-FF synchroniser  — eliminates metastability risk
// Sol 1 : 4-stage majority-vote shift register — rejects glitches < 4 cycles
//
// Latency : 2 (sync) + 4 (filter) = 6 clock cycles = 60 ns @ 100 MHz
// Minimum accepted pulse width : 4 cycles = 40 ns
// At 3000 RPM, 100 PPR : valid dwell ≈ 50 µs >> 40 ns  (no valid pulse discarded)
//
// NOTE: Do NOT use this module for the Z (index) channel.
//       Z pulses are intentionally narrow and must pass through the
//       2-FF synchroniser only.  Instantiate those two flip-flops
//       directly in quad_encoder_top.

`timescale 1ns / 1ps

module signal_conditioner (
    input  wire clk,
    input  wire rst_n,          // Active-low synchronous reset
    input  wire raw_in,         // Direct pin input (asynchronous, may be noisy)
    output reg  filtered_out    // Clean, synchronised, glitch-free output
);

    // ------------------------------------------------------------------
    // Stage A : 2-FF synchroniser
    // ------------------------------------------------------------------
    reg sync1, sync2;

    always @(posedge clk) begin
        if (!rst_n) begin
            sync1 <= 1'b0;
            sync2 <= 1'b0;
        end else begin
            sync1 <= raw_in;    // FF1: captures asynchronous input
            sync2 <= sync1;     // FF2: resolves any metastability
        end
    end

    // ------------------------------------------------------------------
    // Stage B : 4-stage majority-vote shift register
    // Output changes ONLY when all 4 stages agree.
    // Any transition shorter than 4 clock cycles is silently absorbed.
    // ------------------------------------------------------------------
    reg [3:0] sr;

    always @(posedge clk) begin
        if (!rst_n)
            sr <= 4'b0000;
        else
            sr <= {sr[2:0], sync2};     // Shift in from synchroniser
    end

    always @(posedge clk) begin
        if (!rst_n)
            filtered_out <= 1'b0;
        else if (sr == 4'b1111)
            filtered_out <= 1'b1;       // All four stages HIGH  → accept HIGH
        else if (sr == 4'b0000)
            filtered_out <= 1'b0;       // All four stages LOW   → accept LOW
        // else: stages disagree → hold last stable value (glitch rejected)
    end

endmodule
