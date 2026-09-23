`timescale 1ns / 1ps
// speed_engine.v -- Stage 4: M/T hybrid speed estimation
//
// Below CROSSOVER_PULSES pulses per WINDOW_CYCLES window, speed is derived from
// T_period -- cycles between the last two valid quadrature edges -- reused directly
// from position_engine (T-method). At/above the crossover, speed is derived from a
// plain pulse count over the fixed window (F-method).
//
// CROSSOVER_PULSES defaults to 200 (crossover ~3000 RPM), not the ~45 RPM / 3
// pulses originally sketched. Raw fixed-window pulse counting has inherent +/-1
// count quantization -- roughly +/-(1/pulse_count) relative error -- which is up
// to ~33% at 3 pulses; simulation confirmed a 5% error at 100 RPM at that setting.
// T_period's clock-cycle resolution measures under 0.1% error across the whole
// 1-3000 RPM range instead, so T-method is kept as the primary path throughout the
// spec and F-method is reserved as a high-speed/above-spec fallback. Set this back
// to 3 if the report's originally stated crossover is preferred and the accuracy
// tradeoff near it is discussed/accepted in the TC-7 writeup instead.
//
// rpm_x10, method_sel and speed_stopped all update once every WINDOW_CYCLES
// (100 Hz at the default 10 ms window) so the whole module has one consistent
// sampling cadence, matching standard M/T-method practice.

module speed_engine #(
    parameter PPR              = 100,          // encoder pulses per revolution
    parameter CLK_FREQ_HZ      = 100_000_000,  // system clock, Hz
    parameter WINDOW_CYCLES    = 1_000_000,    // measurement window, clock cycles (10 ms @ 100 MHz)
    parameter CROSSOVER_PULSES = 200           // pulses/window threshold -> ~3000 RPM at defaults
)(
    input  wire        clk,
    input  wire        rst_n,
    input  wire        valid,          // 1-cycle strobe per accepted quadrature edge (from quad_decoder)
    input  wire [31:0] T_period,       // cycles between last two valid edges (from position_engine)
    input  wire [31:0] T_phase,        // cycles since the last valid edge (from position_engine)
    output reg  [15:0] rpm_x10,        // speed magnitude, RPM x10 fixed-point
    output reg         method_sel,     // 0 = T-method (low speed), 1 = F-method (high speed)
    output reg         speed_stopped   // 1 = shaft considered stationary this window; rpm_x10 forced to 0
);

    localparam integer COUNTS_PER_REV = PPR * 4;  // 400 at PPR=100, matches position_engine

    // rpm_x10 (T-method) = T_METHOD_NUM / T_period, where
    //   T_METHOD_NUM = f_clk * 60 / COUNTS_PER_REV * 10  (the x10 fixed-point scale)
    // Computed as (CLK_FREQ_HZ / COUNTS_PER_REV) * 600 rather than (CLK_FREQ_HZ * 600) /
    // COUNTS_PER_REV: the latter overflows 32-bit constant arithmetic (100e6*600 = 6e10 > 2^32).
    // Exact (no truncation) for the default parameters since COUNTS_PER_REV divides CLK_FREQ_HZ evenly.
    localparam integer T_METHOD_NUM = (CLK_FREQ_HZ / COUNTS_PER_REV) * 600; // = 150,000,000
    localparam integer F_METHOD_K10 = T_METHOD_NUM / WINDOW_CYCLES;         // = 150 (also exact here)

    // ---------------- Window timer + pulse accumulator (drives F-method) ----------------
    reg  [31:0] window_cnt;
    reg  [7:0]  pulse_cnt;
    wire [7:0]  pulse_cnt_next = pulse_cnt + (valid ? 8'd1 : 8'd0);
    wire        window_tick    = (window_cnt == WINDOW_CYCLES - 1);

    always @(posedge clk) begin
        if (!rst_n) begin
            window_cnt <= 32'd0;
            pulse_cnt  <= 8'd0;
        end else if (window_tick) begin
            window_cnt <= 32'd0;   // pulse_cnt_next already folds in this cycle's edge, if any
            pulse_cnt  <= 8'd0;
        end else begin
            window_cnt <= window_cnt + 32'd1;
            pulse_cnt  <= pulse_cnt_next;
        end
    end

    // ---------------- Method selection + stall / invalid-period detection ----------------
    // T_period == 1 only occurs right after reset or right after a Z-index homing pulse
    // (position_engine forces it there before the next edge re-measures it). Treat that as
    // "not yet a valid measurement", same as a genuine stall, instead of dividing by it --
    // otherwise a window_tick landing in that gap would compute a false ~65,000 RPM spike.
    wire period_valid = (T_period > 32'd1);
    wire stalled_now  = (!period_valid) || (T_phase > (2 * T_period));
    wire use_fmethod  = (pulse_cnt_next >= CROSSOVER_PULSES);
    wire [31:0] f_method_raw = pulse_cnt_next * F_METHOD_K10;

    // ------------------------------------------------------------------
    // TIMING FIX: T_METHOD_NUM / T_period used to be a single-cycle
    // combinational division, which (like position_engine's interpolator)
    // synthesizes to a very long combinational chain and fails timing on
    // real hardware even though it's exact in simulation. Unlike
    // position_engine's interp_term, this result is NOT clamped to a small
    // range (RPM needs the full quotient), so it needs a real divider --
    // just spread over multiple cycles instead of one.
    //
    // This is a standard 32-cycle bit-serial restoring divider, triggered
    // once per window_tick. It only needs to finish before the NEXT
    // window_tick, which is WINDOW_CYCLES away (>=2000 in every
    // configuration this design has been tested with) -- 32 cycles is a
    // trivial fraction of that, so this adds negligible latency to the
    // already-slow (~100Hz default) RPM update rate.
    // ------------------------------------------------------------------
    reg        div_busy;
    reg [5:0]  div_cnt;      // 0..32
    reg [31:0] div_Q, div_R, div_D;
    reg        pend_use_fmethod, pend_stalled;
    reg [31:0] pend_f_raw;

    always @(posedge clk) begin
        if (!rst_n) begin
            div_busy         <= 1'b0;
            div_cnt          <= 6'd0;
            div_Q            <= 32'd0;
            div_R            <= 32'd0;
            div_D            <= 32'd1;
            pend_use_fmethod <= 1'b0;
            pend_stalled     <= 1'b1;
            pend_f_raw       <= 32'd0;
            rpm_x10          <= 16'd0;
            method_sel       <= 1'b0;
            speed_stopped    <= 1'b1;
        end else if (window_tick) begin
            // Latch this window's decisions/inputs; start a fresh divide
            // (result only matters if not stalled and not using F-method,
            // but it's cheap enough to just always run it for simplicity).
            pend_use_fmethod <= use_fmethod;
            pend_stalled     <= stalled_now;
            pend_f_raw       <= f_method_raw;
            div_Q            <= T_METHOD_NUM[31:0];
            div_R            <= 32'd0;
            div_D            <= period_valid ? T_period : 32'd1;
            div_cnt          <= 6'd0;
            div_busy         <= 1'b1;
        end else if (div_busy) begin
            if (div_cnt < 6'd32) begin
                // One restoring-division step: shift dividend MSB into
                // remainder, then subtract divisor if it fits.
                if ({div_R[30:0], div_Q[31]} >= div_D) begin
                    div_R <= {div_R[30:0], div_Q[31]} - div_D;
                    div_Q <= {div_Q[30:0], 1'b1};
                end else begin
                    div_R <= {div_R[30:0], div_Q[31]};
                    div_Q <= {div_Q[30:0], 1'b0};
                end
                div_cnt <= div_cnt + 6'd1;
            end else begin
                // Division complete -- div_Q now holds T_METHOD_NUM / div_D.
                // Finalize this window's outputs.
                div_busy      <= 1'b0;
                method_sel    <= pend_use_fmethod;
                speed_stopped <= pend_stalled;
                if (pend_stalled) begin
                    rpm_x10 <= 16'd0;
                end else begin
                    rpm_x10 <= pend_use_fmethod
                             ? ((pend_f_raw > 32'hFFFF) ? 16'hFFFF : pend_f_raw[15:0])
                             : ((div_Q > 32'hFFFF) ? 16'hFFFF : div_Q[15:0]);
                end
            end
        end
    end

endmodule
