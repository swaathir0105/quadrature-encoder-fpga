// position_engine.v
// Stage 3 — Position Engine
//
// Sol 3 : 32-bit signed counter
//   Range : ±2,147,483,647 counts = ±5,368,709 revolutions (vs 127 counts
//   with the original 8-bit counter).  Overflow is now physically impossible
//   for any mechanical system.
//
// Sol 6 : Sub-PPR interpolation
//   Base angle (theta_x10) steps in 0.9° increments (= 9 tenth-degrees)
//   at 100 PPR with ×4 quadrature decoding.  The interpolator estimates
//   position between pulses using the ratio of elapsed time to the last
//   inter-pulse period:
//
//       theta_interp_x10 = base_theta_x10 + (T_phase × 9) / T_period
//
//   T_period : clock cycles between the previous two valid pulses
//              (captured at the moment the most recent valid pulse arrived)
//   T_phase  : clock cycles elapsed since the most recent valid pulse
//   Factor 9 : one quadrature step in tenth-degrees (3600 / 400 = 9)
//
//   Interpolated output is clamped to base_theta_x10 + 8 to ensure it
//   never reaches the NEXT base value (which would be double-counted).
//   Wraparound at 3600 (360.0°) is handled explicitly.
//
// Stop condition:
//   If T_phase > 2 × T_period the motor is assumed to have stopped or
//   slowed dramatically.  theta_interp_x10 is frozen at base_theta_x10
//   to avoid accumulating phantom position.
//
// Interface:
//   valid   — 1-cycle pulse from quad_decoder on every legal transition
//   Y       — direction (1 = anticlockwise, 0 = clockwise) from quad_decoder
//   Z_rise  — 1-cycle pulse from quad_encoder_top on Z rising edge
//   count   — 32-bit signed accumulated count (Sol 3)
//   base_theta_x10    — angle at last valid pulse, ×10 degrees [0..3599]
//   theta_interp_x10  — interpolated angle, ×10 degrees [0..3599]
//   T_period          — inter-pulse period in clock cycles (for Stage 4 reuse)
//   T_phase           — time since last pulse in clock cycles (diagnostic)

`timescale 1ns / 1ps

module position_engine #(
    parameter PPR = 100
)(
    input  wire        clk,
    input  wire        rst_n,
    input  wire        valid,       // From quad_decoder: 1-cycle pulse per legal transition
    input  wire        Y,           // Direction from quad_decoder (1=ACW, 0=CW)
    input  wire        Z_rise,      // Rising edge of Z (index): resets count to 0

    output reg  signed [31:0] count,           // Sol 3: 32-bit signed position counter
    output reg         [15:0] base_theta_x10,  // Angle at last pulse, ×10 [0..3599]
    output reg         [15:0] theta_interp_x10,// Interpolated angle, ×10 [0..3599]
    output reg         [31:0] T_period,        // Inter-pulse period (clock cycles)
    output reg         [31:0] T_phase          // Elapsed cycles since last pulse
);

    // ------------------------------------------------------------------
    // Local parameters
    // ------------------------------------------------------------------
    localparam integer COUNTS_PER_REV = PPR * 4;          // 400 for 100 PPR
    localparam integer THETA_MAX      = 3600;              // 360.0 × 10
    localparam integer STEP_X10       = THETA_MAX / COUNTS_PER_REV; // 9 tenth-deg/count

    // ------------------------------------------------------------------
    // Free-running inter-pulse timer
    //
    // T_phase counts up every clock cycle since the last valid pulse.
    // On each valid pulse: T_period is snapped from T_phase, then
    // T_phase resets to 1 (so the very next cycle T_phase = 1, not 0,
    // which avoids a divide-by-zero if two valid pulses land back-to-back).
    //
    // T_period is initialised to 1 (not 0) to guard the division on
    // startup before any pulse has arrived.
    // ------------------------------------------------------------------

    always @(posedge clk) begin
        if (!rst_n) begin
            T_phase  <= 32'd0;
            T_period <= 32'd1;   // Non-zero so interpolator is safe at power-up
        end else if (Z_rise) begin
            // Z homing resets the timer too — position is now known exactly
            T_phase  <= 32'd0;
            T_period <= 32'd1;
        end else if (valid) begin
            // Capture inter-pulse period; reset phase timer
            T_period <= (T_phase == 32'd0) ? 32'd1 : T_phase;
            T_phase  <= 32'd1;
        end else begin
            // Free-running increment; saturate at 32'hFFFF_FFFF to avoid rollover
            T_phase <= (T_phase == 32'hFFFF_FFFF) ? T_phase : T_phase + 32'd1;
        end
    end

    // ------------------------------------------------------------------
    // 32-bit signed position counter (Sol 3)
    //
    // Priority (highest first):
    //   1. Z_rise  : absolute home → count = 0
    //   2. valid   : increment (CW) or decrement (ACW)
    // ------------------------------------------------------------------
    always @(posedge clk) begin
        if (!rst_n) begin
            count <= 32'sd0;
        end else if (Z_rise) begin
            count <= 32'sd0;
        end else if (valid) begin
            if (Y)
                count <= count + 32'sd1;
            else
                count <= count - 32'sd1;
        end
    end

    // ------------------------------------------------------------------
    // Base angle computation
    //
    // base_theta_x10 = (|count| mod COUNTS_PER_REV) × STEP_X10
    //
    // count is signed; the modulo must always be taken on the magnitude
    // so that CW and ACW rotation both produce angles in [0, THETA_MAX).
    //
    // Computed combinatorially, then registered on valid or Z_rise so
    // it only snaps to a new value at a genuine pulse edge.
    //
    // Intermediate wire is 48 bits wide to safely hold
    //   2,147,483,647 × 9 = 19,327,352,823 (requires 35 bits).
    // ------------------------------------------------------------------
    // ------------------------------------------------------------------
    // TIMING FIX: abs_count % COUNTS_PER_REV (400, not a power of 2) still
    // requires real division-like hardware, and splitting it across just
    // one extra register (an earlier attempt) wasn't enough -- it showed
    // its own -2.75ns violation once the bigger interpolator violation was
    // fixed. This reuses the exact same proven 32-cycle restoring-divider
    // pattern as speed_engine.v's T_METHOD_NUM/T_period division, just
    // keeping the remainder instead of the quotient. base_theta_x10 only
    // needs to update once per quadrature pulse, so 32 extra cycles here
    // is trivial slack (pulses are >=~5000 cycles apart even at 3000 RPM).
    //
    // NOTE: an incremental "track count_mod alongside count" shortcut was
    // considered instead, but rejected: it has a subtle correctness trap
    // exactly at count==0 (where |count| reflects rather than continuing
    // to decrement), which a naive same-direction-as-count tracker gets
    // wrong. Reusing the already-verified divider pattern avoids
    // introducing a new, harder-to-verify algorithm for a timing fix.
    // ------------------------------------------------------------------
    reg        mdiv_busy;
    reg [5:0]  mdiv_cnt;
    reg [31:0] mdiv_Q, mdiv_R;
    reg        commit_pending;
    reg        commit_is_zrise;

    wire [31:0] abs_count;
    assign abs_count = (count[31]) ? (~count + 32'd1) : count;   // |count|

    always @(posedge clk) begin
        if (!rst_n) begin
            mdiv_busy       <= 1'b0;
            mdiv_cnt        <= 6'd0;
            mdiv_Q          <= 32'd0;
            mdiv_R          <= 32'd0;
            commit_pending  <= 1'b0;
            commit_is_zrise <= 1'b0;
        end else if (Z_rise) begin
            // Z homing: result is 0, no division needed -- commit immediately
            mdiv_busy       <= 1'b0;
            commit_pending  <= 1'b1;
            commit_is_zrise <= 1'b1;
        end else if (valid) begin
            // Start a fresh modulo computation: abs_count % COUNTS_PER_REV
            mdiv_Q          <= abs_count;
            mdiv_R          <= 32'd0;
            mdiv_cnt        <= 6'd0;
            mdiv_busy       <= 1'b1;
            commit_is_zrise <= 1'b0;
            commit_pending  <= 1'b0;
        end else if (mdiv_busy) begin
            if (mdiv_cnt < 6'd32) begin
                if ({mdiv_R[30:0], mdiv_Q[31]} >= COUNTS_PER_REV[31:0]) begin
                    mdiv_R <= {mdiv_R[30:0], mdiv_Q[31]} - COUNTS_PER_REV[31:0];
                    mdiv_Q <= {mdiv_Q[30:0], 1'b1};
                end else begin
                    mdiv_R <= {mdiv_R[30:0], mdiv_Q[31]};
                    mdiv_Q <= {mdiv_Q[30:0], 1'b0};
                end
                mdiv_cnt <= mdiv_cnt + 6'd1;
            end else begin
                // mdiv_R now holds abs_count % COUNTS_PER_REV
                mdiv_busy      <= 1'b0;
                commit_pending <= 1'b1;
            end
        end else begin
            commit_pending <= 1'b0;
        end
    end

    wire [47:0] base_raw;
    assign base_raw = mdiv_R * STEP_X10;                       // [0..3591]

    // base_raw is already in [0, THETA_MAX) so no further mod needed
    // (399 × 9 = 3591 < 3600).  But guard defensively:
    wire [15:0] base_next;
    assign base_next = (base_raw >= THETA_MAX)
                       ? (base_raw - THETA_MAX)
                       : base_raw[15:0];

    always @(posedge clk) begin
        if (!rst_n) begin
            base_theta_x10 <= 16'd0;
        end else if (commit_pending) begin
            base_theta_x10 <= commit_is_zrise ? 16'd0 : base_next;
        end
    end

    // ------------------------------------------------------------------
    // Sub-PPR interpolator (Sol 6) -- pipelined, timing-fixed version
    //
    // theta_interp_x10 = base_theta_x10 + (T_phase × STEP_X10) / T_period
    //
    // TIMING FIX: the original single-cycle combinational division
    // ((T_phase * STEP_X10) / T_period) synthesized to ~369 logic levels
    // (a full 32-bit runtime divider), causing a -97ns setup violation on
    // a 10ns (100MHz) clock during Vivado implementation -- it passed
    // Icarus Verilog simulation cleanly because functional simulation
    // assumes zero gate delay and never models real propagation delay.
    //
    // Since interp_term is clamped to [0, STEP_X10-1] = [0,8] regardless
    // of the true quotient, the full division is unnecessary: we only
    // need to find the largest k in 0..8 such that k*T_period <= numerator.
    // This is computed as an 8-stage pipeline of cheap add+compare steps
    // (one 36-bit add/compare per stage) instead of one giant combinational
    // divider. This adds ~9 cycles (90ns) of latency to theta_interp_x10,
    // which is completely negligible against real mechanical dynamics
    // (base_theta_x10 only changes once per quadrature pulse, which even
    // at the top of spec -- 3000 RPM -- is ~5000 cycles apart).
    //
    // Stop condition and wraparound logic are unchanged and stay
    // combinational off the live registers, since a plain compare/add is
    // cheap and was never the source of the timing violation.
    // ------------------------------------------------------------------

    // Stage 0: snapshot numerator and a divide-by-zero-safe T_period
    reg [35:0] numer_p0;
    reg [31:0] tper_p0;
    always @(posedge clk) begin
        if (!rst_n) begin
            numer_p0 <= 36'd0;
            tper_p0  <= 32'd1;
        end else begin
            numer_p0 <= T_phase * STEP_X10;
            tper_p0  <= (T_period == 32'd0) ? 32'd1 : T_period;
        end
    end

    // Stages 1-8: iterative "does k*T_period fit in numerator" pipeline.
    // acc accumulates k*T_period; k stops incrementing once it no longer fits
    // (monotonic, so once false it stays false for the remaining stages).
    reg [35:0] numer_p [1:8];
    reg [31:0] tper_p  [1:8];
    reg [35:0] acc_p   [1:8];
    reg [3:0]  k_p     [1:8];
    integer s;

    always @(posedge clk) begin
        if (!rst_n) begin
            for (s = 1; s <= 8; s = s + 1) begin
                numer_p[s] <= 36'd0;
                tper_p[s]  <= 32'd1;
                acc_p[s]   <= 36'd0;
                k_p[s]     <= 4'd0;
            end
        end else begin
            // Stage 1 draws from the stage-0 snapshot
            numer_p[1] <= numer_p0;
            tper_p[1]  <= tper_p0;
            if ({4'd0, tper_p0} <= numer_p0) begin
                acc_p[1] <= {4'd0, tper_p0};
                k_p[1]   <= 4'd1;
            end else begin
                acc_p[1] <= 36'd0;
                k_p[1]   <= 4'd0;
            end
            // Stages 2-8 each add one more T_period if it still fits
            for (s = 2; s <= 8; s = s + 1) begin
                numer_p[s] <= numer_p[s-1];
                tper_p[s]  <= tper_p[s-1];
                if ((acc_p[s-1] + {4'd0, tper_p[s-1]}) <= numer_p[s-1]) begin
                    acc_p[s] <= acc_p[s-1] + {4'd0, tper_p[s-1]};
                    k_p[s]   <= k_p[s-1] + 4'd1;
                end else begin
                    acc_p[s] <= acc_p[s-1];
                    k_p[s]   <= k_p[s-1];
                end
            end
        end
    end

    // Final stage output is the clamped interpolation term (0..8), exact
    // match for what the old (STEP_X10-1)-clamped division used to produce
    wire [15:0] interp_term;
    assign interp_term = {12'd0, k_p[8]};

    // Stop condition flag -- unchanged, cheap compare, not a timing source
    wire stopped;
    assign stopped = (T_phase > (2 * T_period));

    // Final interpolated value (combinatorial, registered below)
    wire [16:0] interp_sum;    // 17 bits to detect overflow past 3599
    assign interp_sum = stopped ? {1'b0, base_theta_x10}
                                : ({1'b0, base_theta_x10} + {1'b0, interp_term});

    wire [15:0] interp_wrapped;
    assign interp_wrapped = (interp_sum >= THETA_MAX)
                            ? (interp_sum - THETA_MAX)
                            : interp_sum[15:0];

    // Register the output so it is stable for the downstream AXI4-Lite
    // interface (Stage 5) and for TC-6 sampling in the testbench.
    always @(posedge clk) begin
        if (!rst_n) begin
            theta_interp_x10 <= 16'd0;
        end else if (Z_rise) begin
            theta_interp_x10 <= 16'd0;
        end else begin
            theta_interp_x10 <= interp_wrapped;
        end
    end

endmodule
