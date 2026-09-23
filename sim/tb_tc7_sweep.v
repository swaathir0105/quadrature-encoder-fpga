`timescale 1ns / 1ps
// tb_tc7_sweep.v -- TC-7: RPM accuracy sweep, 1-3000 RPM, target <1% error
//
// SIMULATION SPEEDUP NOTE:
// speed_engine's RPM math (T_METHOD_NUM, F_METHOD_K10) is fully parameterized
// by CLK_FREQ_HZ and WINDOW_CYCLES. Running the real hardware config
// (100 MHz, 1,000,000-cycle window) for a full 1-3000 RPM sweep requires tens
// of millions of simulated clock cycles per low-RPM test point (a 1 RPM pulse
// is 15,000,000 cycles wide at 100 MHz) -- prohibitively slow to simulate.
// Instead this sweep uses a scaled CLK_FREQ_HZ=1,000,000 (1 MHz) and
// WINDOW_CYCLES=2,000, with CROSSOVER_PULSES rescaled so the F-method
// crossover still lands at 3000 RPM, matching the real hardware's design
// intent. Because every quantity in speed_engine scales linearly with
// CLK_FREQ_HZ, the measured RPM error at each point is representative of
// the real 100 MHz/1,000,000-cycle hardware configuration.
//
// Pulse period formula (cycles between successive quadrature edges) at the
// scaled clock: period = 60 * CLK_FREQ_HZ / (RPM * COUNTS_PER_REV)
//                       = 60,000,000 / (RPM * 400) = 150,000 / RPM
//
// Compile: iverilog -g2005 -o sim_tc7 position_engine.v speed_engine.v tb_tc7_sweep.v
// Run:     vvp sim_tc7

module tb_tc7_sweep;
    localparam PPR              = 100;
    localparam CLK_PERIOD       = 10;         // arbitrary sim time unit
    localparam CLK_FREQ_HZ_SIM  = 1_000_000;  // scaled sim clock (see note above)
    localparam WINDOW_CYCLES    = 2_000;      // scaled measurement window
    // pulses/window at 3000 RPM = WINDOW_CYCLES / period(3000) = 2000 / 50 = 40
    // -> set crossover here so F-method engages at 3000 RPM, matching hardware intent
    localparam CROSSOVER_PULSES = 40;

    reg clk, rst_n, valid, Y, Z_rise;
    wire signed [31:0] count;
    wire [15:0] base_theta_x10, theta_interp_x10;
    wire [31:0] T_period, T_phase;
    wire [15:0] rpm_x10;
    wire        method_sel, speed_stopped;

    position_engine #(.PPR(PPR)) u_pos (
        .clk(clk), .rst_n(rst_n), .valid(valid), .Y(Y), .Z_rise(Z_rise),
        .count(count), .base_theta_x10(base_theta_x10),
        .theta_interp_x10(theta_interp_x10), .T_period(T_period), .T_phase(T_phase)
    );

    speed_engine #(
        .PPR(PPR), .CLK_FREQ_HZ(CLK_FREQ_HZ_SIM),
        .WINDOW_CYCLES(WINDOW_CYCLES), .CROSSOVER_PULSES(CROSSOVER_PULSES)
    ) u_speed (
        .clk(clk), .rst_n(rst_n), .valid(valid),
        .T_period(T_period), .T_phase(T_phase),
        .rpm_x10(rpm_x10), .method_sel(method_sel), .speed_stopped(speed_stopped)
    );

    initial clk = 0;
    always #(CLK_PERIOD/2) clk = ~clk;

    task do_reset; integer i;
        begin
            rst_n = 0; valid = 0; Y = 0; Z_rise = 0;
            for (i=0;i<4;i=i+1) @(posedge clk);
            rst_n = 1; @(posedge clk);
        end
    endtask

    task wait_cycles; input integer n; integer i;
        begin for (i=0;i<n;i=i+1) @(posedge clk); end
    endtask

    task send_pulse; input integer period;
        begin
            valid = 1'b1; Y = 1'b1;
            @(posedge clk); #1;
            valid = 1'b0;
            wait_cycles(period - 1);
        end
    endtask

    task send_pulse_train; input integer period; input integer n; integer k;
        begin for (k=0;k<n;k=k+1) send_pulse(period); end
    endtask

    // ---- TC-7 sweep ----
    real    target_rpm;
    integer period_cycles;
    integer n_pulses;
    real    measured_rpm;
    real    err_pct;
    integer pass_count, fail_count;
    integer idx;
    real    rpm_list [0:15];

    initial begin
        rpm_list[0]  = 1.0;
        rpm_list[1]  = 2.0;
        rpm_list[2]  = 5.0;
        rpm_list[3]  = 10.0;
        rpm_list[4]  = 20.0;
        rpm_list[5]  = 45.0;
        rpm_list[6]  = 50.0;
        rpm_list[7]  = 100.0;
        rpm_list[8]  = 300.0;
        rpm_list[9]  = 500.0;
        rpm_list[10] = 1000.0;
        rpm_list[11] = 1500.0;
        rpm_list[12] = 2000.0;
        rpm_list[13] = 2500.0;
        rpm_list[14] = 2999.0;
        rpm_list[15] = 3000.0;

        pass_count = 0; fail_count = 0;

        $display("=== TC-7: RPM Accuracy Sweep (target <1%% error, 1-3000 RPM) ===");
        $display("Target_RPM  Period_cyc  Measured_RPM  Error_pct  Method  Result");

        for (idx = 0; idx <= 15; idx = idx + 1) begin
            target_rpm = rpm_list[idx];

            // period = 150,000 / RPM (scaled clock), rounded to nearest cycle
            period_cycles = $rtoi(150000.0 / target_rpm + 0.5);

            // enough pulses to guarantee a window_tick falls WHILE pulses are
            // still steadily arriving (avoids a false stall from going silent
            // right before the tick, which happens whenever period << window)
            n_pulses = (WINDOW_CYCLES / period_cycles) + 3;

            do_reset;
            send_pulse_train(period_cycles, n_pulses);

            measured_rpm = rpm_x10 / 10.0;
            err_pct = 100.0 * (measured_rpm - target_rpm) / target_rpm;
            if (err_pct < 0) err_pct = -err_pct;

            if (!speed_stopped && err_pct < 1.0) begin
                $display("%8.1f  %10d  %11.2f  %8.4f%%  %s      PASS",
                    target_rpm, period_cycles, measured_rpm, err_pct,
                    method_sel ? "F" : "T");
                pass_count = pass_count + 1;
            end else begin
                $display("%8.1f  %10d  %11.2f  %8.4f%%  %s      FAIL (speed_stopped=%b)",
                    target_rpm, period_cycles, measured_rpm, err_pct,
                    method_sel ? "F" : "T", speed_stopped);
                fail_count = fail_count + 1;
            end
        end

        $display("=== TC-7 SUMMARY: %0d PASS, %0d FAIL out of %0d points ===", pass_count, fail_count, pass_count+fail_count);
        $display("TC-7 sweep complete.");
        #100; $finish;
    end
endmodule
