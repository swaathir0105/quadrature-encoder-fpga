`timescale 1ns / 1ps
// tb_speed_engine.v -- standalone unit test for speed_engine.v
// Drives position_engine directly with a synthetic valid/Y pulse train at known
// intervals (bypassing signal_conditioner/quad_decoder) so exact target RPM values
// can be hit precisely, and checks speed_engine's rpm_x10 / method_sel / speed_stopped.
//
// Compile: iverilog -g2005 -o sim_speed position_engine.v speed_engine.v tb_speed_engine.v
// Run:     vvp sim_speed

module tb_speed_engine;
    localparam PPR = 100;
    localparam CLK_PERIOD = 10; // 100 MHz

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

    speed_engine #(.PPR(PPR)) u_speed (
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

    // One accepted edge, then idle for (period-1) cycles, so consecutive calls
    // land exactly `period` clock cycles apart -- matching position_engine's T_period.
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

    initial begin
        $display("T_METHOD_NUM=%0d F_METHOD_K10=%0d (expect 150000000, 150)",
                   u_speed.T_METHOD_NUM, u_speed.F_METHOD_K10);

        // ---- Case 1: stationary shaft, no pulses at all ----
        do_reset;
        wait_cycles(1_000_010);
        $display("[SPEED-STOPPED]   speed_stopped=%b (exp 1) rpm_x10=%0d (exp 0)",
                   speed_stopped, rpm_x10);

        // ---- Case 2: 20 RPM -> T_period=750,000 cyc, ~1.33 pulses/window -> T-method ----
        do_reset;
        send_pulse_train(750_000, 4);
        wait_cycles(50_000);
        $display("[SPEED-20RPM]     rpm_x10=%0d (exp ~200) method_sel=%b (exp 0) speed_stopped=%b (exp 0)",
                   rpm_x10, method_sel, speed_stopped);

        // ---- Case 3: 100 RPM -> T_period=150,000 cyc, ~6.67 pulses/window -> T-method ----
        // (well under CROSSOVER_PULSES=200, so this stays on T-method, which is exact here;
        //  raw F-method would have rounded 6.67 pulses to 6 or 7, i.e. -10%/+5% error)
        do_reset;
        send_pulse_train(150_000, 8);
        wait_cycles(50_000);
        $display("[SPEED-100RPM]    rpm_x10=%0d (exp 1000) method_sel=%b (exp 0) speed_stopped=%b (exp 0)",
                   rpm_x10, method_sel, speed_stopped);

        // ---- Case 4: 3000 RPM (top of spec) -> T_period=5,000 cyc, 200 pulses/window ----
        do_reset;
        send_pulse_train(5_000, 210);
        wait_cycles(50_000);
        $display("[SPEED-3000RPM]   rpm_x10=%0d (exp ~30000) method_sel=%b (exp 1) speed_stopped=%b (exp 0)",
                   rpm_x10, method_sel, speed_stopped);

        // ---- Case 5: Z_rise mid-rotation should never spike rpm_x10 ----
        // Re-establish 100 RPM, then pulse Z_rise and immediately check nothing has
        // latched a bogus reading before the next real edge re-measures T_period.
        do_reset;
        send_pulse_train(150_000, 5);
        @(posedge clk); Z_rise = 1'b1; @(posedge clk); Z_rise = 1'b0;
        wait_cycles(2);
        $display("[SPEED-ZRISE]     just after Z_rise: T_period=%0d (exp 1) rpm_x10 unchanged=%0d (no spike)",
                   T_period, rpm_x10);
        send_pulse_train(150_000, 8);
        wait_cycles(50_000);
        $display("[SPEED-ZRISE-END] after re-settling: rpm_x10=%0d (exp ~1000) speed_stopped=%b (exp 0)",
                   rpm_x10, speed_stopped);

        $display("Speed engine unit test complete.");
        #100; $finish;
    end
endmodule
