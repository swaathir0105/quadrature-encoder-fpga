`timescale 1ns / 1ps
// Compile: iverilog -g2005 -o sim_stage4 signal_conditioner.v quad_decoder.v position_engine.v speed_engine.v quad_encoder_top.v tb_quad_encoder.v
// Run:     vvp sim_stage4
// Outputs: quad_encoder.vcd, encoder_log.csv

module tb_quad_encoder;
    localparam PPR=100, SPEED_WINDOW=1_000_000, CLK_PERIOD=10;

    reg  clk, rst_n, A_pin, B_pin, Z_pin, fault_clear;
    wire signed [31:0] count;
    wire [15:0] theta_interp_x10, base_theta_x10;
    wire [31:0] T_period, T_phase;
    wire [15:0] rpm_x10;
    wire        method_sel, speed_stopped;
    wire        direction, fault_flag;

    quad_encoder_top #(.PPR(PPR),.SPEED_WINDOW(SPEED_WINDOW)) dut (
        .clk(clk),.rst_n(rst_n),.A_pin(A_pin),.B_pin(B_pin),.Z_pin(Z_pin),
        .fault_clear(fault_clear),.count(count),.theta_interp_x10(theta_interp_x10),
        .base_theta_x10(base_theta_x10),.T_period(T_period),.T_phase(T_phase),
        .rpm_x10(rpm_x10),.method_sel(method_sel),.speed_stopped(speed_stopped),
        .direction(direction),.fault_flag(fault_flag)
    );

    initial clk = 0;
    always #(CLK_PERIOD/2) clk = ~clk;

    wire [15:0] theta_deg  = theta_interp_x10 / 10;
    wire [15:0] theta_frac = theta_interp_x10 % 10;

    reg [15:0] captured_base, captured_interp, sc_base, sc_interp;
    reg [31:0] captured_T_per, captured_T_ph, sc_T_ph;

    // --- DEBUG: independent valid-pulse counter for TC-3 dropout investigation ---
    integer valid_pulse_cnt;
    always @(posedge clk) begin
        if (!rst_n) valid_pulse_cnt <= 0;
        else if (dut.valid) valid_pulse_cnt <= valid_pulse_cnt + 1;
    end

    integer fp;
    real actual_deg, measured_deg, quant_err, latency_err, mech_err, total_err;
    real TC9_PI, TC9_A_ECC, TC9_A_GRAD, TC9_LATENCY_S, TC9_OMEGA_DPS;
    integer tc9_cnt, tc9_hold, tc9_i, tc9_st, tc9_a, tc9_b;
    reg [0:0] cw_a [0:3]; reg [0:0] cw_b [0:3];

    initial begin $dumpfile("quad_encoder.vcd"); $dumpvars(0, tb_quad_encoder); end

    task apply_state; input a_val, b_val; input integer hold_cycles; integer i;
        begin A_pin=a_val; B_pin=b_val; for(i=0;i<hold_cycles;i=i+1) @(posedge clk); #1; end
    endtask
    task rotate_cw; input integer reps, hold; integer r;
        begin for(r=0;r<reps;r=r+1) begin
            apply_state(0,0,hold); apply_state(0,1,hold);
            apply_state(1,1,hold); apply_state(1,0,hold); end end
    endtask
    task rotate_acw; input integer reps, hold; integer r;
        begin for(r=0;r<reps;r=r+1) begin
            apply_state(0,0,hold); apply_state(1,0,hold);
            apply_state(1,1,hold); apply_state(0,1,hold); end end
    endtask

    initial begin
        A_pin=0; B_pin=0; Z_pin=0; fault_clear=0;
        rst_n=0; repeat(4) @(posedge clk); rst_n=1; @(posedge clk);

        // TC-1: Glitch filter
        A_pin=1; repeat(3) @(posedge clk); A_pin=0; repeat(10) @(posedge clk);
        A_pin=1; repeat(4) @(posedge clk); A_pin=0; repeat(10) @(posedge clk);

        // TC-2: Sync latency
        @(posedge clk); #3; A_pin=1; repeat(5) @(posedge clk); A_pin=0; repeat(5) @(posedge clk);

        rst_n=0; repeat(4) @(posedge clk); rst_n=1; @(posedge clk); A_pin=0; B_pin=0;

        // TC-8: Regression
        rotate_acw(25,10); repeat(20) @(posedge clk); rotate_cw(25,10);
        begin:vibration integer v; for(v=0;v<10;v=v+1)
            begin apply_state(1,0,10); apply_state(0,0,10); end end
        rotate_acw(10,10); rotate_cw(10,10);

        // TC-4: Illegal state
        rst_n=0; repeat(4) @(posedge clk); rst_n=1; @(posedge clk); A_pin=0; B_pin=0;
        rotate_acw(4,10); A_pin=0; B_pin=0; repeat(15) @(posedge clk);
        @(posedge clk); A_pin=1; B_pin=1; repeat(10) @(posedge clk);
        $display("[TC-4] fault_flag=%b (exp 1) count=%0d", fault_flag, count);
        @(posedge clk); fault_clear=1; @(posedge clk); fault_clear=0;
        $display("[TC-4] After fault_clear: fault_flag=%b (exp 0)", fault_flag);

        // TC-5: Z homing
        rst_n=0; repeat(4) @(posedge clk); rst_n=1; @(posedge clk); A_pin=0; B_pin=0; Z_pin=0;
        rotate_acw(10,10);
        @(posedge clk); Z_pin=1; @(posedge clk); Z_pin=0; repeat(3) @(posedge clk);
        $display("[TC-5] Count after Z = %0d (exp 0)", count);

        // TC-3: 32-bit range
        rst_n=0; repeat(4) @(posedge clk); rst_n=1; @(posedge clk); A_pin=0; B_pin=0;
        rotate_cw(50,10);
        $display("[TC-3] count=%0d (exp ~+200) valid_pulse_cnt=%0d (exp ~199)", count, valid_pulse_cnt);
        rst_n=0; repeat(4) @(posedge clk); rst_n=1; @(posedge clk); A_pin=0; B_pin=0;
        rotate_acw(50,10);
        $display("[TC-3] count=%0d (exp ~-200) valid_pulse_cnt=%0d (exp ~199)", count, valid_pulse_cnt);

        // TC-6: Interpolation + stop condition
        rst_n=0; repeat(4) @(posedge clk); rst_n=1; @(posedge clk); A_pin=0; B_pin=0;
        rotate_cw(4,50);
        apply_state(0,0,50); apply_state(0,1,50); apply_state(1,1,50);
        A_pin=1; B_pin=0; repeat(30) @(posedge clk); #1;
        captured_base=base_theta_x10; captured_interp=theta_interp_x10;
        if (captured_interp > captured_base && captured_interp < (captured_base+9))
            $display("[TC-6c] PASS");
        repeat(110) @(posedge clk); #1;
        sc_base=base_theta_x10; sc_interp=theta_interp_x10;
        if (sc_interp==sc_base) $display("[TC-6d] PASS: stop condition correct");

        // TC-9: Full CSV log
        rst_n=0; repeat(4) @(posedge clk); rst_n=1; @(posedge clk);
        A_pin=0; B_pin=0; Z_pin=0; fault_clear=0; repeat(20) @(posedge clk);

        TC9_PI=3.14159265358979; TC9_A_ECC=0.20; TC9_A_GRAD=0.05;
        TC9_LATENCY_S=6.0/100000000.0;
        TC9_OMEGA_DPS=(60.0/(400.0*500.0*10.0e-9))*360.0/60.0;

        cw_a[0]=0; cw_b[0]=0; cw_a[1]=1; cw_b[1]=0;
        cw_a[2]=1; cw_b[2]=1; cw_a[3]=0; cw_b[3]=1;

        tc9_hold=500;
        // Prime 2 revolutions
        for(tc9_cnt=0; tc9_cnt<800; tc9_cnt=tc9_cnt+1) begin
            tc9_st=tc9_cnt%4; A_pin=cw_a[tc9_st]; B_pin=cw_b[tc9_st];
            for(tc9_i=0; tc9_i<tc9_hold; tc9_i=tc9_i+1) @(posedge clk);
        end
        A_pin=0; B_pin=0; repeat(10) @(posedge clk);
        @(posedge clk); Z_pin=1; @(posedge clk); Z_pin=0; repeat(20) @(posedge clk);
        A_pin=0; B_pin=0; repeat(50) @(posedge clk);

        fp=$fopen("encoder_log.csv","w");
        $fwrite(fp,"Count No.,Actual Angle (deg),VA (V),VB (V),State AB,");
        $fwrite(fp,"FPGA Count,theta_interp_x10,Measured Angle (deg),");
        $fwrite(fp,"base_theta_x10,T_period (cycles),T_phase (cycles),");
        $fwrite(fp,"Quant Error (deg),Latency Error (deg),Mech Error (deg),Total Error (deg)\n");

        for(tc9_cnt=0; tc9_cnt<800; tc9_cnt=tc9_cnt+1) begin
            tc9_st=tc9_cnt%4; tc9_a=cw_a[tc9_st]; tc9_b=cw_b[tc9_st];
            A_pin=tc9_a; B_pin=tc9_b;
            for(tc9_i=0; tc9_i<tc9_hold; tc9_i=tc9_i+1) @(posedge clk);
            #1;
            actual_deg   = tc9_cnt * 0.45;
            measured_deg = base_theta_x10 / 10.0;
            quant_err    = measured_deg - actual_deg;
            latency_err  = TC9_OMEGA_DPS * TC9_LATENCY_S;
            mech_err     = TC9_A_ECC*$sin(2.0*TC9_PI*actual_deg/360.0)
                         + TC9_A_GRAD*$sin(2.0*TC9_PI*actual_deg*100/360.0);
            total_err    = quant_err + latency_err + mech_err;
            $fwrite(fp,"%0d,%.2f,%.1f,%.1f,%0b%0b,",
                tc9_cnt+1,actual_deg,(tc9_a?3.3:0.0),(tc9_b?3.3:0.0),tc9_a,tc9_b);
            $fwrite(fp,"%0d,%0d,%.2f,",count,theta_interp_x10,measured_deg);
            $fwrite(fp,"%0d,%0d,%0d,",base_theta_x10,T_period,T_phase);
            $fwrite(fp,"%.6f,%.6f,%.6f,%.6f\n",quant_err,latency_err,mech_err,total_err);
        end
        $fclose(fp);
        $display("[TC-9] DONE — encoder_log.csv written");
        $display("All tests complete."); #100; $finish;
    end
endmodule
