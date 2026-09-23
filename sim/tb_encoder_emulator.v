`timescale 1ns / 1ps
// Compile: iverilog -g2005 -o sim_emu encoder_emulator.v tb_encoder_emulator.v
// Run:     vvp sim_emu
// Output:  encoder_emulator.vcd (view in GTKWave)

module tb_encoder_emulator;
    localparam CLK_FREQ_HZ = 100_000_000, PPR = 100;

    reg clk, rst_n;
    reg [2:0] speed_sel;
    reg direction;
    wire A_emu, B_emu, Z_emu;

    encoder_emulator #(.CLK_FREQ_HZ(CLK_FREQ_HZ), .PPR(PPR)) dut (
        .clk(clk), .rst_n(rst_n),
        .speed_sel(speed_sel), .direction(direction),
        .A_emu(A_emu), .B_emu(B_emu), .Z_emu(Z_emu)
    );

    initial clk = 0;
    always #5 clk = ~clk;   // 100 MHz

    initial begin
        $dumpfile("encoder_emulator.vcd");
        $dumpvars(0, tb_encoder_emulator);
    end

    reg [1:0] prev_ab, cur_ab;
    integer   cycle_cnt, i, errors, z_hits;
    reg [1:0] expect_seq [0:3];

    task wait_for_transition;
        begin
            prev_ab = {A_emu, B_emu};
            cur_ab  = prev_ab;
            while (cur_ab == prev_ab) begin
                @(posedge clk);
                cur_ab = {A_emu, B_emu};
            end
        end
    endtask

    initial begin
        rst_n = 0; speed_sel = 3'd7; direction = 1'b0;  // 3000 RPM, CW -> fastest, quick sim
        repeat (4) @(posedge clk);
        rst_n = 1;

        // TC-E1: CW order 00 -> 01 -> 11 -> 10 -> 00
        expect_seq[0]=2'b01; expect_seq[1]=2'b11; expect_seq[2]=2'b10; expect_seq[3]=2'b00;
        errors = 0;
        for (i = 0; i < 4; i = i + 1) begin
            wait_for_transition;
            if (cur_ab !== expect_seq[i]) begin
                $display("[TC-E1] FAIL step %0d: expected %b got %b", i, expect_seq[i], cur_ab);
                errors = errors + 1;
            end
        end
        if (errors == 0) $display("[TC-E1] PASS: CW sequence correct");

        // TC-E2: ACW reversal 10 -> 11 -> 01 -> 00
        direction = 1'b1;
        expect_seq[0]=2'b10; expect_seq[1]=2'b11; expect_seq[2]=2'b01; expect_seq[3]=2'b00;
        errors = 0;
        for (i = 0; i < 4; i = i + 1) begin
            wait_for_transition;
            if (cur_ab !== expect_seq[i]) begin
                $display("[TC-E2] FAIL step %0d: expected %b got %b", i, expect_seq[i], cur_ab);
                errors = errors + 1;
            end
        end
        if (errors == 0) $display("[TC-E2] PASS: ACW reversal correct");

        // TC-E3: divider timing == 5000 cycles at speed_sel=7 (3000 RPM)
        direction = 1'b0;
        wait_for_transition;               // sync to a transition boundary first
        cycle_cnt = 0;
        prev_ab = cur_ab;
        while ({A_emu, B_emu} == prev_ab) begin
            @(posedge clk);
            cycle_cnt = cycle_cnt + 1;
        end
        if (cycle_cnt == 5000)
            $display("[TC-E3] PASS: divider = %0d cycles (exp 5000 @ 3000 RPM)", cycle_cnt);
        else
            $display("[TC-E3] FAIL: divider = %0d cycles (exp 5000)", cycle_cnt);

        // TC-E4: Z_emu asserted exactly once per 400 (COUNTS_PER_REV) transitions
        rst_n = 0; repeat (4) @(posedge clk); rst_n = 1; direction = 1'b0;
        z_hits = 0;
        for (i = 0; i < 401; i = i + 1) begin
            wait_for_transition;
            if (Z_emu) z_hits = z_hits + 1;
        end
        if (z_hits == 1)
            $display("[TC-E4] PASS: Z_emu asserted exactly once in 401 transitions");
        else
            $display("[TC-E4] FAIL: Z_emu asserted %0d times (exp 1)", z_hits);

        $display("All encoder_emulator tests complete.");
        #100; $finish;
    end
endmodule
