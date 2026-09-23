`timescale 1ns / 1ps
//=============================================================================
// encoder_emulator.v
// On-chip quadrature encoder emulator for Arty Z7 hardware validation.
// Generates A/B (Gray-coded, 4x decode) and Z (once-per-revolution index)
// signals so quad_encoder_top can be exercised on real hardware without a
// physical motor + encoder attached.
//
// Timing relation (same form as the T_period relation already used in
// position_engine.v / the report):
//   state_period_cycles = (60 * CLK_FREQ_HZ) / (RPM * COUNTS_PER_REV)
//
// Speed steps below span the TC-7 sweep (1-3000 RPM) and bracket the M/T
// hybrid crossover (~45 RPM). These are assumed sample points -- edit
// RPM_0..RPM_7 if your report's Stage 4 performance table uses different
// values so the emulator and the documented numbers line up exactly.
//=============================================================================
module encoder_emulator #(
    parameter integer CLK_FREQ_HZ = 100_000_000,
    parameter integer PPR         = 100
)(
    input  wire       clk,
    input  wire       rst_n,
    input  wire [2:0] speed_sel,   // see RPM table below
    input  wire       direction,   // 0 = CW, 1 = ACW
    output wire        A_emu,
    output wire        B_emu,
    output wire        Z_emu
);
    localparam integer COUNTS_PER_REV = PPR * 4;   // 400 for PPR = 100

    // speed_sel : RPM
    //     0     : 10    (deep T-method region)
    //     1     : 30
    //     2     : 45    (M/T crossover)
    //     3     : 100
    //     4     : 500
    //     5     : 1000
    //     6     : 2000
    //     7     : 3000  (top of TC-7 sweep)
    localparam integer RPM_0 = 10,   RPM_1 = 30,   RPM_2 = 45,   RPM_3 = 100;
    localparam integer RPM_4 = 500,  RPM_5 = 1000, RPM_6 = 2000, RPM_7 = 3000;

    // 64-bit intermediate avoids overflow: 60 * 100_000_000 = 6e9 > 2^32,
    // which silently wraps if computed at plain 32-bit width.
    localparam [31:0] DIV_0 = (64'd60 * CLK_FREQ_HZ) / (RPM_0 * COUNTS_PER_REV);
    localparam [31:0] DIV_1 = (64'd60 * CLK_FREQ_HZ) / (RPM_1 * COUNTS_PER_REV);
    localparam [31:0] DIV_2 = (64'd60 * CLK_FREQ_HZ) / (RPM_2 * COUNTS_PER_REV);
    localparam [31:0] DIV_3 = (64'd60 * CLK_FREQ_HZ) / (RPM_3 * COUNTS_PER_REV);
    localparam [31:0] DIV_4 = (64'd60 * CLK_FREQ_HZ) / (RPM_4 * COUNTS_PER_REV);
    localparam [31:0] DIV_5 = (64'd60 * CLK_FREQ_HZ) / (RPM_5 * COUNTS_PER_REV);
    localparam [31:0] DIV_6 = (64'd60 * CLK_FREQ_HZ) / (RPM_6 * COUNTS_PER_REV);
    localparam [31:0] DIV_7 = (64'd60 * CLK_FREQ_HZ) / (RPM_7 * COUNTS_PER_REV);

    reg [31:0] divider;
    always @(*) begin
        case (speed_sel)
            3'd0: divider = DIV_0;
            3'd1: divider = DIV_1;
            3'd2: divider = DIV_2;
            3'd3: divider = DIV_3;
            3'd4: divider = DIV_4;
            3'd5: divider = DIV_5;
            3'd6: divider = DIV_6;
            3'd7: divider = DIV_7;
            default: divider = DIV_3;
        endcase
    end

    // ---- Clock divider: one-cycle tick every `divider` cycles -------------
    reg [31:0] div_cnt;
    reg        tick;
    always @(posedge clk) begin
        if (!rst_n) begin
            div_cnt <= 32'd0;
            tick    <= 1'b0;
        end else if (div_cnt >= divider - 32'd1) begin
            div_cnt <= 32'd0;
            tick    <= 1'b1;
        end else begin
            div_cnt <= div_cnt + 32'd1;
            tick    <= 1'b0;
        end
    end

    // ---- 2-bit Gray state: 00->01->11->10->00 (CW), reverse for ACW -------
    // Matches the rotate_cw / rotate_acw ordering already used in
    // tb_quad_encoder.v, so this drop-in produces identical sequences to
    // the ones your simulation testbench already generates by hand.
    reg [1:0] gray_state;
    always @(posedge clk) begin
        if (!rst_n)      gray_state <= 2'd0;
        else if (tick)   gray_state <= direction ? (gray_state - 2'd1)
                                                   : (gray_state + 2'd1);
    end

    reg A_r, B_r;
    always @(*) begin
        case (gray_state)
            2'd0: {A_r, B_r} = 2'b00;
            2'd1: {A_r, B_r} = 2'b01;
            2'd2: {A_r, B_r} = 2'b11;
            default: {A_r, B_r} = 2'b10; // 2'd3
        endcase
    end
    assign A_emu = A_r;
    assign B_emu = B_r;

    // ---- Revolution counter -> Z_emu once every COUNTS_PER_REV ticks ------
    // Held high for the full dwell of state 0 on the home revolution, which
    // is far longer than the 2-cycle sync in quad_encoder_top needs to catch
    // a clean rising edge.
    reg [15:0] rev_cnt;   // 0 .. COUNTS_PER_REV-1
    always @(posedge clk) begin
        if (!rst_n) begin
            rev_cnt <= 16'd0;
        end else if (tick) begin
            if (direction) // ACW
                rev_cnt <= (rev_cnt == 16'd0) ? (COUNTS_PER_REV - 1) : (rev_cnt - 16'd1);
            else           // CW
                rev_cnt <= (rev_cnt == COUNTS_PER_REV - 1) ? 16'd0 : (rev_cnt + 16'd1);
        end
    end
    assign Z_emu = (rev_cnt == 16'd0);

endmodule
