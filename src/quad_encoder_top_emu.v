`timescale 1ns / 1ps
//=============================================================================
// quad_encoder_top_emu.v
// Emulator-driven top level for hardware bring-up testing.
// Hardcodes encoder_emulator's speed_sel/direction to a fixed, safe test
// value so COUNT/RPM/DIR change on real hardware with zero new pin wiring.
//
// Fixed test point: speed_sel = 3'd3 (100 RPM per encoder_emulator.v's
// table), direction = 0 (CW). Change EMU_SPEED_SEL / EMU_DIRECTION below
// and rebuild if you want a different fixed point later.
//=============================================================================
module quad_encoder_top_emu #(
    parameter PPR          = 100,
    parameter SPEED_WINDOW = 1_000_000
)(
    input  wire        clk,
    input  wire        rst_n,
    input  wire        fault_clear,     // still external -- wire to fault_clear_ext as before

    output wire signed [31:0] count,
    output wire        [15:0] theta_interp_x10,
    output wire        [15:0] base_theta_x10,
    output wire        [31:0] T_period,
    output wire        [31:0] T_phase,
    output wire        [15:0] rpm_x10,
    output wire               method_sel,
    output wire               speed_stopped,
    output wire                direction,
    output wire               fault_flag,
    output wire               dbg_emu_alive,  // TEMP DEBUG: latches HIGH forever the first
                                                // time A_emu ever toggles. Wired to an LED via
                                                // axi_quad_encoder.v so we can see, with zero
                                                // tools, whether the emulator produces any
                                                // edges at all on real hardware. Remove once
                                                // root cause is found.
    output wire               dbg_valid_seen  // TEMP DEBUG: latches HIGH forever the first
                                                // time quad_decoder ever asserts valid. Tells us
                                                // whether an edge survives signal_conditioner +
                                                // quad_decoder on real hardware. Remove once
                                                // root cause is found.
);
    // ---- Hardcoded emulator drive values ----
    localparam [2:0] EMU_SPEED_SEL  = 3'd3;   // 100 RPM (see encoder_emulator.v table)
    localparam       EMU_DIRECTION  = 1'b0;   // 0 = CW

    wire A_emu, B_emu, Z_emu;
    wire dbg_valid;

    encoder_emulator #(.CLK_FREQ_HZ(100_000_000), .PPR(PPR)) u_emu (
        .clk(clk), .rst_n(rst_n),
        .speed_sel(EMU_SPEED_SEL), .direction(EMU_DIRECTION),
        .A_emu(A_emu), .B_emu(B_emu), .Z_emu(Z_emu)
    );

    // ---- TEMP DEBUG: visible "emulator repeatedly toggling" blink ----
    // Watches BOTH A_emu and B_emu -- the first CW transition (00->01) only
    // moves B_emu, so watching A_emu alone would miss it.
    // At 100 RPM the emulator produces ~666 transitions/sec if genuinely
    // running continuously. Toggling the LED once every 666 transitions
    // gives a ~1 Hz blink if it's truly ongoing. If it stalls after only a
    // handful of transitions (like the earlier one-shot sticky latch
    // suggested), the LED will flip AT MOST once and then freeze -- clearly
    // different from a steady ~1 Hz blink.
    localparam integer BLINK_DIVISOR = 666;
    reg [1:0] AB_emu_prev;
    reg [15:0] transition_count;
    reg emu_alive;
    always @(posedge clk) begin
        if (!rst_n) begin
            AB_emu_prev      <= 2'b00;
            transition_count <= 16'd0;
            emu_alive        <= 1'b0;
        end else begin
            AB_emu_prev <= {A_emu, B_emu};
            if ({A_emu, B_emu} != AB_emu_prev) begin
                if (transition_count + 16'd1 >= BLINK_DIVISOR) begin
                    transition_count <= 16'd0;
                    emu_alive        <= ~emu_alive;   // one blink-edge per BLINK_DIVISOR
                                                         // transitions -- steady ~1Hz IF
                                                         // genuinely ongoing at 100 RPM
                end else begin
                    transition_count <= transition_count + 16'd1;
                end
            end
        end
    end
    assign dbg_emu_alive = emu_alive;

    // ---- TEMP DEBUG: visible "valid repeatedly firing" blink ----
    // At 100 RPM, quad_decoder should assert valid ~666 times/sec if the
    // pipeline is genuinely running continuously (matching the emulator's
    // confirmed transition rate). Blinking once every 666 valid pulses
    // gives a ~1Hz blink IF valid keeps firing at the same steady rate as
    // A_emu/B_emu now proven to. If valid only fired once early on and
    // then stopped (even though A/B keep toggling), this LED will flip at
    // most once and freeze -- clearly different from a steady ~1Hz blink.
    localparam integer VALID_BLINK_DIVISOR = 666;
    reg [15:0] valid_count;
    reg valid_seen;
    always @(posedge clk) begin
        if (!rst_n) begin
            valid_count <= 16'd0;
            valid_seen  <= 1'b0;
        end else if (dbg_valid) begin
            if (valid_count + 16'd1 >= VALID_BLINK_DIVISOR) begin
                valid_count <= 16'd0;
                valid_seen  <= ~valid_seen;
            end else begin
                valid_count <= valid_count + 16'd1;
            end
        end
    end
    assign dbg_valid_seen = valid_seen;

    quad_encoder_top #(.PPR(PPR), .SPEED_WINDOW(SPEED_WINDOW)) u_top (
        .clk(clk), .rst_n(rst_n),
        .A_pin(A_emu), .B_pin(B_emu), .Z_pin(Z_emu),
        .fault_clear(fault_clear),
        .count(count), .theta_interp_x10(theta_interp_x10),
        .base_theta_x10(base_theta_x10),
        .T_period(T_period), .T_phase(T_phase),
        .rpm_x10(rpm_x10), .method_sel(method_sel),
        .speed_stopped(speed_stopped),
        .direction(direction), .fault_flag(fault_flag),
        .dbg_valid(dbg_valid)
    );
endmodule
