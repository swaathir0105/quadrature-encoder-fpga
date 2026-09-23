`timescale 1ns / 1ps
module quad_encoder_top #(
    parameter PPR          = 100,
    parameter SPEED_WINDOW = 1_000_000   // fixed: was 1000, needs to be 1,000,000 for a 10 ms window @ 100 MHz
)(
    input  wire        clk,
    input  wire        rst_n,
    input  wire        A_pin,
    input  wire        B_pin,
    input  wire        Z_pin,
    input  wire        fault_clear,
    output wire signed [31:0] count,
    output wire        [15:0] theta_interp_x10,
    output wire        [15:0] base_theta_x10,
    output wire        [31:0] T_period,
    output wire        [31:0] T_phase,
    output wire        [15:0] rpm_x10,        // was: output reg [7:0] omega (placeholder, always 0)
    output wire               method_sel,     // new: 0 = T-method, 1 = F-method
    output wire               speed_stopped,  // new: 1 = shaft stationary this window
    output reg                direction,
    output wire               fault_flag,
    output wire               dbg_valid    // TEMP DEBUG: raw pulse from quad_decoder.
                                             // Exposed so we can tap it further up the
                                             // hierarchy without hierarchical references.
);
    wire A_clean, B_clean;
    signal_conditioner sc_A (.clk(clk),.rst_n(rst_n),.raw_in(A_pin),.filtered_out(A_clean));
    signal_conditioner sc_B (.clk(clk),.rst_n(rst_n),.raw_in(B_pin),.filtered_out(B_clean));

    reg Z_sync1, Z_sync2, Z_prev;
    always @(posedge clk) begin
        if (!rst_n) begin Z_sync1<=0; Z_sync2<=0; Z_prev<=0; end
        else begin Z_sync1<=Z_pin; Z_sync2<=Z_sync1; Z_prev<=Z_sync2; end
    end
    wire Z_clean = Z_sync2;
    wire Z_rise  = Z_clean & ~Z_prev;

    wire Y, valid;
    quad_decoder u_decoder (
        .clk(clk),.rst_n(rst_n),.A(A_clean),.B(B_clean),
        .fault_clear(fault_clear),.Y(Y),.valid(valid),.fault_flag(fault_flag)
    );
    assign dbg_valid = valid;

    position_engine #(.PPR(PPR)) u_pos (
        .clk(clk),.rst_n(rst_n),.valid(valid),.Y(Y),.Z_rise(Z_rise),
        .count(count),.base_theta_x10(base_theta_x10),
        .theta_interp_x10(theta_interp_x10),.T_period(T_period),.T_phase(T_phase)
    );

    speed_engine #(.PPR(PPR), .WINDOW_CYCLES(SPEED_WINDOW)) u_speed (
        .clk(clk),.rst_n(rst_n),.valid(valid),
        .T_period(T_period),.T_phase(T_phase),
        .rpm_x10(rpm_x10),.method_sel(method_sel),.speed_stopped(speed_stopped)
    );

    always @(posedge clk) begin
        if (!rst_n) direction <= 1'b0;
        else if (valid) direction <= Y;
    end
endmodule
