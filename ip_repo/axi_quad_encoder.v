`timescale 1ns / 1ps
//=============================================================================
// axi_quad_encoder.v
// Stage 5 -- AXI4-Lite slave wrapper around quad_encoder_top, for memory-
// mapped access from the ARM Cortex-A9 (PS) on the Arty Z7 (Zynq-7000).
//
// Single clock/reset domain: S_AXI_ACLK drives both the AXI slave logic and
// the encoder pipeline directly (no CDC), and S_AXI_ARESETN (active-low) is
// used directly as the encoder's rst_n. This matches the existing design's
// single 100 MHz clock assumption, and mirrors how a Zynq PS-to-PL AXI-Lite
// peripheral is normally clocked from one shared FCLK/reset pair.
//
// Register map (word-aligned, C_S_AXI_ADDR_WIDTH=5 -> 8 x 32-bit registers):
//   0x00  CTRL         [W]  bit0: fault_clear (write 1 to pulse for 1 cycle,
//                                  self-clearing -- reads back as 0)
//   0x04  STATUS       [R]  bit0: fault_flag
//                            bit1: direction      (1=ACW, 0=CW)
//                            bit2: method_sel     (1=F-method, 0=T-method)
//                            bit3: speed_stopped
//   0x08  COUNT        [R]  signed 32-bit position count
//   0x0C  BASE_THETA   [R]  [15:0] base_theta_x10 (angle at last pulse, x10 deg)
//   0x10  THETA_INTERP [R]  [15:0] theta_interp_x10 (interpolated angle, x10 deg)
//   0x14  T_PERIOD     [R]  inter-pulse period, clock cycles
//   0x18  T_PHASE      [R]  cycles since last pulse
//   0x1C  RPM          [R]  [15:0] rpm_x10 (speed magnitude, RPM x10)
//
// All registers other than CTRL are hardware-driven (read-only); writes to
// them still complete the AXI handshake with an OKAY response but have no
// effect, consistent with common AXI4-Lite peripheral practice.
//
// A_pin/B_pin/Z_pin remain direct top-level pins (real encoder hardware
// signals, not software-controlled) and are unchanged from quad_encoder_top.
// fault_clear_ext is an optional external pin, OR'd with the AXI-triggered
// pulse, for board-level test/debug flexibility; tie low if unused.
//=============================================================================

module axi_quad_encoder #(
    parameter PPR                        = 100,
    parameter SPEED_WINDOW               = 1_000_000,
    parameter integer C_S_AXI_DATA_WIDTH = 32,
    parameter integer C_S_AXI_ADDR_WIDTH = 5
)(
    // ---- Encoder physical pins ----
    input  wire        A_pin,
    input  wire        B_pin,
    input  wire        Z_pin,
    input  wire        fault_clear_ext,   // optional external fault-clear pin; tie 0 if unused

    // ---- Board status LEDs (optional, e.g. Arty Z7 onboard LEDs) ----
    output wire         led_fault,
    output wire         led_direction,

    // ---- AXI4-Lite slave interface ----
    input  wire                              S_AXI_ACLK,
    input  wire                              S_AXI_ARESETN,
    input  wire [C_S_AXI_ADDR_WIDTH-1:0]     S_AXI_AWADDR,
    input  wire [2:0]                        S_AXI_AWPROT,
    input  wire                              S_AXI_AWVALID,
    output reg                               S_AXI_AWREADY,
    input  wire [C_S_AXI_DATA_WIDTH-1:0]     S_AXI_WDATA,
    input  wire [(C_S_AXI_DATA_WIDTH/8)-1:0] S_AXI_WSTRB,
    input  wire                              S_AXI_WVALID,
    output reg                               S_AXI_WREADY,
    output reg  [1:0]                        S_AXI_BRESP,
    output reg                               S_AXI_BVALID,
    input  wire                              S_AXI_BREADY,
    input  wire [C_S_AXI_ADDR_WIDTH-1:0]     S_AXI_ARADDR,
    input  wire [2:0]                        S_AXI_ARPROT,
    input  wire                              S_AXI_ARVALID,
    output reg                               S_AXI_ARREADY,
    output wire [C_S_AXI_DATA_WIDTH-1:0]     S_AXI_RDATA,
    output reg  [1:0]                        S_AXI_RRESP,
    output reg                               S_AXI_RVALID,
    input  wire                              S_AXI_RREADY
);

    // ------------------------------------------------------------------
    // Single clock/reset domain shared with the encoder pipeline
    // ------------------------------------------------------------------
    wire clk   = S_AXI_ACLK;
    wire rst_n = S_AXI_ARESETN;

    // ------------------------------------------------------------------
    // Encoder core instantiation (unchanged interface from Stage 4)
    // ------------------------------------------------------------------
    wire signed [31:0] count;
    wire        [15:0] theta_interp_x10, base_theta_x10;
    wire        [31:0] T_period, T_phase;
    wire        [15:0] rpm_x10;
    wire                method_sel, speed_stopped;
    wire                direction, fault_flag;

    reg  fault_clear_pulse;   // 1-cycle software-triggered pulse, see write logic below
    wire fault_clear = fault_clear_ext | fault_clear_pulse;

    wire dbg_emu_alive;    // TEMP DEBUG: see quad_encoder_top_emu.v
    wire dbg_valid_seen;   // TEMP DEBUG: see quad_encoder_top_emu.v

    quad_encoder_top_emu #(.PPR(PPR), .SPEED_WINDOW(SPEED_WINDOW)) u_encoder (
        .clk(clk), .rst_n(rst_n),
        .fault_clear(fault_clear),
        .count(count),
        .theta_interp_x10(theta_interp_x10), .base_theta_x10(base_theta_x10),
        .T_period(T_period), .T_phase(T_phase),
        .rpm_x10(rpm_x10), .method_sel(method_sel), .speed_stopped(speed_stopped),
        .direction(direction), .fault_flag(fault_flag),
        .dbg_emu_alive(dbg_emu_alive),
        .dbg_valid_seen(dbg_valid_seen)
    );

    // TEMP DEBUG: led_fault is repurposed to show dbg_valid_seen instead of real
    // fault_flag, so we can see -- LED OFF forever = quad_decoder never asserts
    // valid on real hardware (signal_conditioner or quad_decoder is the break);
    // LED ON = valid does fire, problem is further downstream (position_engine
    // or the AXI read path). Confirmed safe to reuse: fault_flag has been
    // observed to read 0 continuously on real hardware, never latching.
    // Revert to `assign led_fault = fault_flag;` once root cause is found.
    assign led_fault     = dbg_valid_seen;
    // TEMP DEBUG: led_direction is repurposed to show dbg_emu_alive instead of
    // real direction, so we can see on the board -- no tools, no license needed --
    // whether the emulator has EVER produced an A_emu edge since reset.
    // LED OFF forever  -> emulator genuinely never toggles on real hardware.
    // LED turns ON and STAYS ON -> it does toggle; problem is further downstream.
    // Revert to `assign led_direction = direction;` once root cause is found.
    assign led_direction = dbg_emu_alive;

    // ------------------------------------------------------------------
    // AXI4-Lite write channel: address + data handshake
    // (standard aw_en-gated pattern: prevents latching a new address until
    //  the previous write's response has been accepted)
    // ------------------------------------------------------------------
    reg [C_S_AXI_ADDR_WIDTH-1:0] axi_awaddr;
    reg                          aw_en;

    always @(posedge clk) begin
        if (!rst_n) begin
            S_AXI_AWREADY <= 1'b0;
            axi_awaddr    <= {C_S_AXI_ADDR_WIDTH{1'b0}};
            aw_en         <= 1'b1;
        end else begin
            if (~S_AXI_AWREADY && S_AXI_AWVALID && S_AXI_WVALID && aw_en) begin
                S_AXI_AWREADY <= 1'b1;
                axi_awaddr    <= S_AXI_AWADDR;
                aw_en         <= 1'b0;
            end else if (S_AXI_BVALID && S_AXI_BREADY) begin
                aw_en         <= 1'b1;
                S_AXI_AWREADY <= 1'b0;
            end else begin
                S_AXI_AWREADY <= 1'b0;
            end
        end
    end

    always @(posedge clk) begin
        if (!rst_n)
            S_AXI_WREADY <= 1'b0;
        else if (~S_AXI_WREADY && S_AXI_WVALID && S_AXI_AWVALID && aw_en)
            S_AXI_WREADY <= 1'b1;
        else
            S_AXI_WREADY <= 1'b0;
    end

    wire slv_reg_wren = S_AXI_WREADY && S_AXI_WVALID && S_AXI_AWREADY && S_AXI_AWVALID;

    // ------------------------------------------------------------------
    // CTRL register write -> fault_clear pulse generation
    //
    // fault_clear is a "write-1-to-pulse" bit: writing CTRL[0]=1 produces
    // exactly one clock cycle of fault_clear, then self-clears. There is
    // no stored CTRL state to read back (reads of 0x00 return 0).
    // ------------------------------------------------------------------
    always @(posedge clk) begin
        if (!rst_n) begin
            fault_clear_pulse <= 1'b0;
        end else if (slv_reg_wren && axi_awaddr[C_S_AXI_ADDR_WIDTH-1:2] == 3'h0
                     && S_AXI_WSTRB[0]) begin
            fault_clear_pulse <= S_AXI_WDATA[0];
        end else begin
            fault_clear_pulse <= 1'b0;   // self-clear every other cycle
        end
    end

    // ------------------------------------------------------------------
    // Write response channel
    // ------------------------------------------------------------------
    always @(posedge clk) begin
        if (!rst_n) begin
            S_AXI_BVALID <= 1'b0;
            S_AXI_BRESP  <= 2'b00;
        end else if (slv_reg_wren && ~S_AXI_BVALID) begin
            S_AXI_BVALID <= 1'b1;
            S_AXI_BRESP  <= 2'b00;   // OKAY
        end else if (S_AXI_BVALID && S_AXI_BREADY) begin
            S_AXI_BVALID <= 1'b0;
        end
    end

    // ------------------------------------------------------------------
    // AXI4-Lite read channel: address handshake
    // ------------------------------------------------------------------
    reg [C_S_AXI_ADDR_WIDTH-1:0] axi_araddr;

    always @(posedge clk) begin
        if (!rst_n) begin
            S_AXI_ARREADY <= 1'b0;
            axi_araddr    <= {C_S_AXI_ADDR_WIDTH{1'b0}};
        end else if (~S_AXI_ARREADY && S_AXI_ARVALID) begin
            S_AXI_ARREADY <= 1'b1;
            axi_araddr    <= S_AXI_ARADDR;
        end else begin
            S_AXI_ARREADY <= 1'b0;
        end
    end

    always @(posedge clk) begin
        if (!rst_n) begin
            S_AXI_RVALID <= 1'b0;
            S_AXI_RRESP  <= 2'b00;
        end else if (S_AXI_ARREADY && S_AXI_ARVALID && ~S_AXI_RVALID) begin
            S_AXI_RVALID <= 1'b1;
            S_AXI_RRESP  <= 2'b00;   // OKAY
        end else if (S_AXI_RVALID && S_AXI_RREADY) begin
            S_AXI_RVALID <= 1'b0;
        end
    end

    // ------------------------------------------------------------------
    // Read data mux (combinational off the registered read address) +
    // RDATA as a plain continuous assign, per standard AXI4-Lite slave
    // practice -- RDATA tracks reg_rdata directly, and is already settled
    // to the correct value by the time RVALID asserts one cycle later.
    // ------------------------------------------------------------------
    reg [C_S_AXI_DATA_WIDTH-1:0] reg_rdata;
    always @(*) begin
        case (axi_araddr[C_S_AXI_ADDR_WIDTH-1:2])
            3'h0: reg_rdata = 32'h0;                                    // CTRL (write-only, reads as 0)
            3'h1: reg_rdata = {28'h0, speed_stopped, method_sel,
                                direction, fault_flag};                 // STATUS
            3'h2: reg_rdata = count;                                    // COUNT (signed, full 32 bits)
            3'h3: reg_rdata = {16'h0, base_theta_x10};                  // BASE_THETA
            3'h4: reg_rdata = {16'h0, theta_interp_x10};                // THETA_INTERP
            3'h5: reg_rdata = T_period;                                 // T_PERIOD
            3'h6: reg_rdata = T_phase;                                  // T_PHASE
            3'h7: reg_rdata = {16'h0, rpm_x10};                         // RPM
            default: reg_rdata = 32'h0;
        endcase
    end

    assign S_AXI_RDATA = reg_rdata;

endmodule
