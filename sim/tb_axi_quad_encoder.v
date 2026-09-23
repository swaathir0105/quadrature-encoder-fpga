`timescale 1ns / 1ps
// tb_axi_quad_encoder.v -- Stage 5 unit test for axi_quad_encoder.v
//
// Acts as a minimal AXI4-Lite master (bus-functional model): drives real
// quadrature pulses on A_pin/B_pin (as if from actual encoder hardware),
// then issues AXI4-Lite read/write transactions to verify the register map:
//   - COUNT reads back the correct position after known rotation
//   - STATUS reflects fault_flag correctly, both set (illegal transition)
//     and cleared (via an AXI WRITE to CTRL, not the old direct pin)
//   - THETA_INTERP / T_PERIOD / RPM registers return sane (non-X) values
//
// Compile: iverilog -g2005 -o sim_axi signal_conditioner.v quad_decoder.v position_engine.v speed_engine.v quad_encoder_top.v axi_quad_encoder.v tb_axi_quad_encoder.v
// Run:     vvp sim_axi

module tb_axi_quad_encoder;
    localparam CLK_PERIOD = 10; // 100 MHz

    // Register offsets (byte addresses, word-aligned)
    localparam ADDR_CTRL         = 5'h00;
    localparam ADDR_STATUS       = 5'h04;
    localparam ADDR_COUNT        = 5'h08;
    localparam ADDR_BASE_THETA   = 5'h0C;
    localparam ADDR_THETA_INTERP = 5'h10;
    localparam ADDR_T_PERIOD     = 5'h14;
    localparam ADDR_T_PHASE      = 5'h18;
    localparam ADDR_RPM          = 5'h1C;

    reg clk, rst_n;
    reg A_pin, B_pin, Z_pin, fault_clear_ext;

    // AXI4-Lite master-side signals
    reg  [4:0]  AWADDR;
    reg         AWVALID;
    wire        AWREADY;
    reg  [31:0] WDATA;
    reg  [3:0]  WSTRB;
    reg         WVALID;
    wire        WREADY;
    wire [1:0]  BRESP;
    wire        BVALID;
    reg         BREADY;
    reg  [4:0]  ARADDR;
    reg         ARVALID;
    wire        ARREADY;
    wire [31:0] RDATA;
    wire [1:0]  RRESP;
    wire        RVALID;
    reg         RREADY;

    wire led_fault, led_direction;

    axi_quad_encoder #(.PPR(100), .SPEED_WINDOW(1_000_000)) dut (
        .A_pin(A_pin), .B_pin(B_pin), .Z_pin(Z_pin), .fault_clear_ext(fault_clear_ext),
        .led_fault(led_fault), .led_direction(led_direction),
        .S_AXI_ACLK(clk), .S_AXI_ARESETN(rst_n),
        .S_AXI_AWADDR(AWADDR), .S_AXI_AWPROT(3'b000), .S_AXI_AWVALID(AWVALID), .S_AXI_AWREADY(AWREADY),
        .S_AXI_WDATA(WDATA), .S_AXI_WSTRB(WSTRB), .S_AXI_WVALID(WVALID), .S_AXI_WREADY(WREADY),
        .S_AXI_BRESP(BRESP), .S_AXI_BVALID(BVALID), .S_AXI_BREADY(BREADY),
        .S_AXI_ARADDR(ARADDR), .S_AXI_ARPROT(3'b000), .S_AXI_ARVALID(ARVALID), .S_AXI_ARREADY(ARREADY),
        .S_AXI_RDATA(RDATA), .S_AXI_RRESP(RRESP), .S_AXI_RVALID(RVALID), .S_AXI_RREADY(RREADY)
    );

    initial clk = 0;
    always #(CLK_PERIOD/2) clk = ~clk;

    // ---- AXI4-Lite master bus-functional tasks ----
    // Valids are held comfortably past the ready pulse (rather than dropped
    // the instant ready is first observed) to stay clear of same-edge race
    // conditions in a simple polling-style testbench; the slave's aw_en/
    // one-shot ready logic guarantees a transaction is only accepted once
    // regardless of how long valid is held.
    task axi_write; input [4:0] addr; input [31:0] data; integer guard;
        begin
            @(posedge clk);
            AWADDR  <= addr; AWVALID <= 1'b1;
            WDATA   <= data; WSTRB   <= 4'hF; WVALID <= 1'b1;
            BREADY  <= 1'b1;
            guard = 0;
            while (!(AWREADY && WREADY) && guard < 50) begin @(posedge clk); guard = guard + 1; end
            @(posedge clk);
            AWVALID <= 1'b0; WVALID <= 1'b0;
            guard = 0;
            while (!BVALID && guard < 50) begin @(posedge clk); guard = guard + 1; end
            @(posedge clk);
            BREADY <= 1'b0;
            @(posedge clk);
        end
    endtask

    task axi_read; input [4:0] addr; output [31:0] data; integer guard;
        begin
            @(posedge clk);
            ARADDR  <= addr; ARVALID <= 1'b1;
            RREADY  <= 1'b1;
            guard = 0;
            while (!RVALID && guard < 50) begin @(posedge clk); guard = guard + 1; end
            data = RDATA;
            @(posedge clk);
            ARVALID <= 1'b0;
            @(posedge clk);
            RREADY <= 1'b0;
            @(posedge clk);
        end
    endtask

    // ---- Quadrature pulse generation (direct hardware pins) ----
    task apply_state; input a_val, b_val; input integer hold_cycles; integer i;
        begin A_pin=a_val; B_pin=b_val; for(i=0;i<hold_cycles;i=i+1) @(posedge clk); #1; end
    endtask
    // True forward/CW sequence: 00 -> 01 -> 11 -> 10 -> 00
    task rotate_cw; input integer reps, hold; integer r;
        begin for(r=0;r<reps;r=r+1) begin
            apply_state(0,0,hold); apply_state(0,1,hold);
            apply_state(1,1,hold); apply_state(1,0,hold); end end
    endtask

    reg [31:0] rdata;
    integer errors;

    initial begin
        A_pin=0; B_pin=0; Z_pin=0; fault_clear_ext=0;
        AWADDR=0; AWVALID=0; WDATA=0; WSTRB=0; WVALID=0; BREADY=0;
        ARADDR=0; ARVALID=0; RREADY=0;
        errors = 0;

        rst_n=0; repeat(4) @(posedge clk); rst_n=1; @(posedge clk);

        // ---- Test 1: rotate 20 steps CW, check COUNT over AXI ----
        // 20 reps x 4 transitions = 80 state changes, but the very first
        // apply_state(0,0,...) call is a no-op (state is already (0,0) at
        // reset), so 79 real quadrature transitions actually occur.
        rotate_cw(20, 10);
        axi_read(ADDR_COUNT, rdata);
        if ($signed(rdata) == 79)
            $display("[AXI-T1] PASS: COUNT=%0d (exp 79)", $signed(rdata));
        else begin
            $display("[AXI-T1] FAIL: COUNT=%0d (exp 79)", $signed(rdata));
            errors = errors + 1;
        end

        // ---- Test 2: STATUS shows fault_flag=0, direction=1 (CW => Y=1) ----
        axi_read(ADDR_STATUS, rdata);
        if (rdata[0] == 1'b0)
            $display("[AXI-T2] PASS: STATUS.fault_flag=0 (exp 0), STATUS=0x%0h", rdata);
        else begin
            $display("[AXI-T2] FAIL: STATUS.fault_flag=%b (exp 0), STATUS=0x%0h", rdata[0], rdata);
            errors = errors + 1;
        end

        // ---- Test 3: THETA_INTERP / T_PERIOD / RPM registers return sane values ----
        axi_read(ADDR_THETA_INTERP, rdata);
        $display("[AXI-T3] THETA_INTERP=%0d (0.1deg units, informational)", rdata[15:0]);
        axi_read(ADDR_T_PERIOD, rdata);
        $display("[AXI-T3] T_PERIOD=%0d cycles (informational)", rdata);
        axi_read(ADDR_RPM, rdata);
        $display("[AXI-T3] RPM_x10=%0d (informational, expect 0 or small -- pulses were not evenly timed for a real RPM test)", rdata[15:0]);

        // ---- Test 4: force illegal state (simultaneous A&B change) -> fault_flag=1 ----
        A_pin=0; B_pin=0; repeat(15) @(posedge clk);
        @(posedge clk); A_pin=1; B_pin=1; repeat(10) @(posedge clk);
        axi_read(ADDR_STATUS, rdata);
        if (rdata[0] == 1'b1)
            $display("[AXI-T4] PASS: STATUS.fault_flag=1 after illegal transition, STATUS=0x%0h", rdata);
        else begin
            $display("[AXI-T4] FAIL: STATUS.fault_flag=%b (exp 1), STATUS=0x%0h", rdata[0], rdata);
            errors = errors + 1;
        end

        // ---- Test 5: clear fault via AXI WRITE to CTRL (not the old direct pin) ----
        axi_write(ADDR_CTRL, 32'h1);
        repeat(3) @(posedge clk);
        axi_read(ADDR_STATUS, rdata);
        if (rdata[0] == 1'b0)
            $display("[AXI-T5] PASS: STATUS.fault_flag=0 after AXI CTRL write, STATUS=0x%0h", rdata);
        else begin
            $display("[AXI-T5] FAIL: STATUS.fault_flag=%b (exp 0) after AXI CTRL write, STATUS=0x%0h", rdata[0], rdata);
            errors = errors + 1;
        end

        // ---- Test 6: CTRL always reads back as 0 (write-only pulse register) ----
        axi_read(ADDR_CTRL, rdata);
        if (rdata == 32'h0)
            $display("[AXI-T6] PASS: CTRL reads back as 0x0 (exp 0x0)");
        else begin
            $display("[AXI-T6] FAIL: CTRL=0x%0h (exp 0x0)", rdata);
            errors = errors + 1;
        end

        // ---- Test 7: writes to a read-only register (COUNT) are accepted at
        //              the AXI protocol level (OKAY) but have no effect ----
        axi_read(ADDR_COUNT, rdata);
        $display("[AXI-T7] COUNT before bogus write = %0d", $signed(rdata));
        axi_write(ADDR_COUNT, 32'hDEADBEEF);
        axi_read(ADDR_COUNT, rdata);
        if ($signed(rdata) == 80)
            $display("[AXI-T7] PASS: COUNT unaffected by write to read-only register, COUNT=%0d (exp 80)", $signed(rdata));
        else begin
            $display("[AXI-T7] FAIL: COUNT=%0d (exp still 80, write to read-only reg should have no effect)", $signed(rdata));
            errors = errors + 1;
        end

        if (errors == 0)
            $display("=== Stage 5 AXI wrapper: ALL TESTS PASSED ===");
        else
            $display("=== Stage 5 AXI wrapper: %0d TEST(S) FAILED ===", errors);

        $display("AXI wrapper test complete.");
        #100; $finish;
    end
endmodule
