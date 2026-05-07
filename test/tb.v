`timescale 1ns/1ps

module tb;
    reg clk, rst_n, ena;
    reg [7:0] ui_in, uio_in;
    wire [7:0] uo_out, uio_out, uio_oe;

    // Initialize all inputs to 0 at time zero to prevent GL X-propagation
    initial begin
        clk = 0;
        rst_n = 0;
        ena = 0;
        ui_in = 8'b0;
        uio_in = 8'b0;
        $dumpfile("tb.vcd");
        $dumpvars(0, tb);
    end

    // Drive power pins using wires for inout compatibility
`ifdef GL_TEST
    wire VPWR = 1'b1;
    wire VGND = 1'b0;
`endif

    // Instantiate the DUT with power pins attached for Gate-Level testing
    tt_um_wearlevel_controller dut (
    `ifdef GL_TEST
        .VPWR(VPWR),
        .VGND(VGND),
    `endif
        .ui_in(ui_in),
        .uo_out(uo_out),
        .uio_in(uio_in),
        .uio_out(uio_out),
        .uio_oe(uio_oe),
        .ena(ena),
        .clk(clk),
        .rst_n(rst_n)
    );

endmodule
