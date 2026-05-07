`timescale 1ns/1ps

module tb;
    reg clk, rst_n, ena;
    reg [7:0] ui_in, uio_in;
    wire [7:0] uo_out, uio_out, uio_oe;

    tt_um_wearlevel_controller dut (
        .ui_in(ui_in),
        .uo_out(uo_out),
        .uio_in(uio_in),
        .uio_out(uio_out),
        .uio_oe(uio_oe),
        .ena(ena),
        .clk(clk),
        .rst_n(rst_n)
    );

    initial begin
        $dumpfile("tb.vcd");
        $dumpvars(0, tb);
        clk = 0;
        forever #5 clk = ~clk;
    end
endmodule
