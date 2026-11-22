`timescale 1ns/1ns

`include "../src/test.v"

module tb_top;

reg din;
reg [1:0] sel;
wire [3:0] dout;

// DUT Instantiation
demux_1x4 dut (
    .din(din),
    .sel(sel),
    .dout(dout)
);

// Test stimulus
initial begin

    $dumpfile("waveform.vcd");
    $dumpvars(0, tb_top);

    $monitor("Time=%0t | DIN=%b | SEL=%b | DOUT=%b",
              $time, din, sel, dout);

    din = 1'b0;
    sel = 2'b00;
    #10;

    din = 1'b1;
    sel = 2'b00;
    #10;

    sel = 2'b01;
    #10;

    sel = 2'b10;
    #10;

    sel = 2'b11;
    #10;

    din = 1'b0;
    #10;

    $finish;

end

endmodule



