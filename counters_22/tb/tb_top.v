`timescale 1ns/1ns

`include "../src/dut.v"

module tb_top;

reg clk;
reg rst;
wire [3:0] count;

counter_4b dut (
    .clk(clk),
    .rst(rst),
    .count(count)
);

// Clock Generation
initial
begin
    clk = 0;
    forever #5 clk = ~clk;
end

initial
begin
    $monitor("Time=%0t | rst=%b | count=%b | Decimal=%d",
              $time, rst, count, count);

    rst = 1;
    #10;

    rst = 0;
    #160;

    rst = 1;
    #10;

    rst = 0;
    #50;

    $finish;
end


initial 
begin
	$dumpfile("waveform.vcd");
	$dumpvars(0, tb_top);
end


 $finish;

endmodule
