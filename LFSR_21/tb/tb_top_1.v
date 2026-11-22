`timescale 1ns/1ns

`include "../src/dut_1.v"

module tb_top_1;

    reg clk;
    reg arst_n;

    wire [7:0] q;

    galois_lfsr_8b dut (.clk(clk),.arst_n(arst_n),.q(q));

    initial
    begin
        clk = 0;
        forever #5 clk = ~clk;
    end
    initial
    begin
        $dumpfile("galois_8bit.vcd");
        $dumpvars(0, tb_top_1);
    end
    initial
    begin
        arst_n = 0;

        #12;
        arst_n = 1;

        #88;
        $finish;
    end
    initial
    begin
        $monitor("Time=%0t | CLK=%b | RESET=%b | Q=%b",$time, clk, arst_n, q);
    end
endmodule


