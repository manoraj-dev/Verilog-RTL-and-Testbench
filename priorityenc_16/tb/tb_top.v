`timescale 1ns/1ns

`include "../src/test.v"

module tb_top;

reg A, B, C,D;

wire VALID;
wire Y1, Y0;

priority_encoder dut (.A(A),.B(B),.C(C),.D(D),.VALID(VALID),.Y1(Y1),.Y0(Y0));

initial begin

    $dumpfile("waveform.vcd");
    $dumpvars(0, tb_top);

    $monitor("A=%b B=%b C=%b D=%b VALID=%b| Y1=%b Y0=%b",A, B, C,D,VALID, Y1, Y0);

    $random();
    rand=();
    $finish;
end

endmodule



