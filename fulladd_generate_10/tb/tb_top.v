`timescale 1ns/1ns

`include "../src/dut.v"

module tb_top;

reg [7:0] A, B;
reg CIN;

wire [7:0] SUM;
wire COUT;

full_adder_8bit dut (
    .A(A),
    .B(B),
    .CIN(CIN),
    .SUM(SUM),
    .COUT(COUT)
);
 initial begin
    $dumpfile("waveform.vcd");
    $dumpvars(0, tb_top);
end
initial
begin
    $monitor("Time=%0t | A=%b | B=%b | CIN=%b | Sum=%b | cout=%b",
              $time, A, B, CIN, SUM, COUT);

    A = 8'b00000000;
    B = 8'b00000000;
    CIN = 0;
    #10;

    A = 8'b00001111;
    B = 8'b00000001;
    CIN = 0;
    #10;

    A = 8'b00001111;
    B = 8'b00000001;
    CIN = 1;
    #10;

    A = 8'b11111111;
    B = 8'b00000001;
    CIN = 0;
    #10;

    A = 8'b10101010;
    B = 8'b01010101;
    CIN = 1;
    #10;

    $finish;
end

endmodule
