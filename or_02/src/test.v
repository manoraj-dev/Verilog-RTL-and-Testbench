`timescale 1ns/1ns

module NAND4 (
 output Y,
 input A,
 input B,
 input C,
 input D
);

//reg Y;
wire int_a, int_b;

//instantiate primitives

nand u0(int_a, A, B);
nand u1(int_b, C, D);
or   u3(Y, int_a, int_b);

endmodule
