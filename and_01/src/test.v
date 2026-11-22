`timescale 1ns/1ns

module test(

    input  A,
    input  B,
    output reg Y_AND,
    output reg Y_OR,
    output reg Y_NAND,
    output reg Y_NOR,
    output reg Y_NOT,
    output reg Y_BUF,
    output reg Y_XOR,
    output reg Y_XNOR
);

always @(*) 
begin
   Y_AND  = A & B;
   Y_OR   = A | B;
   Y_NAND = ~(A & B);
   Y_NOR  = ~(A | B);
   Y_NOT  = ~A;
   Y_BUF  = A;
   Y_XOR  = A ^ B;
   Y_XNOR = ~(A ^ B);
end

endmodule

