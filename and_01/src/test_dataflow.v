module test_dataflow (
    input A,
    input B,

    output Y_AND,
    output Y_OR,
    output Y_NAND,
    output Y_NOR,
    output Y_NOT,
    output Y_BUF,
    output Y_XOR,
    output Y_XNOR
);

assign Y_AND  = A & B;   
assign Y_OR   = A | B;   
assign Y_NAND = ~(A & B); 
assign Y_NOR  = ~(A | B); 
assign Y_NOT  = ~A;       
assign Y_BUF  = A;        
assign Y_XOR  = A ^ B;  
assign Y_XNOR = ~(A ^ B); 

endmodule

