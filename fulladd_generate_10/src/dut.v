module full_adder_8bit (
    input  [7:0] A, B,
    input  CIN,
    output [7:0] SUM,
    output COUT
);

wire [8:0] CARRY;

assign CARRY[0] = CIN;
assign COUT = CARRY[8];

genvar i;
generate
    for (i = 0; i < 8; i = i + 1)
    begin
        full_adder FA[i](
            .A(A[i]),
            .B(B[i]),
            .CIN(CARRY[i]),
            .SUM(SUM[i]),
            .COUT(CARRY[i+1])
        );
    end
endgenerate

endmodule

//full_adder definition
module full_adder(
input A,
input B,
input CIN,
output SUM,
output COUT
);

//Dataflow
assign {COUT, SUM} = A + B + CIN;

endmodule
