module addsub8bit(
input[7:0] A,B, 
input CIN,
output[7:0] SUM,
output COUT
);

wire[7:0] xor_out;
wire[7:0] INT_CARRY;

xor X0(xor_out[0], B[0], CIN);
xor X1(xor_out[1], B[1], CIN);
xor X2(xor_out[2], B[2], CIN);
xor X3(xor_out[3], B[3], CIN);
xor X4(xor_out[4], B[4], CIN);
xor X5(xor_out[5], B[5], CIN);
xor X6(xor_out[6], B[6], CIN);
xor X7(xor_out[7], B[7], CIN);
full_adder FA0(A[0], xor_out[0], CIN, 	  		  SUM[0], INT_CARRY[0]);
full_adder FA1(A[1], xor_out[1], INT_CARRY[0], SUM[1], INT_CARRY[1]);
full_adder FA2(A[2], xor_out[2], INT_CARRY[1], SUM[2], INT_CARRY[2]);
full_adder FA3(A[3], xor_out[3], INT_CARRY[2], SUM[3], INT_CARRY[3]);
full_adder FA4(A[4], xor_out[4], INT_CARRY[3], SUM[4], INT_CARRY[4]);
full_adder FA5(A[5], xor_out[5], INT_CARRY[4], SUM[5], INT_CARRY[5]);
full_adder FA6(A[6], xor_out[6], INT_CARRY[5], SUM[6], INT_CARRY[6]);
full_adder FA7(A[7], xor_out[7], INT_CARRY[6], SUM[7], INT_CARRY[7]);

assign COUT = INT_CARRY[7];
endmodule

//
module full_adder(
  input A,
  input B,
  input CIN,
  output SUM,
  output COUT
);
assign {COUT, SUM} = A + B + CIN;
endmodule
