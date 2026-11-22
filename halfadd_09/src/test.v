`timescale 1ns/1ns

module halfadder_b (
 output reg sum,
 output reg carry,
 input A,
 input B,
);

always @(*)
 begin
case({A,B})
	2'b00:begin
	sum=0;
	carry=0;
end
	2'b01:begin
	sum=1;
	carry=0;
end
2'b10:begin
  sum=1;
  carry=0;
end
2'b11:begin
  sum=0;
  carry=1;
end 
endcase
end 
endmodule
