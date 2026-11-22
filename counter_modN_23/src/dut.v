`timescale 1ns/1ns

module counter_mod_N #(
parameter N=10 , parameter WIDTH=4)

(
input clk,
input rstn,
output reg [WIDTH-1:0] y
);

always@(posedge clk or negedge rstn) begin
	if(!rstn) begin
		y <=0;
	end
else if(y == N-1)
	y<=0;
else
	y<=y+1;
end
endmodule
