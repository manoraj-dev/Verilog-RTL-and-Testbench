`timescale 1ns/1ns

module shiftregister_3b(
output reg[2:0] y,
 input wire clk,
 input wire rst_n,
 input wire serial_in
);

always @(posedge clk or negedge rst_n)
begin
if(!rst_n)
begin
			  y=3'b000;
end
else
begin
//y <= {y[1:0], serial_in};
		  y[0] <= serial_in;
		  y[1] <=y[0];
		  y[2] <=y[1];
end
end

endmodule
