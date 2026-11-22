`timescale 1ns/1ns

module RAM_64KB #(
	parameter ADDRWIDTH=16,
	parameter WORDSIZE=32,
	parameter DEPTH=1024
) (
    input clk,
	 input wr_en,
    input rd_en,
    input [ADDRWIDTH-1:0] wr_addr,
    input [ADDRWIDTH-1:0] rd_addr,
    input [WORDSIZE-1:0] data_in,
    output reg [WORDSIZE-1:0] data_out
);

reg [WORDSIZE-1:0]  memory [0:DEPTH-1];

always @(posedge clk)
begin   
    if(wr_en)
        memory[wr_addr] <= data_in;
    else if(rd_en)
        data_out <= memory[rd_addr];
end

endmodule


