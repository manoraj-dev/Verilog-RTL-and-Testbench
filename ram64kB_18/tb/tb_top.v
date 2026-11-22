`timescale 1ns/1ns

`include "../src/dut.v"

module tb_top;

localparam ADDRWIDTH=16;
localparam WORDSIZE=32;
localparam DEPTH=1024;

reg clk;
reg wr_en, rd_en;
reg [ADDRWIDTH-1:0] wr_addr;
reg [ADDRWIDTH-1:0] rd_addr;
reg  [WORDSIZE-1:0] data_in;
wire [WORDSIZE-1:0] data_out;

RAM_64KB #(ADDRWIDTH, WORDSIZE, DEPTH) dut(
	.clk(clk),
	.wr_en(wr_en),
	.rd_en(rd_en),
	.wr_addr(wr_addr),
	.rd_addr(rd_addr),
	.data_in(data_in),
	.data_out(data_out)
);

initial begin
    clk    =0;
	 wr_en  =0;
	 rd_en  =0;
	 wr_addr=0;
	 rd_addr=0;
end

always #5 clk = ~clk;

initial begin
	#1;
	write_mem_nwords(16);
	#50;
	read_mem_nwords(16);
	#20;
	$finish;
end

initial begin
   $monitor("$TIME=%0t | WR_EN=%b | RD_EN=%b | WR_ADDR=%b | RD_ADDR=%b | DATA_IN=%b | DATA_OUT=%b", $time, wr_en, rd_en, wr_addr, rd_addr, data_in, data_out);
end

initial begin
	#1;
	$dumpfile("waveform.vcd");
	$dumpvars(0, tb_top);
end

task write_mem_nwords(input integer count=16);
integer k;
begin
	//@(posedge clk);
	//#1;
	for (k=0; k<count; k=k+1)
	begin
		wr_en   = 1'b1;
		rd_en   = 1'b0;
		wr_addr = k;
		data_in = $random;
		@(posedge clk);
		#1;
	end
	wr_en = 1'b0;
	$display("Written %0d words into the RAM locations...", count);
end
endtask

task read_mem_nwords(input integer count=16);
integer k;
begin
	//@(posedge clk);
	//#1;
	for (k=0; k<count; k=k+1)
	begin
		wr_en   = 1'b0;
		rd_en   = 1'b1;
		rd_addr = k;
		@(posedge clk);
		#1;
		$display("READ addr=%0d data=%0h", k, data_out);
	end
	rd_en = 1'b0;
	$display("Read %0d words from the RAM locations...", count);
end
endtask

endmodule
