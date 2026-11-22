`timescale 1ns/1ns
`include "../src/dut_1.v"

module tb_top_1;
    reg PCLK;
    reg PRESETn;
    reg transfer;
    reg PWRITE ;
    reg PENABLE;
    reg PSEL;
	 reg [31:0] PADDR;
    reg [31:0] PWDATA;

    wire [31:0] PRDATA;
    wire  PREADY;
    wire PSLVERR;

    integer i;
    reg [31:0] mem [0:3];

    apb_protocol dut(
        .PCLK(PCLK),
        .PRESETn(PRESETn),
        .transfer(transfer),
        .PWRITE(PWRITE),
		  .PENABLE(PENABLE),
		  .PSEL(PSEL),
		  .PADDR(PADDR),
		  .PWDATA(PWDATA),
		  .PRDATA(PRDATA), 
        .PREADY(PREADY),   
        .PSLVERR(PSLVERR),
);
initial begin
$dumpfile("waveform.vcd");
$dumpvars(0,tb_top_1);

always begin
PCLK=0;
#5 PCLK= ~PCLK;
end

task apb_write(input [11:0] addr, input [31:0] data);
begin
@(posedge PCLK);
PSEL <=1;
PENABLE <=0;
PWRITE <=0;
PADDR <= addr;
PWDATA <= data;

@(posedge PCLK);
PENABLE <=1;

@(posedge PCLK);
while (!PREADY); 

@(posedge PCLK);
PSEL <=0;
PENABLE <=0;
end
endtask

task apb_read(input [31:0] addr ,output [31:0] data);
begin 
@(posedge PCLK);
PCLK <=1;
PENABLE <=0;
PWRITE <=0;
PADDR <=addr;
PRDATA <=data;
@(posedge PCLK);
PENABLE <=1;

@(posedge PCLK);
while (!PREADY)

@(posedge PCLK);
data <= PRDATA;
PSEL <=0;
PENABLE <=0;
end
endtask

endmodule


 
