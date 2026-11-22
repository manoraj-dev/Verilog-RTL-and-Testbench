`timescale 1ns/1ns


module apb_protocol(
input PCLK,
input PRESETn,
input PENABLE,
input PSEL,
input PWRITE,
input transfer, 
input [11:0] PADDR,
input [31:0] PWDATA,
output reg [31:0] PRDATA,
output PSLVERR,
output PREADY
);

reg[31:0] mem[0:3];
integer i;
wire [1:0] idx = PADDR[3:2]; 

//write takes effect in the Access phase: PSEL and PENABLE both high
always @(posedge PCLK)begin
if(!PRESETn)
begin
for (i=0;i<4;i=i+1)
mem[i]<=32'h0;
end
else if (PSEL && PENABLE && PWRITE)
      begin
         mem[idx] <= PWDATA;
      end
end

//read data
always @(*)begin
if(PSEL && !PWRITE)
PRDATA = mem[idx];
else 
PRDATA =32'h0;
end

assign PREADY  = (PSEL && PENABLE && !PWRITE);  // high only during read access
assign PSLVERR =1'b0;

endmodule


