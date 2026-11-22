`timescale 1ns/1ns

module slave2(
    input PCLK,
    input PRESETn,
    input PSEL,
    input PENABLE,
    input PWRITE,
    input [7:0] PADDR,
    input [7:0] PWDATA,
    output wire [7:0] PRDATA2,
    output wire PREADY
);

reg [7:0] mem2 [0:63];

always @(posedge PCLK)
begin
    if (PRESETn && PSEL && PENABLE && PWRITE)
    begin
        mem2[PADDR[5:0]] <= PWDATA;
    end
end

assign PRDATA2 =
    (PSEL && PENABLE && !PWRITE) ?
    mem2[PADDR[5:0]] : 8'b0;

assign PREADY = PSEL && PENABLE;

endmodule

