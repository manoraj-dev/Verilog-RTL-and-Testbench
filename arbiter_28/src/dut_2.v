`timescale 1ns/1ns

module rr_arbiter #(
    parameter N = 4
) (
    input  wire  clk,
    input  wire  rst_n,
    input  wire [N-1:0] req,
    output wire [N-1:0] grant
);
    reg  [N-1:0] ptr;
    wire [N-1:0] hi_sel  = ~(ptr - 1'b1);
    wire [N-1:0] hi_req  = req & hi_sel;

    wire [N-1:0] grant_hi = hi_req & (~hi_req + 1'b1);
    wire [N-1:0] grant_lo = req    & (~req    + 1'b1);

    assign grant = (|hi_req) ? grant_hi : grant_lo;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            ptr <= {{(N-1){1'b0}}, 1'b1};     
        else if (|grant)
            ptr <= {grant[N-2:0], grant[N-1]}; 
    end
endmodule
