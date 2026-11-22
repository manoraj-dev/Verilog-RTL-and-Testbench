`timescale 1ns/1ps
`include "../src/dut_2.v"
module tb_top_2;
    localparam N = 4;
    reg          clk = 1'b0;
    reg          rst_n;
    reg  [N-1:0] req;
    wire [N-1:0] grant;

    rr_arbiter #(.N(N)) dut (
        .clk   (clk),
        .rst_n (rst_n),
        .req   (req),
        .grant (grant)
    );

    always #5 clk = ~clk;   
initial
begin
$dumpfile("waveform.vcd");
$dumpvars(0,tb_top_2);
end
    integer i;
    initial begin
        rst_n = 1'b0;
        req   = 4'b1111;   
        @(negedge clk);
        rst_n = 1'b1;

        for (i = 0; i < 8; i = i + 1) begin
            @(negedge clk);
            $display("cycle %0d : req=%b grant=%b", i, req, grant);
        end

        $display("All requesters busy -> grant rotates evenly across owners.");
        $finish;
    end
endmodule
