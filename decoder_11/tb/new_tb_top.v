`timescale 1ns/1ns

`include "../src/dut.v"
module tb_top;

reg [2:0] A;
wire [7:0] Y;
reg [7:0] expected;

integer i;
integer pass_count = 0;
integer fail_count = 0;

decoder_3to8 DUT(
    .A(A),
    .Y(Y)
);

initial
begin

    $monitor("TIME=%0t | A=%b | Y=%b | EXPECTED=%b",
              $time, A, Y, expected);

    for(i = 0; i < 8; i = i + 1)
    begin

        A = i;
        expected = 8'b00000001 << i;

        #10;

        if(Y === expected)
        begin
            pass_count = pass_count + 1;
        end
        else
        begin
            fail_count = fail_count + 1;
            $display("FAIL: A=%b | Y=%b | EXPECTED=%b",
                      A, Y, expected);
        end

    end

    // FINAL CHECKER
    $display("\n================================");
    $display("       FINAL TEST REPORT");
    $display("================================");

    $display("TOTAL TEST CASES : %0d", pass_count + fail_count);
    $display("PASSED           : %0d", pass_count);
    $display("FAILED           : %0d", fail_count);

    if(fail_count == 0)
        $display("FINAL RESULT     : TEST PASS");
    else
        $display("FINAL RESULT     : TEST FAIL");

    $display("================================");

    $finish;

end

endmodule
