`timescale 1ns/1ps

`include "../src/dut.v"

module tb_top;

reg [2:0] A;
reg [7:0] expected;
wire [7:0] Y;

integer pass_count = 0;
integer fail_count = 0;

decoder_3to8 dut (
    .A(A),
    .Y(Y)
);

initial begin
    $dumpfile("waveform.vcd");
    $dumpvars(0, tb_top);
    $monitor("TIME=%0t | A=%b | Y=%b | EXPECTED=%b", $time, A, Y, expected);
    // RANDOM TESTING
    repeat(20)
    begin
        A = $random;
        expected = 8'b00000001 << A;
        #1;
        if(Y === expected)
        begin
            pass_count = pass_count + 1;
        end
        else
        begin
            fail_count = fail_count + 1;
            $display("TESTFAIL | A=%b | Y=%b | EXPECTED=%b",
                      A, Y, expected);
        end
    end
    //SUMMARY
    $display("\n================================");
    $display("   		SUMMARY");
    $display("================================");
    $display("TOTAL TESTS : %0d", pass_count + fail_count);
    $display("TEST PASS   : %0d", pass_count);
    $display("TEST FAIL   : %0d", fail_count);

    if(fail_count == 0)
        $display("RESULT : TEST PASS");
    else
        $display("RESULT : TEST FAIL");

    $display("================================");
    $finish;
end

endmodule
