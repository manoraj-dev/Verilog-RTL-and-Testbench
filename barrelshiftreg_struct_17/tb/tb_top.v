`timescale 1ns/1ns

`include "../src/test.v"

module tb_top;
reg [7:0] A, B;
reg CIN;
wire [7:0] SUM;
wire COUT;
reg [8:0] expected;
reg [7:0]  BB;

integer pass_count = 0;
integer fail_count = 0;

addsub8bit  DUT(
    .A(A),
    .B(B),
    .CIN(CIN),
    .SUM(SUM),
    .COUT(COUT)
);

task check_result;
begin
    #1;
    if({COUT, SUM} === expected)
    begin
        pass_count = pass_count + 1;
    end
    else
    begin
        fail_count = fail_count + 1;
        $display("TESTFAIL | A=%b B=%b CIN=%b SUM=%b COUT=%b EXPECTED=%b", A, B, CIN, SUM, COUT, expected);
    end
end
endtask

initial begin
    $monitor("A=%b | cwBA=%b | CIN=%b | COUT=%b | SUM=%b | EXPECTED=%b", CIN , A, B, COUT, SUM, expected);
	 repeat(20)
	 begin
    	CIN  = $random;
    	A = $random;
    	B = $random;
		if (CIN===1) begin
			BB = ~B;
    		expected = A+BB+CIN;
		end
		else
    		expected = A+B+CIN;
		#1;
    	check_result;
 	 end   

    // Final report
    $display("\n==============================");
    $display("       FINAL TEST REPORT");
    $display("==============================");
    $display("TOTAL TESTS : %0d", pass_count + fail_count);
    $display("TEST PASS   : %0d", pass_count);
    $display("TEST FAIL   : %0d", fail_count);
    if(fail_count == 0)
        $display("FINAL RESULT: TEST PASS");
    else
        $display("FINAL RESULT: TEST FAIL");
    $display("==============================");
    $finish;
end

endmodule
