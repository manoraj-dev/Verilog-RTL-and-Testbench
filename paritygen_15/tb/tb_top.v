`timescale 1ns/1ns

`include "../src/test.v"

module tb_top;

reg [7:0] data;

wire even_parity;
wire odd_parity;

reg expected_even;
reg expected_odd;

integer i, j;
integer count_ones;

 paritygenerator  dut (
    .data(data),
    .even_parity(even_parity),
    .odd_parity(odd_parity)
);
initial
begin
    $dumpfile("waveform.vcd");
    $dumpvars(0,tb_top );
end

initial
begin

    $monitor("Time=%0t | DATA=%b | EVEN=%b | ODD=%b",
              $time, data, even_parity, odd_parity);

    for (i = 0; i < 8; i = i + 1)
    begin

        data = i;
        #1;

        count_ones = 0;

        // Count number of 1s
        for (j = 0; j < 8; j = j + 1)
        begin
            if (data[j] == 1'b1)
                count_ones = count_ones + 1;
        end

        // Calculate expected outputs
        expected_even = count_ones % 2;
        expected_odd  = ~expected_even;

        // Output checker
        if ((even_parity === expected_even) &&
            (odd_parity === expected_odd))
        begin
            $display("PASS | DATA=%b | EVEN=%b | ODD=%b",
                      data, even_parity, odd_parity);
        end
        else
        begin
            $display("FAIL | DATA=%b | Expected EVEN=%b ODD=%b | Actual EVEN=%b ODD=%b",
                      data, expected_even, expected_odd,
                      even_parity, odd_parity);
        end

    end

    $display("Simulation Completed!");
    $finish;

end

endmodule

