`timescale 1ns/1ps

module tb_top;

reg [3:0] I;
reg [1:0] S;
wire Y;

// DUT Instantiation
mux_4x1 u_dut (
    .I(I),
    .S(S),
    .Y(Y)
);

// Test stimulus
initial
begin
    $dumpfile("waveform.vcd");
    $dumpvars(0, tb_top);

    $monitor("Time=%0t | S=%b | I=%b | Y=%b",
              $time, S, I, Y);

    I = 4'b1010;

    S = 2'b00; #10;
    S = 2'b01; #10;
    S = 2'b10; #10;
    S = 2'b11; #10;

    I = 4'b1101;

    S = 2'b00; #10;
    S = 2'b01; #10;
    S = 2'b10; #10;
    S = 2'b11; #10;

    $finish;
end

endmodule







