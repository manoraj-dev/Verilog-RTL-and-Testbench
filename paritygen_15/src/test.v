`timescale 1ns/1ns

module  paritygenerator (
    input  [7:0] data,
    output even_parity,
    output odd_parity
);

assign even_parity = ^data;

assign odd_parity = ~^data;

endmodule
