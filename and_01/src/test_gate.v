module test_gate (
    input A, B,
    output Y_AND,
    output Y_OR,
    output Y_NAND,
    output Y_NOR,
    output Y_NOT,
    output Y_BUF,
    output Y_XOR,
    output Y_XNOR
);
    and  G1 (Y_AND, A, B);

    or   G2 (Y_OR, A, B);

    nand G3 (Y_NAND, A, B);

    nor  G4 (Y_NOR, A, B);

    not  G5 (Y_NOT, A);

    buf  G6 (Y_BUF, A);

    xor  G7 (Y_XOR, A, B);

    xnor G8 (Y_XNOR, A, B);

endmodule

