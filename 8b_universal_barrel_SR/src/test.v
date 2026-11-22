`timescale 1ns/1ns


module mux4_1 (
    input d0, d1, d2, d3,
    input [1:0] sel,
    output y
);

    assign y = (sel == 2'b00) ? d0 :
               (sel == 2'b01) ? d1 :
               (sel == 2'b10) ? d2 : d3;

endmodule


// D Flip-Flop with Active Low Reset

module dff_rst_n (
    input clk,
    input rst_n,
    input d,
    output reg q
);

    always @(posedge clk or negedge rst_n)
    begin
        if (!rst_n)
            q <= 1'b0;
        else
            q <= d;
    end

endmodule


// 4-bit Universal Shift Register

module universal_shift_reg (
    input clk,
    input rst_n,
    input [1:0] sel,
    input [3:0] p_din,
    input s_left_din,
    input s_right_din,
    output [3:0] p_dout
);

    wire [3:0] mux_out;

    // Bit 3
    mux4_1 MUX3 (
        .d0(p_dout[3]),
        .d1(s_right_din),
        .d2(p_dout[2]),
        .d3(p_din[3]),
        .sel(sel),
        .y(mux_out[3])
    );

    // Bit 2
    mux4_1 MUX2 (
        .d0(p_dout[2]),
        .d1(p_dout[3]),
        .d2(p_dout[1]),
        .d3(p_din[2]),
        .sel(sel),
        .y(mux_out[2])
    );

    // Bit 1
    mux4_1 MUX1 (
        .d0(p_dout[1]),
        .d1(p_dout[2]),
        .d2(p_dout[0]),
        .d3(p_din[1]),
        .sel(sel),
        .y(mux_out[1])
    );

    // Bit 0
    mux4_1 MUX0 (
        .d0(p_dout[0]),
        .d1(p_dout[1]),
        .d2(s_left_din),
        .d3(p_din[0]),
        .sel(sel),
        .y(mux_out[0])
    );

    // Four D Flip-Flops

    dff_rst_n DFF3 (
        .clk(clk),
        .rst_n(rst_n),
        .d(mux_out[3]),
        .q(p_dout[3])
    );

    dff_rst_n DFF2 (
        .clk(clk),
        .rst_n(rst_n),
        .d(mux_out[2]),
        .q(p_dout[2])
    );

    dff_rst_n DFF1 (
        .clk(clk),
        .rst_n(rst_n),
        .d(mux_out[1]),
        .q(p_dout[1])
    );

    dff_rst_n DFF0 (
        .clk(clk),
        .rst_n(rst_n),
        .d(mux_out[0]),
        .q(p_dout[0])
    );

endmodule

