`timescale 1ns/1ns

`include "../src/test.v"


module tb_top;

    reg clk;
    reg rst_n;
    reg [1:0] sel;
    reg [3:0] p_din;
    reg s_left_din;
    reg s_right_din;

    wire [3:0] p_dout;

    universal_shift_reg DUT (
        .clk(clk),
        .rst_n(rst_n),
        .sel(sel),
        .p_din(p_din),
        .s_left_din(s_left_din),
        .s_right_din(s_right_din),
        .p_dout(p_dout)
    );

    // Clock generation
    always #5 clk = ~clk;
    task check_output;
        input [3:0] expected;
        begin
            #1;
            if (p_dout !== expected)
                $display("FAIL: Expected=%b Actual=%b",
                         expected, p_dout);
            else
                $display("PASS: Output=%b", p_dout);
        end
    endtask

    initial begin

        $dumpfile("waveform.vcd");
        $dumpvars(0, tb_top);

        clk = 0;
        rst_n = 0;
        sel = 2'b11;
        p_din = 4'b1101;
        s_left_din = 0;
        s_right_din = 0;

        // Reset
        #12 rst_n = 1;

        // Parallel load
        @(posedge clk);
        check_output(4'b1101);

        // No change
        @(negedge clk);
        sel = 2'b00;
        @(posedge clk);
        check_output(4'b1101);

        // Shift right
        @(negedge clk);
        sel = 2'b01;
        s_right_din = 0;
        @(posedge clk);
        check_output(4'b0110);

        // Shift left
        @(negedge clk);
        sel = 2'b10;
        s_left_din = 1;
        @(posedge clk);
        check_output(4'b1101);

        #10;
        $finish;

    end

endmodule


