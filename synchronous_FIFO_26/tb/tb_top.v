`timescale 1ns/1ns

`include "../src/dut.v"


module tb_top;

reg clk;
reg rst_n;
reg w_en;
reg r_en;
reg [7:0] data_in;

wire [7:0] data_out;
wire full;
wire empty;

integer i;

sync_fifo dut (
    .clk(clk),
    .rst_n(rst_n),
    .w_en(w_en),
    .r_en(r_en),
    .data_in(data_in),
    .data_out(data_out),
    .full(full),
    .empty(empty)
);

initial begin
    clk = 0;
    forever #5 clk = ~clk;
end

initial begin
    $dumpfile("waveform.vcd");
    $dumpvars(0, tb_top);
end

initial begin

    rst_n = 0;
    w_en = 0;
    r_en = 0;
    data_in = 0;

    repeat(2) @(posedge clk);
    @(negedge clk);
    rst_n = 1;

    $display("--------------------------------");
    $display("       FIFO TEST STARTED");
    $display("--------------------------------");

    push(8'h11);
    push(8'h22);
    push(8'h33);
    push(8'h44);
    push(8'h55);

 
    pop;
    pop;
    pop;

    // Random write and read operations
    for(i = 0; i < 20; i = i + 1) begin

        if(($random & 1) == 0)
            push($random);
        else
            pop;

    end

     while(empty == 0)
        pop;

    $display("--------------------------------");
    $display("        FIFO TEST ENDED");
    $display("--------------------------------");

    #20;
    $finish;

end

// PUSH TASK
task push;
input [7:0] value;
begin

    @(negedge clk);

    if(full == 0) begin

        w_en = 1;
        r_en = 0;
        data_in = value;

        @(posedge clk);
        #1;

        $display("WRITE: Data = %h | Full = %b | Empty = %b",
                 data_in, full, empty);

        @(negedge clk);
        w_en = 0;

    end
    else begin

        $display("FIFO FULL! Write operation rejected.");

    end

end
endtask

// POP TASK
task pop;
begin

    @(negedge clk);

    if(empty == 0) begin

        r_en = 1;
        w_en = 0;

        @(posedge clk);
        #1;

        $display("READ : Data = %h | Full = %b | Empty = %b",
                 data_out, full, empty);

        @(negedge clk);
        r_en = 0;

    end
    else begin

        $display("FIFO EMPTY! Read operation rejected.");

    end

end
endtask

endmodule

