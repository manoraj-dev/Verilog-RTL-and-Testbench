`timescale 1ns / 1ps

`include "../src/dp_ram_wb.v"

module tb_top_3;

// Parameters
parameter DATA_WIDTH = 32;
parameter ADDR_WIDTH = 16;
parameter SELECT_WIDTH = 4;

// Port A signals
reg a_clk = 0;
reg [ADDR_WIDTH-1:0] a_adr_i = 0;
reg [DATA_WIDTH-1:0] a_dat_i = 0;
wire [DATA_WIDTH-1:0] a_dat_o;
reg a_we_i = 0;
reg [SELECT_WIDTH-1:0] a_sel_i = 0;
reg a_stb_i = 0;
wire a_ack_o;
reg a_cyc_i = 0;

// Port B signals
reg b_clk = 0;
reg [ADDR_WIDTH-1:0] b_adr_i = 0;
reg [DATA_WIDTH-1:0] b_dat_i = 0;
wire [DATA_WIDTH-1:0] b_dat_o;
reg b_we_i = 0;
reg [SELECT_WIDTH-1:0] b_sel_i = 0;
reg b_stb_i = 0;
wire b_ack_o;
reg b_cyc_i = 0;

// Clock generation
always #5 a_clk = ~a_clk;
always #5 b_clk = ~b_clk;

// DUT
dpram_wb #(
    .DATA_WIDTH(DATA_WIDTH),
    .ADDR_WIDTH(ADDR_WIDTH),
    .SELECT_WIDTH(SELECT_WIDTH)
)
dut (
    .a_clk(a_clk),
    .a_adr_i(a_adr_i),
    .a_dat_i(a_dat_i),
    .a_dat_o(a_dat_o),
    .a_we_i(a_we_i),
    .a_sel_i(a_sel_i),
    .a_stb_i(a_stb_i),
    .a_ack_o(a_ack_o),
    .a_cyc_i(a_cyc_i),

    .b_clk(b_clk),
    .b_adr_i(b_adr_i),
    .b_dat_i(b_dat_i),
    .b_dat_o(b_dat_o),
    .b_we_i(b_we_i),
    .b_sel_i(b_sel_i),
    .b_stb_i(b_stb_i),
    .b_ack_o(b_ack_o),
    .b_cyc_i(b_cyc_i)
);

// Port A write task
task port_a_write;
input [ADDR_WIDTH-1:0] addr;
input [DATA_WIDTH-1:0] data;
begin
    @(negedge a_clk);
    a_adr_i = addr;
    a_dat_i = data;
    a_we_i  = 1'b1;
    a_sel_i = {SELECT_WIDTH{1'b1}};
    a_cyc_i = 1'b1;
    a_stb_i = 1'b1;

    // Wait for registered acknowledgement
    @(posedge a_clk);
    #1;
    while (a_ack_o !== 1'b1) begin
        @(posedge a_clk);
        #1;
    end

    $display("PORT A WRITE: Address=%h Data=%h ACK=%b",
             addr, data, a_ack_o);

    @(negedge a_clk);
    a_stb_i = 0;
    a_cyc_i = 0;
    a_we_i  = 0;
end
endtask

// Port A read task
task port_a_read;
input [ADDR_WIDTH-1:0] addr;
begin
    @(negedge a_clk);
    a_adr_i = addr;
    a_we_i  = 1'b0;
    a_sel_i = {SELECT_WIDTH{1'b1}};
    a_cyc_i = 1'b1;
    a_stb_i = 1'b1;

    @(posedge a_clk);
    #1;
    while (a_ack_o !== 1'b1) begin
        @(posedge a_clk);
        #1;
    end

    $display("PORT A READ : Address=%h Data=%h ACK=%b",
             addr, a_dat_o, a_ack_o);

    @(negedge a_clk);
    a_stb_i = 0;
    a_cyc_i = 0;
end
endtask

// Port B write task
task port_b_write;
input [ADDR_WIDTH-1:0] addr;
input [DATA_WIDTH-1:0] data;
begin
    @(negedge b_clk);
    b_adr_i = addr;
    b_dat_i = data;
    b_we_i  = 1'b1;
    b_sel_i = {SELECT_WIDTH{1'b1}};
    b_cyc_i = 1'b1;
    b_stb_i = 1'b1;

    @(posedge b_clk);
    #1;
    while (b_ack_o !== 1'b1) begin
        @(posedge b_clk);
        #1;
    end

    $display("PORT B WRITE: Address=%h Data=%h ACK=%b",
             addr, data, b_ack_o);

    @(negedge b_clk);
    b_stb_i = 0;
    b_cyc_i = 0;
    b_we_i  = 0;
end
endtask

// Port B read task
task port_b_read;
input [ADDR_WIDTH-1:0] addr;
begin
    @(negedge b_clk);
    b_adr_i = addr;
    b_we_i  = 1'b0;
    b_sel_i = {SELECT_WIDTH{1'b1}};
    b_cyc_i = 1'b1;
    b_stb_i = 1'b1;

    @(posedge b_clk);
    #1;
    while (b_ack_o !== 1'b1) begin
        @(posedge b_clk);
        #1;
    end

    $display("PORT B READ : Address=%h Data=%h ACK=%b",
             addr, b_dat_o, b_ack_o);

    @(negedge b_clk);
    b_stb_i = 0;
    b_cyc_i = 0;
end
endtask


integer i;
reg [DATA_WIDTH-1:0] random_data;

initial begin

    $dumpfile("waveform.vcd");
    $dumpvars(0, tb_top_3);

    $display("--------------------------------");
    $display("DUAL PORT RAM TEST STARTED");
    $display("--------------------------------");

    #20;


    port_a_write(16'h0004, 32'h12345678);


    port_a_read(16'h0004);


    port_b_write(16'h0008, 32'hABCDEF12);


    port_b_read(16'h0008);


    port_b_read(16'h0004);


    port_a_read(16'h0008);


    for (i = 0; i < 10; i = i + 1) begin
        random_data = $random;
        port_a_write(i, random_data);
        port_a_read(i);
    end


    for (i = 10; i < 20; i = i + 1) begin
        random_data = $random;
        port_b_write(i, random_data);
        port_b_read(i);
    end

    #20;

    $display("--------------------------------");
    $display("DUAL PORT RAM TEST COMPLETED");
    $display("--------------------------------");

    $finish;
end

endmodule

