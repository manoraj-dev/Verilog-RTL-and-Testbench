`timescale 1ns/1ps

`include "../src/ram_wb.v"

module tb_top_1;

    parameter DATA_WIDTH = 32;
    parameter ADDR_WIDTH = 16;
    parameter SELECT_WIDTH = 4;
    reg clk;
    reg [ADDR_WIDTH-1:0] add_i;
    reg [DATA_WIDTH-1:0] dat_i;
    wire [DATA_WIDTH-1:0] dat_o;
    reg we_i;
    reg [SELECT_WIDTH-1:0] sel_i;
    reg stb_i;
    wire ack_o;
    reg cyc_i;
    integer errors;
    integer timeout;
wb_ram #( .DATA_WIDTH(DATA_WIDTH),.ADDR_WIDTH(ADDR_WIDTH),.SELECT_WIDTH(SELECT_WIDTH)) 
DUT (.clk(clk),.add_i(add_i),.dat_i(dat_i),.dat_o(dat_o),.we_i(we_i),.sel_i(sel_i),.stb_i(stb_i),.ack_o(ack_o),.cyc_i(cyc_i));
 
always #5 clk = ~clk;
always @(add_i)
begin
  #1;
$display("ADD_I = %h | ADD_I_VALID = %h",add_i, DUT.add_i_valid);
end

//WRITE TASK
    task write_ram;
        input [15:0] address;
        input [31:0] data;
        input [3:0] byte_enable;

        begin
            @(negedge clk);

            add_i = address;
            dat_i = data;
            we_i = 1'b1;
            sel_i = byte_enable;
            cyc_i = 1'b1;
            stb_i = 1'b1;

            @(posedge clk);
 while (ack_o !== 1'b1 && timeout < 20) begin
    @(posedge clk);
    #1;
    timeout = timeout + 1;
end
            $display("DEBUG WRITE: ADDR=%h VALID_ADDR=%h",
                     add_i, DUT.add_i_valid);

            if (ack_o === 1'b1)
                $display("WRITE PASS: ADDR=%h DATA=%h SEL=%b",
                         address, data, byte_enable);
            else begin
                $display("WRITE FAIL: ACK=%b", ack_o);
                errors = errors + 1;
            end

            @(negedge clk);
            stb_i = 1'b0;
            cyc_i = 1'b0;

            @(posedge clk);
        end
    endtask

    // READ TASK
    task read_ram;
        input [15:0] address;
        input [31:0] expected_data;

        begin
            @(negedge clk);

            add_i = address;
            dat_i = 32'b0;
            we_i = 1'b0;
            sel_i = 4'b1111;
            cyc_i = 1'b1;
            stb_i = 1'b1;

            @(posedge clk);
            #1;
timeout = 0;

// Wait for ACK
while (ack_o !== 1'b1 && timeout < 20) begin
    @(posedge clk);
    #1;
    timeout = timeout + 1;
end

            $display("DEBUG READ:");
            $display("Address       = %h", add_i);
            $display("Valid Address = %h", DUT.add_i_valid);
            $display("Data Register = %h", DUT.dat_o_reg);
            $display("Data Output   = %h", dat_o);
            $display("ACK           = %b", ack_o);

            if (ack_o === 1'b1 && dat_o === expected_data) begin
                $display("READ PASS: ADDR=%h EXPECTED=%h ACTUAL=%h",
                         address, expected_data, dat_o);
            end
            else begin
                $display("READ FAIL: ADDR=%h EXPECTED=%h ACTUAL=%h",
                         address, expected_data, dat_o);
                errors = errors + 1;
            end

            @(negedge clk);
            stb_i = 1'b0;
            cyc_i = 1'b0;

            @(posedge clk);
        end
    endtask

    // MAIN TEST
    initial begin

        $dumpfile("waveform.vcd");
        $dumpvars(0, tb_top_1);

        clk = 0;
        add_i = 0;
        dat_i = 0;
        we_i = 0;
        sel_i = 0;
        stb_i = 0;
        cyc_i = 0;
        errors = 0;

        $display("==============================");
        $display("WISHBONE RAM SIMULATION START");
        $display("==============================");

        repeat(2) @(negedge clk);

        // TEST 1: INITIAL MEMORY READ
        $display("\nTEST 1: INITIAL READ");
        read_ram(16'h0000, 32'h00000000);

        // TEST 2: FULL WORD WRITE
        $display("\nTEST 2: FULL WORD WRITE");
        write_ram(16'h0000, 32'h12345678, 4'b1111);
        read_ram(16'h0000, 32'h12345678);

        // TEST 3: SECOND ADDRESS
        $display("\nTEST 3: SECOND ADDRESS");
        write_ram(16'h0004, 32'hAABBCCDD, 4'b1111);
        read_ram(16'h0004, 32'hAABBCCDD);

        // TEST 4: BYTE ENABLE
        $display("\nTEST 4: BYTE ENABLE");
        write_ram(16'h0008, 32'hAABBCCDD, 4'b1111);
        write_ram(16'h0008, 32'h11223344, 4'b0011);
        read_ram(16'h0008, 32'hAABB3344);

        // TEST 5: INDIVIDUAL BYTE WRITE
        $display("\nTEST 5: INDIVIDUAL BYTE WRITE");
        write_ram(16'h000C, 32'h000000AA, 4'b0001);
        write_ram(16'h000C, 32'h0000BB00, 4'b0010);
        write_ram(16'h000C, 32'h00CC0000, 4'b0100);
        write_ram(16'h000C, 32'hDD000000, 4'b1000);
        read_ram(16'h000C, 32'hDDCCBBAA);

        // TEST 6: LAST ADDRESS
        $display("\nTEST 6: LAST ADDRESS");
        write_ram(16'hFFFC, 32'hDEADBEEF, 4'b1111);
        read_ram(16'hFFFC, 32'hDEADBEEF);

        // FINAL RESULT
        $display("\n==============================");

        if (errors == 0)
            $display("ALL TESTS PASSED");
        else
            $display("TEST FAILED: %0d ERRORS", errors);

        $display("==============================");

        #20;
        $finish;

    end

endmodule

