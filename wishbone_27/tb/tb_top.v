`timescale 1ns/1ns

`include "../src/ram_wb.v"

module tb_top;

parameter DATA_WIDTH = 32;
parameter ADDR_WIDTH = 16;
parameter SELECT_WIDTH = DATA_WIDTH/8;
reg clk;
reg [ADDR_WIDTH-1:0] add_i;
reg [DATA_WIDTH-1:0] dat_i;
wire [DATA_WIDTH-1:0] dat_o;
reg we_i;
reg [SELECT_WIDTH-1:0] sel_i;
reg stb_i;
wire ack_o;
reg cyc_i;

reg [DATA_WIDTH-1:0] reference_mem [0:16383];

    integer i;
    integer errors;
    integer test_count;

    reg [ADDR_WIDTH-1:0] random_addr;
    reg [DATA_WIDTH-1:0] random_data;
    reg [SELECT_WIDTH-1:0] random_sel;
    reg random_we;

wb_ram #(.DATA_WIDTH(DATA_WIDTH),.ADDR_WIDTH(ADDR_WIDTH),.SELECT_WIDTH(SELECT_WIDTH))
 dut  (.clk(clk),.add_i(add_i),.dat_i(dat_i),.dat_o(dat_o),.we_i(we_i),.sel_i(sel_i),.stb_i(stb_i),.ack_o(ack_o),.cyc_i(cyc_i) );

    always #5 clk = ~clk;

    // Wishbone transaction task
    task wb_transaction;
        input [ADDR_WIDTH-1:0] address;
        input [DATA_WIDTH-1:0] write_data;
        input write_enable;
        input [SELECT_WIDTH-1:0] byte_select;

        reg [DATA_WIDTH-1:0] expected;
        integer k;

        begin

            k = address >> 2;

            @(negedge clk);

            add_i = address;
            dat_i = write_data;
            we_i = write_enable;
            sel_i = byte_select;
            cyc_i = 1'b1;
            stb_i = 1'b1;

            @(posedge clk);
            #1;

            if (ack_o !== 1'b1) begin
                $display("FAIL: ACK not received at address %h",
                         address);
                errors = errors + 1;
            end
            else begin

                if (write_enable) begin

                        for (i = 0; i < SELECT_WIDTH; i = i + 1) begin
                        if (byte_select[i]) begin
                            reference_mem[k][8*i +: 8] =
                                write_data[8*i +: 8];
                        end
                    end

                    $display("WRITE: ADDR=%h DATA=%h SEL=%b",
                             address, write_data, byte_select);

                end
                else begin

                    expected = reference_mem[k];

                    if (dat_o === expected) begin
                        $display("READ PASS: ADDR=%h EXPECTED=%h ACTUAL=%h",
                                 address, expected, dat_o);
                    end
                    else begin
                        $display("READ FAIL: ADDR=%h EXPECTED=%h ACTUAL=%h",
                                 address, expected, dat_o);
                        errors = errors + 1;
                    end

                end

            end

            @(negedge clk);
            stb_i = 1'b0;
            cyc_i = 1'b0;

            @(posedge clk);
        end
    endtask

  initial begin

        $dumpfile("waveform.vcd");
        $dumpvars(0, tb_top);

        clk = 0;
        add_i = 0;
        dat_i = 0;
        we_i = 0;
        sel_i = 0;
        stb_i = 0;
        cyc_i = 0;

        errors = 0;
        test_count = 0;

        // Initialize reference memory
        for (i = 0; i < 16384; i = i + 1)
            reference_mem[i] = 0;

        // Reproducible random sequence
        random_addr = 0;
        random_data = 0;
        random_sel = 0;
        random_we = 0;

        $display("--------------------------------------");
        $display("WISHBONE RAM RANDOM TEST STARTED");
        $display("--------------------------------------");

        repeat(2) @(negedge clk);

        // Random transactions
        for (test_count = 0; test_count < 50;
             test_count = test_count + 1) begin

            // Generate random word-aligned address
            random_addr = ($random & 16'h3FFC);

            // Generate random write data
            random_data = {$random, $random};

            // Generate random byte select
            random_sel = $random;

            // Generate random read/write operation
            random_we = $random;

$display("\nRANDOM WRITE AND READ TEST");

for (test_count = 0; test_count < 50;
     test_count = test_count + 1) begin

    // Generate random aligned address
    random_addr = ($random & 16'hFFFC);

    // Generate random data
    random_data = {$random, $random};

    // Generate random byte selection
    random_sel = $random;

    // WRITE
    wb_transaction(
        random_addr,
        random_data,
        1'b1,
        random_sel
    );

    // READ FROM THE SAME ADDRESS
    wb_transaction(
        random_addr,
        32'b0,
        1'b0,
        4'b1111
    );

end

      // Execute transaction
            wb_transaction(
                random_addr,
                random_data,
                random_we,
                random_sel
            );

        end

        // Read back all tested addresses
        $display("\n----- FINAL RANDOM DATA VERIFICATION -----");

        for (test_count = 0; test_count < 50;
             test_count = test_count + 1) begin

            // Use the same deterministic address sequence
            random_addr = ($random & 16'h3FFC);

            // This loop is not used for address reconstruction.
            // All transactions are verified in the transaction task.
        end

        $display("\n--------------------------------------");

        if (errors == 0)
            $display("ALL RANDOM TESTS PASSED");
        else
            $display("RANDOM TEST FAILED: %0d errors", errors);

        $display("--------------------------------------");

        #20;
        $finish;

    end


endmodule
