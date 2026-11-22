
`timescale 1ns/1ps

module tb_top_1;

    reg clk;
    reg rst;

    reg cyc_i;
    reg stb_i;
    reg we_i;
    reg [3:0] sel_i;
    reg [15:0] add_i;
    reg [31:0] dat_i;

    wire ack_o;
    wire [31:0] dat_o;

    integer i;
    integer timeout;
    integer pass_count;
    integer fail_count;

    reg [31:0] test_data [0:9];

    // DUT
    wb_ram DUT (
        .clk(clk),
        .rst(rst),
        .cyc_i(cyc_i),
        .stb_i(stb_i),
        .we_i(we_i),
        .sel_i(sel_i),
        .add_i(add_i),
        .dat_i(dat_i),
        .ack_o(ack_o),
        .dat_o(dat_o)
    );

    // Clock generation: 10 ns
    initial begin
        clk = 0;
        forever #5 clk = ~clk;
    end

    // --------------------------------------------------
    // WRITE TASK
    // --------------------------------------------------
    task write_ram;
        input [15:0] address;
        input [31:0] data;
        begin
            @(negedge clk);

            cyc_i = 1'b1;
            stb_i = 1'b1;
            we_i  = 1'b1;
            sel_i = 4'b1111;
            add_i = address;
            dat_i = data;

            timeout = 0;

            // Wait for ACK
            while (ack_o !== 1'b1 && timeout < 20) begin
                @(posedge clk);
                timeout = timeout + 1;
            end

            #1;

            if (ack_o === 1'b1) begin
                $display("WRITE PASS | ADDR=%h | DATA=%h | ACK=%b",
                         address, data, ack_o);
                pass_count = pass_count + 1;
            end
            else begin
                $display("WRITE FAIL | ADDR=%h | ACK TIMEOUT", address);
                fail_count = fail_count + 1;
            end

            @(negedge clk);
            cyc_i = 1'b0;
            stb_i = 1'b0;
            we_i  = 1'b0;
            sel_i = 4'b0000;
            dat_i = 32'b0;
        end
    endtask

    // --------------------------------------------------
    // READ TASK
    // --------------------------------------------------
    task read_ram;
        input [15:0] address;
        input [31:0] expected_data;
        begin
            @(negedge clk);

            cyc_i = 1'b1;
            stb_i = 1'b1;
            we_i  = 1'b0;
            sel_i = 4'b1111;
            add_i = address;
            dat_i = 32'b0;

            timeout = 0;

            // Wait for ACK
            while (ack_o !== 1'b1 && timeout < 20) begin
                @(posedge clk);
                timeout = timeout + 1;
            end

            #1;

            if (ack_o === 1'b1) begin
                if (dat_o === expected_data) begin
                    $display("READ PASS  | ADDR=%h | DATA=%h | ACK=%b",
                             address, dat_o, ack_o);
                    pass_count = pass_count + 1;
                end
                else begin
                    $display("READ FAIL  | ADDR=%h | EXPECTED=%h | GOT=%h | ACK=%b",
                             address, expected_data, dat_o, ack_o);
                    fail_count = fail_count + 1;
                end
            end
            else begin
                $display("READ FAIL  | ADDR=%h | ACK TIMEOUT",
                         address);
                fail_count = fail_count + 1;
            end

            @(negedge clk);
            cyc_i = 1'b0;
            stb_i = 1'b0;
            we_i  = 1'b0;
            sel_i = 4'b0000;
            dat_i = 32'b0;
        end
    endtask

    // --------------------------------------------------
    // MAIN TEST
    // --------------------------------------------------
    initial begin

        rst       = 1'b0;
        cyc_i     = 1'b0;
        stb_i     = 1'b0;
        we_i      = 1'b0;
        sel_i     = 4'b0000;
        add_i     = 16'b0;
        dat_i     = 32'b0;

        pass_count = 0;
        fail_count = 0;
        timeout    = 0;

        // 10 different data values
        test_data[0] = 32'h12345678;
        test_data[1] = 32'hAABBCCDD;
        test_data[2] = 32'hDEADBEEF;
        test_data[3] = 32'hCAFEBABE;
        test_data[4] = 32'h11223344;
        test_data[5] = 32'h55667788;
        test_data[6] = 32'h99887766;
        test_data[7] = 32'h0F0F0F0F;
        test_data[8] = 32'hF0F0F0F0;
        test_data[9] = 32'hABCDEF12;

        $dumpfile("wishbone_ram.vcd");
        $dumpvars(0, tb_top_1);

        $display("------------------------------------------");
        $display("      WISHBONE RAM TEST START");
        $display("------------------------------------------");

        // Wait for initial clock cycles
        repeat (3) @(posedge clk);

        // ==========================================
        // PHASE 1: WRITE 10 DIFFERENT LOCATIONS
        // ==========================================
        $display("\nPHASE 1: WRITING 10 DATA VALUES");

        for (i = 0; i < 10; i = i + 1) begin
            write_ram(i * 4, test_data[i]);
        end

        // ==========================================
        // PHASE 2: READ THE SAME 10 LOCATIONS
        // ==========================================
        $display("\nPHASE 2: READING THE SAME 10 LOCATIONS");

        for (i = 0; i < 10; i = i + 1) begin
            read_ram(i * 4, test_data[i]);
        end

        // ==========================================
        // PHASE 3: 5 ADDITIONAL WRITE/READ PAIRS
        // ==========================================
        $display("\nPHASE 3: 5 ADDITIONAL WRITE/READ PAIRS");

        for (i = 0; i < 5; i = i + 1) begin

            write_ram(16'h0040 + (i * 4),
                     32'h90000000 + i);

            read_ram(16'h0040 + (i * 4),
                    32'h90000000 + i);

        end

        // ==========================================
        // FINAL RESULT
        // ==========================================
        $display("\n------------------------------------------");
        $display("             TEST SUMMARY");
        $display("------------------------------------------");
        $display("PASS COUNT = %0d", pass_count);
        $display("FAIL COUNT = %0d", fail_count);

        if (fail_count == 0)
            $display("ALL TESTS PASSED");
        else
            $display("TEST FAILED");

        $display("------------------------------------------");

        #20;
        $finish;
    end

endmodule

