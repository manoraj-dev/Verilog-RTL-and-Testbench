`timescale 1ns/1ns
`include "../src/dut.v"

module tb_top;

    reg PCLK;
    reg PRESETn;
    reg transfer;
    reg READ_WRITE;

    reg [8:0] apb_write_paddr;
    reg [7:0] apb_write_data;
    reg [8:0] apb_read_paddr;

    wire [7:0] apb_read_data_out;
    wire PSLVERR;

    integer i;
    reg [7:0] expected;

    APB_Protocol dut_c (
        .PCLK(PCLK),
        .PRESETn(PRESETn),
        .transfer(transfer),
        .READ_WRITE(READ_WRITE),
        .apb_write_paddr(apb_write_paddr),
        .apb_write_data(apb_write_data),
        .apb_read_paddr(apb_read_paddr),
        .PSLVERR(PSLVERR),
        .apb_read_data_out(apb_read_data_out)
    );

    initial begin
        PCLK = 1'b0;
        forever #5 PCLK = ~PCLK;
    end

    task apb_write;
        input [8:0] address;
        input [7:0] write_value;
        begin
            @(negedge PCLK);
            READ_WRITE      = 1'b0;
            apb_write_paddr = address;
            apb_write_data  = write_value;
            transfer        = 1'b1;

            wait (dut_c.dut0.state == 3'b100);
            wait (dut_c.PREADY == 1'b1);
            @(posedge PCLK);
            #1;
            @(negedge PCLK);
            transfer = 1'b0;
            $display("%0t ns WRITE addr=%03h data=%02h",
                     $time, address, write_value);
        end
    endtask

    task apb_read;
        input [8:0] address;
        input [7:0] expected_value;
        begin
            @(negedge PCLK);
            READ_WRITE    = 1'b1;
            apb_read_paddr = address;
            transfer      = 1'b1;

            wait (dut_c.dut0.state == 3'b100);
            wait (dut_c.PREADY == 1'b1);
            @(posedge PCLK);
            #1;

            if (apb_read_data_out !== expected_value) begin
                $display("ERROR: addr=%03h expected=%02h actual=%02h",
                         address, expected_value, apb_read_data_out);
            end
            else begin
                $display("%0t ns READ  addr=%03h data=%02h PASS",
                         $time, address, apb_read_data_out);
            end

            @(negedge PCLK);
            transfer = 1'b0;
        end
    endtask

    initial begin
        $dumpfile("Waveform.vcd");
        $dumpvars(0, tb_top);
    end

    initial begin
        PRESETn         = 1'b0;
        transfer        = 1'b0;
        READ_WRITE      = 1'b0;
        apb_write_paddr = 9'd0;
        apb_write_data  = 8'd0;
        apb_read_paddr  = 9'd0;

        repeat (3) @(negedge PCLK);
        PRESETn = 1'b1;
        repeat (2) @(negedge PCLK);

        $display("\n===== WRITE SLAVE 1 =====");
        for (i = 0; i < 8; i = i + 1)
            apb_write({1'b0, i[7:0]}, i * 2);

        $display("\n===== WRITE SLAVE 2 =====");
        for (i = 0; i < 8; i = i + 1)
            apb_write({1'b1, i[7:0]}, i);

        repeat (2) @(negedge PCLK);

        $display("\n===== READ SLAVE 1 =====");
        for (i = 0; i < 8; i = i + 1)
            apb_read({1'b0, i[7:0]}, i * 2);

        $display("\n===== READ SLAVE 2 =====");
        for (i = 0; i < 8; i = i + 1)
            apb_read({1'b1, i[7:0]}, i);

        $display("\n===== APB TEST COMPLETED =====");
        repeat (3) @(negedge PCLK);
        $finish;
    end

endmodule

