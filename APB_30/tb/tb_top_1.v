`timescale 1ns/1ns
`include "../src/dut_1.v"

module tb_top_1;
    reg PCLK;
    reg PRESETn;
    reg transfer;
    reg PWRITE ;
    reg PENABLE;
    reg PSEL;
    reg [11:0] PADDR;
    reg [31:0] PWDATA;

    wire [31:0] PRDATA;
    wire  PREADY;
    wire PSLVERR;

    reg [31:0] mem [0:3];
    reg [31:0] rd_data;

    apb_protocol dut(
        .PCLK(PCLK),
        .PRESETn(PRESETn),
        .transfer(transfer),
        .PWRITE(PWRITE),
        .PENABLE(PENABLE),
        .PSEL(PSEL),
        .PADDR(PADDR),
        .PWDATA(PWDATA),
        .PRDATA(PRDATA),
        .PREADY(PREADY),
        .PSLVERR(PSLVERR)
    );

    initial begin
        $dumpfile("waveform.vcd");
        $dumpvars(0,tb_top_1);
    end

    initial begin
        PCLK = 0;
        forever #4 PCLK = ~PCLK;
    end

    initial begin
        // Reset
        PRESETn = 0;
        PSEL = 0;
        PENABLE = 0;
        PWRITE = 0;
        PADDR = 0;
        PWDATA = 0;
        transfer = 0;
        repeat(5) @(posedge PCLK);
        PRESETn = 1;
        @(posedge PCLK);

        // WRITE: addr = 12'h004, data = 32'hCAFE0001
        apb_write(12'h004, 32'hCAFE0001);
        repeat(2) @(posedge PCLK);

        // READ: addr = 12'h004
        apb_read(12'h004, rd_data);
        repeat(2) @(posedge PCLK);

        // WRITE: addr = 12'h008, data = 32'h12345678
        apb_write(12'h008, 32'h12345678);
        repeat(2) @(posedge PCLK);

        // READ: addr = 12'h008
        apb_read(12'h008, rd_data);
        repeat(2) @(posedge PCLK);

        $finish;
    end

    // APB write: setup (1 clk) -> access (1 clk) -> idle
    task apb_write(input [11:0] addr, input [31:0] data);
    begin
        @(posedge PCLK);
        PSEL   <= 1;
        PENABLE <= 0;
        PWRITE <= 1;
        PADDR  <= addr;
        PWDATA <= data;

        @(posedge PCLK);
        PENABLE <= 1;

        @(posedge PCLK);
        PSEL    <= 0;
        PENABLE <= 0;
        PWRITE  <= 0;
    end
    endtask

    // APB read: setup (1 clk) -> access (1 clk) -> idle
    task apb_read(input [11:0] addr, output [31:0] data);
    begin
        @(posedge PCLK);
        PSEL   <= 1;
        PENABLE <= 0;
        PWRITE <= 0;
        PADDR  <= addr;

        @(posedge PCLK);
        PENABLE <= 1;

        @(posedge PCLK);
        data = PRDATA;
        PSEL    <= 0;
        PENABLE <= 0;
    end
    endtask

endmodule
