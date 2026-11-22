`timescale 1ns/1ns


module APB_Protocol(
    input         PCLK,
    input         PRESETn,
    input         transfer,
    input         READ_WRITE,
    input  [8:0]  apb_write_paddr,
    input  [7:0]  apb_write_data,
    input  [8:0]  apb_read_paddr,
    output        PSLVERR,
    output [7:0]  apb_read_data_out
);

    wire [8:0] PADDR;
    wire [7:0] PWDATA;
    wire [7:0] PRDATA;
    wire       PWRITE;
    wire       PENABLE;
    wire       PSEL1, PSEL2;
    wire       PREADY, PREADY1, PREADY2;
    wire [7:0] PRDATA1, PRDATA2;

    assign PREADY = PADDR[8] ? PREADY2 : PREADY1;
    assign PRDATA = PADDR[8] ? PRDATA2 : PRDATA1;

    master_bridge dut0 (
        .PCLK(PCLK),
        .PRESETn(PRESETn),
        .transfer(transfer),
        .READ_WRITE(READ_WRITE),
        .apb_write_paddr(apb_write_paddr),
        .apb_write_data(apb_write_data),
        .apb_read_paddr(apb_read_paddr),
        .PRDATA(PRDATA),
        .PREADY(PREADY),
        .PSEL1(PSEL1),
        .PSEL2(PSEL2),
        .PENABLE(PENABLE),
        .PADDR(PADDR),
        .PWRITE(PWRITE),
        .PWDATA(PWDATA),
        .apb_read_data_out(apb_read_data_out),
        .PSLVERR(PSLVERR)
    );

    slave1 dut1 (
        .PCLK(PCLK),
        .PRESETn(PRESETn),
        .PSEL(PSEL1),
        .PENABLE(PENABLE),
        .PWRITE(PWRITE),
        .PADDR(PADDR[7:0]),
        .PWDATA(PWDATA),
        .PRDATA1(PRDATA1),
        .PREADY(PREADY1)
    );

    slave2 dut2 (
        .PCLK(PCLK),
        .PRESETn(PRESETn),
        .PSEL(PSEL2),
        .PENABLE(PENABLE),
        .PWRITE(PWRITE),
        .PADDR(PADDR[7:0]),
        .PWDATA(PWDATA),
        .PRDATA2(PRDATA2),
        .PREADY(PREADY2)
    );

endmodule


module master_bridge(
    input         PCLK,
    input         PRESETn,
    input         transfer,
    input         READ_WRITE,
    input  [8:0]  apb_write_paddr,
    input  [7:0]  apb_write_data,
    input  [8:0]  apb_read_paddr,
    input  [7:0]  PRDATA,
    input         PREADY,
    output wire   PSEL1,
    output wire   PSEL2,
    output wire   PENABLE,
    output reg [8:0] PADDR,
    output wire   PWRITE,
    output reg [7:0] PWDATA,
    output reg [7:0] apb_read_data_out,
    output wire   PSLVERR
);

    localparam IDLE   = 3'b001;
    localparam SETUP  = 3'b010;
    localparam ENABLE = 3'b100;

    reg [2:0] state;

    assign PWRITE  = ~READ_WRITE;
    assign PENABLE = (state == ENABLE);

    assign PSEL1 = ((state != IDLE) && (PADDR[8] == 1'b0));
    assign PSEL2 = ((state != IDLE) && (PADDR[8] == 1'b1));

    assign PSLVERR = 1'b0;

    always @(posedge PCLK or negedge PRESETn) begin
        if (!PRESETn) begin
            state            <= IDLE;
            PADDR            <= 9'd0;
            PWDATA           <= 8'd0;
            apb_read_data_out <= 8'd0;
        end
        else begin
            case (state)
                IDLE: begin
                    if (transfer) begin
                        if (READ_WRITE)
                            PADDR <= apb_read_paddr;
                        else begin
                            PADDR  <= apb_write_paddr;
                            PWDATA <= apb_write_data;
                        end
                        state <= SETUP;
                    end
                end

                SETUP: begin
                    state <= ENABLE;
                end

                ENABLE: begin
                    if (PREADY) begin
                        if (READ_WRITE)
                            apb_read_data_out <= PRDATA;
                        state <= IDLE;
                    end
                end

                default: begin
                    state <= IDLE;
                end
            endcase
        end
    end

endmodule


module slave1(
    input         PCLK,
    input         PRESETn,
    input         PSEL,
    input         PENABLE,
    input         PWRITE,
    input  [7:0]  PADDR,
    input  [7:0]  PWDATA,
    output wire [7:0] PRDATA1,
    output wire   PREADY
);

    reg [7:0] mem [0:255];

    assign PREADY = PSEL && PENABLE;
    assign PRDATA1 = mem[PADDR];

    always @(posedge PCLK or negedge PRESETn) begin
        if (!PRESETn) begin
        end
        else if (PSEL && PENABLE && PWRITE && PREADY) begin
            mem[PADDR] <= PWDATA;
        end
    end

endmodule


module slave2(
    input         PCLK,
    input         PRESETn,
    input         PSEL,
    input         PENABLE,
    input         PWRITE,
    input  [7:0]  PADDR,
    input  [7:0]  PWDATA,
    output wire [7:0] PRDATA2,
    output wire   PREADY
);

    reg [7:0] mem2 [0:255];

    assign PREADY = PSEL && PENABLE;
    assign PRDATA2 = mem2[PADDR];
        if (!PRESETn) begin
        end
        else if (PSEL && PENABLE && PWRITE && PREADY) begin
            mem2[PADDR] <= PWDATA;
        end
    end

endmodule

