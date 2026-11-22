
`timescale 1ns/1ns

module master(
    input  [8:0] apb_write_paddr,
    input  [8:0] apb_read_paddr,
    input  [7:0] apb_write_data,
    input  [7:0] PRDATA,

    input PRESETn,
    input PCLK,
    input READ_WRITE,
    input transfer,
    input PREADY,

    output wire PSEL1,
    output wire PSEL2,
    output wire PENABLE,
    output reg  [8:0] PADDR,
    output reg  PWRITE,
    output reg  [7:0] PWDATA,
    output reg  [7:0] apb_read_data_out,
    output wire PSLVERR
);

// FSM STATES
localparam IDLE   = 3'b001,
           SETUP  = 3'b010,
           ENABLE = 3'b100;

reg [2:0] state;

// FSM SEQUENTIAL LOGIC
always @(posedge PCLK or negedge PRESETn)
begin
    if (!PRESETn)
    begin
        state            <= IDLE;
        PADDR            <= 9'd0;
        PWDATA           <= 8'd0;
        PWRITE           <= 1'b0;
        apb_read_data_out <= 8'd0;
    end
    else
    begin
        case (state)

            // IDLE STATE
            IDLE:
            begin
                if (transfer)
                begin
                    if (READ_WRITE)
                    begin
                        // READ OPERATION
                        PADDR <= apb_read_paddr;
                        PWRITE <= 1'b0;
                    end
                    else
                    begin
                        // WRITE OPERATION
                        PADDR <= apb_write_paddr;
                        PWDATA <= apb_write_data;
                        PWRITE <= 1'b1;
                    end

                    state <= SETUP;
                end
                else
                begin
                    state <= IDLE;
                end
            end

            // SETUP STATE
            SETUP:
            begin
                state <= ENABLE;
            end

            // ENABLE STATE
            ENABLE:
            begin
                if (PREADY)
                begin
                    if (!PWRITE)
                    begin
                        // CAPTURE READ DATA
                        apb_read_data_out <= PRDATA;
                    end

                    state <= IDLE;
                end
                else
                begin
                    // WAIT UNTIL SLAVE IS READY
                    state <= ENABLE;
                end
            end

            default:
            begin
                state <= IDLE;
            end

        endcase
    end
end

// APB CONTROL SIGNALS
assign PENABLE = (state == ENABLE);

// ADDRESS DECODING
// PADDR[8] = 0 : SLAVE 1
// PADDR[8] = 1 : SLAVE 2

assign PSEL1 = ((state != IDLE) && (PADDR[8] == 1'b0));

assign PSEL2 = ((state != IDLE) && (PADDR[8] == 1'b1));

// NO ERROR
assign PSLVERR = 1'b0;

endmodule

