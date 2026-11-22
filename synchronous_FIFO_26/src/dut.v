`timescale 1ns/1ns

module sync_fifo (
    input clk,
    input rst_n,
    input w_en,
    input r_en,
    input [7:0] data_in,
    output reg [7:0] data_out,
    output full,
    output empty
);


reg [7:0] memory [0:15];
reg [3:0] wr_ptr;
reg [3:0] rd_ptr;
reg [4:0] count;

// Full and empty conditions
assign full  = (count == 16);
assign empty = (count == 0);

// FIFO write and read operation
always @(posedge clk or negedge rst_n)
begin

    if (!rst_n)
    begin
        wr_ptr  <= 0;
        rd_ptr  <= 0;
        count   <= 0;
        data_out <= 0;
    end

    else
    begin

        // Write operation
        if (w_en && !full)
        begin
            memory[wr_ptr] <= data_in;

            if (wr_ptr == 15)
                wr_ptr <= 0;
            else
                wr_ptr <= wr_ptr + 1'b1;
        end

        // Read operation
        if (r_en && !empty)
        begin
            data_out <= memory[rd_ptr];

            if (rd_ptr == 15)
                rd_ptr <= 0;
            else
                rd_ptr <= rd_ptr + 1'b1;
        end

        // Update FIFO count
        case ({w_en && !full, r_en && !empty})

            2'b10: count <= count + 1'b1;

            2'b01: count <= count - 1'b1;

            2'b11: count <= count;

            default: count <= count;

        endcase

    end

end

endmodule

