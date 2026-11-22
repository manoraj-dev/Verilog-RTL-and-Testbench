`timescale 1ns/1ns

module arbiter(
    input  [3:0] req,      
    output reg  [3:0] grant,
    output reg        irq_valid,
    output reg  [1:0] irq_id
);

    always @(*) begin
        grant     = 4'b0000;
        irq_valid = 1'b0;
        irq_id    = 2'b00;

           if (req[0]) begin
            grant     = 4'b0001;
            irq_valid = 1'b1;
            irq_id    = 2'b00;
        end

        else if (req[1]) begin
            grant     = 4'b0010;
            irq_valid = 1'b1;
            irq_id    = 2'b01;
        end

        else if (req[2]) begin
            grant     = 4'b0100;
            irq_valid = 1'b1;
            irq_id    = 2'b10;
        end

        else if (req[3]) begin
            grant     = 4'b1000;
            irq_valid = 1'b1;
            irq_id    = 2'b11;
        end
    end
endmodule

