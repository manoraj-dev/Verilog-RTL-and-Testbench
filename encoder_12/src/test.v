`timescale 1ns/1ns

module encoder_8to3 (
    input [7:0] D,
    output reg [2:0] Y,
    output reg valid
);

always @(*) begin
    case (D)
        8'b00000001: begin Y = 3'b000; valid = 1; end
        8'b00000010: begin Y = 3'b001; valid = 1; end
        8'b00000100: begin Y = 3'b010; valid = 1; end
        8'b00001000: begin Y = 3'b011; valid = 1; end
        8'b00010000: begin Y = 3'b100; valid = 1; end
        8'b00100000: begin Y = 3'b101; valid = 1; end
        8'b01000000: begin Y = 3'b110; valid = 1; end
        8'b10000000: begin Y = 3'b111; valid = 1; end

        default: begin
            Y = 3'b000;
            valid = 0;
        end
    endcase
end

endmodule
