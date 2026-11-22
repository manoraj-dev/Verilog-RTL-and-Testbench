`timescale 1ns/1ns

module priority_encoder (
    input [3:0]A,
    output reg VALID,
    output reg Y1, Y0
);

always @(*) begin
casex(A)
 
4'b1xxx : begin
     Y = 2'b11;
     VALID = 1'b1;
end

4'b01xx : begin
     Y = 2'b10;
     VALID = 1'b1;
4'b001x : begin
     Y = 2'b01;
     VALID = 1'b1;
 end

4'b0001 : begin
     Y = 2'b00;
     VALID = 1'b1;
  
end

endmodule

