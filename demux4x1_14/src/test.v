`timescale 1ns/1ns
module demux_1x4 (
    input din,            
    input [1:0] sel,       
    output reg [3:0] dout  
);

    always @(*) begin
       
        dout = 4'b0000; 
        
        case (sel)
            2'b00: dout[0] = din;
            2'b01: dout[1] = din;
            2'b10: dout[2] = din;
            2'b11: dout[3] = din;
            default: dout = 4'b0000;
        endcase
    end

endmodule

