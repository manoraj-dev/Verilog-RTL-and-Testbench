`timescale 1ns/1ns

module moore_fsm_non_overlapping_1001 (
    input clk,
    input reset,
    input x,
    output reg z,
    output reg [2:0] present_state, 
	 output reg [2:0] next_state
);

parameter S0 = 3'b000;
parameter S1 = 3'b001;
parameter S2 = 3'b010;
parameter S3 = 3'b011;
parameter S4 = 3'b100;
reg [2:0] present_state, next_state;


// Present state block
always @(posedge clk)
begin
    if (reset)
        present_state <= S0;
    else
        present_state <= next_state;
end


// Next state block
always @(*)
begin
    case (present_state)

        S0:
            if (x == 1'b1)
                next_state = S1;
            else
                next_state = S0;

        S1:
            if (x == 1'b0)
                next_state = S2;
            else
                next_state = S1;

        S2:
            if (x == 1'b0)
                next_state = S3;
            else
                next_state = S1;

        S3:
            if (x == 1'b1)
                next_state = S4;
            else
                next_state = S0;

        S4:
            if (x == 1'b0)
                next_state = S0;
            else
                next_state = S1;

        default:
            next_state = S0;

    endcase
end


// Output block (Moore model)
always @(*)
begin
    if (present_state == S4)
        z = 1'b1;
    else
        z = 1'b0;
end
 
endmodule
