`timescale 1ns/1ns

`include "../src/dut.v"

module tb_top;

reg clk;
reg reset;
reg x;
wire z;
wire [2:0] present_state, next_state; 
moore_fsm_1011 dut (.clk(clk),.reset(reset),.x(x),.z(z),.present_state(present_state),.next_state(next_state));

always #5 clk = ~clk;
initial
    begin
        $dumpfile("waveform.vcd");
        $dumpvars(0, tb_top);
    end
 
initial
  	begin
        clk = 0;
        reset = 1;
        x = 0;
		  #20;
        reset = 0;
		  repeat(200) begin
				@(posedge clk);
				x=$random();
			end
	#20;
	$finish;
end 

initial 
  begin
        #1;
        $monitor("Time=%0t | Reset=%b | X=%b | Z=%b | present_state=%b | next_state=%b", $time, reset, x, z,present_state,next_state);
    end
endmodule 

