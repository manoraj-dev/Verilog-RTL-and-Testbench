`timescale 1ns/1ns

module galois_lfsr(
    input clk,
    input arst_n,
    output reg [2:0] q
);

    wire feedback;
	 wire sync_rstn;

    // Feedback equation
    assign feedback = q[1]^ q[2];
	
	 synchronizer u0(.clk(clk), .qin(arst_n), .qout(sync_rstn));

    // Fibonacci LFSR operation
    always @(posedge clk)
    begin
        if (!sync_rstn)
         q <= 8'b001;
        else
        begin
        		q[2] <= q[1] ^ q[2];
        		q[1] <= q[0];
        		q[0] <= q[2];
        end 
		  end
endmodule

//2-stage synchronizer
module synchronizer(
	input  clk,
	input  qin,
	output qout
);
	reg  temp_reg0;
	reg  temp_reg1;
always @(posedge clk )
begin
	//2stage flops in series
		temp_reg0 <= qin;
		temp_reg1 <= temp_reg0;
end
assign qout=temp_reg1; //direct assign to output from temp_reg1
endmodule
