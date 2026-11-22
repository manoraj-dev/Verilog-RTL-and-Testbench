`timescale 1ns/1ns

`include "../src/dut.v"

module tb_top;

parameter DATA_WIDTH=8, 
		  ADDR_WIDTH=8,  
		  NUM_INTR=16, 
     	  INTR_SERV=4, 
		  MAX_DELAY=30, 
		  TIME_PERIOD=2; 

reg pclk_i, pwrite_i, penable_i, intr_serviced_i, preset_i, psel_i;
reg [DATA_WIDTH-1 : 0] pwdata_i;
reg [ADDR_WIDTH-1 : 0] paddr_i;
reg [NUM_INTR-1 : 0] intr_active_i;
wire pready_o, perror_o, intr_valid_o;
wire [DATA_WIDTH-1 : 0] prdata_o;
wire [INTR_SERV-1 : 0] intr_to_service_o;

reg [INTR_SERV-1 : 0] random_priority_array [NUM_INTR-1 : 0]; 
reg [30*8 : 1] testname;
integer i;

interrupt_controller #(.DATA_WIDTH(DATA_WIDTH), .ADDR_WIDTH(ADDR_WIDTH), .NUM_INTR(NUM_INTR), .INTR_SERV(INTR_SERV)) u0 (.*);

initial begin
	pclk_i = 0;
	forever #(TIME_PERIOD/2.0) pclk_i = ~pclk_i;
end

initial begin

	$value$plusargs("testname=%s",testname);
	

	pwrite_i = 0;
	penable_i = 0;
	intr_serviced_i = 0;
	psel_i = 0;
	pwdata_i = 0;
	paddr_i = 0;
	intr_active_i = 0;
	for (i=0; i<NUM_INTR; i=i+1) random_priority_array[i] = 0;

	// Hold and release reset
	preset_i = 1;
	repeat (5) @ (posedge pclk_i);
	preset_i = 0;
    
	randomizer(); 
    for (i=0; i<NUM_INTR; i=i+1) begin 
		if (testname == "ascending_priority") write_intc(i, i); 
		else if (testname == "descending_priority") write_intc(i, NUM_INTR-1-i); 
		else if (testname == "random_priority") write_intc(i, random_priority_array[i]);
		else begin
			$display("*** Error testname ***");
			i = NUM_INTR;
		end
	end
	intr_active_i = $random;
	#300;
	intr_active_i = $random;
	#300;
	intr_active_i = $random;
	#300;
	$finish;
end

initial begin
	forever begin
		@(posedge pclk_i);
		if (intr_valid_o == 1) begin
			#($urandom_range(1,MAX_DELAY)); 
			intr_active_i[intr_to_service_o] = 0;   
			intr_serviced_i = 1;
			@(posedge pclk_i);
			intr_serviced_i = 0;
		end
	end
end

task write_intc (input reg [ADDR_WIDTH-1 : 0] addr, input reg [DATA_WIDTH-1 : 0]data); 
begin
	paddr_i = addr;
	pwdata_i = data;
	pwrite_i = 1;
	psel_i = 1;
	penable_i = 1;
	wait (pready_o == 1);
	@(posedge pclk_i);
	paddr_i = 0;
	pwdata_i = 0;
	pwrite_i = 0;
	psel_i = 0;
	penable_i = 0;
end
endtask

task randomizer();
reg [INTR_SERV-1 : 0] temp;
reg unique_flag;
integer j, k;
begin
	for (j=0; j<NUM_INTR-1; ) begin
		temp = $urandom;
		unique_flag = 1;
		for (k=0; k<j; k=k+1) begin
			if (random_priority_array[k] == temp) begin
				unique_flag = 0;
				k = j;
			end
		end
		if (unique_flag == 1) begin
			random_priority_array[j] = temp;
			j=j+1;
		end
	end	
	//for (j=0; j<NUM_INTR-1; j=j+1) $display("Port-%d Priority-%d",j,random_priority_array[j]);
end
endtask

initial begin
	$dumpfile("waveform.vcd");
	$dumpvars(0, tb_top);
end

endmodule
