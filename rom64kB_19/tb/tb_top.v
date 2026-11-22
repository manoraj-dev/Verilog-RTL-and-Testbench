`timescale 1ns/1ns

`include "../src/dut.v"

module tb_top;
reg clk; //clk
  reg en; //enable
  reg [3:0] addr; //address
  wire [3:0] data; //output data
  
  rom_64kb dut(
    .clk(clk),
    .en(en),
    .addr(addr),
    .data(data)
  );
  
  initial
    begin
      $dumpfile("waveform.vcd");
      $dumpvars(0, tb_top);       
      
      clk=1'b1;
      forever #5 clk = ~clk;
    end
  
  initial
    begin
      en = 1'b0;
      #10;                  
      
      en = 1'b1;      
      addr = 4'b1010;
      #10;
      
      addr = 4'b0110;
      #10;
      
      addr = 4'b0011;
      #10;
      
      en = 1'b0;
      addr = 4'b1111;
      #10;
            
      en = 1'b1;
      addr = 4'b1000;
      #10;
      
      addr = 4'b0000;
      #10;
      
      addr = 4'bxxxx;
      #10;
    end
  
  initial
    begin
      #80 $finish;
    end
  
endmodule  



