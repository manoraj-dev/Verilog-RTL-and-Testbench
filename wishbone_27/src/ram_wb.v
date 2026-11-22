`timescale 1ns/1ns

module wb_ram #(parameter DATA_WIDTH = 32, parameter ADDR_WIDTH =16 , parameter SELECT_WIDTH=(DATA_WIDTH/8) )

( input wire clk , input wire [ADDR_WIDTH -1:0] add_i ,input wire [DATA_WIDTH -1:0] dat_i , input wire [SELECT_WIDTH -1:0]sel_i ,input wire we_i,
input wire stb_i,input wire cyc_i, input add_i_valid, 
output wire [DATA_WIDTH-1:0] dat_o, output wire ack_o // acknowledge output 
 );

// for interfaces that are more than one word wide, disable address lines
parameter VALID_ADDR_WIDTH = ADDR_WIDTH - $clog2(SELECT_WIDTH);

parameter WORD_WIDTH = SELECT_WIDTH;

parameter WORD_SIZE = DATA_WIDTH/WORD_WIDTH;

//internal register

reg [DATA_WIDTH-1:0] dat_o_reg = { DATA_WIDTH{1'b0}};

reg ack_o_reg =1'b0;
//memory allocation to ram

reg [DATA_WIDTH -1:0] mem[ (2**VALID_ADDR_WIDTH)-1:0];

//address conversion

assign add_i_valid = add_i >> 2 ; 

assign dat_o=dat_o_reg;
assign ack_o=ack_o_reg;

integer i,j;

initial begin

    for (i = 0; i < 2**VALID_ADDR_WIDTH;i = i + 2**(VALID_ADDR_WIDTH/2)) begin

     for (j = i; j < i + 2**(VALID_ADDR_WIDTH/2);j = j + 1) begin

            mem[j] = 0;

        end
    end
end

always @(posedge clk) begin
    ack_o_reg <= 1'b0;
    for (i = 0; i < WORD_WIDTH; i = i + 1) begin
        if (cyc_i & stb_i & ~ack_o) begin
            if (we_i & sel_i[i]) begin
                mem[add_i_valid][WORD_SIZE*i +: WORD_SIZE] <= dat_i[WORD_SIZE*i +: WORD_SIZE];
            end
            dat_o_reg[WORD_SIZE*i +: WORD_SIZE] <= mem[add_i_valid][WORD_SIZE*i +: WORD_SIZE];
            ack_o_reg <= 1'b1;
        end
    end
end

endmodule
