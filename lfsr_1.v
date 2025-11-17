module lfsr_1(input clk,
					input rst,
					input [3:0]KEY,
					output [6:0]HEX0,
					output [3:0]lfsr_out);
	
	wire lfsr_clk;
	assign lfsr_clk = KEY[0];

lfsr my_lfsr(lfsr_out,lfsr_clk,rst);

seven_segment my_ss_lfsr(lfsr_out,HEX0);

endmodule