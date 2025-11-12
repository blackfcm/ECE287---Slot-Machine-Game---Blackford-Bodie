module lfsr_1(input CLOCK_50,
					input [3:0]KEY,
					output [6:0]HEX0);
					
	wire [3:0]ran_num;
	wire slow_clk;
	
	slow_clock clk_div(.clk_in(CLOCK_50), .clk_out(slow_clk));
	
	lfsr my_lfsr(.clk(slow_clk), .rst(KEY[3]), .en_seed(KEY[0]), .ran_num(ran_num));
	
	seven_segment my_ss(.i(ran_num), .o(HEX0));
endmodule