module lfsr(out,clk,rst);
	input clk, rst;
	output reg [3:0]out;
	
	always @ (posedge clk or negedge rst)
	begin	
		if (rst == 1'b0)
			out = 4'b0101;
		else 
			out = {out[2:0], ~(out[3] ^ out[2])};
	end
	
endmodule