module slow_clock(
    input clk_in,
    output reg clk_out
);
    reg [25:0] counter;

    always @(posedge clk_in) 
	 begin
        counter <= counter + 1;
        clk_out <= counter[25];  // ~0.75 Hz at 50 MHz
    end
endmodule
