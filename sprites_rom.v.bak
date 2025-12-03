module sprites_rom(
    input wire clk,
    input wire [16:0] addr,  // 131072 addresses: 0 to 131071
    output reg [23:0] pixel  // 24-bit color output
);

    reg [23:0] memory_array [0:131071];
	 integer  offset;
	 localparam CLOVER_SIZE = 128*128;
	 localparam WATERMELON_SIZE = 128*128;
	 localparam BAR_SIZE = 128*128;
	 localparam BELL_SIZE = 128*128;
	 localparam CHERRY_SIZE = 128*128;
	 localparam DIAMOND_SIZE = 128*128;
	 localparam SEVEN_SIZE = 128*128;
	 localparam ORANGE_SIZE = 128*128;
	 localparam GRAPE_SIZE = 128*128;






initial begin
    offset = 0;
    $readmemh("clover_128x128.mem",    memory_array, offset); 
    offset = offset + CLOVER_SIZE;

    $readmemh("watermelon_128x128.mem", memory_array, offset);
    offset = offset + WATERMELON_SIZE;

    $readmemh("bell_128x128.mem",       memory_array, offset);
    offset = offset + BELL_SIZE;

    $readmemh("bar_128x128.mem",        memory_array, offset);
    offset = offset + BAR_SIZE;

    $readmemh("cherry_128.mem",         memory_array, offset);
    offset = offset + CHERRY_SIZE;

    $readmemh("diamond_fixed_24bit.mem", memory_array, offset);
    offset = offset + DIAMOND_SIZE;

    $readmemh("seven_128x128.mem",      memory_array, offset);
    offset = offset + SEVEN_SIZE;

    $readmemh("orange_128.mem",         memory_array, offset);
    offset = offset + ORANGE_SIZE;


end



	  
    

    always @(posedge clk) begin
        pixel <= memory_array[addr];
    end
endmodule