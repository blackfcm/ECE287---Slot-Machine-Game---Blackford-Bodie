module orange_rom(
    input  wire        clk,
    input  wire [13:0] addr,
    output reg  [23:0] pixel   // 24-bit RGB
);
    reg [23:0] mem [0:16383];  // Example 32×32 image

    initial begin
        $readmemh("orange_128.mem", mem);
    end

    always @(posedge clk) begin
        pixel <= mem[addr];
    end

endmodule