module vga_driver_memory	(
  	//////////// ADC //////////
	//output		          		ADC_CONVST,
	//output		          		ADC_DIN,
	//input 		          		ADC_DOUT,
	//output		          		ADC_SCLK,

	//////////// Audio //////////
	//input 		          		AUD_ADCDAT,
	//inout 		          		AUD_ADCLRCK,
	//inout 		          		AUD_BCLK,
	//output		          		AUD_DACDAT,
	//inout 		          		AUD_DACLRCK,
	//output		          		AUD_XCK,

	//////////// CLOCK //////////
	//input 		          		CLOCK2_50,
	//input 		          		CLOCK3_50,
	//input 		          		CLOCK4_50,
	input 		          		CLOCK_50,

	//////////// SDRAM //////////
	//output		    [12:0]		DRAM_ADDR,
	//output		     [1:0]		DRAM_BA,
	//output		          		DRAM_CAS_N,
	//output		          		DRAM_CKE,
	//output		          		DRAM_CLK,
	//output		          		DRAM_CS_N,
	//inout 		    [15:0]		DRAM_DQ,
	//output		          		DRAM_LDQM,
	//output		          		DRAM_RAS_N,
	//output		          		DRAM_UDQM,
	//output		          		DRAM_WE_N,

	//////////// I2C for Audio and Video-In //////////
	//output		          		FPGA_I2C_SCLK,
	//inout 		          		FPGA_I2C_SDAT,

	//////////// SEG7 //////////
	output		     [6:0]		HEX0,
	output		     [6:0]		HEX1,
	output		     [6:0]		HEX2,
	output		     [6:0]		HEX3,
	//output		     [6:0]		HEX4,
	//output		     [6:0]		HEX5,

	//////////// IR //////////
	//input 		          		IRDA_RXD,
	//output		          		IRDA_TXD,

	//////////// KEY //////////
	input 		     [3:0]		KEY,

	//////////// LED //////////
	output		     [9:0]		LEDR,

	//////////// PS2 //////////
	//inout 		          		PS2_CLK,
	//inout 		          		PS2_CLK2,
	//inout 		          		PS2_DAT,
	//inout 		          		PS2_DAT2,

	//////////// SW //////////
	input 		     [9:0]		SW,

	//////////// Video-In //////////
	//input 		          		TD_CLK27,
	//input 		     [7:0]		TD_DATA,
	//input 		          		TD_HS,
	//output		          		TD_RESET_N,
	//input 		          		TD_VS,

	//////////// VGA //////////
	output		          		VGA_BLANK_N,
	output reg	     [7:0]		VGA_B,
	output		          		VGA_CLK,
	output reg	     [7:0]		VGA_G,
	output		          		VGA_HS,
	output reg	     [7:0]		VGA_R,
	output		          		VGA_SYNC_N,
	output		          		VGA_VS

	//////////// GPIO_0, GPIO_0 connect to GPIO Default //////////
	//inout 		    [35:0]		GPIO_0,

	//////////// GPIO_1, GPIO_1 connect to GPIO Default //////////
	//inout 		    [35:0]		GPIO_1

);

  // Turn off all displays.
	assign	HEX0		=	7'h00;
	assign	HEX1		=	7'h00;
	assign	HEX2		=	7'h00;
	assign	HEX3		=	7'h00;

wire active_pixels; // is on when we're in the active draw space

wire [9:0]x; // current x
wire [9:0]y; // current y - 10 bits = 1024 ... a little bit more than we need

wire clk;
wire rst;

assign clk = CLOCK_50;
assign rst = SW[0];

assign LEDR[0] = active_pixels;
//assign LEDR[1] = flag;

vga_driver the_vga(
.clk(clk),
.rst(rst),

.vga_clk(VGA_CLK),

.hsync(VGA_HS),
.vsync(VGA_VS),

.active_pixels(active_pixels),

.xPixel(x),
.yPixel(y),

.VGA_BLANK_N(VGA_BLANK_N),
.VGA_SYNC_N(VGA_SYNC_N)
);

always @(*)
begin
	{VGA_R, VGA_G, VGA_B} = vga_color;
end


reg [23:0] vga_color;

localparam SPRITE_W = 128;
localparam SPRITE_H = 128;

// === Cherry sprite parameters === (middle)
localparam CENTER_X = 320 - (SPRITE_W/2);  // centered on screen
localparam CENTER_Y = 240 - (SPRITE_H/2);
// Diamond parameter (left)
localparam LEFT_X = CENTER_X - 160;
// Seven parameter (right)
localparam RIGHT_X = CENTER_X + 160;



wire cherry_inside;
wire [23:0] cherry_pixel;
wire [23:0] diamond_pixel;
wire [23:0] seven_pixel;
wire [23:0] orange_pixel;
wire [23:0] grape_pixel;
wire [23:0] bell_pixel;
wire [23:0] clover_pixel;
wire [23:0] watermelon_pixel;
wire [23:0] bar_pixel;

reg  [13:0] cherry_addr;
reg  [13:0] diamond_addr;
reg  [13:0] seven_addr;
reg [13:0] orange_addr;
reg [13:0] grape_addr;
reg [13:0] bell_addr;
reg [13:0] clover_addr;
reg [13:0] watermelon_addr;
reg [13:0] bar_addr;


cherry_rom cherry_image (
    .clk(clk),
    .addr(cherry_addr),
    .pixel(cherry_pixel)
);

diamond_rom diamond_image (
    .clk(clk),
    .addr(diamond_addr),
    .pixel(diamond_pixel)
);

seven_rom seven_image (
    .clk(clk),
    .addr(seven_addr),
    .pixel(seven_pixel)
);

orange_rom orange_image(clk,orange_addr,orange_pixel);

grape_rom grape_image(clk,grape_addr,grape_pixel);

bell_rom bell_image(clk,bell_addr,bell_pixel);

clover_rom clover_image(clk,clover_addr, clover_pixel);

watermelon_rom watermelon_image(clk, watermelon_addr, watermelon_pixel);

bar_rom bar_image (clk, bar_addr, bar_pixel);

	 
integer i;
integer sx;
integer ex;
integer sy;


always @(*) begin
    vga_color = 24'h000000;   // background black
    cherry_addr  = 0;
    diamond_addr = 0;
    seven_addr   = 0;

    if (active_pixels) begin

        // CENTER
        if (x >= CENTER_X && x < CENTER_X + SPRITE_W &&
            y >= CENTER_Y && y < CENTER_Y + SPRITE_H) begin

            bar_addr = (y - CENTER_Y) * SPRITE_W +
                          (x - CENTER_X);
            vga_color = bar_pixel;

        end 

        // LEFT
        else if (x >= LEFT_X && x < LEFT_X + SPRITE_W &&
                 y >= CENTER_Y && y < CENTER_Y + SPRITE_H) begin

            clover_addr = (y - CENTER_Y) * SPRITE_W +
                           (x - LEFT_X);
            vga_color = clover_pixel;

        end 

        // === SEVEN (right) ===
        else if (x >= RIGHT_X && x < RIGHT_X + SPRITE_W &&
                 y >= CENTER_Y && y < CENTER_Y + SPRITE_H) begin

            watermelon_addr = (y - CENTER_Y) * SPRITE_W +
                         (x - RIGHT_X);
            vga_color = watermelon_pixel;

        end
    end
end

	 



endmodule 