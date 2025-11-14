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


localparam integer BOX_COUNT = 3;      // number of boxes (set to 3 for your request)
localparam integer BOX_W    = 100;   // width of each box in pixels
localparam integer BOX_H    = 80;    // height of each box in pixels
localparam integer SPACING  = 40;    // pixels between the two boxes
localparam integer CENTER_X = 320;   // assumed center X of active area
localparam integer CENTER_Y = 240;   // assumed center Y of active area

localparam integer TOTAL_W = BOX_COUNT*BOX_W + (BOX_COUNT-1)*SPACING;
localparam integer LEFT_X  = CENTER_X - (TOTAL_W/2);
localparam integer Y0       = CENTER_Y - (BOX_H/2);
localparam integer Y1       = Y0 + BOX_H - 1;

// desired color for the boxes (24-bit: R[23:16], G[15:8], B[7:0])
localparam [23:0] BOX_COLOR0 = 24'hD2042D; // cherry red
localparam [23:0] BOX_COLOR2 = 24'hA2B06D; // leafy green
localparam [23:0] BOX_COLOR3 = 24'h5CA904; // darker forest green




integer i;
integer sx;
integer ex;

always @(*)

	begin
		vga_color = 24'hFFFFFF;
		
			// draw boxes only when in active pixel region
	if (active_pixels) begin
		// for each box compute its start/end and check if current x,y inside
		for (i = 0; i < BOX_COUNT; i = i + 1) begin
			sx = LEFT_X + i * (BOX_W + SPACING);
			ex = sx + BOX_W - 1;
			if ((x >= sx) && (x <= ex) && (y >= Y0) && (y <= Y1)) begin
			case(i)
				0:vga_color = BOX_COLOR0;
				1:vga_color = BOX_COLOR2;
				2:vga_color = BOX_COLOR3;
				default: vga_color = 24'h000000;
			endcase

				
			end
		end
	end
end
endmodule 	