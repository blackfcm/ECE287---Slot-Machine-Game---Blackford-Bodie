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
assign slot_rst = SW[1];

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

always @(*) begin
	{VGA_R, VGA_G, VGA_B} = vga_color;
end

reg [23:0] vga_color;

localparam SPRITE_W = 128;
localparam SPRITE_H = 128;
localparam CENTER_X = 320 - (SPRITE_W/2);
localparam CENTER_Y = 240 - (SPRITE_H/2);
localparam LEFT_X   = CENTER_X - 160;
localparam RIGHT_X  = CENTER_X + 160;
localparam NUM_SPRITES = 8;

//Title Parameters
localparam CHAR_W = 8;
localparam CHAR_H = 8;

localparam TITLE_SCALE = 3;
localparam TITLE_W = CHAR_W * TITLE_SCALE;
localparam TITLE_H = CHAR_H * TITLE_SCALE;

localparam TITLE_LEN = 14;
localparam TITLE_PIX_W = TITLE_LEN * TITLE_W;
localparam TITLE_X = (640 - TITLE_PIX_W) / 2;
localparam TITLE_Y = (CENTER_Y / 2) - (TITLE_H / 2);

wire [23:0] sprite_pixel;
wire [7:0] font_pixel;

reg [16:0] sprite_addr;
reg [7:0] font_ascii;
reg [3:0] font_row;


// TITLE TEXT
reg [7:0] title_text [0:13];  // 14 letters

// LFSR and Spinning Functions
reg [3:0]center_symbol;
reg spinning;
reg [23:0] spin_counter;

//---------------------------------------------------------
// Slow clock divider and edge-pulse generator for reels
//---------------------------------------------------------
reg [22:0] div;                 // widen for slower ticks if you want
reg prev_div15, prev_div16, prev_div17;
reg tickA_pulse_reg, tickB_pulse_reg, tickC_pulse_reg;

always @(posedge clk or negedge rst) begin
    if (!rst) begin
        div <= 0;
        prev_div15 <= 0;
        prev_div16 <= 0;
        prev_div17 <= 0;
        tickA_pulse_reg <= 0;
        tickB_pulse_reg <= 0;
        tickC_pulse_reg <= 0;
    end else begin
        div <= div + 1'b1;

        // detect rising edges of the chosen bits
        prev_div15 <= div[20];  // choose slower bits (20..22) for nicer speed
        prev_div16 <= div[21];
        prev_div17 <= div[22];

        tickA_pulse_reg <= div[20] & ~prev_div15; // single-cycle pulse when bit 20 rises
        tickB_pulse_reg <= div[21] & ~prev_div16; // single-cycle pulse when bit 21 rises
        tickC_pulse_reg <= div[22] & ~prev_div17; // single-cycle pulse when bit 22 rises
    end
end

// Active-high single-cycle pulses to use in logic
wire tickA_pulse = tickA_pulse_reg;
wire tickB_pulse = tickB_pulse_reg;
wire tickC_pulse = tickC_pulse_reg;

// optional: route pulses to LEDs for debugging (can remove later)
assign LEDR[1] = tickA_pulse | 1'b0;
assign LEDR[2] = tickB_pulse | 1'b0;
assign LEDR[3] = tickC_pulse | 1'b0;


wire [7:0] rnd_left; 
wire [7:0] rnd_center;
wire [7:0] rnd_right;

reg  [3:0] idx_left, idx_center, idx_right;  // final displayed symbols

reg key0_prev;

reg [7:0] stop_counter;
localparam MAX_TICKS = 8'd20; // number of center ticks before auto-stop

always @(posedge clk or negedge rst) begin
    if (!rst) begin
        key0_prev    <= 1;
        idx_center   <= 0;
        idx_left     <= 1;
        idx_right    <= 2;
        spinning     <= 0;
        stop_counter <= 0;

        start_left_pick   <= 0;
        start_center_pick <= 0;
        start_right_pick  <= 0;
    end 
    else begin
        // ------------------------------------------------
        // 1) DEFAULT — these must be zero every clock
        // ------------------------------------------------
        start_left_pick   <= 0;
        start_center_pick <= 0;
        start_right_pick  <= 0;

        // ------------------------------------------------
        // 2) CAPTURE KEY edge
        // ------------------------------------------------
        key0_prev <= KEY[0];

        if (key0_prev && !KEY[0]) begin
            spinning     <= 1;
            stop_counter <= 0;
        end

        // ------------------------------------------------
        // 3) REEL SPINNING ANIMATION (your original logic)
        // ------------------------------------------------
        if (spinning) begin
            if (tickA_pulse)
                idx_left <= rnd_left[2:0];

            if (tickB_pulse)
                idx_center <= rnd_center[2:0];

            if (tickC_pulse)
                idx_right <= rnd_right[2:0];
        end
			//---------------------------------------------------------
			// 3B) EARLY STOP WHEN ALL THREE ALIGN
			//---------------------------------------------------------
			//if (spinning) begin
			//	 if ((idx_left == idx_center) && (idx_center == idx_right)) begin
				//	  spinning <= 0;        // stop immediately
				 //end
			//end
        // ------------------------------------------------
        // 4) STOP COUNTER ADVANCE
        // ------------------------------------------------
        if (spinning && tickB_pulse)
            stop_counter <= stop_counter + 1;

        // ------------------------------------------------
        // 5) TRIGGER WEIGHTED PICKS (clean version)
        // ------------------------------------------------
        if (spinning && tickB_pulse) begin
            if (stop_counter == MAX_TICKS - 3)
                start_left_pick <= 1;

            if (stop_counter == MAX_TICKS - 2)
                start_center_pick <= 1;

            if (stop_counter == MAX_TICKS - 1)
                start_right_pick <= 1;
        end

        // ------------------------------------------------
        // 6) LOCK THE FINAL WEIGHTED SYMBOLS
        // ------------------------------------------------
        if (done_left)
            idx_left <= sym_left;

        if (done_center)
            idx_center <= sym_center;

        if (done_right)
            idx_right <= sym_right;

        // ------------------------------------------------
        // 7) STOP SPINNING
        // ------------------------------------------------
        if (stop_counter >= MAX_TICKS)
            spinning <= 0;

    end
end


initial begin
    title_text[0]  = "F";
    title_text[1]  = "o";
    title_text[2]  = "r";
    title_text[3]  = "t";
    title_text[4]  = "u";
    title_text[5]  = "n";
    title_text[6]  = "e";
    title_text[7]  = " ";
    title_text[8]  = "F";
    title_text[9]  = "r";
    title_text[10] = "e";
    title_text[11] = "n";
    title_text[12] = "z";
    title_text[13] = "y";
end


sprites_rom icons(clk,sprite_addr, sprite_pixel);
title_rom title(clk, font_ascii, font_row, font_pixel);

lfsr_1 left(clk,rst, rnd_left);
lfsr_2 center(clk,rst, rnd_center);
lfsr_3 right(clk,rst, rnd_right);

wire [3:0] sym_left, sym_center, sym_right;
wire done_left, done_center, done_right;

reg start_left_pick, start_center_pick, start_right_pick;

weighted_picker_fsm w_left(clk,rst,start_left_pick,rnd_left,sym_left,done_left);
weighted_picker_fsm w_center(clk,rst,start_center_pick,rnd_center,sym_center,done_center);
weighted_picker_fsm w_right(clk,rst,start_right_pick,rnd_right,sym_right,done_right);

reg [3:0] slot_pos;
reg [23:0] spin_cnt;


reg [7:0] char_index;
reg [2:0] char_x;
reg [2:0] char_y;

reg [16:0] addr_left, addr_center, addr_right;



always @(*) begin
    vga_color = 24'h000000;
    sprite_addr = 17'h0;
	 font_ascii = 8'h00;
	 font_row = 4'h0;

    // DEFAULT assignments for *all* combinational regs:
    addr_center = 0;
    addr_left   = 0;
    addr_right  = 0;
    char_index  = 0;
    char_x      = 0;
    char_y      = 0;
	
	if (active_pixels)
	begin
    // CENTER
         if (x >= CENTER_X && x < CENTER_X + SPRITE_W &&
				 y >= CENTER_Y && y < CENTER_Y + SPRITE_H) begin
				 
					  addr_center = idx_center * SPRITE_W * SPRITE_H +
										 (y - CENTER_Y) * SPRITE_W +
										 (x - CENTER_X);
					  sprite_addr = addr_center;
					  vga_color   = sprite_pixel;
				 end
			

          // LEFT
          else if (x >= LEFT_X && x < LEFT_X + SPRITE_W &&
                   y >= CENTER_Y && y < CENTER_Y + SPRITE_H) begin

                  addr_left  = idx_left * SPRITE_W * SPRITE_H +
                               (y - CENTER_Y) * SPRITE_W +
                               (x - LEFT_X);
                  sprite_addr = addr_left;
                  vga_color   = sprite_pixel;
              end

    

          // RIGHT
          else if (x >= RIGHT_X && x < RIGHT_X + SPRITE_W &&
                   y >= CENTER_Y && y < CENTER_Y + SPRITE_H) begin

                  addr_right = idx_right * SPRITE_W * SPRITE_H +
                               (y - CENTER_Y) * SPRITE_W +
                               (x - RIGHT_X);
                  sprite_addr = addr_right;
                  vga_color   = sprite_pixel;
              end

      

          // TITLE
          else if (x >= TITLE_X && x < TITLE_X + TITLE_PIX_W &&
                   y >= TITLE_Y && y < TITLE_Y + TITLE_H) begin

              char_index = (x - TITLE_X) / TITLE_W;
              char_x     = ((x - TITLE_X) % TITLE_W) / TITLE_SCALE;
              char_y     = ((y - TITLE_Y) % TITLE_H) / TITLE_SCALE;

              // protect out-of-range read (in case of timing)
              if (char_index < TITLE_LEN) begin
                  font_ascii = title_text[char_index];
              end else begin
                  font_ascii = 8'h20; // space
              end

              font_row = char_y[2:0];

              if (font_pixel[7 - char_x])
                  vga_color = 24'hFFFFFF; // white text
          end
      end
end

endmodule