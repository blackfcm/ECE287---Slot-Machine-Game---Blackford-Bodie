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
wire slot_rst;

assign clk = CLOCK_50;
assign rst = SW[0];
assign slot_rst = SW[1];

assign LEDR[0] = active_pixels;

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

localparam TITLE_SCALE = 4;
localparam TITLE_W = CHAR_W * TITLE_SCALE;
localparam TITLE_H = CHAR_H * TITLE_SCALE;

localparam TITLE_LEN = 14;
localparam TITLE_PIX_W = TITLE_LEN * TITLE_W;
localparam TITLE_X = (640 - TITLE_PIX_W) / 2;
localparam TITLE_Y = (CENTER_Y / 2) - (TITLE_H / 2);

// "You Win" Parameters
localparam WIN_MSG = 8;
localparam WIN_LEN = 8;
localparam WIN_X = (640 - (WIN_LEN * TITLE_W)) / 2;
localparam WIN_Y = 340;

wire [23:0] sprite_pixel;
wire [7:0] font_pixel;

reg [16:0] sprite_addr;
reg [7:0] font_ascii;
reg [3:0] font_row;
reg win_flag;

// WIN TEXT
reg [7:0] win_text [0:7];


// TITLE TEXT
reg [7:0] title_text [0:13];  // 14 letters

// LFSR and Spinning Functions
reg spinning;
reg [23:0] spin_counter;

// CLOCK DIVIDER FOR SLOTS
reg [22:0] div;                 
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

        // Speeds of spinning
        prev_div15 <= div[20];  
        prev_div16 <= div[21];
        prev_div17 <= div[22];

        tickA_pulse_reg <= div[20] & ~prev_div15; 
        tickB_pulse_reg <= div[21] & ~prev_div16; 
        tickC_pulse_reg <= div[22] & ~prev_div17; 
    end
end


wire tickA_pulse = tickA_pulse_reg;
wire tickB_pulse = tickB_pulse_reg;
wire tickC_pulse = tickC_pulse_reg;

// LFSR AND SPINNING LOGIC REGs and WIREs

wire [7:0] rnd_left; 
wire [7:0] rnd_center;
wire [7:0] rnd_right;

reg  [3:0] idx_left, idx_center, idx_right;  

reg key0_prev;
reg key1_prev;

reg [7:0] stop_counter;
localparam MAX_TICKS = 8'd20; 


// LFSR AND SPINNING LOGIC

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
        
        start_left_pick   <= 0;
        start_center_pick <= 0;
        start_right_pick  <= 0;

        
        key0_prev <= KEY[0];

        if (key0_prev && !KEY[0]) begin
            spinning     <= 1;
            stop_counter <= 0;
				win_flag <= 0;
        end

       
        // SLOT SPINNING 
       
        if (spinning) begin
            if (tickA_pulse)
                idx_left <= rnd_left[2:0];

            if (tickB_pulse)
                idx_center <= rnd_center[2:0];

            if (tickC_pulse)
                idx_right <= rnd_right[2:0];
        end
			
			// EARLY STOP WHEN ALL THREE ALIGN (IF NEEDED TO DEMO)
			
			if (spinning) begin
				if ((idx_left == idx_center) && (idx_center == idx_right)) begin
					  spinning <=  0;        // stop immediately
				 end
			end
       
        // STOP COUNTER
       
        if (spinning && tickB_pulse)
            stop_counter <= stop_counter + 1;

        
        // ENABLE WEIGHTED PICKS 
       
        if (spinning && tickB_pulse) begin
            if (stop_counter == MAX_TICKS - 3)
                start_left_pick <= 1;

            if (stop_counter == MAX_TICKS - 2)
                start_center_pick <= 1;

            if (stop_counter == MAX_TICKS - 1)
                start_right_pick <= 1;
        end

       
        // LOCK THE FINAL WEIGHTED SYMBOLS
       
        if (done_left)
            idx_left <= sym_left;

        if (done_center)
            idx_center <= sym_center;

        if (done_right)
            idx_right <= sym_right;

        
        // STOP SPINNING
       
        if (stop_counter >= MAX_TICKS)
            spinning <= 0;
				
				if((idx_left == idx_center) && (idx_center == idx_right))
					win_flag <= 1;
				else 
					win_flag <= 0;

    end
end


// TITLE TEXTS

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

initial begin
	win_text [0] = "Y";
	win_text [1] = "O";
	win_text [2] = "U";
	win_text [3] = " ";
	win_text [4] = "W";
	win_text [5] = "I";
	win_text [6] = "N";
	win_text [7] = "!";
end


// DRAWING and INSTATION REGs and WIREs

wire [3:0] sym_left, sym_center, sym_right;
wire done_left, done_center, done_right;

reg start_left_pick, start_center_pick, start_right_pick;
reg [7:0] char_index;
reg [2:0] char_x;
reg [2:0] char_y;
reg [16:0] addr_left, addr_center, addr_right;


// BET AND PAYOUT
reg [6:0] bet_amount;

localparam BET_LEN = 7;
localparam BET_X   = 20;
localparam BET_Y   = 420;
localparam PAY_LEN = 7;
localparam PAY_X   = 640 - (PAY_LEN * TITLE_W) - 20;
localparam PAY_Y   = 420;

reg key1_prev_reg;
wire key1_falling;

// BET ALWAYS BLOCKS
always @(posedge clk or negedge rst) begin
    if (!rst)
        key1_prev_reg <= 1;
    else
        key1_prev_reg <= KEY[1];
end

assign key1_falling = key1_prev_reg & ~KEY[1];

always @(posedge clk or negedge rst) begin
    if (!rst)
        bet_amount <= 7'd1;
    else if (key1_falling) begin
        if (bet_amount < 7'd100)
            bet_amount <= bet_amount + 1;
        else
            bet_amount <= 7'd1;
    end
end

// PAYOUT ALWAYS BLOCKS
reg [9:0] payout_reg;  // new register for payout
always @(posedge clk or negedge rst) begin
    if (!rst)
        payout_reg <= 10'd0;            // reset payout
    else if (win_flag)
        payout_reg <= bet_amount * 5;   // update payout only on win
    else if (spinning)
        payout_reg <= 10'd0;            // reset payout when spinning starts
end



// BET DIGITS
wire [3:0] bet_hundreds = (bet_amount >= 100) ? 1 : 0;
wire [6:0] bet_after_h  = bet_amount - (bet_hundreds ? 100 : 0);

wire [3:0] bet_tens = (bet_after_h >= 90) ? 9 :
                      (bet_after_h >= 80) ? 8 :
                      (bet_after_h >= 70) ? 7 :
                      (bet_after_h >= 60) ? 6 :
                      (bet_after_h >= 50) ? 5 :
                      (bet_after_h >= 40) ? 4 :
                      (bet_after_h >= 30) ? 3 :
                      (bet_after_h >= 20) ? 2 :
                      (bet_after_h >= 10) ? 1 : 0;

wire [3:0] bet_ones = bet_after_h - (bet_tens * 10);


// PAYOUT DIGITS
wire [3:0] p_hundreds =
    (payout_reg >= 500) ? 5 :
    (payout_reg >= 400) ? 4 :
    (payout_reg >= 300) ? 3 :
    (payout_reg >= 200) ? 2 :
    (payout_reg >= 100) ? 1 : 0;

wire [9:0] p_after_h = payout_reg - (p_hundreds * 100);

wire [3:0] p_tens = (p_after_h >= 90) ? 9 :
                    (p_after_h >= 80) ? 8 :
                    (p_after_h >= 70) ? 7 :
                    (p_after_h >= 60) ? 6 :
                    (p_after_h >= 50) ? 5 :
                    (p_after_h >= 40) ? 4 :
                    (p_after_h >= 30) ? 3 :
                    (p_after_h >= 20) ? 2 :
                    (p_after_h >= 10) ? 1 : 0;

wire [3:0] p_ones = p_after_h - (p_tens * 10);

reg [7:0] bet_text [0:6];     
reg [7:0] pay_text [0:6];     

always @(*) begin
    bet_text[0] = "B";
	 bet_text[1] = "E";
	 bet_text[2] = "T";
	 bet_text[3] = " ";
    bet_text[4] = "0" + bet_hundreds;
    bet_text[5] = "0" + bet_tens;
    bet_text[6] = "0" + bet_ones;

    pay_text[0] = "P";
	 pay_text[1] = "A";
	 pay_text[2] = "Y";
	 pay_text[3] = " ";
    pay_text[4] = "0" + p_hundreds;
    pay_text[5] = "0" + p_tens;
    pay_text[6] = "0" + p_ones;
    
end



// INSTANTIATIONS

sprites_rom icons(clk,sprite_addr, sprite_pixel);
title_rom title(clk, font_ascii, font_row, font_pixel);

lfsr left(clk,rst, rnd_left);
lfsr center(clk,rst, rnd_center);
lfsr right(clk,rst, rnd_right);

weighted_picker_fsm w_left(clk,rst,start_left_pick,rnd_left,sym_left,done_left);
weighted_picker_fsm w_center(clk,rst,start_center_pick,rnd_center,sym_center,done_center);
weighted_picker_fsm w_right(clk,rst,start_right_pick,rnd_right,sym_right,done_right);


// DRAWING LOGIC

always @(*) begin
    vga_color   = 24'h000000;
    sprite_addr = 17'h0;
    font_ascii  = 8'h00;
    font_row    = 4'h0;

    char_index  = 0;
    char_x      = 0;
    char_y      = 0;

    addr_left   = 0;
    addr_center = 0;
    addr_right  = 0;

    if (active_pixels) begin

        //-------------------------
        // CENTER SPRITE
        //-------------------------
        if (x >= CENTER_X && x < CENTER_X + SPRITE_W &&
            y >= CENTER_Y && y < CENTER_Y + SPRITE_H)
        begin
            addr_center = idx_center * SPRITE_W * SPRITE_H +
                          (y - CENTER_Y) * SPRITE_W +
                          (x - CENTER_X);

            sprite_addr = addr_center;
            vga_color   = sprite_pixel;
        end

        //-------------------------
        // LEFT SPRITE
        //-------------------------
        else if (x >= LEFT_X && x < LEFT_X + SPRITE_W &&
                 y >= CENTER_Y && y < CENTER_Y + SPRITE_H)
        begin
            addr_left = idx_left * SPRITE_W * SPRITE_H +
                        (y - CENTER_Y) * SPRITE_W +
                        (x - LEFT_X);

            sprite_addr = addr_left;
            vga_color   = sprite_pixel;
        end

        //-------------------------
        // RIGHT SPRITE
        //-------------------------
        else if (x >= RIGHT_X && x < RIGHT_X + SPRITE_W &&
                 y >= CENTER_Y && y < CENTER_Y + SPRITE_H)
        begin
            addr_right = idx_right * SPRITE_W * SPRITE_H +
                         (y - CENTER_Y) * SPRITE_W +
                         (x - RIGHT_X);

            sprite_addr = addr_right;
            vga_color   = sprite_pixel;
        end

        //-------------------------
        // TITLE
        //-------------------------
        else if (x >= TITLE_X && x < TITLE_X + TITLE_PIX_W &&
                 y >= TITLE_Y && y < TITLE_Y + TITLE_H)
        begin
            char_index = (x - TITLE_X) / TITLE_W;
            char_x     = ((x - TITLE_X) % TITLE_W) / TITLE_SCALE;
            char_y     = ((y - TITLE_Y) % TITLE_H) / TITLE_SCALE;

            font_ascii = (char_index < TITLE_LEN) ? 
                         title_text[char_index] : 8'h20;

            font_row = char_y[2:0];

            if (font_pixel[7 - char_x])
                vga_color = 24'hFFFFFF;
        end

        //-------------------------
        // YOU WIN TEXT
        //-------------------------
        else if (win_flag &&
                 x >= WIN_X && x < WIN_X + WIN_LEN * TITLE_W &&
                 y >= WIN_Y && y < WIN_Y + TITLE_H)
        begin
            char_index = (x - WIN_X) / TITLE_W;
            char_x     = ((x - WIN_X) % TITLE_W) / TITLE_SCALE;
            char_y     = ((y - WIN_Y) % TITLE_H) / TITLE_SCALE;

            font_ascii = (char_index < WIN_LEN) ?
                         win_text[char_index] : 8'h20;

            font_row = char_y[2:0];

            if (font_pixel[7 - char_x])
                vga_color = 24'hFFFF00;
        end

        // BET TEXT
        
        else if (x >= BET_X && x < BET_X + BET_LEN * TITLE_W &&
                 y >= BET_Y && y < BET_Y + TITLE_H)
        begin
            char_index = (x - BET_X) / TITLE_W;
            char_x     = ((x - BET_X) % TITLE_W) / TITLE_SCALE;
            char_y     = ((y - BET_Y) % TITLE_H) / TITLE_SCALE;

            font_ascii = (char_index < BET_LEN) ?
                         bet_text[char_index] : 8'h20;

            font_row = char_y[2:0];

            if (font_pixel[7 - char_x])
                vga_color = 24'hFFFFFF;
        end

        // PAYOUT TEXT
        else if (x >= PAY_X && x < PAY_X + PAY_LEN * TITLE_W &&
                 y >= PAY_Y && y < PAY_Y + TITLE_H)
        begin
            char_index = (x - PAY_X) / TITLE_W;
            char_x     = ((x - PAY_X) % TITLE_W) / TITLE_SCALE;
            char_y     = ((y - PAY_Y) % TITLE_H) / TITLE_SCALE;

            font_ascii = (char_index < PAY_LEN) ?
                         pay_text[char_index] : 8'h20;

            font_row = char_y[2:0];

            if (font_pixel[7 - char_x])
                vga_color = 24'h00FF00;
        end

    end 
end



endmodule