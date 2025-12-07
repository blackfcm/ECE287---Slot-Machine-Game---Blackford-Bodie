module title_rom(
    input  wire       clk,
    input  wire [7:0] ascii,   // ASCII character
    input  wire [3:0] row,     // row 0–7
    output reg  [7:0] pixels   // 8 horizontal bits
);

    // 8x8 VGA ASCII ROM (128 characters)
    reg [7:0] font[0:128*8-1];

    initial begin
        // ------- Minimal ASCII set for letters, numbers, space --------
        // You can expand later; this contains only needed chars

        // SPACE " "
        font[32*8+0]=8'b00000000;
        font[32*8+1]=8'b00000000;
        font[32*8+2]=8'b00000000;
        font[32*8+3]=8'b00000000;
        font[32*8+4]=8'b00000000;
        font[32*8+5]=8'b00000000;
        font[32*8+6]=8'b00000000;
        font[32*8+7]=8'b00000000;

        // F
        font["F"*8+0]=8'b11111110;
        font["F"*8+1]=8'b11111110;
        font["F"*8+2]=8'b11000000;
        font["F"*8+3]=8'b11000000;
        font["F"*8+4]=8'b11111100;
        font["F"*8+5]=8'b11111100;
        font["F"*8+6]=8'b11000000;
        font["F"*8+7]=8'b11000000;

        // o
        font["o"*8+0]=8'b00000000;
        font["o"*8+1]=8'b01111000;
        font["o"*8+2]=8'b11001100;
        font["o"*8+3]=8'b11001100;
        font["o"*8+4]=8'b11001100;
        font["o"*8+5]=8'b11001100;
        font["o"*8+6]=8'b01111000;
        font["o"*8+7]=8'b00000000;

        // r
        font["r"*8+0]=8'b00000000;
        font["r"*8+1]=8'b11111000;
        font["r"*8+2]=8'b11001100;
        font["r"*8+3]=8'b11000000;
        font["r"*8+4]=8'b11000000;
        font["r"*8+5]=8'b11000000;
        font["r"*8+6]=8'b11000000;
        font["r"*8+7]=8'b00000000;

        // t
        font["t"*8+0]=8'b00110000;
        font["t"*8+1]=8'b00110000;
        font["t"*8+2]=8'b11111100;
        font["t"*8+3]=8'b11111100;
        font["t"*8+4]=8'b00110000;
        font["t"*8+5]=8'b00110000;
        font["t"*8+6]=8'b00111100;
        font["t"*8+7]=8'b00000000;

        // u
        font["u"*8+0]=8'b00000000;
        font["u"*8+1]=8'b11001100;
        font["u"*8+2]=8'b11001100;
        font["u"*8+3]=8'b11001100;
        font["u"*8+4]=8'b11001100;
        font["u"*8+5]=8'b11001100;
        font["u"*8+6]=8'b01111100;
        font["u"*8+7]=8'b00000000;

        // n
        font["n"*8+0]=8'b00000000;
        font["n"*8+1]=8'b11111000;
        font["n"*8+2]=8'b11001100;
        font["n"*8+3]=8'b11001100;
        font["n"*8+4]=8'b11001100;
        font["n"*8+5]=8'b11001100;
        font["n"*8+6]=8'b11001100;
        font["n"*8+7]=8'b00000000;

        // e
        font["e"*8+0]=8'b00000000;
        font["e"*8+1]=8'b01111000;
        font["e"*8+2]=8'b11001100;
        font["e"*8+3]=8'b11111100;
        font["e"*8+4]=8'b11000000;
        font["e"*8+5]=8'b11000000;
        font["e"*8+6]=8'b01111100;
        font["e"*8+7]=8'b00000000;

        // z
        font["z"*8+0]=8'b00000000;
        font["z"*8+1]=8'b11111100;
        font["z"*8+2]=8'b00001100;
        font["z"*8+3]=8'b00111000;
        font["z"*8+4]=8'b01100000;
        font["z"*8+5]=8'b11000000;
        font["z"*8+6]=8'b11111100;
        font["z"*8+7]=8'b00000000;

        // y
        font["y"*8+0]=8'b00000000;
        font["y"*8+1]=8'b11001100;
        font["y"*8+2]=8'b11001100;
        font["y"*8+3]=8'b11001100;
        font["y"*8+4]=8'b01111100;
        font["y"*8+5]=8'b00001100;
        font["y"*8+6]=8'b11111000;
        font["y"*8+7]=8'b00000000;
    end

    always @(posedge clk)
        pixels <= font[ascii*8 + row];

endmodule
