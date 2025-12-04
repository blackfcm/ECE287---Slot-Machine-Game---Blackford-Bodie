module weighted_picker_fsm (
    input  wire        clk,
    input  wire        rst_n,    // active-low reset
    input  wire        start,    // pulse starts selection
    input  wire [7:0]  rnd_in,
    output reg  [3:0]  sym_out,
    output reg         done
);

    // state encoding
    localparam S_IDLE  = 2'd0;
    localparam S_CHECK = 2'd1;
    localparam S_DONE  = 2'd2;

    reg [1:0] state;
    reg [7:0] rnd_reg;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state   <= S_IDLE;
            rnd_reg <= 8'd0;
            sym_out <= 4'd0;
            done    <= 1'b0;
        end else begin
            done <= 1'b0;  // default every cycle except S_DONE

            case (state)

                // ---------------------------------------------------------
                S_IDLE: begin
                    if (start) begin
                        rnd_reg <= rnd_in;
                        state   <= S_CHECK;
                    end
                end

                // ---------------------------------------------------------
                S_CHECK: begin
                    // final weight ranges
                    if      (rnd_reg <= 8'd29)   sym_out <= 4'd0; // clover
                    else if (rnd_reg <= 8'd72)   sym_out <= 4'd1; // watermelon
                    else if (rnd_reg <= 8'd91)   sym_out <= 4'd2; // bell
                    else if (rnd_reg <= 8'd110)  sym_out <= 4'd3; // bar
                    else if (rnd_reg <= 8'd153)  sym_out <= 4'd4; // cherry
                    else if (rnd_reg <= 8'd183)  sym_out <= 4'd5; // diamond
                    else if (rnd_reg <= 8'd213)  sym_out <= 4'd6; // seven
                    else                         sym_out <= 4'd7; // orange

                    state <= S_DONE;
                end

                // ---------------------------------------------------------
                S_DONE: begin
                    done  <= 1'b1;   // 1-cycle pulse
                    state <= S_IDLE; // return to idle automatically
                end

            endcase
        end
    end

endmodule

