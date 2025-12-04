module weighted_picker_fsm (
    input  wire        clk,
    input  wire        rst_n,    
    input  wire        start,    
    input  wire [7:0]  rnd_in,
    output reg  [3:0]  sym_out,
    output reg         done
);

    
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
            done <= 1'b0; 

            case (state)

                
                S_IDLE: begin
                    if (start) begin
                        rnd_reg <= rnd_in;
                        state   <= S_CHECK;
                    end
                end

                
                S_CHECK: begin
                    // Weights
                    if      (rnd_reg <= 8'd32)   sym_out <= 4'd0; // clover
                    else if (rnd_reg <= 8'd64)   sym_out <= 4'd1; // watermelon
                    else if (rnd_reg <= 8'd96)   sym_out <= 4'd2; // bell
                    else if (rnd_reg <= 8'd128)  sym_out <= 4'd3; // bar
                    else if (rnd_reg <= 8'd160)  sym_out <= 4'd4; // cherry
                    else if (rnd_reg <= 8'd192)  sym_out <= 4'd5; // diamond
                    else if (rnd_reg <= 8'd224)  sym_out <= 4'd6; // seven
                    else                         sym_out <= 4'd7; // orange

                    state <= S_DONE;
                end

                    
                S_DONE: begin
                    done  <= 1'b1;   
                    state <= S_IDLE; 
                end

            endcase
        end
    end

endmodule

