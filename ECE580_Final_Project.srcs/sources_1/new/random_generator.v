//----------------------------------------------------------------------
// Han
// ECE580 Final Project
// rrt_random_point:
// generate LFSR-based pseudo-random (x, y) point in grid
//----------------------------------------------------------------------
module rrt_random_point 
#(
    // adjustable parameters to set grid boundaries
    // should match grid.v
    parameter X_BITS = 6,
    parameter Y_BITS = 6,

    parameter LFSR_BITS = 16 // any default value would do as long as >= X_BITS + Y_BITS
)
(
    input                    clk,
    input                    rst,
    input                    generate_req, // pulse to get a new random point

    output reg [X_BITS-1:0]  rand_x,
    output reg [Y_BITS-1:0]  rand_y,
    output reg               rand_valid
);

    reg [LFSR_BITS-1:0] lfsr = {LFSR_BITS{1'b1}}; // non-zero seed 

    wire feedback = lfsr[LFSR_BITS-1] ^ lfsr[2] ^ lfsr[1] ^ lfsr[0]; // example taps

    always @(posedge clk) begin
        if (rst) begin
            lfsr       <= {LFSR_BITS{1'b1}}; // non-zero feed
            rand_x     <= {X_BITS{1'b0}};
            rand_y     <= {Y_BITS{1'b0}};
            rand_valid <= 1'b0;
        end 
        else begin
            rand_valid <= 1'b0;
            if (generate_req) begin
                // advance LFSR
                lfsr <= {lfsr[LFSR_BITS-2:0], feedback};

                // map bits to coordinates
                rand_x     <= lfsr[X_BITS-1:0];
                rand_y     <= lfsr[X_BITS+Y_BITS-1:X_BITS];
                rand_valid <= 1'b1;
            end
        end
    end

endmodule
