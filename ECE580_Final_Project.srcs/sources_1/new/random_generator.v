//----------------------------------------------------------------------
// Han
// ECE580 Final Project
// rrt_random_point:
// generate LFSR-based pseudo-random (x, y) point in grid
//----------------------------------------------------------------------
module rrt_random_point 
#(
    // adjustable parameters to set grid boundaries
    parameter X_BITS = 10,
    parameter Y_BITS = 10,
    parameter ADDR_BITS = 20 
)
(
    input                    clk,
    input                    rst,
    input                    generate_req, // pulse to get a new random point
    output reg [X_BITS-1:0]  random_point_x,
    output reg [Y_BITS-1:0]  random_point_y
);

    localparam LFSR_BITS = ADDR_BITS + 1; // any default value would do as long as >= X_BITS + Y_BITS

    reg [LFSR_BITS-1:0] lfsr = {LFSR_BITS{1'b1}}; // non-zero seed 
    wire feedback = lfsr[LFSR_BITS-1] ^ lfsr[2] ^ lfsr[1] ^ lfsr[0]; // example taps
    // Next LFSR value: only advance when generate_req is 1
    wire [LFSR_BITS-1:0] lfsr_next = (generate_req==1'b1) ? {lfsr[LFSR_BITS-2:0], feedback} : lfsr;

    always @(posedge clk) begin
        if (rst) begin
            lfsr <= {LFSR_BITS{1'b1}}; // non-zero feed
            random_point_x <= {X_BITS{1'b0}};
            random_point_y <= {Y_BITS{1'b0}};
        end 
        else begin
            lfsr <= {lfsr[LFSR_BITS-2:0], feedback};
            if (generate_req == 1'b1) begin
                random_point_x <= lfsr[X_BITS-1:0];
                random_point_y <= lfsr[ADDR_BITS-1:X_BITS];
            end`

        end
    end
endmodule

