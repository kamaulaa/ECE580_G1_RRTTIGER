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
    wire [LFSR_BITS-1:0] lfsr_next = {lfsr[LFSR_BITS-2:0], feedback};

    // whitening / mixing (xorshift-ish + rotate) 
    wire [LFSR_BITS-1:0] rot8  = {lfsr_next[LFSR_BITS-9:0], lfsr_next[LFSR_BITS-1:LFSR_BITS-8]};
    wire [LFSR_BITS-1:0] mix0  = lfsr_next ^ (lfsr_next >> 13) ^ (lfsr_next << 7) ^ rot8;

    wire [LFSR_BITS-1:0] rot2  = {mix0[LFSR_BITS-3:0],  mix0[LFSR_BITS-1:LFSR_BITS-2]};
    wire [LFSR_BITS-1:0] mix1  = mix0 ^ (mix0 >> 17) ^ (mix0 << 5) ^ rot2;

    wire [LFSR_BITS-1:0] rot4  = {mix1[LFSR_BITS-5:0],  mix1[LFSR_BITS-1:LFSR_BITS-4]};
    wire [LFSR_BITS-1:0] mix2  = mix1 ^ (mix1 >> 11) ^ (mix1 << 13) ^ rot4;

    always @(posedge clk) begin
        if (rst) begin
            lfsr <= {LFSR_BITS{1'b1}}; // non-zero feed
            random_point_x <= {X_BITS{1'b0}};
            random_point_y <= {Y_BITS{1'b0}};
        end 
        else begin
            lfsr <= lfsr_next;
            if (generate_req == 1'b1) begin
                random_point_x <= mix1[X_BITS-1:0];
                random_point_y <= mix2[X_BITS+Y_BITS-1:X_BITS];
            end
        end
    end
    
endmodule

