`timescale 1ns/1ps
////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Kamaula
// File: quantization_block
// calculates manhattan distance (|x2-x1| + |y2-y1|) between two points
// 
// quantization: uses integer manhattan distance instead of floating-point euclidean distance
// - replaces expensive sqrt((x2-x1)² + (y2-y1)²) with simple |x2-x1| + |y2-y1|
// - all operations are integer arithmetic (subtraction, absolute value, addition)
// - output stored as COST_WIDTH-bit integer (default 12 bits)
// - achieves 62.5% bit savings vs 32-bit floating-point (12 bits vs 32 bits)
// - hardware-friendly: no floating-point units required, only integer 
// - manhattan distance preserves cost ordering and allows proper accumulation for RRT*
//
// latency: 1 clock cycle
////////////////////////////////////////////////////////////////////////////////////////////////////////////

module quantization_block #(
    parameter COORDINATE_WIDTH = 10,
    parameter COST_WIDTH = 16  // bits for cost output
                                // for 1024x1024 grid: max single edge = 2046 (needs 11 bits)
                                // max accumulated cost ~32 edges = 65,472 (needs 16 bits)
)(
    input wire clk,
    input wire rst,
    input wire valid_in,         // only calculate when valid data present

    input wire [COORDINATE_WIDTH-1:0] r_x1,  // random point x
    input wire [COORDINATE_WIDTH-1:0] r_y1,  // random point y
    input wire [COORDINATE_WIDTH-1:0] n_x2,  // nearest neighbor x
    input wire [COORDINATE_WIDTH-1:0] n_y2,  // nearest neighbor y

    output reg [COST_WIDTH-1:0] cost_out    // manhattan distance cost
);

//  Manhattan distance calculation
wire signed [COORDINATE_WIDTH:0] diff_x = $signed({1'b0, r_x1}) - $signed({1'b0, n_x2});
wire signed [COORDINATE_WIDTH:0] diff_y = $signed({1'b0, r_y1}) - $signed({1'b0, n_y2});

wire [COORDINATE_WIDTH:0] abs_x = (diff_x < 0) ? -diff_x : diff_x;
wire [COORDINATE_WIDTH:0] abs_y = (diff_y < 0) ? -diff_y : diff_y;

wire [COST_WIDTH-1:0] manhattan_distance = abs_x + abs_y;

// adds one cycle latency to register output
always @(posedge clk) begin
    if (rst) begin
        cost_out <= 0;
    end else if (valid_in) begin
        cost_out <= manhattan_distance;
    end
end

endmodule