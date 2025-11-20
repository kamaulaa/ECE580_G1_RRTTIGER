`timescale 1ns/1ps
////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Kamaula
// File: quantization_block
// Calculates Manhattan distance (|x2-x1| + |y2-y1|) between two points
// Manhattan distance is used instead of Euclidean to avoid expensive square root
// and to allow proper accumulation of costs in RRT*
// Latency: 1 clock cycle
////////////////////////////////////////////////////////////////////////////////////////////////////////////

module quantization_block #(
    parameter COORDINATE_WIDTH = 10,
    parameter COST_WIDTH = 12  // bits for cost output
)(
    input wire clk,
    input wire rst,
    input wire valid_in,

    input wire [COORDINATE_WIDTH-1:0] r_x1,  // random point x
    input wire [COORDINATE_WIDTH-1:0] r_y1,  // random point y
    input wire [COORDINATE_WIDTH-1:0] n_x2,  // nearest neighbor x
    input wire [COORDINATE_WIDTH-1:0] n_y2,  // nearest neighbor y

    output reg [COST_WIDTH-1:0] cost_out,    // manhattan distance cost
    output reg valid_out                      // Valid signal delayed by 1 cycle
);

// Combinational logic for Manhattan distance calculation
wire signed [COORDINATE_WIDTH:0] diff_x = $signed({1'b0, r_x1}) - $signed({1'b0, n_x2});
wire signed [COORDINATE_WIDTH:0] diff_y = $signed({1'b0, r_y1}) - $signed({1'b0, n_y2});

wire [COORDINATE_WIDTH:0] abs_x = (diff_x < 0) ? -diff_x : diff_x;
wire [COORDINATE_WIDTH:0] abs_y = (diff_y < 0) ? -diff_y : diff_y;

wire [COST_WIDTH-1:0] manhattan_distance = abs_x + abs_y;

// Register output for 1-cycle latency
always @(posedge clk) begin
    if (rst) begin
        cost_out <= 0;
        valid_out <= 0;
    end else begin
        cost_out <= manhattan_distance;
        valid_out <= valid_in;
    end
end

endmodule