`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 11/18/2025 09:05:34 PM
// Design Name: 
// Module Name: datapath
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////


module datapath #(
    parameter COORDINATE_WIDTH = 10,
    parameter NUM_PE = 10,
    parameter N = 32,
    parameter N_SQUARED = 1024,
    parameter COST_WIDTH = 12  // Bits for accumulated cost storage 
)(
    input clk,
    input reset,
    
    // Start and goal points (from testbench)
    input wire [COORDINATE_WIDTH-1:0] start_x,
    input wire [COORDINATE_WIDTH-1:0] start_y,
    input wire [COORDINATE_WIDTH-1:0] goal_x,
    input wire [COORDINATE_WIDTH-1:0] goal_y,
    input wire [COORDINATE_WIDTH-1:0] goal_top_bound,
    input wire [COORDINATE_WIDTH-1:0] goal_bottom_bound,
    input wire [COORDINATE_WIDTH-1:0] goal_left_bound,
    input wire [COORDINATE_WIDTH-1:0] goal_right_bound,
    
    // Obstacle data inputs (from testbench)
    input wire [NUM_PE*COORDINATE_WIDTH-1:0] obs_left,
    input wire [NUM_PE*COORDINATE_WIDTH-1:0] obs_right,
    input wire [NUM_PE*COORDINATE_WIDTH-1:0] obs_top,
    input wire [NUM_PE*COORDINATE_WIDTH-1:0] obs_bottom,
    
    // Dpath -> control
    output path_found,
    output point_hit,
    output done_draining,
    output fifo_full,
    output fifo_empty,
    output parent_equals_current,
    
    // Control -> dpath
    input init_state,
    input rd_fifo,
    input put_item_in_fifo,
    input add_edge_state,
    input sample_point_state,
    input outer_loop_check_state,
    input [N_SQUARED-1:0] inner_loop_counter
);

reg [9:0] x_rand; // Han: this is a register that holds the output of the random number generator 
reg [9:0] y_rand; // Han: same as above 
reg [COORDINATE_WIDTH-1:0] x_min;
reg [COORDINATE_WIDTH-1:0] y_min;

reg  [COST_WIDTH-1:0]  c_min; // minimum cost found so far
wire [COST_WIDTH-1:0] rd_cost; // TODO: connect to cost memory read data for nearest neighbor location (what was the cost at the nearest neighbor, are we storing this?)
wire [COST_WIDTH-1:0] calculated_cost; // new connection cost from quantization block

// wires for systolic array outputs
wire systolic_valid_out;
wire systolic_valid_pair;
wire [COORDINATE_WIDTH-1:0] systolic_val_x1, systolic_val_y1; // non-collided point pair (random is 1 and nearest is 2)
wire [COORDINATE_WIDTH-1:0] systolic_val_x2, systolic_val_y2;

// Pipeline registers to delay systolic outputs by 1 cycle to match quantization block latency
reg systolic_valid_pair_q;
reg [COORDINATE_WIDTH-1:0] systolic_val_x1_q, systolic_val_y1_q;
reg [COORDINATE_WIDTH-1:0] systolic_val_x2_q, systolic_val_y2_q;

always @(posedge clk) begin
    if (reset) begin
        systolic_valid_pair_q <= 0;
        systolic_val_x1_q <= 0;
        systolic_val_y1_q <= 0;
        systolic_val_x2_q <= 0;
        systolic_val_y2_q <= 0;
    end else begin
        systolic_valid_pair_q <= systolic_valid_pair;
        systolic_val_x1_q <= systolic_val_x1;
        systolic_val_y1_q <= systolic_val_y1;
        systolic_val_x2_q <= systolic_val_x2;
        systolic_val_y2_q <= systolic_val_y2;
    end
end

// TODO: Define nearest neighbor coordinates (n_x2, n_y2)
wire [COORDINATE_WIDTH-1:0] nearest_neighbor_x; // default to start points if init_state??
wire [COORDINATE_WIDTH-1:0] nearest_neighbor_y; 

// quantization block for cost calculation
quantization_block #(.COORDINATE_WIDTH(COORDINATE_WIDTH), .COST_WIDTH(COST_WIDTH)) quantized_cost (
    .clk(clk),
    .rst(reset),
    .valid_in(systolic_valid_pair), // only calculate cost when we have valid pair
    .r_x1(systolic_val_x1),      // random point x
    .r_y1(systolic_val_y1),      // random point y
    .n_x2(systolic_val_x2),      // nearest neighbor x (with no collisions) - HOW DO WE ACCESS PREVIOUSLY SAVED COST?
    .n_y2(systolic_val_y2),      // nearest neighbor y (with no collisions)
    .cost_out(calculated_cost) // quantized distance/cost output
);

// obstacle detection systolic array - constantly being fed the newly calculated ranom points and their nearest neighbors
oc_array #(.COORDINATE_WIDTH(COORDINATE_WIDTH), .NUM_PE(NUM_PE)) pe_array (
            .clk(clk),
            .rst(reset),
            .obs_left(obs_left),
            .obs_right(obs_right),
            .obs_top(obs_top),
            .obs_bottom(obs_bottom),
            .r_x1(x_rand),
            .r_y1(y_rand),
            .n_x2(nearest_neighbor_x),
            .n_y2(nearest_neighbor_y),
            .valid_out(systolic_valid_out),
            .valid_pair(systolic_valid_pair),
            .val_x1(systolic_val_x1),
            .val_y1(systolic_val_y1), 
            .val_x2(systolic_val_x2),
            .val_y2(systolic_val_y2)
        );

// Han: the "add_edge_state" signal should serve as the write enable signal to the vertex grid because it tells us that we're in the state where we want to record the new random point and its optimal parent
// The "add edge state" signal should also be used as the write enable signal for the costs grid

// If vertices[vertices_grid_i_rd][vertices_grid_j_rd] == 1 (aka there's a hit) then "point_hit" to controller should be 1
// Han: need a fifo module - input should be vertices_grid_i_rd and vertices_grid_j_rd if vertices[vertices_grid_i_rd][vertices_grid_j_rd] == 1
// Fifo should use "put_item_in_fifo" from controller as its write enable signal 
// Fifo output should be fed directly into the systolic array
// Fifo should also output fifo_empty and fifo_full signals to the controller based on state of fifo
// The systolic array should read in the i and j value at the head pointer of the fifo

assign done_draining = ~( rd_fifo || systolic_valid_out || systolic_valid_pair); // idk if valid_pair should be here - just bc we have avalid pair doesnt mean we're done draining

// Goal check: check if current point is within goal bounds AND doesn't collide with obstacles
// Use delayed systolic outputs to match the timing of calculated_cost from quantization block
wire goal_reached = (systolic_val_x1_q < goal_right_bound) && (systolic_val_x1_q > goal_left_bound) && (systolic_val_y1_q < goal_top_bound) && (systolic_val_y1_q > goal_bottom_bound);
assign path_found = goal_reached && systolic_valid_pair_q; // Only set path_found if we reach goal AND connection is collision-free

// Connecting the vertices grid (Han - need to instantiate the grid)
wire [N:0] vertices_grid_i_rd;
wire [N:0] vertices_grid_j_rd;
assign vertices_grid_i_rd = inner_loop_counter / N; // Han: use these as indices to read the vertices grid, need to handle N and N_SQUARED
assign vertices_grid_j_rd = inner_loop_counter % N;
// Han: the "add_edge_state" input signal from the controller should serve as the write enable signal to the vertex grid because it tells us that we're in the state where we want to record the new random point and its optimal parent

// Update x_rand, y_rand - REPLACE With the random number generator you made (Han)
always @( posedge clk ) begin
    if ( reset ) begin
        // maybe should do something?
    end else begin
        if ( sample_point_state==1'b1 ) begin
            x_rand <= $random & 10'b1111111111;
            y_rand <= $random & 10'b1111111111;
        end else begin
            x_rand <= x_rand;
            y_rand <= y_rand;
        end
    end
 end
 
 // Han/Kamaula note that i didn't decide how many bits we should use to store the costs, just pick something reasonable i guess?
 // Han/Kamaula: note that rd_cost is supposed to be the cost currently being read of the data structure storing the costs and the indices for what we should be reading should be the output x and y coordinates from the systolic array (they should be propagated through entire systolic array)
 // also the cost data structure should have a read enable signal that's the "valid_output" signal from the systolic array (cost should only be read if the point didn't hit anything)
 // Han/Kamaula: i think the only thing i didn't put a note about yet is that when "add_edge_state" is high from the controller, we should make sure
 // to take the values in xmin, ymin, and cmin and put xmin and ymin into the vertices grid as the parents of xrand and yrand. we should also make sure to mark the vertice grid at 
 // xrand and yrand as 1 and we should make sure that the cost data structure at xrand yrand is updated to cmin. i think that should be good.
 
 // total cost calculation: edge cost + existing cost at neighbor location
 wire [COST_WIDTH-1:0] total_cost = calculated_cost + rd_cost;
 wire update_min_point = (total_cost < c_min) ? 1'b1 : 1'b0;
 
 always @( posedge clk) begin
    if ( reset ) begin
        x_min <= {COORDINATE_WIDTH{1'b0}};
        y_min <= {COORDINATE_WIDTH{1'b0}};
        c_min <= {COST_WIDTH{1'b1}};  // Initialize to max value for minimum comparison
    end else begin
        if ( update_min_point ) begin //stores valid nearest neighbor point with minimal cost calculated for connection to random point
            x_min <= systolic_val_x2_q;  // used registered value since it takes an extra cycle after we find a valid pair for us to actually update these values
            y_min <= systolic_val_y2_q;  
            c_min <= total_cost; // we need to store total cost (edge + existing)
        end
    end 
 end
 

endmodule
