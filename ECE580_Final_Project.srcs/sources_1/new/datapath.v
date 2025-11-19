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


module datapath
(
    input clk,
    input reset,
    
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
reg [9:0] x_min;
reg [9:0] y_min;

reg [9:0] top_bound;
reg [9:0] bottom_bound;
reg [9:0] top_bound;
reg [9:0] left_bound;
reg [9:0] right_bound;

reg [9:0] current_cost;
reg valid_input_in_inner_loop_update_stage;
reg [9:0] potential_x_min_in_inner_loop_update_stage;
reg [9:0] potential_y_min_in_inner_loop_update_stage;

// Kamaula: potential_x_min_in_inner_loop_update_stage and potential_y_min_in_inner_loop_update_stage need to be upat
always @( posedge clk ) begin
    potential_x_min_in_inner_loop_update_stage <= THIS SHOULD BE THE X OUTPUT OF THE SYSTOLIC ARRAY // Han/Kamaula link in
    potential_y_min_in_inner_loop_update_stage <= THIS SHOULD BE THE y OUTPUT OF THE SYSTOLIC ARRAY // Han/Kamaula link in
end

always @( posedge clk ) begin
    valid_input_in_inner_loop_update_stage <= xxx // Han/Kamaula: the xxx should be replaced by the "valid_out" of the overall systolic array 
end

// Han: the "add_edge_state" signal should serve as the write enable signal to the vertex grid because it tells us that we're in the state where we want to record the new random point and its optimal parent
// The "add edge state" signal should also be used as the write enable signal for the costs grid

// If vertices[vertices_grid_i_rd][vertices_grid_j_rd] == 1 (aka there's a hit) then "point_hit" to controller should be 1
// Han: need a fifo module - input should be vertices_grid_i_rd and vertices_grid_j_rd if vertices[vertices_grid_i_rd][vertices_grid_j_rd] == 1
// Fifo should use "put_item_in_fifo" from controller as its write enable signal 
// Fifo output should be fed directly into the systolic array
// Fifo should also output fifo_empty and fifo_full signals to the controller based on state of fifo
// The systolic array should read in the i and j value at the head pointer of the fifo

assign wire systolic_array_contains_valid_data = <replace>; // the systolic array module should OR together all of the "valid_out" signals of all the PEs --> if any of them have a high "valid_out" this overall signal should be high
// also note Kamaula: to make this work could you check the "collision_out" to "valid_out" and flip the logic?

assign wire done_draining = ~( rd_fifo || systolic_array_contains_valid_data || valid_input_in_inner_loop_update_stage);

assign path_found = (y_rand < top_bound) && (y_rand > bottom_bound) && (x_rand < right_bound) && (x_rand > left_bound);

// Connecting the vertices grid
assign wire [N:0] vertices_grid_i_rd = inner_loop_counter / N; // Han: use these as indices to read the vertices grid, need to handle N and N_SQUARED
assign wire [N:0] vertices_grid_j_rd = inner_loop_counter % N;
// Han: the "add_edge_state" input signal from the controller should serve as the write enable signal to the vertex grid because it tells us that we're in the state where we want to record the new random point and its optimal parent

// Update x_rand, y_rand
always @( posedge clk ) begin
    if ( reset ) begin
        // maybe should do something?
    end else begin
        if ( sample_point_state==1'b1 ) begin
            x_rand <= $urandom(seed) & 10'b1111111111; // Han: need to set the seed somewhere, i think this is simpler than using an entirely additional module for randomization
            y_rand <= $urandom(seed) & 10'b1111111111;
        end else begin
            x_rand <= x_rand;
            y_rand <= y_rand;
        end
    end
 end
 
 reg [9:0] current_cost;
 
 // Han/Kamaula note that i didn't decide how many bits we should use to store the costs, just pick something reasonable i guess? 
 assign wire [9:0] current_cost_input = quantization_block((x_rand - i_output_of_systolic_array) * (x_rand - i_output_of_systolic_array) + (y_rand - j_output_of_systolic_array) * (y_rand - j_output_of_systolic_array)); // Han/Kamaula: todo create a quantization module to handle this
 // Han/Kamaula: note that rd_cost is supposed to be the cost currently being read of the data structure storing the costs and the indices for what we should be reading should be the output x and y coordinates from the systolic array (they should be propagated through entire systolic array)
 // also the cost data structure should have a read enable signal that's the "valid_output" signal from the systolic array (cost should only be read if the point didn't hit anything)
 // Han/Kamaula: i think the only thing i didn't put a note about yet is that when "add_edge_state" is high from the controller, we should make sure
 // to take the values in xmin, ymin, and cmin and put xmin and ymin into the vertices grid as the parents of xrand and yrand. we should also make sure to mark the vertice grid at 
 // xrand and yrand as 1 and we should make sure that the cost data structure at xrand yrand is updated to cmin. i think that should be good.
 
 wire update_min_point = c_min < (current_cost + rd_cost) ? 1'b1 : 1'b0;
 
 always @( posedge clk) begin
    if ( reset ) begin
        // maybe do something?
    end else begin
        if ( update_min_point == 1'b1 ) begin
            x_min <= potential_x_min_in_inner_loop_update_stage;
            y_min <= potential_y_min_in_inner_loop_update_stage;
            c_min <= current_cost;
        end
    end 
 end
 

endmodule
