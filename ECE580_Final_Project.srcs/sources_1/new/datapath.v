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
#(
    // ADJUSTABLE GRID PARAMETERS
    // TODO: need to pass these from core into dpath and control modules 
    parameter N = 1024,
    parameter N_SQUARED = N * N,
    parameter X_BITS = 10, // log2(GRID_WIDTH)
    parameter Y_BITS = 10, // log2(GRID_HEIGHT )
    parameter ADDR_BITS = 20 // log2(GRID_WIDTH * GRID_HEIGHT) for flattened addr
)
(
    input clk,
    input reset,
    
    // Dpath -> control
    output path_found,
    output point_hit,
    output done_draining,
    output parent_equals_current,
    output reg new_random_point_valid,
    output neighbor_search_busy,
    
    // Control -> dpath
    input init_state,
    input add_edge_state,
    input outer_loop_check_state,
    input generate_req, // to random point generator module
    input search_start // to neighbor search module 
);

    reg [X_BITS-1:0]  x_min;
    reg [Y_BITS-1:0]  y_min;

    reg [Y_BITS-1:0] top_bound;
    reg [Y_BITS-1:0] bottom_bound;
    reg [X_BITS-1:0] left_bound;
    reg [X_BITS-1:0] right_bound;

    // store occupancy state of grid points (whether coordinates has point)
    reg [N_SQUARED-1:0] occupancy_status; 
    // compute flattened address : y * GRID_W + x
    function [N_SQUARED-1:0] idx;
        input [X_BITS-1:0] x_coord;
        input [Y_BITS-1:0] y_coord;
        begin
            idx = y_coord * N + x_coord;
        end
    endfunction

    // x and y coordinates of random point generated
    reg [X_BITS-1:0]  x_rand; 
    reg [Y_BITS-1:0]  y_rand; 

    // need to make bit def dynmaic
    reg [9:0] current_cost;
    reg valid_input_in_inner_loop_update_stage;
    reg [9:0] potential_x_min_in_inner_loop_update_stage;
    reg [9:0] potential_y_min_in_inner_loop_update_stage;
    // Kamaula: potential_x_min_in_inner_loop_update_stage and potential_y_min_in_inner_loop_update_stage need to be upat
    always @( posedge clk ) begin
        potential_x_min_in_inner_loop_update_stage <= THIS SHOULD BE THE X OUTPUT OF THE SYSTOLIC ARRAY; // Han/Kamaula link in
        potential_y_min_in_inner_loop_update_stage <= THIS SHOULD BE THE y OUTPUT OF THE SYSTOLIC ARRAY; // Han/Kamaula link in
    end

    always @( posedge clk ) begin
        valid_input_in_inner_loop_update_stage <= xxx; // Han/Kamaula: the xxx should be replaced by the "valid_out" of the overall systolic array 
    end

    // Han: the "add_edge_state" signal should serve as the write enable signal to the vertex grid because it tells us that we're in the state where we want to record the new random point and its optimal parent
    // The "add edge state" signal should also be used as the write enable signal for the costs grid

    // If vertices[vertices_grid_i_rd][vertices_grid_j_rd] == 1 (aka there's a hit) then "point_hit" to controller should be 1

    assign wire systolic_array_contains_valid_data = <replace>; // the systolic array module should OR together all of the "valid_out" signals of all the PEs --> if any of them have a high "valid_out" this overall signal should be high
    // also note Kamaula: to make this work could you check the "collision_out" to "valid_out" and flip the logic?

    assign wire done_draining = ~( nb_found || systolic_array_contains_valid_data || valid_input_in_inner_loop_update_stage);

    assign path_found = (y_rand < top_bound) && (y_rand > bottom_bound) && (x_rand < right_bound) && (x_rand > left_bound);

    // Han: the "add_edge_state" input signal from the controller should serve as the write enable signal to the vertex grid because it tells us that we're in the state where we want to record the new random point and its optimal parent

////////////////////////////////////////////////////////////////////////
// RANDOM GENERATOR 

    // wires from random generator
    wire [X_BITS-1:0] x_rand_wire;
    wire [Y_BITS-1:0] y_rand_wire;
    // instantiate random point generator
    rrt_random_point #(
        .X_BITS(X_BITS),
        .Y_BITS(Y_BITS),
        .ADDR_BITS(ADDR_BITS)
    ) generate_point (
        .clk          (clk),
        .rst          (reset),
        .generate_req (generate_req),
        .random_point_x       (x_rand_wire),
        .random_point_y       (y_rand_wire)
    );

    // check if generated point already exists
    wire rand_valid_wire = !occupancy_status[idx(x_rand_wire, y_rand_wire)];

    // Update x_rand, y_rand
    always @( posedge clk ) begin
        if ( reset ) begin
            x_rand <= {X_BITS{1'b0}};
            y_rand <= {Y_BITS{1'b0}};
            new_random_point_valid <= 1'b0;
        end 
        else begin
            new_random_point_valid <= 1'b0; // default
            if (generate_req==1'b1 && rand_valid_wire==1'b1) begin           
                x_rand <= x_rand_wire;
                y_rand <= y_rand_wire;
                new_random_point_valid <= 1'b1;
                occupancy_status[idx(x_rand_wire, y_rand_wire)] <= 1'b1; // mark point as occupied
            end
        end
    end
////////////////////////////////////////////////////////////////////////

// NEIGHBOR SEARCH IN WINDOW FRAME

    // adjustable window radius
    localparam [X_BITS-1:0] WINDOW_RADIUS = {{(X_BITS-3){1'b0}},3'b101}; // 5

    // wires that are fed into systolic array 
    wire neighbor_busy;
    wire nb_found;
    wire [X_BITS-1:0] nb_x;
    wire [Y_BITS-1:0] nb_y;

    // instantiate random point generator
    neighbor_search #(
        .GRID_W   (N),
        .GRID_H   (N),
        .X_BITS   (X_BITS),
        .Y_BITS   (Y_BITS),
        .N_SQUARED (N_SQUARED)
    ) find_neighbor (
        .clk             (clk),
        .rst             (reset),

        // control
        .search_start    (search_start),
        .node_x          (x_rand),
        .node_y          (y_rand),
        .window_radius   (WINDOW_RADIUS),
        .occupancy_status(occupancy_status),
        .neighbor_search_busy (neighbor_search_busy),

        // detected neighbor node output (queue-style stream)
        // .nb_ready        (nb_ready),
        .nb_found        (nb_found),
        .nb_x            (nb_x),
        .nb_y            (nb_y)
    );

////////////////////////////////////////////////////////////////////////

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
