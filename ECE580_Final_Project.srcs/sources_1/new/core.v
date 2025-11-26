`timescale 1ns / 1ps

// ECE580 Final Project

module core
#(
    // ADJUSTABLE GRID PARAMETERS
    parameter COORDINATE_WIDTH = 10,
    parameter NUM_PE = 5,
    parameter NUM_PE_WIDTH = 3,
    parameter N = 1024,
    parameter N_SQUARED = N * N,
    parameter N_BITS = 10,
    parameter OUTERMOST_ITER_MAX = 1024,
    parameter OUTERMOST_ITER_BITS = 10,
    parameter COST_WIDTH = 16,
    parameter ADDR_BITS = 20
)
(
    input  clk,
    input  reset,
    
    // Start and goal points
    input wire [COORDINATE_WIDTH-1:0] start_x,
    input wire [COORDINATE_WIDTH-1:0] start_y,
    input wire [COORDINATE_WIDTH-1:0] goal_top_bound,
    input wire [COORDINATE_WIDTH-1:0] goal_bottom_bound,
    input wire [COORDINATE_WIDTH-1:0] goal_left_bound,
    input wire [COORDINATE_WIDTH-1:0] goal_right_bound,
    
    // Obstacle data inputs
    input wire [NUM_PE*COORDINATE_WIDTH-1:0] obs_left,
    input wire [NUM_PE*COORDINATE_WIDTH-1:0] obs_right,
    input wire [NUM_PE*COORDINATE_WIDTH-1:0] obs_top,
    input wire [NUM_PE*COORDINATE_WIDTH-1:0] obs_bottom,

    // Status outputs
    output path_found
);

    // Control -> Datapath signals
    wire init_state;
    wire add_edge_state;
    wire generate_req;
    wire window_search_start;
    wire search_neighbor;
    wire entering_search_nearest_neighbor;
    wire add_new_point_q;
    wire eval_random_point;
    wire generate_random_point;
    wire entering_check_new_point_q_collision;
    wire check_points_in_square_radius;
    wire drain_arr;
    
    // Datapath -> Control signals
    wire new_point_q_collided;
    wire done_draining;
    wire parent_equals_current;
    wire random_point_already_exists;
    wire window_search_busy;
    wire done_with_search_nearest_neighbor;
    wire done_evaluating_random_point;
    wire done_detecting_new_point_q_collision;

    // Datapath instantiation
    datapath #(
        .COORDINATE_WIDTH(COORDINATE_WIDTH),
        .NUM_PE(NUM_PE),
        .NUM_PE_WIDTH(NUM_PE_WIDTH),
        .N(N),
        .N_SQUARED(N_SQUARED),
        .N_BITS(N_BITS),
        .OUTERMOST_ITER_MAX(OUTERMOST_ITER_MAX),
        .OUTERMOST_ITER_BITS(OUTERMOST_ITER_BITS),
        .COST_WIDTH(COST_WIDTH),
        .ADDR_BITS(ADDR_BITS)
    ) datapath_inst (
        .clk(clk),
        .reset(reset),
        
        // Start and goal points
        .start_x(start_x),
        .start_y(start_y),
        .goal_top_bound(goal_top_bound),
        .goal_bottom_bound(goal_bottom_bound),
        .goal_left_bound(goal_left_bound),
        .goal_right_bound(goal_right_bound),
        
        // Obstacle data
        .obs_left(obs_left),
        .obs_right(obs_right),
        .obs_top(obs_top),
        .obs_bottom(obs_bottom),
        
        // Outputs to control
        .path_found(path_found),
        .new_point_q_collided(new_point_q_collided),
        .done_draining(done_draining),
        .parent_equals_current(parent_equals_current),
        .random_point_already_exists(random_point_already_exists),
        .window_search_busy(window_search_busy),
        .done_with_search_nearest_neighbor(done_with_search_nearest_neighbor),
        .done_evaluating_random_point(done_evaluating_random_point),
        .done_detecting_new_point_q_collision(done_detecting_new_point_q_collision),
        
        // Inputs from control
        .init_state(init_state),
        .add_edge_state(add_edge_state),
        .outer_loop_check_state(1'b0),  // Not used in current implementation
        .generate_req(generate_req),
        .window_search_start(window_search_start),
        .search_neighbor(search_neighbor),
        .entering_search_nearest_neighbor(entering_search_nearest_neighbor),
        .add_new_point_q(add_new_point_q),
        .eval_random_point(eval_random_point),
        .generate_random_point(generate_random_point),
        .entering_check_new_point_q_collision(entering_check_new_point_q_collision),
        .check_points_in_square_radius(check_points_in_square_radius),
        .drain_arr(drain_arr)
    );

    // Control FSM instantiation
    core_ctrl #(
        .N(N),
        .N_SQUARED(N_SQUARED),
        .OUTERMOST_ITER_MAX(OUTERMOST_ITER_MAX),
        .OUTERMOST_ITER_BITS(OUTERMOST_ITER_BITS),
        .X_BITS(N_BITS),
        .Y_BITS(N_BITS),
        .ADDR_BITS(ADDR_BITS)
    ) core_ctrl_inst (
        .clk(clk),
        .reset(reset),
        
        // Inputs from datapath
        .path_found(path_found),
        .new_point_q_collided(new_point_q_collided),
        .done_draining(done_draining),
        .parent_equals_current(parent_equals_current),
        .random_point_already_exists(random_point_already_exists),
        .window_search_busy(window_search_busy),
        .done_with_search_nearest_neighbor(done_with_search_nearest_neighbor),
        .done_evaluating_random_point(done_evaluating_random_point),
        .done_detecting_new_point_q_collision(done_detecting_new_point_q_collision),
        
        // Outputs to datapath
        .init_state(init_state),
        .add_edge_state(add_edge_state),
        .generate_req(generate_req),
        .window_search_start(window_search_start),
        .search_neighbor(search_neighbor),
        .entering_search_nearest_neighbor(entering_search_nearest_neighbor),
        .add_new_point_q(add_new_point_q),
        .eval_random_point(eval_random_point),
        .generate_random_point(generate_random_point),
        .entering_check_new_point_q_collision(entering_check_new_point_q_collision),
        .check_points_in_square_radius(check_points_in_square_radius),
        .drain_arr(drain_arr)
    );
endmodule
