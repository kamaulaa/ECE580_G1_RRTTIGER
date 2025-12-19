`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Module Name: Core.v
//////////////////////////////////////////////////////////////////////////////////

module core
#(
    // ADJUSTABLE GRID PARAMETERS
    parameter COORDINATE_WIDTH = 7,
    parameter NUM_PE = 5,
    parameter NUM_PE_WIDTH = 3,
    parameter N = 128,
    parameter N_SQUARED = N * N,
    parameter N_BITS = 7,
    parameter OUTERMOST_ITER_MAX = 511, 
    parameter OUTERMOST_ITER_BITS = 9, 
    parameter COST_WIDTH = 16,
    parameter ADDR_BITS = 14,
    parameter ARRAY_WIDTH = OUTERMOST_ITER_BITS + COORDINATE_WIDTH*2 + COST_WIDTH
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
    output path_found,
    
    output [3:0] output_state,
    output generate_req,
    output random_point_already_exists,
    output done_detecting_new_point_q_collision,   
    output entering_check_steered_point,
    output steered_point_in_obstacle,    
    output check_steered_point,
    output check_new_point_q_collision,
    output add_new_point_q,
    output search_neighbor,
    output done_with_search_nearest_neighbor,
    output add_edge_state,
    output entering_search_nearest_neighbor,
    output entering_check_new_point_q_collision,
    output do_traceback,
    
    output [COST_WIDTH-1:0] finalcost,
    output [COORDINATE_WIDTH-1:0] final_xcoord,
    output [COORDINATE_WIDTH-1:0] final_ycoord,
    output [OUTERMOST_ITER_BITS-1:0] tracebackptr,
    output [OUTERMOST_ITER_BITS-1:0] new_tracebackptr
);

    // Control -> Datapath signals
    wire init_state;
    wire add_edge_state;
    wire generate_req;
    wire search_neighbor;
    wire entering_search_nearest_neighbor;
    wire add_new_point_q;
    wire eval_random_point;
    wire generate_random_point;
    wire entering_check_steered_point;
    wire entering_check_new_point_q_collision;
    wire check_points_in_square_radius;
    wire drain_arr;
    
    // Datapath -> Control signals
    wire new_point_q_collided;
    wire done_draining;
    wire done_traceback;
    wire random_point_already_exists;
    wire done_with_search_nearest_neighbor;
    wire done_evaluating_random_point;
    wire done_detecting_new_point_q_collision;
    wire steered_point_in_obstacle;
    wire done_checking_steered_point;
    wire check_steered_point;

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
        .done_traceback(done_traceback),
        .random_point_already_exists(random_point_already_exists),
        .done_with_search_nearest_neighbor(done_with_search_nearest_neighbor),
        .done_evaluating_random_point(done_evaluating_random_point),
        .done_detecting_new_point_q_collision(done_detecting_new_point_q_collision),
        .steered_point_in_obstacle(steered_point_in_obstacle),
        .done_checking_steered_point(done_checking_steered_point),
        
        // Inputs from control
        .init_state(init_state),
        .add_edge_state(add_edge_state),
        .outer_loop_check_state(1'b0),  // Not used in current implementation
        .generate_req(generate_req),
        .search_neighbor(search_neighbor),
        .entering_search_nearest_neighbor(entering_search_nearest_neighbor),
        .add_new_point_q(add_new_point_q),
        .eval_random_point(eval_random_point),
        .generate_random_point(generate_random_point),
        .entering_check_steered_point(entering_check_steered_point),
        .entering_check_new_point_q_collision(entering_check_new_point_q_collision),
        .check_points_in_square_radius(check_points_in_square_radius),
        .drain_arr(drain_arr),
        .do_traceback(do_traceback),
        .check_steered_point(check_steered_point),
        .check_new_point_q_collision(check_new_point_q_collision),
        
        .finalcost(finalcost),
        .final_xcoord(final_xcoord),
        .final_ycoord(final_ycoord),
        .tracebackptr(tracebackptr),
        .new_tracebackptr(new_tracebackptr)
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
        
        // Outputs to core
        .output_state(output_state),
        
        // Inputs from datapath
        .path_found(path_found),
        .new_point_q_collided(new_point_q_collided),
        .done_draining(done_draining),
        .done_traceback(done_traceback),
        .random_point_already_exists(random_point_already_exists),
        .done_with_search_nearest_neighbor(done_with_search_nearest_neighbor),
        .done_evaluating_random_point(done_evaluating_random_point),
        .done_detecting_new_point_q_collision(done_detecting_new_point_q_collision),
        .steered_point_in_obstacle(steered_point_in_obstacle),
        .done_checking_steered_point(done_checking_steered_point),
        
        // Outputs to datapath
        .init_state(init_state),
        .add_edge_state(add_edge_state),
        .generate_req(generate_req),
        .search_neighbor(search_neighbor),
        .entering_search_nearest_neighbor(entering_search_nearest_neighbor),
        .add_new_point_q(add_new_point_q),
        .eval_random_point(eval_random_point),
        .generate_random_point(generate_random_point),
        .entering_check_steered_point(entering_check_steered_point),
        .entering_check_new_point_q_collision(entering_check_new_point_q_collision),
        .check_points_in_square_radius(check_points_in_square_radius),
        .drain_arr(drain_arr),
        .check_steered_point(check_steered_point),
        .check_new_point_q_collision(check_new_point_q_collision),
        .do_traceback(do_traceback)
    );
endmodule
