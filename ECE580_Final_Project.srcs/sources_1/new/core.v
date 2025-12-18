`timescale 1ns / 1ps

// ECE580 Final Project

module core
#(
    // ADJUSTABLE GRID PARAMETERS
    parameter COORDINATE_WIDTH = 7,
    parameter NUM_PE = 5,
    parameter NUM_PE_WIDTH = 3,
    parameter N = 128,
    parameter N_SQUARED = N * N,
    parameter N_BITS = 7,
    parameter OUTERMOST_ITER_MAX = 4095, // lauren: was 1023
    parameter OUTERMOST_ITER_BITS = 12, // lauren: was 10
    parameter COST_WIDTH = 16, // TODO: maybe change the cost width --> max it would take up 19 bits, so maybe we could truncate it more
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
    
//    // Expose states so that testbench knows when to end
    output failure_state,
    output traceback_state,
    output [3:0] output_state,
    output generate_req,
    output random_point_already_exists,
    output [COORDINATE_WIDTH-1:0] xrand_wire,
    output [COORDINATE_WIDTH-1:0] yrand_wire,
    output [COORDINATE_WIDTH-1:0] occupied_array_currentidx,
    output current_array_entry_same_asrandom,
    output [COORDINATE_WIDTH-1:0] occupied_points_array_occupied_array_current_idx_X_MSB_X_LSB,
    output [COORDINATE_WIDTH-1:0] occupied_points_array_occupied_array_current_idx_Y_MSB_Y_LSB,
    output x_equal,
    output y_equal,
    
    output done_detecting_new_point_q_collision,
    output new_point_qcollided,
    output [4:0] total_draincycles,
    output [4:0] detecting_new_point_q_collision_cyclecount,
    
    output entering_check_steered_point,
    output steered_point_in_obstacle,    
    output done_checking_steeredpoint,
    output [NUM_PE_WIDTH:0] steered_point_check_cyclecount,
    
    output [3:0] nearest_neighborcount,
    output searchneighbor,
    output entering_search_nearestneighbor,
    
    output systolic_validout,
    output systolic_validpair,
    
    output [OUTERMOST_ITER_BITS-1:0] outermost_loopcounter,
    output check_steered_point,
    output check_new_point_q_collision,
    
    output update_minpoint,
    output systolic_valid_pairq,
    output [COST_WIDTH-1:0] rdcost,
    output [COST_WIDTH-1:0] calculatedcost,
    output [COST_WIDTH-1:0] totalcost,
    
    output validin,
    
    output [COORDINATE_WIDTH-1:0] systolic_valx1,
    output [COORDINATE_WIDTH-1:0] systolic_valy1,
    output [COORDINATE_WIDTH-1:0] systolic_valx2,
    output [COORDINATE_WIDTH-1:0] systolic_valy2,
    output [COORDINATE_WIDTH-1:0] systolic_val_parentindex,
    
    output [COORDINATE_WIDTH-1:0] new_pointx,
    output [COORDINATE_WIDTH-1:0] new_pointy,
    
    output add_new_point_q,
    
    output [COORDINATE_WIDTH-1:0] new_point_parentx,
    output [COORDINATE_WIDTH-1:0] new_point_parenty,
    
    output [OUTERMOST_ITER_BITS-1:0] best_neighboridx,
    
    output search_neighbor,
    output done_with_search_nearest_neighbor,
    output [COORDINATE_WIDTH-1:0] potential_new_pointx,
    output [COORDINATE_WIDTH-1:0] potential_new_pointy,
    
    output [OUTERMOST_ITER_BITS-1:0] occupied_arrayidx,
    
    output add_edge_state,
    output entering_search_nearest_neighbor,
    output entering_check_new_point_q_collision,
    output do_traceback,
    
    output [COST_WIDTH-1:0] finalcost, // this stays the same during traceback, it's always the cost of the last element added
    output [COORDINATE_WIDTH-1:0] final_xcoord, // this changes each cycle of traceback
    output [COORDINATE_WIDTH-1:0] final_ycoord, // this changes each cycle of traceback
    
    output [OUTERMOST_ITER_BITS-1:0] tracebackptr,
    output [OUTERMOST_ITER_BITS-1:0] new_tracebackptr,
    
    output outermost_loopcheck,
    output outermost_counter_less_than,
    
    output goalreached,
    
    output [COORDINATE_WIDTH-1:0] systolic_val_x1q,
    output [COORDINATE_WIDTH-1:0] systolic_val_y1q,
    output [COST_WIDTH-1:0] cmin,
    output [OUTERMOST_ITER_BITS-1:0] systolic_val_parent_indexq,
    
    output [COORDINATE_WIDTH-1:0] nbx, // coords of that nearest neighbor
    output [COORDINATE_WIDTH-1:0] nby,
    output [OUTERMOST_ITER_BITS-1:0] nbindex
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
        
        .xrand_wire(xrand_wire),
        .yrand_wire(yrand_wire),
        .occupied_array_currentidx(occupied_array_currentidx),
        .current_array_entry_same_asrandom(current_array_entry_same_asrandom),
        .occupied_points_array_occupied_array_current_idx_X_MSB_X_LSB(occupied_points_array_occupied_array_current_idx_X_MSB_X_LSB),
        .occupied_points_array_occupied_array_current_idx_Y_MSB_Y_LSB(occupied_points_array_occupied_array_current_idx_Y_MSB_Y_LSB),
        .x_equal(x_equal),
        .y_equal(y_equal),
        
//        .done_detecting_new_point_qcollision(done_detecting_new_point_q_collision),
        .new_point_qcollided(new_point_qcollided),
        .total_draincycles(total_draincycles),
        .detecting_new_point_q_collision_cyclecount(detecting_new_point_q_collision_cyclecount),
        
        .done_checking_steeredpoint(done_checking_steeredpoint),
        .steered_point_check_cyclecount(steered_point_check_cyclecount),
        
        .nearest_neighborcount(nearest_neighborcount),
        .searchneighbor(searchneighbor),
        .entering_search_nearestneighbor(entering_search_nearestneighbor),
        
        .systolic_validout(systolic_validout),
        .systolic_validpair(systolic_validpair),
        
        .check_steered_point(check_steered_point),
        .check_new_point_q_collision(check_new_point_q_collision),
        
        .update_minpoint(update_minpoint),
        .systolic_valid_pairq(systolic_valid_pairq),
        .rdcost(rdcost),
        .calculatedcost(calculatedcost),
        .totalcost(totalcost),
        
        .validin(validin),
        
        .systolic_valx1(systolic_valx1),
        .systolic_valy1(systolic_valy1),
        .systolic_valx2(systolic_valx2),
        .systolic_valy2(systolic_valy2),
        .systolic_val_parentindex(systolic_val_parentindex),
        
        .new_pointx(new_pointx),
        .new_pointy(new_pointy),
        
        .new_point_parentx(new_point_parentx),
        .new_point_parenty(new_point_parenty),
        .best_neighboridx(best_neighboridx),
        
        .potential_new_pointx(potential_new_pointx),
        .potential_new_pointy(potential_new_pointy),
        .occupied_arrayidx(occupied_arrayidx),
        
        .finalcost(finalcost),
        .final_xcoord(final_xcoord),
        .final_ycoord(final_ycoord),
        .tracebackptr(tracebackptr),
        .new_tracebackptr(new_tracebackptr),
        
        .goalreached(goalreached),
        .systolic_val_x1q(systolic_val_x1q),
        .systolic_val_y1q(systolic_val_y1q),
        .cmin(cmin),
        .systolic_val_parent_indexq(systolic_val_parent_indexq),
        
        .nbx(nbx),
        .nby(nby),
        .nbindex(nbindex)
        
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
        .failure_state(failure_state),
        .traceback_state(traceback_state),
        .output_state(output_state),
        .outermost_loopcounter(outermost_loopcounter),
        .outermost_loopcheck(outermost_loopcheck),    
        .outermost_counter_less_than(outermost_counter_less_than),
        
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
