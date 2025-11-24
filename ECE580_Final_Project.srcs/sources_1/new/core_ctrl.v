`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////

// Create Date: 11/15/2025 10:31:48 AM
// Design Name: 
// Module Name: core_ctrl
// Project Name: ECE580 Final Project

//////////////////////////////////////////////////////////////////////////////////

module core_ctrl
#(
    // ADJUSTABLE GRID PARAMETERS
    // TODO: need to pass these from core into dpath and control modules 
    parameter N = 1024,
    parameter N_SQUARED = N * N,
    parameter OUTERMOST_ITER_MAX = 1024, // NEED THIS?
    parameter X_BITS = 10, // log2(GRID_WIDTH)
    parameter Y_BITS = 10, // log2(GRID_HEIGHT )
    parameter ADDR_BITS = 20 // log2(GRID_WIDTH * GRID_HEIGHT) for flattened addr
)
(
    input clk,
    input reset,
    
    // Inputs from the datapath
    input path_found,
    input new_point_q_collided,
    input done_draining,
    input parent_equals_current,
    input random_point_already_exists,
    input window_search_busy,
    input new_point_created,
    input done_with_search_nearest_neighbor,
    input done_evaluating_new_random_point,
    input done_detecting_new_point_q_collision,
    
    // Need output control signals to the datapath
    output reg generate_req, // to random point generator module
    output reg window_search_start, // to window search module 
    output reg search_neighbor, // signal to search neighbor from random generated point 
    output reg entering_search_nearest_neighbor,
    output reg add_new_point_q,
    output reg eval_random_point,
    output reg generate_random_point,
    output reg entering_check_new_point_q_collision,
    output reg check_points_in_square_radius,
    output reg drain_arr
    
);

    // Define states
    localparam INIT = 4'b0000;
    localparam OUTERMOST_LOOP_CHECK = 4'b0001;
    localparam GENERATE_RANDOM_POINT = 4'b0010;
    localparam SEARCH_NEAREST_NEIGHBOR = 4'b0011;
    localparam CHECK_NEW_POINT_Q_COLLISION = 4'b0100;
    localparam CHECK_POINTS_IN_SQUARE_RADIUS = 4'b0101;
    localparam DRAIN_ARR = 4'b0110;
    localparam ADD_EDGE = 4'b0111;
    localparam FAILURE = 4'b1000;
    localparam TRACEBACK = 4'b1001;
    localparam SUCCESS = 4'b1010;
    localparam ADD_NEW_POINT_Q = 4'b1011;
    localparam EVAL_RANDOM_POINT = 4'b1100;

    reg [N_SQUARED-1:0] outermost_loop_counter = {OUTERMOST_ITER_MAX{1'b0}}; 
    wire outermost_loop_check = !path_found && (outermost_loop_counter <= OUTERMOST_ITER_MAX);

    reg [3:0] state;
    reg [3:0] next_state;

    always @ ( posedge clk ) begin
        if ( reset ) begin
            state <= INIT;
        end
        else begin
            state <= next_state;
        end
    end

    always @ (*) begin
        case (state)
            INIT : begin
                next_state <= OUTERMOST_LOOP_CHECK;
            end

            // iterate until path is found or max iterations limit reached
            OUTERMOST_LOOP_CHECK: begin
                // This means we still have attempts left at finding a path
                if ( outermost_loop_check == 1'b1) begin // HAN: check if in neighbot searching loop?
                    next_state <= GENERATE_RANDOM_POINT;
                    generate_req <= 1'b1; // request new random point
                end
                else begin // This means we either found a path or ran out of iteration attempts
                    if ( path_found ) begin // We got to the end because we successfully found a path
                        next_state <= TRACEBACK;
                    end
                    else begin // We got to the end because we ran out of iteration attempts
                        next_state <= FAILURE;
                    end             
                end
            end

            GENERATE_RANDOM_POINT: begin
                // always go to eval the point
                next_state <= EVAL_RANDOM_POINT;
                generate_req <= 1'b0; // make sure this is a 0 to prevent the random point from being overwritten as we check it
                eval_random_point <= 1'b1;
            end
            
            EVAL_RANDOM_POINT: begin
                if ( done_evaluating_new_random_point == 1'b0 ) begin
                    // stay evaluating
                    next_state <= EVAL_RANDOM_POINT;
                end else if ( random_point_already_exists == 1'b0 ) begin
                    next_state <= SEARCH_NEAREST_NEIGHBOR;
                    search_neighbor <= 1'b1;
                    entering_search_nearest_neighbor <= 1'b1;
                    eval_random_point <= 1'b0;
                end else begin // random point generated is an already existing point
                    next_state <= GENERATE_RANDOM_POINT;
                    generate_req <= 1'b1;
                    eval_random_point <= 1'b0;
                end
            end

            // there will always be a nearest neighbor
            SEARCH_NEAREST_NEIGHBOR: begin
                if (done_with_search_nearest_neighbor == 1'b1) begin
                    search_neighbor <= 1'b0;
                    entering_search_nearest_neighbor <= 1'b0;
                    add_new_point_q <= 1'b1;
                    next_state <= ADD_NEW_POINT_Q; 
                end else begin
                    entering_search_nearest_neighbor <= 1'b0;
                    next_state <= SEARCH_NEAREST_NEIGHBOR;
                end
            end

            // new point should be computed here too 
            ADD_NEW_POINT_Q: begin
                next_state <= CHECK_NEW_POINT_Q_COLLISION;
                add_new_point_q <= 1'b0;
                entering_check_new_point_q_collision <= 1'b1; // turn this on for only 1 cycle to let the systolic array take in the new point q and immediately not take any more input
            end

            // check collision of new point q with obstacles via systolic array
            // this is also where new point q will be added to occupancy array, occupancy grid, and its parent node (the nearest neighbor) will be added to the occupancy array
            // if collision occurs, new point q is not added and new random point is generated 
            CHECK_NEW_POINT_Q_COLLISION: begin
                if ( done_detecting_new_point_q_collision == 1'b0 ) begin
                    next_state <= CHECK_NEW_POINT_Q_COLLISION;
                    entering_check_new_point_q_collision <= 1'b0;
                end else if (new_point_q_collided == 1'b1) begin
                    next_state <= GENERATE_RANDOM_POINT;
                    generate_req <= 1'b1;
                end else begin
                    next_state <= CHECK_POINTS_IN_SQUARE_RADIUS;
                    window_search_start <= 1'b1;
                    check_points_in_square_radius <= 1'b1;
                end
            end

            // check if their are any neighbors in the square window radius around new node
            CHECK_POINTS_IN_SQUARE_RADIUS: begin
                if (window_search_start == 1'b1 && window_search_busy == 1'b0) begin
                    next_state <= CHECK_POINTS_IN_SQUARE_RADIUS;
                    window_search_start <= 1'b0; 
                end else if (window_search_busy == 1'b1) begin
                    next_state <= CHECK_POINTS_IN_SQUARE_RADIUS;
                end else begin //  KAMUALA TODO: update this logic
                    check_points_in_square_radius <= 1'b0;
                    if (done_draining == 1'b1) begin
                        drain_arr <= 1'b1;
                        next_state <= DRAIN_ARR;
                    end else begin
                        next_state <= ADD_EDGE;
                    end
                end
            end

            // KAMUALA TODO: combine DRAIN_ARR and ADD_EDGE states?
            // compare costs of neighbors found in window radius and add edge to best one
            DRAIN_ARR: begin
                if (done_draining == 1'b1) begin
                    drain_arr <= 1'b0;
                    next_state <= ADD_EDGE;
                end else begin
                    next_state <= DRAIN_ARR;
                end
            end
            
            ADD_EDGE: begin
                next_state <= OUTERMOST_LOOP_CHECK;
                outermost_loop_counter <= outermost_loop_counter + 1'b1;
            end

            FAILURE: begin
                next_state <= FAILURE; // not sure how to end the simulation --> i think to end the simulation we use a $finish flag in the testbench
                // thus this should be sufficient to keep us in the next state
            end

            TRACEBACK: begin
                if (parent_equals_current == 1'b1) begin
                    next_state <= SUCCESS;
                end else begin
                    next_state <= TRACEBACK;
                end
            end

            SUCCESS: begin
                next_state <= SUCCESS;
            end

        endcase
    end

endmodule