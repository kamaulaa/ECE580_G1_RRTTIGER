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
    parameter N = 1024,
    parameter N_SQUARED = N * N,
    parameter OUTERMOST_ITER_MAX = 1024, // NEED THIS?
    parameter OUTERMOST_ITER_BITS = 10, // log2(OUTERMOST_ITER_MAX)
    parameter X_BITS = 10, // log2(GRID_WIDTH)
    parameter Y_BITS = 10, // log2(GRID_HEIGHT )
    parameter ADDR_BITS = 20 // log2(GRID_WIDTH * GRID_HEIGHT) for flattened addr
)
(
    input clk,
    input reset,
    
    output failure_state,
    output traceback_state,
    output [3:0] output_state,
    
    // Inputs from the datapath
    input path_found,
    input new_point_q_collided,
    input done_draining,
    input parent_equals_current,
    input random_point_already_exists,
    input done_with_search_nearest_neighbor,
    input done_evaluating_random_point,
    input done_detecting_new_point_q_collision,
    input steered_point_in_obstacle,
    input done_checking_steered_point,
    
    // Need output control signals to the datapath
    output reg init_state,
    output reg add_edge_state,
    output reg generate_req, // to random point generator module
    output reg search_neighbor, // signal to search neighbor from random generated point 
    output reg entering_search_nearest_neighbor,
    output reg add_new_point_q,
    output reg eval_random_point,
    output reg generate_random_point,
    output reg entering_check_steered_point,
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
    localparam CHECK_STEERED_POINT = 4'b1101;

    reg [OUTERMOST_ITER_BITS-1:0] outermost_loop_counter = {OUTERMOST_ITER_BITS{1'b0}}; 
    wire outermost_loop_check = !path_found && (outermost_loop_counter <= OUTERMOST_ITER_MAX);

    reg [3:0] state;
    reg [3:0] next_state;

    assign output_state = state;

    always @ ( posedge clk ) begin
        if ( reset ) begin
            state <= INIT;
            outermost_loop_counter <= 0;
        end
        else begin
            state <= next_state;
        end
    end
    
    always @ ( posedge clk ) begin
        if ( reset ) begin
            init_state <= 1'b0;
            add_edge_state <= 1'b0;
            generate_req <= 1'b0;
            window_search_start <= 1'b0;
            search_neighbor <= 1'b0;
            entering_search_nearest_neighbor <= 1'b0;
            add_new_point_q <= 1'b0;
            eval_random_point <= 1'b0;
            generate_random_point <= 1'b0;
            entering_check_new_point_q_collision <= 1'b0;
            check_points_in_square_radius <= 1'b0;
            drain_arr <= 1'b0;
        end
        else begin
            if ( state == OUTERMOST_LOOP_CHECK ) begin
                if ( outermost_loop_check == 1'b1 ) begin
                    generate_req <= 1'b1;
                end
            end
            
            else if ( state == GENERATE_RANDOM_POINT ) begin
                generate_req <= 1'b0;
                eval_random_point <= 1'b1;
            end
            
            else if ( state == EVAL_RANDOM_POINT ) begin
                if ( done_evaluating_random_point == 1'b0 ) begin
                    eval_random_point <= 1'b1;
                end else if ( random_point_already_exists == 1'b0 ) begin
                    search_neighbor <= 1'b1;
                    entering_search_nearest_neighbor <= 1'b1;
                    eval_random_point <= 1'b0;
                end else begin // random point generated is an already existing point
                    generate_req <= 1'b1;
                    eval_random_point <= 1'b0;
                end
            end
            
            else if ( state == SEARCH_NEAREST_NEIGHBOR ) begin
                if (done_with_search_nearest_neighbor == 1'b1) begin
                    search_neighbor <= 1'b0;
                    entering_search_nearest_neighbor <= 1'b0;
                    add_new_point_q <= 1'b1;
                end else begin
                    search_neighbor <= 1'b1;
                    entering_search_nearest_neighbor <= 1'b0;
                end      
            end
            
            else if ( state == ADD_NEW_POINT_Q ) begin
                add_new_point_q <= 1'b1;
                entering_check_new_point_q_collision <= 1'b1;  
            end            
            
            else if ( state == CHECK_NEW_POINT_Q_COLLISION ) begin
                if ( done_detecting_new_point_q_collision == 1'b0 ) begin
                    entering_check_new_point_q_collision <= 1'b0;
                end else if (new_point_q_collided == 1'b1) begin
                    generate_req <= 1'b1;
                end else begin
                    add_edge_state <= 1'b1;
                end
            end
            
            else if ( state ==  ADD_EDGE ) begin
                add_edge_state <= 1'b0;
            end

            
        end
    end
       
    assign failure_state = state == FAILURE;
    assign traceback_state = state == TRACEBACK;

    always @ (*) begin
        // Default assignments to prevent latches
        next_state = state;
        
        case (state)
            INIT : begin
                init_state = 1'b1;
                next_state = OUTERMOST_LOOP_CHECK;
            end

            // iterate until path is found or max iterations limit reached
            OUTERMOST_LOOP_CHECK: begin
                // This means we still have attempts left at finding a path
                if (outermost_loop_check == 1'b1) begin 
                    next_state = GENERATE_RANDOM_POINT;
//                    generate_req = 1'b1; // request new random point
                end
                else begin // This means we either found a path or ran out of iteration attempts
                    if ( path_found ) begin // We got to the end because we successfully found a path
                        next_state = TRACEBACK;
                    end
                    else begin // We got to the end because we ran out of iteration attempts
                        next_state = FAILURE;
                    end             
                end
            end

            GENERATE_RANDOM_POINT: begin
                // always go to eval the point
                next_state = EVAL_RANDOM_POINT;
//                generate_req = 1'b0; // make sure this is a 0 to prevent the random point from being overwritten as we check it
//                eval_random_point = 1'b1;
            end
            
            EVAL_RANDOM_POINT: begin
//                eval_random_point = 1'b1;
                if ( done_evaluating_random_point == 1'b0 ) begin
                    // stay evaluating
                    next_state = EVAL_RANDOM_POINT;
                end else if ( random_point_already_exists == 1'b0 ) begin
                    next_state = SEARCH_NEAREST_NEIGHBOR;
//                    search_neighbor = 1'b1;
//                    entering_search_nearest_neighbor = 1'b1;
//                    eval_random_point = 1'b0;
                end else begin // random point generated is an already existing point
                    next_state = GENERATE_RANDOM_POINT;
//                    generate_req = 1'b1;
//                    eval_random_point = 1'b0;
                end
            end

            // there will always be a nearest neighbor
            SEARCH_NEAREST_NEIGHBOR: begin
//                search_neighbor = 1'b1;
                if (done_with_search_nearest_neighbor == 1'b1) begin
//                    search_neighbor = 1'b0;
//                    entering_search_nearest_neighbor = 1'b0;
//                    add_new_point_q = 1'b1;
                    next_state = ADD_NEW_POINT_Q; 
                end else begin
//                    entering_search_nearest_neighbor = 1'b0;
                    next_state = SEARCH_NEAREST_NEIGHBOR;
                end
            end

            // new point should be computed here too 
            ADD_NEW_POINT_Q: begin
                next_state = CHECK_STEERED_POINT;
//                add_new_point_q = 1'b1;
//                entering_check_steered_point = 1'b1; // Start fast check of steered point only
            end

            // Fast check: Is steered point inside any obstacle? (NUM_PE cycles)
            // This avoids wasting time checking all 10 neighbors if steered point itself is invalid
            CHECK_STEERED_POINT: begin
                if (done_checking_steered_point == 1'b0) begin
                    next_state = CHECK_STEERED_POINT;
                    // entering_check_steered_point = 1'b0;
                end else if (steered_point_in_obstacle == 1'b1) begin
                    // Steered point is inside obstacle - fast reject, generate new random point
                    next_state = GENERATE_RANDOM_POINT;
                    // generate_req = 1'b1;
                end else begin
                    // Steered point is valid - proceed to check all neighbor connections
                    next_state = CHECK_NEW_POINT_Q_COLLISION;
                    // entering_check_new_point_q_collision = 1'b1;
                end
            end

            // check collision of new point q with obstacles via systolic array
            // this is also where new point q will be added to occupancy array, occupancy grid, and its parent node (the nearest neighbor) will be added to the occupancy array
            // if collision occurs, new point q is not added and new random point is generated 
            CHECK_NEW_POINT_Q_COLLISION: begin
                if ( done_detecting_new_point_q_collision == 1'b0 ) begin
                    next_state = CHECK_NEW_POINT_Q_COLLISION;
//                    entering_check_new_point_q_collision = 1'b0;
                end else if (new_point_q_collided == 1'b1) begin
                    next_state = GENERATE_RANDOM_POINT;
//                    generate_req = 1'b1;
                end else begin
                    next_state = ADD_EDGE;
//                    add_edge_state = 1'b1;
                end
            end

            ADD_EDGE: begin
//                add_edge_state = 1'b1;
                next_state = OUTERMOST_LOOP_CHECK;
            end

            FAILURE: begin
                next_state = FAILURE;
            end

            TRACEBACK: begin
                if (parent_equals_current == 1'b1) begin // has not been implemented
                    next_state = SUCCESS;
                end else begin
                    next_state = TRACEBACK;
                end
            end

            SUCCESS: begin
                next_state = SUCCESS;
            end

            default: begin
                next_state = INIT;
            end

        endcase
    end
    
    // Sequential counter update
    always @(posedge clk) begin
        if (reset) begin
            outermost_loop_counter <= {OUTERMOST_ITER_BITS{1'b0}};
        end else if (state == ADD_EDGE) begin
            outermost_loop_counter <= outermost_loop_counter + 1'b1;
        end
    end

endmodule