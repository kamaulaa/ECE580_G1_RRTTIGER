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
    parameter OUTERMOST_ITER_MAX = N, // NEED THIS?
    parameter X_BITS = 10, // log2(GRID_WIDTH)
    parameter Y_BITS = 10, // log2(GRID_HEIGHT )
    parameter ADDR_BITS = 20 // log2(GRID_WIDTH * GRID_HEIGHT) for flattened addr
)
(
    input clk,
    input reset,
    
    // Inputs from the datapath
    input path_found,
    input point_hit,
    input done_draining,
    input parent_equals_current,
    input new_random_point_valid,
    input neighbor_search_busy,
    
    // Need output control signals to the datapath
    output init_state,
    output add_edge_state,
    output outer_loop_check_state,
    // output reg [N_SQUARED-1:0] inner_loop_counter,
    output reg generate_req, // to random point generator module
    output reg search_start // to neighbor search module 
);

// Define states
localparam INIT = 4'b0000;
localparam OUTERMOST_LOOP_CHECK = 4'b0001;
localparam GENERATE_RANDOM_POINT = 4'b0010;
localparam CHECK_POINTS_IN_SQUARE_RADIUS = 4'b0011;
localparam DRAIN_ARR = 4'b0100;
localparam ADD_EDGE = 4'b0101;
localparam FAILURE = 4'b0110;
localparam TRACEBACK = 4'b0111;
localparam SUCCESS = 4'b1000;

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

// REALLY DIRTY (WRONG) COMBINATION AND SEQUENTIAL LOGIC MIX GOING ON HERE...
always @ (*) begin
    case (state)
        INIT : begin
            // Decide next state
            next_state <= OUTERMOST_LOOP_CHECK;
            
            // TODO: Set any relevant output signals to the datapath
            // vertices_reset <= 1'b1; // reset the regs holding the points
            // edges_reset <= 1'b1; // reset the regs holding the edges
            
        end

        OUTERMOST_LOOP_CHECK: begin
            // This means we still have attempts left at finding a path
            if ( outermost_loop_check == 1'b1) begin // HAN: check if in neighbot searching loop?
                // Decide next state
                next_state <= GENERATE_RANDOM_POINT;
                generate_req <= 1'b1; // request new random point
                
                // Set any relevant output signals to the datapath
                // not sure what we need here yet
            end
            else begin // This means we either found a path or ran out of iteration attempts
                // Decide next state
                if ( path_found ) begin // We got to the end because we successfully found a path
                    next_state <= SUCCESS;
                end
                else begin // We got to the end because we ran out of iteration attempts
                    next_state <= FAILURE;
                end
                
                // Set any relevant output signals to the datapath
                // not sure what we need here yet                
            end
        end

        GENERATE_RANDOM_POINT: begin
            // Decide next state
            if (new_random_point_valid == 1'b1) begin // new random point generated 
                next_state <= CHECK_POINTS_IN_SQUARE_RADIUS;
                generate_req <= 1'b0;
                search_start <= 1'b1; // start neighbor search
            end
            else begin // random point generated is an already existing point
                next_state <= GENERATE_RANDOM_POINT;
                generate_req <= 1'b1;
            end
            
            // Set any relevant output signals to the datapath
            // not sure what we need here yet
        end

        CHECK_POINTS_IN_SQUARE_RADIUS: begin
            if (search_start == 1'b1 && neighbor_search_busy == 1'b0) begin
                next_state <= CHECK_POINTS_IN_SQUARE_RADIUS;
                search_start <= 1'b0; 
            end
            else if (neighbor_search_busy == 1'b1) begin
                next_state <= CHECK_POINTS_IN_SQUARE_RADIUS;
            end
            else begin // HAN TODO: not sure what this logic is doing
                if (done_draining == 1'b1) begin
                    next_state <= DRAIN_ARR;
                end else begin
                    next_state <= ADD_EDGE;
                end
            end
        end

        // HAN TODO: get rid of?
        DRAIN_ARR: begin
            if (done_draining == 1'b1) begin
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