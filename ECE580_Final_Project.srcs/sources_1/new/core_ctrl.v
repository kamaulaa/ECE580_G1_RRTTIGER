`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////

// Create Date: 11/15/2025 10:31:48 AM
// Design Name: 
// Module Name: core_ctrl
// Project Name: 
 
//////////////////////////////////////////////////////////////////////////////////

module core_ctrl
(
    input clk,
    input reset,
    
    // Inputs from the datapath
    input path_found,
    input point_hit,
    input done_draining,
    input fifo_full,
    input fifo_empty,
    input parent_equals_current,
    
    // Need output control signals to the datapath
    output init_state,
    output rd_fifo,
    output put_item_in_fifo,
    output add_edge_state,
    output sample_point_state,
    output outer_loop_check_state
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

localparam N = 1024;
localparam N_SQUARED = N * N;
localparam OUTERMOST_ITER_MAX = 1024;

reg [9:0] outermost_loop_counter = 10'b0;
wire outermost_loop_check = !path_found && outermost_loop_counter <= OUTERMOST_ITER_MAX;

reg [2:0] state;
wire [2:0] next_state;

// Define control variables?

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
            // Decide next state
            next_state <= OUTERMOST_LOOP_CHECK;
            
            // Set any relevant output signals to the datapath
            vertices_reset <= 1'b1; // reset the regs holding the points
            edges_reset <= 1'b1; // reset the regs holding the edges
            
        end
        OUTERMOST_LOOP_CHECK: begin
            if ( outermost_loop_check == 1'b1 ) begin // This means we still have attempts left at finding a path
                // Decide next state
                next_state <= GENERATE_RANDOM_POINT;
                
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
            next_state <= CHECK_POINTS_IN_SQUARE_RADIUS;
            
            // Set any relevant output signals to the datapath
            // not sure what we need here yet
        end
        CHECK_POINTS_IN_SQUARE_RADIUS: begin
            if (fifo_empty == 1'b0) begin
                rd_fifo <= 1'b1;
            end else begin
                rd_fifo <= 1'b0; 
            end
        
            if (inner_loop_counter < N_SQUARED) begin
                if (point_hit == 1'b1) begin
                    if (fifo_full == 1'b1) begin
                        next_state <= CHECK_POINTS_IN_SQUARE_RADIUS;
                        put_item_in_fifo <= 1'b0;
                        inner_loop_counter <= inner_loop_counter;
                    end else begin
                        next_state <= CHECK_POINTS_IN_SQUARE_RADIUS;
                        put_item_in_fifo <= 1'b1;
                        inner_loop_counter <= inner_loop_counter + 1'b1;
                    end
                end else begin
                    next_state <= CHECK_POINTS_IN_SQUARE_RADIUS;
                    put_item_in_fifo <= put_item_in_fifo;
                    inner_loop_counter <= inner_loop_counter + 1'b1;
                end
            end else begin
                if (done_draining == 1'b1) begin
                    next_state <= DRAIN_ARR;
                end else begin
                    next_state <= ADD_EDGE;
                end
            end
        end
        DRAIN_ARR: begin
            if (done_draining == 1'b1) begin
                next_state <= ADD_EDGE;
            end else begin
                next_state <= DRAIN_ARR;
            end
        end
        ADD_EDGE: begin
            next_state <= OUTERMOST_LOOP_COUNTER;
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

