//----------------------------------------------------------------------
// Han
// ECE580 Final Project
// neighbor node search in a window radius from random point
// output: neighbor nodes found (1 cycle per cell iteration in window)
//----------------------------------------------------------------------

module neighbor_search
#(
    // adjustable grid parameters
    // TODO: need to move all these to dpath 
    parameter GRID_W    = 1024,
    parameter GRID_H    = 1024,
    parameter X_BITS    = 10, // log2(GRID_W)
    parameter Y_BITS    = 10, // log2(GRID_H)
    parameter N_SQUARED = 1024*1024  // log2(GRID_W*GRID_H) for flattened addr
)(
    input                       clk,
    input                       rst,

    // control
    input                      search_start,  // pulse to start a search
    input [X_BITS-1:0]         node_x,
    input [Y_BITS-1:0]         node_y,
    input [X_BITS-1:0]         window_radius, // search window radius 
    input [N_SQUARED-1:0]      occupancy_status, // base addr of grid in memory
    output reg                 neighbor_search_busy, // window loop status
    
    // detected neighbor node output (queue-style stream)
    // input                      nb_ready, // neighbor ready?
    output reg                 nb_found, // neighbor found 
    output reg [X_BITS-1:0]    nb_x,    
    output reg [Y_BITS-1:0]    nb_y

);

    // compute flattened address: y * GRID_W + x
    function [N_SQUARED-1:0] idx;
        input [X_BITS-1:0] x_coord;
        input [Y_BITS-1:0] y_coord;
        begin
            idx = y_coord * GRID_W + x_coord;
        end
    endfunction

    // TODO: this may not be the most effective window search implementation 
    //       since area of window will differ if node is near edges of grid
    // left x-coordinate 
    function [X_BITS-1:0] left_x;
        input [X_BITS-1:0] node_x;
        input [X_BITS-1:0] window_radius;
        begin           
            left_x = (node_x < window_radius) ? 0 : (node_x - window_radius);
        end
    endfunction

    // right x-coordinate
    function [X_BITS-1:0] right_x;
        input [X_BITS-1:0] node_x;
        input [X_BITS-1:0] window_radius;
        begin
            right_x = (node_x + window_radius >= GRID_W) ? (GRID_W - 1) : (node_x + window_radius);
        end
    endfunction

    // bottom y-coordinate
    function [Y_BITS-1:0] bottom_y;
        input [Y_BITS-1:0] node_y;
        input [X_BITS-1:0] window_radius;
        begin
            bottom_y = (node_y < window_radius) ? 0 : (node_y - window_radius);
        end

    endfunction

    // top y-coordinate
    function [Y_BITS-1:0] top_y;
        input [Y_BITS-1:0] node_y;
        input [X_BITS-1:0] window_radius;
        begin
            top_y = (node_y + window_radius >= GRID_H) ? (GRID_H - 1) : (node_y + window_radius);
        end
    endfunction

    reg [X_BITS-1:0] x_coord; 
    reg [Y_BITS-1:0] y_coord; 

    always @(posedge clk) begin
        if (rst) begin
            x_coord   <= {X_BITS{1'b0}};
            y_coord   <= {Y_BITS{1'b0}};
            nb_found   <= 1'b0;
            nb_x       <= {X_BITS{1'b0}};
            nb_y       <= {Y_BITS{1'b0}};
            neighbor_search_busy       <= 1'b0;
        end
        else begin
            nb_found <= 1'b0; // default

            if (search_start == 1'b1 && neighbor_search_busy == 1'b0) begin 
                x_coord <= left_x(node_x, window_radius);
                y_coord <= bottom_y(node_y, window_radius);
                neighbor_search_busy     <= 1'b1;
            end

            else if (neighbor_search_busy == 1'b1) begin // search_start == 1'b0 && neighbor_search_busy == 1'b1
                // occupancy checks --> if node found, output coordinates
                if (occupancy_status[idx(x_coord,y_coord)] == 1'b1) begin
                    nb_found <= 1'b1;
                    nb_x     <= x_coord;
                    nb_y     <= y_coord;
                end
                else begin
                    nb_found <= 1'b0;
                    nb_x     <= {X_BITS{1'b0}};
                    nb_y     <= {Y_BITS{1'b0}}; 
                end

                // loop iterations 
                // update when x-coordinate needs traversal
                if (x_coord < right_x(node_x, window_radius)) begin
                    x_coord <= x_coord + 1;
                    neighbor_search_busy     <= 1'b1;
                end
                // update when y-coordinate needs traversal
                else if (x_coord == right_x(node_x, window_radius) && 
                    y_coord < top_y(node_y, window_radius)) begin
                    x_coord <= left_x(node_x, window_radius);
                    y_coord <= y_coord + 1;
                    neighbor_search_busy     <= 1'b1;
                end
                // update when last iteration 
                else if (x_coord == right_x(node_x, window_radius) && 
                    y_coord == top_y(node_y, window_radius)) begin
                    // finished searching window
                    x_coord <= {X_BITS{1'b0}};
                    y_coord <= {Y_BITS{1'b0}}; 
                    neighbor_search_busy <= 1'b0;
                end
            end
        end
    end

endmodule