//----------------------------------------------------------------------
// Han
// ECE580 Final Project
// neighbor node search in a window radius from new point on grid 
// this is after a new point is added at after random point generation and collision check 
// purpose is to minimize rewiring cross 
// output: neighbor nodes found (1 cycle per cell iteration in window) --> NEEDS UPDATE AFTER 11/20 CHANGE DISCUSSION
//----------------------------------------------------------------------

module window_frame_search
#(
    // adjustable grid parameters
    // TODO: need to move all these to dpath 
    parameter N = 1024,
    parameter COORDINATE_WIDTH = 10, // log2(N)
    parameter N_SQUARED = 1024*1024,  // log2(N*N) for flattened addr
    parameter OUTERMOST_ITER_MAX = 1024, // adjustable limit
    parameter ARRAY_WIDTH = COORDINATE_WIDTH*4 // occupancy status array width (1 bit per cell)
)(
    input                       clk,
    input                       rst,

    // control
    input                      search_start,  // pulse to start a search
    input [COORDINATE_WIDTH-1:0]         node_x,
    input [COORDINATE_WIDTH-1:0]         node_y,
    input [COORDINATE_WIDTH-1:0]         window_radius, // search window radius 
    // TODO NEED UNROLLED IMPLEMENTATION FOR THIS
    input [N_SQUARED-1:0]    occupancy_status_grid, // occupancy status grid
    output reg                 neighbor_search_busy, // window loop status
    
    // detected neighbor node output (queue-style stream)
    // input                      nb_ready, // neighbor ready?
    output reg                 nb_found, // neighbor found 
    output reg [COORDINATE_WIDTH-1:0]    nb_x,    
    output reg [COORDINATE_WIDTH-1:0]    nb_y

);

    // compute flattened address: y * N + x
    function [N_SQUARED-1:0] idx;
        input [COORDINATE_WIDTH-1:0] x_coord;
        input [COORDINATE_WIDTH-1:0] y_coord;
        begin
            idx = y_coord * N + x_coord;
        end
    endfunction

    // TODO: this may not be the most effective window search implementation 
    //       since area of window will differ if node is near edges of grid
    // left x-coordinate 
    function [COORDINATE_WIDTH-1:0] left_x;
        input [COORDINATE_WIDTH-1:0] node_x;
        input [COORDINATE_WIDTH-1:0] window_radius;
        begin           
            left_x = (node_x < window_radius) ? 0 : (node_x - window_radius);
        end
    endfunction

    // right x-coordinate
    function [COORDINATE_WIDTH-1:0] right_x;
        input [COORDINATE_WIDTH-1:0] node_x;
        input [COORDINATE_WIDTH-1:0] window_radius;
        begin
            right_x = (node_x + window_radius >= N) ? (N - 1) : (node_x + window_radius);
        end
    endfunction

    // bottom y-coordinate
    function [COORDINATE_WIDTH-1:0] bottom_y;
        input [COORDINATE_WIDTH-1:0] node_y;
        input [COORDINATE_WIDTH-1:0] window_radius;
        begin
            bottom_y = (node_y < window_radius) ? 0 : (node_y - window_radius);
        end

    endfunction

    // top y-coordinate
    function [COORDINATE_WIDTH-1:0] top_y;
        input [COORDINATE_WIDTH-1:0] node_y;
        input [COORDINATE_WIDTH-1:0] window_radius;
        begin
            top_y = (node_y + window_radius >= N) ? (N - 1) : (node_y + window_radius);
        end
    endfunction

    reg [COORDINATE_WIDTH-1:0] x_coord; 
    reg [COORDINATE_WIDTH-1:0] y_coord; 

    always @(posedge clk) begin
        if (rst) begin
            x_coord   <= {COORDINATE_WIDTH{1'b0}};
            y_coord   <= {COORDINATE_WIDTH{1'b0}};
            nb_found   <= 1'b0;
            nb_x       <= {COORDINATE_WIDTH{1'b0}};
            nb_y       <= {COORDINATE_WIDTH{1'b0}};
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
                if (occupancy_status_grid[idx(x_coord,y_coord)] == 1'b1) begin
                    nb_found <= 1'b1;
                    nb_x     <= x_coord;
                    nb_y     <= y_coord;
                end
                else begin
                    nb_found <= 1'b0;
                    nb_x     <= {COORDINATE_WIDTH{1'b0}};
                    nb_y     <= {COORDINATE_WIDTH{1'b0}}; 
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
                    x_coord <= {COORDINATE_WIDTH{1'b0}};
                    y_coord <= {COORDINATE_WIDTH{1'b0}}; 
                    neighbor_search_busy <= 1'b0;
                end
            end
        end
    end

endmodule