//----------------------------------------------------------------------
// Han
// neighbor node search in a window radius from random point
// output: neighbor nodes found (1 cycle per cell iteration in window)
//----------------------------------------------------------------------

module neighbor_search
#(
    // adjustable grid parameters
    // TODO: need to move all these to dpath 
    parameter GRID_W    = 64,
    parameter GRID_H    = 64,
    parameter X_BITS    = 6, // log2(GRID_W)
    parameter Y_BITS    = 6, // log2(GRID_H)
    parameter ADDR_BITS = 12  // log2(GRID_W*GRID_H) for flattened addr
)(
    input                       clk,
    input                       rst,

    // control
    input                      search_start,  // pulse to start a search
    input [X_BITS-1:0]         node_x,
    input [Y_BITS-1:0]         node_y,
    input [X_BITS-1:0]         window_radius, // search window radius 
    input [ADDR_BITS-1:0]      occupancy_status, // base addr of grid in memory
    output reg                 busy, // window loop status
    
    // detected neighbor node output (queue-style stream)
    input                      nb_ready, // neighbor ready?
    output reg                 nb_found, // neighbor found 
    output reg [X_BITS-1:0]    nb_x,    
    output reg [Y_BITS-1:0]    nb_y

);

    // compute flattened address: y * GRID_W + x
    function [ADDR_BITS-1:0] idx;
        input [X_BITS-1:0] x;
        input [Y_BITS-1:0] y;
        begin
            idx = y * GRID_W + x;
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

    reg [X_BITS-1:0] x_search; 
    reg [Y_BITS-1:0] y_search; 

    always @(posedge clk) begin
        if (rst) begin
            x_search   <= {X_BITS{1'b0}};
            y_search   <= {Y_BITS{1'b0}};
            nb_found   <= 1'b0;
            nb_x       <= {X_BITS{1'b0}};
            nb_y       <= {Y_BITS{1'b0}};
            busy       <= 1'b0;
        end
        else begin
            nb_found <= 1'b0; // default

            // TODO: NEED TO RESET search_start SIGNAL IN DPATH AFTER FIRST ITERATION (if busy == 1'b1 then search_start == 1'b0)
            if (search_start == 1'b1 && busy == 1'b0) begin 
                x_search <= left_x(node_x, window_radius);
                y_search <= bottom_y(node_y, window_radius);
                busy     <= 1'b1;
            end

            else if (busy == 1'b1) begin // search_start == 1'b0 && busy == 1'b1
                // occupancy checks --> if node found, output coordinates
                if (occupancy_status[idx(x_search,y_search)] == 1'b1) begin
                    nb_found <= 1'b1;
                    nb_x     <= x_search;
                    nb_y     <= y_search;
                end
                else begin
                    nb_found = 1'b0;
                    nb_x     <= {X_BITS{1'b0}};
                    nb_y     <= {Y_BITS{1'b0}}; 
                end

                // loop iterations 
                // update when x-coordinate needs traversal
                if (x_search < right_x(node_x, window_radius)) begin
                    x_search <= x_search + 1;
                    busy     <= 1'b1;
                end
                // update when y-coordinate needs traversal
                else if (x_search == right_x(node_x, window_radius) && 
                    y_search < top_y(node_y, window_radius)) begin
                    x_search <= left_x(node_x, window_radius);
                    y_search <= y_search + 1;
                    busy     <= 1'b1;
                end
                // update when last iteration 
                else if (x_search == right_x(node_x, window_radius) && 
                    y_search == top_y(node_y, window_radius)) begin
                    // finished searching window
                    x_search <= {X_BITS{1'b0}};
                    y_search <= {Y_BITS{1'b0}}; 
                    busy <= 1'b0;
                end
            end
        end
    end

endmodule