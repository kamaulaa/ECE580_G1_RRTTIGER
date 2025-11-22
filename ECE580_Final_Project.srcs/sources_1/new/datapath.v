`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 11/18/2025 09:05:34 PM
// Design Name: 
// Module Name: datapath
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
//////////////////////////////////////////////////////////////////////////////////


module datapath #(
    parameter COORDINATE_WIDTH = 10,
    parameter NUM_PE = 10,
    parameter N = 1024,
    parameter N_SQUARED = N * N,  // 1024 * 1024 = 1,048,576
    parameter OUTERMOST_ITER_MAX = 1024,
    parameter OUTERMOST_ITER_BITS = 10, // log2(OUTERMOST_ITER_MAX)
    parameter COST_WIDTH = 16,  // bits for accumulated cost storage - THIS IS AN ESTIMATE IDKK                 // for 1024x1024 grid: max single edge = 2046, max accumulated ~32 edges = 65,472 (needs 16 bits)
    parameter ADDR_BITS = 20    // log2(N_SQUARED) = log2(1,048,576) = 20
)(
    input clk,
    input reset,
    
    // Start and goal points (from testbench)
    input wire [COORDINATE_WIDTH-1:0] start_x,
    input wire [COORDINATE_WIDTH-1:0] start_y,
    input wire [COORDINATE_WIDTH-1:0] goal_top_bound, // use a bound for goal instead of specific point
    input wire [COORDINATE_WIDTH-1:0] goal_bottom_bound,
    input wire [COORDINATE_WIDTH-1:0] goal_left_bound,
    input wire [COORDINATE_WIDTH-1:0] goal_right_bound,
    
    // Obstacle data inputs (from testbench)
    input wire [NUM_PE*COORDINATE_WIDTH-1:0] obs_left,
    input wire [NUM_PE*COORDINATE_WIDTH-1:0] obs_right,
    input wire [NUM_PE*COORDINATE_WIDTH-1:0] obs_top,
    input wire [NUM_PE*COORDINATE_WIDTH-1:0] obs_bottom,
    
    // Dpath -> control
    output path_found,
    output point_hit,
    output done_draining,
    output parent_equals_current,
    output reg new_random_point_valid, // valid random point
    output window_search_busy, // already looking for nearest neighbor
    output reg nearest_neighbor_found,
    
    // Control -> dpath
    input init_state,
    input add_edge_state,
    input outer_loop_check_state,
    input generate_req, // to random point generator module
    input window_search_start, // to window search module 
    input search_neighbor // signal to search neighbor from random generated point
);

////////////////////////////////////////////////////////////////////////
// SIGNAL DECLARATIONS

// Random point registers
reg [COORDINATE_WIDTH-1:0] x_rand; // register that holds the output of the random number generator 
reg [COORDINATE_WIDTH-1:0] y_rand; 


// Minimum cost point registers
reg [COORDINATE_WIDTH-1:0] x_min; // coordinates of nearest neighbor with min cost for this iteration of radius/window search
reg [COORDINATE_WIDTH-1:0] y_min;
reg [COST_WIDTH-1:0] c_min; // minimum cost found so far

// Neighbor search signals
wire nb_found; // a nearest neighbor was found in the window
wire [COORDINATE_WIDTH-1:0] nb_x; // coords of that nearest neighbor
wire [COORDINATE_WIDTH-1:0] nb_y;

// Control signals
wire valid_in = new_random_point_valid && nb_found; // valid when we have both valid random point and neighbor

// Cost calculation signals
wire [COST_WIDTH-1:0] rd_cost; // TODO: connect to cost memory read data for nearest neighbor location
wire [COST_WIDTH-1:0] calculated_cost; // new connection cost from quantization block
wire [COST_WIDTH-1:0] total_cost = calculated_cost + rd_cost;
wire update_min_point = (total_cost < c_min) ? 1'b1 : 1'b0;

// Systolic array signals
wire systolic_valid_out;
wire systolic_valid_pair;
wire [COORDINATE_WIDTH-1:0] systolic_val_x1, systolic_val_y1; // non-collided point pair (random is 1 and nearest is 2)
wire [COORDINATE_WIDTH-1:0] systolic_val_x2, systolic_val_y2;

// Pipeline registers to delay systolic outputs by 1 cycle to match quantization block latency
reg systolic_valid_pair_q;
reg [COORDINATE_WIDTH-1:0] systolic_val_x1_q, systolic_val_y1_q;
reg [COORDINATE_WIDTH-1:0] systolic_val_x2_q, systolic_val_y2_q;

////////////////////////////////////////////////////////////////////////
// SYSTOLIC ARRAY PIPELINE REGISTERS

always @(posedge clk) begin
    if (reset) begin
        systolic_valid_pair_q <= 0;
        systolic_val_x1_q <= 0;
        systolic_val_y1_q <= 0;
        systolic_val_x2_q <= 0;
        systolic_val_y2_q <= 0;
    end else begin
        systolic_valid_pair_q <= systolic_valid_pair;
        systolic_val_x1_q <= systolic_val_x1;
        systolic_val_y1_q <= systolic_val_y1;
        systolic_val_x2_q <= systolic_val_x2;
        systolic_val_y2_q <= systolic_val_y2;
    end
end

////////////////////////////////////////////////////////////////////////
// QUANTIZATION BLOCK - COST CALCULATION

// quantization block for cost calculation
quantization_block #(.COORDINATE_WIDTH(COORDINATE_WIDTH), .COST_WIDTH(COST_WIDTH)) quantized_cost (
    .clk(clk),
    .rst(reset),
    .r_x1(systolic_val_x1),      // random point x
    .r_y1(systolic_val_y1),      // random point y
    .n_x2(systolic_val_x2),      // nearest neighbor x (with no collisions)
    .n_y2(systolic_val_y2),      // nearest neighbor y (with no collisions)
    .cost_out(calculated_cost)   // quantized distance/cost output
);

////////////////////////////////////////////////////////////////////////
// OBSTACLE COLLISION DETECTION - SYSTOLIC ARRAY

// obstacle detection systolic array - constantly being fed the newly calculated ranom points and their nearest neighbors
oc_array #(.COORDINATE_WIDTH(COORDINATE_WIDTH), .NUM_PE(NUM_PE)) pe_array (
            .clk(clk),
            .rst(reset),
            .obs_left(obs_left),
            .obs_right(obs_right),
            .obs_top(obs_top),
            .obs_bottom(obs_bottom),
            .r_x1(x_rand),
            .r_y1(y_rand),
            .n_x2(nb_x),
            .n_y2(nb_y),
            .valid_in(valid_in),
            .valid_out(systolic_valid_out),
            .valid_pair(systolic_valid_pair),
            .val_x1(systolic_val_x1),
            .val_y1(systolic_val_y1), 
            .val_x2(systolic_val_x2),
            .val_y2(systolic_val_y2)
        );

////////////////////////////////////////////////////////////////////////
// GOAL CHECK LOGIC

// Goal check: check if current point is within goal bounds AND doesn't collide with obstacles
// Use delayed systolic outputs to match the timing of calculated_cost from quantization block
wire goal_reached = (systolic_val_x1_q < goal_right_bound) && (systolic_val_x1_q > goal_left_bound) && (systolic_val_y1_q < goal_top_bound) && (systolic_val_y1_q > goal_bottom_bound);
assign path_found = goal_reached && systolic_valid_pair_q; // Only set path_found if we reach goal AND connection is collision-free

////////////////////////////////////////////////////////////////////////
// CONTROL SIGNALS

assign done_draining = ~(nb_found || systolic_valid_out || systolic_valid_pair); // not sure if nb_found is correct here to replace rd_fifo

////////////////////////////////////////////////////////////////////////
// VERTICES GRID 

// Han: the "add_edge_state" signal should serve as the write enable signal to update the vertex grid (occupancy_status) 
// because it tells us that we're in the state where we want to record the new random point and its optimal parent
// The "add edge state" signal should also be used as the write enable signal for the costs grid and parent grid

// NEED STARTING POINT TO BE FIRST POINT ADDED TO OCCUPANCY STATUS

// array to store points in grid 
reg [N_SQUARED-1:0] occupancy_status; // gradually take away as clear up code

localparam ARRAY_WIDTH = COORDINATE_WIDTH*4; // x_coord + y_coord + parent_x + parent_y
localparam X_MSB       = ARRAY_WIDTH - 1;
localparam X_LSB       = X_MSB - (COORDINATE_WIDTH-1);
localparam Y_MSB       = X_LSB - 1;
localparam Y_LSB       = Y_MSB - (COORDINATE_WIDTH-1);
localparam PX_MSB      = Y_LSB - 1;
localparam PX_LSB      = PX_MSB - (COORDINATE_WIDTH-1);
localparam PY_MSB      = PX_LSB - 1;
localparam PY_LSB      = PY_MSB - (COORDINATE_WIDTH-1);

reg [ARRAY_WIDTH-1:0] occupied_points_array [0:OUTERMOST_ITER_MAX-1];
reg [OUTERMOST_ITER_BITS-1:0] occupied_array_count; // counts number of occupied points stored for array indexing

// wire [COORDINATE_WIDTH-1:0] x_coord  = occupied_points_array[occupied_array_count][X_MSB : X_LSB];
// wire [COORDINATE_WIDTH-1:0] y_coord  = occupied_points_array[occupied_array_count][Y_MSB : Y_LSB];
// wire [COORDINATE_WIDTH-1:0] parent_x = occupied_points_array[occupied_array_count][PX_MSB : PX_LSB];
// wire [COORDINATE_WIDTH-1:0] parent_y = occupied_points_array[occupied_array_count][PY_MSB : PY_LSB];

// compute flattened address : y * GRID_W + x
// TODO may need to change to bit shifting instead of multiplication 
function [N_SQUARED-1:0] idx;
    input [COORDINATE_WIDTH-1:0] x_coord;
    input [COORDINATE_WIDTH-1:0] y_coord;
    begin
        idx = y_coord * N + x_coord;
    end
endfunction

////////////////////////////////////////////////////////////////////////
// RANDOM GENERATOR 

    // wires from random generator
    wire [COORDINATE_WIDTH-1:0] x_rand_wire;
    wire [COORDINATE_WIDTH-1:0] y_rand_wire;
    // instantiate random point generator
    rrt_random_point #(
        .X_BITS(COORDINATE_WIDTH),
        .Y_BITS(COORDINATE_WIDTH),
        .ADDR_BITS(ADDR_BITS)
    ) generate_point (
        .clk          (clk),
        .rst          (reset),
        .generate_req (generate_req),
        .random_point_x       (x_rand_wire),
        .random_point_y       (y_rand_wire)
    );

    // check if generated random point already exists
    always @(*) begin
        points_already_exists = 1'b0; // default to invalid
        for (integer i = 0; i < occupied_array_count; i = i + 1) begin
            points_already_exists = points_already_exists || ((occupied_points_array[i][X_MSB : X_LSB] == x_rand_wire) 
            && (occupied_points_array[i][Y_MSB : Y_LSB] == y_rand_wire));
        end
    end

    // Update x_rand, y_rand, and occupancy_status
    always @( posedge clk ) begin
        if (reset) begin
            x_rand <= {COORDINATE_WIDTH{1'b0}};
            y_rand <= {COORDINATE_WIDTH{1'b0}};
            new_random_point_valid <= 1'b0;
            occupancy_status <= {N_SQUARED{1'b0}};
        end 
        else begin
            new_random_point_valid <= 1'b0; // default
            if (generate_req==1'b1 && points_already_exists==1'b0) begin           
                x_rand <= x_rand_wire;
                y_rand <= y_rand_wire;
                new_random_point_valid <= 1'b1;
            end
        end
    end
////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////
// NEARBOR NEIGHBOR SEARCH 
// search across all existing points for nearest point to randomly generated point 

    reg [COORDINATE_WIDTH-1:0] x_i, y_i;
    reg [COORDINATE_WIDTH:0] dx, dy; // |x1-x2|, |y1-y2|
    reg [COORDINATE_WIDTH*2+1:0] distance; // dx^2 + dy^2
    reg [COORDINATE_WIDTH-1:0] best_x, best_y;
    reg [OUTERMOST_ITER_BITS-1:0] best_index;
    reg [COORDINATE_WIDTH*2+1:0] best_dist;

    reg [COORDINATE_WIDTH:0] nearest_neighbor_x;
    reg [COORDINATE_WIDTH:0] nearest_neighbor_y;
    reg [OUTERMOST_ITER_BITS-1:0] nearest_neighbor_index;

    // combinational logic done in 1 cycle?
    always @(*) begin
        best_dist  = {COORDINATE_WIDTH*2+1{1'b1}}; // max distance to start comparison
        best_x     = {COORDINATE_WIDTH{1'b0}};
        best_y     = {COORDINATE_WIDTH{1'b0}};
        best_index = {OUTERMOST_ITER_BITS{1'b0}};

        if (search_neighbor==1'b1 && (occupied_array_count != 0)) begin
            for (i = 0; i < occupied_array_count; i = i + 1) begin
                x_i = occupied_points_array[i][X_MSB : X_LSB];
                y_i = occupied_points_array[i][Y_MSB : Y_LSB];

                // dx = |x_rand - x_i|
                if (x_rand >= x_i)
                    dx = x_rand - x_i;
                else
                    dx = x_i - x_rand;
                // dy = |y_rand - y_i|
                if (y_rand >= y_i)
                    dy = y_rand - y_i;
                else
                    dy = y_i - y_rand;

                // squared euclidean distance
                distance = dx*dx + dy*dy;
                // keep smallest distance across all points compared
                if (distance < best_dist) begin
                    best_dist  = distance;
                    best_x     = x_i;
                    best_y     = y_i;
                    best_index = i[OUTERMOST_ITER_BITS-1:0];  // cast integer → index width
                end
            end
        end
    end

    // TODO: NEED TO COMPUTE NEW POINT LOCATION AND SEND NEW POINT TO SYSTOLIC ARRAY
    
    always @( posedge clk ) begin
        if (reset) begin
            best_dist  <= {COORDINATE_WIDTH*2+1{1'b1}}; // max distance to start comparison
            best_x     <= {COORDINATE_WIDTH{1'b0}};
            best_y     <= {COORDINATE_WIDTH{1'b0}};
            best_index <= {OUTERMOST_ITER_BITS{1'b0}};
            nearest_neighbor_x <= {COORDINATE_WIDTH{1'b0}};
            nearest_neighbor_y <= {COORDINATE_WIDTH{1'b0}};
            nearest_neighbor_index <= {OUTERMOST_ITER_BITS{1'b0}};
            nearest_neighbor_found <= 1'b0;
        end 
        else begin
            nearest_neighbor_found <= 1'b0;
            if (search_neighbor==1'b1) begin     
                nearest_neighbor_found <= 1'b1;      
                nearest_neighbor_x <= best_x;
                nearest_neighbor_y <= best_y;
                nearest_neighbor_index <= best_index;
            end
        end
    end
////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////
// NEIGHBOR SEARCH IN WINDOW FRAME

    // adjustable window radius
    localparam [COORDINATE_WIDTH-1:0] WINDOW_RADIUS = {{(COORDINATE_WIDTH-3){1'b0}},3'b101}; // 5

    // instantiate neighbor search module
    window_frame_search #(
        .GRID_W   (N),
        .GRID_H   (N),
        .X_BITS   (COORDINATE_WIDTH),
        .Y_BITS   (COORDINATE_WIDTH),
        .N_SQUARED (N_SQUARED)
    ) find_neighbor (
        .clk             (clk),
        .rst             (reset),

        // control
        .window_search_start    (window_search_start),
        .node_x          (x_rand),
        .node_y          (y_rand),
        .window_radius   (WINDOW_RADIUS),
        .occupancy_status(occupancy_status),
        .window_search_busy (window_search_busy),

        // detected neighbor node output (queue-style stream)
        // .nb_ready        (nb_ready),
        .nb_found        (nb_found), // what happens if we dont find a nearest neighbor for a point? we would have to discard that point and stay in generate state right?
        .nb_x            (nb_x),
        .nb_y            (nb_y)
    );

////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////
// MINIMUM COST TRACKING

// Han/Kamaula note that i didn't decide how many bits we should use to store the costs, just pick something reasonable i guess?
// Han/Kamaula: note that rd_cost is supposed to be the cost currently being read of the data structure storing the costs and the indices for what we should be reading should be the output x and y coordinates from the systolic array (they should be propagated through entire systolic array)
// also the cost data structure should have a read enable signal that's the "valid_output" signal from the systolic array (cost should only be read if the point didn't hit anything)
// Han/Kamaula: i think the only thing i didn't put a note about yet is that when "add_edge_state" is high from the controller, we should make sure
// to take the values in xmin, ymin, and cmin and put xmin and ymin into the vertices grid as the parents of xrand and yrand. we should also make sure to mark the vertice grid at 
// xrand and yrand as 1 and we should make sure that the cost data structure at xrand yrand is updated to cmin. i think that should be good.

// TODO: rd_cost is supposed to be the cost currently being read from cost memory at neighbor location
// TODO: cost memory read indices should be the output x and y coordinates from systolic array (systolic_val_x2_q, systolic_val_y2_q)
// TODO: cost memory should have read enable signal = systolic_valid_pair (cost only read if point didn't collide)
// TODO: when add_edge_state is high, update cost memory at idx(x_rand, y_rand) with c_min
// TODO: when add_edge_state is high, update parent grid at idx(x_rand, y_rand) with {x_min, y_min}
// TODO: when add_edge_state is high, verify occupancy_status[idx(x_rand, y_rand)] is already set to 1 (should happen during random point generation)

// update minimum point connection that gives random point a minimum cost - store this minimum point as parent of random point
always @( posedge clk) begin
    if ( reset ) begin
        x_min <= {COORDINATE_WIDTH{1'b0}};
        y_min <= {COORDINATE_WIDTH{1'b0}};
        c_min <= {COST_WIDTH{1'b1}};  // Initialize to max value for minimum comparison
    end else begin
        if ( update_min_point ) begin //stores valid nearest neighbor point with minimal cost calculated for connection to random point
            x_min <= systolic_val_x2_q;  // used registered value since it takes an extra cycle after we find a valid pair for us to actually update these values
            y_min <= systolic_val_y2_q;  
            c_min <= total_cost; // we need to store total cost (edge + existing)
        end
    end 
end

endmodule
