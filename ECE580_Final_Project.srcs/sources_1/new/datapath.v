`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////

// Create Date: 11/18/2025 09:05:34 PM
// Design Name: 
// Module Name: datapath
// Project Name: 

//////////////////////////////////////////////////////////////////////////////////


module datapath #(
    parameter COORDINATE_WIDTH = 10,
    parameter NUM_PE = 5,
    parameter NUM_PE_WIDTH = 3, // log2(NUM_PE)
    parameter N = 1024,
    parameter N_SQUARED = N * N,  // 1024 * 1024 = 1,048,576
    parameter N_BITS = 10, // log2(N)
    parameter OUTERMOST_ITER_MAX = 1024, // number of points that can be generated & stored until failure
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
    output new_point_q_collided,
    output done_draining,
    output parent_equals_current,
    output random_point_already_exists, // valid random point
    output done_with_search_nearest_neighbor,
    output done_evaluating_random_point,
    output done_detecting_new_point_q_collision,
    output steered_point_in_obstacle,
    output done_checking_steered_point,
    
    // Control -> dpath
    input init_state,
    input add_edge_state,
    input outer_loop_check_state,
    input generate_req, // to random point generator module
    input search_neighbor, // signal to search neighbor from random generated point
    input entering_search_nearest_neighbor,
    input add_new_point_q,
    input eval_random_point,
    input generate_random_point,
    input entering_check_steered_point,
    input entering_check_new_point_q_collision,
    input check_points_in_square_radius,
    input drain_arr,
    
    // lots of debugging wires
    output [9:0] xrand_wire,
    output [9:0] yrand_wire,
    output [9:0] occupied_array_currentidx,
    output current_array_entry_same_asrandom,
    output [9:0] occupied_points_array_occupied_array_current_idx_X_MSB_X_LSB,
    output [9:0] occupied_points_array_occupied_array_current_idx_Y_MSB_Y_LSB,
    output x_equal,
    output y_equal,
    
//    output done_detecting_new_point_qcollision,
    output new_point_qcollided,
    output [4:0] total_draincycles,
    output [4:0] detecting_new_point_q_collision_cyclecount,
    
    output done_checking_steeredpoint,
    output [NUM_PE_WIDTH:0] steered_point_check_cyclecount
       
);

assign steered_point_check_cyclecount = steered_point_check_cycle_count;

//assign done_detecting_new_point_qcollision = done_detecting_new_point_q_collision;
assign new_point_qcollided = new_point_q_collided;
assign total_draincycles = total_drain_cycles;
assign detecting_new_point_q_collision_cyclecount = detecting_new_point_q_collision_cycle_count;

assign occupied_array_currentidx = occupied_array_current_idx;
assign current_array_entry_same_asrandom = current_array_entry_same_as_random;
assign xrand_wire = x_rand_wire;
assign yrand_wire = y_rand_wire;
assign x_equal = occupied_points_array[occupied_array_current_idx][X_MSB:X_LSB] == x_rand_wire;
assign y_equal = occupied_points_array[occupied_array_current_idx][Y_MSB:Y_LSB] == y_rand_wire;
assign occupied_points_array_occupied_array_current_idx_X_MSB_X_LSB = occupied_points_array[occupied_array_current_idx][X_MSB:X_LSB];
assign occupied_points_array_occupied_array_current_idx_Y_MSB_Y_LSB = occupied_points_array[occupied_array_current_idx][Y_MSB:Y_LSB];

////////////////////////////////////////////////////////////////////////
// SIGNAL DECLARATIONS

// Random point registers
reg [COORDINATE_WIDTH-1:0] x_rand; // register that holds the output of the random number generator 
reg [COORDINATE_WIDTH-1:0] y_rand; 

// Minimum cost point registers
reg [COORDINATE_WIDTH-1:0] x_min; // coordinates of nearest neighbor with min cost for this iteration of nearest neighbor search
reg [COORDINATE_WIDTH-1:0] y_min;
reg [COST_WIDTH-1:0] c_min; // minimum cost found so far
reg [OUTERMOST_ITER_BITS-1:0] parent_index_min; // index of the best parent (minimum cost neighbor)

// Neighbor search signals
reg [COORDINATE_WIDTH-1:0] nb_x; // coords of that nearest neighbor
reg [COORDINATE_WIDTH-1:0] nb_y;
reg [OUTERMOST_ITER_BITS-1:0] nb_index;

// Systolic array control input signal - valid when we're feeding neighbors into the array
reg valid_in;

// Cost calculation signals
// Read the neighbor's accumulated cost from occupied_points_array using the pipelined parent index
wire [COST_WIDTH-1:0] rd_cost = occupied_points_array[systolic_val_parent_index_q][COST_MSB:COST_LSB]; // nearest neighbor's current cost (distance from start)
wire [COST_WIDTH-1:0] calculated_cost; // new connection cost from quantization block
wire [COST_WIDTH-1:0] total_cost = calculated_cost + rd_cost;
// Gate update_min_point to only fire during CHECK_NEW_POINT_Q_COLLISION phase
wire update_min_point = (total_cost < c_min) && systolic_valid_pair_q && 
                        detecting_new_point_q_collision_cycle_count_incremented_on_prev_cycle;

// Systolic array signals
wire systolic_valid_out;
wire systolic_valid_pair;
wire [COORDINATE_WIDTH-1:0] systolic_val_x1, systolic_val_y1; // non-collided point pair (random is 1 and nearest is 2)
wire [COORDINATE_WIDTH-1:0] systolic_val_x2, systolic_val_y2;
wire [OUTERMOST_ITER_BITS-1:0] systolic_val_parent_index; // parent index from systolic array

// Pipeline registers to delay systolic outputs by 1 cycle to match quantization block latency
reg systolic_valid_pair_q;
reg [COORDINATE_WIDTH-1:0] systolic_val_x1_q, systolic_val_y1_q;
reg [COORDINATE_WIDTH-1:0] systolic_val_x2_q, systolic_val_y2_q;
reg [OUTERMOST_ITER_BITS-1:0] systolic_val_parent_index_q;

////////////////////////////////////////////////////////////////////////
// GOAL CHECK LOGIC

// Goal check: check if current point is within goal bounds AND doesn't collide with obstacles
// Use delayed systolic outputs to match the timing of calculated_cost from quantization block

wire goal_reached = (systolic_val_x1_q <= goal_right_bound) && (systolic_val_x1_q >= goal_left_bound) && (systolic_val_y1_q >= goal_top_bound) && (systolic_val_y1_q <= goal_bottom_bound);

// Latch path_found so it stays high once goal is reached (since systolic_valid_pair_q is transient)
reg path_found_q;
always @(posedge clk) begin
    if (reset)
        path_found_q <= 1'b0;
    else if (goal_reached && systolic_valid_pair_q)
        path_found_q <= 1'b1;
end
assign path_found = path_found_q;

////////////////////////////////////////////////////////////////////////
// CONTROL SIGNALS

assign done_draining = ~(systolic_valid_out ||  systolic_valid_pair_q); // not used anymore

////////////////////////////////////////////////////////////////////////
// POINT ARRAY & GRID 

// array to store points in grid and parent index
localparam ARRAY_WIDTH = OUTERMOST_ITER_BITS + COORDINATE_WIDTH*2 + COST_WIDTH; // parent_index + x_coord + y_coord + cost
localparam PARENT_IDX_MSB = ARRAY_WIDTH - 1;
localparam PARENT_IDX_LSB = ARRAY_WIDTH - OUTERMOST_ITER_BITS;
localparam Y_MSB = PARENT_IDX_LSB - 1;
localparam Y_LSB = PARENT_IDX_LSB - COORDINATE_WIDTH;
localparam X_MSB = Y_LSB -1;
localparam X_LSB = Y_LSB - COORDINATE_WIDTH;
localparam COST_MSB = X_LSB -1;
localparam COST_LSB = 0;

reg [ARRAY_WIDTH-1:0] occupied_points_array [0:OUTERMOST_ITER_MAX-1]; // array to store points in first-come order
reg [OUTERMOST_ITER_BITS-1:0] occupied_array_idx; // counts number of occupied points stored for array indexing

// compute flattened address : y * N + x
function [N_SQUARED-1:0] idx;
    input [COORDINATE_WIDTH-1:0] x_coord;
    input [COORDINATE_WIDTH-1:0] y_coord;
    begin
        idx = (y_coord << N_BITS) + x_coord;
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

    reg [OUTERMOST_ITER_BITS-1:0] occupied_array_current_idx;
    wire current_array_entry_same_as_random = (occupied_points_array[occupied_array_current_idx][X_MSB:X_LSB] == x_rand_wire) && (occupied_points_array[occupied_array_current_idx][Y_MSB:Y_LSB] == y_rand_wire); 
    assign done_evaluating_random_point = (eval_random_point == 1'b1) && (current_array_entry_same_as_random || (occupied_array_current_idx == occupied_array_idx));
    assign random_point_already_exists = current_array_entry_same_as_random;

    // check if generated random point already exists
    always @( posedge clk ) begin
        if ( reset == 1'b1) begin
            x_rand <= {COORDINATE_WIDTH{1'b0}};
            y_rand <= {COORDINATE_WIDTH{1'b0}};
        end else begin
            if ( generate_req == 1'b1 ) begin
                occupied_array_current_idx <= 0;
            end
            if ( eval_random_point == 1'b1 ) begin
                occupied_array_current_idx <= (done_evaluating_random_point == 1'b1) ? 1'b0 : occupied_array_current_idx + 1;
            end
            if ( done_evaluating_random_point == 1'b1 && current_array_entry_same_as_random == 1'b0) begin
                x_rand <= x_rand_wire;
                y_rand <= y_rand_wire;
            end
        end
    end

////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////
// NEARBOR NEIGHBOR SEARCH
// search across all existing points for nearest point to randomly generated point 

// need to use 5 obstacles, 1/8 for q

    localparam DIST_WIDTH = COORDINATE_WIDTH*2+2;
    reg [OUTERMOST_ITER_BITS-1:0] new_point_parent_index;
    reg [DIST_WIDTH-1:0] new_point_parent_dist;

    localparam[2:0] tau_denom_bits = 2'b11; // 2^3 = 8 for bit shifting division
    reg [COORDINATE_WIDTH-1:0] new_point_x, new_point_y;
    reg [COORDINATE_WIDTH-1:0] new_point_parent_x, new_point_parent_y;
        
    // Need to tell the controller if we can stop searching 
    assign done_with_search_nearest_neighbor = (occupied_array_current_idx == occupied_array_idx);

    // Do the distance as combinational logic
    wire [COORDINATE_WIDTH-1:0] dx = x_rand >= occupied_points_array[occupied_array_current_idx][X_MSB:X_LSB] ? x_rand - occupied_points_array[occupied_array_current_idx][X_MSB:X_LSB] : occupied_points_array[occupied_array_current_idx][X_MSB:X_LSB] - x_rand;
    wire [COORDINATE_WIDTH-1:0] dy = y_rand >= occupied_points_array[occupied_array_current_idx][Y_MSB:Y_LSB] ? y_rand - occupied_points_array[occupied_array_current_idx][Y_MSB:Y_LSB] : occupied_points_array[occupied_array_current_idx][Y_MSB:Y_LSB] - y_rand;
    wire [DIST_WIDTH-1:0] distance = dx*dx + dy*dy;

    // array to store indices of 10 nearest neigbors
    // nearest neighbor with shortest distance will be used to generate new point 
    localparam NEIGHBOR_ARRAY_WIDTH = OUTERMOST_ITER_BITS + DIST_WIDTH; // neighbor index + distance to random 
    localparam IDX_MSB = NEIGHBOR_ARRAY_WIDTH - 1;
    localparam IDX_LSB = DIST_WIDTH;
    localparam DIST_MSB = IDX_LSB - 1;
    localparam DIST_LSB = 0;

    reg [NEIGHBOR_ARRAY_WIDTH-1:0] ten_nearest_neighbors [0:9]; // stores index (occupied_array_current_idx)
    reg [3:0] nearest_neighbor_count; 

    reg [3:0] best_neighbor_ten_idx; // index of top ten array 
    reg [OUTERMOST_ITER_BITS-1:0] best_neighbor_idx; // index of full 1024 entries array
    reg [DIST_WIDTH-1:0] best_neighbor_dist;
    reg [3:0] worst_neighbor_ten_idx;
    reg [DIST_WIDTH-1:0] worst_neighbor_dist;
    
    // find best and worst neighbors among top ten neighbors 
    always @(*) begin
        best_neighbor_ten_idx  = 0;
        best_neighbor_dist = {DIST_WIDTH{1'b1}}; // max
        worst_neighbor_ten_idx = 0;
        worst_neighbor_dist= {DIST_WIDTH{1'b0}}; // min

        if (nearest_neighbor_count != 0) begin
            for (integer i = 0; i < nearest_neighbor_count; i = i + 1) begin
                // find best neighbor among neighbors in ten_nearest_neighbors
                if (ten_nearest_neighbors[i][DIST_MSB:DIST_LSB] < best_neighbor_dist) begin
                    best_neighbor_dist = ten_nearest_neighbors[i][DIST_MSB:DIST_LSB];
                    best_neighbor_ten_idx  = i;
                    best_neighbor_idx = ten_nearest_neighbors[i][IDX_MSB:IDX_LSB];
                end
                // find worst neighbor among neighbors in ten_nearest_neighbors
                if (ten_nearest_neighbors[i][DIST_MSB:DIST_LSB] > worst_neighbor_dist) begin
                    worst_neighbor_dist = ten_nearest_neighbors[i][DIST_MSB:DIST_LSB];
                    worst_neighbor_ten_idx  = i;
                end
            end
            
        end
    end

    wire [COORDINATE_WIDTH-1:0] potential_new_point_x = (3'b111*(new_point_parent_x >> tau_denom_bits)) + (x_rand >> tau_denom_bits);
    wire [COORDINATE_WIDTH-1:0] potential_new_point_y = (3'b111*(new_point_parent_y >> tau_denom_bits)) + (y_rand >> tau_denom_bits);
    // NOTE: OR SHOULD THIS BE THIS?
    // wire [COORDINATE_WIDTH-1:0] potential_new_point_x = (5'b11111*new_point_parent_x + x_rand) >> tau_denom_bits;
    // wire [COORDINATE_WIDTH-1:0] potential_new_point_y = (5'b11111*new_point_parent_y + y_rand) >> tau_denom_bits;

    localparam [COORDINATE_WIDTH-1:0] TWO_CONSTANT = {{(COORDINATE_WIDTH-2){1'b0}}, 2'b10};

    always @( posedge clk ) begin
        if (reset) begin
            new_point_x <= {COORDINATE_WIDTH{1'b0}};
            new_point_y <= {COORDINATE_WIDTH{1'b0}};
            new_point_parent_x <= {COORDINATE_WIDTH{1'b0}};
            new_point_parent_y <= {COORDINATE_WIDTH{1'b0}};
            occupied_array_current_idx <= 0;
            nearest_neighbor_count <= 4'b0;

        end else begin
            // Reset nearest_neighbor_count at the start of each neighbor search
            if (entering_search_nearest_neighbor == 1'b1) begin
                nearest_neighbor_count <= 4'b0;
            end
            // add first ten points into top 10 nearest neighbor array
            else if (nearest_neighbor_count < 4'd10 && search_neighbor == 1'b1) begin
                ten_nearest_neighbors[nearest_neighbor_count] <= {occupied_array_current_idx, distance};
                nearest_neighbor_count <= nearest_neighbor_count + 1'b1; // max at 10
            end
            else if (search_neighbor == 1'b1) begin
                // replace current worst if new distance is smaller
                if (distance < ten_nearest_neighbors[worst_neighbor_ten_idx][DIST_MSB:DIST_LSB]) begin
                    ten_nearest_neighbors[worst_neighbor_ten_idx] <= {occupied_array_current_idx, distance};
                end
            end
            
            if (search_neighbor == 1'b1) begin
                occupied_array_current_idx <= done_with_search_nearest_neighbor ? 1'b0 : occupied_array_current_idx + 1'b1;
            end

            // use best neighbor index to generate new point new point AFTER ALL NODES HAVE BEEN TRAVERSED
            if (done_with_search_nearest_neighbor == 1'b1 && search_neighbor == 1'b1) begin
                new_point_parent_x <= occupied_points_array[best_neighbor_idx][X_MSB:X_LSB];
                new_point_parent_y <= occupied_points_array[best_neighbor_idx][Y_MSB:Y_LSB];
                new_point_parent_index <= best_neighbor_idx;
                new_point_parent_dist <= best_neighbor_dist;
            end

            if ( add_new_point_q == 1'b1 ) begin
                // when adding a new point what needs to be recorded? the coordinates of the new point but also the parents?              
                if ( x_rand > new_point_parent_x && y_rand < new_point_parent_y ) begin // New point in quad 1
                    new_point_x <= (TWO_CONSTANT + new_point_parent_x) > N ? N : potential_new_point_x < (TWO_CONSTANT + new_point_parent_x) ? (TWO_CONSTANT + new_point_parent_x) : potential_new_point_x;
                    new_point_y <= (new_point_parent_y < TWO_CONSTANT) ? 0 : potential_new_point_y > (new_point_parent_y - TWO_CONSTANT) ? (new_point_parent_y - TWO_CONSTANT) : potential_new_point_y; 
                end else if ( x_rand < new_point_parent_x && y_rand < new_point_parent_y ) begin // New point in quad 2
                    new_point_x <= (new_point_parent_x < TWO_CONSTANT) ? 0 : potential_new_point_x > (new_point_parent_x - TWO_CONSTANT) ? (new_point_parent_x - TWO_CONSTANT) : potential_new_point_x;
                    new_point_y <= (new_point_parent_y < TWO_CONSTANT) ? 0 : potential_new_point_y > (new_point_parent_y - TWO_CONSTANT) ? (new_point_parent_y - TWO_CONSTANT) : potential_new_point_y;                       
                end else if ( x_rand < new_point_parent_x && y_rand > new_point_parent_y ) begin // New point in quad 3
                    new_point_x <= (new_point_parent_x < TWO_CONSTANT) ? 0 : potential_new_point_x > (new_point_parent_x - TWO_CONSTANT) ? (new_point_parent_x - TWO_CONSTANT) : potential_new_point_x;
                    new_point_y <= (TWO_CONSTANT + new_point_parent_y) > N ? N : potential_new_point_y < (TWO_CONSTANT + new_point_parent_y) ? (TWO_CONSTANT + new_point_parent_y): potential_new_point_y;          
                end else if ( x_rand > new_point_parent_x && y_rand > new_point_parent_y ) begin // New point in quad 4
                    new_point_x <= (TWO_CONSTANT + new_point_parent_x) > N ? N : potential_new_point_x < (TWO_CONSTANT + new_point_parent_x) ? (TWO_CONSTANT + new_point_parent_x) : potential_new_point_x;
                    new_point_y <= (TWO_CONSTANT + new_point_parent_y) > N ? N : potential_new_point_y < (TWO_CONSTANT + new_point_parent_y) ? (TWO_CONSTANT + new_point_parent_y): potential_new_point_y;         
                end          
            end
        end
    end
    
    
////////////////////////////////////////////////////////////////////////
// SYSTOLIC ARRAY PIPELINE REGISTERS

always @(posedge clk) begin
    if (reset) begin
        systolic_valid_pair_q <= 0;
        systolic_val_x1_q <= 0;
        systolic_val_y1_q <= 0;
        systolic_val_x2_q <= 0;
        systolic_val_y2_q <= 0;
        systolic_val_parent_index_q <= 0;
    end else begin
        // Pipeline stage to align coordinates/index with quantization block output (1 cycle latency)
        systolic_valid_pair_q <= systolic_valid_pair;
        systolic_val_x1_q <= systolic_val_x1;
        systolic_val_y1_q <= systolic_val_y1;
        systolic_val_x2_q <= systolic_val_x2;
        systolic_val_y2_q <= systolic_val_y2;
        systolic_val_parent_index_q <= systolic_val_parent_index;
    end
end

////////////////////////////////////////////////////////////////////////
// OBSTACLE COLLISION DETECTION - SYSTOLIC ARRAY

// obstacle detection systolic array - constantly being fed the newly calculated random points and their nearest neighbors

reg [3:0] nearest_neighbors_checked;

oc_array #(.COORDINATE_WIDTH(COORDINATE_WIDTH), .PARENT_BITS(OUTERMOST_ITER_BITS), .NUM_PE(NUM_PE)) pe_array (
            .clk(clk),
            .rst(reset),
            .obs_left(obs_left), // Shouldn't each PE be loaded with its own obstacle in the init stage? - idk how to do that since they get instantiated inside the actual array
            .obs_right(obs_right),
            .obs_top(obs_top),
            .obs_bottom(obs_bottom),
            .r_x1(new_point_x), // new point x and new point y are the coords of the steered point
            .r_y1(new_point_y),
            .n_x2(nb_x), // nearest neighbor inputs from top 10 
            .n_y2(nb_y),
            .n_index(nb_index), 
            .valid_in(valid_in),
            .valid_out(systolic_valid_out),
            .valid_pair(systolic_valid_pair),
            .val_x1(systolic_val_x1),
            .val_y1(systolic_val_y1), 
            .val_x2(systolic_val_x2),
            .val_y2(systolic_val_y2),
            .val_parent_index(systolic_val_parent_index)
        );


// Steered point collision check - for fast rejections
reg [NUM_PE_WIDTH-1:0] steered_point_check_cycle_count;

// If any PE detects steered point inside obstacle, set flag
wire steered_point_collided = done_checking_steered_point || reset || entering_check_steered_point ? (!systolic_valid_pair ? 1'b1 : 1'b0 ) : 1'b0;
assign done_checking_steered_point = (steered_point_check_cycle_count == 3'd5);
assign steered_point_in_obstacle = steered_point_collided;

always @(posedge clk) begin
    if (reset) begin
        steered_point_check_cycle_count <= 0;
    end else begin
        if (entering_check_steered_point) begin
            steered_point_check_cycle_count <= 1'b0;
        end else if (steered_point_check_cycle_count >= 0 && steered_point_check_cycle_count < 3'd5) begin
            steered_point_check_cycle_count <= steered_point_check_cycle_count + 1'b1;
        end else if (done_checking_steered_point) begin
            steered_point_check_cycle_count <= 0;
        end   
    end
end

// Feed neighbors into systolic array on consecutive cycles for pipelined processing
always @(posedge clk) begin
    if (reset) begin
        nearest_neighbors_checked <= 4'b0;
        nb_index <= {OUTERMOST_ITER_BITS{1'b0}};
        nb_x <= {COORDINATE_WIDTH{1'b0}};
        nb_y <= {COORDINATE_WIDTH{1'b0}};
        valid_in <= 1'b0;
    end
    else begin
        if (entering_check_steered_point) begin
            // For steered point check: feed steered point with itself (dummy neighbor)
            // We only care if steered point (x1,y1) is inside an obstacle
            nearest_neighbors_checked <= 4'b0;
            nb_index <= {OUTERMOST_ITER_BITS{1'b0}};
            nb_x <= new_point_x;  // Use steered point as both endpoints
            nb_y <= new_point_y;
            valid_in <= 1'b1;
        end
        else if (entering_check_new_point_q_collision) begin
            // Load first neighbor (index 0) to be fed into systolic array on next clock edge
            nearest_neighbors_checked <= 4'b1;  // Counter = 1 means we've queued neighbor 0
            nb_index <= ten_nearest_neighbors[0][IDX_MSB:IDX_LSB];
            nb_x <= occupied_points_array[ten_nearest_neighbors[0][IDX_MSB:IDX_LSB]][X_MSB:X_LSB];
            nb_y <= occupied_points_array[ten_nearest_neighbors[0][IDX_MSB:IDX_LSB]][Y_MSB:Y_LSB];
            valid_in <= (nearest_neighbor_count > 0) ? 1'b1 : 1'b0;
        end
        else if (nearest_neighbors_checked < nearest_neighbor_count) begin
            // Feed one neighbor per cycle into the systolic array
            nb_index <= ten_nearest_neighbors[nearest_neighbors_checked][IDX_MSB:IDX_LSB];
            nb_x <= occupied_points_array[ten_nearest_neighbors[nearest_neighbors_checked][IDX_MSB:IDX_LSB]][X_MSB:X_LSB];
            nb_y <= occupied_points_array[ten_nearest_neighbors[nearest_neighbors_checked][IDX_MSB:IDX_LSB]][Y_MSB:Y_LSB];
            nearest_neighbors_checked <= nearest_neighbors_checked + 1;
            valid_in <= 1'b1;
        end
        else begin
            // Done feeding neighbors
            valid_in <= 1'b0;
        end
    end
end
        
////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////
// CHECK NEW POINT Q COLLISION
// Checks if connections from steered point to all 10 nearest neighbors are collision-free
// Steered point itself is pre-checked in CHECK_STEERED_POINT state for fast rejection

reg [4:0] detecting_new_point_q_collision_cycle_count;  // 5 bits to hold up to 14 (10 neighbors + 5 PEs - 1)
reg detecting_new_point_q_collision_cycle_count_incremented_on_prev_cycle;
reg found_valid_neighbor; // Track if at least one neighbor produced a valid (collision-free) connection

// new_point_q_collided = 1 if ALL neighbors collided (no valid connections found)
assign new_point_q_collided = ~found_valid_neighbor;
// Wait for pipeline to fully drain: nearest_neighbor_count cycles to feed + NUM_PE-1 cycles to drain
wire [4:0] total_drain_cycles = nearest_neighbor_count + NUM_PE - 1; // LAUREN - maybe +1 here but maybe not... should be fine like this
assign done_detecting_new_point_q_collision = detecting_new_point_q_collision_cycle_count == total_drain_cycles;

always @( posedge clk ) begin
    if ( reset ) begin
        detecting_new_point_q_collision_cycle_count <= 0;
        detecting_new_point_q_collision_cycle_count_incremented_on_prev_cycle <= 1'b0;
        found_valid_neighbor <= 1'b0;
    end else begin
        if (entering_check_new_point_q_collision == 1'b1) begin
            // Reset flag when starting collision check for new steered point
            found_valid_neighbor <= 1'b0;
            detecting_new_point_q_collision_cycle_count <= 1'b1; // detecting_new_point_q_collision_cycle_count
            detecting_new_point_q_collision_cycle_count_incremented_on_prev_cycle <= 1'b1;
        end else if (done_detecting_new_point_q_collision == 1'b1) begin
            detecting_new_point_q_collision_cycle_count <= 0;
            detecting_new_point_q_collision_cycle_count_incremented_on_prev_cycle <= 1'b0;
        end else begin    
            // Set flag if we find any valid collision-free connection
            if (systolic_valid_pair == 1'b1) begin
                found_valid_neighbor <= 1'b1;
            end
            if (detecting_new_point_q_collision_cycle_count_incremented_on_prev_cycle == 1'b1) begin
                detecting_new_point_q_collision_cycle_count <= detecting_new_point_q_collision_cycle_count + 1'b1;
            end
        end
    end
end

////////////////////////////////////////////////////////////////////////
// MINIMUM COST TRACKING

// quantization block for cost calculation (has 1-cycle latency, enabled by valid_in)
// Note: Uses non-registered systolic outputs, pipeline delay in datapath aligns coordinates with cost output
quantization_block #(.COORDINATE_WIDTH(COORDINATE_WIDTH), .COST_WIDTH(COST_WIDTH)) quantized_cost (
    .clk(clk),
    .rst(reset),
    .valid_in(systolic_valid_pair),  // use non-registered valid to start calculation ASAP
    .r_x1(systolic_val_x1),      // steered point x (non-registered)
    .r_y1(systolic_val_y1),      // steered point y (non-registered)
    .n_x2(systolic_val_x2),      // nearest neighbor x (non-registered)
    .n_y2(systolic_val_y2),      // nearest neighbor y (non-registered)
    .cost_out(calculated_cost)   // manhattan distance cost output (ready 1 cycle later)
);

// update minimum point connection that gives random point a minimum cost - store this minimum point as parent of random point
always @( posedge clk) begin
    if ( reset ) begin
        x_min <= {COORDINATE_WIDTH{1'b0}};
        y_min <= {COORDINATE_WIDTH{1'b0}};
        c_min <= {COST_WIDTH{1'b1}};  // Initialize to max value for minimum comparison
        parent_index_min <= {OUTERMOST_ITER_BITS{1'b0}};
        occupied_array_idx <= {OUTERMOST_ITER_BITS{1'b0}};
    end else if ( init_state ) begin
        // Initialize start point as first entry in tree: parent=self (0), coordinates, cost=0
        occupied_points_array[0] <= {{OUTERMOST_ITER_BITS{1'b0}}, start_y, start_x, {COST_WIDTH{1'b0}}};
//        occupied_array_idx <= {{(OUTERMOST_ITER_BITS-1){1'b0}}, 1'b1};  // Next point will be added at index 1
    end else begin
        if (entering_check_new_point_q_collision == 1'b1) begin
            // Reset min cost when starting collision checks for new steered point
            c_min <= {COST_WIDTH{1'b1}};
        end else if ( update_min_point &&  systolic_valid_pair_q ) begin //stores valid nearest neighbor point with minimal cost calculated for connection to random point
            x_min <= systolic_val_x2_q;  // use pipelined values - they align with calculated_cost timing
            y_min <= systolic_val_y2_q;  
            c_min <= total_cost; // we need to store total cost (edge + existing) 
            parent_index_min <= systolic_val_parent_index_q; // store the index of this best parent
        end else if ( add_edge_state == 1'b1 ) begin
            // Add steered point to occupied points array with best parent and cost
            occupied_points_array[occupied_array_idx+1'b1] <= {parent_index_min, new_point_y, new_point_x, c_min};
            occupied_array_idx <= occupied_array_idx + 1'b1; // LAUREN
        end
    end 
end
endmodule
