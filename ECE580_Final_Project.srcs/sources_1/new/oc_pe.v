`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Kamaula
// File: oc_pe
// processing element implementation for obstacle checking
//
// NOTE: Check 3 uses combinational division which creates long critical path
//       To increase throughput, we should consider pipelining the divisions in Check 3
//       across multiple cycles (adds latency but increases clock frequency).
//////////////////////////////////////////////////////////////////////////////////

module oc_pe #(
    parameter COORDINATE_WIDTH = 10  // 10 bits per coordinate (x or y)
)(
    input wire clk,
    input wire rst,
    
    // previous PE inputs
    input wire valid_in,           // collision detected by previous PE (1 = no (valid) , 0 = yes (not valid))
    
    // obstacle bounds: left, right, top, bottom (10 bits each)
    input wire [COORDINATE_WIDTH-1:0] obs_left_in,
    input wire [COORDINATE_WIDTH-1:0] obs_right_in,
    input wire [COORDINATE_WIDTH-1:0] obs_top_in,
    input wire [COORDINATE_WIDTH-1:0] obs_bottom_in,
    
    // endpoint 1 (x1, y1)
    input wire [COORDINATE_WIDTH-1:0] x1_in,
    input wire [COORDINATE_WIDTH-1:0] y1_in,
    
    // endpoint 2 (x2, y2)
    input wire [COORDINATE_WIDTH-1:0] x2_in,
    input wire [COORDINATE_WIDTH-1:0] y2_in,
    input wire [COORDINATE_WIDTH-1:0] parent_index_in,
    
    // outputs to next PE
    output reg valid_out,          // valid is HIGH (1) if no collision was detected
    
    // pass point  data for next PE 
    output reg [COORDINATE_WIDTH-1:0] x1_out,
    output reg [COORDINATE_WIDTH-1:0] y1_out,
    output reg [COORDINATE_WIDTH-1:0] x2_out,
    output reg [COORDINATE_WIDTH-1:0] y2_out,
    output reg [COORDINATE_WIDTH-1:0] parent_index_out
);
    
    // (NO) COLLISION CHECK 1: both endpoints on the same side, outside of obstacle
    wire both_above, both_below, both_left, both_right;
    wire check1_pass;  // If true, no collision possible
    
    assign both_above  = (y1_in > obs_top_in) && (y2_in > obs_top_in);
    assign both_below  = (y1_in < obs_bottom_in) && (y2_in < obs_bottom_in);
    assign both_left   = (x1_in < obs_left_in) && (x2_in < obs_left_in);
    assign both_right  = (x1_in > obs_right_in) && (x2_in > obs_right_in);
    
    // If any of these is true, segment doesn't collide with obstacle
    assign check1_pass = both_above || both_below || both_left || both_right;
    
    
    // COLLISION CHECK 2: endpoints span across obstacle - DEFINITE COLLISION
    wire x_span, y_span;
    wire check2_collision; // 1 means collided with obstacle
    
    // Both x-coordinates within obstacle bounds means they span vertically across it
    assign x_span = (x1_in >= obs_left_in) && (x1_in <= obs_right_in) &&
                    (x2_in >= obs_left_in) && (x2_in <= obs_right_in);
    
    // Both y-coordinates within obstacle bounds means they span horizontally across it  
    assign y_span = (y1_in >= obs_bottom_in) && (y1_in <= obs_top_in) &&
                    (y2_in >= obs_bottom_in) && (y2_in <= obs_top_in);
    
    assign check2_collision = x_span || y_span;
    
    
    // COLLISION CHECK 3: Corner intersection detection
    // Detects when line segment enters/exits through obstacle corners
    // Example: p1 inside obstacle horizontally, p2 outside to the left
    //          Does the line cross through the left edge within the obstacle's vertical bounds?
    
    //  3A: Determine position of each point relative to obstacle 
    wire p1_x_inside, p1_y_inside;  // Is point 1's x-coordinate and y-coordinate within obstacle bounds?
    wire p2_x_inside, p2_y_inside;  // Is point 2's x-coordinate and y-coordinate within obstacle bounds?
    
    assign p1_x_inside = (x1_in >= obs_left_in) && (x1_in <= obs_right_in);
    assign p1_y_inside = (y1_in >= obs_bottom_in) && (y1_in <= obs_top_in);
    assign p2_x_inside = (x2_in >= obs_left_in) && (x2_in <= obs_right_in);
    assign p2_y_inside = (y2_in >= obs_bottom_in) && (y2_in <= obs_top_in);
    
    // 3B: Calculate line slope components (for intersection calculations) 
    wire signed [2*COORDINATE_WIDTH:0] dx, dy;
    assign dx = $signed(x2_in) - $signed(x1_in);
    assign dy = $signed(y2_in) - $signed(y1_in);
    
    // 3C: Case 1 - Point 1's x is inside obstacle, Point 2's x is outside
    wire check3_left_edge, check3_right_edge;
    wire signed [2*COORDINATE_WIDTH:0] y_at_left_edge, y_at_right_edge;
    
    // Calculate y-coordinate where line crosses left boundary using: y = y1 + slope * (x_boundary - x1)
    assign y_at_left_edge = $signed(y1_in) + (dy / dx) * ($signed(obs_left_in) - $signed(x1_in));
    // Collision if: p1's x inside, p2's x to the left, AND crossing point's y is within obstacle bounds
    assign check3_left_edge = p1_x_inside && !p2_x_inside && (x2_in < obs_left_in) && 
                              (y_at_left_edge >= obs_bottom_in) && (y_at_left_edge <= obs_top_in);
    
    // Calculate y-coordinate where line crosses right boundary
    assign y_at_right_edge = $signed(y1_in) + (dy / dx) * ($signed(obs_right_in) - $signed(x1_in));
    // Collision if: p1's x inside, p2's x to the right, AND crossing point's y is within obstacle bounds
    assign check3_right_edge = p1_x_inside && !p2_x_inside && (x2_in > obs_right_in) &&
                               (y_at_right_edge >= obs_bottom_in) && (y_at_right_edge <= obs_top_in);
    
    // 3D: Case 2 - Point 1's y is inside obstacle, Point 2's y is outside ---
    // Check if line crosses top or bottom boundary within the obstacle's horizontal range
    wire check3_top_edge, check3_bottom_edge;
    wire signed [2*COORDINATE_WIDTH:0] x_at_top_edge, x_at_bottom_edge;
    
    // Calculate x-coordinate where line crosses top boundary using: x = x1 + (1/slope) * (y_boundary - y1)
    assign x_at_top_edge = $signed(x1_in) + (dx / dy) * ($signed(obs_top_in) - $signed(y1_in));
    // Collision if: p1's y inside, p2's y above, AND crossing point's x is within obstacle bounds
    assign check3_top_edge = p1_y_inside && !p2_y_inside && (y2_in > obs_top_in) &&
                             (x_at_top_edge >= obs_left_in) && (x_at_top_edge <= obs_right_in);
    
    // Calculate x-coordinate where line crosses bottom boundary
    assign x_at_bottom_edge = $signed(x1_in) + (dx / dy) * ($signed(obs_bottom_in) - $signed(y1_in));
    // Collision if: p1's y inside, p2's y below, AND crossing point's x is within obstacle bounds
    assign check3_bottom_edge = p1_y_inside && !p2_y_inside && (y2_in < obs_bottom_in) &&
                                (x_at_bottom_edge >= obs_left_in) && (x_at_bottom_edge <= obs_right_in);
    
    // Check 3 detects collision if line crosses any of the 4 edges (left, right, top, bottom)
    wire check3_collision;
    assign check3_collision = check3_left_edge || check3_right_edge || 
                              check3_top_edge || check3_bottom_edge;
    
    
    // COLLISION CHECK 4: Either endpoint is completely inside the obstacle
    // If any point is fully enclosed, that's definitely a collision
    wire p1_inside, p2_inside;
    assign p1_inside = p1_x_inside && p1_y_inside;  // Point 1 inside if BOTH x AND y are inside bounds
    assign p2_inside = p2_x_inside && p2_y_inside;  // Point 2 inside if BOTH x AND y are inside bounds
    

   // Collision occurs if Check 1 fails to reject AND any other check detects collision
    wire local_collision;
    assign local_collision = !check1_pass && (check2_collision || check3_collision || p1_inside || p2_inside);

    // OUTPUT registers 
    // registers endpoint data and collision result on each clock cycle
    // data flows through systolic array with 1 cycle latency per stage
    always @(posedge clk) begin
        if (rst) begin
            valid_out <= 1'b0;
            x1_out <= {COORDINATE_WIDTH{1'b0}};
            y1_out <= {COORDINATE_WIDTH{1'b0}};
            x2_out <= {COORDINATE_WIDTH{1'b0}};
            y2_out <= {COORDINATE_WIDTH{1'b0}};
            parent_index_out <= {COORDINATE_WIDTH{1'b0}};
        end else begin
            // pass through endpoint coordinates to next PE in systolic array
            x1_out <= x1_in;
            y1_out <= y1_in;
            x2_out <= x2_in;
            y2_out <= y2_in;
            parent_index_out <= parent_index_in;
            
            // if collision detected, not valid, propagate 0
            // if previous PE found collision, will be invalid ; otherwise validity is based on if a collision was found with this PE's obstacle
            valid_out <= !valid_in ? 1'b0 : !local_collision;
        end
    end

endmodule
