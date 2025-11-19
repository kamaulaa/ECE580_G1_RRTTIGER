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
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////


module datapath
(
    input clk,
    input reset,
    
    // Dpath -> control
    output path_found,
    output point_hit,
    output done_draining,
    output fifo_full,
    output fifo_empty,
    output parent_equals_current,
    
    // Control -> dpath
    input init_state,
    input rd_fifo,
    input put_item_in_fifo,
    input add_edge_state,
    input sample_point_state,
    input outer_loop_check_state
);

reg [9:0] x_rand;
reg [9:0] y_rand;
reg [9:0] x_min;
reg [9:0] y_min;

reg [9:0] top_bound;
reg [9:0] bottom_bound;
reg [9:0] top_bound;
reg [9:0] left_bound;
reg [9:0] right_bound;

reg [9:0] current_cost;

assign path_found = (y_rand < top_bound) && (y_rand > bottom_bound) && (x_rand < right_bound) && (x_rand > left_bound);

always @( posedge clk ) begin
    if ( reset ) begin
        // maybe should do something?
    end else begin
        if ( sample_point_state==1'b1 ) begin
            x_rand <= output_from_rand_x_generator;
            y_rand <= output_from_rand_y_generator;
        end else begin
            x_rand <= x_rand;
            y_rand <= y_rand;
        end
    end
 end
 
 wire current_cost = ??;
 wire update_min_point = c_min < (current_cost + rd_cost) ? 1'b1 : 1'b0;
 
 always @( posedge clk) begin
    if ( reset ) begin
        // maybe do something?
    end else begin
        if ( update_min_point == 1'b1 ) begin
            x_min <= // something;
            y_min <= // something;
            c_min <= current_cost;
        end
    end 
 end
 

endmodule
