`timescale 1ns/1ps
////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Kamaula
// File: oc_array
// processes outputs from all oc_arrays to send valid paths to the datapath for processing (adding to FIFO)
////////////////////////////////////////////////////////////////////////////////////////////////////////////

module oc_array #(
    parameter NUM_PE = 5, // need a way to change this parameter based on how many randomly generated obstacles we have 
    // number of PEs is the number of obstacles
    parameter PARENT_BITS = 10,
    parameter COORDINATE_WIDTH = 7
)(
    input wire clk,
    input wire rst,

    // obstacle data (different obs for each PE)
    // Packed as concatenated vectors: [PE0_data, PE1_data, ..., PE9_data]
    input wire [NUM_PE*COORDINATE_WIDTH-1:0] obs_left,
    input wire [NUM_PE*COORDINATE_WIDTH-1:0] obs_right,
    input wire [NUM_PE*COORDINATE_WIDTH-1:0] obs_top,
    input wire [NUM_PE*COORDINATE_WIDTH-1:0] obs_bottom,

    // current random point
    input wire [COORDINATE_WIDTH-1:0] r_x1,
    input wire [COORDINATE_WIDTH-1:0] r_y1,

    // current nearest neighbor point
    input wire [COORDINATE_WIDTH-1:0] n_x2,
    input wire [COORDINATE_WIDTH-1:0] n_y2,
    input wire [PARENT_BITS-1:0] n_index,

    // valid input signal - high when we have both valid random point and neighbor found
    input wire valid_in,
   
    output valid_out,  // does the systolic array have ANY valid data?
    output valid_pair, // last pair passes all obstacles
    
    // what is the valid output pair (from last PE) --- only store to this if the last PE is VALID
    output [COORDINATE_WIDTH-1:0]   val_x1,  //random point
    output [COORDINATE_WIDTH-1:0]   val_y1,  //random point
    output [COORDINATE_WIDTH-1:0]   val_x2, //nearest neighbor
    output [COORDINATE_WIDTH-1:0]   val_y2,  //nearest neighbor
    output [PARENT_BITS-1:0] val_parent_index // nearest neighbor index in occupied points array
);

// Intermediate wires to chain PEs together (NUM_PE+1 to include input and final output)
wire valid_chain [0:NUM_PE];  // valid signal between PEs (1 = valid/no collision, 0 = invalid/collision)
wire [COORDINATE_WIDTH-1:0] x1_chain [0:NUM_PE];
wire [COORDINATE_WIDTH-1:0] y1_chain [0:NUM_PE];
wire [COORDINATE_WIDTH-1:0] x2_chain [0:NUM_PE];
wire [COORDINATE_WIDTH-1:0] y2_chain [0:NUM_PE];
wire [PARENT_BITS-1:0] parent_index_chain [0:NUM_PE];

// First PE gets valid_in signal (high when we have both valid random point and neighbor found)
assign valid_chain[0] = valid_in;

// First PE gets input coordinates
assign x1_chain[0] = r_x1;
assign y1_chain[0] = r_y1;
assign x2_chain[0] = n_x2;
assign y2_chain[0] = n_y2;
assign parent_index_chain[0] = n_index;

genvar i;
generate 
    for (i=0; i < NUM_PE; i=i+1) begin: PEs
        oc_pe #(.COORDINATE_WIDTH(COORDINATE_WIDTH)) pe (
            .clk(clk),
            .rst(rst),
            .valid_in(valid_chain[i]),              // From previous PE (or 1 for first)
            .obs_left_in(obs_left[i*COORDINATE_WIDTH +: COORDINATE_WIDTH]),
            .obs_right_in(obs_right[i*COORDINATE_WIDTH +: COORDINATE_WIDTH]),
            .obs_top_in(obs_top[i*COORDINATE_WIDTH +: COORDINATE_WIDTH]),
            .obs_bottom_in(obs_bottom[i*COORDINATE_WIDTH +: COORDINATE_WIDTH]),
            .x1_in(x1_chain[i]),                    // From previous PE
            .y1_in(y1_chain[i]),
            .x2_in(x2_chain[i]),
            .y2_in(y2_chain[i]),
            .parent_index_in(parent_index_chain[i]),
            .valid_out(valid_chain[i+1]),           // To next PE
            .x1_out(x1_chain[i+1]),                 // To next PE
            .y1_out(y1_chain[i+1]),
            .x2_out(x2_chain[i+1]),
            .y2_out(y2_chain[i+1]),
            .parent_index_out(parent_index_chain[i+1])
        );
    end
endgenerate

// output from last PE
assign valid_pair = valid_chain[NUM_PE];  // Valid = 1 means no collision across ALL obstacles

// Capture valid output pair from last PE immediately 
assign val_x1 = valid_pair ? x1_chain[NUM_PE] : {COORDINATE_WIDTH{1'b0}};
assign val_y1 = valid_pair ? y1_chain[NUM_PE] : {COORDINATE_WIDTH{1'b0}};
assign val_x2 = valid_pair ? x2_chain[NUM_PE] : {COORDINATE_WIDTH{1'b0}};
assign val_y2 = valid_pair ? y2_chain[NUM_PE] : {COORDINATE_WIDTH{1'b0}};
assign val_parent_index = valid_pair ? parent_index_chain[NUM_PE] : {COORDINATE_WIDTH{1'b0}};

// Check if ANY PE in the systolic array has valid data using direct OR reduction
assign valid_out = |{valid_chain[5], valid_chain[4], valid_chain[3], 
                      valid_chain[2], valid_chain[1], valid_chain[0]};

endmodule 