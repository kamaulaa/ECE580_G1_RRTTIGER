//----------------------------------------------------------------------
// Han
// rrt_grid:
// grid based environment storage module
// read access to check if a cell is occupied, write access to add nodes
//----------------------------------------------------------------------

module rrt_grid 
#(
    // adjustable grid parameters
    parameter GRID_W    = 64,
    parameter GRID_H    = 64,
    parameter X_BITS    = 6,   // log2(GRID_W)
    parameter Y_BITS    = 6,   // log2(GRID_H)
    parameter ADDR_BITS = 12   // log2(GRID_W*GRID_H) for flattened addr
)
(
    input                       clk,
    input                       rst,

    // write port
    input                       wr_en,
    input [X_BITS-1:0]          wr_x,
    input [Y_BITS-1:0]          wr_y,
    input                       wr_occupied,
    input [X_BITS-1:0]          wr_parent_x,
    input [Y_BITS-1:0]          wr_parent_y,

    // read port
    input                       rd_en,
    input [X_BITS-1:0]          rd_x,
    input [Y_BITS-1:0]          rd_y,
    output reg                  rd_occupied,
    output reg [X_BITS-1:0]     rd_parent_x,
    output reg [Y_BITS-1:0]     rd_parent_y
);

    // flattened memory representation: depth = GRID_W * GRID_H
    localparam DEPTH = GRID_W * GRID_H;

    // instead of having current node + parent node in same memory, 
    // separated them into multiple regs that are updated simultaneously
    reg                      mem_occupied [0:DEPTH-1];
    reg [X_BITS-1:0]         mem_parent_x [0:DEPTH-1];
    reg [Y_BITS-1:0]         mem_parent_y [0:DEPTH-1];

    reg [ADDR_BITS-1:0]      wr_addr;
    reg [ADDR_BITS-1:0]      rd_addr;

    // compute flattened address : y * GRID_W + x
    function [ADDR_BITS-1:0] idx;
        input [X_BITS-1:0] x_coord;
        input [Y_BITS-1:0] y_coord;
        begin
            idx = y_coord * GRID_W + x_coord;
        end
    endfunction

    integer i;

    always @(posedge clk) begin
        if (rst) begin
            // expensive in hw?
            for (i = 0; i < DEPTH; i = i + 1) begin
                mem_occupied[i] <= 1'b0;
                mem_parent_x[i] <= {X_BITS{1'b0}};
                mem_parent_y[i] <= {Y_BITS{1'b0}};
            end
        end 
        else begin
            if (wr_en) begin
                wr_addr = idx(wr_x, wr_y);
                mem_occupied[wr_addr] <= wr_occupied;
                mem_parent_x[wr_addr] <= wr_parent_x;
                mem_parent_y[wr_addr] <= wr_parent_y;
            end

            if (rd_en) begin
                rd_addr        = idx(rd_x, rd_y);
                rd_occupied    <= mem_occupied[rd_addr];
                rd_parent_x    <= mem_parent_x[rd_addr];
                rd_parent_y    <= mem_parent_y[rd_addr];
            end
        end
    end

endmodule
