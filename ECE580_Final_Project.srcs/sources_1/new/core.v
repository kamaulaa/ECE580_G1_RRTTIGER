`timescale 1ns / 1ps

// ECE580 Final Project
`include "datapath.v"
`include "core_ctrl.v"

module core
#(
    // ADJUSTABLE GRID PARAMETERS
    parameter N              = 1024,
    parameter N_SQUARED      = N * N,
    parameter OUTERMOST_ITER_MAX = 1024, // adjustable limit 
    parameter OUTERMOST_ITER_BITS = 10, // log2(OUTERMOST_ITER_MAX)
    parameter X_BITS         = 10,    // log2(GRID_WIDTH)
    parameter Y_BITS         = 10,    // log2(GRID_HEIGHT)
    parameter ADDR_BITS      = 20     // log2(GRID_WIDTH * GRID_HEIGHT) for flattened addr
)
(
    input  clk,
    input  reset,

    // expose some status signals at the top level (optional / for debug)
    output path_found,
    output point_hit,
    output done_draining,
    output parent_equals_current,
    output new_random_point_valid,
    output neighbor_search_busy
);

    // ctrl <-> dpath signals
    wire init_state;
    wire add_edge_state;
    wire outer_loop_check_state;
    wire generate_req;
    wire search_start;

    // dpath
    datapath #(
        .N         (N),
        .N_SQUARED (N_SQUARED),
        .X_BITS    (X_BITS),
        .Y_BITS    (Y_BITS),
        .ADDR_BITS (ADDR_BITS),
        .OUTERMOST_ITER_MAX (OUTERMOST_ITER_MAX),
        .OUTERMOST_ITER_BITS (OUTERMOST_ITER_BITS)
    ) datapath_setup (
        .clk                 (clk),
        .reset               (reset),

        // dpath -> control
        .path_found          (path_found),
        .point_hit           (point_hit),
        .done_draining       (done_draining),
        .parent_equals_current(parent_equals_current),
        .new_random_point_valid(new_random_point_valid),
        .neighbor_search_busy(neighbor_search_busy),

        // control -> dpath
        .init_state          (init_state),
        .add_edge_state      (add_edge_state),
        .outer_loop_check_state(outer_loop_check_state),
        .generate_req        (generate_req),
        .search_start        (search_start)
    );

    // ctrl
    core_ctrl #(
        .N                  (N),
        .N_SQUARED          (N_SQUARED),
        .OUTERMOST_ITER_MAX (OUTERMOST_ITER_MAX),
        .X_BITS             (X_BITS),
        .Y_BITS             (Y_BITS),
        .ADDR_BITS          (ADDR_BITS)
    ) core_ctrl_setup (
        .clk                 (clk),
        .reset               (reset),

        // inputs from datapath
        .path_found          (path_found),
        .point_hit           (point_hit),
        .done_draining       (done_draining),
        .parent_equals_current(parent_equals_current),
        .new_random_point_valid(new_random_point_valid),
        .neighbor_search_busy(neighbor_search_busy),

        // outputs to datapath
        .init_state          (init_state),
        .add_edge_state      (add_edge_state),
        .outer_loop_check_state(outer_loop_check_state),
        .generate_req        (generate_req),
        .search_start        (search_start)
    );

endmodule
