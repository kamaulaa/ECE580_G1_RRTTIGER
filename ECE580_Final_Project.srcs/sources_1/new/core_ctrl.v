`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////

// Create Date: 11/15/2025 10:31:48 AM
// Design Name: 
// Module Name: core_ctrl
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

module core_ctrl
(
    input clk,
    input reset,
    
    // Inputs from the datapath
    input outermost_loop_check, // outermost_loop_check = !end_found && num_iter <= OUTERMOST_ITER_MAX
    
    // Need output control signals to the datapath
    output example_output,
    output vertices_reset,
    output edges_reset
);

// Define states
localparam INIT = 3'b000;
localparam OUTERMOST_LOOP_CHECK = 3'b001;
localparam GENERATE_RANDOM_POINT = 3'b010;
localparam INNER_LOOP_CHECK = 3'b011;
localparam CHECK_DIST_BTW_RAND_EACH_POINT = 3'b100;

reg [2:0] state;
wire [2:0] next_state;

// Define control variables?

always @ ( posedge clk ) begin
    if ( reset ) begin
        state <= INIT;
    end
    else begin
        state <= next_state;
    end
end


always @ (*) begin
    case (state)
        INIT : begin
            // Decide next state
            next_state <= OUTERMOST_LOOP_CHECK;
            
            // Set any relevant output signals to the datapath
            vertices_reset <= 1'b1; // reset the regs holding the points
            edges_reset <= 1'b1; // reset the regs holding the edges
            
        end
        OUTERMOST_LOOP_CHECK: begin
            if ( outermost_loop_check == 1'b1 ) begin
                
            end
            else begin
                
            end
        end
        GENERATE_RANDOM_POINT: begin
        
        end
        INNER_LOOP_CHECK: begin
        
        end
        CHECK_DIST_BTW_RAND_EACH_POINT: begin
        
        end

    endcase

end

assign next_state = state == INIT ? INSERT_NEXT_STATE_PARAM_HERE :
                    state == OTHER_STATE ? INSERT_STATE_HERE :
                    DEFAULT_STATE;
                    
assign example_output = state == INIT ? certain_output :
                        other_output;       
                   

