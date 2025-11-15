`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 11/12/2025 09:46:06 PM
// Design Name: 
// Module Name: simple_adder
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


// A simple 2-bit adder module
module simple_adder (
    input  wire [1:0] a,
    input  wire [1:0] b,
    output wire [2:0] sum);
    
    assign sum = a + b;
endmodule

