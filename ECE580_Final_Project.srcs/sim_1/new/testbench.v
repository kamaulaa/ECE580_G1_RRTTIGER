`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 11/12/2025 09:48:23 PM
// Design Name: 
// Module Name: testbench
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

`timescale 1ns/1ps
module testbench;
reg [1:0] a;
reg [1:0] b;
wire [2:0] sum;

simple_adder uut(.a(a), .b(b), .sum(sum));

initial begin
    a = 2'b00;
    b = 2'b00;
    $monitor("Time: %0t | a: %b, b: %b, sum: %b", $time, a, b, sum);
    #10 a = 2'b01; b = 2'b01;
    #10 a = 2'b10; b = 2'b01;
    #10 a = 2'b11; b = 2'b11;
    #10 $finish;
end endmodule

