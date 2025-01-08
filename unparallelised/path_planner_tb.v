`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 11.11.2024 23:55:27
// Design Name: 
// Module Name: path_planner_tb
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


`timescale 1ns / 1ps

module path_planner_tb;

  // Inputs
  reg reset;
  reg clk;
  reg start;
  reg [4:0] s_node;
  reg [4:0] e_node;
  wire [31:0] clock_cycles;
  wire [31:0] cc1;
  // Outputs
  wire done;
  wire [10*5-1:0] final_path;

  // Instantiate the Unit Under Test (UUT)
  path_planner #(
    .node_count(19),
    .max_edges(4)
  ) uut (
    .clk(clk),
    .start(start),
    .reset(reset),
    .s_node(s_node),
    .e_node(e_node),
    .done(done),
    .final_path(final_path),
    .clock_cycles(clock_cycles),
    .cc1(cc1)
  );
  initial begin
  clk = 0;
  forever #5 clk = ~clk; 
  end
  
  initial begin
    // Initialize Inputs
    
    start = 0;
    s_node = 0;
    e_node = 0;

    // Wait 100 ns for global reset to finish
    #15;
       
    // Add stimulus here
    start = 1;
    reset = 1;
    #10
    reset =0;
    s_node = 0;
    e_node = 11;
   
    #10;
    start = 0;
    #200;

    // Wait for completion
    //while (!done) begin
     // #10;
     // clk = ~clk;
    //end

    wait(done);
    $display("Clock cycles: %d", clock_cycles);
    $display("Clock cycles1: %d", cc1);
    $display("Final path: %b", final_path);

    $finish;
  end
endmodule