`timescale 1ns / 1ps

module Dijkstra_tb;
    // Parameters
    parameter node_count = 19;
    parameter max_edges = 4;

    // Inputs
    reg reset;
    reg clk;
    reg start;
    reg [4:0] s_node;
    reg [4:0] e_node;

    // Outputs
    wire done;
    wire [31:0] clock_cycles;
    wire [10*5-1:0] final_path;
    wire [4:0] min_index;
    wire [5:0] visited_count;
    wire [3:0] next_state;

    // Instantiate the Dijkstra module
    Dijkstra #(
        .node_count(node_count),
        .max_edges(max_edges)
    ) uut (
        .reset(reset),
        .clk(clk),
        .start(start),
        .s_node(s_node),
        .e_node(e_node),
        .done(done),
        .clock_cycles(clock_cycles),
        .final_path(final_path),
        .min_index(min_index),
        .visited_count(visited_count),
        .next_state(next_state)
    );

    // Clock generation
    initial begin
        clk = 0;
        forever #5 clk = ~clk; // 10ns clock period
    end

    // Test sequence
    initial begin
        // Initialize signals
        reset = 1;
        start = 0;
        s_node = 5'd0;     // Default start node
        e_node = 5'd11;    // Default end node

        // Perform multiple test cases with different start and end nodes
        #10 reset = 0;
        start = 1;
        #10 start = 0;     // First test

        // Wait for completion of current run
        wait(done);
        #10; // Small delay before starting the next test

        // Test 2
        reset = 1;
        #10 reset = 0;
        s_node = 5'd0;
        e_node = 5'd5;
        start = 1;
        #10 start = 0;
        wait(done);
        #10;

        // Test 3
        reset = 1;
        #10 reset = 0;
        s_node = 5'd0;
        e_node = 5'd16;
        start = 1;
        #10 start = 0;
        wait(done);
        #10;

        // Test 4
        reset = 1;
        #10 reset = 0;
        s_node = 5'd0;
        e_node = 5'd17;
        start = 1;
        #10 start = 0;
        wait(done);
        #10;

        // Test 5
        reset = 1;
        #10 reset = 0;
        s_node = 5'd3;
        e_node = 5'd12;
        start = 1;
        #10 start = 0;
        wait(done);
        #10;

        // Test 6
        reset = 1;
        #10 reset = 0;
        s_node = 5'd0;
        e_node = 5'd9;
        start = 1;
        #10 start = 0;
        wait(done);
        #10;

        // Test 7
        reset = 1;
        #10 reset = 0;
        s_node = 5'd0;
        e_node = 5'd8;
        start = 1;
        #10 start = 0;
        wait(done);
        #10;

	// Test 8
        reset = 1;
        #10 reset = 0;
        s_node = 5'd0;
        e_node = 5'd4;
        start = 1;
        #10 start = 0;
        wait(done);
        #10;

        // After running all tests, display results
        $display("Dijkstra Algorithm Complete!");
        $display("Clock Cycles: %d", clock_cycles);
        $display("Final Path: %b", final_path);
        $display("Visited Count: %d", visited_count);
        
        // Finish simulation
        #10;
        $finish;
    end
endmodule

