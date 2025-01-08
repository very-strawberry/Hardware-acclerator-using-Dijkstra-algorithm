module Dijkstra
#(parameter node_count = 19, parameter max_edges = 4)
(   input reset,
    input clk,
    input start,
    input [4:0] s_node,
    input [4:0] e_node,
    output reg done,
    output reg [31:0] clock_cycles,
    output reg [31:0] cc1,
    output reg [10*5-1:0] final_path,
    output reg [4:0] min_index,
    output reg [5:0] visited_count,
    output reg [3:0] next_state
);


integer k;
reg [4:0] i;
reg [4:0] j;
reg [15:0] sel;
reg [15:0] min;
reg [3:0] count;
// reg [4:0] min_index;
// reg [3:0] next_state;
reg [15:0] dist [0:18]; // keep track of current visited nodes
// reg [5:0] visited_count;
reg [15:0] dist_h [0:18]; // keeps track of history of visited nodes
reg [10*5-1:0] final_path_reg;
 wire [9:0] c1 [0:18];// Cost of the 1st neighbor for each node
    wire [9:0] c2 [0:18]; // Cost of the 2nd neighbor for each node
    wire [9:0] c3 [0:18];// Cost of the 3rd neighbor for each node
    wire [9:0] c4 [0:18];
parameter [9:0] infinity = 10'b1111111111; // infinity
parameter IDLE = 0, UPDATE_VISIT = 3, PATH_RETRACE = 4, NODE_DIST_UPDATE = 1, CHOOSE_NEXT_NODE = 2;
// Node 0 connections
assign c1[0] = {5'd5, 5'd1};  // Cost = 5, Connection to Node 1 
assign c2[0] = {5'd10, 5'd2}; // Cost = 10, Connection to Node 2
assign c3[0] = {5'd15, 5'd3}; // Cost = 15, Connection to Node 3
assign c4[0] = {5'd20, 5'd4}; // Cost = 20, Connection to Node 4

// Node 1 connections
assign c1[1] = {5'd5, 5'd0};  // Cost = 5, Connection to Node 0
assign c2[1] = {5'd7, 5'd5};  // Cost = 7, Connection to Node 5
assign c3[1] = {5'd9, 5'd6};  // Cost = 9, Connection to Node 6
assign c4[1] = {5'd12, 5'd7}; // Cost = 12, Connection to Node 7

// Node 2 connections
assign c1[2] = {5'd10, 5'd0}; // Cost = 10, Connection to Node 0
assign c2[2] = {5'd11, 5'd3}; // Cost = 11, Connection to Node 3
assign c3[2] = {5'd14, 5'd8}; // Cost = 14, Connection to Node 8
assign c4[2] = {5'd18, 5'd9}; // Cost = 18, Connection to Node 9

// Node 3 connections
assign c1[3] = {5'd15, 5'd0}; // Cost = 15, Connection to Node 0
assign c2[3] = {5'd11, 5'd2}; // Cost = 11, Connection to Node 2
assign c3[3] = {5'd8, 5'd10}; // Cost = 8, Connection to Node 10
assign c4[3] = {5'd13, 5'd11}; // Cost = 13, Connection to Node 11

// Node 4 connections
assign c1[4] = {5'd20, 5'd0}; // Cost = 20, Connection to Node 0
assign c2[4] = {5'd22, 5'd5}; // Cost = 22, Connection to Node 5
assign c3[4] = {5'd17, 5'd6}; // Cost = 17, Connection to Node 6
assign c4[4] = {5'd25, 5'd12}; // Cost = 25, Connection to Node 12

// Node 5 connections
assign c1[5] = {5'd7, 5'd1};  // Cost = 7, Connection to Node 1
assign c2[5] = {5'd22, 5'd4}; // Cost = 22, Connection to Node 4
assign c3[5] = {5'd13, 5'd6}; // Cost = 13, Connection to Node 6
assign c4[5] = {5'd16, 5'd7}; // Cost = 16, Connection to Node 7

// Node 6 connections
assign c1[6] = {5'd9, 5'd1};  // Cost = 9, Connection to Node 1
assign c2[6] = {5'd17, 5'd4}; // Cost = 17, Connection to Node 4
assign c3[6] = {5'd13, 5'd5}; // Cost = 13, Connection to Node 5
assign c4[6] = {5'd21, 5'd8}; // Cost = 21, Connection to Node 8

// Node 7 connections
assign c1[7] = {5'd12, 5'd1}; // Cost = 12, Connection to Node 1
assign c2[7] = {5'd16, 5'd5}; // Cost = 16, Connection to Node 5
assign c3[7] = {5'd14, 5'd9}; // Cost = 14, Connection to Node 9
assign c4[7] = {5'd19, 5'd10}; // Cost = 19, Connection to Node 10

// Node 8 connections
assign c1[8] = {5'd14, 5'd2}; // Cost = 14, Connection to Node 2
assign c2[8] = {5'd21, 5'd6}; // Cost = 21, Connection to Node 6
assign c3[8] = {5'd10, 5'd9}; // Cost = 10, Connection to Node 9
assign c4[8] = {5'd16, 5'd11}; // Cost = 16, Connection to Node 11

// Node 9 connections
assign c1[9] = {5'd18, 5'd2}; // Cost = 18, Connection to Node 2
assign c2[9] = {5'd14, 5'd7}; // Cost = 14, Connection to Node 7
assign c3[9] = {5'd10, 5'd8}; // Cost = 10, Connection to Node 8
assign c4[9] = {5'd22, 5'd12}; // Cost = 22, Connection to Node 12

// Node 10 connections
assign c1[10] = {5'd8, 5'd3};  // Cost = 8, Connection to Node 3
assign c2[10] = {5'd19, 5'd7}; // Cost = 19, Connection to Node 7
assign c3[10] = {5'd20, 5'd11}; // Cost = 20, Connection to Node 11
assign c4[10] = {5'd25, 5'd13}; // Cost = 25, Connection to Node 13

// Node 11 connections
assign c1[11] = {5'd13, 5'd3};  // Cost = 13, Connection to Node 3
assign c2[11] = {5'd16, 5'd8};  // Cost = 16, Connection to Node 8
assign c3[11] = {5'd20, 5'd10}; // Cost = 20, Connection to Node 10
assign c4[11] = {5'd22, 5'd12}; // Cost = 22, Connection to Node 12

// Node 12 connections
assign c1[12] = {5'd25, 5'd4};  // Cost = 25, Connection to Node 4
assign c2[12] = {5'd22, 5'd9};  // Cost = 22, Connection to Node 9
assign c3[12] = {5'd22, 5'd11}; // Cost = 22, Connection to Node 11
assign c4[12] = {5'd18, 5'd13}; // Cost = 18, Connection to Node 13

// Node 13 connections
assign c1[13] = {5'd25, 5'd10}; // Cost = 25, Connection to Node 10
assign c2[13] = {5'd18, 5'd12}; // Cost = 18, Connection to Node 12
assign c3[13] = {5'd20, 5'd14}; // Cost = 20, Connection to Node 14
assign c4[13] = {5'd30, 5'd15}; // Cost = 30, Connection to Node 15

// Node 14 connections
assign c1[14] = {5'd20, 5'd13}; // Cost = 20, Connection to Node 13
assign c2[14] = {5'd25, 5'd16}; // Cost = 25, Connection to Node 15
assign c3[14] = {5'd25, 5'd17}; // Cost = 10, Connection to Node 16
assign c4[14] = {5'd20, 5'd18}; // No connection

// Node 15 connections
assign c1[15] = {5'd30, 5'd13}; // Cost = 30, Connection to Node 13
assign c2[15] = {5'd25, 5'd18}; // Cost = 25, Connection to Node 14
assign c3[15] = {5'd28, 5'd16}; // Cost = 15, Connection to Node 16
assign c4[15] = {5'd30, 5'd17}; // Cost = 22, Connection to Node 17

// Node 16 connections
assign c1[16] = {5'd22, 5'd17}; // Cost = 10, Connection to Node 14
assign c2[16] = {5'd28, 5'd15}; // Cost = 15, Connection to Node 15
assign c3[16] = {5'd25, 5'd14}; // Cost = 25, Connection to Node 17
assign c4[16] = {5'd20, 5'd18}; // No connection

// Node 17 connections
assign c1[17] = {5'd22, 5'd14}; // Cost = 22, Connection to Node 15
assign c2[17] = {5'd25, 5'd16}; // Cost = 25, Connection to Node 16
assign c3[17] = {5'd10, 5'd18}; // Cost = 10, Connection to Node 18
assign c4[17] = {5'd10, 5'd16}; // No connection

// Node 18 connections
assign c1[18] = {5'd10, 5'd17}; // Cost = 10, Connection to Node 17
assign c2[18] = {5'd10, 5'd16}; // No connection
assign c3[18] = {5'd20, 5'd14}; // No connection
assign c4[18] = {5'd10, 5'd15}; // No connection


always @(posedge clk) begin
      if(reset) begin
       final_path_reg <= 0;
    final_path <= 0;
    clock_cycles <= 0;
    cc1<=0;
    next_state <= IDLE;
    min <= 0;
    min_index <= 0;
    done <= 0;
    sel <= {1'b1, {1'b0, 1'b0, 1'b0, 1'b0, 1'b0}, {infinity}};
    visited_count <= 0;
    count <= 0;
end
  else begin
    clock_cycles <= clock_cycles + 1;
     case(next_state)
        IDLE: begin
            if (start == 1) begin
                done <= 0;
                visited_count <= 0;
                j <= s_node;
                count <= 1;
                final_path <= e_node;
                sel <= {1'b1,s_node,{infinity}};
    next_state <= NODE_DIST_UPDATE;
                i <= 0;
                //for (k = 1; k < node_count; k = k + 1) begin
                 // dist[k] <= {1'b0, s_node, infinity};
                 // dist_h[k] <= {1'b0, s_node, infinity};
                //end
                                dist[0] <= {1'b0,s_node[4:0],{infinity}};
                                dist[1] <= {1'b0,s_node[4:0],{infinity}};
                                dist[2] <= {1'b0,s_node[4:0],{infinity}};
                                dist[3] <= {1'b0,s_node[4:0],{infinity}};
                                dist[4] <= {1'b0,s_node[4:0],{infinity}};
                                dist[5] <= {1'b0,s_node[4:0],{infinity}};
                                dist[6] <= {1'b0,s_node[4:0],{infinity}};
                                dist[7] <= {1'b0,s_node[4:0],{infinity}};
                                dist[8] <= {1'b0,s_node[4:0],{infinity}};
                                dist[9] <= {1'b0,s_node[4:0],{infinity}};
                                dist[10] <= {1'b0,s_node[4:0],{infinity}};
                                dist[11] <= {1'b0,s_node[4:0],{infinity}};
                                dist[12] <= {1'b0,s_node[4:0],{infinity}};
                    dist[13] <= {1'b0,s_node[4:0],{infinity}};
                                dist[14] <= {1'b0,s_node[4:0],{infinity}};
                                dist[15] <= {1'b0,s_node[4:0],{infinity}};
                                dist[16] <= {1'b0,s_node[4:0],{infinity}};
                                dist[17] <= {1'b0,s_node[4:0],{infinity}};
                                dist[18] <= {1'b0,s_node[4:0],{infinity}};
                               
                             dist_h[0] <= {1'b0,s_node[4:0],{infinity}};
                                dist_h[1] <= {1'b0,s_node[4:0],{infinity}};
                                dist_h[2] <= {1'b0,s_node[4:0],{infinity}};
                                dist_h[3] <= {1'b0,s_node[4:0],{infinity}};
                                dist_h[4] <= {1'b0,s_node[4:0],{infinity}};
                                dist_h[5] <= {1'b0,s_node[4:0],{infinity}};
                                dist_h[6] <= {1'b0,s_node[4:0],{infinity}};
                                dist_h[7] <= {1'b0,s_node[4:0],{infinity}};
                                dist_h[8] <= {1'b0,s_node[4:0],{infinity}};
                                dist_h[9] <= {1'b0,s_node[4:0],{infinity}};
                                dist_h[10] <= {1'b0,s_node[4:0],{infinity}};
                                dist_h[11] <= {1'b0,s_node[4:0],{infinity}};
                                dist_h[12] <= {1'b0,s_node[4:0],{infinity}};
                    dist_h[13] <= {1'b0,s_node[4:0],{infinity}};
                                dist_h[14] <= {1'b0,s_node[4:0],{infinity}};
                                dist_h[15] <= {1'b0,s_node[4:0],{infinity}};
                                dist_h[16] <= {1'b0,s_node[4:0],{infinity}};
                                dist_h[17] <= {1'b0,s_node[4:0],{infinity}};
                                dist_h[18] <= {1'b0,s_node[4:0],{infinity}};

                dist[s_node] <= {1'b1, s_node, 10'b0};
                dist_h[s_node] <= {1'b1, s_node, 10'b0};
                final_path_reg[4:0] <= e_node;
                sel <= {1'b1, s_node, infinity};
                end else begin
                    next_state <= IDLE;
                end
            end
       
       
        NODE_DIST_UPDATE: begin
        cc1<=cc1+1;
if (((c1[j][9:5] + min[9:0]) < dist[c1[j][4:0]][9:0]) & (dist[c1[j][4:0]][15] != 1)) begin
    dist[c1[j][4:0]] <= {1'b0, j, {c1[j][9:5] + min[9:0]}};
end else begin
    dist[c1[j][4:0]] <= dist_h[c1[j][4:0]];
end

if (((c2[j][9:5] + min[9:0]) < dist[c2[j][4:0]][9:0]) & (dist[c2[j][4:0]][15] != 1)) begin
    dist[c2[j][4:0]] <= {1'b0, j, {c2[j][9:5] + min[9:0]}};
end else begin
    dist[c2[j][4:0]] <= dist_h[c2[j][4:0]];
end

if (((c3[j][9:5] + min[9:0]) < dist[c3[j][4:0]][9:0]) & (dist[c3[j][4:0]][15] != 1)) begin
    dist[c3[j][4:0]] <= {1'b0, j, {c3[j][9:5] + min[9:0]}};
end else begin
    dist[c3[j][4:0]] <= dist_h[c3[j][4:0]];
end

if (((c4[j][9:5] + min[9:0]) < dist[c4[j][4:0]][9:0]) & (dist[c4[j][4:0]][15] != 1)) begin
    dist[c4[j][4:0]] <= {1'b0, j, {c4[j][9:5] + min[9:0]}};
end else begin
    dist[c4[j][4:0]] <= dist_h[c4[j][4:0]];
end

next_state <= CHOOSE_NEXT_NODE;
        end
        CHOOSE_NEXT_NODE: begin
            if(i < node_count) begin
                if ((dist[i][9:0] < sel[9:0]) & (dist[i][15] != 1)) begin
                    sel <= {1'b0, j, dist[i][9:0]};
                    min_index <= i;
                end
                i <= i + 1;
                next_state <= CHOOSE_NEXT_NODE;
            end else begin
                next_state <= UPDATE_VISIT;
               
            end
        end
              
        
 
         UPDATE_VISIT: begin
            if (visited_count == 19) begin // Check if all nodes have been mapped
                next_state <= PATH_RETRACE;
                j <= e_node;
                dist_h[s_node][14:10] <= 5'b11011;
                final_path_reg[49:5] <= {10{5'd27}};
            end else if (i == node_count) begin // Check if all nodes have been mapped for the current node
                min <= {1'b1, sel[14:0]};
                visited_count <= visited_count + 1;
                dist[min_index][15] <= 1'b1;
                dist_h[min_index][15] <= 1'b1;
                i <= 0;
                j <= min_index;
                sel <= {1'b1, min_index, infinity}; // This is the node to begin next iteration of mapping (having min distance)
                next_state <= NODE_DIST_UPDATE;
                dist_h[0] <= dist[0];
                dist_h[1] <= dist[1];
                dist_h[2] <= dist[2];
                dist_h[3] <= dist[3];
                dist_h[4] <= dist[4];
                dist_h[5] <= dist[5];
                dist_h[6] <= dist[6];
                dist_h[7] <= dist[7];
                dist_h[8] <= dist[8];
                dist_h[9] <= dist[9];
                dist_h[10] <= dist[10];
                dist_h[11] <= dist[11];
                dist_h[12] <= dist[12];
                dist_h[13] <= dist[13];
                dist_h[14] <= dist[14];
                dist_h[15] <= dist[15];
                dist_h[16] <= dist[16];
                dist_h[17] <= dist[17];
                dist_h[18] <= dist[18];
               
               
            end
        end
        PATH_RETRACE: begin
            if (j == 5'b11011) begin
                done <= 1;
                final_path <= final_path_reg;
                next_state <= PATH_RETRACE;
            end else begin
                final_path_reg[(5 * count + 4) -: 5] <= dist_h[j][14:10];
                count <= count + 1;
                j <= dist_h[j][14:10];
            end
        end
     endcase
end 
end
endmodule