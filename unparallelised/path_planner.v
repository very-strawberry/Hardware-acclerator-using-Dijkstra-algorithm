module path_planner
#(parameter node_count = 19, parameter max_edges = 4)
(
	input clk,
	input reset,
	input start,
	input [4:0] s_node,
	input [4:0] e_node,
	output reg [31:0] clock_cycles,
	output reg [31:0] cc1,
	output reg done,	
	output reg [10*5-1:0] final_path
);
reg [9:0] check;
reg [4:0] i;
reg [4:0] j;
reg [4:0] temp;
parameter [9:0] infinity = 10'b1111111111; //infinity
wire [9:0] cost_matrix[0:18][0:3];

reg [15:0] dist_h [0:18]; //keep track of previous visited nodes
reg [15:0] dist [0:18]; //keep track of current visited nodes
reg [10*5-1:0] final_path_reg = 0; //{10{5'd27}};
localparam IDLE = 0, UPDATE_VISIT = 3, S1 = 4,NODE_DIST_UPDATE=1,CHOOSE_NEXT_NODE=2;

// Node 0 connections
assign cost_matrix[0][0] = {5'd5, 5'd1};   // Cost = 5, Connection to Node 1
assign cost_matrix[0][1] = {5'd10, 5'd2};  // Cost = 10, Connection to Node 2
assign cost_matrix[0][2] = {5'd15, 5'd3};  // Cost = 15, Connection to Node 3
assign cost_matrix[0][3] = {5'd20, 5'd4};  // Cost = 20, Connection to Node 4

// Node 1 connections
assign cost_matrix[1][0] = {5'd5, 5'd0};   // Cost = 5, Connection to Node 0
assign cost_matrix[1][1] = {5'd7, 5'd5};   // Cost = 7, Connection to Node 5
assign cost_matrix[1][2] = {5'd9, 5'd6};   // Cost = 9, Connection to Node 6
assign cost_matrix[1][3] = {5'd12, 5'd7};  // Cost = 12, Connection to Node 7

// Node 2 connections
assign cost_matrix[2][0] = {5'd10, 5'd0};  // Cost = 10, Connection to Node 0
assign cost_matrix[2][1] = {5'd11, 5'd3};  // Cost = 11, Connection to Node 3
assign cost_matrix[2][2] = {5'd14, 5'd8};  // Cost = 14, Connection to Node 8
assign cost_matrix[2][3] = {5'd18, 5'd9};  // Cost = 18, Connection to Node 9

// Node 3 connections
assign cost_matrix[3][0] = {5'd15, 5'd0};  // Cost = 15, Connection to Node 0
assign cost_matrix[3][1] = {5'd11, 5'd2};  // Cost = 11, Connection to Node 2
assign cost_matrix[3][2] = {5'd8, 5'd10};  // Cost = 8, Connection to Node 10
assign cost_matrix[3][3] = {5'd13, 5'd11}; // Cost = 13, Connection to Node 11

// Node 4 connections
assign cost_matrix[4][0] = {5'd20, 5'd0};  // Cost = 20, Connection to Node 0
assign cost_matrix[4][1] = {5'd22, 5'd5};  // Cost = 22, Connection to Node 5
assign cost_matrix[4][2] = {5'd17, 5'd6};  // Cost = 17, Connection to Node 6
assign cost_matrix[4][3] = {5'd25, 5'd12}; // Cost = 25, Connection to Node 12

// Node 5 connections
assign cost_matrix[5][0] = {5'd7, 5'd1};   // Cost = 7, Connection to Node 1
assign cost_matrix[5][1] = {5'd22, 5'd4};  // Cost = 22, Connection to Node 4
assign cost_matrix[5][2] = {5'd13, 5'd6};  // Cost = 13, Connection to Node 6
assign cost_matrix[5][3] = {5'd16, 5'd7};  // Cost = 16, Connection to Node 7

// Node 6 connections
assign cost_matrix[6][0] = {5'd9, 5'd1};   // Cost = 9, Connection to Node 1
assign cost_matrix[6][1] = {5'd17, 5'd4};  // Cost = 17, Connection to Node 4
assign cost_matrix[6][2] = {5'd13, 5'd5};  // Cost = 13, Connection to Node 5
assign cost_matrix[6][3] = {5'd21, 5'd8};  // Cost = 21, Connection to Node 8

// Node 7 connections
assign cost_matrix[7][0] = {5'd12, 5'd1};  // Cost = 12, Connection to Node 1
assign cost_matrix[7][1] = {5'd16, 5'd5};  // Cost = 16, Connection to Node 5
assign cost_matrix[7][2] = {5'd14, 5'd9};  // Cost = 14, Connection to Node 9
assign cost_matrix[7][3] = {5'd19, 5'd10}; // Cost = 19, Connection to Node 10

// Node 8 connections
assign cost_matrix[8][0] = {5'd14, 5'd2};  // Cost = 14, Connection to Node 2
assign cost_matrix[8][1] = {5'd21, 5'd6};  // Cost = 21, Connection to Node 6
assign cost_matrix[8][2] = {5'd10, 5'd9};  // Cost = 10, Connection to Node 9
assign cost_matrix[8][3] = {5'd16, 5'd11}; // Cost = 16, Connection to Node 11

// Node 9 connections
assign cost_matrix[9][0] = {5'd18, 5'd2};  // Cost = 18, Connection to Node 2
assign cost_matrix[9][1] = {5'd14, 5'd7};  // Cost = 14, Connection to Node 7
assign cost_matrix[9][2] = {5'd10, 5'd8};  // Cost = 10, Connection to Node 8
assign cost_matrix[9][3] = {5'd22, 5'd12}; // Cost = 22, Connection to Node 12

// Node 10 connections
assign cost_matrix[10][0] = {5'd8, 5'd3};   // Cost = 8, Connection to Node 3
assign cost_matrix[10][1] = {5'd19, 5'd7};  // Cost = 19, Connection to Node 7
assign cost_matrix[10][2] = {5'd20, 5'd11}; // Cost = 20, Connection to Node 11
assign cost_matrix[10][3] = {5'd25, 5'd13}; // Cost = 25, Connection to Node 13

// Node 11 connections
assign cost_matrix[11][0] = {5'd13, 5'd3};  // Cost = 13, Connection to Node 3
assign cost_matrix[11][1] = {5'd16, 5'd8};  // Cost = 16, Connection to Node 8
assign cost_matrix[11][2] = {5'd20, 5'd10}; // Cost = 20, Connection to Node 10
assign cost_matrix[11][3] = {5'd22, 5'd12}; // Cost = 22, Connection to Node 12

// Node 12 connections
assign cost_matrix[12][0] = {5'd25, 5'd4};  // Cost = 25, Connection to Node 4
assign cost_matrix[12][1] = {5'd22, 5'd9};  // Cost = 22, Connection to Node 9
assign cost_matrix[12][2] = {5'd22, 5'd11}; // Cost = 22, Connection to Node 11
assign cost_matrix[12][3] = {5'd18, 5'd13}; // Cost = 18, Connection to Node 13

// Node 13 connections
assign cost_matrix[13][0] = {5'd25, 5'd10}; // Cost = 25, Connection to Node 10
assign cost_matrix[13][1] = {5'd18, 5'd12}; // Cost = 18, Connection to Node 12
assign cost_matrix[13][2] = {5'd20, 5'd14}; // Cost = 20, Connection to Node 14
assign cost_matrix[13][3] = {5'd30, 5'd15}; // Cost = 30, Connection to Node 15

// Node 14 connections
assign cost_matrix[14][0] = {5'd20, 5'd13}; // Cost = 20, Connection to Node 13
assign cost_matrix[14][1] = {5'd25, 5'd16}; // Cost = 25, Connection to Node 16
assign cost_matrix[14][2] = {5'd25, 5'd17}; // Cost = 25, Connection to Node 17
assign cost_matrix[14][3] = {5'd20, 5'd18}; // Cost = 20, Connection to Node 18

// Node 15 connections
assign cost_matrix[15][0] = {5'd30, 5'd13}; // Cost = 30, Connection to Node 13
assign cost_matrix[15][1] = {5'd25, 5'd18}; // Cost = 25, Connection to Node 18
assign cost_matrix[15][2] = {5'd28, 5'd16}; // Cost = 28, Connection to Node 16
assign cost_matrix[15][3] = {5'd30, 5'd17}; // Cost = 30, Connection to Node 17

// Node 16 connections
assign cost_matrix[16][0] = {5'd22, 5'd17}; // Cost = 22, Connection to Node 17
assign cost_matrix[16][1] = {5'd28, 5'd15}; // Cost = 28, Connection to Node 15
assign cost_matrix[16][2] = {5'd25, 5'd14}; // Cost = 25, Connection to Node 14
assign cost_matrix[16][3] = {5'd20, 5'd18}; // Cost = 20, Connection to Node 18

// Node 17 connections
assign cost_matrix[17][0] = {5'd22, 5'd14}; // Cost = 22, Connection to Node 14
assign cost_matrix[17][1] = {5'd25, 5'd16}; // Cost = 25, Connection to Node 16
assign cost_matrix[17][2] = {5'd10, 5'd18}; // Cost = 10, Connection to Node 18
assign cost_matrix[17][3] = {5'd10, 5'd16}; // Cost = 10, Connection to Node 16

// Node 18 connections
assign cost_matrix[18][0] = {5'd10, 5'd17}; // Cost = 10, Connection to Node 17
assign cost_matrix[18][1] = {5'd20, 5'd14}; // Cost = 20, Connection to Node 14
assign cost_matrix[18][2] = {5'd25, 5'd15}; // Cost = 25, Connection to Node 15
assign cost_matrix[18][3] = {5'd22, 5'd16}; // Cost = 22, Connection to Node 16



reg [3:0] next_state = 0;

reg [15:0] min = 0;
reg [4:0] min_index = 0;
reg [15:0] sel;
reg [4:0] k;
reg [4:0] visited_count = 0;
reg [3:0] count=0;


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
			if(start==1) begin
				done <= 0;
				visited_count <=0;
				j <= s_node;
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
				dist_h[s_node][15] <= 1;
				dist[s_node][15] <= 1;
				count <= 1;
                k <= 0;
                i <= 0;
				final_path_reg[4:0] <= e_node;
				sel <= {1'b1,s_node,{infinity}};
				next_state <= NODE_DIST_UPDATE;
			end 
			else 
			next_state <=IDLE;
        end
        NODE_DIST_UPDATE: begin
        cc1<= cc1+1;
            if( i < max_edges) begin
                    if ( ((cost_matrix[j][i][9:5] + min[9:0]) < dist_h[cost_matrix[j][i][4:0]][9:0]) && (dist_h[cost_matrix[j][i][4:0]][15] != 1) ) begin
		        		dist[cost_matrix[j][i][4:0]] <= {1'b0,j,{cost_matrix[j][i][9:5] + min[9:0]}};    //update node dstance if it is less than path(min)
		        	end
		        	else begin
		        		dist[cost_matrix[j][i][4:0]] <= dist_h[cost_matrix[j][i][4:0]];
		        	end
                    i <= i + 1;
		        	next_state<= NODE_DIST_UPDATE;
		        end
            else begin
                dist_h[0] <= dist[0] ;
		    	dist_h[1] <= dist[1] ; 
		    	dist_h[2] <= dist[2] ;
		    	dist_h[3] <= dist[3] ;
		    	dist_h[4] <= dist[4] ; 
		    	dist_h[5] <= dist[5] ;
		    	dist_h[6] <= dist[6] ; 
		    	dist_h[7] <= dist[7] ; 
                dist_h[8] <= dist[8] ; 
                dist_h[9] <= dist[9] ; 
                dist_h[10] <= dist[10] ; 
                dist_h[11] <= dist[11] ; 
                dist_h[12] <= dist[12] ; 
                dist_h[13] <= dist[13] ; 
                dist_h[14] <= dist[14] ; 
                dist_h[15] <= dist[15] ; 
                dist_h[16] <= dist[16] ; 
                dist_h[17] <= dist[17] ; 
                dist_h[18] <= dist[18] ;
                next_state <= CHOOSE_NEXT_NODE;
            end
        end
		UPDATE_VISIT: begin 
			if(visited_count == 19) begin
				next_state <= S1;
				j<=e_node;
				dist_h[s_node][14:10] <= 5'b11011;
                final_path_reg[49:5] <= {10{5'd27}};
			end
			else if(k == node_count) begin         //check end of nodes
				min <= {1'b1,sel[14:0]};              //minimum = select(marked as done)
				visited_count <= visited_count + 1;   //increase visit count
				dist[min_index][15] <= 1'b1;
				dist_h[min_index][15] <= 1'b1;           //mark minimum index as visited
				j <= min_index;   //next column node = minimum index
				i <= 0;  
				k <= 0;        
				sel <=  {1'b1,{min_index[4:0]},{infinity}}; //selected node = minimum index marked as visited and with infinite distance
				next_state<=NODE_DIST_UPDATE;
			end
		end
		CHOOSE_NEXT_NODE: begin
        
            if(k < node_count) begin
			    if((dist[k][9:0] < sel[9:0]) && (dist[k][15] != 1)) begin
			    	sel <= {1'b0,j,dist[k][9:0]};
			    	min_index <= k;
			    end else begin
                    k <= k + 1;
                end
                next_state <= CHOOSE_NEXT_NODE;
            end 
            else begin
                next_state <= UPDATE_VISIT;
            end
		end
		S1: begin
			if(j == 5'b11011) begin
				done <= 1;
				next_state <= S1;
				final_path <= final_path_reg;
			end
			else begin
				final_path_reg[5 * count + 4] <= dist_h[j][14];
                final_path_reg[5 * count + 3] <= dist_h[j][13];
                final_path_reg[5 * count + 2] <= dist_h[j][12];
                final_path_reg[5 * count + 1] <= dist_h[j][11];
                final_path_reg[5 * count]     <= dist_h[j][10]; 
				final_path <= final_path_reg;
				check <= dist_h[j][14:0]; // check ,remove later
				count <= count+1;
				j <= dist_h[j][14:10];
			end
		end
	endcase
    end
end


////////////////////////YOUR CODE ENDS HERE//////////////////////////
endmodule