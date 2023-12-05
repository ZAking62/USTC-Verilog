module top_module (
    input 				clk,
    input [7:0] 		d,
    output reg [7:0] 	q
);
// 请用户在下方编辑代码
    always@(posedge clk)
        q <= d;


//用户编辑到此为止
endmodule

module top_module (
    input clk,
    input reset,            // Synchronous reset
    input [7:0] d,
    output reg [7:0] q
);
    always@(posedge clk)
        q <= reset ? 0 : d;
endmodule

module top_module (
    input				clk		,
    input				reset	,
    input		[7:0]	d		,
    output	reg	[7:0]	q
);
	// Write your code here
    always@(negedge clk)
        q <= reset ? 7'h34 : d;
endmodule

module top_module (
    input clk,
    input areset,   // active high asynchronous reset
    input [7:0] d,
    output reg [7:0] q
);
// Write your code here
    always@(posedge clk or posedge areset)
        q <= areset ? 0 : d;
endmodule

