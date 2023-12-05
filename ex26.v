module add16 ( 
	input[15:0] a, 
    input[15:0] b, 
    input cin, 
    output[15:0] sum, 
    output cout 
);
	assign {cout,sum} = a + b + cin;
endmodule

module top_module(
    input [31:0] a,
    input [31:0] b,
    output [31:0] sum
);
    wire [15:0] sum1, sum2, sum3, sum4;
    wire op;
    add16 add16_1(.a(a[31:16]), .b(b[31:16]), .cin(0), .sum(sum1));
    add16 add16_2(.a(a[31:16]), .b(b[31:16]), .cin(1), .sum(sum2));
    add16 add16_3(.a(a[15:0]), .b(b[15:0]), .cin(0), .sum(sum3), .cout(op));
    assign sum4 = (op == 0) ? sum1 : sum2;
    assign sum = {sum4, sum3};
endmodule
