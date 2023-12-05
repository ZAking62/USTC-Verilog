module add16 ( input[15:0] a, input[15:0] b, input cin, output[15:0] sum, output cout );
	assign {cout,sum} = a + b + cin;
endmodule
module top_module(
    input [31:0] a,
    input [31:0] b,
    input sub,
    output [31:0] sum
);
    wire cout1;
    wire [15:0] sum1, sum2;
    wire [31:0]toaddb;
    assign toaddb = (sub == 0) ? b : ({32{1'b1}} ^ b);
    add16 add16_1(.a(a[15:0]), .b(toaddb[15:0]), .cin(sub), .sum(sum1), .cout(cout1));
    add16 add16_2(.a(a[31:16]), .b(toaddb[31:16]), .cin(cout1), .sum(sum2));
    assign sum = {sum2, sum1};
endmodule