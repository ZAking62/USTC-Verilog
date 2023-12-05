module my_dff(input clk,input d,output reg q);
	always@(posedge clk)
    	q <= d;
endmodule

module top_module ( input clk, input d, output q );
 // Write your code here
    wire q1, q2;
    my_dff my1(clk, d, q1);
    my_dff my2(clk, q1, q2);
    my_dff my3(clk, q2, q);
endmodule
