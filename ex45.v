module top_module (
    input clk,
    input x,
    output z
); 
	wire d1, d2, d3, q1, q2, q3;
    assign d1 = q1 ^ x;
    assign d2 = (~q2) & x;
    assign d3 = (~q3) | x;
    dff dff1(clk, d1, q1);
    dff dff2(clk, d2, q2);
    dff dff3(clk, d3, q3);
    assign z = ~(q1 | q2 | q3);
endmodule

module dff (
    input clk,
    input d,
    output reg q = 0
);
    always@(posedge clk)
        q <= d;
endmodule
        
        
