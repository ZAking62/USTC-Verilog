module top_module (
    input clk,
    input in, 
    output reg out);

    wire d1;
    reg q1;
    assign d1 = in ^ q1;
    always@(posedge clk)begin
     	out <= d1;
    	q1 <= d1;
    end
endmodule
