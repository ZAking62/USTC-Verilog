module top_module (
    input clk,
    input in,
    output reg out
);
	wire dout;
    always@(posedge clk)begin
        out = dout ^ in;
    end
    dff df1(.clk(clk), .in(in), .out(dout)); 
    
endmodule

module dff (
    input clk,
    input in,
    output reg out
);
    always@(posedge clk)
        out <= in;
endmodule