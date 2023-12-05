module top_module (
    input clk,
    input in,
    output reg out = 0
);
	wire dout;
    dff df1(.clk(clk), .in(in), .out(dout));
    always@(posedge clk)begin
        if(dout == 0 && in == 1)
            out <= 1;
        else
            out <= 0;
    end 
       
    

endmodule

module dff (
    input clk,
    input in,
    output reg out = 0
);
    always@(posedge clk)
        out <= in;
endmodule