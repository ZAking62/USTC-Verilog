module top_module (
    input clk,
    input reset,      // 异步复位，高电平有效，复位值为0
    output [3:0] q);
    wire [3:0]dout,din;
    assign din = dout + 4'b0001;
    assign q = dout;
    dff df1(.clk(clk), .reset(reset), .in(din), .out(dout));
endmodule

module dff (
    input clk,
    input reset,
    input [3:0]in,
    output reg [3:0]out
);
    always@(posedge clk or posedge reset)begin
        if(reset == 1)
            out <= 4'b0000;
        else
            out <= in;
    end
endmodule