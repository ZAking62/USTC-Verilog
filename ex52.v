module top_module(
    input clk,
    input areset,  //异步、高有效、复位值为0
    input load,
    input ena,
    input [3:0] data,
    output reg [3:0] q); 
//Write your code here
    
    always@(posedge clk or posedge areset)begin
        if(areset) q <= 0;
        else begin
            if(load) q <= data;
            else begin
                if(ena) q <= {1'b0, q[3], q[2], q[1]};
                else q <= q;
            end
        end
    end
endmodule
