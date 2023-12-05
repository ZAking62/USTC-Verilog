module top_module(
    input clk,
    input reset,
    input en,
    output reg [3:0]q);
    always@(posedge clk)begin
        if(reset) q <= 4'd5;
        else begin
            if(en) begin 
                if(q == 4'd5) q <= 4'd15;
                else q <= q - 1;
            end
        end        
    end
            
endmodule
