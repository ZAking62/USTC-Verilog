module top_module(
    input 			clk		,  //4Hz
    input 			reset	,
	output	[7:0]	ss
); 
	// Write your code here
    reg [3:0]ss1, ss2;
    reg clk_s = 0;
    reg clk_count = 0;
    always@(posedge clk) begin
        if(reset)begin
            ss1 <= 0;
            ss2 <= 0;
            clk_s <= 0;
            clk_count <= 0;
        end
        else begin
            clk_count <= ~clk_count;
            if(clk_count) clk_s <= ~clk_s;
        end
    end
    always@(negedge clk_s)begin
        if(ss1 < 4'd10) ss1 <= ss1 + 1;
        else begin
            ss1 <= 0;
            if(ss2 < 4'd6) ss2 <= ss2 + 1;
            else ss2 <= 0;
        end
    end
    assign ss = {ss2, ss1};
    
endmodule
