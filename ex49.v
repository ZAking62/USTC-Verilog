module top_module (
    input 				clk		,	//时钟信号
    input 				reset	,   //同步高电平有效复位信号
    output reg 	[3:0] 	q			//计数结果
);
// 请用户在下方编辑代码
	
    always@(posedge clk)begin
        if(reset || q == 10)
            q <= 1;
    	else 
            q <= q + 1;
    end


//用户编辑到此为止 
endmodule
