module top_module(
    input clk,
    input areset,    // Asynchronous reset to state B
    input in,
    output out);//  
  
    parameter A=0, B=1; 
	reg state, next_state;
    always @(*) begin    //有限状态机第一段
        // State transition logic
        if(state == B) next_state = in ? B : A;
        else next_state = in ? A:B;
	end
	always @(posedge clk, posedge areset) begin    //有限状态机第二段
        if(areset) state <= B;
        else state = next_state;
        // State flip-flops with asynchronous reset
    end
    //有限状态机第三段，信号输出逻辑
    // assign out = (state == ...);
    assign out = (state == B) ? 1 : 0;
endmodule
