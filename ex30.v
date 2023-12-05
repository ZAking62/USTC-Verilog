module top_module(
    input a,
    input b,
    input sel_b1,
    input sel_b2,
    output wire out_assign,
    output reg out_always); 
// 请用户在下方编辑代码
    assign out_assign = ({sel_b1, sel_b2} == 2'b11) ? b : a;
    always@(*)begin
            if({sel_b1, sel_b2} == 2'b11)
                out_always = b;
            else
                out_always = a;
    end
  
             
//用户编辑到此为止
endmodule