`timescale 10ns/10ns
module tb();
reg a,b;
wire q;
  
//对ab信号进行初始化
  initial begin
      a = 1'b0;
      b = 1'b0;
      #3 b = 1'b1;
      #2 a = 1'b1; b = 1'b0;
      #2 b = 1'b1;
      #2 a = 1'b0; b = 1'b0;
      #2 b = 1'b1;
      #2 a = 1'b1; b = 1'b0;
      #2 b = 1'b1;
      #2 a = 1'b0; b = 1'b0;
      #2 $finish;
  end
  
      
      
//例化mymodule模块
    mymodule my(.a(a), .b(b), .q(q));
endmodule


module mymodule(
input a,b,
output q
);
  
assign q = a & b;
  
endmodule