module top_module(
  output out
);
  assign out = 1;
endmodule

module top_module(
  output out
);
  // Write your code here
    assign out = 1'b0;      
endmodule

module top_module(
  input in, output out
);
  assign out = in;
endmodule

module top_module( 
    input a,b,c,
    output w,x,y,z );
// 请用户在下方编辑代码
	assign w = a;
    assign x = b;
    assign y = b;
    assign z = c;
//用户编辑到此为止
endmodule

module top_module( input in, output out );
// 请用户在下方编辑代码
  assign out = ~in;
//用户编辑到此为止
endmodule

module top_module(
  input a, 
  input b,
  output out );
// 请用户在下方编辑代码
	assign out = a & b;
//用户编辑到此为止
endmodule

module top_module( 
    input a, 
    input b, 
    output out );
// 请用户在下方编辑代码
    assign out = ~(a | b);
//用户编辑到此为止
endmodule

module top_module( 
    input a, 
    input b, 
    output out );
// 请用户在下方编辑代码
	assign out = a ~^ b;
//用户编辑到此为止
endmodule

module top_module(
    input a,
    input b,
    input c,
    input d,
    output out,
    output out_n   
); 
// 请用户在下方编辑代码
	wire and1, and2, or1;
    assign and1 = a & b;
    assign and2 = c & d;
    assign or1 = and1 | and2;
    assign out = or1;
    assign out_n = ~or1;
//用户编辑到此为止
endmodule

module top_module ( 
    input wire [2:0] vec,
    output wire [2:0] outv,
    output wire o2,
    output wire o1,
    output wire o0);
// Module body starts after module declaration
// 请用户在下方编辑代码
	assign outv = vec;
    assign o2 = vec[2];
    assign o1 = vec[1];
    assign o0 = vec[0];
// 用户编辑到此为止
endmodule


