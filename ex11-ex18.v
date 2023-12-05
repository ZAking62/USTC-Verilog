`default_nettype none     // Disable implicit nets. Reduces some types of bugs.
module top_module( 
    input	wire	[15:0]	in,
    output	wire	[7:0]	out_hi,
    output	wire	[7:0]	out_lo 
);
	// Write your code here
    assign out_hi = in[15:8];
    assign out_lo = in[7:0];
endmodule

module top_module(
  input [31:0] in,
  output [31:0] out
);
  // assign out[31:24] = ...;
    assign out[31-:8] = in[7:0];
    assign out[23-:8] = in[8+:8];
    assign out[15:8] = in[23-:8];
    assign out[7:0] = in[31:24];
endmodule

module top_module( 
    input [2:0] a,
    input [2:0] b,
    output [2:0] out_or_bitwise,
    output out_or_logical,
    output [5:0] out_not
);
  // Write your code here
    wire [5:0]out_n;
    assign out_or_bitwise = a | b;
    assign out_or_logical = a || b;
	assign out_n = {b,a}; 
    assign out_not = ~out_n;
endmodule

module top_module( 
    input [3:0] in,
    output out_and,
    output out_or,
    output out_xor
);
    assign out_and = &in;
    assign out_or = |in;
    assign out_xor = ^in;
endmodule


module top_module (
    input [4:0] a, b, c, d, e, f,
    output [7:0] w, x, y, z );
    // assign { ... } = { ... };
    wire [31:0]temp;
    assign temp = {a, b, c, d, e, f, 2'b11};
    assign w = temp[31:24];
    assign x = temp[23-:8];
    assign y = temp[15:8];
    assign z = temp[7:0];
endmodule


module top_module( 
    input [7:0] in,
    output [7:0] out
);
    assign out = {in[0], in[1], in[2], in[3], in[4], in[5], in[6], in[7]};
endmodule


module top_module (
    input [7:0] in,
    output [31:0] out );//
    // assign out = { replicate-sign-bit , the-input };
    assign out = {{24{in[7]}}, in };
endmodule

module top_module (
    input a, b, c, d, e,
    output [24:0] out );//
    // The output is XNOR of two vectors created by 
    // concatenating and replicating the five inputs.
    // assign out = ~{ ... } ^ { ... };
    wire [24:0]nxora, nxorb;
    assign nxora = {5{a, b, c, d, e}};
    assign nxorb = {{5{a}}, {5{b}}, {5{c}}, {5{d}}, {5{e}}};
    assign out = nxora ~^ nxorb;
endmodule




