module top_module(
	input	[2:0] addr,
	output	[3:0]	q
);
    reg [3:0]mem [7:0];
    initial begin
        mem[0] = 4'd0;
        mem[1] = 4'd1;
        mem[2] = 4'd2;
        mem[3] = 4'd3;
        mem[4] = 4'd4;
        mem[5] = 4'd5;
        mem[6] = 4'd6;
        mem[7] = 4'd7;
    end
    assign q = mem[addr];
endmodule
