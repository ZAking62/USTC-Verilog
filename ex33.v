module top_module(
  input [3:0] in,
  output reg [1:0] pos
);
  // Write your code here
	integer i;
    always@(*)begin
        if(in == 4'b0000)
            pos = 2'b00;
    	else
            for(i = 3; i >= 0; i--)
                if(in[i] == 1)
                    pos = i[1:0];
    end
endmodule