module counter(out,clk,en,reset);
	input clk,en,reset;
	output [31:0] out;
	wire whoCares;
	wire [31:0] w1, currCount;
	assign w1 = 32'b1;
    counter_reg creg(currCount, clk, en, 1'b1, reset, out);
	adder_32 adder(currCount,out,w1,whoCares,1'b0);
endmodule
