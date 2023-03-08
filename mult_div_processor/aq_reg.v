module aq_reg(in, clk, enable_in, enable_out, reset, out);
    input [63:0] in;
    input clk, enable_in, enable_out, reset;

    output [63:0] out;

    wire [63:0] q;
    wire impState;
    assign impState = 1'bz;
    genvar p;
    generate
        for(p=0; p < 64; p = p +1) begin: loop
            dffe_ref dffe(q[p], in[p], clk, enable_in, reset);
            assign out[p] = enable_out ? q[p] : impState;
        end
    endgenerate
endmodule