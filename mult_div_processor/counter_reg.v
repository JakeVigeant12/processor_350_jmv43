module counter_reg(in, clk, enable_in, enable_out, reset, out);
    input [31:0] in;
    input clk, enable_in, enable_out, reset;

    output [31:0] out;

    wire [31:0] q;

    genvar p;
    generate
        for(p=0; p < 32; p = p +1) begin: loop
            dffe_ref dffe(q[p], in[p], clk, enable_in, reset);
            assign out[p] = enable_out ? q[p] : 1'bz;
        end
    endgenerate
endmodule