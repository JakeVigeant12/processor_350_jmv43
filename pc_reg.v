module pc_reg(clock, reset, in_enable, in, out);
    input clock, reset, in_enable;
    input [31:0] in;
    output [31:0] out;

    genvar i;
    generate
        for (i = 0; i < 32; i = i + 1) begin: loop
            dffe_neg pc(out[i], in[i], clock, in_enable, reset);
        end
    endgenerate
    
endmodule