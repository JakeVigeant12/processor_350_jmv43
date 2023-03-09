module xm_latch(clk, o_in, ovfIn, b_in, inIns,  o_out, outOvf, bOut, insOut);

    input clk;
    input [31:0] cPc, inIns, o_in, b_in, ovfIn;
    output [31:0] pcOut, insOut, o_out, bOut, outOvf;

    genvar i;
    generate
        for (i = 0; i < 32; i = i + 1) begin: loop
            dffe_ref o(o_out[i], o_in[i], clk, 1'b1, 1'b0);
            dffe_ref b(bOut[i], b_in[i], clk, 1'b1, 1'b0);
            dffe_ref ins(insOut[i], inIns[i], clk, enable, 1'b0);
        end
    endgenerate
    
endmodule