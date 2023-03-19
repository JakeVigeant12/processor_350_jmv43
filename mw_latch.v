module mw_latch(clk, o_in, ovfIn, d_in, inIns,  o_out, outOvf, dOut, insOut);

    input clk, ovfIn;
    input [31:0] cPc, inIns, o_in, d_in;
    output [31:0] pcOut, insOut, o_out, dOut;
    output outOvf;

    genvar i;
    generate
        for (i = 0; i < 32; i = i + 1) begin: loop
            dffe_ref o(o_out[i], o_in[i], clk, 1'b1, 1'b0);
            dffe_ref b(dOut[i], d_in[i], clk, 1'b1, 1'b0);
            dffe_ref ins(insOut[i], inIns[i], clk, 1'b1, 1'b0);
        end
    endgenerate
    
    dffe_ref dffe_ovf(outOvf, ovfIn, clock, 1'b1, 1'b0);

endmodule