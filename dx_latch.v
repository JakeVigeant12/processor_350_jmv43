module dx_latch(clk, cPc, a_in, b_in, inIns, pcOut, aOut, bOut, insOut);

    input clk;
    input [31:0] cPc, inIns, a_in, b_in;
    output [31:0] pcOut, insOut, aOut, bOut;

    genvar i;
    generate
        for (i = 0; i < 32; i = i + 1) begin: loop
            dffe_ref a(aOut[i], a_in[i], clk, 1'b1, 1'b0);
            dffe_ref b(bOut[i], b_in[i], clk, 1'b1, 1'b0);
            dffe_ref pc(pcOut[i], cPc[i], clk, 1'b1, 1'b0);
            dffe_ref ins(insOut[i], inIns[i], clk, 1'b1, 1'b0);
        end
    endgenerate
    
endmodule