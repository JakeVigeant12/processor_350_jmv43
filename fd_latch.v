module fd_latch(clk, enable, cPc, inIns, pcOut, insOut);
    input clk, enable;
    input [31:0] cPc, inIns;
    output [31:0] pcOut, insOut;

    genvar i;
    generate
        for (i = 0; i < 32; i = i + 1) begin: loop
            dffe_ref pc(pcOut[i], cPc[i], clk, enable, 1'b0);
            dffe_ref ins(insOut[i], inIns[i], clk, enable, 1'b0);
        end
    endgenerate
    
endmodule