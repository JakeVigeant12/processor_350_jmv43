module muldiv_latch(clk, mdiv_ctrl, running, ready, inA, inB, ir, out_a, out_b, out_ir);
    input clk, ready, mdiv_ctrl;
    input [31:0] ir, inA, inB;
    output [31:0] out_ir, out_a, out_b;
    output running;

    genvar i;
    generate
        for (i = 0; i < 32; i = i + 1) begin: loop
            dffe_ref ir(out_ir[i], ir[i], clk, mdiv_ctrl, 1'b0);
            dffe_ref a(out_a[i], inA[i], clk, mdiv_ctrl, 1'b0);
            dffe_ref b(out_b[i], inB[i], clk, mdiv_ctrl, 1'b0);
        end   

        dffe_ref dffe_running(running, 1'b1, clk, mdiv_ctrl, ready);
             
    endgenerate
    
endmodule