module mdiv_latch(clock, ctrl_multdiv, is_running, result_ready, a, b, ir, out_a, out_b, out_ir);
    input clock, ctrl_multdiv, result_ready;
    input [31:0] a, b, ir;
    output [31:0] out_a, out_b, out_ir;
    output is_running;

    genvar i;
    generate
        for (i = 0; i < 32; i = i + 1) begin: loop
            dffe_ref dffe_a(out_a[i], a[i], clock, ctrl_multdiv, 1'b0);
            dffe_ref dffe_b(out_b[i], b[i], clock, ctrl_multdiv, 1'b0);
            dffe_ref dffe_ir(out_ir[i], ir[i], clock, ctrl_multdiv, 1'b0);
        end   

        dffe_ref dffe_is_running(is_running, 1'b1, clock, ctrl_multdiv, result_ready);
             
    endgenerate
    
endmodule