module ctrl_reg(in, clk, enable_in, out);
    input in, clk, enable_in;

    output out;
    wire const;
    assign const = 1'b0;
    dffe_ref ctrl_dffe(out, in, clk, enable_in, const);
endmodule