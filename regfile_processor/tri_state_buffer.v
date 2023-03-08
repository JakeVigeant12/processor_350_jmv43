module tri_state_buffer(out, inp, enable);
    input [31:0] inp;
    input enable;
    output[31:0] out;
    //Either input or high impedance state
    assign out = enable ? inp : 32'bz;
endmodule