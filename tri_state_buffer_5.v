module tri_state_buffer_5(in, enable, out);
    input [4:0] in;
    input enable;
    output [4:0] out;

    assign out = enable ? in : 5'bz;

endmodule