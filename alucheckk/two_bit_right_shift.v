module two_bit_right_shift(data, out);
    input [31:0] data;
    output [31:0] out;
    assign out[29:0] = data[31:2];
    assign out[30] = data[31] ? 1'b1 : 1'b0;
    assign out[31] = data[31];
endmodule