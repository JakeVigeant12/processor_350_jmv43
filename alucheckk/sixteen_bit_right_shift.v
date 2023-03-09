module sixteen_bit_right_shift(data, out);
    input [31:0] data;
    output [31:0] out;

    assign out[15:0] = data[31:16];
    assign out[30:16] = data[31] ? 15'b111111111111111 : 15'b0;
    assign out[31] = data[31];
endmodule