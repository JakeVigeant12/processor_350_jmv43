module eight_bit_right_shift(data, out);
    input [31:0] data;
    output [31:0] out;

    assign out[23:0] = data[31:8];
    assign out[30:24] = data[31] ? 7'b1111111 : 7'b0;
    assign out[31] = data[31];
endmodule