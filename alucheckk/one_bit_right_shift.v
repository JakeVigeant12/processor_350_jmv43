module one_bit_right_shift (data, out);
    input [31:0] data;
    output [31:0] out;

    assign out[30:0] = data[31:1];
    assign out[31] = data[31];

endmodule