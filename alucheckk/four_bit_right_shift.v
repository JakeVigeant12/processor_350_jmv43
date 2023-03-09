module four_bit_right_shift(data, out);
    input [31:0] data;
    output [31:0] out;

    assign out[27:0] = data[31:4];
    assign out[30:28] = data[31] ? 3'b111 : 3'b0;
    assign out[31] = data[31];
    
endmodule