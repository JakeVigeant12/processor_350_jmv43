module right_barrel_shifter (data, amt, out);
    input [31:0] data;
    input [4:0] amt;
    output [31:0] out;

    wire [31:0] w1, w2, w3, w4, w5;
    wire [31:0] shift1, shift2, shift3, shift4;

    sixteen_bit_right_shift s16(data, w1);
    assign shift1 = amt[4] ? w1 : data;

    eight_bit_right_shift s8(shift1, w2);
    assign shift2 = amt[3] ? w2 : shift1;

    four_bit_right_shift s4(shift2, w3);
    assign shift3 = amt[2] ? w3 : shift2;

    two_bit_right_shift s2(shift3, w4);
    assign shift4 = amt[1] ? w4 : shift3;

    one_bit_right_shift s1(shift4, w5);
    assign out = amt[0] ? w5 : shift4;



endmodule