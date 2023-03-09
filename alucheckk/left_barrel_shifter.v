module left_barrel_shifter (data, amt, out);
    input [31:0] data;
    input [4:0] amt;
    output [31:0] out;

    wire [31:0] w1,w2,w3,w4, s1, s2,s3,s4,s5;

    //for each mux, input previous level's result, let amt act as select bits to choose
    //if this shift occurs
    //module mux_2(out, select, in0, in1);
    one_bit_left_shift shift1(data, s1);
    mux_2 level1(w1,amt[0],data, s1);

    two_bit_left_shift shift2(w1, s2);
    mux_2 level2(w2,amt[1],w1, s2);

    four_bit_left_shift shift3(w2, s3);
    mux_2 level3(w3,amt[2],w2, s3);

    eight_bit_left_shift shift4(w3, s4);
    mux_2 level4(w4,amt[3],w3, s4);

    sixteen_bit_left_shift shift5(w4, s5);
    mux_2 level5(out,amt[4],w4, s5);

endmodule