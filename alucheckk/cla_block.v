module cla_block(a, b, cin, p, g, Po, Go, s);
    input [7:0] a, b, p, g;
    input cin;
    output Po, Go;
    output [7:0] s;

    //Line for carrb ins to each adder 
    wire [8:0] c;
    wire c_out;
    //Carry in to first cell is input to block
    assign c[0] = cin;
    //carry out to next block 
    assign c_out = c[8];

    //Construct sum bits with adder cells
    one_bit_adder first(a[0], b[0], c[0], s[0]);
    one_bit_adder second(a[1], b[1], c[1], s[1]);
    one_bit_adder third(a[2], b[2], c[2], s[2]);
    one_bit_adder fourth(a[3], b[3], c[3], s[3]);
    one_bit_adder fifth(a[4], b[4], c[4], s[4]);
    one_bit_adder siath(a[5], b[5], c[5], s[5]);
    one_bit_adder seventh(a[6], b[6], c[6], s[6]);
    one_bit_adder eight(a[7], b[7], c[7], s[7]);

    // calculate carry in's, ci+1 = pi&&ci + gi
    wire w1;
    and c1_and1(w1, p[0], c[0]);
    or c1_or(c[1], g[0], w1);

    wire [1:0] w2;
    and c2_and1(w2[0], p[1], g[0]);
    and c2_and2(w2[1], p[1], p[0], c[0]);
    or c2_or(c[2], g[1], w2[0], w2[1]);

    wire [2:0] w3;
    and c3_and1(w3[0], p[2], g[1]);
    and c3_and2(w3[1], p[2], p[1], g[0]);
    and c3_and3(w3[2], p[2], p[1], p[0], c[0]);
    or c3_or(c[3], g[2], w3[0], w3[1], w3[2]);

    wire [3:0] w4;
    and c4_and1(w4[0], p[3], g[2]);
    and c4_and2(w4[1], p[3], p[2], g[1]);
    and c4_and3(w4[2], p[3], p[2], p[1], g[0]);
    and c4_and4(w4[3], p[3], p[2], p[1], p[0], c[0]);
    or c4_or(c[4], g[3], w4[0], w4[1], w4[2], w4[3]);

    wire [4:0] w5;
    and c5_and1(w5[0], p[4], g[3]);
    and c5_and2(w5[1], p[4], p[3], g[2]);
    and c5_and3(w5[2], p[4], p[3], p[2], g[1]);
    and c5_and4(w5[3], p[4], p[3], p[2], p[1], g[0]);
    and c5_and5(w5[4], p[4], p[3], p[2], p[1], p[0], c[0]);
    or c5_or(c[5], g[4], w5[0], w5[1], w5[2], w5[3], w5[4]);

    wire [5:0] w6;
    and c6_and1(w6[0], p[5], g[4]);
    and c6_and2(w6[1], p[5], p[4], g[3]);
    and c6_and3(w6[2], p[5], p[4], p[3], g[2]);
    and c6_and4(w6[3], p[5], p[4], p[3], p[2], g[1]);
    and c6_and5(w6[4], p[5], p[4], p[3], p[2], p[1], g[0]);
    and c6_and6(w6[5], p[5], p[4], p[3], p[2], p[1], p[0], c[0]);
    or c6_or(c[6], g[5], w6[0], w6[1], w6[2], w6[3], w6[4], w6[5]);

    wire [6:0] w7;
    and c7_and1(w7[0], p[6], g[5]);
    and c7_and2(w7[1], p[6], p[5], g[4]);
    and c7_and3(w7[2], p[6], p[5], p[4], g[3]);
    and c7_and4(w7[3], p[6], p[5], p[4], p[3], g[2]);
    and c7_and5(w7[4], p[6], p[5], p[4], p[3], p[2], g[1]);
    and c7_and6(w7[5], p[6], p[5], p[4], p[3], p[2], p[1], g[0]);
    and c7_and7(w7[6], p[6], p[5], p[4], p[3], p[2], p[1], p[0], c[0]);
    or c7_or(c[7], g[6], w7[0], w7[1], w7[2], w7[3], w7[4], w7[5], w7[6]);

    wire [7:0] w8;
    and c8_and1(w8[0], p[7], g[6]);
    and c8_and2(w8[1], p[7], p[6], g[5]);
    and c8_and3(w8[2], p[7], p[6], p[5], g[4]);
    and c8_and4(w8[3], p[7], p[6], p[5], p[4], g[3]);
    and c8_and5(w8[4], p[7], p[6], p[5], p[4], p[3], g[2]);
    and c8_and6(w8[5], p[7], p[6], p[5], p[4], p[3], p[2], g[1]);
    and c8_and7(w8[6], p[7], p[6], p[5], p[4], p[3], p[2], p[1], g[0]);
    and c8_and8(w8[7], p[7], p[6], p[5], p[4], p[3], p[2], p[1], p[0], c[0]);
    or c8_or(c[8], g[7], w8[0], w8[1], w8[2], w8[3], w8[4], w8[5], w8[6], w8[7]);

    // compute p and g out for next block
    and Pout(Po, p[7], p[6], p[5], p[4], p[3], p[2], p[1], p[0]);
    or Gout(Go, g[7], w8[0], w8[1], w8[2], w8[3], w8[4], w8[5], w8[6]);

endmodule