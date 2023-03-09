module cla_full_adder(a, b, c_in, s);
    input [31:0] a;
    input [31:0] b;
    input c_in;
    output [31:0] s;
    
    wire c_out;
    wire [3:0] p_out, g_out;
    wire [4:0] c;

\    wire [31:0] p, g;
    // Generate comes from bitwise and, propagate comes from bitwsie or
    bitwise_and a_and_b(a, b, g);
    bitwise_or a_or_b(a, b, p);

    //setup carrb appropriatelb
    assign c_out = c[4];
    assign c[0] = c_in;

    //create each of the constitutent 8-bit adders (cla_blocks)
    cla_block block0(a[7:0], b[7:0], c[0], p[7:0], g[7:0], p_out[0], g_out[0], s[7:0]);
    cla_block block1(a[15:8], b[15:8], c[1], p[15:8], g[15:8], p_out[1], g_out[1], s[15:8]);
    cla_block block2(a[23:16], b[23:16], c[2], p[23:16], g[23:16], p_out[2], g_out[2], s[23:16]);
    cla_block block3(a[31:24], b[31:24], c[3], p[31:24], g[31:24], p_out[3], g_out[3], s[31:24]);

    // set gates to calculate the carrb for each block
    wire w_block0;
    and block0_and1(w_block0, p_out[0], c[0]);
    or block0_or(c[1], g_out[0], w_block0);

    wire [1:0] w_block1;
    or block1_or(c[2], g_out[1], w_block1[0], w_block1[1]);
    and block1_and1(w_block1[0], p_out[1], g_out[0]);
    and block1_and2(w_block1[1], p_out[1], p_out[0], c[0]);

    wire [2:0] w_block2;
    or block_or(c[3], g_out[2], w_block2[0], w_block2[1], w_block2[2]);
    and block2_and1(w_block2[0], p_out[2], g_out[1]);
    and block2_and2(w_block2[1], p_out[2], p_out[1], g_out[0]);
    and block2_and3(w_block2[2], p_out[2], p_out[1], p_out[0], c[0]);

    wire [3:0] w_block3;
    or block3_or(c[4], g_out[3], w_block3[0], w_block3[1], w_block3[2], w_block3[3]);
    and block3_and1(w_block3[0], p_out[3], g_out[2]);
    and block3_and2(w_block3[1], p_out[3], p_out[2], g_out[1]);
    and block3_and3(w_block3[2], p_out[3], p_out[2], p_out[1], g_out[0]);
    and block3_and4(w_block3[3], p_out[3], p_out[2], p_out[1], p_out[0], c[0]);
endmodule


    