module eightbitlookahead(S,A,B,Cout, cin,P);
  input [7:0]A;
  input [7:0]B;
  input cin;
  output [7:0]S;
  output Cout;
  output P;
  wire p0,g0,p0c0,carrybit1,p1,g1,p1g0,p1p0c0,carrybit2;
  wire p2,g2,p2g1,p2p1g0,p2p1p0c0,carrybit3;
  wire p3,g3,p3g2,p3p2g1,p3p2p1g0,p3p2p1p0c0,carrybit4;
  wire carrybit5,p4g3,p4p3g2,p4p3p2g1, p4p3p2p1g0,p4p3p2p1p0c0, p4,g4;
  wire carrybit6,g5,p5g4,p5p4g3,p5p4p3g2,p5p4p3p2g1,p5p4p3p2p1g0,p5p4p3p2p1p0c0,p5;
  wire carrybit7,g6,p6g5,p6p5g4,p6p5p4g3,p6p5p4p3g2,p6p5p4p3p2g1,p6p5p4p3p2p1g0,p6p5p4p3p2p1p0c0,p6;
  wire g7,p7g6,p7p6g5,p7p6p5g4,p7p6p5p4g3,p7p6p5p4p3p2g1,p7p6p5p4p3g2,p7p6p5p4p3p2p1g0,p7p6p5p4p3p2p1p0c0,p7;
  wire a1,a2,a3,a4,a5,a6,a7,a8;
  //first carrybit
  or o1(p0,A[0],B[0]); // p0
  and a109e(p0c0,p0,cin); //p0c0
  and a24e8(g0,A[0],B[0]); //g0
  or o2(carrybit1,g0,p0c0);

  //second
  or o3(p1,A[1],B[1]);
  and a384(g1,A[1],B[1]);
  and a48948(p1g0,p1,g0);
  and a5765(p1p0c0,p1,p0,cin);
  or o4(carrybit2,g1, p1g0,p1p0c0);

  //third
  or o5(p2,A[2],B[2]);
  and a6779(g2,A[2],B[2]);
  and a7976(p2g1,p2,g1);
  and a8777(p2p1g0,p2,p1,g0);
  and a9(p2p1p0c0,p2,p1,p0,cin);
  or o6(carrybit3,g2,p2g1, p2p1g0,p2p1p0c0);

  //fourth
  or o7(p3,A[3],B[3]);
  and a10(g3,A[3],B[3]);
  and a11(p3g2,p3,g2);
  and a12(p3p2g1,p3,p2,g1);
  and a13(p3p2p1g0,p3,p2,p1,g0);
  and a14(p3p2p1p0c0,p3,p2,p1,p0,cin);
  or o8(carrybit4,g3,p3g2,p3p2g1, p3p2p1g0,p3p2p1p0c0);


  //fifth
  or o9(p4,A[4],B[4]);
  and a15(g4,A[4],B[4]);
  and a16(p4g3,p4,g3);
  and a17(p4p3g2,p4,p3,g2);
  and a18(p4p3p2g1,p4,p3,p2,g1);
  and a19(p4p3p2p1g0,p4,p3,p2,p1,g0);
  and a20(p4p3p2p1p0c0,p4,p3,p2,p1,p0,cin);
  or o10(carrybit5,g4,p4g3,p4p3g2,p4p3p2g1, p4p3p2p1g0,p4p3p2p1p0c0);

  //sixth
  or o11(p5,A[5],B[5]);
  and a21(g5,A[5],B[5]);
  and a22(p5g4,p5,g4);
  and a23(p5p4g3,p5,p4,g3);
  and a24(p5p4p3g2,p5,p4,p3,g2);
  and a25(p5p4p3p2g1,p5,p4,p3,p2,g1);
  and a26(p5p4p3p2p1g0,p5,p4,p3,p2,p1,g0);
  and a27(p5p4p3p2p1p0c0,p5,p4,p3,p2,p1,p0,cin);
  or o12(carrybit6,g5,p5g4,p5p4g3,p5p4p3g2,p5p4p3p2g1,p5p4p3p2p1g0,p5p4p3p2p1p0c0);

  //seventh
  or o13(p6,A[6],B[6]);
  and a31(g6,A[6],B[6]);
  and a81(p6g5,p6,g5);
  and a82(p6p5g4,p6,p5,g4);
  and a83(p6p5p4g3,p6,p5,p4,g3);
  and a84(p6p5p4p3g2,p6,p5,p4,p3,g2);
  and a85(p6p5p4p3p2g1,p6,p5,p4,p3,p2,g1);
  and a46(p6p5p4p3p2p1g0,p6,p5,p4,p3,p2,p1,g0);
  and a57(p6p5p4p3p2p1p0c0,p6,p5,p4,p3,p2,p1,p0,cin);
  or o14(carrybit7,g6,p6g5,p6p5g4,p6p5p4g3,p6p5p4p3g2,p6p5p4p3p2g1,p6p5p4p3p2p1g0,p6p5p4p3p2p1p0c0);


  //eighth
  or o15(p7,A[7],B[7]);
  and a39(g7,A[7],B[7]);
  and a866(p7g6,p7,g6);
  and a867(p7p6g5,p7,p6,g5);
  and a868(p7p6p5g4,p7,p6,p5,g4);
  and a861(p7p6p5p4g3,p7,p6,p5,p4,g3);
  and a853(p7p6p5p4p3g2,p7,p6,p5,p4,p3,g2);
  and a834(p7p6p5p4p3p2g1,p7,p6,p5,p4,p3,p2,g1);
  and a434(p7p6p5p4p3p2p1g0,p7,p6,p5,p4,p3,p2,p1,g0);
  //and a556(p7p6p5p4p3p2p1p0c0,p7,p6,p5,p4,p3,p2,p1,p0,cin);
  or o16(Cout,g7,p7g6,p7p6g5,p7p6p5g4,p7p6p5p4g3,p7p6p5p4p3g2,p7p6p5p4p3p2g1,p7p6p5p4p3p2p1g0);

  xor bit1(S[0],A[0],B[0],cin);
  xor bit2(S[1],A[1],B[1],carrybit1);
  xor bit3(S[2],A[2],B[2],carrybit2);
  xor bit4(S[3],A[3],B[3],carrybit3);
  xor bit5(S[4],A[4],B[4],carrybit4);
  xor bit6(S[5],A[5],B[5],carrybit5);
  xor bit7(S[6],A[6],B[6],carrybit6);
  xor bit8(S[7],A[7],B[7],carrybit7);

  and a101(P,p0,p1,p2,p3,p4,p5,p6,p7);

endmodule
