module adder_32(S,A,B,Cout, cin);
  input [31:0]A;
  input [31:0]B;
  input cin;
  output [31:0]S;
  output Cout;
  wire G0,P0,p0c0,c8,p0,G1,P1,p1g0,p1p0c0,p1,c16,G2,P2,p2g1,p2p1g0,p2p1p0c0,c24,G3,P3,p3g2,p3p2g1,p3p2p1p0c0,p3p2p1g0;
  //first carrybit
  eightbitlookahead adder1(S[7:0],A[7:0],B[7:0],G0,cin,P0);
  and and1(p0c0,P0,cin);
  or or1(c8,G0,p0c0);
  eightbitlookahead adder2(S[15:8],A[15:8],B[15:8],G1,c8,P1);
  and and2(p1g0,P1,G0);
  and and3(p1p0c0,P1,P0,cin);
  or or3(c16,G1,p1g0,p1p0c0);
  eightbitlookahead adder3(S[23:16],A[23:16],B[23:16],G2,c16,P2);
  and and4(p2g1,P2,G1);
  and and5(p2p1g0,P2,P1,G0);
  and and6(p2p1p0c0,P2,P1,P0,cin);
  or or5(c24,G2,p2g1,p2p1g0,p2p1p0c0);
  eightbitlookahead adder4(S[31:24],A[31:24],B[31:24],G3,c24,P3);
  and (p3g2,P2,G1);
  and and9(p3p2g1,P3,P2,G1);
  and and87(p3p2p1g0,P3,P2,P1,G0);
  and and56(p3p2p1p0c0,P3,P2,P1,P0,cin);
  or or54(Cout,G3,p3g2,p3p2g1,p3p2p1g0,p3p2p1p0c0);




endmodule
