module ctrl_latch(s, r, out);
    input s, r;

    output out;

    wire qA, qB;

    nor nor1(qA, r, qB);
    nor nor2(qB, s, qA);

    assign out = qA;
endmodule