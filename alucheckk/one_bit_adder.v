module one_bit_adder(a, b, cin, s);
    input a, b, cin;
    output s;
    //compute sum
    xor sumGen(s, a, b, cin);
endmodule