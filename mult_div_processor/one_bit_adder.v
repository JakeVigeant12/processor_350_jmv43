module one_bit_adder(x, y, c_in, s);
    input x, y, c_in;
    output s;
    
    xor sum(s, x, y, c_in);

endmodule
