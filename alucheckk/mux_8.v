module mux_8(in0,in1,in2,in3,in4,in5,in6,in7, out, select);
    input [2:0] select;
    input [31:0] in0, in1, in2, in3,in4,in5,in6,in7;
    output [31:0] out;
    wire [31:0] pick1, pick2;


    //module mux_4(in0, in1, in2, in3, out, select);
    mux_4 layer1_1(in0, in1, in2, in3, pick1, select[1:0]);
    mux_4 layer1_2(in4, in5, in6, in7, pick2, select[1:0]);
    mux_2 finalSelect(out, select[2], pick1,pick2);
endmodule