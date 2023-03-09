module mux_4(in0, in1, in2, in3, out, sel);
    input [1:0] sel;
    input [31:0] in0, in1, in2, in3;
    output [31:0] out;
    wire [31:0] w1, w2;
    //module mux_2(out, select, in0, in1);
    mux_2 layer1_1(w1,sel[0],in0, in1);
    mux_2 layer1_2(w2,sel[0], in2, in3);
    mux_2 layer2(out, sel[1], w1, w2);
endmodule