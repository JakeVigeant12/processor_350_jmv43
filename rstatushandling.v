module rstatus_res(alu_op, op, rstatus_write_val);
    input[4:0] alu_op, op;
    output [31:0] rstatus_write_val;

    wire is_rOp;
    assign is_rOp = (op === 5'b00000);

    wire addi;
    assign addi = (op === 5'b00101);

    //If r determinne optype to select statusval
    wire add, sub, mult, div;
    assign add = is_rOp & (alu_op === 5'b00000);
    assign sub = is_rOp & (alu_op === 5'b00001);
    assign mult = is_rOp & (alu_op === 5'b00110);
    assign div = is_rOp & (alu_op === 5'b00111);

    //Select write output based on op
    //module mux_8(in0,in1,in2,in3,in4,in5,in6,in7, out, select);
    wire [2:0]mux_select;
    //If none write 0
    assign mux_select = addi ? 3'b000 : (add ? 3'b001 : (sub ? 3'b010 : (mult ? 3'b011 : (div? 3'b100 : 3'b101))));
    mux_8 rStatusVal(1,2,3,4,5,0,0,0,rstatus_write_val,mux_select);

endmodule

