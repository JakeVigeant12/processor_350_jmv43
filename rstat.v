module handleRstat(rstatus, op, aluOp);
    input [4:0] op, aluOp;
    output [31:0] rstatus;

    wire addi, rOp, add, sub, mult, div;

    //i type ovf
    assign addi = (op === 5'b00101);
    
    //r type ovf
    assign rOp = (op === 5'b00000);

    //find r type math kind
    assign add = rOp & (aluOp === 5'b00000);
    assign sub = rOp & (aluOp === 5'b00001);
    assign mult = rOp & (aluOp === 5'b00110);
    assign div = rOp & (aluOp === 5'b00111);

    //write correct ovf to rstatus
    tri_state_buffer tri_add(rstatus, 32'd1, add);
    tri_state_buffer tri_addi(rstatus, 32'd2, addi);
    tri_state_buffer tri_sub(rstatus, 32'd3, sub);
    tri_state_buffer tri_mult(rstatus, 32'd4, mult);
    tri_state_buffer tri_div(rstatus, 32'd5, div);
endmodule