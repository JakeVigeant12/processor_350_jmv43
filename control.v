module control(pc_next_def, dx_pc, imm, rd, dx_ir, neq, lt, pc_next, BorJ);
    input [31:0] pc_next_def, dx_pc, imm, rd, dx_ir;
    input neq, lt;
    output BorJ;
    output [31:0] pc_next;

    assign BorJ = pc_next != pc_next_def;
    //produced branched pc
    wire [31:0] immPc;
    cla_full_adder pcImm(dx_pc, imm, 1'b0, immPc);

    
    wire [4:0] dx_opcode;
    assign dx_opcode = dx_ir[31:27];

    wire dx_bex;
    assign dx_bex = (dx_opcode === 5'b10110);

    //pc mux
    wire [31:0] mux_pcOut;
    wire [2:0] pc_sel;
    assign pc_sel = !dx_bex ? dx_opcode[2:0] : 31'b0;
    //module mux_8(in0,in1,in2,in3,in4,in5,in6,in7, out, select);
    mux_8 next_pc(pc_next_def, dx_ir[26:0], neq ? immPc : pc_next_def, dx_ir[26:0], rd, pc_next_def, lt ? pc_branch : pc_next_def, pc_next_def, mux_pcOut, pc_sel);


    assign pc_next = (dx_bex && rd != 0) ? dx_ir[26:0] : mux_pcOut;

    
endmodule