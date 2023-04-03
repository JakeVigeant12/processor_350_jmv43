module control(advanced_pc, dx_pc, imm, rd, dx_ir, neq, lt, pc_out_cont, BorJ);
    input [31:0] advanced_pc, dx_pc, imm, rd, dx_ir;
    input neq, lt;
    output BorJ;
    output [31:0] pc_out_cont;

    wire [31:0] target;
    assign target[31:27] = 5'b0;
    assign target[26:0] = dx_ir[26:0];


    wire [31:0] pc_branch, advanced_pc_mux;
    cla_full_adder pc_jump(dx_pc, imm, 1'b0, pc_branch);


    wire [4:0] dx_opcode;
    assign dx_opcode = dx_ir[31:27];

    wire dx_bex;
    assign dx_bex = (dx_opcode === 5'b10110);

    wire [2:0] mux_select;
    assign mux_select = !dx_bex ? dx_opcode[2:0] : 3'b0;

    wire [31:0] neqSelect, ltSel, doBex;
    assign neqSelect = neq ? pc_branch : advanced_pc;
    assign ltSel = lt ? pc_branch : advanced_pc;
    assign dobex = (dx_bex && rd != 0);

    mux_8 pcSelection(advanced_pc, target, neqSelect, target, rd, advanced_pc, ltSel, advanced_pc, advanced_pc_mux, mux_select);

    assign pc_out_cont =  dobex ? target : advanced_pc_mux;
    //track branches or jumps on wire for latches in proc
    assign BorJ = pc_out_cont != advanced_pc;
    
endmodule
