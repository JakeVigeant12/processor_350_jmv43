module control(pc_next_def, dx_pc_out, sx_imm, rd, dx_ir_out, alu_neq_out, alu_lt_out, pc_next, branch_or_jump_taken);
    input [31:0] pc_next_def, dx_pc_out, sx_imm, rd, dx_ir_out;
    input alu_neq_out, alu_lt_out;
    output branch_or_jump_taken;
    output [31:0] pc_next;

    wire [31:0] pc_branch;
    cla_full_adder pc_jump(dx_pc_out, sx_imm, 1'b0, pc_branch);

    wire [31:0] t;
    assign t[31:27] = 5'b0;
    assign t[26:0] = dx_ir_out[26:0];

    wire [4:0] dx_opcode;
    assign dx_opcode = dx_ir_out[31:27];

    wire dx_is_bex_op;
    assign dx_is_bex_op = dx_opcode[4] & ~dx_opcode[3] & dx_opcode[2] & dx_opcode[1] & ~dx_opcode[0];

    wire [2:0] mux_select;
    assign mux_select = dx_is_bex_op ? 31'b0 : dx_opcode[2:0];

    wire [31:0] pc_next_mux;
    //module mux_8(in0,in1,in2,in3,in4,in5,in6,in7, out, select);
    mux_8 next_pc(pc_next_def, t, alu_neq_out ? pc_branch : pc_next_def, t, rd, pc_next_def, alu_lt_out ? pc_branch : pc_next_def, pc_next_def, pc_next_mux, mux_select);

    assign pc_next = (dx_is_bex_op && rd != 0) ? t : pc_next_mux;

    assign branch_or_jump_taken = pc_next != pc_next_def;
    
endmodule