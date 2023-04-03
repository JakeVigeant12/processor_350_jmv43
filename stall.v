module stall(fd_ir_out, dx_ir_out, xm_ir_out, multdiv_is_running, multdiv_result_ready, select_stall);
    input [31:0] fd_ir_out, dx_ir_out, xm_ir_out;
    input multdiv_is_running, multdiv_result_ready;
    output select_stall;

    // Retrieve opcode from inputs
    wire [4:0] fd_opcode, dx_opcode;
    assign fd_opcode = fd_ir_out[31:27];
    assign dx_opcode = dx_ir_out[31:27];

    // Identify whether operation is load word or save word
    wire dx_is_lw_op, fd_is_sw_op;
    assign dx_is_lw_op = ~dx_opcode[4] & dx_opcode[3] & ~dx_opcode[2] & ~dx_opcode[1] & ~dx_opcode[0];
    assign fd_is_sw_op = ~fd_opcode[4] & ~fd_opcode[3] & fd_opcode[2] & fd_opcode[1] & fd_opcode[0];

    // Distinguish input values and opcodes as subsets of input
    wire [4:0] fd_rs, fd_rt, dx_rd;
    assign dx_rd = dx_ir_out[26:22];
    assign fd_rs = fd_ir_out[21:17];
    assign fd_rt = fd_ir_out[16:12];

    // Identify whether operation is type R
    wire dx_is_r_type_op;
    // Identify ALU opcode
    wire [4:0] alu_opcode;
    assign dx_is_r_type_op = ~dx_opcode[4] & ~dx_opcode[3] & ~dx_opcode[2] & ~dx_opcode[1] & ~dx_opcode[0];
    assign alu_opcode = dx_ir_out[6:2];

    // If R type operation, identify whether instruction is mult or div operation
    wire dx_is_mult_op, dx_is_div_op;
    assign dx_is_mult_op = dx_is_r_type_op & ~alu_opcode[4] & ~alu_opcode[3] & alu_opcode[2] & alu_opcode[1] & ~alu_opcode[0];
    assign dx_is_div_op = dx_is_r_type_op & ~alu_opcode[4] & ~alu_opcode[3] & alu_opcode[2] & alu_opcode[1] & alu_opcode[0];

    // If we know that (instruction in DX is mult) OR (instruction in DX is div) then stall
    // One multdiv operation follows another multdiv operation
    assign select_stall = (dx_is_lw_op && ((fd_rs == dx_rd) || ((fd_rt == dx_rd) && !fd_is_sw_op))) || multdiv_is_running || dx_is_mult_op || dx_is_div_op;
endmodule