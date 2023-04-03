module stall(fd_ir_out, dx_ir_out, xm_ir_out, multdiv_is_running, multdiv_result_ready, select_stall);
    input [31:0] fd_ir_out, dx_ir_out, xm_ir_out;
    input multdiv_is_running, multdiv_result_ready;
    output select_stall;

    wire [4:0] fd_opcode, dx_opcode;
    assign fd_opcode = fd_ir_out[31:27];
    assign dx_opcode = dx_ir_out[31:27];

    wire dxLw, fdSw, dxrROp;
    assign dxLw = (dx_opcode === 5'b01000);
    assign fdSw = (fd_opcode === 5'b00111);
    assign dxrROp = (dx_opcode === 5'b00000)

    wire [4:0] fd_rs, fd_rt, dx_rd;
    assign dx_rd = dx_ir_out[26:22];
    assign fd_rs = fd_ir_out[21:17];
    assign fd_rt = fd_ir_out[16:12];

    wire [4:0] alu_opcode;

    assign alu_opcode = dx_ir_out[6:2];

    wire dxMult, dxDiv;
    assign dxMult = dxrROp & ~alu_opcode[4] & ~alu_opcode[3] & alu_opcode[2] & alu_opcode[1] & ~alu_opcode[0];
    assign dxDiv = dxrROp & ~alu_opcode[4] & ~alu_opcode[3] & alu_opcode[2] & alu_opcode[1] & alu_opcode[0];

    // If we know that (instruction in DX is mult) OR (instruction in DX is div) then stall
    // One multdiv operation follows another multdiv operation
    assign select_stall = (dxLw && ((fd_rs == dx_rd) || ((fd_rt == dx_rd) && !fdSw))) || multdiv_is_running || dxMult || dxDiv;
endmodule