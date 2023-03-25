module stall(fd_ir, dx_ir, xm_ir, mdiv_running, mdiv_ready, stall_sel);
    input [31:0] fd_ir, dx_ir, xm_ir;
    input mdiv_running, mdiv_ready;
    output stall_sel;

    wire [4:0] fd_opcode, dx_opcode;
    assign fd_opcode = fd_ir[31:27];
    assign dx_opcode = dx_ir[31:27];

    //instruction kinds
    wire dx_lw, fd_sw, dx_rOp, dx_mult, dx_div;
    assign dx_lw = (dx_opcode === 5'b01000);
    assign fd_sw = (fd_opcode === 5'b00111);
    assign dx_rOp = (dx_opcode === 5'b00000);
    assign aluOp = dx_ir[6:2];
    assign dx_mult = dx_rOp & (aluOp === 5'b00110);
    assign dx_div = dx_rOp & (aluOp === 5'b00111);

    wire [4:0] fd_rs, fd_rt, dx_rd, aluOp;
    //get registers
    assign dx_rd = dx_ir[26:22];
    assign fd_rs = fd_ir[21:17];
    assign fd_rt = fd_ir[16:12];


    // stall logic
    assign stall_sel = (dx_lw && ((fd_rs == dx_rd) || ((fd_rt == dx_rd) && !fd_sw))) || dx_mult || dx_div || mdiv_running;
endmodule