module bypass(dx_ir, xm_ir, mw_ir, xm_ovf_out, mw_ovf_out, muxA_select, muxB_select, wmSelect);
    input [31:0] dx_ir, xm_ir, mw_ir;
    input xm_ovf_out, mw_ovf_out;
    output [1:0] muxA_select, muxB_select;
    output wmSelect;

    //Opcodes
    wire [4:0] dx_opcode, xm_opcode, mw_opcode;
    assign dx_opcode = dx_ir[31:27];
    assign xm_opcode = xm_ir[31:27];
    assign mw_opcode = mw_ir[31:27];

    //Handle 0s logic
    wire is_xm_rd_0, is_mw_rd_0;
    assign is_xm_rd_0 = (xm_rd == 5'b0);
    assign is_mw_rd_0 = (mw_rd == 5'b0);

    //Instruction types
    wire  is_mw_setx, is_xm_branch, is_mw_sw, is_mw_branch, is_xm_sw, is_dx_rOp, is_xm_setx, is_dx_bex;
    assign is_dx_rOp = (dx_opcode === 5'b00000);
    assign is_xm_sw = (xm_opcode === 5'b00111);
    assign is_mw_sw = (mw_opcode === 5'b00111);
    assign is_xm_setx = (xm_opcode === 5'b10101);
    assign is_mw_setx = (mw_opcode === 5'b10101);
    assign is_dx_bex = (dx_opcode === 5'b10110);
    assign is_mw_branch = (mw_opcode === 5'b00010) | (mw_opcode === 5'b00110);
    assign is_xm_branch = (xm_opcode === 5'b00010) | (xm_opcode === 5'b00110);

    //conditions for alu input muxes
    wire [4:0] dx_a, dx_b, xm_rd_ins, mw_rd_ins, xm_rd, mw_rd;
    assign dx_a = dx_ir[21:17]; 
    assign dx_b = is_dx_rOp ? dx_ir[16:12] : (is_dx_bex ? 5'd30 : dx_ir[26:22]);
    assign xm_rd_ins = xm_ir[26:22];
    assign mw_rd_ins = mw_ir[26:22];




    //Check data hazards
    wire xm_a_hz, mw_a_hz, xm_b_hz, mw_b_hz;
    assign xm_a_hz = !is_xm_sw && !is_xm_branch && dx_a == xm_rd && !(is_xm_rd_0); 
    assign mw_a_hz = !is_mw_sw && !is_mw_branch && dx_a == mw_rd && !(is_mw_rd_0); 
    assign xm_b_hz = !is_xm_sw && !is_xm_branch && dx_b == xm_rd && !(is_xm_rd_0); 
    assign mw_b_hz = !is_mw_sw && !is_mw_branch && dx_b == mw_rd && !(is_mw_rd_0);

    // ovf and setx reg30's , otherwise default to coded values
    assign xm_rd = !(xm_ovf_out || is_xm_setx) ? xm_rd_ins : 5'd30;
    assign mw_rd = !(mw_ovf_out || is_mw_setx) ? mw_rd_ins : 5'd30;

    //muxA selection condition
    assign muxA_select[0] = !xm_a_hz && mw_a_hz;
    assign muxA_select[1] = !xm_a_hz && !mw_a_hz;
    //muxB selection condition
    assign muxB_select[0] = !xm_b_hz && mw_b_hz;
    assign muxB_select[1] = !xm_b_hz && !mw_b_hz;


    // WM bypassing
    assign wmSelect = is_xm_sw && (xm_rd_ins == mw_rd_ins);

endmodule

