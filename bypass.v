// module bypass(dx_ir,xm_ir,mw_ir, mux_a_select, mux_b_select);

// //ALU A, some arithematic is on xm, lw is on dx
// input [31:0] xm_ir, dx_ir, mw_ir;
// output [1:0] mux_a_select;

// wire [4:0] dx_opcode, xm_opcode, mw_opcode;
// assign dx_opcode = dx_ir[31:27];
// assign xm_opcode = xm_ir[31:27];
// assign mw_opcode = mw_ir[31:27];

// //ALU A
// wire xm_is_r, dx_is_lw, mw_is_r;
// assign xm_is_r = (xm_opcode == 5'b00000);
// assign dx_is_lw = (dx_opcode == 5'b01000);
// assign mw_is_r = (mw_opcode == 5'b00000)

// //rs1 = rt
// wire [4:0]xm_rd, dx_rs1, mw_rd;
// assign dx_rs1 = dx_ir[16:12];
// assign xm_rd = xm_ir[26:22];
// assign mw_rd = mw_ir[26:22];


// // assign mux_a_select = (xm_is_r && dx_is_lw) ? ((dx_rs1 == xm_rd) ? 0 : ( () ?  : )) : 0;


// endmodule