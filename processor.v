/**
 * READ THIS DESCRIPTION!
 *
 * This is your processor module that will contain the bulk of your code submission. You are to implement
 * a 5-stage pipelined processor in this module, accounting for hazards and implementing bypasses as
 * necessary.
 *
 * Ultimately, your processor will be tested by a master skeleton, so the
 * testbench can see which controls signal you active when. Therefore, there needs to be a way to
 * "inject" imem, dmem, and regfile interfaces from some external controller module. The skeleton
 * file, Wrapper.v, acts as a small wrapper around your processor for this purpose. Refer to Wrapper.v
 * for more details.
 *
 * As a result, this module will NOT contain the RegFile nor the memory modules. Study the inputs 
 * very carefully - the RegFile-related I/Os are merely signals to be sent to the RegFile instantiated
 * in your Wrapper module. This is the same for your memory elements. 
 *
 *
 */
module processor(
    // Control signals
    clock,                          // I: The master clock
    reset,                          // I: A reset signal

    // Imem
    address_imem,                   // O: The address of the data to get from imem
    q_imem,                         // I: The data from imem

    // Dmem
    address_dmem,                   // O: The address of the data to get or put from/to dmem
    data,                           // O: The data to write to dmem
    wren,                           // O: Write enable for dmem
    q_dmem,                         // I: The data from dmem

    // Regfile
    ctrl_writeEnable,               // O: Write enable for RegFile
    ctrl_writeReg,                  // O: Register to write to in RegFile
    ctrl_readRegA,                  // O: Register to read from port A of RegFile
    ctrl_readRegB,                  // O: Register to read from port B of RegFile
    data_writeReg,                  // O: Data to write to for RegFile
    data_readRegA,                  // I: Data from port A of RegFile
    data_readRegB                   // I: Data from port B of RegFile
	 
	);

	// Control signals
	input clock, reset;
	
	// Imem
    output [31:0] address_imem;
	input [31:0] q_imem;

	// Dmem
	output [31:0] address_dmem, data;
	output wren;
	input [31:0] q_dmem;

	// Regfile
	output ctrl_writeEnable;
	output [4:0] ctrl_writeReg, ctrl_readRegA, ctrl_readRegB;
	output [31:0] data_writeReg;
	input [31:0] data_readRegA, data_readRegB;

    /* YOUR CODE STARTS HERE */


//PC
    wire [31:0] pc, pcAdv, pc_next, fd_pc_out, fd_ins_curr;
    //hold current pc
    pc_reg pcReg(clock, reset, ~stall, pc_next, pc); 
    assign address_imem = pc; 

    //module cla_full_adder(a, b, c_in, s);
    //next pc line out
    cla_full_adder inc_pc(pc, 1, 0, pcAdv); 
    //next pc fetched with refactored out control logic
    wire imemOpcode, isImemJump, pcNextActual;
    assign imemOpcode = q_imem[31:27];
    assign isImemJump = (imemOpcode == 5'b00001) | (imemOpcode == 5'b00011) === 1'b1;
    assign pcNextActual = isImemJump ? q_imem[26:0] : (is_dx_jr ? data_readRegB : pcAdv);

//FD
    wire doNop;
    assign doNop = bj;

    wire[31:0] nop;
    assign nop = 32'b0;
    fd_latch fd(clock, ~stall, pc, doNop ? nop : q_imem, fd_pc_out, fd_ins_curr);

  
    wire [4:0] fd_opcode;
    assign fd_opcode = fd_ins_curr[31:27];

    wire fd_rOp, fd_bex;
    assign fd_rOp = (fd_opcode === 5'b00000);
    assign fd_bex = (fd_opcode === 5'b10110);

    assign ctrl_readRegA = fd_ins_curr[21:17];
    assign ctrl_readRegB = fd_rOp ? fd_ins_curr[16:12] : (!fd_bex ? fd_ins_curr[26:22] : 5'd30);

//DX
    wire [31:0] dx_ins_in, dx_pc_out, dx_a_out, dx_b_out, dx_ins_out;

    dx_latch dx(clock, fd_pc_out, data_readRegA, data_readRegB, bj ? 32'b0 : (stall ? 32'b0 : fd_ins_curr), dx_pc_out, dx_a_out, dx_b_out, dx_ins_out);

    wire [4:0] dx_opcode;
    assign dx_opcode = dx_ins_out[31:27];


    wire [1:0] mux_a_select, mux_b_select;
    wire [31:0] xm_o_out;
    
    wire is_dx_jr;
    assign is_dx_jr = (dx_opcode === 5'b00100);
    //Configure ALU Inputs
    wire [31:0] inp_A, inp_B, binp_mux_out;
    //module mux_4(in0, in1, in2, in3, out, sel);
    mux_4 inpAMux( xm_o_out, data_writeReg, dx_a_out, 32'b0, inp_A, mux_a_select);
    mux_4 inpBmux(xm_o_out, data_writeReg, dx_b_out, 32'b0, binp_mux_out, mux_b_select);

    // sign extend imm
    wire [31:0] imm;
    assign imm[16:0] = dx_ins_out[16:0];
    assign imm[31:17] = dx_ins_out[16] ? 15'b111111111111111 : 15'b0;

    wire dx_brOp, dx_rOp;
    assign dx_brOp = (~dx_opcode[4] & ~dx_opcode[3] & ~dx_opcode[2] & dx_opcode[1] & ~dx_opcode[0]) | 
    (~dx_opcode[4] & ~dx_opcode[3] & dx_opcode[2] & dx_opcode[1] & ~dx_opcode[0]);

    assign dx_rOp = (dx_opcode === 5'b00000);

    wire [4:0] alu_opcode, shamt;
    assign alu_opcode = dx_rOp ? dx_ins_out[6:2] : (dx_brOp ? 5'b1 : 5'b0);
    assign shamt = dx_rOp ? dx_ins_out[11:7] : 5'b0;
    assign inp_B = (dx_rOp || dx_brOp) ? binp_mux_out : imm;

    //alu unit
    wire [31:0] alu_out, alu_out_ovf;
    wire notEq, lessthn, alu_overflow;
    alu ula(inp_A, inp_B, alu_opcode, shamt, alu_out, notEq, lessthn, alu_overflow);

    //mdiv coproccessor
    wire isMult, isDiv;
    assign isMult = dx_rOp & ~alu_opcode[4] & ~alu_opcode[3] & alu_opcode[2] & alu_opcode[1] & ~alu_opcode[0];
    assign isDiv = dx_rOp & ~alu_opcode[4] & ~alu_opcode[3] & alu_opcode[2] & alu_opcode[1] & alu_opcode[0];

    wire [31:0] mdiv_inpA, mdiv_inpB, mdiv_cIns, mdiv_res;
    wire mdiv_runnin, mdiv_exc, mdiv_done;

    muldiv_latch mdiv(clock, isMult | isDiv, mdiv_runnin, mdiv_done, inp_A, inp_B, dx_ins_out, mdiv_inpA, mdiv_inpB, mdiv_cIns);
    multdiv multdiv_unit(mdiv_inpA, mdiv_inpB, isMult, isDiv, clock, mdiv_res, mdiv_exc, mdiv_done);

    wire overflow, dx_jal, dx_setx;
    assign overflow = alu_overflow | (mdiv_exc & mdiv_done);

    assign dx_jal = (dx_opcode === 5'b00011);
    assign dx_setx = (dx_opcode === 5'b10101);

    //handle differing xm o inputs w tristate, simplified ternary app.
    wire [31:0] xm_o_in;
    //module tri_state_buffer(out, inp, enable);
    tri_state_buffer tri_alu(xm_o_in, alu_out, !overflow && !dx_jal && !dx_setx);
    tri_state_buffer tri_ovf(xm_o_in, rstatWrite, overflow && !dx_jal && !dx_setx);
    tri_state_buffer tri_jal(xm_o_in, dx_pc_out, !overflow && dx_jal && !dx_setx);

  
    tri_state_buffer tri_setx(xm_o_in, dx_ins_out[26:0], !overflow && !dx_jal && dx_setx);

// XM latch
    wire [31:0] xm_b_out, xm_ir_out;
    wire xm_ovf_out;
    xm_latch xm(clock, xm_o_in, overflow, binp_mux_out, dx_ins_out, xm_o_out, xm_ovf_out, xm_b_out, xm_ir_out);
	
    //Read/write dmem
    wire [4:0] xm_opcode;
    assign xm_opcode = xm_ir_out[31:27];
    assign wren = xm_sw;

    wire xm_sw, xm_bex, wm_sel;
    assign xm_sw = (xm_opcode === 5'b00111);

    assign address_dmem = xm_o_out;
    assign data = wm_sel ? data_writeReg : xm_b_out;

// MW latch
    wire [31:0] mw_o_out, mw_d_out, mw_ir_out;
    wire mw_ovf_out;
    mw_latch mw(clock, xm_o_out, xm_ovf_out, q_dmem, xm_ir_out, mw_o_out, mw_ovf_out, mw_d_out, mw_ir_out);

    // regfile writeback stage
    wire [4:0] mw_opcode;
    assign mw_opcode = mw_ir_out[31:27];
    wire mw_rOp, mw_addi, mw_lw, mw_sw, mw_jal, mw_bex, mw_setx;
    assign mw_rOp = (mw_opcode === 5'b00000);
    assign mw_addi = (mw_opcode === 5'b00101);
    assign mw_lw = (mw_opcode === 5'b01000);
    assign mw_jal = (mw_opcode === 5'b00011);
    assign mw_setx = (mw_opcode === 5'b10101);


    //choose writedata
    tri_state_buffer_5 basecase(mw_ir_out[26:22], !(mw_ovf_out || mw_setx) && !mw_jal && !(mdiv_done && !mdiv_exc), ctrl_writeReg);
    tri_state_buffer_5 jal(5'd31, !(mw_ovf_out || mw_setx) && mw_jal && !(mdiv_done && !mdiv_exc), ctrl_writeReg);
    tri_state_buffer_5 mdivWrite(mdiv_cIns[26:22], !(mw_ovf_out || mw_setx) && !mw_jal && (mdiv_done && !mdiv_exc), ctrl_writeReg);
    tri_state_buffer_5 statusWrite(5'd30, (mw_ovf_out || mw_setx) && !mw_jal && !(mdiv_done && !mdiv_exc), ctrl_writeReg);

    // select logic for write reg
    assign data_writeReg = mw_lw ? mw_d_out : (!(mdiv_done && !mdiv_exc) ?  mw_o_out : mdiv_res);
    assign ctrl_writeEnable = mw_rOp |  mw_jal | mw_addi | mw_lw | mw_setx | (mdiv_done && !mdiv_exc);

    //refactored control signals
    wire [31:0] rstatWrite;
    wire stall, bj;
    control controll(pcAdv, dx_pc_out, imm, binp_mux_out, dx_ins_out, notEq, notEq ? ~lessthn : 1'b0, pc_next, bj);
    stall stalll(fd_ins_curr, dx_ins_out, xm_ir_out, mdiv_runnin, mdiv_done, stall);
    bypass bypasss(dx_ins_out, xm_ir_out, mw_ir_out, xm_ovf_out, mw_ovf_out, mux_a_select, mux_b_select, wm_sel);
    handleRstat pickVal(rstatWrite, mdiv_done ? mdiv_cIns[31:27] : dx_opcode, mdiv_done ? mdiv_cIns[6:2] : alu_opcode);

endmodule
