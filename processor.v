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



// Refactored out units
    wire isStall, brOrJu, wmSel;
    control cntrl(pc_adv, dx_pc_curr, imm, alu_b_choice, dx_curr_ir, neq, neq ? ~lthn : 1'b0, pc_chosen, brOrJu);
    stall stl(fd_ir_curr, dx_curr_ir, xm_curr_ir, multdiv_is_running, mdiv_ready, isStall);
    bypass byp(dx_curr_ir, xm_curr_ir, mw_curr_ir, xm_ovf_curr, mw_ovf_out, mux_inpA_sel, mux_inpB_sel, wmSel);

//PC
    wire [31:0] pc_curr, pc_adv, pc_chosen, fd_pc_curr, fd_ir_curr;
    pc_reg pc(clock, reset, ~isStall, pc_chosen, pc_curr); 
    cla_full_adder pcadvan(pc_curr, 32'b1, 1'b0, pc_adv);

    //access current ir
    assign address_imem = pc_curr;

    
//FD
    fd_latch fd(clock, ~isStall, pc_curr, brOrJu ? 32'b0 : q_imem, fd_pc_curr, fd_ir_curr);

    wire [4:0] fd_opcode;
    assign fd_opcode = fd_ir_curr[31:27];

    //ir info
    wire fdROp, fdBex;
    assign fdROp = (fd_opcode === 5'b00000);
    //if bex, need to access r30 to check
    assign fdBex = (fd_opcode === 5'b10110);

    //Acces rfile
    assign ctrl_readRegA = fd_ir_curr[21:17];
    assign ctrl_readRegB = fdROp ? fd_ir_curr[16:12] : (!fdBex  ? fd_ir_curr[26:22] : 5'd30);

//DX
    wire [31:0] dx_nex_ir, dx_pc_curr, dx_a_curr, dx_b_curr, dx_curr_ir;
    wire [4:0] dx_opcode;

    assign dx_opcode = dx_curr_ir[31:27];
    assign dx_nex_ir = isStall ? 32'b0 : fd_ir_curr;
    dx_latch dx(clock, fd_pc_curr, data_readRegA, data_readRegB, brOrJu ? 32'b0 : dx_nex_ir, dx_pc_curr, dx_a_curr, dx_b_curr, dx_curr_ir);


    wire [1:0] mux_inpA_sel, mux_inpB_sel;
    wire [31:0] xm_o_out;
    
    //prepare imm
    wire [31:0] imm;
    assign imm[16:0] = dx_curr_ir[16:0];
    assign imm[31:17] = dx_curr_ir[16] ? 111111111111111 : 15'b0;


    //Choose alu inputs
    wire [31:0] alu_inpA, alu_inpB, alu_b_choice;
    mux_4 inpAMux(xm_o_out, data_writeReg, dx_a_curr, 0, alu_inpA, mux_inpA_sel);
    mux_4 inpBMux(xm_o_out, data_writeReg, dx_b_curr, 0, alu_b_choice, mux_inpB_sel);

    //prepare ALU inputs
    wire [4:0] alu_opcode, shamt;
    assign alu_opcode = dxROp ? dx_curr_ir[6:2] : (dxBrnch ? 5'b1 : 5'b0);
    assign shamt = dxROp ? dx_curr_ir[11:7] : 5'b0;
    assign alu_inpB = (dxROp || dxBrnch) ? alu_b_choice : imm;

    //Prepare ins info
    wire dxBrnch, dxROp, neq, lthn, alu_overflow, dxJal, dxSetx;
    assign dxROp = (dx_opcode === 5'b00000);
    //blt bne
    assign dxBrnch = (dx_opcode === 5'b00010) | (dx_opcode === 5'b00110);
    assign dxJal = (dx_opcode === 5'b00011);
    assign dxSetx = (dx_opcode === 5'b10101);

    // Outputs of ALU and ALU unit itself
    wire [31:0] alu_out;
    alu alu_unit(alu_inpA, alu_inpB, alu_opcode, shamt, alu_out, neq, lthn, alu_overflow);

    // determine mdiv controls
    wire ctrl_mult, ctrl_div;
    assign ctrl_mult = dxROp & (alu_opcode === 5'b00110);
    assign ctrl_div = dxROp & (alu_opcode === 5'b00111);

    wire [31:0] muldiv_inpA, muldiv_inpB, multdiv_ir, mdiv_res;
    wire multdiv_is_running, mdiv_exc, mdiv_ready;

    multdiv_input_latch mdivInpsLatch(clock, ctrl_mult | ctrl_div, multdiv_is_running, mdiv_ready, alu_inpA, alu_inpB, dx_curr_ir, muldiv_inpA, muldiv_inpB, multdiv_ir);
    multdiv multdiv_unit(muldiv_inpA, muldiv_inpB, ctrl_mult, ctrl_div, clock, mdiv_res, mdiv_exc, mdiv_ready);

    //Capture overflow info
    wire overflow, didMdivOvf;
    assign didMdivOvf = mdiv_exc & mdiv_ready;
    assign overflow = alu_overflow | didMdivOvf;

    wire [31:0] rstatus;
    overflow ovfhandle(mdiv_ready ? multdiv_ir[31:27] : dx_opcode, mdiv_ready ? multdiv_ir[6:2] : alu_opcode, rstatus);

   
    //select between alu, rstatExcep, currPc for xm latch input
    wire [31:0] xm_o_in;
    wire [2:0] xm_o_in_sel;

    assign xm_o_in_sel[0] = ovf;
    assign xm_o_in_sel[1] = dxJal;
    assign xm_o_in_sel[2] = dxSetx;
    mux_8 xmoin(alu_out, 0, dx_pc_curr, 0, rstatus, 0, 0, 0, xm_o_in, xm_o_in_sel);


    wire [31:0] t;
    assign t[31:27] = 5'b0;
    assign t[26:0] = dx_curr_ir[26:0];
    tri_state_buffer tri_setx(xm_o_in, t, !overflow && !dxJal && dxSetx);

//XM
    wire [31:0] xm_b_curr, xm_curr_ir;
    wire xm_ovf_curr;
    xm_latch xm(clock, xm_o_in, overflow, alu_b_choice, dx_curr_ir, xm_o_out, xm_ovf_curr, xm_b_curr, xm_curr_ir);
	
    // Data memory
     wire xmSw, xmBex;
    wire [4:0] xm_opcode;
    assign xm_opcode = xm_curr_ir[31:27];
    assign xmSw = (xm_opcode === 5'b00111);
    //allow dmem write
    assign wren = xmSw;

    assign address_dmem = xm_o_out;
    assign data = wmSel ? data_writeReg : xm_b_curr;

// MW
    wire [31:0] mw_o_curr, mw_d_curr, mw_curr_ir;
    wire mw_ovf_out;
    mw_latch mw(clock, xm_o_out, xm_ovf_curr, q_dmem, xm_curr_ir, mw_o_curr, mw_ovf_out, mw_d_curr, mw_curr_ir);

    // Writing back to Regfile
    wire [4:0] mw_opcode;
    assign mw_opcode = mw_curr_ir[31:27];
    wire mwRop, mwAddi, mwLw, mwSw, mwJal, mwBex, mwSetx;
    assign mwRop = (mw_opcode === 5'b00000);
    assign mwAddi = (mw_opcode === 5'b00101);
    assign mwLw = (mw_opcode === 5'b01000);
    assign mwJal = (mw_opcode === 5'b00011);
    assign mwSetx = (mw_opcode === 5'b10101);

    //select proper register to writeback
    wire isExc, mdivisWrite;
    assign isExc = mw_ovf_out || mwSetx;
    assign mdivisWrite = mdiv_ready && !mdiv_exc;
 
    tri_buffer_rstat jalW(5'd31, !(isExc) && mwJal && !(mdivisWrite), ctrl_writeReg);
    tri_buffer_rstat normW(mw_curr_ir[26:22], !(isExc) && !mwJal && !(mdivisWrite), ctrl_writeReg);
    //force reg 30
    tri_buffer_rstat exceptW(5'd30, (isExc) && !mwJal && !(mdivisWrite), ctrl_writeReg);
    tri_buffer_rstat mdivW(multdiv_ir[26:22], !(isExc) && !mwJal && (mdivisWrite), ctrl_writeReg);

    //Assign data back
    assign data_writeReg = mwLw ? mw_d_curr : ((mdivisWrite) ? mdiv_res : mw_o_curr);
    assign ctrl_writeEnable = mwRop | mwAddi | mwLw | mwJal | mwSetx | (mdivisWrite);




endmodule
