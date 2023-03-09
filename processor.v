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
    wire [31:0] pc, pcAdv, pc_next, fd_out, fd_ir_out;
    pc_reg pcC(clock, reset, 1'b1, pc_next, pc); 
    assign address_imem = pc; 
    cla_full_adder pcAdder(pc, 32'b1, 1'b0, pcAdv); 

    //FD stage
    fd_latch fd(clock, 1'b1, pc, q_imem, fd_out, fd_ir_out);

    wire [4:0] fd_opcode;
    assign fd_opcode = fd_ir_out[31:27];

    wire fd_isR;
    assign fd_isR = ~fd_opcode[4] & ~fd_opcode[3] & ~fd_opcode[2] & ~fd_opcode[1] & ~fd_opcode[0];


    assign ctrl_readRegA = fd_ir_out[21:17];
    assign ctrl_readRegB = fd_isR ? fd_ir_out[16:12] : 5'b0;

    //DX stage
    assign dx_ir_in = fd_ir_out;
    wire [31:0] dx_ir_in, dx_pcOut, dx_a_curr, dx_b_curr, dx_ir_out;
    dx_latch dx(clock, fd_out, data_readRegA, data_readRegB, dx_ir_in, dx_pcOut, dx_a_curr, dx_b_curr, dx_ir_out);

    // get operation
    wire [4:0] dx_opcode;
    assign dx_opcode = dx_ir_out[31:27];

    // XM stage
    wire [1:0] mux_b;
    wire [31:0] xm_o_curr;

    wire [31:0] alu_data_a, alu_data_b;


    wire [31:0] alu_b_mux_out;
    mux_4 alumux1(xm_o_curr, data_writeReg, dx_b_curr, 32'b0, alu_b_mux_out, mux_b);

    wire [31:0] imm;
    assign imm[16:0] = dx_ir_out[16:0];
    //sign extend the imm
    assign imm[31:17] = dx_ir_out[16] ? 15'b1 : 15'b0;


    wire dx_rOp;
    assign dx_rOp = ~dx_opcode[4] & ~dx_opcode[3] & ~dx_opcode[2] & ~dx_opcode[1] & ~dx_opcode[0];
   

    // Configure ALU opcode, inputs,and shift amount
    wire [4:0] alu_opcode, shift_amount;
    assign alu_opcode = dx_ir_out[6:2];
    assign shift_amount = dx_ir_out[11:7];
    assign alu_data_b = dx_rOp ? alu_b_mux_out : imm;

    // Outputs of ALU and ALU unit itself
    wire [31:0] alu_out, alu_out_ovf;
    wire is_not_equal, is_less_than, alu_overflow;
    alu ula(alu_data_a, alu_data_b, alu_opcode, shift_amount, alu_out, is_not_equal, is_less_than, alu_overflow);

    wire overflow;
    assign overflow = alu_overflow;


    wire [31:0] xm_o_in;
    assign xm_o_in = alu_out;

    // XM latch
    wire [31:0] xm_b_curr, xm_ir_curr;
    wire xm_overfl;
    xm_latch xm(clock, xm_o_in, overflow, alu_b_mux_out, dx_ir_out, xm_o_curr, xm_overfl, xm_b_curr, xm_ir_curr);
	
   

    // MW latch
    wire [31:0] mw_o_out, mw_d_out, mw_ir_out;
    wire mw_ovf_out;
    mw_latch mw(clock, xm_o_curr, xm_overfl, q_dmem, xm_ir_curr, mw_o_out, mw_ovf_out, mw_d_out, mw_ir_out);

    //writeback step
    wire [4:0] mw_opcode;
    assign mw_opcode = mw_ir_out[31:27];
    wire mw_is_r_type_op, mw_is_addi_op, mw_is_lw_op, mw_is_sw_op, mw_is_jal_op, mw_is_bex_op, mw_is_setx_op;
    assign mw_is_r_type_op = ~mw_opcode[4] & ~mw_opcode[3] & ~mw_opcode[2] & ~mw_opcode[1] & ~mw_opcode[0];
    assign mw_is_addi_op = ~mw_opcode[4] & ~mw_opcode[3] & mw_opcode[2] & ~mw_opcode[1] & mw_opcode[0];
    assign mw_is_lw_op = ~mw_opcode[4] & mw_opcode[3] & ~mw_opcode[2] & ~mw_opcode[1] & ~mw_opcode[0];
    assign mw_is_jal_op = ~mw_opcode[4] & ~mw_opcode[3] & ~mw_opcode[2] & mw_opcode[1] & mw_opcode[0];
    assign mw_is_setx_op = mw_opcode[4] & ~mw_opcode[3] & mw_opcode[2] & ~mw_opcode[1] & mw_opcode[0];

    tri_state_buffer_5 normalCase(mw_ir_out[26:22], 1'b1, ctrl_writeReg);

    // Write the data to the relevant registers
    assign data_writeReg = mw_is_lw_op ? mw_d_out : mw_o_out;
    assign ctrl_writeEnable = mw_is_r_type_op | mw_is_addi_op | mw_is_lw_op | mw_is_jal_op | mw_is_setx_op;




	/* END CODE */

endmodule
