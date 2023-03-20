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
    wire [31:0] pcActive, pcAdv, fd_pc_out, fd_ir_out;
    //module pc_reg(clock, reset, in_enable, in, out);
    //load next pc into 
    pc_reg pc(clock, reset, 1'b1, pcAdv, pcActive); 
    assign address_imem = pcActive; 
    //module cla_full_adder(a, b, c_in, s);\
    cla_full_adder inc_pc(pcActive, 1, 1'b0, pcAdv); 

//FD stage
    //Disable enable toggle if time to stall later
    //module fd_latch(clk, enable, cPc, inIns, pcOut, insOut);
    fd_latch fd(clock, 1'b1, pcActive, q_imem, fd_pc_out, fd_ir_out);

    wire [4:0] fd_opcode;
    assign fd_opcode = fd_ir_out[31:27];

    //CHheck if R type instruction
    wire fd_isR;
    assign fd_isR = ~fd_opcode[4] & ~fd_opcode[3] & ~fd_opcode[2] & ~fd_opcode[1] & ~fd_opcode[0];
    //Check if add I instruction
    wire fd_isAddI;
    assign fd_isAddI = ~fd_opcode[4] & ~fd_opcode[3] & fd_opcode[2] & ~fd_opcode[1] & fd_opcode[0];


    assign ctrl_readRegA = fd_ir_out[21:17];
    //If not r type, just read reg0
    assign ctrl_readRegB = fd_isR ? fd_ir_out[16:12] : 5'b0;



//DX stage
    wire [31:0] dx_ir_in, dx_pcOut, dx_a_curr, dx_b_curr, dx_ir_out;
    //module dx_latch(clk, cPc, a_in, b_in, inIns, pcOut, aOut, bOut, insOut);
    dx_latch dx(clock, fd_pc_out, data_readRegA, data_readRegB, fd_ir_out, dx_pcOut, dx_a_curr, dx_b_curr, dx_ir_out);

    // get operation for execute stage
    wire [4:0] dx_opcode;
    assign dx_opcode = dx_ir_out[31:27];

    // // Feed into XM stage
    // wire [1:0] mux_b;
    // wire [31:0] xm_o_curr;

    wire [31:0] inp_a, inp_b;
    wire [31:0] alu_b_mux_out;
    //ALU input A is always just A in basic case
    assign inp_a = dx_a_curr;


    //Get immediate value
    wire [31:0] imm;
    assign imm[16:0] = dx_ir_out[16:0];
    //sign extend the imm
    assign imm[31:17] = dx_ir_out[16] ? 15'b111111111111111 : 15'b0;

    //Choose between the immediate and the value from regB
    //module mux_2(out, select, in0, in1);
    wire dx_is_I,dx_is_R;
    assign dx_is_I = ~dx_opcode[4] & ~dx_opcode[3] & dx_opcode[2] & ~dx_opcode[1] & dx_opcode[0];
    assign dx_is_R = ~dx_opcode[4] & ~dx_opcode[3] & ~dx_opcode[2] & ~dx_opcode[1] & ~dx_opcode[0];

    mux_2 operandBMux(inp_b,dx_is_I,dx_b_curr,imm);

    //Wire through ALU inputs, shamt, op
    wire [4:0] alu_opcode, shamt;
    
    //If I type, add, otherwise find
    assign alu_opcode = dx_is_I ? 5'b0 : dx_ir_out[6:2];
    assign shamt = dx_ir_out[11:7];

    //ALU unit and output
    wire [31:0] alu_out, alu_out_ovf;
    wire is_not_equal, is_less_than, alu_overflow;
    alu ula(inp_a, inp_b, alu_opcode, shamt, alu_out, is_not_equal, is_less_than, alu_overflow);

    // //MULTDIV

    //Overflow from all arithematic units
    wire overflow;
    assign overflow = alu_overflow;

//XM stage
    //module xm_latch(clk, o_in, ovfIn, b_in, inIns,  o_out, outOvf, bOut, insOut);
    wire [31:0] xm_o_out, xm_b_out, xm_ir_curr;
    wire xm_overflow_out;
    xm_latch xm(clock, alu_out, overflow, data_readRegB, dx_ir_out, xm_o_out, xm_overflow_out, xm_b_out, xm_ir_curr);

    //HANDLE data memory reads and writes here
    //Wire data and memory adress in case of sw
    assign address_dmem = xm_o_out;
    assign data = xm_b_out;
    wire is_sw;
    //Allow writes to dmem only if instruction is store word
    assign is_sw = ~dx_opcode[4] & ~dx_opcode[3] & dx_opcode[2] & dx_opcode[1] & dx_opcode[0];
    assign wren = is_sw;

    wire[31:0] dataMemOut;
    //lw data from dmem
    assign dataMemOut = q_dmem;
    

//MW Stage
    wire [31:0] mw_o_out, mw_d_out, mw_ir_out;
    wire mw_ovf_out;
    //module mw_latch(clk, o_in, ovfIn, d_in, inIns,  o_out, outOvf, dOut, insOut);
    mw_latch mw(clock, xm_o_out, xm_overflow_out, dataMemOut, xm_ir_curr, mw_o_out, mw_ovf_out, mw_d_out, mw_ir_out);

    //For now just writeback arithematic output, O
    //With lw instruction, dmem output will be stored (use mux) 
    assign ctrl_writeReg = mw_ir_out[26:22];
    assign data_writeReg =  mw_o_out;
    //Disable write enable with other instruction types as added
    assign ctrl_writeEnable = 1'b1;


// 	/* END CODE */
endmodule
