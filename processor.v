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
    wire [31:0] pcActive, pcAdv, fd_pc_out, fd_ir_out, pcNextActual;
    //module pc_reg(clock, reset, in_enable, in, out);
    //load next pc into 
    pc_reg pc(!clock, reset, 1'b1, pcNextActual, pcActive); 
    assign address_imem = pcActive; 
    //module cla_full_adder(a, b, c_in, s);\
    cla_full_adder inc_pc(pcActive, 1, 1'b0, pcAdv); 

    //CHECK IMEM JUMP BEFORE ITS SENT INTO LATCH
    wire isImemJump;
    wire [4:0] imemOpcode;
    assign imemOpcode = q_imem[31:27];
    assign isImemJump = (imemOpcode == 5'b00001) | (imemOpcode == 5'b00011) === 1'b1;
    assign isImemJR = (imemOpcode == 00100) === 1'b1;
    
    assign pcNextActual = isImemJump ? q_imem[26:0] : pcAdv;


    //for JAL, we can already jump to the new pc, but need to store the pc to reg 31

//FD stage



    //Disable enable toggle if time to stall later
    //module fd_latch(clk, enable, cPc, inIns, pcOut, insOut);
    fd_latch fd(!clock, 1'b1, pcActive, q_imem, fd_pc_out, fd_ir_out);

    wire [4:0] fd_opcode;
    assign fd_opcode = fd_ir_out[31:27];

    //CHheck if R type instruction
    wire fd_isR;
    assign fd_isR = ~fd_opcode[4] & ~fd_opcode[3] & ~fd_opcode[2] & ~fd_opcode[1] & ~fd_opcode[0];
    //Check if add I instruction
    wire fd_isAddI;
    assign fd_isAddI = ~fd_opcode[4] & ~fd_opcode[3] & fd_opcode[2] & ~fd_opcode[1] & fd_opcode[0];


    assign ctrl_readRegA = fd_ir_out[21:17];
    //If not r type, read I type result FIX WHEN J
    assign ctrl_readRegB = fd_isR ? fd_ir_out[16:12] : fd_ir_out[26:22];



//DX stage
    //Transform JAL instruction to an addition
    wire is_fd_jal;
    assign is_fd_jal = ~fd_opcode[4] & ~fd_opcode[3] & ~fd_opcode[2] & fd_opcode[1] & fd_opcode[0];
    wire [31:0] dx_ir_in, dx_pcOut, dx_a_curr, dx_b_curr, dx_ir_out;
    assign dx_ir_in = is_fd_jal ? 32'b00000111110000000000000000000000 : fd_ir_out;
    //module dx_latch(clk, cPc, a_in, b_in, inIns, pcOut, aOut, bOut, insOut);
    dx_latch dx(!clock, fd_pc_out, data_readRegA, data_readRegB, dx_ir_in, dx_pcOut, dx_a_curr, dx_b_curr, dx_ir_out);

    // get operation for execute stage
    wire [4:0] dx_opcode;
    assign dx_opcode = dx_ir_out[31:27];

    // // Feed into XM stage
    // wire [1:0] mux_b;
    // wire [31:0] xm_o_curr;

    wire [31:0] inp_a, inp_b;
    wire [31:0] alu_b_mux_out;

    wire [1:0] mux_inpa_select, mux_inpb_select;
    wire mux_wmselect;
    //module mux_4(in0, in1, in2, in3, out, sel);


    mux_4 aluAinputmux(xm_o_out,data_writeReg,dx_a_curr,32'b0,inp_a,mux_inpa_select);
    //Get immediate value
    wire [31:0] imm;
    assign imm[16:0] = dx_ir_out[16:0];
    //sign extend the imm
    assign imm[31:17] = dx_ir_out[16] ? 15'b111111111111111 : 15'b0;

    //Choose between the immediate and the value from regB
    //module mux_2(out, select, in0, in1);
    wire dx_is_I,dx_is_R, dx_is_addi,dx_is_sw_I,dx_is_lw_I, dx_is_jal;
    assign dx_is_addi = ~dx_opcode[4] & ~dx_opcode[3] & dx_opcode[2] & ~dx_opcode[1] & dx_opcode[0];
    assign dx_is_sw_I = ~dx_opcode[4] & ~dx_opcode[3] & dx_opcode[2] & dx_opcode[1] & dx_opcode[0];
    assign dx_is_lw_I = ~dx_opcode[4] & dx_opcode[3] & ~dx_opcode[2] & ~dx_opcode[1] & ~dx_opcode[0];
    assign dx_is_R = ~dx_opcode[4] & ~dx_opcode[3] & ~dx_opcode[2] & ~dx_opcode[1] & ~dx_opcode[0];
    assign dx_is_jal = (dx_ir_out == 32'b00000111110000000000000000000000);

    assign dx_is_I = dx_is_addi | dx_is_sw_I | dx_is_lw_I;
    wire [31:0] bybassBout;
    //module mux_4(in0, in1, in2, in3, out, sel);
    mux_4 aluBmuxBypass(xm_o_out,data_writeReg,dx_b_curr,32'b0,bybassBout,mux_inpb_select);
    mux_2 operandBMux(inp_b,dx_is_I,bybassBout,imm);


    //Wire through ALU inputs, shamt, op
    wire [4:0] alu_opcode, shamt;
    
    //If I type, add, otherwise find
    assign alu_opcode = dx_is_I ? 5'b0 : dx_ir_out[6:2];
    assign shamt = dx_is_R ? dx_ir_out[11:7] : 5'b0;

    //ALU unit and output
    wire [31:0] alu_out, alu_out_ovf;
    wire is_not_equal, is_less_than, alu_overflow;
    alu ula(inp_a, inp_b, alu_opcode, shamt, alu_out, is_not_equal, is_less_than, alu_overflow);



    //module multdiv(
	// data_operandA, data_operandB,
	// ctrl_MULT, ctrl_DIV,
	// clock,
	// data_result, data_exception, data_resultRDY);
    // //MULTDIV, 
    wire [32:0] multdiv_inpa, multdiv_inpb, mdiv_result;
    wire isMult, isDiv, is_result_ready, mdivClk, is_mdiv_exception;
    assign multdiv_in_a = dx_ir_out[21:17];
    assign multdiv_in_b = dx_ir_out[16:12];

    
    // multdiv muldivunit(multdiv_inpa, multdiv_inpb, isMult, isDiv, mdivClk, mdiv_result, is_mdiv_exception, is_result_ready);

    //Overflow from all arithematic units
    //JR

    wire [31:0] jal_pc;
    //(a, b, c_in, s);
    cla_full_adder jalPC(,32'b1,1'b0,jal_pc);
    wire overflow;
    //CHECK IF MDIV OVF WHEN IMPLEMENTED
    assign overflow = alu_overflow;


    //module rstatus_res(alu_op, op, rstatus_write_val);
    wire [31:0] rstatus_exception_val;
    rstatus_res assignExcept(alu_opcode, dx_opcode, rstatus_exception_val);


    wire [31:0] xm_o_in;
    assign xm_o_in = (dx_is_jal&&!overflow) ? dx_pcOut : (overflow && !dx_is_jal ? rstatus_exception_val : alu_out);




//XM stage
    //module xm_latch(clk, o_in, ovfIn, b_in, inIns,  o_out, outOvf, bOut, insOut);
    wire [31:0] xm_o_out, xm_b_out, xm_ir_curr;
    wire xm_overflow_out;
    xm_latch xm(!clock, xm_o_in, overflow, bybassBout, dx_ir_out, xm_o_out, xm_overflow_out, xm_b_out, xm_ir_curr);

    //HANDLE data memory reads and writes here
    //Wire data and memory adress in case of sw
    wire [4:0] xm_opcode;
    assign xm_opcode = xm_ir_curr[31:27];
    assign address_dmem = xm_o_out;
    assign data = mux_wmselect ? data_writeReg : xm_b_out;
    wire is_sw_xm;
    //Allow writes to dmem only if instruction is store word
    assign is_sw_xm = ~xm_opcode[4] & ~xm_opcode[3] & xm_opcode[2] & xm_opcode[1] & xm_opcode[0];
    assign wren = is_sw_xm;
    

//MW Stage
    wire is_mw_jal;
    wire [31:0] mw_o_out, mw_d_out, mw_ir_out;
    wire mw_ovf_out;
    //module mw_latch(clk, o_in, ovfIn, d_in, inIns,  o_out, outOvf, dOut, insOut);
    mw_latch mw(!clock, xm_o_out, xm_overflow_out, q_dmem, xm_ir_curr, mw_o_out, mw_ovf_out, mw_d_out, mw_ir_out);

    //If jal, change data to dlatch pc
    assign is_mw_jal = (mw_ir_out == 32'b00000111110000000000000000000000) === 1'b1;

    wire [4:0] mw_opcode;
    wire is_mw_rOp, is_mw_lw, is_mw_addi;
    assign mw_opcode = mw_ir_out[31:27];
    assign is_mw_rOp = ~mw_opcode[4] & ~mw_opcode[3] & ~mw_opcode[2] & ~mw_opcode[1] & ~mw_opcode[0];
    assign is_mw_lw = ~mw_opcode[4] & mw_opcode[3] & ~mw_opcode[2] & ~mw_opcode[1] & ~mw_opcode[0];
    assign is_mw_addi = ~mw_opcode[4] & ~mw_opcode[3] & mw_opcode[2] & ~mw_opcode[1] & mw_opcode[0];

    //module mux_2(out, select, in0, in1);
    //out, inp, enab
    tri_state_buffer aluData(data_writeReg, mw_o_out, (is_mw_rOp));
    tri_state_buffer lw(data_writeReg, mw_d_out, (is_mw_lw));



    mux_2 writebackmux(data_writeReg, is_mw_lw, mw_o_out, mw_d_out);

    //assign writeback reg, changes if status
    assign ctrl_writeReg = (mw_ovf_out) && !is_mw_jal ? 5'd30 : mw_ir_out[26:22];
    //Disable write enable with other instruction types as added
    assign ctrl_writeEnable = is_mw_lw | is_mw_addi | is_mw_rOp | is_mw_jal;

    //module bypass(dx_ir, xm_ir, mw_ir, xm_ovf_out, mw_ovf_out, muxA_select, muxB_select, wmSelect);
    bypass bypassUnit(dx_ir_out, xm_ir_curr, mw_ir_out, xm_overflow_out, mw_ovf_out, mux_inpa_select, mux_inpb_select, mux_wmselect);



// 	/* END CODE */
endmodule