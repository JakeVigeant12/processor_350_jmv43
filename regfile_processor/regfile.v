module regfile (
	clock,
	ctrl_writeEnable, ctrl_reset, ctrl_writeReg,
	ctrl_readRegA, ctrl_readRegB, data_writeReg,
	data_readRegA, data_readRegB
);

	input clock, ctrl_writeEnable, ctrl_reset;
	input [4:0] ctrl_writeReg, ctrl_readRegA, ctrl_readRegB;
	input [31:0] data_writeReg;

	output [31:0] data_readRegA, data_readRegB;


	//decoder outputs 1 in index of register to be selected
	//module register(clock, write_enable, input_data, output_data, reset);
	//decoder_32(out, select, enable)
	wire [31:0] selectedWriteReg, selectedReadRegA, selectedReadRegB;

	decoder_32 selectWriteReg(selectedWriteReg, ctrl_writeReg, ctrl_writeEnable);
	decoder_32 selectReadA(selectedReadRegA, ctrl_readRegA, 1'b1);
	decoder_32 selectReadB(selectedReadRegB, ctrl_readRegB, 1'b1);

	//32 registers 
	//module register(clock, write_enable, input_data, output_data, reset);
	wire [31:0] reg0out,reg1out,reg2out,reg3out,reg4out,reg5out,reg6out,reg7out,reg8out,reg9out,
	reg10out,reg11out,reg12out,reg13out,reg14out,reg15out,reg16out,reg17out,reg18out,reg19out,
	reg20out,reg21out,reg22out,reg23out,reg24out,reg25out,reg26out,reg27out,reg28out,reg29out,reg30out,reg31out;


	register reg0(clock, 1'b0, data_writeReg, reg0out, 1'b1);
	register reg1(clock, selectedWriteReg[1], data_writeReg, reg1out, ctrl_reset);
	register reg2(clock, selectedWriteReg[2], data_writeReg, reg2out, ctrl_reset);
	register reg3(clock, selectedWriteReg[3], data_writeReg, reg3out, ctrl_reset);
	register reg4(clock, selectedWriteReg[4], data_writeReg, reg4out, ctrl_reset);
	register reg5(clock, selectedWriteReg[5], data_writeReg, reg5out, ctrl_reset);
	register reg6(clock, selectedWriteReg[6], data_writeReg, reg6out, ctrl_reset);
	register reg7(clock, selectedWriteReg[7], data_writeReg, reg7out, ctrl_reset);
	register reg8(clock, selectedWriteReg[8], data_writeReg, reg8out, ctrl_reset);
	register reg9(clock, selectedWriteReg[9], data_writeReg, reg9out, ctrl_reset);
	register reg10(clock, selectedWriteReg[10], data_writeReg, reg10out, ctrl_reset);
	register reg11(clock, selectedWriteReg[11], data_writeReg, reg11out, ctrl_reset);
	register reg12(clock, selectedWriteReg[12], data_writeReg, reg12out, ctrl_reset);
	register reg13(clock, selectedWriteReg[13], data_writeReg, reg13out, ctrl_reset);
	register reg14(clock, selectedWriteReg[14], data_writeReg, reg14out, ctrl_reset);
	register reg15(clock, selectedWriteReg[15], data_writeReg, reg15out, ctrl_reset);
	register reg16(clock, selectedWriteReg[16], data_writeReg, reg16out, ctrl_reset);
	register reg17(clock, selectedWriteReg[17], data_writeReg, reg17out, ctrl_reset);
	register reg18(clock, selectedWriteReg[18], data_writeReg, reg18out, ctrl_reset);
	register reg19(clock, selectedWriteReg[19], data_writeReg, reg19out, ctrl_reset);
	register reg20(clock, selectedWriteReg[20], data_writeReg, reg20out, ctrl_reset);
	register reg21(clock, selectedWriteReg[21], data_writeReg, reg21out, ctrl_reset);
	register reg22(clock, selectedWriteReg[22], data_writeReg, reg22out, ctrl_reset);
	register reg23(clock, selectedWriteReg[23], data_writeReg, reg23out, ctrl_reset);
	register reg24(clock, selectedWriteReg[24], data_writeReg, reg24out, ctrl_reset);
	register reg25(clock, selectedWriteReg[25], data_writeReg, reg25out, ctrl_reset);
	register reg26(clock, selectedWriteReg[26], data_writeReg, reg26out, ctrl_reset);
	register reg27(clock, selectedWriteReg[27], data_writeReg, reg27out, ctrl_reset);
	register reg28(clock, selectedWriteReg[28], data_writeReg, reg28out, ctrl_reset);
	register reg29(clock, selectedWriteReg[29], data_writeReg, reg29out, ctrl_reset);
	register reg30(clock, selectedWriteReg[30], data_writeReg, reg30out, ctrl_reset);
	register reg31(clock, selectedWriteReg[31], data_writeReg, reg31out, ctrl_reset);
	//select register A to read
	//module tri_state_buffer(out, inp, enable);
	tri_state_buffer outA0(data_readRegA, reg0out, selectedReadRegA[0]);
tri_state_buffer outA1(data_readRegA, reg1out, selectedReadRegA[1]);
tri_state_buffer outA2(data_readRegA, reg2out, selectedReadRegA[2]);
tri_state_buffer outA3(data_readRegA, reg3out, selectedReadRegA[3]);
tri_state_buffer outA4(data_readRegA, reg4out, selectedReadRegA[4]);
tri_state_buffer outA5(data_readRegA, reg5out, selectedReadRegA[5]);
tri_state_buffer outA6(data_readRegA, reg6out, selectedReadRegA[6]);
tri_state_buffer outA7(data_readRegA, reg7out, selectedReadRegA[7]);
tri_state_buffer outA8(data_readRegA, reg8out, selectedReadRegA[8]);
tri_state_buffer outA9(data_readRegA, reg9out, selectedReadRegA[9]);
tri_state_buffer outA10(data_readRegA, reg10out, selectedReadRegA[10]);
tri_state_buffer outA11(data_readRegA, reg11out, selectedReadRegA[11]);
tri_state_buffer outA12(data_readRegA, reg12out, selectedReadRegA[12]);
tri_state_buffer outA13(data_readRegA, reg13out, selectedReadRegA[13]);
tri_state_buffer outA14(data_readRegA, reg14out, selectedReadRegA[14]);
tri_state_buffer outA15(data_readRegA, reg15out, selectedReadRegA[15]);
tri_state_buffer outA16(data_readRegA, reg16out, selectedReadRegA[16]);
tri_state_buffer outA17(data_readRegA, reg17out, selectedReadRegA[17]);
tri_state_buffer outA18(data_readRegA, reg18out, selectedReadRegA[18]);
tri_state_buffer outA19(data_readRegA, reg19out, selectedReadRegA[19]);
tri_state_buffer outA20(data_readRegA, reg20out, selectedReadRegA[20]);
tri_state_buffer outA21(data_readRegA, reg21out, selectedReadRegA[21]);
tri_state_buffer outA22(data_readRegA, reg22out, selectedReadRegA[22]);
tri_state_buffer outA23(data_readRegA, reg23out, selectedReadRegA[23]);
tri_state_buffer outA24(data_readRegA, reg24out, selectedReadRegA[24]);
tri_state_buffer outA25(data_readRegA, reg25out, selectedReadRegA[25]);
tri_state_buffer outA26(data_readRegA, reg26out, selectedReadRegA[26]);
tri_state_buffer outA27(data_readRegA, reg27out, selectedReadRegA[27]);
tri_state_buffer outA28(data_readRegA, reg28out, selectedReadRegA[28]);
tri_state_buffer outA29(data_readRegA, reg29out, selectedReadRegA[29]);
tri_state_buffer outA30(data_readRegA, reg30out, selectedReadRegA[30]);
tri_state_buffer outA31(data_readRegA, reg31out, selectedReadRegA[31]);

//select register B to read
//module tri_state_buffer(out, inp, enable);
tri_state_buffer outB0(data_readRegB, reg0out, selectedReadRegB[0]);
tri_state_buffer outB1(data_readRegB, reg1out, selectedReadRegB[1]);
tri_state_buffer outB2(data_readRegB, reg2out, selectedReadRegB[2]);
tri_state_buffer outB3(data_readRegB, reg3out, selectedReadRegB[3]);
tri_state_buffer outB4(data_readRegB, reg4out, selectedReadRegB[4]);
tri_state_buffer outB5(data_readRegB, reg5out, selectedReadRegB[5]);
tri_state_buffer outB6(data_readRegB, reg6out, selectedReadRegB[6]);
tri_state_buffer outB7(data_readRegB, reg7out, selectedReadRegB[7]);
tri_state_buffer outB8(data_readRegB, reg8out, selectedReadRegB[8]);
tri_state_buffer outB9(data_readRegB, reg9out, selectedReadRegB[9]);
tri_state_buffer outB10(data_readRegB, reg10out, selectedReadRegB[10]);
tri_state_buffer outB11(data_readRegB, reg11out, selectedReadRegB[11]);
tri_state_buffer outB12(data_readRegB, reg12out, selectedReadRegB[12]);
tri_state_buffer outB13(data_readRegB, reg13out, selectedReadRegB[13]);
tri_state_buffer outB14(data_readRegB, reg14out, selectedReadRegB[14]);
tri_state_buffer outB15(data_readRegB, reg15out, selectedReadRegB[15]);
tri_state_buffer outB16(data_readRegB, reg16out, selectedReadRegB[16]);
tri_state_buffer outB17(data_readRegB, reg17out, selectedReadRegB[17]);
tri_state_buffer outB18(data_readRegB, reg18out, selectedReadRegB[18]);
tri_state_buffer outB19(data_readRegB, reg19out, selectedReadRegB[19]);
tri_state_buffer outB20(data_readRegB, reg20out, selectedReadRegB[20]);
tri_state_buffer outB21(data_readRegB, reg21out, selectedReadRegB[21]);
tri_state_buffer outB22(data_readRegB, reg22out, selectedReadRegB[22]);
tri_state_buffer outB23(data_readRegB, reg23out, selectedReadRegB[23]);
tri_state_buffer outB24(data_readRegB, reg24out, selectedReadRegB[24]);
tri_state_buffer outB25(data_readRegB, reg25out, selectedReadRegB[25]);
tri_state_buffer outB26(data_readRegB, reg26out, selectedReadRegB[26]);
tri_state_buffer outB27(data_readRegB, reg27out, selectedReadRegB[27]);
tri_state_buffer outB28(data_readRegB, reg28out, selectedReadRegB[28]);
tri_state_buffer outB29(data_readRegB, reg29out, selectedReadRegB[29]);
tri_state_buffer outB30(data_readRegB, reg30out, selectedReadRegB[30]);
tri_state_buffer outB31(data_readRegB, reg31out, selectedReadRegB[31]);
endmodule
