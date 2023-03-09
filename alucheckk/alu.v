module alu(data_operandA, data_operandB, ctrl_ALUopcode, ctrl_shiftamt, data_result, isNotEqual, isLessThan, overflow);
        
    input [31:0] data_operandA, data_operandB;
    input [4:0] ctrl_ALUopcode, ctrl_shiftamt;

    output [31:0] data_result;
    output isNotEqual, isLessThan, overflow;


    wire [31:0] inputB;
    wire [31:0] data_operandB_inverted;

    bit_inverter not_b(data_operandB, data_operandB_inverted);
    assign inputB = ctrl_ALUopcode[0] ? data_operandB_inverted : data_operandB;

 

    wire [31:0] addOut;
    cla_full_adder add(data_operandA, inputB, ctrl_ALUopcode[0], addOut);

    wire [31:0] rsaRes, llsRes;
    right_barrel_shifter right_shifter(data_operandA, ctrl_shiftamt, rsaRes);
    left_barrel_shifter left_shifter(data_operandA, ctrl_shiftamt, llsRes);

    wire [31:0] andRes;
    wire [31:0] orRes;

    bitwise_and ander (data_operandA, data_operandB, andRes);
    bitwise_or orrer (data_operandA, data_operandB, orRes);


    mux_8 alu_mux(addOut, addOut, andRes, orRes, llsRes, rsaRes, 32'b0, 32'b0, data_result, ctrl_ALUopcode[2:0]);

    wire not_msb_A, not_msb_B, not_msb_addOut;
    not complement_msb_A(not_msb_A, data_operandA[31]);
    not complement_msb_B(not_msb_B, inputB[31]);
    not complement_msb_addOut(not_msb_addOut, addOut[31]);

    wire overflow_pos, overflow_neg;
    and check_overflow_neg(overflow_neg, data_operandA[31], inputB[31], not_msb_addOut);
    and check_pos_overfow(overflow_pos, not_msb_A, not_msb_B, addOut[31]);
    or check_overflow(overflow, overflow_pos, overflow_neg);

    wire check_less_than_standard, check_less_than_special;
    and check_special_less_than(check_less_than_special, addOut[31] ? 0 : 1, overflow_neg);
    and check_normal_less_than(check_less_than_standard, addOut[31], overflow_pos ? 0 : 1);
    or check_less_than(isLessThan, check_less_than_standard, check_less_than_special);


    or check_not_equal(isNotEqual, addOut[0], addOut[1], addOut[2], addOut[3], addOut[4], addOut[5], addOut[6], addOut[7], addOut[8], addOut[9], addOut[10], 
        addOut[11], addOut[12], addOut[13], addOut[14], addOut[15], addOut[16], addOut[17], addOut[18], addOut[19], addOut[20], addOut[21], addOut[22], addOut[23],
        addOut[24], addOut[25], addOut[26], addOut[27], addOut[28], addOut[29], addOut[30], addOut[31]);
endmodule