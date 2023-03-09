`timescale 1 ns/10 ps

module alu_toy_tb;
//module alu(data_operandA, data_operandB, ctrl_ALUopcode, ctrl_shiftamt, data_result, isNotEqual, isLessThan, overflow);

    reg [31:0] data_operandA, data_operandB;
    reg [4:0] ctrl_ALUopcode, ctrl_shiftamt;
    wire [31:0] data_result;
    wire isNotEqual, isLessThan, overflow;

    alu testUnit(data_operandA, data_operandB,ctrl_ALUopcode, ctrl_shiftamt,data_result, isNotEqual, isLessThan, overflow);

    initial // initial block executes only once
        begin
            // values for a and b
            data_operandA = 2147483647;
;
            data_operandB = 17;
            ctrl_ALUopcode = 5'b00000;
            ctrl_shiftamt = 12;
            #80; // wait for period 

            $display("a:%d, b:%d, opcode:%b, shift:%d => result:%d, notEq:%b, lesThan:%b, ovf:%b",data_operandA, data_operandB,ctrl_ALUopcode, ctrl_shiftamt,data_result, isNotEqual, isLessThan, overflow);
            $finish;

        end


endmodule