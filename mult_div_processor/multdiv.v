module multdiv(
	data_operandA, data_operandB,
	ctrl_MULT, ctrl_DIV,
	clock,
	data_result, data_exception, data_resultRDY);

    input [31:0] data_operandA, data_operandB;
    input ctrl_MULT;
	input ctrl_DIV, clock;


    output [31:0] data_result;
    output data_exception, data_resultRDY;

	wire dffeResM,dffeResD;
	wire startCalc;
	assign startCalc = ctrl_MULT | ctrl_DIV;
	wire zerConst;
	assign zerConst = 1'b0;

	dffe_ref dffe1(dffeResM,ctrl_MULT,clock,startCalc,zerConst);
	dffe_ref dffe2(dffeResD,ctrl_DIV,clock,startCalc,zerConst);

	wire[31:0]counter1,counter2;
	counter count1(counter1,clock,dffeResM,ctrl_MULT);
	counter count2(counter2,clock,dffeResD,ctrl_DIV);

	wire [31:0] multiplied,divided;
	wire[31:0]remainder;
	wire multoverflow;
	wire divoverflow;
    wire overflow;
	mult m1(multiplied,multoverflow,data_operandA,data_operandB,clock,counter1);
	div d1(divided,remainder,divoverflow,data_operandA,data_operandB,clock,ctrl_DIV);
	assign data_result =dffeResM?multiplied:divided;
	assign overflow= dffeResM?multoverflow:divoverflow;
	wire ans238;
    wire ans239;
	or or2(ans238,counter1[4]&counter1[0],zerConst);
	or or3(ans239,counter2[0]&~counter2[1]&
		~counter2[2]&~counter2[3]&~counter2[4]&counter2[5]&
		~counter2[6]&~counter2[7]&~counter2[8]&~counter2[9]&
		~counter2[10]&~counter2[11]&~counter2[12]&~counter2[13]&
		~counter2[14]&~counter2[15]&~counter2[16]&~counter2[17]&
		~counter2[18]&~counter2[19]&~counter2[20]&~counter2[21]&
		~counter2[22]&~counter2[23]&~counter2[24]&~counter2[25]&
		~counter2[26]&~counter2[27]&~counter2[28]&~counter2[29]&
		~counter2[30]&~counter2[31],
		zerConst);
	assign data_resultRDY = dffeResM ? ans238
		:ans239;
	assign data_exception=dffeResM ?multoverflow:divoverflow;


endmodule
