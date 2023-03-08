
module div(quot,rem,exceptRes, divid,divis, clk,ctrl_DIV);
	input[31:0]divid;
	input[31:0]divis;
	input clk;
	input ctrl_DIV;
	output[31:0]quot;
	output[31:0]rem;
	output exceptRes;
	wire exceptHold;
	wire [31:0] counter;


	wire[31:0] opera1;
	wire[31:0] invDivid;
	wire[31:0] resAdd;
	wire saveTemp;
	wire carrIn;
	assign carrIn = 0;
	wire dividPos,divisPos;
	assign dividPos=~divid[31];
	assign divisPos=~divis[31];

	not32 invert(invDivid,divid);
	assign opera1=dividPos ? divid : invDivid;
	adder_32 addRes(resAdd,opera1,carrIn,saveTemp,~dividPos);


	wire[31:0] divisSelect;
	wire[31:0] invDivis;
	wire[31:0] runOut;
	wire inAd;
	wire cnstZer;
	assign cnstZer = 1'b0;
	not32 inverter(invDivis,divis);
	assign divisSelect=divisPos ? divis : invDivis;
	adder_32 adder(runOut,divisSelect,cnstZer,inAd,~divisPos);
	wire[63:0] regI,regBits1,uppaReg;
	wire[64:0]reggOut,sigReg;
	assign regI[63:32] = 32'b0;
	bitwise_or getLow(regI[31:0],resAdd,cnstZer);
	wire countB;

	and countera(countB,~counter[0],
		~counter[1],~counter[2],~counter[3],~counter[4],
		~counter[5],ws1[31]);
	wire countB2;
	and counterCheck(countB2,~counter[0],
			~counter[1],~counter[2],~counter[3],~counter[4],
			counter[5],ws1[31]);
	wire countB3;
	and andIhate(countB3,~counter[0],
					~counter[1],~counter[2],~counter[3],~counter[4],
					~counter[5]);
	wire [31:0] wa,wb;
	bitwise_or or23(wa,uppaReg[63:32],cnstZer);
	assign wb=(~sigReg[63])? ~runOut:runOut;
	wire [31:0]ws1,ws2;
	wire useless1,useless2;
	adder_32 addIn(ws1,wa,wb,useless1,~sigReg[63]);
	adder_32 addinLow(ws2,ws1,runOut,useless2,1'b0);
	not reccon(regBits1[0],ws1[31]);
	assign uppaReg=sigReg << 1;
	assign regBits1[31:1]=uppaReg[31:1];
	assign regBits1[63:32]=countB2?ws2:ws1;
    product_reg register64(reggOut, clk, 1'b1, 1'b1, ctrl_DIV, sigReg);
	counter counterDiv(counter,clk,1'b1,ctrl_DIV);

	assign reggOut=countB3?regI:regBits1;
	wire[31:0] wire99,wire100,wire101,wire102,wore103,wire104;
	wire[31:0]answer;
	wire neg;
	assign exceptHold = except;
	wire wore104;
	not32 inver2t(not1,divid);
	assign neg = ((dividPos&~divisPos)|(~dividPos&divisPos));
	assign wire99=neg?~sigReg[31:0]:sigReg[31:0];
	adder_32 adder2(answer,wire99,0,wore104,neg);
	wire except;
	not32 invert1(not1,divid);


	and checkAnd(except,~divis[31] , ~divis[30] ,
		 ~divis[29] , ~divis[28] ,
		 ~divis[27] , ~divis[26] ,
     ~divis[25] , ~divis[24] ,
		 ~divis[23] , ~divis[22] ,
		 ~divis[21] , ~divis[20] ,
		 ~divis[19] , ~divis[18] ,
     ~divis[17] , ~divis[16] ,
		 ~divis[15] , ~divis[14] ,
		 ~divis[13] , ~divis[12] ,
		 ~divis[11] , ~divis[10] ,
     ~divis[9] , ~divis[8] ,
		 ~divis[7] , ~divis[6] ,
		 ~divis[5] , ~divis[4] ,
		 ~divis[3] , ~divis[2] ,
		 ~divis[1] , ~divis[0]);
	assign exceptRes=except;
	assign quot=except?0:answer;
	bitwise_or rnofjwof(rem,0,sigReg[63:32]);
endmodule