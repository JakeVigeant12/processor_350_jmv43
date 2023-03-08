
module mult(product,overflow,multiplicand, multiplier, clk, counter);
	input [31:0] multiplicand,multiplier, counter;
	input clk;
	output [31:0] product;
	output overflow;

	wire signed [64:0] running_prod_init, in, running_prod, running_prod_out;
	wire signed [2:0]ctrl_bits;
	wire temp2,temp3,temp4, temp5;

	assign running_prod_init[0] = andZeroToo;
    assign running_prod_init[32:1] = multiplier[31:0];
    assign running_prod_init[64:33] = 32'b0;

	and gateTemp(temp2,~counter[0],~counter[1],~counter[2],~counter[3],~counter[4]);


	assign in=temp2 ? running_prod_init:running_prod>>>2;


	wire yeaIneedOnes, andZeroToo;
	assign yeaIneedOnes = 1'b1;
	assign andZeroToo = 1'b0;
    product_reg prReg(in, clk, yeaIneedOnes, yeaIneedOnes, andZeroToo, running_prod_out);
	assign ctrl_bits[0] = running_prod_out[0];
	assign ctrl_bits[1] = running_prod_out[1];
	assign ctrl_bits[2] = running_prod_out[2];


	wire w1,w3;


	and tempGate2(w1,ctrl_bits[0],ctrl_bits[1],ctrl_bits[2]);
	or tempgate3(w3,!ctrl_bits[2],w1);
	wire [31:0]cntrlCom1,cntrlCom,cntrlCom1Shi;
	assign cntrlCom1= (ctrl_bits[0] & ctrl_bits[1] & ctrl_bits[2]) | (~ctrl_bits[0] & ~ctrl_bits[1] & ~ctrl_bits[2]) ? 0 : multiplicand;
	assign cntrlCom1Shi=cntrlCom1<<1;
  assign cntrlCom = (ctrl_bits[0] & ctrl_bits[1] & ~ctrl_bits[2]) |  (~ctrl_bits[0] & ~ctrl_bits[1] & ctrl_bits[2]) ? cntrlCom1Shi :cntrlCom1;

	wire [31:0]num1,num2,ans,notted;
	wire tempHold;
	bitwise_or ored(num1,0,running_prod_out[64:33]);
	not32 nottedout(notted,cntrlCom);
	assign num2= w3 ? cntrlCom : notted;
	wire holdComp;
	assign holdComp = !w3;
	adder_32 adder2(ans,num1,num2,tempHold,holdComp);


	bitwise_or prodRes(running_prod[64:33],ans,0);
	bitwise_or prodRuns(running_prod[32:1],running_prod_out[32:1],0);
	or prodRunt(running_prod[0],running_prod_out[0],1'b0);

	bitwise_or prodMain(product,running_prod[32:1],0);

	wire wrong;
	and chekcWrong(wrong,multiplicand[31],multiplier[31],product[31]);
	wire chkFir;
	and andFirst(chkFir,running_prod_out[64] , running_prod_out[63] , running_prod_out[62] ,
        running_prod_out[61] , running_prod_out[60] , running_prod_out[59] , running_prod_out[58] ,
        running_prod_out[57] , running_prod_out[56] , running_prod_out[55] , running_prod_out[54] ,
        running_prod_out[53] , running_prod_out[52] , running_prod_out[51] , running_prod_out[50] ,
        running_prod_out[49] , running_prod_out[48] , running_prod_out[47] , running_prod_out[46] ,
        running_prod_out[45] , running_prod_out[44] , running_prod_out[43] , running_prod_out[42] ,
        running_prod_out[41] , running_prod_out[40] , running_prod_out[39] , running_prod_out[38] ,
        running_prod_out[37] , running_prod_out[36] , running_prod_out[35] , running_prod_out[34] ,
        running_prod_out[33] , running_prod_out[32]);
	wire chkSec;
	and andSec(chkSec,~running_prod_out[64] , ~running_prod_out[63] , ~running_prod_out[62] ,
			  ~running_prod_out[61] , ~running_prod_out[60] , ~running_prod_out[59] , ~running_prod_out[58] ,
			  ~running_prod_out[57] , ~running_prod_out[56] , ~running_prod_out[55] , ~running_prod_out[54] ,
			  ~running_prod_out[53] , ~running_prod_out[52] , ~running_prod_out[51] , ~running_prod_out[50] ,
			  ~running_prod_out[49] , ~running_prod_out[48] , ~running_prod_out[47] , ~running_prod_out[46] ,
			  ~running_prod_out[45] , ~running_prod_out[44] , ~running_prod_out[43] , ~running_prod_out[42] ,
			  ~running_prod_out[41] , ~running_prod_out[40] , ~running_prod_out[39] , ~running_prod_out[38] ,
			  ~running_prod_out[37] , ~running_prod_out[36] , ~running_prod_out[35] , ~running_prod_out[34] ,
			  ~running_prod_out[33] , ~running_prod_out[32]);

	or ovfRes(overflow,wrong,~(chkFir | chkSec));

endmodule