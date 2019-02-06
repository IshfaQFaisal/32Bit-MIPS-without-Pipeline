module ProgCount(clk,reset,iPC,oPC);
input clk;
input reset;
input [31:0]iPC; // iPC is the output of PCUpdate block
output reg[31:0]oPC; // oPC goes to the Instruction Memory block

always@(posedge clk,negedge reset)
	if(reset==0) oPC<=32'h0;
	else oPC<=iPC;

endmodule
