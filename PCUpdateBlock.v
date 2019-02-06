///
//Updates the PC according to Control Signals & Instruction 
module PCUpdateBLock(currentPC,InsOffset,zeroALU,ConBeq,ConBneq,ConJump,nextPC);

input [31:0]currentPC; // Current PC address
input [25:0]InsOffset; // Last 26 bits of instruction
input zeroALU,ConBeq,ConBneq,ConJump; // zero result signal from ALU, Branch & Jump signal from Main Control Unit

output reg [31:0]nextPC; // PC will be updated by this value
wire [31:0]BOffsetExtnd; // Holds the 32 bit sign extended Branch offset address (not shifted)

SignExtend Boffset(InsOffset[15:0],BOffsetExtnd);// Call the sign extension function 

wire k;
assign k=(zeroALU & ConBeq)||(~zeroALU & ConBneq);


always@(currentPC,InsOffset,zeroALU,ConBeq,ConBneq,ConJump) // only currentPC in the sensitivity list will suffice
	begin
	nextPC=currentPC+32'h4;
	// default PC+4
	
	if(k) 
		nextPC=nextPC+{BOffsetExtnd[29:0],2'b00};
	//Sign extended & shifted branch offset address from InsOffset[15:0] added with PC+4 
	
	if(ConJump) 
		nextPC={nextPC[31:28],InsOffset,2'b00};
	//Jump offset address from shifted InsOffset with first 4 bits from PC+4
	end

endmodule
///

// 16 bit to 32 bit sign extention block
module SignExtend(X,Y); 
input [15:0]X;
wire [15:0]T;
output wire [31:0]Y;

assign T={16{X[15]}};
assign Y={T,X};

endmodule
///

