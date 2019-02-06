////// ***** Main MIPS Execution Module*****//////
//////////////////////////////////////////////////
module MIPS(clk,Reset,ALUResult);
input clk,Reset;
output wire [31:0]ALUResult;

wire [31:0]currentPC;
wire [31:0]nextPC;

wire [31:0]Instruction;

wire [31:0]SignExtendedOffset;
wire [31:0]ReadData1,ReadData2;
wire [31:0]ALUIn2;

wire [31:0]WriteDataReg;
wire [4:0]WriteRegAdr;

wire RegDst,ALUSrc,MemtoReg,RegWrite,MemRead,MemWrite,BrEq,BrNeq,Jump;
wire [1:0]ALUOp;
wire [3:0]ALUConInput;

wire ZeroALU;

wire [31:0]MemReadData;

//#1.Input to Instruction memory
Instr_Mem MyInstr_Mem(.PC (currentPC),.Instruction (Instruction));

//#2.Invoking the main control
MainControl MyMainControl(.Op (Instruction[31:26]), .RegDst (RegDst), .ALUSrc (ALUSrc), .MemtoReg (MemtoReg), .RegWrite (RegWrite), .MemRead (MemRead), .MemWrite (MemWrite), .BrEq (BrEq), .BrNeq (BrNeq), .ALUOp (ALUOp), .Jump (Jump));

//#3.Invoking RegDst 5bit MUX
MUX2to1_5bit MyMUX2to1_5bit(.Sw (RegDst), .A (Instruction[20:16]), .B (Instruction[15:11]), .Z (WriteRegAdr));

//#4.Invoking The RegisterFile
Register MyRegister(.RegWrite (RegWrite), .ReadReg1 (Instruction[25:21]), .ReadReg2 (Instruction[20:16]), .ReadData1 (ReadData1), .ReadData2 (ReadData2), .WriteReg (WriteRegAdr), .WriteData (WriteDataReg), .clk (clk));

//#5.Invoking the sign extend block
SignExtend MySignExtend(.X(Instruction[15:0]),.Y(SignExtendedOffset));

//#6.Invoking the 32bit MUX ALUSrc
MUX2to1_32bit MyMUX2to1_32bit1(.Sw(ALUSrc),.A(ReadData2),.B(SignExtendedOffset),.Z(ALUIn2));

//#7.Invoking the ALU Control
ALUControl MyALUControl(.ALUOp(ALUOp),.FieldFunc(Instruction[5:0]),.ALUConInput(ALUConInput));

//#8.Invoking the ALU
ALU MyALU(.ALUControl(ALUConInput),.A(ReadData1),.B(ALUIn2),.ALUResult(ALUResult),.ZeroALU(ZeroALU));

//#9.Invoking the Data Memory
DataMemory MyDataMemory(.clk(clk) ,.MemReadEn(MemRead),.MemWriteEn(MemWrite),.MemAccessAdr(ALUResult),.MemWriteData(ReadData2),.MemReadData(MemReadData));

//#10.Invoking the 32bit MemtoReg MUX
MUX2to1_32bit MyMUX2to1_32bit2(.Sw(MemtoReg),.A(ALUResult),.B(MemReadData),.Z(WriteDataReg));

//#11.Invoking the PC Update Block
PCUpdateBLock MyPCUpdateBLock(.currentPC (currentPC),.InsOffset (Instruction[25:0]),.zeroALU (ZeroALU),.ConBeq (BrEq),.ConBneq(BrNeq),.ConJump (Jump),.nextPC (nextPC));

//#12.Finally the PC module
ProgCount MyProgCount(.clk(clk),.Reset(Reset),.iPC(nextPC),.oPC(currentPC));

endmodule
///*************************///
//#3.5bit MUX
module MUX2to1_5bit(Sw,A,B,Z);
input [4:0]A,B;
input Sw;
output reg [4:0]Z;

always@(*)
	if(Sw) Z<=B;
	else Z<=A;

endmodule
////
//#6#10.32bit MUX
module MUX2to1_32bit(Sw,A,B,Z);
input [31:0]A,B;
input Sw;
output reg [31:0]Z;

always@(*)
	if(Sw) Z<=B;
	else Z<=A;

endmodule
////
//#5.Sign Extend Block
module SignExtend(X,Y); 
input [15:0]X;
wire [15:0]T;
output wire [31:0]Y;

assign T={16{X[15]}};
assign Y={T,X};

endmodule
////
//#2.Main Control
module MainControl(Op,RegDst,ALUSrc,MemtoReg,RegWrite,MemRead,MemWrite,BrEq,BrNeq,ALUOp,Jump);

input [5:0]Op;
output reg RegDst,ALUSrc,MemtoReg,RegWrite,MemRead,MemWrite,BrEq,BrNeq,Jump;
output reg [1:0]ALUOp;

parameter   RformatOp=6'b000000,
			LoadOp=6'b100011,
			StoreOp=6'b101011,
			BeqOp=6'b000100,
			BneqOp=6'b000110,
			JumpOp=6'b100110,
			AddiOp=6'b101000;

always@(Op)
	case(Op)
		RformatOp:begin //R-format
					RegDst=1'b1;
					ALUSrc=1'b0;
					MemtoReg=1'b0;
					RegWrite=1'b1;
					MemRead=1'b0;
					MemWrite=1'b0;
					BrEq=1'b0;
					BrNeq=1'b0;
					ALUOp=2'b10;
					Jump=1'b0;
					end
		LoadOp:   begin //Load
					RegDst=1'b0;
					ALUSrc=1'b1;
					MemtoReg=1'b1;
					RegWrite=1'b1;
					MemRead=1'b1;
					MemWrite=1'b0;
					BrEq=1'b0;
					BrNeq=1'b0;
					ALUOp=2'b00;
					Jump=1'b0;
					end
		StoreOp:  begin //Store
					RegDst=1'b0;
					ALUSrc=1'b1;
					MemtoReg=1'b0;
					RegWrite=1'b0;
					MemRead=1'b0;
					MemWrite=1'b1;
					BrEq=1'b0;
					BrNeq=1'b0;
					ALUOp=2'b00;
					Jump=1'b0;
					end				
		BeqOp:    begin //Branch Equal
					RegDst=1'b0;
					ALUSrc=1'b0;
					MemtoReg=1'b0;
					RegWrite=1'b0;
					MemRead=1'b0;
					MemWrite=1'b0;
					BrEq=1'b1;
					BrNeq=1'b0;
					ALUOp=2'b01;//To execute Add in ALU
					Jump=1'b0;
					end
		BneqOp:   begin //Branch Not equal
					RegDst=1'b0;
					ALUSrc=1'b0;
					MemtoReg=1'b0;
					RegWrite=1'b0;
					MemRead=1'b0;
					MemWrite=1'b0;
					BrEq=1'b0;
					BrNeq=1'b1;
					ALUOp=2'b01;// Same as Branch Equal
					Jump=1'b0;
					end	
		JumpOp:   begin //Jump
					RegDst=1'b0;
					ALUSrc=1'b0;
					MemtoReg=1'b0;
					RegWrite=1'b0;
					MemRead=1'b0;
					MemWrite=1'b0;
					BrEq=1'b0;
					BrNeq=1'b0;
					ALUOp=2'b00;
					Jump=1'b1;
					end
		AddiOp:   begin //Addi
					RegDst=1'b0;
					ALUSrc=1'b1;
					MemtoReg=1'b0;
					RegWrite=1'b1;
					MemRead=1'b0;
					MemWrite=1'b0;
					BrEq=1'b0;
					BrNeq=1'b0;
					ALUOp=2'b00;// To execute Add in ALU just like any other I-type Instruction
					Jump=1'b0;
					end
					
	endcase

endmodule
///
//#7.ALU Control
module ALUControl(ALUOp,FieldFunc,ALUConInput);
input [1:0]ALUOp;
input [5:0]FieldFunc;
output reg [3:0]ALUConInput;
always@(ALUOp,FieldFunc)
	case(ALUOp)
		2'b00:ALUConInput=4'b0010;
		2'b01:ALUConInput=4'b0110;
		
		2'b10: case(FieldFunc)
					6'b100000:ALUConInput=4'b0010;//ADD
					6'b100010:ALUConInput=4'b0110;//SUB
					6'b100100:ALUConInput=4'b0000;//AND
					6'b100101:ALUConInput=4'b0001;//OR
					6'b101010:ALUConInput=4'b0111;//SLT
			   endcase
	
	endcase
endmodule	
////
//#1.Instruction Memory
module Instr_Mem(PC,Instruction);  
      input [31:0] PC;  
      output wire [31:0] Instruction; 
      wire [4 : 0] rom_addr = PC[6 : 2];  //eta thik korte hobe      
      reg [31:0] rom[31:0];    
      initial  
		  begin  
					rom[0] = 32'd0;
		            rom[1] = 32'b10100001001010010000000000000000;  //A1290000
		            rom[2] = 32'b10100001010010100000000000000001;  //A14A0001
		            rom[3] = 32'b10100001011010110000000000000000;  //A16B0000
		            rom[4] = 32'b10100001100011000000000000010100;  //A18C0014
		            rom[5] = 32'b00000001001010101000100000100000;  //012A8820
		            rom[6] = 32'b10100001010010010000000000000000;  //A1490000
		            rom[7] = 32'b10100010001010100000000000000000;  //A22A0000
		            rom[8] = 32'b10100001011010110000000000000010;  //A16B0002
			        rom[9] = 32'b00010001100010110000000000000001;  //118B0001
			        rom[10] = 32'b10011000000000000000000000000101;  //98000005
			        rom[11] = 32'b10101101100100010000000000000101; //AD910005
			        rom[12] = 32'b00000001100100011001000000101010; //0191902A 
		            rom[13] = 32'd0;  
		            rom[14] = 32'd0;  
		            rom[15] = 32'd0;  
		            rom[16] = 32'd0;
					rom[17] = 32'd0;  
		            rom[18] = 32'd0;  
		            rom[19] = 32'd0;  
		            rom[20] = 32'd0;  
		            rom[21] = 32'd0;  
		            rom[22] = 32'd0; 
		            rom[23] = 32'd0;  
		            rom[24] = 32'd0;  
		            rom[25] = 32'd0;  
		            rom[26] = 32'd0;  
		            rom[27] = 32'd0;  
		            rom[28] = 32'd0;  
		            rom[29] = 32'd0;  
		            rom[30] = 32'd0;  
		            rom[31] = 32'd0;  
		              
      	end  
      	assign Instruction = (PC[31:0] < 125 )? rom[rom_addr]: 32'd0; 
 endmodule  
////
//#8.ALU
module ALU(ALUControl, A, B, ALUResult, ZeroALU);
	input   [3:0]   ALUControl; // control bits for ALU operation instruction 
	input   [31:0]  A, B;	    // inputs 32 bit
	output  reg [31:0]  ALUResult;	// answer of ALU_operation
	output  ZeroALU;	    	
    
    always @(ALUControl,A,B)
    begin
		case (ALUControl)
				0: // AND
					ALUResult <= A & B;
				1: // OR
					ALUResult <= A | B;
				2: // ADD
					ALUResult <= A + B;
				3: // NOR
					ALUResult <= ~(A | B);
				6: // SUB
					ALUResult <= A - B;
				7:  //SLT
					    ALUResult <= A<B ? 1:0;
			default : ALUResult <= 0;//considering default result ZeroALU
		endcase
     end
assign ZeroALU = (ALUResult == 0);// ZeroALU=1 if ALUResult == 0

endmodule
////
//#9.Data Memory
module DataMemory(clk, MemReadEn,MemWriteEn,MemAccessAdr,MemWriteData,MemReadData);
  
	  input clk, MemReadEn, MemWriteEn;   
      input [31:0] MemAccessAdr,MemWriteData;  
      output reg [31:0] MemReadData;

      integer i;  
      reg [31:0] DataMemory [31:0];  
      wire [4 : 0] DataMemory_addr = MemAccessAdr[6 : 2];  //eta thik korte hobe
      initial begin  
           for(i=0;i<32;i=i+1)  
                DataMemory[i] <= 32'd0;  
      end  
      always @(posedge clk) begin  
           if (MemWriteEn)  DataMemory[DataMemory_addr] <= MemWriteData; 
 
           MemReadData <= (MemReadEn==1'b1) ? DataMemory[DataMemory_addr]:32'd0;
      end  
         
 endmodule
////
//#11.PC Update Module
module PCUpdateBLock(currentPC,InsOffset,zeroALU,ConBeq,ConBneq,ConJump,nextPC);

input [31:0]currentPC; // Current PC address
input [25:0]InsOffset; // Last 26 bits of Instruction
input zeroALU,ConBeq,ConBneq,ConJump; // zero result signal from ALU, Branch & Jump signal from Main Control Unit

output reg [31:0]nextPC; // PC will be updated by this value
wire [31:0]BOffsetExtnd; // Holds the 32 bit sign extended Branch offset address (not shifted)

SignExtend Boffset(.X(InsOffset[15:0]),.Y(BOffsetExtnd));// Call the sign extension function 

wire k;
assign k=(zeroALU & ConBeq)||(~zeroALU & ConBneq);// PCSrc Signal


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
//#12.Program Counter
module ProgCount(clk,Reset,iPC,oPC);
input clk;
input Reset;
input [31:0]iPC; // iPC is the output of PCUpdate block
output reg[31:0]oPC; // oPC goes to the Instruction Memory block
always@(posedge clk)
	if(Reset==0) oPC<=32'h0;
	else oPC<=iPC;
endmodule
////
//#4.Register File
module Register(RegWrite,ReadReg1,ReadReg2,ReadData1,ReadData2,WriteReg,WriteData,clk);
input RegWrite;  	//A signal that decides whether we write into file (if 1 -> write)
input [4:0] ReadReg1;	//In case of read, the address of first Register
input [4:0] ReadReg2;	//In case of read, the address of second Register
input [4:0] WriteReg;	//In case of write, the address of the Register in which to write
input [31:0] WriteData;	//In case of write, the value to be written

input clk;

output wire [31:0] ReadData1;	//In case of read, the value of the first Register
output wire [31:0] ReadData2;	//In case of read, the value of the second Register

reg [31:0] RF [31:0];//32bit Register 32 registers

initial 
	begin
		RF[0] = 32'h0;	//Register File initialized with random values
        RF[1] = 32'h0;	
        RF[2] = 32'h0;
        RF[3] = 32'h0;
        RF[4] = 32'h0;
        RF[5] = 32'h0;
        RF[6] = 32'h0;
        RF[7] = 32'h0;
        RF[8] = 32'h0;
        RF[9] = 32'h0;
        RF[10] = 32'h0;
        RF[11] = 32'h0;
        RF[12] = 32'h0;
        RF[13] = 32'h0;
        RF[14] = 32'h0;
        RF[15] = 32'h0;
        RF[16] = 32'h0;
        RF[17] = 32'h0;
        RF[18] = 32'h0;
        RF[19] = 32'h0;
        RF[20] = 32'h0;
        RF[21] = 32'h0;
        RF[22] = 32'h0;
        RF[23] = 32'h0;
        RF[24] = 32'h0;
        RF[25] = 32'h0;
        RF[26] = 32'h0;
        RF[27] = 32'h0;
        RF[28] = 32'h0;
        RF[29] = 32'h0;
        RF[30] = 32'h0;
        RF[31] = 32'h0;
	end
		assign ReadData1 = RF[ReadReg1];
		assign ReadData2 = RF[ReadReg2];

	always@ (posedge clk)
	begin
		if(RegWrite==1'b1)		
		 RF[WriteReg] <= WriteData;
	end
endmodule
////
