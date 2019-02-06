module MainControl(Op,RegDst,ALUSrc,MemtoReg,RegWrite,MemRead,MemWrite,BrEq,BrNeq,ALUOp,Jump);

input [5:0]Op;
output reg RegDst,ALUSrc,MemtoReg,RegWrite,MemRead,MemWrite,BrEq,BrNeq,Jump;
output reg [1:0]ALUOp;

parameter   RformatOp=6'b000000,
			LoadOp=6'b100011,
			StoreOp=6'b101011,
			BeqOp=6'b000100,
			BneqOp=6'b000110,
			JumpOp=6'b000010,
			AddiOp=6'b001000;

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
					ALUOp=2'b00;// To execute Add in ALU just like any other I-type instruction
					Jump=1'b0;
					end
					
	endcase

endmodule
