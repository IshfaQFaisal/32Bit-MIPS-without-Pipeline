module MainControlTest;

reg [5:0]Op;
wire RegDst,ALUSrc,MemtoReg,RegWrite,MemRead,MemWrite,BrEq,BrNeq,Jump;
wire [1:0]ALUOp;

initial 
	begin
	$shm_open("shm.db",1);// Opens a wave form database
	$shm_probe("AS");// Saves all signals to database
    #80 $finish;
	#80 $shm_close();// Closes the waveform database
	end

MainControl MyMainControl(Op,RegDst,ALUSrc,MemtoReg,RegWrite,MemRead,MemWrite,BrEq,BrNeq,ALUOp,Jump);

// Waveform Generation

initial
	begin
    Op=6'b000010;
	#10;//Jump

	Op=6'b000110;
	#10;//Branch Not Equal

	Op=6'b001000;
	#10;// Add immediate

	$stop;
	end

endmodule
