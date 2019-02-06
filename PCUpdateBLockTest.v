module PCUpdateBLockTest;

reg [31:0]currentPC; // Current PC address
reg [25:0]InsOffset; // Last 26 bits of instruction
reg zeroALU,ConBeq,ConBneq,ConJump; // zero result signal from ALU, Branch & Jump signal from Main Control Unit

wire [31:0]nextPC; // PC will be updated by this value

initial 
	begin
	$shm_open("shm.db",1);// Opens a wave form database
	$shm_probe("AS");// Saves all signals to database
    #80 $finish;
	#80 $shm_close();// Closes the waveform database
	end

PCUpdateBLock MyPCUpdateBLock(currentPC,InsOffset,zeroALU,ConBeq,ConBneq,ConJump,nextPC);

// Waveform Generation

initial
	begin
    currentPC=32'h00A94FB2;
	InsOffset=26'h3A6F80;
	zeroALU=1;
	ConBeq=0;
	ConBneq=0;
	ConJump=0;
	#10;// nextPC=PC+4 =32'h00A94FB6

    currentPC=32'h00A94FB2;
	InsOffset=26'h3A6F80;
	zeroALU=0;
	ConBeq=0;
	ConBneq=1;
	ConJump=0;
	#10;//nextPC=PC+4+ Shifted Branch offset (32'h0001BE00) =32'h00AB0DB6

    currentPC=32'h00A94FB2;
	InsOffset=26'h3A6F80;
	zeroALU=1;
	ConBeq=0;
	ConBneq=1;
	ConJump=0;
	#10;// nextPC=PC+4 =32'h00A94FB6


    currentPC=32'h00A94FB2;
	InsOffset=26'h3A6F80;
	zeroALU=1;
	ConBeq=1;
	ConBneq=0;
	ConJump=0;
	#10;//nextPC=PC+4+ Shifted Branch offset (32'h0001BE00) =32'h00AB0DB6

    currentPC=32'h00A94FB2;
	InsOffset=26'h3A6F80;
	zeroALU=0;
	ConBeq=0;
	ConBneq=0;
	ConJump=1;
	#10;//nextPC= Jump Address 32'h00E9BE00

	$stop;
	end

endmodule

