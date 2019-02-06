module ALUControlTest;

reg [1:0]ALUop;
reg [5:0]FieldFunc;
wire [3:0]ALUConInput;

initial 
	begin
	$shm_open("shm.db",1);// Opens a wave form database
	$shm_probe("AS");// Saves all signals to database
    #80 $finish;
	#80 $shm_close();// Closes the waveform database
	end

ALUControl ALUControl(ALUop,FieldFunc,ALUConInput);

// Waveform Generation

initial
	begin
    ALUop=2'b01;
	FieldFunc=6'b100010;
	#10;

	ALUop=2'b00;
	FieldFunc=6'b100010;
	#10;

	ALUop=2'b10;
	FieldFunc=6'b100101;
	#10;

	$stop;
	end

endmodule
