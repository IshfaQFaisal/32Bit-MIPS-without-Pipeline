module MIPS_Test;

reg clk,Reset;
wire [31:0]ALUResult;

initial 
	begin
	$shm_open("shm.db",1);// Opens a wave form database
	$shm_probe("AS");// Saves all signals to database
    #800 $finish;
	#800 $shm_close();// Closes the waveform database
	end

MIPS MyMIPS(clk,Reset,ALUResult);

// Clock Definition

initial 
	begin
	clk=1'b0;
	forever #5 clk=~clk;
	end

// Waveform Generation

initial
	begin
	
	Reset=0;
	#10;
	Reset=1;
	#10;
	#700;
	$stop;
	end
endmodule
