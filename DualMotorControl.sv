/////////////////////////////////////////////////
// DualMotorControl.sv
// HMC E155 15 November 2016
// Jeewan_Naik@hmc.edu
/////////////////////////////////////////////

/////////////////////////////////////////////
// testbench
//   Tests motor control through SPI input
/////////////////////////////////////////////

module testbench();
    logic clk, rst, load, sck, sdi, aziEn, polEn, polEncoder, AziEncoder, emergencyStop, isPolDisplay, comp_relay, solenoid_relay;
	 logic [1:0] aziCTL;
	 logic [1:0] polCTL;
    logic [7:0] azi, polar, f, azi2, polar2, f2, LEDs;
	 logic [1:0] result, expected;
	 logic [23:0] comb, comb2;
    logic [8:0] i;
	 logic run;
    
    // device under test
    DualMotorControl dut(clk, rst, sck, sdi, load, AziEncoder, polEncoder, isPolDisplay, emergencyStop, result[1], result[0], aziEn, polEn, aziCTL, polCTL, comp_relay, solenoid_relay, LEDs);
    
    // test case
    initial begin   
        azi       <= 8'b00001010;
        polar 		<= 8'b00011010;
		  f			<= 8'b00001010;
		  azi2      <= 8'b00000010;
        polar2 	<= 8'b00000010;
		  f2			<= 8'b00101000;
		  
        expected  <= 2'b11;
    end
    
    // generate clock and load signals
    initial 
        forever begin
            clk = 1'b0; #5;
            clk = 1'b1; #5;
        end
        
	 initial
			forever begin
				polEncoder = 1'b1;
				AziEncoder = 1'b1; #1000;
				polEncoder = 1'b0; 
				AziEncoder = 1'b0; #9000;
			end
	 
    initial begin
      i = 0;
		run = 0;
      load = 1'b0;
	emergencyStop = 1'b0;
	isPolDisplay = 1'b1;
		rst = 1; #10;
		rst = 0; #10;
    end 
    
	assign comb = {azi, polar, f};
	assign comb2 = {azi2, polar2, f2};
    // shift in test vectors, wait until done, and shift out result
    always @(posedge clk) begin
      if (i == 24) load = 1'b1;
      if (i<24 && !run) begin
        #1; sdi = comb[23-i];
        #1; sck = 1; #5; sck = 0;
        i = i + 1;
		  if (result & !comp_relay) load = 1'b0;
			end
		else if (i<24 && run) begin
        #1; sdi = comb2[23-i];
        #1; sck = 1; #5; sck = 0;
        i = i + 1;
		  if (result & comp_relay) load = 1'b0;
			end
		else if ((result == expected) && (run==0) && (i==24)) begin
                $display("Position Aquired");
				i = 0; run = 1; load = 1'b0; #10000; 
			end
		else if ((result == expected) && (i==24)) begin
				$display("Testbench ran successfully");
				#10000; $stop();
			end
    end
    
endmodule

/////////////////////////////////////////////
// DualMotorControl
// 	Top module which calls the SPI module and two motor control modules
/////////////////////////////////////////////

module DualMotorControl(input  logic clk,
								input  logic rst,
								input  logic sck, 
								input  logic sdi,
								input  logic load,
								input  logic aziEncoder,
								input  logic polEncoder,
								input  logic isPolDisplay,
								input  logic emergencyStop,
								output logic aziDone, polDone,
								output logic realAziEn,
								output logic realPolEn,
								output logic [1:0] aziCTL,
								output logic [1:0] polCTL,
								output logic comp_relay,
								output logic solenoid_relay,
								output logic [7:0] LEDs);
                    
    logic [7:0] aziPosition, currentAziPos, polPosition, currentPolPos, forceCount;
	 logic [1:0] aziState, polState;
	 logic aziEn, polEn;
	 
	 assign realAziEn = ~emergencyStop & aziEn;
	 assign realPolEn = ~emergencyStop & polEn;
	 assign LEDs = (isPolDisplay)? {currentAziPos[5:0], aziState}: {currentPolPos[5:0], polState};
	 
    dmc_spi spi(sck, sdi, aziPosition, polPosition, forceCount);   
    //dmc_azi azi(clk, rst, load, aziPosition, aziDone, aziCTL);
	 dmc_pol azi(clk, rst, load, aziPosition, aziEncoder, currentAziPos, aziState, aziDone, aziEn, aziCTL);
	 dmc_pol pol(clk, rst, load, polPosition, polEncoder, currentPolPos, polState, polDone, polEn, polCTL);
	 
	 logic solActivate;
	 assign solActivate = aziDone & polDone & !comp_relay;
	 relay solenoid  (clk, rst, solActivate, 8'h05, solenoid_relay);
	 
	 logic compActivate;
	 assign compActivate = load;
	 relay compressor(clk, rst, compActivate, forceCount, comp_relay);
	 
endmodule

/////////////////////////////////////////////
// dmc_spi
//   SPI interface.  Shifts in the azimuthal and polar position
//   Doesn't bother sending any information (can be used later for debugging)
/////////////////////////////////////////////

module dmc_spi(input  logic sck, 
               input  logic sdi,
               output logic [7:0] aziPosition, polPosition, forceCount);
    // apply 16 sclks to shift in aziPosition and polPosition, starting with aziposition[7]
    always_ff @(posedge sck)
		{aziPosition, polPosition, forceCount} <= {aziPosition[6:0], polPosition, forceCount, sdi};
endmodule
					
/////////////////////////////////////////////
// dmc_pol
//   Top module for polar bidirectional DC motor control
//		with encoder feedback
/////////////////////////////////////////////
					
module dmc_pol(input logic clk,
					input logic rst,
					input logic load,
					input logic [7:0] polPosition,
					input logic encoder,
					output logic [7:0] currentPos,
					output logic [1:0] stateLights,
					output logic polDone,
					output logic polEn,
					output logic [1:0] polCTL);
	//logic [7:0] currentPos;//possible values: 0-95 (ie. 96 revolutions from fully up to fully down.
	logic [4:0] pulseCount; //counts the number of pulses (reset each revolution/24 pulses)
	logic direction; //0 is up, 1 is down
	logic slowclk;
	logic pulse, oldPulse;
	logic [17:0] count;
	
	//Define how many values there will be in the new variable type (statetype)
	typedef enum logic [1:0] {S0, S1, S2, S3} statetype;
	statetype state, nextState;
	
	assign stateLights = state;
	
	//2^17 clock divider to make 152.6 Hz.  This is to avoid bouncing on the encoder
	always_ff@(posedge clk, posedge rst)
		if(rst) count <= 0;
		else count <= count + 1;
	assign slowclk = count[17]; //Change to 17 for real life

	always_ff@(posedge slowclk, posedge rst)
	begin
		if(rst)
			begin
				currentPos <= 0;
				state <= S0;
				pulseCount <= 0;
			end
		else 
			begin
				state <= nextState;
				oldPulse <= pulse;
				
				if ({oldPulse, pulse} == 2'b01) //Pulses are recorded when A on the encoder goes from low to high
					if (pulseCount<23)
						pulseCount <= pulseCount + 1;
					else
						begin
							pulseCount <= 0;
							if (direction)
								currentPos <= currentPos - 1;
							else
								currentPos <= currentPos + 1;
						end
			end
	end
	
	always_comb
		case(state)
			S0: 	if (load) nextState = S1;
					else nextState = S0;
			S1: 	if (currentPos == polPosition) nextState = S3;
					else if (encoder) nextState = S2;
					else nextState = S1;
			S2: 	if (currentPos == polPosition) nextState = S3;
					else if (!encoder) nextState = S1;
					else nextState = S2;
			S3:	if (!load) nextState = S0;
					else nextState = S3;
			default: nextState = S0;
		endcase
	
	always_comb
		case(state)
			S0:	{polDone, direction, polEn, pulse} = 4'b0000;
			S1:	{polDone, direction, polEn, pulse} = {1'b0, (currentPos>polPosition), 2'b10};
			S2:	{polDone, direction, polEn, pulse} = {1'b0, (currentPos>polPosition), 2'b11};
			S3:	{polDone, direction, polEn, pulse} = 4'b1000;
			default: {polDone, direction, polEn, pulse} = 4'b0000;
		endcase
			
	assign polCTL = (direction)? 2'b01: 2'b10;
endmodule

/////////////////////////////////////////////
// relay
//   Top module for electromechanical relay control.
/////////////////////////////////////////////
module relay(input  logic       clk,
				 input  logic       rst,
				 input  logic       activate,
				 input  logic [7:0] delay,
				 output logic       openRelay);
	
	logic [24:0] unitCounter;
	logic [7:0]  timeCounter;
	logic        isActivated, slowclk;
	
	always_ff@(posedge clk, posedge rst)
		if(rst) unitCounter <= 0;
		else unitCounter <= unitCounter	+ 1;
	assign slowclk = unitCounter[24]; //Change to 24 for real life
	
	always_ff@(posedge slowclk, posedge rst)
		if(rst) begin
			isActivated <= 0;
			timeCounter <= 0;
		end 
		else begin
			if(activate) begin
				if(timeCounter == delay) isActivated <= 0;
				else begin
					isActivated <= 1;
					timeCounter <= 1 + timeCounter;
				end
			end 
			else begin
				isActivated <= 0;
				timeCounter <= 0;
			end
		end
	assign openRelay = isActivated;
endmodule