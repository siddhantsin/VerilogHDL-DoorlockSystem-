module doorlock(SW, CLOCK_25, CLOCK_50, GPIO, HEX0, HEX1, HEX2, HEX3, HEX4, HEX5, LEDR, LEDG);

	input CLOCK_25, CLOCK_50;
	inout [35:0] GPIO;
	output [6:0] HEX0, HEX1, HEX2, HEX3, HEX4, HEX5;
	output [17:0] LEDR;
	input [17:0] SW;
	output [8:0] LEDG;

	//wire led_on=1'b1;
	wire [25:0]sensor_output;
	wire binary_Signal;8
	wire unlock_Signal;
	wire Clk_2hz;
	wire resetn;
	wire [8:0]state_Signal;
	wire [6:0]hex_output0;
	wire [6:0]hex_output1;
	wire [6:0]hex_output2;
	wire [6:0]hex_output3;
	wire [6:0]hex_output4;
	wire [6:0]hex_output5;



	usensor sensor(.distance(sensor_output),
						.trig(GPIO[0]),
						.echo(GPIO[1]),
						.clock(CLOCK_50)
						);
 
	unlock_signal_binary sig(.inp(sensor_output),
				     .signal(binary_Signal),
						 .clock(Clk_2hz)
						 );
					
	two_hz_clk cclk(.inputclock(CLOCK_50),
			   .outputclock(Clk_2hz)
			   );
				 
	unlock_signal_fsm fsm(.binary_signal(binary_Signal),
				  .clock1(Clk_2hz),
				  .clock2(CLOCK_50),
				  .selector_bits(SW[17:9]),
				  .resetn(resetn),
				  .unlock_signal(unlock_Signal),
				  .state_signal(state_Signal)
				  );
				  
	hexout hex(.unlock_signal(unlock_Signal),
				.hex_output0(hex_output0),
				.hex_output1(hex_output1),
				.hex_output2(hex_output2),
				.hex_output3(hex_output3),
				.hex_output4(hex_output4),
				.hex_output5(hex_output5)
				);
				
	assign HEX0 = hex_output0;
	assign HEX1 = hex_output1;
	assign HEX2 = hex_output2;
	assign HEX3 = hex_output3;
	assign HEX4 = hex_output4;
	assign HEX5 = hex_output5;
	assign resetn = SW[0];
	assign LEDG[0] = Clk_2hz;
	assign LEDG[3]=binary_Signal;
	assign LEDR[8:0]=state_Signal;
	assign LEDG[8] = unlock_Signal;

endmodule






module usensor(distance, trig, echo, clock);
  input clock, echo;
  output reg [25:0] distance;
  output reg trig;

  reg [25:0] master_timer;
  reg [25:0] trig_timer;
  reg [25:0] echo_timer;
  reg [25:0] echo_shift10;
  reg [25:0] echo_shift12;
  reg [25:0] temp_distance;
  reg echo_sense, echo_high;

  localparam  TRIG_THRESHOLD = 14'b10011100010000,
              MASTER_THRESHOLD = 26'b10111110101111000010000000;


  always @(posedge clock)
  begin
    if (master_timer == MASTER_THRESHOLD)
		begin
        master_timer <= 0;
		  
		  end
    else if (trig_timer == TRIG_THRESHOLD || echo_sense)
      begin
        trig <= 0;
        echo_sense <= 1;
        if (echo)
			   			    begin
					echo_high <= 1;
					echo_timer <= echo_timer + 1;
					//////////////////////////////////////////////////////
					// CLOCK_50 -> 50 000 000 clock cycles per second
					// let n = number of cycles
					// speed of sound in air: 340m/s
					// n / 50 000 000 = num of seconds
					// num of seconds * 340m/s = meters
					// meters * 100 = cm ~ distance to object and back
					// So we divide by 2 to get distance to object
					// 1/ 50 000 000 * 340 * 100 / 2 = 0.00034
					// n * 0.00034 = n * 34/100 000 = n / (100 000/34)
					// = 2941
					// To make up for sensor inaccuracy and simple math
					// we round down to 2900
					temp_distance <= (echo_timer / 2900);
					//////////////////////////////////////////////////////
			    end
        else
          begin
				distance <= temp_distance + 2'd2;
				echo_timer <= 0;
				trig_timer <= 0;
				echo_sense <= 0;
          end
      end
    else
	   begin
      trig <= 1;
      trig_timer <= trig_timer + 1;
      master_timer <= master_timer + 1;
    end
  end

endmodule



module unlock_signal_binary(input [10:0]inp, 
                           output reg signal,
						   input clock);
	
	always @(posedge clock)
	begin
		if (inp < 10'd10)
			begin
			signal <= 1'b1;
			end
		else
			begin
			signal <= 1'b0;
			end
	end
endmodule



module two_hz_clk(input inputclock, 
                   output reg outputclock);
	
	reg [24:0] counter;
	initial begin
		counter = 0;
		outputclock = 0;
	end
	always @(posedge inputclock) begin
		if (counter == 0) begin
			counter <= 19999999;
			outputclock <= ~outputclock;
		end else begin
			counter <= counter -1;
		end
	end
endmodule



module unlock_signal_fsm(input binary_signal, 
								input clock1,
								input clock2,
							   input	resetn, 
							   input [8:0] selector_bits,
								output unlock_signal,
								output [8:0]state_signal);

	
	
	

    
	 
	 reg [8:0] y_Q; 
	 reg [8:0]Y_D; // y_Q represents current state, Y_D represents next state

    // These are arbitrary and are our flip-flop values for our states
    localparam A = 9'b000000000, B = 9'b000000001, C = 9'b000000011, D = 9'b000000110, E = 9'b000001101, F = 9'b000011011, G = 9'b000110111, H = 9'b001101110, I = 9'b011011100, J = 9'b110111001;

	localparam FSM_1=9'b110111001,FSM_2=9'b101000110,FSM_3=9'b110001110,FSM_4=9'b100110001;

    //State table
    //The state table should only contain the logic for state transitions`	 
    //Do not mix in any output logic. The output logic should be handled separately.
    //This will make it easier to read, modify and debug the code.
    always@(*)  //MAKE CHANGES TO THIS ALWAYS BLOCK....
    begin: state_table
    case(selector_bits)
		FSM_1:case (y_Q)
            A: begin // If current state is A, if 0 go back to A, if 1 go to B
                   if (!binary_signal) Y_D <= A;
                   else Y_D <= B;
               end
            B: begin // If current state is B, if 0 go to C, if 1 go to B
                   if(!binary_signal) Y_D <= A;
                   else Y_D <= C;
               end
            C:  begin // If current state is C, if 0 go to E, if 1 go to D
                   if(!binary_signal) Y_D <= D;
                   else Y_D <= A;
                end
            D: begin // If 0 go to E, else go to B
                    if(!binary_signal) Y_D <= A;
                    else Y_D <= E;
               end
            E: begin
                    if(!binary_signal) Y_D <= A;
                    else Y_D <= F;
               end
			F: begin
                    if(!binary_signal) Y_D <= A;
                    else Y_D <= G;
               end
			G: begin
                    if(!binary_signal) Y_D <= H;
                    else Y_D <= A;
               end
			H: begin
                    if(!binary_signal) Y_D <= I;
                    else Y_D <= A;
               end
			I: begin
                    if(!binary_signal) Y_D <= A;
                    else Y_D <= J;
               end
			J: begin
                    if(!binary_signal) Y_D <= J;
                    else Y_D <= J;
               end
            default: Y_D = A;
        endcase
		FSM_2:case (y_Q) //101000110
            A: begin // If current state is A, if 0 go back to A, if 1 go to B
                   if (!binary_signal) Y_D <= A;
                   else Y_D <= B;
               end
            B: begin // If current state is B, if 0 go to C, if 1 go to B
                   if(!binary_signal) Y_D <= C;
                   else Y_D <= A;
               end
            C:  begin // If current state is C, if 0 go to D, if 1 go to B
                   if(!binary_signal) Y_D <= A;
                   else Y_D <= D;
                end
            D: begin // If 1 go to E, else go to B
                    if(!binary_signal) Y_D <= E;
                    else Y_D <= A;
               end
            E: begin
                    if(!binary_signal) Y_D <= F;
                    else Y_D <= A;
               end
				F: begin
                    if(!binary_signal) Y_D <= G;
                    else Y_D <= A;
               end
			G: begin
                    if(!binary_signal) Y_D <= A;
                    else Y_D <= H;
               end
			H: begin
                    if(!binary_signal) Y_D <= A;
                    else Y_D <= I;
               end
			I: begin
                    if(!binary_signal) Y_D <= J;
                    else Y_D <= A;
               end
			J: begin
                    if(!binary_signal) Y_D <= J;
                    else Y_D <= J;
               end
            default: Y_D = A;
        
        
           endcase
		FSM_3:case (y_Q) //110001110
            A: begin // If current state is A, if 0 go back to A, if 1 go to B
                   if (!binary_signal) Y_D <= A;
                   else Y_D <= B;
               end
            B: begin // If current state is B, if 0 go to C, if 1 go to B
                   if(!binary_signal) Y_D <= A;
                   else Y_D <= C;
               end
            C:  begin // If current state is C, if 0 go to E, if 1 go to D
                   if(!binary_signal) Y_D <= D;
                   else Y_D <= A;
                end
            D: begin // If 0 go to E, else go to B
                    if(!binary_signal) Y_D <= E;
                    else Y_D <= A;
               end
            E: begin
                    if(!binary_signal) Y_D <= F;
                    else Y_D <= A;
               end
			F: begin
                    if(!binary_signal) Y_D <= A;
                    else Y_D <= G;
               end
			G: begin
                    if(!binary_signal) Y_D <= A;
                    else Y_D <= H;
               end
			H: begin
                    if(!binary_signal) Y_D <= A;
                    else Y_D <= I;
               end
			I: begin
                    if(!binary_signal) Y_D <= J;
                    else Y_D <= A;
               end
			J: begin
                    if(!binary_signal) Y_D <= J;
                    else Y_D <= J;
               end
            default: Y_D = A;
        endcase
		FSM_4:case (y_Q) //100110001
            A: begin // If current state is A, if 0 go back to A, if 1 go to B
                   if (!binary_signal) Y_D <= A;
                   else Y_D <= B;
               end
            B: begin // If current state is B, if 0 go to C, if 1 go to B
                   if(!binary_signal) Y_D <= C;
                   else Y_D <= A;
               end
            C:  begin // If current state is C, if 0 go to E, if 1 go to D
                   if(!binary_signal) Y_D <= D;
                   else Y_D <= A;
                end
            D: begin // If 0 go to E, else go to B
                    if(!binary_signal) Y_D <= A;
                    else Y_D <= E;
               end
            E: begin
                    if(!binary_signal) Y_D <= A;
                    else Y_D <= F;
               end
			F: begin
                    if(!binary_signal) Y_D <= G;
                    else Y_D <= A;
               end
			G: begin
                    if(!binary_signal) Y_D <= H;
                    else Y_D <= A;
               end
			H: begin
                    if(!binary_signal) Y_D <= I;
                    else Y_D <= A;
               end
			I: begin
                    if(!binary_signal) Y_D <= A;
                    else Y_D <= J;
               end
			J: begin
                    if(!binary_signal) Y_D <= J;
                    else Y_D <= J;
               end
            default: Y_D = A;
        endcase
     // state_table
	
	endcase
		end
    // State Registers
    always @(posedge clock1)
    begin: state_FFs
        if(resetn == 1'b0)
            y_Q <=  A; // Should set reset state to state A
        else
            y_Q <= Y_D;
    end // state_FFS

    // Output logic
    // Set out_light to 1 to turn on LED when in relevant states
    // F = 1111, G = 1101
	 
    assign unlock_signal = (y_Q == J) ;
	 assign state_signal=y_Q;
	
endmodule



module hexout(input unlock_signal, output reg [6:0]hex_output0, output reg [6:0]hex_output1, output reg [6:0]hex_output2,
					output reg [6:0]hex_output3, output reg [6:0]hex_output4, output reg [6:0]hex_output5);
	always@(*)
	begin
	if (unlock_signal)
	begin
		hex_output0 = 7'b0001001;
		hex_output1 = 7'b1000110;
		hex_output2 = 7'b1000000;
		hex_output3 = 7'b1000111;
		hex_output4 = 7'b1001000;
		hex_output5 = 7'b1000001;
	end

	if  (~unlock_signal)
	begin
		hex_output0 = 7'b0001001;
		hex_output1 = 7'b1000110;
		hex_output2 = 7'b1000000;
		hex_output3 = 7'b1000111;
		hex_output4 = 7'b1111111;
		hex_output5 = 7'b1111111;
	end
	end
endmodule




