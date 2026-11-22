module mini_ascon_fpga (
    input clk,
    input reset,
    input start,
    input [3:0] IV,
    input [3:0] KeyH,
    input [3:0] KeyL,
    input [3:0] NonceH,
    input [3:0] NonceL,
    input [3:0] Plaintext,
    output reg [3:0] Ciphertext,
    output reg [7:0] Tag,
    output reg done
);

    // Internal registers
    reg [3:0] x0, x1, x2, x3, x4;
    reg [3:0] round;
    reg [2:0] state;
	 reg [3:0] t0, t1, t2, t3, t4;
	 

    // FSM states
    localparam IDLE = 0, INIT = 1, PERMUTE = 2, ABSORB = 3, FINALIZE = 4, DONE = 5;

    always @(posedge clk or posedge reset) begin
        if (reset) begin
		  // Reset all registers
            x0 <= 0; x1 <= 0; x2 <= 0; x3 <= 0; x4 <= 0;
            Ciphertext <= 0;
            Tag <= 0;
            done <= 0;
            round <= 0;
            state <= IDLE;
        end else begin
            case (state)
				   IDLE: if (start) begin
                    x0 <= IV;
                    x1 <= KeyH;
                    x2 <= KeyL;
                    x3 <= NonceH;
                    x4 <= NonceL;
                    round <= 0;
                    state <= INIT;
                end
					 
                INIT: begin
					 if (round < 12) begin
										// Step 1: Round constant
						  t0 = x0;
						  t1 = x1;
						  t2 = x2 ^ 4'b0010; // apply round constant early
						  t3 = x3;
						  t4 = x4;

						  // Step 2: Nonlinear S-box
						  t0 = t0 ^ (t4 & t1);
						  t1 = t1 ^ (t0 | t2);
						  t2 = t2 ^ (t1 & t3);
						  t3 = ~(t3 ^ t2);
						  t4 = t4 ^ t0;
						  // Step 3: Rotation
						  x0 <= {t0[0], t0[3:1]};
						  x1 <= {t1[1:0], t1[3:2]};
						  x2 <= {t2[0], t2[3:1]};
						  x3 <= {t3[2:0], t3[3]};
						  x4 <= {t4[0], t4[3:1]};

						  round <= round + 1;
					 end else begin
						  x3 <= x3 ^ KeyH;
						  x4 <= x4 ^ KeyL;
						  state <= ABSORB;
						  end

										
                end

                ABSORB: begin
                    Ciphertext <= x0 ^ Plaintext;  // Encrypt
                    x0 <= Plaintext;
                    round <= 0;
                    state <= FINALIZE;                end

                FINALIZE: begin
							if (round < 6) begin
								  t0 = x0;
								  t1 = x1;
								  t2 = x2 ^ 4'b0010;
								  t3 = x3;
								  t4 = x4;

								  t0 = t0 ^ (t4 & t1);
								  t1 = t1 ^ (t0 | t2);
								  t2 = t2 ^ (t1 & t3);
								  t3 = ~(t3 ^ t2);
								  t4 = t4 ^ t0;x0 <= {t0[0], t0[3:1]};
								  x1 <= {t1[1:0], t1[3:2]};
								  x2 <= {t2[0], t2[3:1]};
								  x3 <= {t3[2:0], t3[3]};
								  x4 <= {t4[0], t4[3:1]};

								  round <= round + 1;
							 end else begin
								  x1 <= x1 ^ KeyH;
								  x2 <= x2 ^ KeyL;
								  Tag <= {x3 ^ KeyH, x4 ^ KeyL};
								  done <= 1;
								  state <= DONE;
							 end
                end

                DONE: done <= 1;

            endcase
        end
    end
endmodule

// encryption testbench code

`timescale 1ns / 1ps

module tb_ascon4;

	// Inputs
	reg clk;
	reg reset;
	reg start;
	reg [3:0] IV;
	reg [3:0] KeyH;
	reg [3:0] KeyL;
	reg [3:0] NonceH;
	reg [3:0] NonceL;
	reg [3:0] Plaintext;
// Outputs
	wire [3:0] Ciphertext;
	wire [7:0] Tag;
	wire done;

	// Instantiate the Unit Under Test (UUT)
	mini_ascon_fpga uut (
		.clk(clk), 
		.reset(reset), 
		.start(start), 
		.IV(IV), 
		.KeyH(KeyH), 
		.KeyL(KeyL), 
		.NonceH(NonceH), 
		.NonceL(NonceL), 
		.Plaintext(Plaintext), 
		.Ciphertext(Ciphertext), 
		.Tag(Tag), 
		.done(done)
	);
// Clock generation — 10ns period (100MHz)
	always #5 clk = ~clk;

	initial begin
		// Initialize Inputs
		clk = 0;
		reset = 1;   // Start with reset active
		start = 0;
		IV = 4'b0000;
		KeyH = 4'b0000;
		KeyL = 4'b0000;
		NonceH = 4'b0000;
		NonceL = 4'b0000;
		Plaintext = 4'b0000;
// Wait for 20 ns then release reset
		#20 reset = 0;

		// Apply Input Test Vectors
		#10;
		IV        = 4'b1010;   // Initialization vector
		KeyH      = 4'b1100;   // Upper key nibble
		KeyL      = 4'b0011;   // Lower key nibble
		NonceH    = 4'b1111;   // Upper nonce
		NonceL    = 4'b0101;   // Lower nonce
		Plaintext = 4'b0110;   // Data input
// Start encryption
		#10 start = 1;
		#10 start = 0;

		// Wait for done signal to go high
		wait(done == 1);
		#10;

		$display("==========================================");
		$display("Encryption Complete");
		$display("Ciphertext = %b (Hex = %h)", Ciphertext, Ciphertext);
		$display("Tag        = %b (Hex = %h)", Tag, Tag);
		$display("==========================================");
		#20 ;
	end
      
endmodule

// below is the corresponding ascon decryption code


module mini_ascon_decrypt (
    input clk,
    input reset,
    input start,
    input [3:0] IV,
    input [3:0] KeyH,
    input [3:0] KeyL,
    input [3:0] NonceH,
    input [3:0] NonceL,
    input [3:0] Ciphertext,
    input [7:0] Tag_in,       // tag from encryption (optional for verification)
    output reg [3:0] Plaintext,
    output reg Tag_match,     // tag verification flag
    output reg done
);

    // Internal registers
    reg [3:0] x0, x1, x2, x3, x4;
    reg [3:0] round;
    reg [2:0] state;
    reg [3:0] t0, t1, t2, t3, t4;

    // FSM states
    localparam IDLE = 0, INIT = 1, RECOVER = 2, FINALIZE = 3, VERIFY = 4, DONE = 5; always @(posedge clk or posedge reset) begin
        if (reset) begin
            x0 <= 0; x1 <= 0; x2 <= 0; x3 <= 0; x4 <= 0;
            Plaintext <= 0;
            Tag_match <= 0;
            round <= 0;
            done <= 0;
            state <= IDLE;
        end else begin            case (state)
                // Initialize registers
                IDLE: if (start) begin
                    x0 <= IV;
                    x1 <= KeyH;
                    x2 <= KeyL;
                    x3 <= NonceH;
                    x4 <= NonceL;
                    round <= 0;
                    state <= INIT;
                end
  // Perform same 12 rounds as encryption
                INIT: begin
                    if (round < 12) begin
                        x2 <= x2 ^ 4'b0010;
                        t0 = x0 ^ (x4 & x1);
                        t1 = x1 ^ (x0 | x2);
                        t2 = x2 ^ (x1 & x3);
                        t3 = ~(x3 ^ x2);
                        t4 = x4 ^ x0; x0 <= {t0[0], t0[3:1]};
                        x1 <= {t1[1:0], t1[3:2]};
                        x2 <= {t2[0], t2[3:1]};
                        x3 <= {t3[2:0], t3[3]};
                        x4 <= {t4[0], t4[3:1]};
                        round <= round + 1;
                    end else begin
                        x3 <= x3 ^ KeyH;
                        x4 <= x4 ^ KeyL;
                        state <= RECOVER;
                    end
                end  // Recover plaintext
                RECOVER: begin
                    Plaintext <= Ciphertext ^ x0;  // reverse of encryption
                    x0 <= Plaintext;
                    round <= 0;
                    state <= FINALIZE;
                end
                // Perform same 6 final rounds (optional)
                FINALIZE: begin
                    if (round < 6) begin
                        x2 <= x2 ^ 4'b0010;
                        t0 = x0 ^ (x4 & x1);
                        t1 = x1 ^ (x0 | x2);
                        t2 = x2 ^ (x1 & x3);
                        t3 = ~(x3 ^ x2);
                        t4 = x4 ^ x0;
                        x0 <= {t0[0], t0[3:1]};
                        x1 <= {t1[1:0], t1[3:2]};
                        x2 <= {t2[0], t2[3:1]};
                        x3 <= {t3[2:0], t3[3]};
                        x4 <= {t4[0], t4[3:1]};
                        round <= round + 1;
                    end else begin
                        state <= VERIFY;
                    end
                end

                // Verify tag (optional)
                VERIFY: begin
                    if (Tag_in == {x3 ^ KeyH, x4 ^ KeyL})
                        Tag_match <= 1;
                    else
                        Tag_match <= 0;
                    done <= 1;
                    state <= DONE;
                end

                DONE: done <= 1;
            endcase
        end
    end
