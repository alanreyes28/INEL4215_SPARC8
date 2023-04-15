// Condition Handler and Program Status Register testbench
module condition_handler_program_status_reg_testbench;
    reg Z, N, C, V, LE, Clr,Clk;
    wire [3:0] PSR_Out;
    wire bit_C;
    Program_Status_Register psr(PSR_Out, bit_C, Z, N, C, V, LE, Clr, Clk);
    reg [3:0] I28_25;
    reg [3:0] PSR;
    reg ID_B_Instr;
    wire IF_B;
    Condition_Handler ch(IF_B, I28_25, PSR, ID_B_Instr);

    initial #100 $finish;

    initial begin
        Clk = 0;
        forever #2 Clk = ~Clk;
    end

    initial begin
        Clr = 1'b1;
        #3 Clr = 1'b0;
    end

    initial begin
        Z = 1'b1;
        N = 1'b0;
        C = 1'b1;
        V = 1'b1;
        LE = 1'b1;
    end
  
    initial begin
        $display("PSR Results:");
        $monitor("PSR_Out = %b, bit_C = %b", PSR_Out, bit_C);
    end
  
    initial fork 
      #15 ID_B_Instr = 1'b1;
      #15 I28_25 = 4'b1000;
      #15 PSR = 4'b1010;
      
      #17 ID_B_Instr = 1'b0;
      #17 I28_25 = 4'b1000;
      #17 PSR = 4'b1010;
      
      #19 ID_B_Instr = 1'b1;
      #19 I28_25 = 4'b0101;
      #19 PSR = 4'b1000;
      
      #21 ID_B_Instr = 1'b1;
      #21 I28_25 = 4'b0101;
      #21 PSR = 4'b1010;
      
      #22 ID_B_Instr = 1'b1;
      #22 I28_25 = 4'b1010;
      #22 PSR = 4'b0101;
    join

  	initial begin
       #15 $monitor("ID_B_Instr = %b, I28_25 = %b, PSR = %b, IF_B = %b", ID_B_Instr, I28_25, PSR, IF_B);
    end
endmodule


// Program Status Register module
module Program_Status_Register (output reg [3:0] PSR_Out, output reg bit_C,
                                input Z, N, C, V, LE, Clr, Clk);
    always @ (posedge Clk) begin // Rising edge triggered Register
        if(Clr) begin
            PSR_Out <= 4'b0;
            bit_C <= 1'b0;
        end
        else if(LE) begin
            PSR_Out <= {Z,N,C,V}; // Condition Flags
            bit_C <= C;
        end
    end
endmodule

// Condition Handler module
module Condition_Handler (output reg IF_B, input [3:0] I28_25, input [3:0] PSR_Out, input ID_B_Instr);
  	reg Z;
    reg N;
    reg C;
    reg V;
    always @(ID_B_Instr, PSR_Out, I28_25) begin
      	Z = PSR_Out[3];
        N = PSR_Out[2];
        C = PSR_Out[1];
        V = PSR_Out[0];
        if (ID_B_Instr) begin
            case(I28_25)
                4'b1000: IF_B = 1'b1; // Banch always
                4'b0000: IF_B = 1'b0; // Branch never
                4'b1001: IF_B = !Z;   // Branch on not equal
                4'b0001: IF_B = Z;    // Branch on equal
                4'b1010: IF_B = !(Z | (N ^ V)); // Branch on greater
                4'b0010: IF_B = Z | (N ^ V);   // Branch on less or equal
                4'b1011: IF_B = !(N ^ V); // Branch on	greater or equal
                4'b0011: IF_B = N ^ V;    // Branch on	less
                4'b1100: IF_B = !(C | Z); // Branch on greater unsigned
                4'b0100: IF_B = C | Z;    // Branch on less or equal unsigned
                4'b1101: IF_B = !C;  // Branch on	Carry = 0
                4'b0101: IF_B = C;   // Branch on Carry = 1
                4'b1110: IF_B = !N;  // Branch on positive
                4'b0110: IF_B = N;   // Branch on negative
                4'b1111: IF_B = !V;  // Branch overreflow = 0
                4'b0111: IF_B = V;   // Branch overreflow = 1
            endcase
        end
        else begin
            IF_B = 1'b0;
        end
    end
endmodule
