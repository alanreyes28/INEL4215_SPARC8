// Testign bench for ALU & Source Operand2 Handler modules
module test_alu_src_op2_hand_sparc_component;
    // ALU parameters
    reg [31:0] A, B;   // Operands
    reg [3:0] OP;      // Opcode
    reg Cin;           // Carry in
    wire [31:0] Out;   // Result 
    wire Z, N, C, V;   // Flags
    alu_sparc_component aluComponent(Out, Z, N, C, V, A, B, OP, Cin);
    
    // Source Operand 2 Hanlder parameters
    reg [31:0] R;      // Input number
    reg [21:0] Imm;    // Immediate
    reg [3:0] Is;      // Instructions
    wire [31:0] Nout;  // Name to avoid using N as the previous flag
    source_operand2_handler_sparc_component srcOp2HandlerComp(Nout, R, Imm, Is);
    
    initial #3000 $finish;
    initial begin
        // ALU opcodes
        OP = 4'b0000;
        repeat (15) #20 OP = OP + 4'b0001;
    end
    
    initial fork
        // Inputs for ALU
        // Overflow Inputs (V=1) for Addition without carry
        A = 32'b11001000111011001110110010001001;
        B = 32'b10001010001100000100100101101000;
        // No Overflow Inputs (V=0) for Addition without carry
        #10 A = 32'b00000000000000000000000000000000;
        #10 B = 32'b00000000000000000000000000000000;
        // Overflow Inputs (V=1) for Addition with carry
        #20 A = 32'b11001000111011001110110010001101;
        #20 B = 32'b10001010001100000100100101101001;
        #20 Cin = 1;
        // No Overflow Inputs (V=0) for Addition with carry
        #30 A = 32'b01001000111011001110110010001001;
        #30 B = 32'b10001010001100000100100101101011;
        #30 Cin = 0;
        // Overflow Inputs (V=1) for Subtraction without carry
        #40 A = 32'b01110000000000000001111000000000;
        #40 B = 32'b10100000000111100000000000000000;
        // No Overflow Inputs (V=0) for Subtraction without carry
        #50 A = 32'b11001000111011001110110010001001;
        #50 B = 32'b00001010001100000100100101101000;
        // Overflow Inputs (V=1) for Subtraction with carry
        #60 A = 32'b11001000111011001001010010001001;
        #60 B = 32'b01001010000011000100100101101000;
        #60 Cin = 1;
        // No Overflow Inputs (V=0) for Subtraction with carry
        #70 A = 32'b00101101010110111010111001101100;
        #70 B = 32'b00001010101101001010011101100011;
        #70 Cin = 0;
        // Inputs for logic operations
        #80 A = 32'b10001011100111001101001010001010;
        #80 B = 32'b00100101111000101010010011000101;
        // Inputs for rest of operations
        #200 A = 32'b10001011100111001101001010001010;
        #200 B = 32'b00000000000000000000000000001010;
    join
    
    initial begin
        $display("ALU Results:");
        $monitor("OP (binary) = %b | A (binary) = %b | A (decimal) = %d | B (binary) = %b | B (decimal) = %d |Out (binary) = %b | Out (decimal) = %d | Z (binary) = %b | N (binary) = %b | C (binary) = %b | V (binary) = %b", OP, A, A, B, B, Out, Out, Z, N, C, V);
    end
    
    initial begin
        // Source Operand2 Handler instructions
        #320 Is = 4'b0000;
        #320 repeat (15) #10 Is = Is + 4'b0001;
        #500 Is = 4'b1000;
        #500 Imm = 22'b1000110000000100010011;
        #500 repeat (7) #10 Is = Is + 4'b0001;
    end
    
    initial fork
        // Inputs for source operan2 handler
        #320 R = 32'b11100000000000000000000000000011;
        #320 Imm = 22'b1000110001000100010011;
    join
    
    initial begin
        #320 $display("\nSource Operand2 Handler results:");
        #320 $monitor("Is (binary) = %b | N (binary) = %b", Is, Nout);
        #500 $display("Results with new Imm value and Is 8-15 combination:");
    end
endmodule

// Module implementation of ALU SPARC component
module alu_sparc_component (output reg [31:0] Out, output reg Z, N, C, V, 
                            input [31:0] A, B, input [3:0] OP, input Cin);
    always @(A, B, OP, Cin) begin 
        case (OP)
            4'b0000: {C,Out} = A + B;
            4'b0001: {C,Out} = A + B + Cin;
            4'b0010: {C,Out} = A - B;
            4'b0011: {C,Out} = A - B - Cin;
            4'b0100: Out = A & B;
            4'b0101: Out = A | B;
            4'b0110: Out = A ^ B;
            4'b0111: Out = ~ (A ^ B);
            4'b1000: Out = A & (~ B);
            4'b1001: Out = A | (~ B);
            4'b1010: Out = A << B;
            4'b1011: Out = A >> B;
            4'b1100: Out = $signed(A) >>> B; // Preserve sign of most significant bit
            4'b1101: Out = A;
            4'b1110: Out = B;
            4'b1111: Out = ~B;
        endcase
        if((4'b0000 <= OP) && (OP <= 4'b1001)) begin
            N = Out[31] ; // Most significant bit
            Z = ((Out && 1) == 0); // Is the result zero
        end  
        if((4'b0000 <= OP) && (OP <= 4'b0001)) begin // Flags for addition
            V = (A[31] == B[31]) && (A[31] != (Out[31])); // Overflow, the operands have the same sign but the result does not
        end else if ((4'b0010 <= OP) && (OP <= 4'b0011)) begin // Flags for subtraction
            V = (A[31] != B[31]) && (A[31] != (Out[31])); // Overflow, the operands have different signs and the result's sign is different from the first operand
        end
    end
endmodule

// Module implementation of Source Operand2 Handler SPARC component
module source_operand2_handler_sparc_component(output reg [31:0] N, 
                                                input [31:0] R, 
                                                input [21:0] Imm, 
                                                input [3:0] Is);
    always @(R, Imm, Is)
        case(Is)
            4'b0000: N = {Imm, 10'b0000000000};
            4'b0001: N = {Imm, 10'b0000000000};
            4'b0010: N = {Imm, 10'b0000000000};
            4'b0011: N = {Imm, 10'b0000000000};
            4'b0100: N = {{10{Imm[21]}}, Imm}; // For sign extended replicate MSB
            4'b0101: N = {{10{Imm[21]}}, Imm};
            4'b0110: N = {{10{Imm[21]}}, Imm};
            4'b0111: N = {{10{Imm[21]}}, Imm};
            4'b1000: N = R;
            4'b1001: N = {{19{Imm[12]}}, Imm[12:0]};
            4'b1010: N = {{27{1'b0}}, R[4:0]};
            4'b1011: N = {{27{1'b0}}, Imm[4:0]};
            4'b1100: N = R;
            4'b1101: N = {{19{Imm[12]}}, Imm[12:0]};
            4'b1110: N = R;
            4'b1111: N = {{19{Imm[12]}}, Imm[12:0]};
        endcase
endmodule
