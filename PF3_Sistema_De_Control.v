module Sistema_De_Control_tb;

Special_Register nPC (
nPC_Out,Adder_Out,LE,Clr,Clk
);

General_Register PC (
PC_Out,nPC_Out,LE,Clr,Clk
);

alu_sparc_component Adder (
 Adder_Out, Z, N, C, V, 
 PC_Out, 4, 4'b0000,  Cin);

ROM Instruction_Memory(
    PC_Out,DataOut
);






General_Register IF_ID (
    I21_0,
    I29_0,
    PC31_0,
    I29,
    I18_14,
    I4_0,
    I29_25,
    I28_25,
    I12_0,
    I31_0, 
    I29_25_2, //pal mux
    InstuctionMemoryOut, PC , 
    1, Clr, Clk
);

control_unit ControlUnit (
I31, I30, I24, I13,
ID_Load_Instr, ID_RF_Enable, 
RAM_Enable, RAM_RW, RAM_SE,
ID_Jumpl_Instr, ID_Instr_Alter_CC,
ID_B_Instr, ID_Call_Instr,
RAM_Size, ID_Load_CallOrJumpl_Instr,
ID_ALU_OP,
I31_0
);



General_Register ID_EX (
Q,D,1,Clr,Clk
);

General_Register EX_MEM (
Q,D,1,Clr,Clk
);

General_Register MEM_WB (
Q,D,1,Clr,Clk
);


endmodule



/************************************************************************************************************************************************************************************************************************************************************************/

module control_unit(output reg I31, I30, I24, I13,
                    output reg ID_Load_Instr, ID_RF_Enable, 
                    output reg RAM_Enable, RAM_RW, RAM_SE,
                    output reg ID_Jumpl_Instr, ID_Instr_Alter_CC,
                    output reg ID_B_Instr, ID_Call_Instr,
                    output reg [1:0] RAM_Size, ID_Load_CallOrJumpl_Instr,
                    output reg [3:0] ID_ALU_OP,
                    input [31:0] Instr);
    reg [8:0] opcode;
    always @(Instr) begin
        I31 = Instr[31];
        I30 = Instr[30];
        I24 = Instr[24];
        I13 = Instr[13];
        ID_Load_Instr = ((Instr[31:30] == 2'b11) && (Instr[24:19] == 6'b001001 || Instr[24:19] == 6'b001010 ||
                         Instr[24:19] == 6'b000000 || Instr[24:19] == 6'b000001 || Instr[24:19] == 6'b000010 ||
                         Instr[24:19] == 6'b000011 || Instr[24:19] == 6'b001101)); // Check the OP and then if it's a Load instruction
        ID_Jumpl_Instr = ((Instr[31:30] == 2'b10) && (Instr[24:19] == 6'b111000)); // Check the OP and then if it's a Jumpl instruction
        ID_B_Instr = ((Instr[31:30] == 2'b00) && (4'b0000 <= Instr[28:25] && Instr[28:25] >= 4'b1111)); // Check the OP and then if it's a branch instruction
        ID_Call_Instr = (Instr[31:30] == 2'b01); // Check the OP to see if it's a call instruction
        
        // Signal used for multiplexer that selects the resulting operand in the register file's PW
        if (ID_Load_Instr && !(ID_Call_Instr || ID_Jumpl_Instr)) begin
            ID_Load_CallOrJumpl_Instr = 2'b11;
        end else if(!ID_Load_Instr && (ID_Call_Instr || ID_Jumpl_Instr)) begin
            ID_Load_CallOrJumpl_Instr = 2'b00;
        end else begin
            ID_Load_CallOrJumpl_Instr = 2'b01;
        end

        // Convert the opcode from instr to the opcode that will be used for the ALU
        ID_Instr_Alter_CC = 0; // Preliminary value if conditions are met then change it
        opcode = {Instr[31:30],Instr[24:19]}; // Join op and op3
        case(opcode)
            // Basic Arithmetic Instructions
            8'b10000000: ID_ALU_OP = 4'b0000; // add -> A + B for ALU
            8'b10010000: begin 
                            ID_ALU_OP = 4'b0000; // addcc -> A + B for ALU modify icc
                            ID_Instr_Alter_CC = 1;
            end
            8'b10001000: ID_ALU_OP = 4'b0001; // addx -> A + B + Cin for ALU
            8'b10011000: begin
                            ID_ALU_OP = 4'b0001; // addxcc -> A + B + Cin for ALU modify icc
                            ID_Instr_Alter_CC = 1;
            end
            8'b10000100: ID_ALU_OP = 4'b0010; // sub -> A - B for ALU
            8'b10010100: begin
                            ID_ALU_OP = 4'b0010; // subcc -> A - B for ALU modify icc
                            ID_Instr_Alter_CC = 1;
            end
            8'b10001100: ID_ALU_OP = 4'b0011; // subx -> A - B - Cin for ALU
            8'b10011100: begin
                            ID_ALU_OP = 4'b0011; // subxcc -> A - B - Cin for ALU modify icc
                            ID_Instr_Alter_CC = 1;
            end
            // Logical Instructions
            8'b10000001: ID_ALU_OP = 4'b0100; // and -> A and B for ALU
            8'b10010001: begin
                            ID_ALU_OP = 4'b0100; // andcc -> A and B for ALU modify icc
                            ID_Instr_Alter_CC = 1;
            end
            8'b10000101: ID_ALU_OP = 4'b1000; // andn  -> A and (not B) for ALU
            8'b10010101: begin
                            ID_ALU_OP = 4'b1000; // andncc -> A and (not B) for ALU modify icc
                            ID_Instr_Alter_CC = 1;
            end
            8'b10000010: ID_ALU_OP = 4'b0101; // or -> A or B for ALU
            8'b10010010: begin
                            ID_ALU_OP = 4'b0101; // orcc -> A or B for ALU modify icc
                            ID_Instr_Alter_CC = 1;
            end
            8'b10000110: ID_ALU_OP = 4'b1001; // orn -> A or (not B) for ALU
            8'b10010110: begin
                            ID_ALU_OP = 4'b1001; //orncc -> A or (not B) for ALU modify icc
                            ID_Instr_Alter_CC = 1;
            end
            8'b10000011: ID_ALU_OP = 4'b0110; // xor -> A xor B for ALU
            8'b10010011: begin
                            ID_ALU_OP = 4'b0110; // xorcc -> A xor B for ALU modify icc
                            ID_Instr_Alter_CC = 1;
            end
            8'b10000111: ID_ALU_OP = 4'b0111; // xorn -> A xnor B for ALU
            8'b10010111: begin
                            ID_ALU_OP = 4'b0111; // xorncc -> A xnor B for ALU modify icc
                            ID_Instr_Alter_CC = 1;
            end
            // Shift Instructions
            8'b10100101: ID_ALU_OP = 4'b1010; // sll -> shift left logical (A) B positions for ALU
            8'b10100110: ID_ALU_OP = 4'b1011; // srl -> shift right logical (A) B positions for ALU
            8'b10100111: ID_ALU_OP = 4'b1100; // sra -> shift right arithmetic (A) B positions for ALU
            // TODO: Continue for load/store, call, jump, etc.
            default: ID_ALU_OP = 4'b1101;
        endcase
        // TODO: Set values for ID_RF_Enable, RAM_Enable, RAM_RW, RAM_SE, RAM_Size
    end
endmodule


module ROM (
  input [8:0] address,
  output reg [31:0] DataOut
);

reg [7:0] mem [0:511]; // 512x8 ROM

always @(address) 
  begin
  DataOut = {mem[address], mem[address+1], mem[address+2], mem[address+3]};
  end
endmodule

module mux_2x1 (output tri O, input A, B, S); //tiene como entrada el 0 y el control Signal
bufif0 (O, A, S);
bufif1 (O, B, S);
endmodule







module General_Register (output reg [31:0] Q, input [31:0] D, input //PC Y IF/ID
LE, Clr, Clk);
always @ (posedge Clk) //rising edge triggered Register
if (Clr) Q <= 32'h00000000;
else if (LE) Q <= D; // estar pendiente cuando inicialice el LE para el testbench, ya que el registro PC lleva LE y los demas se le deberia asignar un valor constante de 1 (que siempre cargue los valores)
endmodule

module Special_Register (output reg [31:0] Q, input [31:0] D, input
LE, Clr, Clk);
always @ (posedge Clk) //rising edge triggered Register
if (Clr) Q <= 32'h00000100; //produce un 4 binario cuando es reseteado
else if (LE) Q <= D; 
endmodule

module Pipeline_Register_IF_ID (

    output reg [21:0] I21_0,
    output reg [29:0] I29_0,
    output reg [31:0] PC31_0,
    output reg [29] I29,
    output reg [18:14] I18_14,
    output reg [4:0] I4_0,
    output reg [29:25] I29_25,
    output reg [28:25] I28_25,
    output reg [12:0] I12_0,
    output reg [31:0] I31_0, 
    output reg [29:25] I29_25_2, //pal mux
    input [31:0] InstuctionMemoryOut, PC , 
    input LE, Clr, Clk
    );
always @ (posedge Clk) //rising edge triggered Register
if (Clr) I31_0 <= 32'h00000000;
else if (LE) begin
    I21_0 <= InstuctionMemoryOut[21:0]; 
    I29_0 <= InstuctionMemoryOut[29:0]; 
    PC31_0 <= PC; 
    I29 <= InstuctionMemoryOut[29]; 
    I18_14 <= InstuctionMemoryOut[18:14]; 
    I4_0 <= InstuctionMemoryOut[4:0]; 
    I29_25 <= InstuctionMemoryOut[29:25]; 
    I28_25 <= InstuctionMemoryOut[28:25]; 
    I12_0 <= InstuctionMemoryOut[12:0]; 
    I31_0 <= InstuctionMemoryOut; 
    I29_25_2 <= InstuctionMemoryOut[29:25]; 
end
endmodule   

module Pipeline_Register_ID_EX (output reg [31:0] Q, input [31:0] D, input
LE, Clr, Clk);
always @ (posedge Clk) //rising edge triggered Register
if (Clr) Q <= 32'h00000000;
else if (LE) Q <= D; // estar pendiente cuando inicialice el LE para el testbench, ya que el registro PC lleva LE y los demas se le deberia asignar un valor constante de 1 (que siempre cargue los valores)
endmodule

module Pipeline_Register_EX_MEM (output reg [31:0] Q, input [31:0] D, input
LE, Clr, Clk);
always @ (posedge Clk) //rising edge triggered Register
if (Clr) Q <= 32'h00000000;
else if (LE) Q <= D; // estar pendiente cuando inicialice el LE para el testbench, ya que el registro PC lleva LE y los demas se le deberia asignar un valor constante de 1 (que siempre cargue los valores)
endmodule

module Pipeline_Register_MEM_WB (output reg [31:0] Q, input [31:0] D, input
LE, Clr, Clk);
always @ (posedge Clk) //rising edge triggered Register
if (Clr) Q <= 32'h00000000;
else if (LE) Q <= D; // estar pendiente cuando inicialice el LE para el testbench, ya que el registro PC lleva LE y los demas se le deberia asignar un valor constante de 1 (que siempre cargue los valores)
endmodule






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