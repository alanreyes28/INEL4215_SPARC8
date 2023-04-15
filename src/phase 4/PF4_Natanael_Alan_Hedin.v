// Control System Testbench
module Sistema_De_Control_tb;

// Parameters for Registers
reg select,Clr,Clk,LE;

wire [31:0] nPC_Out,Adder_Out , DataOut;
reg Z, N, C, V, Cin;

// Parameters for CU
wire  I31, I30, I24, I13, ID_Load_Instr, ID_RF_Enable,RAM_Enable, RAM_RW, RAM_SE,	ID_Jumpl_Instr, ID_Instr_Alter_CC, ID_B_Instr, ID_Call_Instr; 
wire [1:0] RAM_Size, ID_Load_CallOrJumpl_Instr;
wire [3:0] ID_ALU_OP;

/// Parameters for Multiplexer
wire I31_OUT, I30_OUT, I24_OUT, I13_OUT, ID_Load_Instr_OUT, ID_RF_Enable_OUT,ID_Jumpl_Instr_OUT, ID_Instr_Alter_CC_OUT;
wire [3:0] ID_ALU_OP_OUT; 
wire [1:0] ID_Load_CallOrJumpl_Instr_OUT; 


// Parameters for IF/ID
wire [21:0] I21_0;
wire [29:0] I29_0;
wire [31:0] PC31_0, I31_0_2;
wire I29;
wire [4:0] I18_14, I4_0, I29_25,I29_25_2;
wire [3:0] I28_25; 
wire [12:0] I12_0;


// Parameters for ID/EX
wire [31:0] MX1_OUT, MX2_OUT, MX3_OUT, PC_OUT;
wire [12:0] I12_0_OUT;
wire [4:0] RD4_0_OUT; 
wire [3:0] ID_ALU_OP_OUT_REG;
wire I31_OUT_REG,I30_OUT_REG,I24_OUT_REG,I13_OUT_REG, ID_Load_Instr_OUT_REG, ID_RF_Enable_OUT_REG, RAM_Enable_OUT_REG,RAM_RW_OUT_REG,RAM_SE_OUT_REG, ID_Jumpl_Instr_OUT_REG,ID_Instr_Alter_CC_OUT_REG;
wire [1:0] RAM_Size_OUT_REG, ID_Load_CallOrJumpl_Instr_OUT_REG;
reg [31:0] MX1_IN,MX2_IN,MX3_IN,PC_IN;
reg [12:0] I12_0_IN;
reg [4:0] RD4_0_IN;
reg I31_OUT_MUX,I30_OUT_MUX,I24_OUT_MUX,I13_OUT_MUX, ID_Load_Instr_OUT_MUX, ID_RF_Enable_OUT_MUX, RAM_Enable_OUT_MUX,RAM_RW_OUT_MUX,RAM_SE_OUT_MUX, ID_Jumpl_Instr_OUT_MUX,ID_Instr_Alter_CC_OUT_MUX;
reg [3:0] ID_ALU_OP_OUT_MUX;
reg [1:0] RAM_Size_OUT_MUX, ID_Load_CallOrJumpl_Instr_OUT_MUX;

// Parameters for EX/MEM
wire [31:0] Out_Out;
wire [31:0] PC_OUT_MEM;
wire [31:0] MX3_MEM_Out;
wire [4:0]  RD_Out;
wire ID_RF_enable_MEM_Out;
wire RAW_Enable_Out;
wire RAM_RW_Out;
wire RAM_SE_Out;
wire [1:0] RAM_Size_Out;
wire [1:0] ID_load_callOrJumpl_instr_MEM_Out;
reg [31:0] Out_In;
reg Z_In,V_In,N_In,C_In;
reg [31:0] MX3_MEM_In;
reg [31:0] PC_MEM_In;
reg [4:0] RD_MEM_In;
  
//Parameters for MEM/WB
wire [31:0] PW_WB;
wire [4:0] RD_OUT_WB;
wire ID_RF_enable_OUT_WB;
reg [31:0] DO;
reg [31:0] MEM_RD;

// Parameters for Preload of ROM
wire [31:0] PC_Out;
reg [31:0] DataIn;
integer file, code;


Special_Register nPC (
    nPC_Out, // Output
    Adder_Out,LE,Clr,Clk // Inputs
);

General_Register PC (
    PC_Out, // Output
    nPC_Out,LE,Clr,Clk // Inputs
);

alu_sparc_component Adder (
    Adder_Out, Z, N, C, V, // Outputs
    nPC_Out, 32'b00000000000000000000000000000100, 4'b0000,  Cin // Inputs
 );

ROM Instruction_Memory(
    PC_Out, // Input
    DataOut // Output
);

control_unit ControlUnit (
    I31, I30, I24, I13, ID_Load_Instr, ID_RF_Enable, 
    RAM_Enable, RAM_RW, RAM_SE, ID_Jumpl_Instr,
    ID_Instr_Alter_CC, ID_B_Instr, ID_Call_Instr,
    RAM_Size, ID_Load_CallOrJumpl_Instr, ID_ALU_OP, // Outputs
    I31_0_2 // Input
);

ctrl_unit_mux_2x1 CU_MUX(
    I31_OUT, I30_OUT, I24_OUT, I13_OUT, ID_Load_Instr_OUT, ID_RF_Enable_OUT,
    ID_ALU_OP_OUT, ID_Jumpl_Instr_OUT, ID_Instr_Alter_CC_OUT, ID_Load_CallOrJumpl_Instr_OUT, // Outputs
    I31, I30, I24, I13, ID_Load_Instr, ID_RF_Enable, 
    ID_ALU_OP, ID_Jumpl_Instr, ID_Instr_Alter_CC, 
    ID_Load_CallOrJumpl_Instr, select // Inputs
); 

Pipeline_Register_IF_ID IF_ID (
    I21_0, I29_0, PC31_0, I29, I18_14, I4_0,
    I29_25, I28_25, I12_0, I31_0_2, I29_25_2, // Outputs
    DataOut, PC_Out, Clr, Clk // Inputs
);

Pipeline_Register_ID_EX ID_EX (
    //Outputs Parte Amarilla
/**********************************/
    MX1_OUT, MX2_OUT,MX3_OUT,PC_OUT,
    I12_0_OUT,
    RD4_0_OUT,
/**********************************/

    //Outputs de parte Gris
/**********************************/
    I31_OUT_REG,I30_OUT_REG,I24_OUT_REG,I13_OUT_REG,
    ID_Load_Instr_OUT_REG,
    ID_ALU_OP_OUT_REG,
    ID_RF_Enable_OUT_REG, RAM_Enable_OUT_REG,RAM_RW_OUT_REG,RAM_SE_OUT_REG,
    RAM_Size_OUT_REG,
    ID_Jumpl_Instr_OUT_REG,ID_Instr_Alter_CC_OUT_REG,
    ID_Load_CallOrJumpl_Instr_OUT_REG,
/**********************************/

    //Inputs Parte Amarilla
/**********************************/
    MX1_IN,MX2_IN,MX3_IN,PC_IN,
    I12_0, //pendiente a ver si el nombre de esto causa problemas (no creo)
    RD4_0_IN,
/**********************************/

    //Inputs Parte Gris
/**********************************/
    I31_OUT,I30_OUT,I24_OUT,I13_OUT,
    ID_Load_Instr_OUT,
    ID_ALU_OP_OUT,
    ID_RF_Enable_OUT,RAM_Enable,RAM_RW,RAM_SE,
    RAM_Size,
    ID_Jumpl_Instr_OUT,ID_Instr_Alter_CC_OUT,
    ID_Load_CallOrJumpl_Instr_OUT,
/**********************************/
    Clr, Clk
);

Pipeline_Register_EX_MEM EX_MEM (
    Out_Out, MX3_MEM_Out, PC_OUT_MEM, RD_Out,
    ID_RF_enable_MEM_Out, RAW_Enable_Out,
    RAM_RW_Out, RAM_SE_Out, RAM_Size_Out,
    ID_load_callOrJumpl_instr_MEM_Out, // Outputs
    Out_In, Z_In,V_In,N_In,C_In, MX3_MEM_In,
    PC_MEM_In, RD_MEM_In, ID_RF_Enable_OUT_REG, RAM_Enable_OUT_REG,
    RAM_RW_OUT_REG, RAM_SE_OUT_REG, RAM_Size_OUT_REG,
    ID_Load_CallOrJumpl_Instr_OUT_REG, Clr, Clk //Inputs
);

Pipeline_Register_MEM_WB MEM_WB(
  	PW_WB,RD_OUT_WB,ID_RF_enable_OUT_WB, //Outputs
    DO,MEM_RD,RD_Out,ID_RF_enable_MEM_Out, Clr, Clk //Inputs
); 

// Preload Instruction Memory
reg [8:0] InstrIn;
initial begin
    file = $fopen("control_system_test.txt","r+b");
    InstrIn =  8'b00000000;
        while (!$feof(file)) begin 
            code = $fscanf(file, "%b", DataIn);
            Instruction_Memory.mem[InstrIn] = DataIn;
            InstrIn = InstrIn + 1;
        end
    $fclose(file);
end

initial #54 $finish;
  
initial begin
  Cin = 1;
end

initial begin
  Clk = 0;
  forever #2 Clk = ~Clk;
end

initial begin
  Clr = 1'b1;
  #3 Clr = 1'b0;
end

initial begin
  LE = 1'b1;
  select = 1'b0; 
  #49 select = 1'b1; // Before the NOP instruction enters the ID stage
end

initial begin
  $display("PF3 Control System Results:");
  $monitor("0) Time: %0t\n1) Instruction, PC, nPC:\n-Instr going to CU = %b, PC = %d, nPC = %d\n2) Ouputs of the Control Unit:\n-I31 = %b, I30 = %b, I24 = %b, I13 = %b, ID_Load_Instr = %b, ID_RF_Enable = %b, RAM_Enable = %b, RAM_RW = %b, RAM_SE = %b, ID_Jumpl_Instr = %b, ID_Instr_Alter_CC = %b, ID_B_Instr = %b, ID_Call_Instr = %b, RAM_Size = %b, ID_Load_CallOrJumpl_Instr = %b, ID_ALU_OP = %b\n3) Outputs of EX stage:\n-I31_OUT_REG = %b,I30_OUT_REG = %b,I24_OUT_REG = %b,I13_OUT_REG = %b, ID_Load_Instr_OUT_REG, = %b, ID_ALU_OP_OUT_REG = %b, ID_RF_Enable_OUT_REG = %b, RAM_Enable_OUT_REG = %b,RAM_RW_OUT_REG = %b,RAM_SE_OUT_REG = %b, RAM_Size_OUT_REG = %b, ID_Jumpl_Instr_OUT_REG = %b,ID_Instr_Alter_CC_OUT_REG = %b, ID_Load_CallOrJumpl_Instr_OUT_REG = %b\n4) Outputs of MEM stage:\n-ID_RF_enable_MEM_Out = %b, RAW_Enable_Out = %b, RAM_RW_Out = %b, RAM_SE_Out = %b, RAM_Size_Out = %b, ID_load_callOrJumpl_instr_MEM_Out = %b\n5) Outputs of WB Stage:\n-ID_RF_enable_OUT_WB = %b \n", $time,I31_0_2, PC_Out, nPC_Out,   I31, I30, I24, I13, ID_Load_Instr, ID_RF_Enable, RAM_Enable, RAM_RW, RAM_SE, ID_Jumpl_Instr, ID_Instr_Alter_CC, ID_B_Instr, ID_Call_Instr, RAM_Size, ID_Load_CallOrJumpl_Instr, ID_ALU_OP,   I31_OUT_REG,I30_OUT_REG,I24_OUT_REG,I13_OUT_REG,ID_Load_Instr_OUT_REG,ID_ALU_OP_OUT_REG, ID_RF_Enable_OUT_REG, RAM_Enable_OUT_REG,RAM_RW_OUT_REG,RAM_SE_OUT_REG,RAM_Size_OUT_REG,ID_Jumpl_Instr_OUT_REG,ID_Instr_Alter_CC_OUT_REG,ID_Load_CallOrJumpl_Instr_OUT_REG,   ID_RF_enable_MEM_Out, RAW_Enable_Out, RAM_RW_Out, RAM_SE_Out, RAM_Size_Out, ID_load_callOrJumpl_instr_MEM_Out,ID_RF_enable_OUT_WB);
end

endmodule

//Partes de la implementacion de la fase 2
/************************************************************************************************************************************************************************************************************************************************************************/

// Control Unit module
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
        ID_B_Instr = ((Instr[31:30] == 2'b00) && (4'b0000 <= Instr[28:25] && Instr[28:25] <= 4'b1111) && (Instr[24:22] == 3'b010)); // Check the OP and then if it's a branch instruction
        ID_Call_Instr = (Instr[31:30] == 2'b01); // Check the OP to see if it's a call instruction
        
        // Signal used for multiplexer that selects the resulting operand in the register file's PW
        if (ID_Load_Instr && !(ID_Call_Instr || ID_Jumpl_Instr)) begin
            ID_Load_CallOrJumpl_Instr = 2'b10;
        end else if(!ID_Load_Instr && (ID_Call_Instr || ID_Jumpl_Instr)) begin
            ID_Load_CallOrJumpl_Instr = 2'b00;
        end else begin
            ID_Load_CallOrJumpl_Instr = 2'b01;
        end

        // Convert the opcode from instr to the opcode that will be used for the ALU
        ID_Instr_Alter_CC = 0; // Preliminary value, change for corresponding instructions
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
            default: ID_ALU_OP = 4'b0000;  // For other instructions like load, store, jumpl, branch, call, etc.
        endcase
        if(Instr[31:30] == 2'b00 && Instr[24:22] == 3'b100) begin // Sethi Instructions
            ID_ALU_OP = 4'b1110; // sethi -> B for ALU. Let the ALU choose what the src op2 handler provides as input in B
        end
        
        // Check which instruction can write to the Register File
        ID_RF_Enable = 1; // Preliminary value, change for corresponding instructions
        if(Instr[31:30] == 2'b00 && Instr[24:22] == 3'b010) begin // Branch Instructions
            ID_RF_Enable = 0;
        end else if(Instr == 32'b0) begin // NOP Instruction
            ID_RF_Enable = 0;
        end else if(Instr[31:30] == 2'b11 && (Instr[24:19] == 6'b000101 || Instr[24:19] == 6'b000110 || 
                    Instr[24:19] == 6'b000100 || Instr[24:19] == 6'b000111)) begin // Store Instructions
            ID_RF_Enable = 0;
        end

        // RAM Signals:
      	RAM_Enable = 0; //default to zero RAM_Enable
      	RAM_RW = 0; //default to zero RAM_RW
        if( opcode == 8'b11000101 ||  opcode == 8'b11000110 ||  opcode == 8'b11000100 || opcode == 8'b11000111) begin  // Check if it's a store instruction
            RAM_RW = 1;
            RAM_Enable = 1;
        end else if (opcode == 8'b11001001 || opcode == 8'b11001010 || opcode == 8'b11000000 ||  opcode == 8'b11000001 ||  opcode == 8'b11000010 || opcode == 8'b11000011 || opcode == 8'b11001101) begin  // Check if it's a load instruction
            RAM_RW = 0;
          	RAM_Enable = 1;
        end
        // RAM Size and Sign Extention (SE):
        RAM_SE = 0; // Preliminary value
        case(opcode)
            // Load Integer Instructions
            8'b11001001: begin
                RAM_Size = 2'b00; // lsb -> byte
                RAM_SE = 1;
            end
            8'b11001010: begin 
                RAM_Size = 2'b01;// ldsh -> halfword
                RAM_SE = 1;
            end
            8'b11000000: RAM_Size = 2'b10; // ld -> word
            8'b11000001: RAM_Size = 2'b00; // ldub -> byte
            8'b11000010: RAM_Size = 2'b01; // lduh -> halfword
            8'b11001101: RAM_Size = 2'b00; // ldstub -> byte
            // Store Integer Instructions
            8'b11000101: RAM_Size = 2'b00; // stb -> byte
            8'b11000110: RAM_Size = 2'b01; // sth -> halword
            8'b11000100: RAM_Size = 2'b10; // st -> word
        endcase
    end
endmodule



// Multiplexer module for Control Unit
module ctrl_unit_mux_2x1(output reg I31_OUT, I30_OUT, I24_OUT, I13_OUT, ID_Load_Instr_OUT, ID_RF_Enable_OUT,
            output reg [3:0] ID_ALU_OP_OUT,
            output reg ID_Jumpl_Instr_OUT, ID_Instr_Alter_CC_OUT,
            output reg [1:0] ID_Load_CallOrJumpl_Instr_OUT,
            input I31_IN, I30_IN, I24_IN, I13_IN, ID_Load_Instr_IN, ID_RF_Enable_IN, 
            input [3:0] ID_ALU_OP_IN,
            input ID_Jumpl_Instr_IN, ID_Instr_Alter_CC_IN,
            input [1:0] ID_Load_CallOrJumpl_Instr_IN,
            input select);
    always @ (*) begin
        if(select == 1'b0) begin // Pass Control Unit values when select is 0
            I31_OUT = I31_IN;
            I30_OUT = I30_IN;
            I24_OUT = I24_IN;
            I13_OUT = I13_IN;
            ID_Load_Instr_OUT = ID_Load_Instr_IN;
            ID_RF_Enable_OUT = ID_RF_Enable_IN;
            ID_ALU_OP_OUT = ID_ALU_OP_IN;
            ID_Jumpl_Instr_OUT = ID_Jumpl_Instr_IN;
            ID_Instr_Alter_CC_OUT = ID_Instr_Alter_CC_IN;
            ID_Load_CallOrJumpl_Instr_OUT = ID_Load_CallOrJumpl_Instr_IN;
        end else begin
            I31_OUT = 1'b0;
            I30_OUT = 1'b0;
            I24_OUT = 1'b0;
            I13_OUT = 1'b0;
            ID_Load_Instr_OUT = 1'b0;
            ID_RF_Enable_OUT = 1'b0;
            ID_ALU_OP_OUT = 4'b0;
            ID_Jumpl_Instr_OUT = 1'b0;
            ID_Instr_Alter_CC_OUT = 1'b0;
            ID_Load_CallOrJumpl_Instr_OUT = 2'b0;
        end
    end
endmodule

// Register module for PC
module General_Register (output reg [31:0] Q, input [31:0] D, input LE, Clr, Clk);
    always @ (posedge Clk) begin //rising edge triggered Register
        if (Clr) Q <= 32'b00000000000000000000000000000000;
        else if (LE) Q <= D; // estar pendiente cuando inicialice el LE para el testbench, ya que el registro PC lleva LE y los demas se le deberia asignar un valor constante de 1 (que siempre cargue los valores)
    end
endmodule

// Register module for nPC
module Special_Register (output reg [31:0] Q, input [31:0] D, input LE, Clr, Clk);
    always @ (posedge Clk) begin//rising edge triggered Register
        if (Clr) Q <=  32'b00000000000000000000000000000100; //produce un 4 binario cuando es reseteado
        else if (LE) Q <= D;
    end
endmodule

// Pipeline module for IF/ID
module Pipeline_Register_IF_ID (output reg [21:0] I21_0,
                                output reg [29:0] I29_0,
                                output reg [31:0] PC31_0,
                                output reg  I29,
                                output reg [4:0] I18_14,
                                output reg [4:0] I4_0,
                                output reg [4:0] I29_25,
                                output reg [3:0] I28_25,
                                output reg [12:0] I12_0,
                                output reg [31:0] I31_0, 
                                output reg [4:0] I29_25_2, //pal mux
                                input [31:0] InstuctionMemoryOut, PC , 
                                input  Clr, Clk);
    always @ (posedge Clk) begin//rising edge triggered Register
        if (Clr) begin //tar pendiente por si explota
            I21_0 <= 22'b0; 
            I29_0 <= 30'b0; 
            PC31_0 <= 32'b0; 
            I29 <= 1'b0; 
            I18_14 <= 5'b0; 
            I4_0 <= 5'b0; 
            I29_25 <= 5'b0; 
            I28_25 <= 4'b0; 
            I12_0 <= 12'b0; 
            I31_0 <= 32'b0; 
            I29_25_2 <= 5'b0; 
        end
        else begin
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
    end
endmodule
  
// Pipeline module for ID/EX
module Pipeline_Register_ID_EX (
                //Outputs Parte Amarilla
            /**********************************/
  			output reg [31:0] MX1_OUT, MX2_OUT,MX3_OUT,PC_OUT,
            output reg [12:0] I12_0_OUT,
            output reg [4:0] RD4_0_OUT,
            /**********************************/

                //Outputs de parte Gris
            /**********************************/
            output reg I31_OUT_REG,I30_OUT_REG,I24_OUT_REG,I13_OUT_REG,
            output reg ID_Load_Instr_OUT_REG,
  			output reg [3:0] ID_ALU_OP_OUT_REG,
            output reg ID_RF_Enable_OUT_REG, RAM_Enable_OUT_REG,RAM_RW_OUT_REG,RAM_SE_OUT_REG,
            output reg [1:0] RAM_Size_OUT_REG,
            output reg ID_Jumpl_Instr_OUT_REG,ID_Instr_Alter_CC_OUT_REG,
            output reg [1:0] ID_Load_CallOrJumpl_Instr_OUT_REG,
            /**********************************/

                //Inputs Parte Amarilla
            /**********************************/
            input [31:0] MX1_IN,MX2_IN,MX3_IN,PC_IN,
            input [12:0] I12_0_IN, //pendiente a ver si el nombre de esto causa problemas (no creo)
            input [4:0] RD4_0_IN,
            /**********************************/

                //Inputs Parte Gris
            /**********************************/
            input I31_OUT_MUX,I30_OUT_MUX,I24_OUT_MUX,I13_OUT_MUX,
            input ID_Load_Instr_OUT_MUX,
  			input [3:0] ID_ALU_OP_OUT_MUX,
            input ID_RF_Enable_OUT_MUX, RAM_Enable_OUT_MUX,RAM_RW_OUT_MUX,RAM_SE_OUT_MUX,
            input [1:0] RAM_Size_OUT_MUX,
            input ID_Jumpl_Instr_OUT_MUX,ID_Instr_Alter_CC_OUT_MUX,
            input [1:0] ID_Load_CallOrJumpl_Instr_OUT_MUX,
            /**********************************/
            input Clr, Clk);
    always @ (posedge Clk) begin //rising edge triggered Register
        if (Clr)begin 
            MX1_OUT<= 32'b0;
            MX2_OUT<= 32'b0;
            MX3_OUT<= 32'b0;
            PC_OUT <= 32'b0;
            I12_0_OUT <= 13'b0;
            RD4_0_OUT <= 5'b0;
            I31_OUT_REG <= 1'b0;
            I30_OUT_REG <= 1'b0;
            I24_OUT_REG <= 1'b0;
            I13_OUT_REG <= 1'b0;
            ID_Load_Instr_OUT_REG <= 1'b0;
            ID_ALU_OP_OUT_REG <= 5'b0;
            ID_RF_Enable_OUT_REG <= 1'b0;
            RAM_Enable_OUT_REG <= 1'b0;
            RAM_RW_OUT_REG <= 1'b0;
            RAM_SE_OUT_REG <= 1'b0;
            RAM_Size_OUT_REG <= 2'b0;
            ID_Jumpl_Instr_OUT_REG <= 1'b0;
            ID_Instr_Alter_CC_OUT_REG <= 1'b0;
            ID_Load_CallOrJumpl_Instr_OUT_REG <= 2'b0;
        end
        else begin 
            MX1_OUT<= MX1_IN;
            MX2_OUT<= MX2_IN;
            MX3_OUT<= MX3_IN;
            PC_OUT <= PC_IN;
            I12_0_OUT <= I12_0_IN;
            RD4_0_OUT <= RD4_0_IN;
            I31_OUT_REG <= I31_OUT_MUX;
            I30_OUT_REG <= I30_OUT_MUX;
            I24_OUT_REG <= I24_OUT_MUX;
            I13_OUT_REG <= I13_OUT_MUX;
            ID_Load_Instr_OUT_REG <= ID_Load_Instr_OUT_MUX;
            ID_ALU_OP_OUT_REG <= ID_ALU_OP_OUT_MUX;
            ID_RF_Enable_OUT_REG <= ID_RF_Enable_OUT_MUX;
            RAM_Enable_OUT_REG <= RAM_Enable_OUT_MUX;
            RAM_RW_OUT_REG<= RAM_RW_OUT_MUX;
            RAM_SE_OUT_REG<= RAM_SE_OUT_MUX;
            RAM_Size_OUT_REG <= RAM_Size_OUT_MUX;
            ID_Jumpl_Instr_OUT_REG <= ID_Jumpl_Instr_OUT_MUX;
            ID_Instr_Alter_CC_OUT_REG <= ID_Instr_Alter_CC_OUT_MUX;
            ID_Load_CallOrJumpl_Instr_OUT_REG <= ID_Load_CallOrJumpl_Instr_OUT_MUX; 
        end
    end
endmodule

// Pipeline module for EX/MEM
module Pipeline_Register_EX_MEM(output reg [31:0] Out_Out,
                                output reg [31:0] MX3_Out,
                                output reg [31:0] PC_Out,
                                output reg [4:0]  RD_Out,
                                output reg ID_RF_enable_Out,
                                output reg RAW_Enable_Out,
                                output reg RAM_RW_Out,
                                output reg RAM_SE_Out,
                                output reg [1:0] RAM_Size_Out,
                                output reg [1:0] ID_load_callOrJumpl_instr_Out,
                                input [31:0] Out_In,
                                input Z_In,V_In,N_In,C_In,
                                input [31:0] MX3_In,
                                input [31:0] PC_In,
                                input [4:0] RD_In,
                                input ID_RF_enable_In,
                                input RAW_Enable_In,
                                input RAM_RW_In,
                                input RAM_SE_In,
                                input [1:0] RAM_Size_In,
                                input [1:0] ID_load_callOrJumpl_instr_In,
                                input Clr, Clk);
    always @ (posedge Clk) begin //rising edge triggered Register
        if (Clr) begin
            Out_Out <= 32'h00000000;
            MX3_Out <= 32'h00000000;
            PC_Out <= 32'h00000000;
            RD_Out <= 4'b0000;
            ID_RF_enable_Out <= 1'b0;
            RAW_Enable_Out <= 1'b0;
            RAM_RW_Out <= 1'b0;
            RAM_RW_Out <= 1'b0;
            RAM_SE_Out  <= 1'b0;
            RAM_Size_Out <= 2'b00;
            ID_load_callOrJumpl_instr_Out <= 2'b00;
        end
        else begin
            Out_Out <= Out_In[31:0]; // estar pendiente cuando inicialice el LE para el testbench, ya que el registro PC lleva LE y los demas se le deberia asignar un valor constante de 1 (que siempre cargue los valores)
            PC_Out <= PC_In;
            MX3_Out <= MX3_In;
            RD_Out <= RD_In;
            ID_RF_enable_Out <= ID_RF_enable_In;
            RAW_Enable_Out <= RAW_Enable_In;
            RAM_RW_Out <=RAM_RW_In;
            RAM_SE_Out <= RAM_SE_In;
            RAM_Size_Out <= RAM_Size_In[1:0];
            ID_load_callOrJumpl_instr_Out <= ID_load_callOrJumpl_instr_In[1:0];
        end
    end
endmodule

// Pipeline module for MEM/WB
module Pipeline_Register_MEM_WB(output reg [31:0] PW_WB,
                                output reg [4:0] RD_OUT,
                                output reg ID_RF_enable_OUT,
                                input [31:0] DO, // Data Memory Out
                                input [31:0] MEM_RD,
                                input [4:0] RD_IN,
                                input ID_RF_enable_IN, Clr, Clk);
    always @ (posedge Clk) begin // Rising edge triggered Register
        if (Clr) begin
            PW_WB <= 32'b0;
            RD_OUT <= 4'b0;
            ID_RF_enable_OUT <= 32'b0;
        end
        else begin 
            PW_WB <= MEM_RD;
            RD_OUT <= RD_IN;
            ID_RF_enable_OUT <= ID_RF_enable_IN;
        end
    end
endmodule



//Partes del Register File Implementation
/************************************************************************************************************************************************************************************************************************************************************************/
module Three_Port_Register_File (
    input  [4:0] RA,RB,RC,RW,  // Entradas del registro para seleccionar registros A, B, C y el registro de escritura
    input  [31:0] PW,          // Entrada de datos del registro de escritura
    output  [31:0] PA, PB, PC, // Salidas de los registros A, B y C
    input Clk, LE            // Entradas de reloj y carga del registro
);

wire [31:0] O;
wire [31:0] R0,R1,R2,R3,R4,R5,R6,R7,R8,R9,R10,R11,R12,R13,R14,R15,R16,R17,R18,R19,R20,R21,R22,R23,R24,R25,R26,R27,R28,R29,R30,R31;

// Instanciación de los módulos

// Instanciación del decodificador binario para decodificar las entradas RA, RB, RC y activar la selección de registro correspondiente
Binary_Decoder BD (RW,LE,O);

// Instanciación del array de registros de 32 bits cada uno para almacenar los datos en el registro de escritura y en los registros A, B y C
Register Regs0 (PW,R0,O[0],Clk);
Register Regs1 (PW,R1,O[1],Clk);
Register Regs2 (PW,R2,O[2],Clk);
Register Regs3 (PW,R3,O[3],Clk);
Register Regs4 (PW,R4,O[4],Clk);
Register Regs5 (PW,R5,O[5],Clk);
Register Regs6 (PW,R6,O[6],Clk);
Register Regs7 (PW,R7,O[7],Clk);
Register Regs8 (PW,R8,O[8],Clk);
Register Regs9 (PW,R9,O[9],Clk);
Register Regs10 (PW,R10,O[10],Clk);
Register Regs11 (PW,R11,O[11],Clk);
Register Regs12 (PW,R12,O[12],Clk);
Register Regs13 (PW,R13,O[13],Clk);
Register Regs14 (PW,R14,O[14],Clk);
Register Regs15 (PW,R15,O[15],Clk);
Register Regs16 (PW,R16,O[16],Clk);
Register Regs17 (PW,R17,O[17],Clk);
Register Regs18 (PW,R18,O[18],Clk);
Register Regs19 (PW,R19,O[19],Clk);
Register Regs20 (PW,R20,O[20],Clk);
Register Regs21 (PW,R21,O[21],Clk);
Register Regs22 (PW,R22,O[22],Clk);
Register Regs23 (PW,R23,O[23],Clk);
Register Regs24 (PW,R24,O[24],Clk);
Register Regs25 (PW,R25,O[25],Clk);
Register Regs26 (PW,R26,O[26],Clk);
Register Regs27 (PW,R27,O[27],Clk);
Register Regs28 (PW,R28,O[28],Clk);
Register Regs29 (PW,R29,O[29],Clk);
Register Regs30 (PW,R30,O[30],Clk);
Register Regs31 (PW,R31,O[31],Clk);

// Instanciación de los multiplexores de 32x1 para seleccionar el valor almacenado en el registro correspondiente y enviarlo a las salidas de los registros A, B y C
Multiplexer_32x1 MuxA (RA,0,R1,R2,R3,R4,R5,R6,R7,R8,R9,R10,R11,R12,R13,R14,R15,R16,R17,R18,R19,R20,R21,R22,R23,R24,R25,R26,R27,R28,R29,R30,R31,PA);

Multiplexer_32x1 MuxB (RB,0,R1,R2,R3,R4,R5,R6,R7,R8,R9,R10,R11,R12,R13,R14,R15,R16,R17,R18,R19,R20,R21,R22,R23,R24,R25,R26,R27,R28,R29,R30,R31,PB);

  Multiplexer_32x1 MuxC (RC,0,R1,R2,R3,R4,R5,R6,R7,R8,R9,R10,R11,R12,R13,R14,R15,R16,R17,R18,R19,R20,R21,R22,R23,R24,R25,R26,R27,R28,R29,R30,R31,PC);

endmodule

module Binary_Decoder ( //El módulo tiene tres puertos: D, E y O. D es una entrada de 5 bits, E es una entrada de 1 bit y O es una salida de 32 bits.
    input [4:0] D,
    input E,
    output reg [31:0] O
);

always @(*) begin
    if (E == 1'b1) begin
        case (D)
            5'b00000: O = 32'b00000000000000000000000000000001;
            5'b00001: O = 32'b00000000000000000000000000000010;
            5'b00010: O = 32'b00000000000000000000000000000100;
            5'b00011: O = 32'b00000000000000000000000000001000;
            5'b00100: O = 32'b00000000000000000000000000010000;
            5'b00101: O = 32'b00000000000000000000000000100000;
            5'b00110: O = 32'b00000000000000000000000001000000;
            5'b00111: O = 32'b00000000000000000000000010000000;
            5'b01000: O = 32'b00000000000000000000000100000000;
            5'b01001: O = 32'b00000000000000000000001000000000;
            5'b01010: O = 32'b00000000000000000000010000000000;
            5'b01011: O = 32'b00000000000000000000100000000000;
            5'b01100: O = 32'b00000000000000000001000000000000;
            5'b01101: O = 32'b00000000000000000010000000000000;
            5'b01110: O = 32'b00000000000000000100000000000000;
            5'b01111: O = 32'b00000000000000001000000000000000;
            5'b10000: O = 32'b00000000000000010000000000000000;
            5'b10001: O = 32'b00000000000000100000000000000000;
            5'b10010: O = 32'b00000000000001000000000000000000;
            5'b10011: O = 32'b00000000000010000000000000000000;
            5'b10100: O = 32'b00000000000100000000000000000000;
            5'b10101: O = 32'b00000000001000000000000000000000;
            5'b10110: O = 32'b00000000010000000000000000000000;
            5'b10111: O = 32'b00000000100000000000000000000000;
            5'b11000: O = 32'b00000001000000000000000000000000;
            5'b11001: O = 32'b00000010000000000000000000000000;
            5'b11010: O = 32'b00000100000000000000000000000000;
            5'b11011: O = 32'b00001000000000000000000000000000;
            5'b11100: O = 32'b00010000000000000000000000000000;
            5'b11101: O = 32'b00100000000000000000000000000000;
            5'b11110: O = 32'b01000000000000000000000000000000;
            5'b11111: O = 32'b10000000000000000000000000000000;
    endcase
    end
    else 
        O = 32'b00000000000000000000000000000000;
end
endmodule

/************************************************************************************************************************************************************************************************************************************************************************/

//Registers:
// Descripcion:el módulo Register se encarga de cargar los datos de entrada Ds en el registro Q en la posición indicada 
// por la señal Ld cuando ésta es diferente de 0, y de proporcionar como salida Qs los bits [31:1] del registro Q. 
// La señal Clk es la señal de reloj que se utiliza para sincronizar el registro.

module Register (
    input [31:0] Ds, // Entrada de datos que se cargarán en el registro.
    output reg [31:0] Qs, // Salida del registro.
    input Ld, // Señal que indica cuándo cargar los datos en el registro. //LE en el documento
    input Clk // Señal de reloj.
);

always@(posedge Clk) begin
    if(Ld) Qs <=Ds;
end

endmodule

/************************************************************************************************************************************************************************************************************************************************************************/

//Multiplexors:
// Descripcion: El módulo Multiplexer_32x1 toma dos entradas de 32 bits llamadas S y D, y produce una salida de 32 bits llamada Y.
// La entrada S especifica qué uno de los 32 bits en D se debe seleccionar y enviar a la salida Y.

module Multiplexer_32x1 (
input [4:0] S, // Entrada selectora de 5 bits
  input [31:0] D0, D1, D2, D3, D4, D5, D6, D7, D8, D9, D10, D11, D12, D13, D14, D15, D16, D17, D18, D19, D20, D21, D22, D23, D24, D25, D26, D27, D28, D29, D30, D31, // Entrada de datos de 32 bits
output reg [31:0] Y // Salida de datos de 32 bits
);

always @ (*)
begin
case(S)
5'b00000: Y = D0;
5'b00001: Y = D1;
5'b00010: Y = D2;
5'b00011: Y = D3;
5'b00100: Y = D4;
5'b00101: Y = D5;
5'b00110: Y = D6;
5'b00111: Y = D7;
5'b01000: Y = D8;
5'b01001: Y = D9;
5'b01010: Y = D10;
5'b01011: Y = D11;
5'b01100: Y = D12;
5'b01101: Y = D13;
5'b01110: Y = D14;
5'b01111: Y = D15;
5'b10000: Y = D16;
5'b10001: Y = D17;
5'b10010: Y = D18;
5'b10011: Y = D19;
5'b10100: Y = D20;
5'b10101: Y = D21;
5'b10110: Y = D22;
5'b10111: Y = D23;
5'b11000: Y = D24;
5'b11001: Y = D25;
5'b11010: Y = D26;
5'b11011: Y = D27;
5'b11100: Y = D28;
5'b11101: Y = D29;
5'b11110: Y = D30;
5'b11111: Y = D31;
endcase
end
endmodule

// Condition Handler and Program Status Register Implementation
/************************************************************************************************************************************************************************************************************************************************************************/

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

//Partes del RAM/ROM Implementation
/************************************************************************************************************************************************************************************************************************************************************************/

// Instruction Memory module
module ROM (input [8:0] address, output reg [31:0] DataOut);
    reg [7:0] mem [0:511]; // 512x8 ROM
    always @(address) begin
        DataOut = {mem[address], mem[address+1], mem[address+2], mem[address+3]};
    end
endmodule

module RAM(output reg[31:0] DataOut, input RW, input[8:0] address, input[31:0] DataIn, input [1:0] Size, input SE, input E);//inicacion de variables

  reg[7:0] mem[0:511]; // RAM 512x8
  reg [31:0] temp;
  
  always @ (RW, address, DataIn, Size, SE, E)       
    
        case(Size)
        2'b00://localizaciones de byte para leer y escribir 
          if (RW==1 && E ==1) //When Write 
            begin
              mem[address] = DataIn[7:0]; 
            end
          else if (RW==0 && E==1 && SE == 0)
            begin
                DataOut = {24'b000000000000000000000000, mem[address]};
            end  
          else if (RW==0 && E==1 && SE ==1)begin
            temp = {24'b000000000000000000000000, mem[address]};
            if (temp[7]==1) DataOut = {24'b111111111111111111111111, mem[address]};
            else DataOut = temp;
          end 
        2'b01: //localizaciones de halfword para leer y escribir
            if (RW==1 && E ==1) //When Write 
            begin
              mem[address] = DataIn[15:8];
              mem[address+1] = DataIn[7:0]; 
            end
          else if (RW==0 && E==1 && SE == 0)
            begin
              DataOut = {16'b0000000000000000, mem[address], mem[address+1]};
            end  
          else if (RW==0 && E==1 && SE ==1)begin
            temp = {16'b0000000000000000, mem[address], mem[address+1]};
            if (temp[15]==1) DataOut = {16'b1111111111111111, mem[address], mem[address+1]};
            else DataOut = temp;
          end 
        2'b10: //localizaciones de word para leer y escribir
          if (RW == 1 && E== 1) //When Write 
            begin
                mem[address] = DataIn[31:24];
                mem[address + 1] = DataIn[23:16];
                mem[address + 2] = DataIn[15:8]; 
                mem[address + 3] = DataIn[7:0]; 
            end                 
            else //When Read
            begin
                DataOut = ({mem[address + 0], mem[address + 1], mem[address + 2], mem[address + 3]}); // 4 espacios en memoria
            end    
    endcase  
endmodule

//Partes del ALU/Source Operan2 Handler Implementation
/************************************************************************************************************************************************************************************************************************************************************************/

// ALU module
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

//Source Operand2 Handler
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


/************************************************************************************************************************************************************************************************************************************************************************/
