// Control System Testbench
module Sistema_De_Control_tb;

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
    PC_Out, 4, 4'b0000,  Cin // Inputs
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
    I31_0 // Input
);

control_unit_mux_2x1 CU_MUX(
    I31_OUT, I30_OUT, I24_OUT, I13_OUT, ID_Load_Instr_OUT, ID_RF_Enable_OUT,
    ID_ALU_OP_OUT, ID_Jumpl_Instr_OUT, ID_Instr_Alter_CC_OUT, ID_Load_CallOrJumpl_Instr_OUT, // Outputs
    I31, I30, I24, I13, ID_Load_Instr, ID_RF_Enable, 
    ID_ALU_OP, ID_Jumpl_Instr, ID_Instr_Alter_CC, 
    ID_Load_CallOrJumpl_Instr, select // Inputs
); 

Pipeline_Register_IF_ID IF_ID (
    I21_0, I29_0, PC31_0, I29, I18_14, I4_0,
    I29_25, I28_25, I12_0, I31_0, I29_25_2, // Outputs
    InstuctionMemoryOut, PC, LE, Clr, Clk // Inputs
);

Pipeline_Register_ID_EX ID_EX (
    //Outputs Parte Amarilla
/**********************************/
    MX1_OUT, MX2_OUT,MX3_OUT,PC_OUT
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
    I12_0_IN, //pendiente a ver si el nombre de esto causa problemas (no creo)
    RD4_0_IN,
/**********************************/

    //Inputs Parte Gris
/**********************************/
    I31_OUT_MUX,I30_OUT_MUX,I24_OUT_MUX,I13_OUT_MUX,
    ID_Load_Instr_OUT_MUX,
    ID_ALU_OP_OUT_MUX,
    ID_RF_Enable_OUT_MUX, RAM_Enable_OUT_MUX,RAM_RW_OUT_MUX,RAM_SE_OUT_MUX,
    RAM_Size_OUT_MUX,
    ID_Jumpl_Instr_OUT_MUX,ID_Instr_Alter_CC_OUT_MUX,
    ID_Load_CallOrJumpl_Instr_OUT_MUX,
/**********************************/
    Clr, Clk
);

Pipeline_Register_EX_MEM EX_MEM (
    Out_Out, MX3_Out, PC_Out, RD_Out,
    ID_RF_enable_Out, RAW_Enable_Out,
    RAM_R/W_Out, RAM_SE_Out, RAM_Size_Out,
    ID_load_callOrJumpl_instr_Out, // Outputs
    Out_In, Z_In,V_In,N_In,C_In, MX3_In,
    PC_In, RD_In, ID_RF_enable_In, RAW_Enable_In,
    RAM_R/W_In, RAM_SE_In, RAM_Size_In,
    ID_load_callOrJumpl_instr_In, Clr, Clk //Inputs
);

Pipeline_Register_MEM_WB MEM_WB (
    PW_WB, RD_OUT, ID_RF_enable_OUT, // Ouputs,
    DO, MEM_RD, RD_IN, ID_RF_enable_IN, Clr, Clk // Inputs
);
endmodule

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
            // Load Integer Instructions
            8'b11001001: ID_ALU_OP = 4'b0000; // lsb -> A + B for ALU
            8'b11001010: ID_ALU_OP = 4'b0000; // ldsh -> A + B for ALU
            8'b11000000: ID_ALU_OP = 4'b0000; // ld -> A + B for ALU
            8'b11000001: ID_ALU_OP = 4'b0000; // ldub -> A + B for ALU
            8'b11000010: ID_ALU_OP = 4'b0000; // lduh -> A + B for ALU
            8'b11000011: ID_ALU_OP = 4'b0000; // ldd -> A + B for ALU
            8'b11001101: ID_ALU_OP = 4'b0000; // ldstub -> A + B for ALU
            8'b11001111: ID_ALU_OP = 4'b0000; // swap -> A + B for ALU
            // Store Integer Instructions
            8'b11000101: ID_ALU_OP = 4'b0000; // stb -> A + B for ALU
            8'b11000110: ID_ALU_OP = 4'b0000; // sth -> A + B for ALU
            8'b11000100: ID_ALU_OP = 4'b0000; // st -> A + B for ALU
            8'b11000111: ID_ALU_OP = 4'b0000; // std -> A + B for ALU
            // Jumpl Instruction
            8'b10111000: ID_ALU_OP = 4'b0000; // jumpl -> A + B for ALU
            default: ID_ALU_OP = 4'b0000;
        endcase
        if(Instr[31:30] == 2'b00 && Instr[24:22] == 3'b010) begin // Branch Instructions
            ID_ALU_OP = 4'b0000; // A + B for ALU. Passing a default operation
        end else if(Instr[31:30] == 2'b00 && Instr[24:22] == 3'b010) begin // Sethi Instructions
            ID_ALU_OP = 4'b1110; // sethi -> B for ALU. Let the ALU choose what the src op2 handler provides as input in B
        end else if(Instr[31:30] == 2'b01) begin // Call Instructions
            ID_ALU_OP = 4'b0000; // A + B for ALU. Passing a default operation
        end
        // RAM Signals:
        RAM_Enable = 1; // Always one if not there is no operation for RAM
        if( opcode == 8'b11000101 ||  opcode == 8'b11000110 ||  opcode == 8'b11000100 || opcode == 8'b11000111) begin  // Check if it's a store instruction
            RAM_RW = 1;
        end else begin
            RAM_RW = 0; // default case for load instruction
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
        ID_RF_Enable = 1; // TODO: Check cases for when it is 0
    end
endmodule

// Instruction Memory module
module ROM (input [8:0] address, output reg [31:0] DataOut);
    reg [7:0] mem [0:511]; // 512x8 ROM
    always @(address) begin
        DataOut = {mem[address], mem[address+1], mem[address+2], mem[address+3]};
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
        if(select) begin
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
                                output reg [29] I29,
                                output reg [18:14] I18_14,
                                output reg [4:0] I4_0,
                                output reg [29:25] I29_25,
                                output reg [28:25] I28_25,
                                output reg [12:0] I12_0,
                                output reg [31:0] I31_0, 
                                output reg [29:25] I29_25_2, //pal mux
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
            output reg [31:0] MX1_OUT, MX2_OUT,MX3_OUT,PC_OUT
            output reg [12:0] I12_0_OUT,
            output reg [4:0] RD4_0_OUT,
            /**********************************/

                //Outputs de parte Gris
            /**********************************/
            output I31_OUT_REG,I30_OUT_REG,I24_OUT_REG,I13_OUT_REG,
            output ID_Load_Instr_OUT_REG,
            output reg [4:0] ID_ALU_OP_OUT_REG,
            output ID_RF_Enable_OUT_REG, RAM_Enable_OUT_REG,RAM_RW_OUT_REG,RAM_SE_OUT_REG,
            output reg [1:0] RAM_Size_OUT_REG,
            output ID_Jumpl_Instr_OUT_REG,ID_Instr_Alter_CC_OUT_REG,
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
            input [4:0] ID_ALU_OP_OUT_MUX,
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
                                output ID_RF_enable_Out,
                                output RAW_Enable_Out,
                                output RAM_R/W_Out,
                                output RAM_SE_Out,
                                output reg [1:0] RAM_Size_Out,
                                output reg [1:0] ID_load_callOrJumpl_instr_Out,
                                input [31:0] Out_In,
                                input Z_In,V_In,N_In,C_In,
                                input [31:0] MX3_In,
                                input [31:0] PC_In,
                                input [4:0] RD_In,
                                input ID_RF_enable_In,
                                input RAW_Enable_In,
                                input RAM_R/W_In,
                                input RAM_SE_In,
                                input reg [1:0] RAM_Size_In,
                                input reg [1:0] ID_load_callOrJumpl_instr_In,
                                input Clr, Clk,);
    always @ (posedge Clk) begin //rising edge triggered Register
        if (Clr) begin
            Out_Out <= 32'h00000000;
            MX3_Out <= 32'h00000000;
            PC_Out <= 32'h00000000;
            RD_Out <= 4'h0000;
            ID_RF_enable_Out <= 1'b0;
            RAW_Enable_Out <= 1'b0;
            RAM_R/W_Out <= 1'b0;
            RAM_R/W_Out <= 1'b0;
            RAM_SE_Out  <= 1'b0;
            RAM_Size_Out <= 2'b00;
            ID_load_callOrJumpl_instr_Out <= 2'b00;
        end
        else begin
            Out_Out <= Out_In[31;0]; // estar pendiente cuando inicialice el LE para el testbench, ya que el registro PC lleva LE y los demas se le deberia asignar un valor constante de 1 (que siempre cargue los valores)
            PC_Out <= PC_In[31;0];
            MX3_Out <= MX3_In[31:0];
            RD_Out <= RD_In[31:0];
            ID_RF_enable_Out <= ID_RF_enable_In;
            RAW_Enable_Out <= RAW_Enable_In;
            RAM_R/W_Out <=RAM_R/W_In;
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
            PW_WB <= MEM_WB;
            RD_OUT <= RD_IN;
            ID_RF_enable_OUT <= ID_RF_enable_IN;
        end
    end
endmodule

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
