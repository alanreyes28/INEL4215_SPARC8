// Testing bench for Control Unit module
module control_unit_testbench;
    wire I31, I30, I24, I13, ID_Load_Instr, ID_RF_Enable;
    wire RAM_Enable, RAM_RW, RAM_SE, ID_Jumpl_Instr, ID_Instr_Alter_CC;
    wire ID_B_Instr, ID_Call_Instr;
    wire [1:0] RAM_Size, ID_Load_CallOrJumpl_Instr;
    wire [3:0] ID_ALU_OP;
    reg [31:0] Instr; // Instruction
    control_unit cu(I31, I30, I24, I13, ID_Load_Instr, ID_RF_Enable, RAM_Enable, RAM_RW,
                    RAM_SE, ID_Jumpl_Instr, ID_Instr_Alter_CC, ID_B_Instr, ID_Call_Instr,
                    RAM_Size, ID_Load_CallOrJumpl_Instr, ID_ALU_OP, Instr);

    initial #100 $finish;
    initial begin
        // ALU opcodes
        Instr = 32'b11000100000010000000000000000001;
        #10 Instr = 32'b10001010000000000000000000000000;
        #20 Instr = 32'b10001010100000000000000000000000;
        #30 Instr = 32'b10001011001100000000000000000000;
    end
    initial begin
        $display("\nControl Unit results:");
        $monitor("I31 = %b, I30 = %b, I24 = %b, I13 = %b, ID_Load_Instr = %b,\nID_Jumpl_Instr = %b, ID_B_Instr = %b, ID_Call_Instr = %b,\nID_Load_CallOrJumpl_Instr = %b, ID_ALU_OP = %b, ID_Instr_Alter_CC = %b\nID_RF_Enable = %b, RAM_Enable = %b, RAM_RW = %b, RAM_SE = %b, RAM_Size = %b\n", I31, I30, I24, I13, ID_Load_Instr, ID_Jumpl_Instr, ID_B_Instr, ID_Call_Instr, ID_Load_CallOrJumpl_Instr, ID_ALU_OP, ID_Instr_Alter_CC, ID_RF_Enable, RAM_Enable, RAM_RW, RAM_SE, RAM_Size);
    end
endmodule

// Control Unit of the Pipelined Processing Unit for SPARC module
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
