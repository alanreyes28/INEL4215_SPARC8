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
    end
    initial begin
        $display("\nControl Unit results:");
        $monitor("I31 = %b, I30 = %b, I24 = %b, I13 = %b, ID_Load_Instr = %b,\nID_Jumpl_Instr = %b, ID_B_Instr = %b, ID_Call_Instr = %b,\nID_Load_CallOrJumpl_Instr = %b, ID_ALU_OP = %b, ID_Instr_Alter_CC = %b\n", I31, I30, I24, I13, ID_Load_Instr, ID_Jumpl_Instr, ID_B_Instr, ID_Call_Instr, ID_Load_CallOrJumpl_Instr, ID_ALU_OP, ID_Instr_Alter_CC);
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
        ID_Instr_Alter_CC = 0; // Preliminary value if conditions met then 
        opcode = {Instr[31:30],Instr[24:19]}; // Join op and op3
        case(opcode) 
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
            // TODO: Continue for logic, shifts, load/store, etc.
            default: ID_ALU_OP = 4'b1101;
        endcase
        // TODO: Set values for ID_RF_Enable, RAM_Enable, RAM_RW, RAM_SE, RAM_Size
    end
endmodule
