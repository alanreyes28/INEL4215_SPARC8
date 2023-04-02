// Testbench for MuMultiplexer module for Control Unit
module main;
    wire I31_OUT, I30_OUT, I24_OUT, I13_OUT, ID_Load_Instr_OUT, ID_RF_Enable_OUT;
    wire [3:0] ID_ALU_OP_OUT;
    wire ID_Jumpl_Instr_OUT, ID_Instr_Alter_CC_OUT;
    wire [1:0] ID_Load_CallOrJumpl_Instr_OUT;
    reg I31_IN, I30_IN, I24_IN, I13_IN, ID_Load_Instr_IN, ID_RF_Enable_IN;
    reg [3:0] ID_ALU_OP_IN;
    reg ID_Jumpl_Instr_IN, ID_Instr_Alter_CC_IN;
    reg [1:0] ID_Load_CallOrJumpl_Instr_IN;
    reg select;
    ctrl_unit_mux_2x1 mx(I31_OUT, I30_OUT, I24_OUT, I13_OUT, ID_Load_Instr_OUT, ID_RF_Enable_OUT, ID_ALU_OP_OUT, ID_Jumpl_Instr_OUT, ID_Instr_Alter_CC_OUT, ID_Load_CallOrJumpl_Instr_OUT, I31_IN, I30_IN, I24_IN, I13_IN, ID_Load_Instr_IN, ID_RF_Enable_IN, ID_ALU_OP_IN, ID_Jumpl_Instr_IN, ID_Instr_Alter_CC_IN, ID_Load_CallOrJumpl_Instr_IN, select);
    initial begin
        I31_IN =1;
        I30_IN = 1; 
        I24_IN = 1;
        I13_IN = 1;
        ID_Load_Instr_IN = 1;
        ID_RF_Enable_IN = 1;
        ID_ALU_OP_IN = 4'b1010;
        ID_Jumpl_Instr_IN = 1; 
        ID_Instr_Alter_CC_IN = 1;
        ID_Load_CallOrJumpl_Instr_IN = 2'b11;
        select = 1;
        #3 select = 0;
    end
  initial 
    begin
      $display("Results:");
      $monitor("Outputs: I31_OUT = %b, I30_OUT = %b, I24_OUT = %b, I13_OUT = %b, ID_Load_Instr_OUT = %b, ID_RF_Enable_OUT = %b, ID_ALU_OP_OUT = %b, ID_Jumpl_Instr_OUT = %b, ID_Instr_Alter_CC_OUT = %b, ID_Load_CallOrJumpl_Instr_OUT = %b\nInputs: I31_IN = %b, I30_IN = %b, I24_IN = %b, I13_IN = %b, ID_Load_Instr_IN = %b, ID_RF_Enable_IN = %b, ID_ALU_OP_IN = %b, ID_Jumpl_Instr_IN = %b, ID_Instr_Alter_CC_IN = %b, ID_Load_CallOrJumpl_Instr_IN = %b, select = %b\n", I31_OUT, I30_OUT, I24_OUT, I13_OUT, ID_Load_Instr_OUT, ID_RF_Enable_OUT, ID_ALU_OP_OUT, ID_Jumpl_Instr_OUT, ID_Instr_Alter_CC_OUT, ID_Load_CallOrJumpl_Instr_OUT, I31_IN, I30_IN, I24_IN, I13_IN, ID_Load_Instr_IN, ID_RF_Enable_IN, ID_ALU_OP_IN, ID_Jumpl_Instr_IN, ID_Instr_Alter_CC_IN, ID_Load_CallOrJumpl_Instr_IN, select);
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
