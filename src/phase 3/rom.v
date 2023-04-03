// ROM Testbench
module rom_tb;

// Parameters for Preload of ROM
reg [8:0] PC_Out;
reg [31:0] DataIn;
integer file, code;
wire [31:0] DataOut;
  
ROM Instruction_Memory(
    PC_Out, // Input
    DataOut // Output
);
  
// Preload Instruction Memory
initial begin
    file = $fopen("control_system_test.txt","r+b");
    PC_Out =  8'b00000000;
        while (!$feof(file)) begin 
            code = $fscanf(file, "%b", DataIn);
            Instruction_Memory.mem[PC_Out] = DataIn;
            PC_Out = PC_Out + 1;
        end
    $fclose(file);
end
initial begin 
    PC_Out = 0;  
  	#1 PC_Out = 4;
    #2 PC_Out = 8;
    #3 PC_Out = 12;
  	#4 PC_Out = 16;
    #5 PC_Out = 20;
    #6 PC_Out = 24;
    #7 PC_Out = 28;
    #8 PC_Out = 32;
    #9 PC_Out = 36;
  	#10 PC_Out = 40;
    #11 PC_Out = 44;
    #12 PC_Out = 48;
end
initial begin
  $monitor("Adress (PC_Out) = %d, Instruction = %b", PC_Out, DataOut);
end

endmodule

// Instruction Memory module
module ROM (input [8:0] address, output reg [31:0] DataOut);
    reg [7:0] mem [0:511]; // 512x8 ROM
    always @(address) begin
        DataOut = {mem[address], mem[address+1], mem[address+2], mem[address+3]};
    end
endmodule