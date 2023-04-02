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

module test_bench;

reg [8:0] address;
reg [31:0] DataIn;
integer file, code;
wire [31:0] DataOut;

ROM ram (
  address,
  DataOut
); // apunta a las variables del modulo de instrucciones
initial begin
        file = $fopen("textfile.txt","r+b"); // se lee el text file 
        address =  8'b00000000;
            while (!$feof(file)) begin 
              code = $fscanf(file, "%b", DataIn);
              ram.mem[address] = DataIn;
            address = address + 1;
        end
    $fclose(file);  
    address = #1 8'b00000000;
end
  initial begin 
    address = 0;  
  	#1 
    address = 4;
    #1
    address = 8;
    #1
    address = 12;
  end
  initial begin
    $monitor("A = %d,I = %h", address, DataOut);
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

module testbench;
 reg  RW;
 reg [31:0] DataIn; 
 reg [1:0] Size; 
 reg  SE; 
 reg  E;
 reg [8:0] address; 
wire [31:0] DataOut;
integer file, code;

RAM ram( 
DataOut, 
RW, 
address, 
DataIn, 
Size,
SE, 
E);

initial begin
        file = $fopen("textfile.txt","r+b"); // se lee el text file 
        address =  8'b00000000;
            while (!$feof(file)) begin 
              code = $fscanf(file, "%b", DataIn);
              ram.mem[address] = DataIn;
            address = address + 1;
        end
    $fclose(file);  
    address = #1 8'b00000000;
end
  initial begin 
    RW = 1'b0;
    SE = 1'b0;
    address = 0; 
    E = 1'b1;
    Size = 2'b10; //lectura de word
  	#1 
    address = 4;
    #1 
    address = 8;
    #1 
    address = 12;
    #1
    Size = 2'b00;
    address = 0;//lectura de byte 
    #1 
    Size = 2'b01;
    address = 2; //lectura de halfword
    #1 
    address = 4;
    #1
    Size = 2'b00;
    address = 0;
    SE = 1'b1; // lectura de byte con signo
    #1
    Size = 2'b01;
    address = 2;
    SE = 1'b1;//lectura de halfword con signo
    #1 
    address = 4;
    #1
    SE = 1'b0;
    Size = 2'b00;  //escritura de byte
    RW = 1'b1;
    address = 0;
    DataIn = 8'hCD;
    #1
    Size = 2'b01; //escritura de halfword
    address = 2;
    DataIn = 16'h35AD;
   	#1
    address = 4;
    DataIn = 16'hAB86;
    #1
    Size = 2'b10; //escritura de word
    address = 8;
    DataIn = 32'h8567C6Ab;
    #1
    Size = 2'b10;
    RW = 2'b00; //lectura de word
    address = 0;
    #1
    address = 4;
    #1
    address = 8;
  end
  initial begin
    $monitor("A = %d, DO = %h, Size = %b, R/W = %b, E = %b, SE = %b\n", address, DataOut, Size, RW, E, SE);
  end
endmodule