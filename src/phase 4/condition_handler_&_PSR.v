// Condition Handler and Program Status Register testbench

module condition_handler_program_status_reg_testbench;
    reg Z, N, C, V, LE, Clr,Clk;
    wire [3:0] PSR_Out;
    wire bit_C;
    Program_Status_Register psr(PSR_Out, bit_C, Z, N, C, V, LE, Clr, Clk);

    initial #10 $finish;

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