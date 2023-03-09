`timescale 1 ns/10 ps

module right_barrel_shifter_tb;
    reg [31:0]data;
    reg [4:0] shamt;
    wire [31:0] result;

    right_barrel_shifter testUnit(data, shamt, result);

    initial // initial block executes only once
        begin
            // values for a and b
            data = 1;
            shamt = 1;
            #80; // wait for period 

            $display("Data:%b, Shamt:%b => result:%b",data, shamt, result);
            $finish;

        end


endmodule