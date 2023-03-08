module register(clock, write_enable, input_data, output_data, reset);
    input clock, write_enable, reset;
    input [31:0] input_data;
    output [31:0] output_data;

    dffe_ref flop1(output_data[0],input_data[0], clock, write_enable, reset);
    genvar i;
    generate
        for (i=0; i<32; i=i+1) begin: loop1
            dffe_ref a_dff(.d(input_data[i]), .q(output_data[i]), .clr(reset), .en(write_enable), .clk(clock));
        end
    endgenerate
    
endmodule