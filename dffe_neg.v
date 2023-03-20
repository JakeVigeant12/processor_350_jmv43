module dffe_neg (q, d, clk, en, clr);
   input d, clk, en, clr;
   wire clr;
   output q;
   reg q;

   initial
   begin
       q = 1'b0;
   end

   //Set value of q on neg edge of the clock or clear
   always @(negedge clk or negedge clr) begin
       if (clr) begin
           q <= 1'b0;
       end else if (en) begin
           q <= d;
       end
   end
endmodule