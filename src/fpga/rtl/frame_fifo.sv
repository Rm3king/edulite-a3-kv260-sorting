`timescale 1ns/1ps
module frame_fifo #(parameter WIDTH=160, parameter LOG_DEPTH=4)(
    input wire clk, resetn, clear,
    input wire push, input wire [WIDTH-1:0] din, output wire full,
    input wire pop, output wire [WIDTH-1:0] dout, output wire empty,
    output reg [LOG_DEPTH:0] count
);
    reg [WIDTH-1:0] mem[0:(1<<LOG_DEPTH)-1];
    reg [LOG_DEPTH-1:0] wp,rp;
    wire take = pop && !empty;
    wire put = push && !full;
    assign full = count == (1<<LOG_DEPTH);
    assign empty = count == 0;
    assign dout = mem[rp];
    always @(posedge clk) begin
        if(!resetn || clear) begin wp<=0; rp<=0; count<=0; end
        else begin
            if(put) begin mem[wp]<=din; wp<=wp+1'b1; end
            if(take) rp<=rp+1'b1;
            case({put,take})
                2'b10:count<=count+1'b1;
                2'b01:count<=count-1'b1;
                default:count<=count;
            endcase
        end
    end
endmodule
