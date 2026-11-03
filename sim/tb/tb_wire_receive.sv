`timescale 1ns/1ps
`include "wire_lengths.vh"
module tb_wire_receive;
 reg clk=0;always #5 clk=~clk;
 reg resetn=0,wire_bit=1,valid=0,wr=0;
 reg[11:0]addr=0;reg[31:0]data=0;
 wire done,err,tx;wire[31:0]result;
 reg valid_bits[0:`VALID_BITS-1],bad_bits[0:`BAD_BITS-1];
 can_engine dut(.clk(clk),.resetn(resetn),.can_rx(wire_bit),.can_tx(tx),
 .req_valid(valid),.req_write(wr),.req_addr(addr),.req_data(data),.req_strb(4'hf),
 .req_done(done),.rsp_data(result),.rsp_error(err),.debug_us(),.debug_status());
 task access(input bit write_en,input[11:0]a,input[31:0]d,output[31:0]r);
 begin
 @(negedge clk);valid=1;wr=write_en;addr=a;data=d;
 while(!done)@(negedge clk);
 if(err)$fatal(1,"MMIO rejected");r=result;valid=0;repeat(4)@(negedge clk);
 end endtask
 task put(input[11:0]a,input[31:0]d);reg[31:0]r;begin access(1,a,d,r);end endtask
 task check(input[11:0]a,input[31:0]expected);reg[31:0]r;
 begin access(0,a,0,r);if(r!==expected)$fatal(1,"addr=%h expected=%h actual=%h",a,expected,r);end endtask
 initial begin
 $readmemb("contest/kv260_can/result/wire_valid.mem",valid_bits);
 $readmemb("contest/kv260_can/result/wire_bad_crc.mem",bad_bits);
 repeat(20)@(negedge clk);resetn=1;
 put('h07c,'h80);put('h000,'h09);put('h018,4);put('h01c,'h16);
 for(integer i=16;i<20;i=i+1)put(i*4,0);
 for(integer i=20;i<24;i=i+1)put(i*4,255);
 put(0,'h0a); // listen-only: no ACK and no active error flag
 put('h404,4);repeat(3000)@(negedge clk);
 for(integer i=0;i<`VALID_BITS;i=i+1)begin wire_bit=valid_bits[i];repeat(100)@(negedge clk);end
 wire_bit=1;repeat(5000)@(negedge clk);
 check('h428,1);check('h480,'h028002fd);check('h488,'h80001234);check('h48c,'h56780190);check('h49c,2);
 put('h498,1);
 for(integer i=0;i<`BAD_BITS;i=i+1)begin wire_bit=bad_bits[i];repeat(100)@(negedge clk);end
 wire_bit=1;repeat(5000)@(negedge clk);
 check('h428,1);check('h49c,2);check('h41c,0);
 if(tx!==1)$fatal(1,"listen-only TX active");
 $display("TEST_PASS independent wire serializer: good extended frame accepted; wrong CRC rejected; passive TX");$finish;
 end
 initial begin #3000000;$fatal(1,"wire watchdog");end
endmodule
