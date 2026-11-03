`timescale 1ns/1ps
// Inject at the engine-to-FIFO boundary; wire reception is tested separately.
module tb_rx_buffer;
 reg clk=0;always #5 clk=~clk;
 reg resetn=0,valid=0,wr=0,inject=0;
 reg [11:0] addr=0;reg [31:0] data=0;reg[159:0] payload=0;
 wire done,err,tx;wire[31:0] result;
 can_engine dut(.clk(clk),.resetn(resetn),.can_rx(1'b1),.can_tx(tx),
  .req_valid(valid),.req_write(wr),.req_addr(addr),.req_data(data),.req_strb(4'hf),
  .req_done(done),.rsp_data(result),.rsp_error(err),.debug_us(),.debug_status());
 task access(input bit write_en,input[11:0] a,input[31:0] d,output[31:0] r);
 integer timeout_count;
 begin
  @(negedge clk);addr=a;data=d;wr=write_en;valid=1;timeout_count=0;
  while(!done)begin @(negedge clk);timeout_count=timeout_count+1;if(timeout_count>20000)$fatal(1,"RX buffer request timeout");end
  r=result;if(err)$fatal(1,"RX buffer AXI error at %h",a);
  valid=0;repeat(4)@(negedge clk);
 end endtask
 reg [31:0] value;integer i;
 initial begin
  repeat(20)@(negedge clk);resetn=1;repeat(20)@(negedge clk);
  force dut.rx_push=inject;force dut.rx_packet=payload;
  for(i=0;i<256;i=i+1)begin
   @(negedge clk);payload={32'(i+1000),32'(i),32'h108,32'(i^32'h12345678),32'(i^32'habcdef00)};inject=1;
  end
  @(negedge clk);inject=0;
  access(0,12'h41c,0,value);if(value!==32'h01000000)$fatal(1,"RX occupancy lost high count bits: %h",value);
  if(!dut.rx_full)$fatal(1,"256-frame FIFO not full");
  // An extra push must not overwrite existing unread frames.
  @(negedge clk);payload=160'hbad;inject=1;@(negedge clk);inject=0;
  for(i=0;i<256;i=i+1)begin
   access(0,12'h480,0,value);if(value!==i)$fatal(1,"RX order at %d got %h",i,value);
   access(0,12'h488,0,value);if(value!==(i^32'h12345678))$fatal(1,"RX payload hi");
   access(0,12'h48c,0,value);if(value!==(i^32'habcdef00))$fatal(1,"RX payload lo");
   access(0,12'h490,0,value);if(value!==i+1000)$fatal(1,"RX timestamp");
   access(1,12'h498,1,value);
  end
  access(0,12'h41c,0,value);if(value!==0 || !dut.rx_empty)$fatal(1,"RX not empty after drain");
  $display("TEST_PASS RX buffer: 256-frame capacity, 9-bit occupancy via unchanged AXI ABI, full protection, all payloads/timestamps/order");$finish;
 end
 initial begin #10000000;$fatal(1,"RX buffer timeout");end
endmodule
