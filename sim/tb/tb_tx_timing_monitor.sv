`timescale 1ns/1ps
module tb_tx_timing_monitor;
 reg clk=0;always #5 clk=~clk;
 reg resetn=0,event_valid=0,access=0,wr=0;
 reg [31:0] event_id=0,event_hi=0,event_us=0,event_due=0,wdata=0;
 reg [11:0] addr=0;wire [31:0] rdata;
 tx_timing_monitor dut(.*);
 task write_reg(input [11:0] a,input [31:0] d);
 begin @(negedge clk);access=1;wr=1;addr=a;wdata=d;@(negedge clk);access=0;wr=0;end endtask
 task expect_reg(input [11:0] a,input [31:0] d);
 begin repeat(4)@(negedge clk);addr=a;#1;if(rdata!==d)$fatal(1,"addr=%h got=%d expected=%d",a,rdata,d);end endtask
 task sample(input [31:0] ident,input [31:0] stamp,input [31:0] due);
 begin @(negedge clk);event_valid=1;event_id=ident;event_us=stamp;event_due=due;
 @(negedge clk);event_valid=0;end endtask
 integer k;
 initial begin
  repeat(3)@(negedge clk);resetn=1;
  expect_reg(12'h100,32'h54494d31);write_reg(12'h104,3);
  // Including wrap, signed interval errors, and a missing-cycle gap.
  sample(32'h01000001,32'hfffffff0,32'hffffffe6);
  sample(32'h01000001,32'h00001378,32'h00001364); // +5000 us
  sample(32'h01000001,32'h0000270a,32'h000026ec); // +5010 us
  sample(32'h01000001,32'h00004e1a,32'h00004df2); // +10000 us
  expect_reg(12'h110,4);expect_reg(12'h114,3);expect_reg(12'h118,5000);expect_reg(12'h11c,10000);
  expect_reg(12'h120,5000);expect_reg(12'h124,5010);expect_reg(12'h12c,25000100);
  expect_reg(12'h134,40);expect_reg(12'h13c,1);expect_reg(12'h140,1);
  expect_reg(12'h180,1);expect_reg(12'h18c,1);expect_reg(12'h1ac,1);
  sample(32'h1100fd01,32'h10000,0);expect_reg(12'h110,4); // reads do not count
  event_hi=32'h16700000;sample(32'h1200fd07,100,90);write_reg(12'h108,6);expect_reg(12'h110,1);
  write_reg(12'h104,0);sample(32'h1200fd07,5100,5090);expect_reg(12'h110,1);
  write_reg(12'h104,3);expect_reg(12'h110,0);expect_reg(12'h114,0);
  for(k=0;k<300;k=k+1)sample(32'h1200fd07,100+k*5000,90+k*5000);
  expect_reg(12'h110,300);expect_reg(12'h114,299);expect_reg(12'h180,299);expect_reg(12'h120,0);
  $display("TEST_PASS tx_timing_monitor: exact statistics, wrap, per-axis isolation, histogram, read-frame rejection, non-gating reset");$finish;
 end
endmodule
