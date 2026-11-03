`timescale 1ns/1ps
module tb_trajectory_wide;
 reg clk=0;always #5 clk=~clk;
 reg resetn=0,clear=0,access=0,wr=0,ready=1,guard_ok=1;
 reg [31:0] now_us=32'hfffff000,wdata;
 reg [11:0] addr=0;reg [6:0] mask=127;
 wire [31:0] rdata,fault;wire error,valid,running;
 wire [191:0] packet;wire [223:0] targets;
 reg [31:0] coefficients[0:55],expected[0:146];
 integer count=0,j,k;reg monitor=0;
 trajectory_stream dut(.*);
 always @(posedge clk)if(resetn)now_us<=now_us+1;
 task put(input [11:0] a,input [31:0] d);
 begin @(negedge clk);access=1;wr=1;addr=a;wdata=d;@(negedge clk);access=0;wr=0;end endtask
 task load_wide(input bit bad_bound);
 begin
  for(j=0;j<56;j=j+1)put(12'ha00+j*4,coefficients[j]);
  for(j=0;j<7;j=j+1)begin
   put(12'hae0+j*4,bad_bound && j==6 ? 16777216:-16777216);
   put(12'hafc+j*4,16777216);
  end
  put(12'hb18,20);put(12'hb1c,16777216/20);put(12'hb20,16777216%20);
 end endtask
 always @(negedge clk)if(valid && ready && monitor)begin
  if(packet[127:96]!==32'h1200fd01+count%7 || packet[95:32]!==64'h0000000816700000)$fatal(1,"wide CAN encoding");
  if(count<147)begin
   if({packet[7:0],packet[15:8],packet[23:16],packet[31:24]}!==expected[count])
    $fatal(1,"wide sample %d got %h expected %h",count,{packet[7:0],packet[15:8],packet[23:16],packet[31:24]},expected[count]);
   if(dut.phase!==dut.tick*64'd16777216/20)$fatal(1,"wide fractional phase");
  end else if({packet[7:0],packet[15:8],packet[23:16],packet[31:24]}!==expected[140+count%7])
   $fatal(1,"legacy after wide reused high coefficients");
  count=count+1;
 end
 initial begin
  $readmemh("contest/kv260_can/tb/wide_coefficients.mem",coefficients);
  $readmemh("contest/kv260_can/tb/wide_expected.mem",expected);
  repeat(5)@(negedge clk);resetn=1;addr=12'h828;#1;
  if(rdata!==32'h00070008 || error)$fatal(1,"wide capability");
  put(12'h820,8);if(dut.rejects!=1 || dut.queued)$fatal(1,"partial wide accepted");
  load_wide(0);put(12'hb20,0);put(12'h820,8);
  if(dut.rejects!=2 || dut.queued)$fatal(1,"bad phase accepted");
  put(12'hb20,16777216%20);put(12'h820,8);
  if(!dut.queued)$fatal(1,"wide not committed");
  put(12'h820,2);monitor=1;
  wait(count>=21);
  // Queue a legacy constant segment after a high-order segment. All unused
  // high coefficients must be cleared by the compatibility commit mapping.
  for(j=0;j<7;j=j+1)begin
   put(12'h900+j*16,j*16777216/32);
   put(12'h904+j*16,0);put(12'h908+j*16,0);put(12'h90c+j*16,0);
   put(12'h970+j*4,-16777216);put(12'h98c+j*4,16777216);
  end
  put(12'h9a8,2);put(12'h9ac,16777216/2);put(12'h9b0,0);put(12'h820,1);
  if(!dut.queued)$fatal(1,"legacy not queued");
  wait(count>=175);monitor=0;
  if(fault || !dut.holding || dut.completed!=2 || dut.queued)$fatal(1,"wide/legacy completion");
  @(negedge clk);clear=1;repeat(2)@(negedge clk);clear=0;
  load_wide(1);put(12'h820,8);put(12'h820,2);wait(fault!=0);
  if(fault!=3 || dut.samples!=0 || valid)$fatal(1,"wide atomic bounds bypass");
  $display("TEST_PASS trajectory_wide 147 degree-seven samples, legacy compatibility, phase validation, partial commit, capability and atomic bounds");$finish;
 end
 initial begin #10000000;$fatal(1,"wide watchdog");end
endmodule
