`timescale 1ns/1ps
module tb_mit_trajectory;
 reg clk=0;always #5 clk=~clk;
 reg resetn=0,clear=0,access=0,wr=0,ready=1,guard_ok=1,allow_start=1;
 reg [31:0] now_us=32'hfffff000,wdata;
 reg [11:0] addr=0;reg [6:0] mask=127;
 wire [31:0] rdata,fault,gripper_target;wire error,valid,running;
 wire [111:0] encoded_position,encoded_velocity,encoded_torque;
 wire [223:0] encoded_gains;
 wire [191:0] packet;
 reg [31:0] words[0:219];reg [95:0] expected[0:34];
 integer count=0,j;reg monitor=0;
 mit_trajectory_stream dut(.*);
 always @(posedge clk)if(resetn)now_us<=now_us+1;
 task put(input [11:0] a,input [31:0] d);
 begin @(negedge clk);access=1;wr=1;addr=a;wdata=d;@(negedge clk);access=0;wr=0;end endtask
 task load;
 begin for(j=0;j<220;j=j+1)put(12'hc00+j*4,words[j]);put(12'hb60,1);end endtask
 always @(negedge clk)if(valid && ready && monitor)begin
  if(count<35 && {packet[127:96],packet[63:0]}!==expected[count])
   $fatal(1,"MIT sample %d got %h %h expected %h",count,packet[127:96],packet[63:0],expected[count]);
  if(count>=35 && {packet[127:96],packet[63:0]}!==expected[28+count%7])$fatal(1,"MIT endpoint hold");
  if(packet[95:64]!==8)$fatal(1,"MIT DLC");count=count+1;
 end
 initial begin
  $readmemh("contest/kv260_can/tb/mit_stage.mem",words);$readmemh("contest/kv260_can/tb/mit_packets.mem",expected);
  repeat(5)@(negedge clk);resetn=1;addr=12'hb68;#1;if(rdata!==32'h70003 || error)$fatal(1,"MIT capability");
  put(12'hb60,1);if(dut.rejects!=1)$fatal(1,"MIT partial commit");
  load;allow_start=0;put(12'hb60,2);if(running || dut.rejects!=2)$fatal(1,"MIT concurrent start");
  allow_start=1;put(12'hb60,2);monitor=1;
  wait(count>=14);load;put(12'hb60,4);
  wait(count>=42);monitor=0;
  if(fault || !dut.holding || dut.completed!=1 || dut.queued)$fatal(1,"MIT hold/pause state");
  @(negedge clk);clear=1;repeat(2)@(negedge clk);clear=0;
  words[2*56]=2*16777216;load;put(12'hb60,2);wait(fault!=0);
  if(fault!=3 || dut.samples!=0 || valid)$fatal(1,"MIT atomic torque bounds");
  @(negedge clk);clear=1;repeat(2)@(negedge clk);clear=0;
  words[2*56]=16777216/2;words[56]=16777216*3/4;load;put(12'hb60,2);wait(fault!=0);
  if(fault!=6 || dut.completed!=1)$fatal(1,"MIT moving queue underrun not faulted");
  $display("TEST_PASS MIT trajectory: 35 position/velocity/torque HLS samples, hybrid gripper, 220-word atomic commit, pause/hold, torque rejection, moving underrun");$finish;
 end
 initial begin #10000000;$fatal(1,"MIT watchdog");end
endmodule
