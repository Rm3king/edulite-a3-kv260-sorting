`timescale 1ns/1ps
// Bounded online correction (840..894): capability, config locks and ceilings, clamp, per-tick slew,
// lease freeze, disable-to-zero, gripper never offset, fault 6 outside the segment bounds.
module tb_trajectory_correction;
 reg clk=0;always #5 clk=~clk;
 reg resetn=0,clear=0,access=0,wr=0,ready=1,guard_ok=1;
 reg [31:0] now_us=0,wdata;
 reg [11:0] addr=0;
 reg [6:0] mask=127;
 wire [31:0] rdata,fault;wire error,valid,running;
 wire [191:0] packet;wire [223:0] targets;
 trajectory_stream dut(.*);
 always @(posedge clk)if(resetn)now_us<=now_us+4;    // 4 us per clock: a 5 ms tick is 1250 clocks, 1125 clocks of deadline
 localparam integer ONE=16777216;
 localparam integer V=ONE/4;                    // nominal 0.25 rad on every joint
 integer j,k,m,checks=0,ticks=0;   // m: per-tick monitor only (k/j belong to the stimulus)
 reg signed [31:0] last[0:6];reg have_last=0;
 reg signed [31:0] limit,step,lim[0:5];
 // every tick: offset = q - nominal; bounded by limit, changes by at most step, gripper untouched
 always @(posedge clk)if(dut.state==4 && dut.running)begin
  for(m=0;m<7;m=m+1)begin
   if(m==6 && $signed(dut.q[m])!==V)$fatal(1,"gripper offset applied");
   if(m<6 && ($signed(dut.q[m])-V>lim[m] || $signed(dut.q[m])-V< -lim[m]))$fatal(1,"offset beyond limit axis %0d: %0d",m,$signed(dut.q[m])-V);
   if(have_last && ($signed(dut.q[m])-last[m]>step || $signed(dut.q[m])-last[m]< -step))$fatal(1,"slew beyond step axis %0d",m);
   last[m]=$signed(dut.q[m]);
  end
  have_last=1;ticks=ticks+1;checks=checks+1;
 end
 reg [31:0] rejects_before;
 // expect_refused: the write must be ignored and counted at 898, never a bus error
 task put(input [11:0] a,input [31:0] d,input expect_refused);
 begin
  rejects_before=dut.corr_rejects;
  @(negedge clk);access=1;wr=1;addr=a;wdata=d;
  #1 if(error!==0)$fatal(1,"write %h=%h returned a bus error",a,d);
  @(negedge clk);access=0;wr=0;
  @(negedge clk);
  if((a>=12'h840 && a<12'h8c0) && ((dut.corr_rejects!=rejects_before)!==expect_refused))$fatal(1,"write %h=%h refused %b expected %b",a,d,dut.corr_rejects!=rejects_before,expect_refused);
 end endtask
 task get(input [11:0] a,output [31:0] d);
 begin @(negedge clk);access=1;wr=0;addr=a;#1 d=rdata;@(negedge clk);access=0;end endtask
 task wait_ticks(input integer n);integer t0,guard,seen;begin t0=ticks;guard=0;seen=ticks;while(ticks<t0+n)begin @(posedge clk);
  if(ticks!=seen)begin seen=ticks;guard=0;end else guard=guard+1;
  if(guard>20000)$fatal(1,"no tick progress: state %0d fault %0d running %b",dut.state,fault,running);end end endtask
 task load(input integer lower,input integer upper);
 begin
  for(j=0;j<7;j=j+1)begin
   put(12'h900+j*16,V,0);put(12'h904+j*16,0,0);put(12'h908+j*16,0,0);put(12'h90c+j*16,0,0);
   put(12'h970+j*4,lower,0);put(12'h98c+j*4,upper,0);
  end
  put(12'h9a8,32'd2000,0);put(12'h9ac,ONE/2000,0);put(12'h9b0,ONE%2000,0);
  put(12'h820,1,0);
 end endtask
 reg [31:0] r;reg signed [31:0] a0;
 initial begin
  limit=0;step=0;for(k=0;k<6;k=k+1)lim[k]=0;
  repeat(5)@(posedge clk);resetn=1;
  get(12'h894,r);if(r!==32'h43520001)$fatal(1,"capability %h",r);
  get(12'h828,r);if(r!==32'h00070008)$fatal(1,"septic capability changed");
  // enabling without a configuration, ceilings, a target while disabled
  put(12'h840,1,1);
  put(12'h8a0,32'd4194305,1);put(12'h848,32'd50333,1);put(12'h84c,32'd0,1);put(12'h84c,32'd500001,1);
  put(12'h850,ONE/100,1);
  limit=ONE/20;step=ONE/2000;                    // 50 mrad (joint 4: 0 = locked), 0.5 mrad per tick
  for(k=0;k<6;k=k+1)begin lim[k]=k==4 ? 0 : limit;put(12'h8a0+k*4,lim[k],0);end
  put(12'h848,step,0);put(12'h84c,32'd20000,0);
  load(-ONE,ONE);put(12'h820,2,0);
  wait_ticks(3);
  put(12'h8a0,limit,1);put(12'h848,step,1);put(12'h84c,32'd20000,1);   // locked while running
  // enable, targets inside the limit: slew then settle
  $display("phase enable t=%0t ticks=%0d",$time,ticks);
  put(12'h840,1,0);
  for(j=0;j<6;j=j+1)put(12'h850+j*4,(j%2 ? -1:1)*(ONE/100),0);    // +-10 mrad
  for(k=0;k<30;k=k+1)begin wait_ticks(1);for(j=0;j<6;j=j+1)put(12'h850+j*4,(j%2 ? -1:1)*(ONE/100),0);end
  for(j=0;j<6;j=j+1)begin get(12'h870+j*4,r);if($signed(r)!==(j==4 ? 0 : (j%2 ? -1:1)*(ONE/100)))$fatal(1,"offset %0d did not settle: %0d",j,$signed(r));end
  // clamp: a target beyond the limit is stored as the limit and counted
  $display("phase clamp t=%0t ticks=%0d",$time,ticks);
  put(12'h850,ONE,0);get(12'h850,r);if($signed(r)!==limit)$fatal(1,"target not clamped");
  get(12'h88c,r);if(r<1)$fatal(1,"clamp count %0d",r);   // joint 4 targets were clamped to its zero limit too
  // lease: stop refreshing; the offset freezes before it reaches the target
  $display("phase lease t=%0t ticks=%0d",$time,ticks);
  wait_ticks(2);get(12'h870,r);a0=r;
  if(a0===limit || a0<=ONE/100)$fatal(1,"slew test precondition");
  wait_ticks(8);get(12'h840,r);if(r[1]!==1)$fatal(1,"lease did not expire");
  get(12'h870,r);if($signed(r)!==a0+2*step && $signed(r)!==a0+3*step && $signed(r)!==a0+4*step)$fatal(1,"offset moved after the lease expired: %0d -> %0d",a0,$signed(r));
  a0=r;wait_ticks(4);get(12'h870,r);if(r!==a0)$fatal(1,"stale offset not frozen");
  // disable: every offset slews back to zero
  $display("phase disable t=%0t ticks=%0d",$time,ticks);
  put(12'h840,0,0);wait_ticks(120);
  for(j=0;j<6;j=j+1)begin get(12'h870+j*4,r);if(r!==0)$fatal(1,"offset %0d not back to zero",j);end
  // an offset leaving the segment's own bounds is fault 6 (tight bounds around the nominal)
  $display("phase bounds t=%0t ticks=%0d",$time,ticks);
  clear=1;@(negedge clk);clear=0;have_last=0;
  for(k=0;k<6;k=k+1)put(12'h8a0+k*4,lim[k],0);put(12'h848,step,0);put(12'h84c,32'd20000,0);
  load(V-ONE/200,V+ONE/200);put(12'h820,2,0);wait_ticks(2);
  put(12'h840,1,0);
  for(k=0;k<20 && fault==0;k=k+1)begin put(12'h850,ONE/50,0);j=ticks;while(ticks==j && fault==0)@(posedge clk);end
  if(fault!==6)$fatal(1,"bounds fault %0d",fault);
  if(valid)$fatal(1,"stream still valid after fault 6");
  $display("TEST_PASS trajectory_correction ticks_checked=%0d",checks);
  $finish;
 end
endmodule
