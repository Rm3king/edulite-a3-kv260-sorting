`timescale 1ns/1ps
module tb_trajectory_stream;
 reg clk=0;always #5 clk=~clk;
 reg resetn=0,clear=0,access=0,wr=0,ready=1,guard_ok=1;
 reg [31:0] now_us=32'hfffff000,wdata;
 reg [11:0] addr=0;
 reg [6:0] mask=127;
 wire [31:0] rdata,fault;wire error,valid,running;
 wire [191:0] packet;wire [223:0] targets;
 integer count=0,checks=0,j,k;reg monitor=0; reg phase_monitor=0; reg reverse_segment=0; integer nticks=16; integer phase_checks=0; reg [63:0] conversion[0:1031];
 reg [31:0] expected[0:118];
 trajectory_stream dut(.*);
 always @(posedge clk)if(resetn)now_us<=now_us+1;
 task put(input [11:0] a,input [31:0] d);
 begin @(negedge clk);access=1;wr=1;addr=a;wdata=d;@(negedge clk);access=0;wr=0;end endtask
 task load(input bit invalid);
 begin
  for(j=0;j<7;j=j+1)begin
   put(12'h900+j*16, (j-3+(reverse_segment ? 1:0))*16777216/8);
   put(12'h904+j*16,0);put(12'h908+j*16,(reverse_segment ? -3:3)*16777216/8);put(12'h90c+j*16,(reverse_segment ? 2:-2)*16777216/8);
   put(12'h970+j*4,invalid && j==6 ? 32'd16777216 : -32'd16777216);
   put(12'h98c+j*4,32'd16777216);
  end
  put(12'h9a8,nticks);put(12'h9ac,16777216/nticks);put(12'h9b0,16777216%nticks);
  put(12'h820,1);
 end endtask
 always @(negedge clk)if(valid && ready && monitor)begin
  if(packet[127:96]!==32'h1200fd01+count%7 || packet[95:32]!==64'h0000000816700000)$fatal(1,"bad CSP packet");
  if(count<119 && {packet[7:0],packet[15:8],packet[23:16],packet[31:24]}!==expected[count])
   $fatal(1,"sample %0d got %h expected %h phase %h",count,{packet[7:0],packet[15:8],packet[23:16],packet[31:24]},expected[count],dut.phase);
  if(count>=119 && {packet[7:0],packet[15:8],packet[23:16],packet[31:24]}!==expected[112+count%7])$fatal(1,"endpoint hold not stable");
  checks=checks+1;count=count+1;
 end
 always @(negedge clk)if(valid && ready && phase_monitor)begin
  if(dut.phase!==(dut.tick*64'd16777216/7))$fatal(1,"fractional phase drift");
  phase_checks=phase_checks+1;
 end
 initial begin
  $readmemh("contest/kv260_can/tb/qfloat_vectors.mem",conversion);
  for(k=0;k<1032;k=k+1)if(dut.qfloat(conversion[k][63:32])!==conversion[k][31:0])$fatal(1,"Q24 float conversion %d",k);
  expected[0]=32'hbec00000;
  expected[1]=32'hbe800000;
  expected[2]=32'hbe000000;
  expected[3]=32'h00000000;
  expected[4]=32'h3e000000;
  expected[5]=32'h3e800000;
  expected[6]=32'h3ec00000;
  expected[7]=32'hbebf4800;
  expected[8]=32'hbe7e9000;
  expected[9]=32'hbdfd2000;
  expected[10]=32'h3ab80000;
  expected[11]=32'h3e017000;
  expected[12]=32'h3e80b800;
  expected[13]=32'h3ec0b800;
  expected[14]=32'hbebd4000;
  expected[15]=32'hbe7a8000;
  expected[16]=32'hbdf50000;
  expected[17]=32'h3bb00000;
  expected[18]=32'h3e058000;
  expected[19]=32'h3e82c000;
  expected[20]=32'h3ec2c000;
  expected[21]=32'hbeba1800;
  expected[22]=32'hbe743000;
  expected[23]=32'hbde86000;
  expected[24]=32'h3c3d0000;
  expected[25]=32'h3e0bd000;
  expected[26]=32'h3e85e800;
  expected[27]=32'h3ec5e800;
  expected[28]=32'hbeb60000;
  expected[29]=32'hbe6c0000;
  expected[30]=32'hbdd80000;
  expected[31]=32'h3ca00000;
  expected[32]=32'h3e140000;
  expected[33]=32'h3e8a0000;
  expected[34]=32'h3eca0000;
  expected[35]=32'hbeb12800;
  expected[36]=32'hbe625000;
  expected[37]=32'hbdc4a000;
  expected[38]=32'h3ced8000;
  expected[39]=32'h3e1db000;
  expected[40]=32'h3e8ed800;
  expected[41]=32'h3eced800;
  expected[42]=32'hbeabc000;
  expected[43]=32'hbe578000;
  expected[44]=32'hbdaf0000;
  expected[45]=32'h3d220000;
  expected[46]=32'h3e288000;
  expected[47]=32'h3e944000;
  expected[48]=32'h3ed44000;
  expected[49]=32'hbea5f800;
  expected[50]=32'hbe4bf000;
  expected[51]=32'hbd97e000;
  expected[52]=32'h3d504000;
  expected[53]=32'h3e341000;
  expected[54]=32'h3e9a0800;
  expected[55]=32'h3eda0800;
  expected[56]=32'hbea00000;
  expected[57]=32'hbe400000;
  expected[58]=32'hbd800000;
  expected[59]=32'h3d800000;
  expected[60]=32'h3e400000;
  expected[61]=32'h3ea00000;
  expected[62]=32'h3ee00000;
  expected[63]=32'hbe9a0800;
  expected[64]=32'hbe341000;
  expected[65]=32'hbd504000;
  expected[66]=32'h3d97e000;
  expected[67]=32'h3e4bf000;
  expected[68]=32'h3ea5f800;
  expected[69]=32'h3ee5f800;
  expected[70]=32'hbe944000;
  expected[71]=32'hbe288000;
  expected[72]=32'hbd220000;
  expected[73]=32'h3daf0000;
  expected[74]=32'h3e578000;
  expected[75]=32'h3eabc000;
  expected[76]=32'h3eebc000;
  expected[77]=32'hbe8ed800;
  expected[78]=32'hbe1db000;
  expected[79]=32'hbced8000;
  expected[80]=32'h3dc4a000;
  expected[81]=32'h3e625000;
  expected[82]=32'h3eb12800;
  expected[83]=32'h3ef12800;
  expected[84]=32'hbe8a0000;
  expected[85]=32'hbe140000;
  expected[86]=32'hbca00000;
  expected[87]=32'h3dd80000;
  expected[88]=32'h3e6c0000;
  expected[89]=32'h3eb60000;
  expected[90]=32'h3ef60000;
  expected[91]=32'hbe85e800;
  expected[92]=32'hbe0bd000;
  expected[93]=32'hbc3d0000;
  expected[94]=32'h3de86000;
  expected[95]=32'h3e743000;
  expected[96]=32'h3eba1800;
  expected[97]=32'h3efa1800;
  expected[98]=32'hbe82c000;
  expected[99]=32'hbe058000;
  expected[100]=32'hbbb00000;
  expected[101]=32'h3df50000;
  expected[102]=32'h3e7a8000;
  expected[103]=32'h3ebd4000;
  expected[104]=32'h3efd4000;
  expected[105]=32'hbe80b800;
  expected[106]=32'hbe017000;
  expected[107]=32'hbab80000;
  expected[108]=32'h3dfd2000;
  expected[109]=32'h3e7e9000;
  expected[110]=32'h3ebf4800;
  expected[111]=32'h3eff4800;
  expected[112]=32'hbe800000;
  expected[113]=32'hbe000000;
  expected[114]=32'h00000000;
  expected[115]=32'h3e000000;
  expected[116]=32'h3e800000;
  expected[117]=32'h3ec00000;
  expected[118]=32'h3f000000;

  repeat(5)@(negedge clk);resetn=1;
  put(12'h820,1);if(dut.rejects!=1)$fatal(1,"partial commit accepted");
  put(12'h820,2);if(running)$fatal(1,"empty start accepted");
  load(0);if(!dut.queued)$fatal(1,"full commit missing");
  put(12'h820,2);monitor=1;
  wait(count>=42);
  // A second segment can be committed while the first is running; PauseAtEnd
  // must discard it without disabling or changing the active polynomial.
  load(0);if(!dut.queued)$fatal(1,"queued segment missing");
  put(12'h820,4);
  wait(count>=140);monitor=0;
  if(fault || !dut.holding || dut.queued || dut.completed!=1)$fatal(1,"hold/pause state");
  @(negedge clk);clear=1;repeat(2)@(negedge clk);clear=0;
  if(running || valid)$fatal(1,"clear replays motion");
  // Non-power-of-two tick count, exact phase endpoints, automatic second
  // segment activation and holding without resuming discarded old work.
  nticks=7;load(0);put(12'h820,2);phase_monitor=1;
  reverse_segment=1;load(0);
  wait(dut.completed==2);repeat(10)@(negedge clk);
  if(fault || !dut.holding || dut.queued || phase_checks!=112)$fatal(1,"two-segment execution");
  phase_monitor=0;reverse_segment=0;nticks=16;
  @(negedge clk);clear=1;repeat(2)@(negedge clk);clear=0;
  // All seven HLS results must pass before the first frame leaves.
  load(1);put(12'h820,2);
  wait(fault!=0);if(fault!=3 || dut.samples!=0 || valid)$fatal(1,"atomic bounds rejection");
  @(negedge clk);clear=1;repeat(2)@(negedge clk);clear=0;
  load(0);guard_ok=0;put(12'h820,2);wait(fault!=0);
  if(fault!=4 || dut.samples!=0)$fatal(1,"guard bypass");
  @(negedge clk);clear=1;repeat(2)@(negedge clk);clear=0;
  load(0);guard_ok=1;ready=0;put(12'h820,2);wait(fault!=0);
  if(fault!=2)$fatal(1,"backpressure deadline");
  $display("TEST_PASS trajectory_stream %0d packets: seven-axis HLS, 1032 Q24-to-float vectors, 112 fractional-phase packets, two segments, wrap, queue, hold, pause-at-end, partial commit, bounds, guard, deadline",checks);$finish;
 end
 initial begin #10000000;$fatal(1,"stream watchdog");end
endmodule
