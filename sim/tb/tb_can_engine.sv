`timescale 1ns/1ps
module tb_can_engine;
 reg clk=0;always #5 clk=~clk;
 reg resetn=0;
 wire [1:0] tx;
 reg disconnected=0;
 wire bus_line=tx[0] & tx[1];
 reg [1:0] valid=0,wr=0;
 reg [11:0] addr[0:1];reg [31:0] data[0:1];
 wire [1:0] done,err;wire [31:0] result[0:1];
 genvar g;
 generate for(g=0;g<2;g=g+1)begin:nodes
  can_engine dut(.clk(clk),.resetn(resetn),.can_rx(disconnected ? tx[g]:bus_line),.can_tx(tx[g]),
   .req_valid(valid[g]),.req_write(wr[g]),.req_addr(addr[g]),.req_data(data[g]),.req_strb(4'hf),
   .req_done(done[g]),.rsp_data(result[g]),.rsp_error(err[g]),.debug_us(),.debug_status());
 end endgenerate
 reg expected_error=0;
 integer pp_mid=7;integer checks=0;reg [31:0] value,t,other;integer k;
 task access(input integer n,input bit write_en,input [11:0] a,input [31:0] d,output [31:0] r);
 integer timeout_count;
 begin
  @(negedge clk);addr[n]=a;data[n]=d;wr[n]=write_en;valid[n]=1;timeout_count=0;
  while(!done[n])begin @(negedge clk);timeout_count=timeout_count+1;if(timeout_count>20000)$fatal(1,"request timeout n=%d a=%h",n,a);end
  r=result[n];if(err[n]!==expected_error)$fatal(1,"request n=%d a=%h error=%b expected=%b id=%h hi=%h lo=%h pp_mid=%d",n,a,err[n],expected_error,nodes[0].dut.stage_id,nodes[0].dut.stage_hi,nodes[0].dut.stage_lo,pp_mid);
  valid[n]=0;repeat(4)@(negedge clk);
 end
 endtask
 task write_reg(input integer n,input [11:0] a,input [31:0] d);
 reg [31:0] unused;begin access(n,1,a,d,unused);end endtask
 task expect_reg(input integer n,input [11:0] a,input [31:0] expected);
 reg[31:0] r;begin access(n,0,a,0,r);checks=checks+1;if(r!==expected)$fatal(1,"n=%d a=%h got=%h expected=%h",n,a,r,expected);end endtask
 task configure(input integer n,input [2:0] ctl);
 begin
  write_reg(n,12'h404,0);repeat(100)@(negedge clk);
  write_reg(n,12'h07c,8'h80); // PeliCAN
  write_reg(n,12'h000,8'h09); // reset, single filter
  write_reg(n,12'h018,8'h04); // 100 MHz / (10 * 10) = 1 Mbps
  write_reg(n,12'h01c,8'h16); // 1 + 7 + 2 tq, sample 80%
  for(integer j=16;j<20;j=j+1)write_reg(n,j*4,0);
  for(integer j=20;j<24;j=j+1)write_reg(n,j*4,255);
  write_reg(n,12'h000,8'h08); // normal mode
  write_reg(n,12'h414,100000);write_reg(n,12'h404,{29'b0,ctl});
 end endtask
 task pp_write(input [15:0] param,input [31:0] bits,input bit reject);
 begin
  access(0,0,12'h408,0,t);
  write_reg(0,12'h440,t+20);write_reg(0,12'h444,t+1000);
  write_reg(0,12'h448,32'h1200fd00+pp_mid);write_reg(0,12'h44c,8);
  write_reg(0,12'h450,{param[7:0],param[15:8],16'b0});
  write_reg(0,12'h454,{bits[7:0],bits[15:8],bits[23:16],bits[31:24]});
  expected_error=reject;write_reg(0,12'h458,1);expected_error=0;checks=checks+1;
  if(!reject)repeat(25000)@(negedge clk);
 end endtask
 task enqueue(input [28:0] id,input [63:0] payload,input [31:0] due,input [31:0] expiry);
 reg reject;
 begin
  reject=expected_error;expected_error=0;
  write_reg(0,12'h440,due);write_reg(0,12'h444,expiry);write_reg(0,12'h448,{3'b0,id});
  write_reg(0,12'h44c,8);write_reg(0,12'h450,payload[63:32]);write_reg(0,12'h454,payload[31:0]);expected_error=reject;write_reg(0,12'h458,1);expected_error=0;
 end endtask
 initial begin
  addr[0]=0;addr[1]=0;data[0]=0;data[1]=0;
  repeat(20)@(negedge clk);resetn=1;
  expect_reg(0,12'h400,32'h43414e31);expect_reg(0,12'h404,0);
  if(tx!==2'b11)$fatal(1,"TX not recessive on reset");
  configure(0,3);configure(1,5); // transmitter scheduler and receiver autonomous RX
  repeat(3000)@(negedge clk);
  access(0,0,12'h408,0,t);
  enqueue(29'h020001fd,64'h0123456789abcdef,t+100,t+1000);
  repeat(50000)@(negedge clk);
  expect_reg(0,12'h420,1);expect_reg(1,12'h428,1);
  expect_reg(1,12'h480,32'h020001fd);expect_reg(1,12'h484,32'h108);
  expect_reg(1,12'h488,32'h01234567);expect_reg(1,12'h48c,32'h89abcdef);
  expect_reg(1,12'h49c,1);expect_reg(1,12'h500,32'h020001fd);
  expect_reg(1,12'h504,32'h01234567);expect_reg(1,12'h508,32'h89abcdef);
  access(1,0,12'h490,0,value);checks=checks+1;if(value<t+100)$fatal(1,"early frame");
  write_reg(1,12'h498,1);expect_reg(1,12'h41c,0);
  // Multiple data patterns exercise stuffing, CRC and FIFO preservation.
  for(k=0;k<10;k=k+1)begin
   access(0,0,12'h408,0,t);
   enqueue(29'h1fffff00+k, k%2 ? 64'hffffffffffffffff:64'h0000000000000000,t+20,t+1000);
   repeat(25000)@(negedge clk);
   expect_reg(1,12'h480,32'h1fffff00+k);
   expect_reg(1,12'h488,k%2 ? 32'hffffffff:0);
   expect_reg(1,12'h48c,k%2 ? 32'hffffffff:0);write_reg(1,12'h498,1);
  end
  expect_reg(0,12'h420,11);expect_reg(1,12'h428,11);
  access(0,0,12'h408,0,t);enqueue(29'h123,0,t-20,t-10);
  repeat(3000)@(negedge clk);expect_reg(0,12'h424,1);expect_reg(0,12'h420,11);
  // Receiver has ACKed all valid frames, no TX errors.
  expect_reg(0,12'h03c,0);
  // Lease expiry must close TX gate, clear queue and require reconfiguration.
  write_reg(0,12'h414,30);repeat(4000)@(negedge clk);
  expect_reg(0,12'h404,0);expect_reg(0,12'h418,1);
  if(tx[0]!==1)$fatal(1,"lease did not inhibit TX");
  configure(0,1);
  write_reg(0,0,8'h09);write_reg(0,0,8'h0c);write_reg(0,12'h404,15);
  access(0,0,12'h408,0,t);enqueue(29'h028001fd,64'h1122334455667788,t+20,t+1000);
  repeat(30000)begin @(negedge clk);if(tx[0]!==1)$fatal(1,"internal loopback leaked external TX");end
  expect_reg(0,12'h428,1);expect_reg(0,12'h480,32'h028001fd);
  expect_reg(0,12'h488,32'h11223344);expect_reg(0,12'h48c,32'h55667788);
  // Submitted packet expires while retransmitting without an ACK.
  write_reg(1,12'h404,0);disconnected=1;
  write_reg(0,12'h404,0);write_reg(0,12'h418,0);configure(0,7);
  access(0,0,12'h408,0,t);
  enqueue(29'h1100fd01,64'h1970000000000000,t+20,t+220);
  repeat(30000)@(negedge clk);
  expect_reg(0,12'h418,4);expect_reg(0,12'h460,1);expect_reg(0,12'h464,0);
  expect_reg(0,12'h404,0);expect_reg(0,12'h41c,0);
  repeat(10000)begin @(negedge clk);if(tx[0]!==1)$fatal(1,"post-expiry retry leaked TX");end
  // Configure fixed limits while stopped. Re-enable normal two-node CAN.
  disconnected=0;write_reg(0,12'h418,0);
  write_reg(0,12'h600,32'h90008000);write_reg(0,12'h604,32'h90007000);
  write_reg(0,12'h608,32'h90007000);write_reg(0,12'h60c,32'h20002000);
  expect_reg(0,12'h600,32'h90008000);
  write_reg(0,12'h5a0,1);write_reg(0,12'h5a4,2000);
  configure(0,7);configure(1,7);
  // Reject limit changes while running and motion without fresh Type2.
  expected_error=1;write_reg(0,12'h600,32'hffff0000);expected_error=0;
  access(0,0,12'h408,0,t);
  write_reg(0,12'h440,t+100);write_reg(0,12'h444,t+1000);
  write_reg(0,12'h448,32'h017fff01);write_reg(0,12'h44c,8);
  write_reg(0,12'h450,32'h80008000);write_reg(0,12'h454,32'h01000100);
  expected_error=1;write_reg(0,12'h458,1);expected_error=0;
  expect_reg(0,12'h46c,1);
  // Independent CAN node produces a fresh feedback frame for motor 1.
  access(1,0,12'h408,0,t);
  write_reg(1,12'h440,t+20);write_reg(1,12'h444,t+1000);
  write_reg(1,12'h448,32'h020001fd);write_reg(1,12'h44c,8);
  write_reg(1,12'h450,32'h80008000);write_reg(1,12'h454,32'h80000100);write_reg(1,12'h458,1);
  repeat(25000)@(negedge clk);expect_reg(0,12'h49c,1);
  // Below/above position, velocity, torque and gain limits are rejected.
  for(k=0;k<8;k=k+1)begin
   write_reg(0,12'h448,(k==4 ? 32'h016fff01:k==5 ? 32'h01900101:32'h017fff01));
   write_reg(0,12'h450,(k==0 ? 32'h7fff8000:k==1 ? 32'h90018000:k==2 ? 32'h80006fff:k==3 ? 32'h80009001:32'h80008000));
   write_reg(0,12'h454,(k==6 ? 32'h20010100:k==7 ? 32'h01002001:32'h01000100));
   expected_error=1;write_reg(0,12'h458,1);expected_error=0;
  end
  expect_reg(0,12'h46c,9);
  // Allowed boundary position is transmitted. Later feedback expiry latches fault.
  access(0,0,12'h408,0,t);enqueue(29'h017fff01,64'h8000800001000100,t+20,t+1000);
  repeat(30000)@(negedge clk);expect_reg(0,12'h5a8,1);
  repeat(200000)@(negedge clk);expect_reg(0,12'h418,8);expect_reg(0,12'h404,0);
  if(tx[0]!==1)$fatal(1,"feedback timeout did not close TX");
  // PP gripper whitelist, bit-exact float ordering, prepare-before-enable.
  write_reg(0,12'h418,0);write_reg(0,12'h5a0,64);write_reg(0,12'h5a4,3000);
  write_reg(0,12'h680,1);write_reg(0,12'h684,32'hbdcccccd);write_reg(0,12'h688,32'h3dcccccd);
  write_reg(0,12'h68c,32'h3dcccccd);write_reg(0,12'h690,32'h3e4ccccd);write_reg(0,12'h694,32'h3e99999a);
  configure(0,7);configure(1,7);expect_reg(0,12'h470,32'h60003);
  pp_write(16'h7016,0,1); // no feedback
  access(1,0,12'h408,0,t);write_reg(1,12'h440,t+20);write_reg(1,12'h444,t+1000);
  write_reg(1,12'h448,32'h020007fd);write_reg(1,12'h44c,8);
  write_reg(1,12'h450,32'h80008000);write_reg(1,12'h454,32'h80000100);write_reg(1,12'h458,1);
  repeat(25000)@(negedge clk);expect_reg(0,12'h49c,64);
  pp_write(16'h7005,32'h3f800000,1); // float 1 is not uint8 mode 1
  pp_write(16'h7028,0,1);pp_write(16'h7028,20001,1);
  pp_write(16'h7016,32'h7fc00000,1);pp_write(16'h7016,32'h7f800000,1);
  pp_write(16'h7016,32'h3dccccce,1);pp_write(16'h7016,32'hbdccccce,1);
  pp_write(16'h7024,32'hbdcccccd,1);pp_write(16'h7024,32'h3e4ccccd,1);
  pp_write(16'h7025,32'h3f800000,1);pp_write(16'h7018,32'h3f800000,1);
  pp_write(16'h7010,32'h3dcccccd,1); // arbitrary current gain denied
  write_reg(0,12'h448,32'h0300fd07);write_reg(0,12'h450,0);write_reg(0,12'h454,0);
  expected_error=1;write_reg(0,12'h458,1);expected_error=0;
  pp_write(16'h7005,1,0);pp_write(16'h7024,32'h3dcccccd,0);
  pp_write(16'h7025,32'h3e4ccccd,0);pp_write(16'h7018,32'h3e99999a,0);
  pp_write(16'h7028,6000,0);pp_write(16'h7016,32'hbdcccccd,0);
  pp_write(16'h7016,32'h3dcccccd,0);expect_reg(0,12'h698,63);
  access(0,0,12'h408,0,t);enqueue(29'h0300fd07,0,t+20,t+1000);
  repeat(25000)@(negedge clk);expect_reg(0,12'h5a8,64);
  pp_write(16'h7005,1,1);pp_write(16'h7024,32'h3dcccccd,1);pp_write(16'h7028,6000,1);
  repeat(350000)@(negedge clk);expect_reg(0,12'h418,8);expect_reg(0,12'h698,0);
  // v4 per-axis banks, independent preparation, seven active axes, global fault clear.
  write_reg(0,12'h418,0);write_reg(0,12'h5a0,127);write_reg(0,12'h5a4,25000);
  for(k=0;k<6;k=k+1)begin
   write_reg(0,12'h700+k*32,1);write_reg(0,12'h704+k*32,32'hbdcccccd);
   write_reg(0,12'h708+k*32,32'h3dcccccd);write_reg(0,12'h70c+k*32,32'h3dcccccd);
   write_reg(0,12'h710+k*32,32'h3e4ccccd);write_reg(0,12'h714+k*32,32'h3e99999a);
   expect_reg(0,12'h718+k*32,0);
  end
  configure(0,7);configure(1,7);
  for(pp_mid=1;pp_mid<=7;pp_mid=pp_mid+1)begin
   // Preparation for another motor never authorizes this enable.
   access(0,0,12'h408,0,t);
   expected_error=1;enqueue(29'h0300fd00+pp_mid,0,t+20,t+1000);expected_error=0;
   pp_write(16'h7016,0,1);
   access(1,0,12'h408,0,t);write_reg(1,12'h440,t+20);write_reg(1,12'h444,t+1000);
   write_reg(1,12'h448,32'h020000fd+(pp_mid<<8));write_reg(1,12'h44c,8);
   write_reg(1,12'h450,32'h80008000);write_reg(1,12'h454,32'h80000100);write_reg(1,12'h458,1);
   repeat(25000)@(negedge clk);
   pp_write(16'h7016,32'h7fc00000,1);pp_write(16'h7016,32'h3dccccce,1);
   pp_write(16'h7005,1,0);pp_write(16'h7024,32'h3dcccccd,0);
   pp_write(16'h7025,32'h3e4ccccd,0);pp_write(16'h7018,32'h3e99999a,0);
   pp_write(16'h7028,6000,0);pp_write(16'h7016,0,0);
   expect_reg(0,pp_mid==7 ? 12'h698 : 12'h718+(pp_mid-1)*32,63);
   access(0,0,12'h408,0,t);enqueue(29'h0300fd00+pp_mid,0,t+20,t+1000);
   repeat(25000)@(negedge clk);expect_reg(0,12'h5a8,(1<<pp_mid)-1);
   pp_write(16'h7005,1,1);pp_write(16'h7028,6000,1);
   expected_error=1;write_reg(0,12'h700,0);expected_error=0;
  end
  // Type4 clears only the addressed active/prepared bank; all others retain state.
  access(0,0,12'h408,0,t);enqueue(29'h0400fd04,0,t+20,t+1000);
  repeat(25000)@(negedge clk);expect_reg(0,12'h5a8,119);expect_reg(0,12'h778,0);
  expect_reg(0,12'h718,63);expect_reg(0,12'h698,63);
  write_reg(0,12'h414,30);repeat(4000)@(negedge clk);expect_reg(0,12'h418,1);expect_reg(0,12'h5a8,0);
  for(k=0;k<6;k=k+1)expect_reg(0,12'h718+k*32,0);
  expect_reg(0,12'h698,0);
  // v5: CSP mode is independently selected while stopped. PP and CSP speed
  // parameters cannot be mixed; mode switching erases every preparation bit.
  write_reg(0,12'h418,0);write_reg(0,12'h5a0,64);write_reg(0,12'h5a4,25000);
  write_reg(0,12'h69c,1);expect_reg(0,12'h69c,1);
  configure(0,7);configure(1,7);pp_mid=7;
  access(1,0,12'h408,0,t);write_reg(1,12'h440,t+20);write_reg(1,12'h444,t+1000);
  write_reg(1,12'h448,32'h020007fd);write_reg(1,12'h44c,8);
  write_reg(1,12'h450,32'h80008000);write_reg(1,12'h454,32'h80000100);write_reg(1,12'h458,1);
  repeat(25000)@(negedge clk);
  pp_write(16'h7005,1,1);pp_write(16'h7005,5,0);
  pp_write(16'h7024,32'h3dcccccd,1);pp_write(16'h7025,32'h3e4ccccd,1);
  pp_write(16'h7017,32'h7fc00000,1);pp_write(16'h7017,32'h3f800000,1);
  pp_write(16'h7017,32'h3dcccccd,0);pp_write(16'h7018,32'h3e99999a,0);
  pp_write(16'h7028,6000,0);pp_write(16'h7016,0,0);
  expect_reg(0,12'h698,63);
  access(0,0,12'h408,0,t);enqueue(29'h0300fd07,0,t+20,t+1000);
  repeat(25000)@(negedge clk);expect_reg(0,12'h5a8,64);
  // Entire HLS -> float -> scheduled FIFO -> CAN wire -> independent node.
  configure(1,7); // flush receiver queue, no change to node 0 guard/preparation
  for(k=0;k<28;k=k+1)write_reg(0,12'h900+k*4,0);
  write_reg(0,12'h968,393216);write_reg(0,12'h96c,-262144);
  for(k=0;k<7;k=k+1)begin
   write_reg(0,12'h970+k*4,-1677721);write_reg(0,12'h98c+k*4,1677721);
  end
  write_reg(0,12'h9a8,2);write_reg(0,12'h9ac,8388608);write_reg(0,12'h9b0,0);
  write_reg(0,12'h820,1);expect_reg(0,12'h800,2);write_reg(0,12'h820,2);
  pp_write(16'h7016,0,1); // PL owns target stream
  repeat(1170000)@(negedge clk);
  expect_reg(0,12'h804,0);expect_reg(0,12'h808,3);expect_reg(0,12'h80c,1);
  expect_reg(1,12'h480,32'h1200fd07);expect_reg(1,12'h488,32'h16700000);expect_reg(1,12'h48c,0);
  access(1,0,12'h490,0,other);write_reg(1,12'h498,1);
  expect_reg(1,12'h48c,32'h0000803b);access(1,0,12'h490,0,t);
  if(t-other<4990 || t-other>5010)$fatal(1,"CSP CAN interval %0d",t-other);
  write_reg(1,12'h498,1);expect_reg(1,12'h48c,32'h0000003c);write_reg(1,12'h498,1);
  write_reg(0,12'h820,4);expect_reg(0,12'h81c,1);
  pp_write(16'h7017,32'h3dcccccd,1);
  expected_error=1;write_reg(0,12'h69c,0);expected_error=0;
  write_reg(0,12'h404,0);write_reg(0,12'h69c,0);expect_reg(0,12'h698,0);
  // v6.1: only joint3 volatile CSP loc_kp, 20..30, fresh disabled feedback.
  write_reg(0,12'h5a0,6);write_reg(0,12'h5a4,1000);
  write_reg(0,12'h720,1);write_reg(0,12'h73c,1);
  write_reg(0,12'h740,1);write_reg(0,12'h75c,1);
  configure(0,7);configure(1,7);pp_mid=3;
  pp_write(16'h701e,32'h41a00000,1); // no new feedback for joint3
  access(1,0,12'h408,0,t);write_reg(1,12'h440,t+20);write_reg(1,12'h444,t+1000);
  write_reg(1,12'h448,32'h020003fd);write_reg(1,12'h44c,8);
  write_reg(1,12'h450,32'h80008000);write_reg(1,12'h454,32'h80000100);write_reg(1,12'h458,1);
  repeat(25000)@(negedge clk);
  pp_write(16'h701e,32'h41a00000,0);pp_write(16'h701e,32'h41f00000,0);
  pp_write(16'h701e,32'h419fffff,1);pp_write(16'h701e,32'h41f00001,1);
  pp_write(16'h701e,32'h7fc00000,1);pp_write(16'h701e,32'hc1a00000,1);
  pp_mid=2;pp_write(16'h701e,32'h41a00000,1);pp_mid=3;
  // Fresh running feedback must not authorize tuning even without a host enable.
  access(1,0,12'h408,0,t);write_reg(1,12'h440,t+20);write_reg(1,12'h444,t+1000);
  write_reg(1,12'h448,32'h028003fd);write_reg(1,12'h44c,8);
  write_reg(1,12'h450,32'h80008000);write_reg(1,12'h454,32'h80000100);write_reg(1,12'h458,1);
  repeat(25000)@(negedge clk);pp_write(16'h701e,32'h41a00000,1);
  // ABI6.3 tuning: actual wire feedback; no forced internal nets.
  write_reg(0,12'h404,0);write_reg(0,12'h5a4,100000);configure(0,7);
  for(integer m=2;m<=3;m=m+1)begin
   pp_mid=m;
   access(1,0,12'h408,0,t);write_reg(1,12'h440,t+20);write_reg(1,12'h444,t+1000);
   write_reg(1,12'h448,32'h020000fd+(m<<8));write_reg(1,12'h44c,8);
   write_reg(1,12'h450,32'h80008000);write_reg(1,12'h454,32'h80000100);write_reg(1,12'h458,1);
   repeat(25000)@(negedge clk);
   pp_write(16'h701f,32'h40800000,0);pp_write(16'h701f,32'h40c00000,0);
   pp_write(16'h7020,32'h3c23d70a,0);pp_write(16'h7020,32'h3ca3d70a,0);
   pp_write(16'h7021,32'h3d4ccccd,0);pp_write(16'h7021,32'h3e19999a,0);
   pp_write(16'h701f,32'h407fffff,1);pp_write(16'h701f,32'h40c00001,1);
   pp_write(16'h7020,32'h3c23d709,1);pp_write(16'h7020,32'h3ca3d70b,1);
   pp_write(16'h7021,32'h3d4ccccc,1);pp_write(16'h7021,32'h3e19999b,1);
   pp_write(16'h7021,32'h7fc00000,1);pp_write(16'h7021,32'h7f800000,1);
   pp_write(16'h7021,32'hbdcccccd,1);
   access(1,0,12'h408,0,t);write_reg(1,12'h440,t+20);write_reg(1,12'h444,t+1000);
   write_reg(1,12'h448,32'h028000fd+(m<<8));write_reg(1,12'h458,1);
   repeat(25000)@(negedge clk);
   pp_write(16'h701f,32'h40a00000,1);pp_write(16'h7020,32'h3ca3d70a,1);pp_write(16'h7021,32'h3dcccccd,1);
  end
  for(integer m=1;m<=7;m=m+1)if(m!=2 && m!=3)begin
   pp_mid=m;pp_write(16'h701f,32'h40a00000,1);pp_write(16'h7020,32'h3ca3d70a,1);pp_write(16'h7021,32'h3dcccccd,1);
  end
  $display("TEST_PASS engine checks=%0d extended CAN, ACK, stuffing patterns, hardware RX, feedback table, deadlines, lease, pending-TTL, motion bounds, feedback-loss",checks);
  $finish;
 end
 initial begin #100000000;$fatal(1,"global watchdog");end
endmodule
