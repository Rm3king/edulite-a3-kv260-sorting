`timescale 1ns/1ps
module tb_mit_damping_can;
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
 integer base,rx_n,tries;reg [31:0] overflow_base;reg[31:0] previous_time[0:6];
 reg[95:0] golden[0:34];reg[31:0] stage_words[0:219];
 initial begin
 $readmemh("contest/kv260_can/tb/mit_packets.mem",golden);
 $readmemh("contest/kv260_can/tb/mit_stage.mem",stage_words);
 addr[0]=0;addr[1]=0;data[0]=0;data[1]=0;
 repeat(20)@(negedge clk);resetn=1;
 write_reg(0,12'h5a0,127);write_reg(0,12'h5a4,100000);
 for(k=1;k<=7;k=k+1)begin
  base=k==7 ? 'h680 : 'h700+(k-1)*32;
  write_reg(0,base,1);write_reg(0,base+4,k==7 ? 0:32'hbdcccccd);write_reg(0,base+8,k==7 ? 32'h40000000:32'h3dcccccd);
  write_reg(0,base+12,32'h3dcccccd);write_reg(0,base+20,32'h3e99999a);
  write_reg(0,base+16,32'h40a00000);write_reg(0,base+28,k==7 ? 1:2);
  write_reg(0,12'h600+(k-1)*16,32'hffff0000);write_reg(0,12'h604+(k-1)*16,32'hffff0000);
  write_reg(0,12'h608+(k-1)*16,32'hffff0000);write_reg(0,12'h60c+(k-1)*16,32'hcccc28f5);
 end
 configure(0,7);configure(1,7);
 for(pp_mid=1;pp_mid<=7;pp_mid=pp_mid+1)begin
  access(1,0,12'h408,0,t);write_reg(1,12'h440,t+20);write_reg(1,12'h444,t+1000);
  write_reg(1,12'h448,32'h028000fd+(pp_mid<<8));write_reg(1,12'h44c,8);
  write_reg(1,12'h450,32'h80008000);write_reg(1,12'h454,32'h80000100);write_reg(1,12'h458,1);
  repeat(25000)@(negedge clk);
  if(pp_mid<7)begin
   pp_write(16'h7005,5,1);pp_write(16'h7005,0,0);
   // A motion frame before torque/timeout preparation must be rejected.
   access(0,0,12'h408,0,t);expected_error=1;enqueue(29'h01800000+pp_mid,64'h8000800028f5cccc,t+20,t+1000);
   pp_write(16'h700b,32'h40a00000,0);pp_write(16'h7028,6000,0);
   pp_write(16'h7016,0,1); // CSP target cannot be written in MIT mode.
   access(0,0,12'h408,0,t);enqueue(29'h01800000+pp_mid,64'h8000800028f5cccc,t+20,t+1000);
   repeat(25000)@(negedge clk);
   expect_reg(0,12'h5a8,(1<<(pp_mid-1))-1); // initial target does not enable
  end else begin
   pp_write(16'h7005,5,0);pp_write(16'h7017,32'h3dcccccd,0);
   pp_write(16'h7018,32'h3e99999a,0);pp_write(16'h7028,6000,0);pp_write(16'h7016,0,0);
  end
  access(0,0,12'h408,0,t);enqueue(29'h0300fd00+pp_mid,0,t+20,t+1000);
  repeat(25000)@(negedge clk);
 end
 expect_reg(0,12'h5a8,127);configure(1,7);
 access(1,0,12'h42c,0,overflow_base); // cumulative count includes undrained preparation frames
 // Refresh joint3 with a known +100 velocity-code error just before start.
 access(1,0,12'h408,0,t);write_reg(1,12'h440,t+20);write_reg(1,12'h444,t+1000);
 write_reg(1,12'h448,32'h028003fd);write_reg(1,12'h44c,8);
 write_reg(1,12'h450,32'h80008064);write_reg(1,12'h454,32'h80000100);write_reg(1,12'h458,1);
 repeat(25000)@(negedge clk);
 stage_words[216]=513;
 for(k=0;k<220;k=k+1)write_reg(0,12'hc00+k*4,stage_words[k]);
 write_reg(0,12'hb60,1);expect_reg(0,12'hb60,1); // reject invalid metadata atomically
 write_reg(0,12'hf60,512);write_reg(0,12'hb60,1);write_reg(0,12'hb60,2);
 rx_n=0;tries=0;
 // Five 5 ms batches need >21 ms; the old three-batch test's 16 ms
 // polling budget is insufficient even when every received value matches.
 while(rx_n<35 && tries<160)begin
  repeat(20000)@(negedge clk);tries=tries+1;
  access(1,0,12'h41c,0,value);
  while(value[31:16]!=0 && rx_n<35)begin
   if(rx_n==2)expect_reg(1,12'h480,golden[rx_n][95:64]-(118<<8));
   else if(rx_n%7!=2 || rx_n==30)expect_reg(1,12'h480,golden[rx_n][95:64]);
   else begin
    access(1,0,12'h480,0,other);
    if(other[31:24]!=1 || other[7:0]!=3 || $signed({1'b0,other[23:8]})-$signed({1'b0,golden[rx_n][87:72]})>702 || $signed({1'b0,golden[rx_n][87:72]})-$signed({1'b0,other[23:8]})>702)$fatal(1,"damping packet field/cap");
   end
   expect_reg(1,12'h488,golden[rx_n][63:32]);
   expect_reg(1,12'h48c,golden[rx_n][31:0]);access(1,0,12'h490,0,t);
   if(rx_n>=7 && (t-previous_time[rx_n%7]<4900 || t-previous_time[rx_n%7]>5100))$fatal(1,"seven-axis scheduling interval");
   previous_time[rx_n%7]=t;write_reg(1,12'h498,1);rx_n=rx_n+1;access(1,0,12'h41c,0,value);
  end
 end
 if(rx_n!=35)$fatal(1,"missing seven-axis targets count=%d stream_fault=%h controller_fault=%h guard=%b",rx_n,nodes[0].dut.mit_fault,nodes[0].dut.fault,nodes[0].dut.mit_guard);
 expect_reg(0,12'hb44,0);expect_reg(0,12'h418,0);expect_reg(0,12'h424,0);expect_reg(0,12'hb48,5);
 expect_reg(1,12'h42c,overflow_base);
 access(0,0,12'h6c4,0,other);if(other==0)$fatal(1,"no physical-path damping applied");
 access(0,0,12'h6cc,0,other);if(other==0)$fatal(1,"stale feedback did not fall back");
 $display("TEST_PASS MIT filtered-damping CAN: first correction -118 codes, later stale bypass, invalid metadata rejected, untouched q/v/gains/other axes; six Type1 q/v/tau/PD and CSP gripper, 35 independent wire-received HLS targets, all IDs/values, 5ms periods within 100us serialization allowance, no expired/overflow/fault");$finish;
 end
 initial begin #100000000;$fatal(1,"global watchdog");end
endmodule
