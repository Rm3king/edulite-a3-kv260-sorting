`timescale 1ns/1ps
module tb_csp_wide_can;
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
 integer base,rx_n,tries;reg [31:0] overflow_base;reg[31:0] previous_time[0:6];reg[31:0] golden[0:34];
 initial begin
 golden[0]=32'h00000000;
 golden[1]=32'h00000000;
 golden[2]=32'h00000000;
 golden[3]=32'h00000000;
 golden[4]=32'h00000000;
 golden[5]=32'h00000000;
 golden[6]=32'h00000000;
 golden[7]=32'h00809038;
 golden[8]=32'h00801039;
 golden[9]=32'h00c05839;
 golden[10]=32'h00809039;
 golden[11]=32'h00a0b439;
 golden[12]=32'h00c0d839;
 golden[13]=32'h00e0fc39;
 golden[14]=32'h0000003a;
 golden[15]=32'h0000803a;
 golden[16]=32'h0000c03a;
 golden[17]=32'h0000003b;
 golden[18]=32'h0000203b;
 golden[19]=32'h0000403b;
 golden[20]=32'h0000603b;
 golden[21]=32'h00f06d3a;
 golden[22]=32'h00f0ed3a;
 golden[23]=32'h0074323b;
 golden[24]=32'h00f06d3b;
 golden[25]=32'h00b6943b;
 golden[26]=32'h0074b23b;
 golden[27]=32'h0032d03b;
 golden[28]=32'h0000803a;
 golden[29]=32'h0000003b;
 golden[30]=32'h0000403b;
 golden[31]=32'h0000803b;
 golden[32]=32'h0000a03b;
 golden[33]=32'h0000c03b;
 golden[34]=32'h0000e03b;
 addr[0]=0;addr[1]=0;data[0]=0;data[1]=0;
 repeat(20)@(negedge clk);resetn=1;
 write_reg(0,12'h5a0,127);write_reg(0,12'h5a4,100000);
 for(k=1;k<=7;k=k+1)begin
  base=k==7 ? 'h680 : 'h700+(k-1)*32;
  write_reg(0,base,1);write_reg(0,base+4,32'hbdcccccd);write_reg(0,base+8,32'h3dcccccd);
  write_reg(0,base+12,32'h3dcccccd);write_reg(0,base+20,32'h3e99999a);write_reg(0,base+28,1);
 end
 configure(0,7);configure(1,7);
 for(pp_mid=1;pp_mid<=7;pp_mid=pp_mid+1)begin
  access(1,0,12'h408,0,t);write_reg(1,12'h440,t+20);write_reg(1,12'h444,t+1000);
  write_reg(1,12'h448,32'h028000fd+(pp_mid<<8));write_reg(1,12'h44c,8);
  write_reg(1,12'h450,32'h80008000);write_reg(1,12'h454,32'h80000100);write_reg(1,12'h458,1);
  repeat(25000)@(negedge clk);
  pp_write(16'h7005,5,0);pp_write(16'h7017,32'h3dcccccd,0);
  pp_write(16'h7018,32'h3e99999a,0);pp_write(16'h7028,6000,0);pp_write(16'h7016,0,0);
  access(0,0,12'h408,0,t);enqueue(29'h0300fd00+pp_mid,0,t+20,t+1000);
  repeat(25000)@(negedge clk);
 end
 expect_reg(0,12'h5a8,127);configure(1,7);
 access(1,0,12'h42c,0,overflow_base); // cumulative count includes undrained preparation frames
 for(k=0;k<56;k=k+1)write_reg(0,12'ha00+k*4,0);
 for(k=0;k<7;k=k+1)begin
  write_reg(0,12'ha10+k*32,(k+1)*16384*35);
  write_reg(0,12'ha14+k*32,-(k+1)*16384*84);
  write_reg(0,12'ha18+k*32,(k+1)*16384*70);
  write_reg(0,12'ha1c+k*32,-(k+1)*16384*20);
  write_reg(0,12'hae0+k*4,-1677721);write_reg(0,12'hafc+k*4,1677721);
 end
 write_reg(0,12'hb18,4);write_reg(0,12'hb1c,4194304);write_reg(0,12'hb20,0);
 write_reg(0,12'h820,8);write_reg(0,12'h820,2);
 rx_n=0;tries=0;
 // Five 5 ms batches need >21 ms; the old three-batch test's 16 ms
 // polling budget is insufficient even when every received value matches.
 while(rx_n<35 && tries<160)begin
  repeat(20000)@(negedge clk);tries=tries+1;
  access(1,0,12'h41c,0,value);
  while(value[31:16]!=0 && rx_n<35)begin
   expect_reg(1,12'h480,32'h1200fd01+rx_n%7);expect_reg(1,12'h488,32'h16700000);
   expect_reg(1,12'h48c,golden[rx_n]);access(1,0,12'h490,0,t);
   if(rx_n>=7 && (t-previous_time[rx_n%7]<4900 || t-previous_time[rx_n%7]>5100))$fatal(1,"seven-axis scheduling interval");
   previous_time[rx_n%7]=t;write_reg(1,12'h498,1);rx_n=rx_n+1;access(1,0,12'h41c,0,value);
  end
 end
 if(rx_n!=35)$fatal(1,"missing seven-axis targets count=%d",rx_n);
 expect_reg(0,12'h804,0);expect_reg(0,12'h418,0);expect_reg(0,12'h424,0);expect_reg(0,12'h808,5);
 expect_reg(1,12'h42c,overflow_base);
 $display("TEST_PASS degree-seven CSP CAN: 35 independent wire-received HLS targets, all IDs/values, 5ms periods within 100us serialization allowance, no expired/overflow/fault");$finish;
 end
 initial begin #100000000;$fatal(1,"global watchdog");end
endmodule
