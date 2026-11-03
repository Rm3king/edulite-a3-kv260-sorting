`timescale 1ns/1ps
// CPU-facing AXI proof: parameter rejection must not enqueue a CAN packet.
// +SOFT also checks the new non-aborting submission port, after ABI 0x60003.
module tb_tx_rejection;
 reg clk=0; always #5 clk=~clk;
 reg resetn=0;
 reg [11:0] awaddr=0,araddr=0;
 reg awvalid=0,wvalid=0,arvalid=0,bready=0,rready=0;
 reg [31:0] wdata=0;
 wire awready,wready,arready,bvalid,rvalid,can_tx;
 wire [1:0] bresp,rresp; wire [31:0] rdata;
 integer checks=0;reg [31:0] value;reg [1:0] response;
 kv260_can_top dut(.clk(clk),.resetn(resetn),.can_rx(1'b1),.can_tx(can_tx),
  .s_axi_awaddr(awaddr),.s_axi_awprot(3'b0),.s_axi_awvalid(awvalid),.s_axi_awready(awready),
  .s_axi_wdata(wdata),.s_axi_wstrb(4'hf),.s_axi_wvalid(wvalid),.s_axi_wready(wready),
  .s_axi_bresp(bresp),.s_axi_bvalid(bvalid),.s_axi_bready(bready),
  .s_axi_araddr(araddr),.s_axi_arprot(3'b0),.s_axi_arvalid(arvalid),.s_axi_arready(arready),
  .s_axi_rdata(rdata),.s_axi_rresp(rresp),.s_axi_rvalid(rvalid),.s_axi_rready(rready),
  .debug_us(),.debug_status());
 task write_reg(input [11:0] a,input [31:0] d,input [1:0] expected);
 integer n;
 begin
  @(negedge clk);awaddr=a;wdata=d;awvalid=1;wvalid=1;n=0;
  do begin @(posedge clk);n=n+1;if(n>20000)$fatal(1,"AW/W timeout");end while(!(awready&&wready));
  @(negedge clk);awvalid=0;wvalid=0;
  while(!bvalid)begin @(negedge clk);n=n+1;if(n>20000)$fatal(1,"BRESP timeout %h",a);end
  if(bresp!==expected)$fatal(1,"addr=%h got BRESP=%b expected=%b",a,bresp,expected);
  checks=checks+1;bready=1;@(negedge clk);bready=0;
 end endtask
 task read_reg(input [11:0] a,output [31:0] d);
 integer n;
 begin
  @(negedge clk);araddr=a;arvalid=1;n=0;
  do begin @(posedge clk);n=n+1;if(n>20000)$fatal(1,"AR timeout");end while(!arready);
  @(negedge clk);arvalid=0;
  while(!rvalid)begin @(negedge clk);n=n+1;if(n>20000)$fatal(1,"R timeout");end
  if(rresp!==0)$fatal(1,"read error %h",a);d=rdata;
  rready=1;@(negedge clk);rready=0;checks=checks+1;
 end endtask
 task parameter_frame(input integer mid,input [15:0] index,input [11:0] commit,input [1:0] expected);
 begin
  read_reg(12'h408,value);
  write_reg(12'h440,value+50000,0);write_reg(12'h444,value+100000,0);
  write_reg(12'h448,32'h1200fd00+mid,0);write_reg(12'h44c,8,0);
  write_reg(12'h450,{index[7:0],index[15:8],16'b0},0);
  write_reg(12'h454,32'hcdcccc3d,0); // LE float .1
  write_reg(commit,1,expected);
 end endtask
 initial begin
  repeat(20)@(negedge clk);resetn=1;
  read_reg(12'h470,value);
  if($test$plusargs("SOFT"))begin
   if(value!=32'h60003)$fatal(1,"safe submission requires ABI60003");
  end else if(value!=32'h60002)$fatal(1,"baseline must be ABI60002");
  write_reg(12'h5a0,6,0);write_reg(12'h5a4,100000,0);
  for(integer mid=2;mid<=3;mid=mid+1)begin
   write_reg(12'h700+(mid-1)*32,1,0);
   write_reg(12'h71c+(mid-1)*32,1,0);
  end
  write_reg(12'h414,1000000,0);write_reg(12'h404,7,0);
  for(integer mid=2;mid<=3;mid=mid+1)
   for(integer index=16'h701f;index<=16'h7021;index=index+1)
    parameter_frame(mid,index,12'h458,2'b10);
  read_reg(12'h46c,value);if(value!=6)$fatal(1,"expected6 guard rejections");
  read_reg(12'h41c,value);if(value!=0)$fatal(1,"rejected command queued");
  read_reg(12'h420,value);if(value!=0)$fatal(1,"rejected command transmitted");
  read_reg(12'h5a8,value);if(value!=0)$fatal(1,"unexpected motor enable");
  $display("EVIDENCE legacy six writes reject with BRESP=SLVERR; no CAN packet, no AXI deadlock");
  if($test$plusargs("SOFT"))begin
   for(integer mid=2;mid<=3;mid=mid+1)
    for(integer index=16'h701f;index<=16'h7021;index=index+1)begin
     parameter_frame(mid,index,12'h45c,0);
     read_reg(12'h474,value);if(value!=1)$fatal(1,"expected soft guard rejection");
    end
   read_reg(12'h478,value);if(value!=6)$fatal(1,"safe sequence mismatch");
   read_reg(12'h41c,value);if(value!=0)$fatal(1,"soft rejection queued");
   // Type17 read is permitted, proving the board-facing distinction.
   write_reg(12'h448,32'h1100fd02,0);write_reg(12'h45c,1,0);
   read_reg(12'h474,value);if(value!=0)$fatal(1,"read request not accepted");
   read_reg(12'h478,value);if(value!=7)$fatal(1,"successful sequence mismatch");
   read_reg(12'h41c,value);if(value!=1)$fatal(1,"accepted read not queued exactly once");
   // Disabled controller rejects without a bus error, does not enqueue.
   write_reg(12'h404,0,0);write_reg(12'h45c,1,0);
   read_reg(12'h474,value);if(value!=2)$fatal(1,"stopped queue reason missing");
   read_reg(12'h478,value);if(value!=8)$fatal(1,"stopped sequence mismatch");
   read_reg(12'h41c,value);if(value!=0)$fatal(1,"stopped request queued");
   $display("EVIDENCE safe port returns OKAY+explicit rejection, permits Type17, preserves limits");
  end
  $display("TEST_PASS tx_rejection checks=%0d",checks);$finish;
 end
 initial begin #10000000;$fatal(1,"global watchdog");end
endmodule
