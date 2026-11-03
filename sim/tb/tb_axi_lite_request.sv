`timescale 1ns/1ps
module tb_axi_lite_request;
 reg clk=0;always #5 clk=~clk;
 reg resetn=0;
 reg [11:0] awaddr=0,araddr=0;reg awvalid=0,wvalid=0,bready=0,arvalid=0,rready=0;
 reg [31:0] wdata=0;reg [3:0] wstrb=15;
 wire awready,wready,bvalid,arready,rvalid;wire [1:0] bresp,rresp;wire [31:0] rdata;
 wire valid,write_en;wire [11:0] addr;wire [31:0] data;wire [3:0] strb;
 reg done=0;reg[31:0] response=0;reg error=0;reg[2:0] delay_count=0;reg wait_release=0;
 reg[31:0] memory[0:255];integer requests=0,checks=0;
 axi_lite_request dut(.clk(clk),.resetn(resetn),
  .s_axi_awaddr(awaddr),.s_axi_awprot(3'b0),.s_axi_awvalid(awvalid),.s_axi_awready(awready),
  .s_axi_wdata(wdata),.s_axi_wstrb(wstrb),.s_axi_wvalid(wvalid),.s_axi_wready(wready),
  .s_axi_bresp(bresp),.s_axi_bvalid(bvalid),.s_axi_bready(bready),
  .s_axi_araddr(araddr),.s_axi_arprot(3'b0),.s_axi_arvalid(arvalid),.s_axi_arready(arready),
  .s_axi_rdata(rdata),.s_axi_rresp(rresp),.s_axi_rvalid(rvalid),.s_axi_rready(rready),
  .req_valid(valid),.req_write(write_en),.req_addr(addr),.req_data(data),.req_strb(strb),
  .req_done(done),.rsp_data(response),.rsp_error(error));
 always @(posedge clk)begin
  done<=0;
  if(!resetn)begin delay_count<=0;wait_release<=0;end
  else if(wait_release)begin if(!valid)wait_release<=0;end
  else if(valid)begin
   if(delay_count==4)begin
    delay_count<=0;done<=1;wait_release<=1;requests<=requests+1;
    error<=addr[1:0]!=0;
    if(write_en)memory[addr[9:2]]<=data;
    else response<=memory[addr[9:2]];
   end else delay_count<=delay_count+1'b1;
  end
 end
 task send_aw(input [11:0] a,input integer delay_ticks);
 begin repeat(delay_ticks)@(negedge clk);awaddr=a;awvalid=1;#1;while(!awready)@(negedge clk);@(posedge clk);@(negedge clk);awvalid=0;end endtask
 task send_w(input [31:0] d,input integer delay_ticks);
 begin repeat(delay_ticks)@(negedge clk);wdata=d;wvalid=1;#1;while(!wready)@(negedge clk);@(posedge clk);@(negedge clk);wvalid=0;end endtask
 task write_word(input [11:0] a,input [31:0] d,input integer skew);
 begin
  fork send_aw(a,skew);send_w(d,7-skew);join
  wait(bvalid);repeat(5)begin @(negedge clk);if(!bvalid || bresp!=0)$fatal(1,"B stalled response");checks=checks+1;end
  bready=1;@(negedge clk);bready=0;
 end endtask
 task read_word(input [11:0] a,input [31:0] expected);
 begin
  @(negedge clk);araddr=a;arvalid=1;#1;while(!arready)@(negedge clk);@(posedge clk);@(negedge clk);arvalid=0;
  wait(rvalid);repeat(5)begin @(negedge clk);if(!rvalid || rresp!=0 || rdata!==expected)$fatal(1,"R data/stability got=%h exp=%h",rdata,expected);checks=checks+1;end
  rready=1;@(negedge clk);rready=0;
 end endtask
 initial begin
  repeat(8)@(negedge clk);resetn=1;
  for(integer i=0;i<64;i=i+1)begin write_word(i*4,32'hfaced000+i,i%8);read_word(i*4,32'hfaced000+i);end
  if(requests!=128)$fatal(1,"duplicate or lost request %d",requests);
  // Reset while only W has arrived: the stale word must never be committed.
  send_w(32'hbad,0);@(negedge clk);resetn=0;repeat(3)@(negedge clk);resetn=1;
  write_word(0,32'h1234,3);read_word(0,32'h1234);
  $display("TEST_PASS AXI checks=%0d split AW/W, backpressure, exactly-once, reset",checks);$finish;
 end
 initial begin #1000000;$fatal(1,"AXI watchdog");end
endmodule
