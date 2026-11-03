`timescale 1ns/1ps
module tb_tracking_monitor;
 reg clk=0;always #5 clk=~clk;
 reg resetn=0,cmd_valid=0,fb_valid=0,access=0,wr=0;
 reg [31:0] cmd_id=0,cmd_hi=0,cmd_lo=0,fb_hi=0,fb_lo=0,now_us=0,wdata=0;reg [2:0] fb_axis=0;
 reg [11:0] addr=0;wire [31:0] rdata;
 tracking_monitor dut(.*);
 task write_reg(input [11:0] a,input [31:0] d);
 begin @(negedge clk);access=1;wr=1;addr=a;wdata=d;@(negedge clk);access=0;wr=0;end endtask
 task expect_reg(input [11:0] a,input [31:0] d);
 begin repeat(2)@(negedge clk);addr=a;#1;if(rdata!==d)$fatal(1,"addr=%h got=%0d expected=%0d",a,rdata,d);end endtask
 task select(input [2:0] axis);begin write_reg(12'h208,axis);end endtask
 task command(input [2:0] axis,input [15:0] pos);
 begin @(negedge clk);cmd_valid=1;cmd_id={3'b0,5'd1,16'h8000,5'b0,axis+3'd1};cmd_hi={pos,16'h0};@(negedge clk);cmd_valid=0;end endtask
 task command_float(input [2:0] axis,input [31:0] wire_lo);
 begin @(negedge clk);cmd_valid=1;cmd_id={24'h1200fd,5'b0,axis+3'd1};cmd_hi=32'h16700000;cmd_lo=wire_lo;@(negedge clk);cmd_valid=0;repeat(16)@(negedge clk);end endtask
 task expect_code(input [2:0] axis,input [15:0] code);
 begin select(axis);addr=12'h228;#1;if(rdata<code-1 || rdata>code+1)$fatal(1,"float setpoint axis=%0d got=%0d expected=%0d",axis,rdata,code);end endtask
 task feedback(input [2:0] axis,input [15:0] pos,input [15:0] torque,input [31:0] stamp);
 begin @(negedge clk);fb_valid=1;fb_axis=axis;fb_hi={pos,16'h0};fb_lo={torque,16'h0};now_us=stamp;@(negedge clk);fb_valid=0;end endtask
 integer k;
 initial begin
  repeat(3)@(negedge clk);resetn=1;
  expect_reg(12'h200,32'h54524b31);expect_reg(12'h20c,300000);expect_reg(12'h210,200000);
  select(0);expect_reg(12'h220,105);select(6);expect_reg(12'h220,0);
  // Configuration: tight torque threshold on axis 2, longer error hold on axis 3; accepted while disabled.
  select(2);write_reg(12'h224,3000);select(3);write_reg(12'h20c,300000);
  write_reg(12'h204,1);
  write_reg(12'h220,7);select(3);expect_reg(12'h220,105); // configuration is refused while enabled
  // CSP float setpoints (Type 18, 0x7016): decoded to the same 16-bit code scale as the feedback.
  write_reg(12'h204,1);
  command_float(3'd1,32'h00000000);expect_code(3'd1,16'd32768);
  command_float(3'd1,32'h0000803f);expect_code(3'd1,16'd35374);
  command_float(3'd1,32'h000080bf);expect_code(3'd1,16'd30161);
  command_float(3'd1,32'hd00f4940);expect_code(3'd1,16'd40957);
  command_float(3'd1,32'h000020c0);expect_code(3'd1,16'd26250);
  command_float(3'd1,32'h0000003f);expect_code(3'd1,16'd34071);
  command_float(3'd1,32'h00004041);expect_code(3'd1,16'd64049);
  command_float(3'd1,32'h000040c1);expect_code(3'd1,16'd1486);
  command_float(3'd1,32'h17b7d138);expect_code(3'd1,16'd32768);
  command_float(3'd1,32'h9a9999be);expect_code(3'd1,16'd31985);
  command_float(3'd1,32'h9a99b140);expect_code(3'd1,16'd47235);
  command_float(3'd1,32'h5f708930);expect_code(3'd1,16'd32768);
  // Wrong parameter index is ignored.
  @(negedge clk);cmd_valid=1;cmd_id={24'h1200fd,5'b0,3'd2};cmd_hi=32'h17700000;cmd_lo=32'h0000803f;@(negedge clk);cmd_valid=0;repeat(16)@(negedge clk);
  expect_code(3'd1,16'd32768+16'd0+16'd0-16'd0+16'd0);
  write_reg(12'h204,3);
  // Axis 0 follows within 20 codes for 1 s: no trip, peak recorded.
  command(0,16'd20000);
  for(k=0;k<200;k=k+1)feedback(0,16'd20000+((k%2)?16'd20:16'd5),16'd32768+16'd100,1000+k*5000);
  select(0);expect_reg(12'h234,20);expect_reg(12'h214,0);expect_reg(12'h250,200);
  // Axis 1 lags by 150 codes (57 mrad) from 2.000 s: tripped only after the 300 ms hold, then latched when the error returns.
  command(1,16'd30000);
  for(k=0;k<100;k=k+1)feedback(1,16'd29850,16'd32768,2000000+k*5000);
  select(1);expect_reg(12'h238,2000000);expect_reg(12'h23c,2300000);expect_reg(12'h214,32'h2);
  expect_reg(12'h218,2300000);expect_reg(12'h21c,1);expect_reg(12'h234,150);
  feedback(1,16'd30000,16'd32768,2600000);expect_reg(12'h214,32'h2);
  // A 200 ms excursion shorter than the hold never trips axis 4.
  command(4,16'd10000);
  for(k=0;k<40;k=k+1)feedback(4,16'd10200,16'd32768,3000000+k*5000);
  feedback(4,16'd10000,16'd32768,3200000);
  for(k=0;k<100;k=k+1)feedback(4,16'd10001,16'd32768,3205000+k*5000);
  select(4);expect_reg(12'h23c,0);expect_reg(12'h214,32'h2);
  // Torque saturation on axis 2 for 250 ms (>200 ms hold) trips the torque flag, not the error flag.
  command(2,16'd5000);
  for(k=0;k<60;k=k+1)feedback(2,16'd5002,16'd32768+16'd3500,4000000+k*5000);
  select(2);expect_reg(12'h214,32'h402);expect_reg(12'h24c,4200000);expect_reg(12'h244,3500);
  // Axis 5 has a threshold of 0 on torque: a full-scale code never trips. Axis 6 (gripper) is not an error axis.
  command(5,16'd1000);
  for(k=0;k<100;k=k+1)feedback(5,16'd1000,16'd65000,5000000+k*5000);
  select(5);expect_reg(12'h214,32'h402);
  // Timestamp wrap: a hold that spans the 32-bit counter wrap still trips.
  write_reg(12'h204,2);expect_reg(12'h214,0);write_reg(12'h204,1);
  command(0,16'd20000);
  for(k=0;k<100;k=k+1)feedback(0,16'd20200,16'd32768,32'hfff00000+k*5000);
  select(0);expect_reg(12'h214,32'h1);
  // A stop frame (Type 4) for axis 0 withdraws its setpoint: a later large error is not compared, a new setpoint re-arms it.
  write_reg(12'h204,3);
  command(0,16'd20000);
  @(negedge clk);cmd_valid=1;cmd_id={3'b0,5'd4,16'h00fd,5'b0,3'd1};cmd_hi=0;@(negedge clk);cmd_valid=0;
  for(k=0;k<100;k=k+1)feedback(0,16'd25000,16'd32768,6000000+k*5000);
  select(0);expect_reg(12'h214,0);expect_reg(12'h230,0);
  command(0,16'd20000);
  for(k=0;k<100;k=k+1)feedback(0,16'd25000,16'd32768,7000000+k*5000);
  expect_reg(12'h214,32'h1);
  // Disabled: nothing accumulates.
  write_reg(12'h204,0);select(0);feedback(0,16'd20000,16'd32768,32'd100);expect_reg(12'h250,200);
  $display("TEST_PASS tracking_monitor");
  $finish;
 end
endmodule
