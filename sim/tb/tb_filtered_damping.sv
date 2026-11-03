`timescale 1ns/1ps
module tb_filtered_damping;
 reg clk=0;always #5 clk=~clk;
 reg resetn=0,clear=0,take=0,fresh=1,request=0;wire ready;
 reg [31:0] gain=512,bounds=32'hc0004000;
 reg [15:0] desired=32768,measured=32768,nominal=32768;
 wire [15:0] corrected;wire signed [31:0] correction;
 wire [31:0] applied,clipped,stale;
 filtered_damping dut(clk,resetn,clear,request,take,fresh,gain,desired,measured,nominal,bounds,ready,corrected,correction,applied,clipped,stale);
 integer checks=0,k,err,expected,raw;real filtered=0.0,next_value,delta;
 function integer nearest(input real x);begin nearest=x>=0 ? $rtoi(x+0.5):-$rtoi(-x+0.5);end endfunction
 initial begin
  repeat(3)@(negedge clk);resetn=1;
  // Independent real-valued reference across signs, arbitrary gains, stale
  // fallbacks and total-torque clamping. Arithmetic tolerance <=2 codes.
  for(k=0;k<2000;k=k+1)begin
   @(negedge clk);take=0;
   gain=k%11==0 ? 0:(k*37)%513;
   desired=32768+(k*31)%301-150;measured=32768+(k*47)%501-250;
   nominal=k%17==0 ? 49000:32768;fresh=k%19!=0;
   err=$signed({1'b0,desired})-$signed({1'b0,measured});
   next_value=gain==0 ? 0.0:fresh ? filtered+0.25*(err-filtered):0.0;
   delta=next_value*(gain/256.0)*66.0/28.0;
   raw=nearest(delta);if(raw>702)raw=702;if(raw< -702)raw= -702;
   expected=nominal+raw;if(expected<16384)expected=16384;if(expected>49152)expected=49152;
   if(!fresh || gain==0)expected=nominal;
   request=1;#1;wait(ready);#1;if($signed({1'b0,corrected})-expected>2 || expected-$signed({1'b0,corrected})>2)
    $fatal(1,"arithmetic k=%d got=%d expected=%d filter=%f",k,corrected,expected,next_value);
   if(!fresh || gain==0)if(corrected!==nominal)$fatal(1,"bypass changed torque");
   if(corrected<bounds[15:0] || corrected>bounds[31:16])$fatal(1,"total torque cap");
   if(correction>702 || correction< -702)$fatal(1,"additional correction cap");
   @(negedge clk);take=1;@(negedge clk);take=0;request=0;filtered=next_value;checks=checks+1;
  end
  if(applied==0 || clipped==0 || stale==0)$fatal(1,"missing exercised counter");
  @(negedge clk);clear=1;gain=512;desired=32768;measured=32768;nominal=32768;fresh=1;
  @(negedge clk);clear=0;request=1;#1;wait(ready);#1;if(corrected!=32768)$fatal(1,"clear filter");
  gain=513;measured=33000;#1;if(corrected!=nominal)$fatal(1,"invalid gain bypass");
  $display("TEST_PASS filtered damping checks=%0d: signed scale, zero gain bit-exact, stale fallback, LPF, bounded correction/total torque",checks);$finish;
 end
endmodule
