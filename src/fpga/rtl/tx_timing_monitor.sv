`timescale 1ns/1ps
// Observation only. No output is connected to a motion permit or CAN reset.
// Timestamp is the CAN-core transmit-command request, NOT a wire SOF timestamp.
module tx_timing_monitor(
 input wire clk,resetn,event_valid,
 input wire [31:0] event_id,event_hi,event_us,event_due,
 input wire access,wr,input wire [11:0] addr,input wire [31:0] wdata,
 output reg [31:0] rdata
);
 reg enabled;reg [2:0] selected;reg [31:0] period_us;
 reg [6:0] seen;
 reg [31:0] frames[0:6],intervals[0:6],previous[0:6],minimum[0:6],maximum[0:6];
 reg [31:0] peak[0:6],late_peak[0:6],gaps[0:6],outside50[0:6];
 reg [63:0] sum_abs[0:6],sum_square[0:6];
 reg [31:0] hist[0:111];
 reg valid0,valid1,has_previous0,has_previous1;
 reg [2:0] axis0,axis1;
 reg [31:0] dt0,dt1,mag0,mag1,late0,late1;
 reg [63:0] square1;
 wire target_frame=(event_id[28:24]==1 && event_id[7:0]>=1 && event_id[7:0]<=6) ||
   (event_id[28:24]==18 && event_id[7:0]==7 && event_hi[31:16]==16'h1670);
 wire [2:0] axis=event_id[2:0]-3'd1;
 wire [31:0] dt=event_us-previous[axis];
 wire [31:0] magnitude=dt>=period_us ? dt-period_us:period_us-dt;
 wire [31:0] late=event_us-event_due;
 function [3:0] bucket(input [31:0] v);
 begin
  if(v<=1)bucket=0;else if(v<=2)bucket=1;else if(v<=5)bucket=2;
  else if(v<=10)bucket=3;else if(v<=25)bucket=4;else if(v<=50)bucket=5;
  else if(v<=100)bucket=6;else if(v<=250)bucket=7;else if(v<=500)bucket=8;
  else if(v<=1000)bucket=9;else if(v<=2000)bucket=10;else if(v<=5000)bucket=11;
  else if(v<=10000)bucket=12;else if(v<=20000)bucket=13;
  else if(v<=50000)bucket=14;else bucket=15;
 end endfunction
 integer i;
 always @(posedge clk)begin
  if(!resetn)begin
   enabled<=0;selected<=0;period_us<=5000;seen<=0;valid0<=0;valid1<=0;
   has_previous0<=0;has_previous1<=0;axis0<=0;axis1<=0;dt0<=0;dt1<=0;mag0<=0;mag1<=0;late0<=0;late1<=0;square1<=0;
   for(i=0;i<7;i=i+1)begin
    frames[i]<=0;intervals[i]<=0;previous[i]<=0;minimum[i]<=32'hffffffff;maximum[i]<=0;
    peak[i]<=0;late_peak[i]<=0;gaps[i]<=0;outside50[i]<=0;sum_abs[i]<=0;sum_square[i]<=0;
   end
   for(i=0;i<112;i=i+1)hist[i]<=0;
  end else if(access && wr && addr==12'h104 && wdata[1])begin
   enabled<=wdata[0];seen<=0;valid0<=0;valid1<=0;
   for(i=0;i<7;i=i+1)begin
    frames[i]<=0;intervals[i]<=0;previous[i]<=0;minimum[i]<=32'hffffffff;maximum[i]<=0;
    peak[i]<=0;late_peak[i]<=0;gaps[i]<=0;outside50[i]<=0;sum_abs[i]<=0;sum_square[i]<=0;
   end
   for(i=0;i<112;i=i+1)hist[i]<=0;
  end else begin
   valid0<=enabled && event_valid && target_frame;valid1<=valid0;
   if(valid0)begin
    has_previous1<=has_previous0;axis1<=axis0;dt1<=dt0;mag1<=mag0;late1<=late0;
    square1<={32'b0,mag0}*mag0;
   end
   if(access && wr)case(addr)
    12'h104:enabled<=wdata[0];
    12'h108:if(wdata<7)selected<=wdata[2:0];
    12'h10c:if(!enabled && wdata>=1000 && wdata<=1000000)period_us<=wdata;
   endcase
   if(enabled && event_valid && target_frame)begin
    seen[axis]<=1;previous[axis]<=event_us;
    has_previous0<=seen[axis];axis0<=axis;dt0<=dt;mag0<=magnitude;late0<=late;
   end
   // Per-axis accumulation avoids a wide 112-to-1 histogram feedback mux.
   for(i=0;i<7;i=i+1)if(valid1 && axis1==i)begin
    frames[i]<=frames[i]+1'b1;if(late1>late_peak[i])late_peak[i]<=late1;
    if(has_previous1)begin
     intervals[i]<=intervals[i]+1'b1;
     if(dt1<minimum[i])minimum[i]<=dt1;if(dt1>maximum[i])maximum[i]<=dt1;
     if(mag1>peak[i])peak[i]<=mag1;
     sum_abs[i]<=sum_abs[i]+mag1;sum_square[i]<=sum_square[i]+square1;
     if(dt1>period_us+period_us/2)gaps[i]<=gaps[i]+1'b1;
     if(mag1>50)outside50[i]<=outside50[i]+1'b1;
     hist[i*16+bucket(mag1)]<=hist[i*16+bucket(mag1)]+1'b1;
    end
   end
  end
 end
 always @*begin
  rdata=0;
  case(addr)
   12'h100:rdata=32'h54494d31;
   12'h104:rdata={31'b0,enabled};12'h108:rdata={29'b0,selected};12'h10c:rdata=period_us;
   12'h110:rdata=frames[selected];12'h114:rdata=intervals[selected];
   12'h118:rdata=minimum[selected];12'h11c:rdata=maximum[selected];12'h120:rdata=peak[selected];
   12'h124:rdata=sum_abs[selected][31:0];12'h128:rdata=sum_abs[selected][63:32];
   12'h12c:rdata=sum_square[selected][31:0];12'h130:rdata=sum_square[selected][63:32];
   12'h134:rdata=late_peak[selected];12'h138:rdata=previous[selected];
   12'h13c:rdata=gaps[selected];12'h140:rdata=outside50[selected];
   default:if(addr>=12'h180 && addr<12'h1c0)rdata=hist[{selected,4'b0}+addr[5:2]];
  endcase
 end
endmodule
