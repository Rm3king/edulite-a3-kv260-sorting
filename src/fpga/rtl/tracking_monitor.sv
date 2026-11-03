`timescale 1ns/1ps
// Observation only. No output is connected to a motion permit, CAN reset or fault register.
// Compares the position the PL last committed for each joint (CSP Type18 write of parameter 0x7016, float32 loc_ref in rad, or a
// Type1 frame, hi[31:16]) with the position the motor reports (Type2
// feedback, data[63:48]) and watches the reported torque code for saturation; both use the same 16-bit code scale (25.14 rad / 65535).
// A "trip" is a condition that persisted for its hold time; it is only latched and time-stamped so software can compare the PL
// detection time with its own polling. Thresholds of 0 disable that check on that axis. Configuration is writable only while disabled.
module tracking_monitor(
 input wire clk,resetn,
 input wire cmd_valid,input wire [31:0] cmd_id,cmd_hi,cmd_lo,
 input wire fb_valid,input wire [2:0] fb_axis,input wire [31:0] fb_hi,fb_lo,
 input wire [31:0] now_us,
 input wire access,wr,input wire [11:0] addr,input wire [31:0] wdata,
 output reg [31:0] rdata
);
 localparam [15:0] TORQUE_MID=16'd32768;
 reg enabled;reg [2:0] selected;reg [31:0] err_hold_us,tq_hold_us;
 reg [15:0] err_thr[0:6],tq_thr[0:6];
 reg [15:0] cmd_pos[0:6],act_pos[0:6],act_tq[0:6],cur_err[0:6],peak_err[0:6],peak_tq[0:6];
 reg [6:0] cmd_seen,err_over,tq_over,err_trip,tq_trip;
 reg [31:0] err_start[0:6],tq_start[0:6],err_trip_us[0:6],tq_trip_us[0:6],samples[0:6];
 reg [31:0] first_trip_us;reg [2:0] first_trip_axis;reg any_trip;
 wire cmd_frame=cmd_valid && cmd_id[28:24]==5'd1 && cmd_id[7:0]>=8'd1 && cmd_id[7:0]<=8'd7;
 wire [2:0] cmd_axis=cmd_id[2:0]-3'd1;
 // Type 4 (stop) for a joint: no setpoint is in force any more (hand guiding, supported rest), so its error check waits for a
 // new setpoint instead of comparing against a stale one.
 wire cmd_stop=cmd_valid && cmd_id[28:24]==5'd4 && cmd_id[7:0]>=8'd1 && cmd_id[7:0]<=8'd7;
 // CSP setpoint: id 0x1200fd|motor, data = 16 70 00 00 + float32 little-endian (cmd_lo holds those four bytes byte-swapped).
 // code = loc_ref*K + 32767.5 with K = 65535/25.14, K in Q16; |loc_ref| >= 2^-17 matters, |loc_ref| > 16 saturates.
 localparam [27:0] KQ16=28'd170839370;
 wire cmd_float=cmd_valid && cmd_id[31:8]==24'h1200fd && cmd_id[7:0]>=8'd1 && cmd_id[7:0]<=8'd7 && cmd_hi==32'h16700000;
 // Pipeline (one register stage each, no stage holds more than one of: multiply, subtract, variable shift, add):
 //  a: byte-swapped bits  b: mantissa*K  c: product retimed + shift amount  d: shift  e: round / clamp  f: store
 reg a_valid,b_valid,c_valid,d_valid,e_valid,f_valid;reg [2:0] a_axis,b_axis,c_axis,d_axis,e_axis,f_axis;
 reg [31:0] a_bits;reg b_sign,c_sign,d_sign,e_sign;reg [7:0] b_exp;reg [51:0] b_prod,c_prod;reg [5:0] c_shift;
 reg c_tiny,c_huge,d_tiny,d_huge;reg [19:0] d_raw;reg [15:0] e_code,f_code;
 wire [7:0] a_exp=a_bits[30:23];
 wire [23:0] a_mant=(a_exp==0) ? 24'd0 : {1'b1,a_bits[22:0]};
 wire [7:0] shift_minus1=8'd165-b_exp;   // exp 110..130 -> 55..35
 wire [18:0] half_up=d_raw[19:1]+d_raw[0];
 wire signed [19:0] signed_mag=d_sign ? -$signed({1'b0,half_up}) : $signed({1'b0,half_up});
 wire signed [19:0] raw_code=20'sd32768+signed_mag;
 wire [15:0] fb_pos=fb_hi[31:16],fb_torque=fb_lo[31:16];
 wire [15:0] ref_pos=cmd_pos[fb_axis];
 wire [15:0] diff=fb_pos>=ref_pos ? fb_pos-ref_pos : ref_pos-fb_pos;
 wire [15:0] tq_dev=fb_torque>=TORQUE_MID ? fb_torque-TORQUE_MID : TORQUE_MID-fb_torque;
 wire err_cond=enabled && cmd_seen[fb_axis] && err_thr[fb_axis]!=0 && diff>err_thr[fb_axis];
 wire tq_cond=enabled && tq_thr[fb_axis]!=0 && tq_dev>tq_thr[fb_axis];
 wire err_fire=err_cond && err_over[fb_axis] && !err_trip[fb_axis] && (now_us-err_start[fb_axis])>=err_hold_us;
 wire tq_fire=tq_cond && tq_over[fb_axis] && !tq_trip[fb_axis] && (now_us-tq_start[fb_axis])>=tq_hold_us;
 integer i;
 task clear_state;begin
  cmd_seen<=0;err_over<=0;tq_over<=0;err_trip<=0;tq_trip<=0;first_trip_us<=0;first_trip_axis<=0;any_trip<=0;
  for(i=0;i<7;i=i+1)begin
   cmd_pos[i]<=0;act_pos[i]<=0;act_tq[i]<=0;cur_err[i]<=0;peak_err[i]<=0;peak_tq[i]<=0;
   err_start[i]<=0;tq_start[i]<=0;err_trip_us[i]<=0;tq_trip_us[i]<=0;samples[i]<=0;
  end
 end endtask
 always @(posedge clk)begin
  if(!resetn)begin
   a_valid<=0;b_valid<=0;c_valid<=0;d_valid<=0;e_valid<=0;f_valid<=0;enabled<=0;selected<=0;err_hold_us<=300000;tq_hold_us<=200000;
   for(i=0;i<7;i=i+1)begin err_thr[i]<=(i<6)?16'd105:16'd0;tq_thr[i]<=0;end  // 105 codes = 40.3 mrad, the ARM guard's limit
   clear_state;
  end else begin
   if(access && wr)case(addr)
    12'h204:begin
     if(wdata[1])clear_state;
     enabled<=wdata[0];
    end
    12'h208:if(wdata<7)selected<=wdata[2:0];
    12'h20c:if(!enabled && wdata!=0)err_hold_us<=wdata;
    12'h210:if(!enabled && wdata!=0)tq_hold_us<=wdata;
    12'h220:if(!enabled)err_thr[selected]<=wdata[15:0];
    12'h224:if(!enabled)tq_thr[selected]<=wdata[15:0];
   endcase
   if(enabled && cmd_frame)begin cmd_pos[cmd_axis]<=cmd_hi[31:16];cmd_seen[cmd_axis]<=1;end
   // Float setpoint pipeline: byte-swap, multiply by K, shift by the exponent, round, add the mid code, clamp, store.
   a_valid<=enabled && cmd_float;a_axis<=cmd_id[2:0]-3'd1;
   a_bits<={cmd_lo[7:0],cmd_lo[15:8],cmd_lo[23:16],cmd_lo[31:24]};
   b_valid<=a_valid;b_axis<=a_axis;b_sign<=a_bits[31];b_exp<=a_exp;b_prod<=a_mant*KQ16;
   c_valid<=b_valid;c_axis<=b_axis;c_sign<=b_sign;c_prod<=b_prod;c_shift<=shift_minus1[5:0];
   c_tiny<=b_exp<8'd110;c_huge<=b_exp>8'd130;
   d_valid<=c_valid;d_axis<=c_axis;d_sign<=c_sign;d_tiny<=c_tiny;d_huge<=c_huge;
   d_raw<=(c_prod>>c_shift);
   e_valid<=d_valid;e_axis<=d_axis;e_sign<=d_sign;
   e_code<=d_huge ? (d_sign ? 16'd0 : 16'd65535) : d_tiny ? 16'd32768 :
           (half_up>19'd32767) ? (d_sign ? 16'd0 : 16'd65535) : raw_code[15:0];
   f_valid<=e_valid;f_axis<=e_axis;f_code<=e_code;
   if(f_valid)begin cmd_pos[f_axis]<=f_code;cmd_seen[f_axis]<=1;end
   if(enabled && cmd_stop)begin cmd_seen[cmd_axis]<=0;err_over[cmd_axis]<=0;end
   if(enabled && fb_valid && fb_axis<7)begin
    act_pos[fb_axis]<=fb_pos;act_tq[fb_axis]<=fb_torque;samples[fb_axis]<=samples[fb_axis]+1'b1;
    if(cmd_seen[fb_axis])begin
     cur_err[fb_axis]<=diff;if(diff>peak_err[fb_axis])peak_err[fb_axis]<=diff;
    end
    if(tq_dev>peak_tq[fb_axis])peak_tq[fb_axis]<=tq_dev;
    if(err_cond)begin
     if(!err_over[fb_axis])begin err_over[fb_axis]<=1;err_start[fb_axis]<=now_us;end
    end else err_over[fb_axis]<=0;
    if(tq_cond)begin
     if(!tq_over[fb_axis])begin tq_over[fb_axis]<=1;tq_start[fb_axis]<=now_us;end
    end else tq_over[fb_axis]<=0;
    if(err_fire)begin
     err_trip[fb_axis]<=1;err_trip_us[fb_axis]<=now_us;
     if(!any_trip)begin any_trip<=1;first_trip_us<=now_us;first_trip_axis<=fb_axis;end
    end
    if(tq_fire)begin
     tq_trip[fb_axis]<=1;tq_trip_us[fb_axis]<=now_us;
     if(!any_trip && !err_fire)begin any_trip<=1;first_trip_us<=now_us;first_trip_axis<=fb_axis;end
    end
   end
  end
 end
 always @*begin
  rdata=0;
  case(addr)
   12'h200:rdata=32'h54524b31;
   12'h204:rdata={30'b0,any_trip,enabled};
   12'h208:rdata={29'b0,selected};12'h20c:rdata=err_hold_us;12'h210:rdata=tq_hold_us;
   12'h214:rdata={17'b0,tq_trip[6:0],1'b0,err_trip[6:0]};
   12'h218:rdata=first_trip_us;12'h21c:rdata={29'b0,first_trip_axis};
   12'h220:rdata={16'b0,err_thr[selected]};12'h224:rdata={16'b0,tq_thr[selected]};
   12'h228:rdata={16'b0,cmd_pos[selected]};12'h22c:rdata={16'b0,act_pos[selected]};
   12'h230:rdata={16'b0,cur_err[selected]};12'h234:rdata={16'b0,peak_err[selected]};
   12'h238:rdata=err_start[selected];12'h23c:rdata=err_trip_us[selected];
   12'h240:rdata={16'b0,act_tq[selected]};12'h244:rdata={16'b0,peak_tq[selected]};
   12'h248:rdata=tq_start[selected];12'h24c:rdata=tq_trip_us[selected];
   12'h250:rdata=samples[selected];
   default:rdata=0;
  endcase
 end
endmodule
