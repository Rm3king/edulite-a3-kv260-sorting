`timescale 1ns/1ps
// 5 ms synchronized position/velocity/feedforward stream. Six Type1 arm
// packets plus one CSP gripper packet. Physical output is gated in can_engine.
// 220 staged words C00..F6C, atomic commit B60=1, start=2, pause at end=4.
module mit_trajectory_stream(
 input wire clk,resetn,clear,input wire [31:0] now_us,
 input wire [6:0] mask,input wire guard_ok,allow_start,
 input wire access,wr,input wire [11:0] addr,input wire [31:0] wdata,
 output reg [31:0] rdata,output reg error,
 output wire valid,input wire ready,output wire [191:0] packet,
 output wire [111:0] encoded_position,encoded_velocity,encoded_torque,
 output wire [223:0] encoded_gains,output wire [31:0] gripper_target,
 output reg running,output reg [31:0] fault
);
 reg [31:0] stage[0:219],pending[0:219],active[0:219];
 reg [219:0] written;
 reg queued,holding,pause_end;
 reg [6:0] active_mask;
 reg [3:0] state;
 reg [1:0] field;
 reg [2:0] axis;
 reg [31:0] tick,next_due,batch_due,samples,completed,rejects,remainder;
 reg [24:0] phase;
 reg start;
 wire done;wire [31:0] position[0:6];wire [6:0] rejected;
 reg [31:0] coef[0:55],eval_lower[0:6],eval_upper[0:6];
 reg [31:0] values[0:20];
 reg [15:0] pq[0:6],vq[0:6],tq[0:6];
 reg [31:0] claw;
 wire at_rest=vq[0]==32768 && vq[1]==32768 && vq[2]==32768 &&
              vq[3]==32768 && vq[4]==32768 && vq[5]==32768;
 wire [63:0] phase_product=stage[217]*{32'b0,stage[218]}+stage[219];
 wire commit_ok=written=={220{1'b1}} && !queued && !pause_end && stage[217]>=1 && stage[217]<=120000 &&
  stage[216]<=512 && stage[218]<=16777216 && stage[219]<stage[217] && phase_product==16777216;
 wire [32:0] rem_sum={1'b0,remainder}+active[219];
 wire [31:0] ident=axis<6 ? {8'h01,tq[axis],5'b0,axis+3'd1} : 32'h1200fd07;
 wire [63:0] payload=axis<6 ? {pq[axis],vq[axis],active[210+axis]} :
   {32'h16700000,claw[7:0],claw[15:8],claw[23:16],claw[31:24]};
 assign packet={batch_due,batch_due+32'd4500,ident,32'd8,payload};
 assign valid=running && state==6 && active_mask[axis] && fault==0 && guard_ok;
 assign gripper_target=claw;
 genvar g;generate for(g=0;g<7;g=g+1)begin:fields_pack
  assign encoded_position[g*16 +: 16]=pq[g];
  assign encoded_velocity[g*16 +: 16]=vq[g];
  assign encoded_torque[g*16 +: 16]=tq[g];
  assign encoded_gains[g*32 +: 32]=active[210+g];
 end endgenerate
 // Round to the nearest protocol code; within one code of SDK truncation.
 // Bounds are checked on the resulting encoded command by can_engine.
 function [15:0] encode(input [31:0] q);
 reg [47:0] product;begin product=q*16'd65535+48'd8388608;encode=product[39:24];end
 endfunction
 function [31:0] qfloat(input [31:0] bits);
 reg sign;reg [31:0] mag,tail,half;reg [24:0] significand;
 integer leading,shift,j,exponent;
 begin
  sign=bits[31];mag=sign ? (~bits+1'b1):bits;leading=0;
  for(j=0;j<32;j=j+1)if(mag[j])leading=j;
  exponent=leading+103;significand=0;tail=0;half=0;
  if(leading<=23)significand=mag<<(23-leading);
  else begin shift=leading-23;significand=mag>>shift;
   tail=mag&((32'b1<<shift)-1);half=32'b1<<(shift-1);
   if(tail>half || (tail==half && significand[0]))significand=significand+1'b1;
   if(significand[24])begin significand=significand>>1;exponent=exponent+1;end
  end
  qfloat=mag==0 ? 0:{sign,exponent[7:0],significand[22:0]};
 end endfunction
 always @* begin
  rdata=0;error=0;
  case(addr)
   12'hb40:rdata={26'b0,pause_end,3'b0,queued,running};
   12'hb44:rdata=fault;
   12'hb48:rdata=samples;
   12'hb4c:rdata=completed;
   12'hb50:rdata=tick;
   12'hb54:rdata=active[217];
   12'hb58:rdata=next_due;
   12'hb5c:rdata={31'b0,holding};
   12'hb60:rdata=rejects;
   12'hb64:rdata={25'b0,active_mask};
   12'hb68:rdata=32'h00070003;
   default:if(addr>=12'hc00 && addr<=12'hf6c)rdata=stage[(addr-12'hc00)>>2];else error=1;
  endcase
  if(wr && addr!=12'hb60 && !(addr>=12'hc00 && addr<=12'hf6c))error=1;
 end
 integer i;
 always @(posedge clk)begin
  if(!resetn || clear)begin
   written<=0;queued<=0;holding<=0;pause_end<=0;active_mask<=0;
   state<=0;field<=0;axis<=0;tick<=0;next_due<=0;batch_due<=0;
   samples<=0;completed<=0;rejects<=0;remainder<=0;phase<=0;start<=0;running<=0;fault<=0;claw<=0;
   for(i=0;i<220;i=i+1)begin stage[i]<=0;pending[i]<=0;active[i]<=0;end
   for(i=0;i<56;i=i+1)coef[i]<=0;
   for(i=0;i<21;i=i+1)values[i]<=0;
   for(i=0;i<7;i=i+1)begin eval_lower[i]<=0;eval_upper[i]<=0;pq[i]<=0;vq[i]<=0;tq[i]<=0;end
  end else begin
   start<=0;
   if(access && wr && !error)begin
    if(addr>=12'hc00 && addr<=12'hf6c)begin stage[(addr-12'hc00)>>2]<=wdata;written[(addr-12'hc00)>>2]<=1;end
    else if(addr==12'hb60)case(wdata)
     1:if(commit_ok)begin
       for(i=0;i<220;i=i+1)pending[i]<=stage[i];queued<=1;written<=0;
      end else rejects<=rejects+1;
     2:if(!running && queued && mask==127 && fault==0 && allow_start)begin
       for(i=0;i<220;i=i+1)active[i]<=pending[i];
       queued<=0;running<=1;active_mask<=mask;next_due<=now_us+1000;phase<=0;tick<=0;remainder<=0;
      end else rejects<=rejects+1;
     4:if(running)begin pause_end<=1;queued<=0;end else rejects<=rejects+1;
     default:rejects<=rejects+1;
    endcase
   end
   if(running && fault==0)begin
    if(mask!=active_mask)fault<=1;
    else if(state!=0 && $signed(now_us-(batch_due+4500))>=0)fault<=2;
    else case(state)
     // Match the optimized ARM producer's bounded 1ms output lookahead.
     // The shared deadline FIFO still releases every frame only at batch_due.
     0:if($signed(now_us-(next_due-32'd1000))>=0)begin batch_due<=next_due;next_due<=next_due+5000;field<=0;state<=1;end
     1:begin
       for(i=0;i<56;i=i+1)coef[i]<=active[field*56+i];
       for(i=0;i<7;i=i+1)begin eval_lower[i]<=active[168+field*14+i];eval_upper[i]<=active[175+field*14+i];end
       state<=2;
      end
     2:begin start<=1;state<=3;end
     3:if(done)begin
       if(rejected!=0)fault<=3;
       else begin
        for(i=0;i<7;i=i+1)values[field*7+i]<=position[i];
        if(field==2)state<=4;else begin field<=field+1;state<=1;end
       end
      end
     4:begin
       for(i=0;i<7;i=i+1)begin pq[i]<=encode(values[i]);vq[i]<=encode(values[7+i]);tq[i]<=encode(values[14+i]);end
       claw<=qfloat(values[6]);state<=5;
      end
     5:if(!guard_ok)fault<=4;else begin axis<=0;state<=6;end
     6:if(!guard_ok)fault<=4;
       else if(!active_mask[axis] || ready)begin
        if(axis==6)begin samples<=samples+1;state<=7;end else axis<=axis+1'b1;
       end
     7:begin
       state<=0;
       if(tick==active[217])begin
        if(!holding)completed<=completed+1;holding<=1;
        if((!queued || pause_end) && !at_rest)fault<=6;
        if(queued && !pause_end && !(access && wr && addr==12'hb60 && wdata==4))begin
         for(i=0;i<220;i=i+1)active[i]<=pending[i];
         queued<=0;holding<=0;phase<=0;remainder<=0;tick<=0;
        end
       end else begin
        tick<=tick+1;
        if(tick+1==active[217])phase<=25'h1000000;
        else phase<=phase+active[218]+(rem_sum>=active[217]);
        remainder<=rem_sum>=active[217] ? rem_sum-active[217]:rem_sum;
       end
      end
     default:fault<=5;
    endcase
   end
  end
 end
 trajectory_eval_wide evaluator(.ap_clk(clk),.ap_rst(!resetn || clear),.ap_start(start),
.ap_done(done),.ap_idle(),.ap_ready(),.phase(phase),.rejected(rejected),.rejected_ap_vld(),
.coefficient_0_0(coef[0]),
.coefficient_0_1(coef[1]),
.coefficient_0_2(coef[2]),
.coefficient_0_3(coef[3]),
.coefficient_0_4(coef[4]),
.coefficient_0_5(coef[5]),
.coefficient_0_6(coef[6]),
.coefficient_0_7(coef[7]),
.coefficient_1_0(coef[8]),
.coefficient_1_1(coef[9]),
.coefficient_1_2(coef[10]),
.coefficient_1_3(coef[11]),
.coefficient_1_4(coef[12]),
.coefficient_1_5(coef[13]),
.coefficient_1_6(coef[14]),
.coefficient_1_7(coef[15]),
.coefficient_2_0(coef[16]),
.coefficient_2_1(coef[17]),
.coefficient_2_2(coef[18]),
.coefficient_2_3(coef[19]),
.coefficient_2_4(coef[20]),
.coefficient_2_5(coef[21]),
.coefficient_2_6(coef[22]),
.coefficient_2_7(coef[23]),
.coefficient_3_0(coef[24]),
.coefficient_3_1(coef[25]),
.coefficient_3_2(coef[26]),
.coefficient_3_3(coef[27]),
.coefficient_3_4(coef[28]),
.coefficient_3_5(coef[29]),
.coefficient_3_6(coef[30]),
.coefficient_3_7(coef[31]),
.coefficient_4_0(coef[32]),
.coefficient_4_1(coef[33]),
.coefficient_4_2(coef[34]),
.coefficient_4_3(coef[35]),
.coefficient_4_4(coef[36]),
.coefficient_4_5(coef[37]),
.coefficient_4_6(coef[38]),
.coefficient_4_7(coef[39]),
.coefficient_5_0(coef[40]),
.coefficient_5_1(coef[41]),
.coefficient_5_2(coef[42]),
.coefficient_5_3(coef[43]),
.coefficient_5_4(coef[44]),
.coefficient_5_5(coef[45]),
.coefficient_5_6(coef[46]),
.coefficient_5_7(coef[47]),
.coefficient_6_0(coef[48]),
.coefficient_6_1(coef[49]),
.coefficient_6_2(coef[50]),
.coefficient_6_3(coef[51]),
.coefficient_6_4(coef[52]),
.coefficient_6_5(coef[53]),
.coefficient_6_6(coef[54]),
.coefficient_6_7(coef[55]),
.lower_0(eval_lower[0]),
.upper_0(eval_upper[0]),
.position_0(position[0]),
.position_0_ap_vld(),
.lower_1(eval_lower[1]),
.upper_1(eval_upper[1]),
.position_1(position[1]),
.position_1_ap_vld(),
.lower_2(eval_lower[2]),
.upper_2(eval_upper[2]),
.position_2(position[2]),
.position_2_ap_vld(),
.lower_3(eval_lower[3]),
.upper_3(eval_upper[3]),
.position_3(position[3]),
.position_3_ap_vld(),
.lower_4(eval_lower[4]),
.upper_4(eval_upper[4]),
.position_4(position[4]),
.position_4_ap_vld(),
.lower_5(eval_lower[5]),
.upper_5(eval_upper[5]),
.position_5(position[5]),
.position_5_ap_vld(),
.lower_6(eval_lower[6]),
.upper_6(eval_upper[6]),
.position_6(position[6]),
.position_6_ap_vld());
endmodule
