`timescale 1ns/1ps
// 100 MHz, fixed 5 ms trajectory tick. One active + one pending segment.
// Registers 900..9b0: 28 Q8.24 coefficients, 7 lower, 7 upper,
// ticks, floor(2^24/ticks), remainder. Atomic commit requires all 45 writes.
// 820 commands: 1 commit, 2 start, 4 finish current segment and discard next.
// Faults never restart. clear (CAN stopped) cancels all buffered work.
// 840..8b4: bounded online correction (visual servo). Six joint offsets (Q8.24 rad) are added to the
// evaluated setpoint before the float conversion. Hardware bounds: |offset_j|<=limit_j (8a0..8b4, each
// <=0.25 rad, 0 locks the joint), change per 5 ms tick <=step (848, <=0.003 rad), a target older than
// lease_us (84c) freezes the offsets, disabling (840) slews them back to zero, limits/step/lease are locked
// while running, the gripper axis is never offset, and an offset setpoint outside the segment's own
// lower/upper bounds is fault 6. 850..864 targets, 870..884 applied, 888 writes, 88c clamps, 890 age.
// A refused correction write (locked, over a ceiling, target while disabled) is ignored and counted at 898;
// it never returns SLVERR, which can abort the A53.
module trajectory_stream(
 input wire clk,resetn,clear,input wire [31:0] now_us,
 input wire [6:0] mask,input wire guard_ok,
 input wire access,wr,input wire [11:0] addr,input wire [31:0] wdata,
 output reg [31:0] rdata,output reg error,
 output wire valid,input wire ready,output wire [191:0] packet,
 output wire [223:0] targets,output reg running,output reg [31:0] fault
);
 reg [31:0] stage[0:44],stage_wide[0:72],pending[0:72],active[0:72];
 reg [72:0] written_wide;
 reg [44:0] written;
 reg queued,holding,pause_end;
 reg [6:0] active_mask;
 reg [3:0] state;
 reg [2:0] axis;
 reg [31:0] tick,next_due,batch_due,samples,completed,rejects;
 reg [24:0] phase;
 reg [31:0] remainder;
 reg start;
 wire done;
 wire [31:0] position[0:6];
 reg [31:0] q[0:6],target[0:6];
 wire [6:0] rejected;
 localparam [31:0] CORR_LIMIT_MAX=32'd4194304,CORR_STEP_MAX=32'd50332,CORR_LEASE_MAX=32'd500000;   // 0.25 rad, 0.003 rad/tick, 0.5 s
 reg corr_enable;
 reg [31:0] corr_step,corr_lease,corr_last_us,corr_writes,corr_clamps,corr_rejects;
 reg [31:0] corr_limit[0:5];
 wire corr_limit_addr=addr>=12'h8a0 && addr<=12'h8b4;
 wire corr_any_limit=corr_limit[0]!=0 || corr_limit[1]!=0 || corr_limit[2]!=0 || corr_limit[3]!=0 || corr_limit[4]!=0 || corr_limit[5]!=0;
 reg signed [31:0] corr_target[0:5],corr_applied[0:5];
 wire corr_stale=$signed(now_us-corr_last_us)>$signed(corr_lease);
 wire signed [31:0] corr_wdata=$signed(wdata);
 wire corr_target_addr=addr>=12'h850 && addr<=12'h864;
 wire [2:0] corr_index=(addr-12'h850)>>2;
 wire signed [32:0] corrected[0:6];
 wire [6:0] corr_out;
 genvar c;generate for(c=0;c<7;c=c+1)begin:corr_sum
  assign corrected[c]=$signed({position[c][31],position[c]})+(c<6 ? $signed({corr_applied[c<6?c:0][31],corr_applied[c<6?c:0]}) : 33'sd0);
  assign corr_out[c]=corrected[c]<$signed({active[56+c][31],active[56+c]}) || corrected[c]>$signed({active[63+c][31],active[63+c]});
 end endgenerate
 wire [63:0] phase_product=stage[42]*{32'b0,stage[43]}+stage[44];
 wire commit_ok=written=={45{1'b1}} && !queued && stage[42]>=1 && stage[42]<=120000 &&
    stage[43]<=16777216 && stage[44]<stage[42] && phase_product==16777216;
 wire [63:0] wide_phase_product=stage_wide[70]*{32'b0,stage_wide[71]}+stage_wide[72];
 wire wide_commit_ok=written_wide=={73{1'b1}} && !queued && stage_wide[70]>=1 && stage_wide[70]<=120000 &&
   stage_wide[71]<=16777216 && stage_wide[72]<stage_wide[70] && wide_phase_product==16777216;
 wire [32:0] rem_sum={1'b0,remainder}+active[72];
 wire [31:0] outbits=target[axis];
 assign packet={batch_due,batch_due+32'd4500,24'h1200fd,5'b0,axis+3'd1,
                32'd8,32'h16700000,outbits[7:0],outbits[15:8],outbits[23:16],outbits[31:24]};
 assign valid=running && state==5 && active_mask[axis] && fault==0 && guard_ok;
 genvar g;generate for(g=0;g<7;g=g+1)begin:targets_pack
  assign targets[g*32 +: 32]=target[g];
 end endgenerate
 // Q8.24 -> IEEE754 binary32, round-to-nearest ties-to-even, exact sign/zero.
 function [31:0] qfloat(input [31:0] bits);
 reg sign;reg [31:0] mag;reg [24:0] significand;
 reg [31:0] tail,half;integer leading,shift,j,exponent;
 begin
  sign=bits[31];mag=sign ? (~bits+1'b1) : bits;
  leading=0;for(j=0;j<32;j=j+1)if(mag[j])leading=j;
  exponent=leading+103;significand=0;tail=0;half=0;
  if(leading<=23)significand=mag<<(23-leading);
  else begin
   shift=leading-23;significand=mag>>shift;
   tail=mag&((32'b1<<shift)-1);half=32'b1<<(shift-1);
   if(tail>half || (tail==half && significand[0]))significand=significand+1'b1;
   if(significand[24])begin significand=significand>>1;exponent=exponent+1;end
  end
  qfloat=mag==0 ? 0 : {sign,exponent[7:0],significand[22:0]};
 end endfunction
 reg corr_refuse;
 always @* begin
  rdata=0;error=0;
  case(addr)
   12'h800:rdata={26'b0,pause_end,3'b0,queued,running};
   12'h804:rdata=fault;
   12'h808:rdata=samples;
   12'h80c:rdata=completed;
   12'h810:rdata=tick;
   12'h814:rdata=active[70];
   12'h818:rdata=next_due;
   12'h81c:rdata={31'b0,holding};
   12'h820:begin rdata=rejects;end
   12'h824:rdata={25'b0,active_mask};
   12'h828:rdata=32'h00070008;
   12'h894:rdata=32'h43520001;   // 'CR' online correction v1
   12'h840:rdata={30'b0,corr_stale,corr_enable};
   12'h848:rdata=corr_step;
   12'h84c:rdata=corr_lease;
   12'h888:rdata=corr_writes;
   12'h88c:rdata=corr_clamps;
   12'h890:rdata=now_us-corr_last_us;
   12'h898:rdata=corr_rejects;
   default:if(corr_target_addr)rdata=corr_target[corr_index];
    else if(addr>=12'h870 && addr<=12'h884)rdata=corr_applied[(addr-12'h870)>>2];
    else if(corr_limit_addr)rdata=corr_limit[(addr-12'h8a0)>>2];
    else if(addr>=12'h900 && addr<=12'h9b0)rdata=stage[(addr-12'h900)>>2];
    else if(addr>=12'ha00 && addr<=12'hb20)rdata=stage_wide[(addr-12'ha00)>>2];else error=1;
  endcase
  if(wr && addr!=12'h820 && !(addr>=12'h900 && addr<=12'h9b0) && !(addr>=12'ha00 && addr<=12'hb20) &&
     addr!=12'h840 && addr!=12'h848 && addr!=12'h84c && !corr_target_addr && !corr_limit_addr)error=1;
  corr_refuse=0;
  if(addr==12'h840 && wdata[0] && (!corr_any_limit || corr_step==0 || corr_lease==0))corr_refuse=1;
  if((corr_limit_addr || addr==12'h848 || addr==12'h84c) && running)corr_refuse=1;
  if(corr_limit_addr && wdata>CORR_LIMIT_MAX)corr_refuse=1;
  if(addr==12'h848 && wdata>CORR_STEP_MAX)corr_refuse=1;
  if(addr==12'h84c && (wdata==0 || wdata>CORR_LEASE_MAX))corr_refuse=1;
  if(corr_target_addr && !corr_enable)corr_refuse=1;
  if(!(addr==12'h840 || addr==12'h848 || addr==12'h84c || corr_limit_addr || corr_target_addr))corr_refuse=0;
 end
 integer i;
 always @(posedge clk)begin
  if(!resetn || clear)begin
   written<=0;written_wide<=0;queued<=0;holding<=0;pause_end<=0;active_mask<=0;state<=0;axis<=0;
   tick<=0;next_due<=0;batch_due<=0;samples<=0;completed<=0;rejects<=0;
   phase<=0;remainder<=0;start<=0;running<=0;fault<=0;
   for(i=0;i<45;i=i+1)stage[i]<=0;
   for(i=0;i<73;i=i+1)begin stage_wide[i]<=0;pending[i]<=0;active[i]<=0;end
   for(i=0;i<7;i=i+1)begin q[i]<=0;target[i]<=0;end
   corr_enable<=0;corr_step<=0;corr_lease<=0;corr_last_us<=0;corr_writes<=0;corr_clamps<=0;corr_rejects<=0;
   for(i=0;i<6;i=i+1)begin corr_target[i]<=0;corr_applied[i]<=0;corr_limit[i]<=0;end
  end else begin
   start<=0;
   if(access && wr && !error)begin
    if(addr>=12'h900 && addr<=12'h9b0)begin stage[(addr-12'h900)>>2]<=wdata;written[(addr-12'h900)>>2]<=1;end
    else if(addr>=12'ha00 && addr<=12'hb20)begin stage_wide[(addr-12'ha00)>>2]<=wdata;written_wide[(addr-12'ha00)>>2]<=1;end
    else if(corr_refuse)corr_rejects<=corr_rejects+1;
    else if(addr==12'h840)begin corr_enable<=wdata[0];corr_last_us<=now_us;if(!wdata[0])for(i=0;i<6;i=i+1)corr_target[i]<=0;end
    else if(corr_limit_addr)corr_limit[(addr-12'h8a0)>>2]<=wdata;
    else if(addr==12'h848)corr_step<=wdata;
    else if(addr==12'h84c)corr_lease<=wdata;
    else if(corr_target_addr)begin
     corr_writes<=corr_writes+1;corr_last_us<=now_us;
     if(corr_wdata>$signed(corr_limit[corr_index]))begin corr_target[corr_index]<=corr_limit[corr_index];corr_clamps<=corr_clamps+1;end
     else if(corr_wdata<-$signed(corr_limit[corr_index]))begin corr_target[corr_index]<=-corr_limit[corr_index];corr_clamps<=corr_clamps+1;end
     else corr_target[corr_index]<=corr_wdata;
    end
    else if(addr==12'h820)case(wdata)
     1:if(commit_ok && !pause_end)begin
       for(i=0;i<56;i=i+1)pending[i]<=(i%8<4) ? stage[(i/8)*4+i%8] : 0;
       for(i=0;i<17;i=i+1)pending[56+i]<=stage[28+i];queued<=1;written<=0;
      end else rejects<=rejects+1;
     8:if(wide_commit_ok && !pause_end)begin
       for(i=0;i<73;i=i+1)pending[i]<=stage_wide[i];queued<=1;written_wide<=0;
      end else rejects<=rejects+1;
     2:if(!running && queued && mask!=0 && fault==0)begin
       for(i=0;i<73;i=i+1)active[i]<=pending[i];
       queued<=0;running<=1;active_mask<=mask;next_due<=now_us+1000;phase<=0;tick<=0;
      end else rejects<=rejects+1;
     4:if(running)begin pause_end<=1;queued<=0;end else rejects<=rejects+1;
     default:rejects<=rejects+1;
    endcase
   end
   if(running && fault==0)begin
    if(mask!=active_mask)fault<=1;
    else if(state!=0 && $signed(now_us-(batch_due+4500))>=0)fault<=2;
    else case(state)
     0:if($signed(now_us-next_due)>=0)begin batch_due<=next_due;next_due<=next_due+5000;state<=1;end
     1:begin start<=1;state<=2;end
     2:if(done)begin
       if(rejected!=0)fault<=3;
       else if(corr_out!=0)fault<=6;
       else begin for(i=0;i<7;i=i+1)q[i]<=corrected[i][31:0];state<=3;end
      end
     3:begin for(i=0;i<7;i=i+1)target[i]<=qfloat(q[i]);state<=4;end
     4:if(!guard_ok)fault<=4;else begin axis<=0;state<=5;end
     5:if(!guard_ok)fault<=4;
       else if(!active_mask[axis] || ready)begin
        if(axis==6)begin samples<=samples+1;state<=6;end else axis<=axis+1'b1;
       end
     6:begin
       state<=0;
       // one bounded step of the applied offsets per tick: toward the target (enabled, fresh), held (stale), toward zero (disabled)
       for(i=0;i<6;i=i+1)begin
        if(!corr_enable || !corr_stale)begin
         if($signed((corr_enable ? corr_target[i] : 32'sd0)-corr_applied[i])>$signed(corr_step))corr_applied[i]<=corr_applied[i]+corr_step;
         else if($signed((corr_enable ? corr_target[i] : 32'sd0)-corr_applied[i])<-$signed(corr_step))corr_applied[i]<=corr_applied[i]-corr_step;
         else corr_applied[i]<=corr_enable ? corr_target[i] : 32'sd0;
        end
       end
       if(tick==active[70])begin
        if(!holding)completed<=completed+1;
        holding<=1;
        if(queued && !pause_end && !(access && wr && addr==12'h820 && wdata==4))begin
         for(i=0;i<73;i=i+1)active[i]<=pending[i];
         queued<=0;holding<=0;phase<=0;remainder<=0;tick<=0;
        end
       end else begin
        tick<=tick+1;
        if(tick+1==active[70])phase<=25'h1000000;
        else phase<=phase+active[71]+(rem_sum>=active[70]);
        remainder<=rem_sum>=active[70] ? rem_sum-active[70] : rem_sum;
       end
      end
     default:fault<=5;
    endcase
   end
  end
 end
 trajectory_eval_wide evaluator(.ap_clk(clk),.ap_rst(!resetn || clear),.ap_start(start),
 .ap_done(done),.ap_idle(),.ap_ready(),.phase(phase),.rejected(rejected),.rejected_ap_vld(),
.coefficient_0_0(active[0]),
.coefficient_0_1(active[1]),
.coefficient_0_2(active[2]),
.coefficient_0_3(active[3]),
.coefficient_0_4(active[4]),
.coefficient_0_5(active[5]),
.coefficient_0_6(active[6]),
.coefficient_0_7(active[7]),
.coefficient_1_0(active[8]),
.coefficient_1_1(active[9]),
.coefficient_1_2(active[10]),
.coefficient_1_3(active[11]),
.coefficient_1_4(active[12]),
.coefficient_1_5(active[13]),
.coefficient_1_6(active[14]),
.coefficient_1_7(active[15]),
.coefficient_2_0(active[16]),
.coefficient_2_1(active[17]),
.coefficient_2_2(active[18]),
.coefficient_2_3(active[19]),
.coefficient_2_4(active[20]),
.coefficient_2_5(active[21]),
.coefficient_2_6(active[22]),
.coefficient_2_7(active[23]),
.coefficient_3_0(active[24]),
.coefficient_3_1(active[25]),
.coefficient_3_2(active[26]),
.coefficient_3_3(active[27]),
.coefficient_3_4(active[28]),
.coefficient_3_5(active[29]),
.coefficient_3_6(active[30]),
.coefficient_3_7(active[31]),
.coefficient_4_0(active[32]),
.coefficient_4_1(active[33]),
.coefficient_4_2(active[34]),
.coefficient_4_3(active[35]),
.coefficient_4_4(active[36]),
.coefficient_4_5(active[37]),
.coefficient_4_6(active[38]),
.coefficient_4_7(active[39]),
.coefficient_5_0(active[40]),
.coefficient_5_1(active[41]),
.coefficient_5_2(active[42]),
.coefficient_5_3(active[43]),
.coefficient_5_4(active[44]),
.coefficient_5_5(active[45]),
.coefficient_5_6(active[46]),
.coefficient_5_7(active[47]),
.coefficient_6_0(active[48]),
.coefficient_6_1(active[49]),
.coefficient_6_2(active[50]),
.coefficient_6_3(active[51]),
.coefficient_6_4(active[52]),
.coefficient_6_5(active[53]),
.coefficient_6_6(active[54]),
.coefficient_6_7(active[55]),
.lower_0(active[56]),
.upper_0(active[63]),
.position_0(position[0]),
.position_0_ap_vld(),
.lower_1(active[57]),
.upper_1(active[64]),
.position_1(position[1]),
.position_1_ap_vld(),
.lower_2(active[58]),
.upper_2(active[65]),
.position_2(position[2]),
.position_2_ap_vld(),
.lower_3(active[59]),
.upper_3(active[66]),
.position_3(position[3]),
.position_3_ap_vld(),
.lower_4(active[60]),
.upper_4(active[67]),
.position_4(position[4]),
.position_4_ap_vld(),
.lower_5(active[61]),
.upper_5(active[68]),
.position_5(position[5]),
.position_5_ap_vld(),
.lower_6(active[62]),
.upper_6(active[69]),
.position_6(position[6]),
.position_6_ap_vld());
endmodule
