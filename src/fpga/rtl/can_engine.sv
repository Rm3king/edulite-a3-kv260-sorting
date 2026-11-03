`timescale 1ns/1ps
// CAN register engine, deadline queue, autonomous RX and feedback table.
// 100 MHz only. All software-visible timestamps are wrapping microseconds.
module can_engine (
 input wire clk, resetn, can_rx, output wire can_tx,
 input wire req_valid, req_write, input wire [11:0] req_addr,
 input wire [31:0] req_data, input wire [3:0] req_strb,
 output reg req_done, output reg [31:0] rsp_data, output reg rsp_error,
 output wire [31:0] debug_us, output wire [7:0] debug_status
);
 localparam IDLE=0, WB=1, GAP=2, HOST_WAIT=3, STATUS=4,
            TX_FILL=5, TX_NEXT=6, TX_FIRE=7, TX_DONE=8,
            RX_READ=9, RX_NEXT=10, RX_RELEASE=11, RX_DONE=12;
 reg [3:0] state, after_wb;
 reg tx_pending;
 reg [31:0] tx_deadline, abort_count, rejected_count;
 // ABI 6.3: ordinary command refusals have an OKAY transport response and
 // an explicit result/sequence. Legacy 0x458 keeps its original ABI.
 reg [31:0] safe_submit_status, safe_submit_sequence;
 reg [6:0] motion_mask, motion_active_mask;
 reg [31:0] feedback_timeout_us;
 // v4: independent bounded PP preparation and limits for all seven motors. Float comparisons use IEEE754 order
 // keys, not arithmetic. Configuration is trusted and writable only stopped.
 reg [6:0] pp_permit, csp_mode, mit_mode;
 reg [5:0] pp_ready[0:6];
 reg [31:0] pp_min[0:6],pp_max[0:6],pp_speed[0:6],pp_accel[0:6],pp_current[0:6];
 reg [5:0] gap;
 reg [31:0] now_us, heartbeat_us;
 reg [6:0] divider;
 reg [3:0] control;
 reg [31:0] watchdog_us, fault, expired_count, submitted_count, received_count, rx_overflow;
 reg [31:0] last_submit_us, last_lateness_us;
 reg [31:0] stage_due,stage_expiry,stage_id,stage_flags,stage_hi,stage_lo;
 reg tx_push,tx_pop,rx_push,rx_pop,clear_queues;
 wire tx_full,tx_empty,rx_full,rx_empty;
 wire [4:0] tx_count;
 wire [8:0] rx_count;
 wire [191:0] tx_head;
 wire [31:0] timing_rdata;
 wire [159:0] rx_head;
 reg [159:0] rx_packet;
 reg [191:0] active_tx;
 tx_timing_monitor timing_observer(.clk(clk),.resetn(resetn),
  .event_valid(state==TX_FIRE && $signed(now_us-active_tx[159:128])<0),
  .event_id(active_tx[127:96]),.event_hi(active_tx[63:32]),
  .event_us(now_us),.event_due(active_tx[191:160]),
  .access(req_valid && state==IDLE && req_addr>=12'h100 && req_addr<12'h200 && req_addr[1:0]==0 && (!req_write || req_strb==15)),
  .wr(req_write),.addr(req_addr),.wdata(req_data),.rdata(timing_rdata));
 reg [7:0] rx_bytes[0:12];
 reg [3:0] index;
 reg [31:0] rx_time;
 reg [63:0] motor_data[0:6];
 reg [31:0] motor_id[0:6],motor_time[0:6],motor_seen;
 reg [7:0] wb_addr,wb_out,wb_result;
 reg wb_cycle,wb_write,core_reset;
 wire [7:0] wb_in;
 wire wb_ack,core_tx,bus_off_n,irq_n;
 wire bus_off = !bus_off_n;
 reg [7:0] core_status;
 wire [31:0] head_due=tx_head[191:160], head_expiry=tx_head[159:128];
 wire head_due_now=$signed(now_us-head_due)>=0;
 wire head_expired=$signed(now_us-head_expiry)>=0;
 wire lease_expired=control[0] && watchdog_us!=0 && (now_us-heartbeat_us)>=watchdog_us;
 wire [28:0] received_id=rx_bytes[0][7] ?
   {rx_bytes[1],rx_bytes[2],rx_bytes[3],rx_bytes[4][7:3]} : {18'b0,rx_bytes[1],rx_bytes[2][7:5]};
 wire [63:0] received_data=rx_bytes[0][7] ?
   {rx_bytes[5],rx_bytes[6],rx_bytes[7],rx_bytes[8],rx_bytes[9],rx_bytes[10],rx_bytes[11],rx_bytes[12]} :
   {rx_bytes[3],rx_bytes[4],rx_bytes[5],rx_bytes[6],rx_bytes[7],rx_bytes[8],rx_bytes[9],rx_bytes[10]};
 wire [31:0] tracking_rdata;
 // Observation only (see tracking_monitor.sv): same transmit event as the timing observer, plus each Type2 feedback frame as stored in the motor table.
 wire feedback_event=state==RX_DONE && rx_bytes[0]==8'h88 && received_id[28:24]==2 && received_id[7:0]==8'hfd && received_id[15:8]>=1 && received_id[15:8]<=7;
 tracking_monitor tracking_observer(.clk(clk),.resetn(resetn),
  .cmd_valid(state==TX_FIRE && $signed(now_us-active_tx[159:128])<0),.cmd_id(active_tx[127:96]),.cmd_hi(active_tx[63:32]),.cmd_lo(active_tx[31:0]),
  .fb_valid(feedback_event),.fb_axis(received_id[10:8]-3'd1),.fb_hi(received_data[63:32]),.fb_lo(received_data[31:0]),.now_us(now_us),
  .access(req_valid && state==IDLE && req_addr>=12'h200 && req_addr<12'h300 && req_addr[1:0]==0 && (!req_write || req_strb==15)),
  .wr(req_write),.addr(req_addr),.wdata(req_data),.rdata(tracking_rdata));
 wire [31:0] received_flags={22'b0,rx_bytes[0][6],rx_bytes[0][7],4'b0,rx_bytes[0][3:0]};
 function [31:0] float_key(input [31:0] v);
 begin float_key=v[31] ? ~v : v^32'h80000000;end
 endfunction
 wire [6:0] fresh_motors;
 genvar fm;
 generate for(fm=0;fm<7;fm=fm+1)begin:freshness
  assign fresh_motors[fm]=motor_seen[fm] && motor_id[fm][21:16]==0 &&
    feedback_timeout_us!=0 && (now_us-motor_time[fm])<feedback_timeout_us;
 end endgenerate
 function pp_allowed(input [31:0] ident,hi,lo,flags,input [6:0] fresh);
 integer n;
 reg [15:0] parameter_id;
 reg [31:0] v;
 begin
  parameter_id={hi[23:16],hi[31:24]};v={lo[7:0],lo[15:8],lo[23:16],lo[31:24]};
  pp_allowed=0;n=ident[7:0]-1;
  if(ident[31:8]==24'h1200fd && ident[7:0]>=1 && ident[7:0]<=7 && flags==8 && hi[15:0]==0 && pp_permit[n] && motion_mask[n])begin
   case(parameter_id)
    16'h7005:pp_allowed=v==(mit_mode[n] ? 0 : csp_mode[n] ? 5 : 1) && !motion_active_mask[n];
    16'h700b:pp_allowed=mit_mode[n] && !motion_active_mask[n] && v>0 && v<=pp_accel[n] && !v[31] && v[30:23]!=255;
    16'h7028:pp_allowed=v>=2000 && v<=20000 && !motion_active_mask[n];
    16'h7016:pp_allowed=!mit_mode[n] && v[30:23]!=255 && pp_min[n][30:23]!=255 && pp_max[n][30:23]!=255 &&
       float_key(v)>=float_key(pp_min[n]) && float_key(v)<=float_key(pp_max[n]) && fresh[n];
    16'h7017:pp_allowed=csp_mode[n] && !motion_active_mask[n] && v>0 && v<=pp_speed[n] && !v[31] && v[30:23]!=255;
    16'h7024:pp_allowed=!mit_mode[n] && !csp_mode[n] && !motion_active_mask[n] && v>0 && v<=pp_speed[n] && !v[31] && v[30:23]!=255;
    16'h7025:pp_allowed=!mit_mode[n] && !csp_mode[n] && !motion_active_mask[n] && v>0 && v<=pp_accel[n] && !v[31] && v[30:23]!=255;
    16'h7018:pp_allowed=!mit_mode[n] && !motion_active_mask[n] && v>0 && v<=pp_current[n] && !v[31] && v[30:23]!=255;
    // Volatile joint3 CSP location gain commissioning only. No enabled motor,
    // fresh disabled feedback, finite positive float in the reviewed 20..30 range.
    16'h701e:pp_allowed=n==2 && csp_mode[n] && motion_active_mask==0 && fresh[n] &&
       motor_id[n][23:22]==0 && v>=32'h41a00000 && v<=32'h41f00000;
    // Volatile J2/J3 tuning, all axes disabled, no Flash command. Raw IEEE
    // bounds also exclude negative values, NaN and Inf. No other axes/gains.
    16'h701f:pp_allowed=(n==1 || n==2) && csp_mode[n] && motion_active_mask==0 && fresh[n] &&
       motor_id[n][23:22]==0 && v>=32'h40800000 && v<=32'h40c00000; // 4..6
    16'h7020:pp_allowed=(n==1 || n==2) && csp_mode[n] && motion_active_mask==0 && fresh[n] &&
       motor_id[n][23:22]==0 && v>=32'h3c23d70a && v<=32'h3ca3d70a; // .01.. .02
    16'h7021:pp_allowed=(n==1 || n==2) && csp_mode[n] && motion_active_mask==0 && fresh[n] &&
       motor_id[n][23:22]==0 && v>=32'h3d4ccccd && v<=32'h3e19999a; // .05.. .15
    default:pp_allowed=0;
   endcase
  end
 end
 endfunction
 reg [31:0] position_bounds[0:6],velocity_bounds[0:6],torque_bounds[0:6],gain_limits[0:6];
 wire tx_expired=tx_pending && $signed(now_us-tx_deadline)>=0;
 reg feedback_expired;
 integer f;
 always @* begin
  feedback_expired=0;
  for(integer n=0;n<7;n=n+1)
   if(motion_active_mask[n] && (!motor_seen[n] || (now_us-motor_time[n])>=feedback_timeout_us || motor_id[n][21:16]!=0))feedback_expired=1;
 end
 function motion_allowed(input [31:0] ident,hi,lo);
 integer n;
 begin
  motion_allowed=0;n=ident[7:0]-1;
  if(ident[7:0]>=1 && ident[7:0]<=7)begin
   motion_allowed=(!pp_permit[n] || (mit_mode[n] && pp_permit[n] && pp_ready[n][4:0]==31)) && motion_mask[n] && motor_seen[n] && motor_id[n][21:16]==0 && feedback_timeout_us!=0 &&
    (now_us-motor_time[n])<feedback_timeout_us &&
    hi[31:16]>=position_bounds[n][15:0] && hi[31:16]<=position_bounds[n][31:16] &&
    hi[15:0]>=velocity_bounds[n][15:0] && hi[15:0]<=velocity_bounds[n][31:16] &&
    ident[23:8]>=torque_bounds[n][15:0] && ident[23:8]<=torque_bounds[n][31:16] &&
    lo[31:16]<=gain_limits[n][15:0] && lo[15:0]<=gain_limits[n][31:16];
  end
 end
 endfunction
 wire staged_motion=stage_id[28:24]==1;
 wire staged_enable=stage_id[28:24]==3;
 wire stage_guard_reject=!control[3] && ((staged_motion &&
    (stage_flags!=8 || !motion_allowed(stage_id,stage_hi,stage_lo))) ||
    (staged_enable && (stage_flags!=8 || stage_hi!=0 || stage_lo!=0 || stage_id[23:8]!=16'hfd ||
      stage_id[7:0]<1 || stage_id[7:0]>7 || !motion_mask[stage_id[7:0]-1] ||
      (!pp_permit[stage_id[7:0]-1] || pp_ready[stage_id[7:0]-1]!=63 || !fresh_motors[stage_id[7:0]-1]))) ||
    (stage_id[28:24]==18 && !pp_allowed(stage_id,stage_hi,stage_lo,stage_flags,fresh_motors)) ||
    stage_id[28:24]==26);


 // The autonomous producer shares the existing deadline FIFO. Host submissions
 // have priority; no simultaneous pushes and no host target overwrite in run.
 wire traj_valid,traj_running,traj_error;
 wire mit_valid,mit_running,mit_error;
 wire [31:0] mit_rdata,mit_fault,mit_claw;
 wire [191:0] mit_packet;
 wire [111:0] mit_position,mit_velocity,mit_torque;
 wire [223:0] mit_gains;
 wire [31:0] traj_rdata,traj_fault;
 wire [191:0] traj_packet;
 wire [223:0] traj_targets;
 reg traj_guard;
 integer tg;
 always @* begin
  traj_guard=motion_active_mask!=0;
  for(integer a=0;a<7;a=a+1)if(motion_active_mask[a])begin
   if(!csp_mode[a] || !pp_permit[a] || !fresh_motors[a] || pp_ready[a]!=63 ||
      traj_targets[a*32+23 +: 8]==255 ||
      float_key(traj_targets[a*32 +: 32])<float_key(pp_min[a]) ||
      float_key(traj_targets[a*32 +: 32])>float_key(pp_max[a]))traj_guard=0;
  end
 end
 reg mit_guard;
 always @* begin
  mit_guard=motion_active_mask==127 && mit_gains[223:192]<=512;
  for(integer a=0;a<7;a=a+1)begin
   if(!pp_permit[a] || !fresh_motors[a] || pp_ready[a]!=63)mit_guard=0;
   if(a<6)begin
    if(!mit_mode[a] ||
      mit_position[a*16 +: 16]<position_bounds[a][15:0] || mit_position[a*16 +: 16]>position_bounds[a][31:16] ||
      mit_velocity[a*16 +: 16]<velocity_bounds[a][15:0] || mit_velocity[a*16 +: 16]>velocity_bounds[a][31:16] ||
      mit_torque[a*16 +: 16]<torque_bounds[a][15:0] || mit_torque[a*16 +: 16]>torque_bounds[a][31:16] ||
      mit_gains[a*32+16 +: 16]>gain_limits[a][15:0] || mit_gains[a*32 +: 16]>gain_limits[a][31:16])mit_guard=0;
   end else if(!csp_mode[a] || mit_mode[a] || mit_claw[30:23]==255 ||
      float_key(mit_claw)<float_key(pp_min[a]) || float_key(mit_claw)>float_key(pp_max[a]))mit_guard=0;
  end
 end
 wire damping_ready;
 wire damping_axis=mit_packet[103:96]==3;
 wire damping_fresh=fresh_motors[2] && motor_id[2][23:22]==2 && (now_us-motor_time[2])<20000;
 wire traj_take=traj_valid && !mit_running && state==IDLE && !req_valid && !tx_full && !tx_push &&
                control[1:0]==3 && !fault && traj_guard;
 wire mit_take=mit_valid && !traj_running && state==IDLE && !req_valid && !tx_full && !tx_push &&
                control[1:0]==3 && !fault && mit_guard && (!damping_axis || damping_ready);
 wire [15:0] damped_torque;
 wire signed [31:0] damping_correction;
 wire [31:0] damping_applied,damping_clipped,damping_stale;
 reg signed [31:0] last_damping_correction;
 filtered_damping damping(.clk(clk),.resetn(resetn),.clear(!control[0] || !mit_running),
  .sample_request(mit_valid && damping_axis),.sample_accept(mit_take && damping_axis),.feedback_fresh(damping_fresh),
  .sample_ready(damping_ready),
  .gain_q8(mit_gains[223:192]),.desired_velocity(mit_packet[47:32]),.measured_velocity(motor_data[2][47:32]),
  .nominal_torque(mit_packet[119:104]),.torque_bounds(torque_bounds[2]),
  .corrected_torque(damped_torque),.correction_codes(damping_correction),
  .applied_count(damping_applied),.clipped_count(damping_clipped),.stale_count(damping_stale));
 wire [191:0] damped_packet=damping_axis ? {mit_packet[191:120],damped_torque,mit_packet[103:0]}:mit_packet;
 wire [191:0] tx_packet=traj_take ? traj_packet : mit_take ? damped_packet :
   {stage_due,stage_expiry,stage_id,stage_flags,stage_hi,stage_lo};
 trajectory_stream stream(.clk(clk),.resetn(resetn),.clear(!control[0]),
  .now_us(now_us),.mask(motion_active_mask),.guard_ok(traj_guard),
  .access(req_valid && state==IDLE && req_addr>=12'h800 && req_addr<12'hb40 && req_addr[1:0]==0 && (!req_write || req_strb==15)),
  .wr(req_write),.addr(req_addr),.wdata(req_data),.rdata(traj_rdata),.error(traj_error),
  .valid(traj_valid),.ready(traj_take),.packet(traj_packet),.targets(traj_targets),
  .running(traj_running),.fault(traj_fault));
 mit_trajectory_stream mit_stream(.clk(clk),.resetn(resetn),.clear(!control[0]),.now_us(now_us),
  .mask(motion_active_mask),.guard_ok(mit_guard),.allow_start(!traj_running),
  .access(req_valid && state==IDLE && req_addr>=12'hb40 && req_addr[1:0]==0 && (!req_write || req_strb==15)),
  .wr(req_write),.addr(req_addr),.wdata(req_data),.rdata(mit_rdata),.error(mit_error),
  .valid(mit_valid),.ready(mit_take),.packet(mit_packet),.encoded_position(mit_position),
  .encoded_velocity(mit_velocity),.encoded_torque(mit_torque),.encoded_gains(mit_gains),.gripper_target(mit_claw),
  .running(mit_running),.fault(mit_fault));


 assign can_tx=resetn && control[0] && !control[3] && !fault && !bus_off && !lease_expired && !tx_expired && !feedback_expired && traj_fault==0 && mit_fault==0 ? core_tx : 1'b1;
 assign debug_us=now_us;
 assign debug_status=core_status;
 frame_fifo #(.WIDTH(192)) txq(clk,resetn,clear_queues,(tx_push || traj_take || mit_take),
   tx_packet,tx_full,tx_pop,tx_head,tx_empty,tx_count);
 frame_fifo #(.WIDTH(160),.LOG_DEPTH(8)) rxq(clk,resetn,clear_queues,rx_push,rx_packet,rx_full,rx_pop,rx_head,rx_empty,rx_count);
 can_top core(.clk_i(clk),.wb_clk_i(clk),.wb_rst_i(!resetn || core_reset),
   .wb_adr_i(wb_addr),.wb_dat_i(wb_out),.wb_dat_o(wb_in),.wb_we_i(wb_write),
   .wb_cyc_i(wb_cycle),.wb_stb_i(wb_cycle),.wb_ack_o(wb_ack),
   .rx_i(control[3] ? core_tx : can_rx),.tx_o(core_tx),.bus_off_on(bus_off_n),.irq_on(irq_n),.clkout_o());
 task start_wb(input wr,input [7:0] a,input [7:0] d,input [3:0] next_state);
 begin wb_write<=wr; wb_addr<=a; wb_out<=d; wb_cycle<=1; state<=WB; after_wb<=next_state; end
 endtask
 function [7:0] tx_byte(input [3:0] n,input [191:0] packet);
 reg [28:0] ident; reg [63:0] payload; reg [31:0] flags;
 begin
 ident=packet[124:96]; flags=packet[95:64]; payload=packet[63:0];
 case(n)
  0:tx_byte={1'b1,1'b0,2'b0,flags[3:0]};
  1:tx_byte=ident[28:21]; 2:tx_byte=ident[20:13];
  3:tx_byte=ident[12:5]; 4:tx_byte={ident[4:0],3'b0};
  default:tx_byte=payload >> ((12-n)*8);
 endcase
 end
 endfunction
 integer i;
 always @(posedge clk) begin
 if(!resetn) begin
  tx_pending<=0;tx_deadline<=0;abort_count<=0;rejected_count<=0;
  motion_mask<=0;motion_active_mask<=0;feedback_timeout_us<=100000;
  pp_permit<=0;csp_mode<=0;mit_mode<=0;last_damping_correction<=0;
  for(f=0;f<7;f=f+1)begin
   pp_ready[f]<=0;pp_min[f]<=32'h7f800000;pp_max[f]<=32'hff800000;
   pp_speed[f]<=0;pp_accel[f]<=0;pp_current[f]<=0;
   position_bounds[f]<=32'h0000ffff;velocity_bounds[f]<=32'h0000ffff;
   torque_bounds[f]<=32'h0000ffff;gain_limits[f]<=0;
  end
  state<=IDLE;after_wb<=IDLE;gap<=0; now_us<=0;divider<=0;heartbeat_us<=0;
  control<=0;watchdog_us<=100000;fault<=0;expired_count<=0;submitted_count<=0;
  received_count<=0;rx_overflow<=0;last_submit_us<=0;last_lateness_us<=0;
  stage_due<=0;stage_expiry<=0;stage_id<=0;stage_flags<=8;stage_hi<=0;stage_lo<=0;
  tx_push<=0;tx_pop<=0;rx_push<=0;rx_pop<=0;clear_queues<=0;
  wb_cycle<=0;wb_write<=0;wb_addr<=0;wb_out<=0;wb_result<=0;core_reset<=0;
  req_done<=0;rsp_data<=0;rsp_error<=0;core_status<=0;active_tx<=0;
  safe_submit_status<=0;safe_submit_sequence<=0;
  index<=0;rx_time<=0;rx_packet<=0;motor_seen<=0;
  for(i=0;i<13;i=i+1)rx_bytes[i]<=0;
  for(i=0;i<7;i=i+1)begin motor_data[i]<=0;motor_id[i]<=0;motor_time[i]<=0;end
 end else begin
  req_done<=0;tx_push<=0;tx_pop<=0;rx_push<=0;rx_pop<=0;clear_queues<=0;core_reset<=0;
  if(mit_take && damping_axis)last_damping_correction<=damping_correction;
  if(divider==99) begin divider<=0;now_us<=now_us+1'b1;end else divider<=divider+1'b1;
  if(lease_expired || tx_expired || feedback_expired || traj_fault!=0 || mit_fault!=0)begin
   tx_pending<=0;motion_active_mask<=0;for(integer j=0;j<7;j=j+1)pp_ready[j]<=0;
   if(tx_expired)abort_count<=abort_count+1'b1;
   control<=0;fault<=lease_expired ? 32'h1 : tx_expired ? 32'h4 : feedback_expired ? 32'h8 : 32'h10;core_reset<=1;clear_queues<=1;wb_cycle<=0;
   state<=HOST_WAIT;req_done<=req_valid;rsp_error<=1;rsp_data<=0;
   if(req_valid && req_write && req_addr==12'h45c && req_strb==15)begin
    rsp_error<=0;safe_submit_status<=3;safe_submit_sequence<=safe_submit_sequence+1'b1;
   end
  end else case(state)
   HOST_WAIT: if(!req_valid)state<=IDLE;
   IDLE: begin
    if(req_valid)begin
     rsp_error<=0;rsp_data<=0;
     if(req_addr[1:0]!=0 || (req_write && req_strb!=4'hf))begin
      rsp_error<=1;req_done<=1;state<=HOST_WAIT;
     end else if(req_addr>=12'h800)begin
      req_done<=1;rsp_data<=req_addr>=12'hb40 ? mit_rdata:traj_rdata;rsp_error<=req_addr>=12'hb40 ? mit_error:traj_error;state<=HOST_WAIT;
     end else if(req_addr>=12'h100 && req_addr<12'h200)begin
      req_done<=1;rsp_data<=timing_rdata;state<=HOST_WAIT;
     end else if(req_addr>=12'h200 && req_addr<12'h300)begin
      req_done<=1;rsp_data<=tracking_rdata;state<=HOST_WAIT;
     end else if(req_addr<12'h100)begin
      if(req_write && control[1])begin rsp_error<=1;req_done<=1;state<=HOST_WAIT;end
      else start_wb(req_write,{2'b0,req_addr[7:2]},req_data[7:0],HOST_WAIT);
     end else begin
      req_done<=1;state<=HOST_WAIT;
      if(req_write)case(req_addr)
       12'h404:begin
        control<=req_data[3:0];heartbeat_us<=now_us;
        if(req_data[3:0]==0)begin core_reset<=1;clear_queues<=1;tx_pending<=0;motion_active_mask<=0;motor_seen<=0;for(integer j=0;j<7;j=j+1)pp_ready[j]<=0;end
       end
       12'h410:heartbeat_us<=now_us;
       12'h414:watchdog_us<=req_data;
       12'h418:if(!control[0])fault<=0;else rsp_error<=1;
       12'h440:stage_due<=req_data;
       12'h444:stage_expiry<=req_data;
       12'h448:stage_id<=req_data;
       12'h44c:stage_flags<=req_data;
       12'h450:stage_hi<=req_data;
       12'h454:stage_lo<=req_data;
       12'h458,12'h45c:begin
        if(req_addr==12'h45c)begin safe_submit_sequence<=safe_submit_sequence+1'b1;safe_submit_status<=0;end
        if(stage_guard_reject || (traj_running && stage_id[28:24]==18) ||
          (mit_running && (stage_id[28:24]==1 || stage_id[28:24]==18)))begin
         rsp_error<=req_addr==12'h458;rejected_count<=rejected_count+1'b1;
         if(req_addr==12'h45c)safe_submit_status<=1;
        end
        else if(tx_full || stage_id[31:29]!=0 || stage_flags>8 ||
           $signed(stage_expiry-stage_due)<=0 || !control[1] ||
           (req_addr==12'h45c && (!control[0] || fault!=0)))begin
         rsp_error<=req_addr==12'h458;
         if(req_addr==12'h45c)safe_submit_status<=2;
        end
        else begin
         tx_push<=1;
         if(staged_motion && !control[3])begin
          if(!mit_mode[stage_id[7:0]-1])motion_active_mask[stage_id[7:0]-1]<=1;
          else pp_ready[stage_id[7:0]-1][5]<=1;
         end
         if(!control[3] && staged_enable)motion_active_mask[stage_id[7:0]-1]<=1;
         if(!control[3] && stage_id[31:8]==24'h0400fd && stage_id[7:0]>=1 && stage_id[7:0]<=7 && stage_flags==8 && stage_hi==0 && stage_lo==0)begin motion_active_mask[stage_id[7:0]-1]<=0;pp_ready[stage_id[7:0]-1]<=0;end
         if(!control[3] && stage_id[31:8]==24'h1200fd)case({stage_hi[23:16],stage_hi[31:24]})
          16'h7005:pp_ready[stage_id[7:0]-1]<=1;
          16'h700b:pp_ready[stage_id[7:0]-1][3:1]<=7;
          16'h7017:pp_ready[stage_id[7:0]-1][2:1]<=3;
          16'h7024:pp_ready[stage_id[7:0]-1][1]<=1;
          16'h7025:pp_ready[stage_id[7:0]-1][2]<=1;
          16'h7018:pp_ready[stage_id[7:0]-1][3]<=1;
          16'h7028:pp_ready[stage_id[7:0]-1][4]<=1;
          16'h7016:pp_ready[stage_id[7:0]-1][5]<=1;
         endcase
        end
       end
       12'h498:if(!rx_empty)rx_pop<=1;else rsp_error<=1;
       12'h5a0:if(control==0)motion_mask<=req_data[6:0];else rsp_error<=1;
       12'h5a4:if(control==0 && req_data>0 && req_data<=1000000)feedback_timeout_us<=req_data;else rsp_error<=1;
       12'h680:if(control==0)begin pp_permit[6]<=req_data[0];pp_ready[6]<=0;end else rsp_error<=1;
       12'h684:if(control==0)pp_min[6]<=req_data;else rsp_error<=1;
       12'h688:if(control==0)pp_max[6]<=req_data;else rsp_error<=1;
       12'h68c:if(control==0)pp_speed[6]<=req_data;else rsp_error<=1;
       12'h690:if(control==0)pp_accel[6]<=req_data;else rsp_error<=1;
       12'h69c:if(control==0 && req_data<=2)begin csp_mode[6]<=req_data==1;mit_mode[6]<=req_data==2;pp_ready[6]<=0;end else rsp_error<=1;
       12'h694:if(control==0)pp_current[6]<=req_data;else rsp_error<=1;
       default:begin
        if(req_addr>=12'h700 && req_addr<12'h7c0 && control==0)case(req_addr[4:2])
         0:begin pp_permit[req_addr[7:5]]<=req_data[0];pp_ready[req_addr[7:5]]<=0;end
         1:pp_min[req_addr[7:5]]<=req_data;
         2:pp_max[req_addr[7:5]]<=req_data;
         3:pp_speed[req_addr[7:5]]<=req_data;
         4:pp_accel[req_addr[7:5]]<=req_data;
         5:pp_current[req_addr[7:5]]<=req_data;
         7:if(req_data<=2)begin csp_mode[req_addr[7:5]]<=req_data==1;mit_mode[req_addr[7:5]]<=req_data==2;pp_ready[req_addr[7:5]]<=0;end else rsp_error<=1;
         default:rsp_error<=1;
        endcase
        else if(req_addr>=12'h600 && req_addr<12'h670 && control==0)case(req_addr[3:2])
         0:position_bounds[req_addr[6:4]]<=req_data;
         1:velocity_bounds[req_addr[6:4]]<=req_data;
         2:torque_bounds[req_addr[6:4]]<=req_data;
         3:gain_limits[req_addr[6:4]]<=req_data;
        endcase
        else rsp_error<=1;
       end
      endcase
      else case(req_addr)
       12'h400:rsp_data<=32'h43414e31;
       12'h404:rsp_data<={28'b0,control};
       12'h408:rsp_data<=now_us;
       12'h40c:rsp_data<={14'b0,bus_off,!irq_n,8'b0,core_status};
       12'h410:rsp_data<=now_us-heartbeat_us;
       12'h414:rsp_data<=watchdog_us;
       12'h418:rsp_data<=fault;
       12'h41c:rsp_data<={7'b0,rx_count,11'b0,tx_count};
       12'h420:rsp_data<=submitted_count;
       12'h424:rsp_data<=expired_count;
       12'h428:rsp_data<=received_count;
       12'h42c:rsp_data<=rx_overflow;
       12'h430:rsp_data<=last_submit_us;
       12'h434:rsp_data<=last_lateness_us;
       12'h480:rsp_data<=rx_empty ? 0:rx_head[127:96];
       12'h484:rsp_data<=rx_empty ? 0:rx_head[95:64];
       12'h488:rsp_data<=rx_empty ? 0:rx_head[63:32];
       12'h48c:rsp_data<=rx_empty ? 0:rx_head[31:0];
       12'h490:rsp_data<=rx_empty ? 0:rx_head[159:128];
       12'h49c:rsp_data<=motor_seen;
       12'h470:rsp_data<=32'h00060003;
       12'h474:rsp_data<=safe_submit_status;
       12'h478:rsp_data<=safe_submit_sequence;
       12'h6c0:rsp_data<=last_damping_correction;
       12'h6c4:rsp_data<=damping_applied;
       12'h6c8:rsp_data<=damping_clipped;
       12'h6cc:rsp_data<=damping_stale;
       12'h6d0:rsp_data<=mit_gains[223:192];
       12'h680:rsp_data<={31'b0,pp_permit[6]};
       12'h684:rsp_data<=pp_min[6];
       12'h688:rsp_data<=pp_max[6];
       12'h68c:rsp_data<=pp_speed[6];
       12'h690:rsp_data<=pp_accel[6];
       12'h694:rsp_data<=pp_current[6];
       12'h69c:rsp_data<=mit_mode[6] ? 2:csp_mode[6] ? 1:0;
       12'h698:rsp_data<={26'b0,pp_ready[6]};
       12'h460:rsp_data<=abort_count;
       12'h464:rsp_data<={31'b0,tx_pending};
       12'h468:rsp_data<=tx_deadline;
       12'h46c:rsp_data<=rejected_count;
       12'h5a0:rsp_data<={25'b0,motion_mask};
       12'h5a4:rsp_data<=feedback_timeout_us;
       12'h5a8:rsp_data<={25'b0,motion_active_mask};
       default:begin
        if(req_addr>=12'h700 && req_addr<12'h7c0)case(req_addr[4:2])
         0:rsp_data<={31'b0,pp_permit[req_addr[7:5]]};
         1:rsp_data<=pp_min[req_addr[7:5]];
         2:rsp_data<=pp_max[req_addr[7:5]];
         3:rsp_data<=pp_speed[req_addr[7:5]];
         4:rsp_data<=pp_accel[req_addr[7:5]];
         5:rsp_data<=pp_current[req_addr[7:5]];
         6:rsp_data<={26'b0,pp_ready[req_addr[7:5]]};
         7:rsp_data<=mit_mode[req_addr[7:5]] ? 2:csp_mode[req_addr[7:5]] ? 1:0;
         default:rsp_error<=1;
        endcase
        else if(req_addr>=12'h500 && req_addr<12'h570)case(req_addr[3:2])
         0:rsp_data<=motor_id[req_addr[6:4]];
         1:rsp_data<=motor_data[req_addr[6:4]][63:32];
         2:rsp_data<=motor_data[req_addr[6:4]][31:0];
         3:rsp_data<=motor_time[req_addr[6:4]];
        endcase
        else if(req_addr>=12'h600 && req_addr<12'h670)case(req_addr[3:2])
         0:rsp_data<=position_bounds[req_addr[6:4]];
         1:rsp_data<=velocity_bounds[req_addr[6:4]];
         2:rsp_data<=torque_bounds[req_addr[6:4]];
         3:rsp_data<=gain_limits[req_addr[6:4]];
        endcase
        else rsp_error<=1;
       end
      endcase
     end
    end else if(control[1] || control[2])start_wb(0,8'd2,0,STATUS);
   end
   WB:if(wb_ack)begin
    wb_result<=wb_in;wb_cycle<=0;gap<=20;state<=GAP;
    if(after_wb==HOST_WAIT)begin req_done<=1;rsp_data<={24'b0,wb_in};rsp_error<=0;end
   end
   GAP:if(gap==0)begin
    // Raw host requests may have completed during the mandatory Wishbone gap.
    if(after_wb==HOST_WAIT)state<=IDLE;else state<=after_wb;
   end else gap<=gap-1'b1;
   STATUS:begin
    core_status<=wb_result;
    if(tx_pending && wb_result[2])tx_pending<=0;
    if(wb_result[7])begin fault<=32'h2;control<=0;clear_queues<=1;tx_pending<=0;motion_active_mask<=0;for(integer j=0;j<7;j=j+1)pp_ready[j]<=0;state<=IDLE;end
    else if(control[2] && wb_result[0])begin index<=0;rx_time<=now_us;state<=RX_READ;end
    else if(control[1] && control[0] && !tx_empty && head_expired)begin
     tx_pop<=1;expired_count<=expired_count+1'b1;state<=IDLE;
    end else if(control[1] && control[0] && !tx_empty && head_due_now && wb_result[2] && !tx_pending)begin
     active_tx<=tx_head;index<=0;state<=TX_FILL;
    end else state<=IDLE;
   end
   TX_FILL:start_wb(1,8'd16+{4'b0,index},tx_byte(index,active_tx),TX_NEXT);
   TX_NEXT:if(index==12)state<=TX_FIRE;else begin index<=index+1'b1;state<=TX_FILL;end
   TX_FIRE:begin
    if($signed(now_us-active_tx[159:128])>=0)begin
     tx_pop<=1;expired_count<=expired_count+1'b1;state<=IDLE;
    end else begin
     last_submit_us<=now_us;last_lateness_us<=now_us-active_tx[191:160];
     start_wb(1,8'd1,control[3] ? 8'h10 : 8'h01,TX_DONE);
    end
   end
   TX_DONE:begin submitted_count<=submitted_count+1'b1;tx_pop<=1;tx_pending<=1;tx_deadline<=active_tx[159:128];state<=IDLE;end
   RX_READ:start_wb(0,8'd16+{4'b0,index},0,RX_NEXT);
   RX_NEXT:begin
    rx_bytes[index]<=wb_result;
    if(index==12)state<=RX_RELEASE;else begin index<=index+1'b1;state<=RX_READ;end
   end
   RX_RELEASE:start_wb(1,8'd1,8'h04,RX_DONE);
   RX_DONE:begin
    received_count<=received_count+1'b1;
    rx_packet<={rx_time,3'b0,received_id,received_flags,received_data};
    if(!rx_full)rx_push<=1;else rx_overflow<=rx_overflow+1'b1;
    if(rx_bytes[0]==8'h88 && received_id[28:24]==2 && received_id[7:0]==8'hfd &&
       received_id[15:8]>=1 && received_id[15:8]<=7)begin
      motor_data[received_id[15:8]-1]<=received_data;
      motor_id[received_id[15:8]-1]<={3'b0,received_id};
      motor_time[received_id[15:8]-1]<=rx_time;
      motor_seen[received_id[15:8]-1]<=1;
    end
    state<=IDLE;
   end
   default:state<=IDLE;
  endcase
 end
 end
endmodule
