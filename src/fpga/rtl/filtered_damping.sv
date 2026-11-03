`timescale 1ns/1ps
// Experimental RS00 joint3 velocity-error damping at its 200 Hz CAN slot.
// LPF alpha=1/4. Extra gain Q8 [0,512] => [0,2] Nm*s/rad.
// Six 100MHz stages precede enqueue; no Linux feedback loop or extra CAN frame.
// Zero gain is bit-exact bypass. Stale feedback removes only this correction.
module filtered_damping(
 input wire clk,resetn,clear,sample_request,sample_accept,feedback_fresh,
 input wire [31:0] gain_q8,
 input wire [15:0] desired_velocity,measured_velocity,nominal_torque,
 input wire [31:0] torque_bounds,
 output wire sample_ready,
 output wire [15:0] corrected_torque,
 output wire signed [31:0] correction_codes,
 output reg [31:0] applied_count,clipped_count,stale_count
);
 reg [2:0] state;
 reg signed [25:0] filtered_q8,candidate_q8;
 reg signed [16:0] error_code;
 reg [9:0] saved_gain;
 reg [15:0] saved_nominal,lower,upper,result;
 reg signed [37:0] product;
 reg signed [48:0] scaled;
 reg signed [24:0] rounded_delta;
 reg signed [31:0] final_correction;
 reg final_clipped;
 reg signed [25:0] error_q8;
 reg signed [17:0] bounded_delta,combined;
 reg clip;
 wire bypass=gain_q8==0 || gain_q8>512 || !feedback_fresh;
 assign sample_ready=bypass || state==6;
 assign corrected_torque=bypass ? nominal_torque:result;
 assign correction_codes=bypass ? 0:final_correction;
 always @* begin
  error_q8=$signed(error_code)*26'sd256;
  bounded_delta=rounded_delta;
  clip=0;
  if(rounded_delta>702)begin bounded_delta=702;clip=1;end
  if(rounded_delta< -702)begin bounded_delta= -702;clip=1;end
  combined=$signed({1'b0,saved_nominal})+bounded_delta;
  if(combined<$signed({1'b0,lower}))begin combined=$signed({1'b0,lower});clip=1;end
  if(combined>$signed({1'b0,upper}))begin combined=$signed({1'b0,upper});clip=1;end
 end
 always @(posedge clk)begin
  if(!resetn)begin
   state<=0;filtered_q8<=0;candidate_q8<=0;error_code<=0;saved_gain<=0;
   saved_nominal<=0;lower<=0;upper<=0;result<=0;product<=0;scaled<=0;
   rounded_delta<=0;final_correction<=0;final_clipped<=0;
   applied_count<=0;clipped_count<=0;stale_count<=0;
  end else begin
   if(clear || bypass)begin state<=0;filtered_q8<=0;end
   else case(state)
    0:if(sample_request)begin
      error_code<=$signed({1'b0,desired_velocity})-$signed({1'b0,measured_velocity});
      saved_gain<=gain_q8[9:0];saved_nominal<=nominal_torque;
      lower<=torque_bounds[15:0];upper<=torque_bounds[31:16];state<=1;
     end
    1:begin candidate_q8<=filtered_q8+((error_q8-filtered_q8)>>>2);state<=2;end
    2:begin product<=$signed(candidate_q8)*$signed({1'b0,saved_gain});state<=3;end
    // 603/256 approximates 66/28 with 0.071% relative scale error,
    // less than one correction code throughout the 0.30Nm output interval.
    3:begin scaled<=product*49'sd603;state<=4;end
    4:begin rounded_delta<=scaled>=0 ? ((scaled+49'sd8388608)>>>24):-(((-scaled)+49'sd8388608)>>>24);state<=5;end
    5:begin result<=combined[15:0];final_correction<=combined-$signed({1'b0,saved_nominal});final_clipped<=clip;state<=6;end
    6:if(sample_accept)begin filtered_q8<=candidate_q8;state<=0;end
   endcase
   if(sample_accept && gain_q8>0 && gain_q8<=512)begin
    if(feedback_fresh)begin applied_count<=applied_count+1;if(final_clipped)clipped_count<=clipped_count+1;end
    else stale_count<=stale_count+1;
   end
  end
 end
endmodule
