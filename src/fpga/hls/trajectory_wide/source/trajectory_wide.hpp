#pragma once
#include <ap_fixed.h>
#include <ap_int.h>
typedef ap_fixed<32,8,AP_RND_CONV,AP_SAT> joint_t;
typedef ap_ufixed<25,1,AP_RND_CONV,AP_SAT> phase_t;
void trajectory_eval_wide(joint_t coefficient[7][8], phase_t phase,
                         joint_t lower[7], joint_t upper[7],
                         joint_t position[7], ap_uint<7>& rejected);
