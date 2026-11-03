#include "trajectory_wide.hpp"
// Seven independent degree-seven Horner evaluators. Legacy cubic segments
// are represented by zero high-order coefficients; bounds remain atomic.
void trajectory_eval_wide(joint_t coefficient[7][8], phase_t phase,
                         joint_t lower[7], joint_t upper[7],
                         joint_t position[7], ap_uint<7>& rejected) {
#pragma HLS PIPELINE II=1
#pragma HLS ARRAY_PARTITION variable=coefficient complete dim=0
#pragma HLS ARRAY_PARTITION variable=lower complete
#pragma HLS ARRAY_PARTITION variable=upper complete
#pragma HLS ARRAY_PARTITION variable=position complete
    // 40 fractional bits retain far more precision than the Q8.24 output.
    // Keep the multiplier's phase operand at its native 25 bits instead of
    // extending it to wide_t; high-order accuracy does not need a 56x56 MAC.
    typedef ap_fixed<56,16,AP_RND_CONV,AP_SAT> wide_t;
    ap_uint<7> mask=0;
    for(int axis=0;axis<7;++axis) {
#pragma HLS UNROLL
        wide_t y=coefficient[axis][7];
        for(int power=6;power>=0;--power) {
#pragma HLS UNROLL
            y=y*phase+coefficient[axis][power];
        }
        bool bad=phase>phase_t(1) || lower[axis]>upper[axis] ||
                 y<wide_t(lower[axis]) || y>wide_t(upper[axis]);
        mask[axis]=bad;position[axis]=bad ? joint_t(0) : joint_t(y);
    }
    rejected=mask;
}
