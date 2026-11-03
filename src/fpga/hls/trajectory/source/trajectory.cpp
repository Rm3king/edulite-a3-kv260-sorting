#include "trajectory.hpp"
// Seven cubic polynomials evaluated in parallel. Coefficients use normalized
// segment time, not wall-clock seconds. Caller must reject any nonzero mask.
void trajectory_eval(joint_t coefficient[7][4], phase_t phase,
                     joint_t lower[7], joint_t upper[7],
                     joint_t position[7], ap_uint<7>& rejected) {
#pragma HLS PIPELINE II=1
#pragma HLS ARRAY_PARTITION variable=coefficient complete dim=0
#pragma HLS ARRAY_PARTITION variable=lower complete
#pragma HLS ARRAY_PARTITION variable=upper complete
#pragma HLS ARRAY_PARTITION variable=position complete
    typedef ap_fixed<56,16,AP_RND_CONV,AP_SAT> wide_t;
    ap_uint<7> mask=0;
    for(int axis=0;axis<7;++axis) {
#pragma HLS UNROLL
        wide_t t=phase;
        wide_t y=wide_t(coefficient[axis][3])*t+coefficient[axis][2];
        y=y*t+coefficient[axis][1];
        y=y*t+coefficient[axis][0];
        bool invalid=phase>phase_t(1) || lower[axis]>upper[axis] ||
                     y<wide_t(lower[axis]) || y>wide_t(upper[axis]);
        mask[axis]=invalid;
        position[axis]=invalid ? joint_t(0) : joint_t(y);
    }
    rejected=mask;
}
