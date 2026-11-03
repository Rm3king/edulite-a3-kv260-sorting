#pragma once
#include <ap_int.h>
#include <ap_fixed.h>
// Model joints (URDF radians), never raw motor directions. Coefficients Q8.24.
void fk_batch(const int coefficient[48], const int box[6], unsigned samples,
              int result[32]);
