#pragma once
#include <ap_int.h>

// Wrist-camera pre-processing on the PL: finger-box fill, 11-tap Gaussian (sigma 1.5), Otsu level over a region of interest,
// threshold of the whole frame (dark or bright side), 7x7 close; the packed mask goes back to DDR. Bit-exact with
// contest/vision_pre/model.py. Contours, moments and the screw geometry stay on the ARM.
//
// img:  640x400 8-bit grey, row-major, 64-bit little-endian words (pixel x at bits 8*(x%8)), 32000 words.
// mask: 640x400 bits, bit x%64 of word y*10 + x/64, 4000 words.
constexpr int VP_W = 640, VP_H = 400, VP_TAPS = 11, VP_R = 5;
constexpr int VP_IMG_WORDS = VP_W * VP_H / 8, VP_MASK_WORDS = VP_W * VP_H / 64;

typedef ap_uint<8> u8;
typedef ap_uint<16> u16;
typedef ap_uint<64> word;

// status: 0 ok; 1 region of interest smaller than 12x12 or outside the frame (nothing written)
void vision_pre(const word *img, word *mask,
                u16 roi_x, u16 roi_y, u16 roi_w, u16 roi_h,
                u16 box0_x0, u16 box0_y0, u16 box0_x1, u16 box0_y1,
                u16 box1_x0, u16 box1_y0, u16 box1_x1, u16 box1_y1,
                u8 fill, u8 cap, u8 level_mode, u8 manual_level, u8 polarity, u8 close_en,
                u8 &level, u8 &otsu_level, ap_uint<32> &count, u8 &status);
