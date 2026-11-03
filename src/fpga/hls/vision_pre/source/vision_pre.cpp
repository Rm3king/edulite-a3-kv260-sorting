#include "vision_pre.hpp"

typedef ap_uint<12> q4;   // horizontal blur, 4 fractional bits
static const ap_uint<11> COEF[VP_TAPS] = {4, 31, 147, 448, 872, 1092, 872, 448, 147, 31, 4};

struct Boxes {
  u16 x0[2], y0[2], x1[2], y1[2];
  u8 fill;
};

static int reflect(int i, int n) {
#pragma HLS INLINE
  return i < 0 ? -i : (i >= n ? 2 * (n - 1) - i : i);
}

static int wrap11(int v) {   // v in [-11, 21] -> v mod 11
#pragma HLS INLINE
  return v < 0 ? v + VP_TAPS : (v >= VP_TAPS ? v - VP_TAPS : v);
}

// The words covering columns [x0, x0+w) of row y.
static void load_row(const word *img, int y, int x0, int w, word buf[VP_W / 8]) {
  int first = x0 >> 3, last = (x0 + w - 1) >> 3, base = y * (VP_W / 8);
load:
  for (int i = first; i <= last; ++i) {
#pragma HLS PIPELINE II = 1
#pragma HLS LOOP_TRIPCOUNT min = 2 max = 80
    buf[i] = img[base + i];
  }
}

static u8 fetch(const word buf[VP_W / 8], int ax, int ay, const Boxes &b) {
#pragma HLS INLINE
  u8 p = buf[ax >> 3].range(8 * (ax & 7) + 7, 8 * (ax & 7));
  for (int i = 0; i < 2; ++i) {
#pragma HLS UNROLL
    if (ax >= b.x0[i] && ax < b.x1[i] && ay >= b.y0[i] && ay < b.y1[i]) p = b.fill;
  }
  return p;
}

// Vertical tap weight of each ring slot for output row y (rows reflected at 0 and h-1); all zero when y is not a row.
// ym = y mod 11; the eleven rows y-5..y+5 occupy distinct slots. One tap per cycle: eleven cycles per row.
static void vcoef(int y, int ym, int h, ap_uint<13> eff[VP_TAPS]) {
  for (int s = 0; s < VP_TAPS; ++s) {
#pragma HLS UNROLL
    eff[s] = 0;
  }
  if (y < 0 || y >= h) return;
taps:
  for (int t = 0; t < VP_TAPS; ++t) {
#pragma HLS PIPELINE II = 1
    int d = reflect(y + t - VP_R, h) - y;          // -5..5
    eff[wrap11(ym + d)] += COEF[t];
  }
}

static ap_uint<7> popcount64(word v) {
#pragma HLS INLINE
  ap_uint<7> n = 0;
  for (int i = 0; i < 64; ++i) {
#pragma HLS UNROLL
    n += v[i];
  }
  return n;
}

// One datapath for both passes (shared hardware, one pixel per cycle):
//  hist_mode: region of interest (x0, y0, w, h) -> blur -> histogram of the blurred pixels;
//  otherwise: whole frame -> blur -> threshold -> 7x7 close (dilate, then erode; outside pixels neutral) -> packed mask.
// Vertical morphology keeps, per column, the last seven bits in a shift word; horizontal morphology a seven-bit shift register.
// The histogram read-modify-write forwards the last three updates, so equal bins in nearby pixels need no stall.
static void blur_pass(const word *img, word *mask, ap_uint<32> hist[256], bool hist_mode, int x0, int y0, int w, int h,
                      u8 level, bool bright, bool close_en, const Boxes &bx, ap_uint<32> &count) {
  word buf[VP_W / 8];
  q4 ring[VP_TAPS][VP_W];
#pragma HLS ARRAY_PARTITION variable = ring complete dim = 1
  ap_uint<7> dcol[VP_W], ecol[VP_W];
init:   // every ring slot starts at 0: taps of weight 0 then add 0, also in RTL simulation (0 * X is X there)
  for (int x = 0; x < VP_W; ++x) {
#pragma HLS PIPELINE II = 1
    dcol[x] = 0;
    ecol[x] = 0x7f;
    if (x < 256) hist[x] = 0;
    for (int s = 0; s < VP_TAPS; ++s) {
#pragma HLS UNROLL
      ring[s][x] = 0;
    }
  }
  ap_uint<32> cnt = 0;
  int rslot = 0;   // r mod 11
  int rows_total = hist_mode ? h + VP_R : h + 11, cols_total = hist_mode ? w + VP_TAPS - 1 : w + 16;
rows:
  for (int r = 0; r < rows_total; ++r) {
#pragma HLS LOOP_TRIPCOUNT min = 17 max = 411
    if (r < h) load_row(img, y0 + r, x0, w, buf);
    int yb = r - VP_R, yd = yb - 3, ye = yd - 3;
    ap_uint<13> eff[VP_TAPS];
#pragma HLS ARRAY_PARTITION variable = eff complete
    vcoef(yb, wrap11(rslot - VP_R), h, eff);
    u8 win[VP_TAPS];
#pragma HLS ARRAY_PARTITION variable = win complete
    for (int t = 0; t < VP_TAPS; ++t) {
#pragma HLS UNROLL
      win[t] = 0;
    }
    ap_uint<7> dsh = 0, esh = 0x7f;
    word obits = 0;
    u8 f_bin[3] = {0, 0, 0};
    ap_uint<32> f_val[3] = {0, 0, 0};
    bool f_ok[3] = {false, false, false};
#pragma HLS ARRAY_PARTITION variable = f_bin complete
#pragma HLS ARRAY_PARTITION variable = f_val complete
#pragma HLS ARRAY_PARTITION variable = f_ok complete
  cols:
    for (int k = 0; k < cols_total; ++k) {
#pragma HLS PIPELINE II = 1
#pragma HLS LOOP_TRIPCOUNT min = 22 max = 656
#pragma HLS DEPENDENCE variable = dcol inter false
#pragma HLS DEPENDENCE variable = ecol inter false
#pragma HLS DEPENDENCE variable = hist inter false
      // horizontal blur of row r; output column xh = k-10
      int kx = k - VP_R < w + VP_R ? k - VP_R : w - 1;
      u8 p = fetch(buf, x0 + reflect(kx, w), y0 + r, bx);
      for (int t = VP_TAPS - 1; t > 0; --t) {
#pragma HLS UNROLL
        win[t] = win[t - 1];
      }
      win[0] = p;
      int xh = k - (VP_TAPS - 1);
      bool hvalid = xh >= 0 && xh < w;
      ap_uint<20> hs = 0;
      for (int t = 0; t < VP_TAPS; ++t) {
#pragma HLS UNROLL
        hs += COEF[t] * win[t];
      }
      q4 hv = (hs + 128) >> 8;
      int xr = hvalid ? xh : 0;
      ap_uint<26> vs = 0;
      for (int s = 0; s < VP_TAPS; ++s) {
#pragma HLS UNROLL
        q4 v = (r < h && s == rslot) ? hv : ring[s][xr];
        vs += eff[s] * v;
      }
      if (r < h && hvalid) ring[rslot][xh] = hv;
      bool bvalid = hvalid && yb >= 0 && yb < h;
      u8 b = (vs + 32768) >> 16;
      // histogram (hist_mode)
      bool hup = hist_mode && bvalid;
      ap_uint<32> hc = hist[b];
      for (int i = 2; i >= 0; --i) {
#pragma HLS UNROLL
        if (f_ok[i] && f_bin[i] == b) hc = f_val[i];
      }
      hc += 1;
      if (hup) hist[b] = hc;
      for (int i = 2; i > 0; --i) {
#pragma HLS UNROLL
        f_bin[i] = f_bin[i - 1];
        f_val[i] = f_val[i - 1];
        f_ok[i] = f_ok[i - 1];
      }
      f_bin[0] = b;
      f_val[0] = hc;
      f_ok[0] = hup;
      // threshold of blurred row yb (mask mode only below)
      bool t = bright ? (b > level) : (b < level);
      // dilate, vertical: column xh holds rows yb-6..yb -> row yd
      bool dv = false;
      if (!hist_mode && hvalid && yb >= 0 && yb < h + 3) {
        ap_uint<7> c = (dcol[xh] << 1) | ap_uint<7>(bvalid && t);
        dcol[xh] = c;
        dv = c != 0;
      }
      bool dvvalid = hvalid && yd >= 0 && yd < h;
      // dilate, horizontal: output column xd = xh-3
      dsh = (dsh << 1) | ap_uint<7>(dvvalid && dv);
      int xd = xh - 3;
      bool xdvalid = xd >= 0 && xd < w;
      bool dd = dsh != 0;
      // erode, vertical: column xd holds rows yd-6..yd -> row ye
      bool ev = true;
      if (!hist_mode && xdvalid && yd >= 0 && yd < h + 3) {
        ap_uint<7> c = (ecol[xd] << 1) | ap_uint<7>(!(yd < h) || dd);
        ecol[xd] = c;
        ev = c == 0x7f;
      }
      bool evvalid = xdvalid && ye >= 0 && ye < h;
      // erode, horizontal: output column xe = xd-3
      esh = (esh << 1) | ap_uint<7>(!evvalid || ev);
      int xe = xd - 3;
      bool closed = esh == 0x7f;
      int oy = close_en ? ye : yb, ox = close_en ? xe : xh;
      bool ob = close_en ? closed : t;
      if (!hist_mode && ox >= 0 && ox < w && oy >= 0 && oy < h) {
        obits[ox & 63] = ob;
        if ((ox & 63) == 63) {
          mask[oy * (VP_W / 64) + (ox >> 6)] = obits;
          cnt += popcount64(obits);
        }
      }
    }
    rslot = rslot == VP_TAPS - 1 ? 0 : rslot + 1;
  }
  count = cnt;
}

// Otsu as OpenCV computes it: first maximum of (s0*w1 - s1*w0)^2 / (w0*w1), compared exactly by cross-multiplication.
static u8 otsu(const ap_uint<32> hist[256]) {
  ap_uint<32> n = 0, total = 0;
sum:
  for (int i = 0; i < 256; ++i) {
#pragma HLS PIPELINE II = 1
    n += hist[i];
    total += ap_uint<32>(i) * hist[i];
  }
  ap_uint<32> w0 = 0, s0 = 0;
  ap_uint<128> best_num = 0;
  ap_uint<64> best_den = 1;
  bool have = false;
  u8 level = 0;
scan:
  for (int t = 0; t < 256; ++t) {
    w0 += hist[t];
    s0 += ap_uint<32>(t) * hist[t];
    ap_uint<32> w1 = n - w0, s1 = total - s0;
    if (w0 == 0 || w1 == 0) continue;
    ap_int<80> d = ap_int<80>(ap_uint<64>(s0) * w1) - ap_int<80>(ap_uint<64>(s1) * w0);
    ap_uint<80> m = d < 0 ? ap_uint<80>(-d) : ap_uint<80>(d);
    ap_uint<128> num = ap_uint<128>(m) * m;
    ap_uint<64> den = ap_uint<64>(w0) * w1;
    if (!have || num * best_den > best_num * den) {
      have = true;
      best_num = num;
      best_den = den;
      level = t;
    }
  }
  return level;
}

void vision_pre(const word *img, word *mask,
                u16 roi_x, u16 roi_y, u16 roi_w, u16 roi_h,
                u16 box0_x0, u16 box0_y0, u16 box0_x1, u16 box0_y1,
                u16 box1_x0, u16 box1_y0, u16 box1_x1, u16 box1_y1,
                u8 fill, u8 cap, u8 level_mode, u8 manual_level, u8 polarity, u8 close_en,
                u8 &level, u8 &otsu_level, ap_uint<32> &count, u8 &status) {
#pragma HLS INTERFACE m_axi port = img bundle = gmem0 offset = slave depth = 32000 max_read_burst_length = 128 num_read_outstanding = 4 num_write_outstanding = 1 max_write_burst_length = 2
#pragma HLS INTERFACE m_axi port = mask bundle = gmem1 offset = slave depth = 4000 max_write_burst_length = 16 num_write_outstanding = 8 num_read_outstanding = 1 max_read_burst_length = 2
#pragma HLS INTERFACE s_axilite port = img bundle = control
#pragma HLS INTERFACE s_axilite port = mask bundle = control
#pragma HLS INTERFACE s_axilite port = roi_x bundle = control
#pragma HLS INTERFACE s_axilite port = roi_y bundle = control
#pragma HLS INTERFACE s_axilite port = roi_w bundle = control
#pragma HLS INTERFACE s_axilite port = roi_h bundle = control
#pragma HLS INTERFACE s_axilite port = box0_x0 bundle = control
#pragma HLS INTERFACE s_axilite port = box0_y0 bundle = control
#pragma HLS INTERFACE s_axilite port = box0_x1 bundle = control
#pragma HLS INTERFACE s_axilite port = box0_y1 bundle = control
#pragma HLS INTERFACE s_axilite port = box1_x0 bundle = control
#pragma HLS INTERFACE s_axilite port = box1_y0 bundle = control
#pragma HLS INTERFACE s_axilite port = box1_x1 bundle = control
#pragma HLS INTERFACE s_axilite port = box1_y1 bundle = control
#pragma HLS INTERFACE s_axilite port = fill bundle = control
#pragma HLS INTERFACE s_axilite port = cap bundle = control
#pragma HLS INTERFACE s_axilite port = level_mode bundle = control
#pragma HLS INTERFACE s_axilite port = manual_level bundle = control
#pragma HLS INTERFACE s_axilite port = polarity bundle = control
#pragma HLS INTERFACE s_axilite port = close_en bundle = control
#pragma HLS INTERFACE s_axilite port = level bundle = control
#pragma HLS INTERFACE s_axilite port = otsu_level bundle = control
#pragma HLS INTERFACE s_axilite port = count bundle = control
#pragma HLS INTERFACE s_axilite port = status bundle = control
#pragma HLS INTERFACE s_axilite port = return bundle = control
  Boxes bx;
  bx.x0[0] = box0_x0; bx.y0[0] = box0_y0; bx.x1[0] = box0_x1; bx.y1[0] = box0_y1;
  bx.x0[1] = box1_x0; bx.y0[1] = box1_y0; bx.x1[1] = box1_x1; bx.y1[1] = box1_y1;
  bx.fill = fill;
#pragma HLS ALLOCATION function instances = blur_pass limit = 1
  u8 lv = manual_level, ot = 0;
  ap_uint<32> hist[256], c;
  if (level_mode == 0) {
    if (roi_w < 12 || roi_h < 12 || roi_x + roi_w > VP_W || roi_y + roi_h > VP_H) {
      status = 1;
      level = 0;
      otsu_level = 0;
      count = 0;
      return;
    }
    blur_pass(img, mask, hist, true, roi_x, roi_y, roi_w, roi_h, 0, false, false, bx, c);
    ot = otsu(hist);
    lv = ot < cap ? ot : cap;
  }
  blur_pass(img, mask, hist, false, 0, 0, VP_W, VP_H, lv, polarity != 0, close_en != 0, bx, c);
  level = lv;
  otsu_level = ot;
  count = c;
  status = 0;
}
