// The vision_pre algorithm on the ARM (C, one core): the software baseline for the PL comparison and the fallback when the PL
// kernel is absent. Same integer arithmetic as contest/vision_pre/model.py and the HLS kernel: masks are bit-identical.
// Written for speed within plain C: boxes filled once, reflect-101 borders by padding, inner loops the compiler vectorises (-O3).
#include <stdint.h>
#include <string.h>

#define W 640
#define H 400
#define R 5
static const uint32_t COEF[11] = {4, 31, 147, 448, 872, 1092, 872, 448, 147, 31, 4};

static int reflect(int i, int n) { return i < 0 ? -i : (i >= n ? 2 * (n - 1) - i : i); }

// Blur of the (x0, y0, w, h) window of `src` (boxes already filled): Q4 horizontal pass, rounded vertical pass.
static void blur(const uint8_t *src, int x0, int y0, int w, int h, uint16_t *tmp, uint8_t *out) {
  uint8_t pad[W + 2 * R];
  for (int y = 0; y < h; ++y) {
    const uint8_t *row = src + (y0 + y) * W + x0;
    memcpy(pad + R, row, (size_t)w);
    for (int i = 1; i <= R; ++i) { pad[R - i] = row[i]; pad[R + w - 1 + i] = row[w - 1 - i]; }
    uint16_t *t = tmp + y * W;
    for (int x = 0; x < w; ++x) {
      const uint8_t *p = pad + x;
      uint32_t s = COEF[0] * (p[0] + p[10]) + COEF[1] * (p[1] + p[9]) + COEF[2] * (p[2] + p[8]) + COEF[3] * (p[3] + p[7]) +
                   COEF[4] * (p[4] + p[6]) + COEF[5] * p[5];
      t[x] = (uint16_t)((s + 128) >> 8);
    }
  }
  for (int y = 0; y < h; ++y) {
    const uint16_t *r[11];
    for (int t = 0; t < 11; ++t) r[t] = tmp + reflect(y + t - R, h) * W;
    uint8_t *o = out + y * W;
    for (int x = 0; x < w; ++x) {
      uint32_t s = COEF[0] * (r[0][x] + r[10][x]) + COEF[1] * (r[1][x] + r[9][x]) + COEF[2] * (r[2][x] + r[8][x]) +
                   COEF[3] * (r[3][x] + r[7][x]) + COEF[4] * (r[4][x] + r[6][x]) + COEF[5] * r[5][x];
      o[x] = (uint8_t)((s + 32768) >> 16);
    }
  }
}

static int otsu(const uint32_t hist[256]) {
  uint64_t n = 0, total = 0;
  for (int i = 0; i < 256; ++i) { n += hist[i]; total += (uint64_t)i * hist[i]; }
  uint64_t w0 = 0, s0 = 0;
  unsigned __int128 best_num = 0, best_den = 1;
  int have = 0, level = 0;
  for (int t = 0; t < 256; ++t) {
    w0 += hist[t]; s0 += (uint64_t)t * hist[t];
    uint64_t w1 = n - w0, s1 = total - s0;
    if (w0 == 0 || w1 == 0) continue;
    __int128 d = (__int128)(s0 * w1) - (__int128)(s1 * w0);
    unsigned __int128 m = d < 0 ? (unsigned __int128)(-d) : (unsigned __int128)d;
    unsigned __int128 num = m * m, den = (unsigned __int128)w0 * w1;
    if (!have || num * best_den > best_num * den) { have = 1; best_num = num; best_den = den; level = t; }
  }
  return level;
}

// 7x7 dilate (erode = 0) or erode (erode = 1) of a 0/1 image, separable, pixels outside the frame neutral.
static void morph(const uint8_t *in, uint8_t *tmp, uint8_t *out, int erode) {
  uint8_t pad[W + 6];
  uint8_t neutral = (uint8_t)erode;
  for (int y = 0; y < H; ++y) {
    memset(pad, neutral, 3); memset(pad + 3 + W, neutral, 3);
    memcpy(pad + 3, in + y * W, W);
    uint8_t *t = tmp + y * W;
    if (erode) for (int x = 0; x < W; ++x) t[x] = pad[x] & pad[x + 1] & pad[x + 2] & pad[x + 3] & pad[x + 4] & pad[x + 5] & pad[x + 6];
    else for (int x = 0; x < W; ++x) t[x] = pad[x] | pad[x + 1] | pad[x + 2] | pad[x + 3] | pad[x + 4] | pad[x + 5] | pad[x + 6];
  }
  static uint8_t ones[W], zeros[W];
  memset(ones, 1, W);
  for (int y = 0; y < H; ++y) {
    const uint8_t *r[7];
    for (int d = 0; d < 7; ++d) {
      int yy = y + d - 3;
      r[d] = (yy < 0 || yy >= H) ? (erode ? ones : zeros) : tmp + yy * W;
    }
    uint8_t *o = out + y * W;
    if (erode) for (int x = 0; x < W; ++x) o[x] = r[0][x] & r[1][x] & r[2][x] & r[3][x] & r[4][x] & r[5][x] & r[6][x];
    else for (int x = 0; x < W; ++x) o[x] = r[0][x] | r[1][x] | r[2][x] | r[3][x] | r[4][x] | r[5][x] | r[6][x];
  }
}

// Arguments as the PL kernel. mask: 32000 bytes, bit x%8 of byte y*80 + x/8. work: >= 7*W*H bytes. Returns status (1: bad ROI).
int vision_ref(const uint8_t *img, uint8_t *mask, int roi_x, int roi_y, int roi_w, int roi_h, const int box[8], int fill, int cap,
               int level_mode, int manual_level, int polarity, int close_en, uint8_t *work, int *level_out, int *otsu_out,
               uint32_t *count_out) {
  uint16_t *tmp = (uint16_t *)work;                 // 2*W*H bytes
  uint8_t *src = work + 2 * W * H, *blurred = src + W * H, *bits = blurred + W * H, *t1 = bits + W * H, *t2 = t1 + W * H;
  if (level_mode == 0 && (roi_w < 12 || roi_h < 12 || roi_x + roi_w > W || roi_y + roi_h > H)) return 1;
  memcpy(src, img, W * H);
  for (int i = 0; i < 2; ++i) {
    int x0 = box[4 * i], y0 = box[4 * i + 1], x1 = box[4 * i + 2] < W ? box[4 * i + 2] : W, y1 = box[4 * i + 3] < H ? box[4 * i + 3] : H;
    for (int y = y0; y < y1; ++y)
      if (x1 > x0) memset(src + y * W + x0, fill, (size_t)(x1 - x0));
  }
  int level = manual_level, ot = 0;
  if (level_mode == 0) {
    uint32_t hist[256];
    memset(hist, 0, sizeof hist);
    blur(src, roi_x, roi_y, roi_w, roi_h, tmp, blurred);
    for (int y = 0; y < roi_h; ++y)
      for (int x = 0; x < roi_w; ++x) hist[blurred[y * W + x]]++;
    ot = otsu(hist);
    level = ot < cap ? ot : cap;
  }
  blur(src, 0, 0, W, H, tmp, blurred);
  uint8_t lv = (uint8_t)level;
  if (polarity) for (int i = 0; i < W * H; ++i) bits[i] = blurred[i] > lv;
  else for (int i = 0; i < W * H; ++i) bits[i] = blurred[i] < lv;
  if (close_en) {
    morph(bits, t1, t2, 0);
    morph(t2, t1, bits, 1);
  }
  uint32_t count = 0;
  for (int i = 0; i < W * H / 8; ++i) {
    const uint8_t *b = bits + 8 * i;
    uint8_t v = (uint8_t)(b[0] | b[1] << 1 | b[2] << 2 | b[3] << 3 | b[4] << 4 | b[5] << 5 | b[6] << 6 | b[7] << 7);
    mask[i] = v;
    count += (uint32_t)__builtin_popcount(v);
  }
  *level_out = level;
  *otsu_out = ot;
  *count_out = count;
  return 0;
}
