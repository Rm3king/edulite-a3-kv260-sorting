// C simulation / co-simulation bench: real wrist frames and expected masks from model.py (--vectors).
// VP_VECTORS: vector directory (default: next to this file); VP_MAX_VECTORS limits the count and VP_ONLY picks names
// (co-simulation is slow).
#include "../source/vision_pre.hpp"
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

static bool read_file(const std::string &path, std::vector<unsigned char> &out, size_t size) {
  FILE *f = fopen(path.c_str(), "rb");
  if (!f) return false;
  out.resize(size);
  size_t n = fread(out.data(), 1, size, f);
  fclose(f);
  return n == size;
}

int main() {
  const char *env = getenv("VP_VECTORS");
  std::string dir = env ? env : "/opt/edulite/project/contest/vision_pre/vectors";
  int max_vectors = getenv("VP_MAX_VECTORS") ? atoi(getenv("VP_MAX_VECTORS")) : 1000;
  const char *only = getenv("VP_ONLY");   // comma-separated names: run just these (co-simulation)
  FILE *index = fopen((dir + "/index.txt").c_str(), "r");
  if (!index) { printf("no vectors in %s\n", dir.c_str()); return 1; }
  static word img[VP_IMG_WORDS], mask[VP_MASK_WORDS];
  char name[64];
  int x, y, w, h, mode, manual, pol, close, cap, level_ref, count_ref, n = 0, failures = 0;
  while (n < max_vectors && fscanf(index, "%63s %d %d %d %d %d %d %d %d %d %d %d", name, &x, &y, &w, &h, &mode, &manual, &pol, &close, &cap, &level_ref, &count_ref) == 12) {
    if (only && !strstr(only, name)) continue;
    std::vector<unsigned char> raw, expected;
    if (!read_file(dir + "/" + name + ".img", raw, VP_W * VP_H) || !read_file(dir + "/" + name + ".mask", expected, VP_W * VP_H / 8)) {
      printf("%s: missing files\n", name); return 1;
    }
    for (int i = 0; i < VP_IMG_WORDS; ++i) {
      word v = 0;
      for (int b = 0; b < 8; ++b) v.range(8 * b + 7, 8 * b) = raw[8 * i + b];
      img[i] = v;
    }
    for (int i = 0; i < VP_MASK_WORDS; ++i) mask[i] = 0xdeadbeefdeadbeefULL;
    u8 level, otsu_level, status;
    ap_uint<32> count;
    vision_pre(img, mask, x, y, w, h, 0, 0, 200, 100, 360, 0, 560, 95, 200, cap, mode, manual, pol, close, level, otsu_level, count, status);
    int bad = 0;
    for (int i = 0; i < VP_MASK_WORDS; ++i)
      for (int b = 0; b < 8; ++b)
        if ((unsigned char)mask[i].range(8 * b + 7, 8 * b) != expected[8 * i + b]) ++bad;
    bool ok = status == 0 && level == level_ref && (int)count == count_ref && bad == 0;
    printf("%s roi=(%d,%d,%d,%d) mode=%d pol=%d close=%d cap=%d level=%d/%d count=%d/%d bad_bytes=%d %s\n", name, x, y, w, h, mode, pol, close, cap,
           (int)level, level_ref, (int)count, count_ref, bad, ok ? "OK" : "FAIL");
    failures += !ok;
    ++n;
  }
  fclose(index);
  // A region of interest that does not fit is refused and leaves the mask untouched.
  u8 level, otsu_level, status;
  ap_uint<32> count;
  mask[0] = 0x1234;
  vision_pre(img, mask, 600, 0, 100, 50, 0, 0, 200, 100, 360, 0, 560, 95, 200, 60, 0, 0, 0, 1, level, otsu_level, count, status);
  if (status != 1 || mask[0] != 0x1234) { printf("bad ROI not refused\n"); ++failures; }
  printf("%d vectors, %d failures\n", n, failures);
  if (failures == 0) printf("TEST_PASS vision_pre\n");
  return failures ? 1 : 0;
}
