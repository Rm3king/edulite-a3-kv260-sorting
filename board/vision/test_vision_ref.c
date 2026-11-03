// Host check: vision_ref.c against the model vectors (bit-identical masks, levels and counts).
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
int vision_ref(const uint8_t *, uint8_t *, int, int, int, int, const int *, int, int, int, int, int, int, uint8_t *, int *, int *, uint32_t *);
int main(int argc, char **argv) {
  const char *dir = argc > 1 ? argv[1] : "../vectors";
  char path[512], name[64];
  snprintf(path, sizeof path, "%s/index.txt", dir);
  FILE *index = fopen(path, "r");
  if (!index) return 1;
  static uint8_t img[640 * 400], mask[32000], expected[32000], work[7 * 640 * 400];
  int box[8] = {0, 0, 200, 100, 360, 0, 560, 95};
  int x, y, w, h, mode, manual, pol, close, cap, lref, cref, n = 0, bad = 0;
  while (fscanf(index, "%63s %d %d %d %d %d %d %d %d %d %d %d", name, &x, &y, &w, &h, &mode, &manual, &pol, &close, &cap, &lref, &cref) == 12) {
    snprintf(path, sizeof path, "%s/%s.img", dir, name); FILE *f = fopen(path, "rb"); fread(img, 1, sizeof img, f); fclose(f);
    snprintf(path, sizeof path, "%s/%s.mask", dir, name); f = fopen(path, "rb"); fread(expected, 1, sizeof expected, f); fclose(f);
    int level, ot; uint32_t count;
    int st = vision_ref(img, mask, x, y, w, h, box, 200, cap, mode, manual, pol, close, work, &level, &ot, &count);
    int ok = st == 0 && level == lref && (int)count == cref && memcmp(mask, expected, sizeof mask) == 0;
    if (!ok) { printf("%s FAIL level %d/%d count %u/%d\n", name, level, lref, count, cref); ++bad; }
    ++n;
  }
  printf("%d vectors, %d failures\n", n, bad);
  return bad != 0;
}
