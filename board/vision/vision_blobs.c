// Components of the PL mask, as cv2.findContours(RETR_EXTERNAL) + drawContours(filled) sees them: holes filled (background
// not 4-connected to the border), then 8-connected labelling. Per label: pixels, boundary pixels, sums, box, border contact.
#include <stdint.h>
#include <string.h>

#define W 640
#define H 400

static int32_t find(int32_t *parent, int32_t a) {
  while (parent[a] != a) { parent[a] = parent[parent[a]]; a = parent[a]; }
  return a;
}

// mask: packed (bit x%8 of byte y*80 + x/8). labels: W*H, 0 = background, 1..n. stats: per label 10 int64:
// pixels, boundary, sum_x, sum_y, min_x, max_x, min_y, max_y, touches_border, sum unused. work: W*H*9 bytes.
// Returns n, or -1 when there are more than max_labels components.
int vision_blobs(const uint8_t *mask, int32_t *labels, int max_labels, int64_t *stats, uint8_t *work) {
  uint8_t *fg = work;
  int32_t *queue = (int32_t *)(work + W * H);     // W*H int32 (also reused as union-find parents)
  for (int i = 0; i < W * H; ++i) fg[i] = (mask[i >> 3] >> (i & 7)) & 1;
  // background reachable from the border (4-connected): 2
  int head = 0, tail = 0;
  for (int x = 0; x < W; ++x) {
    int a = x, b = (H - 1) * W + x;
    if (!fg[a]) { fg[a] = 2; queue[tail++] = a; }
    if (!fg[b]) { fg[b] = 2; queue[tail++] = b; }
  }
  for (int y = 1; y < H - 1; ++y) {
    int a = y * W, b = y * W + W - 1;
    if (!fg[a]) { fg[a] = 2; queue[tail++] = a; }
    if (!fg[b]) { fg[b] = 2; queue[tail++] = b; }
  }
  while (head < tail) {
    int p = queue[head++], x = p % W, y = p / W;
    if (x > 0 && !fg[p - 1]) { fg[p - 1] = 2; queue[tail++] = p - 1; }
    if (x < W - 1 && !fg[p + 1]) { fg[p + 1] = 2; queue[tail++] = p + 1; }
    if (y > 0 && !fg[p - W]) { fg[p - W] = 2; queue[tail++] = p - W; }
    if (y < H - 1 && !fg[p + W]) { fg[p + W] = 2; queue[tail++] = p + W; }
  }
  for (int i = 0; i < W * H; ++i) fg[i] = fg[i] != 2;   // holes become foreground
  // 8-connected labelling, union-find over provisional labels
  int32_t *parent = queue;
  int32_t next = 1;
  parent[0] = 0;
  for (int y = 0; y < H; ++y)
    for (int x = 0; x < W; ++x) {
      int p = y * W + x;
      if (!fg[p]) { labels[p] = 0; continue; }
      int32_t n[4], k = 0;
      if (x > 0 && fg[p - 1]) n[k++] = labels[p - 1];
      if (y > 0) {
        if (x > 0 && fg[p - W - 1]) n[k++] = labels[p - W - 1];
        if (fg[p - W]) n[k++] = labels[p - W];
        if (x < W - 1 && fg[p - W + 1]) n[k++] = labels[p - W + 1];
      }
      if (!k) { parent[next] = next; labels[p] = next++; continue; }
      int32_t m = find(parent, n[0]);
      for (int i = 1; i < k; ++i) { int32_t r = find(parent, n[i]); if (r < m) m = r; }
      for (int i = 0; i < k; ++i) { int32_t r = find(parent, n[i]); if (r != m) parent[r] = m; }
      labels[p] = m;
    }
  // compact labels 1..n: flatten every provisional label onto its root, then number the roots
  int32_t *ids = (int32_t *)(work + W * H + 4 * W * H);
  int n = 0;
  for (int32_t i = 1; i < next; ++i) {
    int32_t r = find(parent, i);
    parent[i] = r;
  }
  for (int32_t i = 1; i < next; ++i) ids[i] = parent[i] == i ? ++n : 0;
  if (n > max_labels) return -1;
  memset(stats, 0, sizeof(int64_t) * 10 * (size_t)n);
  for (int i = 0; i < n; ++i) { stats[10 * i + 4] = W; stats[10 * i + 6] = H; stats[10 * i + 5] = -1; stats[10 * i + 7] = -1; }
  for (int p = 0; p < W * H; ++p)
    if (labels[p]) labels[p] = ids[parent[labels[p]]];
  for (int y = 0; y < H; ++y)
    for (int x = 0; x < W; ++x) {
      int p = y * W + x, id = labels[p];
      if (!id) continue;
      int64_t *s = stats + 10 * (id - 1);
      int boundary = x == 0 || y == 0 || x == W - 1 || y == H - 1 || labels[p - 1] != id || labels[p + 1] != id || labels[p - W] != id || labels[p + W] != id;
      s[0] += 1; s[1] += boundary; s[2] += x; s[3] += y;
      if (x < s[4]) s[4] = x;
      if (x > s[5]) s[5] = x;
      if (y < s[6]) s[6] = y;
      if (y > s[7]) s[7] = y;
      if (x == 0 || y == 0 || x == W - 1 || y == H - 1) s[8] = 1;
    }
  return n;
}
