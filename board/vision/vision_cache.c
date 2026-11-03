// User-space cache maintenance for the cacheable dma-buf mapping shared with the PL (the HP port is not coherent).
// Linux arm64 sets SCTLR_EL1.UCI/UCT, so EL0 may run DC CVAC / DC CIVAC and read CTR_EL0.
#include <stddef.h>
#include <stdint.h>

static size_t line_size(void) {
  uint64_t ctr;
  __asm__ volatile("mrs %0, ctr_el0" : "=r"(ctr));
  return (size_t)4 << ((ctr >> 16) & 0xf);   // DminLine, log2 of words
}

// CPU wrote [p, p+n): push it to memory before the PL reads it.
void vp_clean(void *p, size_t n) {
  size_t line = line_size();
  uintptr_t a = (uintptr_t)p & ~(uintptr_t)(line - 1), end = (uintptr_t)p + n;
  for (; a < end; a += line) __asm__ volatile("dc cvac, %0" ::"r"(a) : "memory");
  __asm__ volatile("dsb sy" ::: "memory");
}

// The PL writes [p, p+n): drop any cached copy (call before starting and again after it finished).
void vp_invalidate(void *p, size_t n) {
  size_t line = line_size();
  uintptr_t a = (uintptr_t)p & ~(uintptr_t)(line - 1), end = (uintptr_t)p + n;
  for (; a < end; a += line) __asm__ volatile("dc civac, %0" ::"r"(a) : "memory");
  __asm__ volatile("dsb sy" ::: "memory");
}
