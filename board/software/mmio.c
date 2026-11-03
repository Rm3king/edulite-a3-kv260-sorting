#include <stdint.h>
void mmio_write32(void *base, unsigned offset, uint32_t value) {
 __asm__ __volatile__("dmb sy" ::: "memory");
 *(volatile uint32_t *)((char *)base + offset) = value;
 __asm__ __volatile__("dmb sy" ::: "memory");
}
uint32_t mmio_read32(void *base, unsigned offset) {
 __asm__ __volatile__("dmb sy" ::: "memory");
 uint32_t value = *(volatile uint32_t *)((char *)base + offset);
 __asm__ __volatile__("dmb sy" ::: "memory");
 return value;
}
