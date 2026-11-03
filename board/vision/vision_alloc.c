// kv260-vision-alloc SIZE FD: allocate a physically contiguous buffer from the CMA dma-heap for the PL vision kernel.
//
// The dma-buf fd and its bus address go to the caller over the unix socket FD (SCM_RIGHTS + "phys size\n"); the buffer
// lives as long as the caller keeps the fd. Installed root-owned with only cap_dac_override (open the heap) and cap_sys_admin
// (read page frame numbers); it touches nothing else and exits. Refuses when the pages are not contiguous.
#define _GNU_SOURCE
#include <errno.h>
#include <fcntl.h>
#include <linux/dma-heap.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/socket.h>
#include <unistd.h>

static int fail(const char *what) { fprintf(stderr, "kv260-vision-alloc: %s: %s\n", what, strerror(errno)); return 1; }

int main(int argc, char **argv) {
  if (argc != 3) { fprintf(stderr, "usage: kv260-vision-alloc SIZE FD (FD: inherited unix socket)\n"); return 2; }
  int sock = atoi(argv[2]);
  long page = sysconf(_SC_PAGESIZE);
  unsigned long long size = strtoull(argv[1], NULL, 0);
  if (size == 0 || size > (64ull << 20)) { fprintf(stderr, "kv260-vision-alloc: size must be 1..64 MiB\n"); return 2; }
  size = (size + page - 1) / page * page;
  int heap = open("/dev/dma_heap/reserved", O_RDWR | O_CLOEXEC);
  if (heap < 0) return fail("open /dev/dma_heap/reserved");
  struct dma_heap_allocation_data request = {.len = size, .fd_flags = O_RDWR | O_CLOEXEC, .heap_flags = 0};
  if (ioctl(heap, DMA_HEAP_IOCTL_ALLOC, &request) < 0) return fail("allocate");
  close(heap);
  int buf = (int)request.fd;
  volatile uint8_t *map = mmap(NULL, size, PROT_READ | PROT_WRITE, MAP_SHARED, buf, 0);
  if (map == MAP_FAILED) return fail("mmap");
  int pm = open("/proc/self/pagemap", O_RDONLY | O_CLOEXEC);
  if (pm < 0) return fail("open pagemap");
  uint64_t first = 0;
  for (unsigned long long off = 0; off < size; off += page) {
    (void)map[off];   // fault the page in
    uint64_t entry;
    if (pread(pm, &entry, 8, ((uintptr_t)map + off) / page * 8) != 8) return fail("read pagemap");
    uint64_t pfn = entry & ((1ull << 55) - 1);
    if (!(entry >> 63) || pfn == 0) { fprintf(stderr, "kv260-vision-alloc: page frame not visible (capability missing?)\n"); return 1; }
    if (off == 0) first = pfn;
    else if (pfn != first + off / page) { fprintf(stderr, "kv260-vision-alloc: buffer not contiguous\n"); return 1; }
  }
  close(pm);
  munmap((void *)map, size);
  char text[64];
  int len = snprintf(text, sizeof text, "0x%llx %llu\n", (unsigned long long)(first * page), size);
  struct iovec iov = {.iov_base = text, .iov_len = (size_t)len};
  char control[CMSG_SPACE(sizeof(int))];
  memset(control, 0, sizeof control);
  struct msghdr msg = {.msg_iov = &iov, .msg_iovlen = 1, .msg_control = control, .msg_controllen = sizeof control};
  struct cmsghdr *cm = CMSG_FIRSTHDR(&msg);
  cm->cmsg_level = SOL_SOCKET;
  cm->cmsg_type = SCM_RIGHTS;
  cm->cmsg_len = CMSG_LEN(sizeof(int));
  memcpy(CMSG_DATA(cm), &buf, sizeof(int));
  if (sendmsg(sock, &msg, 0) != len) return fail("send to the socket");
  return 0;
}
