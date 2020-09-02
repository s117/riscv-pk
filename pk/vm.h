#ifndef _VM_H
#define _VM_H

#include "syscall.h"
#include "file.h"
#include <string.h>
#include <stdint.h>
#include <sys/types.h>

#define PROT_NONE 0
#define PROT_READ 1
#define PROT_WRITE 2
#define PROT_EXEC 4

#define MAP_PRIVATE 0x2
#define MAP_FIXED 0x10
#define MAP_ANONYMOUS 0x20
#define MAP_POPULATE 0x8000
#define MREMAP_FIXED 0x2

void vm_init();
int handle_page_fault(uintptr_t vaddr, int prot);
void populate_mapping(const void* start, size_t size, int prot);
uintptr_t __do_mmap(uintptr_t addr, size_t length, int prot, int flags, file_t* file, off_t offset);
uintptr_t do_mmap(uintptr_t addr, size_t length, int prot, int flags, int fd, off_t offset);
int do_munmap(uintptr_t addr, size_t length);
uintptr_t do_mremap(uintptr_t addr, size_t old_size, size_t new_size, int flags);
uintptr_t do_mprotect(uintptr_t addr, size_t length, int prot);
uintptr_t do_brk(uintptr_t addr);

size_t vm_stat_total_phy_pages();
size_t vm_stat_mapped_kernel_virt_pages();
size_t vm_stat_mapped_user_virt_pages();
size_t vm_stat_mapped_virt_pages();
size_t vm_stat_populated_kernel_virt_pages();
size_t vm_stat_populated_user_virt_pages();
size_t vm_stat_populated_virt_pages();

#endif
