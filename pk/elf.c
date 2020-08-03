// See LICENSE for license details.

#include "file.h"
#include "pk.h"
#include "vm.h"
#include <sys/stat.h>
#include <fcntl.h>
#include <elf.h>
#include <string.h>

/**
 * The protection flags are in the p_flags section of the program header.
 * But rather annoyingly, they are the reverse of what mmap expects.
 */
static inline int get_prot(uint32_t p_flags)
{
  int prot_x = (p_flags & PF_X) ? PROT_EXEC  : PROT_NONE;
  int prot_w = (p_flags & PF_W) ? PROT_WRITE : PROT_NONE;
  int prot_r = (p_flags & PF_R) ? PROT_READ  : PROT_NONE;

  return (prot_x | prot_w | prot_r);
}

void load_elf(const char* fn, elf_info* info)
{
  file_t* file = file_open(fn, O_RDONLY, 0);
  if (IS_ERR_VALUE(file))
    goto fail;

  Elf64_Ehdr eh64;
  ssize_t ehdr_size = file_pread(file, &eh64, sizeof(eh64), 0);
  if (ehdr_size < (ssize_t)sizeof(eh64) ||
      !(eh64.e_ident[0] == '\177' && eh64.e_ident[1] == 'E' &&
        eh64.e_ident[2] == 'L'    && eh64.e_ident[3] == 'F'))
    goto fail;

  size_t bias = 0;
  extern char _end;
  if (eh64.e_type == ET_DYN)
    bias = ROUNDUP((uintptr_t)&_end, RISCV_PGSIZE);

  #define LOAD_ELF do { \
    eh = (typeof(eh))&eh64; \
    size_t phdr_size = eh->e_phnum*sizeof(*ph); \
    if (info->phdr_top - phdr_size < info->stack_bottom) \
      goto fail; \
    info->phdr = info->phdr_top - phdr_size; \
    ssize_t ret = file_pread(file, (void*)info->phdr, phdr_size, eh->e_phoff); \
    if (ret < (ssize_t)phdr_size) goto fail; \
    info->entry = bias + eh->e_entry; \
    info->phnum = eh->e_phnum; \
    info->phent = sizeof(*ph); \
    ph = (typeof(ph))info->phdr; \
    int flags = MAP_FIXED | MAP_PRIVATE; \
    for (int i = eh->e_phnum - 1; i >= 0; i--) { \
      if(ph[i].p_type == PT_INTERP) { \
        panic("not a statically linked ELF program"); \
      } \
      if(ph[i].p_type == PT_LOAD && ph[i].p_memsz) { \
        uintptr_t prepad = ph[i].p_vaddr % RISCV_PGSIZE; \
        uintptr_t vaddr = ph[i].p_vaddr + bias; \
        if (vaddr + ph[i].p_memsz > info->brk_min) \
          info->brk_min = vaddr + ph[i].p_memsz; \
        int flags2 = flags | (prepad ? MAP_POPULATE : 0); \
        int prot = get_prot(ph[i].p_flags); \
        if (__do_mmap(vaddr - prepad, ph[i].p_filesz + prepad, prot | PROT_WRITE, flags2, file, ph[i].p_offset - prepad) != vaddr - prepad) \
          goto fail; \
        memset((void*)vaddr - prepad, 0, prepad); \
        if (!(prot & PROT_WRITE)) \
          if (do_mprotect(vaddr - prepad, ph[i].p_filesz + prepad, prot)) \
            goto fail; \
        size_t mapped = ROUNDUP(ph[i].p_filesz + prepad, RISCV_PGSIZE) - prepad; \
        if (ph[i].p_memsz > mapped) \
          if (__do_mmap(vaddr + mapped, ph[i].p_memsz - mapped, prot, flags|MAP_ANONYMOUS, 0, 0) != vaddr + mapped) \
            goto fail; \
      } \
    } \
  } while(0)

  info->elf64 = IS_ELF64(eh64);
  if (info->elf64)
  {
    Elf64_Ehdr* eh;
    Elf64_Phdr* ph;
    LOAD_ELF;
  }
  else if (IS_ELF32(eh64))
  {
    Elf32_Ehdr* eh;
    Elf32_Phdr* ph;
    LOAD_ELF;
  }
  else
    goto fail;

  file_decref(file);
  return;

fail:
    panic("couldn't open ELF program: %s!", fn);
}
