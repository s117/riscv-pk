#include "vm.h"
#include "file.h"
#include "atomic.h"
#include "pk.h"
#include <stdint.h>
#include <errno.h>

typedef struct {
  uintptr_t addr;
  size_t length;
  file_t* file;
  size_t offset;
  size_t refcnt;
  int prot;
} vmr_t;

#define MAX_VMR 32
spinlock_t vm_lock = SPINLOCK_INIT;
static vmr_t vmrs[MAX_VMR];

typedef uintptr_t pte_t;
static pte_t* root_page_table;
static uintptr_t first_free_page;
static size_t next_free_page;
static size_t free_pages;

// This function is for 'page table page' allocation only
static uintptr_t __page_alloc()
{
  if (next_free_page == free_pages)
    return 0;
  uintptr_t addr = first_free_page + RISCV_PGSIZE * next_free_page++;
  memset((void*)addr, 0, RISCV_PGSIZE);
  return addr;
}

static vmr_t* __vmr_alloc(uintptr_t addr, size_t length, file_t* file,
                          size_t offset, size_t refcnt, int prot)
{
  for (vmr_t* v = vmrs; v < vmrs + MAX_VMR; v++)
  {
    if (v->refcnt == 0)
    {
      v->addr = addr;
      v->length = length;
      v->file = file;
      v->offset = offset;
      v->refcnt = refcnt;
      v->prot = prot;
      return v;
    }
  }
  return NULL;
}

static void __vmr_decref(vmr_t* v, size_t dec)
{
  if ((v->refcnt -= dec) == 0)
  {
    if (v->file)
      file_decref(v->file);
  }
}

static size_t pte_ppn(pte_t pte)
{
  return pte >> RISCV_PGSHIFT;
}

static pte_t ptd_create(uintptr_t ppn)
{
  return ppn << RISCV_PGSHIFT | PTE_T | PTE_V;
}

static uintptr_t ppn(uintptr_t addr)
{
  return addr >> RISCV_PGSHIFT;
}

static size_t pt_idx(uintptr_t addr, int level)
{
  size_t idx = addr >> (RISCV_PGLEVEL_BITS*level + RISCV_PGSHIFT);
  return idx & ((1 << RISCV_PGLEVEL_BITS) - 1);
}

static pte_t super_pte_create(uintptr_t ppn, int kprot, int uprot, int level)
{
  kprot &= (PROT_READ | PROT_WRITE | PROT_EXEC);
  uprot &= (PROT_READ | PROT_WRITE | PROT_EXEC);
  int perm = (kprot * PTE_SR) | (uprot * PTE_UR) | PTE_V;
  return (ppn << (RISCV_PGLEVEL_BITS*level + RISCV_PGSHIFT)) | perm;
}

static pte_t pte_create(uintptr_t ppn, int kprot, int uprot)
{
  return super_pte_create(ppn, kprot, uprot, 0);
}

static __attribute__((always_inline)) pte_t* __walk_internal(uintptr_t addr, int create)
{
  const size_t pte_per_page = RISCV_PGSIZE/sizeof(void*);
  pte_t* t = root_page_table;

  for (unsigned i = RISCV_PGLEVELS-1; i > 0; i--)
  {
    size_t idx = pt_idx(addr, i);
    if (!(t[idx] & PTE_V))
    {
      if (!create)
        return 0;
      // If no PTE for the vaddr and a new one is needed to be created, allocate one ppage from the reserved ppages
      uintptr_t page = __page_alloc();
      if (page == 0)
        return 0;
      t[idx] = ptd_create(ppn(page));
    }
    else
      kassert(t[idx] & PTE_T); // PK doesn't create 'megapage', or Super PTE, so only the PTE in the last level table could be a leaf PTE
    t = (pte_t*)(pte_ppn(t[idx]) << RISCV_PGSHIFT); // goto next level table
  }
  return &t[pt_idx(addr, 0)]; // return the pointer of the PTE in the last level table
}

// Walk the PT for a vaddr, it it doesn't have a PTE returns NULL
static pte_t* __walk(uintptr_t addr)
{
  return __walk_internal(addr, 0);
}

// Walk the PT for a vaddr, it it doesn't have a PTE creates one
static pte_t* __walk_create(uintptr_t addr)
{
  return __walk_internal(addr, 1);
}

// Check whether vaddr in a unmapped vpage
static int __va_avail(uintptr_t vaddr)
{
  pte_t* pte = __walk(vaddr);
  return pte == 0 || *pte == 0;
}

// In virtual address space, find the first continous area after the current.brk which can be used
// for mapping the new npages, if no space left return 0
static uintptr_t __vm_alloc(size_t npage)
{
  uintptr_t start = current.brk, end = current.mmap_max - npage*RISCV_PGSIZE;
  // Only need to check between start ~ end, because if the continous area is not started
  // within this range, the remaining vm space will not be enough to map n pages
  for (uintptr_t a = start; a <= end; a += RISCV_PGSIZE)
  {
    if (!__va_avail(a)) // Skip if the address 'a' is already mapped
      continue;
    uintptr_t first = a, last = a + (npage-1) * RISCV_PGSIZE;
    // Make sure first ~ last is a continous unmapped vm space
    for (a = last; a > first && __va_avail(a); a -= RISCV_PGSIZE)
      ;
    if (a > first)
      continue;
    return a;
  }
  return 0;
}

static void flush_tlb()
{
  write_csr(fatc, 0);
}

static int __handle_page_fault(uintptr_t vaddr, int prot)
{
  uintptr_t vpn = vaddr >> RISCV_PGSHIFT;
  vaddr = vpn << RISCV_PGSHIFT; // operate at vpage level

  pte_t* pte = __walk(vaddr);

  if (pte == 0 || *pte == 0) // If the vpage is not mapped, fail
    return -1;
  else if (!(*pte & PTE_V)) // The vpage is mapped, but need to be populated
  {
    kassert(vaddr < current.stack_top && vaddr >= current.user_min);
    // PK doesn't do the physial page management
    // It simply populates the vpage to the ppage that has the same page number (PPN==VPN)
    uintptr_t ppn = vpn;
  
    // For a PTE that is not NULL and valid, it is a pointer to the VMR ( set in __do_mmap() )
    vmr_t* v = (vmr_t*)*pte;

    // To initialize the target vpage, give it a temporary permission (RW to Kernel, No access to User)
    *pte = pte_create(ppn, PROT_READ|PROT_WRITE, 0);
    flush_tlb(); // Apply the temporary permission
    if (v->file)
    { // If the vpage is mapped to a file, initializes the page with the file data via HTIF syscall, pads zero if not filled up
      size_t flen = MIN(RISCV_PGSIZE, v->length - (vaddr - v->addr));
      ssize_t ret = file_pread(v->file, (void*)vaddr, flen, vaddr - v->addr + v->offset);
      kassert(ret > 0);
      if (ret < RISCV_PGSIZE)
        memset((void*)vaddr + ret, 0, RISCV_PGSIZE - ret);
    }
    else // If not a file mapped vpage, zero out the newly populated page
      memset((void*)vaddr, 0, RISCV_PGSIZE);
    // After populated a vpage, decrease the refcnt in its VMR
    __vmr_decref(v, 1);
    
    // Overwrite the vpage's temporary permission with the acutal permission in VMR
    // (The permission for Kernel and User is set to same)
    *pte = pte_create(ppn, v->prot, v->prot); 
  }

  // check whether the newly populated vpage has covers all permission required in 'prot'
  pte_t perms = pte_create(0, prot, prot);
  if ((*pte & perms) != perms)
    return -1;

  flush_tlb(); // apply the new page table
  return 0;
}

int handle_page_fault(uintptr_t vaddr, int prot)
{
  spinlock_lock(&vm_lock);
    int ret = __handle_page_fault(vaddr, prot);
  spinlock_unlock(&vm_lock);
  return ret;
}

// Takedown the vpage mapping from addr ~ addr+len-1
static void __do_munmap(uintptr_t addr, size_t len)
{
  for (uintptr_t a = addr; a < addr + len; a += RISCV_PGSIZE)
  {
    pte_t* pte = __walk(a);
    if (pte == 0 || *pte == 0)
      continue;
    // Decrease the refcnt for mapped but unpopulated vpage
    if (!(*pte & PTE_V))
      __vmr_decref((vmr_t*)*pte, 1);

    *pte = 0;
  }
  flush_tlb(); // TODO: shootdown
}

// Establish the vpage mapping
uintptr_t __do_mmap(uintptr_t addr, size_t length, int prot, int flags, file_t* f, off_t offset)
{
  size_t npage = (length-1)/RISCV_PGSIZE+1; // Number of pages required by length
  if (flags & MAP_FIXED) // MAP_FIXED means whether the request is to map the exact vaddr from addr ~ addr+length-1
  {
    if ((addr & (RISCV_PGSIZE-1)) || addr < current.user_min ||
        addr + length > current.stack_top || addr + length < addr)
      return (uintptr_t)-1; // Three requirement for a FIXED mapping request: 1. 'addr' aligns to page boundary; 2. mapping region is in user_min ~ stack_top; 3. addr+length not overflow
  }
  else if ((addr = __vm_alloc(npage)) == 0) // If not a FIXED request, after the 'break' of user space, find the nearest unmapped continous n vpages
    return (uintptr_t)-1;

  /*
   * Allocate a VMR to record this new map, the VMR.refcnt is set to the number of vpages use this new VMR
   * (VMRs is static allocated an the total is defined by the macro 'MAX_VMR')
   *
   * For any non-NULL PTE that PTE.V == 0,  it is a pointer to a VMR
   *
   * A VMR tells the kernel how to map a vpage that is not valid yet
   *
   * So for a vpage, its PTE falls into one of these four cases:
   *
   *  1. Valid PTE                  (Populated)
   *  2. Invalid PTE but not NULL   (Mapped, not populated)
   *  3. Invalid and NULL PTE       (Unmapped)
   *  4. No PTE                     (Unmapped)
   */  

  vmr_t* v = __vmr_alloc(addr, length, f, offset, npage, prot);
  if (!v)
    return (uintptr_t)-1;
  
  // Create PTE for the vpages, store the VMR pointer to the PTE
  for (uintptr_t a = addr; a < addr + length; a += RISCV_PGSIZE)
  {
    pte_t* pte = __walk_create(a);
    kassert(pte);

    if (*pte) // For the FIXED request, unmap the conflict old mapping
      __do_munmap(a, RISCV_PGSIZE);

    *pte = (pte_t)v;
  }

  if (f) file_incref(f);

  if (!have_vm || (flags & MAP_POPULATE)) // Populate the vpages immediately (assign it a ppages) if requested
    for (uintptr_t a = addr; a < addr + length; a += RISCV_PGSIZE)
      kassert(__handle_page_fault(a, prot) == 0);

  return addr;
}

// syscall interface
int do_munmap(uintptr_t addr, size_t length)
{
  if ((addr & (RISCV_PGSIZE-1)) || addr < current.user_min ||
      addr + length > current.stack_top || addr + length < addr)
    return -EINVAL;

  spinlock_lock(&vm_lock);
    __do_munmap(addr, length);
  spinlock_unlock(&vm_lock);

  return 0;
}

// syscall interface
uintptr_t do_mmap(uintptr_t addr, size_t length, int prot, int flags, int fd, off_t offset)
{
  // Only support PRIVATE mapping, which implies updates to the mapping are not carried through to the underlying file.
  if (!(flags & MAP_PRIVATE) || length == 0 || (offset & (RISCV_PGSIZE-1)))
    return -EINVAL;

  file_t* f = NULL;
  // MAP_ANONYMOUS: The mapping is not backed by any file; its contents are initialized to zero.
  if (!(flags & MAP_ANONYMOUS) && (f = file_get(fd)) == NULL)
    return -EBADF;

  spinlock_lock(&vm_lock);
    addr = __do_mmap(addr, length, prot, flags, f, offset);
    if (addr < current.brk_max)
      current.brk_max = addr; // Prevents the 'brk' line go beyond an VM region created by sys_mmap() syscall
  spinlock_unlock(&vm_lock);

  if (f) file_decref(f);
  return addr;
}

// Move the 'brk' line around
uintptr_t __do_brk(size_t addr)
{
  uintptr_t newbrk = addr;
  // The brk region is clamped to current.brk_min ~ curent.brk_max
  if (addr < current.brk_min)
    newbrk = current.brk_min;
  else if (addr > current.brk_max)
    newbrk = current.brk_max;

  if (current.brk == 0)
    current.brk = ROUNDUP(current.brk_min, RISCV_PGSIZE);

  uintptr_t newbrk_page = ROUNDUP(newbrk, RISCV_PGSIZE);
  if (current.brk > newbrk_page) // shrink the brk line by unmapping the leftover
    __do_munmap(newbrk_page, current.brk - newbrk_page);
  else if (current.brk < newbrk_page) // expand the brk line by creating new mapping after the current.brk
    kassert(__do_mmap(current.brk, newbrk_page - current.brk, -1, MAP_FIXED|MAP_PRIVATE|MAP_ANONYMOUS, 0, 0) == current.brk);
  current.brk = newbrk_page;

  return newbrk;
}

// syscall interface
uintptr_t do_brk(size_t addr)
{
  spinlock_lock(&vm_lock);
    addr = __do_brk(addr);
  spinlock_unlock(&vm_lock);
  
  return addr;
}

// syscall interface, resize a mmap region
uintptr_t do_mremap(uintptr_t addr, size_t old_size, size_t new_size, int flags)
{
  uintptr_t res = -1;
  // All address and length must be aligned to page boundary
  // And MREMAP_FIXED is not supported (If  this  flag  is specified, mremap() should accept a fifth  argument,  void *new_address,  which  specifies  a  page-aligned  address to which the mapping must be moved.)
  if (((addr | old_size | new_size) & (RISCV_PGSIZE-1)) ||
      (flags & MREMAP_FIXED))
    return -EINVAL;

  spinlock_lock(&vm_lock);
    for (size_t i = 0; i < MAX_VMR; i++)
    {
      if (vmrs[i].refcnt && addr == vmrs[i].addr && old_size == vmrs[i].length)
      {
        size_t old_npage = (vmrs[i].length-1)/RISCV_PGSIZE+1;
        size_t new_npage = (new_size-1)/RISCV_PGSIZE+1;
        if (new_size < old_size)
          __do_munmap(addr + new_size, old_size - new_size);
        else if (new_size > old_size)
          __do_mmap(addr + old_size, new_size - old_size, vmrs[i].prot, 0,
                    vmrs[i].file, vmrs[i].offset + new_size - old_size);
        __vmr_decref(&vmrs[i], old_npage - new_npage);
        // don't need to update the vmrs[i].length here?
        res = addr;
      }
    }
  spinlock_unlock(&vm_lock);
 
  return res;
}

// syscall interface, for reducing the user space permission on a mmap region
uintptr_t do_mprotect(uintptr_t addr, size_t length, int prot)
{
  uintptr_t res = 0;
  // addr muse be align to page boundary
  if ((addr) & (RISCV_PGSIZE-1))
    return -EINVAL;

  spinlock_lock(&vm_lock);
    for (uintptr_t a = addr; a < addr + length; a += RISCV_PGSIZE)
    {
      pte_t* pte = __walk(a);
      if (pte == 0 || *pte == 0) {
        res = -ENOMEM;
        break;
      }
  
      if(!(*pte & PTE_V)){
        vmr_t* v = (vmr_t*)*pte;
        if((v->prot ^ prot) & ~v->prot){ // reduce permission only
          //TODO:look at file to find perms
          res = -EACCES;
          break;
        }
        v->prot = prot;
      }else{
        pte_t perms = pte_create(0, 0, prot);
        if ((*pte & perms) != perms){ // reduce permission only
          //TODO:look at file to find perms
          res = -EACCES;
          break;
        }
        pte_t permset = (*pte & ~(PTE_UR | PTE_UW | PTE_UX)) | perms; //only update the user permission
        *pte = permset;
      }
    }
  spinlock_unlock(&vm_lock);
 
  return res;
}

// Create a verbatim vpage to ppage mapping at paddr ~ paddr+len-1 for kernel, with no user space access permission
static void __map_kernel_range(uintptr_t paddr, size_t len, int prot)
{
  pte_t perms = pte_create(0, prot, 0);
  for (uintptr_t a = paddr; a < paddr + len; a += RISCV_PGSIZE)
  {
    pte_t* pte = __walk_create(a);
    kassert(pte);
    *pte = a | perms;
  }
}

// populate the established mapping by triggering page fault exception
// (using atomic_add/atomic_read for RW/RO mapping)
void populate_mapping(const void* start, size_t size, int prot)
{
  uintptr_t a0 = ROUNDDOWN((uintptr_t)start, RISCV_PGSIZE);
  for (uintptr_t a = a0; a < (uintptr_t)start+size; a += RISCV_PGSIZE)
  {
    atomic_t* atom = (atomic_t*)(a & -sizeof(atomic_t));
    if (prot & PROT_WRITE)
      atomic_add(atom, 0);
    else
      atomic_read(atom);
  }
}

void vm_init()
{
  // VM is still disabled now, no page table setup, MMU running in physical address mode
  extern char _end;
  current.user_min = ROUNDUP((uintptr_t)&_end, RISCV_PGSIZE); // The lowest address user level program can use: the first page after the end of loaded PK
  current.brk_min = current.user_min;
  current.brk = 0;

  uint32_t mem_mb = *(volatile uint32_t*)0; // The total size of available physical memory is passed by address 0, type uint32_t;

  if (mem_mb == 0)
  {
    current.stack_bottom = 0;
    current.stack_top = 0;
    current.brk_max = 0;
    current.mmap_max = 0;
  }
  else
  {
    uintptr_t max_addr = (uintptr_t)mem_mb << 20;
    size_t mem_pages = max_addr >> RISCV_PGSHIFT;
    const size_t min_free_pages = 2*RISCV_PGLEVELS; // Minimum pages reserved for Page Table
    const size_t min_stack_pages = 8; // Minumum pages for userspace stack
    const size_t max_stack_pages = 1024; // Maximum pages for userspace stack
    kassert(mem_pages > min_free_pages + min_stack_pages);
    free_pages = MAX(mem_pages >> (RISCV_PGLEVEL_BITS-1), min_free_pages); // The actual pages reserved for Page Table
    size_t stack_pages = CLAMP(mem_pages/32, min_stack_pages, max_stack_pages); // The actual pages reserved for userspace stack
    first_free_page = max_addr - free_pages * RISCV_PGSIZE; // The pages for Page Table start allocation here

    uintptr_t root_page_table_paddr = __page_alloc();
    kassert(root_page_table_paddr);
    root_page_table = (pte_t*)root_page_table_paddr;

    // Direct map the ppages contains the loaded PK image as kernel space, with RWX kernel permission
    __map_kernel_range(0, current.user_min, PROT_READ|PROT_WRITE|PROT_EXEC);

    if (have_vm)
    {
      write_csr(ptbr, root_page_table_paddr); // Setup up root page table
      set_csr(status, SR_VM); // Temporary enable the Virtual Memory for testing
      have_vm = clear_csr(status, SR_VM) & SR_VM; // Readback then clear the SR_VM bit (keep the VM disable for a few more moment), just for checking whether platform supports VM
    }

    size_t stack_size = RISCV_PGSIZE * stack_pages;
    current.stack_top = MIN(first_free_page, 0x80000000); // for RV32 sanity
    uintptr_t stack_bot = current.stack_top - stack_size;

    if (have_vm)
    { // If the platform do support the VM
      // Direct map the ppages reserved for Page Table as kernel space, with RW kernel permission
      __map_kernel_range(first_free_page, free_pages * RISCV_PGSIZE, PROT_READ|PROT_WRITE);
      // Setup the mapping for user space stack
      kassert(__do_mmap(stack_bot, stack_size, -1, MAP_FIXED|MAP_PRIVATE|MAP_ANONYMOUS, 0, 0) == stack_bot);
      // Enable the Virtual Memory
      set_csr(status, SR_VM);
      /***************************************************************************************
      * Now the MMU is running in Virtual Address mode and starts making address translation *
      ***************************************************************************************/
    }

    current.stack_bottom = stack_bot;
    stack_bot -= RISCV_PGSIZE; // guard page
    // the mmap_max is the upper limit for non-FIXED mmap request
    // the brk_max is the upper limit for sys_brk()
    current.mmap_max = current.brk_max = stack_bot;
  }
}
