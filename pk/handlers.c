// See LICENSE for license details.

#include "pk.h"
#include "config.h"
#include "syscall.h"
#include "vm.h"
 
static void handle_accelerator_disabled(trapframe_t* tf)
{
  if (have_accelerator)
    tf->m_sr |= SR_EA;
  else
  { 
    dump_tf(tf);
    panic("No accelerator hardware!");
  }
}

static void handle_privileged_instruction(trapframe_t* tf)
{
  dump_tf(tf);
  panic("A privileged instruction was executed!");
}

static void handle_illegal_instruction(trapframe_t* tf)
{
  tf->m_insn = *(uint16_t*)tf->m_epc;
  int len = insn_len(tf->m_insn);
  if (len == 4)
    tf->m_insn |= ((uint32_t)*(uint16_t*)(tf->m_epc + 2) << 16);
  else
    kassert(len == 2);

#ifdef PK_ENABLE_FP_EMULATION
  if (emulate_fp(tf) == 0)
  {
    tf->m_epc += len;
    return;
  }
#endif

  if (emulate_int(tf) == 0)
  {
    tf->m_epc += len;
    return;
  }

  dump_tf(tf);
  panic("An illegal instruction was executed!");
}

static void handle_fp_disabled(trapframe_t* tf)
{
  if (have_fp && !(read_csr(status) & SR_EF))
    init_fp(tf);
  else
    handle_illegal_instruction(tf);
}

static void handle_breakpoint(trapframe_t* tf)
{
  dump_tf(tf);
  printk("Breakpoint!\n");
  tf->m_epc += 4;
}

static void handle_misaligned_fetch(trapframe_t* tf)
{
  dump_tf(tf);
  panic("Misaligned instruction access!");
}

void handle_misaligned_load(trapframe_t* tf)
{
  // TODO emulate misaligned loads and stores
  dump_tf(tf);
  panic("Misaligned load!");
}

void handle_misaligned_store(trapframe_t* tf)
{
  dump_tf(tf);
  panic("Misaligned store!");
}

static void segfault(trapframe_t* tf, uintptr_t addr, const char* type)
{
  dump_tf(tf);
  const char* who = (tf->m_sr & SR_PS) ? "Kernel" : "User";
  panic("%s %s segfault @ %p", who, type, addr);
}

static void handle_fault_fetch(trapframe_t* tf)
{
  if (handle_page_fault(tf->m_epc, PROT_EXEC) != 0)
    segfault(tf, tf->m_epc, "fetch");
}

void handle_fault_load(trapframe_t* tf)
{
  tf->m_badvaddr = read_csr(badvaddr);
  if (handle_page_fault(tf->m_badvaddr, PROT_READ) != 0)
    segfault(tf, tf->m_badvaddr, "load");
}

void handle_fault_store(trapframe_t* tf)
{
  tf->m_badvaddr = read_csr(badvaddr);
  if (handle_page_fault(tf->m_badvaddr, PROT_WRITE) != 0)
    segfault(tf, tf->m_badvaddr, "store");
}

static void handle_syscall(trapframe_t* tf)
{
  tf->m_gpr[10] = do_syscall(tf->m_gpr[10], tf->m_gpr[11], tf->m_gpr[12], tf->m_gpr[13],
                           tf->m_gpr[14], tf->m_gpr[15], tf->m_gpr[17]);
  tf->m_epc += 4;
}

void handle_trap(trapframe_t* tf)
{
  set_csr(status, SR_EI);

  typedef void (*trap_handler)(trapframe_t*);

  const static trap_handler trap_handlers[] = {
    [CAUSE_MISALIGNED_FETCH] = handle_misaligned_fetch,
    [CAUSE_FAULT_FETCH] = handle_fault_fetch,
    [CAUSE_ILLEGAL_INSTRUCTION] = handle_illegal_instruction,
    [CAUSE_PRIVILEGED_INSTRUCTION] = handle_privileged_instruction,
    [CAUSE_FP_DISABLED] = handle_fp_disabled,
    [CAUSE_SYSCALL] = handle_syscall,
    [CAUSE_BREAKPOINT] = handle_breakpoint,
    [CAUSE_MISALIGNED_LOAD] = handle_misaligned_load,
    [CAUSE_MISALIGNED_STORE] = handle_misaligned_store,
    [CAUSE_FAULT_LOAD] = handle_fault_load,
    [CAUSE_FAULT_STORE] = handle_fault_store,
    [CAUSE_ACCELERATOR_DISABLED] = handle_accelerator_disabled,
  };

  kassert(tf->m_cause < ARRAY_SIZE(trap_handlers) && trap_handlers[tf->m_cause]);

  trap_handlers[tf->m_cause](tf);

  pop_tf(tf);
}
