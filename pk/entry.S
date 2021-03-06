// See LICENSE for license details.
.include "csr_2.1.inc"

#include "encoding.h"

#ifdef __riscv64
# define STORE    sd
# define LOAD     ld
# define REGBYTES 8
#else
# define STORE    sw
# define LOAD     lw
# define REGBYTES 4
#endif

  .macro save_tf
  # save gprs
  STORE  x3,3*REGBYTES(x2)
  STORE  x4,4*REGBYTES(x2)
  STORE  x5,5*REGBYTES(x2)
  STORE  x6,6*REGBYTES(x2)
  STORE  x7,7*REGBYTES(x2)
  STORE  x8,8*REGBYTES(x2)
  STORE  x9,9*REGBYTES(x2)
  STORE  x10,10*REGBYTES(x2)
  STORE  x11,11*REGBYTES(x2)
  STORE  x12,12*REGBYTES(x2)
  STORE  x13,13*REGBYTES(x2)
  STORE  x14,14*REGBYTES(x2)
  STORE  x15,15*REGBYTES(x2)
  STORE  x16,16*REGBYTES(x2)
  STORE  x17,17*REGBYTES(x2)
  STORE  x18,18*REGBYTES(x2)
  STORE  x19,19*REGBYTES(x2)
  STORE  x20,20*REGBYTES(x2)
  STORE  x21,21*REGBYTES(x2)
  STORE  x22,22*REGBYTES(x2)
  STORE  x23,23*REGBYTES(x2)
  STORE  x24,24*REGBYTES(x2)
  STORE  x25,25*REGBYTES(x2)
  STORE  x26,26*REGBYTES(x2)
  STORE  x27,27*REGBYTES(x2)
  STORE  x28,28*REGBYTES(x2)
  STORE  x29,29*REGBYTES(x2)
  STORE  x30,30*REGBYTES(x2)
  STORE  x31,31*REGBYTES(x2)

  # get sr, epc, badvaddr, cause
  csrr   x3,sup0    # x1
  csrr   x4,sup1    # x2
  csrr   x5,status
  csrr   x6,epc
  csrr   x8,cause
  STORE  x3,1*REGBYTES(x2)
  STORE  x4,2*REGBYTES(x2)
  STORE  x5,32*REGBYTES(x2)
  STORE  x6,33*REGBYTES(x2)
  STORE  x8,35*REGBYTES(x2)

  la gp, _gp

  # get faulting insn, if it wasn't a fetch-related trap
  li x5,-1
  STORE x5,36*REGBYTES(x2)
1:
  .endm

  .text
  .globl  pop_tf
pop_tf:  # write the trap frame onto the stack
  # restore gprs
  LOAD  a1,32*REGBYTES(a0)
  LOAD  a2,1*REGBYTES(a0)
  LOAD  a3,2*REGBYTES(a0)

  csrw   status, a1 # restore sr (disable interrupts)
  csrw   sup0, a2
  csrw   sup1, a3

  move  x1,a0
  LOAD  x3,3*REGBYTES(x1)
  LOAD  x4,4*REGBYTES(x1)
  LOAD  x5,5*REGBYTES(x1)
  LOAD  x6,6*REGBYTES(x1)
  LOAD  x7,7*REGBYTES(x1)
  LOAD  x8,8*REGBYTES(x1)
  LOAD  x9,9*REGBYTES(x1)
  LOAD  x10,10*REGBYTES(x1)
  LOAD  x11,11*REGBYTES(x1)
  LOAD  x12,12*REGBYTES(x1)
  LOAD  x13,13*REGBYTES(x1)
  LOAD  x14,14*REGBYTES(x1)
  LOAD  x15,15*REGBYTES(x1)
  LOAD  x16,16*REGBYTES(x1)
  LOAD  x17,17*REGBYTES(x1)
  LOAD  x18,18*REGBYTES(x1)
  LOAD  x19,19*REGBYTES(x1)
  LOAD  x20,20*REGBYTES(x1)
  LOAD  x21,21*REGBYTES(x1)
  LOAD  x22,22*REGBYTES(x1)
  LOAD  x23,23*REGBYTES(x1)
  LOAD  x24,24*REGBYTES(x1)
  LOAD  x25,25*REGBYTES(x1)
  LOAD  x26,26*REGBYTES(x1)
  LOAD  x27,27*REGBYTES(x1)
  LOAD  x28,28*REGBYTES(x1)
  LOAD  x29,29*REGBYTES(x1)
  LOAD  x30,30*REGBYTES(x1)
  LOAD  x31,31*REGBYTES(x1)

  # gtfo!
  LOAD  x2,33*REGBYTES(x1)
  csrr  x1, sup0
  csrw  epc, x2
  csrr  x2, sup1
  # To cope with the change in 'sret' instruction encoding, issue the
  # old encoding of 'sret' directly
  .word 0x80000073 # sret

  .global  trap_entry
trap_entry:
  csrw  sup0, x1
  csrw  sup1, sp

  # when coming from kernel, continue below its stack
  csrr  x1, status
  and   x1, x1, SR_PS
  add   sp, sp, -320
  bnez  x1, 1f
  la    sp, stack_top-320

1:save_tf
  move  a0,sp
  j     handle_trap

  .bss
  .align 4
  .global stack_bot
  .global stack_top
stack_bot:
  .skip  4096
stack_top:
