// See LICENSE for license details.

#include "pk.h"


#include "softint.h"
#include <stdint.h>

#define noisy 0


int emulate_int(trapframe_t* tf)
{
  if(noisy)
    printk("Int emulation at pc %lx, insn %x\n",tf->m_epc,(uint32_t)tf->m_insn);

  #define RS1 ((tf->m_insn >> 15) & 0x1F)
  #define RS2 ((tf->m_insn >> 20) & 0x1F)
  #define RD  ((tf->m_insn >>  7) & 0x1F)

//  #define XRS1 (tf->m_gpr[RS1])
//  #define XRS2 (tf->m_gpr[RS2])
  #define XRD  (tf->m_gpr[RD])

  unsigned long xrs1 = tf->m_gpr[RS1];
  unsigned long xrs2 = tf->m_gpr[RS2];

  #define IS_INSN(x) ((tf->m_insn & MASK_ ## x) == MATCH_ ## x)
   
  if(IS_INSN(DIV))
  {
    if(noisy)
      printk("emulating div\n");

    int num_negative = 0;
                     
    if ((signed long) xrs1 < 0)
    {
      xrs1 = -xrs1;
      num_negative++;
    }
                     
    if ((signed long) xrs2 < 0)
    {
      xrs2 = -xrs2;
      num_negative++;
    }

    unsigned long res = softint_udivrem(xrs1, xrs2, 0);
    if (num_negative == 1)
      XRD = -res;
    else
      XRD = res;
  }
  else if(IS_INSN(DIVU))
  {
    if(noisy)
      printk("emulating divu\n");
    XRD = softint_udivrem( xrs1, xrs2, 0);
  }
  else if(IS_INSN(MUL))
  {
    if(noisy)
      printk("emulating mul\n");
    XRD = softint_mul(xrs1, xrs2);
  }
  else if(IS_INSN(REM))
  {
    if(noisy)
      printk("emulating rem\n");

    if ((signed long) xrs1 < 0) {xrs1 = -xrs1;}
    if ((signed long) xrs2 < 0) {xrs2 = -xrs2;}

    XRD = softint_udivrem(xrs1, xrs2, 1);
  }
  else if(IS_INSN(REMU))
  {
    if(noisy)
      printk("emulating remu\n");
    XRD = softint_udivrem(xrs1, xrs2, 1);
  }
  else
    return -1;

  return 0;
}


