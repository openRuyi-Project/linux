#ifndef __BOSC_MM_FAULT_WORKAROUND_H__
#define __BOSC_MM_FAULT_WORKAROUND_H__

#include <asm/ptrace.h>

int bosc_kernel_fault_workaround(struct pt_regs *regs);
int bosc_segment_fault_workaround(struct pt_regs *regs);

#endif
