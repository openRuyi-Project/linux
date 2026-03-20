#include <linux/kernel.h>
#include <linux/list.h>
#include <linux/spinlock_types.h>
#include <linux/sched.h>
#include <linux/mm.h>
#include <asm/ptrace.h>
#include <asm/sbi.h>
#include <asm/bosc_mm_fault_workaround.h>

#define BOSC_WORKAROUND_SEGMENT_FAULT_TRY_CNT 3
#define BOSC_WORKAROUND_KERNEL_FAULT_TRY_CNT 3

static unsigned long last_segment_fault_addr = 0;
static unsigned long last_kernel_fault_addr = 0;

static int segment_fault_workaround_cnt = 0;
static int kernel_fault_workaround_cnt = 0;

#define CSR_FLUSH_PWR                           0xBC1
#define FLUSH_PWR_FLUSH_L2_EN                   0
#define FLUSH_PWR_FLUSH_L2_DONE                 1
static void core_cache_ctrl(void)
{
    uint64_t value = 0;

    smp_mb();

    csr_set(CSR_FLUSH_PWR, BIT(0));
    value = csr_read(CSR_FLUSH_PWR);
    while((value & BIT(FLUSH_PWR_FLUSH_L2_DONE)) == 0) {
        value = csr_read(CSR_FLUSH_PWR);
    }
}

int bosc_kernel_fault_workaround(struct pt_regs *regs)
{
	unsigned long addr = regs->badaddr;

	if (addr != last_kernel_fault_addr) {
		kernel_fault_workaround_cnt = 0;
		goto kernel_workaround;
	}

	kernel_fault_workaround_cnt++;

	if (kernel_fault_workaround_cnt < BOSC_WORKAROUND_KERNEL_FAULT_TRY_CNT)
		goto kernel_workaround;

	return 0;

kernel_workaround:
	printk("##### BOSC KERNEL FAULT WORKAROUND, cnt:%d\n", kernel_fault_workaround_cnt);
	last_kernel_fault_addr = addr;
	printk("badaddr: 0x%lx\n", addr);
	dump_stack();

	asm volatile ("sfence.vma" ::: "memory");
	sbi_flush_l2cache();

	return 1;
}
EXPORT_SYMBOL(bosc_kernel_fault_workaround);

int bosc_segment_fault_workaround(struct pt_regs *regs)
{
	unsigned long addr = regs->badaddr;

	if (addr != last_segment_fault_addr) {
		segment_fault_workaround_cnt = 0;
		goto segment_workaround;
	}

	segment_fault_workaround_cnt++;

	if (segment_fault_workaround_cnt < BOSC_WORKAROUND_SEGMENT_FAULT_TRY_CNT)
		goto segment_workaround;

	return 0;

segment_workaround:
	printk("##### BOSC SEGMENT FAULT WORKAROUND, cnt:%d\n", segment_fault_workaround_cnt);
	last_segment_fault_addr = addr;
	printk("badaddr: 0x%lx\n", addr);
	dump_stack();

	asm volatile ("sfence.vma" ::: "memory");
	sbi_flush_l2cache();

	return 1;
}
EXPORT_SYMBOL(bosc_segment_fault_workaround);
