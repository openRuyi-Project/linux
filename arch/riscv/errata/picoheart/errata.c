// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2026 Picoheart (SG) Pte. Ltd.
 *
 * Author: Yicong Yang <yang.yicong@picoheart.com>
 */

#include <linux/align.h>
#include <linux/cacheflush.h>
#include <linux/cleanup.h>
#include <linux/jump_label.h>
#include <linux/kernel.h>
#include <linux/memory.h>
#include <linux/mm_types.h>
#include <linux/mmap_lock.h>
#include <linux/signal.h>
#include <linux/string.h>
#include <asm/asm-extable.h>
#include <asm/alternative.h>
#include <asm/cpufeature.h>
#include <asm/dma-noncoherent.h>
#include <asm/errata_list.h>
#include <asm/insn.h>
#include <asm/insn-def.h>
#include <asm/text-patching.h>
#include <asm/uaccess.h>
#include <asm/vendor_extensions.h>
#include <asm/vendorid_list.h>

#ifdef CONFIG_ERRATA_PICOHEART_CBO_CLEAN

static bool insn_is_cbo_clean_flush(u32 insn_buf)
{
	u32 rd, opcode, funct3, funct12;

	/* CBO instructions don't have compressed variants */
	if (GET_INSN_LENGTH(insn_buf) != 4)
		return false;

	rd = RV_EXTRACT_RD_REG(insn_buf);
	opcode = insn_buf & __INSN_OPCODE_MASK;
	funct3 = insn_buf & RV_INSN_FUNCT3_MASK;
	funct12 = insn_buf & RV_INSN_FUNCT12_MASK;

	if (rd != 0 || opcode != RVG_OPCODE_MISC_MEM ||
	    funct3 != RV_ENCODE_FUNCT3(CBO) ||
	    (funct12 != RV_ENCODE_FUNCT12(CBO_CLEAN) &&
	     funct12 != RV_ENCODE_FUNCT12(CBO_FLUSH)))
		return false;

	return true;
}

static int cbo_clean_fault_sicode(unsigned long addr)
{
	struct mm_struct *mm = current->mm;
	struct vm_area_struct *vma;

	guard(mmap_read_lock)(mm);
	vma = vma_lookup(mm, addr);
	if (!vma)
		return SEGV_MAPERR;

	return SEGV_ACCERR;
}

bool riscv_picoheart_illegal_insn_handler(struct pt_regs *regs)
{
	unsigned long insn = regs->badaddr;
	unsigned long epc = regs->epc;
	void __user *line_addr;
	int ret = 0;
	u64 addr;
	u32 rs1;

	if (!insn) {
		if (get_insn(regs, epc, &insn))
			return false;
	}

	if (!insn_is_cbo_clean_flush(insn))
		return false;

	rs1 = RV_EXTRACT_RS1_REG(insn);
	addr = rs1 ? ((unsigned long *)regs)[rs1] : 0;
	line_addr = (void __user *)ALIGN_DOWN(addr, riscv_cbom_block_size);

	/*
	 * Check if the target address is within the valid userspace
	 * address range. Otherwise update the bad_cause to match the
	 * store page fault. See the comment below.
	 */
	if (!access_ok(line_addr, riscv_cbom_block_size)) {
		current->thread.bad_cause = EXC_STORE_PAGE_FAULT;
		goto err_map;
	}

	__enable_user_access();
	asm volatile("\n"
		"1:	\n"
		CBO_FLUSH(%[addr])
		"2:	\n"
		_ASM_EXTABLE_UACCESS_ERR(1b, 2b, %[err])
	: [err] "+r" (ret)
	: [addr] "r" (untagged_addr(addr))
	: "memory");
	__disable_user_access();

	if (ret)
		goto err_map;

	regs->epc += GET_INSN_LENGTH(insn);
	return true;

err_map:
	/*
	 * We'll reach here if the target address is invalid.
	 * cbo.{flush ,clean} on invalid address will cause
	 * store page fault, update the cause and badaddr.
	 */
	regs->badaddr = addr;
	regs->cause = current->thread.bad_cause;

	/*
	 * In emulation of the cbo.clean, a SIGSEGV rather than
	 * a SIGILL should be issued if the target address is
	 * invalid. Use do_trap() to handle the signal delivery
	 * and related stuffs to make it analogous to the page
	 * fault.
	 */
	do_trap(regs, SIGSEGV, cbo_clean_fault_sicode(untagged_addr(addr)),
		untagged_addr(addr));

	return true;
}

#endif /* CONFIG_ERRATA_PICOHEART_CBO_CLEAN */

static void picoheart_errata_cache_wback(phys_addr_t paddr, size_t size)
{
	void *vaddr = phys_to_virt(paddr);

	ALT_CMO_OP(FLUSH, vaddr, size, riscv_cbom_block_size);
}

/*
 * The errata only affects the cbo.clean (wback) so use standard operations
 * for other semantic.
 */
struct riscv_nonstd_cache_ops picoheart_errata_cmo_ops = {
	.wback = &picoheart_errata_cache_wback,
};

DEFINE_STATIC_KEY_FALSE(has_picoheart_cbo_clean_errata);

static void picoheart_errata_probe_cbo_clean(unsigned int stage,
					     unsigned long archid,
					     unsigned long impid)
{
	if (!IS_ENABLED(CONFIG_ERRATA_PICOHEART_CBO_CLEAN))
		return;

	if (stage != RISCV_ALTERNATIVES_BOOT)
		return;

	if (!riscv_isa_extension_available(NULL, ZICBOM))
		return;

	if (archid != 0x804a555049544552 || impid != 0x100)
		return;

	riscv_noncoherent_supported();
	riscv_noncoherent_register_cache_ops(&picoheart_errata_cmo_ops);
	static_branch_enable(&has_picoheart_cbo_clean_errata);
}

static u32 picoheart_errata_probe(unsigned int stage, unsigned long archid,
				  unsigned long impid)
{
	picoheart_errata_probe_cbo_clean(stage, archid, impid);
	return 0;
}

void picoheart_errata_patch_func(struct alt_entry *begin, struct alt_entry *end,
				 unsigned long archid, unsigned long impid,
				 unsigned int stage)
{
	u32 cpu_req_errata = picoheart_errata_probe(stage, archid, impid);
	struct alt_entry *alt;
	void *oldptr, *altptr;
	u32 tmp;

	BUILD_BUG_ON(ERRATA_PICOHEART_NUMBER >= RISCV_VENDOR_EXT_ALTERNATIVES_BASE);

	for (alt = begin; alt < end; alt++) {
		if (alt->vendor_id != PICOHEART_VENDOR_ID ||
		    alt->patch_id >= ERRATA_PICOHEART_NUMBER)
			continue;

		tmp = (1U << alt->patch_id);
		if (cpu_req_errata & tmp) {
			oldptr = ALT_OLD_PTR(alt);
			altptr = ALT_ALT_PTR(alt);

			if (stage == RISCV_ALTERNATIVES_EARLY_BOOT) {
				memcpy(oldptr, altptr, alt->alt_len);
			} else {
				guard(mutex)(&text_mutex);
				patch_text_nosync(oldptr, altptr, alt->alt_len);
			}
		}
	}

	if (stage == RISCV_ALTERNATIVES_EARLY_BOOT)
		local_flush_icache_all();
}
