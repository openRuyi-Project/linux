// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2026 Picoheart (SG) Pte. Ltd.
 *
 * Author: Yicong Yang <yang.yicong@picoheart.com>
 */

#include <linux/cacheflush.h>
#include <linux/cleanup.h>
#include <linux/kernel.h>
#include <linux/memory.h>
#include <linux/string.h>
#include <asm/alternative.h>
#include <asm/errata_list.h>
#include <asm/text-patching.h>
#include <asm/vendor_extensions.h>
#include <asm/vendorid_list.h>

static u32 picoheart_errata_probe(unsigned int stage, unsigned long archid,
				  unsigned long impid)
{
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
