// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2024 Anlogic
 */

#include <linux/bug.h>
#include <linux/kernel.h>
#include <linux/memory.h>
#include <linux/module.h>
#include <linux/string.h>
#include <linux/uaccess.h>
#include <asm/alternative.h>
#include <asm/cacheflush.h>
#include <asm/errata_list.h>
#include <asm/patch.h>
#include <asm/vendorid_list.h>

static bool errata_probe_pbmt(unsigned int stage,
			      unsigned long arch_id, unsigned long impid)
{
	return false;
}

static bool errata_probe_ccm(unsigned int stage,
			     unsigned long arch_id, unsigned long impid)
{
	if (!IS_ENABLED(CONFIG_ERRATA_ANLOGIC_CCM))
		return false;

	if (stage == RISCV_ALTERNATIVES_EARLY_BOOT)
		return false;

	riscv_noncoherent_supported();
	return true;
}

static u32 anlogic_errata_probe(unsigned int stage,
			      unsigned long archid, unsigned long impid)
{
	u32 cpu_req_errata = 0;

	if (errata_probe_pbmt(stage, archid, impid))
		cpu_req_errata |= BIT(ERRATA_ANLOGIC_PBMT);

	if (errata_probe_ccm(stage, archid, impid))
		cpu_req_errata |= BIT(ERRATA_ANLOGIC_CCM);

	return cpu_req_errata;
}

void __init_or_module anlogic_errata_patch_func(struct alt_entry *begin, struct alt_entry *end,
					      unsigned long archid, unsigned long impid,
					      unsigned int stage)
{
	struct alt_entry *alt;
	u32 cpu_req_errata;
	u32 tmp;

	cpu_req_errata = anlogic_errata_probe(stage, archid, impid);

	for (alt = begin; alt < end; alt++) {
		if (alt->vendor_id != ANLOGIC_VENDOR_ID)
			continue;
		if (alt->errata_id >= ERRATA_ANLOGIC_NUMBER)
			continue;

		tmp = (1U << alt->errata_id);
		if (cpu_req_errata & tmp) {
			/* On vm-alternatives, the mmu isn't running yet */
			if (stage == RISCV_ALTERNATIVES_EARLY_BOOT) {
				memcpy((void *)__pa_symbol(alt->old_ptr),
				       (void *)__pa_symbol(alt->alt_ptr), alt->alt_len);
			} else {
				mutex_lock(&text_mutex);
				patch_text_nosync(alt->old_ptr, alt->alt_ptr, alt->alt_len);
				mutex_unlock(&text_mutex);
			}
		}
	}

	if (stage == RISCV_ALTERNATIVES_EARLY_BOOT)
		local_flush_icache_all();
}
