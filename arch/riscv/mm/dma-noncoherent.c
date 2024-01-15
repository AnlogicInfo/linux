// SPDX-License-Identifier: GPL-2.0-only
/*
 * RISC-V specific functions to support DMA for non-coherent devices
 *
 * Copyright (c) 2021 Western Digital Corporation or its affiliates.
 */

#include <linux/dma-direct.h>
#include <linux/dma-map-ops.h>
#include <linux/mm.h>
#include <asm/cacheflush.h>
#include <asm/sbi.h>
#include <linux/dma-map-ops.h>

#define ALSIP_NCACHE				(SBI_EXT_VENDOR_START+1)
#define ALSIP_NCACHE_SET			0
#define ALSIP_NCACHE_CLR			1

static bool noncoherent_supported;

#ifdef CONFIG_ERRATA_ANLOGIC_CCM
typedef enum CCM_CMD {
	CCM_DC_INVAL = 0x0,			/*!< Unlock and invalidate D-Cache line specified by CSR CCM_XBEGINADDR */
	CCM_DC_WB = 0x1,			/*!< Flush the specific D-Cache line specified by CSR CCM_XBEGINADDR */
	CCM_DC_WBINVAL = 0x2,			/*!< Unlock, flush and invalidate the specific D-Cache line specified by CSR CCM_XBEGINADDR */
	CCM_DC_LOCK = 0x3,			/*!< Lock the specific D-Cache line specified by CSR CCM_XBEGINADDR */
	CCM_DC_UNLOCK = 0x4,			/*!< Unlock the specific D-Cache line specified by CSR CCM_XBEGINADDR */
	CCM_DC_WBINVAL_ALL = 0x6,		/*!< Unlock and flush and invalidate all the valid and dirty D-Cache lines */
	CCM_DC_WB_ALL = 0x7,			/*!< Flush all the valid and dirty D-Cache lines */
	CCM_DC_INVAL_ALL = 0x17,		/*!< Unlock and invalidate all the D-Cache lines */
	CCM_IC_INVAL = 0x8,			/*!< Unlock and invalidate I-Cache line specified by CSR CCM_XBEGINADDR */
	CCM_IC_LOCK = 0xb,			/*!< Lock the specific I-Cache line specified by CSR CCM_XBEGINADDR */
	CCM_IC_UNLOCK = 0xc,			/*!< Unlock the specific I-Cache line specified by CSR CCM_XBEGINADDR */
	CCM_IC_INVAL_ALL = 0xd			/*!< Unlock and invalidate all the I-Cache lines */
} CCM_CMD_Type;

/* CCM DCache Operation */
void ccm_dcache_op(CCM_CMD_Type op, void* va, size_t size, unsigned int block_size)
{
	size_t start = 0;
	// set s-mode operation address ccm_sbeginaddr
	csr_write(0x5cb, va);
	for (start = 0; start < size; start += block_size) {
		// set s-mode ccm operation command
		csr_write(0x5cc, op);
	}
}

#if IS_ENABLED(CONFIG_ERRATA_ANLOGIC_NONCACHE)
static inline unsigned long anlogic_set_noncache(
			unsigned long addr, unsigned long addr_mask)
{
	struct sbiret ret = {0};
	ret = sbi_ecall(ALSIP_NCACHE, ALSIP_NCACHE_SET, addr, addr_mask, 0, 0, 0, 0);
	return ret.error;
}
int anlogic_init_ncache_area(phys_addr_t phys_addr, size_t size)
{
	int ret;
	ulong mask, addr, end, sz;
	void *vaddr;

	if (size < PAGE_SIZE)
		return -EINVAL;

	ret = PAGE_SHIFT - 1;
	addr = phys_addr;
	end = phys_addr + size - 1;
	do {
		ret++;
		mask = (1UL << ret) - 1;
	} while ((addr & ~mask) != (end & ~mask));

	if ((phys_addr & mask) || (size & mask))
		pr_warn("noncache addr/size should be aligned to %lx\n", mask+1);

	addr = (phys_addr & ~mask);
	sz = mask + 1;
	if (anlogic_set_noncache(addr, ~mask)) {
		pr_err("set noncache area fail\n");
		return -EINVAL;
	}
	pr_info("set noncache area: addr 0x%lx, size 0x%lx\n", addr, sz);

	vaddr = phys_to_virt(addr);
	ccm_dcache_op(CCM_DC_WBINVAL, vaddr, sz, riscv_cbom_block_size);

	return dma_init_global_coherent(phys_addr, size);
}
#endif
#endif

void arch_sync_dma_for_device(phys_addr_t paddr, size_t size,
			      enum dma_data_direction dir)
{
	void *vaddr = phys_to_virt(paddr);

	switch (dir) {
	case DMA_TO_DEVICE:
#ifdef CONFIG_ERRATA_ANLOGIC_CCM
		ccm_dcache_op(CCM_DC_WB, vaddr, size, riscv_cbom_block_size);
#else
		ALT_CMO_OP(clean, vaddr, size, riscv_cbom_block_size);
#endif
		break;
	case DMA_FROM_DEVICE:
#ifdef CONFIG_ERRATA_ANLOGIC_CCM
		ccm_dcache_op(CCM_DC_INVAL, vaddr, size, riscv_cbom_block_size);
#else
		ALT_CMO_OP(clean, vaddr, size, riscv_cbom_block_size);
#endif
		break;
	case DMA_BIDIRECTIONAL:
#ifdef CONFIG_ERRATA_ANLOGIC_CCM
		ccm_dcache_op(CCM_DC_WBINVAL, vaddr, size, riscv_cbom_block_size);
#else
		ALT_CMO_OP(flush, vaddr, size, riscv_cbom_block_size);
#endif
		break;
	default:
		break;
	}
}

void arch_sync_dma_for_cpu(phys_addr_t paddr, size_t size,
			   enum dma_data_direction dir)
{
	void *vaddr = phys_to_virt(paddr);

	switch (dir) {
	case DMA_TO_DEVICE:
		break;
	case DMA_FROM_DEVICE:
	case DMA_BIDIRECTIONAL:
#ifdef CONFIG_ERRATA_ANLOGIC_CCM
		ccm_dcache_op(CCM_DC_INVAL, vaddr, size, riscv_cbom_block_size);
#else
		ALT_CMO_OP(flush, vaddr, size, riscv_cbom_block_size);
#endif
		break;
	default:
		break;
	}
}

void arch_dma_prep_coherent(struct page *page, size_t size)
{
	void *flush_addr = page_address(page);

#ifdef CONFIG_ERRATA_ANLOGIC_CCM
	ccm_dcache_op(CCM_DC_WBINVAL, flush_addr, size, riscv_cbom_block_size);
#else
	ALT_CMO_OP(flush, flush_addr, size, riscv_cbom_block_size);
#endif
}

void arch_setup_dma_ops(struct device *dev, u64 dma_base, u64 size,
		const struct iommu_ops *iommu, bool coherent)
{
	WARN_TAINT(!coherent && riscv_cbom_block_size > ARCH_DMA_MINALIGN,
		   TAINT_CPU_OUT_OF_SPEC,
		   "%s %s: ARCH_DMA_MINALIGN smaller than riscv,cbom-block-size (%d < %d)",
		   dev_driver_string(dev), dev_name(dev),
		   ARCH_DMA_MINALIGN, riscv_cbom_block_size);

	WARN_TAINT(!coherent && !noncoherent_supported, TAINT_CPU_OUT_OF_SPEC,
		   "%s %s: device non-coherent but no non-coherent operations supported",
		   dev_driver_string(dev), dev_name(dev));

	dev->dma_coherent = coherent;
}

void riscv_noncoherent_supported(void)
{
	WARN(!riscv_cbom_block_size,
	     "Non-coherent DMA support enabled without a block size\n");
	noncoherent_supported = true;
}
