// SPDX-License-Identifier: GPL-2.0-only
/*
 * RISC-V specific functions to support DMA for non-coherent devices
 *
 * Copyright (c) 2021 Western Digital Corporation or its affiliates.
 */

#include <linux/dma-direct.h>
#include <linux/dma-map-ops.h>
#include <linux/mm.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <asm/cacheflush.h>
#include <linux/delay.h>

static unsigned int riscv_ccm_block_size = L1_CACHE_BYTES;
static bool noncoherent_supported;

typedef enum CCM_CMD {
	CCM_DC_INVAL = 0x0,				/*!< Unlock and invalidate D-Cache line specified by CSR CCM_XBEGINADDR */
	CCM_DC_WB = 0x1,				/*!< Flush the specific D-Cache line specified by CSR CCM_XBEGINADDR */
	CCM_DC_WBINVAL = 0x2,			/*!< Unlock, flush and invalidate the specific D-Cache line specified by CSR CCM_XBEGINADDR */
	CCM_DC_LOCK = 0x3,				/*!< Lock the specific D-Cache line specified by CSR CCM_XBEGINADDR */
	CCM_DC_UNLOCK = 0x4,			/*!< Unlock the specific D-Cache line specified by CSR CCM_XBEGINADDR */
	CCM_DC_WBINVAL_ALL = 0x6,		/*!< Unlock and flush and invalidate all the valid and dirty D-Cache lines */
	CCM_DC_WB_ALL = 0x7,			/*!< Flush all the valid and dirty D-Cache lines */
	CCM_DC_INVAL_ALL = 0x17,		/*!< Unlock and invalidate all the D-Cache lines */
	CCM_IC_INVAL = 0x8,				/*!< Unlock and invalidate I-Cache line specified by CSR CCM_XBEGINADDR */
	CCM_IC_LOCK = 0xb,				/*!< Lock the specific I-Cache line specified by CSR CCM_XBEGINADDR */
	CCM_IC_UNLOCK = 0xc,			/*!< Unlock the specific I-Cache line specified by CSR CCM_XBEGINADDR */
	CCM_IC_INVAL_ALL = 0xd			/*!< Unlock and invalidate all the I-Cache lines */
} CCM_CMD_Type;

/* === Nuclei CCM Registers === */
#define CSR_CCM_MBEGINADDR      0x7CB
#define CSR_CCM_MCOMMAND        0x7CC
#define CSR_CCM_MDATA           0x7CD
#define CSR_CCM_SUEN            0x7CE
#define CSR_CCM_SBEGINADDR      0x5CB
#define CSR_CCM_SCOMMAND        0x5CC
#define CSR_CCM_SDATA           0x5CD
#define CSR_CCM_UBEGINADDR      0x4CB
#define CSR_CCM_UCOMMAND        0x4CC
#define CSR_CCM_UDATA           0x4CD
#define CSR_CCM_FPIPE           0x4CF



//ccm_dcache_op(CCM_DC_WB, vaddr, size);
//ccm_dcache_op(CCM_DC_INVAL, vaddr, size);

void ccm_dcache_op(CCM_CMD_Type op, void* va, size_t size)
{
	size_t start = 0;
	// set s-mode operation address ccm_sbeginaddr
	csr_write(0x5cb, va);
	for (start = 0; start < size; start += riscv_ccm_block_size) {
		// set s-mode ccm operation command
		csr_write(0x5cc, op);
	}
	
	csr_write(CSR_CCM_FPIPE, 0x1);
}

void arch_sync_dma_for_device(phys_addr_t paddr, size_t size,
				  enum dma_data_direction dir)
{
	void *vaddr = phys_to_virt(paddr);
	int i=0;
	int j=0;
	//printk("---> arch_sync_dma_for_device paddr:0x%llx vaddr:0x%lx size:%ld \n", paddr, vaddr, size);

	switch (dir) {
	case DMA_TO_DEVICE:
		ccm_dcache_op(CCM_DC_WB, vaddr, size);
		break;
	case DMA_FROM_DEVICE:
		ccm_dcache_op(CCM_DC_INVAL, vaddr, size);
		break;
	case DMA_BIDIRECTIONAL:
		ccm_dcache_op(CCM_DC_WBINVAL, vaddr, size);
		break;
	default:
		break;
	}
}

void arch_sync_dma_for_cpu(phys_addr_t paddr, size_t size,
			   enum dma_data_direction dir)
{
	void *vaddr = phys_to_virt(paddr);
//printk("<--- riscv arch_sync_dma_for_cpu \n");
	switch (dir) {
	case DMA_TO_DEVICE:
		break;
	case DMA_FROM_DEVICE:
	case DMA_BIDIRECTIONAL:
		ccm_dcache_op(CCM_DC_INVAL, vaddr, size);
		break;
	default:
		break;
	}
}

void arch_dma_prep_coherent(struct page *page, size_t size)
{
	void *flush_addr = page_address(page);

	ccm_dcache_op(CCM_DC_WBINVAL, flush_addr, size);
}

void arch_setup_dma_ops(struct device *dev, u64 dma_base, u64 size,
		const struct iommu_ops *iommu, bool coherent)
{
	WARN_TAINT(!coherent && riscv_ccm_block_size > ARCH_DMA_MINALIGN,
		   TAINT_CPU_OUT_OF_SPEC,
		   "%s %s: ARCH_DMA_MINALIGN smaller than riscv,ccm-block-size (%d < %d)",
		   dev_driver_string(dev), dev_name(dev),
		   ARCH_DMA_MINALIGN, riscv_ccm_block_size);

	WARN_TAINT(!coherent && !noncoherent_supported, TAINT_CPU_OUT_OF_SPEC,
		   "%s %s: device non-coherent but no non-coherent operations supported",
		   dev_driver_string(dev), dev_name(dev));
		   
	dev->dma_coherent = coherent;
}

void riscv_noncoherent_supported(void)
{
	noncoherent_supported = true;
}

#ifdef CONFIG_NUCLEI_RISCV_ISA_XXLCCM
void riscv_init_xxlccm_blocksize(void)
{
	struct device_node *node;
	int ret;
	u32 val;
	int ccm_hartid = -1;
	
	for_each_of_cpu_node(node) {
		int hartid = riscv_of_processor_hartid(node);

		if (hartid < 0)
			continue;

		/* set block-size for ccm extension if available */
		ret = of_property_read_u32(node, "riscv,ccm-block-size", &val);
		if (ret)
			continue;

		pr_info("ccm-block-size for hart %d is %d bytes\n", hartid, val);
		if (ccm_hartid == -1) {
			riscv_ccm_block_size = val;
			ccm_hartid = hartid;
		} else {
			if (riscv_ccm_block_size != val)
				pr_warn("ccm-block-size mismatched between harts %d and %d\n",
					ccm_hartid, hartid);
		}
	}
	
	riscv_noncoherent_supported(); //hcx_add
}
#endif


