// SPDX-License-Identifier: GPL-2.0
/*
 * AL9000 Remote Processor driver
 *
 * Copyright (C) 2023 Anlogic, Inc.
 * 
 * Based on Zynq Remote Processor driver
 *
 * Copyright (C) 2012 Michal Simek <monstr@monstr.eu>
 * Copyright (C) 2012 PetaLogix
 *
 * Based on origin OMAP Remote Processor driver
 *
 * Copyright (C) 2011 Texas Instruments, Inc.
 * Copyright (C) 2011 Google, Inc.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/err.h>
#include <linux/platform_device.h>
#include <linux/dma-mapping.h>
#include <linux/remoteproc.h>
#include <linux/interrupt.h>
#include <linux/irqdomain.h>
#include <linux/irq.h>
#include <linux/irqchip.h>
#include <linux/irqchip/arm-gic.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/of_reserved_mem.h>
#include <linux/smp.h>
#include <linux/irqchip/arm-gic.h>
#include <linux/slab.h>
#include <linux/cpu.h>
#include <linux/genalloc.h>
#include <linux/psci.h>
#include "remoteproc_internal.h"
#include "linux/arm-smccc.h"

/* Function ID passed during SiP SVC call. */
#define FN_RPROC_BASE 	(0xFF000000)
#define FN_RPROC_OFF 	(FN_RPROC_BASE + 21)
#define FN_RPROC_ON 	(FN_RPROC_BASE + 22)

#define MAX_NUM_VRINGS 2
#define NOTIFYID_ANY (-1)

/* Maximum on chip memories used by the driver*/
#define MAX_ON_CHIP_MEMS	32
#define REMOTE_SGI		0
#define HOST_SGI		1

static int vring_sgis[MAX_NUM_VRINGS] = { 14, 15 };
/* count number of SGIs passed via command line if applicable. */
static int n_vring_sgis = MAX_NUM_VRINGS;

/* Structure for storing IRQs */
 struct irq_list {
	int irq;
	struct list_head list;
}; 

/* Structure for IPIs */
struct ipi_info {
	u32 irq;
	u32 notifyid;
	bool pending;
};

/**
 * struct dragon_mem_res - dragon memory resource for firmware memory
 * @res: memory resource
 * @node: list node
 */
struct dragon_mem_res {
	struct resource res;
	struct list_head node;
};

/**
 * struct dragon_rproc_data - dragon rproc private data
 * @irqs: inter processor soft IRQs
 * @ipi_desc: list of IRQ descriptors for each vring's IRQ.
 * @rproc: pointer to remoteproc instance
 * @ipis: interrupt processor interrupts statistics
 * @fw_mems: list of firmware memories
 */
struct dragon_rproc_pdata {
	int irqs[MAX_NUM_VRINGS];
	struct irq_desc *ipi_desc[MAX_NUM_VRINGS];
	struct rproc *rproc;
	struct ipi_info ipis[MAX_NUM_VRINGS];
	struct list_head fw_mems;
};

static bool autoboot __read_mostly;

/* Store rproc for IPI handler */
static struct rproc *rproc;
static struct work_struct workqueue;

static irqreturn_t dragon_remoteproc_interrupt(int irq, void *dev_id)
{
	struct dragon_rproc_pdata *local = dev_id;

	dev_dbg(local->rproc->dev.parent, "KICK Linux because of pending message\n");
	schedule_work(&workqueue);

	return IRQ_HANDLED;
}

struct irqaction action = {
	.handler = dragon_remoteproc_interrupt,
};

static void handle_event(struct work_struct *work)
{
	struct dragon_rproc_pdata *local = rproc->priv;

	rproc_vq_interrupt(local->rproc, local->ipis[0].notifyid);
}

static void kick_pending_ipi(struct rproc *rproc)
{
	struct dragon_rproc_pdata *local = rproc->priv;
	int i;

	for (i = 0; i < MAX_NUM_VRINGS; i++) {
		/*Send swirq to firmware*/
		if (local->ipis[i].pending) {
		    ipi_send_single(local->irqs[HOST_SGI], 1);
			local->ipis[i].pending = false;
		}
	}
}

static int dragon_rproc_start(struct rproc *rproc)
{
	struct device *dev = rproc->dev.parent;
	struct arm_smccc_res res;
	
	dev_dbg(dev, "%s\n", __func__);
	INIT_WORK(&workqueue, handle_event);

	/* Wake up CPU1 by sending SMC call to EL3 */
	arm_smccc_smc(FN_RPROC_ON, rproc->bootaddr, 0, 0, 0, 0, 0, 0, &res);
	if (0 != res.a0) {
		dev_err(dev, "failed to boot remote core(%d)\n", -EINVAL);
		return -EINVAL;
	}

	/* Trigger pending kicks */
	kick_pending_ipi(rproc);

	return 0;
}

/* kick a firmware */
static void dragon_rproc_kick(struct rproc *rproc, int vqid)
{
	struct device *dev = rproc->dev.parent;
	struct dragon_rproc_pdata *local = rproc->priv;
	struct rproc_vdev *rvdev, *rvtmp;
	int i;

	dev_dbg(dev, "KICK Firmware to start send messages vqid %d\n", vqid);

	list_for_each_entry_safe(rvdev, rvtmp, &rproc->rvdevs, node) {
		for (i = 0; i < MAX_NUM_VRINGS; i++) {
			struct rproc_vring *rvring = &rvdev->vring[i];

			/*Send swirq to firmware*/
			if (rvring->notifyid == vqid) {
				local->ipis[i].notifyid = vqid;
				/*
				 * As we do not turn off CPU1 until start,
				 * we delay firmware kick
				 */
				if (rproc->state == RPROC_RUNNING)
					ipi_send_single(local->irqs[REMOTE_SGI], 1);
				else
					local->ipis[i].pending = true;
			}
		}
	}
}

/* power off the remote processor */
static int dragon_rproc_stop(struct rproc *rproc)
{
	struct device *dev = rproc->dev.parent;
	struct arm_smccc_res res;

	dev_dbg(rproc->dev.parent, "%s\n", __func__);
	
	/* Stop CPU1 by sending SMC call to EL3 */
	arm_smccc_smc(FN_RPROC_OFF, 0, 0, 0, 0, 0, 0, 0, &res);
	if (0 != res.a0) {
		dev_err(dev, "failed to stop remote core(%d)\n", -EINVAL);
	       	return -EINVAL;
	}

	return 0;
}

static int dragon_parse_fw(struct rproc *rproc, const struct firmware *fw)
{
	int num_mems, i, ret;
	struct device *dev = rproc->dev.parent;
	struct device_node *np = dev->of_node;
	struct rproc_mem_entry *mem;

	num_mems = of_count_phandle_with_args(np, "memory-region", NULL);
	if (num_mems <= 0)
		return 0;
	for (i = 0; i < num_mems; i++) {
		struct device_node *node;
		struct reserved_mem *rmem;

		node = of_parse_phandle(np, "memory-region", i);
		rmem = of_reserved_mem_lookup(node);
		if (!rmem) {
			dev_err(dev, "unable to acquire memory-region\n");
			return -EINVAL;
		}
		if (strstr(node->name, "vdev") &&
			strstr(node->name, "buffer")) {
			/* Register DMA region */
			mem = rproc_mem_entry_init(dev, NULL,
						   (dma_addr_t)rmem->base,
						   rmem->size, rmem->base,
						   NULL, NULL,
						   node->name);
			if (!mem) {
				dev_err(dev,
					"unable to initialize memory-region %s \n",
					node->name);
				return -ENOMEM;
			}
			rproc_add_carveout(rproc, mem);
		} else if (strstr(node->name, "vdev") &&
			   strstr(node->name, "vring")) {
			/* Register vring */
			mem = rproc_mem_entry_init(dev, NULL,
						   (dma_addr_t)rmem->base,
						   rmem->size, rmem->base,
						   NULL, NULL,
						   node->name);
			mem->va = devm_ioremap_wc(dev, rmem->base, rmem->size);
			if (!mem->va)
				return -ENOMEM;
			if (!mem) {
				dev_err(dev,
					"unable to initialize memory-region %s\n",
					node->name);
				return -ENOMEM;
			}
			rproc_add_carveout(rproc, mem);
		} else {
			mem = rproc_of_resm_mem_entry_init(dev, i,
							rmem->size,
							rmem->base,
							node->name);
			if (!mem) {
				dev_err(dev,
					"unable to initialize memory-region %s \n",
					node->name);
				return -ENOMEM;
			}
			mem->va = devm_ioremap_wc(dev, rmem->base, rmem->size);
			if (!mem->va)
				return -ENOMEM;

			rproc_add_carveout(rproc, mem);
		}
	}

	ret = rproc_elf_load_rsc_table(rproc, fw);
	if (ret == -EINVAL)
		ret = 0;
	return ret;
}

static struct rproc_ops dragon_rproc_ops = {
	.start		= dragon_rproc_start,
	.stop		= dragon_rproc_stop,
	.load		= rproc_elf_load_segments,
	.parse_fw	= dragon_parse_fw,
	.find_loaded_rsc_table = rproc_elf_find_loaded_rsc_table,
	.get_boot_addr	= rproc_elf_get_boot_addr,
	.kick		= dragon_rproc_kick,
};

static int dragon_remoteproc_probe(struct platform_device *pdev)
{
	int ret = 0;
	int i, virq;
	struct dragon_rproc_pdata *local;
	/*
	 * IRQ related structures are used for the following:
	 * for each SGI interrupt ensure its mapped by GIC IRQ domain
	 * and that each corresponding linux IRQ for the HW IRQ has
	 * a handler for when receiving an interrupt from the remote
	 * processor.
	 */
	struct irq_domain *domain;
	struct irq_desc *ipi_desc;
	struct irq_fwspec sgi_fwspec;
	struct device_node *interrupt_parent, *node = (&pdev->dev)->of_node;

	rproc = rproc_alloc(&pdev->dev, dev_name(&pdev->dev),
			    &dragon_rproc_ops, NULL,
		sizeof(struct dragon_rproc_pdata));
	if (!rproc) {
		dev_err(&pdev->dev, "rproc allocation failed\n");
		ret = -ENOMEM;
		return ret;
	}
	local = rproc->priv;
	local->rproc = rproc;

	platform_set_drvdata(pdev, rproc);

	ret = dma_set_coherent_mask(&pdev->dev, DMA_BIT_MASK(32));
	if (ret) {
		dev_err(&pdev->dev, "dma_set_coherent_mask: %d\n", ret);
		goto dma_mask_fault;
	}

	/* validate SGIs */
	if (n_vring_sgis != MAX_NUM_VRINGS) {
		dev_err(&pdev->dev, "invalid number of SGIs provided.\n");
		return -EINVAL;
	}

	/* Find GIC controller to map SGIs. */
	interrupt_parent = of_irq_find_parent(node);
	if (!interrupt_parent) {
		dev_err(&pdev->dev, "invalid phandle for interrupt parent.\n");
		return -EINVAL;
	}

	/* Each SGI needs to be associated with GIC's IRQ domain. */
	domain = irq_find_host(interrupt_parent);

	/* Each mapping needs GIC domain when finding IRQ mapping. */
	sgi_fwspec.fwnode = domain->fwnode;

	/*
	 * When irq domain looks at mapping each arg is as follows:
	 * 1 args for: interrupt # (set later)
	 */
	sgi_fwspec.param_count = 1;

	/*
	 * For each SGI:
	 * Set HW IRQ.
	 * Get corresponding Linux IRQ.
	 * Associate a handler for remotproc driver.
	 * For the IRQ descriptor for each wire in handler for IRQ action.
	 *   (this comes into play when receiving HW IRQ)
	 * Save HW IRQ for later remoteproc handling.
	 */
	for (i = 0; i < MAX_NUM_VRINGS; i++) {
		/* Set SGI's hwirq */
		sgi_fwspec.param[0] = vring_sgis[i];
		virq = irq_create_fwspec_mapping(&sgi_fwspec);

		ret = request_percpu_irq(virq, dragon_remoteproc_interrupt, "vring0", local);
		if (ret < 0) {
			dev_err(&pdev->dev, "request percpu irq@%d failed\n", sgi_fwspec.param[0]);
		}

		/*
		 * The IPI descriptor relates Linux IRQ to HW IRQ and
		 * irqaction. The irqaction will point to the dragon_remoteproc_interrupt.
		 */
		ipi_desc = irq_to_desc(virq);
		ipi_desc->action = &action;
		irq_set_status_flags(virq, IRQ_HIDDEN);
		enable_percpu_irq(virq, 0);
		local->irqs[i] = virq;
	}

	rproc->auto_boot = autoboot;

	ret = rproc_add(local->rproc);
	if (ret) {
		dev_err(&pdev->dev, "rproc registration failed\n");
		goto dma_mask_fault;
	}

	return 0;

dma_mask_fault:
	rproc_free(rproc);

	return ret;
}

static int dragon_remoteproc_remove(struct platform_device *pdev)
{
	struct rproc *rproc = platform_get_drvdata(pdev);

	rproc_del(rproc);

	of_reserved_mem_device_release(&pdev->dev);

	rproc_free(rproc);

	return 0;
}

/* Match table for OF platform binding */
static const struct of_device_id dragon_remoteproc_match[] = {
	{ .compatible = "anlogic,dragon_remoteproc", },
	{ /* end of list */ },
};
MODULE_DEVICE_TABLE(of, dragon_remoteproc_match);

static struct platform_driver dragon_remoteproc_driver = {
	.probe = dragon_remoteproc_probe,
	.remove = dragon_remoteproc_remove,
	.driver = {
		.name = "dragon_remoteproc",
		.of_match_table = dragon_remoteproc_match,
	},
};
module_platform_driver(dragon_remoteproc_driver);

module_param_named(autoboot,  autoboot, bool, 0444);
MODULE_PARM_DESC(autoboot,
		 "enable | disable autoboot. (default: false)");
module_param_array(vring_sgis, int, &n_vring_sgis, 0);

MODULE_AUTHOR("chaoz <chao.zhang@anlogic.com>");
MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("Anlogic Dragon FPSoC remote processor control driver");
