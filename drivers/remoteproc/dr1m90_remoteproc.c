// SPDX-License-Identifier: GPL-2.0
/*
 * DR1M90 Remote Processor driver
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
#define FN_RPROC_BASE	(0xFF000000)
#define FN_RPROC_OFF	(FN_RPROC_BASE + 21)
#define FN_RPROC_ON	(FN_RPROC_BASE + 22)

#define MAX_NUM_VRINGS 2
#define NOTIFYID_ANY (-1)

#define REMOTE_SGI		0
#define HOST_SGI		1

static int vring_sgis[MAX_NUM_VRINGS] = {14, 15};
/* count number of SGIs passed via command line if applicable. */
static int n_vring_sgis = MAX_NUM_VRINGS;

struct dragon_rproc_pdata;

struct rproc_irq {
	struct work_struct work;
	struct dragon_rproc_pdata *local;
	u32 notifyid;
	int irq[2];
	int idx;
};

#define SLVTYPE_APU	0x00000000
#define SLVTYPE_RPU	0x00010000
#define SLVID_MASK	0x0000FFFF
#define SLVTYPE_MASK	0xFFFF0000

#if IS_ENABLED(CONFIG_ARCH_RV64I)
#define MASTER_TYPE	0x00010000
#else
#define MASTER_TYPE	0x00000000
#endif

struct dragon_rproc_pdata {
	struct rproc *rproc;
	int slave_id;
	int master_id;
	struct work_struct work;
	int start;
	uint32_t openamp:1;
	int irq[MAX_NUM_VRINGS];
	struct rproc_irq rpirq[MAX_NUM_VRINGS];
	struct dragon_rproc_pdata * __percpu *devid;
};

static bool autoboot __read_mostly;

static bool slave_same_type(struct dragon_rproc_pdata *local)
{
	return ((local->slave_id & SLVTYPE_MASK) == (local->master_id & SLVTYPE_MASK));
}

static irqreturn_t dragon_remoteproc_interrupt(int irq, void *dev_id)
{
	struct dragon_rproc_pdata *local = *(void **)dev_id;
	struct rproc_irq *rpirq;
	int ret = IRQ_NONE;
	int i;

	for (i = 0; i < MAX_NUM_VRINGS; i++) {
		rpirq = &local->rpirq[i];
		if (irq != rpirq->irq[HOST_SGI])
			continue;
		schedule_work(&rpirq->work);
		ret = IRQ_HANDLED;
	}

	return ret;
}

static void handle_event(struct work_struct *work)
{
	struct rproc_irq *rpirq = container_of(work, struct rproc_irq, work);
	struct dragon_rproc_pdata *local = rpirq->local;

	rproc_vq_interrupt(local->rproc, rpirq->notifyid);
}

/* kick a firmware */
static void dragon_rproc_kick(struct rproc *rproc, int vqid)
{
	struct device *dev = rproc->dev.parent;
	struct dragon_rproc_pdata *local = rproc->priv;
	struct rproc_irq *rpirq;
	int i;

	dev_dbg(dev, "KICK Firmware to start send messages vqid %d\n", vqid);

	for (i = 0; i < MAX_NUM_VRINGS; i++) {
		rpirq = &local->rpirq[i];

		/*Send swirq to firmware*/
		if (rpirq->notifyid == vqid)
			ipi_send_single(rpirq->irq[REMOTE_SGI], local->slave_id & SLVID_MASK);
	}
}

static int dragon_enable_irq(struct dragon_rproc_pdata *local, int enable)
{
	int irq = local->irq[HOST_SGI];

	if (!irq)
		return -EINVAL;

	if (enable)
		enable_percpu_irq(irq, 0);
	else
		disable_percpu_irq(irq);

	return 0;
}

static int dragon_init_ipis(struct platform_device *pdev, struct dragon_rproc_pdata *local)
{
	int i, virq;
	struct irq_domain *domain;
	struct irq_fwspec sgi_fwspec;
	struct device_node *interrupt_parent, *node = pdev->dev.of_node;
	int ret = 0;

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
	of_node_put(interrupt_parent);
	if (!domain) {
		dev_err(&pdev->dev, "invalid phandle for interrupt domain.\n");
		return -EINVAL;
	}

	/* Each mapping needs GIC domain when finding IRQ mapping. */
	sgi_fwspec.fwnode = domain->fwnode;

	/*
	 * When irq domain looks at mapping each arg is as follows:
	 * 1 args for: interrupt # (set later)
	 */
	sgi_fwspec.param_count = 1;

	local->devid = alloc_percpu(struct dragon_rproc_pdata *);
	if (!local->devid)
		return -ENOMEM;

	for_each_possible_cpu(i)
		*per_cpu_ptr(local->devid, i) = local;

	for (i = 0; i < n_vring_sgis; i++) {
		sgi_fwspec.param[0] = vring_sgis[i];
		virq = irq_create_fwspec_mapping(&sgi_fwspec);
		irq_set_status_flags(virq, IRQ_HIDDEN);
		local->irq[i] = virq;
	}

	virq = local->irq[HOST_SGI];
	ret = request_percpu_irq(virq, dragon_remoteproc_interrupt, "vring0", local->devid);
	if (ret < 0) {
		dev_err(&pdev->dev, "request percpu irq@%d failed\n", virq);
		virq = local->irq[HOST_SGI] = 0;
	}

	for (i = 0; i < MAX_NUM_VRINGS; i++) {
		INIT_WORK(&local->rpirq[i].work, handle_event);
		local->rpirq[i].local = local;
		local->rpirq[i].idx = i;
		memcpy(&local->rpirq[i].irq, &local->irq, sizeof(local->rpirq[i].irq));
	}

	return 0;
}

static int dragon_deinit_ipis(struct dragon_rproc_pdata *local)
{
	int irq = local->irq[HOST_SGI];

	if (irq)
		free_percpu_irq(irq, local->devid);
	free_percpu(local->devid);

	return 0;
}

static void dragon_rproc_do_start(struct work_struct *work)
{
	struct dragon_rproc_pdata *local = container_of(work, struct dragon_rproc_pdata, work);
	struct rproc *rproc = local->rproc;
	struct device *dev = rproc->dev.parent;
	struct arm_smccc_res res;

	if (local->start) {
		if (local->openamp)
			dragon_enable_irq(local, 1);

		/* Wake up CPU1 by sending SMC call to EL3 */
		arm_smccc_smc(FN_RPROC_ON, rproc->bootaddr, local->slave_id, 0, 0, 0, 0, 0, &res);
		if (res.a0) {
			dev_err(dev, "failed to boot remote core(%d)\n", -EINVAL);
			goto err_disable_irq;
		}

		return;
	}

	/* Stop CPU1 by sending SMC call to EL3 */
	arm_smccc_smc(FN_RPROC_OFF, local->slave_id, 0, 0, 0, 0, 0, 0, &res);
	if (res.a0)
		dev_err(dev, "failed to stop remote core(%d)\n", -EINVAL);

err_disable_irq:
	if (local->openamp)
		dragon_enable_irq(local, 0);
}

static int dragon_rproc_start(struct rproc *rproc)
{
	struct device *dev = rproc->dev.parent;
	struct dragon_rproc_pdata *local = rproc->priv;
	struct rproc_vdev *rvdev;
	struct rproc_vring *rvring;
	struct rproc_irq *rpirq;
	int i;

	if (rproc->nb_vdev > local->openamp) {
		dev_err(dev, "image not supported\n");
		return -EINVAL;
	}

	list_for_each_entry(rvdev, &rproc->rvdevs, node) {
		for (i = 0; i < MAX_NUM_VRINGS; i++) {
			rpirq = &local->rpirq[i];
			rvring = &rvdev->vring[rpirq->idx];
			rpirq->notifyid = rvring->notifyid;
		}
	}

	if (slave_same_type(local)) {
		if (cpu_online(local->slave_id & SLVID_MASK) ||
		    !cpu_online(local->master_id & SLVID_MASK)) {
			dev_err(dev, "invalid master or slave specified");
			return -EINVAL;
		}
	}

	local->start = 1;
	schedule_work_on(local->master_id & SLVID_MASK, &local->work);
	flush_work(&local->work);

	return 0;
}

/* power off the remote processor */
static int dragon_rproc_stop(struct rproc *rproc)
{
	struct dragon_rproc_pdata *local = rproc->priv;

	local->start = 0;
	schedule_work_on(local->master_id & SLVID_MASK, &local->work);
	flush_work(&local->work);

	return 0;
}

static struct resource_table *dragon_find_loaded_rsc_table(struct rproc *rproc,
							   const struct firmware *fw)
{
	struct resource_table *rsc = NULL;

	rsc = rproc_elf_find_loaded_rsc_table(rproc, fw);

	return rsc;
}

static int dragon_parse_fw(struct rproc *rproc, const struct firmware *fw)
{
	int num_mems, i, ret = 0;
	struct device *dev = rproc->dev.parent;
	struct device_node *np = dev->of_node;
	struct rproc_mem_entry *mem;

	num_mems = of_count_phandle_with_args(np, "memory-region", NULL);
	if (num_mems <= 0)
		return -EINVAL;

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
				dev_err(dev, "unable to initialize memory-region %s\n",
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
			if (!mem) {
				dev_err(dev, "unable to initialize memory-region %s\n",
					node->name);
				return -ENOMEM;
			}

			mem->va = devm_ioremap_wc(dev, rmem->base, rmem->size);
			if (!mem->va)
				return -ENOMEM;

			rproc_add_carveout(rproc, mem);
		} else {
			mem = rproc_of_resm_mem_entry_init(dev, i,
							rmem->size,
							rmem->base,
							node->name);
			if (!mem) {
				dev_err(dev, "unable to initialize memory-region %s\n",
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
	.find_loaded_rsc_table = dragon_find_loaded_rsc_table,
	.get_boot_addr	= rproc_elf_get_boot_addr,
	.kick		= dragon_rproc_kick,
};

static int dragon_remoteproc_probe(struct platform_device *pdev)
{
	int ret = 0;
	struct dragon_rproc_pdata *local;
	struct rproc *rproc;
	struct device_node *node = pdev->dev.of_node;

	ret = dma_set_coherent_mask(&pdev->dev, DMA_BIT_MASK(32));
	if (ret) {
		dev_err(&pdev->dev, "dma_set_coherent_mask: %d\n", ret);
		return -EINVAL;
	}

	rproc = rproc_alloc(&pdev->dev, dev_name(&pdev->dev),
			    &dragon_rproc_ops, NULL,
			    sizeof(struct dragon_rproc_pdata));
	if (!rproc) {
		dev_err(&pdev->dev, "rproc allocation failed\n");
		return -ENOMEM;
	}

	local = rproc->priv;
	local->rproc = rproc;
	platform_set_drvdata(pdev, rproc);
	rproc->auto_boot = autoboot;
	local->slave_id = (MASTER_TYPE | (SLVID_MASK & 1));
	local->master_id = (MASTER_TYPE | (SLVID_MASK & 0));
	INIT_WORK(&local->work, dragon_rproc_do_start);

	if (of_property_read_bool(node, "anlogic,no-openamp"))
		local->openamp = 0;
	else
		local->openamp = 1;

	if (local->openamp) {
		ret = dragon_init_ipis(pdev, local);
		if (ret < 0)
			goto err_free_rproc;
	}

	ret = rproc_add(local->rproc);
	if (ret) {
		dev_err(&pdev->dev, "rproc registration failed\n");
		goto err_deinit_ipis;
	}

	dev_info(&pdev->dev, "anlogic rproc %ssupport openamp\n",
		 local->openamp ? "" : "not ");
	return 0;

err_deinit_ipis:
	if (local->openamp)
		dragon_deinit_ipis(local);
err_free_rproc:
	rproc_free(rproc);

	return ret;
}

static int dragon_remoteproc_remove(struct platform_device *pdev)
{
	struct rproc *rproc = platform_get_drvdata(pdev);
	struct dragon_rproc_pdata *local = rproc->priv;

	rproc_del(rproc);

	of_reserved_mem_device_release(&pdev->dev);

	if (local->openamp)
		dragon_deinit_ipis(local);

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
MODULE_PARM_DESC(autoboot, "enable | disable autoboot. (default: false)");
module_param_array(vring_sgis, int, &n_vring_sgis, 0);

MODULE_AUTHOR("fushan.zeng <fushan.zeng@anlogic.com>");
MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("Anlogic Dragon FPSoC remote processor control driver");
