// SPDX-License-Identifier: GPL-2.0-only

/*
 * Copyright (C) 2024 Anlogic, Inc.
 * Based on uio_pdrv_genirq.c
 */

#include <linux/platform_device.h>
#include <linux/uio_driver.h>
#include <linux/spinlock.h>
#include <linux/bitops.h>
#include <linux/module.h>
#include <linux/dma-mapping.h>
#include <linux/irq.h>
#include <linux/of.h>

struct al_uio_gen_mem {
	phys_addr_t addr;
	void *vaddr;
	resource_size_t size;
};

struct al_uio_gen_priv {
	struct platform_device *pdev;
	struct uio_info *uioinfo;
	struct al_uio_gen_mem mem[MAX_UIO_MAPS];
	spinlock_t lock;
	unsigned long flags;
};

static irqreturn_t al_uio_gen_handler(int irq, struct uio_info *dev_info)
{
	struct al_uio_gen_priv *priv = dev_info->priv;

	spin_lock(&priv->lock);
	if (!test_and_set_bit(0, &priv->flags))
		disable_irq_nosync(irq);
	spin_unlock(&priv->lock);

	return IRQ_HANDLED;
}

static int al_uio_gen_irqcontrol(struct uio_info *dev_info, s32 irq_on)
{
	struct al_uio_gen_priv *priv = dev_info->priv;
	unsigned long flags;

	spin_lock_irqsave(&priv->lock, flags);
	if (irq_on) {
		if (test_and_clear_bit(0, &priv->flags))
			enable_irq(dev_info->irq);
	} else {
		if (!test_and_set_bit(0, &priv->flags))
			disable_irq_nosync(dev_info->irq);
	}
	spin_unlock_irqrestore(&priv->lock, flags);

	return 0;
}

static void al_uio_free_dmem(struct al_uio_gen_priv *priv)
{
	int i = 0;

	for (i = 0; i < MAX_UIO_MAPS && priv->mem[i].size > 0; i++) {
		dma_free_coherent(&priv->pdev->dev, priv->mem[i].size,
				  priv->mem[i].vaddr, priv->mem[i].addr);
		priv->mem[i].size = 0;
	}
}

static int al_uio_gen_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	struct uio_info *uioinfo;
	struct uio_mem *uiomem;
	struct al_uio_gen_priv *priv;
	int ret = -EINVAL;
	int i, cnt;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (IS_ERR_OR_NULL(priv))
		return -ENOMEM;

	uioinfo = devm_kzalloc(dev, sizeof(*uioinfo), GFP_KERNEL);
	if (!uioinfo)
		return -ENOMEM;

	uioinfo->name = of_get_property(np, "al,uioname", NULL);
	if (!uioinfo->name)
		uioinfo->name = dev_name(dev);
	if (uioinfo->name)
		uioinfo->name = devm_kstrdup(dev, uioinfo->name, GFP_KERNEL);
	uioinfo->version = "0.1";
	uioinfo->priv = priv;
	uioinfo->handler = al_uio_gen_handler;
	uioinfo->irqcontrol = al_uio_gen_irqcontrol;

	priv->uioinfo = uioinfo;
	priv->pdev = pdev;
	priv->flags = 0;
	spin_lock_init(&priv->lock);
	platform_set_drvdata(pdev, priv);

	dma_set_mask_and_coherent(dev, DMA_BIT_MASK(32));

	ret = platform_get_irq_optional(pdev, 0);
	if (ret >= 0) {
		struct irq_data *irq_data = irq_get_irq_data(ret);

		uioinfo->irq = ret;
		if (irq_data &&
		    irqd_get_trigger_type(irq_data) & IRQ_TYPE_LEVEL_MASK) {
			irq_set_status_flags(uioinfo->irq, IRQ_DISABLE_UNLAZY);
		}
	}

	cnt = of_property_count_u32_elems(np, "al,dmem");
	if (cnt < 0)
		cnt = 0;

	if ((cnt + pdev->num_resources) > MAX_UIO_MAPS) {
		dev_err(dev, "too many memory regions\n");
		return -EINVAL;
	}

	uiomem = &uioinfo->mem[0];
	for (i = 0; i < pdev->num_resources; ++i) {
		struct resource *r = &pdev->resource[i];

		if (r->flags != IORESOURCE_MEM)
			continue;

		uiomem->memtype = UIO_MEM_PHYS;
		uiomem->addr = r->start;
		uiomem->size = resource_size(r);
		++uiomem;
	}

	for (i = 0; i < cnt; ++i) {
		u32 val;

		ret = of_property_read_u32_index(np, "al,dmem", i, &val);
		if (ret < 0) {
			dev_err(dev, "get dmem %d fail\n", i);
			goto free_dmem;
		}

		priv->mem[i].vaddr = dma_alloc_coherent(dev, val,
				(dma_addr_t *)&priv->mem[i].addr, GFP_KERNEL);
		if (!priv->mem[i].vaddr) {
			dev_err(dev, "alloc dmem %d fail\n", i);
			ret = -ENOMEM;
			goto free_dmem;
		}
		uiomem->memtype = UIO_MEM_PHYS;
		uiomem->addr = priv->mem[i].addr;
		uiomem->size = val;
		priv->mem[i].size = val;
		++uiomem;
	}

	ret = devm_uio_register_device(dev, uioinfo);
	if (ret) {
		dev_err(dev, "register uio device fail\n");
		goto free_dmem;
	}

	if (uioinfo->name)
		pr_info("anlogic uio-gen: uio device %s added\n", uioinfo->name);
	return 0;

free_dmem:
	al_uio_free_dmem(priv);
	return ret;
}

static int al_uio_gen_remove(struct platform_device *pdev)
{
	struct al_uio_gen_priv *priv = platform_get_drvdata(pdev);

	al_uio_free_dmem(priv);
	return 0;
}

static const struct of_device_id al_uio_gen_of_match[] = {
	{ .compatible = "anlogic,uio-gen", },
	{},
};

MODULE_DEVICE_TABLE(of, al_uio_gen_of_match);

static struct platform_driver al_uio_gen_driver = {
	.probe = al_uio_gen_probe,
	.remove = al_uio_gen_remove,
	.driver = {
		.name = "anlogic,uio-gen",
		.of_match_table = of_match_ptr(al_uio_gen_of_match),
	},
};

module_platform_driver(al_uio_gen_driver);

MODULE_AUTHOR("fushan.zeng<fushan.zeng@anlogic.com>");
MODULE_DESCRIPTION("Anlogic uio generic driver");
MODULE_LICENSE("GPL");
