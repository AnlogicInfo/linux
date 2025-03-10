// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2025 Anlogic, Inc.
 */
#include <linux/device.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include "anl_top_reg.h"

enum anl_soc_id {
	ANL_SOC_DR1X90,
	ANL_SOC_MAX
};

struct anl_top_data {
	int chip;
};

static DEFINE_SPINLOCK(top_lock);
static void __iomem *top_base;
static struct anl_top_data dr1x90_top_data = {
	.chip = ANL_SOC_DR1X90,
};

static inline void top_writel(u32 value, u32 off)
{
	if (!top_base) {
		pr_warn("anl write top reg 0x%x fail\n", off);
		return;
	}

	writel(value, top_base + off);
}

static inline u32 top_readl(u32 off)
{
	if (!top_base) {
		pr_warn("anl write top reg 0x%x fail\n", off);
		return 0;
	}

	return readl(top_base + off);
}

void anl_wdt_pause(int id, int pause)
{
	unsigned long flags;
	u32 val;

	spin_lock_irqsave(&top_lock, flags);
	val = top_readl(SYSCTRL_NS_CFG_CTRL_WDT);
	if (pause)
		top_writel(val | (1 << id), SYSCTRL_NS_CFG_CTRL_WDT);
	else
		top_writel(val & ~(1 << id), SYSCTRL_NS_CFG_CTRL_WDT);
	spin_unlock_irqrestore(&top_lock, flags);
}
EXPORT_SYMBOL(anl_wdt_pause);

void anl_spi_set_slave(int id, int slave)
{
	unsigned long flags;
	u32 val;

	spin_lock_irqsave(&top_lock, flags);
	val = top_readl(SYSCTRL_NS_CFG_CTRL_SPI);
	if (slave)
		top_writel(val | (1 << (id * 8)), SYSCTRL_NS_CFG_CTRL_SPI);
	else
		top_writel(val & ~(1 << (id * 8)), SYSCTRL_NS_CFG_CTRL_SPI);
	spin_unlock_irqrestore(&top_lock, flags);
}
EXPORT_SYMBOL(anl_spi_set_slave);

static int anl_top_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	void __iomem *reg;
	int ret = 0;

	reg = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(reg)) {
		dev_err(dev, "top init fail");
		return PTR_ERR(reg);
	}
	top_base = reg;

	return ret;
}

static const struct of_device_id anl_top_ids[] = {
	{ .compatible = "anlogic,dr1x90-topcfg", .data = &dr1x90_top_data },
	{ }
};
MODULE_DEVICE_TABLE(of, anl_top_ids);

static struct platform_driver anl_top_driver = {
	.probe	= anl_top_probe,
	.driver	= {
		.name	= "anl-topcfg",
		.of_match_table	= anl_top_ids,
	},
};

static int __init anl_top_driver_init(void)
{
	return platform_driver_register(&anl_top_driver);
}
arch_initcall(anl_top_driver_init);

MODULE_AUTHOR("fushan.zeng<fushan.zeng@anlogic.com>");
MODULE_DESCRIPTION("anlogic top config");
MODULE_LICENSE("GPL");
