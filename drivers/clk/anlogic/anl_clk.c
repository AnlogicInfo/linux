// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2024 Anlogic, Inc.
 */

#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/device.h>
#include <linux/reset-controller.h>

#include "anl_clk.h"
#include "anl_reset.h"

void anl_init_divtbl(struct clk_div_table *tbl, uint cnt)
{
	uint i;

	for (i = 0; i < cnt; i++) {
		tbl[i].val = i;
		tbl[i].div = i+1;
	}
}
EXPORT_SYMBOL(anl_init_divtbl);

struct anl_crp {
	spinlock_t lock;
	const struct anl_crp_desc *desc;
	struct anl_reset arst;
};

static void anl_crp_release(struct device *dev, void *res)
{
	struct anl_crp *crp = res;
	const struct anl_crp_desc *desc = crp->desc;
	struct anl_clk **crp_clks = desc->crp_clks;
	int i;

	if (desc->arstbl && desc->num_resets > 0)
		reset_controller_unregister(&crp->arst.rcdev);

	if (desc->hw_clks && desc->hw_clks->num > 0)
		of_clk_del_provider(dev->of_node);

	if (desc->crp_clks)
		for (i = 0; i < crp->desc->num_crp_clks; i++)
			clk_hw_unregister(&crp_clks[i]->hw);
}

int anl_crp_probe(struct device *dev, void __iomem *reg,
		  const struct anl_crp_desc *desc)
{
	struct anl_crp *crp;
	struct anl_clk **crp_clks;
	struct anl_reset *arst;
	int i = 0;
	int ret;

	if (!dev)
		return -EINVAL;

	crp = devres_alloc(anl_crp_release, sizeof(*crp), GFP_KERNEL);
	if (!crp)
		return -ENOMEM;

	crp->desc = desc;
	spin_lock_init(&crp->lock);

	crp_clks = desc->crp_clks;
	if (desc->crp_clks) {
		for (i = 0; i < desc->num_crp_clks; i++) {
			if (!crp_clks[i])
				continue;

			crp_clks[i]->base = reg;
			crp_clks[i]->lock = &crp->lock;
			ret = clk_hw_register(dev, &crp_clks[i]->hw);
			if (ret) {
				pr_err("Couldn't register clock %d - %s\n", i,
				       crp_clks[i]->hw.init->name);
				goto unreg_clk;
			}
			clk_hw_set_rate_range(&crp_clks[i]->hw, crp_clks[i]->min_rate,
					      crp_clks[i]->max_rate);
		}
	}

	if (desc->hw_clks && desc->hw_clks->num > 0) {
		ret = of_clk_add_hw_provider(dev->of_node, of_clk_hw_onecell_get,
					     desc->hw_clks);
		if (ret)
			goto unreg_clk;
	}

	arst = &crp->arst;
	if (desc->arstbl && desc->num_resets > 0) {
		arst->rcdev.owner = dev ? dev->driver->owner : THIS_MODULE;
		arst->rcdev.of_node = dev->of_node;
		arst->rcdev.ops = &anl_reset_ops;
		arst->arstbl = desc->arstbl;
		arst->rcdev.nr_resets = desc->num_resets;
		arst->base = reg;
		arst->lock = &crp->lock;

		ret = reset_controller_register(&arst->rcdev);
		if (ret)
			goto del_provider;
	}

	devres_add(dev, crp);
	return 0;

del_provider:
	if (desc->hw_clks && desc->hw_clks->num > 0)
		of_clk_del_provider(dev->of_node);
unreg_clk:
	while (--i >= 0) {
		if (!crp_clks[i])
			continue;
		clk_hw_unregister(&crp_clks[i]->hw);
	}
	devres_free(crp);
	return ret;
}
EXPORT_SYMBOL(anl_crp_probe);
