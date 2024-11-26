// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2024 Anlogic, Inc.
 */

#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/device.h>
#include <linux/io.h>

#include "anl_clk.h"
#include "anl_sel.h"

static u8 anl_sel_get_parent(struct clk_hw *hw)
{
	struct anl_clk *clk = hw_to_anl_clk(hw);
	struct anl_sel *sel = (struct anl_sel *)clk;
	u32 val;

	val = (readl(clk->base + clk->reg) >> sel->shift);
	val &= sel->mask;

	return clk_mux_val_to_index(hw, sel->table, sel->flags, val);
}

static int anl_sel_set_parent(struct clk_hw *hw, u8 index)
{
	struct anl_clk *clk = hw_to_anl_clk(hw);
	struct anl_sel *sel = (struct anl_sel *)clk;
	u32 val = clk_mux_index_to_val(sel->table, sel->flags, index);
	unsigned long flags = 0;
	u32 reg;

	spin_lock_irqsave(clk->lock, flags);

	if (sel->flags & CLK_MUX_HIWORD_MASK) {
		reg = sel->mask << (sel->shift + 16);
	} else {
		reg = readl(clk->base + clk->reg);
		reg &= ~(sel->mask << sel->shift);
	}
	val = val << sel->shift;
	reg |= val;
	writel(reg, clk->base + clk->reg);

	spin_unlock_irqrestore(clk->lock, flags);

	return 0;
}

static int anl_sel_determine_rate(struct clk_hw *hw,
				  struct clk_rate_request *req)
{
	struct anl_clk *clk = hw_to_anl_clk(hw);
	struct anl_sel *sel = (struct anl_sel *)clk;

	return clk_mux_determine_rate_flags(hw, req, sel->flags);
}

const struct clk_ops anl_sel_ro_ops = {
	.get_parent = anl_sel_get_parent,
};
EXPORT_SYMBOL_GPL(anl_sel_ro_ops);

const struct clk_ops anl_sel_ops = {
	.get_parent = anl_sel_get_parent,
	.set_parent = anl_sel_set_parent,
	.determine_rate = anl_sel_determine_rate,
};
EXPORT_SYMBOL_GPL(anl_sel_ops);
