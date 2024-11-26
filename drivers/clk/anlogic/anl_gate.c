// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2024 Anlogic, Inc.
 */

#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/device.h>
#include <linux/io.h>

#include "anl_clk.h"
#include "anl_gate.h"

static void anl_gate_endisable(struct clk_hw *hw, int enable)
{
	struct anl_clk *clk = hw_to_anl_clk(hw);
	struct anl_gate *gate = (struct anl_gate *)clk;
	unsigned long flags;
	u32 val;
	int set;

	spin_lock_irqsave(clk->lock, flags);

	set = gate->flags & CLK_GATE_SET_TO_DISABLE ? 1 : 0;
	set ^= enable;
	if (gate->flags & CLK_GATE_HIWORD_MASK) {
		val = BIT(gate->bitidx + 16);
		if (set)
			val |= BIT(gate->bitidx);
	} else {
		val = readl(clk->base + clk->reg);

		if (set)
			val |= BIT(gate->bitidx);
		else
			val &= ~BIT(gate->bitidx);
	}

	writel(val, clk->base + clk->reg);

	spin_unlock_irqrestore(clk->lock, flags);
}

static int anl_gate_enable(struct clk_hw *hw)
{
	anl_gate_endisable(hw, 1);

	return 0;
}

static void anl_gate_disable(struct clk_hw *hw)
{
	anl_gate_endisable(hw, 0);
}

static int anl_gate_is_enabled(struct clk_hw *hw)
{
	struct anl_clk *clk = hw_to_anl_clk(hw);
	struct anl_gate *gate = (struct anl_gate *)clk;
	u32 val;

	val = readl(clk->base + clk->reg);
	if (gate->flags & CLK_GATE_SET_TO_DISABLE)
		val ^= BIT(gate->bitidx);
	val &= BIT(gate->bitidx);

	return val != 0;
}

const struct clk_ops anl_gate_ops = {
	.enable = anl_gate_enable,
	.disable = anl_gate_disable,
	.is_enabled = anl_gate_is_enabled,
};
EXPORT_SYMBOL(anl_gate_ops);
