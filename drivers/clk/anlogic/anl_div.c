// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2024 Anlogic, Inc.
 */

#include <linux/version.h>
#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/device.h>
#include <linux/io.h>

#include "anl_clk.h"
#include "anl_div.h"

struct clk_div_table anl_divtbl_24[] = {
	{0xFFFFFF, 1},
	{0x555555, 2},
	{0x249249, 3},
	{0x111111, 4},
	{0x041041, 6},
	{0x010101, 8},
	{0x001001, 12},
	{0x000001, 24},
	{0, 0}
};

struct clk_div_table anl_divtbl_32[] = {
	{0xFFFFFFFF, 1},
	{0x55555555, 2},
	{0x11111111, 4},
	{0x01010101, 8},
	{0x00010001, 16},
	{0x00000001, 32},
	{0, 0}
};

static int anl_div_init(struct clk_hw *hw)
{
	struct anl_clk *clk = hw_to_anl_clk(hw);
	struct anl_div *div = (struct anl_div *)clk;
	unsigned int val;

	val = (readl(clk->base + clk->reg) >> div->shift);
	val &= clk_div_mask(div->width);
	div->divval = val;

	return 0;
}

static unsigned long anl_div_recalc_rate(struct clk_hw *hw,
					 unsigned long parent_rate)
{
	struct anl_clk *clk = hw_to_anl_clk(hw);
	struct anl_div *div = (struct anl_div *)clk;
	unsigned int val;

	if (div->flags & CLK_DIVIDER_FIXED_FACTOR) {
		u64 rate;

		rate = (u64)parent_rate * div->width;
		do_div(rate, div->shift);
		return (unsigned long)rate;
	}

	WARN(div->width && !clk_div_mask(div->width),
	     "%s: div mask is zero\n", clk_hw_get_name(hw));

	val = (readl(clk->base + clk->reg) >> div->shift);
	val &= clk_div_mask(div->width);

	if (div->flags & CLK_DIVIDER_ZERO_GATING) {
		if (!val)
			return 0;
	}

	return divider_recalc_rate(hw, parent_rate, val, div->table,
				   div->flags, div->width);
}

static long anl_div_round_rate(struct clk_hw *hw, unsigned long rate,
			       unsigned long *prate)
{
	struct anl_clk *clk = hw_to_anl_clk(hw);
	struct anl_div *div = (struct anl_div *)clk;
	u64 res;
	u32 val;

	if (div->flags & CLK_DIVIDER_FIXED_FACTOR) {
		if (clk_hw_get_flags(hw) & CLK_SET_RATE_PARENT) {
			res = (u64)rate * div->shift;
			do_div(res, div->width);
			*prate = clk_hw_round_rate(clk_hw_get_parent(hw), res);
		}

		res = (u64)*prate * div->width;
		do_div(res, div->shift);
		return res;
	}

	if (div->flags & CLK_DIVIDER_READ_ONLY) {
		val = (readl(clk->base + clk->reg) >> div->shift);
		val &= clk_div_mask(div->width);

		if (div->flags & CLK_DIVIDER_ZERO_GATING) {
			if (!val)
				return 0;
		}

		return divider_ro_round_rate(hw, rate, prate, div->table,
					     div->width, div->flags,
					     val);
	}

	return divider_round_rate(hw, rate, prate, div->table,
				  div->width, div->flags);
}

#if KERNEL_VERSION(5, 14, 0) <= LINUX_VERSION_CODE
static int anl_div_determine_rate(struct clk_hw *hw,
				  struct clk_rate_request *req)
{
	struct anl_clk *clk = hw_to_anl_clk(hw);
	struct anl_div *div = (struct anl_div *)clk;
	u32 val;
	u64 res;
	int ret;

	if (div->flags & CLK_DIVIDER_FIXED_FACTOR) {
		if (clk_hw_get_flags(hw) & CLK_SET_RATE_PARENT) {
			struct clk_hw *parent = clk_hw_get_parent(hw);
			struct clk_rate_request parent_req;

			res = (u64)req->rate * div->shift;
			do_div(res, div->width);

			clk_hw_forward_rate_request(hw, req, parent,
						    &parent_req, res);
			ret = __clk_determine_rate(parent, &parent_req);
			if (ret)
				return ret;

			req->best_parent_rate = parent_req.rate;
		}

		return (req->best_parent_rate / div->shift) * div->width;
	}

	if (div->flags & CLK_DIVIDER_READ_ONLY) {
		val = (readl(clk->base + clk->reg) >> div->shift);
		val &= clk_div_mask(div->width);

		if (div->flags & CLK_DIVIDER_ZERO_GATING) {
			if (!val) {
				req->rate = 0;
				return 0;
			}
		}

		return divider_ro_determine_rate(hw, req, div->table,
						 div->width,
						 div->flags, val);
	}

	return divider_determine_rate(hw, req, div->table, div->width,
				      div->flags);
}
#endif

static int anl_div_set_rate(struct clk_hw *hw, unsigned long rate,
			    unsigned long parent_rate)
{
	struct anl_clk *clk = hw_to_anl_clk(hw);
	struct anl_div *div = (struct anl_div *)clk;
	int value;
	unsigned long flags = 0;
	u32 val;

	if (div->flags & CLK_DIVIDER_FIXED_FACTOR)
		return 0;

	value = divider_get_val(rate, parent_rate, div->table,
				div->width, div->flags);
	if (value < 0)
		return value;

	spin_lock_irqsave(clk->lock, flags);

	if (div->flags & CLK_DIVIDER_HIWORD_MASK) {
		val = clk_div_mask(div->width) << (div->shift + 16);
	} else {
		val = readl(clk->base + clk->reg);
		val &= ~(clk_div_mask(div->width) << div->shift);
	}
	div->divval = value;
	if (!(div->flags & CLK_DIVIDER_ZERO_GATING) ||
	    __clk_get_enable_count(hw->clk)) {
		val |= (u32)value << div->shift;
		writel(val, clk->base + clk->reg);
	}

	spin_unlock_irqrestore(clk->lock, flags);

	return 0;
}

static void anl_div_endisable(struct clk_hw *hw, int enable)
{
	struct anl_clk *clk = hw_to_anl_clk(hw);
	struct anl_div *div = (struct anl_div *)clk;
	unsigned long flags;
	u32 val;

	spin_lock_irqsave(clk->lock, flags);

	if (div->flags & CLK_DIVIDER_ZERO_GATING) {
		val = readl(clk->base + clk->reg);
		val &= ~(clk_div_mask(div->width) << div->shift);
		if (enable)
			val |= ((u32)div->divval << div->shift);
		writel(val, clk->base + clk->reg);
	} else {
		int set;

		set = div->flags & CLK_DIVIDER_GATE_SETTODIS ? 1 : 0;
		set ^= enable;
		if (div->flags & CLK_DIVIDER_HIWORD_MASK) {
			val = BIT(div->en_bitidx + 16);
			if (set)
				val |= BIT(div->en_bitidx);
		} else {
			val = readl(clk->base + div->en_reg);

			if (set)
				val |= BIT(div->en_bitidx);
			else
				val &= ~BIT(div->en_bitidx);
		}

		writel(val, clk->base + div->en_reg);
	}

	spin_unlock_irqrestore(clk->lock, flags);
}

static int anl_div_enable(struct clk_hw *hw)
{
	anl_div_endisable(hw, 1);

	return 0;
}

static void anl_div_disable(struct clk_hw *hw)
{
	anl_div_endisable(hw, 0);
}

static int anl_div_is_enabled(struct clk_hw *hw)
{
	struct anl_clk *clk = hw_to_anl_clk(hw);
	struct anl_div *div = (struct anl_div *)clk;
	u32 val;

	if (div->flags & CLK_DIVIDER_ZERO_GATING) {
		val = (readl(clk->base + clk->reg) >> div->shift);
		val &= clk_div_mask(div->width);

		return val != 0;
	}

	val = readl(clk->base + div->en_reg);
	if (div->flags & CLK_DIVIDER_GATE_SETTODIS)
		val ^= BIT(div->en_bitidx);
	val &= BIT(div->en_bitidx);

	return val != 0;
}

/****************************************************/
#define ANL_CALC_RATE_FRAC(p, t) DIV_ROUND_UP_ULL((u64)(p) * (t)->mul, (t)->div)

static inline struct clk_div_table_frac *
anl_get_div(struct anl_div *div, unsigned int val)
{
	struct clk_div_table_frac *tbl = ANL_FRAC_TABLE(div);

	while (tbl->div) {
		if (tbl->val == val)
			return tbl;
		tbl++;
	}

	return NULL;
}

static inline struct clk_div_table_frac *
anl_get_best_div(struct anl_div *div, unsigned long prate,
		 unsigned long rate)
{
	struct clk_div_table_frac *tbl = ANL_FRAC_TABLE(div);
	struct clk_div_table_frac *best_tbl = tbl;
	unsigned long best = ULONG_MAX, tmp;

	while (tbl->div) {
		tmp = ANL_CALC_RATE_FRAC(prate, tbl);
		if (tmp == rate)
			return tbl;
		else if (tmp > rate)
			tmp = tmp - rate;
		else
			tmp = rate - tmp;

		if (tmp < best) {
			best_tbl = tbl;
			best = tmp;
		}

		tbl++;
	}

	return best_tbl;
}

static unsigned long
anl_div_recalc_rate_frac(struct clk_hw *hw,
			 unsigned long parent_rate)
{
	struct anl_clk *clk = hw_to_anl_clk(hw);
	struct anl_div *div = (struct anl_div *)clk;
	struct clk_div_table_frac *tbl;
	unsigned int val;

	if (!(div->flags & CLK_DIVIDER_FRAC_TABLE)) {
		pr_err("%s: is not fraction table divider\n",
		       clk_hw_get_name(hw));
		return 0;
	}

	val = (readl(clk->base + clk->reg) >> div->shift);
	val &= clk_div_mask(div->width);
	tbl = anl_get_div(div, val);
	if (!tbl) {
		pr_err("%s: invalid divider value %u\n",
		       clk_hw_get_name(hw), val);
		return 0;
	}

	return ANL_CALC_RATE_FRAC(parent_rate, tbl);
}

static long anl_div_round_rate_frac(struct clk_hw *hw, unsigned long rate,
				    unsigned long *prate)
{
	struct anl_clk *clk = hw_to_anl_clk(hw);
	struct anl_div *div = (struct anl_div *)clk;
	struct clk_div_table_frac *tbl;
	u32 val;

	if (!(div->flags & CLK_DIVIDER_FRAC_TABLE)) {
		pr_err("%s: is not fraction table divider\n",
		       clk_hw_get_name(hw));
		return 0;
	}

	if (div->flags & CLK_DIVIDER_READ_ONLY) {
		val = (readl(clk->base + clk->reg) >> div->shift);
		val &= clk_div_mask(div->width);

		tbl = anl_get_div(div, val);
		if (!tbl) {
			pr_err("%s: invalid divider value %u\n",
			       clk_hw_get_name(hw), val);
			return 0;
		}
	} else
		tbl = anl_get_best_div(div, *prate, rate);

	return ANL_CALC_RATE_FRAC(*prate, tbl);
}

static int anl_div_determine_rate_frac(struct clk_hw *hw,
				       struct clk_rate_request *req)
{
	long rate = anl_div_round_rate_frac(hw, req->rate, &req->best_parent_rate);

	req->rate = rate;

	return 0;
}

static int anl_div_set_rate_frac(struct clk_hw *hw, unsigned long rate,
				 unsigned long parent_rate)
{
	struct anl_clk *clk = hw_to_anl_clk(hw);
	struct anl_div *div = (struct anl_div *)clk;
	struct clk_div_table_frac *tbl;
	int value;
	unsigned long flags = 0;
	u32 val;

	if (!(div->flags & CLK_DIVIDER_FRAC_TABLE)) {
		pr_err("%s: is not fraction table divider\n",
		       clk_hw_get_name(hw));
		return -EINVAL;
	}

	tbl = anl_get_best_div(div, parent_rate, rate);
	if (!tbl)
		return -EINVAL;

	value = tbl->val;

	spin_lock_irqsave(clk->lock, flags);

	if (div->flags & CLK_DIVIDER_HIWORD_MASK) {
		val = clk_div_mask(div->width) << (div->shift + 16);
	} else {
		val = readl(clk->base + clk->reg);
		val &= ~(clk_div_mask(div->width) << div->shift);
	}
	div->divval = value;
	if (!(div->flags & CLK_DIVIDER_ZERO_GATING) ||
	    __clk_get_enable_count(hw->clk)) {
		val |= (u32)value << div->shift;
		writel(val, clk->base + clk->reg);
	}

	spin_unlock_irqrestore(clk->lock, flags);

	return 0;
}

const struct clk_ops anl_div_ops = {
	.init		= anl_div_init,
	.recalc_rate	= anl_div_recalc_rate,
	.round_rate = anl_div_round_rate,
#if KERNEL_VERSION(5, 14, 0) <= LINUX_VERSION_CODE
	.determine_rate = anl_div_determine_rate,
#endif
	.set_rate = anl_div_set_rate,
};
EXPORT_SYMBOL(anl_div_ops);

const struct clk_ops anl_div_ro_ops = {
	.init		= anl_div_init,
	.recalc_rate	= anl_div_recalc_rate,
	.round_rate = anl_div_round_rate,
#if KERNEL_VERSION(5, 14, 0) <= LINUX_VERSION_CODE
	.determine_rate = anl_div_determine_rate,
#endif
};
EXPORT_SYMBOL(anl_div_ro_ops);

const struct clk_ops anl_div_en_ops = {
	.init		= anl_div_init,
	.recalc_rate	= anl_div_recalc_rate,
	.round_rate = anl_div_round_rate,
#if KERNEL_VERSION(5, 14, 0) <= LINUX_VERSION_CODE
	.determine_rate = anl_div_determine_rate,
#endif
	.set_rate = anl_div_set_rate,
	.enable = anl_div_enable,
	.disable = anl_div_disable,
	.is_enabled = anl_div_is_enabled,
};
EXPORT_SYMBOL(anl_div_en_ops);

const struct clk_ops anl_div_ro_en_ops = {
	.init		= anl_div_init,
	.recalc_rate	= anl_div_recalc_rate,
	.round_rate = anl_div_round_rate,
#if KERNEL_VERSION(5, 14, 0) <= LINUX_VERSION_CODE
	.determine_rate = anl_div_determine_rate,
#endif
	.enable = anl_div_enable,
	.disable = anl_div_disable,
	.is_enabled = anl_div_is_enabled,
};
EXPORT_SYMBOL(anl_div_ro_en_ops);

const struct clk_ops anl_div_ops_frac = {
	.init		= anl_div_init,
	.recalc_rate	= anl_div_recalc_rate_frac,
	.round_rate = anl_div_round_rate_frac,
	.determine_rate = anl_div_determine_rate_frac,
	.set_rate = anl_div_set_rate_frac,
};
EXPORT_SYMBOL(anl_div_ops_frac);

const struct clk_ops anl_div_ro_ops_frac = {
	.init		= anl_div_init,
	.recalc_rate	= anl_div_recalc_rate_frac,
	.round_rate = anl_div_round_rate_frac,
	.determine_rate = anl_div_determine_rate_frac,
};
EXPORT_SYMBOL(anl_div_ro_ops_frac);

const struct clk_ops anl_div_en_ops_frac = {
	.init		= anl_div_init,
	.recalc_rate	= anl_div_recalc_rate_frac,
	.round_rate = anl_div_round_rate_frac,
	.determine_rate = anl_div_determine_rate_frac,
	.set_rate = anl_div_set_rate_frac,
	.enable = anl_div_enable,
	.disable = anl_div_disable,
	.is_enabled = anl_div_is_enabled,
};
EXPORT_SYMBOL(anl_div_en_ops_frac);

const struct clk_ops anl_div_ro_en_ops_frac = {
	.init		= anl_div_init,
	.recalc_rate	= anl_div_recalc_rate_frac,
	.round_rate = anl_div_round_rate_frac,
	.determine_rate = anl_div_determine_rate_frac,
	.enable = anl_div_enable,
	.disable = anl_div_disable,
	.is_enabled = anl_div_is_enabled,
};
EXPORT_SYMBOL(anl_div_ro_en_ops_frac);
