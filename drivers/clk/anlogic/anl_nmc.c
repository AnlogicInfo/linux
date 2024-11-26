// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2024 Anlogic, Inc.
 */

#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/device.h>
#include <linux/io.h>

#include "anl_clk.h"
#include "anl_nmc.h"

static int anl_nm_init(struct clk_hw *hw)
{
	struct anl_clk *clk = hw_to_anl_clk(hw);
	struct anl_nm *nm = (struct anl_nm *)clk;

	nm->m = (readl(clk->base + clk->reg) & 0x7F) + 1;
	nm->n = (readl(clk->base + clk->reg + 4) & 0x7F) + 1;

	return 0;
}

static int anl_nmc_init(struct clk_hw *hw)
{
	struct anl_clk *clk = hw_to_anl_clk(hw);
	struct anl_nmc *nm = (struct anl_nmc *)clk;

	nm->c = ((readl(clk->base + clk->reg) >> 24) & 0x7F) + 1;

	return 0;
}

static unsigned long anl_nm_recalc_rate(struct clk_hw *hw,
					unsigned long parent_rate)
{
	struct anl_clk *clk = hw_to_anl_clk(hw);
	struct anl_nm *nm = (struct anl_nm *)clk;
	u64 ret = (u64)parent_rate * nm->n;

	do_div(ret, nm->m);

	return ret;
}

static unsigned long anl_nmc_recalc_rate(struct clk_hw *hw,
					 unsigned long parent_rate)
{
	struct anl_clk *clk = hw_to_anl_clk(hw);
	struct anl_nmc *nmc = (struct anl_nmc *)clk;

	return parent_rate / nmc->c;
}

static long anl_nm_round_rate(struct clk_hw *hw, unsigned long rate,
			      unsigned long *prate)
{
	struct anl_clk *clk = hw_to_anl_clk(hw);
	struct anl_nm *nm = (struct anl_nm *)clk;
	u64 ret;

	if (clk_hw_get_flags(hw) & CLK_SET_RATE_PARENT) {
		ret = (u64)rate * nm->m;
		do_div(ret, nm->n);

		*prate = clk_hw_round_rate(clk_hw_get_parent(hw), ret);
	}

	ret = (u64)*prate * nm->n;
	do_div(ret, nm->m);

	return ret;
}

static int anl_nm_determine_rate(struct clk_hw *hw,
				 struct clk_rate_request *req)
{
	struct anl_clk *clk = hw_to_anl_clk(hw);
	struct anl_nm *nm = (struct anl_nm *)clk;
	u64 val;
	int ret;

	if (clk_hw_get_flags(hw) & CLK_SET_RATE_PARENT) {
		struct clk_hw *parent = clk_hw_get_parent(hw);
		struct clk_rate_request parent_req;

		val = (u64)req->rate * nm->m;
		do_div(val, nm->n);

		clk_hw_forward_rate_request(hw, req, parent,
					    &parent_req, val);
		ret = __clk_determine_rate(parent, &parent_req);
		if (ret)
			return ret;

		req->best_parent_rate = parent_req.rate;
	}

	val = (u64)req->best_parent_rate * nm->n;
	do_div(val, nm->m);
	req->rate = val;
	return 0;
}

static long anl_nmc_round_rate(struct clk_hw *hw, unsigned long rate,
			       unsigned long *prate)
{
	struct anl_clk *clk = hw_to_anl_clk(hw);
	struct anl_nmc *nmc = (struct anl_nmc *)clk;

	if (clk_hw_get_flags(hw) & CLK_SET_RATE_PARENT) {
		unsigned long best_parent;

		best_parent = rate * nmc->c;
		*prate = clk_hw_round_rate(clk_hw_get_parent(hw), best_parent);
	}

	return *prate / nmc->c;
}

static int anl_nmc_determine_rate(struct clk_hw *hw,
				  struct clk_rate_request *req)
{
	struct anl_clk *clk = hw_to_anl_clk(hw);
	struct anl_nmc *nmc = (struct anl_nmc *)clk;
	int ret;

	if (clk_hw_get_flags(hw) & CLK_SET_RATE_PARENT) {
		struct clk_hw *parent = clk_hw_get_parent(hw);
		struct clk_rate_request parent_req;
		unsigned long best_parent;

		best_parent = req->rate * nmc->c;
		clk_hw_forward_rate_request(hw, req, parent,
					    &parent_req, best_parent);
		ret = __clk_determine_rate(parent, &parent_req);
		if (ret)
			return ret;

		req->best_parent_rate = parent_req.rate;
	}

	req->rate = req->best_parent_rate / nmc->c;
	return 0;
}

const struct clk_ops anl_nm_ops = {
	.init		= anl_nm_init,
	.recalc_rate	= anl_nm_recalc_rate,
	.round_rate = anl_nm_round_rate,
	.determine_rate = anl_nm_determine_rate,
};
EXPORT_SYMBOL(anl_nm_ops);

const struct clk_ops anl_nmc_ops = {
	.init		= anl_nmc_init,
	.recalc_rate	= anl_nmc_recalc_rate,
	.round_rate = anl_nmc_round_rate,
	.determine_rate = anl_nmc_determine_rate,
};
EXPORT_SYMBOL(anl_nmc_ops);
