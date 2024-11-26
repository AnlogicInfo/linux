/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Copyright (C) 2024 Anlogic, Inc.
 */
#ifndef _ANL_CLK_H_
#define _ANL_CLK_H_

#define ANL_CLK_INIT(_name, _parent, _ops, _flags, _reg)	\
	.min_rate = 0,	\
	.max_rate = ULONG_MAX,	\
	.reg = _reg,	\
	.hw.init = CLK_HW_INIT(_name,		\
			      _parent,		\
			      &_ops,	\
			      ((_flags) | CLK_GET_RATE_NOCACHE))

#define ANL_CLK_INIT_PARENTS(_name, _parents, _ops, _flags, _reg)	\
	.min_rate = 0,	\
	.max_rate = ULONG_MAX,	\
	.reg = _reg,	\
	.hw.init = CLK_HW_INIT_PARENTS(_name,		\
			      _parents,		\
			      &_ops,	\
			      ((_flags) | CLK_GET_RATE_NOCACHE))

struct anl_clk {
	struct clk_hw	hw;
	void __iomem	*base;
	u16		reg;
	spinlock_t	*lock;
	ulong		min_rate;
	ulong		max_rate;
};

struct anl_crp_desc {
	struct anl_clk			**crp_clks;
	ulong				num_crp_clks;
	struct clk_hw_onecell_data	*hw_clks;
	struct anl_rstbl		*arstbl;
	ulong				num_resets;
};

static inline struct anl_clk *hw_to_anl_clk(struct clk_hw *hw)
{
	return container_of(hw, struct anl_clk, hw);
}

void anl_init_divtbl(struct clk_div_table *tbl, uint cnt);
int anl_crp_probe(struct device *dev, void __iomem *reg,
		  const struct anl_crp_desc *desc);
#endif
