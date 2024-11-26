/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Copyright (C) 2024 Anlogic, Inc.
 */

#ifndef _ANL_DIV_H_
#define _ANL_DIV_H_

#define CLK_DIVIDER_FIXED_FACTOR	BIT(24)
#define CLK_DIVIDER_ZERO_GATING		BIT(25)
#define CLK_DIVIDER_GATE_SETTODIS	BIT(26)

#define ANL_DIV_INIT(_width, _shift, _flags)	\
	.table = anl_divtbl_##_width,		\
	.width = _width,			\
	.shift = _shift,			\
	.flags = _flags

#define ANL_DIV_INIT_VAL(_width, _shift, _flags)	\
	.table = NULL,				\
	.width = _width,			\
	.shift = _shift,			\
	.flags = _flags

#define ANL_DIV_INIT_VAL_TBL(_tbl, _width, _shift, _flags)	\
	.table = _tbl,				\
	.width = _width,			\
	.shift = _shift,			\
	.flags = _flags

#define ANL_DIV_FIXED_INIT(_mult, _div, _flags)	\
	.table = NULL,		\
	.width = _mult,				\
	.shift = _div,				\
	.flags = ((_flags) | CLK_DIVIDER_FIXED_FACTOR)

#define ANL_DIV_ZERO_GATE_INIT(_width, _shift, _flags)	\
	ANL_DIV_INIT_VAL_TBL(&anl_divtbl_##_width##bits[1], _width, _shift, \
			(_flags | CLK_DIVIDER_ZERO_GATING))

struct anl_div {
	struct anl_clk clk;
	struct clk_div_table *table;
	uint width;
	uint shift;
	uint flags;
	u16 en_reg;
	u16 en_bitidx;
	u32 divval;
};

extern const struct clk_ops anl_div_ops;
extern const struct clk_ops anl_div_ro_ops;
extern const struct clk_ops anl_div_en_ops;
extern const struct clk_ops anl_div_ro_en_ops;
extern struct clk_div_table anl_divtbl_24[];
extern struct clk_div_table anl_divtbl_32[];

#endif
