/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Copyright (C) 2024 Anlogic, Inc.
 */
#ifndef _ANL_SEL_H_
#define _ANL_SEL_H_

#define ANL_SEL_INIT(_mask, _shift, _flags, _table)	\
	.table = _table,			\
	.mask = _mask,				\
	.shift = _shift,			\
	.flags = _flags

struct anl_sel {
	struct anl_clk clk;
	const u32 *table;
	u32 mask;
	u32 shift;
	u32 flags;
};

extern const struct clk_ops anl_sel_ro_ops;
extern const struct clk_ops anl_sel_ops;

#endif

