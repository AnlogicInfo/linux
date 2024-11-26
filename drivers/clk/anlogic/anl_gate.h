/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Copyright (C) 2024 Anlogic, Inc.
 */

#ifndef _ANL_GATE_H_
#define _ANL_GATE_H_

#define ANL_GATE_INIT(_bitidx, _flags)	\
	.bitidx = _bitidx,			\
	.flags = _flags

struct anl_gate {
	struct anl_clk clk;
	uint flags;
	u16 bitidx;
};

extern const struct clk_ops anl_gate_ops;

#endif
