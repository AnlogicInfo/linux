/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Copyright (C) 2024 Anlogic, Inc.
 */
#ifndef _ANL_NMC_H_
#define _ANL_NMC_H_

struct anl_nm {
	struct anl_clk	clk;
	ulong		n;
	ulong		m;
};

struct anl_nmc {
	struct anl_clk	clk;
	ulong		c;
};

extern const struct clk_ops anl_nm_ops;
extern const struct clk_ops anl_nmc_ops;

#endif
