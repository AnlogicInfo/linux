/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Copyright (C) 2024 Anlogic, Inc.
 */

#ifndef _ANL_RESET_H_
#define _ANL_RESET_H_

#define ANL_RSTBL_SET_TO_ASSERT		BIT(0)

#define ANL_RSTBL_INIT(_reg, _bit, _flags)	\
	.reg = (_reg),				\
	.bit = BIT(_bit),			\
	.flags = (_flags)

struct anl_rstbl {
	u16 flags;
	u16 reg;
	u32 bit;
};

struct anl_reset {
	struct reset_controller_dev rcdev;
	void __iomem *base;
	struct anl_rstbl *arstbl;
	spinlock_t *lock;
};

static inline struct anl_reset *rcdev_to_anl_reset(struct reset_controller_dev *rcdev)
{
	return container_of(rcdev, struct anl_reset, rcdev);
}

extern const struct reset_control_ops anl_reset_ops;
#endif
