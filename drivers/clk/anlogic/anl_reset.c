// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2024 Anlogic, Inc.
 */

#include <linux/delay.h>
#include <linux/io.h>
#include <linux/reset-controller.h>

#include "anl_reset.h"

static int anl_reset_status(struct reset_controller_dev *rcdev,
			    unsigned long id)
{
	struct anl_reset *arst = rcdev_to_anl_reset(rcdev);
	const struct anl_rstbl *arstbl = &arst->arstbl[id];
	u32 reg;

	reg = readl(arst->base + arstbl->reg);
	reg &= arstbl->bit;
	if (!(arstbl->flags & ANL_RSTBL_SET_TO_ASSERT))
		reg ^= arstbl->bit;

	return reg != 0;
}

static int anl_reset_assert(struct reset_controller_dev *rcdev,
			    unsigned long id)
{
	struct anl_reset *arst = rcdev_to_anl_reset(rcdev);
	const struct anl_rstbl *arstbl = &arst->arstbl[id];
	unsigned long flags;
	u32 reg;

	spin_lock_irqsave(arst->lock, flags);

	reg = readl(arst->base + arstbl->reg);
	if (arstbl->flags & ANL_RSTBL_SET_TO_ASSERT)
		reg |= arstbl->bit;
	else
		reg &= ~arstbl->bit;
	writel(reg, arst->base + arstbl->reg);

	spin_unlock_irqrestore(arst->lock, flags);

	return 0;
}

static int anl_reset_deassert(struct reset_controller_dev *rcdev,
			      unsigned long id)
{
	struct anl_reset *arst = rcdev_to_anl_reset(rcdev);
	const struct anl_rstbl *arstbl = &arst->arstbl[id];
	unsigned long flags;
	u32 reg;

	spin_lock_irqsave(arst->lock, flags);

	reg = readl(arst->base + arstbl->reg);
	if (arstbl->flags & ANL_RSTBL_SET_TO_ASSERT)
		reg &= ~arstbl->bit;
	else
		reg |= arstbl->bit;
	writel(reg, arst->base + arstbl->reg);

	spin_unlock_irqrestore(arst->lock, flags);

	return 0;
}

static int anl_reset_reset(struct reset_controller_dev *rcdev,
			   unsigned long id)
{
	anl_reset_assert(rcdev, id);
	udelay(10);
	anl_reset_deassert(rcdev, id);

	return 0;
}

const struct reset_control_ops anl_reset_ops = {
	.assert		= anl_reset_assert,
	.deassert	= anl_reset_deassert,
	.reset		= anl_reset_reset,
	.status		= anl_reset_status,
};
EXPORT_SYMBOL(anl_reset_ops);
