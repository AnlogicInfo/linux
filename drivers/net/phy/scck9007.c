// SPDX-License-Identifier: GPL-2.0+
/*
 * drivers/net/phy/scck9007.c
 *
 * Driver for SCCK9007 PHYs
 *
 *
 * Copyright (c) 2019 SCCK, Inc.
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 *
 * Support : Phys:
 *		Giga phys: SCCK9007
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/phy.h>
#include <linux/scck9007_phy.h>
#include <linux/of.h>
#include <linux/clk.h>
#ifndef LINUX_VERSION_CODE
#include <linux/version.h>
#else
#define KERNEL_VERSION(a,b,c) (((a) << 16) + ((b) << 8) + (c))
#endif


#define GMAC_CLOCK_INPUT_NEEDED 0


#define SCCK9007_PHY_MODE_FIBER	1 //fiber mode only
#define SCCK9007_PHY_MODE_UTP		2 //utp mode only
#define SCCK9007_PHY_MODE_POLL	3 //fiber and utp, poll mode

/*
 * Fiber only,  SCCK9007_PHY_MODE_CURR 1
 * UTP only, SCCK9007_PHY_MODE_CURR 2
 * combo, SCCK9007_PHY_MODE_CURR 3
 */
#define SCCK9007_PHY_MODE_CURR	3



#if (KERNEL_VERSION(4, 0, 0) > LINUX_VERSION_CODE) || (KERNEL_VERSION(5, 3, 0) < LINUX_VERSION_CODE)
static int scckphy_config_init(struct phy_device *phydev)
{
	return 0;
}
#endif

static int scckphy_read_ext(struct phy_device *phydev, u32 regnum)
{
	int ret;
	int val;

	ret = phy_write(phydev, REG_DEBUG_ADDR_OFFSET, regnum);
	if (ret < 0)
		return ret;

	val = phy_read(phydev, REG_DEBUG_DATA);

	return val;
}

static int scckphy_write_ext(struct phy_device *phydev, u32 regnum, u16 val)
{
	int ret;

	ret = phy_write(phydev, REG_DEBUG_ADDR_OFFSET, regnum);
	if (ret < 0)
		return ret;

	ret = phy_write(phydev, REG_DEBUG_DATA, val);

	return ret;
}

#if (KERNEL_VERSION(4, 0, 0) > LINUX_VERSION_CODE) || (KERNEL_VERSION(5, 3, 0) < LINUX_VERSION_CODE)
#else
int scck9007_soft_reset(struct phy_device *phydev)
{
	int ret;

	scckphy_write_ext(phydev, 0xa000, 0);
	ret = genphy_soft_reset(phydev);
	if (ret < 0)
		return ret;

	scckphy_write_ext(phydev, 0xa000, 2);
	ret = genphy_soft_reset(phydev);
	if (ret < 0) {
		scckphy_write_ext(phydev, 0xa000, 0);
		return ret;
	}

	return 0;
}
#endif


static int scck9007_config_init(struct phy_device *phydev)
{
	int ret;
	int val;


	phydev->irq = PHY_POLL;
	scckphy_write_ext(phydev, 0xa000, 0);
#if (KERNEL_VERSION(4, 0, 0) > LINUX_VERSION_CODE) || (KERNEL_VERSION(5, 3, 0) < LINUX_VERSION_CODE)
	ret = scckphy_config_init(phydev);
#else
	ret = genphy_config_init(phydev);
#endif
	if (ret < 0)
		return ret;

	/* disable auto sleep */
	val = scckphy_read_ext(phydev, SCCK9007_EXTREG_SLEEP_CONTROL1);
	if (val < 0)
		return val;

	val &= (~BIT(SCCK9007_EN_SLEEP_SW_BIT));

	ret = scckphy_write_ext(phydev, SCCK9007_EXTREG_SLEEP_CONTROL1, val);
	if (ret < 0)
		return ret;

	/* enable RXC clock when no wire plug */
	ret = scckphy_write_ext(phydev, 0xa000, 0);
	if (ret < 0)
		return ret;

	val = scckphy_read_ext(phydev, 0xc);
	if (val < 0)
		return val;
	val &= ~(1 << 12);
	ret = scckphy_write_ext(phydev, 0xc, val);
	if (ret < 0)
		return ret;
	printk (KERN_INFO "init call out...\n");
	return ret;
}

static int scck9007_adjust_status(struct phy_device *phydev, int val, int is_utp)
{
	int speed_mode, duplex;
#if (KERNEL_VERSION(4, 0, 0) > LINUX_VERSION_CODE) || (KERNEL_VERSION(5, 3, 0) < LINUX_VERSION_CODE)
	int speed = -1;
#else
	int speed = SPEED_UNKNOWN;
#endif

	duplex = (val & SCCK9007_DUPLEX) >> SCCK9007_DUPLEX_BIT;
	speed_mode = (val & SCCK9007_SPEED_MODE) >> SCCK9007_SPEED_MODE_BIT;
	switch (speed_mode) {
	case 0:
		if (is_utp)
			speed = SPEED_10;
		break;
	case 1:
		speed = SPEED_100;
		break;
	case 2:
		speed = SPEED_1000;
		break;
	case 3:
		break;
	default:
#if (KERNEL_VERSION(4, 0, 0) > LINUX_VERSION_CODE) || (KERNEL_VERSION(5, 3, 0) < LINUX_VERSION_CODE)
		speed = -1;
#else
		speed = SPEED_UNKNOWN;
#endif
		break;
	}

	phydev->speed = speed;
	phydev->duplex = duplex;

	return 0;
}


int scck9007_aneg_done (struct phy_device *phydev)
{
	return genphy_aneg_done(phydev);
}

static int scck9007_read_status(struct phy_device *phydev)
{
        //int i;
	int ret;
	volatile int val, scck9007_fiber_latch_val, scck9007_fiber_curr_val;
	volatile int link;
	int link_utp = 0, link_fiber = 0;

#if (SCCK9007_PHY_MODE_CURR != SCCK9007_PHY_MODE_FIBER)
	/* UTP */
	ret = scckphy_write_ext(phydev, 0xa000, 0);
	if (ret < 0)
		return ret;

	val = phy_read(phydev, REG_PHY_SPEC_STATUS);
	if (val < 0)
		return val;

	link = val & (BIT(SCCK9007_LINK_STATUS_BIT));
	if (link) {
		link_utp = 1;
		scck9007_adjust_status(phydev, val, 1);
	} else {
		link_utp = 0;
	}
#endif

#if (SCCK9007_PHY_MODE_CURR != SCCK9007_PHY_MODE_UTP)
	/* Fiber */
	ret = scckphy_write_ext(phydev, 0xa000, 2);
	if (ret < 0)
		return ret;

	val = phy_read(phydev, REG_PHY_SPEC_STATUS);
	if (val < 0)
		return val;

	/* for fiber, from 1000m to 100m, there is not link down from 0x11, and check reg 1 to identify such case */
	scck9007_fiber_latch_val = phy_read(phydev, MII_BMSR);
	scck9007_fiber_curr_val = phy_read(phydev, MII_BMSR);
	link = val & (BIT(SCCK9007_LINK_STATUS_BIT));
	if((link) && (scck9007_fiber_latch_val != scck9007_fiber_curr_val))
	{
		link = 0;
	}

	if (link) {
		link_fiber = 1;
		scck9007_adjust_status(phydev, val, 0);


	} else {
		link_fiber = 0;
	}
#endif

	if (link_utp || link_fiber) {
		phydev->link = 1;
	} else {
		phydev->link = 0;
	}

	if (link_utp) {
		scckphy_write_ext(phydev, 0xa000, 0);
	}

	return 0;
}

int scck9007_suspend(struct phy_device *phydev)
{
	int value;

#if (KERNEL_VERSION(4, 0, 0) > LINUX_VERSION_CODE) || (KERNEL_VERSION(5, 3, 0) < LINUX_VERSION_CODE)
	mutex_lock(&phydev->lock);
#else
	/* no need lock in 4.19 */
#endif

	scckphy_write_ext(phydev, 0xa000, 0);
	value = phy_read(phydev, MII_BMCR);
	phy_write(phydev, MII_BMCR, value | BMCR_PDOWN);

	scckphy_write_ext(phydev, 0xa000, 2);
	value = phy_read(phydev, MII_BMCR);
	phy_write(phydev, MII_BMCR, value | BMCR_PDOWN);

	scckphy_write_ext(phydev, 0xa000, 0);

#if (KERNEL_VERSION(4, 0, 0) > LINUX_VERSION_CODE) || (KERNEL_VERSION(5, 3, 0) < LINUX_VERSION_CODE)
	mutex_unlock(&phydev->lock);
#else
	/* no need lock/unlock in 4.19 */
#endif

	return 0;
}

int scck9007_resume(struct phy_device *phydev)
{
	int value;

#if (KERNEL_VERSION(4, 0, 0) > LINUX_VERSION_CODE) || (KERNEL_VERSION(5, 3, 0) < LINUX_VERSION_CODE)
	mutex_lock(&phydev->lock);
#else
	/* no need lock/unlock in 4.19 */
#endif

	scckphy_write_ext(phydev, 0xa000, 0);
	value = phy_read(phydev, MII_BMCR);
	phy_write(phydev, MII_BMCR, value & ~BMCR_PDOWN);

	scckphy_write_ext(phydev, 0xa000, 2);
	value = phy_read(phydev, MII_BMCR);
	phy_write(phydev, MII_BMCR, value & ~BMCR_PDOWN);

	scckphy_write_ext(phydev, 0xa000, 0);

#if (KERNEL_VERSION(4, 0, 0) > LINUX_VERSION_CODE) || (KERNEL_VERSION(5, 3, 0) < LINUX_VERSION_CODE)
	mutex_unlock(&phydev->lock);
#else
	/* no need lock/unlock in 4.19 */
#endif

	return 0;
}

static struct phy_driver scckphy_drvs[] = {
		{
                .phy_id         = PHY_ID_SCCK9007,
                .name           = "SCCK9007 Ethernet",
                .phy_id_mask    = SCCK9007_PHY_ID_MASK,
                .features       = PHY_GBIT_FEATURES,
                .flags          = PHY_POLL,
#if (KERNEL_VERSION(4, 0, 0) > LINUX_VERSION_CODE) || (KERNEL_VERSION(5, 3, 0) < LINUX_VERSION_CODE)
#else
				.soft_reset	= scck9007_soft_reset,
#endif
                .config_aneg    = genphy_config_aneg,
                .aneg_done		= scck9007_aneg_done,
                .config_init    = scck9007_config_init,
                .read_status    = scck9007_read_status,
                .suspend        = scck9007_suspend,
                .resume         = scck9007_resume,
        },
};

#if (KERNEL_VERSION(4, 0, 0) > LINUX_VERSION_CODE)
static int scckphy_drivers_register(struct phy_driver* phy_drvs, int size)
{
	int i, j;
	int ret;

	for (i = 0; i < size; i++) {
		ret = phy_driver_register(&phy_drvs[i]);
		if (ret)
			goto err;
	}

	return 0;

err:
	for (j = 0; j < i; j++)
		phy_driver_unregister(&phy_drvs[j]);

	return ret;
}

static void scckphy_drivers_unregister(struct phy_driver* phy_drvs, int size)
{
	int i;

	for (i = 0; i < size; i++) {
		phy_driver_unregister(&phy_drvs[i]);
	}
}

static int __init scckphy_init(void)
{
	printk("scck9007 phy register\n");
	return scckphy_drivers_register(scckphy_drvs, ARRAY_SIZE(scckphy_drvs));
}

static void __exit scckphy_exit(void)
{
	printk("scck9007 phy unregister\n");
	scckphy_drivers_unregister(scckphy_drvs, ARRAY_SIZE(scckphy_drvs));
}

module_init(scckphy_init);
module_exit(scckphy_exit);
#else
/* for linux 4.x */
module_phy_driver(scckphy_drvs);
#endif

MODULE_DESCRIPTION("scck9007 PHY driver");
MODULE_AUTHOR("scck");
MODULE_LICENSE("GPL");

static struct mdio_device_id __maybe_unused scck9007_tbl[] = {
	{ PHY_ID_SCCK9007, SCCK9007_PHY_ID_MASK },
	{ }
};

MODULE_DEVICE_TABLE(mdio, scck9007_tbl);
