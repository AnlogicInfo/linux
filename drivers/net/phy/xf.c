// SPDX-License-Identifier: GPL-2.0
/*
 * Driver for XF5211C Gigabit Ethernet PHYs.
 */

#include <linux/module.h>
#include <linux/phy.h>

#define PHY_ID_XF5211C				0x6080b411

#define XF5211X_REG_PAGE			0x1f

#define XF5211X_COPPER_INTR_REG			0x12
#define XF5211X_COPPER_INTR_STATUS_REG		0x13
#define XF5211X_INTR_LINK_STATUS		BIT(4)

static int xf5211x_soft_reset(struct phy_device *phydev)
{
	int val;
	int ret;

	val = phy_read_paged(phydev, 0, MII_BMCR);
	if (val < 0)
		return val;

	ret = phy_write_paged(phydev, 0, MII_BMCR, val | BMCR_RESET);
	if (ret < 0)
		return ret;

	return phy_read_paged(phydev, 0, MII_BMCR);
}

static int xf5211x_read_page(struct phy_device *phydev)
{
	return __phy_read(phydev, XF5211X_REG_PAGE);
}

static int xf5211x_write_page(struct phy_device *phydev, int page)
{
	return __phy_write(phydev, XF5211X_REG_PAGE, page);
}

static int xf5211x_ack_interrupt(struct phy_device *phydev)
{
	int ret;

	ret = phy_read_paged(phydev, 0, XF5211X_COPPER_INTR_STATUS_REG);

	return ret < 0 ? ret : 0;
}

static int xf5211c_config_init(struct phy_device *phydev)
{
	u16 val;
	int ret;

	ret = phy_read_paged(phydev, 0, 0x1b);
	if (ret < 0)
		return ret;

	val = ret & 0xffbf;
	ret = phy_write_paged(phydev, 0, 0x1b, val);
	if (ret < 0)
		return ret;

	ret = phy_write_paged(phydev, 0, MII_ADVERTISE, 0x05e1);
	if (ret < 0)
		return ret;

	ret = phy_read_paged(phydev, 0x304, 0x1c);
	if (ret < 0)
		return ret;

	val = ret & 0xc7ff;
	switch (phydev->interface) {
	case PHY_INTERFACE_MODE_RGMII:
		val |= 0x2800;
		break;
	case PHY_INTERFACE_MODE_RGMII_ID:
		val |= 0x3000;
		break;
	case PHY_INTERFACE_MODE_RGMII_RXID:
		val |= 0x2000;
		break;
	case PHY_INTERFACE_MODE_RGMII_TXID:
		val |= 0x3800;
		break;
	default:
		return 0;
	}

	ret = phy_write_paged(phydev, 0x304, 0x1c, val);
	if (ret < 0)
		return ret;

	return phy_read_paged(phydev, 0, MII_BMCR);
}

static int xf5211x_config_intr(struct phy_device *phydev)
{
	u16 val;
	int err;

	if (phydev->interrupts == PHY_INTERRUPT_ENABLED) {
		err = xf5211x_ack_interrupt(phydev);
		if (err)
			return err;

		val = XF5211X_INTR_LINK_STATUS;
		err = phy_write_paged(phydev, 0, XF5211X_COPPER_INTR_REG, val);
		if (err)
			return err;

		return phy_read_paged(phydev, 0, XF5211X_COPPER_INTR_REG);
	}

	val = 0;
	err = phy_write_paged(phydev, 0, XF5211X_COPPER_INTR_REG, val);
	if (err)
		return err;

	err = phy_read_paged(phydev, 0, XF5211X_COPPER_INTR_REG);
	if (err)
		return err;

	return xf5211x_ack_interrupt(phydev);
}

static struct phy_driver xf5211x_drvs[] = {
	{
		PHY_ID_MATCH_EXACT(PHY_ID_XF5211C),
		.name		= "XF5211C Gigabit Ethernet",
		.features	= PHY_GBIT_FEATURES,
		.soft_reset	= xf5211x_soft_reset,
		.config_init	= xf5211c_config_init,
		.config_aneg	= genphy_config_aneg,
		.read_status	= genphy_read_status,
		.config_intr	= xf5211x_config_intr,
		.ack_interrupt	= xf5211x_ack_interrupt,
		.suspend	= genphy_suspend,
		.resume		= genphy_resume,
		.read_page	= xf5211x_read_page,
		.write_page	= xf5211x_write_page,
	},
};

module_phy_driver(xf5211x_drvs);

static struct mdio_device_id __maybe_unused xf5211x_tbl[] = {
	{ PHY_ID_MATCH_EXACT(PHY_ID_XF5211C) },
	{ }
};

MODULE_DEVICE_TABLE(mdio, xf5211x_tbl);

MODULE_DESCRIPTION("XF5211X PHY driver");
MODULE_AUTHOR("Liya");
MODULE_LICENSE("GPL");
