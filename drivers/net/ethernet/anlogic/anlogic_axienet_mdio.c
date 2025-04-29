// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2025 Anlogic, Inc.
 *
 */

#include <linux/clk.h>
#include <linux/of_address.h>
#include <linux/of_mdio.h>
#include <linux/jiffies.h>
#include <linux/iopoll.h>

#include "anlogic_axienet.h"

#define MAX_MDIO_FREQ		2500000 /* 2.5 MHz */
#define DEFAULT_HOST_CLOCK	150000000 /* 150 MHz */

/* Wait till MDIO interface is ready to accept a new transaction.*/
static int anlogic_axienet_mdio_wait_until_ready(struct anlogic_axienet_local *lp)
{
	u32 val;

	return readx_poll_timeout(axinet_ior_read_mcr, lp,
				  val, val & AE_MDIO_MCR_READY_MASK,
				  1, 20000);
}

/* Enable the MDIO MDC. Called prior to a read/write operation */
static void anlogic_axienet_mdio_mdc_enable(struct anlogic_axienet_local *lp)
{
	anlogic_axienet_iow(lp, AE_MDIO_MC_OFFSET,
			    ((u32)lp->mii_clk_div | AE_MDIO_MC_MDIOEN_MASK));
}

/* Disable the MDIO MDC. Called after a read/write operation*/
static void anlogic_axienet_mdio_mdc_disable(struct anlogic_axienet_local *lp)
{
	u32 mc_reg;

	mc_reg = anlogic_axienet_ior(lp, AE_MDIO_MC_OFFSET);
	anlogic_axienet_iow(lp, AE_MDIO_MC_OFFSET,
			    (mc_reg & ~AE_MDIO_MC_MDIOEN_MASK));
}

/**
 * anlogic_axienet_mdio_read - MDIO interface read function
 * @bus:	Pointer to mii bus structure
 * @phy_id:	Address of the PHY device
 * @reg:	PHY register to read
 *
 * Return:	The register contents on success, -ETIMEDOUT on a timeout
 *
 * Reads the contents of the requested register from the requested PHY
 * address by first writing the details into MCR register. After a while
 * the register MRD is read to obtain the PHY register content.
 */
static int anlogic_axienet_mdio_read(struct mii_bus *bus, int phy_id, int reg)
{
	u32 rc;
	int ret;
	struct anlogic_axienet_local *lp = bus->priv;

	anlogic_axienet_mdio_mdc_enable(lp);

	ret = anlogic_axienet_mdio_wait_until_ready(lp);
	if (ret < 0) {
		anlogic_axienet_mdio_mdc_disable(lp);
		return ret;
	}

	anlogic_axienet_iow(lp, AE_MDIO_MCR_OFFSET,
			    (((phy_id << AE_MDIO_MCR_PHYAD_SHIFT) &
			      AE_MDIO_MCR_PHYAD_MASK) |
			     ((reg << AE_MDIO_MCR_REGAD_SHIFT) &
			      AE_MDIO_MCR_REGAD_MASK) |
			     AE_MDIO_MCR_INITIATE_MASK |
			     AE_MDIO_MCR_OP_READ_MASK));

	ret = anlogic_axienet_mdio_wait_until_ready(lp);
	if (ret < 0) {
		anlogic_axienet_mdio_mdc_disable(lp);
		return ret;
	}

	rc = anlogic_axienet_ior(lp, AE_MDIO_MRD_OFFSET) & 0x0000FFFF;

	anlogic_axienet_mdio_mdc_disable(lp);
	return rc;
}

/**
 * anlogic_axienet_mdio_write - MDIO interface write function
 * @bus:	Pointer to mii bus structure
 * @phy_id:	Address of the PHY device
 * @reg:	PHY register to write to
 * @val:	Value to be written into the register
 *
 * Return:	0 on success, -ETIMEDOUT on a timeout
 *
 * Writes the value to the requested register by first writing the value
 * into MWD register. The MCR register is then appropriately setup
 * to finish the write operation.
 */
static int anlogic_axienet_mdio_write(struct mii_bus *bus, int phy_id, int reg,
				      u16 val)
{
	int ret;
	struct anlogic_axienet_local *lp = bus->priv;

	anlogic_axienet_mdio_mdc_enable(lp);

	ret = anlogic_axienet_mdio_wait_until_ready(lp);
	if (ret < 0) {
		anlogic_axienet_mdio_mdc_disable(lp);
		return ret;
	}

	anlogic_axienet_iow(lp, AE_MDIO_MWD_OFFSET, (u32)val);
	anlogic_axienet_iow(lp, AE_MDIO_MCR_OFFSET,
			    (((phy_id << AE_MDIO_MCR_PHYAD_SHIFT) &
			      AE_MDIO_MCR_PHYAD_MASK) |
			     ((reg << AE_MDIO_MCR_REGAD_SHIFT) &
			      AE_MDIO_MCR_REGAD_MASK) |
			     AE_MDIO_MCR_INITIATE_MASK |
			     AE_MDIO_MCR_OP_WRITE_MASK));

	ret = anlogic_axienet_mdio_wait_until_ready(lp);
	if (ret < 0) {
		anlogic_axienet_mdio_mdc_disable(lp);
		return ret;
	}
	anlogic_axienet_mdio_mdc_disable(lp);
	return 0;
}

/**
 * anlogic_axienet_mdio_enable - MDIO hardware setup function
 * @lp:		Pointer to axienet local data structure.
 *
 * Return:	0 on success, -ETIMEDOUT on a timeout.
 *
 * Sets up the MDIO interface by initializing the MDIO clock and enabling the
 * MDIO interface in hardware.
 **/
int anlogic_axienet_mdio_enable(struct anlogic_axienet_local *lp)
{
	u32 host_clock;

	lp->mii_clk_div = 0;

	if (lp->axi_clk) {
		host_clock = clk_get_rate(lp->axi_clk);
	} else {
		struct device_node *np1;

		/* Legacy fallback: detect CPU clock frequency and use as AXI
		 * bus clock frequency. This only works on certain platforms.
		 */
		np1 = of_find_node_by_name(NULL, "cpu");
		if (!np1) {
			netdev_warn(lp->ndev, "Could not find CPU device node.\n");
			host_clock = DEFAULT_HOST_CLOCK;
		} else {
			int ret = of_property_read_u32(np1, "clock-frequency",
						       &host_clock);
			if (ret) {
				netdev_warn(lp->ndev, "CPU clock-frequency property not found.\n");
				host_clock = DEFAULT_HOST_CLOCK;
			}
			of_node_put(np1);
		}
		netdev_info(lp->ndev, "Setting assumed host clock to %u\n",
			    host_clock);
	}

	/* clk_div can be calculated by deriving it from the equation:
	 * fMDIO = fHOST / ((1 + clk_div) * 2)
	 *
	 * Where fMDIO <= 2500000, so we get:
	 * fHOST / ((1 + clk_div) * 2) <= 2500000
	 *
	 * Then we get:
	 * 1 / ((1 + clk_div) * 2) <= (2500000 / fHOST)
	 *
	 * Then we get:
	 * 1 / (1 + clk_div) <= ((2500000 * 2) / fHOST)
	 *
	 * Then we get:
	 * 1 / (1 + clk_div) <= (5000000 / fHOST)
	 *
	 * So:
	 * (1 + clk_div) >= (fHOST / 5000000)
	 *
	 * And finally:
	 * clk_div >= (fHOST / 5000000) - 1
	 *
	 * fHOST can be read from the flattened device tree as property
	 * "clock-frequency" from the CPU
	 */

	lp->mii_clk_div = (host_clock / (MAX_MDIO_FREQ * 2)) - 1;
	/* If there is any remainder from the division of
	 * fHOST / (MAX_MDIO_FREQ * 2), then we need to add
	 * 1 to the clock divisor or we will surely be above 2.5 MHz
	 */
	if (host_clock % (MAX_MDIO_FREQ * 2))
		lp->mii_clk_div++;

	netdev_dbg(lp->ndev,
		   "Setting MDIO clock divisor to %u/%u Hz host clock.\n",
		   lp->mii_clk_div, host_clock);

	anlogic_axienet_iow(lp, AE_MDIO_MC_OFFSET, lp->mii_clk_div | AE_MDIO_MC_MDIOEN_MASK);

	return anlogic_axienet_mdio_wait_until_ready(lp);
}

/**
 * anlogic_axienet_mdio_disable - MDIO hardware disable function
 * @lp:		Pointer to axienet local data structure.
 *
 * Disable the MDIO interface in hardware.
 **/
void anlogic_axienet_mdio_disable(struct anlogic_axienet_local *lp)
{
	anlogic_axienet_iow(lp, AE_MDIO_MC_OFFSET, 0);
}

/**
 * anlogic_axienet_mdio_setup - MDIO setup function
 * @lp:		Pointer to axienet local data structure.
 *
 * Return:	0 on success, -ETIMEDOUT on a timeout, -ENOMEM when
 *		mdiobus_alloc (to allocate memory for mii bus structure) fails.
 *
 * Sets up the MDIO interface by initializing the MDIO clock.
 * Register the MDIO interface.
 **/
int anlogic_axienet_mdio_setup(struct anlogic_axienet_local *lp)
{
	struct device_node *mdio_node;
	struct mii_bus *bus;
	int ret;

	ret = anlogic_axienet_mdio_enable(lp);
	if (ret < 0)
		return ret;

	bus = mdiobus_alloc();
	if (!bus)
		return -ENOMEM;

	snprintf(bus->id, MII_BUS_ID_SIZE, "axienet-%.8llx",
		 (unsigned long long)lp->regs_start);

	bus->priv = lp;
	bus->name = "Anlogic Axi Ethernet MDIO";
	bus->read = anlogic_axienet_mdio_read;
	bus->write = anlogic_axienet_mdio_write;
	bus->parent = lp->dev;
	lp->mii_bus = bus;

	mdio_node = of_get_child_by_name(lp->dev->of_node, "mdio");
	ret = of_mdiobus_register(bus, mdio_node);
	of_node_put(mdio_node);
	if (ret) {
		mdiobus_free(bus);
		lp->mii_bus = NULL;
		return ret;
	}
	anlogic_axienet_mdio_mdc_disable(lp);
	return 0;
}

/**
 * anlogic_axienet_mdio_teardown - MDIO remove function
 * @lp:		Pointer to axienet local data structure.
 *
 * Unregisters the MDIO and frees any associate memory for mii bus.
 */
void anlogic_axienet_mdio_teardown(struct anlogic_axienet_local *lp)
{
	mdiobus_unregister(lp->mii_bus);
	mdiobus_free(lp->mii_bus);
	lp->mii_bus = NULL;
}
