// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2025 Anlogic, Inc.
 *
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/etherdevice.h>
#include <linux/module.h>
#include <linux/netdevice.h>
#include <linux/of_mdio.h>
#include <linux/of_net.h>
#include <linux/of_platform.h>
#include <linux/of_irq.h>
#include <linux/of_address.h>
#include <linux/skbuff.h>
#include <linux/math64.h>
#include <linux/phy.h>
#include <linux/mii.h>
#include <linux/ethtool.h>

#include "anlogic_axienet.h"

/* Descriptors defines for Tx and Rx DMA */
#define TX_BD_NUM_DEFAULT		1024
#define RX_BD_NUM_DEFAULT		1024
#define TX_BD_NUM_MIN			(MAX_SKB_FRAGS + 1)
#define TX_BD_NUM_MAX			4096
#define RX_BD_NUM_MAX			4096

/* Must be shorter than length of ethtool_drvinfo.driver field to fit */
#define DRIVER_NAME		"axienet"
#define DRIVER_DESCRIPTION	"Anlogic Axi Ethernet driver"
#define DRIVER_VERSION		"1.00a"

#define AXIENET_REGS_N		40

/* Match table for of_platform binding */
static const struct of_device_id axienet_of_match[] = {
	{ .compatible = "anlogic,axi-ethernet-1.00.a", },
	{ .compatible = "anlogic,axi-ethernet-1.01.a", },
	{ .compatible = "anlogic,axi-ethernet-2.01.a", },
	{},
};

MODULE_DEVICE_TABLE(of, axienet_of_match);

/* Option table for setting up Axi Ethernet hardware options */
static struct anlogic_axienet_option axienet_options[] = {
	/* Turn on jumbo packet support for both Rx and Tx */
	{
		.opt = AE_OPTION_JUMBO,
		.reg = AE_TC_OFFSET,
		.m_or = AE_TC_JUM_MASK,
	}, {
		.opt = AE_OPTION_JUMBO,
		.reg = AE_RCW1_OFFSET,
		.m_or = AE_RCW1_JUM_MASK,
	}, { /* Turn on VLAN packet support for both Rx and Tx */
		.opt = AE_OPTION_VLAN,
		.reg = AE_TC_OFFSET,
		.m_or = AE_TC_VLAN_MASK,
	}, {
		.opt = AE_OPTION_VLAN,
		.reg = AE_RCW1_OFFSET,
		.m_or = AE_RCW1_VLAN_MASK,
	}, { /* Turn on FCS stripping on receive packets */
		.opt = AE_OPTION_FCS_STRIP,
		.reg = AE_RCW1_OFFSET,
		.m_or = AE_RCW1_FCS_MASK,
	}, { /* Turn on FCS insertion on transmit packets */
		.opt = AE_OPTION_FCS_INSERT,
		.reg = AE_TC_OFFSET,
		.m_or = AE_TC_FCS_MASK,
	}, { /* Turn off length/type field checking on receive packets */
		.opt = AE_OPTION_LENTYPE_ERR,
		.reg = AE_RCW1_OFFSET,
		.m_or = AE_RCW1_LT_DIS_MASK,
	}, { /* Turn on Rx flow control */
		.opt = AE_OPTION_FLOW_CONTROL,
		.reg = AE_FCC_OFFSET,
		.m_or = AE_FCC_FCRX_MASK,
	}, { /* Turn on Tx flow control */
		.opt = AE_OPTION_FLOW_CONTROL,
		.reg = AE_FCC_OFFSET,
		.m_or = AE_FCC_FCTX_MASK,
	}, { /* Turn on promiscuous frame filtering */
		.opt = AE_OPTION_PROMISC,
		.reg = AE_FMI_OFFSET,
		.m_or = AE_FMI_PM_MASK,
	}, { /* Enable transmitter */
		.opt = AE_OPTION_TXEN,
		.reg = AE_TC_OFFSET,
		.m_or = AE_TC_TX_MASK,
	}, { /* Enable receiver */
		.opt = AE_OPTION_RXEN,
		.reg = AE_RCW1_OFFSET,
		.m_or = AE_RCW1_RX_MASK,
	},
	{}
};

/**
 * anlogic_axienet_dma_in32 - Memory mapped Axi DMA register read
 * @lp:		Pointer to axienet local structure
 * @reg:	Address offset from the base address of the Axi DMA core
 *
 * Return: The contents of the Axi DMA register
 *
 * This function returns the contents of the corresponding Axi DMA register.
 */
static inline u32 anlogic_axienet_dma_in32(struct anlogic_axienet_local *lp, off_t reg)
{
	return ioread32(lp->dma_regs + reg);
}

static void anlogic_desc_set_phys_addr(struct anlogic_axienet_local *lp, dma_addr_t addr,
				       struct anlogic_axidma_bd *desc)
{
	desc->phys = lower_32_bits(addr);
	if (lp->features & AE_FEATURE_DMA_64BIT)
		desc->phys_msb = upper_32_bits(addr);
}

static dma_addr_t anlogic_desc_get_phys_addr(struct anlogic_axienet_local *lp,
					     struct anlogic_axidma_bd *desc)
{
	dma_addr_t ret = desc->phys;

	if (lp->features & AE_FEATURE_DMA_64BIT)
		ret |= ((dma_addr_t)desc->phys_msb << 16) << 16;

	return ret;
}

/**
 * anlogic_axienet_dma_bd_release - Release buffer descriptor rings
 * @ndev:	Pointer to the net_device structure
 *
 * This function is used to release the descriptors allocated in
 * anlogic_axienet_dma_bd_init. anlogic_axienet_dma_bd_release is called when Axi Ethernet
 * driver stop api is called.
 */
static void anlogic_axienet_dma_bd_release(struct net_device *ndev)
{
	int i;
	struct anlogic_axienet_local *lp = netdev_priv(ndev);

	/* If we end up here, tx_bd_v must have been DMA allocated. */
	dma_free_coherent(lp->dev,
			  sizeof(*lp->tx_bd_v) * lp->tx_bd_num,
			  lp->tx_bd_v,
			  lp->tx_bd_p);

	if (!lp->rx_bd_v)
		return;

	for (i = 0; i < lp->rx_bd_num; i++) {
		dma_addr_t phys;

		/* A NULL skb means this descriptor has not been initialised
		 * at all.
		 */
		if (!lp->rx_bd_v[i].skb)
			break;

		/* For each descriptor, we programmed cntrl with the (non-zero)
		 * descriptor size, after it had been successfully allocated.
		 * So a non-zero value in there means we need to unmap it.
		 */
		if (lp->rx_bd_v[i].cntrl) {
			phys = anlogic_desc_get_phys_addr(lp, &lp->rx_bd_v[i]);
			dma_unmap_single(lp->dev, phys,
					 lp->max_frm_size, DMA_FROM_DEVICE);
		}

		dev_kfree_skb(lp->rx_bd_v[i].skb);
	}

	dma_free_coherent(lp->dev,
			  sizeof(*lp->rx_bd_v) * lp->rx_bd_num,
			  lp->rx_bd_v,
			  lp->rx_bd_p);
}

/**
 * anlogic_axienet_usec_to_timer - Calculate IRQ delay timer value
 * @lp:		Pointer to the anlogic_axienet_local structure
 * @coalesce_usec: Microseconds to convert into timer value
 */
static u32 anlogic_axienet_usec_to_timer(struct anlogic_axienet_local *lp, u32 coalesce_usec)
{
	u32 result;
	u64 clk_rate = 125000000; /* arbitrary guess if no clock rate set */

	if (lp->axi_clk)
		clk_rate = clk_get_rate(lp->axi_clk);

	/* 1 Timeout Interval = 125 * (clock period of SG clock) */
	result = DIV64_U64_ROUND_CLOSEST((u64)coalesce_usec * clk_rate,
					 (u64)125000000);
	if (result > 255)
		result = 255;

	return result;
}

/**
 * anlogic_axienet_dma_start - Set up DMA registers and start DMA operation
 * @lp:		Pointer to the anlogic_axienet_local structure
 */
static void anlogic_axienet_dma_start(struct anlogic_axienet_local *lp)
{
	/* Start updating the Rx channel control register */
	lp->rx_dma_cr = (lp->coalesce_count_rx << AXIDMA_COALESCE_SHIFT) |
			AXIDMA_IRQ_IOC_MASK | AXIDMA_IRQ_ERROR_MASK;
	/* Only set interrupt delay timer if not generating an interrupt on
	 * the first RX packet. Otherwise leave at 0 to disable delay interrupt.
	 */
	if (lp->coalesce_count_rx > 1)
		lp->rx_dma_cr |= (anlogic_axienet_usec_to_timer(lp, lp->coalesce_usec_rx)
					<< AXIDMA_DELAY_SHIFT) |
				 AXIDMA_IRQ_DELAY_MASK;
	anlogic_axienet_dma_out32(lp, AXIDMA_RX_CR_OFFSET, lp->rx_dma_cr);

	/* Start updating the Tx channel control register */
	lp->tx_dma_cr = (lp->coalesce_count_tx << AXIDMA_COALESCE_SHIFT) |
			AXIDMA_IRQ_IOC_MASK | AXIDMA_IRQ_ERROR_MASK;
	/* Only set interrupt delay timer if not generating an interrupt on
	 * the first TX packet. Otherwise leave at 0 to disable delay interrupt.
	 */
	if (lp->coalesce_count_tx > 1)
		lp->tx_dma_cr |= (anlogic_axienet_usec_to_timer(lp, lp->coalesce_usec_tx)
					<< AXIDMA_DELAY_SHIFT) |
				 AXIDMA_IRQ_DELAY_MASK;
	anlogic_axienet_dma_out32(lp, AXIDMA_TX_CR_OFFSET, lp->tx_dma_cr);

	/* Populate the tail pointer and bring the Rx Axi DMA engine out of
	 * halted state. This will make the Rx side ready for reception.
	 */
	anlogic_axienet_dma_out_addr(lp, AXIDMA_RX_CDESC_OFFSET, lp->rx_bd_p);
	lp->rx_dma_cr |= AXIDMA_CR_RUNSTOP_MASK;
	anlogic_axienet_dma_out32(lp, AXIDMA_RX_CR_OFFSET, lp->rx_dma_cr);
	anlogic_axienet_dma_out_addr(lp, AXIDMA_RX_TDESC_OFFSET, lp->rx_bd_p +
				     (sizeof(*lp->rx_bd_v) * (lp->rx_bd_num - 1)));

	/* Write to the RS (Run-stop) bit in the Tx channel control register.
	 * Tx channel is now ready to run. But only after we write to the
	 * tail pointer register that the Tx channel will start transmitting.
	 */
	anlogic_axienet_dma_out_addr(lp, AXIDMA_TX_CDESC_OFFSET, lp->tx_bd_p);
	lp->tx_dma_cr |= AXIDMA_CR_RUNSTOP_MASK;
	anlogic_axienet_dma_out32(lp, AXIDMA_TX_CR_OFFSET, lp->tx_dma_cr);
}

/**
 * anlogic_axienet_dma_bd_init - Setup buffer descriptor rings for Axi DMA
 * @ndev:	Pointer to the net_device structure
 *
 * Return: 0, on success -ENOMEM, on failure
 *
 * This function is called to initialize the Rx and Tx DMA descriptor
 * rings. This initializes the descriptors with required default values
 * and is called when Axi Ethernet driver reset is called.
 */
static int anlogic_axienet_dma_bd_init(struct net_device *ndev)
{
	int i;
	struct sk_buff *skb;
	struct anlogic_axienet_local *lp = netdev_priv(ndev);

	/* Reset the indexes which are used for accessing the BDs */
	lp->tx_bd_ci = 0;
	lp->tx_bd_tail = 0;
	lp->rx_bd_ci = 0;

	/* Allocate the Tx and Rx buffer descriptors. */
	lp->tx_bd_v = dma_alloc_coherent(lp->dev,
					 sizeof(*lp->tx_bd_v) * lp->tx_bd_num,
					 &lp->tx_bd_p, GFP_KERNEL);
	if (!lp->tx_bd_v)
		return -ENOMEM;

	lp->rx_bd_v = dma_alloc_coherent(lp->dev,
					 sizeof(*lp->rx_bd_v) * lp->rx_bd_num,
					 &lp->rx_bd_p, GFP_KERNEL);
	if (!lp->rx_bd_v)
		goto out;

	for (i = 0; i < lp->tx_bd_num; i++) {
		dma_addr_t addr = lp->tx_bd_p +
				  sizeof(*lp->tx_bd_v) *
				  ((i + 1) % lp->tx_bd_num);

		lp->tx_bd_v[i].next = lower_32_bits(addr);
		if (lp->features & AE_FEATURE_DMA_64BIT)
			lp->tx_bd_v[i].next_msb = upper_32_bits(addr);
	}

	for (i = 0; i < lp->rx_bd_num; i++) {
		dma_addr_t addr;

		addr = lp->rx_bd_p + sizeof(*lp->rx_bd_v) *
			((i + 1) % lp->rx_bd_num);
		lp->rx_bd_v[i].next = lower_32_bits(addr);
		if (lp->features & AE_FEATURE_DMA_64BIT)
			lp->rx_bd_v[i].next_msb = upper_32_bits(addr);

		skb = netdev_alloc_skb_ip_align(ndev, lp->max_frm_size);
		if (!skb)
			goto out;

		lp->rx_bd_v[i].skb = skb;
		addr = dma_map_single(lp->dev, skb->data,
				      lp->max_frm_size, DMA_FROM_DEVICE);
		if (dma_mapping_error(lp->dev, addr)) {
			netdev_err(ndev, "DMA mapping error\n");
			goto out;
		}
		anlogic_desc_set_phys_addr(lp, addr, &lp->rx_bd_v[i]);

		lp->rx_bd_v[i].cntrl = lp->max_frm_size;
	}

	anlogic_axienet_dma_start(lp);

	return 0;
out:
	anlogic_axienet_dma_bd_release(ndev);
	return -ENOMEM;
}

/**
 * anlogic_axienet_set_mac_address - Write the MAC address
 * @ndev:	Pointer to the net_device structure
 * @address:	6 byte Address to be written as MAC address
 *
 * This function is called to initialize the MAC address of the Axi Ethernet
 * core. It writes to the UAW0 and UAW1 registers of the core.
 */
static void anlogic_axienet_set_mac_address(struct net_device *ndev,
					    const void *address)
{
	struct anlogic_axienet_local *lp = netdev_priv(ndev);

	if (address)
		eth_hw_addr_set(ndev, address);
	if (!is_valid_ether_addr(ndev->dev_addr))
		eth_hw_addr_random(ndev);

	/* Set up unicast MAC address filter set its mac address */
	anlogic_axienet_iow(lp, AE_UAW0_OFFSET,
			    (ndev->dev_addr[0]) |
			    (ndev->dev_addr[1] << 8) |
			    (ndev->dev_addr[2] << 16) |
			    (ndev->dev_addr[3] << 24));
	anlogic_axienet_iow(lp, AE_UAW1_OFFSET,
			    (((anlogic_axienet_ior(lp, AE_UAW1_OFFSET)) &
			     ~AE_UAW1_UNICASTADDR_MASK) |
			     (ndev->dev_addr[4] |
			     (ndev->dev_addr[5] << 8))));
}

/**
 * anlogic_netdev_set_mac_address - Write the MAC address (from outside the driver)
 * @ndev:	Pointer to the net_device structure
 * @p:		6 byte Address to be written as MAC address
 *
 * Return: 0 for all conditions. Presently, there is no failure case.
 *
 * This function is called to initialize the MAC address of the Axi Ethernet
 * core. It calls the core specific anlogic_axienet_set_mac_address. This is the
 * function that goes into net_device_ops structure entry ndo_set_mac_address.
 */
static int anlogic_netdev_set_mac_address(struct net_device *ndev, void *p)
{
	struct sockaddr *addr = p;

	anlogic_axienet_set_mac_address(ndev, addr->sa_data);
	return 0;
}

/**
 * anlogic_axienet_set_multicast_list - Prepare the multicast table
 * @ndev:	Pointer to the net_device structure
 *
 * This function is called to initialize the multicast table during
 * initialization. The Axi Ethernet basic multicast support has a four-entry
 * multicast table which is initialized here. Additionally this function
 * goes into the net_device_ops structure entry ndo_set_multicast_list. This
 * means whenever the multicast table entries need to be updated this
 * function gets called.
 */
static void anlogic_axienet_set_multicast_list(struct net_device *ndev)
{
	int i = 0;
	u32 reg, af0reg, af1reg;
	struct anlogic_axienet_local *lp = netdev_priv(ndev);

	if (ndev->flags & (IFF_ALLMULTI | IFF_PROMISC) ||
	    netdev_mc_count(ndev) > AE_MULTICAST_CAM_TABLE_NUM) {
		/* We must make the kernel realize we had to move into
		 * promiscuous mode. If it was a promiscuous mode request
		 * the flag is already set. If not we set it.
		 */
		ndev->flags |= IFF_PROMISC;
		reg = anlogic_axienet_ior(lp, AE_FMI_OFFSET);
		reg |= AE_FMI_PM_MASK;
		anlogic_axienet_iow(lp, AE_FMI_OFFSET, reg);
		dev_info(&ndev->dev, "Promiscuous mode enabled.\n");
	} else if (!netdev_mc_empty(ndev)) {
		struct netdev_hw_addr *ha;

		reg = anlogic_axienet_ior(lp, AE_FMI_OFFSET);
		reg &= ~AE_FMI_PM_MASK;
		anlogic_axienet_iow(lp, AE_FMI_OFFSET, reg);

		netdev_for_each_mc_addr(ha, ndev) {
			if (i >= AE_MULTICAST_CAM_TABLE_NUM)
				break;

			af0reg = (ha->addr[0]);
			af0reg |= (ha->addr[1] << 8);
			af0reg |= (ha->addr[2] << 16);
			af0reg |= (ha->addr[3] << 24);

			af1reg = (ha->addr[4]);
			af1reg |= (ha->addr[5] << 8);

			reg = anlogic_axienet_ior(lp, AE_FMI_OFFSET) & 0xFFFFFF00;
			reg |= i;

			anlogic_axienet_iow(lp, AE_FMI_OFFSET, reg);
			anlogic_axienet_iow(lp, AE_AF0_OFFSET, af0reg);
			anlogic_axienet_iow(lp, AE_AF1_OFFSET, af1reg);
			anlogic_axienet_iow(lp, AE_FFE_OFFSET, 1);
			i++;
		}
	} else {
		reg = anlogic_axienet_ior(lp, AE_FMI_OFFSET);
		reg &= ~AE_FMI_PM_MASK;

		anlogic_axienet_iow(lp, AE_FMI_OFFSET, reg);
		dev_info(&ndev->dev, "Promiscuous mode disabled.\n");
	}

	for (; i < AE_MULTICAST_CAM_TABLE_NUM; i++) {
		reg = anlogic_axienet_ior(lp, AE_FMI_OFFSET) & 0xFFFFFF00;
		reg |= i;
		anlogic_axienet_iow(lp, AE_FMI_OFFSET, reg);
		anlogic_axienet_iow(lp, AE_FFE_OFFSET, 0);
	}
}

/**
 * anlogic_axienet_setoptions - Set an Axi Ethernet option
 * @ndev:	Pointer to the net_device structure
 * @options:	Option to be enabled/disabled
 *
 * The Axi Ethernet core has multiple features which can be selectively turned
 * on or off. The typical options could be jumbo frame option, basic VLAN
 * option, promiscuous mode option etc. This function is used to set or clear
 * these options in the Axi Ethernet hardware. This is done through
 * anlogic_axienet_option structure .
 */
static void anlogic_axienet_setoptions(struct net_device *ndev, u32 options)
{
	int reg;
	struct anlogic_axienet_local *lp = netdev_priv(ndev);
	struct anlogic_axienet_option *tp = &axienet_options[0];

	while (tp->opt) {
		reg = ((anlogic_axienet_ior(lp, tp->reg)) & ~(tp->m_or));
		if (options & tp->opt)
			reg |= tp->m_or;
		anlogic_axienet_iow(lp, tp->reg, reg);
		tp++;
	}

	lp->options |= options;
}

static int __anlogic_axienet_device_reset(struct anlogic_axienet_local *lp)
{
	u32 value;
	int ret;

	/* Reset Axi DMA. This would reset Axi Ethernet core as well. The reset
	 * process of Axi DMA takes a while to complete as all pending
	 * commands/transfers will be flushed or completed during this
	 * reset process.
	 * Note that even though both TX and RX have their own reset register,
	 * they both reset the entire DMA core, so only one needs to be used.
	 */
	anlogic_axienet_dma_out32(lp, AXIDMA_TX_CR_OFFSET, AXIDMA_CR_RESET_MASK);
	ret = read_poll_timeout(anlogic_axienet_dma_in32, value,
				!(value & AXIDMA_CR_RESET_MASK),
				DELAY_OF_ONE_MILLISEC, 50000, false, lp,
				AXIDMA_TX_CR_OFFSET);
	if (ret) {
		dev_err(lp->dev, "%s: DMA reset timeout!\n", __func__);
		return ret;
	}

	/* Wait for PhyRstCmplt bit to be set, indicating the PHY reset has finished */
	ret = read_poll_timeout(anlogic_axienet_ior, value,
				value & AE_INT_PHYRSTCMPLT_MASK,
				DELAY_OF_ONE_MILLISEC, 50000, false, lp,
				AE_IS_OFFSET);
	if (ret) {
		dev_err(lp->dev, "%s: timeout waiting for PhyRstCmplt\n", __func__);
		return ret;
	}

	return 0;
}

/**
 * anlogic_axienet_dma_stop - Stop DMA operation
 * @lp:		Pointer to the anlogic_axienet_local structure
 */
static void anlogic_axienet_dma_stop(struct anlogic_axienet_local *lp)
{
	int count;
	u32 cr, sr;

	cr = anlogic_axienet_dma_in32(lp, AXIDMA_RX_CR_OFFSET);
	cr &= ~(AXIDMA_CR_RUNSTOP_MASK | AXIDMA_IRQ_ALL_MASK);
	anlogic_axienet_dma_out32(lp, AXIDMA_RX_CR_OFFSET, cr);
	synchronize_irq(lp->rx_irq);

	cr = anlogic_axienet_dma_in32(lp, AXIDMA_TX_CR_OFFSET);
	cr &= ~(AXIDMA_CR_RUNSTOP_MASK | AXIDMA_IRQ_ALL_MASK);
	anlogic_axienet_dma_out32(lp, AXIDMA_TX_CR_OFFSET, cr);
	synchronize_irq(lp->tx_irq);

	/* Give DMAs a chance to halt gracefully */
	sr = anlogic_axienet_dma_in32(lp, AXIDMA_RX_SR_OFFSET);
	for (count = 0; !(sr & AXIDMA_SR_HALT_MASK) && count < 5; ++count) {
		msleep(20);
		sr = anlogic_axienet_dma_in32(lp, AXIDMA_RX_SR_OFFSET);
	}

	sr = anlogic_axienet_dma_in32(lp, AXIDMA_TX_SR_OFFSET);
	for (count = 0; !(sr & AXIDMA_SR_HALT_MASK) && count < 5; ++count) {
		msleep(20);
		sr = anlogic_axienet_dma_in32(lp, AXIDMA_TX_SR_OFFSET);
	}

	/* Do a reset to ensure DMA is really stopped */
	anlogic_axienet_lock_mii(lp);
	__anlogic_axienet_device_reset(lp);
	anlogic_axienet_unlock_mii(lp);
}

/**
 * anlogic_axienet_device_reset - Reset and initialize the Axi Ethernet hardware.
 * @ndev:	Pointer to the net_device structure
 *
 * This function is called to reset and initialize the Axi Ethernet core. This
 * is typically called during initialization. It does a reset of the Axi DMA
 * Rx/Tx channels and initializes the Axi DMA BDs. Since Axi DMA reset lines
 * are connected to Axi Ethernet reset lines, this in turn resets the Axi
 * Ethernet core. No separate hardware reset is done for the Axi Ethernet
 * core.
 * Returns 0 on success or a negative error number otherwise.
 */
static int anlogic_axienet_device_reset(struct net_device *ndev)
{
	u32 axienet_status;
	struct anlogic_axienet_local *lp = netdev_priv(ndev);
	int ret;

	ret = __anlogic_axienet_device_reset(lp);
	if (ret)
		return ret;

	lp->max_frm_size = AE_MAX_VLAN_FRAME_SIZE;
	lp->options |= AE_OPTION_VLAN;
	lp->options &= (~AE_OPTION_JUMBO);

	if (ndev->mtu > AE_MTU && ndev->mtu <= AE_JUMBO_MTU) {
		lp->max_frm_size = ndev->mtu + VLAN_ETH_HLEN + AE_TRL_SIZE;
		if (lp->max_frm_size <= lp->rxmem)
			lp->options |= AE_OPTION_JUMBO;
	}

	ret = anlogic_axienet_dma_bd_init(ndev);
	if (ret) {
		netdev_err(ndev, "%s: descriptor allocation failed\n",
			   __func__);
		return ret;
	}

	axienet_status = anlogic_axienet_ior(lp, AE_RCW1_OFFSET);
	axienet_status &= ~AE_RCW1_RX_MASK;
	anlogic_axienet_iow(lp, AE_RCW1_OFFSET, axienet_status);

	axienet_status = anlogic_axienet_ior(lp, AE_IP_OFFSET);
	if (axienet_status & AE_INT_RXRJECT_MASK)
		anlogic_axienet_iow(lp, AE_IS_OFFSET, AE_INT_RXRJECT_MASK);
	anlogic_axienet_iow(lp, AE_IE_OFFSET, lp->eth_irq > 0 ?
			    AE_INT_RECV_ERROR_MASK : 0);

	anlogic_axienet_iow(lp, AE_FCC_OFFSET, AE_FCC_FCRX_MASK);

	/* Sync default options with HW but leave receiver and
	 * transmitter disabled.
	 */
	anlogic_axienet_setoptions(ndev, lp->options &
				   ~(AE_OPTION_TXEN | AE_OPTION_RXEN));
	anlogic_axienet_set_mac_address(ndev, NULL);
	anlogic_axienet_set_multicast_list(ndev);
	anlogic_axienet_setoptions(ndev, lp->options);

	netif_trans_update(ndev);

	return 0;
}

/**
 * anlogic_axienet_free_tx_chain - Clean up a series of linked TX descriptors.
 * @lp:		Pointer to the anlogic_axienet_local structure
 * @first_bd:	Index of first descriptor to clean up
 * @nr_bds:	Max number of descriptors to clean up
 * @force:	Whether to clean descriptors even if not complete
 * @sizep:	Pointer to a u32 filled with the total sum of all bytes
 *		in all cleaned-up descriptors. Ignored if NULL.
 * @budget:	NAPI budget (use 0 when not called from NAPI poll)
 *
 * Would either be called after a successful transmit operation, or after
 * there was an error when setting up the chain.
 * Returns the number of descriptors handled.
 */
static int anlogic_axienet_free_tx_chain(struct anlogic_axienet_local *lp, u32 first_bd,
					 int nr_bds, bool force, u32 *sizep, int budget)
{
	struct anlogic_axidma_bd *cur_p;
	unsigned int status;
	dma_addr_t phys;
	int i;

	for (i = 0; i < nr_bds; i++) {
		cur_p = &lp->tx_bd_v[(first_bd + i) % lp->tx_bd_num];
		status = cur_p->status;

		/* If force is not specified, clean up only descriptors
		 * that have been completed by the MAC.
		 */
		if (!force && !(status & AXIDMA_BD_STS_COMPLETE_MASK))
			break;

		/* Ensure we see complete descriptor update */
		dma_rmb();
		phys = anlogic_desc_get_phys_addr(lp, cur_p);
		dma_unmap_single(lp->dev, phys,
				 (cur_p->cntrl & AXIDMA_BD_CTRL_LENGTH_MASK),
				 DMA_TO_DEVICE);

		if (cur_p->skb && (status & AXIDMA_BD_STS_COMPLETE_MASK))
			napi_consume_skb(cur_p->skb, budget);

		cur_p->app0 = 0;
		cur_p->app1 = 0;
		cur_p->app2 = 0;
		cur_p->app4 = 0;
		cur_p->skb = NULL;
		/* ensure our transmit path and device don't prematurely see status cleared */
		wmb();
		cur_p->cntrl = 0;
		cur_p->status = 0;

		if (sizep)
			*sizep += status & AXIDMA_BD_STS_ACTUAL_LEN_MASK;
	}

	return i;
}

/**
 * anlogic_axienet_check_tx_bd_space - Checks if a BD/group of BDs are currently busy
 * @lp:		Pointer to the anlogic_axienet_local structure
 * @num_frag:	The number of BDs to check for
 *
 * Return: 0, on success
 *	    NETDEV_TX_BUSY, if any of the descriptors are not free
 *
 * This function is invoked before BDs are allocated and transmission starts.
 * This function returns 0 if a BD or group of BDs can be allocated for
 * transmission. If the BD or any of the BDs are not free the function
 * returns a busy status.
 */
static inline int anlogic_axienet_check_tx_bd_space(struct anlogic_axienet_local *lp,
						    int num_frag)
{
	struct anlogic_axidma_bd *cur_p;

	/* Ensure we see all descriptor updates from device or TX polling */
	rmb();
	cur_p = &lp->tx_bd_v[(READ_ONCE(lp->tx_bd_tail) + num_frag) %
			     lp->tx_bd_num];
	if (cur_p->cntrl)
		return NETDEV_TX_BUSY;
	return 0;
}

/**
 * anlogic_axienet_tx_poll - Invoked once a transmit is completed by the
 * Axi DMA Tx channel.
 * @napi:	Pointer to NAPI structure.
 * @budget:	Max number of TX packets to process.
 *
 * Return: Number of TX packets processed.
 *
 * This function is invoked from the NAPI processing to notify the completion
 * of transmit operation. It clears fields in the corresponding Tx BDs and
 * unmaps the corresponding buffer so that CPU can regain ownership of the
 * buffer. It finally invokes "netif_wake_queue" to restart transmission if
 * required.
 */
static int anlogic_axienet_tx_poll(struct napi_struct *napi, int budget)
{
	struct anlogic_axienet_local *lp = container_of(napi,
							struct anlogic_axienet_local, napi_tx);
	struct net_device *ndev = lp->ndev;
	u32 size = 0;
	int packets;

	packets = anlogic_axienet_free_tx_chain(lp, lp->tx_bd_ci, budget, false, &size, budget);

	if (packets) {
		lp->tx_bd_ci += packets;
		if (lp->tx_bd_ci >= lp->tx_bd_num)
			lp->tx_bd_ci %= lp->tx_bd_num;

		u64_stats_update_begin(&lp->tx_stat_sync);
		u64_stats_add(&lp->tx_packets, packets);
		u64_stats_add(&lp->tx_bytes, size);
		u64_stats_update_end(&lp->tx_stat_sync);

		/* Matches barrier in anlogic_axienet_start_xmit */
		smp_mb();

		if (!anlogic_axienet_check_tx_bd_space(lp, MAX_SKB_FRAGS + 1))
			netif_wake_queue(ndev);
	}

	if (packets < budget && napi_complete_done(napi, packets)) {
		/* Re-enable TX completion interrupts. This should
		 * cause an immediate interrupt if any TX packets are
		 * already pending.
		 */
		anlogic_axienet_dma_out32(lp, AXIDMA_TX_CR_OFFSET, lp->tx_dma_cr);
	}
	return packets;
}

/**
 * anlogic_axienet_start_xmit - Starts the transmission.
 * @skb:	sk_buff pointer that contains data to be Txed.
 * @ndev:	Pointer to net_device structure.
 *
 * Return: NETDEV_TX_OK, on success
 *	    NETDEV_TX_BUSY, if any of the descriptors are not free
 *
 * This function is invoked from upper layers to initiate transmission. The
 * function uses the next available free BDs and populates their fields to
 * start the transmission. Additionally if checksum offloading is supported,
 * it populates AXI Stream Control fields with appropriate values.
 */
static netdev_tx_t
anlogic_axienet_start_xmit(struct sk_buff *skb, struct net_device *ndev)
{
	u32 ii;
	u32 num_frag;
	u32 csum_start_off;
	u32 csum_index_off;
	skb_frag_t *frag;
	dma_addr_t tail_p, phys;
	u32 orig_tail_ptr, new_tail_ptr;
	struct anlogic_axienet_local *lp = netdev_priv(ndev);
	struct anlogic_axidma_bd *cur_p;

	orig_tail_ptr = lp->tx_bd_tail;
	new_tail_ptr = orig_tail_ptr;

	num_frag = skb_shinfo(skb)->nr_frags;
	cur_p = &lp->tx_bd_v[orig_tail_ptr];

	if (anlogic_axienet_check_tx_bd_space(lp, num_frag + 1)) {
		/* Should not happen as last start_xmit call should have
		 * checked for sufficient space and queue should only be
		 * woken when sufficient space is available.
		 */
		netif_stop_queue(ndev);
		if (net_ratelimit())
			netdev_warn(ndev, "TX ring unexpectedly full\n");
		return NETDEV_TX_BUSY;
	}

	if (skb->ip_summed == CHECKSUM_PARTIAL) {
		if (lp->features & AE_FEATURE_FULL_TX_CSUM) {
			/* Tx Full Checksum Offload Enabled */
			cur_p->app0 |= 2;
		} else if (lp->features & AE_FEATURE_PARTIAL_TX_CSUM) {
			csum_start_off = skb_transport_offset(skb);
			csum_index_off = csum_start_off + skb->csum_offset;
			/* Tx Partial Checksum Offload Enabled */
			cur_p->app0 |= 1;
			cur_p->app1 = (csum_start_off << 16) | csum_index_off;
		}
	} else if (skb->ip_summed == CHECKSUM_UNNECESSARY) {
		cur_p->app0 |= 2; /* Tx Full Checksum Offload Enabled */
	}

	phys = dma_map_single(lp->dev, skb->data,
			      skb_headlen(skb), DMA_TO_DEVICE);
	if (unlikely(dma_mapping_error(lp->dev, phys))) {
		if (net_ratelimit())
			netdev_err(ndev, "TX DMA mapping error\n");
		ndev->stats.tx_dropped++;
		return NETDEV_TX_OK;
	}
	anlogic_desc_set_phys_addr(lp, phys, cur_p);
	cur_p->cntrl = skb_headlen(skb) | AXIDMA_BD_CTRL_TXSOF_MASK;

	for (ii = 0; ii < num_frag; ii++) {
		if (++new_tail_ptr >= lp->tx_bd_num)
			new_tail_ptr = 0;
		cur_p = &lp->tx_bd_v[new_tail_ptr];
		frag = &skb_shinfo(skb)->frags[ii];
		phys = dma_map_single(lp->dev,
				      skb_frag_address(frag),
				      skb_frag_size(frag),
				      DMA_TO_DEVICE);
		if (unlikely(dma_mapping_error(lp->dev, phys))) {
			if (net_ratelimit())
				netdev_err(ndev, "TX DMA mapping error\n");
			ndev->stats.tx_dropped++;
			anlogic_axienet_free_tx_chain(lp, orig_tail_ptr, ii + 1,
						      true, NULL, 0);
			return NETDEV_TX_OK;
		}
		anlogic_desc_set_phys_addr(lp, phys, cur_p);
		cur_p->cntrl = skb_frag_size(frag);
	}

	cur_p->cntrl |= AXIDMA_BD_CTRL_TXEOF_MASK;
	cur_p->skb = skb;

	tail_p = lp->tx_bd_p + sizeof(*lp->tx_bd_v) * new_tail_ptr;
	if (++new_tail_ptr >= lp->tx_bd_num)
		new_tail_ptr = 0;
	WRITE_ONCE(lp->tx_bd_tail, new_tail_ptr);

	/* Start the transfer */
	anlogic_axienet_dma_out_addr(lp, AXIDMA_TX_TDESC_OFFSET, tail_p);

	/* Stop queue if next transmit may not have space */
	if (anlogic_axienet_check_tx_bd_space(lp, MAX_SKB_FRAGS + 1)) {
		netif_stop_queue(ndev);

		/* Matches barrier in anlogic_axienet_tx_poll */
		smp_mb();

		/* Space might have just been freed - check again */
		if (!anlogic_axienet_check_tx_bd_space(lp, MAX_SKB_FRAGS + 1))
			netif_wake_queue(ndev);
	}

	return NETDEV_TX_OK;
}

/**
 * anlogic_axienet_rx_poll - Triggered by RX ISR to complete the BD processing.
 * @napi:	Pointer to NAPI structure.
 * @budget:	Max number of RX packets to process.
 *
 * Return: Number of RX packets processed.
 */
static int anlogic_axienet_rx_poll(struct napi_struct *napi, int budget)
{
	u32 length;
	u32 csumstatus;
	u32 size = 0;
	int packets = 0;
	dma_addr_t tail_p = 0;
	struct anlogic_axidma_bd *cur_p;
	struct sk_buff *skb, *new_skb;
	struct anlogic_axienet_local *lp = container_of(napi,
							struct anlogic_axienet_local, napi_rx);

	cur_p = &lp->rx_bd_v[lp->rx_bd_ci];

	while (packets < budget && (cur_p->status & AXIDMA_BD_STS_COMPLETE_MASK)) {
		dma_addr_t phys;

		/* Ensure we see complete descriptor update */
		dma_rmb();

		skb = cur_p->skb;
		cur_p->skb = NULL;

		/* skb could be NULL if a previous pass already received the
		 * packet for this slot in the ring, but failed to refill it
		 * with a newly allocated buffer. In this case, don't try to
		 * receive it again.
		 */
		if (likely(skb)) {
			length = cur_p->app4 & 0x0000FFFF;

			phys = anlogic_desc_get_phys_addr(lp, cur_p);
			dma_unmap_single(lp->dev, phys, lp->max_frm_size,
					 DMA_FROM_DEVICE);

			skb_put(skb, length);
			skb->protocol = eth_type_trans(skb, lp->ndev);
			/*skb_checksum_none_assert(skb);*/
			skb->ip_summed = CHECKSUM_NONE;

			/* if we're doing Rx csum offload, set it up */
			if (lp->features & AE_FEATURE_FULL_RX_CSUM) {
				csumstatus = (cur_p->app2 &
					      AE_FULL_CSUM_STATUS_MASK) >> 3;
				if (csumstatus == AE_IP_TCP_CSUM_VALIDATED ||
				    csumstatus == AE_IP_UDP_CSUM_VALIDATED) {
					skb->ip_summed = CHECKSUM_UNNECESSARY;
				}
			} else if ((lp->features & AE_FEATURE_PARTIAL_RX_CSUM) != 0 &&
				   skb->protocol == htons(ETH_P_IP) &&
				   skb->len > 64) {
				skb->csum = be32_to_cpu(cur_p->app3 & 0xFFFF);
				skb->ip_summed = CHECKSUM_COMPLETE;
			}

			napi_gro_receive(napi, skb);

			size += length;
			packets++;
		}

		new_skb = napi_alloc_skb(napi, lp->max_frm_size);
		if (!new_skb)
			break;

		phys = dma_map_single(lp->dev, new_skb->data,
				      lp->max_frm_size,
				      DMA_FROM_DEVICE);
		if (unlikely(dma_mapping_error(lp->dev, phys))) {
			if (net_ratelimit())
				netdev_err(lp->ndev, "RX DMA mapping error\n");
			dev_kfree_skb(new_skb);
			break;
		}
		anlogic_desc_set_phys_addr(lp, phys, cur_p);

		cur_p->cntrl = lp->max_frm_size;
		cur_p->status = 0;
		cur_p->skb = new_skb;

		/* Only update tail_p to mark this slot as usable after it has
		 * been successfully refilled.
		 */
		tail_p = lp->rx_bd_p + sizeof(*lp->rx_bd_v) * lp->rx_bd_ci;

		if (++lp->rx_bd_ci >= lp->rx_bd_num)
			lp->rx_bd_ci = 0;
		cur_p = &lp->rx_bd_v[lp->rx_bd_ci];
	}

	u64_stats_update_begin(&lp->rx_stat_sync);
	u64_stats_add(&lp->rx_packets, packets);
	u64_stats_add(&lp->rx_bytes, size);
	u64_stats_update_end(&lp->rx_stat_sync);

	if (tail_p)
		anlogic_axienet_dma_out_addr(lp, AXIDMA_RX_TDESC_OFFSET, tail_p);

	if (packets < budget && napi_complete_done(napi, packets)) {
		/* Re-enable RX completion interrupts. This should
		 * cause an immediate interrupt if any RX packets are
		 * already pending.
		 */
		anlogic_axienet_dma_out32(lp, AXIDMA_RX_CR_OFFSET, lp->rx_dma_cr);
	}
	return packets;
}

/**
 * anlogic_axienet_tx_irq - Tx Done Isr.
 * @irq:	irq number
 * @_ndev:	net_device pointer
 *
 * Return: IRQ_HANDLED if device generated a TX interrupt, IRQ_NONE otherwise.
 *
 * This is the Axi DMA Tx done Isr. It invokes NAPI polling to complete the
 * TX BD processing.
 */
static irqreturn_t anlogic_axienet_tx_irq(int irq, void *_ndev)
{
	unsigned int status;
	struct net_device *ndev = _ndev;
	struct anlogic_axienet_local *lp = netdev_priv(ndev);

	status = anlogic_axienet_dma_in32(lp, AXIDMA_TX_SR_OFFSET);

	if (!(status & AXIDMA_IRQ_ALL_MASK))
		return IRQ_NONE;

	anlogic_axienet_dma_out32(lp, AXIDMA_TX_SR_OFFSET, status);

	if (unlikely(status & AXIDMA_IRQ_ERROR_MASK)) {
		netdev_err(ndev, "DMA Tx error 0x%x\n", status);
		netdev_err(ndev, "Current BD is at: 0x%x%08x\n",
			   (lp->tx_bd_v[lp->tx_bd_ci]).phys_msb,
			   (lp->tx_bd_v[lp->tx_bd_ci]).phys);
		schedule_work(&lp->dma_err_task);
	} else {
		/* Disable further TX completion interrupts and schedule
		 * NAPI to handle the completions.
		 */
		u32 cr = lp->tx_dma_cr;

		cr &= ~(AXIDMA_IRQ_IOC_MASK | AXIDMA_IRQ_DELAY_MASK);
		anlogic_axienet_dma_out32(lp, AXIDMA_TX_CR_OFFSET, cr);

		napi_schedule(&lp->napi_tx);
	}

	return IRQ_HANDLED;
}

/**
 * anlogic_axienet_rx_irq - Rx Isr.
 * @irq:	irq number
 * @_ndev:	net_device pointer
 *
 * Return: IRQ_HANDLED if device generated a RX interrupt, IRQ_NONE otherwise.
 *
 * This is the Axi DMA Rx Isr. It invokes NAPI polling to complete the RX BD
 * processing.
 */
static irqreturn_t anlogic_axienet_rx_irq(int irq, void *_ndev)
{
	unsigned int status;
	struct net_device *ndev = _ndev;
	struct anlogic_axienet_local *lp = netdev_priv(ndev);

	status = anlogic_axienet_dma_in32(lp, AXIDMA_RX_SR_OFFSET);

	if (!(status & AXIDMA_IRQ_ALL_MASK))
		return IRQ_NONE;

	anlogic_axienet_dma_out32(lp, AXIDMA_RX_SR_OFFSET, status);

	if (unlikely(status & AXIDMA_IRQ_ERROR_MASK)) {
		netdev_err(ndev, "DMA Rx error 0x%x\n", status);
		netdev_err(ndev, "Current BD is at: 0x%x%08x\n",
			   (lp->rx_bd_v[lp->rx_bd_ci]).phys_msb,
			   (lp->rx_bd_v[lp->rx_bd_ci]).phys);
		schedule_work(&lp->dma_err_task);
	} else {
		/* Disable further RX completion interrupts and schedule
		 * NAPI receive.
		 */
		u32 cr = lp->rx_dma_cr;

		cr &= ~(AXIDMA_IRQ_IOC_MASK | AXIDMA_IRQ_DELAY_MASK);
		anlogic_axienet_dma_out32(lp, AXIDMA_RX_CR_OFFSET, cr);

		napi_schedule(&lp->napi_rx);
	}

	return IRQ_HANDLED;
}

/**
 * anlogic_axienet_eth_irq - Ethernet core Isr.
 * @irq:	irq number
 * @_ndev:	net_device pointer
 *
 * Return: IRQ_HANDLED if device generated a core interrupt, IRQ_NONE otherwise.
 *
 * Handle miscellaneous conditions indicated by Ethernet core IRQ.
 */
static irqreturn_t anlogic_axienet_eth_irq(int irq, void *_ndev)
{
	struct net_device *ndev = _ndev;
	struct anlogic_axienet_local *lp = netdev_priv(ndev);
	unsigned int pending;

	pending = anlogic_axienet_ior(lp, AE_IP_OFFSET);
	if (!pending)
		return IRQ_NONE;

	if (pending & AE_INT_RXFIFOOVR_MASK)
		ndev->stats.rx_missed_errors++;

	if (pending & AE_INT_RXRJECT_MASK)
		ndev->stats.rx_frame_errors++;

	anlogic_axienet_iow(lp, AE_IS_OFFSET, pending);
	return IRQ_HANDLED;
}

static void anlogic_axienet_dma_err_handler(struct work_struct *work);

/**
 * anlogic_axienet_open - Driver open routine.
 * @ndev:	Pointer to net_device structure
 *
 * Return: 0, on success.
 *	    non-zero error value on failure
 *
 * This is the driver open routine. It calls phylink_start to start the
 * PHY device.
 * It also allocates interrupt service routines, enables the interrupt lines
 * and ISR handling. Axi Ethernet core is reset through Axi DMA core. Buffer
 * descriptors are initialized.
 */
static int anlogic_axienet_open(struct net_device *ndev)
{
	int ret;
	struct anlogic_axienet_local *lp = netdev_priv(ndev);

	/* When we do an Axi Ethernet reset, it resets the complete core
	 * including the MDIO. MDIO must be disabled before resetting.
	 * Hold MDIO bus lock to avoid MDIO accesses during the reset.
	 */
	anlogic_axienet_lock_mii(lp);
	ret = anlogic_axienet_device_reset(ndev);
	anlogic_axienet_unlock_mii(lp);

	ret = phylink_of_phy_connect(lp->phylink, lp->dev->of_node, 0);
	if (ret) {
		dev_err(lp->dev, "phylink_of_phy_connect() failed: %d\n", ret);
		return ret;
	}

	phylink_start(lp->phylink);

	/* Enable worker thread for Axi DMA error handling */
	lp->stopping = false;
	INIT_WORK(&lp->dma_err_task, anlogic_axienet_dma_err_handler);

	napi_enable(&lp->napi_rx);
	napi_enable(&lp->napi_tx);

	/* Enable interrupts for Axi DMA Tx */
	ret = request_irq(lp->tx_irq, anlogic_axienet_tx_irq, IRQF_SHARED,
			  ndev->name, ndev);
	if (ret)
		goto err_tx_irq;
	/* Enable interrupts for Axi DMA Rx */
	ret = request_irq(lp->rx_irq, anlogic_axienet_rx_irq, IRQF_SHARED,
			  ndev->name, ndev);
	if (ret)
		goto err_rx_irq;
	/* Enable interrupts for Axi Ethernet core (if defined) */
	if (lp->eth_irq > 0) {
		ret = request_irq(lp->eth_irq, anlogic_axienet_eth_irq, IRQF_SHARED,
				  ndev->name, ndev);
		if (ret)
			goto err_eth_irq;
	}

	return 0;

err_eth_irq:
	free_irq(lp->rx_irq, ndev);
err_rx_irq:
	free_irq(lp->tx_irq, ndev);
err_tx_irq:
	napi_disable(&lp->napi_tx);
	napi_disable(&lp->napi_rx);
	phylink_stop(lp->phylink);
	phylink_disconnect_phy(lp->phylink);
	cancel_work_sync(&lp->dma_err_task);
	dev_err(lp->dev, "request_irq() failed\n");
	return ret;
}

/**
 * anlogic_axienet_stop - Driver stop routine.
 * @ndev:	Pointer to net_device structure
 *
 * Return: 0, on success.
 *
 * This is the driver stop routine. It calls phylink_disconnect to stop the PHY
 * device. It also removes the interrupt handlers and disables the interrupts.
 * The Axi DMA Tx/Rx BDs are released.
 */
static int anlogic_axienet_stop(struct net_device *ndev)
{
	struct anlogic_axienet_local *lp = netdev_priv(ndev);

	dev_dbg(&ndev->dev, "axienet_close()\n");

	WRITE_ONCE(lp->stopping, true);
	flush_work(&lp->dma_err_task);

	napi_disable(&lp->napi_tx);
	napi_disable(&lp->napi_rx);

	phylink_stop(lp->phylink);
	phylink_disconnect_phy(lp->phylink);

	anlogic_axienet_setoptions(ndev, lp->options &
			   ~(AE_OPTION_TXEN | AE_OPTION_RXEN));

	anlogic_axienet_dma_stop(lp);

	anlogic_axienet_iow(lp, AE_IE_OFFSET, 0);

	cancel_work_sync(&lp->dma_err_task);

	if (lp->eth_irq > 0)
		free_irq(lp->eth_irq, ndev);
	free_irq(lp->tx_irq, ndev);
	free_irq(lp->rx_irq, ndev);

	anlogic_axienet_dma_bd_release(ndev);
	return 0;
}

/**
 * anlogic_axienet_change_mtu - Driver change mtu routine.
 * @ndev:	Pointer to net_device structure
 * @new_mtu:	New mtu value to be applied
 *
 * Return: Always returns 0 (success).
 *
 * This is the change mtu driver routine. It checks if the Axi Ethernet
 * hardware supports jumbo frames before changing the mtu. This can be
 * called only when the device is not up.
 */
static int anlogic_axienet_change_mtu(struct net_device *ndev, int new_mtu)
{
	struct anlogic_axienet_local *lp = netdev_priv(ndev);

	if (netif_running(ndev))
		return -EBUSY;

	if ((new_mtu + VLAN_ETH_HLEN +
		AE_TRL_SIZE) > lp->rxmem)
		return -EINVAL;

	ndev->mtu = new_mtu;

	return 0;
}

#ifdef CONFIG_NET_POLL_CONTROLLER
/**
 * anlogic_axienet_poll_controller - Axi Ethernet poll mechanism.
 * @ndev:	Pointer to net_device structure
 *
 * This implements Rx/Tx ISR poll mechanisms. The interrupts are disabled prior
 * to polling the ISRs and are enabled back after the polling is done.
 */
static void anlogic_axienet_poll_controller(struct net_device *ndev)
{
	struct anlogic_axienet_local *lp = netdev_priv(ndev);

	disable_irq(lp->tx_irq);
	disable_irq(lp->rx_irq);
	anlogic_axienet_rx_irq(lp->tx_irq, ndev);
	anlogic_axienet_tx_irq(lp->rx_irq, ndev);
	enable_irq(lp->tx_irq);
	enable_irq(lp->rx_irq);
}
#endif

static int anlogic_axienet_ioctl(struct net_device *dev, struct ifreq *rq, int cmd)
{
	struct anlogic_axienet_local *lp = netdev_priv(dev);

	if (!netif_running(dev))
		return -EINVAL;

	return phylink_mii_ioctl(lp->phylink, rq, cmd);
}

static void
anlogic_axienet_get_stats64(struct net_device *dev, struct rtnl_link_stats64 *stats)
{
	struct anlogic_axienet_local *lp = netdev_priv(dev);
	unsigned int start;

	netdev_stats_to_stats64(stats, &dev->stats);

	do {
		start = u64_stats_fetch_begin(&lp->rx_stat_sync);
		stats->rx_packets = u64_stats_read(&lp->rx_packets);
		stats->rx_bytes = u64_stats_read(&lp->rx_bytes);
	} while (u64_stats_fetch_retry(&lp->rx_stat_sync, start));

	do {
		start = u64_stats_fetch_begin(&lp->tx_stat_sync);
		stats->tx_packets = u64_stats_read(&lp->tx_packets);
		stats->tx_bytes = u64_stats_read(&lp->tx_bytes);
	} while (u64_stats_fetch_retry(&lp->tx_stat_sync, start));
}

static const struct net_device_ops axienet_netdev_ops = {
	.ndo_open = anlogic_axienet_open,
	.ndo_stop = anlogic_axienet_stop,
	.ndo_start_xmit = anlogic_axienet_start_xmit,
	.ndo_get_stats64 = anlogic_axienet_get_stats64,
	.ndo_change_mtu	= anlogic_axienet_change_mtu,
	.ndo_set_mac_address = anlogic_netdev_set_mac_address,
	.ndo_validate_addr = eth_validate_addr,
	.ndo_eth_ioctl = anlogic_axienet_ioctl,
	.ndo_set_rx_mode = anlogic_axienet_set_multicast_list,
#ifdef CONFIG_NET_POLL_CONTROLLER
	.ndo_poll_controller = anlogic_axienet_poll_controller,
#endif
};

/**
 * axienet_ethtools_get_drvinfo - Get various Axi Ethernet driver information.
 * @ndev:	Pointer to net_device structure
 * @ed:		Pointer to ethtool_drvinfo structure
 *
 * This implements ethtool command for getting the driver information.
 * Issue "ethtool -i ethX" under linux prompt to execute this function.
 */
static void axienet_ethtools_get_drvinfo(struct net_device *ndev,
					 struct ethtool_drvinfo *ed)
{
	strscpy(ed->driver, DRIVER_NAME, sizeof(ed->driver));
	strscpy(ed->version, DRIVER_VERSION, sizeof(ed->version));
}

/**
 * axienet_ethtools_get_regs_len - Get the total regs length present in the
 *				   AxiEthernet core.
 * @ndev:	Pointer to net_device structure
 *
 * This implements ethtool command for getting the total register length
 * information.
 *
 * Return: the total regs length
 */
static int axienet_ethtools_get_regs_len(struct net_device *ndev)
{
	return sizeof(u32) * AXIENET_REGS_N;
}

/**
 * axienet_ethtools_get_regs - Dump the contents of all registers present
 *			       in AxiEthernet core.
 * @ndev:	Pointer to net_device structure
 * @regs:	Pointer to ethtool_regs structure
 * @ret:	Void pointer used to return the contents of the registers.
 *
 * This implements ethtool command for getting the Axi Ethernet register dump.
 * Issue "ethtool -d ethX" to execute this function.
 */
static void axienet_ethtools_get_regs(struct net_device *ndev,
				      struct ethtool_regs *regs, void *ret)
{
	u32 *data = (u32 *)ret;
	size_t len = sizeof(u32) * AXIENET_REGS_N;
	struct anlogic_axienet_local *lp = netdev_priv(ndev);

	regs->version = 0;
	regs->len = len;

	memset(data, 0, len);
	data[0] = anlogic_axienet_ior(lp, AE_RAF_OFFSET);
	data[1] = anlogic_axienet_ior(lp, AE_TPF_OFFSET);
	data[2] = anlogic_axienet_ior(lp, AE_IFGP_OFFSET);
	data[3] = anlogic_axienet_ior(lp, AE_IS_OFFSET);
	data[4] = anlogic_axienet_ior(lp, AE_IP_OFFSET);
	data[5] = anlogic_axienet_ior(lp, AE_IE_OFFSET);
	data[6] = anlogic_axienet_ior(lp, AE_TTAG_OFFSET);
	data[7] = anlogic_axienet_ior(lp, AE_RTAG_OFFSET);
	data[8] = anlogic_axienet_ior(lp, AE_UAWL_OFFSET);
	data[9] = anlogic_axienet_ior(lp, AE_UAWU_OFFSET);
	data[10] = anlogic_axienet_ior(lp, AE_TPID0_OFFSET);
	data[11] = anlogic_axienet_ior(lp, AE_TPID1_OFFSET);
	data[12] = anlogic_axienet_ior(lp, AE_PPST_OFFSET);
	data[13] = anlogic_axienet_ior(lp, AE_RCW0_OFFSET);
	data[14] = anlogic_axienet_ior(lp, AE_RCW1_OFFSET);
	data[15] = anlogic_axienet_ior(lp, AE_TC_OFFSET);
	data[16] = anlogic_axienet_ior(lp, AE_FCC_OFFSET);
	data[17] = anlogic_axienet_ior(lp, AE_EMMC_OFFSET);
	data[18] = anlogic_axienet_ior(lp, AE_PHYC_OFFSET);
	data[19] = anlogic_axienet_ior(lp, AE_MDIO_MC_OFFSET);
	data[20] = anlogic_axienet_ior(lp, AE_MDIO_MCR_OFFSET);
	data[21] = anlogic_axienet_ior(lp, AE_MDIO_MWD_OFFSET);
	data[22] = anlogic_axienet_ior(lp, AE_MDIO_MRD_OFFSET);
	data[27] = anlogic_axienet_ior(lp, AE_UAW0_OFFSET);
	data[28] = anlogic_axienet_ior(lp, AE_UAW1_OFFSET);
	data[29] = anlogic_axienet_ior(lp, AE_FMI_OFFSET);
	data[30] = anlogic_axienet_ior(lp, AE_AF0_OFFSET);
	data[31] = anlogic_axienet_ior(lp, AE_AF1_OFFSET);
	data[32] = anlogic_axienet_dma_in32(lp, AXIDMA_TX_CR_OFFSET);
	data[33] = anlogic_axienet_dma_in32(lp, AXIDMA_TX_SR_OFFSET);
	data[34] = anlogic_axienet_dma_in32(lp, AXIDMA_TX_CDESC_OFFSET);
	data[35] = anlogic_axienet_dma_in32(lp, AXIDMA_TX_TDESC_OFFSET);
	data[36] = anlogic_axienet_dma_in32(lp, AXIDMA_RX_CR_OFFSET);
	data[37] = anlogic_axienet_dma_in32(lp, AXIDMA_RX_SR_OFFSET);
	data[38] = anlogic_axienet_dma_in32(lp, AXIDMA_RX_CDESC_OFFSET);
	data[39] = anlogic_axienet_dma_in32(lp, AXIDMA_RX_TDESC_OFFSET);
}

static void
anlogic_axienet_ethtools_get_ringparam(struct net_device *ndev,
				       struct ethtool_ringparam *ering,
				       struct kernel_ethtool_ringparam *kernel_ering,
				       struct netlink_ext_ack *extack)
{
	struct anlogic_axienet_local *lp = netdev_priv(ndev);

	ering->rx_max_pending = RX_BD_NUM_MAX;
	ering->rx_mini_max_pending = 0;
	ering->rx_jumbo_max_pending = 0;
	ering->tx_max_pending = TX_BD_NUM_MAX;
	ering->rx_pending = lp->rx_bd_num;
	ering->rx_mini_pending = 0;
	ering->rx_jumbo_pending = 0;
	ering->tx_pending = lp->tx_bd_num;
}

static int
anlogic_axienet_ethtools_set_ringparam(struct net_device *ndev,
				       struct ethtool_ringparam *ering,
				       struct kernel_ethtool_ringparam *kernel_ering,
				       struct netlink_ext_ack *extack)
{
	struct anlogic_axienet_local *lp = netdev_priv(ndev);

	if (ering->rx_pending > RX_BD_NUM_MAX ||
	    ering->rx_mini_pending ||
	    ering->rx_jumbo_pending ||
	    ering->tx_pending < TX_BD_NUM_MIN ||
	    ering->tx_pending > TX_BD_NUM_MAX)
		return -EINVAL;

	if (netif_running(ndev))
		return -EBUSY;

	lp->rx_bd_num = ering->rx_pending;
	lp->tx_bd_num = ering->tx_pending;
	return 0;
}

/**
 * anlogic_axienet_ethtools_get_pauseparam - Get the pause parameter setting for
 *				     Tx and Rx paths.
 * @ndev:	Pointer to net_device structure
 * @epauseparm:	Pointer to ethtool_pauseparam structure.
 *
 * This implements ethtool command for getting axi ethernet pause frame
 * setting. Issue "ethtool -a ethX" to execute this function.
 */
static void
anlogic_axienet_ethtools_get_pauseparam(struct net_device *ndev,
					struct ethtool_pauseparam *epauseparm)
{
	struct anlogic_axienet_local *lp = netdev_priv(ndev);

	phylink_ethtool_get_pauseparam(lp->phylink, epauseparm);
}

/**
 * anlogic_axienet_ethtools_set_pauseparam - Set device pause parameter(flow control)
 *				     settings.
 * @ndev:	Pointer to net_device structure
 * @epauseparm:Pointer to ethtool_pauseparam structure
 *
 * This implements ethtool command for enabling flow control on Rx and Tx
 * paths. Issue "ethtool -A ethX tx on|off" under linux prompt to execute this
 * function.
 *
 * Return: 0 on success, -EFAULT if device is running
 */
static int
anlogic_axienet_ethtools_set_pauseparam(struct net_device *ndev,
					struct ethtool_pauseparam *epauseparm)
{
	struct anlogic_axienet_local *lp = netdev_priv(ndev);

	return phylink_ethtool_set_pauseparam(lp->phylink, epauseparm);
}

/**
 * anlogic_axienet_ethtools_get_coalesce - Get DMA interrupt coalescing count.
 * @ndev:	Pointer to net_device structure
 * @ecoalesce:	Pointer to ethtool_coalesce structure
 * @kernel_coal: ethtool CQE mode setting structure
 * @extack:	extack for reporting error messages
 *
 * This implements ethtool command for getting the DMA interrupt coalescing
 * count on Tx and Rx paths. Issue "ethtool -c ethX" under linux prompt to
 * execute this function.
 *
 * Return: 0 always
 */
static int
anlogic_axienet_ethtools_get_coalesce(struct net_device *ndev,
				      struct ethtool_coalesce *ecoalesce,
				      struct kernel_ethtool_coalesce *kernel_coal,
				      struct netlink_ext_ack *extack)
{
	struct anlogic_axienet_local *lp = netdev_priv(ndev);

	ecoalesce->rx_max_coalesced_frames = lp->coalesce_count_rx;
	ecoalesce->rx_coalesce_usecs = lp->coalesce_usec_rx;
	ecoalesce->tx_max_coalesced_frames = lp->coalesce_count_tx;
	ecoalesce->tx_coalesce_usecs = lp->coalesce_usec_tx;
	return 0;
}

/**
 * anlogic_axienet_ethtools_set_coalesce - Set DMA interrupt coalescing count.
 * @ndev:	Pointer to net_device structure
 * @ecoalesce:	Pointer to ethtool_coalesce structure
 * @kernel_coal: ethtool CQE mode setting structure
 * @extack:	extack for reporting error messages
 *
 * This implements ethtool command for setting the DMA interrupt coalescing
 * count on Tx and Rx paths. Issue "ethtool -C ethX rx-frames 5" under linux
 * prompt to execute this function.
 *
 * Return: 0, on success, Non-zero error value on failure.
 */
static int
anlogic_axienet_ethtools_set_coalesce(struct net_device *ndev,
				      struct ethtool_coalesce *ecoalesce,
				      struct kernel_ethtool_coalesce *kernel_coal,
				      struct netlink_ext_ack *extack)
{
	struct anlogic_axienet_local *lp = netdev_priv(ndev);

	if (netif_running(ndev)) {
		netdev_err(ndev,
			   "Please stop netif before applying configuration\n");
		return -EFAULT;
	}

	if (ecoalesce->rx_max_coalesced_frames)
		lp->coalesce_count_rx = ecoalesce->rx_max_coalesced_frames;
	if (ecoalesce->rx_coalesce_usecs)
		lp->coalesce_usec_rx = ecoalesce->rx_coalesce_usecs;
	if (ecoalesce->tx_max_coalesced_frames)
		lp->coalesce_count_tx = ecoalesce->tx_max_coalesced_frames;
	if (ecoalesce->tx_coalesce_usecs)
		lp->coalesce_usec_tx = ecoalesce->tx_coalesce_usecs;

	return 0;
}

static int
anlogic_axienet_ethtools_get_link_ksettings(struct net_device *ndev,
					    struct ethtool_link_ksettings *cmd)
{
	struct anlogic_axienet_local *lp = netdev_priv(ndev);

	return phylink_ethtool_ksettings_get(lp->phylink, cmd);
}

static int
anlogic_axienet_ethtools_set_link_ksettings(struct net_device *ndev,
					    const struct ethtool_link_ksettings *cmd)
{
	struct anlogic_axienet_local *lp = netdev_priv(ndev);

	return phylink_ethtool_ksettings_set(lp->phylink, cmd);
}

static int anlogic_axienet_ethtools_nway_reset(struct net_device *dev)
{
	struct anlogic_axienet_local *lp = netdev_priv(dev);

	return phylink_ethtool_nway_reset(lp->phylink);
}

static const struct ethtool_ops anlogic_axienet_ethtool_ops = {
	.supported_coalesce_params = ETHTOOL_COALESCE_MAX_FRAMES |
				     ETHTOOL_COALESCE_USECS,
	.get_drvinfo    = axienet_ethtools_get_drvinfo,
	.get_regs_len   = axienet_ethtools_get_regs_len,
	.get_regs       = axienet_ethtools_get_regs,
	.get_link       = ethtool_op_get_link,
	.get_ringparam	= anlogic_axienet_ethtools_get_ringparam,
	.set_ringparam	= anlogic_axienet_ethtools_set_ringparam,
	.get_pauseparam = anlogic_axienet_ethtools_get_pauseparam,
	.set_pauseparam = anlogic_axienet_ethtools_set_pauseparam,
	.get_coalesce   = anlogic_axienet_ethtools_get_coalesce,
	.set_coalesce   = anlogic_axienet_ethtools_set_coalesce,
	.get_link_ksettings = anlogic_axienet_ethtools_get_link_ksettings,
	.set_link_ksettings = anlogic_axienet_ethtools_set_link_ksettings,
	.nway_reset	= anlogic_axienet_ethtools_nway_reset,
};

static struct anlogic_axienet_local *anlogic_pcs_to_axienet_local(struct phylink_pcs *pcs)
{
	return container_of(pcs, struct anlogic_axienet_local, pcs);
}

static void anlogic_axienet_pcs_get_state(struct phylink_pcs *pcs,
					  struct phylink_link_state *state)
{
	struct mdio_device *pcs_phy = anlogic_pcs_to_axienet_local(pcs)->pcs_phy;

	phylink_mii_c22_pcs_get_state(pcs_phy, state);
}

static void anlogic_axienet_pcs_an_restart(struct phylink_pcs *pcs)
{
	struct mdio_device *pcs_phy = anlogic_pcs_to_axienet_local(pcs)->pcs_phy;

	phylink_mii_c22_pcs_an_restart(pcs_phy);
}

static int anlogic_axienet_pcs_config(struct phylink_pcs *pcs, unsigned int mode,
				      phy_interface_t interface,
				      const unsigned long *advertising,
				      bool permit_pause_to_mac)
{
	struct mdio_device *pcs_phy = anlogic_pcs_to_axienet_local(pcs)->pcs_phy;
	struct net_device *ndev = anlogic_pcs_to_axienet_local(pcs)->ndev;
	struct anlogic_axienet_local *lp = netdev_priv(ndev);
	int ret;

	if (lp->switch_x_sgmii) {
		ret = mdiodev_write(pcs_phy, ANLOGIC_MII_STD_SELECT_REG,
				    interface == PHY_INTERFACE_MODE_SGMII ?
					ANLOGIC_MII_STD_SELECT_SGMII : 0);
		if (ret < 0) {
			netdev_warn(ndev,
				    "Failed to switch PHY interface: %d\n",
				    ret);
			return ret;
		}
	}

	ret = phylink_mii_c22_pcs_config(pcs_phy, mode, interface, advertising);
	if (ret < 0)
		netdev_warn(ndev, "Failed to configure PCS: %d\n", ret);

	return ret;
}

static const struct phylink_pcs_ops anlogic_axienet_pcs_ops = {
	.pcs_get_state = anlogic_axienet_pcs_get_state,
	.pcs_config = anlogic_axienet_pcs_config,
	.pcs_an_restart = anlogic_axienet_pcs_an_restart,
};

static struct phylink_pcs *anlogic_axienet_mac_select_pcs(struct phylink_config *config,
							  phy_interface_t interface)
{
	struct net_device *ndev = to_net_dev(config->dev);
	struct anlogic_axienet_local *lp = netdev_priv(ndev);

	if (interface == PHY_INTERFACE_MODE_1000BASEX ||
	    interface ==  PHY_INTERFACE_MODE_SGMII)
		return &lp->pcs;

	return NULL;
}

static void anlogic_axienet_mac_config(struct phylink_config *config, unsigned int mode,
				       const struct phylink_link_state *state)
{
	/* nothing meaningful to do */
}

static void anlogic_axienet_mac_link_down(struct phylink_config *config,
					  unsigned int mode,
					  phy_interface_t interface)
{
	/* nothing meaningful to do */
}

static void anlogic_axienet_mac_link_up(struct phylink_config *config,
					struct phy_device *phy,
					unsigned int mode, phy_interface_t interface,
					int speed, int duplex,
					bool tx_pause, bool rx_pause)
{
	struct net_device *ndev = to_net_dev(config->dev);
	struct anlogic_axienet_local *lp = netdev_priv(ndev);
	u32 emmc_reg, fcc_reg;

	emmc_reg = anlogic_axienet_ior(lp, AE_EMMC_OFFSET);
	emmc_reg &= ~AE_EMMC_LINKSPEED_MASK;

	switch (speed) {
	case SPEED_1000:
		emmc_reg |= AE_EMMC_LINKSPD_1000;
		break;
	case SPEED_100:
		emmc_reg |= AE_EMMC_LINKSPD_100;
		break;
	case SPEED_10:
		emmc_reg |= AE_EMMC_LINKSPD_10;
		break;
	default:
		dev_err(&ndev->dev,
			"Speed other than 10, 100 or 1Gbps is not supported\n");
		break;
	}

	anlogic_axienet_iow(lp, AE_EMMC_OFFSET, emmc_reg);

	fcc_reg = anlogic_axienet_ior(lp, AE_FCC_OFFSET);
	if (tx_pause)
		fcc_reg |= AE_FCC_FCTX_MASK;
	else
		fcc_reg &= ~AE_FCC_FCTX_MASK;
	if (rx_pause)
		fcc_reg |= AE_FCC_FCRX_MASK;
	else
		fcc_reg &= ~AE_FCC_FCRX_MASK;
	anlogic_axienet_iow(lp, AE_FCC_OFFSET, fcc_reg);
}

static const struct phylink_mac_ops anlogic_axienet_phylink_ops = {
	.validate = phylink_generic_validate,
	.mac_select_pcs = anlogic_axienet_mac_select_pcs,
	.mac_config = anlogic_axienet_mac_config,
	.mac_link_down = anlogic_axienet_mac_link_down,
	.mac_link_up = anlogic_axienet_mac_link_up,
};

/**
 * anlogic_axienet_dma_err_handler - Work queue task for Axi DMA Error
 * @work:	pointer to work_struct
 *
 * Resets the Axi DMA and Axi Ethernet devices, and reconfigures the
 * Tx/Rx BDs.
 */
static void anlogic_axienet_dma_err_handler(struct work_struct *work)
{
	u32 i;
	u32 axienet_status;
	struct anlogic_axidma_bd *cur_p;
	struct anlogic_axienet_local *lp = container_of(work, struct anlogic_axienet_local,
						dma_err_task);
	struct net_device *ndev = lp->ndev;

	/* Don't bother if we are going to stop anyway */
	if (READ_ONCE(lp->stopping))
		return;

	napi_disable(&lp->napi_tx);
	napi_disable(&lp->napi_rx);

	anlogic_axienet_setoptions(ndev, lp->options &
				   ~(AE_OPTION_TXEN | AE_OPTION_RXEN));

	anlogic_axienet_dma_stop(lp);

	for (i = 0; i < lp->tx_bd_num; i++) {
		cur_p = &lp->tx_bd_v[i];
		if (cur_p->cntrl) {
			dma_addr_t addr = anlogic_desc_get_phys_addr(lp, cur_p);

			dma_unmap_single(lp->dev, addr,
					 (cur_p->cntrl &
					  AXIDMA_BD_CTRL_LENGTH_MASK),
					 DMA_TO_DEVICE);
		}
		if (cur_p->skb)
			dev_kfree_skb_irq(cur_p->skb);
		cur_p->phys = 0;
		cur_p->phys_msb = 0;
		cur_p->cntrl = 0;
		cur_p->status = 0;
		cur_p->app0 = 0;
		cur_p->app1 = 0;
		cur_p->app2 = 0;
		cur_p->app3 = 0;
		cur_p->app4 = 0;
		cur_p->skb = NULL;
	}

	for (i = 0; i < lp->rx_bd_num; i++) {
		cur_p = &lp->rx_bd_v[i];
		cur_p->status = 0;
		cur_p->app0 = 0;
		cur_p->app1 = 0;
		cur_p->app2 = 0;
		cur_p->app3 = 0;
		cur_p->app4 = 0;
	}

	lp->tx_bd_ci = 0;
	lp->tx_bd_tail = 0;
	lp->rx_bd_ci = 0;

	anlogic_axienet_dma_start(lp);

	axienet_status = anlogic_axienet_ior(lp, AE_RCW1_OFFSET);
	axienet_status &= ~AE_RCW1_RX_MASK;
	anlogic_axienet_iow(lp, AE_RCW1_OFFSET, axienet_status);

	axienet_status = anlogic_axienet_ior(lp, AE_IP_OFFSET);
	if (axienet_status & AE_INT_RXRJECT_MASK)
		anlogic_axienet_iow(lp, AE_IS_OFFSET, AE_INT_RXRJECT_MASK);
	anlogic_axienet_iow(lp, AE_IE_OFFSET, lp->eth_irq > 0 ?
			    AE_INT_RECV_ERROR_MASK : 0);
	anlogic_axienet_iow(lp, AE_FCC_OFFSET, AE_FCC_FCRX_MASK);

	/* Sync default options with HW but leave receiver and
	 * transmitter disabled.
	 */
	anlogic_axienet_setoptions(ndev, lp->options &
				   ~(AE_OPTION_TXEN | AE_OPTION_RXEN));
	anlogic_axienet_set_mac_address(ndev, NULL);
	anlogic_axienet_set_multicast_list(ndev);
	napi_enable(&lp->napi_rx);
	napi_enable(&lp->napi_tx);
	anlogic_axienet_setoptions(ndev, lp->options);
}

/**
 * anlogic_axienet_probe - Axi Ethernet probe function.
 * @pdev:	Pointer to platform device structure.
 *
 * Return: 0, on success
 *	    Non-zero error value on failure.
 *
 * This is the probe routine for Axi Ethernet driver. This is called before
 * any other driver routines are invoked. It allocates and sets up the Ethernet
 * device. Parses through device tree and populates fields of
 * anlogic_axienet_local. It registers the Ethernet device.
 */
static int anlogic_axienet_probe(struct platform_device *pdev)
{
	int ret;
	struct device_node *np;
	struct anlogic_axienet_local *lp;
	struct net_device *ndev;
	struct resource *ethres;
	u8 mac_addr[ETH_ALEN];
	int addr_width = 32;
	u32 value;

	ndev = alloc_etherdev(sizeof(*lp));
	if (!ndev)
		return -ENOMEM;

	platform_set_drvdata(pdev, ndev);

	SET_NETDEV_DEV(ndev, &pdev->dev);
	ndev->flags &= ~IFF_MULTICAST;  /* clear multicast */
	ndev->features = NETIF_F_SG;
	ndev->netdev_ops = &axienet_netdev_ops;
	ndev->ethtool_ops = &anlogic_axienet_ethtool_ops;

	/* MTU range: 64 - 9000 */
	ndev->min_mtu = 64;
	ndev->max_mtu = AE_JUMBO_MTU;

	lp = netdev_priv(ndev);
	lp->ndev = ndev;
	lp->dev = &pdev->dev;
	lp->options = AE_OPTION_DEFAULTS;
	lp->rx_bd_num = RX_BD_NUM_DEFAULT;
	lp->tx_bd_num = TX_BD_NUM_DEFAULT;

	u64_stats_init(&lp->rx_stat_sync);
	u64_stats_init(&lp->tx_stat_sync);

	netif_napi_add(ndev, &lp->napi_rx, anlogic_axienet_rx_poll);
	netif_napi_add(ndev, &lp->napi_tx, anlogic_axienet_tx_poll);

	lp->axi_clk = devm_clk_get_optional(&pdev->dev, "s_axi_lite_clk");
	if (!lp->axi_clk) {
		/* For backward compatibility, if named AXI clock is not present,
		 * treat the first clock specified as the AXI clock.
		 */
		lp->axi_clk = devm_clk_get_optional(&pdev->dev, NULL);
	}
	if (IS_ERR(lp->axi_clk)) {
		ret = PTR_ERR(lp->axi_clk);
		goto free_netdev;
	}
	ret = clk_prepare_enable(lp->axi_clk);
	if (ret) {
		dev_err(&pdev->dev, "Unable to enable AXI clock: %d\n", ret);
		goto free_netdev;
	}

	lp->misc_clks[0].id = "axis_clk";
	lp->misc_clks[1].id = "ref_clk";

	ret = devm_clk_bulk_get_optional(&pdev->dev, AE_NUM_MISC_CLOCKS, lp->misc_clks);
	if (ret)
		goto cleanup_clk;

	ret = clk_bulk_prepare_enable(AE_NUM_MISC_CLOCKS, lp->misc_clks);
	if (ret)
		goto cleanup_clk;

	/* Map device registers */
	lp->regs = devm_platform_get_and_ioremap_resource(pdev, 0, &ethres);
	if (IS_ERR(lp->regs)) {
		ret = PTR_ERR(lp->regs);
		goto cleanup_clk;
	}
	lp->regs_start = ethres->start;

	/* Setup checksum offload, but default to off if not specified */
	lp->features = 0;

	ret = of_property_read_u32(pdev->dev.of_node, "anlogic,txcsum", &value);
	if (!ret) {
		switch (value) {
		case 1:
			lp->csum_offload_on_tx_path =
				AE_FEATURE_PARTIAL_TX_CSUM;
			lp->features |= AE_FEATURE_PARTIAL_TX_CSUM;
			/* Can checksum TCP/UDP over IPv4. */
			ndev->features |= NETIF_F_IP_CSUM;
			break;
		case 2:
			lp->csum_offload_on_tx_path =
				AE_FEATURE_FULL_TX_CSUM;
			lp->features |= AE_FEATURE_FULL_TX_CSUM;
			/* Can checksum TCP/UDP over IPv4. */
			ndev->features |= NETIF_F_IP_CSUM;
			break;
		default:
			lp->csum_offload_on_tx_path = AE_NO_CSUM_OFFLOAD;
		}
	}
	ret = of_property_read_u32(pdev->dev.of_node, "anlogic,rxcsum", &value);
	if (!ret) {
		switch (value) {
		case 1:
			lp->csum_offload_on_rx_path =
				AE_FEATURE_PARTIAL_RX_CSUM;
			lp->features |= AE_FEATURE_PARTIAL_RX_CSUM;
			break;
		case 2:
			lp->csum_offload_on_rx_path =
				AE_FEATURE_FULL_RX_CSUM;
			lp->features |= AE_FEATURE_FULL_RX_CSUM;
			break;
		default:
			lp->csum_offload_on_rx_path = AE_NO_CSUM_OFFLOAD;
		}
	}
	/* For supporting jumbo frames, the Axi Ethernet hardware must have
	 * a larger Rx/Tx Memory. Typically, the size must be large so that
	 * we can enable jumbo option and start supporting jumbo frames.
	 * Here we check for memory allocated for Rx/Tx in the hardware from
	 * the device-tree and accordingly set flags.
	 */
	of_property_read_u32(pdev->dev.of_node, "anlogic,rxmem", &lp->rxmem);

	lp->switch_x_sgmii = of_property_read_bool(pdev->dev.of_node,
						   "anlogic,switch-x-sgmii");

	/* Start with the proprietary, and broken phy_type */
	ret = of_property_read_u32(pdev->dev.of_node, "anlogic,phy-type", &value);
	if (!ret) {
		netdev_warn(ndev, "Please upgrade your device tree binary blob to use phy-mode");
		switch (value) {
		case AE_PHY_TYPE_MII:
			lp->phy_mode = PHY_INTERFACE_MODE_MII;
			break;
		case AE_PHY_TYPE_GMII:
			lp->phy_mode = PHY_INTERFACE_MODE_GMII;
			break;
		case AE_PHY_TYPE_RGMII_2_0:
			lp->phy_mode = PHY_INTERFACE_MODE_RGMII_ID;
			break;
		case AE_PHY_TYPE_SGMII:
			lp->phy_mode = PHY_INTERFACE_MODE_SGMII;
			break;
		case AE_PHY_TYPE_1000BASE_X:
			lp->phy_mode = PHY_INTERFACE_MODE_1000BASEX;
			break;
		default:
			ret = -EINVAL;
			goto cleanup_clk;
		}
	} else {
		ret = of_get_phy_mode(pdev->dev.of_node, &lp->phy_mode);
		if (ret)
			goto cleanup_clk;
	}
	if (lp->switch_x_sgmii && lp->phy_mode != PHY_INTERFACE_MODE_SGMII &&
	    lp->phy_mode != PHY_INTERFACE_MODE_1000BASEX) {
		dev_err(&pdev->dev, "anlogic,switch-x-sgmii only supported with SGMII or 1000BaseX\n");
		ret = -EINVAL;
		goto cleanup_clk;
	}

	/* Find the DMA node, map the DMA registers, and decode the DMA IRQs */
	np = of_parse_phandle(pdev->dev.of_node, "axistream-connected", 0);
	if (np) {
		struct resource dmares;

		ret = of_address_to_resource(np, 0, &dmares);
		if (ret) {
			dev_err(&pdev->dev,
				"unable to get DMA resource\n");
			of_node_put(np);
			goto cleanup_clk;
		}
		lp->dma_regs = devm_ioremap_resource(&pdev->dev,
						     &dmares);
		lp->rx_irq = irq_of_parse_and_map(np, 1);
		lp->tx_irq = irq_of_parse_and_map(np, 0);
		of_node_put(np);
		lp->eth_irq = platform_get_irq_optional(pdev, 0);
	} else {
		/* Check for these resources directly on the Ethernet node. */
		lp->dma_regs = devm_platform_get_and_ioremap_resource(pdev, 1, NULL);
		lp->rx_irq = platform_get_irq(pdev, 1);
		lp->tx_irq = platform_get_irq(pdev, 0);
		lp->eth_irq = platform_get_irq_optional(pdev, 2);
	}
	if (IS_ERR(lp->dma_regs)) {
		dev_err(&pdev->dev, "could not map DMA regs\n");
		ret = PTR_ERR(lp->dma_regs);
		goto cleanup_clk;
	}
	if (lp->rx_irq <= 0 || lp->tx_irq <= 0) {
		dev_err(&pdev->dev, "could not determine irqs\n");
		ret = -ENOMEM;
		goto cleanup_clk;
	}

	/* Reset core now that clocks are enabled, prior to accessing MDIO */
	ret = __anlogic_axienet_device_reset(lp);
	if (ret)
		goto cleanup_clk;

	/* Autodetect the need for 64-bit DMA pointers.
	 * When the IP is configured for a bus width bigger than 32 bits,
	 * writing the MSB registers is mandatory, even if they are all 0.
	 * We can detect this case by writing all 1's to one such register
	 * and see if that sticks: when the IP is configured for 32 bits
	 * only, those registers are RES0.
	 * Those MSB registers were introduced in IP v7.1, which we check first.
	 */
	if ((anlogic_axienet_ior(lp, AE_ID_OFFSET) >> 24) >= 0x9) {
		void __iomem *desc = lp->dma_regs + AXIDMA_TX_CDESC_OFFSET + 4;

		iowrite32(0x0, desc);
		if (ioread32(desc) == 0) {	/* sanity check */
			iowrite32(0xffffffff, desc);
			if (ioread32(desc) > 0) {
				lp->features |= AE_FEATURE_DMA_64BIT;
				addr_width = 64;
				dev_info(&pdev->dev,
					 "autodetected 64-bit DMA range\n");
			}
			iowrite32(0x0, desc);
		}
	}
	if (!IS_ENABLED(CONFIG_64BIT) && lp->features & AE_FEATURE_DMA_64BIT) {
		dev_err(&pdev->dev, "64-bit addressable DMA is not compatible with 32-bit archecture\n");
		ret = -EINVAL;
		goto cleanup_clk;
	}

	ret = dma_set_mask_and_coherent(&pdev->dev, DMA_BIT_MASK(addr_width));
	if (ret) {
		dev_err(&pdev->dev, "No suitable DMA available\n");
		goto cleanup_clk;
	}

	/* Check for Ethernet core IRQ (optional) */
	if (lp->eth_irq <= 0)
		dev_info(&pdev->dev, "Ethernet core IRQ not defined\n");

	/* Retrieve the MAC address */
	ret = of_get_mac_address(pdev->dev.of_node, mac_addr);
	if (!ret) {
		anlogic_axienet_set_mac_address(ndev, mac_addr);
	} else {
		dev_warn(&pdev->dev, "could not find MAC address property: %d\n",
			 ret);
		anlogic_axienet_set_mac_address(ndev, NULL);
	}

	lp->coalesce_count_rx = AXIDMA_DFT_RX_THRESHOLD;
	lp->coalesce_usec_rx = AXIDMA_DFT_RX_USEC;
	lp->coalesce_count_tx = AXIDMA_DFT_TX_THRESHOLD;
	lp->coalesce_usec_tx = AXIDMA_DFT_TX_USEC;

	ret = anlogic_axienet_mdio_setup(lp);
	if (ret)
		dev_warn(&pdev->dev,
			 "error registering MDIO bus: %d\n", ret);

	if (lp->phy_mode == PHY_INTERFACE_MODE_SGMII ||
	    lp->phy_mode == PHY_INTERFACE_MODE_1000BASEX) {
		np = of_parse_phandle(pdev->dev.of_node, "pcs-handle", 0);
		if (!np) {
			/* Deprecated: Always use "pcs-handle" for pcs_phy.
			 * Falling back to "phy-handle" here is only for
			 * backward compatibility with old device trees.
			 */
			np = of_parse_phandle(pdev->dev.of_node, "phy-handle", 0);
		}
		if (!np) {
			dev_err(&pdev->dev, "pcs-handle (preferred) or phy-handle required for 1000BaseX/SGMII\n");
			ret = -EINVAL;
			goto cleanup_mdio;
		}
		lp->pcs_phy = of_mdio_find_device(np);
		if (!lp->pcs_phy) {
			ret = -EPROBE_DEFER;
			of_node_put(np);
			goto cleanup_mdio;
		}
		of_node_put(np);
		lp->pcs.ops = &anlogic_axienet_pcs_ops;
		lp->pcs.poll = true;
	}

	lp->phylink_config.dev = &ndev->dev;
	lp->phylink_config.type = PHYLINK_NETDEV;
	lp->phylink_config.mac_capabilities = MAC_SYM_PAUSE | MAC_ASYM_PAUSE |
		MAC_10FD | MAC_100FD | MAC_1000FD;

	__set_bit(lp->phy_mode, lp->phylink_config.supported_interfaces);
	if (lp->switch_x_sgmii) {
		__set_bit(PHY_INTERFACE_MODE_1000BASEX,
			  lp->phylink_config.supported_interfaces);
		__set_bit(PHY_INTERFACE_MODE_SGMII,
			  lp->phylink_config.supported_interfaces);
	}

	lp->phylink = phylink_create(&lp->phylink_config, pdev->dev.fwnode,
				     lp->phy_mode,
				     &anlogic_axienet_phylink_ops);
	if (IS_ERR(lp->phylink)) {
		ret = PTR_ERR(lp->phylink);
		dev_err(&pdev->dev, "phylink_create error (%i)\n", ret);
		goto cleanup_mdio;
	}

	ret = register_netdev(lp->ndev);
	if (ret) {
		dev_err(lp->dev, "register_netdev() error (%i)\n", ret);
		goto cleanup_phylink;
	}

	return 0;

cleanup_phylink:
	phylink_destroy(lp->phylink);

cleanup_mdio:
	if (lp->pcs_phy)
		put_device(&lp->pcs_phy->dev);
	if (lp->mii_bus)
		anlogic_axienet_mdio_teardown(lp);
cleanup_clk:
	clk_bulk_disable_unprepare(AE_NUM_MISC_CLOCKS, lp->misc_clks);
	clk_disable_unprepare(lp->axi_clk);

free_netdev:
	free_netdev(ndev);

	return ret;
}

static int anlogic_axienet_remove(struct platform_device *pdev)
{
	struct net_device *ndev = platform_get_drvdata(pdev);
	struct anlogic_axienet_local *lp = netdev_priv(ndev);

	unregister_netdev(ndev);

	if (lp->phylink)
		phylink_destroy(lp->phylink);

	if (lp->pcs_phy)
		put_device(&lp->pcs_phy->dev);

	anlogic_axienet_mdio_teardown(lp);

	clk_bulk_disable_unprepare(AE_NUM_MISC_CLOCKS, lp->misc_clks);
	clk_disable_unprepare(lp->axi_clk);

	free_netdev(ndev);

	return 0;
}

static void anlogic_axienet_shutdown(struct platform_device *pdev)
{
	struct net_device *ndev = platform_get_drvdata(pdev);

	rtnl_lock();
	netif_device_detach(ndev);

	if (netif_running(ndev))
		dev_close(ndev);

	rtnl_unlock();
}

static struct platform_driver anlogic_axienet_driver = {
	.probe = anlogic_axienet_probe,
	.remove = anlogic_axienet_remove,
	.shutdown = anlogic_axienet_shutdown,
	.driver = {
		 .name = "anlogic_axienet",
		 .of_match_table = axienet_of_match,
	},
};

module_platform_driver(anlogic_axienet_driver);

MODULE_DESCRIPTION("Anlogic Axi Ethernet driver");
MODULE_AUTHOR("Anlogic");
MODULE_LICENSE("GPL");
