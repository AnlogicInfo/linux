// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Anlogic EthLite Linux driver for the Anlogic Ethernet MAC Lite device.
 *
 * Copyright (C) 2023 Anlogic, Inc.
 *
 * Author: HuangJian <jian.huang@anlogic.com>
 */

#include <linux/module.h>
#include <linux/uaccess.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/skbuff.h>
#include <linux/ethtool.h>
#include <linux/io.h>
#include <linux/slab.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/of_platform.h>
#include <linux/of_mdio.h>
#include <linux/of_net.h>
#include <linux/phy.h>
#include <linux/interrupt.h>
#include <linux/iopoll.h>

#define DRIVER_NAME "anlogic_ethlite"

/* Anlogic EthLite Register Map */
#define ELITE_ETXSTAT0_OFFSET   (0x00ULL)
#define ELITE_ERXSTAT0_OFFSET   (0x04ULL)
#define ELITE_ETXSTAT1_OFFSET   (0x08ULL)
#define ELITE_ERXSTAT1_OFFSET   (0x0CULL)
#define ELITE_EIC_OFFSET		(0x18ULL)
#define ELITE_EIP_OFFSET		(0x1CULL)
#define ELITE_MDIOCCF_OFFSET	(0xD0ULL)
#define ELITE_MDIOCTL_OFFSET	(0xD4ULL)
#define ELITE_MDIORD_OFFSET		(0xD8ULL)
#define ELITE_MDIOWR_OFFSET		(0xDCULL)
#define ELITE_ESET0_OFFSET		(0xE0ULL)
#define ELITE_ERST_OFFSET		(0xF0ULL)
#define ELITE_EMACL_OFFSET		(0xF4ULL)
#define ELITE_EMACH_OFFSET		(0xF8ULL)
#define ELITE_IPINFO_OFFSET		(0xFCULL)
#define ELITE_TXPINGBUF_OFFSET	(0x2000ULL)
#define ELITE_TXPONGBUF_OFFSET	(0x3000ULL)
#define ELITE_RXPINGBUF_OFFSET	(0x4000ULL)
#define ELITE_RXPONGBUF_OFFSET	(0x5000ULL)

/* EthLite TX Status0 Register */
#define ETXSTAT0_BUFSTAT_SHIFT		31
#define ETXSTAT0_SENDREQ_SHIFT		30
#define ETXSTAT0_FRAMELEN_SHIFT		0

/* EthLite RX Status0 Register */
#define ERXSTAT0_BUFSTAT_SHIFT		31
#define ERXSTAT0_CLRREQ_SHIFT	   30
#define ERXSTAT0_FRAMELEN_SHIFT		0

/* EthLite TX Status1 Register */
#define ETXSTAT1_BUFSTAT_SHIFT		31
#define ETXSTAT1_SENDREQ_SHIFT		30
#define ETXSTAT1_FRAMELEN_SHIFT		0

/* EthLite RX Status1 Register */
#define ERXSTAT1_BUFSTAT_SHIFT		31
#define ERXSTAT1_CLRREQ_SHIFT		30
#define ERXSTAT1_FRAMELEN_SHIFT		0

/* EthLite Interrupt Control Register */
#define EIC_RXBADPKT_SHIFT			7
#define EIC_TXDONE_SHIFT			2
#define EIC_TXIDLE_SHIFT			1
#define EIC_RXVALID_SHIFT			0

/* EthLite Interrupt Pending Register */
#define EIP_RXBADPKT_SHIFT			7
#define EIP_TXDONE_SHIFT			2
#define EIP_TXIDLE_SHIFT			1
#define EIP_RXVALID_SHIFT			0

/* EthLite MDIO Clk Control Register */
#define MDIOCCF_MDIOEN_SHIFT		31
#define MDIOCCF_MDIODIV_SHIFT		0

/* EthLite MDIO Control Register */
#define MDIOCTL_REGAD_SHIFT			8
#define MDIOCTL_PHYAD_SHIFT			3
#define MDIOCTL_RWSEL_SHIFT			1
#define MDIOCTL_ACTTRIG_SHIFT		0

/* EthLite Mac Configure Register */
/* Set Mac Speed Configure */
#define ESET0_SPEEDCFG_SHIFT		30
/* Set Tx IFG */
#define ESET0_TXIFG_SHIFT			16
/* Set Rx Enable */
#define ESET0_RXEN_SHIFT			15
/* Set Rx Flow Control Enable */
#define ESET0_RXFLOWCTRLEN_SHIFT	14
/* Set Rx Length Type Check Enable */
#define ESET0_RXLTCHKEN_SHIFT		13
/* Set Rx Control Frame Check Enable */
#define ESET0_RXCFCHKEN_SHIFT		12
/* Set Rx Jumpbo Package Enable */
#define ESET0_RXJUMBOEN_SHIFT		11
/* Set Rx FCS Check Enable */
#define ESET0_RXFCSCHKEN_SHIFT		10
/* Set Rx Vlan Enable */
#define ESET0_RXVLANEN_SHIFT		9
/* Set Rx Half-duplex Receive Enable */
#define ESET0_RXSEMIEN_SHIFT		8
/* Set Tx Enable */
#define ESET0_TXEN_SHIFT			7
/* Set Tx Flow Control Enable */
#define ESET0_TXFLOWCTRLEN_SHIFT	6
/* Set Tx InterFrameGap Adjustment Enable */
#define ESET0_TXGAPADJEN_SHIFT		5
/* Set Tx Jumpbo Package Enable */
#define ESET0_TXJUMBOEN_SHIFT		3
/* Set Tx FCS Generate Enable */
#define ESET0_TXFCSGENEN_SHIFT		2
/* Set Tx Vlan Enable */
#define ESET0_TXVLANEN_SHIFT		1
/* Set Tx Half-duplex Send Enable */
#define ESET0_TXSEMIEN_SHIFT		0

/* EthLite Mac Reset Register */
#define ERST_EMACRST_SHIFT			31
#define ERST_RXRST_SHIFT			1
#define ERST_TXRST_SHIFT			0

#define CONFIG_1000MB_SPEED			0x2
#define CONFIG_100MB_SPEED			0x1
#define CONFIG_10MB_SPEED			0x0

#define MDIO_WR_SEL 0x1
#define MDIO_RD_SEL 0x2

#define MDIO_STOP_TRANS 0x0
#define MDIO_START_TRANS 0x1

#define MDIO_ENABLE 0x1
#define MDIO_DISABLE 0x0

#define TX_TIMEOUT		(60 * HZ)	/* Tx timeout is 60 seconds. */

#ifdef __BIG_ENDIAN
#define al_ethlite_readl		ioread32be
#define al_ethlite_writel	iowrite32be
#else
#define al_ethlite_readl		ioread32
#define al_ethlite_writel	iowrite32
#endif

/**
 * struct net_local - Our private per device data
 * @ndev:		instance of the network device
 * @next_tx_buf_to_use:	next Tx buffer to write to
 * @next_rx_buf_to_use:	next Rx buffer to read from
 * @base_addr:		base address of the Ethlite device
 * @reset_lock:		lock to serialize xmit and tx_timeout execution
 * @deferred_skb:	holds an skb (for transmission at a later time) when the
 *			Tx buffer is not free
 * @phy_dev:		pointer to the PHY device
 * @phy_node:		pointer to the PHY device node
 * @mii_bus:		pointer to the MII bus
 * @last_link:		last link status
 */
struct net_local {
	struct net_device *ndev;

	u32 next_tx_buf_to_use;
	u32 next_rx_buf_to_use;
	void __iomem *base_addr;

	spinlock_t reset_lock; /* serialize xmit and tx_timeout execution */
	struct sk_buff *deferred_skb;

	struct phy_device *phy_dev;
	struct device_node *phy_node;

	struct mii_bus *mii_bus;

	int last_link;
};

/*************************/
/* EthLite driver calls */
/*************************/

/**
 * al_ethlite_enable_interrupts - Enable the interrupts for the EthLite device
 * @drvdata:	Pointer to the Ethlite device private data
 *
 * This function enables  interrupt. | (1 << EIP_TXIDLE_SHIFT) | (1 << EIP_TXDONE_SHIFT)
 */
static void al_ethlite_enable_interrupts(struct net_local *drvdata)
{
	/* Clear all interrupts flag */
	al_ethlite_writel(0xffffffff, drvdata->base_addr + ELITE_EIP_OFFSET);

	/* Enable the interrupts */
	al_ethlite_writel((1 << EIP_RXBADPKT_SHIFT)
					 | (1 << EIP_RXVALID_SHIFT) ,
					drvdata->base_addr + ELITE_EIC_OFFSET);
}

/**
 * al_ethlite_disable_interrupts - Disable the interrupts for the EthLite device
 * @drvdata:	Pointer to the Ethlite device private data
 *
 * This function disables the interrupts.
 */
static void al_ethlite_disable_interrupts(struct net_local *drvdata)
{
	/* Disable the interrupts */
	al_ethlite_writel(0, drvdata->base_addr + ELITE_EIC_OFFSET);

	/* Clear all interrupts flag */
	al_ethlite_writel(0xffffffff, drvdata->base_addr + ELITE_EIP_OFFSET);
}

/**
 * al_ethlite_aligned_write - Write from 16-bit aligned to 32-bit aligned address
 * @src_ptr:	Void pointer to the 16-bit aligned source address
 * @dest_ptr:	Pointer to the 32-bit aligned destination address
 * @length:	Number bytes to write from source to destination
 *
 * This function writes data from a 16-bit aligned buffer to a 32-bit aligned
 * address in the EmacLite device.
 */
static void al_ethlite_aligned_write(const void *src_ptr, u32 *dest_ptr,
					unsigned int length)
{
	const u16 *from_u16_ptr;
	u32 align_buffer;
	u32 *to_u32_ptr;
	u16 *to_u16_ptr;

	to_u32_ptr = dest_ptr;
	from_u16_ptr = src_ptr;
	align_buffer = 0;

	for (; length > 3; length -= 4) {
		to_u16_ptr = (u16 *)&align_buffer;
		*to_u16_ptr++ = *from_u16_ptr++;
		*to_u16_ptr++ = *from_u16_ptr++;

		/* This barrier resolves occasional issues seen around
		 * cases where the data is not properly flushed out
		 * from the processor store buffers to the destination
		 * memory locations.
		 */
		wmb();

		/* Output a word */
		*to_u32_ptr++ = align_buffer;
	}
	if (length) {
		u8 *from_u8_ptr, *to_u8_ptr;

		/* Set up to output the remaining data */
		align_buffer = 0;
		to_u8_ptr = (u8 *)&align_buffer;
		from_u8_ptr = (u8 *)from_u16_ptr;

		/* Output the remaining data */
		for (; length > 0; length--)
			*to_u8_ptr++ = *from_u8_ptr++;

		/* This barrier resolves occasional issues seen around
		 * cases where the data is not properly flushed out
		 * from the processor store buffers to the destination
		 * memory locations.
		 */
		wmb();
		*to_u32_ptr = align_buffer;
	}
}

/**
 * al_ethlite_aligned_read - Read from 32-bit aligned to 16-bit aligned buffer
 * @src_ptr:	Pointer to the 32-bit aligned source address
 * @dest_ptr:	Pointer to the 16-bit aligned destination address
 * @length:	Number bytes to read from source to destination
 *
 * This function reads data from a 32-bit aligned address in the EmacLite device
 * to a 16-bit aligned buffer.
 */
static void al_ethlite_aligned_read(u32 *src_ptr, u8 *dest_ptr,
				   unsigned int length)
{
	u16 *to_u16_ptr, *from_u16_ptr;
	u32 *from_u32_ptr;
	u32 align_buffer;

	from_u32_ptr = src_ptr;
	to_u16_ptr = (u16 *)dest_ptr;

	for (; length > 3; length -= 4) {
		/* Copy each word into the temporary buffer */
		align_buffer = *from_u32_ptr++;
		from_u16_ptr = (u16 *)&align_buffer;

		/* Read data from source */
		*to_u16_ptr++ = *from_u16_ptr++;
		*to_u16_ptr++ = *from_u16_ptr++;
	}

	if (length) {
		u8 *to_u8_ptr, *from_u8_ptr;

		/* Set up to read the remaining data */
		to_u8_ptr = (u8 *)to_u16_ptr;
		align_buffer = *from_u32_ptr++;
		from_u8_ptr = (u8 *)&align_buffer;

		/* Read the remaining data */
		for (; length > 0; length--)
			*to_u8_ptr = *from_u8_ptr;
	}
}

/**
 * al_ethlite_send_data - Send an Ethernet frame
 * @drvdata:	Pointer to the Ethlite device private data
 * @data:	Pointer to the data to be sent
 * @byte_count:	Total frame size, including header
 *
 * This function checks if the Tx buffer of the Ethlite device is free to send
 * data. If so, it fills the Tx buffer with data for transmission. Otherwise, it
 * returns an error.
 *
 * Return:	0 upon success or -1 if the buffer(s) are full.
 *
 * Note:	The maximum Tx packet size can not be more than Ethernet header
 *		(14 Bytes) + Maximum MTU (1500 bytes). This is excluding FCS.
 */
static int al_ethlite_send_data(struct net_local *drvdata, u8 *data,
								unsigned int byte_count)
{
	void __iomem *addr;

	if ((al_ethlite_readl(drvdata->base_addr + ELITE_ETXSTAT0_OFFSET)
						& BIT(ETXSTAT0_BUFSTAT_SHIFT)) == 0) {
		drvdata->next_tx_buf_to_use = ELITE_TXPINGBUF_OFFSET;
	} else if ((al_ethlite_readl(drvdata->base_addr + ELITE_ETXSTAT1_OFFSET)
						& BIT(ETXSTAT1_BUFSTAT_SHIFT)) == 0) {
		drvdata->next_tx_buf_to_use = ELITE_TXPONGBUF_OFFSET;
	} else {
		return -1; /* Buffer was full, return failure */
	}

	addr = drvdata->base_addr + drvdata->next_tx_buf_to_use;

	/* If the length is too large, truncate it */
	if (byte_count > ETH_FRAME_LEN)
		byte_count = ETH_FRAME_LEN;

	/* Write the frame to the buffer */
	al_ethlite_aligned_write(data, (u32 __force *)addr, byte_count);

	if(drvdata->next_tx_buf_to_use == ELITE_TXPINGBUF_OFFSET) {
		al_ethlite_writel(BIT(ETXSTAT0_SENDREQ_SHIFT) | (byte_count & 0xfff),
						drvdata->base_addr + ELITE_ETXSTAT0_OFFSET);
	} else {
		al_ethlite_writel(BIT(ETXSTAT1_SENDREQ_SHIFT) | (byte_count & 0xfff),
						drvdata->base_addr + ELITE_ETXSTAT1_OFFSET);
	}

	return 0;
}

/**
 * al_ethlite_recv_data - Receive a frame
 * @drvdata:	Pointer to the Ethlite device private data
 * @data:	Address where the data is to be received
 * @maxlen:	Maximum supported ethernet packet length
 *
 * This function is intended to be called from the interrupt context or
 * with a wrapper which waits for the receive frame to be available.
 *
 * Return:	Total number of bytes received
 */
static u16 al_ethlite_recv_data(struct net_local *drvdata, u8 *data, int maxlen)
{
	void __iomem *addr;
	u16 length;
	u32 reg_data;

	if (al_ethlite_readl(drvdata->base_addr + ELITE_ERXSTAT1_OFFSET) &
		BIT(ERXSTAT1_BUFSTAT_SHIFT)) {
		drvdata->next_rx_buf_to_use = ELITE_RXPONGBUF_OFFSET;
	} else if (al_ethlite_readl(drvdata->base_addr + ELITE_ERXSTAT0_OFFSET) &
		BIT(ERXSTAT0_BUFSTAT_SHIFT)) {
		drvdata->next_rx_buf_to_use = ELITE_RXPINGBUF_OFFSET;
	}else {
		dev_err(&drvdata->ndev->dev, "The rx interrupt was triggered but rx data invalid.\n");
		return 0;
	}

	/* Determine the expected buffer address */
	addr = (drvdata->base_addr + drvdata->next_rx_buf_to_use);

	if(drvdata->next_rx_buf_to_use == ELITE_RXPINGBUF_OFFSET) {
		reg_data = al_ethlite_readl(drvdata->base_addr + ELITE_ERXSTAT0_OFFSET);
		if ((reg_data & 0xfff) == 0) {
			dev_err(&drvdata->ndev->dev, "The rx data0 valid but rx data0 length is 0.\n");
			return 0;
		}
	} else {
		reg_data = al_ethlite_readl(drvdata->base_addr + ELITE_ERXSTAT1_OFFSET);
		if ((reg_data & 0xfff) == 0) {
			dev_err(&drvdata->ndev->dev, "The rx data1 valid but rx data1 length is 0.\n");
			return 0;
		}
	}

	length = reg_data & 0xfff;

	if (WARN_ON(length > maxlen)) {
		netdev_dbg(drvdata->ndev, "length > maxlen,recieve length:%d\n", length);
		length = maxlen;
	}

	/* Read from the EmacLite device */
	al_ethlite_aligned_read((u32 __force *)addr, data, length);

	/* Clear rx request stat */
	if (drvdata->next_rx_buf_to_use == ELITE_RXPINGBUF_OFFSET) {
		reg_data |= BIT(ERXSTAT0_CLRREQ_SHIFT);
		al_ethlite_writel(reg_data,
				  drvdata->base_addr + ELITE_ERXSTAT0_OFFSET);
	} else {
		reg_data |= BIT(ERXSTAT1_CLRREQ_SHIFT);
		al_ethlite_writel(reg_data, drvdata->base_addr +
						ELITE_ERXSTAT1_OFFSET);
	}

	return length;
}

/**
 * al_ethlite_update_address - Update the MAC address in the device
 * @drvdata:	Pointer to the Ethlite device private data
 * @address_ptr:Pointer to the MAC address (MAC address is a 48-bit value)
 *
 * Tx must be idle and Rx should be idle for deterministic results.
 * It is recommended that this function should be called after the
 * initialization and before transmission of any packets from the device.
 * The MAC address can be programmed using any of the two transmit
 * buffers (if configured).
 */
static void al_ethlite_update_address(struct net_local *drvdata,
					 const u8 *address_ptr)
{
	u64 mac_address = *(u64 *)address_ptr;

	al_ethlite_writel(mac_address & 0xffffffff, drvdata->base_addr + ELITE_EMACL_OFFSET);
	al_ethlite_writel(mac_address >> 32, drvdata->base_addr + ELITE_EMACH_OFFSET);
}

/**
 * al_ethlite_set_mac_address - Set the MAC address for this device
 * @dev:	Pointer to the network device instance
 * @address:	Void pointer to the sockaddr structure
 *
 * This function copies the HW address from the sockaddr structure to the
 * net_device structure and updates the address in HW.
 *
 * Return:	Error if the net device is busy or 0 if the addr is set
 *		successfully
 */
static int al_ethlite_set_mac_address(struct net_device *dev, void *address)
{
	struct net_local *lp = netdev_priv(dev);
	struct sockaddr *addr = address;

	if (netif_running(dev))
		return -EBUSY;

	eth_hw_addr_set(dev, addr->sa_data);
	al_ethlite_update_address(lp, dev->dev_addr);
	return 0;
}

/**
 * al_ethlite_tx_timeout - Callback for Tx Timeout
 * @dev:	Pointer to the network device
 * @txqueue:	Unused
 *
 * This function is called when Tx time out occurs for Ethlite device.
 */
static void al_ethlite_tx_timeout(struct net_device *dev, unsigned int txqueue)
{
	struct net_local *lp = netdev_priv(dev);
	unsigned long flags;

	dev_err(&lp->ndev->dev, "Exceeded transmit timeout of %lu ms\n",
		TX_TIMEOUT * 1000UL / HZ);

	dev->stats.tx_errors++;

	/* Reset the device */
	spin_lock_irqsave(&lp->reset_lock, flags);

	/* Shouldn't really be necessary, but shouldn't hurt */
	netif_stop_queue(dev);

	al_ethlite_disable_interrupts(lp);
	al_ethlite_enable_interrupts(lp);

	if (lp->deferred_skb) {
		dev_kfree_skb_irq(lp->deferred_skb);
		lp->deferred_skb = NULL;
		dev->stats.tx_errors++;
	}

	/* To exclude tx timeout */
	netif_trans_update(dev); /* prevent tx timeout */

	/* We're all ready to go. Start the queue */
	netif_wake_queue(dev);
	spin_unlock_irqrestore(&lp->reset_lock, flags);
}

/**********************/
/* Interrupt Handlers */
/**********************/

/**
 * al_ethlite_tx_handler - Interrupt handler for frames sent
 * @dev:	Pointer to the network device
 *
 * This function updates the number of packets transmitted and handles the
 * deferred skb, if there is one.
 */
static void al_ethlite_tx_handler(struct net_device *dev)
{
	struct net_local *lp = netdev_priv(dev);

	al_ethlite_writel((1 << EIP_RXBADPKT_SHIFT) | (1 << EIP_RXVALID_SHIFT),
						 lp->base_addr + ELITE_EIC_OFFSET);
	dev->stats.tx_packets++;

	if (!lp->deferred_skb)
		return;

	if (al_ethlite_send_data(lp, (u8 *)lp->deferred_skb->data,
				lp->deferred_skb->len))
		return;

	dev->stats.tx_bytes += lp->deferred_skb->len;
	dev_consume_skb_irq(lp->deferred_skb);
	lp->deferred_skb = NULL;
	netif_trans_update(dev); /* prevent tx timeout */
	netif_wake_queue(dev);
}


/**
 * al_ethlite_rx_handler- Interrupt handler for frames received
 * @dev:	Pointer to the network device
 *
 * This function allocates memory for a socket buffer, fills it with data
 * received and hands it over to the TCP/IP stack.
 */
static void al_ethlite_rx_handler(struct net_device *dev)
{
	struct net_local *lp = netdev_priv(dev);
	struct sk_buff *skb;
	u32 len;

	len = ETH_FRAME_LEN + ETH_FCS_LEN;
	skb = netdev_alloc_skb(dev, len + NET_IP_ALIGN);
	if (!skb) {
		/* Couldn't get memory. */
		dev->stats.rx_dropped++;
		dev_err(&lp->ndev->dev, "Could not allocate receive buffer\n");
		return;
	}

	skb_reserve(skb, NET_IP_ALIGN);

	len = al_ethlite_recv_data(lp, (u8 *)skb->data, len);

	if (!len) {
		dev->stats.rx_errors++;
		dev_kfree_skb_irq(skb);
		return;
	}

	skb_put(skb, len);	/* Tell the skb how much data we got */

	skb->protocol = eth_type_trans(skb, dev);
	skb_checksum_none_assert(skb);

	dev->stats.rx_packets++;
	dev->stats.rx_bytes += len;

	if (!skb_defer_rx_timestamp(skb))
		netif_rx(skb);	/* Send the packet upstream */
}

#define INTR_PEND(base_addr, shift) (al_ethlite_readl(base_addr + ELITE_EIP_OFFSET) \
										& (1 << shift) & \
									al_ethlite_readl(base_addr + ELITE_EIC_OFFSET))
/**
 * al_ethlite_interrupt - Interrupt handler for this driver
 * @irq:	Irq of the Ethlite device
 * @dev_id:	Void pointer to the network device instance used as callback
 *		reference
 *
 * Return:	IRQ_HANDLED
 *
 * This function handles the Tx and Rx interrupts of the EthLite device.
 */
static irqreturn_t al_ethlite_interrupt(int irq, void *dev_id)
{
	bool tx_complete = false;
	struct net_device *dev = dev_id;
	struct net_local *lp = netdev_priv(dev);
	void __iomem *base_addr = lp->base_addr;
	u32 val;

	if(INTR_PEND(base_addr, EIP_TXIDLE_SHIFT)) {
		val = al_ethlite_readl(base_addr + ELITE_EIP_OFFSET);
		tx_complete = true;
	}

	if(INTR_PEND(base_addr, EIP_TXDONE_SHIFT)) {
		val = al_ethlite_readl(base_addr + ELITE_EIP_OFFSET);
		tx_complete = true;
	}

	/* If there was a Tx interrupt, call the Tx Handler */
	if (tx_complete != 0) {
		al_ethlite_tx_handler(dev);
		al_ethlite_writel((1 << EIP_TXDONE_SHIFT) & val & (1 << EIP_TXIDLE_SHIFT), base_addr + ELITE_EIP_OFFSET);
	}

	if(INTR_PEND(base_addr, EIP_RXBADPKT_SHIFT)) {
		netdev_dbg(lp->ndev, "An error packet was received\n");
		val = al_ethlite_readl(base_addr + ELITE_EIP_OFFSET);
		al_ethlite_writel((1 << EIP_RXBADPKT_SHIFT) & val, base_addr + ELITE_EIP_OFFSET);
	}

	if(INTR_PEND(base_addr, EIP_RXVALID_SHIFT)) {
		val = al_ethlite_readl(base_addr + ELITE_EIP_OFFSET);
		al_ethlite_rx_handler(dev);
		al_ethlite_writel((1 << EIP_RXVALID_SHIFT) & val, base_addr + ELITE_EIP_OFFSET);
	}

	return IRQ_HANDLED;
}

/**********************/
/* MDIO Bus functions */
/**********************/

/**
 * al_ethlite_mdio_wait - Wait for the MDIO to be ready to use
 * @lp:		Pointer to the Ethlite device private data
 *
 * This function waits till the device is ready to accept a new MDIO
 * request.
 *
 * Return:	0 for success or ETIMEDOUT for a timeout
 */

static int al_ethlite_mdio_wait(struct net_local *lp)
{
	u32 val;

	return readx_poll_timeout(al_ethlite_readl,
				(lp->base_addr + ELITE_MDIOCTL_OFFSET),
				val, !(val & 0x1),
				1000, 20000);
}

/**
 * al_ethlite_mdio_read - Read from a given MII management register
 * @bus:	the mii_bus struct
 * @phy_id:	the phy address
 * @reg:	register number to read from
 *
 * This function waits till the device is ready to accept a new MDIO
 * request and then writes the phy address to the MDIO Address register
 * and reads data from MDIO Read Data register, when its available.
 *
 * Return:	Value read from the MII management register
 */
static int al_ethlite_mdio_read(struct mii_bus *bus, int phy_id, int reg)
{
	struct net_local *lp = bus->priv;
	u32 rc;
	u32 temp_reg;

	if (al_ethlite_mdio_wait(lp))
		return -ETIMEDOUT;

	temp_reg = al_ethlite_readl(lp->base_addr + ELITE_MDIOCCF_OFFSET);
	al_ethlite_writel((MDIO_ENABLE << MDIOCCF_MDIOEN_SHIFT) | temp_reg,
			 lp->base_addr + ELITE_MDIOCCF_OFFSET);

	/* Write the PHY address, register number and set the OP bit in the
	 * MDIO Address register. Set the Status bit in the MDIO Control
	 * register to start a MDIO read transaction.
	 */
	rc = (phy_id << MDIOCTL_PHYAD_SHIFT) | (reg << MDIOCTL_REGAD_SHIFT) |
		 (MDIO_RD_SEL << MDIOCTL_RWSEL_SHIFT) | (MDIO_START_TRANS << MDIOCTL_ACTTRIG_SHIFT);

	al_ethlite_writel(rc, lp->base_addr + ELITE_MDIOCTL_OFFSET);

	if (al_ethlite_mdio_wait(lp))
		return -ETIMEDOUT;

	rc = al_ethlite_readl(lp->base_addr + ELITE_MDIORD_OFFSET);

	temp_reg = al_ethlite_readl(lp->base_addr + ELITE_MDIOCCF_OFFSET);
	al_ethlite_writel(0x7fffffff & temp_reg,
			 lp->base_addr + ELITE_MDIOCCF_OFFSET);

	netdev_dbg(lp->ndev,
		"%s(phy_id=%i, reg=%x) == %x\n", __func__,
		phy_id, reg, rc);

	return rc;
}

/**
 * al_ethlite_mdio_write - Write to a given MII management register
 * @bus:	the mii_bus struct
 * @phy_id:	the phy address
 * @reg:	register number to write to
 * @val:	value to write to the register number specified by reg
 *
 * This function waits till the device is ready to accept a new MDIO
 * request and then writes the val to the MDIO Write Data register.
 *
 * Return:	  0 upon success or a negative error upon failure
 */
static int al_ethlite_mdio_write(struct mii_bus *bus, int phy_id, int reg,
				u16 val)
{
	struct net_local *lp = bus->priv;
	u32 rc;
	u32 temp_reg;

	netdev_dbg(lp->ndev,
		"%s(phy_id=%i, reg=%x, val=%x)\n", __func__,
		phy_id, reg, val);

	if (al_ethlite_mdio_wait(lp))
		return -ETIMEDOUT;

	temp_reg = al_ethlite_readl(lp->base_addr + ELITE_MDIOCCF_OFFSET);
	al_ethlite_writel((MDIO_ENABLE << MDIOCCF_MDIOEN_SHIFT) | temp_reg,
			 lp->base_addr + ELITE_MDIOCCF_OFFSET);

	/* Write the value into the MDIO Write Data register. Finally,write the PHY address,
	 * register number and then set the Status bit in the MDIO Control register to start a MDIO write transaction.
	 */
	al_ethlite_writel(val, lp->base_addr + ELITE_MDIOWR_OFFSET);

	rc = (phy_id << MDIOCTL_PHYAD_SHIFT) | (reg << MDIOCTL_REGAD_SHIFT) |
		(MDIO_WR_SEL << MDIOCTL_RWSEL_SHIFT) | (MDIO_START_TRANS << MDIOCTL_ACTTRIG_SHIFT);

	al_ethlite_writel(rc, lp->base_addr + ELITE_MDIOCTL_OFFSET);

	if (al_ethlite_mdio_wait(lp))
		return -ETIMEDOUT;

	temp_reg = al_ethlite_readl(lp->base_addr + ELITE_MDIOCCF_OFFSET);
	al_ethlite_writel(0x7fffffff & temp_reg,
			 lp->base_addr + ELITE_MDIOCCF_OFFSET);

	return 0;
}

/**
 * al_ethlite_mdio_setup - Register mii_bus for the Ethlite device
 * @lp:		Pointer to the Ethlite device private data
 * @dev:	Pointer to OF device structure
 *
 * This function enables MDIO bus in the Ethlite device and registers a
 * mii_bus.
 *
 * Return:	0 upon success or a negative error upon failure
 */
static int al_ethlite_mdio_setup(struct net_local *lp, struct device *dev)
{
	struct mii_bus *bus;
	struct resource res;
	struct device_node *np = of_get_parent(lp->phy_node);
	struct device_node *npp;
	int rc, ret;

	/* Don't register the MDIO bus if the phy_node or its parent node
	 * can't be found.
	 */
	if (!np) {
		dev_err(dev, "Failed to register mdio bus.\n");
		return -ENODEV;
	}

	npp = of_get_parent(np);
	ret = of_address_to_resource(npp, 0, &res);
	of_node_put(npp);
	if (ret) {
		dev_err(dev, "%s resource error!\n",
			dev->of_node->full_name);
		of_node_put(np);
		return ret;
	}
	if (lp->ndev->mem_start != res.start) {
		struct phy_device *phydev;

		phydev = of_phy_find_device(lp->phy_node);
		if (!phydev)
			dev_info(dev,
				 "MDIO of the phy is not registered yet\n");
		else
			put_device(&phydev->mdio.dev);
		of_node_put(np);
		return 0;
	}

	bus = mdiobus_alloc();
	if (!bus) {
		dev_err(dev, "Failed to allocate mdiobus\n");
		of_node_put(np);
		return -ENOMEM;
	}

	snprintf(bus->id, MII_BUS_ID_SIZE, "%.8llx",
		 (unsigned long long)res.start);
	bus->priv = lp;
	bus->name = "Anlogic EthLite MDIO";
	bus->read = al_ethlite_mdio_read;
	bus->write = al_ethlite_mdio_write;
	bus->parent = dev;

	rc = of_mdiobus_register(bus, np);
	of_node_put(np);
	if (rc) {
		dev_err(dev, "Failed to register mdio bus.\n");
		goto err_register;
	}

	lp->mii_bus = bus;

	return 0;

err_register:
	mdiobus_free(bus);
	return rc;
}

/**
 * al_ethlite_adjust_link - Link state callback for the Ethlite device
 * @ndev: pointer to net_device struct
 *
 * There's nothing in the Ethlite device to be configured when the link
 * state changes. We just print the status.
 */
static void al_ethlite_adjust_link(struct net_device *ndev)
{
	struct net_local *lp = netdev_priv(ndev);
	struct phy_device *phy = lp->phy_dev;
	int link_state;

	/* hash together the state values to decide if something has changed */
	link_state = phy->speed | (phy->duplex << 1) | phy->link;

	if (lp->last_link != link_state) {
		lp->last_link = link_state;
		phy_print_status(phy);
	}
}

/**
 * al_ethlite_open - Open the network device
 * @dev:	Pointer to the network device
 *
 * This function sets the MAC address, requests an IRQ and enables interrupts
 * for the Ethlite device and starts the Tx queue.
 * It also connects to the phy device, if MDIO is included in Ethlite device.
 *
 * Return:	0 on success. -ENODEV, if PHY cannot be connected.
 *		Non-zero error value on failure.
 */
static int al_ethlite_open(struct net_device *dev)
{
	struct net_local *lp = netdev_priv(dev);
	int retval;

	al_ethlite_disable_interrupts(lp);

	if (lp->phy_node) {
		lp->phy_dev = of_phy_connect(lp->ndev, lp->phy_node,
						 al_ethlite_adjust_link, 0,
						 PHY_INTERFACE_MODE_MII);
		if (!lp->phy_dev) {
			dev_err(&lp->ndev->dev, "of_phy_connect() failed\n");
			return -ENODEV;
		}

		/* EthLite doesn't support giga-bit speeds */
		phy_set_max_speed(lp->phy_dev, SPEED_100);
		phy_start(lp->phy_dev);
	}

	/* Set the MAC address each time opened */
	al_ethlite_update_address(lp, dev->dev_addr);

	/* Grab the IRQ */
	retval = request_irq(dev->irq, al_ethlite_interrupt, 0, dev->name, dev);
	if (retval) {
		dev_err(&lp->ndev->dev, "Could not allocate interrupt %d\n",
			dev->irq);
		if (lp->phy_dev)
			phy_disconnect(lp->phy_dev);
		lp->phy_dev = NULL;

		return retval;
	}

	/* Enable Interrupts */
	al_ethlite_enable_interrupts(lp);

	/* We're ready to go */
	netif_start_queue(dev);

	return 0;
}

/**
 * al_ethlite_close - Close the network device
 * @dev:	Pointer to the network device
 *
 * This function stops the Tx queue, disables interrupts and frees the IRQ for
 * the Ethlite device.
 * It also disconnects the phy device associated with the Ethlite device.
 *
 * Return:	0, always.
 */
static int al_ethlite_close(struct net_device *dev)
{
	struct net_local *lp = netdev_priv(dev);

	netif_stop_queue(dev);
	al_ethlite_disable_interrupts(lp);
	free_irq(dev->irq, dev);

	if (lp->phy_dev)
		phy_disconnect(lp->phy_dev);
	lp->phy_dev = NULL;

	return 0;
}

/**
 * al_ethlite_send - Transmit a frame
 * @orig_skb:	Pointer to the socket buffer to be transmitted
 * @dev:	Pointer to the network device
 *
 * This function checks if the Tx buffer of the Ethlite device is free to send
 * data. If so, it fills the Tx buffer with data from socket buffer data,
 * updates the stats and frees the socket buffer. The Tx completion is signaled
 * by an interrupt. If the Tx buffer isn't free, then the socket buffer is
 * deferred and the Tx queue is stopped so that the deferred socket buffer can
 * be transmitted when the Ethlite device is free to transmit data.
 *
 * Return:	NETDEV_TX_OK, always.
 */
static netdev_tx_t
al_ethlite_send(struct sk_buff *orig_skb, struct net_device *dev)
{

	struct net_local *lp = netdev_priv(dev);
	struct sk_buff *new_skb;
	unsigned int len;
	unsigned long flags;

	len = orig_skb->len;
	new_skb = orig_skb;

	spin_lock_irqsave(&lp->reset_lock, flags);
	if (al_ethlite_send_data(lp, (u8 *)new_skb->data, len) != 0) {
		/* If the Emaclite Tx buffer is busy, stop the Tx queue and
		 * defer the skb for transmission during the ISR, after the
		 * current transmission is complete
		 */
		netif_stop_queue(dev);
		lp->deferred_skb = new_skb;
		/* Take the time stamp now, since we can't do this in an ISR. */
		skb_tx_timestamp(new_skb);
		spin_unlock_irqrestore(&lp->reset_lock, flags);

		al_ethlite_writel((1 << EIP_RXBADPKT_SHIFT) | (1 << EIP_TXIDLE_SHIFT)
					| (1 << EIP_RXVALID_SHIFT), lp->base_addr + ELITE_EIC_OFFSET);
		return NETDEV_TX_OK;
	}
	spin_unlock_irqrestore(&lp->reset_lock, flags);

	skb_tx_timestamp(new_skb);

	dev->stats.tx_bytes += len;
	dev_consume_skb_any(new_skb);

	return NETDEV_TX_OK;
}

/**
 * al_ethlite_ethtools_get_drvinfo - Get various Axi Emac Lite driver info
 * @ndev:	   Pointer to net_device structure
 * @ed:		 Pointer to ethtool_drvinfo structure
 *
 * This implements ethtool command for getting the driver information.
 * Issue "ethtool -i ethX" under linux prompt to execute this function.
 */
static void al_ethlite_ethtools_get_drvinfo(struct net_device *ndev,
					   struct ethtool_drvinfo *ed)
{
	strscpy(ed->driver, DRIVER_NAME, sizeof(ed->driver));
}

static const struct ethtool_ops al_ethlite_ethtool_ops = {
	.get_drvinfo	= al_ethlite_ethtools_get_drvinfo,
	.get_link	   = ethtool_op_get_link,
	.get_link_ksettings = phy_ethtool_get_link_ksettings,
	.set_link_ksettings = phy_ethtool_set_link_ksettings,
};

static const struct net_device_ops al_ethlite_netdev_ops;

/**
 * al_ethlite_of_probe - Probe method for the Ethlite device.
 * @ofdev:	Pointer to OF device structure
 *
 * This function probes for the Ethlite device in the device tree.
 * It initializes the driver data structure and the hardware, sets the MAC
 * address and registers the network device.
 * It also registers a mii_bus for the Ethlite device, if MDIO is included
 * in the device.
 *
 * Return:	0, if the driver is bound to the Ethlite device, or
 *		a negative error if there is failure.
 */
static int al_ethlite_of_probe(struct platform_device *ofdev)
{
	struct resource *res;
	struct net_device *ndev = NULL;
	struct net_local *lp = NULL;
	struct device *dev = &ofdev->dev;
	u32 temp_reg;
	int rc = 0;

	/* Create an ethernet device instance */
	ndev = alloc_etherdev(sizeof(struct net_local));
	if (!ndev)
		return -ENOMEM;

	dev_set_drvdata(dev, ndev);
	SET_NETDEV_DEV(ndev, &ofdev->dev);
	lp = netdev_priv(ndev);
	lp->ndev = ndev;

	/* Get IRQ for the device */
	rc = platform_get_irq(ofdev, 0);
	if (rc < 0)
		goto error;

	ndev->irq = rc;

	res = platform_get_resource(ofdev, IORESOURCE_MEM, 0);
	lp->base_addr = devm_ioremap_resource(&ofdev->dev, res);
	if (IS_ERR(lp->base_addr)) {
		rc = PTR_ERR(lp->base_addr);
		goto error;
	}

	ndev->mem_start = res->start;
	ndev->mem_end = res->end;

	spin_lock_init(&lp->reset_lock);

	rc = of_get_ethdev_address(ofdev->dev.of_node, ndev);
	if (rc) {
		dev_warn(dev, "No MAC address found, using random\n");
		eth_hw_addr_random(ndev);
	}

	/* Release Mac TX RX and mdio */
	al_ethlite_writel(0, lp->base_addr + ELITE_ERST_OFFSET);

	/* Set Mac Configure Register */
	temp_reg = (0 << ESET0_TXSEMIEN_SHIFT)	  | (0 << ESET0_TXVLANEN_SHIFT) |
			   (1 << ESET0_TXFCSGENEN_SHIFT)	| (0 << ESET0_TXJUMBOEN_SHIFT) |
			   (0 << ESET0_TXGAPADJEN_SHIFT)	| (0 << ESET0_TXFLOWCTRLEN_SHIFT) |
			   (1 << ESET0_TXEN_SHIFT)		  | (0 << ESET0_RXSEMIEN_SHIFT) |
			   (0 << ESET0_RXVLANEN_SHIFT)	  | (1 << ESET0_RXFCSCHKEN_SHIFT) |
			   (0 << ESET0_RXJUMBOEN_SHIFT)	 | (1 << ESET0_RXCFCHKEN_SHIFT) |
			   (1 << ESET0_RXLTCHKEN_SHIFT)	 | (0 << ESET0_RXFLOWCTRLEN_SHIFT) |
			   (1 << ESET0_RXEN_SHIFT)		  | (0 << ESET0_TXIFG_SHIFT) |
			   (CONFIG_100MB_SPEED << ESET0_SPEEDCFG_SHIFT);

	al_ethlite_writel(temp_reg, lp->base_addr + ELITE_ESET0_OFFSET);

	/* Set the MAC address in the EthLite device */
	al_ethlite_update_address(lp, ndev->dev_addr);

	lp->phy_node = of_parse_phandle(ofdev->dev.of_node, "phy-handle", 0);
	al_ethlite_mdio_setup(lp, &ofdev->dev);

	dev_info(dev, "MAC address is now %pM\n", ndev->dev_addr);

	ndev->netdev_ops = &al_ethlite_netdev_ops;
	ndev->ethtool_ops = &al_ethlite_ethtool_ops;
	ndev->flags &= ~IFF_MULTICAST;
	ndev->watchdog_timeo = TX_TIMEOUT;

	/* Finally, register the device */
	rc = register_netdev(ndev);
	if (rc) {
		dev_err(dev, "Cannot register network device, aborting\n");
		goto put_node;
	}

	dev_info(dev, "Anlogic EthLite at 0x%08lX mapped to 0x%p, irq=%d\n",
		 (unsigned long __force)ndev->mem_start, lp->base_addr, ndev->irq);
	return 0;

put_node:
	of_node_put(lp->phy_node);
error:
	free_netdev(ndev);
	return rc;
}

/**
 * al_ethlite_of_remove - Unbind the driver from the Ethlite device.
 * @of_dev:	Pointer to OF device structure
 *
 * This function is called if a device is physically removed from the system or
 * if the driver module is being unloaded. It frees any resources allocated to
 * the device.
 *
 * Return:	0, always.
 */
static int al_ethlite_of_remove(struct platform_device *of_dev)
{
	struct net_device *ndev = platform_get_drvdata(of_dev);

	struct net_local *lp = netdev_priv(ndev);

	/* Un-register the mii_bus, if configured */
	if (lp->mii_bus) {
		mdiobus_unregister(lp->mii_bus);
		mdiobus_free(lp->mii_bus);
		lp->mii_bus = NULL;
	}

	unregister_netdev(ndev);

	of_node_put(lp->phy_node);
	lp->phy_node = NULL;

	free_netdev(ndev);

	return 0;
}

#ifdef CONFIG_NET_POLL_CONTROLLER
static void
al_ethlite_poll_controller(struct net_device *ndev)
{
	disable_irq(ndev->irq);
	al_ethlite_interrupt(ndev->irq, ndev);
	enable_irq(ndev->irq);
}
#endif

/* Ioctl MII Interface */
static int al_ethlite_ioctl(struct net_device *dev, struct ifreq *rq, int cmd)
{
	if (!dev->phydev || !netif_running(dev))
		return -EINVAL;

	switch (cmd) {
	case SIOCGMIIPHY:
	case SIOCGMIIREG:
	case SIOCSMIIREG:
		return phy_mii_ioctl(dev->phydev, rq, cmd);
	default:
		return -EOPNOTSUPP;
	}
}

static const struct net_device_ops al_ethlite_netdev_ops = {
	.ndo_open		= al_ethlite_open,
	.ndo_stop		= al_ethlite_close,
	.ndo_start_xmit		= al_ethlite_send,
	.ndo_set_mac_address	= al_ethlite_set_mac_address,
	.ndo_tx_timeout		= al_ethlite_tx_timeout,
	.ndo_eth_ioctl		= al_ethlite_ioctl,
#ifdef CONFIG_NET_POLL_CONTROLLER
	.ndo_poll_controller = al_ethlite_poll_controller,
#endif
};

/* Match table for OF platform binding */
static const struct of_device_id al_ethlite_of_match[] = {
	{ .compatible = "anlogic,ethernetlite1.0", },
	{ /* end of list */ },
};
MODULE_DEVICE_TABLE(of, al_ethlite_of_match);

static struct platform_driver al_ethlite_of_driver = {
	.driver = {
		.name = DRIVER_NAME,
		.of_match_table = al_ethlite_of_match,
	},
	.probe		= al_ethlite_of_probe,
	.remove		= al_ethlite_of_remove,
};

module_platform_driver(al_ethlite_of_driver);

MODULE_AUTHOR("Anlogic, Inc.");
MODULE_DESCRIPTION("Anlogic Ethernet MAC Lite driver");
MODULE_LICENSE("GPL");
