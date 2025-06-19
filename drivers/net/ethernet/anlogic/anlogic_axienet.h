/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (C) 2025 Anlogic, Inc.
 *
 */

#ifndef ANLOGIC_AXIENET_H
#define ANLOGIC_AXIENET_H

#include <linux/netdevice.h>
#include <linux/spinlock.h>
#include <linux/interrupt.h>
#include <linux/if_vlan.h>
#include <linux/phylink.h>

/* Packet size info */
#define AE_HDR_SIZE			14 /* Size of Ethernet header */
#define AE_TRL_SIZE			 4 /* Size of Ethernet trailer (FCS) */
#define AE_MTU			      1500 /* Max MTU of an Ethernet frame */
#define AE_JUMBO_MTU		      9000 /* Max MTU of a jumbo Eth. frame */

#define AE_MAX_FRAME_SIZE	 (AE_MTU + AE_HDR_SIZE + AE_TRL_SIZE)
#define AE_MAX_VLAN_FRAME_SIZE  (AE_MTU + VLAN_ETH_HLEN + AE_TRL_SIZE)
#define AE_MAX_JUMBO_FRAME_SIZE (AE_JUMBO_MTU + AE_HDR_SIZE + AE_TRL_SIZE)

/* Configuration options */

/* Accept all incoming packets. Default: disabled (cleared) */
#define AE_OPTION_PROMISC			BIT(0)

/* Jumbo frame support for Tx & Rx. Default: disabled (cleared) */
#define AE_OPTION_JUMBO				BIT(1)

/* VLAN Rx & Tx frame support. Default: disabled (cleared) */
#define AE_OPTION_VLAN				BIT(2)

/* Enable recognition of flow control frames on Rx. Default: enabled (set) */
#define AE_OPTION_FLOW_CONTROL			BIT(4)

/* Strip FCS and PAD from incoming frames. Note: PAD from VLAN frames is not
 * stripped. Default: disabled (set)
 */
#define AE_OPTION_FCS_STRIP			BIT(5)

/* Generate FCS field and add PAD automatically for outgoing frames.
 * Default: enabled (set)
 */
#define AE_OPTION_FCS_INSERT			BIT(6)

/* Enable Length/Type error checking for incoming frames. When this option is
 * set, the MAC will filter frames that have a mismatched type/length field
 * and if AE_OPTION_REPORT_RXERR is set, the user is notified when these
 * types of frames are encountered. When this option is cleared, the MAC will
 * allow these types of frames to be received. Default: enabled (set)
 */
#define AE_OPTION_LENTYPE_ERR			BIT(7)

/* Enable the transmitter. Default: enabled (set) */
#define AE_OPTION_TXEN				BIT(11)

/*  Enable the receiver. Default: enabled (set) */
#define AE_OPTION_RXEN				BIT(12)

/*  Default options set when device is initialized or reset */
#define AE_OPTION_DEFAULTS				   \
				(AE_OPTION_TXEN |	   \
				 AE_OPTION_FLOW_CONTROL | \
				 AE_OPTION_RXEN)

/* Axi DMA Register definitions */

#define AXIDMA_TX_CR_OFFSET	0x00000000 /* Channel control */
#define AXIDMA_TX_SR_OFFSET	0x00000004 /* Status */
#define AXIDMA_TX_CDESC_OFFSET	0x00000008 /* Current descriptor pointer */
#define AXIDMA_TX_TDESC_OFFSET	0x00000010 /* Tail descriptor pointer */

#define AXIDMA_RX_CR_OFFSET	0x00000030 /* Channel control */
#define AXIDMA_RX_SR_OFFSET	0x00000034 /* Status */
#define AXIDMA_RX_CDESC_OFFSET	0x00000038 /* Current descriptor pointer */
#define AXIDMA_RX_TDESC_OFFSET	0x00000040 /* Tail descriptor pointer */

#define AXIDMA_CR_RUNSTOP_MASK	0x00000001 /* Start/stop DMA channel */
#define AXIDMA_CR_RESET_MASK	0x00000004 /* Reset DMA engine */

#define AXIDMA_SR_HALT_MASK	0x00000001 /* Indicates DMA channel halted */

#define AXIDMA_BD_NDESC_OFFSET		0x00 /* Next descriptor pointer */
#define AXIDMA_BD_BUFA_OFFSET		0x08 /* Buffer address */
#define AXIDMA_BD_CTRL_LEN_OFFSET	0x18 /* Control/buffer length */
#define AXIDMA_BD_STS_OFFSET		0x1C /* Status */
#define AXIDMA_BD_USR0_OFFSET		0x20 /* User IP specific word0 */
#define AXIDMA_BD_USR1_OFFSET		0x24 /* User IP specific word1 */
#define AXIDMA_BD_USR2_OFFSET		0x28 /* User IP specific word2 */
#define AXIDMA_BD_USR3_OFFSET		0x2C /* User IP specific word3 */
#define AXIDMA_BD_USR4_OFFSET		0x30 /* User IP specific word4 */
#define AXIDMA_BD_ID_OFFSET		0x34 /* Sw ID */
#define AXIDMA_BD_HAS_STSCNTRL_OFFSET	0x38 /* Whether has stscntrl strm */
#define AXIDMA_BD_HAS_DRE_OFFSET	0x3C /* Whether has DRE */

#define AXIDMA_BD_HAS_DRE_SHIFT	8 /* Whether has DRE shift */
#define AXIDMA_BD_HAS_DRE_MASK		0xF00 /* Whether has DRE mask */
#define AXIDMA_BD_WORDLEN_MASK		0xFF /* Whether has DRE mask */

#define AXIDMA_BD_CTRL_LENGTH_MASK	0x007FFFFF /* Requested len */
#define AXIDMA_BD_CTRL_TXSOF_MASK	0x08000000 /* First tx packet */
#define AXIDMA_BD_CTRL_TXEOF_MASK	0x04000000 /* Last tx packet */
#define AXIDMA_BD_CTRL_ALL_MASK	0x0C000000 /* All control bits */

#define AXIDMA_DELAY_MASK		0xFF000000 /* Delay timeout counter */
#define AXIDMA_COALESCE_MASK		0x00FF0000 /* Coalesce counter */

#define AXIDMA_DELAY_SHIFT		24
#define AXIDMA_COALESCE_SHIFT		16

#define AXIDMA_IRQ_IOC_MASK		0x00001000 /* Completion intr */
#define AXIDMA_IRQ_DELAY_MASK		0x00002000 /* Delay interrupt */
#define AXIDMA_IRQ_ERROR_MASK		0x00004000 /* Error interrupt */
#define AXIDMA_IRQ_ALL_MASK		0x00007000 /* All interrupts */

/* Default TX/RX Threshold and waitbound values for SGDMA mode */
#define AXIDMA_DFT_TX_THRESHOLD	24
#define AXIDMA_DFT_TX_WAITBOUND	254
#define AXIDMA_DFT_RX_THRESHOLD	24
#define AXIDMA_DFT_RX_WAITBOUND	254

#define AXIDMA_BD_CTRL_TXSOF_MASK	0x08000000 /* First tx packet */
#define AXIDMA_BD_CTRL_TXEOF_MASK	0x04000000 /* Last tx packet */
#define AXIDMA_BD_CTRL_ALL_MASK	0x0C000000 /* All control bits */

#define AXIDMA_BD_STS_ACTUAL_LEN_MASK	0x007FFFFF /* Actual len */
#define AXIDMA_BD_STS_COMPLETE_MASK	0x80000000 /* Completed */
#define AXIDMA_BD_STS_DEC_ERR_MASK	0x40000000 /* Decode error */
#define AXIDMA_BD_STS_SLV_ERR_MASK	0x20000000 /* Slave error */
#define AXIDMA_BD_STS_INT_ERR_MASK	0x10000000 /* Internal err */
#define AXIDMA_BD_STS_ALL_ERR_MASK	0x70000000 /* All errors */
#define AXIDMA_BD_STS_RXSOF_MASK	0x08000000 /* First rx pkt */
#define AXIDMA_BD_STS_RXEOF_MASK	0x04000000 /* Last rx pkt */
#define AXIDMA_BD_STS_ALL_MASK		0xFC000000 /* All status bits */

#define AXIDMA_BD_MINIMUM_ALIGNMENT	0x40

/* Axi Ethernet registers definition */
#define AE_RAF_OFFSET		0x00000000 /* Reset and Address filter */
#define AE_TPF_OFFSET		0x00000004 /* Tx Pause Frame */
#define AE_IFGP_OFFSET		0x00000008 /* Tx Inter-frame gap adjustment*/
#define AE_IS_OFFSET		0x0000000C /* Interrupt status */
#define AE_IP_OFFSET		0x00000010 /* Interrupt pending */
#define AE_IE_OFFSET		0x00000014 /* Interrupt enable */
#define AE_TTAG_OFFSET		0x00000018 /* Tx VLAN TAG */
#define AE_RTAG_OFFSET		0x0000001C /* Rx VLAN TAG */
#define AE_UAWL_OFFSET		0x00000020 /* Unicast address word lower */
#define AE_UAWU_OFFSET		0x00000024 /* Unicast address word upper */
#define AE_TPID0_OFFSET	0x00000028 /* VLAN TPID0 register */
#define AE_TPID1_OFFSET	0x0000002C /* VLAN TPID1 register */
#define AE_PPST_OFFSET		0x00000030 /* PCS PMA Soft Temac Status Reg */
#define AE_RCW0_OFFSET		0x00000400 /* Rx Configuration Word 0 */
#define AE_RCW1_OFFSET		0x00000404 /* Rx Configuration Word 1 */
#define AE_TC_OFFSET		0x00000408 /* Tx Configuration */
#define AE_FCC_OFFSET		0x0000040C /* Flow Control Configuration */
#define AE_EMMC_OFFSET		0x00000410 /* MAC speed configuration */
#define AE_PHYC_OFFSET		0x00000414 /* RX Max Frame Configuration */
#define AE_ID_OFFSET		0x000004F8 /* Identification register */
#define AE_MDIO_MC_OFFSET	0x00000500 /* MDIO Setup */
#define AE_MDIO_MCR_OFFSET	0x00000504 /* MDIO Control */
#define AE_MDIO_MWD_OFFSET	0x00000508 /* MDIO Write Data */
#define AE_MDIO_MRD_OFFSET	0x0000050C /* MDIO Read Data */
#define AE_UAW0_OFFSET		0x00000700 /* Unicast address word 0 */
#define AE_UAW1_OFFSET		0x00000704 /* Unicast address word 1 */
#define AE_FMI_OFFSET		0x00000708 /* Frame Filter Control */
#define AE_FFE_OFFSET		0x0000070C /* Frame Filter Enable */
#define AE_AF0_OFFSET		0x00000710 /* Address Filter 0 */
#define AE_AF1_OFFSET		0x00000714 /* Address Filter 1 */

#define AE_TX_VLAN_DATA_OFFSET 0x00004000 /* TX VLAN data table address */
#define AE_RX_VLAN_DATA_OFFSET 0x00008000 /* RX VLAN data table address */
#define AE_MCAST_TABLE_OFFSET	0x00020000 /* Multicast table address */

/* Bit Masks for Axi Ethernet RAF register */
/* Reject receive multicast destination address */
#define AE_RAF_MCSTREJ_MASK		0x00000002
/* Reject receive broadcast destination address */
#define AE_RAF_BCSTREJ_MASK		0x00000004
#define AE_RAF_TXVTAGMODE_MASK		0x00000018 /* Tx VLAN TAG mode */
#define AE_RAF_RXVTAGMODE_MASK		0x00000060 /* Rx VLAN TAG mode */
#define AE_RAF_TXVSTRPMODE_MASK	0x00000180 /* Tx VLAN STRIP mode */
#define AE_RAF_RXVSTRPMODE_MASK	0x00000600 /* Rx VLAN STRIP mode */
#define AE_RAF_NEWFNCENBL_MASK		0x00000800 /* New function mode */
/* Extended Multicast Filtering mode */
#define AE_RAF_EMULTIFLTRENBL_MASK	0x00001000
#define AE_RAF_STATSRST_MASK		0x00002000 /* Stats. Counter Reset */
#define AE_RAF_RXBADFRMEN_MASK		0x00004000 /* Recv Bad Frame Enable */
#define AE_RAF_TXVTAGMODE_SHIFT	3 /* Tx Tag mode shift bits */
#define AE_RAF_RXVTAGMODE_SHIFT	5 /* Rx Tag mode shift bits */
#define AE_RAF_TXVSTRPMODE_SHIFT	7 /* Tx strip mode shift bits*/
#define AE_RAF_RXVSTRPMODE_SHIFT	9 /* Rx Strip mode shift bits*/

/* Bit Masks for Axi Ethernet TPF and IFGP registers */
#define AE_TPF_TPFV_MASK		0x0000FFFF /* Tx pause frame value */
/* Transmit inter-frame gap adjustment value */
#define AE_IFGP0_IFGP_MASK		0x0000007F

/* Bit Masks for Axi Ethernet IS, IE and IP registers, Same masks apply
 * for all 3 registers.
 */
/* Hard register access complete */
#define AE_INT_HARDACSCMPLT_MASK	0x00000001
/* Auto negotiation complete */
#define AE_INT_AUTONEG_MASK		0x00000002
#define AE_INT_RXCMPIT_MASK		0x00000004 /* Rx complete */
#define AE_INT_RXRJECT_MASK		0x00000008 /* Rx frame rejected */
#define AE_INT_RXFIFOOVR_MASK		0x00000010 /* Rx fifo overrun */
#define AE_INT_TXCMPIT_MASK		0x00000020 /* Tx complete */
#define AE_INT_RXDCMLOCK_MASK		0x00000040 /* Rx Dcm Lock */
#define AE_INT_MGTRDY_MASK		0x00000080 /* MGT clock Lock */
#define AE_INT_PHYRSTCMPLT_MASK	0x00000100 /* Phy Reset complete */
#define AE_INT_ALL_MASK		0x0000003F /* All the ints */

/* INT bits that indicate receive errors */
#define AE_INT_RECV_ERROR_MASK				\
	(AE_INT_RXRJECT_MASK | AE_INT_RXFIFOOVR_MASK)

/* Bit masks for Axi Ethernet VLAN TPID Word 0 register */
#define AE_TPID_0_MASK		0x0000FFFF /* TPID 0 */
#define AE_TPID_1_MASK		0xFFFF0000 /* TPID 1 */

/* Bit masks for Axi Ethernet VLAN TPID Word 1 register */
#define AE_TPID_2_MASK		0x0000FFFF /* TPID 0 */
#define AE_TPID_3_MASK		0xFFFF0000 /* TPID 1 */

/* Bit masks for Axi Ethernet RCW1 register */
#define AE_RCW1_RST_MASK	0x80000000 /* Reset */
#define AE_RCW1_JUM_MASK	0x40000000 /* Jumbo frame enable */
/* In-Band FCS enable (FCS not stripped) */
#define AE_RCW1_FCS_MASK	0x20000000
#define AE_RCW1_RX_MASK	0x10000000 /* Receiver enable */
#define AE_RCW1_VLAN_MASK	0x08000000 /* VLAN frame enable */
/* Length/type field valid check disable */
#define AE_RCW1_LT_DIS_MASK	0x02000000
/* Control frame Length check disable */
#define AE_RCW1_CL_DIS_MASK	0x01000000
/* Pause frame source address bits [47:32]. Bits [31:0] are
 * stored in register RCW0
 */
#define AE_RCW1_PAUSEADDR_MASK 0x0000FFFF

/* Bit masks for Axi Ethernet TC register */
#define AE_TC_RST_MASK		0x80000000 /* Reset */
#define AE_TC_JUM_MASK		0x40000000 /* Jumbo frame enable */
/* In-Band FCS enable (FCS not generated) */
#define AE_TC_FCS_MASK		0x20000000
#define AE_TC_TX_MASK		0x10000000 /* Transmitter enable */
#define AE_TC_VLAN_MASK	0x08000000 /* VLAN frame enable */
/* Inter-frame gap adjustment enable */
#define AE_TC_IFG_MASK		0x02000000

/* Bit masks for Axi Ethernet FCC register */
#define AE_FCC_FCRX_MASK	0x20000000 /* Rx flow control enable */
#define AE_FCC_FCTX_MASK	0x40000000 /* Tx flow control enable */

/* Bit masks for Axi Ethernet EMMC register */
#define AE_EMMC_LINKSPEED_MASK	0xC0000000 /* Link speed */
#define AE_EMMC_RGMII_MASK	0x20000000 /* RGMII mode enable */
#define AE_EMMC_SGMII_MASK	0x10000000 /* SGMII mode enable */
#define AE_EMMC_GPCS_MASK	0x08000000 /* 1000BaseX mode enable */
#define AE_EMMC_HOST_MASK	0x04000000 /* Host interface enable */
#define AE_EMMC_TX16BIT	0x02000000 /* 16 bit Tx client enable */
#define AE_EMMC_RX16BIT	0x01000000 /* 16 bit Rx client enable */
#define AE_EMMC_LINKSPD_10	0x00000000 /* Link Speed mask for 10 Mbit */
#define AE_EMMC_LINKSPD_100	0x40000000 /* Link Speed mask for 100 Mbit */
#define AE_EMMC_LINKSPD_1000	0x80000000 /* Link Speed mask for 1000 Mbit */

/* Bit masks for Axi Ethernet PHYC register */
#define AE_PHYC_SGMIILINKSPEED_MASK	0xC0000000 /* SGMII link speed mask*/
#define AE_PHYC_RGMIILINKSPEED_MASK	0x0000000C /* RGMII link speed */
#define AE_PHYC_RGMIIHD_MASK		0x00000002 /* RGMII Half-duplex */
#define AE_PHYC_RGMIILINK_MASK		0x00000001 /* RGMII link status */
#define AE_PHYC_RGLINKSPD_10		0x00000000 /* RGMII link 10 Mbit */
#define AE_PHYC_RGLINKSPD_100		0x00000004 /* RGMII link 100 Mbit */
#define AE_PHYC_RGLINKSPD_1000		0x00000008 /* RGMII link 1000 Mbit */
#define AE_PHYC_SGLINKSPD_10		0x00000000 /* SGMII link 10 Mbit */
#define AE_PHYC_SGLINKSPD_100		0x40000000 /* SGMII link 100 Mbit */
#define AE_PHYC_SGLINKSPD_1000		0x80000000 /* SGMII link 1000 Mbit */

/* Bit masks for Axi Ethernet MDIO interface MC register */
#define AE_MDIO_MC_MDIOEN_MASK		0x00000040 /* MII management enable */
#define AE_MDIO_MC_CLOCK_DIVIDE_MAX	0x3F	   /* Maximum MDIO divisor */

/* Bit masks for Axi Ethernet MDIO interface MCR register */
#define AE_MDIO_MCR_PHYAD_MASK		0x1F000000 /* Phy Address Mask */
#define AE_MDIO_MCR_PHYAD_SHIFT	24	   /* Phy Address Shift */
#define AE_MDIO_MCR_REGAD_MASK		0x001F0000 /* Reg Address Mask */
#define AE_MDIO_MCR_REGAD_SHIFT	16	   /* Reg Address Shift */
#define AE_MDIO_MCR_OP_MASK		0x0000C000 /* Operation Code Mask */
#define AE_MDIO_MCR_OP_SHIFT		13	   /* Operation Code Shift */
#define AE_MDIO_MCR_OP_READ_MASK	0x00008000 /* Op Code Read Mask */
#define AE_MDIO_MCR_OP_WRITE_MASK	0x00004000 /* Op Code Write Mask */
#define AE_MDIO_MCR_INITIATE_MASK	0x00000800 /* Ready Mask */
#define AE_MDIO_MCR_READY_MASK		0x00000080 /* Ready Mask */

/* Bit masks for Axi Ethernet MDIO interface MIS, MIP, MIE, MIC registers */
#define AE_MDIO_INT_MIIM_RDY_MASK	0x00000001 /* MIIM Interrupt */

/* Bit masks for Axi Ethernet UAW1 register */
/* Station address bits [47:32]; Station address
 * bits [31:0] are stored in register UAW0
 */
#define AE_UAW1_UNICASTADDR_MASK	0x0000FFFF

/* Bit masks for Axi Ethernet FMC register */
#define AE_FMI_PM_MASK			0x80000000 /* Promis. mode enable */
#define AE_FMI_IND_MASK		0x00000003 /* Index Mask */

#define AE_MDIO_DIV_DFT		29 /* Default MDIO clock divisor */

/* Defines for different options for C_PHY_TYPE parameter in Axi Ethernet IP */
#define AE_PHY_TYPE_MII		0
#define AE_PHY_TYPE_GMII		1
#define AE_PHY_TYPE_RGMII_1_3		2
#define AE_PHY_TYPE_RGMII_2_0		3
#define AE_PHY_TYPE_SGMII		4
#define AE_PHY_TYPE_1000BASE_X		5

 /* Total number of entries in the hardware multicast table. */
#define AE_MULTICAST_CAM_TABLE_NUM	4

/* Axi Ethernet Synthesis features */
#define AE_FEATURE_PARTIAL_RX_CSUM	BIT(0)
#define AE_FEATURE_PARTIAL_TX_CSUM	BIT(1)
#define AE_FEATURE_FULL_RX_CSUM		BIT(2)
#define AE_FEATURE_FULL_TX_CSUM		BIT(3)
#define AE_FEATURE_DMA_64BIT		BIT(4)

#define AE_NO_CSUM_OFFLOAD		0

#define AE_FULL_CSUM_STATUS_MASK	0x00000038
#define AE_IP_UDP_CSUM_VALIDATED	0x00000003
#define AE_IP_TCP_CSUM_VALIDATED	0x00000002

#define DELAY_OF_ONE_MILLISEC		1000

/**
 * struct axidma_bd - Axi Dma buffer descriptor layout
 * @next:         MM2S/S2MM Next Descriptor Pointer
 * @next_msb:     MM2S/S2MM Next Descriptor Pointer (high 32 bits)
 * @phys:         MM2S/S2MM Buffer Address
 * @phys_msb:     MM2S/S2MM Buffer Address (high 32 bits)
 * @reserved3:    Reserved and not used
 * @reserved4:    Reserved and not used
 * @cntrl:        MM2S/S2MM Control value
 * @status:       MM2S/S2MM Status value
 * @app0:         MM2S/S2MM User Application Field 0.
 * @app1:         MM2S/S2MM User Application Field 1.
 * @app2:         MM2S/S2MM User Application Field 2.
 * @app3:         MM2S/S2MM User Application Field 3.
 * @app4:         MM2S/S2MM User Application Field 4.
 */
struct axidma_bd {
	u32 next;	/* Physical address of next buffer descriptor */
	u32 next_msb;	/* high 32 bits for IP >= v7.1, reserved on older IP */
	u32 phys;
	u32 phys_msb;	/* for IP >= v7.1, reserved for older IP */
	u32 reserved3;
	u32 reserved4;
	u32 cntrl;
	u32 status;
	u32 app0;
	u32 app1;	/* TX start << 16 | insert */
	u32 app2;	/* TX csum seed */
	u32 app3;
	u32 app4;   /* Last field used by HW */
	struct sk_buff *skb;
} __aligned(AXIDMA_BD_MINIMUM_ALIGNMENT);

/**
 * struct anlogic_axienet_local - axienet private per device data
 * @ndev:	Pointer for net_device to which it will be attached.
 * @dev:	Pointer to device structure
 * @phy_node:	Pointer to device node structure
 * @mii_bus:	Pointer to MII bus structure
 * @regs_start: Resource start for axienet device addresses
 * @regs:	Base address for the anlogic_axienet_local device address space
 * @dma_regs:	Base address for the axidma device address space
 * @dma_err_tasklet: Tasklet structure to process Axi DMA errors
 * @tx_irq:	Axidma TX IRQ number
 * @rx_irq:	Axidma RX IRQ number
 * @phy_mode:	Phy type to identify between MII/GMII/RGMII/SGMII/1000 Base-X
 * @options:	AxiEthernet option word
 * @last_link:	Phy link state in which the PHY was negotiated earlier
 * @features:	Stores the extended features supported by the axienet hw
 * @tx_bd_v:	Virtual address of the TX buffer descriptor ring
 * @tx_bd_p:	Physical address(start address) of the TX buffer descr. ring
 * @rx_bd_v:	Virtual address of the RX buffer descriptor ring
 * @rx_bd_p:	Physical address(start address) of the RX buffer descr. ring
 * @tx_bd_ci:	Stores the index of the Tx buffer descriptor in the ring being
 *		accessed currently. Used while alloc. BDs before a TX starts
 * @tx_bd_tail:	Stores the index of the Tx buffer descriptor in the ring being
 *		accessed currently. Used while processing BDs after the TX
 *		completed.
 * @rx_bd_ci:	Stores the index of the Rx buffer descriptor in the ring being
 *		accessed currently.
 * @max_frm_size: Stores the maximum size of the frame that can be that
 *		  Txed/Rxed in the existing hardware. If jumbo option is
 *		  supported, the maximum frame size would be 9k. Else it is
 *		  1522 bytes (assuming support for basic VLAN)
 * @rxmem:	Stores rx memory size for jumbo frame handling.
 * @csum_offload_on_tx_path:	Stores the checksum selection on TX side.
 * @csum_offload_on_rx_path:	Stores the checksum selection on RX side.
 * @coalesce_count_rx:	Store the irq coalesce on RX side.
 * @coalesce_count_tx:	Store the irq coalesce on TX side.
 */
struct anlogic_axienet_local {
	struct net_device *ndev;
	struct device *dev;

	/* Connection to PHY device */
	struct device_node *phy_node;

	struct phylink *phylink;
	struct phylink_config phylink_config;

	/* Reference to PCS/PMA PHY if used */
	struct mdio_device *pcs_phy;

	/* Clock for AXI bus */
	struct clk *clk;

	/* MDIO bus data */
	struct mii_bus *mii_bus;	/* MII bus reference */

	/* IO registers, dma functions and IRQs */
	resource_size_t regs_start;
	void __iomem *regs;
	void __iomem *dma_regs;

	struct work_struct dma_err_task;

	int tx_irq;
	int rx_irq;
	int eth_irq;
	phy_interface_t phy_mode;

	u32 options;			/* Current options word */
	u32 features;

	/* Buffer descriptors */
	struct axidma_bd *tx_bd_v;
	dma_addr_t tx_bd_p;
	u32 tx_bd_num;
	struct axidma_bd *rx_bd_v;
	dma_addr_t rx_bd_p;
	u32 rx_bd_num;
	u32 tx_bd_ci;
	u32 tx_bd_tail;
	u32 rx_bd_ci;

	u32 max_frm_size;
	u32 rxmem;

	int csum_offload_on_tx_path;
	int csum_offload_on_rx_path;

	u32 coalesce_count_rx;
	u32 coalesce_count_tx;
};

/**
 * struct axiethernet_option - Used to set axi ethernet hardware options
 * @opt:	Option to be set.
 * @reg:	Register offset to be written for setting the option
 * @m_or:	Mask to be ORed for setting the option in the register
 */
struct anlogic_axienet_option {
	u32 opt;
	u32 reg;
	u32 m_or;
};

/**
 * anlogic_axienet_ior - Memory mapped Axi Ethernet register read
 * @lp:         Pointer to axienet local structure
 * @offset:     Address offset from the base address of Axi Ethernet core
 *
 * Return: The contents of the Axi Ethernet register
 *
 * This function returns the contents of the corresponding register.
 */
static inline u32 anlogic_axienet_ior(struct anlogic_axienet_local *lp, off_t offset)
{
	return ioread32(lp->regs + offset);
}

static inline u32 axinet_ior_read_mcr(struct anlogic_axienet_local *lp)
{
	return anlogic_axienet_ior(lp, AE_MDIO_MCR_OFFSET);
}

/**
 * anlogic_axienet_iow - Memory mapped Axi Ethernet register write
 * @lp:         Pointer to axienet local structure
 * @offset:     Address offset from the base address of Axi Ethernet core
 * @value:      Value to be written into the Axi Ethernet register
 *
 * This function writes the desired value into the corresponding Axi Ethernet
 * register.
 */
static inline void anlogic_axienet_iow(struct anlogic_axienet_local *lp, off_t offset,
				       u32 value)
{
	iowrite32(value, lp->regs + offset);
}

/* Function prototypes visible in anlogic_axienet_mdio.c for other files */
int anlogic_axienet_mdio_enable(struct anlogic_axienet_local *lp);
void anlogic_axienet_mdio_disable(struct anlogic_axienet_local *lp);
int anlogic_axienet_mdio_setup(struct anlogic_axienet_local *lp);
void anlogic_axienet_mdio_teardown(struct anlogic_axienet_local *lp);

#endif /* ANLOGIC_AXI_ENET_H */
