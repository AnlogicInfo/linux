// SPDX-License-Identifier: GPL-2.0+
/*
 * Anlogic Axi CanFD controller driver
 *
 * Copyright (C) 2025 Anlogic, Inc.
 *
 * Author: lkcao <likun.cao@anlogic.com>
 */

#include <linux/bitfield.h>
#include <linux/clk.h>
#include <linux/errno.h>
#include <linux/ethtool.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/netdevice.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/skbuff.h>
#include <linux/spinlock.h>
#include <linux/string.h>
#include <linux/types.h>
#include <linux/can/dev.h>
#include <linux/can/error.h>
#include <linux/pm_runtime.h>

#define AXI_CANFD_SW_RST_OFFSET                           0x00ULL
#define AXI_CANFD_MODE_SEL_OFFSET                         0x04ULL
#define AXI_CANFD_BRP_OFFSET                              0x08ULL
#define AXI_CANFD_APBT_OFFSET                             0x0CULL
#define AXI_CANFD_ERR_CNT_OFFSET                          0x10ULL
#define AXI_CANFD_ERR_STS_OFFSET                          0x14ULL
#define AXI_CANFD_STS_OFFSET                              0x18ULL
#define AXI_CANFD_INT_STS_OFFSET                          0x1CULL
#define AXI_CANFD_INT_EN_OFFSET                           0x20ULL
#define AXI_CANFD_INT_CLR_OFFSET                          0x24ULL
#define AXI_CANFD_TS_OFFSET                               0x28ULL
#define AXI_CANFD_DPBRP_OFFSET                            0x88ULL
#define AXI_CANFD_DPBT_OFFSET                             0x8CULL
#define AXI_CANFD_TBRR_OFFSET                             0x90ULL
#define AXI_CANFD_INT_EN_TBRR_OFFSET                      0x94ULL
#define AXI_CANFD_TBCR_OFFSET                             0x98ULL
#define AXI_CANFD_INT_EN_TBCSC_OFFSET                     0x9CULL
#define AXI_CANFD_TXE_FIFO_STS_OFFSET                     0xA0ULL
#define AXI_CANFD_TXE_FIFO_WM_OFFSET                      0xA4ULL
#define AXI_CANFD_RXBC_STS0_OFFSET                        0xB0ULL
#define AXI_CANFD_INT_EN_RXBF0_OFFSET                     0xC0ULL
#define AXI_CANFD_INT_EN_RXBF1_OFFSET                     0xC4ULL
#define AXI_CANFD_AF_OFFSET                               0xE0ULL
#define AXI_CANFD_RXF_STS_OFFSET                          0xE8ULL
#define AXI_CANFD_RXF_WM_OFFSET                           0xECULL
#define AXI_CANFD_TB_ID_OFFSET(index)                     (0x100ULL + ((index) * 0x48))
#define AXI_CANFD_TB_DLC_OFFSET(index)                    (0x104ULL + ((index) * 0x48))
#define AXI_CANFD_TB_DW0_OFFSET(index)                    (0x108ULL + ((index) * 0x48))
#define AXI_CANFD_RB_ID_OFFSET(index)                     (0x2100ULL + ((index) * 0x48))
#define AXI_CANFD_RB_DLC_OFFSET(index)                    (0x2104ULL + ((index) * 0x48))
#define AXI_CANFD_RB_DW0_OFFSET(index)                    (0x2108ULL + ((index) * 0x48))

#define AXI_CANFD_SW_RST_SRST_SHIFT                       0
#define AXI_CANFD_SW_RST_CEN_SHIFT                        1
#define AXI_CANFD_STS_CONFIG_SHIFT                        0
#define AXI_CANFD_STS_LBACK_SHIFT                         1
#define AXI_CANFD_STS_SLEEP_SHIFT                         2
#define AXI_CANFD_STS_NORMAL_SHIFT                        3
#define AXI_CANFD_STS_BIDLE_SHIFT                         4
#define AXI_CANFD_STS_BBSY_SHIFT                          5
#define AXI_CANFD_STS_ERRWRN_SHIFT                        6
#define AXI_CANFD_STS_ESTAT_SHIFT                         7
#define AXI_CANFD_STS_PEE_CONFIG_SHIFT                    9
#define AXI_CANFD_STS_BSFR_CONFIG_SHIFT                   10
#define AXI_CANFD_STS_SNOOP_SHIFT                         12
#define AXI_CANFD_STS_TDCV_SHIFT                          16
#define AXI_CANFD_DPBRP_DP_BRP_SHIFT                      0
#define AXI_CANFD_DPBRP_TDCOFF_SHIFT                      8
#define AXI_CANFD_DPBRP_TDC_SHIFT                         16
#define AXI_CANFD_APBT_TS1_SHIFT                          0
#define AXI_CANFD_APBT_TS2_SHIFT                          8
#define AXI_CANFD_APBT_SJW_SHIFT                          16
#define AXI_CANFD_DPBT_DP_TS1_SHIFT                       0
#define AXI_CANFD_DPBT_DP_TS2_SHIFT                       8
#define AXI_CANFD_DPBT_DP_SJW_SHIFT                       16
#define AXI_CANFD_INT_EN_ARBLST_SHIFT                     0
#define AXI_CANFD_INT_EN_TXOK_SHIFT                       1
#define AXI_CANFD_INT_EN_PEE_SHIFT                        2
#define AXI_CANFD_INT_EN_BSFRD_SHIFT                      3
#define AXI_CANFD_INT_EN_RXOK_SHIFT                       4
#define AXI_CANFD_INT_EN_TSCNT_OFLW_SHIFT                 5
#define AXI_CANFD_INT_EN_RXFOFLW_SHIFT                    6
#define AXI_CANFD_INT_EN_ERROR_SHIFT                      8
#define AXI_CANFD_INT_EN_BSOFF_SHIFT                      9
#define AXI_CANFD_INT_EN_SLP_SHIFT                        10
#define AXI_CANFD_INT_EN_WKUP_SHIFT                       11
#define AXI_CANFD_INT_EN_RXFWMFLL_SHIFT                   12
#define AXI_CANFD_INT_EN_TXRRS_SHIFT                      13
#define AXI_CANFD_INT_EN_TXCRS_SHIFT                      14
#define AXI_CANFD_INT_EN_RXRBF_RXFOFLW_1_SHIFT            15
#define AXI_CANFD_INT_EN_RXBOFLW_RXFWMFLL_1_SHIFT         16
#define AXI_CANFD_INT_EN_RXMNF_SHIFT                      17
#define AXI_CANFD_INT_EN_TXEOFLW_SHIFT                    30
#define AXI_CANFD_INT_EN_TXEWMFLL_SHIFT                   31
#define AXI_CANFD_INT_CLR_ARBLST_SHIFT                    0
#define AXI_CANFD_INT_CLR_TXOK_SHIFT                      1
#define AXI_CANFD_INT_CLR_PEE_SHIFT                       2
#define AXI_CANFD_INT_CLR_BSFRD_SHIFT                     3
#define AXI_CANFD_INT_CLR_RXOK_SHIFT                      4
#define AXI_CANFD_INT_CLR_TSCNT_OFLW_SHIFT                5
#define AXI_CANFD_INT_CLR_RXFOFLW_SHIFT                   6
#define AXI_CANFD_INT_CLR_ERROR_SHIFT                     8
#define AXI_CANFD_INT_CLR_BSOFF_SHIFT                     9
#define AXI_CANFD_INT_CLR_SLP_SHIFT                       10
#define AXI_CANFD_INT_CLR_WKUP_SHIFT                      11
#define AXI_CANFD_INT_CLR_RXFWMFLL_SHIFT                  12
#define AXI_CANFD_INT_CLR_TXRRS_SHIFT                     13
#define AXI_CANFD_INT_CLR_TXCRS_SHIFT                     14
#define AXI_CANFD_INT_CLR_RXRBF_RXFOFLW_1_SHIFT           15
#define AXI_CANFD_INT_CLR_RXBOFLW_RXFWMFLL_1_SHIFT        16
#define AXI_CANFD_INT_CLR_RXMNF_SHIFT                     17
#define AXI_CANFD_INT_CLR_TXEOFLW_SHIFT                   30
#define AXI_CANFD_INT_CLR_TXEWMFLL_SHIFT                  31
#define AXI_CANFD_MODE_SEL_SLEEP_SHIFT                    0
#define AXI_CANFD_MODE_SEL_LBACK_SHIFT                    1
#define AXI_CANFD_MODE_SEL_SNOOP_SHIFT                    2
#define AXI_CANFD_MODE_SEL_BRSD_SHIFT                     3
#define AXI_CANFD_MODE_SEL_DAR_SHIFT                      4
#define AXI_CANFD_MODE_SEL_DPEE_SHIFT                     5
#define AXI_CANFD_MODE_SEL_SBR_SHIFT                      6
#define AXI_CANFD_MODE_SEL_ABR_SHIFT                      7
#define AXI_CANFD_MODE_SEL_ITO_SHIFT                      8
#define AXI_CANFD_ERR_CNT_TEC_SHIFT                       0
#define AXI_CANFD_ERR_CNT_REC_SHIFT                       8
#define AXI_CANFD_TB_ID_RTR_RRS_SHIFT                     0
#define AXI_CANFD_TB_ID_EXT_ID_SHIFT                      1
#define AXI_CANFD_TB_ID_IDE_SHIFT                         19
#define AXI_CANFD_TB_ID_SRR_RTR_RRS_SHIFT                 20
#define AXI_CANFD_TB_ID_ID_SHIFT                          21
#define AXI_CANFD_TB_DLC_MM_SHIFT                         16
#define AXI_CANFD_TB_DLC_EFC_SHIFT                        24
#define AXI_CANFD_TB_DLC_BRS_SHIFT                        26
#define AXI_CANFD_TB_DLC_EDL_FDF_SHIFT                    27
#define AXI_CANFD_TB_DLC_DLC_SHIFT                        28
#define AXI_CANFD_RB_ID_RTR_RRS_SHIFT                     0
#define AXI_CANFD_RB_ID_EXT_ID_SHIFT                      1
#define AXI_CANFD_RB_ID_IDE_SHIFT                         19
#define AXI_CANFD_RB_ID_SRR_RTR_RRS_SHIFT                 20
#define AXI_CANFD_RB_ID_ID_SHIFT                          21
#define AXI_CANFD_RB_DLC_TS_SHIFT                         0
#define AXI_CANFD_RB_DLC_MATCHED_FILTER_IDX_SHIFT         16
#define AXI_CANFD_RB_DLC_ESI_SHIFT                        25
#define AXI_CANFD_RB_DLC_BRS_SHIFT                        26
#define AXI_CANFD_RB_DLC_EDL_FDF_SHIFT                    27
#define AXI_CANFD_RB_DLC_DLC_SHIFT                        28
#define AXI_CANFD_RXF_STS_RI_SHIFT                        0
#define AXI_CANFD_RXF_STS_IRI_SHIFT                       7
#define AXI_CANFD_RXF_STS_FL_SHIFT                        8
#define AXI_CANFD_RXF_STS_RI_1_SHIFT                      16
#define AXI_CANFD_RXF_STS_IRI_1_SHIFT                     23
#define AXI_CANFD_RXF_STS_FL_1_SHIFT                      24
#define AXI_CANFD_ERR_STS_CRCER_SHIFT                     0
#define AXI_CANFD_ERR_STS_FMER_SHIFT                      1
#define AXI_CANFD_ERR_STS_STER_SHIFT                      2
#define AXI_CANFD_ERR_STS_BERR_SHIFT                      3
#define AXI_CANFD_ERR_STS_ACKER_SHIFT                     4
#define AXI_CANFD_ERR_STS_F_CRCER_SHIFT                   8
#define AXI_CANFD_ERR_STS_F_FMER_SHIFT                    9
#define AXI_CANFD_ERR_STS_F_STER_SHIFT                    10
#define AXI_CANFD_ERR_STS_F_BERR_SHIFT                    11

#define EXTRACT_BITS(val, high, low)                       \
({                                                         \
	typeof(val) __val = (val);                         \
	typeof(high) __high = (high);                      \
	typeof(low) __low = (low);                         \
	(((__val) & GENMASK(__high, __low)) >> (__low));   \
})

struct anlogic_axicanfd_priv {
	struct can_priv can;
	spinlock_t tx_lock; /* Lock for synchronizing TX interrupt handling */
	void __iomem *base_addr;
	struct device *dev;
	unsigned int tx_head;
	unsigned int tx_tail;
	int rx_mailbox_start_pos;
	unsigned long irq_flags;
	struct clk *can_clk;
	struct clk *bus_clk;
	struct napi_struct napi;
	struct {
		unsigned int bus_type;
		unsigned int tx_buffer_num;
		unsigned int rx_mode;
		unsigned int rx_enable_fifo_1;
		unsigned int rx_fifo_0_depth;
		unsigned int rx_fifo_1_depth;
		unsigned int rx_buffer_num;
	} hw_info;
};

static const struct can_bittiming_const axican_bittiming_arbi = {
	.name = "al-axi-canfd",
	.tseg1_min = 1,
	.tseg1_max = 256,
	.tseg2_min = 1,
	.tseg2_max = 128,
	.sjw_max = 128,
	.brp_min = 1,
	.brp_max = 256,
	.brp_inc = 1,
};

static const struct can_bittiming_const axican_bittiming_data = {
	.name = "al-axi-canfd",
	.tseg1_min = 1,
	.tseg1_max = 32,
	.tseg2_min = 1,
	.tseg2_max = 16,
	.sjw_max = 16,
	.brp_min = 1,
	.brp_max = 256,
	.brp_inc = 1,
};

static const struct can_tdc_const axican_tdc = {
	.tdcv_min = 0,
	.tdcv_max = 0, /* Manual mode not supported. */
	.tdco_min = 0,
	.tdco_max = 64,
	.tdcf_min = 0, /* Filter window not supported */
	.tdcf_max = 0,
};

static int anlogic_axicanfd_set_bittiming(struct net_device *ndev)
{
	struct anlogic_axicanfd_priv *priv = netdev_priv(ndev);
	struct can_bittiming *bt = &priv->can.bittiming;
	struct can_bittiming *dbt = &priv->can.data_bittiming;
	u32 btr0, btr1;
	u32 is_config_mode;

	/* Check whether Xilinx CAN is in configuration mode.
	 * It cannot set bit timing if Xilinx CAN is not in configuration mode.
	 */
	is_config_mode = ioread32(priv->base_addr + AXI_CANFD_STS_OFFSET) &
				BIT(AXI_CANFD_STS_CONFIG_SHIFT);
	if (!is_config_mode) {
		netdev_alert(ndev,
			     "BUG! Cannot set bittiming - CAN is not in config mode\n");
		return -EPERM;
	}

	btr0 = (bt->brp - 1);
	btr1 = (bt->prop_seg + bt->phase_seg1 - 1);
	btr1 |= (bt->phase_seg2 - 1) << AXI_CANFD_APBT_TS2_SHIFT;
	btr1 |= (bt->sjw - 1) << AXI_CANFD_APBT_SJW_SHIFT;

	iowrite32(btr0, priv->base_addr + AXI_CANFD_BRP_OFFSET);
	iowrite32(btr1, priv->base_addr + AXI_CANFD_APBT_OFFSET);

	btr0 = dbt->brp - 1;
	if (can_tdc_is_enabled(&priv->can)) {
		btr0 |= ((priv->can.tdc.tdco << AXI_CANFD_DPBRP_TDCOFF_SHIFT) & 0x3F00U) |
			BIT(AXI_CANFD_DPBRP_TDC_SHIFT);
	}

	btr1 = dbt->prop_seg + dbt->phase_seg1 - 1;
	btr1 |= (dbt->phase_seg2 - 1) << AXI_CANFD_DPBT_DP_TS2_SHIFT;
	btr1 |= (dbt->sjw - 1) << AXI_CANFD_DPBT_DP_SJW_SHIFT;

	iowrite32(btr0, priv->base_addr + AXI_CANFD_DPBRP_OFFSET);
	iowrite32(btr1, priv->base_addr + AXI_CANFD_DPBT_OFFSET);

	return 0;
}

static int anlogic_axicanfd_reset(struct net_device *ndev)
{
	struct anlogic_axicanfd_priv *priv = netdev_priv(ndev);
	unsigned long timeout;

	iowrite32(BIT(AXI_CANFD_SW_RST_SRST_SHIFT), priv->base_addr + AXI_CANFD_SW_RST_OFFSET);

	timeout = jiffies + 1 * HZ;
	while (!(ioread32(priv->base_addr + AXI_CANFD_STS_OFFSET) &
		BIT(AXI_CANFD_STS_CONFIG_SHIFT))) {
		if (time_after(jiffies, timeout)) {
			netdev_warn(ndev, "timed out for config mode\n");
			return -ETIMEDOUT;
		}
		usleep_range(500, 10000);
	}

	/* reset clears FIFOs */
	priv->tx_head = 0;
	priv->tx_tail = 0;
	priv->rx_mailbox_start_pos = 0;

	return 0;
}

static int anlogic_axicanfd_init(struct net_device *ndev)
{
	int ret;
	struct anlogic_axicanfd_priv *priv = netdev_priv(ndev);

	ret = anlogic_axicanfd_reset(ndev);
	if (ret < 0)
		return ret;

	ret = anlogic_axicanfd_set_bittiming(ndev);
	if (ret < 0)
		return ret;

	iowrite32(BIT(AXI_CANFD_INT_EN_ARBLST_SHIFT) |
		  BIT(AXI_CANFD_INT_EN_TXOK_SHIFT) |
		  BIT(AXI_CANFD_INT_EN_RXOK_SHIFT) |
		  BIT(AXI_CANFD_INT_EN_RXFOFLW_SHIFT) |
		  BIT(AXI_CANFD_INT_EN_ERROR_SHIFT) |
		  BIT(AXI_CANFD_INT_EN_BSOFF_SHIFT) |
		  BIT(AXI_CANFD_INT_EN_SLP_SHIFT) |
		  BIT(AXI_CANFD_INT_EN_WKUP_SHIFT) |
		  BIT(AXI_CANFD_INT_EN_RXMNF_SHIFT),
		  priv->base_addr + AXI_CANFD_INT_EN_OFFSET);

	if (priv->hw_info.rx_mode == 0) {
		iowrite32(0x00000001, priv->base_addr + AXI_CANFD_AF_OFFSET);
	} else {
		iowrite32(0x0000FFFFU,
			  priv->base_addr + AXI_CANFD_RXBC_STS0_OFFSET);
		iowrite32(0x0000FFFFU,
			  priv->base_addr + AXI_CANFD_RXBC_STS0_OFFSET + 4U);
		iowrite32(0x0000FFFFU,
			  priv->base_addr + AXI_CANFD_RXBC_STS0_OFFSET + 8U);
	}

	iowrite32((priv->can.ctrlmode & CAN_CTRLMODE_LOOPBACK) ?
		   BIT(AXI_CANFD_MODE_SEL_LBACK_SHIFT) : 0,
		   priv->base_addr + AXI_CANFD_MODE_SEL_OFFSET);
	iowrite32(BIT(AXI_CANFD_SW_RST_CEN_SHIFT), priv->base_addr + AXI_CANFD_SW_RST_OFFSET);

	priv->can.state = CAN_STATE_ERROR_ACTIVE;
	return 0;
}

static int anlogic_axicanfd_set_mode(struct net_device *ndev, enum can_mode mode)
{
	int ret;

	switch (mode) {
	case CAN_MODE_START:
		ret = anlogic_axicanfd_init(ndev);
		if (ret < 0)
			return ret;
		netif_wake_queue(ndev);
		break;
	default:
		return -EOPNOTSUPP;
	}

	return 0;
}

static int anlogic_axicanfd_get_auto_tdcv(const struct net_device *ndev, u32 *tdcv)
{
	struct anlogic_axicanfd_priv *priv = netdev_priv(ndev);

	*tdcv = ((ioread32(priv->base_addr + AXI_CANFD_STS_OFFSET) & 0x7F0000) >>
		AXI_CANFD_STS_TDCV_SHIFT);

	return 0;
}

static int anlogic_axicanfd_get_berr_counter(const struct net_device *ndev,
					     struct can_berr_counter *bec)
{
	int ret;
	u32 err;
	struct anlogic_axicanfd_priv *priv = netdev_priv(ndev);

	ret = pm_runtime_get_sync(priv->dev);
	if (ret < 0) {
		netdev_err(ndev, "%s: pm_runtime_get failed(%d)\n",
			   __func__, ret);
		pm_runtime_put(priv->dev);
		return ret;
	}

	err = ioread32(priv->base_addr + AXI_CANFD_ERR_CNT_OFFSET);
	bec->txerr = err & 0xFFU;
	bec->rxerr = (err & 0xFF00U) >> AXI_CANFD_ERR_CNT_REC_SHIFT;

	return 0;
}

static void anlogic_axicanfd_set_error_state(struct net_device *ndev,
					     enum can_state new_state,
					     struct can_frame *cf)
{
	struct anlogic_axicanfd_priv *priv = netdev_priv(ndev);
	u32 ecr = ioread32(priv->base_addr + AXI_CANFD_ERR_CNT_OFFSET);
	u32 txerr = EXTRACT_BITS(ecr, 7, 0);
	u32 rxerr = EXTRACT_BITS(ecr, 15, 8);
	enum can_state tx_state = txerr >= rxerr ? new_state : 0;
	enum can_state rx_state = txerr <= rxerr ? new_state : 0;

	/* non-ERROR states are handled elsewhere */
	if (WARN_ON(new_state > CAN_STATE_ERROR_PASSIVE))
		return;

	can_change_state(ndev, cf, tx_state, rx_state);

	if (cf) {
		cf->can_id |= CAN_ERR_CNT;
		cf->data[6] = txerr;
		cf->data[7] = rxerr;
	}
}

static void anlogic_axicanfd_update_error_state_after_rxtx(struct net_device *ndev)
{
	struct anlogic_axicanfd_priv *priv = netdev_priv(ndev);
	enum can_state old_state = priv->can.state;
	enum can_state new_state;
	u32 status;

	/* changing error state due to successful frame RX/TX can only
	 * occur from these states
	 */
	if (old_state != CAN_STATE_ERROR_WARNING &&
	    old_state != CAN_STATE_ERROR_PASSIVE)
		return;

	status = ioread32(priv->base_addr + AXI_CANFD_STS_OFFSET);
	if (EXTRACT_BITS(status, 8, AXI_CANFD_STS_ESTAT_SHIFT) == 0x3U)
		new_state = CAN_STATE_ERROR_PASSIVE;
	else if (status & BIT(AXI_CANFD_STS_ERRWRN_SHIFT))
		new_state = CAN_STATE_ERROR_WARNING;
	else
		new_state = CAN_STATE_ERROR_ACTIVE;

	if (new_state != old_state) {
		struct sk_buff *skb;
		struct can_frame *cf;

		skb = alloc_can_err_skb(ndev, &cf);

		anlogic_axicanfd_set_error_state(ndev, new_state, skb ? cf : NULL);

		if (skb)
			netif_rx(skb);
	}
}

static int anlogic_axicanfd_rx_frame_sequential(struct napi_struct *napi, int quota)
{
	u32 rx_sts;
	u32 ier;
	u32 frame_idx;
	int i;
	u32 id_reg, dlc_reg;
	u32 dw_offset;
	u32 frame_cnt = 0;
	u32 data[2] = {0, 0};
	struct sk_buff *skb;
	struct canfd_frame *cf;
	struct net_device *ndev = napi->dev;
	struct net_device_stats *stats = &ndev->stats;
	struct anlogic_axicanfd_priv *priv = netdev_priv(ndev);

	while (1) {
		iowrite32(BIT(AXI_CANFD_INT_CLR_RXOK_SHIFT),
			  priv->base_addr + AXI_CANFD_INT_CLR_OFFSET);

		rx_sts = ioread32(priv->base_addr + AXI_CANFD_RXF_STS_OFFSET);

		if (EXTRACT_BITS(rx_sts, 14, 8) == 0)
			break;

		frame_idx = EXTRACT_BITS(rx_sts, 5, 0);

		id_reg = ioread32(priv->base_addr + AXI_CANFD_RB_ID_OFFSET(frame_idx));
		dlc_reg = ioread32(priv->base_addr + AXI_CANFD_RB_DLC_OFFSET(frame_idx));

		if (dlc_reg & BIT(AXI_CANFD_RB_DLC_EDL_FDF_SHIFT))
			skb = alloc_canfd_skb(ndev, &cf);
		else
			skb = alloc_can_skb(ndev, (struct can_frame **)&cf);

		if (unlikely(!skb)) {
			stats->rx_dropped++;
			iowrite32(rx_sts | BIT(AXI_CANFD_RXF_STS_IRI_SHIFT),
				  priv->base_addr + AXI_CANFD_RXF_STS_OFFSET);
			continue;
		}

		if (id_reg & BIT(AXI_CANFD_RB_ID_IDE_SHIFT)) {
			cf->can_id = (EXTRACT_BITS(id_reg, 31, 21) << 18) |
				(EXTRACT_BITS(id_reg, 18, 1) << 0);
			cf->can_id |= CAN_EFF_FLAG;
			if (id_reg & BIT(AXI_CANFD_RB_ID_RTR_RRS_SHIFT))
				cf->can_id |= CAN_RTR_FLAG;
		} else {
			cf->can_id = EXTRACT_BITS(id_reg, 31, 21);
			if (!(dlc_reg & BIT(AXI_CANFD_RB_DLC_EDL_FDF_SHIFT)) &&
			    (id_reg & BIT(AXI_CANFD_RB_ID_SRR_RTR_RRS_SHIFT)))
				cf->can_id |= CAN_RTR_FLAG;
		}

		cf->len = can_fd_dlc2len(EXTRACT_BITS(dlc_reg, 31, 28));

		for (i = 0; i < cf->len; i += 4) {
			dw_offset = AXI_CANFD_RB_DW0_OFFSET(frame_idx);
			data[0] = ioread32(priv->base_addr + dw_offset + i);
			*(__be32 *)(cf->data + i) = cpu_to_be32(data[0]);
		}

		if (!(cf->can_id & CAN_RTR_FLAG))
			stats->rx_bytes += cf->len;
		stats->rx_packets++;

		netif_receive_skb(skb);

		iowrite32(rx_sts | BIT(AXI_CANFD_RXF_STS_IRI_SHIFT),
			  priv->base_addr + AXI_CANFD_RXF_STS_OFFSET);

		frame_cnt++;

		if (frame_cnt >= quota)
			break;
	}

	if (frame_cnt)
		anlogic_axicanfd_update_error_state_after_rxtx(ndev);

	if (frame_cnt < quota) {
		if (napi_complete_done(napi, frame_cnt)) {
			ier = ioread32(priv->base_addr + AXI_CANFD_INT_EN_OFFSET);
			iowrite32(ier | BIT(AXI_CANFD_INT_EN_RXOK_SHIFT),
				  priv->base_addr + AXI_CANFD_INT_EN_OFFSET);
		}
	}

	return frame_cnt;
}

static int anlogic_axicanfd_rx_frame_mailbox(struct napi_struct *napi, int quota)
{
	u32 ier;
	int i;
	int frame_idx;
	int loop_cnt = 0;
	u32 id_reg, dlc_reg;
	u32 dw_offset;
	u32 rx_buf_sts;
	u32 frame_cnt = 0;
	u32 data[2] = {0, 0};
	struct sk_buff *skb;
	struct canfd_frame *cf;
	struct net_device *ndev = napi->dev;
	struct net_device_stats *stats = &ndev->stats;
	struct anlogic_axicanfd_priv *priv = netdev_priv(ndev);

	frame_idx = priv->rx_mailbox_start_pos - 1;

	while (1) {
		if (frame_idx >= 0 && loop_cnt != 0)
			iowrite32(BIT(frame_idx % 16 + 16) | 0x0000FFFFU,
				  priv->base_addr +
				  AXI_CANFD_RXBC_STS0_OFFSET + (frame_idx / 16) * 4);

		if (loop_cnt >= priv->hw_info.rx_buffer_num)
			break;

		loop_cnt++;

		if (frame_cnt >= quota)
			break;

		frame_idx++;
		frame_idx %= priv->hw_info.rx_buffer_num;

		iowrite32(BIT(AXI_CANFD_INT_CLR_RXOK_SHIFT),
			  priv->base_addr + AXI_CANFD_INT_CLR_OFFSET);

		rx_buf_sts = ioread32(priv->base_addr +
			AXI_CANFD_RXBC_STS0_OFFSET + (frame_idx / 16) * 4);
		if (!(rx_buf_sts & BIT(frame_idx % 16 + 16)))
			continue;

		id_reg = ioread32(priv->base_addr + AXI_CANFD_RB_ID_OFFSET(frame_idx));
		dlc_reg = ioread32(priv->base_addr + AXI_CANFD_RB_DLC_OFFSET(frame_idx));

		if (dlc_reg & BIT(AXI_CANFD_RB_DLC_EDL_FDF_SHIFT))
			skb = alloc_canfd_skb(ndev, &cf);
		else
			skb = alloc_can_skb(ndev, (struct can_frame **)&cf);

		if (unlikely(!skb)) {
			stats->rx_dropped++;
			continue;
		}

		if (id_reg & BIT(AXI_CANFD_RB_ID_IDE_SHIFT)) {
			cf->can_id = (EXTRACT_BITS(id_reg, 31, 21) << 18) |
				(EXTRACT_BITS(id_reg, 18, 1) << 0);
			cf->can_id |= CAN_EFF_FLAG;
			if (id_reg & BIT(AXI_CANFD_RB_ID_RTR_RRS_SHIFT))
				cf->can_id |= CAN_RTR_FLAG;
		} else {
			cf->can_id = EXTRACT_BITS(id_reg, 31, 21);
			if (!(dlc_reg & BIT(AXI_CANFD_RB_DLC_EDL_FDF_SHIFT)) &&
			    (id_reg & BIT(AXI_CANFD_RB_ID_SRR_RTR_RRS_SHIFT)))
				cf->can_id |= CAN_RTR_FLAG;
		}

		cf->len = can_fd_dlc2len(EXTRACT_BITS(dlc_reg, 31, 28));

		for (i = 0; i < cf->len; i += 4) {
			dw_offset = AXI_CANFD_RB_DW0_OFFSET(frame_idx);
			data[0] = ioread32(priv->base_addr + dw_offset + i);
			*(__be32 *)(cf->data + i) = cpu_to_be32(data[0]);
		}

		if (!(cf->can_id & CAN_RTR_FLAG))
			stats->rx_bytes += cf->len;
		stats->rx_packets++;

		netif_receive_skb(skb);
		frame_cnt++;
		priv->rx_mailbox_start_pos = (frame_idx + 1) % priv->hw_info.rx_buffer_num;
	}

	if (frame_cnt)
		anlogic_axicanfd_update_error_state_after_rxtx(ndev);

	if (frame_cnt < quota) {
		if (napi_complete_done(napi, frame_cnt)) {
			ier = ioread32(priv->base_addr + AXI_CANFD_INT_EN_OFFSET);
			iowrite32(ier | BIT(AXI_CANFD_INT_EN_RXOK_SHIFT),
				  priv->base_addr + AXI_CANFD_INT_EN_OFFSET);
		}
	}

	return frame_cnt;
}

static int anlogic_axicanfd_rx_poll(struct napi_struct *napi, int quota)
{
	struct net_device *ndev = napi->dev;
	struct anlogic_axicanfd_priv *priv = netdev_priv(ndev);

	if (priv->hw_info.rx_mode == 0)
		/* Sequential */
		return anlogic_axicanfd_rx_frame_sequential(napi, quota);

	/* Mailbox */
	return anlogic_axicanfd_rx_frame_mailbox(napi, quota);
}

static void anlogic_axicanfd_state_intr_handle(struct net_device *ndev, u32 isr)
{
	struct anlogic_axicanfd_priv *priv = netdev_priv(ndev);

	if (isr & AXI_CANFD_INT_EN_SLP_SHIFT)
		priv->can.state = CAN_STATE_SLEEPING;

	if (isr & AXI_CANFD_INT_EN_WKUP_SHIFT)
		priv->can.state = CAN_STATE_ERROR_ACTIVE;
}

static void anlogic_axicanfd_tx_intr_handle(struct net_device *ndev, u32 isr)
{
	struct anlogic_axicanfd_priv *priv = netdev_priv(ndev);
	struct net_device_stats *stats = &ndev->stats;
	unsigned int frames_in_fifo;
	int frames_sent = 1; /* TXOK => at least 1 frame was sent */
	unsigned long flags;

	/* Synchronize with xmit as we need to know the exact number
	 * of frames in the FIFO to stay in sync due to the TXFEMP
	 * handling.
	 * This also prevents a race between netif_wake_queue() and
	 * netif_stop_queue().
	 */
	spin_lock_irqsave(&priv->tx_lock, flags);

	frames_in_fifo = priv->tx_head - priv->tx_tail;

	if (WARN_ON_ONCE(frames_in_fifo == 0)) {
		/* clear TXOK anyway to avoid getting back here */
		iowrite32(BIT(AXI_CANFD_INT_CLR_TXOK_SHIFT),
			  priv->base_addr + AXI_CANFD_INT_CLR_OFFSET);
		spin_unlock_irqrestore(&priv->tx_lock, flags);
		return;
	}

	iowrite32(BIT(AXI_CANFD_INT_CLR_TXOK_SHIFT),
		  priv->base_addr + AXI_CANFD_INT_CLR_OFFSET);

	while (frames_sent--) {
		stats->tx_bytes += can_get_echo_skb(ndev, priv->tx_tail %
				priv->hw_info.tx_buffer_num, NULL);
		priv->tx_tail++;
		stats->tx_packets++;
	}

	netif_wake_queue(ndev);

	spin_unlock_irqrestore(&priv->tx_lock, flags);

	anlogic_axicanfd_update_error_state_after_rxtx(ndev);
}

static void anlogic_axicanfd_err_interrupt(struct net_device *ndev, u32 isr)
{
	struct anlogic_axicanfd_priv *priv = netdev_priv(ndev);
	struct net_device_stats *stats = &ndev->stats;
	struct can_frame cf = { };
	u32 err_status;

	err_status = ioread32(priv->base_addr + AXI_CANFD_ERR_STS_OFFSET);
	iowrite32(err_status, priv->base_addr + AXI_CANFD_ERR_STS_OFFSET);

	if (isr & BIT(AXI_CANFD_INT_EN_BSOFF_SHIFT)) {
		priv->can.state = CAN_STATE_BUS_OFF;
		priv->can.can_stats.bus_off++;
		/* Leave device in Config Mode in bus-off state */
		iowrite32(BIT(AXI_CANFD_SW_RST_SRST_SHIFT),
			  priv->base_addr + AXI_CANFD_SW_RST_OFFSET);
		can_bus_off(ndev);
		cf.can_id |= CAN_ERR_BUSOFF;
	} else {
		enum can_state new_state;
		u32 status;

		status = ioread32(priv->base_addr + AXI_CANFD_STS_OFFSET);
		if (EXTRACT_BITS(status, 8, AXI_CANFD_STS_ESTAT_SHIFT) == 0x3U)
			new_state = CAN_STATE_ERROR_PASSIVE;
		else if (status & BIT(AXI_CANFD_STS_ERRWRN_SHIFT))
			new_state = CAN_STATE_ERROR_WARNING;
		else
			new_state = CAN_STATE_ERROR_ACTIVE;

		if (new_state != priv->can.state)
			anlogic_axicanfd_set_error_state(ndev, new_state, &cf);
	}

	/* Check for Arbitration lost interrupt */
	if (isr & BIT(AXI_CANFD_INT_EN_ARBLST_SHIFT)) {
		priv->can.can_stats.arbitration_lost++;
		cf.can_id |= CAN_ERR_LOSTARB;
		cf.data[0] = CAN_ERR_LOSTARB_UNSPEC;
	}

	/* Check for RX FIFO Overflow interrupt */
	if (isr & BIT(AXI_CANFD_INT_EN_RXFOFLW_SHIFT)) {
		stats->rx_over_errors++;
		stats->rx_errors++;
		cf.can_id |= CAN_ERR_CRTL;
		cf.data[1] |= CAN_ERR_CRTL_RX_OVERFLOW;
	}

	/* Check for RX Match Not Finished interrupt */
	if (isr & BIT(AXI_CANFD_INT_EN_RXMNF_SHIFT)) {
		stats->rx_dropped++;
		stats->rx_errors++;
		netdev_err(ndev, "RX match not finished, frame discarded\n");
		cf.can_id |= CAN_ERR_CRTL;
		cf.data[1] |= CAN_ERR_CRTL_UNSPEC;
	}

	/* Check for error interrupt */
	if (isr & BIT(AXI_CANFD_INT_EN_ERROR_SHIFT)) {
		bool berr_reporting = false;

		if (priv->can.ctrlmode & CAN_CTRLMODE_BERR_REPORTING) {
			berr_reporting = true;
			cf.can_id |= CAN_ERR_PROT | CAN_ERR_BUSERROR;
		}

		/* Check for Ack error interrupt */
		if (err_status & BIT(AXI_CANFD_ERR_STS_ACKER_SHIFT)) {
			stats->tx_errors++;
			if (berr_reporting) {
				cf.can_id |= CAN_ERR_ACK;
				cf.data[3] = CAN_ERR_PROT_LOC_ACK;
			}
		}

		/* Check for Bit error interrupt */
		if (err_status & BIT(AXI_CANFD_ERR_STS_BERR_SHIFT)) {
			stats->tx_errors++;
			if (berr_reporting) {
				cf.can_id |= CAN_ERR_PROT;
				cf.data[2] = CAN_ERR_PROT_BIT;
			}
		}

		/* Check for Stuff error interrupt */
		if (err_status & BIT(AXI_CANFD_ERR_STS_STER_SHIFT)) {
			stats->rx_errors++;
			if (berr_reporting) {
				cf.can_id |= CAN_ERR_PROT;
				cf.data[2] = CAN_ERR_PROT_STUFF;
			}
		}

		/* Check for Form error interrupt */
		if (err_status & BIT(AXI_CANFD_ERR_STS_FMER_SHIFT)) {
			stats->rx_errors++;
			if (berr_reporting) {
				cf.can_id |= CAN_ERR_PROT;
				cf.data[2] = CAN_ERR_PROT_FORM;
			}
		}

		/* Check for CRC error interrupt */
		if (err_status & BIT(AXI_CANFD_ERR_STS_CRCER_SHIFT)) {
			stats->rx_errors++;
			if (berr_reporting) {
				cf.can_id |= CAN_ERR_PROT;
				cf.data[3] = CAN_ERR_PROT_LOC_CRC_SEQ;
			}
		}
		priv->can.can_stats.bus_error++;
	}

	if (cf.can_id) {
		struct can_frame *skb_cf;
		struct sk_buff *skb = alloc_can_err_skb(ndev, &skb_cf);

		if (skb) {
			skb_cf->can_id |= cf.can_id;
			memcpy(skb_cf->data, cf.data, CAN_ERR_DLC);
			netif_rx(skb);
		}
	}
}

static irqreturn_t anlogic_axicanfd_intr_handle(int irq, void *dev_id)
{
	struct net_device *ndev = (struct net_device *)dev_id;
	struct anlogic_axicanfd_priv *priv = netdev_priv(ndev);
	u32 ier;
	u32 status;
	u32 err_status;

	status = ioread32(priv->base_addr + AXI_CANFD_INT_STS_OFFSET);
	if (!status)
		return IRQ_NONE;

	if (status & (BIT(AXI_CANFD_INT_EN_SLP_SHIFT) | BIT(AXI_CANFD_INT_EN_WKUP_SHIFT))) {
		iowrite32(BIT(AXI_CANFD_INT_CLR_SLP_SHIFT) | BIT(AXI_CANFD_INT_CLR_WKUP_SHIFT),
			  priv->base_addr + AXI_CANFD_INT_CLR_OFFSET);
		anlogic_axicanfd_state_intr_handle(ndev, status);
	}

	if (status & BIT(AXI_CANFD_INT_EN_TXOK_SHIFT))
		anlogic_axicanfd_tx_intr_handle(ndev, status);

	err_status = status & (BIT(AXI_CANFD_INT_EN_ARBLST_SHIFT) |
			       BIT(AXI_CANFD_INT_EN_RXFOFLW_SHIFT) |
			       BIT(AXI_CANFD_INT_EN_ERROR_SHIFT) |
			       BIT(AXI_CANFD_INT_EN_BSOFF_SHIFT) |
			       BIT(AXI_CANFD_INT_EN_RXMNF_SHIFT));
	if (err_status) {
		iowrite32(err_status, priv->base_addr + AXI_CANFD_INT_CLR_OFFSET);
		anlogic_axicanfd_err_interrupt(ndev, status);
	}

	if (status & BIT(AXI_CANFD_INT_EN_RXOK_SHIFT)) {
		ier = ioread32(priv->base_addr + AXI_CANFD_INT_EN_OFFSET);
		ier &= ~BIT(AXI_CANFD_INT_EN_RXOK_SHIFT);
		iowrite32(ier, priv->base_addr + AXI_CANFD_INT_EN_OFFSET);
		napi_schedule(&priv->napi);
	}

	return IRQ_HANDLED;
}

static int anlogic_axicanfd_ndo_open(struct net_device *ndev)
{
	int ret;
	struct anlogic_axicanfd_priv *priv = netdev_priv(ndev);

	ret = pm_runtime_get_sync(priv->dev);
	if (ret < 0) {
		netdev_err(ndev, "%s: pm_runtime_get failed(%d)\n",
			   __func__, ret);
		goto err0;
	}

	ret = request_irq(ndev->irq, anlogic_axicanfd_intr_handle, priv->irq_flags,
			  ndev->name, ndev);
	if (ret < 0) {
		netdev_err(ndev, "irq allocation for CAN failed\n");
		goto err1;
	}

	ret = anlogic_axicanfd_reset(ndev);
	if (ret < 0)
		goto err2;

	ret = open_candev(ndev);
	if (ret < 0)
		goto err2;

	ret = anlogic_axicanfd_init(ndev);
	if (ret < 0)
		goto err3;

	napi_enable(&priv->napi);
	netif_start_queue(ndev);

	return 0;

err3:
	close_candev(ndev);
err2:
	free_irq(ndev->irq, ndev);
err1:
	pm_runtime_put(priv->dev);
err0:
	return ret;
}

static int anlogic_axicanfd_ndo_stop(struct net_device *ndev)
{
	int ret;
	struct anlogic_axicanfd_priv *priv = netdev_priv(ndev);

	netif_stop_queue(ndev);
	napi_disable(&priv->napi);

	ret = anlogic_axicanfd_reset(ndev);
	if (ret < 0)
		netdev_dbg(ndev, "anlogic_axicanfd_reset() Failed\n");

	priv->can.state = CAN_STATE_STOPPED;

	free_irq(ndev->irq, ndev);
	close_candev(ndev);
	pm_runtime_put(priv->dev);

	return 0;
}

static int anlogic_axicanfd_ndo_start_xmit(struct sk_buff *skb, struct net_device *ndev)
{
	int ret, i;
	u32 id_reg, dlc_reg;
	unsigned long flags;
	struct anlogic_axicanfd_priv *priv = netdev_priv(ndev);
	struct canfd_frame *cf = (struct canfd_frame *)skb->data;

	if (can_dev_dropped_skb(ndev, skb))
		return NETDEV_TX_OK;

	if (unlikely(ioread32(priv->base_addr + AXI_CANFD_TBRR_OFFSET) & BIT(0)))
		return -ENOSPC;

	spin_lock_irqsave(&priv->tx_lock, flags);

	if (cf->can_id & CAN_EFF_FLAG) {
		id_reg = EXTRACT_BITS(cf->can_id, 17, 0) << AXI_CANFD_TB_ID_EXT_ID_SHIFT;
		id_reg |= (EXTRACT_BITS(cf->can_id, 28, 18) << AXI_CANFD_TB_ID_ID_SHIFT);
		id_reg |= BIT(AXI_CANFD_TB_ID_IDE_SHIFT) |
			BIT(AXI_CANFD_TB_ID_SRR_RTR_RRS_SHIFT);

		if (cf->can_id & CAN_RTR_FLAG)
			id_reg |= BIT(AXI_CANFD_TB_ID_RTR_RRS_SHIFT);
	} else {
		id_reg = EXTRACT_BITS(cf->can_id, 10, 0) << AXI_CANFD_TB_ID_ID_SHIFT;

		if (cf->can_id & CAN_RTR_FLAG)
			id_reg |= BIT(AXI_CANFD_TB_ID_RTR_RRS_SHIFT);
	}

	dlc_reg = can_fd_len2dlc(cf->len) << AXI_CANFD_TB_DLC_DLC_SHIFT;
	if (can_is_canfd_skb(skb)) {
		dlc_reg |= BIT(AXI_CANFD_TB_DLC_EDL_FDF_SHIFT);
		if (cf->flags & CANFD_BRS)
			dlc_reg |= BIT(AXI_CANFD_TB_DLC_BRS_SHIFT);
	}

	ret = can_put_echo_skb(skb, ndev, 0, 0);  // to do
	if (ret < 0) {
		spin_unlock_irqrestore(&priv->tx_lock, flags);
		return ret;
	}

	priv->tx_head++;

	iowrite32(id_reg, priv->base_addr + AXI_CANFD_TB_ID_OFFSET(0));
	iowrite32(dlc_reg, priv->base_addr + AXI_CANFD_TB_DLC_OFFSET(0));

	for (i = 0; i < cf->len; i += 4)
		iowrite32(be32_to_cpup((__be32 *)(cf->data + i)),
			  priv->base_addr + AXI_CANFD_TB_DW0_OFFSET(0) + i);

	iowrite32(BIT(0), priv->base_addr + AXI_CANFD_TBRR_OFFSET);

	netif_stop_queue(ndev);

	spin_unlock_irqrestore(&priv->tx_lock, flags);

	return NETDEV_TX_OK;
}

static int __maybe_unused anlogic_axicanfd_suspend(struct device *dev)
{
	struct net_device *ndev = dev_get_drvdata(dev);
	struct anlogic_axicanfd_priv *priv = netdev_priv(ndev);

	if (netif_running(ndev)) {
		netif_stop_queue(ndev);
		netif_device_detach(ndev);

		anlogic_axicanfd_reset(ndev);
		priv->can.state = CAN_STATE_STOPPED;
	}

	return pm_runtime_force_suspend(dev);
}

static int __maybe_unused anlogic_axicanfd_resume(struct device *dev)
{
	struct net_device *ndev = dev_get_drvdata(dev);
	int ret;

	ret = pm_runtime_force_resume(dev);
	if (ret) {
		dev_err(dev, "pm_runtime_force_resume failed on resume\n");
		return ret;
	}

	if (netif_running(ndev)) {
		ret = anlogic_axicanfd_init(ndev);
		if (ret) {
			dev_err(dev, "anlogic_axicanfd_init failed on resume\n");
			return ret;
		}

		netif_device_attach(ndev);
		netif_start_queue(ndev);
	}

	return 0;
}

static int __maybe_unused anlogic_axicanfd_runtime_suspend(struct device *dev)
{
	struct net_device *ndev = dev_get_drvdata(dev);
	struct anlogic_axicanfd_priv *priv = netdev_priv(ndev);

	clk_disable_unprepare(priv->bus_clk);
	clk_disable_unprepare(priv->can_clk);

	return 0;
}

static int __maybe_unused anlogic_axicanfd_runtime_resume(struct device *dev)
{
	struct net_device *ndev = dev_get_drvdata(dev);
	struct anlogic_axicanfd_priv *priv = netdev_priv(ndev);
	int ret;

	ret = clk_prepare_enable(priv->bus_clk);
	if (ret) {
		dev_err(dev, "Cannot enable clock.\n");
		return ret;
	}
	ret = clk_prepare_enable(priv->can_clk);
	if (ret) {
		dev_err(dev, "Cannot enable clock.\n");
		clk_disable_unprepare(priv->bus_clk);
		return ret;
	}

	return 0;
}

static const struct net_device_ops axi_canfd_netdev_ops = {
	.ndo_open	= anlogic_axicanfd_ndo_open,
	.ndo_stop	= anlogic_axicanfd_ndo_stop,
	.ndo_start_xmit	= anlogic_axicanfd_ndo_start_xmit,
	.ndo_change_mtu	= can_change_mtu,
};

static const struct ethtool_ops axi_canfd_ethtool_ops = {
	.get_ts_info = ethtool_op_get_ts_info,
};

static const struct dev_pm_ops xcan_dev_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(anlogic_axicanfd_suspend, anlogic_axicanfd_resume)
	SET_RUNTIME_PM_OPS(anlogic_axicanfd_runtime_suspend, anlogic_axicanfd_runtime_resume, NULL)
};

static const struct of_device_id of_match_table[] = {
	{ .compatible = "anlogic,axi-canfd"},
	{ /* end of list */ },
};
MODULE_DEVICE_TABLE(of, xcan_of_match);

static int anlogic_axicanfd_probe(struct platform_device *pdev)
{
	int ret;
	int tx_buf_num;
	int rx_weight;
	void __iomem *addr;
	struct net_device *ndev;
	struct anlogic_axicanfd_priv *priv;

	addr = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(addr)) {
		ret = PTR_ERR(addr);
		goto end1;
	}

	ret = of_property_read_u32(pdev->dev.of_node, "tx-buffer-num",
				   &tx_buf_num);
	if (ret < 0) {
		dev_err(&pdev->dev, "get property error\n");
		goto end1;
	}

	tx_buf_num = 1;

	ndev = alloc_candev(sizeof(struct anlogic_axicanfd_priv), tx_buf_num);
	if (!ndev) {
		ret = -ENOMEM;
		goto end1;
	}

	priv = netdev_priv(ndev);
	priv->hw_info.tx_buffer_num = tx_buf_num;
	ret |= of_property_read_u32(pdev->dev.of_node, "bus-type",
				    &priv->hw_info.bus_type) |
		of_property_read_u32(pdev->dev.of_node, "rx-mode",
				     &priv->hw_info.rx_mode) |
		of_property_read_u32(pdev->dev.of_node, "rx-enable-fifo_1",
				     &priv->hw_info.rx_enable_fifo_1) |
		of_property_read_u32(pdev->dev.of_node, "rx-fifo-0-depth",
				     &priv->hw_info.rx_fifo_0_depth) |
		of_property_read_u32(pdev->dev.of_node, "rx-fifo-1-depth",
				     &priv->hw_info.rx_fifo_1_depth) |
		of_property_read_u32(pdev->dev.of_node, "rx-buffer-num",
				     &priv->hw_info.rx_buffer_num);
	if (ret < 0) {
		dev_err(&pdev->dev, "get property error\n");
		goto end2;
	}

	priv->dev = &pdev->dev;
	priv->can.bittiming_const = &axican_bittiming_arbi;
	priv->can.do_set_mode = anlogic_axicanfd_set_mode;
	priv->can.do_get_berr_counter = anlogic_axicanfd_get_berr_counter;
	priv->can.ctrlmode_supported = CAN_CTRLMODE_LOOPBACK |
					CAN_CTRLMODE_BERR_REPORTING |
					CAN_CTRLMODE_FD |
					CAN_CTRLMODE_TDC_AUTO;

	priv->can.data_bittiming_const = &axican_bittiming_data;
	priv->can.tdc_const = &axican_tdc;
	priv->can.do_get_auto_tdcv = anlogic_axicanfd_get_auto_tdcv;

	priv->base_addr = addr;
	spin_lock_init(&priv->tx_lock);

	ret = platform_get_irq(pdev, 0);
	if (ret < 0)
		goto end2;

	ndev->irq = ret;
	ndev->flags |= IFF_ECHO;

	platform_set_drvdata(pdev, ndev);
	SET_NETDEV_DEV(ndev, &pdev->dev);
	ndev->netdev_ops = &axi_canfd_netdev_ops;
	ndev->ethtool_ops = &axi_canfd_ethtool_ops;

	priv->can_clk = devm_clk_get(&pdev->dev, "can_clk");
	if (IS_ERR(priv->can_clk)) {
		ret = dev_err_probe(&pdev->dev, PTR_ERR(priv->can_clk),
				    "failed to get can_clk\n");
		goto end2;
	}

	priv->bus_clk = devm_clk_get(&pdev->dev, "bus_clk");
	if (IS_ERR(priv->bus_clk)) {
		ret = dev_err_probe(&pdev->dev, PTR_ERR(priv->bus_clk),
				    "failed to get bus_clk\n");
		goto end2;
	}

	pm_runtime_enable(&pdev->dev);
	ret = pm_runtime_get_sync(&pdev->dev);
	if (ret < 0) {
		dev_err(&pdev->dev, "pm_runtime_get_sync failed\n");
		goto end3;
	}

	priv->can.clock.freq = clk_get_rate(priv->can_clk);

	rx_weight = priv->hw_info.rx_mode == 0 ? priv->hw_info.rx_fifo_0_depth :
		priv->hw_info.rx_buffer_num;
	netif_napi_add_weight(ndev, &priv->napi, anlogic_axicanfd_rx_poll, rx_weight);

	ret = register_candev(ndev);
	if (ret < 0) {
		dev_err(&pdev->dev, "register_candev failed\n");
		goto end4;
	}

	pm_runtime_put(&pdev->dev);

	return 0;

end4:
	pm_runtime_put(&pdev->dev);
end3:
	pm_runtime_disable(&pdev->dev);
end2:
	free_candev(ndev);
end1:
	return ret;
}

static int anlogic_axicanfd_remove(struct platform_device *pdev)
{
	struct net_device *ndev = platform_get_drvdata(pdev);

	unregister_candev(ndev);
	pm_runtime_disable(&pdev->dev);
	free_candev(ndev);

	return 0;
}

static struct platform_driver xcan_driver = {
	.probe = anlogic_axicanfd_probe,
	.remove	= anlogic_axicanfd_remove,
	.driver	= {
		.name = "anlogic,axi-canfd",
		.pm = &xcan_dev_pm_ops,
		.of_match_table	= of_match_table,
	},
};

module_platform_driver(xcan_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Anlogic Inc.");
MODULE_DESCRIPTION("Anlogic Axi-CanFD interface");
