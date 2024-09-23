// SPDX-License-Identifier: GPL-2.0+
/*
 * Anlogic AXI QSPI controller driver
 *
 * Copyright (C) 2024 Anlogic, Inc.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/wait.h>

#include <linux/spi/spi.h>
#include <linux/spi/spi-mem.h>

/**
 * Register offset
 */
#define AL_AXI_QSPI_DEV_GLB_INTR_EN_OFFSET		0x1C
#define AL_AXI_QSPI_INTR_STA_OFFSET			0x20
#define AL_AXI_QSPI_INTR_EN_OFFSET			0x28
#define AL_AXI_QSPI_SW_RST_OFFSET			0x40
#define AL_AXI_QSPI_CTL_OFFSET				0x60
#define AL_AXI_QSPI_STATUS_OFFSET			0x64
#define AL_AXI_QSPI_DTR_OFFSET				0x68
#define AL_AXI_QSPI_DRR_OFFSET				0x6C
#define AL_AXI_QSPI_SLV_SEL_OFFSET			0x70
#define AL_AXI_QSPI_TX_FIFO_OCY_OFFSET			0x74
#define AL_AXI_QSPI_RX_FIFO_OCY_OFFSET			0x78

/**
 * for AL_AXI_QSPI_CTL_OFFSET
 */
#define AL_AXI_QSPI_CTL_LOOP_SHIFT			0
#define AL_AXI_QSPI_CTL_EN_SHIFT			1
#define AL_AXI_QSPI_CTL_MASTER_SHIFT			2
#define AL_AXI_QSPI_CTL_CPOL_SHIFT			3
#define AL_AXI_QSPI_CTL_CPHA_SHIFT			4
#define AL_AXI_QSPI_CTL_TX_RST_SHIFT			5
#define AL_AXI_QSPI_CTL_RX_RST_SHIFT			6
#define AL_AXI_QSPI_CTL_MANUAL_SHIFT			7
#define AL_AXI_QSPI_CTL_MASTER_INHIBIT_SHIFT		8
#define AL_AXI_QSPI_CTL_LSB_FIRST_SHIFT			9

/**
 * for AL_AXI_QSPI_STATUS_OFFSET
 */
#define AL_AXI_QSPI_STATUS_RX_EMPTY_SHIFT             0
#define AL_AXI_QSPI_STATUS_RX_FULL_SHIFT              1
#define AL_AXI_QSPI_STATUS_TX_EMPTY_SHIFT             2
#define AL_AXI_QSPI_STATUS_TX_FULL_SHIFT              3
#define AL_AXI_QSPI_STATUS_MODF_SHIFT                 4
#define AL_AXI_QSPI_STATUS_SLAVE_SEL_SHIFT            5
#define AL_AXI_QSPI_STATUS_CPOL_CPHA_ERR_SHIFT        6
#define AL_AXI_QSPI_STATUS_SLAVE_MODE_ERR_SHIFT       7
#define AL_AXI_QSPI_STATUS_MSB_ERR_SHIFT              8
#define AL_AXI_QSPI_STATUS_LOOPBACK_ERR_SHIFT         9
#define AL_AXI_QSPI_STATUS_CMD_ERR_SHIFT             10

/**
 * for AL_AXI_QSPI_DEV_GLB_INTR_EN_OFFSET
 */
#define AL_AXI_QSPI_DEV_GLB_INTR_EN_SHIFT		31

/**
 * for AL_AXI_QSPI_INTR_EN_OFFSET
 */
#define AL_AXI_QSPI_INTR_EN_MODF_SHIFT				0
#define AL_AXI_QSPI_INTR_EN_SLAVE_MODF_SHIFT			1
#define AL_AXI_QSPI_INTR_EN_DTR_EMPTY_SHIFT			2
#define AL_AXI_QSPI_INTR_EN_DTR_UNDER_SHIFT			3
#define AL_AXI_QSPI_INTR_EN_DRR_FULL_SHIFT			4
#define AL_AXI_QSPI_INTR_EN_DRR_OVER_SHIFT			5
#define AL_AXI_QSPI_INTR_EN_TX_FIFO_HALF_SHIFT			6
#define AL_AXI_QSPI_INTR_EN_SLV_SEL_SHIFT			7
#define AL_AXI_QSPI_INTR_EN_DRR_NOT_EMPTY_SHIFT			8
#define AL_AXI_QSPI_INTR_EN_CPOL_CPHA_ERR_SHIFT			9
#define AL_AXI_QSPI_INTR_EN_SLAVE_MODE_ERR_SHIFT		10
#define AL_AXI_QSPI_INTR_EN_MSB_ERR_SHIFT			11
#define AL_AXI_QSPI_INTR_EN_LOOPBACK_ERR_SHIFT			12
#define AL_AXI_QSPI_INTR_EN_CMD_ERR_SHIFT			13

#define AL_AXI_QSPI_ERR_MASK	(BIT(AL_AXI_QSPI_INTR_EN_MODF_SHIFT) | \
				BIT(AL_AXI_QSPI_INTR_EN_SLAVE_MODF_SHIFT) | \
				BIT(AL_AXI_QSPI_INTR_EN_DTR_UNDER_SHIFT) | \
				BIT(AL_AXI_QSPI_INTR_EN_DRR_OVER_SHIFT) | \
				BIT(AL_AXI_QSPI_INTR_EN_CPOL_CPHA_ERR_SHIFT) | \
				BIT(AL_AXI_QSPI_INTR_EN_SLAVE_MODE_ERR_SHIFT) | \
				BIT(AL_AXI_QSPI_INTR_EN_MSB_ERR_SHIFT) | \
				BIT(AL_AXI_QSPI_INTR_EN_LOOPBACK_ERR_SHIFT) | \
				BIT(AL_AXI_QSPI_INTR_EN_CMD_ERR_SHIFT))

#define AL_AXI_QSPI_STANDARD		0
#define AL_AXI_QSPI_DUAL		1
#define AL_AXI_QSPI_QUAD		2

enum al_axi_trans_status {
	AL_AXI_STATUS_NONE			= 0,

	AL_AXI_MODF			= BIT(AL_AXI_QSPI_INTR_EN_MODF_SHIFT),
	AL_AXI_SLV_MODF			= BIT(AL_AXI_QSPI_INTR_EN_SLAVE_MODF_SHIFT),
	AL_AXI_DTR_UNDER		= BIT(AL_AXI_QSPI_INTR_EN_DTR_UNDER_SHIFT),
	AL_AXI_DRR_OVER			= BIT(AL_AXI_QSPI_INTR_EN_DRR_OVER_SHIFT),
	AL_AXI_CPOL_CPHA_ERR		= BIT(AL_AXI_QSPI_INTR_EN_CPOL_CPHA_ERR_SHIFT),
	AL_AXI_SLAVE_MODE_ERR		= BIT(AL_AXI_QSPI_INTR_EN_SLAVE_MODE_ERR_SHIFT),
	AL_AXI_MSB_ERR			= BIT(AL_AXI_QSPI_INTR_EN_MSB_ERR_SHIFT),
	AL_AXI_LOOPBACK_ERR		= BIT(AL_AXI_QSPI_INTR_EN_LOOPBACK_ERR_SHIFT),
	AL_AXI_CMD_ERR			= BIT(AL_AXI_QSPI_INTR_EN_CMD_ERR_SHIFT),

	AL_AXI_TRANS_DONE		= BIT(31),
};

struct al_axi_qspi_trans_info {
	u32 phase1_size;
	u32 phase1_cnt;
	u32 phase2_size;
	u32 phase2_send_cnt;
	u32 phase2_recv_cnt;

	u8 phase1_ary[64];
	u8 *phase1_buf;
	union {
		u8 *in;
		const u8 *out;
	} phase2_buf;

	u32 invaild_cnt;

	enum spi_mem_data_dir dir;
};

struct al_axi_qspi {
	void __iomem *regs;
	struct spi_master *master;

	struct {
		u32 fifo_depth;
		u32 num_cs;
		u32 trans_bits;
		u32 spi_mode;
		u32 flash_type;
		int irq;
	} hw_config;

	struct al_axi_qspi_trans_info trans_info;

	u32 is_polled;
	u32 trans_status;
	u32 is_transfer_done;
	wait_queue_head_t wait;
};

#define AL_AXI_QSPI_MIXED		0
#define AL_AXI_QSPI_WINBOND		1
#define AL_AXI_QSPI_MICRON		2
#define AL_AXI_QSPI_SPANSION		3
#define AL_AXI_QSPI_MACRONIX		4

#define FLASH_SUPPORT_SHIFT		16
#define CMD_SUPPORT_MASK		GENMASK(15, 0)
#define MIX_SPT				BIT(FLASH_SUPPORT_SHIFT + AL_AXI_QSPI_MIXED)
#define WINBOND_SPT			BIT(FLASH_SUPPORT_SHIFT + AL_AXI_QSPI_WINBOND)
#define MICRON_SPT			BIT(FLASH_SUPPORT_SHIFT + AL_AXI_QSPI_MICRON)
#define SPANSION_SPT			BIT(FLASH_SUPPORT_SHIFT + AL_AXI_QSPI_SPANSION)
#define MACRONIX_SPT			BIT(FLASH_SUPPORT_SHIFT + AL_AXI_QSPI_MACRONIX)
static u32 al_axi_cmd_support[] = {
	0x9E | WINBOND_SPT | SPANSION_SPT | MICRON_SPT | MACRONIX_SPT,
	0x9F | WINBOND_SPT | SPANSION_SPT | MICRON_SPT | MACRONIX_SPT,
	0x0B | WINBOND_SPT | SPANSION_SPT | MICRON_SPT | MACRONIX_SPT,
	0x3B | WINBOND_SPT | SPANSION_SPT | MICRON_SPT | MACRONIX_SPT,
	0xBB | WINBOND_SPT | SPANSION_SPT | MICRON_SPT | MACRONIX_SPT,
	0x6B | WINBOND_SPT | SPANSION_SPT | MICRON_SPT | MACRONIX_SPT,
	0xEB | WINBOND_SPT | SPANSION_SPT | MICRON_SPT | MACRONIX_SPT,
	0x06 | WINBOND_SPT | SPANSION_SPT | MICRON_SPT | MACRONIX_SPT,
	0x04 | WINBOND_SPT | SPANSION_SPT | MICRON_SPT | MACRONIX_SPT,
	0x02 | WINBOND_SPT | SPANSION_SPT | MICRON_SPT | MACRONIX_SPT,
	0x32 | WINBOND_SPT | SPANSION_SPT | MICRON_SPT,
	0xD8 | WINBOND_SPT | SPANSION_SPT | MICRON_SPT | MACRONIX_SPT,
	0x01 | WINBOND_SPT | SPANSION_SPT | MICRON_SPT | MACRONIX_SPT,
	0x03 | WINBOND_SPT | SPANSION_SPT | MICRON_SPT | MACRONIX_SPT,
	0x05 | WINBOND_SPT | SPANSION_SPT | MICRON_SPT | MACRONIX_SPT,
	0x7A | WINBOND_SPT | SPANSION_SPT | MICRON_SPT,
	0x75 | WINBOND_SPT | SPANSION_SPT | MICRON_SPT,
	0xC7 | WINBOND_SPT | SPANSION_SPT | MICRON_SPT | MACRONIX_SPT,
	0x4B | WINBOND_SPT | SPANSION_SPT | MICRON_SPT,
	0x13               | SPANSION_SPT | MICRON_SPT | MACRONIX_SPT,
	0x0C               | SPANSION_SPT | MICRON_SPT | MACRONIX_SPT,
	0x5A               | SPANSION_SPT | MICRON_SPT | MACRONIX_SPT,
	0x3C               | SPANSION_SPT | MICRON_SPT | MACRONIX_SPT,
	0xBC               | SPANSION_SPT | MICRON_SPT | MACRONIX_SPT,
	0x6C               | SPANSION_SPT | MICRON_SPT | MACRONIX_SPT,
	0xEC               | SPANSION_SPT | MICRON_SPT | MACRONIX_SPT,
	0x12               | SPANSION_SPT | MICRON_SPT | MACRONIX_SPT,
	0x42               | SPANSION_SPT | MICRON_SPT | MACRONIX_SPT,
	0xE8               | SPANSION_SPT | MICRON_SPT | MACRONIX_SPT,
	0x85               | SPANSION_SPT | MICRON_SPT | MACRONIX_SPT,
	0xE9               | SPANSION_SPT | MICRON_SPT | MACRONIX_SPT,
	0x52 | WINBOND_SPT                | MICRON_SPT | MACRONIX_SPT,
	0xA2                              | MICRON_SPT,
	0xD2                              | MICRON_SPT,
	0xE5                              | MICRON_SPT,
	0x70                              | MICRON_SPT,
	0x50                              | MICRON_SPT,
	0xB5                              | MICRON_SPT,
	0xB1                              | MICRON_SPT | MACRONIX_SPT,
	0x81                              | MICRON_SPT,
	0x65                              | MICRON_SPT,
	0x61                              | MICRON_SPT,
	0xB7                              | MICRON_SPT | MACRONIX_SPT,
	0xC5                              | MICRON_SPT | MACRONIX_SPT,
	0xC8                              | MICRON_SPT | MACRONIX_SPT,
	0x66                              | MICRON_SPT | MACRONIX_SPT,
	0x99                              | MICRON_SPT | MACRONIX_SPT,
	0x20 | WINBOND_SPT                | MICRON_SPT | MACRONIX_SPT,
	0x35 | WINBOND_SPT | SPANSION_SPT | MICRON_SPT | MACRONIX_SPT,
	0xB9 | WINBOND_SPT | SPANSION_SPT | MICRON_SPT | MACRONIX_SPT,
	0xA3 | WINBOND_SPT,
	0xFF               | SPANSION_SPT,
	0xAB | WINBOND_SPT | SPANSION_SPT | MICRON_SPT | MACRONIX_SPT,
	0x90 | WINBOND_SPT | SPANSION_SPT              | MACRONIX_SPT,
	0x30               | SPANSION_SPT              | MACRONIX_SPT,
	0x15                                           | MACRONIX_SPT,
	0x16               | SPANSION_SPT              | MACRONIX_SPT,
	0x17               | SPANSION_SPT              | MACRONIX_SPT,
	0x38               | SPANSION_SPT | MICRON_SPT | MACRONIX_SPT,
	0x34               | SPANSION_SPT | MICRON_SPT,
	0x60 | WINBOND_SPT | SPANSION_SPT              | MACRONIX_SPT,
	0xE0               | SPANSION_SPT | MICRON_SPT | MACRONIX_SPT,
	0xE1               | SPANSION_SPT | MICRON_SPT | MACRONIX_SPT,
	0xE2               | SPANSION_SPT | MICRON_SPT | MACRONIX_SPT,
	0xE3 | WINBOND_SPT | SPANSION_SPT | MICRON_SPT | MACRONIX_SPT,
	0xE4               | SPANSION_SPT | MICRON_SPT | MACRONIX_SPT,
	0x2B               | SPANSION_SPT              | MACRONIX_SPT,
	0x2F               | SPANSION_SPT              | MACRONIX_SPT,
	0xA7               | SPANSION_SPT | MICRON_SPT,
	0xA6               | SPANSION_SPT | MICRON_SPT,
	0xE7               | SPANSION_SPT,
	0xF0               | SPANSION_SPT,
	0x8A               | SPANSION_SPT,
	0x07               | SPANSION_SPT,
	0xDC               | SPANSION_SPT | MICRON_SPT | MACRONIX_SPT,
	0x27                              | MICRON_SPT | MACRONIX_SPT,
	0x28                              | MICRON_SPT | MACRONIX_SPT,
	0x29                              | MICRON_SPT | MACRONIX_SPT,
	0x3E                              | MICRON_SPT | MACRONIX_SPT,
	0x21                              | MICRON_SPT | MACRONIX_SPT,
	0x5C                              | MICRON_SPT | MACRONIX_SPT,
	0xC0                                           | MACRONIX_SPT,
	0x18                                           | MACRONIX_SPT,
	0xF5                              | MICRON_SPT | MACRONIX_SPT,
	0x2D                              | MICRON_SPT | MACRONIX_SPT,
	0x2C                              | MICRON_SPT | MACRONIX_SPT,
	0xAF                              | MICRON_SPT | MACRONIX_SPT,
	0xB0                                           | MACRONIX_SPT,
	0xC4                              | MICRON_SPT,
	0x68                                           | MACRONIX_SPT,
	0xC1                                           | MACRONIX_SPT,
	0x7E                                           | MACRONIX_SPT,
	0x98                                           | MACRONIX_SPT,
	0x00                                           | MACRONIX_SPT,
	0, /* end */
};

static const struct of_device_id al_axi_qspi_match[] = {
	{ .compatible = "anlogic-axi-qspi-controller", },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, al_axi_qspi_match);

/**
 * @return 0: rx fifo is not empty, else empty
 */
static inline u32 al_axi_qspi_check_rx_empty(struct al_axi_qspi *axi_qspi)
{
	return readl_relaxed(axi_qspi->regs + AL_AXI_QSPI_STATUS_OFFSET) &
		(1 << AL_AXI_QSPI_STATUS_RX_EMPTY_SHIFT);
}

/**
 * @return 0: rx fifo is not empty, else empty
 */
static inline u32 al_axi_qspi_check_tx_empty(struct al_axi_qspi *axi_qspi)
{
	return readl_relaxed(axi_qspi->regs + AL_AXI_QSPI_STATUS_OFFSET) &
		(1 << AL_AXI_QSPI_STATUS_TX_EMPTY_SHIFT);
}

static inline u32 al_axi_pre_send_data(const u8 *buf, u32 index, u32 bytes_step)
{
	u32 data = 0;

	if (bytes_step == 1)
		data = (u32)(*(u8 *)(buf + index));
	else if (bytes_step == 2)
		data = (u32)(*(u16 *)(buf + index));
	else
		data = *(u32 *)(buf + index);

	return data;
}

/**
 * @return 0: no error, -1: error
 */
static inline int al_axi_qspi_check_err(struct al_axi_qspi *axi_qspi, u32 intr_sta)
{
	u32 val = intr_sta & AL_AXI_QSPI_ERR_MASK;

	if (val) {

		if ((val) == BIT(AL_AXI_QSPI_INTR_EN_DRR_OVER_SHIFT))
			return 0;

		axi_qspi->trans_status |= (val);
		return -1;
	}

	return 0;
}

static inline void al_axi_save_recv_data(u8 *buf, u32 index, u32 bytes_step, u32 data)
{
	if (bytes_step == 1)
		*(u8 *)(buf + index) = (u8)data;
	else if (bytes_step == 2)
		*(u16 *)(buf + index) = (u16)data;
	else
		*(u32 *)(buf + index) = data;
}

static irqreturn_t al_axi_qspi_irq(int irq, void *dev_id)
{
	struct al_axi_qspi *axi_qspi = (struct al_axi_qspi *)dev_id;
	u32 intr_sta = 0;
	u32 send_data = 0;
	u32 recv_data = 0;
	u32 bytes_step = 0;
	u32 fill_cnt = 0;
	u8 *phase1_ptr = axi_qspi->trans_info.phase1_buf;
	u8 *in_buf = axi_qspi->trans_info.phase2_buf.in;
	const u8 *out_buf = axi_qspi->trans_info.phase2_buf.out;

	bytes_step = axi_qspi->hw_config.trans_bits >> 3;

	intr_sta = readl_relaxed(axi_qspi->regs + AL_AXI_QSPI_INTR_STA_OFFSET) &
		readl_relaxed(axi_qspi->regs + AL_AXI_QSPI_INTR_EN_OFFSET);
	writel_relaxed(intr_sta, axi_qspi->regs + AL_AXI_QSPI_INTR_STA_OFFSET);

	if (!intr_sta)
		return IRQ_HANDLED;

	if (axi_qspi->trans_info.dir == SPI_MEM_DATA_IN) {
		while (!al_axi_qspi_check_rx_empty(axi_qspi)) {
			recv_data = readl_relaxed(axi_qspi->regs + AL_AXI_QSPI_DRR_OFFSET);
			if (axi_qspi->trans_info.invaild_cnt < axi_qspi->trans_info.phase1_size)
				axi_qspi->trans_info.invaild_cnt++;
			else {
				al_axi_save_recv_data(in_buf, axi_qspi->trans_info.phase2_recv_cnt,
					bytes_step, recv_data);

				axi_qspi->trans_info.phase2_recv_cnt += bytes_step;
			}
		}
	}

	if ((axi_qspi->trans_info.phase1_cnt == axi_qspi->trans_info.phase1_size) &&
		(axi_qspi->trans_info.phase2_send_cnt == axi_qspi->trans_info.phase2_size) &&
		(intr_sta & (1 << AL_AXI_QSPI_INTR_EN_DTR_EMPTY_SHIFT))) {

		axi_qspi->is_transfer_done = 1;
		axi_qspi->trans_status |= AL_AXI_TRANS_DONE;
		wake_up_interruptible(&axi_qspi->wait);
		return IRQ_HANDLED;
	}

	if (al_axi_qspi_check_err(axi_qspi, intr_sta)) {
		axi_qspi->is_transfer_done = 1;
		wake_up_interruptible(&axi_qspi->wait);
		return IRQ_HANDLED;
	}

	if (axi_qspi->hw_config.fifo_depth == 0)
		fill_cnt = 1;
	else
		fill_cnt = axi_qspi->hw_config.fifo_depth -
			readl_relaxed(axi_qspi->regs + AL_AXI_QSPI_TX_FIFO_OCY_OFFSET) - 1;

	while (fill_cnt--) {
		if ((axi_qspi->trans_info.phase1_cnt == axi_qspi->trans_info.phase1_size) &&
			(axi_qspi->trans_info.phase2_send_cnt == axi_qspi->trans_info.phase2_size))
			break;

		if (axi_qspi->trans_info.phase1_cnt < axi_qspi->trans_info.phase1_size) {
			send_data = al_axi_pre_send_data(phase1_ptr,
				axi_qspi->trans_info.phase1_cnt, bytes_step);
			axi_qspi->trans_info.phase1_cnt += bytes_step;

		} else if (axi_qspi->trans_info.phase2_send_cnt <
			 axi_qspi->trans_info.phase2_size) {
			if (axi_qspi->trans_info.dir == SPI_MEM_DATA_OUT) {
				send_data = al_axi_pre_send_data(out_buf,
					axi_qspi->trans_info.phase2_send_cnt, bytes_step);
			} else {
				send_data = 0;
			}
			axi_qspi->trans_info.phase2_send_cnt += bytes_step;
		}

		writel_relaxed(send_data, axi_qspi->regs + AL_AXI_QSPI_DTR_OFFSET);
	}

	return IRQ_HANDLED;
}

static inline void al_axi_qspi_sw_rst(struct al_axi_qspi *axi_qspi)
{
	writel_relaxed(0xa, axi_qspi->regs + AL_AXI_QSPI_SW_RST_OFFSET);
}

static inline void al_axi_qspi_config_controller(struct al_axi_qspi *axi_qspi,
						struct spi_device *dev)
{
	u32 reg_val = 0;

	reg_val = (1 << AL_AXI_QSPI_CTL_MASTER_SHIFT) |
		(1 << AL_AXI_QSPI_CTL_MANUAL_SHIFT) |
		(1 << AL_AXI_QSPI_CTL_MASTER_INHIBIT_SHIFT);
	if (dev->mode & SPI_CPOL)
		reg_val |= (1 << AL_AXI_QSPI_CTL_CPOL_SHIFT);
	if (dev->mode & SPI_CPHA)
		reg_val |= (1 << AL_AXI_QSPI_CTL_CPHA_SHIFT);
	if (dev->mode & SPI_LSB_FIRST)
		reg_val |= (1 << AL_AXI_QSPI_CTL_LSB_FIRST_SHIFT);
	writel_relaxed(reg_val, axi_qspi->regs + AL_AXI_QSPI_CTL_OFFSET);

	/* disable all interrupts, clear all interrupt status */
	writel_relaxed(0, axi_qspi->regs + AL_AXI_QSPI_INTR_EN_OFFSET);
	reg_val = readl_relaxed(axi_qspi->regs + AL_AXI_QSPI_INTR_STA_OFFSET);
	writel_relaxed(reg_val, axi_qspi->regs + AL_AXI_QSPI_INTR_STA_OFFSET);

	writel_relaxed(1 << AL_AXI_QSPI_DEV_GLB_INTR_EN_SHIFT,
		axi_qspi->regs + AL_AXI_QSPI_DEV_GLB_INTR_EN_OFFSET);
	writel_relaxed(~(1 << dev->chip_select),
		axi_qspi->regs + AL_AXI_QSPI_SLV_SEL_OFFSET);

	/* reset tx/rx fifo */
	reg_val = readl_relaxed(axi_qspi->regs + AL_AXI_QSPI_CTL_OFFSET);
	reg_val |= (1 << AL_AXI_QSPI_CTL_TX_RST_SHIFT) |
		(1 << AL_AXI_QSPI_CTL_RX_RST_SHIFT);
	writel_relaxed(reg_val, axi_qspi->regs + AL_AXI_QSPI_CTL_OFFSET);

	reg_val = readl_relaxed(axi_qspi->regs + AL_AXI_QSPI_CTL_OFFSET);
	reg_val |= (1 << AL_AXI_QSPI_CTL_EN_SHIFT);
	writel_relaxed(reg_val, axi_qspi->regs + AL_AXI_QSPI_CTL_OFFSET);
}

/**
 * @return 0: not valid, else valid
 */
static u32 al_axi_check_cmd_valid(struct al_axi_qspi *axi_qspi, u32 cmd)
{
	u32 idx = 0;
	u32 ary_size = ARRAY_SIZE(al_axi_cmd_support);

	for (idx = 0; idx < ary_size; idx++) {
		if (cmd != (al_axi_cmd_support[idx] & CMD_SUPPORT_MASK))
			continue;

		if (BIT(FLASH_SUPPORT_SHIFT + axi_qspi->hw_config.flash_type) &
			al_axi_cmd_support[idx])
			return 1;

		break;
	}

	return 0;
}

static int al_axi_qspi_transfer_intr(struct al_axi_qspi *axi_qspi, struct spi_device *dev)
{
	u32 reg_val = 0;
	u32 fill_cnt = 0;
	u32 bytes_step = 0;
	u32 send_data = 0;
	int ret = 0;

	al_axi_qspi_config_controller(axi_qspi, dev);

	bytes_step = axi_qspi->hw_config.trans_bits >> 3;
	if (axi_qspi->hw_config.fifo_depth == 0)
		fill_cnt = 1;
	else
		fill_cnt = axi_qspi->hw_config.fifo_depth;

	while (fill_cnt--) {
		if (axi_qspi->trans_info.phase1_cnt == axi_qspi->trans_info.phase1_size &&
			axi_qspi->trans_info.phase2_send_cnt == axi_qspi->trans_info.phase2_size)
			break;

		if (axi_qspi->trans_info.phase1_cnt < axi_qspi->trans_info.phase1_size) {
			send_data = al_axi_pre_send_data(axi_qspi->trans_info.phase1_buf,
				axi_qspi->trans_info.phase1_cnt, bytes_step);
			axi_qspi->trans_info.phase1_cnt += bytes_step;

		} else if (axi_qspi->trans_info.phase2_send_cnt <
			axi_qspi->trans_info.phase2_size) {
			if (axi_qspi->trans_info.dir ==	SPI_MEM_DATA_OUT) {
				send_data =
					al_axi_pre_send_data(axi_qspi->trans_info.phase2_buf.out,
					axi_qspi->trans_info.phase2_send_cnt, bytes_step);
			} else {
				send_data = 0;
			}

			axi_qspi->trans_info.phase2_send_cnt += bytes_step;
		}

		writel_relaxed(send_data, axi_qspi->regs + AL_AXI_QSPI_DTR_OFFSET);
	}

	reg_val = (1 << AL_AXI_QSPI_INTR_EN_DTR_EMPTY_SHIFT) |
		(1 << AL_AXI_QSPI_INTR_EN_TX_FIFO_HALF_SHIFT) |
		(1 << AL_AXI_QSPI_INTR_EN_DRR_OVER_SHIFT) |
		(1 << AL_AXI_QSPI_INTR_EN_MODF_SHIFT);

	if (axi_qspi->hw_config.spi_mode != AL_AXI_QSPI_STANDARD) {
		reg_val |= ((1 << AL_AXI_QSPI_INTR_EN_CPOL_CPHA_ERR_SHIFT) |
			(1 << AL_AXI_QSPI_INTR_EN_SLAVE_MODE_ERR_SHIFT) |
			(1 << AL_AXI_QSPI_INTR_EN_MSB_ERR_SHIFT) |
			(1 << AL_AXI_QSPI_INTR_EN_CMD_ERR_SHIFT));
	}

	writel_relaxed(reg_val, axi_qspi->regs + AL_AXI_QSPI_INTR_EN_OFFSET);

	reg_val = readl_relaxed(axi_qspi->regs + AL_AXI_QSPI_CTL_OFFSET);
	reg_val &= ~(1 << AL_AXI_QSPI_CTL_MASTER_INHIBIT_SHIFT);
	writel_relaxed(reg_val, axi_qspi->regs + AL_AXI_QSPI_CTL_OFFSET);

	axi_qspi->is_transfer_done = 0;
	ret = wait_event_interruptible_timeout(axi_qspi->wait,
		axi_qspi->is_transfer_done, 10 * HZ);
	if (!ret) {
		dev_err(&axi_qspi->master->dev,
			"interrupt timeout, current command: 0x%x\n",
			axi_qspi->trans_info.phase1_ary[0]);
		goto out;
	}

	if (axi_qspi->trans_status & AL_AXI_QSPI_ERR_MASK) {
		dev_err(&axi_qspi->master->dev, "transfer error, status: 0x%x\n",
			axi_qspi->trans_status);
		goto out;
	}

	writel_relaxed(0, axi_qspi->regs + AL_AXI_QSPI_DEV_GLB_INTR_EN_OFFSET);
	writel_relaxed(0, axi_qspi->regs + AL_AXI_QSPI_INTR_EN_OFFSET);
	writel_relaxed(0xFFFFFFFF, axi_qspi->regs + AL_AXI_QSPI_SLV_SEL_OFFSET);

	reg_val = readl_relaxed(axi_qspi->regs + AL_AXI_QSPI_CTL_OFFSET);
	reg_val &= ~(1 << AL_AXI_QSPI_CTL_EN_SHIFT);
	reg_val |= (1 << AL_AXI_QSPI_CTL_MASTER_INHIBIT_SHIFT);
	writel_relaxed(reg_val, axi_qspi->regs + AL_AXI_QSPI_CTL_OFFSET);

	return 0;

out:
	al_axi_qspi_sw_rst(axi_qspi);
	return -1;
}

static int al_axi_qspi_transfer_polled(struct al_axi_qspi *axi_qspi, struct spi_device *dev)
{
	u32 data = 0;
	u32 reg_val = 0;
	u32 fill_cnt = 0;
	u32 to_enable_trans = 1;
	u32 bytes_step = axi_qspi->hw_config.trans_bits >> 3;
	u8 *recv_buf = axi_qspi->trans_info.phase2_buf.in;
	const u8 *send_buf = axi_qspi->trans_info.phase2_buf.out;
	struct al_axi_qspi_trans_info *trans_info = &axi_qspi->trans_info;

	al_axi_qspi_config_controller(axi_qspi, dev);
	writel_relaxed(0, axi_qspi->regs + AL_AXI_QSPI_DEV_GLB_INTR_EN_OFFSET);

	while (1) {
		while (!al_axi_qspi_check_rx_empty(axi_qspi)) {
			data = readl_relaxed(axi_qspi->regs + AL_AXI_QSPI_DRR_OFFSET);
			if (trans_info->invaild_cnt < trans_info->phase1_size)
				trans_info->invaild_cnt++;
			else {
				al_axi_save_recv_data(recv_buf,
					trans_info->phase2_recv_cnt, bytes_step, data);
				trans_info->phase2_recv_cnt += bytes_step;
			}
		}

		if (trans_info->phase1_cnt == trans_info->phase1_size &&
			trans_info->phase2_send_cnt == trans_info->phase2_size &&
			trans_info->phase2_recv_cnt == trans_info->phase2_size) {
			axi_qspi->is_transfer_done = 1;
			break;
		}

		if (axi_qspi->hw_config.fifo_depth == 0)
			fill_cnt = 1;
		else
			fill_cnt = axi_qspi->hw_config.fifo_depth - 1 -
				readl_relaxed(axi_qspi->regs + AL_AXI_QSPI_TX_FIFO_OCY_OFFSET);

		while (fill_cnt--) {
			if ((trans_info->phase1_cnt == trans_info->phase1_size) &&
				(trans_info->phase2_send_cnt == trans_info->phase2_size))
				break;

			if (trans_info->phase1_cnt < trans_info->phase1_size) {
				data = al_axi_pre_send_data(trans_info->phase1_buf,
					trans_info->phase1_cnt, bytes_step);
				trans_info->phase1_cnt += bytes_step;

			} else if (trans_info->phase2_send_cnt <
				trans_info->phase2_size) {
				if (trans_info->dir == SPI_MEM_DATA_OUT) {
					data = al_axi_pre_send_data(send_buf,
						trans_info->phase2_send_cnt, bytes_step);
				} else {
					data = 0;
				}
				trans_info->phase2_send_cnt += bytes_step;
			}

			writel_relaxed(data, axi_qspi->regs + AL_AXI_QSPI_DTR_OFFSET);
		}

		if (readl_relaxed(axi_qspi->regs + AL_AXI_QSPI_STATUS_OFFSET) &
			(1 << AL_AXI_QSPI_STATUS_CMD_ERR_SHIFT)) {
			dev_err(&axi_qspi->master->dev, "command error\n");
			al_axi_qspi_sw_rst(axi_qspi);
			return -1;
		}

		if (to_enable_trans) {
			reg_val = readl_relaxed(axi_qspi->regs + AL_AXI_QSPI_CTL_OFFSET);
			reg_val &= ~(1 << AL_AXI_QSPI_CTL_MASTER_INHIBIT_SHIFT);
			writel_relaxed(reg_val, axi_qspi->regs + AL_AXI_QSPI_CTL_OFFSET);
			to_enable_trans = 0;
		}
	}

	/* Wait for tx finish */
	while (!al_axi_qspi_check_tx_empty(axi_qspi))
		;

	writel_relaxed(0, axi_qspi->regs + AL_AXI_QSPI_INTR_EN_OFFSET);
	writel_relaxed(0xFFFFFFFF, axi_qspi->regs + AL_AXI_QSPI_SLV_SEL_OFFSET);

	reg_val = readl_relaxed(axi_qspi->regs + AL_AXI_QSPI_CTL_OFFSET);
	reg_val &= ~(1 << AL_AXI_QSPI_CTL_EN_SHIFT);
	reg_val |= (1 << AL_AXI_QSPI_CTL_MASTER_INHIBIT_SHIFT);
	writel_relaxed(reg_val, axi_qspi->regs + AL_AXI_QSPI_CTL_OFFSET);

	return 0;
}


static bool al_axi_qspi_supports_op(struct spi_mem *mem, const struct spi_mem_op *op)
{
	struct al_axi_qspi *axi_qspi = spi_master_get_devdata(mem->spi->master);

	if (axi_qspi->hw_config.spi_mode == AL_AXI_QSPI_STANDARD) {
		if (op->cmd.buswidth > 1 || op->addr.buswidth > 1 ||
			op->dummy.buswidth > 1 || op->data.buswidth > 1)
			return false;
	} else if (axi_qspi->hw_config.spi_mode == AL_AXI_QSPI_DUAL) {
		if (op->cmd.buswidth > 2 || op->addr.buswidth > 2 ||
			op->dummy.buswidth > 2 || op->data.buswidth > 2)
			return false;
	} else if (axi_qspi->hw_config.spi_mode == AL_AXI_QSPI_QUAD) {
		if (op->cmd.buswidth > 4 || op->addr.buswidth > 4 ||
			op->dummy.buswidth > 4 || op->data.buswidth > 4)
			return false;
	}


	if (!al_axi_check_cmd_valid(axi_qspi, (u32)op->cmd.opcode)) {
		dev_notice(&mem->spi->dev, "found unsupported command: 0x%2x\n",
			op->cmd.opcode);
		return false;
	}

	return true;
}

static int al_axi_qspi_exec_op(struct spi_mem *mem, const struct spi_mem_op *op)
{
	u32 idx = 0;
	u8 *buf_ptr = NULL;
	u32 ret = 0;
	u32 dummy_bytes = 0;

	struct spi_device *dev = mem->spi;
	struct al_axi_qspi *axi_qspi = spi_master_get_devdata(dev->master);

	if (op->dummy.nbytes && op->data.nbytes)
		dummy_bytes = op->dummy.nbytes * op->data.buswidth / op->dummy.buswidth;

	memset(&axi_qspi->trans_info, 0, sizeof(axi_qspi->trans_info));

	axi_qspi->trans_info.phase1_size = op->cmd.nbytes + op->addr.nbytes + dummy_bytes;
	axi_qspi->trans_info.phase1_buf = &axi_qspi->trans_info.phase1_ary[0];
	buf_ptr = axi_qspi->trans_info.phase1_buf;

	for (idx = 0; idx < op->cmd.nbytes; idx++)
		*(buf_ptr + idx) =
			(op->cmd.opcode >> (8 * (op->cmd.nbytes - 1 - idx))) & 0xFF;

	for (idx = 0; idx < op->addr.nbytes; idx++)
		*(buf_ptr + op->cmd.nbytes + idx) =
			(op->addr.val >> (8 * (op->addr.nbytes - 1 - idx))) & 0xFF;

	axi_qspi->trans_info.phase2_size = op->data.nbytes;
	if (op->data.dir == SPI_MEM_DATA_OUT)
		axi_qspi->trans_info.phase2_buf.out = op->data.buf.out;
	else
		axi_qspi->trans_info.phase2_buf.in = op->data.buf.in;

	axi_qspi->trans_info.dir = op->data.dir;

	if (axi_qspi->is_polled)
		ret = al_axi_qspi_transfer_polled(axi_qspi, dev);
	else
		ret = al_axi_qspi_transfer_intr(axi_qspi, dev);

	return ret;
}

static const struct spi_controller_mem_ops al_axi_qspi_mem_ops = {
	.supports_op = al_axi_qspi_supports_op,
	.exec_op = al_axi_qspi_exec_op,
};

static int al_axi_qspi_probe(struct platform_device *pdev)
{
	struct al_axi_qspi *axi_qspi;
	struct spi_master *master;
	struct resource *res;
	int ret = 0;

	master = spi_alloc_master(&pdev->dev, sizeof(struct al_axi_qspi));
	if (!master)
		return -ENOMEM;

	axi_qspi = spi_controller_get_devdata(master);
	axi_qspi->master = master;

	axi_qspi->regs = devm_platform_get_and_ioremap_resource(pdev, 0, &res);
	if (IS_ERR(axi_qspi->regs))
		return PTR_ERR(axi_qspi->regs);

	init_waitqueue_head(&axi_qspi->wait);

	ret = platform_get_irq(pdev, 0);
	if (ret < 0)
		goto exit;

	axi_qspi->hw_config.irq = ret;
	ret = devm_request_irq(&pdev->dev, axi_qspi->hw_config.irq, al_axi_qspi_irq,
		0, dev_name(&pdev->dev), axi_qspi);
	if (ret) {
		dev_err(&pdev->dev, "request irq failed, ret: %d\n", ret);
		goto exit;
	}

	ret = 0;
	ret |= of_property_read_u32(pdev->dev.of_node, "al-fifo-depth",
		&axi_qspi->hw_config.fifo_depth);
	ret |= of_property_read_u32(pdev->dev.of_node, "al-num-ss-bits",
		&axi_qspi->hw_config.num_cs);
	ret |= of_property_read_u32(pdev->dev.of_node, "al-num-trans-bits",
		&axi_qspi->hw_config.trans_bits);
	ret |= of_property_read_u32(pdev->dev.of_node, "al-spi-mode",
		&axi_qspi->hw_config.spi_mode);
	ret |= of_property_read_u32(pdev->dev.of_node, "al-spi-memory",
		&axi_qspi->hw_config.flash_type);
	if (ret)
		goto exit;

	axi_qspi->is_polled = 0;

	master->mode_bits = SPI_CPHA | SPI_CPOL | SPI_RX_DUAL | SPI_RX_QUAD |
		SPI_TX_DUAL | SPI_TX_QUAD;
	master->bits_per_word_mask = SPI_BPW_MASK(axi_qspi->hw_config.trans_bits);
	platform_set_drvdata(pdev, master);

	al_axi_qspi_sw_rst(axi_qspi);

	master->auto_runtime_pm = true;
	master->dev.of_node = pdev->dev.of_node;
	master->dev.fwnode = pdev->dev.fwnode;
	master->num_chipselect = axi_qspi->hw_config.num_cs;
	master->mem_ops = &al_axi_qspi_mem_ops;
	ret = spi_register_controller(master);
	if (ret) {
		dev_err(&pdev->dev, "register SPI controller failed: %d\n", ret);
		goto exit;
	}

	return 0;

exit:
	spi_controller_put(master);
	return ret;
}

static int al_axi_qspi_remove(struct platform_device *pdev)
{
	struct spi_master *master = platform_get_drvdata(pdev);

	spi_unregister_controller(master);
	return 0;
}

static struct platform_driver al_axi_qspi_driver = {
	.probe = al_axi_qspi_probe,
	.remove = al_axi_qspi_remove,
	.driver = {
		.name = "anlogic-axi-qspi",
		.of_match_table = al_axi_qspi_match,
	},
};

module_platform_driver(al_axi_qspi_driver);

MODULE_AUTHOR("Cao Likun <likun.cao@anlogic.com>");
MODULE_DESCRIPTION("Driver for Anlogic Axi-Qspi controller");
MODULE_LICENSE("GPL");
