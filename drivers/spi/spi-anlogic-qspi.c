// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2023 Anlogic, Inc.
 *
 * Author: HuangJian <jian.huang@anlogic.com>
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of_irq.h>
#include <linux/of_address.h>
#include <linux/platform_device.h>
#include <linux/spi/spi.h>
#include <linux/workqueue.h>
#include <linux/spi/spi-mem.h>
#include <linux/reset.h>
#include "spi-anlogic-qspi.h"

struct al_qspi_mode {
	u8 cmd_buswidth;
	u8 addr_buswidth;
	u8 data_buswidth;
	u32 config_trans_type;
	u32 config_frame_format;
};

static const struct al_qspi_mode al_qspi_modes[] = {
	{ 1, 1, 1, QSPI_TT0, SPI_STANDARD_FORMAT},
	{ 1, 1, 2, QSPI_TT0, SPI_DUAL_FORMAT},
	{ 1, 1, 4, QSPI_TT0, SPI_QUAD_FORMAT},
	{ 1, 2, 2, QSPI_TT1, SPI_DUAL_FORMAT},
	{ 1, 4, 4, QSPI_TT1, SPI_QUAD_FORMAT},
	{ 2, 2, 2, QSPI_TT2, SPI_DUAL_FORMAT},
	{ 4, 4, 4, QSPI_TT2, SPI_QUAD_FORMAT},
};


static inline bool al_qspi_is_compatible(const struct spi_mem_op *op,
						const struct al_qspi_mode *mode)
{
	if (op->cmd.buswidth != mode->cmd_buswidth)
		return false;

	if (op->addr.nbytes && op->addr.buswidth != mode->addr_buswidth)
		return false;

	if (op->data.nbytes && op->data.buswidth != mode->data_buswidth)
		return false;

	return true;
}

static int al_qspi_find_mode(const struct spi_mem_op *op)
{
	u32 i;

	for (i = 0; i < ARRAY_SIZE(al_qspi_modes); i++)
		if (al_qspi_is_compatible(op, &al_qspi_modes[i]))
			return i;
	return 0;
}



static irqreturn_t al_qspi_irq(int irq, void *dev_id)
{
	struct spi_controller *master = dev_id;
	struct al_qspi *qspi = spi_controller_get_devdata(master);
	u16 irq_status = al_readl(qspi, AL_QSPI_ISR_OFFSET) & 0x3f;

	if (!irq_status)
		return IRQ_NONE;

	if (!master->cur_msg) {
		qspi_mask_intr(qspi, 0xff);
		return IRQ_HANDLED;
	}

	return qspi->transfer_handler(qspi);
}

void al_qspi_set_cs(struct spi_device *spi, bool enable)
{
	struct al_qspi *qspi = spi_controller_get_devdata(spi->controller);
	bool cs_high = !!(spi->mode & SPI_CS_HIGH);

	/*
	 * AL SPI controller demands any native CS being set in order to
	 * proceed with data transfer. So in order to activate the SPI
	 * communications we must set a corresponding bit in the Slave
	 * Enable register no matter whether the SPI core is configured to
	 * support active-high or active-low CS level.
	 */
	if (cs_high == enable)
		al_writel(qspi, AL_QSPI_SER_OFFSET, BIT(spi->chip_select));
	else
		al_writel(qspi, AL_QSPI_SER_OFFSET, 0);
}

static int al_qspi_write_then_read(struct al_qspi *qspi, struct spi_device *spi, const struct spi_mem_op *op)
{
	u32 room = 0, entries, sts;
	unsigned int len = 0;
	u8 *buf;
	int dummy_temp;

	qspi->n_bytes = qspi->info.dfs / 8;
	dummy_temp = op->dummy.nbytes;

	/*
	 * At initial stage we just pre-fill the Tx FIFO in with no rush,
	 * since native CS hasn't been enabled yet and the automatic data
	 * transmission won't start til we do that.
	 */
	if (qspi->info.spi_frame_format == SPI_STANDARD_FORMAT) {
		switch (qspi->info.dfs - 1) {
		case QSPI_DFS_32BITS:
			al_writel(qspi, AL_QSPI_DR0_OFFSET, (op->cmd.opcode & 0xff) | ((op->addr.val & 0xff) << 24)
												| (((op->addr.val >> 8) & 0xff) << 16) | (((op->addr.val >> 16) & 0xff) << 8));
			while(dummy_temp) {
				al_writel(qspi, AL_QSPI_DR0_OFFSET, 0x0);
				dummy_temp -= 4;
			}
			break;
		case QSPI_DFS_16BITS:
			al_writel(qspi, AL_QSPI_DR0_OFFSET, (((op->addr.val >> 24) & 0xff) << 8) | (op->cmd.opcode & 0xff));
			al_writel(qspi, AL_QSPI_DR0_OFFSET, ((op->addr.val & 0xff) << 8) | ((op->addr.val >> 8) & 0xff));
			while(dummy_temp) {
				al_writel(qspi, AL_QSPI_DR0_OFFSET, 0x0);
				dummy_temp -= 2;
			}
			break;
		case QSPI_DFS_8BITS:
			al_writel(qspi, AL_QSPI_DR0_OFFSET, (op->cmd.opcode & 0xff));
			while (op->addr.nbytes > room) {
				al_writel(qspi, AL_QSPI_DR0_OFFSET, (op->addr.val >> ((op->addr.nbytes - room - 1) * 8)) & 0xff);
				room++;
			}
			while(dummy_temp) {
				al_writel(qspi, AL_QSPI_DR0_OFFSET, 0x0);
				dummy_temp--;
			}
			break;
		default:
			break;
		}
	} else {
		switch (qspi->info.dfs - 1) {
		case QSPI_DFS_32BITS:
			al_writel(qspi, AL_QSPI_DR0_OFFSET, (op->cmd.opcode & 0xff) << 24);
			if(op->addr.nbytes != 0)
			al_writel(qspi, AL_QSPI_DR0_OFFSET, ((op->addr.val & 0xff) << 24) | (((op->addr.val >> 8) & 0xff) << 16)
					| (((op->addr.val >> 16) & 0xff) << 8) | ((op->addr.val >> 24) & 0xff));
			break;
		case QSPI_DFS_16BITS:
			al_writel(qspi, AL_QSPI_DR0_OFFSET, (op->cmd.opcode & 0xff) << 8);
			if(op->addr.nbytes != 0)
			al_writel(qspi, AL_QSPI_DR0_OFFSET, ((op->addr.val & 0xff) << 8) | ((op->addr.val >> 8) & 0xff)
					| (((op->addr.val >> 16) & 0xff) << 24) | (((op->addr.val >> 24) & 0xff) << 16));
			break;
		case QSPI_DFS_8BITS:
			al_writel(qspi, AL_QSPI_DR0_OFFSET, (op->cmd.opcode& 0xff));
			if(op->addr.nbytes != 0)
			al_writel(qspi, AL_QSPI_DR0_OFFSET, op->addr.val);
			break;
		default:
			break;
		}
	}

	if (SPI_MEM_DATA_OUT == op->data.dir) {
		len = qspi->fifo_len - al_readl(qspi, AL_QSPI_TXFLR_OFFSET);
		len = min(len, (op->data.nbytes ? (op->data.nbytes / (qspi->info.dfs / 8)) : 0));
		buf = (u8 *)op->data.buf.out;
	}
	while (len) {
		al_write_data(qspi, buf);
		buf += qspi->n_bytes;
		len--;
	}

	/*
	 * After setting any bit in the SER register the transmission will
	 * start automatically. We have to keep up with that procedure
	 * otherwise the CS de-assertion will happen whereupon the memory
	 * operation will be pre-terminated.
	 */
	if (SPI_MEM_DATA_OUT == op->data.dir)
		len = (op->data.nbytes - ((void *)buf - op->data.buf.out)) / (qspi->info.dfs / 8);

	al_qspi_set_cs(spi, false);

	while (len) {
		entries = readl_relaxed(qspi->regs + AL_QSPI_TXFLR_OFFSET);
		if ((!entries) && (qspi->info.spi_frame_format == SPI_STANDARD_FORMAT)) {
			dev_err(&qspi->master->dev, "CS de-assertion on Tx\n");
			return -EIO;
		}
		room = min(qspi->fifo_len - entries, len);
		for (; room; --room, --len) {
			al_write_data(qspi, buf);
			buf += qspi->n_bytes;
		}
	}

	/*
	 * Data fetching will start automatically if the EEPROM-read mode is
	 * activated. We have to keep up with the incoming data pace to
	 * prevent the Rx FIFO overflow causing the inbound data loss.
	 */
	if (SPI_MEM_DATA_IN == op->data.dir) {
		len = op->data.nbytes / (qspi->info.dfs / 8);
		buf = (u8 *)op->data.buf.in;
	}
	while (len) {
		entries = readl_relaxed(qspi->regs + AL_QSPI_RXFLR_OFFSET);
		if (!entries) {
			sts = readl_relaxed(qspi->regs + AL_QSPI_RISR_OFFSET);
			if (sts & SPI_INT_RXOI) {
				dev_err(&qspi->master->dev, "FIFO overflow on Rx\n");
				return -EIO;
			}
			continue;
		}
		entries = min(entries, len);
		for (; entries; --entries, --len) {
			al_read_data(qspi, buf);
			buf += qspi->n_bytes;
		}
	}
	return 0;
}

static inline bool al_qspi_ctlr_busy(struct al_qspi *qspi)
{
	return al_readl(qspi, AL_QSPI_SR_OFFSET) & SR_BUSY;
}

static int al_qspi_wait_mem_op_done(struct al_qspi *qspi)
{
	int retry = SPI_WAIT_RETRIES;
	struct spi_delay delay;
	unsigned long ns, us;
	u32 nents;

	nents = al_readl(qspi, AL_QSPI_TXFLR_OFFSET);
	ns = NSEC_PER_SEC * nents / qspi->current_freq;
	ns *= qspi->n_bytes * BITS_PER_BYTE;
	if (ns <= NSEC_PER_USEC) {
		delay.unit = SPI_DELAY_UNIT_NSECS;
		delay.value = ns;
	} else {
		us = DIV_ROUND_UP(ns, NSEC_PER_USEC);
		delay.unit = SPI_DELAY_UNIT_USECS;
		delay.value = clamp_val(us, 0, USHRT_MAX);
	}

	while (al_qspi_ctlr_busy(qspi) && retry--)
		spi_delay_exec(&delay, NULL);

	if (retry < 0) {
		dev_err(&qspi->master->dev, "Mem op hanged up\n");
		return -EIO;
	}

	return 0;
}

static void al_qspi_stop_mem_op(struct al_qspi *qspi, struct spi_device *spi)
{
	qspi_enable_chip(qspi, 0);
	al_qspi_set_cs(spi, true);
	qspi_enable_chip(qspi, 1);
}

int al_qspi_check_status(struct al_qspi *qspi, bool raw)
{
	u32 irq_status;
	int ret = 0;

	if (raw)
		irq_status = al_readl(qspi, AL_QSPI_RISR_OFFSET);
	else
		irq_status = al_readl(qspi, AL_QSPI_ISR_OFFSET);

	if (irq_status & SPI_INT_RXOI) {
		dev_err(&qspi->master->dev, "RX FIFO overflow detected\n");
		ret = -EIO;
	}

	if (irq_status & SPI_INT_RXUI) {
		dev_err(&qspi->master->dev, "RX FIFO underflow detected\n");
		ret = -EIO;
	}

	if (irq_status & SPI_INT_TXOI) {
		dev_err(&qspi->master->dev, "TX FIFO overflow detected\n");
		ret = -EIO;
	}

	/* Generically handle the erroneous situation */
	if (ret) {
		qspi_reset_chip(qspi);
		if (qspi->master->cur_msg)
			qspi->master->cur_msg->status = ret;
	}

	return ret;
}
EXPORT_SYMBOL_GPL(al_qspi_check_status);


static int al_qspi_adjust_mem_op_size(struct spi_mem *mem, struct spi_mem_op *op)
{
	if (op->data.dir == SPI_MEM_DATA_IN)
		op->data.nbytes = clamp_val(op->data.nbytes, 0, SPI_NDF_MASK + 1);
	return 0;
}

static bool al_qspi_supports_mem_op(struct spi_mem *mem,
					 const struct spi_mem_op *op)
{

	if (op->data.buswidth > 4 || op->addr.buswidth > 4 ||
		op->dummy.buswidth > 4 || op->cmd.buswidth > 4 || op->addr.nbytes > 4) {
		return false;
	}

	if (op->data.nbytes > 0x40000) {
		return false;
	}

	return spi_mem_default_supports_op(mem, op);
}



/* Return the max entries we can fill into tx fifo */
static inline u32 tx_max(struct al_qspi *qspi)
{
	u32 tx_room, rxtx_gap;

	tx_room = qspi->fifo_len - al_readl(qspi, AL_QSPI_TXFLR_OFFSET);

	/*
	 * Another concern is about the tx/rx mismatch, we
	 * though to use (qspi->fifo_len - rxflr - txflr) as
	 * one maximum value for tx, but it doesn't cover the
	 * data which is out of tx/rx fifo and inside the
	 * shift registers. So a control from sw point of
	 * view is taken.
	 */
	rxtx_gap = qspi->fifo_len - (qspi->rx_len - qspi->tx_len);

	return min3((u32)qspi->tx_len, tx_room, rxtx_gap);
}

/* Return the max entries we should read out of rx fifo */
static inline u32 rx_max(struct al_qspi *qspi)
{
	return min_t(u32, qspi->rx_len, al_readl(qspi, AL_QSPI_RXFLR_OFFSET));
}


static void al_writer(struct al_qspi *qspi)
{
	u32 max = tx_max(qspi);
	al_reg32_set_bits(qspi, AL_QSPI_TXFTLR_OFFSET, QSPI_TXFTLR_TXFTHR_SHIFT,
							QSPI_TXFTLR_TXFTHR_SIZE, max ? max - 1 : 0);
	while (max--) {
		if (qspi->tx) {
			al_write_data(qspi, qspi->tx);
			qspi->tx += qspi->n_bytes;
		}
		--qspi->tx_len;
	}
}

static void al_reader(struct al_qspi *qspi)
{
	u32 max = rx_max(qspi);

	while (max--) {
		if (qspi->rx) {
			al_read_data(qspi, qspi->rx);
			qspi->rx += qspi->n_bytes;
		}
		--qspi->rx_len;
	}
}

static int al_qspi_exec_mem_op(struct spi_mem *mem,
					 const struct spi_mem_op *op)
{
	struct al_qspi *qspi = spi_controller_get_devdata(mem->spi->controller);
	unsigned long flags;
	int mode;
	int ret;
	u16 clk_div;
	u32 speed_hz;
	unsigned int len;

	/*
	 * Anlogic SPI EEPROM-read mode is required only for the SPI memory Data-IN
	 * operation. Transmit-only mode is suitable for the rest of them.
	 */
	mode = al_qspi_find_mode(op);

	qspi->info.spi_frame_format = al_qspi_modes[mode].config_frame_format;
	qspi->info.trans_type = al_qspi_modes[mode].config_trans_type;

	qspi_enable_chip(qspi, 0);
	al_reg32_set_bits(qspi, AL_QSPI_CTRLR0_OFFSET, QSPI_CTRLR0_SPI_FRF_SHIFT,
							QSPI_CTRLR0_SPI_FRF_SIZE, al_qspi_modes[mode].config_frame_format);

	al_reg32_set_bits(qspi, AL_QSPI_SPI_CTRLR0_OFFSET, QSPI_SPI_CTRLR0_TRANS_TYPE_SHIFT,
							QSPI_SPI_CTRLR0_TRANS_TYPE_SIZE, al_qspi_modes[mode].config_trans_type);
	qspi->info.addr_length = op->addr.nbytes * 2;
	qspi->info.inst_length = op->cmd.nbytes + 1;
	if(0 != op->dummy.buswidth)
		qspi->info.wait_cycles = op->dummy.nbytes * 8 / op->dummy.buswidth;
	else
		qspi->info.wait_cycles = 0;
	al_reg32_set_bits(qspi,AL_QSPI_SPI_CTRLR0_OFFSET , QSPI_SPI_CTRLR0_WAIT_CYCLES_SHIFT,
							QSPI_SPI_CTRLR0_WAIT_CYCLES_SIZE, qspi->info.wait_cycles);
	al_reg32_set_bits(qspi, AL_QSPI_SPI_CTRLR0_OFFSET, QSPI_SPI_CTRLR0_INST_L_SHIFT,
							QSPI_SPI_CTRLR0_INST_L_SIZE, qspi->info.inst_length);
	al_reg32_set_bits(qspi, AL_QSPI_SPI_CTRLR0_OFFSET, QSPI_SPI_CTRLR0_ADDR_L_SHIFT,
							QSPI_SPI_CTRLR0_ADDR_L_SIZE, qspi->info.addr_length);
	if((0 != op->data.nbytes) && (0 != op->addr.nbytes) &&
		(SPI_STANDARD_FORMAT != al_qspi_modes[mode].config_frame_format)) {
		if(0 == op->data.nbytes % 4) {
			qspi->info.dfs = 32;
		} else if(0 == op->data.nbytes % 2) {
			qspi->info.dfs = 16;
		} else {
			qspi->info.dfs = 8;
		}
	} else {
		qspi->info.dfs = 8;
	}
	al_reg32_set_bits(qspi, AL_QSPI_CTRLR0_OFFSET, QSPI_CTRLR0_DFS_SHIFT,
							QSPI_CTRLR0_DFS_SIZE, qspi->info.dfs - 1);

	qspi->info.freq = clamp(mem->spi->max_speed_hz, 0U, qspi->max_mem_freq);
	clk_div = (DIV_ROUND_UP(qspi->max_freq, qspi->info.freq) + 1) & 0xfffe;
	speed_hz = qspi->max_freq / clk_div;

	if (qspi->current_freq != speed_hz) {
		qspi_set_clk(qspi, clk_div);
		qspi->current_freq = speed_hz;
	}
	if (op->data.dir == SPI_MEM_DATA_IN) {
		if(SPI_STANDARD_FORMAT == al_qspi_modes[mode].config_frame_format)
			qspi->info.tmod = SPI_TMOD_EPROMREAD;
		else
			qspi->info.tmod = SPI_TMOD_RO;
		al_writel(qspi, AL_QSPI_RXFTLR_OFFSET, min(qspi->fifo_len/2, op->data.nbytes * 8 / qspi->info.dfs));
	} else {
		qspi->info.tmod = SPI_TMOD_TO;
		al_writel(qspi, AL_QSPI_TXFTLR_OFFSET, min(qspi->fifo_len/2, op->data.nbytes * 8 / qspi->info.dfs));
	}
	al_reg32_set_bits(qspi, AL_QSPI_CTRLR0_OFFSET, QSPI_CTRLR0_TMOD_SHIFT,
							QSPI_CTRLR0_TMOD_SIZE, qspi->info.tmod);
	if(SPI_STANDARD_FORMAT == al_qspi_modes[mode].config_frame_format)
		len = min(qspi->fifo_len / 2, op->data.nbytes + op->cmd.nbytes + op->addr.nbytes);
	else
		len = min(qspi->fifo_len / 2, op->data.nbytes * 8 / qspi->info.dfs + 2);

	al_reg32_set_bits(qspi, AL_QSPI_TXFTLR_OFFSET, QSPI_TXFTLR_TXFTHR_SHIFT,
							QSPI_TXFTLR_TXFTHR_SIZE, 0);

	qspi->info.ndf = op->data.nbytes / (qspi->info.dfs / 8);
	al_writel(qspi, AL_QSPI_CTRLR1_OFFSET, qspi->info.ndf ? qspi->info.ndf - 1 : 0);

	qspi_mask_intr(qspi, 0xff);
	qspi_enable_chip(qspi, 1);

	/*
	 * AL APB SSI controller has very nasty peculiarities. First originally
	 * (without any vendor-specific modifications) it doesn't provide a
	 * direct way to set and clear the native chip-select signal. Instead
	 * the controller asserts the CS lane if Tx FIFO isn't empty and a
	 * transmission is going on, and automatically de-asserts it back to
	 * the high level if the Tx FIFO doesn't have anything to be pushed
	 * out. Due to that a multi-tasking or heavy IRQs activity might be
	 * fatal, since the transfer procedure preemption may cause the Tx FIFO
	 * getting empty and sudden CS de-assertion, which in the middle of the
	 * transfer will most likely cause the data loss. Secondly the
	 * EEPROM-read or Read-only AL SPI transfer modes imply the incoming
	 * data being automatically pulled in into the Rx FIFO. So if the
	 * driver software is late in fetching the data from the FIFO before
	 * it's overflown, new incoming data will be lost. In order to make
	 * sure the executed memory operations are CS-atomic and to prevent the
	 * Rx FIFO overflow we have to disable the local interrupts so to block
	 * any preemption during the subsequent IO operations.
	 *
	 * Note. At some circumstances disabling IRQs may not help to prevent
	 * the problems described above. The CS de-assertion and Rx FIFO
	 * overflow may still happen due to the relatively slow system bus or
	 * CPU not working fast enough, so the write-then-read algo implemented
	 * here just won't keep up with the SPI bus data transfer. Such
	 * situation is highly platform specific and is supposed to be fixed by
	 * manually restricting the SPI bus frequency using the
	 * qspi->max_mem_freq parameter.
	 */
	if (qspi->info.spi_frame_format == SPI_STANDARD_FORMAT) {
 		local_irq_save(flags);
		preempt_disable();
	}
	ret = al_qspi_write_then_read(qspi, mem->spi, op);
	if (qspi->info.spi_frame_format == SPI_STANDARD_FORMAT) {
		local_irq_restore(flags);
		preempt_enable();
	}

	/*
	 * Wait for the operation being finished and check the controller
	 * status only if there hasn't been any run-time error detected. In the
	 * former case it's just pointless. In the later one to prevent an
	 * additional error message printing since any hw error flag being set
	 * would be due to an error detected on the data transfer.
	 */
	if (!ret) {
		ret = al_qspi_wait_mem_op_done(qspi);
		if (!ret)
			ret = al_qspi_check_status(qspi, true);
	}

	al_qspi_stop_mem_op(qspi, mem->spi);
	return 0;
}

static irqreturn_t al_qspi_transfer_handler(struct al_qspi *qspi)
{
	u16 irq_status = al_readl(qspi, AL_QSPI_ISR_OFFSET);

	if (al_qspi_check_status(qspi, false)) {
		spi_finalize_current_transfer(qspi->master);
		return IRQ_HANDLED;
	}

	/*
	 * Read data from the Rx FIFO every time we've got a chance executing
	 * this method. If there is nothing left to receive, terminate the
	 * procedure. Otherwise adjust the Rx FIFO Threshold level if it's a
	 * final stage of the transfer. By doing so we'll get the next IRQ
	 * right when the leftover incoming data is received.
	 */
	al_reader(qspi);
	if (!qspi->rx_len) {
		qspi_mask_intr(qspi, 0xff);
		spi_finalize_current_transfer(qspi->master);
	} else if (qspi->rx_len <= al_readl(qspi, AL_QSPI_RXFTLR_OFFSET)) {
		al_writel(qspi, AL_QSPI_RXFTLR_OFFSET, qspi->rx_len - 1);
	}

	/*
	 * Send data out if Tx FIFO Empty IRQ is received. The IRQ will be
	 * disabled after the data transmission is finished so not to
	 * have the TXE IRQ flood at the final stage of the transfer.
	 */
	if (irq_status & SPI_INT_TXEI) {
		al_writer(qspi);
		if (!qspi->tx_len)
			qspi_mask_intr(qspi, SPI_INT_TXEI);
	}

	return IRQ_HANDLED;
}

static const struct of_device_id al_qspi_match[] = {
	{.compatible = "anlogic,dr1x90-qspi" },
	{},
};
MODULE_DEVICE_TABLE(of, al_qspi_match);


static void al_qspi_irq_setup(struct al_qspi *qspi)
{
	u16 level;
	u8 imask;

	/*
	 * Originally Tx and Rx data lengths match. Rx FIFO Threshold level
	 * will be adjusted at the final stage of the IRQ-based SPI transfer
	 * execution so not to lose the leftover of the incoming data.
	 */
	level = min_t(u16, qspi->fifo_len / 2, qspi->tx_len);
	al_writel(qspi, AL_QSPI_TXFTLR_OFFSET, level);
	al_writel(qspi, AL_QSPI_RXFTLR_OFFSET, level - 1);

	qspi->transfer_handler = al_qspi_transfer_handler;

	imask = SPI_INT_TXEI | SPI_INT_TXOI | SPI_INT_RXUI | SPI_INT_RXOI |
		SPI_INT_RXFI;
	qspi_umask_intr(qspi, imask);

}

/*
 * The iterative procedure of the poll-based transfer is simple: write as much
 * as possible to the Tx FIFO, wait until the pending to receive data is ready
 * to be read, read it from the Rx FIFO and check whether the performed
 * procedure has been successful.
 *
 * Note this method the same way as the IRQ-based transfer won't work well for
 * the SPI devices connected to the controller with native CS due to the
 * automatic CS assertion/de-assertion.
 */
static int al_qspi_poll_transfer(struct al_qspi *qspi,
				struct spi_transfer *transfer)
{
	struct spi_delay delay;
	u16 nbits;
	int ret;

	delay.unit = SPI_DELAY_UNIT_SCK;
	nbits = qspi->n_bytes * BITS_PER_BYTE;

	do {
		/* Set addr size and inst size to 0 */
		al_writer(qspi);
		delay.value = nbits * (qspi->rx_len - qspi->tx_len);
		spi_delay_exec(&delay, transfer);

		al_reader(qspi);
		ret = al_qspi_check_status(qspi, true);
		if (ret)
			return ret;
	} while (qspi->rx_len);

	return 0;
}

/* set qspi master width by spi device param rx_nbits and tx_nbits */

static int al_qspi_transfer_one(struct spi_master *master,
		struct spi_device *spi, struct spi_transfer *transfer)
{
	u16 clk_div;
	u32 speed_hz;

	struct al_qspi *qspi = spi_controller_get_devdata(master);
	qspi->info.tmod = SPI_TMOD_TR;
	qspi->info.dfs = transfer->bits_per_word;
	qspi->info.freq = transfer->speed_hz;

	qspi->n_bytes = DIV_ROUND_UP(transfer->bits_per_word, BITS_PER_BYTE);
	qspi->tx = (void *)transfer->tx_buf;
	qspi->tx_len = transfer->len / qspi->n_bytes;
	qspi->rx = transfer->rx_buf;
	qspi->rx_len = qspi->tx_len;

	if (transfer->tx_nbits == SPI_NBITS_DUAL &&
		transfer->rx_nbits == SPI_NBITS_DUAL)
		qspi->info.spi_frame_format = SPI_DUAL_FORMAT;
	else if (transfer->tx_nbits == SPI_NBITS_QUAD &&
			 transfer->rx_nbits == SPI_NBITS_QUAD)
		qspi->info.spi_frame_format = SPI_QUAD_FORMAT;
	else if (transfer->tx_nbits == SPI_NBITS_SINGLE &&
			 transfer->rx_nbits == SPI_NBITS_SINGLE)
		qspi->info.spi_frame_format = SPI_STANDARD_FORMAT;
	else
		return EINVAL;

	if((qspi->info.spi_frame_format == SPI_STANDARD_FORMAT) && (qspi->info.dfs > 8))
		return EINVAL;

	if(qspi->tx_len > 0x10000)
		return EINVAL;

	/* Ensure the data above is visible for all CPUs */
	smp_mb();

	qspi_enable_chip(qspi, 0);

	al_reg32_set_bits(qspi, AL_QSPI_CTRLR0_OFFSET, QSPI_CTRLR0_TMOD_SHIFT,
							QSPI_CTRLR0_TMOD_SIZE, qspi->info.tmod);
	al_reg32_set_bits(qspi, AL_QSPI_CTRLR0_OFFSET, QSPI_CTRLR0_DFS_SHIFT,
							QSPI_CTRLR0_DFS_SIZE, qspi->info.dfs - 1);
	al_reg32_set_bits(qspi, AL_QSPI_CTRLR0_OFFSET, QSPI_CTRLR0_SPI_FRF_SHIFT,
							QSPI_CTRLR0_SPI_FRF_SIZE, qspi->info.spi_frame_format);

	al_writel(qspi, AL_QSPI_CTRLR1_OFFSET, qspi->rx_len - 1);

	/* Note anlogic qspi clock divider doesn't support odd numbers */
	clk_div = (DIV_ROUND_UP(qspi->max_freq, qspi->info.freq) + 1) & 0xfffe;
	speed_hz = qspi->max_freq / clk_div;

	if (qspi->current_freq != speed_hz) {
		qspi_set_clk(qspi, clk_div);
		qspi->current_freq = speed_hz;
	}
	transfer->effective_speed_hz = qspi->current_freq;

	al_reg32_set_bits(qspi, AL_QSPI_SPI_CTRLR0_OFFSET, QSPI_SPI_CTRLR0_WAIT_CYCLES_SHIFT,
							QSPI_SPI_CTRLR0_WAIT_CYCLES_SIZE, 0);
	al_reg32_set_bits(qspi,AL_QSPI_SPI_CTRLR0_OFFSET , QSPI_SPI_CTRLR0_INST_L_SHIFT,
							QSPI_SPI_CTRLR0_INST_L_SIZE, 0);
	al_reg32_set_bits(qspi,AL_QSPI_SPI_CTRLR0_OFFSET , QSPI_SPI_CTRLR0_ADDR_L_SHIFT,
							QSPI_SPI_CTRLR0_ADDR_L_SIZE, 0);
	al_reg32_set_bits(qspi, AL_QSPI_SPI_CTRLR0_OFFSET, QSPI_SPI_CTRLR0_TRANS_TYPE_SHIFT,
							QSPI_SPI_CTRLR0_TRANS_TYPE_SIZE, 0);

	/* For poll mode just disable all interrupts */
	qspi_mask_intr(qspi, 0xff);

	qspi_enable_chip(qspi, 1);

	al_qspi_poll_transfer(qspi, transfer);

	al_qspi_irq_setup(qspi);

	return 1;
}

static void al_qspi_handle_err(struct spi_master *master,
		struct spi_message *msg)
{
	struct al_qspi *qspi = spi_controller_get_devdata(master);

	qspi_reset_chip(qspi);
}

static void al_qspi_hardware_init(struct al_qspi *qspi, struct device *dev)
{
	qspi_reset_chip(qspi);
	qspi_enable_chip(qspi, 0);
	/* Get default rx sample delay */
	device_property_read_u32(dev, "rx-sample-delay-ns",
				 &qspi->def_rx_sample_dly_ns);
	al_reg32_set_bit(qspi, AL_QSPI_SPI_CTRLR0_OFFSET,
						QSPI_SPI_CTRLR0_CLK_STRETCH_EN_SHIFT, 1);
	al_reg32_set_bit(qspi, AL_QSPI_CTRLR0_OFFSET,
						QSPI_CTRLR0_SSTE_SHIFT, 0);
	al_writel(qspi, AL_QSPI_SER_OFFSET, 0);
	qspi_enable_chip(qspi, 1);
}

static void al_qspi_cleanup(struct spi_device *spi)
{
	struct chip_data *chip = spi_get_ctldata(spi);

	kfree(chip);
	spi_set_ctldata(spi, NULL);
}

static void al_qspi_init_mem_ops(struct al_qspi *qspi)
{
	if (!qspi->mem_ops.exec_op) {
		qspi->mem_ops.adjust_op_size = al_qspi_adjust_mem_op_size;
		qspi->mem_ops.supports_op = al_qspi_supports_mem_op;
		qspi->mem_ops.exec_op = al_qspi_exec_mem_op;
		if (!qspi->max_mem_freq)
			qspi->max_mem_freq = qspi->max_freq;
	}
}

static int al_qspi_probe(struct platform_device *pdev)
{
	struct	al_qspi *qspi;
	struct spi_master *master;
	struct resource *res;
	int irq, num_cs, err = 0;

	master = spi_alloc_master(&pdev->dev, sizeof(*qspi));
	if (!master)
		return -ENOMEM;
	qspi = spi_controller_get_devdata(master);
	qspi->master = master;
	master->mode_bits = SPI_MODE_0 | SPI_RX_DUAL | SPI_RX_QUAD | SPI_TX_DUAL | SPI_TX_QUAD;
	master->bits_per_word_mask = SPI_BPW_MASK(32) | SPI_BPW_MASK(16) | SPI_BPW_MASK(8);

	qspi->regs = devm_platform_get_and_ioremap_resource(pdev, 0, &res);
	if (IS_ERR(qspi->regs))
		return PTR_ERR(qspi->regs);

	/* Request the IRQ */
	irq = platform_get_irq(pdev, 0);
	if (irq < 0) {
		err = irq;
		goto exit;
	}

	qspi->clk = devm_clk_get(&pdev->dev, NULL);
	if (IS_ERR(qspi->clk)) {
		err = PTR_ERR(qspi->clk);
		goto exit;
	}

	err = clk_prepare_enable(qspi->clk);
	if (err)
		 goto exit;

	/* find an optional reset controller */
	qspi->rstc = devm_reset_control_get_optional_exclusive(&pdev->dev, "spi");
	if (IS_ERR(qspi->rstc)) {
		err = PTR_ERR(qspi->rstc);
		goto out_clk;
	}
	reset_control_deassert(qspi->rstc);

	/* Optional clock needed to access the registers */
	qspi->pclk = devm_clk_get_optional(&pdev->dev, "pclk");
	if (IS_ERR(qspi->pclk))
	{
		err = PTR_ERR(qspi->pclk);
		goto out_clk;
	}
	err = clk_prepare_enable(qspi->pclk);
	if (err)
		goto out;

	qspi->max_freq = clk_get_rate(qspi->clk);
	qspi->fifo_len = 128;
	al_qspi_init_mem_ops(qspi);
	master->auto_runtime_pm = true;
	master->dev.of_node = pdev->dev.of_node;
	master->dev.fwnode = pdev->dev.fwnode;
	master->mem_ops = &qspi->mem_ops;
	master->handle_err = al_qspi_handle_err;
	master->set_cs = al_qspi_set_cs;
	master->transfer_one = al_qspi_transfer_one;
	master->cleanup = al_qspi_cleanup;

	if (!qspi->max_mem_freq)
			qspi->max_mem_freq = qspi->max_freq;
	master->max_speed_hz = qspi->max_freq;
	platform_set_drvdata(pdev, master);

	if (!of_property_read_u32(pdev->dev.of_node, "num-cs", &num_cs)) {
		master->num_chipselect = num_cs;
		qspi->num_cs = num_cs;
	}

	err = devm_request_irq(&pdev->dev, irq, al_qspi_irq,
					 0, dev_name(&pdev->dev), qspi);
	if (err)
		goto out;

	al_qspi_hardware_init(qspi, &pdev->dev);

	err = spi_register_controller(master);
	if (err)
		goto out;

	return 0;
out:
	clk_disable_unprepare(qspi->pclk);
out_clk:
	reset_control_assert(qspi->rstc);
	clk_disable_unprepare(qspi->clk);

exit:
	spi_controller_put(master);

	return err;
}

static int al_qspi_remove(struct platform_device *pdev)
{
	struct spi_master *master = platform_get_drvdata(pdev);
	struct al_qspi *qspi = spi_master_get_devdata(master);

	spi_unregister_controller(master);
	clk_disable_unprepare(qspi->pclk);
	clk_disable_unprepare(qspi->clk);
	reset_control_assert(qspi->rstc);

	return 0;
}

static struct platform_driver al_qspi_driver = {
	.probe	= al_qspi_probe,
	.remove = al_qspi_remove,
	.driver = {
		.name	= "al-qspi",
		.of_match_table = al_qspi_match,
	}
};

module_platform_driver(al_qspi_driver);

MODULE_AUTHOR("jian huang <jian.huang@anlogic.com>");
MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("Anlogic QSPI controller driver");
MODULE_ALIAS("platform:al-qspi");
