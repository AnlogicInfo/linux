// SPDX-License-Identifier: GPL-2.0
/*
 * anlogic AXI I2C Controller Driver
 *
 * Copyright (c) 2024 anlogic Technologies Co., Ltd.
 */

#include <linux/bits.h>
#include <linux/bitfield.h>
#include <linux/completion.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/mod_devicetable.h>
#include <linux/platform_device.h>
#include <linux/property.h>
#include <linux/units.h>

#define ANLOGIC_I2C_SLV_ADDR_VAL GENMASK(9, 0)
#define ANLOGIC_I2C_DATA_CNT GENMASK(7, 0)

/* anlogic AXI I2C Register Offsets */
#define AXI_I2C_ID (0x00) /* ID Register */
#define AXI_I2C_CFG (0x10) /* Configuration Register */
#define AXI_I2C_INTEN (0x14) /* Interrupt Enable Register */
#define AXI_I2C_SR (0x18) /* Status Register */
#define AXI_I2C_ADDR (0x1C) /* Address Register */
#define AXI_I2C_DATA (0x20) /* Data Register */
#define AXI_I2C_CR (0x24) /* Control Register */
#define AXI_I2C_CMD (0x28) /* Command Register */
#define AXI_I2C_SETUP (0x2C) /* Setup Register */
#define AXI_I2C_TPM (0x30) /* TPM Register */

/***************************** AXI_I2C_ID_OFFSET Register *****************************/
#define AXI_I2C_ID_START 8
#define AXI_I2C_ID_SIZE 24

/***************************** AXI_I2C_CFG_OFFSET Register *****************************/
#define AXI_I2C_CFG_START 0
#define AXI_I2C_CFG_SIZE 2

/***************************** AXI_I2C_INTEN_OFFSET Register *****************************/
#define INT_CMPL 9
#define INT_BYTE_RECV 8
#define INT_BYTE_TRANS 7
#define INT_START 6
#define INT_STOP 5
#define INT_ARB_LOSE 4
#define INT_ADDR_HIT 3
#define INT_FIFO_HALF 2
#define INT_FIFO_FULL 1
#define INT_FIFO_EMPTY 0

/***************************** AXI_I2C_SR_OFFSET Register *****************************/
#define SR_LINE_SDA 14
#define SR_LINE_SCL 13
#define SR_GEN_CALL 12
#define SR_BUS_BUSY 11
#define SR_ACK 10
#define SR_CMPL 9
#define SR_BYTE_RECV 8
#define SR_BYTE_TRANS 7
#define SR_START 6
#define SR_STOP 5
#define SR_ARB_LOSE 4
#define SR_ADDR_HIT 3
#define SR_FIFO_HALF 2
#define SR_FIFO_FULL 1
#define SR_FIFO_EMPTY 0

/***************************** AXI_I2C_CR_OFFSET Register *****************************/
#define CR_PHASE_START 12
#define CR_PHASE_ADDR 11
#define CR_PHASE_DATA 10
#define CR_PHASE_STOP 9
#define CR_DIR 8
#define CR_DATA_CNT 0 /* 0 - 7 */
#define CR_DATA_CNT_SIZE 8

/***************************** AXI_I2C_CMD_OFFSET Register *****************************/
#define CMD 0 /* 0 - 2 */
#define CMD_SIZE 3

/***************************** AXI_I2C_SETUP_OFFSET Register *****************************/
#define T_SUDAT 24
#define T_SP 21
#define T_HDDAT 16
#define T_SCL_RATIO 13
#define T_SCL_HIGH 4
#define I2C_MODE 2
#define I2C_ADDRESS_MODE 1
#define I2C_EN 0

#define T_SUDAT_SIZE 5
#define T_SP_SIZE 3
#define T_HDDAT_SIZE 5
#define T_SCL_HIGH_SIZE 9

#define I2C_SEND_INT  ((1 << INT_FIFO_EMPTY) | (1 << INT_CMPL) | (1 << INT_ARB_LOSE))
#define I2C_RECV_INT  ((1 << INT_FIFO_FULL) | (1 << INT_CMPL) | (1 << INT_ARB_LOSE))

enum {
	MASTER_TX_SLAVE_RX = 0,
	MASTER_RX_SLAVE_TX = 1
};

enum {
	AL_AXI_I2C_CMD_NONE = 0x0,
	AL_AXI_I2C_CMD_TRANS = 0x1,
	AL_AXI_I2C_CMD_ACK = 0x2,
	AL_AXI_I2C_CMD_NACK = 0x3,
	AL_AXI_I2C_CMD_CLR_FIFO = 0x4,
	AL_AXI_I2C_CMD_RST = 0x5
};

enum {
	AL_AXI_I2C_SCL_RATIO_1 = 0,
	AL_AXI_I2C_SCL_RATIO_2 = 1
};

enum {
	AL_AXI_I2C_SLAVE = 0,
	AL_AXI_I2C_MASTER = 1
};

enum {
	AL_AXI_I2C_7BIT = 0,
	AL_AXI_I2C_10BIT = 1
};

enum i2c_xfer_state {
	I2C_XFER_STATE_IDLE,          // Idle state: no ongoing transfer
	I2C_XFER_STATE_COMPLETE,      // Transfer complete state
	I2C_XFER_STATE_SENDING_ADDR,  // Sending address state
	I2C_XFER_STATE_SENDING_DATA,  // Sending data state
	I2C_XFER_STATE_RECVING_DATA,  // Recving data state
	I2C_XFER_STATE_STOPPING,      // Stopping transfer state
};

struct anlogic_i2c_controller {
	struct i2c_adapter adapter;
	void __iomem *iobase;
	struct device *dev;
	int irq;
	enum i2c_xfer_state state;

	/* Intermediates for recording the transfer process */
	struct completion *completion;
	struct i2c_msg *msgs;
	int num_messages;       // Total number of messages
	int tx_buffer_index;    // Index within the transmit buffer
	int rx_buffer_index;    // Index within the receive buffer
	int current_msg_index;  // Index of the current message
	u16 target_address;     // Target address
	u32 transfer_error;     // Transfer error

	/* I2C bus configuration */
	struct i2c_timings t;
	u32 clk_rate_khz;
	u32 spk_len;
};

static inline void anlogic_i2c_modify_int(struct anlogic_i2c_controller *ctlr,
					  u32 mask, bool enable)
{
	u32 reg = readl(ctlr->iobase + AXI_I2C_INTEN);

	if (enable)
		reg |= mask;
	else
		reg &= ~mask;
	writel(reg, ctlr->iobase + AXI_I2C_INTEN);
}

static inline void anlogic_i2c_enable_int(struct anlogic_i2c_controller *ctlr,
					  u32 mask)
{
	anlogic_i2c_modify_int(ctlr, mask, true);
}

static inline void anlogic_i2c_disable_int(struct anlogic_i2c_controller *ctlr,
					   u32 mask)
{
	anlogic_i2c_modify_int(ctlr, mask, false);
}

static inline void anlogic_i2c_clear_int(struct anlogic_i2c_controller *ctlr,
					 u32 mask)
{
	writel(mask, ctlr->iobase + AXI_I2C_SR);
}

static void anlogic_i2c_handle_errors(struct anlogic_i2c_controller *ctlr)
{
	u32 int_err = ctlr->transfer_error;

	if (int_err & (1 << SR_ARB_LOSE))
		dev_err(ctlr->dev, "AXI I2C Arbitration Lost Error\n");
}

static int anlogic_i2c_start_xfer(struct anlogic_i2c_controller *ctlr,
				  struct i2c_msg *msg, u32 mask)
{
	u32 cr_reg, setup_reg, addr_reg;

	/* 1. Add start/Address/data/stop byte */
	cr_reg = readl(ctlr->iobase + AXI_I2C_CR);
	cr_reg |= ((1 << CR_PHASE_START) | (1 << CR_PHASE_ADDR) |
		   (1 << CR_PHASE_DATA) | (1 << CR_PHASE_STOP));
	writel(cr_reg, ctlr->iobase + AXI_I2C_CR);

	/* 2. Set Address mode */
	setup_reg = readl(ctlr->iobase + AXI_I2C_SETUP);
	if (msg->flags & I2C_M_TEN)
		setup_reg |= (AL_AXI_I2C_10BIT << I2C_ADDRESS_MODE);
	else
		setup_reg &= ~(AL_AXI_I2C_10BIT << I2C_ADDRESS_MODE);
	writel(setup_reg, ctlr->iobase + AXI_I2C_SETUP);

	/* 3. Set slave address */
	addr_reg = FIELD_PREP(ANLOGIC_I2C_SLV_ADDR_VAL, msg->addr);
	writel(addr_reg, ctlr->iobase + AXI_I2C_ADDR);

	/* 4. Set Transaction direction */
	if (msg->flags & I2C_M_RD)
		cr_reg |= (MASTER_RX_SLAVE_TX << CR_DIR);
	else
		cr_reg &= ~(MASTER_RX_SLAVE_TX << CR_DIR);

	/* 5. Set DATA count */
	cr_reg &= ~ANLOGIC_I2C_DATA_CNT;
	cr_reg |= FIELD_PREP(ANLOGIC_I2C_DATA_CNT, msg->len);
	writel(cr_reg, ctlr->iobase + AXI_I2C_CR);

	anlogic_i2c_clear_int(ctlr, mask);
	anlogic_i2c_enable_int(ctlr, mask);

	/* 6. Start Transaction */
	ctlr->state = I2C_XFER_STATE_SENDING_ADDR;
	writel(AL_AXI_I2C_CMD_TRANS, ctlr->iobase + AXI_I2C_CMD);

	return 0;
}

static void anlogic_i2c_reset_xfer(struct anlogic_i2c_controller *ctlr)
{
	ctlr->transfer_error = 0;
	ctlr->tx_buffer_index = 0;
	ctlr->rx_buffer_index = 0;
	ctlr->state = I2C_XFER_STATE_IDLE;
}

static int anlogic_i2c_master_xfer(struct i2c_adapter *adap,
				   struct i2c_msg *msgs, int num)
{
	struct anlogic_i2c_controller *ctlr = i2c_get_adapdata(adap);
	DECLARE_COMPLETION_ONSTACK(done);
	int ret = num;
	u32 mask;

	ctlr->completion = &done;
	ctlr->num_messages = num;
	ctlr->msgs = msgs;
	ctlr->current_msg_index = 0;

	for (ctlr->current_msg_index = 0; ctlr->current_msg_index < ctlr->num_messages;
		ctlr->current_msg_index++) {
		struct i2c_msg *current_msg = &ctlr->msgs[ctlr->current_msg_index];
		/* Reset ctlr related indices */
		anlogic_i2c_reset_xfer(ctlr);
		reinit_completion(ctlr->completion);

		/* Start the transfer with only address hit interrupt enabled */
		anlogic_i2c_start_xfer(ctlr, current_msg, (1 << INT_ADDR_HIT));

		/* Wait for the current message transfer to complete */
		if (!wait_for_completion_timeout(ctlr->completion, adap->timeout)) {
			anlogic_i2c_disable_int(ctlr, mask);
			synchronize_irq(ctlr->irq);
			i2c_recover_bus(&ctlr->adapter);
			ret = -ETIMEDOUT;
			goto out;
		}

		if (ctlr->transfer_error) {
			anlogic_i2c_handle_errors(ctlr);
			ret = -EIO;
			break;
		}
	}

out:
	anlogic_i2c_reset_xfer(ctlr);
	ctlr->state = I2C_XFER_STATE_IDLE;
	ctlr->completion = NULL;

	return ret;
}

static u32 anlogic_i2c_functionality(struct i2c_adapter *adap)
{
	return I2C_FUNC_I2C | I2C_FUNC_10BIT_ADDR | I2C_FUNC_SMBUS_EMUL;
}

static const struct i2c_algorithm anlogic_i2c_algo = {
	.master_xfer = anlogic_i2c_master_xfer,
	.functionality = anlogic_i2c_functionality,
};

static int anlogic_i2c_read_rx_fifo(struct anlogic_i2c_controller *ctlr)
{
	struct i2c_msg *current_msg = &ctlr->msgs[ctlr->current_msg_index];

	/* Receive data while the FIFO is not empty and there is space in the buffer */
	while (!(readl(ctlr->iobase + AXI_I2C_SR) & (1 << SR_FIFO_EMPTY)) &&
	       ctlr->rx_buffer_index < current_msg->len) {
		current_msg->buf[ctlr->rx_buffer_index] =
			(unsigned char)(readl(ctlr->iobase + AXI_I2C_DATA) &
					0xFF);
		ctlr->rx_buffer_index++;
	}

	if (ctlr->rx_buffer_index == current_msg->len) {
		ctlr->state = I2C_XFER_STATE_COMPLETE;
		anlogic_i2c_disable_int(ctlr, (1 << INT_FIFO_FULL));
	}

	return 0;
}

static void anlogic_i2c_xfer_msg(struct anlogic_i2c_controller *ctlr)
{
	struct i2c_msg *current_msg = &ctlr->msgs[ctlr->current_msg_index];

	/* Send data while the FIFO is not full and there is data to send */
	while (!(readl(ctlr->iobase + AXI_I2C_SR) & (1 << SR_FIFO_FULL)) &&
	       ctlr->tx_buffer_index < current_msg->len) {
		writel(current_msg->buf[ctlr->tx_buffer_index], ctlr->iobase + AXI_I2C_DATA);
		ctlr->tx_buffer_index++;
	}

	/*
	 * Disable the TX_EMPTY interrupt after finishing all messages
	 * to avoid overwhelming the CPU
	 */
	if (ctlr->tx_buffer_index == current_msg->len) {
		ctlr->state = I2C_XFER_STATE_COMPLETE;
		anlogic_i2c_disable_int(ctlr, (1 << INT_FIFO_EMPTY));
	}
}

static irqreturn_t anlogic_i2c_irq(int irq, void *context)
{
	struct anlogic_i2c_controller *ctlr = context;
	u32 int_stat;

	if (ctlr->state == I2C_XFER_STATE_IDLE)
		return IRQ_NONE;

	if (!ctlr->completion)
		return IRQ_NONE;

	int_stat = readl(ctlr->iobase + AXI_I2C_SR); /* Read status register */
	anlogic_i2c_clear_int(ctlr, int_stat); /* Clear interrupt status */

	if (int_stat & (1 << SR_ADDR_HIT)
		&& ctlr->state == I2C_XFER_STATE_SENDING_ADDR) {
		ctlr->state = (ctlr->msgs[ctlr->current_msg_index].flags & I2C_M_RD) ?
		I2C_XFER_STATE_RECVING_DATA : I2C_XFER_STATE_SENDING_DATA;
		anlogic_i2c_disable_int(ctlr, (1 << INT_ADDR_HIT));
		anlogic_i2c_enable_int(ctlr, ctlr->msgs[ctlr->current_msg_index].flags & I2C_M_RD ?
			I2C_RECV_INT : I2C_SEND_INT);
		return IRQ_HANDLED;
	}

	if (int_stat & (1 << SR_FIFO_EMPTY)
		&& ctlr->state == I2C_XFER_STATE_SENDING_DATA)
		anlogic_i2c_xfer_msg(ctlr);

	if (int_stat & ((1 << SR_FIFO_FULL) | (1 << SR_CMPL))
		&& ctlr->state == I2C_XFER_STATE_RECVING_DATA)
		anlogic_i2c_read_rx_fifo(ctlr);

	if (int_stat & (1 << SR_ARB_LOSE))
		ctlr->transfer_error = int_stat;

	if (int_stat & (1 << SR_CMPL)
		&& (ctlr->state == I2C_XFER_STATE_COMPLETE)) {
		ctlr->state = I2C_XFER_STATE_STOPPING;   /* Transition to stop state */
		anlogic_i2c_clear_int(ctlr, int_stat);   /* Clear interrupts status */
		anlogic_i2c_disable_int(ctlr, int_stat); /* Disable interrupts */
		complete(ctlr->completion); /* Signal transfer completion or error */
	}

	return IRQ_HANDLED;
}

static void anlogic_deinit(struct anlogic_i2c_controller *ctlr)
{
	u8 setup_reg;

	/* Reset AXI I2C */
	writel(AL_AXI_I2C_CMD_RST, ctlr->iobase + AXI_I2C_CFG);

	/* Disable AXI I2C */
	setup_reg = readl(ctlr->iobase + AXI_I2C_SETUP);
	setup_reg &= ~(1 << I2C_EN);
	writel(setup_reg, ctlr->iobase + AXI_I2C_SETUP);
}

static void anlogic_i2c_set_scl(struct anlogic_i2c_controller *ctlr)
{
	u32 setup_reg;
	u32 Tsclh;

	/* Read the current setup register value */
	setup_reg = readl(ctlr->iobase + AXI_I2C_SETUP);

	/* Set SCL Ratio */
	setup_reg &= ~(AL_AXI_I2C_SCL_RATIO_2 << T_SCL_RATIO);
	writel(setup_reg, ctlr->iobase + AXI_I2C_SETUP);

	/* Set SCL High Time */
	Tsclh = (5000 / 20) - 5;
	setup_reg |= (Tsclh << T_SCL_HIGH);
	writel(setup_reg, ctlr->iobase + AXI_I2C_SETUP);
}

static void anlogic_i2c_configure_bus(struct anlogic_i2c_controller *ctlr)
{
	u32 setup_reg;
	u32 axi_i2c_controller_id;

	/* 1. Reset AXI I2C */
	writel(AL_AXI_I2C_CMD_RST, ctlr->iobase + AXI_I2C_CFG);

	/* 2. Check I2C ID */
	axi_i2c_controller_id = ((readl(ctlr->iobase + AXI_I2C_ID)) & 0xFFFFFF00) >> 8;
	dev_info(ctlr->dev, "anlogic axi i2c controller id 0x%x\n",
		 axi_i2c_controller_id);

	/* 3. Enable IP and Set Master/Slave Mode */
	setup_reg = readl(ctlr->iobase + AXI_I2C_SETUP);
	setup_reg |= (1 << I2C_EN) | (AL_AXI_I2C_MASTER << I2C_MODE);
	writel(setup_reg, ctlr->iobase + AXI_I2C_SETUP);

	/* 4. Set default speed Mode: I2C_STANDARD_MODE */
	anlogic_i2c_set_scl(ctlr);
}

static int anlogic_i2c_probe(struct platform_device *pdev)
{
	struct anlogic_i2c_controller *ctlr;
	struct device *dev = &pdev->dev;
	struct i2c_adapter *adapter;
	int ret;

	ctlr = devm_kzalloc(dev, sizeof(*ctlr), GFP_KERNEL);
	if (!ctlr)
		return -ENOMEM;

	ctlr->iobase = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(ctlr->iobase))
		return PTR_ERR(ctlr->iobase);

	ctlr->irq = platform_get_irq(pdev, 0);
	if (ctlr->irq < 0)
		return ctlr->irq;

	ctlr->dev = dev;

	ret = devm_request_irq(dev, ctlr->irq, anlogic_i2c_irq, 0,
			       "anlogic-axi-i2c", ctlr);
	if (ret) {
		dev_err(dev, "Failed to request IRQ handler, ret = %d\n", ret);
		return ret;
	}

	anlogic_i2c_configure_bus(ctlr);

	adapter = &ctlr->adapter;
	snprintf(adapter->name, sizeof(adapter->name),
		 "anlogic axi i2c Controller %s", dev_name(dev));
	adapter->owner = THIS_MODULE;
	adapter->algo = &anlogic_i2c_algo;
	adapter->dev.parent = dev;
	adapter->dev.of_node = pdev->dev.of_node;
	adapter->nr = -1;

	i2c_set_adapdata(adapter, ctlr);

	ret = i2c_add_adapter(&ctlr->adapter);
	if (ret) {
		anlogic_deinit(ctlr);
		return ret;
	}

	return 0;
}

static int anlogic_i2c_remove(struct platform_device *pdev)
{
	struct anlogic_i2c_controller *ctlr = platform_get_drvdata(pdev);

	/* remove adapter & data */
	i2c_del_adapter(&ctlr->adapter);

	anlogic_deinit(ctlr);

	return 0;
}

#if defined(CONFIG_OF)
static const struct of_device_id anlogic_of_match[] = {
	{
		.compatible = "anlogic-axi-i2c-controller",
	},
	{},
};
MODULE_DEVICE_TABLE(of, anlogic_of_match);
#endif

static struct platform_driver anlogic_i2c_driver = {
	.probe = anlogic_i2c_probe,
	.remove = anlogic_i2c_remove,
	.driver = {
		.name = "anlogic-axi-i2c",
		.of_match_table = of_match_ptr(anlogic_of_match),
	},
};
module_platform_driver(anlogic_i2c_driver);

MODULE_AUTHOR("Naiyuan Que <naiyuan.que@anlogic.com>");
MODULE_DESCRIPTION("anlogic axi i2c Controller Driver");
MODULE_LICENSE("GPL");
