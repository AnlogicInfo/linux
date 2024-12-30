// SPDX-License-Identifier: GPL-2.0
/*
 * anlogic axi i2c Controller Driver
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

#define I2C_TIMEOUT                   msecs_to_jiffies(100)

#define ANLOGIC_I2C_SLAVE_ADDR_VAL    GENMASK(9, 0)
#define ANLOGIC_I2C_DATA_CNT          GENMASK(7, 0)
#define ANLOGIC_I2C_INT_ALL           GENMASK(9, 0)

/* anlogic axi i2c register offsets */
#define AXI_I2C_ID                    (0x00) /* ID Register */
#define AXI_I2C_CFG                   (0x10) /* Configuration Register */
#define AXI_I2C_INTEN                 (0x14) /* Interrupt Enable Register */
#define AXI_I2C_SR                    (0x18) /* Status Register */
#define AXI_I2C_ADDR                  (0x1C) /* Address Register */
#define AXI_I2C_DATA                  (0x20) /* Data Register */
#define AXI_I2C_CR                    (0x24) /* Control Register */
#define AXI_I2C_CMD                   (0x28) /* Command Register */
#define AXI_I2C_SETUP                 (0x2C) /* Setup Register */
#define AXI_I2C_TPM                   (0x30) /* TPM Register */

/***************************** AXI_I2C_ID_OFFSET Register *****************************/
#define AXI_I2C_ID_START                  8
#define AXI_I2C_ID_SIZE                  24

/***************************** AXI_I2C_CFG_OFFSET Register *****************************/
#define AXI_I2C_CFG_START                 0
#define AXI_I2C_CFG_SIZE                  2

/***************************** AXI_I2C_INTEN_OFFSET Register *****************************/
#define INT_CMPL                          9
#define INT_BYTE_RECV                     8
#define INT_BYTE_TRANS                    7
#define INT_START                         6
#define INT_STOP                          5
#define INT_ARB_LOSE                      4
#define INT_ADDR_HIT                      3
#define INT_FIFO_HALF                     2
#define INT_FIFO_FULL                     1
#define INT_FIFO_EMPTY                    0

/***************************** AXI_I2C_SR_OFFSET Register *****************************/
#define SR_LINE_SDA                      14
#define SR_LINE_SCL                      13
#define SR_GEN_CALL                      12
#define SR_BUS_BUSY                      11
#define SR_ACK                           10
#define SR_CMPL                           9
#define SR_BYTE_RECV                      8
#define SR_BYTE_TRANS                     7
#define SR_START                          6
#define SR_STOP                           5
#define SR_ARB_LOSE                       4
#define SR_ADDR_HIT                       3
#define SR_FIFO_HALF                      2
#define SR_FIFO_FULL                      1
#define SR_FIFO_EMPTY                     0

/***************************** AXI_I2C_CR_OFFSET Register *****************************/
#define CR_PHASE_START                   12
#define CR_PHASE_ADDR                    11
#define CR_PHASE_DATA                    10
#define CR_PHASE_STOP                     9
#define CR_DIR                            8
#define CR_DATA_CNT                       0
#define CR_DATA_CNT_SIZE                  8

/***************************** AXI_I2C_CMD_OFFSET Register *****************************/
#define CMD                               0
#define CMD_SIZE                          3

/***************************** AXI_I2C_SETUP_OFFSET Register *****************************/
#define T_SUDAT_SHIFT                    24
#define T_SP_SHIFT                       21
#define T_HDDAT_SHIFT                    16
#define T_SCL_RATIO                      13
#define T_SCL_HIGH                        4
#define I2C_MODE                          2
#define I2C_ADDRESS_MODE                  1
#define I2C_EN                            0

#define T_SUDAT_SIZE                      5
#define T_SP_SIZE                         3
#define T_HDDAT_SIZE                      5
#define T_SCL_HIGH_SIZE                   9

#define I2C_INIT_INT                  (BIT(INT_ADDR_HIT) | BIT(INT_ARB_LOSE))
#define I2C_SEND_INT                  (BIT(INT_FIFO_EMPTY) | BIT(INT_CMPL) | BIT(INT_ARB_LOSE))
#define I2C_RECV_INT                  (BIT(INT_FIFO_FULL) | BIT(INT_CMPL) | BIT(INT_ARB_LOSE))

#define I2C_READ(reg)                 readl(ctlr->iobase + (reg))
#define I2C_WRITE(reg, val)           writel((val), ctlr->iobase + (reg))
#define I2C_UPDATE(reg, mask, val)    writel(((readl(ctlr->iobase + (reg)) \
				      & ~(mask)) | ((val) & (mask))), ctlr->iobase + (reg))

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
	AL_AXI_I2C_7BIT = 0,
	AL_AXI_I2C_10BIT = 1
};

static const u32 fifo_depth_map[] = {2, 4, 8, 16};

struct anlogic_i2c_controller {
	struct i2c_adapter adapter;
	void __iomem *iobase;
	struct device *dev;
	u32 fifo_size;
	int irq;

	struct completion *completion;   /* Synchronization for transfer completion */
	struct i2c_msg *msgs;            /* Pointer to the array of I2C messages */
	int num_msgs;                    /* Total number of I2C messages */
	int cur_msg_idx;                 /* Index of the current I2C message */
	u32 tx_index;                    /* Index of the current byte in TX buffer */
	u32 rx_index;                    /* Index of the current byte in RX buffer */
	u32 xfer_error;                  /* Transfer error status */

	/* I2C bus configuration */
	u32 clk_freq;                    /* I2C clock frequency */
	u32 pulse_width_ns;              /* Pulse width in nanoseconds */
	u32 sda_setup_ns;                /* SDA setup time in nanoseconds */
	u32 sda_hold_ns;                 /* SDA hold time in nanoseconds */
	u32 input_frequency;             /* Input clock frequency */
};

static inline void anlogic_i2c_modify_int(struct anlogic_i2c_controller *ctlr,
					  u32 mask, bool enable)
{
	u32 val = enable ? mask : 0;

	I2C_UPDATE(AXI_I2C_INTEN, mask, val);
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
	I2C_WRITE(AXI_I2C_SR, mask);
}

static void anlogic_i2c_handle_errors(struct anlogic_i2c_controller *ctlr)
{
	u32 int_err = ctlr->xfer_error;

	if (int_err & BIT(SR_ARB_LOSE))
		dev_err(ctlr->dev, "Arbitration Lost Error\n");
}

static void anlogic_i2c_start_xfer(struct anlogic_i2c_controller *ctlr,
				  struct i2c_msg *msg, u32 mask)
{
	u32 cr_val, setup_val, addr_val;

	/* === 1. Configure Control Register (AXI_I2C_CR) === */
	cr_val = I2C_READ(AXI_I2C_CR);

	/* Set transmission phases: START, ADDRESS, and STOP phases */
	cr_val |= BIT(CR_PHASE_START) | BIT(CR_PHASE_ADDR) | BIT(CR_PHASE_STOP);

	/* Conditionally enable the DATA phase based on message length */
	cr_val = (msg->len > 0) ? (cr_val | BIT(CR_PHASE_DATA)) : (cr_val & ~BIT(CR_PHASE_DATA));

	/* Set the transfer direction */
	if (msg->flags & I2C_M_RD)
		cr_val |= (MASTER_RX_SLAVE_TX << CR_DIR);
	else
		cr_val &= ~(MASTER_RX_SLAVE_TX << CR_DIR);

	/* Set the data count */
	cr_val &= ~ANLOGIC_I2C_DATA_CNT;
	cr_val |= FIELD_PREP(ANLOGIC_I2C_DATA_CNT, msg->len);

	/* Updated control register */
	I2C_WRITE(AXI_I2C_CR, cr_val);

	/* === 2. Configure Address Mode (AXI_I2C_SETUP) === */
	setup_val = I2C_READ(AXI_I2C_SETUP);

	/* Check if the transfer uses 10-bit addressing mode */
	if (msg->flags & I2C_M_TEN)
		setup_val |= (AL_AXI_I2C_10BIT << I2C_ADDRESS_MODE);
	else
		setup_val &= ~(AL_AXI_I2C_10BIT << I2C_ADDRESS_MODE);

	/* Updated setup register value */
	I2C_WRITE(AXI_I2C_SETUP, setup_val);

	/* === 3. Set Slave Address (AXI_I2C_ADDR) === */
	addr_val = FIELD_PREP(ANLOGIC_I2C_SLAVE_ADDR_VAL, msg->addr);
	I2C_WRITE(AXI_I2C_ADDR, addr_val);

	/* === 4. Clear Interrupts and Enable Interrupts === */
	anlogic_i2c_clear_int(ctlr, mask);
	anlogic_i2c_enable_int(ctlr, mask);

	/* === 5. Start the Transmission (AXI_I2C_CMD) === */
	I2C_WRITE(AXI_I2C_CMD, AL_AXI_I2C_CMD_TRANS);
}

static void anlogic_i2c_reset_xfer(struct anlogic_i2c_controller *ctlr)
{
	ctlr->xfer_error = 0;
	ctlr->tx_index = 0;
	ctlr->rx_index = 0;
}

static int anlogic_i2c_master_xfer(struct i2c_adapter *adap,
				   struct i2c_msg *msgs, int num)
{
	struct anlogic_i2c_controller *ctlr = i2c_get_adapdata(adap);
	DECLARE_COMPLETION_ONSTACK(done);
	int ret = num;

	ctlr->completion = &done;
	ctlr->num_msgs = num;
	ctlr->msgs = msgs;
	ctlr->cur_msg_idx = 0;

	for (ctlr->cur_msg_idx = 0; ctlr->cur_msg_idx < ctlr->num_msgs;
		ctlr->cur_msg_idx++) {
		struct i2c_msg *current_msg = &ctlr->msgs[ctlr->cur_msg_idx];
		/* Reset ctlr related indices */
		anlogic_i2c_reset_xfer(ctlr);
		reinit_completion(ctlr->completion);

		/* Start the transfer with address hit and arbitration lost interrupt */
		anlogic_i2c_start_xfer(ctlr, current_msg, I2C_INIT_INT);

		/* Wait for the current message transfer to complete */
		if (!wait_for_completion_timeout(ctlr->completion, adap->timeout)) {
			anlogic_i2c_disable_int(ctlr, ANLOGIC_I2C_INT_ALL);
			anlogic_i2c_clear_int(ctlr, ANLOGIC_I2C_INT_ALL);
			synchronize_irq(ctlr->irq);
			i2c_recover_bus(&ctlr->adapter);
			ret = -EIO;
			goto out;
		}

		if (ctlr->xfer_error) {
			anlogic_i2c_handle_errors(ctlr);
			ret = -EIO;
			break;
		}
	}

out:
	anlogic_i2c_reset_xfer(ctlr);
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

static void anlogic_i2c_read_fifo(struct anlogic_i2c_controller *ctlr)
{
	struct i2c_msg *current_msg = &ctlr->msgs[ctlr->cur_msg_idx];
	unsigned char *buf = current_msg->buf;
	u32 remaining_length  = current_msg->len - ctlr->rx_index;
	u32 count = min(remaining_length, ctlr->fifo_size);

	/* Read data from the FIFO */
	while (count--)
		buf[ctlr->rx_index++] = I2C_READ(AXI_I2C_DATA);

	/* Disable FIFO FULL interrupt if all data has been received */
	if (ctlr->rx_index == current_msg->len)
		anlogic_i2c_disable_int(ctlr, BIT(INT_FIFO_FULL));
}

static void anlogic_i2c_write_fifo(struct anlogic_i2c_controller *ctlr)
{
	struct i2c_msg *current_msg = &ctlr->msgs[ctlr->cur_msg_idx];
	unsigned char *buf = current_msg->buf;
	u32 remaining_length  = current_msg->len - ctlr->tx_index;
	u32 count = min(remaining_length, ctlr->fifo_size);

	/* Write data to the FIFO */
	while (count--)
		I2C_WRITE(AXI_I2C_DATA, buf[ctlr->tx_index++]);

	/* Disable FIFO EMPTY interrupt if all data has been transmitted */
	if (ctlr->tx_index == current_msg->len)
		anlogic_i2c_disable_int(ctlr, BIT(INT_FIFO_EMPTY));
}

static irqreturn_t anlogic_i2c_irq(int irq, void *context)
{
	struct anlogic_i2c_controller *ctlr = context;
	struct i2c_msg *current_msg;
	u32 int_stat, int_mask;

	if (!ctlr->completion)
		return IRQ_NONE;

	current_msg = &ctlr->msgs[ctlr->cur_msg_idx];

	/* Process only enabled interrupts */
	int_stat = I2C_READ(AXI_I2C_SR);
	int_mask = I2C_READ(AXI_I2C_INTEN);
	int_stat &= int_mask;
	anlogic_i2c_clear_int(ctlr, int_stat);

	/* 1. Handle error interrupt: Arbitration lost (high priority) */
	if (int_stat & BIT(SR_ARB_LOSE)) {
		anlogic_i2c_disable_int(ctlr, BIT(INT_ARB_LOSE));
		ctlr->xfer_error = int_stat;
		complete(ctlr->completion);
		return IRQ_HANDLED;
	}

	/* 2. Handle Address Hit interrupt */
	if (int_stat & BIT(SR_ADDR_HIT)) {
		anlogic_i2c_disable_int(ctlr, BIT(INT_ADDR_HIT));

		if (current_msg->len == 0) {
			complete(ctlr->completion);
			return IRQ_HANDLED;
		}

		anlogic_i2c_enable_int(ctlr, current_msg->flags & I2C_M_RD ?
			I2C_RECV_INT : I2C_SEND_INT);

		return IRQ_HANDLED;
	}

	/* 3. Handle FIFO Full interrupt: Receive data */
	if (int_stat & BIT(SR_FIFO_FULL)) {
		anlogic_i2c_read_fifo(ctlr);

	/* 4. Handle FIFO Empty interrupt: Transmit data */
	} else if (int_stat & BIT(SR_FIFO_EMPTY)) {
		anlogic_i2c_write_fifo(ctlr);
	}

	/* 5. Handle Completion interrupt */
	if (int_stat & BIT(SR_CMPL)) {
		/* In read operation, ensure any remaining data */
		if (current_msg->flags & I2C_M_RD)
			anlogic_i2c_read_fifo(ctlr);

		anlogic_i2c_disable_int(ctlr, current_msg->flags & I2C_M_RD ?
					I2C_RECV_INT : I2C_SEND_INT);

		complete(ctlr->completion);
	}

	return IRQ_HANDLED;
}

static void anlogic_deinit(struct anlogic_i2c_controller *ctlr)
{
	/* Reset axi i2c */
	I2C_WRITE(AXI_I2C_CFG, AL_AXI_I2C_CMD_RST);

	/* Disable axi i2c */
	I2C_UPDATE(AXI_I2C_SETUP, I2C_EN, false);
}

static void anlogic_i2c_set_scl(struct anlogic_i2c_controller *ctlr)
{
	u64 TPM = 0;
	u64 setup_val = 0;
	u64 SCL_HIGH_period;
	u64 T_SP, T_SUDAT, T_HDDAT, T_SCLHi;

	/* PulseWidth = T_SP * taclk * (TPM + 1)
	 * T_SP = PulseWidth / taclk / (TPM + 1)
	 */
	T_SP = ctlr->pulse_width_ns * (u64)ctlr->input_frequency /
	       (u64)1000000000 / (TPM + 1);

	/* SetupTime = (2 * taclk) + (2 + T_SP + T_SUDAT) * taclk * (TPM + 1);
	 * T_SUDAT = ((SetupTime - (2 * taclk)) / taclk / (TPM + 1)) - 2 - T_SP
	 */
	T_SUDAT = DIV_ROUND_CLOSEST(ctlr->sda_setup_ns *
				   (u64)ctlr->input_frequency,
				   (u64)1000000000 * (u64)(TPM + 1)) -
		  DIV_ROUND_CLOSEST(2, (u64)(TPM + 1)) - 2 - T_SP;

	/* HoldTime = (2 * taclk) + (2 + T_SP + T_HDDAT) * taclk * (TPM + 1)
	 * T_HDDAT = ((HoldTime - (2 * taclk)) / taclk / (TPM + 1)) - 2 - T_SP;
	 */
	T_HDDAT = DIV_ROUND_CLOSEST(ctlr->sda_hold_ns *
				   (u64)ctlr->input_frequency,
				   (u64)1000000000 * (u64)(TPM + 1)) -
		  DIV_ROUND_CLOSEST(2, (u64)(TPM + 1)) - 2 - T_SP;

	/* SCL_HIGH_period = (2 * taclk) + (2 + T_SP + T_SCLHi) * taclk * (TPM + 1)
	 * T_SCLHi = ((SCL_HIGH_period - (2 * taclk)) / taclk / (TPM + 1)) - 2 - T_SP;
	 */
	if (ctlr->clk_freq == 1000000)
		SCL_HIGH_period = 500;
	else if (ctlr->clk_freq == 400000)
		SCL_HIGH_period = 1300;
	else
		SCL_HIGH_period = 5000;

	T_SCLHi = DIV_ROUND_CLOSEST(SCL_HIGH_period *
				   (u64)ctlr->input_frequency,
				   (u64)1000000000 * (u64)(TPM + 1)) -
		  DIV_ROUND_CLOSEST(2, (u64)(TPM + 1)) - 2 - T_SP;

	setup_val |= ((T_SP << T_SP_SHIFT) | (T_SUDAT << T_SUDAT_SHIFT) |
		      (T_HDDAT << T_HDDAT_SHIFT) | (T_SCLHi << T_SCL_HIGH));

	/* Set the low and high level time of scl to be equal */
	setup_val &= ~BIT(T_SCL_RATIO);

	/* Write the config value to setup register  */
	I2C_WRITE(AXI_I2C_SETUP, setup_val);
}

static void anlogic_i2c_configure_bus(struct anlogic_i2c_controller *ctlr)
{
	u32 ctlr_id;

	/* === 1. Reset the AXI I2C controller === */
	I2C_WRITE(AXI_I2C_CFG, AL_AXI_I2C_CMD_RST);

	/* === 2. Read and print the AXI I2C controller ID === */
	ctlr_id = (I2C_READ(AXI_I2C_ID) & 0xFFFFFF00) >> 8;
	dev_info(ctlr->dev, "Anlogic AXI I2C controller ID: 0x%x\n", ctlr_id);

	/* === 3. Configure I2C clock timing === */
	anlogic_i2c_set_scl(ctlr);

	/* === 4. Enable master mode and enable the I2C module === */
	I2C_UPDATE(AXI_I2C_SETUP, BIT(I2C_MODE), BIT(I2C_MODE));  /* Enable master mode */
	I2C_UPDATE(AXI_I2C_SETUP, BIT(I2C_EN), BIT(I2C_EN));      /* Enable I2C IP */
}

static int anlogic_i2c_probe(struct platform_device *pdev)
{
	struct anlogic_i2c_controller *ctlr;
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	struct i2c_adapter *adapter;
	u32 cfg_value;
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

	ret = devm_request_irq(dev, ctlr->irq, anlogic_i2c_irq, 0, "anlogic-axi-i2c", ctlr);
	if (ret) {
		dev_err(dev, "failed to request irq handler, ret = %d\n", ret);
		return ret;
	}

	/* Read required properties */
	if (of_property_read_u32(np, "input-frequency", &ctlr->input_frequency) ||
	    of_property_read_u32(np, "clock-frequency", &ctlr->clk_freq) ||
	    of_property_read_u32(np, "i2c-filter-width-time-ns", &ctlr->pulse_width_ns) ||
	    of_property_read_u32(np, "i2c-sda-setup-time-ns", &ctlr->sda_setup_ns) ||
	    of_property_read_u32(np, "i2c-sda-hold-time-ns", &ctlr->sda_hold_ns)) {
		dev_err(dev, "Failed to read required properties\n");
		return -EINVAL;
	}

	anlogic_i2c_configure_bus(ctlr);

	/* Read FIFO size */
	cfg_value = I2C_READ(AXI_I2C_CFG);
	ctlr->fifo_size = fifo_depth_map[cfg_value];

	adapter = &ctlr->adapter;
	snprintf(adapter->name, sizeof(adapter->name),
		"anlogic axi i2c controller %s", dev_name(dev));
	adapter->owner = THIS_MODULE;
	adapter->algo = &anlogic_i2c_algo;
	adapter->dev.parent = dev;
	adapter->dev.of_node = np;
	adapter->timeout = I2C_TIMEOUT;
	adapter->nr = -1;

	i2c_set_adapdata(adapter, ctlr);

	ret = devm_i2c_add_adapter(dev, adapter);
	if (ret)
		anlogic_deinit(ctlr);

	return ret;
}

static int anlogic_i2c_remove(struct platform_device *pdev)
{
	struct anlogic_i2c_controller *ctlr = platform_get_drvdata(pdev);

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
