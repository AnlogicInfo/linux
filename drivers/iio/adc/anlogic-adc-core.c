// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2023 Anlogic, Inc.
 *
*/

#include <linux/clk.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/sysfs.h>

#include <linux/iio/buffer.h>
#include <linux/iio/events.h>
#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include <linux/iio/trigger.h>
#include <linux/iio/trigger_consumer.h>
#include <linux/iio/triggered_buffer.h>

#include <linux/ktime.h>
#include <linux/hrtimer.h>
#include <linux/delay.h>


#include "anlogic-adc.h"

static const unsigned int ADC_FPSOC_UNMASK_TIMEOUT = 500;

/* FPSOC register definitions */
#define ADC_FPSOC_REG_CFG	0x00
#define ADC_FPSOC_REG_INTSTS	0x04
#define ADC_FPSOC_REG_INTMSK	0x08
#define ADC_FPSOC_REG_STATUS	0x0c
#define ADC_FPSOC_REG_CFIFO	0x10
#define ADC_FPSOC_REG_DFIFO	0x14
#define ADC_FPSOC_REG_CTL		0x18

#define ADC_FPSOC_CFG_ENABLE		BIT(31)
#define ADC_FPSOC_CFG_CFIFOTH_MASK	(0xf << 24)
#define ADC_FPSOC_CFG_CFIFOTH_OFFSET	24
#define ADC_FPSOC_CFG_DFIFOTH_MASK	(0xf << 20)
#define ADC_FPSOC_CFG_DFIFOTH_OFFSET	20
#define ADC_FPSOC_CFG_WEDGE		BIT(30)
#define ADC_FPSOC_CFG_REDGE		BIT(29)
#define ADC_FPSOC_CFG_TCKRATE_MASK	(0x3 << 10)
#define ADC_FPSOC_CFG_TCKRATE_DIV2	(0x0 << 10)
#define ADC_FPSOC_CFG_TCKRATE_DIV4	(0x1 << 10)
#define ADC_FPSOC_CFG_TCKRATE_DIV8	(0x2 << 10)
#define ADC_FPSOC_CFG_TCKRATE_DIV16	(0x3 << 10)
#define ADC_FPSOC_CFG_IGAP_MASK		0x1f
#define ADC_FPSOC_CFG_IGAP(x)		(x)

#define ADC_FPSOC_INT_PS_ADC		BIT(0)
#define ADC_FPSOC_INT_DFIFO_LTH		BIT(8)
#define ADC_FPSOC_INT_CFIFO_LTH		BIT(9)
#define ADC_FPSOC_INT_DFIFO_GTH		BIT(10)
#define ADC_FPSOC_INT_CFIFO_GTH		BIT(11)

#define ADC_FPSOC_STATUS_CFIFO_LVL_MASK	(0xf << 16)
#define ADC_FPSOC_STATUS_CFIFO_LVL_OFFSET	16
#define ADC_FPSOC_STATUS_DFIFO_LVL_MASK	(0xf << 12)
#define ADC_FPSOC_STATUS_DFIFO_LVL_OFFSET	12
#define ADC_FPSOC_STATUS_CFIFOF		BIT(11)
#define ADC_FPSOC_STATUS_CFIFOE		BIT(10)
#define ADC_FPSOC_STATUS_DFIFOF		BIT(9)
#define ADC_FPSOC_STATUS_DFIFOE		BIT(8)

#define ADC_FPSOC_CTL_CFIFO_FLUSH		BIT(0)
#define ADC_FPSOC_CTL_DFIFO_FLUSH		BIT(1)
#define ADC_FPSOC_CTL_RESET		BIT(4)

#define ADC_FPSOC_CMD_READ		0x78
#define ADC_FPSOC_CMD_WRITE		0x77

#define ADC_FPSOC_CMD(cmd, addr, data) (((cmd) << 24) | ((addr) << 16) | (data))

#define ADC_FLAGS_BUFFERED BIT(0)


/* AXI register definitions */
#define PLS_PROT				(0xF8800080)
#define ADC_AXI_ADC_REG_OFFSET	(0x80004000)

#define ADC_MAX_SAMPLERATE 15000

static void adc_write_reg(struct adc *adc, unsigned int reg,
	uint32_t val)
{
	writel(val, adc->base + reg);
}

static void adc_read_reg(struct adc *adc, unsigned int reg,
	uint32_t *val)
{
	*val = readl(adc->base + reg);
}

/*
 * The FPSOC interface uses two asynchronous FIFOs for communication with the
 * ADC. Reads and writes to the ADC register are performed by submitting a
 * request to the command FIFO (CFIFO), once the request has been completed the
 * result can be read from the data FIFO (DFIFO). The method currently used in
 * this driver is to submit the request for a read/write operation, then go to
 * sleep and wait for an interrupt that signals that a response is available in
 * the data FIFO.
 */

static void adc_fpsoc_write_fifo(struct adc *adc, uint32_t *cmd,
	unsigned int n)
{
	unsigned int i;

	for (i = 0; i < n; i++)
		adc_write_reg(adc, ADC_FPSOC_REG_CFIFO, cmd[i]);
}

static void adc_fpsoc_drain_fifo(struct adc *adc)
{
	uint32_t status, tmp;

	adc_read_reg(adc, ADC_FPSOC_REG_STATUS, &status);

	while (!(status & ADC_FPSOC_STATUS_DFIFOE)) {
		adc_read_reg(adc, ADC_FPSOC_REG_DFIFO, &tmp);
		adc_read_reg(adc, ADC_FPSOC_REG_STATUS, &status);
	}
}

static void adc_fpsoc_update_intmsk(struct adc *adc, unsigned int mask,
	unsigned int val)
{
	adc->fpsoc_intmask &= ~mask;
	adc->fpsoc_intmask |= val;

	adc_write_reg(adc, ADC_FPSOC_REG_INTMSK, adc->fpsoc_intmask);
}

static int adc_fpsoc_write_adc_reg(struct adc *adc, unsigned int reg,
	uint16_t val)
{
	uint32_t cmd[1];
	uint32_t tmp;
	int ret;

	spin_lock_irq(&adc->lock);
	adc_fpsoc_update_intmsk(adc, ADC_FPSOC_INT_DFIFO_GTH,
			ADC_FPSOC_INT_DFIFO_GTH);

	reinit_completion(&adc->completion);

	cmd[0] = ADC_FPSOC_CMD(ADC_FPSOC_CMD_WRITE, reg, val);
	adc_fpsoc_write_fifo(adc, cmd, ARRAY_SIZE(cmd));
	adc_read_reg(adc, ADC_FPSOC_REG_CFG, &tmp);
	tmp &= ~ADC_FPSOC_CFG_DFIFOTH_MASK;
	tmp |= 0 << ADC_FPSOC_CFG_DFIFOTH_OFFSET;
	adc_write_reg(adc, ADC_FPSOC_REG_CFG, tmp);

	adc_fpsoc_update_intmsk(adc, ADC_FPSOC_INT_DFIFO_GTH, 0);
	spin_unlock_irq(&adc->lock);

	ret = wait_for_completion_interruptible_timeout(&adc->completion, HZ);
	if (ret == 0)
		ret = -EIO;
	else
		ret = 0;

	adc_read_reg(adc, ADC_FPSOC_REG_DFIFO, &tmp);

	return ret;
}

static int adc_fpsoc_read_adc_reg(struct adc *adc, unsigned int reg,
	uint16_t *val)
{
	uint32_t cmd[1];
	uint32_t resp;
	uint32_t tmp;
	int ret;

	cmd[0] = ADC_FPSOC_CMD(ADC_FPSOC_CMD_READ, reg, 0);

	spin_lock_irq(&adc->lock);
	adc_fpsoc_update_intmsk(adc, ADC_FPSOC_INT_DFIFO_GTH,
			ADC_FPSOC_INT_DFIFO_GTH);
	adc_fpsoc_drain_fifo(adc);
	reinit_completion(&adc->completion);

	adc_fpsoc_write_fifo(adc, cmd, ARRAY_SIZE(cmd));
	adc_read_reg(adc, ADC_FPSOC_REG_CFG, &tmp);
	tmp &= ~ADC_FPSOC_CFG_DFIFOTH_MASK;
	tmp |= 1 << ADC_FPSOC_CFG_DFIFOTH_OFFSET;
	adc_write_reg(adc, ADC_FPSOC_REG_CFG, tmp);

	adc_fpsoc_update_intmsk(adc, ADC_FPSOC_INT_DFIFO_GTH, 0);
	spin_unlock_irq(&adc->lock);
	ret = wait_for_completion_interruptible_timeout(&adc->completion, HZ);
	if (ret == 0)
		ret = -EIO;
	if (ret < 0)
		return ret;

	udelay(25);
	adc_read_reg(adc, ADC_FPSOC_REG_DFIFO, &resp);

	*val = resp & 0xffff;

	return 0;
}

static irqreturn_t adc_fpsoc_interrupt_handler(int irq, void *devid)
{
	struct iio_dev *indio_dev = devid;
	struct adc *adc = iio_priv(indio_dev);
	uint32_t status;

	adc_read_reg(adc, ADC_FPSOC_REG_INTSTS, &status);
	status &= ~(adc->fpsoc_intmask);

	if (!status)
		return IRQ_NONE;

	spin_lock(&adc->lock);

	adc_write_reg(adc, ADC_FPSOC_REG_INTSTS, status);

	if (status & ADC_FPSOC_INT_DFIFO_GTH) {
		adc_fpsoc_update_intmsk(adc, ADC_FPSOC_INT_DFIFO_GTH,
			ADC_FPSOC_INT_DFIFO_GTH);
		complete(&adc->completion);
	}

	spin_unlock(&adc->lock);
	return IRQ_HANDLED;
}

#define ADC_FPSOC_TCK_RATE_MAX 50000000
#define ADC_FPSOC_IGAP_DEFAULT 20
#define ADC_FPSOC_PCAP_RATE_MAX 200000000

static int adc_fpsoc_setup(struct platform_device *pdev,
	struct iio_dev *indio_dev, int irq)
{
	struct adc *adc = iio_priv(indio_dev);
	unsigned long pcap_rate;
	unsigned int tck_div;
	unsigned int div;
	unsigned int igap;
	unsigned int tck_rate;
	int ret;
	uint32_t tmp;

	igap = ADC_FPSOC_IGAP_DEFAULT;
	tck_rate = ADC_FPSOC_TCK_RATE_MAX;

	adc->fpsoc_intmask = 0xbff;

	pcap_rate = clk_get_rate(adc->clk);
	if (!pcap_rate)
		return -EINVAL;

	if (pcap_rate > ADC_FPSOC_PCAP_RATE_MAX) {
		ret = clk_set_rate(adc->clk,
				   (unsigned long)ADC_FPSOC_PCAP_RATE_MAX);
		if (ret)
			return ret;
	}

	if (tck_rate > pcap_rate / 2) {
		div = 2;
	} else {
		div = pcap_rate / tck_rate;
		if (pcap_rate / div > ADC_FPSOC_TCK_RATE_MAX)
			div++;
	}

	if (div <= 3)
		tck_div = ADC_FPSOC_CFG_TCKRATE_DIV2;
	else if (div <= 7)
		tck_div = ADC_FPSOC_CFG_TCKRATE_DIV4;
	else if (div <= 15)
		tck_div = ADC_FPSOC_CFG_TCKRATE_DIV8;
	else
		tck_div = ADC_FPSOC_CFG_TCKRATE_DIV16;

	adc_write_reg(adc, ADC_FPSOC_REG_CTL, ADC_FPSOC_CTL_RESET |
			ADC_FPSOC_CTL_CFIFO_FLUSH | ADC_FPSOC_CTL_DFIFO_FLUSH);
	adc_write_reg(adc, ADC_FPSOC_REG_CTL, 0);
	adc_write_reg(adc, ADC_FPSOC_REG_INTSTS, ~0);
	adc_write_reg(adc, ADC_FPSOC_REG_INTMSK, adc->fpsoc_intmask);

	adc_read_reg(adc, ADC_FPSOC_REG_CFG, &tmp);
	tmp |= ADC_FPSOC_CFG_ENABLE | ADC_FPSOC_CFG_REDGE |
		ADC_FPSOC_CFG_WEDGE | tck_div | ADC_FPSOC_CFG_IGAP(igap);
	adc_write_reg(adc, ADC_FPSOC_REG_CFG, tmp);

	if (pcap_rate > ADC_FPSOC_PCAP_RATE_MAX) {
		ret = clk_set_rate(adc->clk, pcap_rate);
		if (ret)
			return ret;
	}

	return 0;
}

static unsigned long adc_fpsoc_get_dclk_rate(struct adc *adc)
{
	unsigned int div;
	uint32_t val;

	adc_read_reg(adc, ADC_FPSOC_REG_CFG, &val);

	switch (val & ADC_FPSOC_CFG_TCKRATE_MASK) {
	case ADC_FPSOC_CFG_TCKRATE_DIV4:
		div = 4;
		break;
	case ADC_FPSOC_CFG_TCKRATE_DIV8:
		div = 8;
		break;
	case ADC_FPSOC_CFG_TCKRATE_DIV16:
		div = 16;
		break;
	default:
		div = 2;
		break;
	}

	return clk_get_rate(adc->clk) / div;
}

static const struct adc_ops adc_fpsoc_ops = {
	.read = adc_fpsoc_read_adc_reg,
	.write = adc_fpsoc_write_adc_reg,
	.setup = adc_fpsoc_setup,
	.get_dclk_rate = adc_fpsoc_get_dclk_rate,
	.interrupt_handler = adc_fpsoc_interrupt_handler,
};

static int adc_axi_read_adc_reg(struct adc *adc, unsigned int reg,
	uint16_t *val)
{
	uint32_t val32;

	adc_read_reg(adc, ADC_AXI_ADC_REG_OFFSET + reg, &val32);
	*val = val32 & 0xffff;

	return 0;
}

static int adc_axi_write_adc_reg(struct adc *adc, unsigned int reg,
	uint16_t val)
{
	adc_write_reg(adc, ADC_AXI_ADC_REG_OFFSET + reg, val);

	return 0;
}

static int adc_axi_setup(struct platform_device *pdev,
	struct iio_dev *indio_dev, int irq)
{
	return 0;
}

static int _adc_update_adc_reg(struct adc *adc, unsigned int reg,
	uint16_t mask, uint16_t val)
{
	uint16_t tmp;
	int ret;

	ret = _adc_read_adc_reg(adc, reg, &tmp);
	if (ret)
		return ret;

	return _adc_write_adc_reg(adc, reg, (tmp & ~mask) | val);
}

static int adc_update_adc_reg(struct adc *adc, unsigned int reg,
	uint16_t mask, uint16_t val)
{
	int ret;

	mutex_lock(&adc->mutex);
	ret = _adc_update_adc_reg(adc, reg, mask, val);
	mutex_unlock(&adc->mutex);

	return ret;
}

static irqreturn_t adc_axi_interrupt_handler(int irq, void *devid)
{
	struct iio_dev *indio_dev = devid;
	struct adc *adc = iio_priv(indio_dev);
	uint32_t status, mask;
	uint16_t intr_done_flag;

	adc_read_reg(adc, ADC_FPSOC_REG_INTSTS, &status);
	adc_read_reg(adc, ADC_FPSOC_REG_INTMSK, &mask);
	status &= ~mask;

	if (!status)
		return IRQ_NONE;

	adc_fpsoc_read_adc_reg(adc, ADC_REG_CONFIG1, &intr_done_flag);
	if ((intr_done_flag & ADC_CONFIG1_INTR_DONE) && adc->trigger) {
		adc_update_adc_reg(adc, ADC_REG_CONFIG1, ADC_CONFIG1_INTR_DONE_MASK,
				ADC_CONFIG1_INTR_DONE_MASK);
		adc_fpsoc_read_adc_reg(adc, ADC_REG_CONFIG1, &intr_done_flag);
		iio_trigger_poll(adc->trigger);
	}

	adc_write_reg(adc, ADC_FPSOC_REG_INTSTS, status);

	return IRQ_HANDLED;
}

static unsigned long adc_axi_get_dclk(struct adc *adc)
{
	return clk_get_rate(adc->clk);
}

static const struct adc_ops adc_axi_ops = {
	.read = adc_axi_read_adc_reg,
	.write = adc_axi_write_adc_reg,
	.setup = adc_axi_setup,
	.get_dclk_rate = adc_axi_get_dclk,
	.interrupt_handler = adc_axi_interrupt_handler,
	.flags = ADC_FLAGS_BUFFERED,
};

static unsigned long adc_get_dclk_rate(struct adc *adc)
{
	return adc->ops->get_dclk_rate(adc);
}

static int adc_update_scan_mode(struct iio_dev *indio_dev,
	const unsigned long *mask)
{
	struct adc *adc = iio_priv(indio_dev);
	unsigned int n;

	n = bitmap_weight(mask, indio_dev->masklength);

	kfree(adc->data);
	adc->data = kcalloc(n, sizeof(*adc->data), GFP_KERNEL);
	if (!adc->data)
		return -ENOMEM;

	return 0;
}

static unsigned int adc_scan_index_to_channel(unsigned int scan_index)
{
	return ADC_REG_VAUX(scan_index);
}

static irqreturn_t adc_trigger_handler(int irq, void *p)
{
	struct iio_poll_func *pf = p;
	struct iio_dev *indio_dev = pf->indio_dev;
	struct adc *adc = iio_priv(indio_dev);
	unsigned int chan;
	int i, j;

	if (!adc->data)
		goto out;

	j = 0;
	for_each_set_bit(i, indio_dev->active_scan_mask,
		indio_dev->masklength) {
		chan = adc_scan_index_to_channel(i);
		adc_read_adc_reg(adc, chan, &adc->data[j]);
		j++;
	}

	iio_push_to_buffers(indio_dev, adc->data);

out:
	iio_trigger_notify_done(indio_dev->trig);

	return IRQ_HANDLED;
}

static int adc_trigger_set_state(struct iio_trigger *trigger, bool state)
{
	struct adc *adc = iio_trigger_get_drvdata(trigger);
	unsigned int convst;
	int ret = 0;

	mutex_lock(&adc->mutex);

	if (state) {
		/* Only one of the two triggers can be active at a time. */
		if (adc->trigger != NULL) {
			ret = -EBUSY;
			goto err_out;
		} else {
			adc->trigger = trigger;
			if (trigger == adc->convst_trigger)
				convst = ADC_CONFIG3_REG_ADC_SOC;
			else
				convst = 0;
		}

		ret = _adc_update_adc_reg(adc, ADC_REG_CONFIG3,
					ADC_CONFIG3_REG_ADC_SOC_MASK, convst);
		if (ret)
			goto err_out;
	} else {
		adc->trigger = NULL;
	}

err_out:
	mutex_unlock(&adc->mutex);

	return ret;
}

static const struct iio_trigger_ops adc_trigger_ops = {
	.set_trigger_state = &adc_trigger_set_state,
};

static struct iio_trigger *adc_alloc_trigger(struct iio_dev *indio_dev,
	const char *name)
{
	struct iio_trigger *trig;
	int ret;

	trig = iio_trigger_alloc("%s%d-%s", indio_dev->name,
				indio_dev->id, name);
	if (trig == NULL)
		return ERR_PTR(-ENOMEM);

	trig->dev.parent = indio_dev->dev.parent;
	trig->ops = &adc_trigger_ops;
	iio_trigger_set_drvdata(trig, iio_priv(indio_dev));

	ret = iio_trigger_register(trig);
	if (ret)
		goto error_free_trig;

	return trig;

error_free_trig:
	iio_trigger_free(trig);
	return ERR_PTR(ret);
}

static int adc_postdisable(struct iio_dev *indio_dev)
{
	struct adc *adc = iio_priv(indio_dev);
	int ret;

	ret = adc_update_adc_reg(adc, ADC_REG_CONFIG3, ADC_CONFIG3_MODE_SEL_MASK,
		ADC_CONFIG3_CONV_MODE_SINGELE_PASS);
	if (ret)
		return ret;

	return ret;
}

static int adc_preenable(struct iio_dev *indio_dev)
{
 	struct adc *adc = iio_priv(indio_dev);
 	int ret;

 	ret = adc_update_adc_reg(adc, ADC_REG_CONFIG3, ADC_CONFIG3_MODE_SEL_MASK,
 		ADC_CONFIG3_CONV_MODE_CONTINUOUS);
	if (ret)
		goto err;

	return 0;
 err:
 	adc_postdisable(indio_dev);
 	return ret;
}

static const struct iio_buffer_setup_ops adc_buffer_ops = {
	.preenable = &adc_preenable,
	.postdisable = &adc_postdisable,
};

static int adc_read_samplerate(struct adc *adc)
{
	unsigned int div;
	uint16_t val16;
	int ret;

	ret = adc_read_adc_reg(adc, ADC_REG_CONFIG2, &val16);
	if (ret)
		return ret;

	div = (val16 & ADC_CONFIG2_ADC_CLK_DIV_MASK) >> ADC_CONFIG2_CLK_DIV_SHIFT;

	if (div == 0)
		div = 1;
	else
		div = 2 * div;

	return adc_get_dclk_rate(adc) / div / ( adc->resolution + 2);
}

static int adc_read_raw(struct iio_dev *indio_dev,
	struct iio_chan_spec const *chan, int *val, int *val2, long info)
{
	struct adc *adc = iio_priv(indio_dev);
	uint16_t val16;
	uint16_t flag;
	int ret;

	switch (info) {
	case IIO_CHAN_INFO_RAW:
		if (iio_buffer_enabled(indio_dev))
			return -EBUSY;

		adc_update_adc_reg(adc, ADC_REG_CONFIG3, ADC_CONFIG3_REG_ADC_SOC_MASK,
				ADC_CONFIG3_REG_ADC_SOC);
		do {
			adc_read_adc_reg(adc, ADC_REG_CONFIG1, &flag);
		} while (!(flag & ADC_CONFIG1_INTR_DONE));
		adc_update_adc_reg(adc, ADC_REG_CONFIG1, ADC_CONFIG1_INTR_DONE_MASK,
			ADC_CONFIG1_INTR_DONE_MASK);
		ret = adc_read_adc_reg(adc, chan->address, &val16);
		adc_update_adc_reg(adc, ADC_REG_CONFIG3, ADC_CONFIG3_REG_ADC_SOC_MASK,
				0 << ADC_CONFIG3_REG_SOC_SHIFT);

		if (ret < 0)
			return ret;
		val16 >>= 4;
		if (chan->scan_type.sign == 'u')
			*val = val16;
		else
			*val = sign_extend32(val16, 11);
		return IIO_VAL_INT;
	case IIO_CHAN_INFO_SAMP_FREQ:
		ret = adc_read_samplerate(adc);
		if (ret < 0)
			return ret;

		*val = ret;
		return IIO_VAL_INT;
	default:
		return -EINVAL;
	}
}

static int adc_write_samplerate(struct adc *adc, int val)
{
	unsigned long clk_rate = adc_get_dclk_rate(adc);
	unsigned int div;

	if (!clk_rate)
		return -EINVAL;

	if (val <= 0)
		return -EINVAL;

	/* Max. 150 kSPS */
	if (val > ADC_MAX_SAMPLERATE)
		val = ADC_MAX_SAMPLERATE;

	val *= (adc->resolution + 2);

	/* Min 1MHz */
	if (val < 1000000)
		val = 1000000;

	div = clk_rate / val;

	if (clk_rate / div / (adc->resolution + 2) > ADC_MAX_SAMPLERATE)
		div++;
	if (div == 0)
		div = 1;
	else if (div > 0xff)
		div = 0xff;

	return adc_update_adc_reg(adc, ADC_REG_CONFIG2, ADC_CONFIG2_ADC_CLK_DIV_MASK,
		div << ADC_CONFIG2_CLK_DIV_SHIFT);
}

static int adc_write_raw(struct iio_dev *indio_dev,
	struct iio_chan_spec const *chan, int val, int val2, long info)
{
	struct adc *adc = iio_priv(indio_dev);

	if (info != IIO_CHAN_INFO_SAMP_FREQ)
		return -EINVAL;

	return adc_write_samplerate(adc, val);
}

static const struct of_device_id adc_of_match_table[] = {
	{ .compatible = "anlogic,fpsoc-adc-1.00.a", (void *)&adc_fpsoc_ops },
};
MODULE_DEVICE_TABLE(of, adc_of_match_table);

#define ADC_CHAN_VOLTAGE(_chan, _scan_index, _addr) { \
	.type = IIO_VOLTAGE, \
	.indexed = 1, \
	.channel = (_chan), \
	.address = (_addr), \
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW), \
	.info_mask_shared_by_all = BIT(IIO_CHAN_INFO_SAMP_FREQ), \
	.scan_index = (_scan_index), \
	.scan_type = { \
		.sign = 'u', \
		.realbits = 12, \
		.storagebits = 16, \
		.shift = 4, \
		.endianness = IIO_CPU, \
	}, \
}

static const struct iio_chan_spec adc_channels[] = {
	ADC_CHAN_VOLTAGE(0, 0, ADC_REG_VAUX(0)),
	ADC_CHAN_VOLTAGE(1, 1, ADC_REG_VAUX(1)),
	ADC_CHAN_VOLTAGE(2, 2, ADC_REG_VAUX(2)),
	ADC_CHAN_VOLTAGE(3, 3, ADC_REG_VAUX(3)),
	ADC_CHAN_VOLTAGE(4, 4, ADC_REG_VAUX(4)),
	ADC_CHAN_VOLTAGE(5, 5, ADC_REG_VAUX(5)),
	ADC_CHAN_VOLTAGE(6, 6, ADC_REG_VAUX(6)),
	ADC_CHAN_VOLTAGE(7, 7, ADC_REG_VAUX(7)),
};

static const struct iio_info adc_info = {
	.read_raw = &adc_read_raw,
	.write_raw = &adc_write_raw,
	.update_scan_mode = &adc_update_scan_mode,
};

static int adc_parse_dt(struct iio_dev *indio_dev, struct device_node *np,
	uint16_t *conf)
{
	struct device *dev = indio_dev->dev.parent;
	struct adc *adc = iio_priv(indio_dev);
	const char *external_mux;
	u32 ext_mux_chan;
	u32 resolution;
	int ret;

	*conf = 0;

	*conf |= ADC_CONFIG0_ANALOG_MUX_EN;

	ret = of_property_read_bool(np, "vref");
	if (ret == 1) {
		*conf |= ADC_CONFIG0_ADC_REF_INTERNAL;
		dev_info(dev, "The ADC using external reference voltage\n");
	} else {
		*conf |= ADC_CONFIG0_ADC_REF_VREF;
		dev_info(dev, "The ADC using internal reference voltage\n");
	}

	ret = of_property_read_u32(np, "resolution", &resolution);
	if (ret < 0) {
		*conf |= ADC_CONFIG0_RES_SEL_10_BIT;
		adc->resolution = 10;
		dev_info(dev, "The ADC resolution is 10 bits\n");
	} else {
		switch (resolution) {
		case 6:
			*conf |= ADC_CONFIG0_RES_SEL_6_BIT;
			adc->resolution = 6;
			dev_info(dev, "The ADC resolution is 6 bits\n");
			break;
		case 8:
			*conf |= ADC_CONFIG0_RES_SEL_8_BIT;
			adc->resolution = 8;
			dev_info(dev, "The ADC resolution is 8 bits\n");
			break;
		case 10:
			*conf |= ADC_CONFIG0_RES_SEL_10_BIT;
			adc->resolution = 10;
			dev_info(dev, "The ADC resolution is 10 bits\n");
			break;
		case 12:
			*conf |= ADC_CONFIG0_RES_SEL_12_BIT;
			adc->resolution = 12;
			dev_info(dev, "The ADC resolution is 12 bits\n");
			break;
		default:
			*conf |= ADC_CONFIG0_RES_SEL_10_BIT;
			adc->resolution = 10;
			dev_info(dev, "The ADC resolution is 10 bits\n");
			break;
		}
	}

	ret = of_property_read_bool(np, "anlogic,bipolar");
	if (ret == 1) {
		adc->is_bipolar = true;
		*conf |= ADC_CONFIG0_DIFFERENTIAL;
		dev_info(dev, "The ADC input signal is bipolar\n");
	} else {
		adc->is_bipolar = false;
		*conf |= ADC_CONFIG0_SINGLE_ENDED;
		dev_info(dev, "The ADC input signal is unipolar\n");
	}

	ret = of_property_read_string(np, "anlogic,external-mux", &external_mux);
	if (ret < 0 || strcasecmp(external_mux, "none") == 0)
		adc->external_mux_mode = ADC_EXTERNAL_MUX_NONE;
	else if (strcasecmp(external_mux, "single") == 0)
		adc->external_mux_mode = ADC_EXTERNAL_MUX_SINGLE;
	else
		return -EINVAL;

	if (adc->external_mux_mode == ADC_EXTERNAL_MUX_SINGLE) {
		ret = of_property_read_u32(np, "anlogic,external-mux-channel",
					&ext_mux_chan);
		if (ret < 0)
			return ret;

		*conf |= ADC_CONFIG0_PHY_EXTERNAL_MUX;
	}

	return 0;
}

static int adc_parse_dt_chan(struct iio_dev *indio_dev, struct device_node *np,
	u32 *iomux)
{
	struct device *dev = indio_dev->dev.parent;
	struct adc *adc = iio_priv(indio_dev);
	struct iio_chan_spec *channels, *chan;
	struct device_node *chan_node, *child;
	unsigned int num_channels;
	u32 reg;
	int ret;

	channels = devm_kmemdup(dev, adc_channels,
				sizeof(adc_channels), GFP_KERNEL);
	if (!channels)
		return -ENOMEM;

	num_channels = 0;
	chan = &channels[8];

	chan_node = of_get_child_by_name(np, "anlogic,channels");
	if (chan_node) {
		for_each_child_of_node(chan_node, child) {
			if (num_channels >= ARRAY_SIZE(adc_channels)) {
				of_node_put(child);
				break;
			}

			ret = of_property_read_u32(child, "reg", &reg);
			if (ret || reg > 7)
				continue;

			if (adc->is_bipolar == 1)
				chan->scan_type.sign = 's';

			chan->scan_index = reg;
			chan->address = ADC_REG_VAUX(reg);

			ret = of_property_read_u32(child, "iomux", iomux);
			if (ret < 0)
				return ret;

			num_channels++;
			chan++;
			iomux++;
		}
	}

	of_node_put(chan_node);

	indio_dev->num_channels = num_channels;
	indio_dev->channels = devm_krealloc(dev, channels,
					    sizeof(*channels) * num_channels,
					    GFP_KERNEL);
	/* If we can't resize the channels array, just use the original */
	if (!indio_dev->channels)
		indio_dev->channels = channels;

	return 0;
}

static int adc_probe(struct platform_device *pdev)
{
	const struct of_device_id *id;
	struct iio_dev *indio_dev;
	uint16_t config0;
	u32 *iomux;
	struct adc *adc;
	int ret;
	int irq;
	int i;

	if (!pdev->dev.of_node)
		return -ENODEV;

	id = of_match_node(adc_of_match_table, pdev->dev.of_node);
	if (!id)
		return -EINVAL;

	irq = platform_get_irq(pdev, 0);
	if (irq <= 0)
		return -ENXIO;

	indio_dev = devm_iio_device_alloc(&pdev->dev, sizeof(*adc));
	if (!indio_dev)
		return -ENOMEM;

	adc = iio_priv(indio_dev);
	adc->ops = id->data;
	adc->irq = irq;
	init_completion(&adc->completion);
	mutex_init(&adc->mutex);
	spin_lock_init(&adc->lock);

	adc->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(adc->base))
		return PTR_ERR(adc->base);

	indio_dev->name = "adc";
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->info = &adc_info;

	ret = adc_parse_dt(indio_dev, pdev->dev.of_node, &config0);
	if (ret)
		return ret;

	iomux = devm_kzalloc(&indio_dev->dev, sizeof(u32), GFP_KERNEL);
	if (!iomux)
		return -ENOMEM;
	ret = adc_parse_dt_chan(indio_dev, pdev->dev.of_node, iomux);
	if (ret)
		return ret;

	if (adc->ops->flags & ADC_FLAGS_BUFFERED) {
		ret = iio_triggered_buffer_setup(indio_dev,
			&iio_pollfunc_store_time, &adc_trigger_handler,
			&adc_buffer_ops);
		if (ret)
			return ret;

		adc->convst_trigger = adc_alloc_trigger(indio_dev, "convst");
		if (IS_ERR(adc->convst_trigger)) {
			ret = PTR_ERR(adc->convst_trigger);
			goto err_triggered_buffer_cleanup;
		}
		adc->samplerate_trigger = adc_alloc_trigger(indio_dev,
			"samplerate");
		if (IS_ERR(adc->samplerate_trigger)) {
			ret = PTR_ERR(adc->samplerate_trigger);
			goto err_free_convst_trigger;
		}
	}

	adc->clk = devm_clk_get(&pdev->dev, NULL);
	if (IS_ERR(adc->clk)) {
		ret = PTR_ERR(adc->clk);
		goto err_free_samplerate_trigger;
	}

	ret = clk_prepare_enable(adc->clk);
	if (ret)
		goto err_free_samplerate_trigger;

	ret = request_irq(adc->irq, adc->ops->interrupt_handler, 0,
			dev_name(&pdev->dev), indio_dev);
	if (ret)
		goto err_clk_disable_unprepare;

	ret = adc->ops->setup(pdev, indio_dev, adc->irq);
	if (ret)
		goto err_free_irq;

	/*
	 * Make sure not to exceed the maximum samplerate since otherwise the
	 * resulting interrupt storm will soft-lock the system.
	 */
	if (adc->ops->flags & ADC_FLAGS_BUFFERED) {
		ret = adc_read_samplerate(adc);
		if (ret < 0)
			goto err_free_samplerate_trigger;
		if (ret > ADC_MAX_SAMPLERATE) {
			ret = adc_write_samplerate(adc, ADC_MAX_SAMPLERATE);
			if (ret < 0)
				goto err_free_samplerate_trigger;
		}
	}

	/* Reset PL-ADC*/
	adc_write_adc_reg(adc, ADC_REG_CONFIG0, 1);
	adc_write_adc_reg(adc, ADC_REG_CONFIG0, 0);

	ret = adc_write_adc_reg(adc, ADC_REG_CONFIG0, config0);

	/* Set the iomux of channel 0 to 7 */
	for (i = 0; i < indio_dev->num_channels; i++) {
		if (i % 2 == 0) {
			adc_update_adc_reg(adc, ADC_CHAN_SEL(i), ADC_CHANNEL_0_1_CHAN0_MASK, iomux[i]);
		} else {
			adc_update_adc_reg(adc, ADC_CHAN_SEL(i), ADC_CHANNEL_0_1_CHAN1_MASK, iomux[i] << 8);
		}
	}

	/* Set the conversion channel */
	adc_update_adc_reg(adc, ADC_REG_CONFIG3, ADC_CONFIG3_CHANNEL_SEL_MASK,
				(indio_dev->num_channels - 1) << ADC_CONFIG3_CHANNEL_SEL_SHIFT);

	/* Enable PL-ADC conversion done interrupt*/
	adc_update_adc_reg(adc, ADC_REG_CONFIG1, ADC_CONFIG1_INTR_DONE_MASK_MASK, 0);

	/* Enable PL-ADC*/
	adc_update_adc_reg(adc, ADC_REG_CONFIG2, ADC_CONFIG2_REG_ADC_ENABLE_MASK,
				ADC_CONFIG2_REG_ADC_ENABLE);

	/* Go to non-buffered mode */
	adc_postdisable(indio_dev);

	ret = iio_device_register(indio_dev);
	if (ret)
		goto err_free_irq;

	platform_set_drvdata(pdev, indio_dev);

	return 0;

err_free_irq:
	free_irq(adc->irq, indio_dev);
	cancel_delayed_work_sync(&adc->fpsoc_unmask_work);
err_clk_disable_unprepare:
	clk_disable_unprepare(adc->clk);
err_free_samplerate_trigger:
	if (adc->ops->flags & ADC_FLAGS_BUFFERED)
		iio_trigger_free(adc->samplerate_trigger);
err_free_convst_trigger:
	if (adc->ops->flags & ADC_FLAGS_BUFFERED)
		iio_trigger_free(adc->convst_trigger);
err_triggered_buffer_cleanup:
	if (adc->ops->flags & ADC_FLAGS_BUFFERED)
		iio_triggered_buffer_cleanup(indio_dev);

	return ret;
}

static int adc_remove(struct platform_device *pdev)
{
	struct iio_dev *indio_dev = platform_get_drvdata(pdev);
	struct adc *adc = iio_priv(indio_dev);

	iio_device_unregister(indio_dev);
	if (adc->ops->flags & ADC_FLAGS_BUFFERED) {
		iio_trigger_free(adc->samplerate_trigger);
		iio_trigger_free(adc->convst_trigger);
		iio_triggered_buffer_cleanup(indio_dev);
	}
	free_irq(adc->irq, indio_dev);
	cancel_delayed_work_sync(&adc->fpsoc_unmask_work);
	clk_disable_unprepare(adc->clk);
	kfree(adc->data);

	return 0;
}

static struct platform_driver adc_driver = {
	.probe = adc_probe,
	.remove = adc_remove,
	.driver = {
		.name = "adc",
		.of_match_table = adc_of_match_table,
	},
};
module_platform_driver(adc_driver);

MODULE_AUTHOR("weikang.zhang<weikang.zhang@anlogic.com>");
MODULE_DESCRIPTION("Anlogic dr1m90 adc driver");
MODULE_LICENSE("GPL");
