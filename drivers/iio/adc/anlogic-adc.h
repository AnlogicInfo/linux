// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2023 Anlogic, Inc.
 *
*/
#ifndef __IIO_ANLOGIC_ADC__
#define __IIO_ANLOGIC_ADC__

#include <linux/interrupt.h>
#include <linux/mutex.h>
#include <linux/spinlock.h>

struct iio_dev;
struct clk;
struct adc_ops;
struct platform_device;

enum adc_external_mux_mode {
	ADC_EXTERNAL_MUX_NONE,
	ADC_EXTERNAL_MUX_SINGLE,
};

struct adc {
	void __iomem *base;
	void __iomem *gp_base;
	struct clk *clk;

	const struct adc_ops *ops;

	uint16_t *data;
	uint32_t resolution;
	u_int8_t conv_chan_num;
	bool is_bipolar;

	struct iio_trigger *trigger;
	struct iio_trigger *convst_trigger;
	struct iio_trigger *samplerate_trigger;

	enum adc_external_mux_mode external_mux_mode;

	unsigned int fpsoc_intmask;
	struct delayed_work fpsoc_unmask_work;

	struct mutex mutex;
	spinlock_t lock;

	struct completion completion;
	int irq;
};

struct adc_ops {
	int (*read)(struct adc *adc, unsigned int reg, uint16_t *val);
	int (*write)(struct adc *adc, unsigned int reg, uint16_t val);
	int (*setup)(struct platform_device *pdev, struct iio_dev *indio_dev,
			int irq);
	unsigned long (*get_dclk_rate)(struct adc *adc);
	irqreturn_t (*interrupt_handler)(int irq, void *devid);

	unsigned int flags;
};

static inline int _adc_read_adc_reg(struct adc *adc, unsigned int reg,
	uint16_t *val)
{
	lockdep_assert_held(&adc->mutex);
	return adc->ops->read(adc, reg, val);
}

static inline int _adc_write_adc_reg(struct adc *adc, unsigned int reg,
	uint16_t val)
{
	lockdep_assert_held(&adc->mutex);
	return adc->ops->write(adc, reg, val);
}

static inline int adc_read_adc_reg(struct adc *adc, unsigned int reg,
	uint16_t *val)
{
	int ret;

	mutex_lock(&adc->mutex);
	ret = _adc_read_adc_reg(adc, reg, val);
	mutex_unlock(&adc->mutex);
	return ret;
}

static inline int adc_write_adc_reg(struct adc *adc, unsigned int reg,
	uint16_t val)
{
	int ret;

	mutex_lock(&adc->mutex);
	ret = _adc_write_adc_reg(adc, reg, val);
	mutex_unlock(&adc->mutex);
	return ret;
}

/* adc hardmacro register definitions */

#define ADC_REG_VAUX(x)	(0x40 + (2*x))
#define ADC_REG_FLAG		0x5E
#define ADC_CHANNEL_0_1		0x60
#define ADC_CHANNEL_2_3		0x62
#define ADC_CHANNEL_4_5		0x64
#define ADC_CHANNEL_6_7		0x66
#define ADC_REG_CONFIG0		0x68
#define ADC_REG_CONFIG1		0x6A
#define ADC_REG_CONFIG2		0x6C
#define ADC_REG_CONFIG3		0x6E
#define ADC_CHAN_LTH(x)	(0x80 + (2*x))
#define ADC_CHAN_GTH(x)	(0x90 + (2*x))

#define ADC_CHANNEL_0_1_CHAN0_MASK		0x001f
#define ADC_CHANNEL_0_1_CHAN1_MASK		0x1f00
#define ADC_CHANNEL_2_3_CHAN2_MASK		0x001f
#define ADC_CHANNEL_2_3_CHAN3_MASK		0x1f00
#define ADC_CHANNEL_4_5_CHAN4_MASK		0x001f
#define ADC_CHANNEL_4_5_CHAN5_MASK		0x1f00
#define ADC_CHANNEL_6_7_CHAN6_MASK		0x001f
#define ADC_CHANNEL_6_7_CHAN7_MASK		0x1f00

#define ADC_CHAN_SEL(x)		(0x60 + (x / 2) * 2)

#define ADC_FLAG_DONE		BIT(0)
#define ADC_FLAG_GTH		BIT(1)
#define ADC_FLAG_LTH		BIT(2)
#define ADC_FLAG_ERROR		BIT(3)
#define ADC_FLAG_VC0		BIT(8)
#define ADC_FLAG_VC1		BIT(9)
#define ADC_FLAG_VC2		BIT(10)
#define ADC_FLAG_VC3		BIT(11)
#define ADC_FLAG_VC4		BIT(12)
#define ADC_FLAG_VC5		BIT(13)
#define ADC_FLAG_VC6		BIT(14)
#define ADC_FLAG_VC7		BIT(15)


#define ADC_CONFIG0_ADC_SW_RELEASE			(0x0 << 0)
#define ADC_CONFIG0_ADC_SW_RESET			(0x1 << 0)
#define ADC_CONFIG0_ADC_REF_VREF			(0x0 << 5)
#define ADC_CONFIG0_ADC_REF_INTERNAL		(0x1 << 5)
#define ADC_CONFIG0_RES_SEL_6_BIT			(0x0 << 6)
#define ADC_CONFIG0_RES_SEL_8_BIT			(0x1 << 6)
#define ADC_CONFIG0_RES_SEL_10_BIT			(0x2 << 6)
#define ADC_CONFIG0_RES_SEL_12_BIT			(0x3 << 6)
#define ADC_CONFIG0_SINGLE_ENDED			(0x0 << 8)
#define ADC_CONFIG0_DIFFERENTIAL			(0x1 << 8)
#define ADC_CONFIG0_PHY_EXTERNAL_MUX		BIT(9)
#define ADC_CONFIG0_ANALOG_MUX_EN			BIT(10)
#define ADC_CONFIG0_REG_ADC_SOC_SEL			BIT(11)
#define ADC_CONFIG0_PHY_EXTERNAL_MUX_GAP	(0xf << 12)

#define ADC_CONFIG0_SW_RESET_SHIFT				0
#define ADC_CONFIG0_ENCTR_SHIFT					1
#define ADC_CONFIG0_REF_SEL_SHIFT				5
#define ADC_CONFIG0_RES_SEL_SHIFT				6
#define ADC_CONFIG0_DIFF_ENABLE_SHIFT			8
#define ADC_CONFIG0_PHY_EXTERNAL_MUX_SHIFT		9
#define ADC_CONFIG0_ANALOG_MUX_EN_SHIFT			10
#define ADC_CONFIG0_REG_SOC_SEL_SHIFT			11
#define ADC_CONFIG0_PHY_EXTERNAL_MUX_GAP_SHIFT	12

#define ADC_CONFIG0_ADC_SW_RESET_MASK			0x0001
#define ADC_CONFIG0_ENCTR_MASK					0x000e
#define ADC_CONFIG0_REF_SEL_MASK				0x0020
#define ADC_CONFIG0_RES_SEL_MASK				0x00c0
#define ADC_CONFIG0_DIFF_ENABLE_MASK			0x0100
#define ADC_CONFIG0_PHY_EXTERNAL_MUX_MASK		0x0200
#define ADC_CONFIG0_ANALOG_MUX_EN_MASK			0x0400
#define ADC_CONFIG0_REG_ADC_SOC_SEL_MASK		0x0800
#define ADC_CONFIG0_PHY_EXTERNAL_MUX_GAP_MASK	0xf000


#define ADC_CONFIG1_INTR_DONE		BIT(8)
#define ADC_CONFIG1_INTR_GTH		BIT(9)
#define ADC_CONFIG1_INTR_LTH		BIT(10)
#define ADC_CONFIG1_INTR_ERROR		BIT(11)

#define ADC_CONFIG1_INTR_DONE_MASK_SHIFT	0
#define ADC_CONFIG1_INTR_GTH_MASK_SHIFT		1
#define ADC_CONFIG1_INTR_LTH_MASK_SHIFT		2
#define ADC_CONFIG1_INTR_ERROR_MASK_SHIFT	3
#define ADC_CONFIG1_INTR_DONE_SHIFT			8
#define ADC_CONFIG1_INTR_GTH_SHIFT			9
#define ADC_CONFIG1_INTR_LTH_SHIFT			10
#define ADC_CONFIG1_INTR_ERROR_SHIFT		11

#define ADC_CONFIG1_INTR_DONE_MASK_MASK		0x0001
#define ADC_CONFIG1_INTR_GTH_MASK_MASK		0x0002
#define ADC_CONFIG1_INTR_LTH_MASK_MASK		0x0004
#define ADC_CONFIG1_INTR_ERROR_MASK_MASK	0x0008
#define ADC_CONFIG1_INTR_DONE_MASK			0x0100
#define ADC_CONFIG1_INTR_GTH_MASK			0x0200
#define ADC_CONFIG1_INTR_LTH_MASK			0x0400
#define ADC_CONFIG1_INTR_ERROR_MASK			0x0800


#define ADC_CONFIG2_REG_ADC_ENABLE		BIT(0)
#define ADC_CONFIG2_REG_ADC_DISLVL		BIT(1)
#define ADC_CONFIG2_ADC_CLK_SEL_OSC		(0x0 << 4)
#define ADC_CONFIG2_ADC_CLK_SEL_PS		(0x1 << 4)
#define ADC_CONFIG2_ADC_CLK_SEL_PL		(0x2 << 4)
#define ADC_CONFIG2_ADC_CLK_GATE		BIT(6)

#define ADC_CONFIG2_REG_ENABLE_SHIFT	0
#define ADC_CONFIG2_REG_DISLVL_SHIFT	1
#define ADC_CONFIG2_CLK_SEL_SHIFT		4
#define ADC_CONFIG2_CLK_GATE_SHIFT		6
#define ADC_CONFIG2_CLK_DIV_SHIFT		8

#define ADC_CONFIG2_REG_ADC_ENABLE_MASK		0x0001
#define ADC_CONFIG2_REG_ADC_DISLVL_MASK		0x0002
#define ADC_CONFIG2_ADC_CLK_SEL_MASK		0x0030
#define ADC_CONFIG2_ADC_CLK_GATE_MASK		0x0040
#define ADC_CONFIG2_ADC_CLK_DIV_MASK		0xff00


#define ADC_CONFIG3_CONV_MODE_SINGELE_CHAN	(0x0)
#define ADC_CONFIG3_CONV_MODE_SINGELE_PASS	(0x1)
#define ADC_CONFIG3_CONV_MODE_CONTINUOUS	(0x2)
#define ADC_CONFIG3_REG_ADC_SOC				BIT(12)

#define ADC_CONFIG3_MODE_SEL_SHIFT				0
#define ADC_CONFIG3_CHANNEL_SEL_SHIFT			4
#define ADC_CONFIG3_EXTERNAL_CHANNEL_SEL_SHIFT	8
#define ADC_CONFIG3_REG_SOC_SHIFT				12

#define ADC_CONFIG3_MODE_SEL_MASK				0x0007
#define ADC_CONFIG3_CHANNEL_SEL_MASK			0x0070
#define ADC_CONFIG3_EXTERNAL_CHANNEL_SEL_MASK	0x0700
#define ADC_CONFIG3_REG_ADC_SOC_MASK			0x1000


#define ADC_CHAN_LTH_SHIFT		4
#define ADC_CHAN_LTH_MASK		0xfff0


#define ADC_CHAN_GTH_SHIFT		4
#define ADC_CHAN_GTH_MASK		0xfff0


#endif
