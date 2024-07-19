// SPDX-License-Identifier: GPL-2.0-only
/*
 * Anlogic fpga axi gpio driver .
 *
 * Copyright 2008 - 2024 Anlogic, Inc.
 */

#include <linux/bitmap.h>
#include <linux/bitops.h>
#include <linux/of_device.h>
#include <linux/of_platform.h>
#include <linux/pm_runtime.h>
#include <linux/slab.h>
#include <linux/clk.h>
#include <linux/errno.h>
#include <linux/gpio/driver.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/irq.h>
#include <linux/module.h>

#define PLGPIO_DATA_OFFSET		(0x0)
#define PLGPIO_TRI_OFFSET		(0x4)
#define PLGPIO_GIER_OFFSET		0x11c
//#define PLGPIO_IPISR_OFFSET		0x120
//#define PLGPIO_IPIER_OFFSET		0x128
#define PLGPIO_IPISR_OFFSET		0x104
#define PLGPIO_IPIER_OFFSET		0x100

#define PLGPIO_CHANNEL0_OFFSET	0x0
#define PLGPIO_CHANNEL1_OFFSET	0x8
#define PLGPIO_GIER_IE			BIT(31)

#define PLGPIO_LINE_NUM	64
#define ANLOGIC_AXI_GPIO_DRIVER_NAME	"gpio-anlogic"

#define plgpio_readreg(offset)		__raw_readl(offset)
#define plgpio_writereg(offset, val)	__raw_writel(val, offset)

struct plgpio_instance {
	struct gpio_chip gc;
	void __iomem *regs;
	DECLARE_BITMAP(hw_map, 64);
	DECLARE_BITMAP(sw_map, 64);
	DECLARE_BITMAP(state, 64);
	DECLARE_BITMAP(last_irq_read, 64);
	DECLARE_BITMAP(dir, 64);
	spinlock_t gpio_lock;
	int irq;
	struct irq_chip irqchip;
	DECLARE_BITMAP(enable, 64);
	DECLARE_BITMAP(rising_edge, 64);
	DECLARE_BITMAP(falling_edge, 64);
	DECLARE_BITMAP(level_low, 64);
	DECLARE_BITMAP(level_high, 64);
	struct clk *clk;
};

static inline int plgpio_from_bit(struct plgpio_instance *plgpio, int bit)
{
	return bitmap_bitremap(bit, plgpio->hw_map, plgpio->sw_map, 64);
}

static inline int plgpio_to_bit(struct plgpio_instance *plgpio, int gpio)
{
	return bitmap_bitremap(gpio, plgpio->sw_map, plgpio->hw_map, 64);
}

static inline u32 plgpio_get_value32(const unsigned long *map, int bit)
{
	const size_t index = BIT_WORD(bit);
	const unsigned long offset = (bit % BITS_PER_LONG) & BIT(5);

	return (map[index] >> offset) & 0xFFFFFFFFul;
}

static inline void plgpio_set_value32(unsigned long *map, int bit, u32 v)
{
	const size_t index = BIT_WORD(bit);
	const unsigned long offset = (bit % BITS_PER_LONG) & BIT(5);

	map[index] &= ~(0xFFFFFFFFul << offset);
	map[index] |= (unsigned long)v << offset;
}

static inline int plgpio_regoffset(int channel)
{
	switch (channel) {
	case 0:
		return PLGPIO_CHANNEL0_OFFSET;
	case 1:
		return PLGPIO_CHANNEL1_OFFSET;
	default:
		return -EINVAL;
	}
}

static void plgpio_read_ch(struct plgpio_instance *plgpio, int reg, int bit, unsigned long *a)
{
	void __iomem *addr = plgpio->regs + reg + plgpio_regoffset(bit / 32);

	plgpio_set_value32(a, bit, plgpio_readreg(addr));
}

static void plgpio_write_ch(struct plgpio_instance *plgpio, int reg, int bit, unsigned long *a)
{
	void __iomem *addr = plgpio->regs + reg + plgpio_regoffset(bit / 32);

	plgpio_writereg(addr, plgpio_get_value32(a, bit));
}

static void plgpio_read_ch_all(struct plgpio_instance *plgpio, int reg, unsigned long *a)
{
	int bit, lastbit = plgpio_to_bit(plgpio, plgpio->gc.ngpio - 1);

	for (bit = 0; bit <= lastbit ; bit += 32)
		plgpio_read_ch(plgpio, reg, bit, a);
}

static void plgpio_write_ch_all(struct plgpio_instance *plgpio, int reg, unsigned long *a)
{
	int bit, lastbit = plgpio_to_bit(plgpio, plgpio->gc.ngpio - 1);

	for (bit = 0; bit <= lastbit ; bit += 32)
		plgpio_write_ch(plgpio, reg, bit, a);
}

static int plgpio_get(struct gpio_chip *gc, unsigned int gpio)
{
	struct plgpio_instance *plgpio = gpiochip_get_data(gc);
	int bit = plgpio_to_bit(plgpio, gpio);
	DECLARE_BITMAP(state, 64);

	plgpio_read_ch(plgpio, PLGPIO_DATA_OFFSET, bit, state);

	return test_bit(bit, state);
}

static void plgpio_set(struct gpio_chip *gc, unsigned int gpio, int val)
{
	unsigned long flags;
	struct plgpio_instance *plgpio = gpiochip_get_data(gc);
	int bit = plgpio_to_bit(plgpio, gpio);

	spin_lock_irqsave(&plgpio->gpio_lock, flags);

	__assign_bit(bit, plgpio->state, val);

	plgpio_write_ch(plgpio, PLGPIO_DATA_OFFSET, bit, plgpio->state);

	spin_unlock_irqrestore(&plgpio->gpio_lock, flags);
}

static int plgpio_dir_in(struct gpio_chip *gc, unsigned int gpio)
{
	unsigned long flags;
	struct plgpio_instance *plgpio = gpiochip_get_data(gc);
	int bit = plgpio_to_bit(plgpio, gpio);

	spin_lock_irqsave(&plgpio->gpio_lock, flags);

	__set_bit(bit, plgpio->dir);
	plgpio_write_ch(plgpio, PLGPIO_TRI_OFFSET, bit, plgpio->dir);

	spin_unlock_irqrestore(&plgpio->gpio_lock, flags);

	return 0;
}

static int plgpio_dir_out(struct gpio_chip *gc, unsigned int gpio, int val)
{
	unsigned long flags;
	struct plgpio_instance *plgpio = gpiochip_get_data(gc);
	int bit = plgpio_to_bit(plgpio, gpio);

	spin_lock_irqsave(&plgpio->gpio_lock, flags);

	__assign_bit(bit, plgpio->state, val);
	plgpio_write_ch(plgpio, PLGPIO_DATA_OFFSET, bit, plgpio->state);

	__clear_bit(bit, plgpio->dir);
	plgpio_write_ch(plgpio, PLGPIO_TRI_OFFSET, bit, plgpio->dir);

	spin_unlock_irqrestore(&plgpio->gpio_lock, flags);

	return 0;
}

static int plgpio_request(struct gpio_chip *plgpio, unsigned int offset)
{
	int ret;

	ret = pm_runtime_get_sync(plgpio->parent);

	return ret < 0 ? ret : 0;
}

static void plgpio_free(struct gpio_chip *plgpio, unsigned int offset)
{
	pm_runtime_put(plgpio->parent);
}

static void plgpio_irq_ack(struct irq_data *irq_data)
{
	/* do nothing */
}

static void plgpio_irq_mask(struct irq_data *irq_data)
{
	unsigned long flags;
	struct plgpio_instance *plgpio = irq_data_get_irq_chip_data(irq_data);
	int irq_offset = irqd_to_hwirq(irq_data);
	int bit = plgpio_to_bit(plgpio, irq_offset);
	u32 mask = BIT(bit / 32), temp;

	spin_lock_irqsave(&plgpio->gpio_lock, flags);

	__clear_bit(bit, plgpio->enable);

	if (plgpio_get_value32(plgpio->enable, bit) == 0) {
		temp = plgpio_readreg(plgpio->regs + PLGPIO_IPIER_OFFSET);
		temp &= ~mask;
		plgpio_writereg(plgpio->regs + PLGPIO_IPIER_OFFSET, temp);
	}
	spin_unlock_irqrestore(&plgpio->gpio_lock, flags);
}

static void plgpio_irq_unmask(struct irq_data *irq_data)
{
	unsigned long flags;
	struct plgpio_instance *plgpio = irq_data_get_irq_chip_data(irq_data);
	int irq_offset = irqd_to_hwirq(irq_data);
	int bit = plgpio_to_bit(plgpio, irq_offset);
	u32 old_enable = plgpio_get_value32(plgpio->enable, bit);
	u32 mask = BIT(bit / 32), val;

	spin_lock_irqsave(&plgpio->gpio_lock, flags);

	__set_bit(bit, plgpio->enable);

	if (old_enable == 0) {
		val = plgpio_readreg(plgpio->regs + PLGPIO_IPISR_OFFSET);
		val &= mask;
		plgpio_writereg(plgpio->regs + PLGPIO_IPISR_OFFSET, val);

		plgpio_read_ch(plgpio, PLGPIO_DATA_OFFSET, bit, plgpio->last_irq_read);

		val = plgpio_readreg(plgpio->regs + PLGPIO_IPIER_OFFSET);
		val |= mask;
		plgpio_writereg(plgpio->regs + PLGPIO_IPIER_OFFSET, val);
	}

	spin_unlock_irqrestore(&plgpio->gpio_lock, flags);
}

static int plgpio_set_irq_type(struct irq_data *irq_data, unsigned int type)
{
	struct plgpio_instance *plgpio = irq_data_get_irq_chip_data(irq_data);
	int irq_offset = irqd_to_hwirq(irq_data);
	int bit = plgpio_to_bit(plgpio, irq_offset);

	switch (type & IRQ_TYPE_SENSE_MASK) {
	case IRQ_TYPE_EDGE_BOTH:
		__set_bit(bit, plgpio->rising_edge);
		__set_bit(bit, plgpio->falling_edge);
		__clear_bit(bit, plgpio->level_low);
		__clear_bit(bit, plgpio->level_high);
		break;
	case IRQ_TYPE_EDGE_RISING:
		__set_bit(bit, plgpio->rising_edge);
		__clear_bit(bit, plgpio->falling_edge);
		__clear_bit(bit, plgpio->level_low);
		__clear_bit(bit, plgpio->level_high);
		break;
	case IRQ_TYPE_EDGE_FALLING:
		__set_bit(bit, plgpio->falling_edge);
		__clear_bit(bit, plgpio->rising_edge);
		break;
	case IRQ_TYPE_LEVEL_HIGH:
		__set_bit(bit, plgpio->level_high);
		__clear_bit(bit, plgpio->level_low);
		__clear_bit(bit, plgpio->rising_edge);
		__clear_bit(bit, plgpio->falling_edge);
		break;
	case IRQ_TYPE_LEVEL_LOW:
		__set_bit(bit, plgpio->level_low);
		__clear_bit(bit, plgpio->level_high);
		__clear_bit(bit, plgpio->rising_edge);
		__clear_bit(bit, plgpio->falling_edge);
		break;
	default:
		return -EINVAL;
	}

	irq_set_handler_locked(irq_data, handle_edge_irq);
	return 0;
}

static void plgpio_irqhandler(struct irq_desc *desc)
{
	struct plgpio_instance *plgpio = irq_desc_get_handler_data(desc);
	struct gpio_chip *gc = &plgpio->gc;
	struct irq_chip *irqchip = irq_desc_get_chip(desc);
	DECLARE_BITMAP(rising, 64);
	DECLARE_BITMAP(falling, 64);
	DECLARE_BITMAP(all, 64);
	int irq_offset;
	u32 status;
	u32 bit;

	status = plgpio_readreg(plgpio->regs + PLGPIO_IPISR_OFFSET);
	plgpio_writereg(plgpio->regs + PLGPIO_IPISR_OFFSET, status);

	chained_irq_enter(irqchip, desc);

	spin_lock(&plgpio->gpio_lock);

	plgpio_read_ch_all(plgpio, PLGPIO_DATA_OFFSET, all);

	bitmap_complement(rising, plgpio->last_irq_read, 64);
	bitmap_and(rising, rising, all, 64);
	bitmap_and(rising, rising, plgpio->enable, 64);
	bitmap_and(rising, rising, plgpio->rising_edge, 64);

	bitmap_complement(falling, all, 64);
	bitmap_and(falling, falling, plgpio->last_irq_read, 64);
	bitmap_and(falling, falling, plgpio->enable, 64);
	bitmap_and(falling, falling, plgpio->falling_edge, 64);

	bitmap_copy(plgpio->last_irq_read, all, 64);
	bitmap_or(all, rising, falling, 64);

	spin_unlock(&plgpio->gpio_lock);

	dev_dbg(gc->parent, "IRQ rising %*pb falling %*pb\n", 64, rising, 64, falling);

	for_each_set_bit(bit, all, 64) {
		irq_offset = plgpio_from_bit(plgpio, bit);
		generic_handle_domain_irq(gc->irq.domain, irq_offset);
	}

	chained_irq_exit(irqchip, desc);
}

static int plgpio_probe(struct platform_device *pdev)
{
	struct plgpio_instance *plgpio;
	int status = 0;
	struct device_node *np = pdev->dev.of_node;
	u32 is_dual = 0;
	u32 cells = 2;
	u32 width[2];
	u32 state[2];
	u32 dir[2];
	struct gpio_irq_chip *girq;
	u32 temp;

	plgpio = devm_kzalloc(&pdev->dev, sizeof(*plgpio), GFP_KERNEL);
	if (!plgpio)
		return -ENOMEM;

	platform_set_drvdata(pdev, plgpio);

	of_property_read_u32(np, "anlogic,is-dual", &is_dual);

	memset32(width, 0, ARRAY_SIZE(width));
	memset32(state, 0, ARRAY_SIZE(state));
	memset32(dir, 0xFFFFFFFF, ARRAY_SIZE(dir));

	of_property_read_u32(np, "anlogic,dout-default", &state[0]);
	of_property_read_u32(np, "anlogic,dout-default-2", &state[1]);

	bitmap_from_arr32(plgpio->state, state, 64);

	of_property_read_u32(np, "anlogic,tri-default", &dir[0]);
	of_property_read_u32(np, "anlogic,tri-default-2", &dir[1]);

	bitmap_from_arr32(plgpio->dir, dir, 64);

	of_property_read_u32(np, "anlogic,gpio-width", &width[0]);
	if (width[0] > 32)
		return -EINVAL;

	of_property_read_u32(np, "anlogic,gpio2-width", &width[1]);
	if (width[1] > 32)
		return -EINVAL;

	bitmap_set(plgpio->sw_map, 0, width[0] + width[1]);
	bitmap_set(plgpio->hw_map,  0, width[0]);
	bitmap_set(plgpio->hw_map, 32, width[1]);

	spin_lock_init(&plgpio->gpio_lock);

	plgpio->gc.base = -1;
	plgpio->gc.ngpio = PLGPIO_LINE_NUM;
	plgpio->gc.parent = &pdev->dev;
	plgpio->gc.direction_input = plgpio_dir_in;
	plgpio->gc.direction_output = plgpio_dir_out;
	plgpio->gc.of_gpio_n_cells = cells;
	plgpio->gc.get = plgpio_get;
	plgpio->gc.set = plgpio_set;
	plgpio->gc.request = plgpio_request;
	plgpio->gc.free = plgpio_free;

	plgpio->gc.label = dev_name(&pdev->dev);

	plgpio->regs = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(plgpio->regs)) {
		dev_err(&pdev->dev, "failed to ioremap memory resource\n");
		return PTR_ERR(plgpio->regs);
	}

	plgpio->clk = devm_clk_get_optional(&pdev->dev, NULL);
	if (IS_ERR(plgpio->clk))
		return dev_err_probe(&pdev->dev, PTR_ERR(plgpio->clk), "input clock not found.\n");

	status = clk_prepare_enable(plgpio->clk);
	if (status < 0) {
		dev_err(&pdev->dev, "Failed to prepare clk\n");
		return status;
	}

	pm_runtime_get_noresume(&pdev->dev);
	pm_runtime_set_active(&pdev->dev);
	pm_runtime_enable(&pdev->dev);

	plgpio_write_ch_all(plgpio, PLGPIO_DATA_OFFSET, plgpio->state);
	plgpio_write_ch_all(plgpio, PLGPIO_TRI_OFFSET, plgpio->dir);

	plgpio->irq = platform_get_irq_optional(pdev, 0);
	if (plgpio->irq <= 0)
		goto skip_irq;

	plgpio->irqchip.name = ANLOGIC_AXI_GPIO_DRIVER_NAME;
	plgpio->irqchip.irq_ack = plgpio_irq_ack;
	plgpio->irqchip.irq_mask = plgpio_irq_mask;
	plgpio->irqchip.irq_unmask = plgpio_irq_unmask;
	plgpio->irqchip.irq_set_type = plgpio_set_irq_type;

	plgpio_writereg(plgpio->regs + PLGPIO_IPIER_OFFSET, 0);

	temp = plgpio_readreg(plgpio->regs + PLGPIO_IPISR_OFFSET);
	plgpio_writereg(plgpio->regs + PLGPIO_IPISR_OFFSET, temp);

	plgpio_writereg(plgpio->regs + PLGPIO_GIER_OFFSET, PLGPIO_GIER_IE);

	girq = &plgpio->gc.irq;
	girq->chip = &plgpio->irqchip;
	girq->parent_handler = plgpio_irqhandler;
	girq->num_parents = 1;
	girq->parents = devm_kcalloc(&pdev->dev, 1,
				     sizeof(*girq->parents),
				     GFP_KERNEL);
	if (!girq->parents) {
		status = -ENOMEM;
		goto err_pm_put;
	}
	girq->parents[0] = plgpio->irq;
	girq->default_type = IRQ_TYPE_NONE;
	girq->handler = handle_bad_irq;

skip_irq:
	status = devm_gpiochip_add_data(&pdev->dev, &plgpio->gc, plgpio);
	if (status) {
		dev_err(&pdev->dev, "failed to add GPIO chip\n");
		goto err_pm_put;
	}

	pm_runtime_put(&pdev->dev);
	return 0;

err_pm_put:
	pm_runtime_disable(&pdev->dev);
	pm_runtime_put_noidle(&pdev->dev);
	clk_disable_unprepare(plgpio->clk);
	return status;
}

static const struct of_device_id plgpio_of_match[] = {
	{ .compatible = "anlogic,axi-gpio-1.00.a", },
	{ /* end of list */ },
};

MODULE_DEVICE_TABLE(of, plgpio_of_match);

static struct platform_driver plgpio_driver = {
	.probe		= plgpio_probe,
	.driver		= {
			.name = ANLOGIC_AXI_GPIO_DRIVER_NAME,
			.of_match_table	= plgpio_of_match,
	},
};

module_platform_driver(plgpio_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Anlogic, Inc.");
MODULE_DESCRIPTION("Anlogic AXI GPIO driver");
MODULE_ALIAS("platform:" ANLOGIC_AXI_GPIO_DRIVER_NAME);
