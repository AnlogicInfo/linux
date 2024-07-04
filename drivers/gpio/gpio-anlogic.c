// SPDX-License-Identifier: GPL-2.0-only
/*
 * Anlogic axi gpio driver .
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
#define PLGPIO_IPISR_OFFSET		0x120
#define PLGPIO_IPIER_OFFSET		0x128

#define PLGPIO_CHANNEL0_OFFSET	0x0
#define PLGPIO_CHANNEL1_OFFSET	0x8
#define PLGPIO_GIER_IE			BIT(31)

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
	struct clk *clk;
};

static inline int plgpio_from_bit(struct plgpio_instance *chip, int bit)
{
	return bitmap_bitremap(bit, chip->hw_map, chip->sw_map, 64);
}

static inline int plgpio_to_bit(struct plgpio_instance *chip, int gpio)
{
	return bitmap_bitremap(gpio, chip->sw_map, chip->hw_map, 64);
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

static inline int plgpio_regoffset(struct plgpio_instance *chip, int ch)
{
	switch (ch) {
	case 0:
		return PLGPIO_CHANNEL0_OFFSET;
	case 1:
		return PLGPIO_CHANNEL1_OFFSET;
	default:
		return -EINVAL;
	}
}

static void plgpio_read_ch(struct plgpio_instance *chip, int reg, int bit, unsigned long *a)
{
	void __iomem *addr = chip->regs + reg + plgpio_regoffset(chip, bit / 32);

	plgpio_set_value32(a, bit, plgpio_readreg(addr));
}

static void plgpio_write_ch(struct plgpio_instance *chip, int reg, int bit, unsigned long *a)
{
	void __iomem *addr = chip->regs + reg + plgpio_regoffset(chip, bit / 32);

	plgpio_writereg(addr, plgpio_get_value32(a, bit));
}

static void plgpio_read_ch_all(struct plgpio_instance *chip, int reg, unsigned long *a)
{
	int bit, lastbit = plgpio_to_bit(chip, chip->gc.ngpio - 1);

	for (bit = 0; bit <= lastbit ; bit += 32)
		plgpio_read_ch(chip, reg, bit, a);
}

static void plgpio_write_ch_all(struct plgpio_instance *chip, int reg, unsigned long *a)
{
	int bit, lastbit = plgpio_to_bit(chip, chip->gc.ngpio - 1);

	for (bit = 0; bit <= lastbit ; bit += 32)
		plgpio_write_ch(chip, reg, bit, a);
}

static int plgpio_get(struct gpio_chip *gc, unsigned int gpio)
{
	struct plgpio_instance *chip = gpiochip_get_data(gc);
	int bit = plgpio_to_bit(chip, gpio);
	DECLARE_BITMAP(state, 64);

	plgpio_read_ch(chip, PLGPIO_DATA_OFFSET, bit, state);

	return test_bit(bit, state);
}

static void plgpio_set(struct gpio_chip *gc, unsigned int gpio, int val)
{
	unsigned long flags;
	struct plgpio_instance *chip = gpiochip_get_data(gc);
	int bit = plgpio_to_bit(chip, gpio);

	spin_lock_irqsave(&chip->gpio_lock, flags);

	__assign_bit(bit, chip->state, val);

	plgpio_write_ch(chip, PLGPIO_DATA_OFFSET, bit, chip->state);

	spin_unlock_irqrestore(&chip->gpio_lock, flags);
}

static void plgpio_set_multiple(struct gpio_chip *gc, unsigned long *mask,
			       unsigned long *bits)
{
	DECLARE_BITMAP(hw_mask, 64);
	DECLARE_BITMAP(hw_bits, 64);
	DECLARE_BITMAP(state, 64);
	unsigned long flags;
	struct plgpio_instance *chip = gpiochip_get_data(gc);

	bitmap_remap(hw_mask, mask, chip->sw_map, chip->hw_map, 64);
	bitmap_remap(hw_bits, bits, chip->sw_map, chip->hw_map, 64);

	spin_lock_irqsave(&chip->gpio_lock, flags);

	bitmap_replace(state, chip->state, hw_bits, hw_mask, 64);

	plgpio_write_ch_all(chip, PLGPIO_DATA_OFFSET, state);

	bitmap_copy(chip->state, state, 64);

	spin_unlock_irqrestore(&chip->gpio_lock, flags);
}

static int plgpio_dir_in(struct gpio_chip *gc, unsigned int gpio)
{
	unsigned long flags;
	struct plgpio_instance *chip = gpiochip_get_data(gc);
	int bit = plgpio_to_bit(chip, gpio);

	spin_lock_irqsave(&chip->gpio_lock, flags);

	__set_bit(bit, chip->dir);
	plgpio_write_ch(chip, PLGPIO_TRI_OFFSET, bit, chip->dir);

	spin_unlock_irqrestore(&chip->gpio_lock, flags);

	return 0;
}

static int plgpio_dir_out(struct gpio_chip *gc, unsigned int gpio, int val)
{
	unsigned long flags;
	struct plgpio_instance *chip = gpiochip_get_data(gc);
	int bit = plgpio_to_bit(chip, gpio);

	spin_lock_irqsave(&chip->gpio_lock, flags);

	__assign_bit(bit, chip->state, val);
	plgpio_write_ch(chip, PLGPIO_DATA_OFFSET, bit, chip->state);

	__clear_bit(bit, chip->dir);
	plgpio_write_ch(chip, PLGPIO_TRI_OFFSET, bit, chip->dir);

	spin_unlock_irqrestore(&chip->gpio_lock, flags);

	return 0;
}

static void plgpio_save_regs(struct plgpio_instance *chip)
{
	plgpio_write_ch_all(chip, PLGPIO_DATA_OFFSET, chip->state);
	plgpio_write_ch_all(chip, PLGPIO_TRI_OFFSET, chip->dir);
}

static int plgpio_request(struct gpio_chip *chip, unsigned int offset)
{
	int ret;

	ret = pm_runtime_get_sync(chip->parent);

	return ret < 0 ? ret : 0;
}

static void plgpio_free(struct gpio_chip *chip, unsigned int offset)
{
	pm_runtime_put(chip->parent);
}

static int __maybe_unused plgpio_suspend(struct device *dev)
{
	struct plgpio_instance *gpio = dev_get_drvdata(dev);
	struct irq_data *data = irq_get_irq_data(gpio->irq);

	if (!data) {
		dev_dbg(dev, "IRQ not connected\n");
		return pm_runtime_force_suspend(dev);
	}

	if (!irqd_is_wakeup_set(data))
		return pm_runtime_force_suspend(dev);

	return 0;
}

static int plgpio_remove(struct platform_device *pdev)
{
	struct plgpio_instance *gpio = platform_get_drvdata(pdev);

	pm_runtime_get_sync(&pdev->dev);
	pm_runtime_put_noidle(&pdev->dev);
	pm_runtime_disable(&pdev->dev);
	clk_disable_unprepare(gpio->clk);

	return 0;
}

static void plgpio_irq_ack(struct irq_data *irq_data)
{
}

static int __maybe_unused plgpio_resume(struct device *dev)
{
	struct plgpio_instance *gpio = dev_get_drvdata(dev);
	struct irq_data *data = irq_get_irq_data(gpio->irq);

	if (!data) {
		dev_dbg(dev, "IRQ not connected\n");
		return pm_runtime_force_resume(dev);
	}

	if (!irqd_is_wakeup_set(data))
		return pm_runtime_force_resume(dev);

	return 0;
}

static int __maybe_unused plgpio_runtime_suspend(struct device *dev)
{
	struct plgpio_instance *gpio = dev_get_drvdata(dev);

	clk_disable(gpio->clk);

	return 0;
}

static int __maybe_unused plgpio_runtime_resume(struct device *dev)
{
	struct plgpio_instance *gpio = dev_get_drvdata(dev);

	return clk_enable(gpio->clk);
}

static const struct dev_pm_ops plgpio_dev_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(plgpio_suspend, plgpio_resume)
	SET_RUNTIME_PM_OPS(plgpio_runtime_suspend,
			   plgpio_runtime_resume, NULL)
};

static void plgpio_irq_mask(struct irq_data *irq_data)
{
	unsigned long flags;
	struct plgpio_instance *chip = irq_data_get_irq_chip_data(irq_data);
	int irq_offset = irqd_to_hwirq(irq_data);
	int bit = plgpio_to_bit(chip, irq_offset);
	u32 mask = BIT(bit / 32), temp;

	spin_lock_irqsave(&chip->gpio_lock, flags);

	__clear_bit(bit, chip->enable);

	if (plgpio_get_value32(chip->enable, bit) == 0) {
		/* Disable per channel interrupt */
		temp = plgpio_readreg(chip->regs + PLGPIO_IPIER_OFFSET);
		temp &= ~mask;
		plgpio_writereg(chip->regs + PLGPIO_IPIER_OFFSET, temp);
	}
	spin_unlock_irqrestore(&chip->gpio_lock, flags);
}

static void plgpio_irq_unmask(struct irq_data *irq_data)
{
	unsigned long flags;
	struct plgpio_instance *chip = irq_data_get_irq_chip_data(irq_data);
	int irq_offset = irqd_to_hwirq(irq_data);
	int bit = plgpio_to_bit(chip, irq_offset);
	u32 old_enable = plgpio_get_value32(chip->enable, bit);
	u32 mask = BIT(bit / 32), val;

	spin_lock_irqsave(&chip->gpio_lock, flags);

	__set_bit(bit, chip->enable);

	if (old_enable == 0) {
		val = plgpio_readreg(chip->regs + PLGPIO_IPISR_OFFSET);
		val &= mask;
		plgpio_writereg(chip->regs + PLGPIO_IPISR_OFFSET, val);

		plgpio_read_ch(chip, PLGPIO_DATA_OFFSET, bit, chip->last_irq_read);

		val = plgpio_readreg(chip->regs + PLGPIO_IPIER_OFFSET);
		val |= mask;
		plgpio_writereg(chip->regs + PLGPIO_IPIER_OFFSET, val);
	}

	spin_unlock_irqrestore(&chip->gpio_lock, flags);
}

static int plgpio_set_irq_type(struct irq_data *irq_data, unsigned int type)
{
	struct plgpio_instance *chip = irq_data_get_irq_chip_data(irq_data);
	int irq_offset = irqd_to_hwirq(irq_data);
	int bit = plgpio_to_bit(chip, irq_offset);

	switch (type & IRQ_TYPE_SENSE_MASK) {
	case IRQ_TYPE_EDGE_BOTH:
		__set_bit(bit, chip->rising_edge);
		__set_bit(bit, chip->falling_edge);
		break;
	case IRQ_TYPE_EDGE_RISING:
		__set_bit(bit, chip->rising_edge);
		__clear_bit(bit, chip->falling_edge);
		break;
	case IRQ_TYPE_EDGE_FALLING:
		__clear_bit(bit, chip->rising_edge);
		__set_bit(bit, chip->falling_edge);
		break;
	default:
		return -EINVAL;
	}

	irq_set_handler_locked(irq_data, handle_edge_irq);
	return 0;
}

static void plgpio_irqhandler(struct irq_desc *desc)
{
	struct plgpio_instance *chip = irq_desc_get_handler_data(desc);
	struct gpio_chip *gc = &chip->gc;
	struct irq_chip *irqchip = irq_desc_get_chip(desc);
	DECLARE_BITMAP(rising, 64);
	DECLARE_BITMAP(falling, 64);
	DECLARE_BITMAP(all, 64);
	int irq_offset;
	u32 status;
	u32 bit;

	status = plgpio_readreg(chip->regs + PLGPIO_IPISR_OFFSET);
	plgpio_writereg(chip->regs + PLGPIO_IPISR_OFFSET, status);

	chained_irq_enter(irqchip, desc);

	spin_lock(&chip->gpio_lock);

	plgpio_read_ch_all(chip, PLGPIO_DATA_OFFSET, all);

	bitmap_complement(rising, chip->last_irq_read, 64);
	bitmap_and(rising, rising, all, 64);
	bitmap_and(rising, rising, chip->enable, 64);
	bitmap_and(rising, rising, chip->rising_edge, 64);

	bitmap_complement(falling, all, 64);
	bitmap_and(falling, falling, chip->last_irq_read, 64);
	bitmap_and(falling, falling, chip->enable, 64);
	bitmap_and(falling, falling, chip->falling_edge, 64);

	bitmap_copy(chip->last_irq_read, all, 64);
	bitmap_or(all, rising, falling, 64);

	spin_unlock(&chip->gpio_lock);

	dev_dbg(gc->parent, "IRQ rising %*pb falling %*pb\n", 64, rising, 64, falling);

	for_each_set_bit(bit, all, 64) {
		irq_offset = plgpio_from_bit(chip, bit);
		generic_handle_domain_irq(gc->irq.domain, irq_offset);
	}

	chained_irq_exit(irqchip, desc);
}

static int plgpio_probe(struct platform_device *pdev)
{
	struct plgpio_instance *chip;
	int status = 0;
	struct device_node *np = pdev->dev.of_node;
	u32 is_dual = 0;
	u32 cells = 2;
	u32 width[2];
	u32 state[2];
	u32 dir[2];
	struct gpio_irq_chip *girq;
	u32 temp;

	chip = devm_kzalloc(&pdev->dev, sizeof(*chip), GFP_KERNEL);
	if (!chip)
		return -ENOMEM;

	platform_set_drvdata(pdev, chip);

	/* First, check if the device is dual-channel */
	of_property_read_u32(np, "anlogic,is-dual", &is_dual);

	/* Setup defaults */
	memset32(width, 0, ARRAY_SIZE(width));
	memset32(state, 0, ARRAY_SIZE(state));
	memset32(dir, 0xFFFFFFFF, ARRAY_SIZE(dir));

	/* Update GPIO state shadow register with default value */
	of_property_read_u32(np, "anlogic,dout-default", &state[0]);
	of_property_read_u32(np, "anlogic,dout-default-2", &state[1]);

	bitmap_from_arr32(chip->state, state, 64);

	/* Update GPIO direction shadow register with default value */
	of_property_read_u32(np, "anlogic,tri-default", &dir[0]);
	of_property_read_u32(np, "anlogic,tri-default-2", &dir[1]);

	bitmap_from_arr32(chip->dir, dir, 64);

	/* Update cells with gpio-cells value */
	if (of_property_read_u32(np, "#gpio-cells", &cells))
		dev_dbg(&pdev->dev, "Missing gpio-cells property\n");

	if (cells != 2) {
		dev_err(&pdev->dev, "#gpio-cells mismatch\n");
		return -EINVAL;
	}

	/*
	 * Check device node and parent device node for device width
	 * and assume default width of 32
	 */
	if (of_property_read_u32(np, "anlogic,gpio-width", &width[0]))
		width[0] = 32;

	if (width[0] > 32)
		return -EINVAL;

	if (is_dual && of_property_read_u32(np, "anlogic,gpio2-width", &width[1]))
		width[1] = 32;

	if (width[1] > 32)
		return -EINVAL;

	/* Setup software pin mapping */
	bitmap_set(chip->sw_map, 0, width[0] + width[1]);

	/* Setup hardware pin mapping */
	bitmap_set(chip->hw_map,  0, width[0]);
	bitmap_set(chip->hw_map, 32, width[1]);

	spin_lock_init(&chip->gpio_lock);

	chip->gc.base = -1;
	chip->gc.ngpio = bitmap_weight(chip->hw_map, 64);
	chip->gc.parent = &pdev->dev;
	chip->gc.direction_input = plgpio_dir_in;
	chip->gc.direction_output = plgpio_dir_out;
	chip->gc.of_gpio_n_cells = cells;
	chip->gc.get = plgpio_get;
	chip->gc.set = plgpio_set;
	chip->gc.request = plgpio_request;
	chip->gc.free = plgpio_free;
	chip->gc.set_multiple = plgpio_set_multiple;

	chip->gc.label = dev_name(&pdev->dev);

	chip->regs = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(chip->regs)) {
		dev_err(&pdev->dev, "failed to ioremap memory resource\n");
		return PTR_ERR(chip->regs);
	}

	chip->clk = devm_clk_get_optional(&pdev->dev, NULL);
	if (IS_ERR(chip->clk))
		return dev_err_probe(&pdev->dev, PTR_ERR(chip->clk), "input clock not found.\n");

	status = clk_prepare_enable(chip->clk);
	if (status < 0) {
		dev_err(&pdev->dev, "Failed to prepare clk\n");
		return status;
	}

	pm_runtime_get_noresume(&pdev->dev);
	pm_runtime_set_active(&pdev->dev);
	pm_runtime_enable(&pdev->dev);

	plgpio_save_regs(chip);

	chip->irq = platform_get_irq_optional(pdev, 0);
	if (chip->irq <= 0)
		goto skip_irq;

	chip->irqchip.name = "axi-gpio-anlogic";
	chip->irqchip.irq_ack = plgpio_irq_ack;
	chip->irqchip.irq_mask = plgpio_irq_mask;
	chip->irqchip.irq_unmask = plgpio_irq_unmask;
	chip->irqchip.irq_set_type = plgpio_set_irq_type;

	/* Disable per-channel interrupts */
	plgpio_writereg(chip->regs + PLGPIO_IPIER_OFFSET, 0);
	/* Clear any existing per-channel interrupts */
	temp = plgpio_readreg(chip->regs + PLGPIO_IPISR_OFFSET);
	plgpio_writereg(chip->regs + PLGPIO_IPISR_OFFSET, temp);
	/* Enable global interrupts */
	plgpio_writereg(chip->regs + PLGPIO_GIER_OFFSET, PLGPIO_GIER_IE);

	girq = &chip->gc.irq;
	girq->chip = &chip->irqchip;
	girq->parent_handler = plgpio_irqhandler;
	girq->num_parents = 1;
	girq->parents = devm_kcalloc(&pdev->dev, 1,
				     sizeof(*girq->parents),
				     GFP_KERNEL);
	if (!girq->parents) {
		status = -ENOMEM;
		goto err_pm_put;
	}
	girq->parents[0] = chip->irq;
	girq->default_type = IRQ_TYPE_NONE;
	girq->handler = handle_bad_irq;

skip_irq:
	status = devm_gpiochip_add_data(&pdev->dev, &chip->gc, chip);
	if (status) {
		dev_err(&pdev->dev, "failed to add GPIO chip\n");
		goto err_pm_put;
	}

	pm_runtime_put(&pdev->dev);
	return 0;

err_pm_put:
	pm_runtime_disable(&pdev->dev);
	pm_runtime_put_noidle(&pdev->dev);
	clk_disable_unprepare(chip->clk);
	return status;
}

static const struct of_device_id plgpio_of_match[] = {
	{ .compatible = "anlogic,axi-gpio-1.00.a", },
	{ /* end of list */ },
};

MODULE_DEVICE_TABLE(of, plgpio_of_match);

static struct platform_driver plgpio_plat_driver = {
	.probe		= plgpio_probe,
	.remove		= plgpio_remove,
	.driver		= {
			.name = "axi-gpio-anlogic",
			.of_match_table	= plgpio_of_match,
			.pm = &plgpio_dev_pm_ops,
	},
};

static int __init plgpio_init(void)
{
	return platform_driver_register(&plgpio_plat_driver);
}

subsys_initcall(plgpio_init);

static void __exit plgpio_exit(void)
{
	platform_driver_unregister(&plgpio_plat_driver);
}
module_exit(plgpio_exit);

MODULE_AUTHOR("Anlogic, Inc.");
MODULE_DESCRIPTION("Anlogic AXI GPIO driver");
MODULE_LICENSE("GPL");
