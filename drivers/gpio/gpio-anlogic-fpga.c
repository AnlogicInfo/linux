// SPDX-License-Identifier: GPL-2.0-only
/*
 * Anlogic axi gpio driver.
 *
 * Copyright 2008 - 2024 Anlogic, Inc.
 */

#include <linux/bitops.h>
#include <linux/clk.h>
#include <linux/errno.h>
#include <linux/gpio/driver.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/irq.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/of_platform.h>
#include <linux/pm_runtime.h>
#include <linux/slab.h>

/* Register Offset Definitions */
#define PLGPIO_DATA_OFFSET   (0x0)	/* Data register  */
#define PLGPIO_TRI_OFFSET    (0x4)	/* I/O direction register  */

#define PLGPIO_CHANNEL_OFFSET	0x8

#define PLGPIO_GIER_OFFSET	0x11c /* Global Interrupt Enable */
#define PLGPIO_GIER_IE		BIT(31)
#define PLGPIO_IPISR_OFFSET	0x120 /* IP Interrupt Status */
#define PLGPIO_IPIER_OFFSET	0x128 /* IP Interrupt Enable */

/* Read/Write access to the GPIO registers */
# define plgpio_readreg(offset)		__raw_readl(offset)
# define plgpio_writereg(offset, val)	__raw_writel(val, offset)

/**
 * struct plgpio_instance - Stores information about GPIO device
 * @gc: GPIO chip
 * @regs: register block
 * @gpio_width: GPIO width for every channel
 * @gpio_state: GPIO write state shadow register
 * @gpio_last_irq_read: GPIO read state register from last interrupt
 * @gpio_dir: GPIO direction shadow register
 * @gpio_lock: Lock used for synchronization
 * @irq: IRQ used by GPIO device
 * @irqchip: IRQ chip
 * @irq_enable: GPIO IRQ enable/disable bitfield
 * @irq_rising_edge: GPIO IRQ rising edge enable/disable bitfield
 * @irq_falling_edge: GPIO IRQ falling edge enable/disable bitfield
 * @clk: clock resource for this driver
 */
struct plgpio_instance {
	struct gpio_chip gc;
	void __iomem *regs;
	unsigned int gpio_width[2];
	u32 gpio_state[2];
	u32 gpio_last_irq_read[2];
	u32 gpio_dir[2];
	spinlock_t gpio_lock;	/* For serializing operations */
	int irq;
	struct irq_chip irqchip;
	u32 irq_enable[2];
	u32 irq_rising_edge[2];
	u32 irq_falling_edge[2];
	struct clk *clk;
};

static inline int plgpio_index(struct plgpio_instance *chip, int gpio)
{
	if (gpio >= chip->gpio_width[0])
		return 1;

	return 0;
}

static inline int plgpio_regoffset(struct plgpio_instance *chip, int gpio)
{
	if (plgpio_index(chip, gpio))
		return PLGPIO_CHANNEL_OFFSET;

	return 0;
}

static inline int plgpio_offset(struct plgpio_instance *chip, int gpio)
{
	if (plgpio_index(chip, gpio))
		return gpio - chip->gpio_width[0];

	return gpio;
}

/**
 * plgpio_get - Read the specified signal of the GPIO device.
 * @gc:     Pointer to gpio_chip device structure.
 * @gpio:   GPIO signal number.
 *
 * This function reads the specified signal of the GPIO device.
 *
 * Return:
 * 0 if direction of GPIO signals is set as input otherwise it
 * returns negative error value.
 */
static int plgpio_get(struct gpio_chip *gc, unsigned int gpio)
{
	struct plgpio_instance *chip = gpiochip_get_data(gc);
	u32 val;

	val = plgpio_readreg(chip->regs + PLGPIO_DATA_OFFSET +
			    plgpio_regoffset(chip, gpio));

	return !!(val & BIT(plgpio_offset(chip, gpio)));
}

/**
 * plgpio_set - Write the specified signal of the GPIO device.
 * @gc:     Pointer to gpio_chip device structure.
 * @gpio:   GPIO signal number.
 * @val:    Value to be written to specified signal.
 *
 * This function writes the specified value in to the specified signal of the
 * GPIO device.
 */
static void plgpio_set(struct gpio_chip *gc, unsigned int gpio, int val)
{
	unsigned long flags;
	struct plgpio_instance *chip = gpiochip_get_data(gc);
	int index =  plgpio_index(chip, gpio);
	int offset =  plgpio_offset(chip, gpio);

	spin_lock_irqsave(&chip->gpio_lock, flags);

	/* Write to GPIO signal and set its direction to output */
	if (val)
		chip->gpio_state[index] |= BIT(offset);
	else
		chip->gpio_state[index] &= ~BIT(offset);

	plgpio_writereg(chip->regs + PLGPIO_DATA_OFFSET +
		       plgpio_regoffset(chip, gpio), chip->gpio_state[index]);

	spin_unlock_irqrestore(&chip->gpio_lock, flags);
}

/**
 * plgpio_set_multiple - Write the specified signals of the GPIO device.
 * @gc:     Pointer to gpio_chip device structure.
 * @mask:   Mask of the GPIOS to modify.
 * @bits:   Value to be wrote on each GPIO
 *
 * This function writes the specified values into the specified signals of the
 * GPIO devices.
 */
static void plgpio_set_multiple(struct gpio_chip *gc, unsigned long *mask,
			       unsigned long *bits)
{
	unsigned long flags;
	struct plgpio_instance *chip = gpiochip_get_data(gc);
	int index = plgpio_index(chip, 0);
	int offset, i;

	spin_lock_irqsave(&chip->gpio_lock, flags);

	/* Write to GPIO signals */
	for (i = 0; i < gc->ngpio; i++) {
		if (*mask == 0)
			break;
		/* Once finished with an index write it out to the register */
		if (index !=  plgpio_index(chip, i)) {
			plgpio_writereg(chip->regs + PLGPIO_DATA_OFFSET +
				       index * PLGPIO_CHANNEL_OFFSET,
				       chip->gpio_state[index]);
			spin_unlock_irqrestore(&chip->gpio_lock, flags);
			index =  plgpio_index(chip, i);
			spin_lock_irqsave(&chip->gpio_lock, flags);
		}
		if (__test_and_clear_bit(i, mask)) {
			offset =  plgpio_offset(chip, i);
			if (test_bit(i, bits))
				chip->gpio_state[index] |= BIT(offset);
			else
				chip->gpio_state[index] &= ~BIT(offset);
		}
	}

	plgpio_writereg(chip->regs + PLGPIO_DATA_OFFSET +
		       index * PLGPIO_CHANNEL_OFFSET, chip->gpio_state[index]);

	spin_unlock_irqrestore(&chip->gpio_lock, flags);
}

/**
 * plgpio_dir_in - Set the direction of the specified GPIO signal as input.
 * @gc:     Pointer to gpio_chip device structure.
 * @gpio:   GPIO signal number.
 *
 * Return:
 * 0 - if direction of GPIO signals is set as input
 * otherwise it returns negative error value.
 */
static int plgpio_dir_in(struct gpio_chip *gc, unsigned int gpio)
{
	unsigned long flags;
	struct plgpio_instance *chip = gpiochip_get_data(gc);
	int index =  plgpio_index(chip, gpio);
	int offset =  plgpio_offset(chip, gpio);

	spin_lock_irqsave(&chip->gpio_lock, flags);

	/* Set the GPIO bit in shadow register and set direction as input */
	chip->gpio_dir[index] |= BIT(offset);
	plgpio_writereg(chip->regs + PLGPIO_TRI_OFFSET +
		       plgpio_regoffset(chip, gpio), chip->gpio_dir[index]);

	spin_unlock_irqrestore(&chip->gpio_lock, flags);

	return 0;
}

/**
 * plgpio_dir_out - Set the direction of the specified GPIO signal as output.
 * @gc:     Pointer to gpio_chip device structure.
 * @gpio:   GPIO signal number.
 * @val:    Value to be written to specified signal.
 *
 * This function sets the direction of specified GPIO signal as output.
 *
 * Return:
 * If all GPIO signals of GPIO chip is configured as input then it returns
 * error otherwise it returns 0.
 */
static int plgpio_dir_out(struct gpio_chip *gc, unsigned int gpio, int val)
{
	unsigned long flags;
	struct plgpio_instance *chip = gpiochip_get_data(gc);
	int index =  plgpio_index(chip, gpio);
	int offset =  plgpio_offset(chip, gpio);

	spin_lock_irqsave(&chip->gpio_lock, flags);

	/* Write state of GPIO signal */
	if (val)
		chip->gpio_state[index] |= BIT(offset);
	else
		chip->gpio_state[index] &= ~BIT(offset);
	plgpio_writereg(chip->regs + PLGPIO_DATA_OFFSET +
			plgpio_regoffset(chip, gpio), chip->gpio_state[index]);

	/* Clear the GPIO bit in shadow register and set direction as output */
	chip->gpio_dir[index] &= ~BIT(offset);
	plgpio_writereg(chip->regs + PLGPIO_TRI_OFFSET +
			plgpio_regoffset(chip, gpio), chip->gpio_dir[index]);

	spin_unlock_irqrestore(&chip->gpio_lock, flags);

	return 0;
}

/**
 * plgpio_xlate - Translate gpio_spec to the GPIO number and flags
 * @gc: Pointer to gpio_chip device structure.
 * @gpiospec:  gpio specifier as found in the device tree
 * @flags: A flags pointer based on binding
 *
 * Return:
 * irq number
 */
static int plgpio_xlate(struct gpio_chip *gc,
		       const struct of_phandle_args *gpiospec, u32 *flags)
{
	if (gc->of_gpio_n_cells == 3 && flags) {
		*flags = gpiospec->args[2];
		dev_warn(gc->parent, "Please convert #gpio-cells=3 to 2\n");
	}
	if (gc->of_gpio_n_cells == 2 && flags)
		*flags = gpiospec->args[1];

	return gpiospec->args[0];
}

/**
 * plgpio_save_regs - Set initial values of GPIO pins
 * @chip: Pointer to GPIO instance
 */
static void plgpio_save_regs(struct plgpio_instance *chip)
{
	plgpio_writereg(chip->regs + PLGPIO_DATA_OFFSET,	chip->gpio_state[0]);
	plgpio_writereg(chip->regs + PLGPIO_TRI_OFFSET, chip->gpio_dir[0]);

	if (!chip->gpio_width[1])
		return;

	plgpio_writereg(chip->regs + PLGPIO_DATA_OFFSET + PLGPIO_CHANNEL_OFFSET,
		       chip->gpio_state[1]);
	plgpio_writereg(chip->regs + PLGPIO_TRI_OFFSET + PLGPIO_CHANNEL_OFFSET,
		       chip->gpio_dir[1]);
}

static int plgpio_request(struct gpio_chip *chip, unsigned int offset)
{
	int ret;

	ret = pm_runtime_get_sync(chip->parent);
	/*
	 * If the device is already active pm_runtime_get() will return 1 on
	 * success, but gpio_request still needs to return 0.
	 */
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

/**
 * plgpio_remove - Remove method for the GPIO device.
 * @pdev: pointer to the platform device
 *
 * This function remove gpiochips and frees all the allocated resources.
 *
 * Return: 0 always
 */
static int plgpio_remove(struct platform_device *pdev)
{
	struct plgpio_instance *gpio = platform_get_drvdata(pdev);

	pm_runtime_get_sync(&pdev->dev);
	pm_runtime_put_noidle(&pdev->dev);
	pm_runtime_disable(&pdev->dev);
	clk_disable_unprepare(gpio->clk);

	return 0;
}

/**
 * plgpio_irq_ack - Acknowledge a child GPIO interrupt.
 * @irq_data: per IRQ and chip data passed down to chip functions
 * This currently does nothing, but irq_ack is unconditionally called by
 * handle_edge_irq and therefore must be defined.
 */
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
	struct platform_device *pdev = to_platform_device(dev);
	struct plgpio_instance *gpio = platform_get_drvdata(pdev);

	clk_disable(gpio->clk);

	return 0;
}

static int __maybe_unused plgpio_runtime_resume(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct plgpio_instance *gpio = platform_get_drvdata(pdev);

	return clk_enable(gpio->clk);
}

static const struct dev_pm_ops plgpio_dev_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(plgpio_suspend, plgpio_resume)
	SET_RUNTIME_PM_OPS(plgpio_runtime_suspend,
			   plgpio_runtime_resume, NULL)
};

/**
 * plgpio_irq_mask - Write the specified signal of the GPIO device.
 * @irq_data: per IRQ and chip data passed down to chip functions
 */
static void plgpio_irq_mask(struct irq_data *irq_data)
{
	unsigned long flags;
	struct plgpio_instance *chip = irq_data_get_irq_chip_data(irq_data);
	int irq_offset = irqd_to_hwirq(irq_data);
	int index = plgpio_index(chip, irq_offset);
	int offset = plgpio_offset(chip, irq_offset);

	spin_lock_irqsave(&chip->gpio_lock, flags);

	chip->irq_enable[index] &= ~BIT(offset);

	if (!chip->irq_enable[index]) {
		/* Disable per channel interrupt */
		u32 temp = plgpio_readreg(chip->regs + PLGPIO_IPIER_OFFSET);

		temp &= ~BIT(index);
		plgpio_writereg(chip->regs + PLGPIO_IPIER_OFFSET, temp);
	}
	spin_unlock_irqrestore(&chip->gpio_lock, flags);
}

/**
 * plgpio_irq_unmask - Write the specified signal of the GPIO device.
 * @irq_data: per IRQ and chip data passed down to chip functions
 */
static void plgpio_irq_unmask(struct irq_data *irq_data)
{
	unsigned long flags;
	struct plgpio_instance *chip = irq_data_get_irq_chip_data(irq_data);
	int irq_offset = irqd_to_hwirq(irq_data);
	int index = plgpio_index(chip, irq_offset);
	int offset = plgpio_offset(chip, irq_offset);
	u32 old_enable = chip->irq_enable[index];

	spin_lock_irqsave(&chip->gpio_lock, flags);

	chip->irq_enable[index] |= BIT(offset);

	if (!old_enable) {
		/* Clear any existing per-channel interrupts */
		u32 val = plgpio_readreg(chip->regs + PLGPIO_IPISR_OFFSET) &
			BIT(index);

		if (val)
			plgpio_writereg(chip->regs + PLGPIO_IPISR_OFFSET, val);

		/* Update GPIO IRQ read data before enabling interrupt*/
		val = plgpio_readreg(chip->regs + PLGPIO_DATA_OFFSET +
				    index * PLGPIO_CHANNEL_OFFSET);
		chip->gpio_last_irq_read[index] = val;

		/* Enable per channel interrupt */
		val = plgpio_readreg(chip->regs + PLGPIO_IPIER_OFFSET);
		val |= BIT(index);
		plgpio_writereg(chip->regs + PLGPIO_IPIER_OFFSET, val);
	}

	spin_unlock_irqrestore(&chip->gpio_lock, flags);
}

/**
 * plgpio_set_irq_type - Write the specified signal of the GPIO device.
 * @irq_data: Per IRQ and chip data passed down to chip functions
 * @type: Interrupt type that is to be set for the gpio pin
 *
 * Return:
 * 0 if interrupt type is supported otherwise -EINVAL
 */
static int plgpio_set_irq_type(struct irq_data *irq_data, unsigned int type)
{
	struct plgpio_instance *chip = irq_data_get_irq_chip_data(irq_data);
	int irq_offset = irqd_to_hwirq(irq_data);
	int index = plgpio_index(chip, irq_offset);
	int offset = plgpio_offset(chip, irq_offset);

	/*
	 * The Xilinx GPIO hardware provides a single interrupt status
	 * indication for any state change in a given GPIO channel (bank).
	 * Therefore, only rising edge or falling edge triggers are
	 * supported.
	 */
	switch (type & IRQ_TYPE_SENSE_MASK) {
	case IRQ_TYPE_EDGE_BOTH:
		chip->irq_rising_edge[index] |= BIT(offset);
		chip->irq_falling_edge[index] |= BIT(offset);
		break;
	case IRQ_TYPE_EDGE_RISING:
		chip->irq_rising_edge[index] |= BIT(offset);
		chip->irq_falling_edge[index] &= ~BIT(offset);
		break;
	case IRQ_TYPE_EDGE_FALLING:
		chip->irq_rising_edge[index] &= ~BIT(offset);
		chip->irq_falling_edge[index] |= BIT(offset);
		break;
	default:
		return -EINVAL;
	}

	irq_set_handler_locked(irq_data, handle_edge_irq);
	return 0;
}

/**
 * plgpio_irqhandler - Gpio interrupt service routine
 * @desc: Pointer to interrupt description
 */
static void plgpio_irqhandler(struct irq_desc *desc)
{
	struct plgpio_instance *chip = irq_desc_get_handler_data(desc);
	struct irq_chip *irqchip = irq_desc_get_chip(desc);
	u32 num_channels = chip->gpio_width[1] ? 2 : 1;
	u32 offset = 0, index;
	u32 status = plgpio_readreg(chip->regs + PLGPIO_IPISR_OFFSET);

	plgpio_writereg(chip->regs + PLGPIO_IPISR_OFFSET, status);

	chained_irq_enter(irqchip, desc);
	for (index = 0; index < num_channels; index++) {
		if ((status & BIT(index))) {
			unsigned long rising_events, falling_events, all_events;
			unsigned long flags;
			u32 data, bit;
			unsigned int irq;

			spin_lock_irqsave(&chip->gpio_lock, flags);
			data = plgpio_readreg(chip->regs + PLGPIO_DATA_OFFSET +
					     index * PLGPIO_CHANNEL_OFFSET);
			rising_events = data &
					~chip->gpio_last_irq_read[index] &
					chip->irq_enable[index] &
					chip->irq_rising_edge[index];
			falling_events = ~data &
					 chip->gpio_last_irq_read[index] &
					 chip->irq_enable[index] &
					 chip->irq_falling_edge[index];
			dev_dbg(chip->gc.parent,
				"IRQ chan %u rising 0x%lx falling 0x%lx\n",
				index, rising_events, falling_events);
			all_events = rising_events | falling_events;
			chip->gpio_last_irq_read[index] = data;
			spin_unlock_irqrestore(&chip->gpio_lock, flags);

			for_each_set_bit(bit, &all_events, 32) {
				irq = irq_find_mapping(chip->gc.irq.domain,
						       offset + bit);
				generic_handle_irq(irq);
			}
		}
		offset += chip->gpio_width[index];
	}

	chained_irq_exit(irqchip, desc);
}

/**
 * plgpio_probe - Probe method for the GPIO device.
 * @pdev: pointer to the platform device
 *
 * Return:
 * It returns 0, if the driver is bound to the GPIO device, or
 * a negative value if there is an error.
 */
static int plgpio_probe(struct platform_device *pdev)
{
	struct plgpio_instance *chip;
	int status = 0;
	struct device_node *np = pdev->dev.of_node;
	u32 is_dual = 0;
	u32 cells = 2;
	struct gpio_irq_chip *girq;
	u32 temp;

	chip = devm_kzalloc(&pdev->dev, sizeof(*chip), GFP_KERNEL);
	if (!chip)
		return -ENOMEM;

	platform_set_drvdata(pdev, chip);

	/* Update GPIO state shadow register with default value */
	if (of_property_read_u32(np, "anlogic,dout-default", &chip->gpio_state[0]))
		chip->gpio_state[0] = 0x0;

	/* Update GPIO direction shadow register with default value */
	if (of_property_read_u32(np, "anlogic,tri-default", &chip->gpio_dir[0]))
		chip->gpio_dir[0] = 0xFFFFFFFF;

	/* Update cells with gpio-cells value */
	if (of_property_read_u32(np, "#gpio-cells", &cells))
		dev_dbg(&pdev->dev, "Missing gpio-cells property\n");

	if (cells != 2 && cells != 3) {
		dev_err(&pdev->dev, "#gpio-cells mismatch\n");
		return -EINVAL;
	}

	/*
	 * Check device node and parent device node for device width
	 * and assume default width of 32
	 */
	if (of_property_read_u32(np, "anlogic,gpio-width", &chip->gpio_width[0]))
		chip->gpio_width[0] = 32;

	if (chip->gpio_width[0] > 32)
		return -EINVAL;

	spin_lock_init(&chip->gpio_lock);

	if (of_property_read_u32(np, "anlogic,is-dual", &is_dual))
		is_dual = 0;

	if (is_dual) {
		/* Update GPIO state shadow register with default value */
		if (of_property_read_u32(np, "anlogic,dout-default-2",
					 &chip->gpio_state[1]))
			chip->gpio_state[1] = 0x0;

		/* Update GPIO direction shadow register with default value */
		if (of_property_read_u32(np, "anlogic,tri-default-2",
					 &chip->gpio_dir[1]))
			chip->gpio_dir[1] = 0xFFFFFFFF;

		/*
		 * Check device node and parent device node for device width
		 * and assume default width of 32
		 */
		if (of_property_read_u32(np, "anlogic,gpio2-width",
					 &chip->gpio_width[1]))
			chip->gpio_width[1] = 32;

		if (chip->gpio_width[1] > 32)
			return -EINVAL;
	}

	chip->gc.base = -1;
	chip->gc.ngpio = chip->gpio_width[0] + chip->gpio_width[1];
	chip->gc.parent = &pdev->dev;
	chip->gc.of_xlate = plgpio_xlate;
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
