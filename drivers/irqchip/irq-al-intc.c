// SPDX-License-Identifier: GPL-2.0
/*
 * anlogic axi Interrupt Controller Driver
 *
 * Copyright (c) 2024 anlogic Technologies Co., Ltd.
 */

#include <linux/irqdomain.h>
#include <linux/irq.h>
#include <linux/irqchip.h>
#include <linux/irqchip/chained_irq.h>
#include <linux/of_address.h>
#include <linux/io.h>
#include <linux/jump_label.h>
#include <linux/bug.h>
#include <linux/of_irq.h>

/* No one else should require these constants, so define them locally here. */
#define ISR 0x00			/* Interrupt Status Register */
#define IPR 0x04			/* Interrupt Pending Register */
#define IER 0x08			/* Interrupt Enable Register */
#define IAR 0x0c			/* Interrupt Acknowledge Register */
#define SIE 0x10			/* Set Interrupt Enable bits */
#define CIE 0x14			/* Clear Interrupt Enable bits */
#define IVR 0x18			/* Interrupt Vector Register */
#define MER 0x1c			/* Master Enable Register */

#define MER_ME (1<<0)
#define MER_HIE (1<<1)

static DEFINE_STATIC_KEY_FALSE(al_intc_is_be);

struct al_intc_irq_chip {
	void		__iomem *base;
	struct		irq_domain *root_domain;
	u32		intr_mask;
	u32		nr_irq;
};

static struct al_intc_irq_chip *primary_intc;

static void al_intc_write(struct al_intc_irq_chip *irqc, int reg, u32 data)
{
	if (static_branch_unlikely(&al_intc_is_be))
		iowrite32be(data, irqc->base + reg);
	else
		iowrite32(data, irqc->base + reg);
}

static u32 al_intc_read(struct al_intc_irq_chip *irqc, int reg)
{
	if (static_branch_unlikely(&al_intc_is_be))
		return ioread32be(irqc->base + reg);
	else
		return ioread32(irqc->base + reg);
}

static void intc_enable_or_unmask(struct irq_data *d)
{
	struct al_intc_irq_chip *irqc = irq_data_get_irq_chip_data(d);
	unsigned long mask = BIT(d->hwirq);

	pr_debug("irq-anlogic: enable_or_unmask: %ld\n", d->hwirq);

	/* ack level irqs because they can't be acked during
	 * ack function since the handle_level_irq function
	 * acks the irq before calling the interrupt handler
	 */
	if (irqd_is_level_type(d))
		al_intc_write(irqc, IAR, mask);

	al_intc_write(irqc, SIE, mask);
}

static void intc_disable_or_mask(struct irq_data *d)
{
	struct al_intc_irq_chip *irqc = irq_data_get_irq_chip_data(d);

	pr_debug("irq-anlogic: disable: %ld\n", d->hwirq);
	al_intc_write(irqc, CIE, BIT(d->hwirq));
}

static void intc_ack(struct irq_data *d)
{
	struct al_intc_irq_chip *irqc = irq_data_get_irq_chip_data(d);

	pr_debug("irq-anlogic: ack: %ld\n", d->hwirq);
	al_intc_write(irqc, IAR, BIT(d->hwirq));
}

static void intc_mask_ack(struct irq_data *d)
{
	struct al_intc_irq_chip *irqc = irq_data_get_irq_chip_data(d);
	unsigned long mask = BIT(d->hwirq);

	pr_debug("irq-anlogic: disable_and_ack: %ld\n", d->hwirq);
	al_intc_write(irqc, CIE, mask);
	al_intc_write(irqc, IAR, mask);
}

static struct irq_chip intc_dev = {
	.name = "Anlogic INTC",
	.irq_unmask = intc_enable_or_unmask,
	.irq_mask = intc_disable_or_mask,
	.irq_ack = intc_ack,
	.irq_mask_ack = intc_mask_ack,
};

static unsigned int al_intc_get_irq_local(struct al_intc_irq_chip *irqc)
{
	unsigned int irq = 0;
	u32 hwirq;

	hwirq = al_intc_read(irqc, IVR);
	if (hwirq != -1U)
		irq = irq_find_mapping(irqc->root_domain, hwirq);

	pr_debug("irq-anlogic: hwirq=%d, irq=%d\n", hwirq, irq);

	return irq;
}

unsigned int al_intc_get_irq(void)
{
	unsigned int irq = -1;
	u32 hwirq;

	hwirq = al_intc_read(primary_intc, IVR);
	if (hwirq != -1U)
		irq = irq_find_mapping(primary_intc->root_domain, hwirq);

	pr_debug("irq-anlogic: hwirq=%d, irq=%d\n", hwirq, irq);

	return irq;
}

static int al_intc_map(struct irq_domain *d, unsigned int irq, irq_hw_number_t hw)
{
	struct al_intc_irq_chip *irqc = d->host_data;

	if (irqc->intr_mask & BIT(hw)) {
		irq_set_chip_and_handler_name(irq, &intc_dev,
					      handle_edge_irq, "edge");
		irq_clear_status_flags(irq, IRQ_LEVEL);
	} else {
		irq_set_chip_and_handler_name(irq, &intc_dev,
					      handle_level_irq, "level");
		irq_set_status_flags(irq, IRQ_LEVEL);
	}
	irq_set_chip_data(irq, irqc);
	return 0;
}

static const struct irq_domain_ops al_intc_irq_domain_ops = {
	.xlate = irq_domain_xlate_onetwocell,
	.map = al_intc_map,
};

static void al_intc_irq_handler(struct irq_desc *desc)
{
	struct irq_chip *chip = irq_desc_get_chip(desc);
	struct al_intc_irq_chip *irqc;
	u32 pending;

	irqc = irq_data_get_irq_handler_data(&desc->irq_data);
	chained_irq_enter(chip, desc);
	do {
		pending = al_intc_get_irq_local(irqc);
		if (pending == 0)
			break;
		generic_handle_irq(pending);
	} while (true);
	chained_irq_exit(chip, desc);
}

static int __init anlogic_intc_of_init(struct device_node *intc,
					     struct device_node *parent)
{
	struct al_intc_irq_chip *irqc;
	int ret, irq;

	irqc = kzalloc(sizeof(*irqc), GFP_KERNEL);
	if (!irqc)
		return -ENOMEM;
	irqc->base = of_iomap(intc, 0);
	if (!irqc->base) {
		pr_err("irq-anlogic: unable to map irq base address\n");
		kfree(irqc);
		return -EINVAL;
	}

	ret = of_property_read_u32(intc, "anlogic,num-intr-inputs", &irqc->nr_irq);
	if (ret < 0) {
		pr_err("irq-anlogic: unable to read anlogic,num-intr-inputs\n");
		goto error;
	}

	ret = of_property_read_u32(intc, "anlogic,kind-of-intr", &irqc->intr_mask);
	if (ret < 0) {
		pr_warn("irq-anlogic: unable to read anlogic,kind-of-intr\n");
		irqc->intr_mask = 0;
	}

	if ((u64)irqc->intr_mask >> irqc->nr_irq)
		pr_warn("irq-anlogic: mismatch in kind-of-intr param\n");

	pr_info("irq-anlogic: %pOF: num_irq=%d, edge=0x%x\n",
		intc, irqc->nr_irq, irqc->intr_mask);


	/*
	 * Disable all external interrupts until they are
	 * explicitly requested.
	 */
	al_intc_write(irqc, IER, 0);

	/* Acknowledge any pending interrupts just in case. */
	al_intc_write(irqc, IAR, 0xffffffff);

	/* Turn on the Master Enable. */
	al_intc_write(irqc, MER, MER_HIE | MER_ME);
	if (al_intc_read(irqc, MER) != (MER_HIE | MER_ME)) {
		static_branch_enable(&al_intc_is_be);
		al_intc_write(irqc, MER, MER_HIE | MER_ME);
	}

	irqc->root_domain = irq_domain_add_linear(intc, irqc->nr_irq,
						  &al_intc_irq_domain_ops, irqc);
	if (!irqc->root_domain) {
		pr_err("irq-anlogic: Unable to create IRQ domain\n");
		ret = -EINVAL;
		goto error;
	}

	if (parent) {
		irq = irq_of_parse_and_map(intc, 0);
		if (irq) {
			irq_set_chained_handler_and_data(irq,
							 al_intc_irq_handler,
							 irqc);
		} else {
			pr_err("irq-anlogic: interrupts property not in DT\n");
			ret = -EINVAL;
			goto error;
		}
	} else {
		primary_intc = irqc;
		irq_set_default_host(primary_intc->root_domain);
	}

	return 0;

error:
	if (irqc->root_domain)
		irq_domain_remove(irqc->root_domain);

	iounmap(irqc->base);
	kfree(irqc);
	return ret;

}

IRQCHIP_DECLARE(al_axi_intc, "anlogic,axi-intc-1.00.a", anlogic_intc_of_init);
