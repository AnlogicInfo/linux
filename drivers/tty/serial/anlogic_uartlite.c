// SPDX-License-Identifier: GPL-2.0
/*
 * anlogic_uartlite.c: Serial driver for Anlogic pl_uart serial controller
 *
 * Copyright (C) 2006 Peter Korsgaard <jacmet@sunsite.dk>
 * Copyright (C) 2007 Secret Lab Technologies Ltd.
 * Copyright (C) 2023 Anlogic, Inc.
 */
#include <linux/clk.h>
#include <linux/console.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/pm_wakeirq.h>
#include <linux/serial_core.h>
#include <linux/serial.h>
#include <linux/tty_flip.h>
#include <linux/tty.h>

#define PLUART_NAME			"ttyUL"
#define PLUART_MAJOR		204
#define PLUART_MINOR		187
#define PLUART_NR_UARTS		CONFIG_SERIAL_ANLOGIC_UARTLITE_NR_UARTS

#define PLUART_TX_EN		0x00
#define PLUART_RX			0x08
#define PLUART_TX			0x04

#define PLUART_INTR_STATUS	0x14
#define PLUART_STATUS		0x0c
#define PLUART_REGION		16

#define UART_AUTOSUSPEND_TIMEOUT	3000	/* ms */

struct pluart_data {
	struct clk *clk;
	unsigned int baud;
	tcflag_t cflags;
};

static inline u32 uart_in32(u32 offset, struct uart_port *port)
{
	return ioread32(port->membase + offset);
}

static inline void uart_out32(u32 val, u32 offset, struct uart_port *port)
{
	iowrite32(val, port->membase + offset);
}

static struct uart_port pluart_ports[PLUART_NR_UARTS];

static struct uart_driver pluart_uart_driver;

static int pluart_receive(struct uart_port *port, int stat)
{
	struct tty_port *tport = &port->state->port;
	unsigned char ch = 0;
	char flag = TTY_NORMAL;

	port->icount.rx++;
	ch = uart_in32(PLUART_RX, port);
	port->icount.parity++;
	flag = TTY_PARITY;
	tty_insert_flip_char(tport, ch, flag);

	return 1;
}

static int pluart_transmit(struct uart_port *port, int stat)
{
	struct circ_buf *xmit  = &port->state->xmit;

	if (port->x_char) {
		uart_out32(port->x_char | 0x100, PLUART_TX, port);
		port->x_char = 0;
		port->icount.tx++;
		return 1;
	}

	if (uart_circ_empty(xmit) || uart_tx_stopped(port))
		return 0;

	uart_out32(xmit->buf[xmit->tail] | 0x100, PLUART_TX, port);
	xmit->tail = (xmit->tail + 1) & (UART_XMIT_SIZE-1);
	port->icount.tx++;

	if (uart_circ_chars_pending(xmit) < WAKEUP_CHARS)
		uart_write_wakeup(port);

	return 1;
}

static irqreturn_t pluart_isr(int irq, void *dev_id)
{
	struct uart_port *port = dev_id;
	int stat;
	unsigned long flags;

	int rx_intr_flag = 0x02;
	int tx_intr_flag = 0x01;

	spin_lock_irqsave(&port->lock, flags);

	stat = uart_in32(PLUART_INTR_STATUS, port);
	if (stat & rx_intr_flag) {
		pluart_receive(port, stat);
		/* 0:rx intr clear */
		uart_out32(uart_in32(PLUART_STATUS, port) & ~0x10,
					PLUART_STATUS, port);
		/* 0:rx intr unmask */
		uart_out32(uart_in32(PLUART_STATUS, port) & ~0x4,
					PLUART_STATUS, port);
	}

	if (stat & tx_intr_flag) {
		pluart_transmit(port, stat);
		/* 0:tx intr clear */
		uart_out32(uart_in32(PLUART_STATUS, port) & ~0x8,
					PLUART_STATUS, port);
		/* 0:tx intr unmask */
		uart_out32(uart_in32(PLUART_STATUS, port) & ~0x2,
					PLUART_STATUS, port);
	}

	spin_unlock_irqrestore(&port->lock, flags);

	tty_flip_buffer_push(&port->state->port);
		return IRQ_HANDLED;
}

static unsigned int pluart_tx_empty(struct uart_port *port)
{
	return TIOCSER_TEMT;
}

static unsigned int pluart_get_mctrl(struct uart_port *port)
{
	return TIOCM_CTS | TIOCM_DSR | TIOCM_CAR;
}

static void pluart_set_mctrl(struct uart_port *port, unsigned int mctrl)
{
	/* N/A */
}

static void pluart_stop_tx(struct uart_port *port)
{
	/* N/A */
}

static void pluart_start_tx(struct uart_port *port)
{
	pluart_transmit(port, uart_in32(PLUART_STATUS, port));
}

static void pluart_stop_rx(struct uart_port *port)
{
	/* N/A */
}

static void pluart_break_ctl(struct uart_port *port, int ctl)
{
	/* N/A */
}

static int pluart_startup(struct uart_port *port)
{
	int ret;

	ret = request_irq(port->irq, pluart_isr, IRQF_SHARED | IRQF_TRIGGER_RISING,
			  "uartlite", port);
	if (ret)
		return ret;

	/* enable tx/rx */
	uart_out32(uart_in32(PLUART_TX_EN, port) & ~0x1, PLUART_TX_EN, port);
	uart_out32(uart_in32(PLUART_TX_EN, port) | 0x1, PLUART_TX_EN, port);

	/* enable intr */
	uart_out32(uart_in32(PLUART_STATUS, port) & ~0x1, PLUART_STATUS, port);
	uart_out32(uart_in32(PLUART_STATUS, port) | 0x1, PLUART_STATUS, port);

	return 0;
}

static void pluart_shutdown(struct uart_port *port)
{
	free_irq(port->irq, port);
}

static void pluart_set_termios(struct uart_port *port,
			      struct ktermios *termios,
			      const struct ktermios *oldtermios)
{
	/* N/A */
}

static const char *pluart_type(struct uart_port *port)
{
	return port->type == PORT_UARTLITE ? "uartlite" : NULL;
}

static void pluart_release_port(struct uart_port *port)
{
	release_mem_region(port->mapbase, PLUART_REGION);
	iounmap(port->membase);
	port->membase = NULL;
}

static int pluart_request_port(struct uart_port *port)
{
	if (!request_mem_region(port->mapbase, PLUART_REGION, "uartlite")) {
		dev_err(port->dev, "Memory region busy\n");
		return -EBUSY;
	}

	port->membase = ioremap(port->mapbase, PLUART_REGION);
	if (!port->membase) {
		dev_err(port->dev, "Unable to map registers\n");
		release_mem_region(port->mapbase, PLUART_REGION);
		return -EBUSY;
	}

	return 0;
}

static void pluart_config_port(struct uart_port *port, int flags)
{
	if (!pluart_request_port(port))
		port->type = PORT_UARTLITE;
}

static int pluart_verify_port(struct uart_port *port, struct serial_struct *ser)
{
	/* No user changeable parameters */
	return -EINVAL;
}

static void pluart_pm(struct uart_port *port, unsigned int state,
		     unsigned int oldstate)
{
	int ret;

	if (!state) {
		ret = pm_runtime_get_sync(port->dev);
		if (ret < 0)
			dev_err(port->dev, "Failed to enable clocks\n");
	} else {
		pm_runtime_mark_last_busy(port->dev);
		pm_runtime_put_autosuspend(port->dev);
	}
}

static const struct uart_ops pluart_ops = {
	.tx_empty	= pluart_tx_empty,
	.set_mctrl	= pluart_set_mctrl,
	.get_mctrl	= pluart_get_mctrl,
	.stop_tx	= pluart_stop_tx,
	.start_tx	= pluart_start_tx,
	.stop_rx	= pluart_stop_rx,
	.break_ctl	= pluart_break_ctl,
	.startup	= pluart_startup,
	.shutdown	= pluart_shutdown,
	.set_termios	= pluart_set_termios,
	.type			= pluart_type,
	.release_port	= pluart_release_port,
	.request_port	= pluart_request_port,
	.config_port	= pluart_config_port,
	.verify_port	= pluart_verify_port,
	.pm				= pluart_pm,
};

static struct uart_driver pluart_uart_driver = {
	.owner		= THIS_MODULE,
	.driver_name	= "uartlite",
	.dev_name	= PLUART_NAME,
	.major		= PLUART_MAJOR,
	.minor		= PLUART_MINOR,
	.nr			= PLUART_NR_UARTS,
};

static int pluart_assign(struct device *dev, int id, phys_addr_t base, int irq,
			struct pluart_data *pdata)
{
	struct uart_port *port;
	int rc;

	if (id < 0) {
		for (id = 0; id < PLUART_NR_UARTS; id++)
			if (pluart_ports[id].mapbase == 0)
				break;
	}
	if (id < 0 || id >= PLUART_NR_UARTS) {
		dev_err(dev, "%s%i too large\n", PLUART_NAME, id);
		return -EINVAL;
	}

	if ((pluart_ports[id].mapbase) && (pluart_ports[id].mapbase != base)) {
		dev_err(dev, "cannot assign to %s%i; it is already in use\n",
			PLUART_NAME, id);
		return -EBUSY;
	}

	port = &pluart_ports[id];

	spin_lock_init(&port->lock);
	port->fifosize = 16;
	port->regshift = 2;
	port->iotype = UPIO_MEM;
	port->iobase = 1;
	port->mapbase = base;
	port->membase = NULL;
	port->ops = &pluart_ops;
	port->irq = irq;
	port->flags = UPF_BOOT_AUTOCONF;
	port->dev = dev;
	port->type = PORT_UNKNOWN;
	port->line = id;
	port->private_data = pdata;

	dev_set_drvdata(dev, port);

	rc = uart_add_one_port(&pluart_uart_driver, port);
	if (rc) {
		dev_err(dev, "uart_add_one_port() failed; err=%i\n", rc);
		port->mapbase = 0;
		dev_set_drvdata(dev, NULL);
		return rc;
	}

	return 0;
}

static int pluart_release(struct device *dev)
{
	struct uart_port *port = dev_get_drvdata(dev);
	int rc = 0;

	if (port) {
		rc = uart_remove_one_port(&pluart_uart_driver, port);
		dev_set_drvdata(dev, NULL);
		port->mapbase = 0;
	}

	return rc;
}

static int __maybe_unused pluart_suspend(struct device *dev)
{
	struct uart_port *port = dev_get_drvdata(dev);

	if (port)
		uart_suspend_port(&pluart_uart_driver, port);

	return 0;
}

static int __maybe_unused pluart_resume(struct device *dev)
{
	struct uart_port *port = dev_get_drvdata(dev);

	if (port)
		uart_resume_port(&pluart_uart_driver, port);

	return 0;
}

static int __maybe_unused pluart_runtime_suspend(struct device *dev)
{
	struct uart_port *port = dev_get_drvdata(dev);
	struct pluart_data *pdata = port->private_data;

	clk_disable(pdata->clk);
	return 0;
};

static int __maybe_unused pluart_runtime_resume(struct device *dev)
{
	struct uart_port *port = dev_get_drvdata(dev);
	struct pluart_data *pdata = port->private_data;
	int ret;

	ret = clk_enable(pdata->clk);
	if (ret) {
		dev_err(dev, "Cannot enable clock.\n");
		return ret;
	}
	return 0;
}

static const struct dev_pm_ops pluart_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(pluart_suspend, pluart_resume)
	SET_RUNTIME_PM_OPS(pluart_runtime_suspend,
			   pluart_runtime_resume, NULL)
};

#if defined(CONFIG_OF)
static const struct of_device_id pluart_of_match[] = {
	{ .compatible = "anlogic,xps-uartlite-1.00.a", },
	{}
};
MODULE_DEVICE_TABLE(of, pluart_of_match);
#endif /* CONFIG_OF */

static int pluart_probe(struct platform_device *pdev)
{
	struct resource *res;
	struct pluart_data *pdata;
	int irq, ret;
	int id = pdev->id;

	pdata = devm_kzalloc(&pdev->dev, sizeof(struct pluart_data),
			     GFP_KERNEL);
	if (!pdata)
		return -ENOMEM;

	if (IS_ENABLED(CONFIG_OF)) {
		struct device_node *np = pdev->dev.of_node;
		u32 val = 0;

		ret = of_property_read_u32(np, "port-number", &id);
		if (ret && ret != -EINVAL)
			return -EINVAL;

		ret = of_property_read_u32(np, "current-speed", &pdata->baud);
		if (ret && ret != -EINVAL)
			return -EINVAL;

		ret = of_property_read_u32(np, "anlogic,use-parity", &val);
		if (ret && ret != -EINVAL)
			return -EINVAL;

		if (val) {
			ret = of_property_read_u32(np, "anlogic,odd-parity", &val);
			if (ret)
				return -EINVAL;

			if (val)
				pdata->cflags |= PARODD;
			pdata->cflags |= PARENB;
		}
	}

	pdata->baud = 115200;
	pdata->cflags = CS8;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res)
		return -ENODEV;

	irq = platform_get_irq(pdev, 0);
	if (irq < 0)
		return irq;

	pdata->clk = devm_clk_get(&pdev->dev, "s_axi_aclk");
	if (IS_ERR(pdata->clk)) {
		if (PTR_ERR(pdata->clk) != -ENOENT)
			return PTR_ERR(pdata->clk);

		pdata->clk = NULL;
	}

	ret = clk_prepare_enable(pdata->clk);
	if (ret) {
		dev_err(&pdev->dev, "Failed to prepare clock\n");
		return ret;
	}

	pm_runtime_use_autosuspend(&pdev->dev);
	pm_runtime_set_autosuspend_delay(&pdev->dev, UART_AUTOSUSPEND_TIMEOUT);
	pm_runtime_set_active(&pdev->dev);
	pm_runtime_enable(&pdev->dev);

	if (!pluart_uart_driver.state) {
		ret = uart_register_driver(&pluart_uart_driver);
		if (ret < 0) {
			dev_err(&pdev->dev, "Failed to register driver\n");
			clk_disable_unprepare(pdata->clk);
			return ret;
		}
	}

	ret = pluart_assign(&pdev->dev, id, res->start, irq, pdata);

	pm_runtime_mark_last_busy(&pdev->dev);
	pm_runtime_put_autosuspend(&pdev->dev);

	return ret;
}

static int pluart_remove(struct platform_device *pdev)
{
	struct uart_port *port = dev_get_drvdata(&pdev->dev);
	struct pluart_data *pdata = port->private_data;
	int rc;

	clk_disable_unprepare(pdata->clk);
	rc = pluart_release(&pdev->dev);
	pm_runtime_disable(&pdev->dev);
	pm_runtime_set_suspended(&pdev->dev);
	pm_runtime_dont_use_autosuspend(&pdev->dev);
	return rc;
}

MODULE_ALIAS("platform:uartlite");

static struct platform_driver pluart_platform_driver = {
	.probe = pluart_probe,
	.remove = pluart_remove,
	.driver = {
		.name  = "uartlite",
		.of_match_table = of_match_ptr(pluart_of_match),
		.pm = &pluart_pm_ops,
	},
};

static int __init pluart_init(void)
{
	return platform_driver_register(&pluart_platform_driver);
}

static void __exit pluart_exit(void)
{
	platform_driver_unregister(&pluart_platform_driver);
	uart_unregister_driver(&pluart_uart_driver);
}

module_init(pluart_init);
module_exit(pluart_exit);

MODULE_AUTHOR("chenxi huang <chenxi.huang@anlogic.com>");
MODULE_DESCRIPTION("pluart serial driver");
MODULE_LICENSE("GPL");
