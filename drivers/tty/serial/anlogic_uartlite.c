// SPDX-License-Identifier: GPL-2.0
/*
 * anlogic_uartlite.c: Serial driver for Anlogic pl_uart serial controller
 *
 * Copyright (C) 2006 Peter Korsgaard <jacmet@sunsite.dk>
 * Copyright (C) 2007 Secret Lab Technologies Ltd.
 * Copyright (C) 2023 Anlogic, Inc.
 */

#include <linux/platform_device.h>
#include <linux/module.h>
#include <linux/console.h>
#include <linux/serial.h>
#include <linux/serial_core.h>
#include <linux/tty.h>
#include <linux/tty_flip.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/of_platform.h>
#include <linux/clk.h>

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

#define PLUART_STATUS_RXVALID	0x01 //0:rx fifo is empty      1:rx fifo has valid data
#define PLUART_STATUS_RXFULL	0x02 //0:rx fifo is not full   1:rx fifo is full

#define PLUART_STATUS_TXEMPTY	0x04 //0:tx fifo is not empty  1:tx fifo is empty
#define PLUART_STATUS_TXFULL	0x0c

#define PLUART_STATUS_OVERRUN	0x20
#define PLUART_STATUS_FRAME		0x40 //0:no frame error    1:a frame error
#define PLUART_STATUS_PARITY	0x80 //0:no parity error   1:a parity error

/* Static pointer to console port */
#ifdef CONFIG_SERIAL_UARTLITE_CONSOLE
static struct uart_port *console_port;
#endif

/**
 * struct uartlite_data: Driver private data
 * reg_ops: Functions to read/write registers
 * clk: Our parent clock, if present
 * baud: The baud rate configured when this device was synthesized
 * cflags: The cflags for parity and data bits
 */
struct uartlite_data {
	const struct uartlite_reg_ops *reg_ops;
	struct clk *clk;
};

struct uartlite_reg_ops {
	u32 (*in)(void __iomem *addr);
	void (*out)(u32 val, void __iomem *addr);
};

static u32 uartlite_inbe32(void __iomem *addr)
{
	return ioread32be(addr);
}

static void uartlite_outbe32(u32 val, void __iomem *addr)
{
	iowrite32be(val, addr);
}

static const struct uartlite_reg_ops uartlite_be = {
	.in = uartlite_inbe32,
	.out = uartlite_outbe32,
};

static u32 uartlite_inle32(void __iomem *addr)
{
	return ioread32(addr);
}

static void uartlite_outle32(u32 val, void __iomem *addr)
{
	iowrite32(val, addr);
}

static const struct uartlite_reg_ops uartlite_le = {
	.in = uartlite_inle32,
	.out = uartlite_outle32,
};

static inline u32 uart_in32(u32 offset, struct uart_port *port)
{
	struct uartlite_data *pdata = port->private_data;

	return pdata->reg_ops->in(port->membase + offset);
}

static inline void uart_out32(u32 val, u32 offset, struct uart_port *port)
{
	struct uartlite_data *pdata = port->private_data;

	pdata->reg_ops->out(val, port->membase + offset);
}

static struct uart_port pluart_ports[PLUART_NR_UARTS];

/* ---------------------------------------------------------------------
 * Core UART driver operations
 */

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

	/* wake up */
	if (uart_circ_chars_pending(xmit) < WAKEUP_CHARS)
		uart_write_wakeup(port);

	return 1;
}

static irqreturn_t pluart_isr(int irq, void *dev_id)
{
	struct uart_port *port = dev_id;
	int stat, busy = 0, n = 0;
	unsigned long flags;

	int rx_intr_flag = 0x02;
	int tx_intr_flag = 0x01;

	do {
		spin_lock_irqsave(&port->lock, flags);

		stat = uart_in32(PLUART_INTR_STATUS, port);
		if (stat & rx_intr_flag) {
			pluart_receive(port, stat);
			uart_out32(uart_in32(PLUART_STATUS, port) & ~0x10,
						PLUART_STATUS, port); // 0:rx intr clear
			uart_out32(uart_in32(PLUART_STATUS, port) & ~0x4,
						PLUART_STATUS, port); //  0:rx intr unmask
		}

		if (stat & tx_intr_flag) {
			pluart_transmit(port, stat);
			uart_out32(uart_in32(PLUART_STATUS, port) & ~0x8,
						PLUART_STATUS, port); // 0:tx intr clear
			uart_out32(uart_in32(PLUART_STATUS, port) & ~0x2,
						PLUART_STATUS, port); // 0:tx intr unmask
		}

		spin_unlock_irqrestore(&port->lock, flags);
		n++;
	} while (busy);

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
	struct uartlite_data *pdata = port->private_data;
	int ret;

	ret = clk_enable(pdata->clk);
	if (ret) {
		dev_err(port->dev, "Failed to enable clock\n");
		return ret;
	}

	uart_out32(uart_in32(PLUART_TX_EN, port) & ~0x1, PLUART_TX_EN, port); // 0:disable
	uart_out32(uart_in32(PLUART_TX_EN, port) | 0x1, PLUART_TX_EN, port); // 1:enable

	ret = request_irq(port->irq, pluart_isr, IRQF_SHARED | IRQF_TRIGGER_RISING,
			  "uartlite", port);
	if (ret)
		return ret;

	//enable intr
	uart_out32(uart_in32(PLUART_STATUS, port) & ~0x1, PLUART_STATUS, port); // 0:disable
	uart_out32(uart_in32(PLUART_STATUS, port) | 0x1, PLUART_STATUS, port); // 1:enable

	return 0;
}

static void pluart_shutdown(struct uart_port *port)
{
	struct uartlite_data *pdata = port->private_data;

	free_irq(port->irq, port);
	clk_disable(pdata->clk);
}

static void pluart_set_termios(struct uart_port *port, struct ktermios *termios,
			      struct ktermios *old)
{
	unsigned long flags;
	unsigned int baud;

	spin_lock_irqsave(&port->lock, flags);

	port->read_status_mask = PLUART_STATUS_RXVALID | PLUART_STATUS_OVERRUN
		| PLUART_STATUS_TXFULL;

	if (termios->c_iflag & INPCK)
		port->read_status_mask |=
			PLUART_STATUS_PARITY | PLUART_STATUS_FRAME;

	port->ignore_status_mask = 0;
	if (termios->c_iflag & IGNPAR)
		port->ignore_status_mask |= PLUART_STATUS_PARITY
			| PLUART_STATUS_FRAME | PLUART_STATUS_OVERRUN;

	/* ignore all characters if CREAD is not set */
	if ((termios->c_cflag & CREAD) == 0)
		port->ignore_status_mask |=
			PLUART_STATUS_RXVALID | PLUART_STATUS_PARITY
			| PLUART_STATUS_FRAME | PLUART_STATUS_OVERRUN;

	/* update timeout */
	baud = uart_get_baud_rate(port, termios, old, 0, 460800);
	uart_update_timeout(port, termios->c_cflag, baud);

	spin_unlock_irqrestore(&port->lock, flags);
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
	struct uartlite_data *pdata = port->private_data;

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

	pdata->reg_ops = &uartlite_le;

	return 0;
}

static void pluart_config_port(struct uart_port *port, int flags)
{
	if (!pluart_request_port(port))
		port->type = PORT_UARTLITE;
}

static int pluart_verify_port(struct uart_port *port, struct serial_struct *ser)
{
	/* we don't want the core code to modify any port params */
	return -EINVAL;
}

static void pluart_pm(struct uart_port *port, unsigned int state,
		     unsigned int oldstate)
{
	struct uartlite_data *pdata = port->private_data;

	if (!state)
		clk_enable(pdata->clk);
	else
		clk_disable(pdata->clk);
}

#ifdef CONFIG_CONSOLE_POLL
static int pluart_get_poll_char(struct uart_port *port)
{
	if (!(uart_in32(PLUART_STATUS, port) & PLUART_STATUS_RXVALID))
		return NO_POLL_CHAR;

	return uart_in32(PLUART_RX, port);
}

static void pluart_put_poll_char(struct uart_port *port, unsigned char ch)
{
	while (uart_in32(PLUART_STATUS, port) & PLUART_STATUS_TXFULL)
		cpu_relax();

	/* write char to device */
	uart_out32(ch, PLUART_TX, port);
}
#endif

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
	.type		= pluart_type,
	.release_port	= pluart_release_port,
	.request_port	= pluart_request_port,
	.config_port	= pluart_config_port,
	.verify_port	= pluart_verify_port,
	.pm		= pluart_pm,
#ifdef CONFIG_CONSOLE_POLL
	.poll_get_char	= pluart_get_poll_char,
	.poll_put_char	= pluart_put_poll_char,
#endif
};

/* ---------------------------------------------------------------------
 * Console driver operations
 */

#ifdef CONFIG_SERIAL_UARTLITE_CONSOLE
static void pluart_console_wait_tx(struct uart_port *port)
{
	u8 val;
	unsigned long timeout;

	/*
	 * Spin waiting for TX fifo to have space available.
	 * When using the Microblaze Debug Module this can take up to 1s
	 */
	timeout = jiffies + msecs_to_jiffies(1000);
	while (1) {
		val = uart_in32(PLUART_STATUS, port);
		if ((val & PLUART_STATUS_TXFULL) == 0)
			break;
		if (time_after(jiffies, timeout)) {
			dev_warn(port->dev,
				 "timeout waiting for TX buffer empty\n");
			break;
		}
		cpu_relax();
	}
}

static void pluart_console_putchar(struct uart_port *port, int ch)
{
	pluart_console_wait_tx(port);
	uart_out32(ch, PLUART_TX, port);
}

static void pluart_console_write(struct console *co, const char *s,
				unsigned int count)
{
	struct uart_port *port = console_port;
	unsigned long flags;
	unsigned int ier;
	int locked = 1;

	if (oops_in_progress)
		locked = spin_trylock_irqsave(&port->lock, flags);
	else
		spin_lock_irqsave(&port->lock, flags);

	/* save and disable interrupt */
	ier = uart_in32(PLUART_STATUS, port) & PLUART_STATUS_IE;
	uart_out32(0, PLUART_CONTROL, port);

	uart_console_write(port, s, count, pluart_console_putchar);

	pluart_console_wait_tx(port);

	/* restore interrupt state */
	if (ier)
		uart_out32(PLUART_CONTROL_IE, PLUART_CONTROL, port);

	if (locked)
		spin_unlock_irqrestore(&port->lock, flags);
}

static int pluart_console_setup(struct console *co, char *options)
{
	struct uart_port *port;
	int baud = 9600;
	int bits = 8;
	int parity = 'n';
	int flow = 'n';


	port = console_port;

	/* Has the device been initialized yet? */
	if (!port->mapbase) {
		pr_debug("console on ttyUL%i not present\n", co->index);
		return -ENODEV;
	}

	/* not initialized yet? */
	if (!port->membase) {
		if (pluart_request_port(port))
			return -ENODEV;
	}

	if (options)
		uart_parse_options(options, &baud, &parity, &bits, &flow);

	return uart_set_options(port, co, baud, parity, bits, flow);
}

static struct uart_driver pluart_uart_driver;

static struct console pluart_console = {
	.name	= PLUART_NAME,
	.write	= pluart_console_write,
	.device	= uart_console_device,
	.setup	= pluart_console_setup,
	.flags	= CON_PRINTBUFFER,
	.index	= -1, /* Specified on the cmdline (e.g. console=ttyUL0 ) */
	.data	= &pluart_uart_driver,
};

static void early_uartlite_putc(struct uart_port *port, int c)
{
	/*
	 * Limit how many times we'll spin waiting for TX FIFO status.
	 * This will prevent lockups if the base address is incorrectly
	 * set, or any other issue on the UARTLITE.
	 * This limit is pretty arbitrary, unless we are at about 10 baud
	 * we'll never timeout on a working UART.
	 */

	unsigned int retries = 1000000;
	/* read status bit - 0x8 offset */
	while (--retries && (readl(port->membase + 8) & (1 << 3)))
		;

	/* Only attempt the iowrite if we didn't timeout */
	/* write to TX_FIFO - 0x4 offset */
	if (retries)
		writel(c & 0xff, port->membase + 4);
}

static void early_uartlite_write(struct console *console,
				 const char *s, unsigned int n)
{
	struct earlycon_device *device = console->data;

	uart_console_write(&device->port, s, n, early_uartlite_putc);
}

static int __init early_uartlite_setup(struct earlycon_device *device,
				       const char *options)
{
	if (!device->port.membase)
		return -ENODEV;

	device->con->write = early_uartlite_write;
	return 0;
}
EARLYCON_DECLARE(uartlite, early_uartlite_setup);
OF_EARLYCON_DECLARE(uartlite_b, "anlogic,opb-uartlite-1.00.b", early_uartlite_setup);
OF_EARLYCON_DECLARE(uartlite_a, "anlogic,xps-uartlite-1.00.a", early_uartlite_setup);

#endif /* CONFIG_SERIAL_UARTLITE_CONSOLE */

static struct uart_driver pluart_uart_driver = {
	.owner		= THIS_MODULE,
	.driver_name	= "uartlite",
	.dev_name	= PLUART_NAME,
	.major		= PLUART_MAJOR,
	.minor		= PLUART_MINOR,
	.nr		= PLUART_NR_UARTS,
#ifdef CONFIG_SERIAL_UARTLITE_CONSOLE
	.cons		= &pluart_console,
#endif
};

/* ---------------------------------------------------------------------
 * Port assignment functions (mapping devices to uart_port structures)
 */

/** pluart_assign: register a uartlite device with the driver
 *
 * @dev: pointer to device structure
 * @id: requested id number.  Pass -1 for automatic port assignment
 * @base: base address of uartlite registers
 * @irq: irq number for uartlite
 * @pdata: private data for uartlite
 *
 * Returns: 0 on success, <0 otherwise
 */
static int pluart_assign(struct device *dev, int id, u32 base, int irq,
			struct uartlite_data *pdata)
{
	struct uart_port *port;
	int rc;

	/* if id = -1; then scan for a free id and use that */
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
	port->iobase = 1; /* mark port in use */
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

#ifdef CONFIG_SERIAL_UARTLITE_CONSOLE
	/*
	 * If console hasn't been found yet try to assign this port
	 * because it is required to be assigned for console setup function.
	 * If register_console() don't assign value, then console_port pointer
	 * is cleanup.
	 */
	if (pluart_uart_driver.cons->index == -1)
		console_port = port;
#endif

	/* Register the port */
	rc = uart_add_one_port(&pluart_uart_driver, port);
	if (rc) {
		dev_err(dev, "uart_add_one_port() failed; err=%i\n", rc);
		port->mapbase = 0;
		dev_set_drvdata(dev, NULL);
		return rc;
	}

#ifdef CONFIG_SERIAL_UARTLITE_CONSOLE
	/* This is not port which is used for console that's why clean it up */
	if (pluart_uart_driver.cons->index == -1)
		console_port = NULL;
#endif

	return 0;
}

/** pluart_release: register a uartlite device with the driver
 *
 * @dev: pointer to device structure
 */
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

/**
 * pluart_suspend - Stop the device.
 *
 * @dev: handle to the device structure.
 * Return: 0 always.
 */
static int __maybe_unused pluart_suspend(struct device *dev)
{
	struct uart_port *port = dev_get_drvdata(dev);

	if (port)
		uart_suspend_port(&pluart_uart_driver, port);

	return 0;
}

/**
 * pluart_resume - Resume the device.
 *
 * @dev: handle to the device structure.
 * Return: 0 on success, errno otherwise.
 */
static int __maybe_unused pluart_resume(struct device *dev)
{
	struct uart_port *port = dev_get_drvdata(dev);

	if (port)
		uart_resume_port(&pluart_uart_driver, port);

	return 0;
}

/* ---------------------------------------------------------------------
 * Platform bus binding
 */

static SIMPLE_DEV_PM_OPS(pluart_pm_ops, pluart_suspend, pluart_resume);

#if defined(CONFIG_OF)
/* Match table for of_platform binding */
static const struct of_device_id pluart_of_match[] = {
	{ .compatible = "anlogic,xps-uartlite-1.00.a", },
	{}
};
MODULE_DEVICE_TABLE(of, pluart_of_match);
#endif /* CONFIG_OF */

static int pluart_probe(struct platform_device *pdev)
{
	struct resource *res;
	struct uartlite_data *pdata;
	int irq, ret;
	int id = pdev->id;
#ifdef CONFIG_OF
	const __be32 *prop;

	prop = of_get_property(pdev->dev.of_node, "port-number", NULL);
	if (prop)
		id = be32_to_cpup(prop);
#endif
	pdata = devm_kzalloc(&pdev->dev, sizeof(struct uartlite_data),
			     GFP_KERNEL);
	if (!pdata)
		return -ENOMEM;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res)
		return -ENODEV;

	irq = platform_get_irq(pdev, 0);
	if (irq <= 0)
		return -ENXIO;

	pdata->clk = devm_clk_get(&pdev->dev, "s_axi_aclk");
	if (IS_ERR(pdata->clk)) {
		if (PTR_ERR(pdata->clk) != -ENOENT)
			return PTR_ERR(pdata->clk);

		/*
		 * Clock framework support is optional, continue on
		 * anyways if we don't find a matching clock.
		 */
		pdata->clk = NULL;
	}

	ret = clk_prepare_enable(pdata->clk);
	if (ret) {
		dev_err(&pdev->dev, "Failed to prepare clock\n");
		return ret;
	}

	if (!pluart_uart_driver.state) {
		dev_dbg(&pdev->dev, "uartlite: calling uart_register_driver()\n");
		ret = uart_register_driver(&pluart_uart_driver);
		if (ret < 0) {
			dev_err(&pdev->dev, "Failed to register driver\n");
			return ret;
		}
	}

	ret = pluart_assign(&pdev->dev, id, res->start, irq, pdata);

	clk_disable(pdata->clk);

	return ret;
}

static int pluart_remove(struct platform_device *pdev)
{
	struct uart_port *port = dev_get_drvdata(&pdev->dev);
	struct uartlite_data *pdata = port->private_data;

	clk_disable_unprepare(pdata->clk);
	return pluart_release(&pdev->dev);
}

/* work with hotplug and coldplug */
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

/* ---------------------------------------------------------------------
 * Module setup/teardown
 */

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
MODULE_DESCRIPTION("pl_uart serial driver");
MODULE_LICENSE("GPL");
