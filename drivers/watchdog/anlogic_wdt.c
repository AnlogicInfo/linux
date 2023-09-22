// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2023 Anlogic, Inc.
 *
*/

#include <linux/bitops.h>
#include <linux/limits.h>
#include <linux/kernel.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/interrupt.h>
#include <linux/of.h>
#include <linux/pm.h>
#include <linux/platform_device.h>
#include <linux/reset.h>
#include <linux/watchdog.h>
#include <linux/debugfs.h>

#define WDOG_CONTROL_REG_OFFSET		    0x00
#define WDOG_CONTROL_REG_WDT_EN_MASK	    0x01
#define WDOG_CONTROL_REG_RESP_MODE_MASK	    0x02
#define WDOG_TIMEOUT_RANGE_REG_OFFSET	    0x04
#define WDOG_TIMEOUT_RANGE_TOPINIT_SHIFT    4
#define WDOG_CURRENT_COUNT_REG_OFFSET	    0x08
#define WDOG_COUNTER_RESTART_REG_OFFSET     0x0c
#define WDOG_COUNTER_RESTART_KICK_VALUE	    0x76
#define WDOG_INTERRUPT_STATUS_REG_OFFSET    0x10
#define WDOG_INTERRUPT_CLEAR_REG_OFFSET     0x14
#define WDOG_COMP_PARAMS_5_REG_OFFSET       0xe4
#define WDOG_COMP_PARAMS_4_REG_OFFSET       0xe8
#define WDOG_COMP_PARAMS_3_REG_OFFSET       0xec
#define WDOG_COMP_PARAMS_2_REG_OFFSET       0xf0
#define WDOG_COMP_PARAMS_1_REG_OFFSET       0xf4
#define WDOG_COMP_PARAMS_1_USE_FIX_TOP      BIT(6)
#define WDOG_COMP_VERSION_REG_OFFSET        0xf8
#define WDOG_COMP_TYPE_REG_OFFSET           0xfc

/* There are sixteen TOPs (timeout periods) that can be set in the watchdog. */
#define ANLOGIC_DW_WDT_NUM_TOPS		16
#define ANLOGIC_DW_WDT_FIX_TOP(_idx)	(1U << (16 + _idx))

#define ANLOGIC_DW_WDT_DEFAULT_SECONDS	30

static const u32 anlogic_dw_wdt_fix_tops[ANLOGIC_DW_WDT_NUM_TOPS] = {
	ANLOGIC_DW_WDT_FIX_TOP(0), ANLOGIC_DW_WDT_FIX_TOP(1), ANLOGIC_DW_WDT_FIX_TOP(2),
	ANLOGIC_DW_WDT_FIX_TOP(3), ANLOGIC_DW_WDT_FIX_TOP(4), ANLOGIC_DW_WDT_FIX_TOP(5),
	ANLOGIC_DW_WDT_FIX_TOP(6), ANLOGIC_DW_WDT_FIX_TOP(7), ANLOGIC_DW_WDT_FIX_TOP(8),
	ANLOGIC_DW_WDT_FIX_TOP(9), ANLOGIC_DW_WDT_FIX_TOP(10), ANLOGIC_DW_WDT_FIX_TOP(11),
	ANLOGIC_DW_WDT_FIX_TOP(12), ANLOGIC_DW_WDT_FIX_TOP(13), ANLOGIC_DW_WDT_FIX_TOP(14),
	ANLOGIC_DW_WDT_FIX_TOP(15)
};

static bool nowayout = WATCHDOG_NOWAYOUT;
module_param(nowayout, bool, 0);
MODULE_PARM_DESC(nowayout, "Watchdog cannot be stopped once started "
		 "(default=" __MODULE_STRING(WATCHDOG_NOWAYOUT) ")");

enum anlogic_dw_wdt_rmod {
	ANLOGIC_DW_WDT_RMOD_RESET = 1,
	ANLOGIC_DW_WDT_RMOD_IRQ = 2
};

struct anlogic_dw_wdt_timeout {
	u32 top_val;
	unsigned int sec;
	unsigned int msec;
};

struct anlogic_dw_wdt {
	void __iomem		*regs;
	void __iomem		*top_regs;
	struct clk		*clk;
	struct clk		*pclk;
	unsigned long		rate;
	enum anlogic_dw_wdt_rmod	rmod;
	struct anlogic_dw_wdt_timeout	timeouts[ANLOGIC_DW_WDT_NUM_TOPS];
	struct watchdog_device	wdd;
	struct reset_control	*rst;
	/* Save/restore */
	u32			control;
	u32			timeout;

#ifdef CONFIG_DEBUG_FS
	struct dentry		*dbgfs_dir;
#endif
};

#define to_anlogic_dw_wdt(wdd)	container_of(wdd, struct anlogic_dw_wdt, wdd)

static inline int anlogic_dw_wdt_is_enabled(struct anlogic_dw_wdt *anlogic_dw_wdt)
{
	return readl(anlogic_dw_wdt->regs + WDOG_CONTROL_REG_OFFSET) &
		WDOG_CONTROL_REG_WDT_EN_MASK;
}

static void anlogic_dw_wdt_update_mode(struct anlogic_dw_wdt *anlogic_dw_wdt, enum anlogic_dw_wdt_rmod rmod)
{
	u32 val;

	val = readl(anlogic_dw_wdt->regs + WDOG_CONTROL_REG_OFFSET);
	if (rmod == ANLOGIC_DW_WDT_RMOD_IRQ)
		val |= WDOG_CONTROL_REG_RESP_MODE_MASK;
	else
		val &= ~WDOG_CONTROL_REG_RESP_MODE_MASK;
	writel(val, anlogic_dw_wdt->regs + WDOG_CONTROL_REG_OFFSET);

	anlogic_dw_wdt->rmod = rmod;
}

static unsigned int anlogic_dw_wdt_find_best_top(struct anlogic_dw_wdt *anlogic_dw_wdt,
					 unsigned int timeout, u32 *top_val)
{
	int idx;

	/*
	 * Find a TOP with timeout greater or equal to the requested number.
	 * Note we'll select a TOP with maximum timeout if the requested
	 * timeout couldn't be reached.
	 */
	for (idx = 0; idx < ANLOGIC_DW_WDT_NUM_TOPS; ++idx) {
		if (anlogic_dw_wdt->timeouts[idx].sec >= timeout)
			break;
	}

	if (idx == ANLOGIC_DW_WDT_NUM_TOPS)
		--idx;

	*top_val = anlogic_dw_wdt->timeouts[idx].top_val;

	return anlogic_dw_wdt->timeouts[idx].sec;
}

static unsigned int anlogic_dw_wdt_get_min_timeout(struct anlogic_dw_wdt *anlogic_dw_wdt)
{
	int idx;

	/*
	 * We'll find a timeout greater or equal to one second anyway because
	 * the driver probe would have failed if there was none.
	 */
	for (idx = 0; idx < ANLOGIC_DW_WDT_NUM_TOPS; ++idx) {
		if (anlogic_dw_wdt->timeouts[idx].sec)
			break;
	}

	return anlogic_dw_wdt->timeouts[idx].sec;
}

static unsigned int anlogic_dw_wdt_get_max_timeout_ms(struct anlogic_dw_wdt *anlogic_dw_wdt)
{
	struct anlogic_dw_wdt_timeout *timeout = &anlogic_dw_wdt->timeouts[ANLOGIC_DW_WDT_NUM_TOPS - 1];
	u64 msec;

	msec = (u64)timeout->sec * MSEC_PER_SEC + timeout->msec;

	return msec < UINT_MAX ? msec : UINT_MAX;
}

static unsigned int anlogic_dw_wdt_get_timeout(struct anlogic_dw_wdt *anlogic_dw_wdt)
{
	int top_val = readl(anlogic_dw_wdt->regs + WDOG_TIMEOUT_RANGE_REG_OFFSET) & 0xF;
	int idx;

	for (idx = 0; idx < ANLOGIC_DW_WDT_NUM_TOPS; ++idx) {
		if (anlogic_dw_wdt->timeouts[idx].top_val == top_val)
			break;
	}

	/*
	 * In IRQ mode due to the two stages counter, the actual timeout is
	 * twice greater than the TOP setting.
	 */
	return anlogic_dw_wdt->timeouts[idx].sec * anlogic_dw_wdt->rmod;
}

static int anlogic_dw_wdt_ping(struct watchdog_device *wdd)
{
	struct anlogic_dw_wdt *anlogic_dw_wdt = to_anlogic_dw_wdt(wdd);

	writel(WDOG_COUNTER_RESTART_KICK_VALUE, anlogic_dw_wdt->regs +
	       WDOG_COUNTER_RESTART_REG_OFFSET);

	return 0;
}

static int anlogic_dw_wdt_set_timeout(struct watchdog_device *wdd, unsigned int top_s)
{
	struct anlogic_dw_wdt *anlogic_dw_wdt = to_anlogic_dw_wdt(wdd);
	unsigned int timeout;
	u32 top_val;

	/*
	 * Note IRQ mode being enabled means having a non-zero pre-timeout
	 * setup. In this case we try to find a TOP as close to the half of the
	 * requested timeout as possible since ANLOGIC_DW Watchdog IRQ mode is designed
	 * in two stages way - first timeout rises the pre-timeout interrupt,
	 * second timeout performs the system reset. So basically the effective
	 * watchdog-caused reset happens after two watchdog TOPs elapsed.
	 */
	timeout = anlogic_dw_wdt_find_best_top(anlogic_dw_wdt, DIV_ROUND_UP(top_s, anlogic_dw_wdt->rmod),
				       &top_val);
	if (anlogic_dw_wdt->rmod == ANLOGIC_DW_WDT_RMOD_IRQ)
		wdd->pretimeout = timeout;
	else
		wdd->pretimeout = 0;

	/*
	 * Set the new value in the watchdog.  Some versions of anlogic_dw_wdt
	 * have have TOPINIT in the TIMEOUT_RANGE register (as per
	 * CP_WDT_DUAL_TOP in WDT_COMP_PARAMS_1).  On those we
	 * effectively get a pat of the watchdog right here.
	 */
	writel(top_val | top_val << WDOG_TIMEOUT_RANGE_TOPINIT_SHIFT,
	       anlogic_dw_wdt->regs + WDOG_TIMEOUT_RANGE_REG_OFFSET);

	/* Kick new TOP value into the watchdog counter if activated. */
	if (watchdog_active(wdd))
		anlogic_dw_wdt_ping(wdd);

	/*
	 * In case users set bigger timeout value than HW can support,
	 * kernel(watchdog_dev.c) helps to feed watchdog before
	 * wdd->max_hw_heartbeat_ms
	 */
	if (top_s * 1000 <= wdd->max_hw_heartbeat_ms)
		wdd->timeout = timeout * anlogic_dw_wdt->rmod;
	else
		wdd->timeout = top_s;

	return 0;
}

static int anlogic_dw_wdt_set_pretimeout(struct watchdog_device *wdd, unsigned int req)
{
	struct anlogic_dw_wdt *anlogic_dw_wdt = to_anlogic_dw_wdt(wdd);

	/*
	 * We ignore actual value of the timeout passed from user-space
	 * using it as a flag whether the pretimeout functionality is intended
	 * to be activated.
	 */
	anlogic_dw_wdt_update_mode(anlogic_dw_wdt, req ? ANLOGIC_DW_WDT_RMOD_IRQ : ANLOGIC_DW_WDT_RMOD_RESET);
	anlogic_dw_wdt_set_timeout(wdd, wdd->timeout);

	return 0;
}

static void anlogic_dw_wdt_arm_system_reset(struct anlogic_dw_wdt *anlogic_dw_wdt)
{
	u32 val = readl(anlogic_dw_wdt->regs + WDOG_CONTROL_REG_OFFSET);
	u32 wdt_pause_status = readl(anlogic_dw_wdt->top_regs);

	/* Disable/enable interrupt mode depending on the RMOD flag. */
	if (anlogic_dw_wdt->rmod == ANLOGIC_DW_WDT_RMOD_IRQ)
		val |= WDOG_CONTROL_REG_RESP_MODE_MASK;
	else
		val &= ~WDOG_CONTROL_REG_RESP_MODE_MASK;
	/* Enable watchdog. */
	val |= WDOG_CONTROL_REG_WDT_EN_MASK;
	writel(val, anlogic_dw_wdt->regs + WDOG_CONTROL_REG_OFFSET);

	/* set the watchdog0 not to pause */
	writel(wdt_pause_status&0xfffffffe, anlogic_dw_wdt->top_regs);

}

static int anlogic_dw_wdt_start(struct watchdog_device *wdd)
{
	struct anlogic_dw_wdt *anlogic_dw_wdt = to_anlogic_dw_wdt(wdd);

	anlogic_dw_wdt_set_timeout(wdd, wdd->timeout);
	anlogic_dw_wdt_ping(&anlogic_dw_wdt->wdd);
	anlogic_dw_wdt_arm_system_reset(anlogic_dw_wdt);

	return 0;
}

static int anlogic_dw_wdt_stop(struct watchdog_device *wdd)
{
	struct anlogic_dw_wdt *anlogic_dw_wdt = to_anlogic_dw_wdt(wdd);

	if (!anlogic_dw_wdt->rst) {
		set_bit(WDOG_HW_RUNNING, &wdd->status);
		return 0;
	}

	reset_control_assert(anlogic_dw_wdt->rst);
	reset_control_deassert(anlogic_dw_wdt->rst);

	return 0;
}

static int anlogic_dw_wdt_restart(struct watchdog_device *wdd,
			  unsigned long action, void *data)
{
	struct anlogic_dw_wdt *anlogic_dw_wdt = to_anlogic_dw_wdt(wdd);
	u32 wdt_pause_status = readl(anlogic_dw_wdt->top_regs);

	writel(0, anlogic_dw_wdt->regs + WDOG_TIMEOUT_RANGE_REG_OFFSET);
	anlogic_dw_wdt_update_mode(anlogic_dw_wdt, ANLOGIC_DW_WDT_RMOD_RESET);
	if (anlogic_dw_wdt_is_enabled(anlogic_dw_wdt)) {
		/* set the watchdog0 not to pause */
		writel(wdt_pause_status&0xfffffffe, anlogic_dw_wdt->top_regs);

		writel(WDOG_COUNTER_RESTART_KICK_VALUE,
		       anlogic_dw_wdt->regs + WDOG_COUNTER_RESTART_REG_OFFSET);
	}

	else
		anlogic_dw_wdt_arm_system_reset(anlogic_dw_wdt);

	/* wait for reset to assert... */
	mdelay(500);

	return 0;
}

static unsigned int anlogic_dw_wdt_get_timeleft(struct watchdog_device *wdd)
{
	struct anlogic_dw_wdt *anlogic_dw_wdt = to_anlogic_dw_wdt(wdd);
	unsigned int sec;
	u32 val;

	val = readl(anlogic_dw_wdt->regs + WDOG_CURRENT_COUNT_REG_OFFSET);
	sec = val / anlogic_dw_wdt->rate;

	if (anlogic_dw_wdt->rmod == ANLOGIC_DW_WDT_RMOD_IRQ) {
		val = readl(anlogic_dw_wdt->regs + WDOG_INTERRUPT_STATUS_REG_OFFSET);
		if (!val)
			sec += wdd->pretimeout;
	}

	return sec;
}

static const struct watchdog_info anlogic_dw_wdt_ident = {
	.options	= WDIOF_KEEPALIVEPING | WDIOF_SETTIMEOUT |
			  WDIOF_MAGICCLOSE,
	.identity	= "Anlogic DesignWare Watchdog",
};

static const struct watchdog_info anlogic_dw_wdt_pt_ident = {
	.options	= WDIOF_KEEPALIVEPING | WDIOF_SETTIMEOUT |
			  WDIOF_PRETIMEOUT | WDIOF_MAGICCLOSE,
	.identity	= "Anlogic DesignWare Watchdog",
};

static const struct watchdog_ops anlogic_dw_wdt_ops = {
	.owner		= THIS_MODULE,
	.start		= anlogic_dw_wdt_start,
	.stop		= anlogic_dw_wdt_stop,
	.ping		= anlogic_dw_wdt_ping,
	.set_timeout	= anlogic_dw_wdt_set_timeout,
	.set_pretimeout	= anlogic_dw_wdt_set_pretimeout,
	.get_timeleft	= anlogic_dw_wdt_get_timeleft,
	.restart	= anlogic_dw_wdt_restart,
};

static irqreturn_t anlogic_dw_wdt_irq(int irq, void *devid)
{
	struct anlogic_dw_wdt *anlogic_dw_wdt = devid;
	u32 val;

	/*
	 * We don't clear the IRQ status. It's supposed to be done by the
	 * following ping operations.
	 */
	val = readl(anlogic_dw_wdt->regs + WDOG_INTERRUPT_STATUS_REG_OFFSET);
	if (!val)
		return IRQ_NONE;

	watchdog_notify_pretimeout(&anlogic_dw_wdt->wdd);

	return IRQ_HANDLED;
}

#ifdef CONFIG_PM_SLEEP
static int anlogic_dw_wdt_suspend(struct device *dev)
{
	struct anlogic_dw_wdt *anlogic_dw_wdt = dev_get_drvdata(dev);

	anlogic_dw_wdt->control = readl(anlogic_dw_wdt->regs + WDOG_CONTROL_REG_OFFSET);
	anlogic_dw_wdt->timeout = readl(anlogic_dw_wdt->regs + WDOG_TIMEOUT_RANGE_REG_OFFSET);

	clk_disable_unprepare(anlogic_dw_wdt->pclk);
	clk_disable_unprepare(anlogic_dw_wdt->clk);

	return 0;
}

static int anlogic_dw_wdt_resume(struct device *dev)
{
	struct anlogic_dw_wdt *anlogic_dw_wdt = dev_get_drvdata(dev);
	int err = clk_prepare_enable(anlogic_dw_wdt->clk);

	if (err)
		return err;

	err = clk_prepare_enable(anlogic_dw_wdt->pclk);
	if (err) {
		clk_disable_unprepare(anlogic_dw_wdt->clk);
		return err;
	}

	writel(anlogic_dw_wdt->timeout, anlogic_dw_wdt->regs + WDOG_TIMEOUT_RANGE_REG_OFFSET);
	writel(anlogic_dw_wdt->control, anlogic_dw_wdt->regs + WDOG_CONTROL_REG_OFFSET);

	anlogic_dw_wdt_ping(&anlogic_dw_wdt->wdd);

	return 0;
}
#endif /* CONFIG_PM_SLEEP */

static SIMPLE_DEV_PM_OPS(anlogic_dw_wdt_pm_ops, anlogic_dw_wdt_suspend, anlogic_dw_wdt_resume);

/*
 * In case if ANLOGIC DW WDT IP core is synthesized with fixed TOP feature disabled the
 * TOPs array can be arbitrary ordered with nearly any sixteen uint numbers
 * depending on the system engineer imagination. The next method handles the
 * passed TOPs array to pre-calculate the effective timeouts and to sort the
 * TOP items out in the ascending order with respect to the timeouts.
 */

static void anlogic_dw_wdt_handle_tops(struct anlogic_dw_wdt *anlogic_dw_wdt, const u32 *tops)
{
	struct anlogic_dw_wdt_timeout tout, *dst;
	int val, tidx;
	u64 msec;

	/*
	 * We walk over the passed TOPs array and calculate corresponding
	 * timeouts in seconds and milliseconds. The milliseconds granularity
	 * is needed to distinguish the TOPs with very close timeouts and to
	 * set the watchdog max heartbeat setting further.
	 */
	for (val = 0; val < ANLOGIC_DW_WDT_NUM_TOPS; ++val) {
		tout.top_val = val;
		tout.sec = tops[val] / anlogic_dw_wdt->rate;
		msec = (u64)tops[val] * MSEC_PER_SEC;
		do_div(msec, anlogic_dw_wdt->rate);
		tout.msec = msec - ((u64)tout.sec * MSEC_PER_SEC);

		/*
		 * Find a suitable place for the current TOP in the timeouts
		 * array so that the list is remained in the ascending order.
		 */
		for (tidx = 0; tidx < val; ++tidx) {
			dst = &anlogic_dw_wdt->timeouts[tidx];
			if (tout.sec > dst->sec || (tout.sec == dst->sec &&
			    tout.msec >= dst->msec))
				continue;
			else
				swap(*dst, tout);
		}

		anlogic_dw_wdt->timeouts[val] = tout;
	}
}

static int anlogic_dw_wdt_init_timeouts(struct anlogic_dw_wdt *anlogic_dw_wdt, struct device *dev)
{
	u32 data, of_tops[ANLOGIC_DW_WDT_NUM_TOPS];
	const u32 *tops;
	int ret;

	/*
	 * Retrieve custom or fixed counter values depending on the
	 * WDT_USE_FIX_TOP flag found in the component specific parameters
	 * #1 register.
	 */
	data = readl(anlogic_dw_wdt->regs + WDOG_COMP_PARAMS_1_REG_OFFSET);
	if (data & WDOG_COMP_PARAMS_1_USE_FIX_TOP) {
		tops = anlogic_dw_wdt_fix_tops;
	} else {
		ret = of_property_read_variable_u32_array(dev_of_node(dev),
			"anlogic_dw,watchdog-tops", of_tops, ANLOGIC_DW_WDT_NUM_TOPS,
			ANLOGIC_DW_WDT_NUM_TOPS);
		if (ret < 0) {
			dev_warn(dev, "No valid TOPs array specified\n");
			tops = anlogic_dw_wdt_fix_tops;
		} else {
			tops = of_tops;
		}
	}

	/* Convert the specified TOPs into an array of watchdog timeouts. */
	anlogic_dw_wdt_handle_tops(anlogic_dw_wdt, tops);
	if (!anlogic_dw_wdt->timeouts[ANLOGIC_DW_WDT_NUM_TOPS - 1].sec) {
		dev_err(dev, "No any valid TOP detected\n");
		return -EINVAL;
	}

	return 0;
}

#ifdef CONFIG_DEBUG_FS

#define ANLOGIC_DW_WDT_DBGFS_REG(_name, _off) \
{				      \
	.name = _name,		      \
	.offset = _off		      \
}

static const struct debugfs_reg32 anlogic_dw_wdt_dbgfs_regs[] = {
	ANLOGIC_DW_WDT_DBGFS_REG("cr", WDOG_CONTROL_REG_OFFSET),
	ANLOGIC_DW_WDT_DBGFS_REG("torr", WDOG_TIMEOUT_RANGE_REG_OFFSET),
	ANLOGIC_DW_WDT_DBGFS_REG("ccvr", WDOG_CURRENT_COUNT_REG_OFFSET),
	ANLOGIC_DW_WDT_DBGFS_REG("crr", WDOG_COUNTER_RESTART_REG_OFFSET),
	ANLOGIC_DW_WDT_DBGFS_REG("stat", WDOG_INTERRUPT_STATUS_REG_OFFSET),
	ANLOGIC_DW_WDT_DBGFS_REG("param5", WDOG_COMP_PARAMS_5_REG_OFFSET),
	ANLOGIC_DW_WDT_DBGFS_REG("param4", WDOG_COMP_PARAMS_4_REG_OFFSET),
	ANLOGIC_DW_WDT_DBGFS_REG("param3", WDOG_COMP_PARAMS_3_REG_OFFSET),
	ANLOGIC_DW_WDT_DBGFS_REG("param2", WDOG_COMP_PARAMS_2_REG_OFFSET),
	ANLOGIC_DW_WDT_DBGFS_REG("param1", WDOG_COMP_PARAMS_1_REG_OFFSET),
	ANLOGIC_DW_WDT_DBGFS_REG("version", WDOG_COMP_VERSION_REG_OFFSET),
	ANLOGIC_DW_WDT_DBGFS_REG("type", WDOG_COMP_TYPE_REG_OFFSET)
};

static void anlogic_dw_wdt_dbgfs_init(struct anlogic_dw_wdt *anlogic_dw_wdt)
{
	struct device *dev = anlogic_dw_wdt->wdd.parent;
	struct debugfs_regset32 *regset;

	regset = devm_kzalloc(dev, sizeof(*regset), GFP_KERNEL);
	if (!regset)
		return;

	regset->regs = anlogic_dw_wdt_dbgfs_regs;
	regset->nregs = ARRAY_SIZE(anlogic_dw_wdt_dbgfs_regs);
	regset->base = anlogic_dw_wdt->regs;

	anlogic_dw_wdt->dbgfs_dir = debugfs_create_dir(dev_name(dev), NULL);

	debugfs_create_regset32("registers", 0444, anlogic_dw_wdt->dbgfs_dir, regset);
}

static void anlogic_dw_wdt_dbgfs_clear(struct anlogic_dw_wdt *anlogic_dw_wdt)
{
	debugfs_remove_recursive(anlogic_dw_wdt->dbgfs_dir);
}

#else /* !CONFIG_DEBUG_FS */

static void anlogic_dw_wdt_dbgfs_init(struct anlogic_dw_wdt *anlogic_dw_wdt) {}
static void anlogic_dw_wdt_dbgfs_clear(struct anlogic_dw_wdt *anlogic_dw_wdt) {}

#endif /* !CONFIG_DEBUG_FS */

static int anlogic_dw_wdt_drv_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct watchdog_device *wdd;
	struct anlogic_dw_wdt *anlogic_dw_wdt;
	int ret;
	u32 wdt_pause_status;

	anlogic_dw_wdt = devm_kzalloc(dev, sizeof(*anlogic_dw_wdt), GFP_KERNEL);
	if (!anlogic_dw_wdt)
		return -ENOMEM;

	anlogic_dw_wdt->regs = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(anlogic_dw_wdt->regs))
		return PTR_ERR(anlogic_dw_wdt->regs);

	anlogic_dw_wdt->top_regs = devm_platform_ioremap_resource(pdev, 1);
	if (IS_ERR(anlogic_dw_wdt->top_regs))
		return PTR_ERR(anlogic_dw_wdt->top_regs);

	/*
	 * Try to request the watchdog dedicated timer clock source. It must
	 * be supplied if asynchronous mode is enabled. Otherwise fallback
	 * to the common timer/bus clocks configuration, in which the very
	 * first found clock supply both timer and APB signals.
	 */
	anlogic_dw_wdt->clk = devm_clk_get(dev, "tclk");
	if (IS_ERR(anlogic_dw_wdt->clk)) {
		anlogic_dw_wdt->clk = devm_clk_get(dev, NULL);
		if (IS_ERR(anlogic_dw_wdt->clk))
			return PTR_ERR(anlogic_dw_wdt->clk);
	}

	ret = clk_prepare_enable(anlogic_dw_wdt->clk);
	if (ret)
		return ret;

	anlogic_dw_wdt->rate = clk_get_rate(anlogic_dw_wdt->clk);
	if (anlogic_dw_wdt->rate == 0) {
		ret = -EINVAL;
		goto out_disable_clk;
	}

	/*
	 * Request APB clock if device is configured with async clocks mode.
	 * In this case both tclk and pclk clocks are supposed to be specified.
	 * Alas we can't know for sure whether async mode was really activated,
	 * so the pclk phandle reference is left optional. If it couldn't be
	 * found we consider the device configured in synchronous clocks mode.
	 */
	anlogic_dw_wdt->pclk = devm_clk_get_optional(dev, "pclk");
	if (IS_ERR(anlogic_dw_wdt->pclk)) {
		ret = PTR_ERR(anlogic_dw_wdt->pclk);
		goto out_disable_clk;
	}

	ret = clk_prepare_enable(anlogic_dw_wdt->pclk);
	if (ret)
		goto out_disable_clk;

	anlogic_dw_wdt->rst = devm_reset_control_get_optional_shared(&pdev->dev, NULL);
	if (IS_ERR(anlogic_dw_wdt->rst)) {
		ret = PTR_ERR(anlogic_dw_wdt->rst);
		goto out_disable_pclk;
	}

	/* Enable normal reset without pre-timeout by default. */
	anlogic_dw_wdt_update_mode(anlogic_dw_wdt, ANLOGIC_DW_WDT_RMOD_RESET);

	/*
	 * Pre-timeout IRQ is optional, since some haranlogic_dware may lack support
	 * of it. Note we must request rising-edge IRQ, since the lane is left
	 * pending either until the next watchdog kick event or up to the
	 * system reset.
	 */
	ret = platform_get_irq_optional(pdev, 0);
	if (ret > 0) {
		ret = devm_request_irq(dev, ret, anlogic_dw_wdt_irq,
				       IRQF_SHARED | IRQF_TRIGGER_RISING,
				       pdev->name, anlogic_dw_wdt);
		if (ret)
			goto out_disable_pclk;

		anlogic_dw_wdt->wdd.info = &anlogic_dw_wdt_pt_ident;
	} else {
		if (ret == -EPROBE_DEFER)
			goto out_disable_pclk;

		anlogic_dw_wdt->wdd.info = &anlogic_dw_wdt_ident;
	}

	reset_control_deassert(anlogic_dw_wdt->rst);

	ret = anlogic_dw_wdt_init_timeouts(anlogic_dw_wdt, dev);
	if (ret)
		goto out_disable_clk;

	wdd = &anlogic_dw_wdt->wdd;
	wdd->ops = &anlogic_dw_wdt_ops;
	wdd->min_timeout = anlogic_dw_wdt_get_min_timeout(anlogic_dw_wdt);
	wdd->max_hw_heartbeat_ms = anlogic_dw_wdt_get_max_timeout_ms(anlogic_dw_wdt);
	wdd->parent = dev;

	watchdog_set_drvdata(wdd, anlogic_dw_wdt);
	watchdog_set_nowayout(wdd, nowayout);
	watchdog_init_timeout(wdd, 0, dev);

	/*
	 * If the watchdog is already running, use its already configured
	 * timeout. Otherwise use the default or the value provided through
	 * devicetree.
	 */
	if (anlogic_dw_wdt_is_enabled(anlogic_dw_wdt)) {
		wdd->timeout = anlogic_dw_wdt_get_timeout(anlogic_dw_wdt);
		set_bit(WDOG_HW_RUNNING, &wdd->status);

		/* set the watchdog0 not to pause */
		wdt_pause_status = readl(anlogic_dw_wdt->top_regs);
		writel(wdt_pause_status&0xfffffffe, anlogic_dw_wdt->top_regs);

	} else {
		wdd->timeout = ANLOGIC_DW_WDT_DEFAULT_SECONDS;
		watchdog_init_timeout(wdd, 0, dev);
	}

	platform_set_drvdata(pdev, anlogic_dw_wdt);

	watchdog_set_restart_priority(wdd, 128);

	ret = watchdog_register_device(wdd);
	if (ret)
		goto out_disable_pclk;

	anlogic_dw_wdt_dbgfs_init(anlogic_dw_wdt);

	return 0;

out_disable_pclk:
	clk_disable_unprepare(anlogic_dw_wdt->pclk);

out_disable_clk:
	clk_disable_unprepare(anlogic_dw_wdt->clk);
	return ret;
}

static int anlogic_dw_wdt_drv_remove(struct platform_device *pdev)
{
	u32 wdt_pause_status;
	struct anlogic_dw_wdt *anlogic_dw_wdt = platform_get_drvdata(pdev);

	anlogic_dw_wdt_dbgfs_clear(anlogic_dw_wdt);

	/* set the watchdog0 on pause status */
	wdt_pause_status = readl(anlogic_dw_wdt->top_regs);
	writel(wdt_pause_status|0x1, anlogic_dw_wdt->top_regs);


	watchdog_unregister_device(&anlogic_dw_wdt->wdd);
	reset_control_assert(anlogic_dw_wdt->rst);
	clk_disable_unprepare(anlogic_dw_wdt->pclk);
	clk_disable_unprepare(anlogic_dw_wdt->clk);

	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id anlogic_dw_wdt_of_match[] = {
	{ .compatible = "anlogic,dw-wdt", },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, anlogic_dw_wdt_of_match);
#endif

static struct platform_driver anlogic_dw_wdt_driver = {
	.probe		= anlogic_dw_wdt_drv_probe,
	.remove		= anlogic_dw_wdt_drv_remove,
	.driver		= {
		.name	= "anlogic_dw_wdt",
		.of_match_table = of_match_ptr(anlogic_dw_wdt_of_match),
		.pm	= &anlogic_dw_wdt_pm_ops,
	},
};

module_platform_driver(anlogic_dw_wdt_driver);

MODULE_AUTHOR("ZhangWeiKang");
MODULE_DESCRIPTION("Anlogic Watchdog Driver");
MODULE_LICENSE("GPL");
