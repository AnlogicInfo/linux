// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2025 Anlogic, Inc.
 */
#include <linux/device.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/of_address.h>
#include <linux/miscdevice.h>
#include <linux/sys_soc.h>
#include <linux/bitfield.h>
#include <linux/regmap.h>
#include <linux/slab.h>
#include <linux/mfd/syscon.h>
#if IS_ENABLED(CONFIG_ARCH_RV64I)
#include <asm/sbi.h>
#else
#include <linux/arm-smccc.h>
#include <asm/cpu_ops.h>
#endif
#include <uapi/linux/anl_top.h>
#include "anl_top_reg.h"

#if IS_ENABLED(CONFIG_ARCH_RV64I)
#define ALSIP_FPGA_PROG				(SBI_EXT_VENDOR_START + 0)
#else
/* Function ID passed during SiP SVC call. */
#define FN_RPROC_BASE				(0xFF000000)
#define FN_RPROC_OFF				(FN_RPROC_BASE + 21)
#define FN_RPROC_ON				(FN_RPROC_BASE + 22)
#define ALSIP_FPGA_PROG				(FN_RPROC_BASE + 23)
#define ALSIP_SYS_MISC				(FN_RPROC_BASE + 24)
#endif

// FPGA command
#define ALSIP_FPGA_PROG_START			0
#define ALSIP_FPGA_PROG_LOAD			1
#define ALSIP_FPGA_PROG_DONE			2
#define ALSIP_FPGA_PLCLK_RST			3
#define ALSIP_FPGA_PROG_INIT			4

// Misc command
#define ALSM_REBOOT				0
#define ALSM_SECRNG_QUERY			1
#define ALSM_SECRNG_ACC				2
#define ALSM_WDT_SOFT_REBOOT_EN			3

#define ANL_SOCID_DR1X90			0x100a9000

enum anl_soc_id {
	ANL_SOC_DR1X90,
	ANL_SOC_MAX
};

struct anl_top_data {
	const char *family;
	int chip;
};

static DEFINE_SPINLOCK(top_lock);
static DEFINE_MUTEX(top_mtx);
static struct soc_device *soc_dev;
static struct regmap *top_reg;
static ulong top_reg_base, top_len;
static void __iomem *top_sec_base;
static ulong top_sec_reg_base, top_sec_len;
static uint64_t anl_chipid;

static struct anl_top_data dr1x90_top_data = {
	.family = "Anlogic-DR1",
	.chip = ANL_SOC_DR1X90,
};

static const char *anl_soc_id(uint64_t chipid)
{
	const char *socid = "UNKNSOC";

	switch (chipid) {
	case ANL_SOCID_DR1X90:
		socid = "DR1X90";
		break;
	}
	return kasprintf(GFP_KERNEL, "%s", socid);
}

static inline __maybe_unused void top_writel(u32 value, u32 off)
{
	if (!top_reg || (off + 4) > top_len) {
		pr_warn("anl write top reg 0x%x fail\n", off);
		BUG_ON(1);
		return;
	}

	regmap_write(top_reg, off, value);
}

static inline __maybe_unused u32 top_readl(u32 off)
{
	u32 value = -1;

	if (!top_reg || (off + 4) > top_len) {
		pr_warn("anl read top reg 0x%x fail\n", off);
		BUG_ON(1);
		return 0;
	}

	regmap_read(top_reg, off, &value);
	return value;
}

static inline __maybe_unused void top_writeq(u64 value, u32 off)
{
	if (!top_reg || (off + 8) > top_len) {
		pr_warn("anl write top reg 0x%x fail\n", off);
		BUG_ON(1);
		return;
	}

	pr_warn("anl write top reg 0x%x with 64bit value, but not supported\n", off);
}

static inline __maybe_unused u64 top_readq(u32 off)
{
	if (!top_reg || (off + 8) > top_len) {
		pr_warn("anl read top reg 0x%x fail\n", off);
		BUG_ON(1);
		return 0;
	}

	pr_warn("anl read top reg 0x%x with 64bit value, but not supported\n", off);
	return -1;
}

static unsigned long invoke_fpga_prog_fn(unsigned long function_id,
			unsigned long arg0, unsigned long arg1,
			unsigned long arg2, unsigned long arg3)
{
#if IS_ENABLED(CONFIG_ARCH_RV64I)
	struct sbiret ret = {0};

	ret = sbi_ecall((int)function_id, (int)arg0, arg1, arg2, arg3, 0, 0, 0);
	if (!ret.error)
		return ret.value;
	else
		return ret.error;
#else
	struct arm_smccc_res res;

	arm_smccc_smc(function_id, arg0, arg1, arg2, arg3, 0, 0, 0, &res);
	return res.a0;
#endif
}

int anl_fpga_prog_init(phys_addr_t addr, uint32_t len)
{
	unsigned long ret;

	ret = invoke_fpga_prog_fn(ALSIP_FPGA_PROG, ALSIP_FPGA_PROG_INIT, addr, len, 0);
	return (int)ret;
}
EXPORT_SYMBOL(anl_fpga_prog_init);

int anl_fpga_prog_start(void)
{
	unsigned long ret;

	ret = invoke_fpga_prog_fn(ALSIP_FPGA_PROG, ALSIP_FPGA_PROG_START, 0, 0, 0);
	return (int)ret;
}
EXPORT_SYMBOL(anl_fpga_prog_start);

int anl_fpga_prog_load(phys_addr_t addr, uint32_t len)
{
	unsigned long ret;

	ret = invoke_fpga_prog_fn(ALSIP_FPGA_PROG, ALSIP_FPGA_PROG_LOAD, addr, len, 0);
	return (int)ret;
}
EXPORT_SYMBOL(anl_fpga_prog_load);

int anl_fpga_prog_done(u32 done)
{
	unsigned long ret;

	ret = invoke_fpga_prog_fn(ALSIP_FPGA_PROG, ALSIP_FPGA_PROG_DONE, done, 0, 0);
	return (int)ret;
}
EXPORT_SYMBOL(anl_fpga_prog_done);

int anl_fpga_plclk_reset(u32 reset)
{
	unsigned long ret;

	ret = invoke_fpga_prog_fn(ALSIP_FPGA_PROG, ALSIP_FPGA_PLCLK_RST, reset, 0, 0);
	return (int)ret;
}
EXPORT_SYMBOL(anl_fpga_plclk_reset);

u64 anl_rproc_start_slave(u64 bootaddr, int slaveid)
{
#if IS_ENABLED(CONFIG_ARCH_RV64I)
	return -1;
#else
	struct arm_smccc_res res;

	arm_smccc_smc(FN_RPROC_ON, bootaddr, slaveid, 0, 0, 0, 0, 0, &res);

	return res.a0;
#endif
}
EXPORT_SYMBOL(anl_rproc_start_slave);

u64 anl_rproc_stop_slave(int slaveid)
{
#if IS_ENABLED(CONFIG_ARCH_RV64I)
	return -1;
#else
	struct arm_smccc_res res;

	arm_smccc_smc(FN_RPROC_OFF, slaveid, 0, 0, 0, 0, 0, 0, &res);

	return res.a0;
#endif
}
EXPORT_SYMBOL(anl_rproc_stop_slave);

void anl_wdt_pause(int id, int pause)
{
	unsigned long flags;
	u32 val;

	spin_lock_irqsave(&top_lock, flags);
	val = top_readl(SYSCTRL_NS_CFG_CTRL_WDT);
	if (pause)
		top_writel(val | (1 << id), SYSCTRL_NS_CFG_CTRL_WDT);
	else
		top_writel(val & ~(1 << id), SYSCTRL_NS_CFG_CTRL_WDT);
	spin_unlock_irqrestore(&top_lock, flags);
}
EXPORT_SYMBOL(anl_wdt_pause);

void anl_spi_set_slave(int id, int slave)
{
	unsigned long flags;
	u32 val;

	spin_lock_irqsave(&top_lock, flags);
	val = top_readl(SYSCTRL_NS_CFG_CTRL_SPI);
	if (slave)
		top_writel(val | (1 << (id * 8)), SYSCTRL_NS_CFG_CTRL_SPI);
	else
		top_writel(val & ~(1 << (id * 8)), SYSCTRL_NS_CFG_CTRL_SPI);
	spin_unlock_irqrestore(&top_lock, flags);
}
EXPORT_SYMBOL(anl_spi_set_slave);

static int anl_sec_op(ulong op, ulong off, u64 *val)
{
	int ret = 0;
	int len = 4;

	if (op == 2 || op == 3)
		len = 8;

	if (off & (len - 1) || (off + len) > top_sec_len)
		return -1;

#if IS_ENABLED(CONFIG_ARCH_RV64I)
	if (!top_sec_base)
		return -1;

	if (op == 0)
		*val = readl(top_sec_base + off);
	else if (op == 1)
		writel((u32)(*val), top_sec_base + off);
	else if (op == 2)
		*val = readq(top_sec_base + off);
	else if (op == 3)
		writeq((u64)(*val), top_sec_base + off);
	else
		ret = -1;

	return ret;
#else
	{
		struct arm_smccc_res res;

		arm_smccc_smc(ALSIP_SYS_MISC, ALSM_SECRNG_ACC, op, top_sec_reg_base + off,
			      *val, 0, 0, 0, &res);
		if (!res.a0) {
			if (op == 0 || op == 2)
				*val = res.a1;
		} else
			ret = -1;
	}
	return ret;
#endif
}

int anl_sec_readl(ulong off, u32 *val)
{
	int ret;
	u64 v = 0;

	ret = anl_sec_op(0, off, &v);
	*val = (u32)v;

	return ret;
}
EXPORT_SYMBOL(anl_sec_readl);

int anl_sec_writel(ulong off, u32 val)
{
	int ret;
	u64 v = 0;

	v = val;
	ret = anl_sec_op(1, off, &v);

	return ret;
}
EXPORT_SYMBOL(anl_sec_writel);

int anl_sec_readq(ulong off, u64 *val)
{
	int ret;
	u64 v = 0;

	ret = anl_sec_op(2, off, &v);
	*val = (u64)v;

	return ret;
}
EXPORT_SYMBOL(anl_sec_readq);

int anl_sec_writeq(ulong off, u64 val)
{
	int ret;
	u64 v = 0;

	v = val;
	ret = anl_sec_op(3, off, &v);

	return ret;
}
EXPORT_SYMBOL(anl_sec_writeq);

#if IS_ENABLED(CONFIG_ANLOGIC_TOP_USER_SACCESS)
static int secreg_getop(size_t len, ulong off, int write)
{
	int op = -1;

	if (len == 4)
		op = 0;
	else if (len == 8)
		op = 2;
	else
		return -1;

	if (write)
		op += 1;

	if ((off + len) > (top_sec_reg_base + top_sec_len) ||
	    off < top_sec_reg_base)
		return -1;

	return op;
}

ssize_t secreg_write(struct file *file, const char __user *data,
		     size_t len, loff_t *ppos)
{
	int err = 0;
	int op;
	uint64_t val64;

	mutex_lock(&top_mtx);

	op = secreg_getop(len, *ppos, 1);
	if (op < 0) {
		err = op;
		goto out;
	}

	if (len == 4)
		err = get_user(val64, (const u32 __user *)data);
	else
		err = get_user(val64, (const u64 __user *)data);

	if (err < 0)
		goto out;

	err = anl_sec_op(op, *ppos - top_sec_reg_base, &val64);
out:
	mutex_unlock(&top_mtx);

	if (err < 0)
		return err;

	return len;
}
EXPORT_SYMBOL(secreg_write);

ssize_t secreg_read(struct file *file, char __user *data,
		    size_t len, loff_t *ppos)
{
	int err = 0;
	int op;
	uint64_t val64 = 0;

	mutex_lock(&top_mtx);

	op = secreg_getop(len, *ppos, 0);
	if (op < 0) {
		err = op;
		goto out;
	}

	err = anl_sec_op(op, *ppos - top_sec_reg_base, &val64);
	if (err < 0)
		goto out;

	if (len == 4)
		err = put_user(val64, (const u32 __user *)data);
	else
		err = put_user(val64, (const u64 __user *)data);

out:
	mutex_unlock(&top_mtx);

	if (err < 0)
		return err;

	return len;
}
EXPORT_SYMBOL(secreg_read);

long secreg_ioctl(struct file *file, unsigned int cmd,
		  unsigned long arg)
{
	int err = 0;
	struct anltop_ioc_secreg_query squery;

	if (_IOC_TYPE(cmd) != ANLTOP_MAGIC)
		return -ENOTTY;

	switch (cmd) {
	case ANLTOP_IOCSQUERY:
		mutex_lock(&top_mtx);
		squery.base = top_sec_reg_base;
		squery.len = top_sec_len;
		mutex_unlock(&top_mtx);
		if (copy_to_user((void __user *)arg, &squery, sizeof(squery)))
			return -EFAULT;
		break;
	default:
		err = -ENOTTY;
		break;
	}

	return err;
}
EXPORT_SYMBOL(secreg_ioctl);

loff_t secreg_llseek(struct file *file, loff_t off, int whence)
{
	loff_t newpos = 0;

	switch (whence) {
	case 0:
		newpos = off;
		break;
	case 1:
		newpos = file->f_pos + off;
		break;
	default:
		return -EINVAL;
	}

	mutex_lock(&top_mtx);
	if ((ulong)newpos < top_sec_reg_base ||
	    (ulong)newpos >= (top_sec_reg_base + top_sec_len))
		newpos = -EINVAL;
	else
		file->f_pos = newpos;
	mutex_unlock(&top_mtx);

	return newpos;
}
EXPORT_SYMBOL(secreg_llseek);

int secreg_open(struct inode *inode, struct file *file)
{
	return 0;
}
EXPORT_SYMBOL(secreg_open);

int secreg_release(struct inode *inode, struct file *file)
{
	return 0;
}
EXPORT_SYMBOL(secreg_release);
#endif

#if IS_ENABLED(CONFIG_ANLOGIC_SOFT_REBOOT)
void al_misc_reboot(unsigned long entry)
{
#if IS_ENABLED(CONFIG_ARCH_RV64I)
	// not implemented
	pr_err("al_reboot not implemented\n");
#else
	struct arm_smccc_res res;

	arm_smccc_smc(ALSIP_SYS_MISC, ALSM_REBOOT, entry, 0, 0, 0, 0, 0, &res);
	if (res.a0)
		pr_err("al_reboot fail\n");
#endif
}
EXPORT_SYMBOL(al_misc_reboot);

void al_misc_cpu_die(int id)
{
#if IS_ENABLED(CONFIG_ARCH_RV64I)
	// not implemented
	pr_err("al_cpu_die not implemented\n");
#else
	const struct cpu_operations *ops;

	ops = get_cpu_ops(id);
	if (ops && ops->cpu_die)
		ops->cpu_die(id);

	pr_err("al_cpu_die fail\n");
#endif
}
EXPORT_SYMBOL(al_misc_cpu_die);

int anl_wdt_softboot_enable(ulong wdtidx, ulong entry_point)
{
#if IS_ENABLED(CONFIG_ARCH_RV64I)
	return -1;
#else
	struct arm_smccc_res res;

	arm_smccc_smc(ALSIP_SYS_MISC, ALSM_WDT_SOFT_REBOOT_EN, wdtidx,
		      entry_point, 0, 0, 0, 0, &res);
	if (res.a0)
		return -1;
	return 0;
#endif
}
EXPORT_SYMBOL(anl_wdt_softboot_enable);
#endif

uint64_t anl_get_chipid(void)
{
	return anl_chipid;
}
EXPORT_SYMBOL(anl_get_chipid);

/* Phase adjustment bitmask (bit6-bit12) and offset in control register */
#define DR1X90_GBE0_CTRL_REG			0x140
#define DR1X90_GBE1_CTRL_REG			0x144
#define DR1X90_PHASE_ADJ_MASK			0x7F
#define DR1X90_PHASE_ADJ_OFFSET			6
void anl_gbe_set_phase(int index, unsigned int speed, u32 value)
{
	u32 reg_addr = index ? DR1X90_GBE1_CTRL_REG : DR1X90_GBE0_CTRL_REG;
	u32 reg_val = top_readl(reg_addr);
	/* Clear existing phase bits and set new value.
	 * Mask ensures phase value does not exceed 7 bits.
	 */
	reg_val &= ~(DR1X90_PHASE_ADJ_MASK << DR1X90_PHASE_ADJ_OFFSET);
	reg_val |= (value & DR1X90_PHASE_ADJ_MASK) << DR1X90_PHASE_ADJ_OFFSET;
	top_writel(reg_val, reg_addr);
}
EXPORT_SYMBOL(anl_gbe_set_phase);

static int anl_top_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct regmap *regmap;
	struct soc_device_attribute *soc_dev_attr;
	void __iomem __maybe_unused *reg;
	struct resource res;
	struct device_node *np;
	struct anl_top_data const *data;
	u32 value;
	int ret = 0;

	mutex_lock(&top_mtx);

	data = device_get_match_data(&pdev->dev);

	soc_dev_attr = devm_kzalloc(dev, sizeof(*soc_dev_attr), GFP_KERNEL);
	if (!soc_dev_attr) {
		ret = -ENOMEM;
		goto out;
	}

	ret = of_address_to_resource(pdev->dev.of_node, 0, &res);
	if (ret) {
		dev_err(dev, "top init fail\n");
		goto out;
	}
	top_reg_base = res.start;
	top_len = resource_size(&res);

	regmap = device_node_to_regmap(pdev->dev.of_node);
	if (IS_ERR(regmap)) {
		dev_err(dev, "top init fail2\n");
		ret = PTR_ERR(regmap);
		goto out;
	}
	top_reg = regmap;
	ret = regmap_read(regmap, SYSCTRL_NS_CFG_CHIPID, &value);
	if (ret) {
		dev_err(dev, "get chipid fail\n");
		goto out;
	}
	anl_chipid = value;

	ret = of_address_to_resource(pdev->dev.of_node, 1, &res);
	if (ret) {
		dev_err(dev, "top sec init fail\n");
		goto out;
	}
	top_sec_reg_base = res.start;
	top_sec_len = resource_size(&res);

#if IS_ENABLED(CONFIG_ARCH_RV64I)
	reg = devm_platform_ioremap_resource(pdev, 1);
	if (IS_ERR(reg)) {
		dev_err(dev, "top sec init fail");
		ret = PTR_ERR(reg);
		goto out;
	}
	top_sec_base = reg;
#endif

	np = of_find_node_by_path("/");
	of_property_read_string(np, "model", &soc_dev_attr->machine);
	of_node_put(np);
	soc_dev_attr->family = data->family;
	soc_dev_attr->revision = "0";
	soc_dev_attr->serial_number = "0";
	soc_dev_attr->soc_id = anl_soc_id(anl_chipid);

	soc_dev = soc_device_register(soc_dev_attr);
	if (IS_ERR(soc_dev)) {
		kfree(soc_dev_attr->soc_id);
		ret = PTR_ERR(soc_dev);
		dev_err(dev, "soc device register fail: %d\n", ret);
		goto out;
	}

	mutex_unlock(&top_mtx);
	return 0;

out:
	soc_dev = NULL;
	top_sec_base = NULL;
	top_sec_reg_base = 0;
	top_sec_len = 0;
	top_reg = NULL;
	top_reg_base = 0;
	top_len = 0;
	mutex_unlock(&top_mtx);
	return ret;
}

static const struct of_device_id anl_top_ids[] = {
	{ .compatible = "anlogic,dr1x90-topcfg", .data = &dr1x90_top_data },
	{ }
};
MODULE_DEVICE_TABLE(of, anl_top_ids);

static struct platform_driver anl_top_driver = {
	.probe	= anl_top_probe,
	.driver	= {
		.name	= "anl-topcfg",
		.of_match_table	= anl_top_ids,
	},
};

static int __init anl_top_driver_init(void)
{
	return platform_driver_register(&anl_top_driver);
}
postcore_initcall(anl_top_driver_init);

MODULE_AUTHOR("fushan.zeng<fushan.zeng@anlogic.com>");
MODULE_DESCRIPTION("anlogic top config");
MODULE_LICENSE("GPL");
