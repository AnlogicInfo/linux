// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2023 Anlogic, Inc.
 */

#include <linux/dma-mapping.h>
#include <linux/fpga/fpga-mgr.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of_address.h>
#include <linux/string.h>
#include <linux/slab.h>
#include <linux/arm-smccc.h>

#define CRP_CFG_STATE				(0x00000454U)
#define CRP_CFG_STATE_MSK_PL2PS_INITN		(1 << 0)
#define CRP_CFG_STATE_MSK_PL2PS_CFG_DONE	(1 << 1)
#define CRP_CFG_STATE_MSK_PL2PS_CFG_WAKEUP	(1 << 2)

#define DR1M90_FLAG_BIG_ENDIAN			0x01

#define ALSIP_FPGA_PROG				(0xFF000000+23)
#define ALSIP_FPGA_PROG_START			0
#define ALSIP_FPGA_PROG_LOAD			1
#define ALSIP_FPGA_PROG_DONE			2

struct dr1m90_fpga_priv {
	struct device *dev;
	void __iomem *crp_cfg;
	u32 flags;
	u32 info_flags;
	struct page *work_pages;
	u32 *work_buf;
	int orders;
};

static int dr1m90_fpga_be32(struct fpga_manager *mgr,
				 const char *buf, size_t size,
				 uint32_t *out)
{
	struct dr1m90_fpga_priv *priv = mgr->priv;
	int i, j;

	if (!IS_ALIGNED(size, 4)) {
		dev_err(priv->dev, "endian convert: buf size is not dword aligned\n");
		return -1;
	}

	for (i = 0, j = 0; (i+4) <= size; i+=4, j++) {
		out[j] = be32_to_cpu(*(const uint32_t*)&buf[i]);
	}

	return 0;
}

static int dr1m90_fpga_le32(struct fpga_manager *mgr,
				 const char *buf, size_t size,
				 uint32_t *out)
{
	struct dr1m90_fpga_priv *priv = mgr->priv;
	int i, j;

	if (!IS_ALIGNED(size, 4)) {
		dev_err(priv->dev, "endian convert: buf size is not dword aligned\n");
		return -1;
	}

	for (i = 0, j = 0; (i+4) <= size; i+=4, j++) {
		out[j] = le32_to_cpu(*(const uint32_t*)&buf[i]);
	}

	return 0;
}

static unsigned long invoke_fpga_prog_fn(unsigned long function_id,
			unsigned long arg0, unsigned long arg1,
			unsigned long arg2, unsigned long arg3)
{
	struct arm_smccc_res res;

	arm_smccc_smc(function_id, arg0, arg1, arg2, arg3, 0, 0, 0, &res);
	return res.a0;
}

static int dr1m90_fpga_prog_start(struct fpga_manager *mgr)
{
	unsigned long ret;
	ret = invoke_fpga_prog_fn(ALSIP_FPGA_PROG, ALSIP_FPGA_PROG_START, 0, 0, 0);
	return (int)ret;
}

static int dr1m90_fpga_prog_load(struct fpga_manager *mgr, phys_addr_t addr,
			uint32_t len)
{
	unsigned long ret;
	ret = invoke_fpga_prog_fn(ALSIP_FPGA_PROG, ALSIP_FPGA_PROG_LOAD, addr, len, 0);
	return (int)ret;
}

static int dr1m90_fpga_prog_done(struct fpga_manager *mgr, u32 done)
{
	unsigned long ret;
	ret = invoke_fpga_prog_fn(ALSIP_FPGA_PROG, ALSIP_FPGA_PROG_DONE, done, 0, 0);
	return (int)ret;
}

static int dr1m90_fpga_ops_write_init(struct fpga_manager *mgr,
				      struct fpga_image_info *info,
				      const char *buf, size_t size)
{
	struct dr1m90_fpga_priv *priv = mgr->priv;

	priv->info_flags = info->flags;

	if (dr1m90_fpga_prog_start(mgr)) {
		dev_err(priv->dev, "fpga prog start fail\n");
		return -EIO;
	}

	return 0;
}

static const char *dr1m90_bf_hd = "# Anlogic ASCII Bitstream";
static int dr1m90_fpga_ops_write(struct fpga_manager *mgr,
				 const char *buf, size_t size)
{
	struct dr1m90_fpga_priv *priv = mgr->priv;
	const char *p = buf;
	size_t sz = size, len, work_len, i;
	int ret = 0;

	//skip header
	if (!memcmp(buf, dr1m90_bf_hd, strlen(dr1m90_bf_hd))) {
		for (i = 1; i < size; i++) {
			if (buf[i-1] == 0xa && buf[i] == 0xa) {
				i++;
				p = buf + i;
				sz -= i;
				dev_info(priv->dev, "skip %ld bytes headers\n", (long)i);
				break;
			}
		}
		if (i >= size)
			dev_warn(priv->dev, "End of header not found\n");
	}

	if (!IS_ALIGNED(sz, 4)) {
		dev_err(priv->dev, "bit data is not dword aligned: %ld\n", (long)sz);
		ret = -EINVAL;
		goto prog_fail;
	}

	work_len = (PAGE_SIZE << priv->orders);
	while (sz > 0) {
		len = (sz > work_len) ? work_len : sz;
		if (priv->flags & DR1M90_FLAG_BIG_ENDIAN)
			dr1m90_fpga_be32(mgr, p, len, priv->work_buf);
		else
			dr1m90_fpga_le32(mgr, p, len, priv->work_buf);

		if (dr1m90_fpga_prog_load(mgr, page_to_phys(priv->work_pages), len)) {
			dev_err(priv->dev, "fpga prog load fail\n");
			ret = -EIO;
			goto prog_fail;
		}

		p += len;
		sz -= len;
	}

	return 0;
prog_fail:
	dr1m90_fpga_prog_done(mgr, 0);
	return ret;
}

static int dr1m90_fpga_ops_write_complete(struct fpga_manager *mgr,
					  struct fpga_image_info *info)
{
	struct dr1m90_fpga_priv *priv = mgr->priv;

	if (dr1m90_fpga_prog_done(mgr, 1)) {
		dev_err(priv->dev, "fpga prog done fail\n");
		return -EIO;
	}

	return 0;
}

static enum fpga_mgr_states dr1m90_fpga_ops_state(struct fpga_manager *mgr)
{
	struct dr1m90_fpga_priv *priv;
	u32 val;

	priv = mgr->priv;
	val = readl(priv->crp_cfg + CRP_CFG_STATE) & 0x7;
	if (val == 7)
		return FPGA_MGR_STATE_OPERATING;

	return FPGA_MGR_STATE_UNKNOWN;
}

static const struct fpga_manager_ops dr1m90_fpga_ops = {
	.state = dr1m90_fpga_ops_state,
	.write_init = dr1m90_fpga_ops_write_init,
	.write = dr1m90_fpga_ops_write,
	.write_complete = dr1m90_fpga_ops_write_complete,
};

static int dr1m90_fpga_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	struct dr1m90_fpga_priv *priv;
	struct fpga_manager *mgr;
	struct page **pages;
	int count, i;
	int ret;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv) {
		dev_err(dev, "alloc memory fail\n");
		return -ENOMEM;
	}

	priv->flags = DR1M90_FLAG_BIG_ENDIAN;
	priv->dev = dev;

	priv->crp_cfg = devm_of_iomap(dev, np, 0, NULL);
	if (IS_ERR_OR_NULL(priv->crp_cfg)) {
		ret = PTR_ERR(priv->crp_cfg);
		dev_err(dev, "mapping crp cfg failed with %d\n", ret);
		return ret;
	}

	priv->orders = get_order(8*PAGE_SIZE);
	priv->work_pages = alloc_pages(GFP_KERNEL, priv->orders);
	if (!priv->work_pages) {
		dev_err(dev, "alloc work buffer fail\n");
		return -ENOMEM;
	}

	count = (1 << priv->orders);
	pages = kmalloc_array(count, sizeof(struct page *), GFP_KERNEL);
	if (!pages) {
		ret = -ENOMEM;
		goto free_workpages;
	}

	for (i = 0; i < count; i++)
		pages[i] = nth_page(priv->work_pages, i);

	priv->work_buf = vmap(pages, count, 0, pgprot_noncached(PAGE_KERNEL));
	kfree(pages);
	if (!priv->work_buf) {
		ret = -ENOMEM;
		goto free_workpages;
	}

	mgr = devm_fpga_mgr_register(dev, "Anlogic dr1m90 FPGA Manager",
				   &dr1m90_fpga_ops, priv);
	if (IS_ERR(mgr)) {
		ret = PTR_ERR(mgr);
		dev_err(dev, "register fpga manager fail: %d\n", ret);
		goto unmap_pages;
	}

	platform_set_drvdata(pdev, mgr);
	return 0;
unmap_pages:
	vunmap(priv->work_buf);
free_workpages:
	__free_pages(priv->work_pages, priv->orders);
	return ret;
}

static int dr1m90_fpga_remove(struct platform_device *pdev)
{
	struct fpga_manager *mgr = platform_get_drvdata(pdev);
	struct dr1m90_fpga_priv *priv = mgr->priv;

	vunmap(priv->work_buf);
	__free_pages(priv->work_pages, priv->orders);

	return 0;
}

static const struct of_device_id dr1m90_fpga_of_match[] = {
	{ .compatible = "anlogic,dr1m90-pcap-fpga", },
	{},
};

MODULE_DEVICE_TABLE(of, dr1m90_fpga_of_match);

static struct platform_driver dr1m90_fpga_driver = {
	.probe = dr1m90_fpga_probe,
	.remove = dr1m90_fpga_remove,
	.driver = {
		.name = "dr1m90_fpga_manager",
		.of_match_table = of_match_ptr(dr1m90_fpga_of_match),
	},
};

module_platform_driver(dr1m90_fpga_driver);

MODULE_AUTHOR("fushan.zeng<fushan.zeng@anlogic.com>");
MODULE_DESCRIPTION("Anlogic dr1m90 FPGA Manager");
MODULE_LICENSE("GPL");
