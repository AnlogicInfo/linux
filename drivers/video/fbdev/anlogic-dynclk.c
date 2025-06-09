// SPDX-License-Identifier: GPL-2.0
/*
 * Anlogic dynamic clock Driver
 *
 * Copyright (c) 2025 anlogic Technologies Co., Ltd.
 */

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/of.h>

#define CHECK_MAX (2000 * 1000)
#define CHECK_MIN (800  * 1000)
#define DIV_MAX 128

#define PLL_DIV			0x04
#define PLL_MUL			0x12
#define PLL_DUTY		0x24
#define PLL_OUT_DIV		0x27

struct dynclk_dev {
	struct platform_device *pdev;
	void __iomem *base;
	unsigned int refclk;
	unsigned int expclk;
};

static int dynclk_set(struct dynclk_dev *clkdev)
{
	unsigned int clk_m = 1, clk_n = 1;
	unsigned int clk_div = 0;
	unsigned int best_m = 0, best_n = 0, best_d = 0;
	unsigned int best_deep = 0;
	unsigned int best_clko = 0, best_check = 0;

	unsigned int check_best = 0;

	unsigned int refclk = (unsigned int)(clkdev->refclk / 1000);
	unsigned int expclk = (unsigned int)(clkdev->expclk / 1000);
	void __iomem *base = clkdev->base;

	while (check_best == 0) {
		if ((refclk * clk_m / clk_n > CHECK_MIN)
		&& (refclk * clk_m / clk_n < CHECK_MAX)) {
			for (clk_div = 2; clk_div <= DIV_MAX; clk_div++) {
				best_clko = refclk * clk_m / clk_n / clk_div;
				if (best_clko == expclk) {
					best_m = clk_m;
					best_n = clk_n;
					best_d = clk_div;
					best_deep = best_check;
					check_best = 1;
					break;
				}
				if (best_clko > expclk)
					best_check = best_clko - expclk;
				else
					best_check = expclk - best_clko;

				if (best_check < best_deep) {
					best_m    = clk_m;
					best_n    = clk_n;
					best_d    = clk_div;
					best_deep = best_check;
				}
			}
			if (clk_n == DIV_MAX) {
				if (clk_m == DIV_MAX)
					break;
				clk_n = 1;
				clk_m++;
			} else {
				clk_n++;
			}
		} else {
			if (clk_n == DIV_MAX) {
				if (clk_m == DIV_MAX)
					break;
				clk_n = 1;
				clk_m++;
			} else {
				clk_n++;
			}
		}
	}
	dev_info(&clkdev->pdev->dev, "vco, m, n, div, %d, %d, %d, %d\r\n",
		refclk * best_m / best_n / 1000, best_m, best_n, best_d);

	writel((PLL_DIV << 8) + ((best_n - 1) & 0xff), base);
	writel((PLL_MUL << 8) + ((best_m - 1) & 0xff), base);
	writel((PLL_DUTY << 8) + (((best_d + 1) / 2) & 0xff), base);
	writel((PLL_OUT_DIV << 8) + ((best_d - 1) & 0xff), base);

	return 0;
}

static int dynclk_probe(struct platform_device *pdev)
{
	struct dynclk_dev *clkdev;
	int ret;

	clkdev = kzalloc(sizeof(*clkdev), GFP_KERNEL);
	if (!clkdev)
		return -ENOMEM;

	clkdev->pdev = pdev;
	clkdev->base = devm_platform_ioremap_resource(pdev, 0);
	if (!clkdev->base) {
		dev_err(&pdev->dev, "Get base address error");
		goto out;
	}

	ret = of_property_read_u32(pdev->dev.of_node, "refclk", &clkdev->refclk);
	if (ret != 0) {
		dev_err(&pdev->dev, "Get dt refclk error");
		goto out;
	}

	ret = of_property_read_u32(pdev->dev.of_node, "expclk", &clkdev->expclk);
	if (ret != 0) {
		dev_err(&pdev->dev, "Get dt expclk error");
		goto out;
	}

	dynclk_set(clkdev);

out:
	kfree(clkdev);
	return 0;
}

static int dynclk_remove(struct platform_device *pdev)
{
	return 0;
}

static void dynclk_shutdown(struct platform_device *pdev)
{

}

static const struct of_device_id dynclk_of_match_table[] = {
	{ .compatible = "anlogic,dynclk", },
	{ /* end of table */ },
};

static struct platform_driver anlogic_dynclk_driver = {
	.probe = dynclk_probe,
	.remove = dynclk_remove,
	.shutdown = dynclk_shutdown,
	.driver = {
		.name = "anlogic-dynclk",
		.of_match_table = dynclk_of_match_table,
	},
};

module_platform_driver(anlogic_dynclk_driver);

MODULE_DESCRIPTION("Anlogic dynamic clk driver");
MODULE_AUTHOR("Guanghao.zhang <guanghao.zhang@anlogic.com>");
MODULE_LICENSE("GPL");
