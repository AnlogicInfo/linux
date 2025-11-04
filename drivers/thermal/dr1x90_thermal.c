// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2025 Anlogic, Inc.
 */

#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of_address.h>
#include <linux/string.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/thermal.h>
#include <linux/delay.h>
#include "thermal_hwmon.h"

#define PVT_XADC_ENABLE		BIT(16)
#define PVT_RDATA_MASK		0x3FF

struct dr1x90_thermal_priv {
	struct device *dev;
	void __iomem *pvt_reg;
	struct clk *clk;
	unsigned long rate;
	struct thermal_zone_device *tzd;
};

static int tvs_tbl[][2] = {
	{0, 149},
	{6, 148},
	{16, 147},
	{25, 146},
	{33, 145},
	{41, 144},
	{49, 143},
	{57, 142},
	{65, 141},
	{72, 140},
	{79, 139},
	{86, 138},
	{93, 137},
	{99, 136},
	{106, 135},
	{112, 134},
	{119, 133},
	{125, 132},
	{131, 131},
	{137, 130},
	{143, 129},
	{149, 128},
	{154, 127},
	{160, 126},
	{166, 125},
	{171, 124},
	{177, 123},
	{182, 122},
	{188, 121},
	{193, 120},
	{198, 119},
	{203, 118},
	{209, 117},
	{214, 116},
	{219, 115},
	{224, 114},
	{229, 113},
	{234, 112},
	{239, 111},
	{244, 110},
	{249, 109},
	{253, 108},
	{258, 107},
	{263, 106},
	{268, 105},
	{272, 104},
	{277, 103},
	{282, 102},
	{286, 101},
	{291, 100},
	{296, 99},
	{300, 98},
	{305, 97},
	{309, 96},
	{314, 95},
	{318, 94},
	{322, 93},
	{327, 92},
	{331, 91},
	{336, 90},
	{340, 89},
	{344, 88},
	{349, 87},
	{353, 86},
	{357, 85},
	{361, 84},
	{365, 83},
	{370, 82},
	{374, 81},
	{378, 80},
	{382, 79},
	{386, 78},
	{390, 77},
	{395, 76},
	{399, 75},
	{403, 74},
	{407, 73},
	{411, 72},
	{415, 71},
	{419, 70},
	{423, 69},
	{427, 68},
	{431, 67},
	{435, 66},
	{438, 65},
	{442, 64},
	{446, 63},
	{450, 62},
	{454, 61},
	{458, 60},
	{462, 59},
	{465, 58},
	{469, 57},
	{473, 56},
	{477, 55},
	{480, 54},
	{484, 53},
	{488, 52},
	{492, 51},
	{495, 50},
	{499, 49},
	{503, 48},
	{506, 47},
	{510, 46},
	{513, 45},
	{517, 44},
	{521, 43},
	{524, 42},
	{528, 41},
	{531, 40},
	{535, 39},
	{538, 38},
	{542, 37},
	{545, 36},
	{549, 35},
	{552, 34},
	{556, 33},
	{559, 32},
	{563, 31},
	{566, 30},
	{569, 29},
	{573, 28},
	{576, 27},
	{580, 26},
	{583, 25},
	{586, 24},
	{590, 23},
	{593, 22},
	{596, 21},
	{599, 20},
	{603, 19},
	{606, 18},
	{609, 17},
	{612, 16},
	{616, 15},
	{619, 14},
	{622, 13},
	{625, 12},
	{628, 11},
	{634, 10},
	{635, 9},
	{638, 8},
	{641, 7},
	{644, 6},
	{647, 5},
	{650, 4},
	{653, 3},
	{656, 2},
	{659, 1},
	{662, 0},
	{665, -1},
	{668, -2},
	{671, -3},
	{674, -4},
	{677, -5},
	{680, -6},
	{683, -7},
	{686, -8},
	{689, -9},
	{692, -10},
	{695, -11},
	{698, -12},
	{700, -13},
	{703, -14},
	{706, -15},
	{709, -16},
	{712, -17},
	{715, -18},
	{717, -19},
	{720, -20},
	{723, -21},
	{726, -22},
	{728, -23},
	{731, -24},
	{734, -25},
	{737, -26},
	{739, -27},
	{742, -28},
	{745, -29},
	{747, -30},
	{750, -31},
	{753, -32},
	{755, -33},
	{758, -34},
	{761, -35},
	{763, -36},
	{766, -37},
	{768, -38},
	{771, -39},
	{773, -40},
	{776, -41},
	{778, -42},
	{781, -43},
	{784, -44},
	{786, -45},
	{789, -46},
	{791, -47},
	{793, -48},
	{796, -49},
	{798, -50}
};

static int dr1x90_thermal_code_to_degree(struct dr1x90_thermal_priv *priv,
					 unsigned int code)
{
	int i;

	for (i = 1; i < ARRAY_SIZE(tvs_tbl); i++) {
		if (code < tvs_tbl[i][0])
			break;
	}

	return tvs_tbl[i - 1][1] * 1000;
}

static int dr1x90_thermal_get_temp(void *data, int *temp)
{
	struct dr1x90_thermal_priv *priv = data;
	unsigned int code;

	if (!priv)
		return -EINVAL;

	code = readl(priv->pvt_reg);
	writel(code & ~PVT_XADC_ENABLE, priv->pvt_reg);
	writel(code | PVT_XADC_ENABLE, priv->pvt_reg);
	// wait 10000 clocks
	mdelay(10000000/priv->rate + 1);
	code = readl(priv->pvt_reg);
	*temp = dr1x90_thermal_code_to_degree(priv, code & PVT_RDATA_MASK);

	return 0;
}

static const struct thermal_zone_of_device_ops dr1x90_thermal_ops = {
	.get_temp = dr1x90_thermal_get_temp,
};

static void dr1x90_tvsclk_disable_unprepare(void *data)
{
	clk_disable_unprepare(data);
}

static int dr1x90_thermal_probe(struct platform_device *pdev)
{
	int ret = 0;
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	struct dr1x90_thermal_priv *priv;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (IS_ERR_OR_NULL(priv))
		return -ENOMEM;

	priv->dev = dev;

	priv->pvt_reg = devm_of_iomap(dev, np, 0, NULL);
	if (IS_ERR_OR_NULL(priv->pvt_reg))
		return dev_err_probe(dev, PTR_ERR(priv->pvt_reg), "mapping pvt reg failed\n");

	priv->clk = devm_clk_get_optional(dev, "tvsclk");
	if (IS_ERR(priv->clk))
		return dev_err_probe(dev, PTR_ERR(priv->clk), "could not get tvs clk\n");

	ret = clk_prepare_enable(priv->clk);
	if (ret)
		return dev_err_probe(dev, ret, "could not enable tvs clk\n");

	ret = devm_add_action_or_reset(dev, dr1x90_tvsclk_disable_unprepare, priv->clk);
	if (ret)
		return ret;

	priv->rate = clk_get_rate(priv->clk);

	platform_set_drvdata(pdev, priv);

	priv->tzd = devm_thermal_zone_of_sensor_register(&pdev->dev, 0, priv,
							 &dr1x90_thermal_ops);
	if (IS_ERR(priv->tzd))
		return dev_err_probe(dev, PTR_ERR(priv->tzd), "register thermal failed\n");

	if (devm_thermal_add_hwmon_sysfs(priv->tzd))
		dev_warn(&pdev->dev, "Failed to add hwmon sysfs\n");

	return 0;
}

static int dr1x90_thermal_remove(struct platform_device *pdev)
{
	return 0;
}

static const struct of_device_id dr1x90_thermal_match[] = {
	{ .compatible = "anlogic,dr1x90_thermal", },
	{ /* end of list */ },
};
MODULE_DEVICE_TABLE(of, dr1x90_thermal_match);

static struct platform_driver dr1x90_thermal_driver = {
	.probe = dr1x90_thermal_probe,
	.remove = dr1x90_thermal_remove,
	.driver = {
		.name = "dr1x90_thermal",
		.of_match_table = dr1x90_thermal_match,
	},
};
module_platform_driver(dr1x90_thermal_driver);

MODULE_AUTHOR("fushan.zeng <fushan.zeng@anlogic.com>");
MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("Anlogic Dr1x90 FPSoC thermal driver");
