// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2024 Anlogic, Inc.
 */

#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/device.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/reset-controller.h>

#include "anl_clk.h"
#include "anl_reset.h"
#include "anl_nmc.h"
#include "anl_div.h"
#include "anl_sel.h"
#include "anl_gate.h"

#include <dt-bindings/clock/anl_dr1x90_crp.h>
#include <dt-bindings/reset/anl_dr1x90_crp.h>

static struct clk_div_table anl_divtbl_6bits[64+1];
static struct clk_div_table anl_divtbl_7bits[128+1];
static struct clk_div_table anl_divtbl_12bits[4096+1];

static struct anl_div oscdiv2 = {
	.clk = {
		ANL_CLK_INIT("oscdiv2", "cryst_osc", anl_div_ops, 0, 0x030)
	},
	ANL_DIV_FIXED_INIT(1, 2, 0),
};

static struct anl_nm cpupll = {
	.clk = {
		ANL_CLK_INIT("cpupll", "cryst_osc", anl_nm_ops, 0, 0x120)
	}
};

static struct anl_nm iopll = {
	.clk = {
		ANL_CLK_INIT("iopll", "cryst_osc", anl_nm_ops, 0, 0x220)
	}
};

static struct anl_nm ddrgpll = {
	.clk = {
		ANL_CLK_INIT("ddrgpll", "oscdiv2", anl_nm_ops, 0, 0x120)
	}
};

static struct anl_nm ddrgpllphy0 = {
	.clk = {
		ANL_CLK_INIT("ddrgpllphy0", "ddrgpll", anl_nmc_ops, 0, 0x148)
	}
};

static struct anl_nm ddrgpllctl0 = {
	.clk = {
		ANL_CLK_INIT("ddrgpllctl0", "ddrgpll", anl_nmc_ops, 0, 0x14c)
	}
};

static struct anl_nm ddrgpllphy1 = {
	.clk = {
		ANL_CLK_INIT("ddrgpllphy1", "ddrgpll", anl_nmc_ops, 0, 0x150)
	}
};

static struct anl_nm ddrgpllctl1 = {
	.clk = {
		ANL_CLK_INIT("ddrgpllctl1", "ddrgpll", anl_nmc_ops, 0, 0x154)
	}
};

static const char * const ddroscctlsel_pnames[] = {
	"ddrgpllctl0", "oscdiv2"
};
static struct anl_sel ddroscctlsel = {
	.clk = {
		ANL_CLK_INIT_PARENTS("ddroscctlsel", ddroscctlsel_pnames, anl_sel_ro_ops, 0, 0x000)
	},
	ANL_SEL_INIT(1, 0, 0, NULL),
};

static const char * const ddrgpllctlsel_pnames[] = {
	"ddrgpllctl0", "ddrgpllctl1"
};
static struct anl_sel ddrgpllctlsel = {
	.clk = {
		ANL_CLK_INIT_PARENTS("ddrgpllctlsel", ddrgpllctlsel_pnames,
				     anl_sel_ro_ops, 0, 0x7F0)
	},
	ANL_SEL_INIT(1, 2, 0, NULL),
};

static const char * const ddrctl_pnames[] = {
	"ddrgpllctlsel", "ddroscctlsel"
};
static struct anl_sel ddrctl = {
	.clk = {
		ANL_CLK_INIT_PARENTS("ddrctl", ddrctl_pnames, anl_sel_ro_ops, 0, 0x7F0)
	},
	ANL_SEL_INIT(1, 3, 0, NULL),
};

static const char * const ddroscphysel_pnames[] = {
	"ddrgpllphy0", "oscdiv2"
};
static struct anl_sel ddroscphysel = {
	.clk = {
		ANL_CLK_INIT_PARENTS("ddroscphysel", ddroscphysel_pnames, anl_sel_ro_ops, 0, 0x000)
	},
	ANL_SEL_INIT(1, 0, 0, NULL),
};

static const char * const ddrgpllphysel_pnames[] = {
	"ddrgpllphy0", "ddrgpllphy1"
};
static struct anl_sel ddrgpllphysel = {
	.clk = {
		ANL_CLK_INIT_PARENTS("ddrgpllphysel", ddrgpllphysel_pnames,
				     anl_sel_ro_ops, 0, 0x7F0)
	},
	ANL_SEL_INIT(1, 2, 0, NULL),
};

static const char * const ddrphy_pnames[] = {
	"ddrgpllphysel", "ddroscphysel"
};
static struct anl_sel ddrphy = {
	.clk = {
		ANL_CLK_INIT_PARENTS("ddrphy", ddrphy_pnames, anl_sel_ro_ops, 0, 0x7F0)
	},
	ANL_SEL_INIT(1, 3, 0, NULL),
};

static struct anl_nm cpupll6x = {
	.clk = {
		ANL_CLK_INIT("cpupll6x", "cpupll", anl_nmc_ops, 0, 0x148)
	}
};

static struct anl_nm cpupll4x = {
	.clk = {
		ANL_CLK_INIT("cpupll4x", "cpupll", anl_nmc_ops, 0, 0x14c)
	}
};

static struct anl_nm io1000 = {
	.clk = {
		ANL_CLK_INIT("io1000", "iopll", anl_nmc_ops, 0, 0x248)
	}
};

static struct anl_nm io400 = {
	.clk = {
		ANL_CLK_INIT("io400", "iopll", anl_nmc_ops, 0, 0x24c)
	}
};

static struct anl_nm io25 = {
	.clk = {
		ANL_CLK_INIT("io25", "iopll", anl_nmc_ops, 0, 0x250)
	}
};

static struct anl_nm io80 = {
	.clk = {
		ANL_CLK_INIT("io80", "iopll", anl_nmc_ops, 0, 0x254)
	}
};

static struct anl_div cpu6xdiv1 = {
	.clk = {
		ANL_CLK_INIT("cpu6xdiv1", "cpupll6x", anl_div_ro_ops, 0, 0x000)
	},
	ANL_DIV_INIT(24, 0, CLK_DIVIDER_READ_ONLY),
};

static struct anl_div cpu6xdiv3 = {
	.clk = {
		ANL_CLK_INIT("cpu6xdiv3", "cpu6xdiv1", anl_div_ro_ops, 0, 0x004)
	},
	ANL_DIV_INIT(24, 0, CLK_DIVIDER_READ_ONLY),
};

static struct anl_div cpu6xdiv6 = {
	.clk = {
		ANL_CLK_INIT("cpu6xdiv6", "cpu6xdiv1", anl_div_ro_ops, 0, 0x004)
	},
	ANL_DIV_INIT(24, 0, CLK_DIVIDER_READ_ONLY),
};

static struct anl_div cpu4xdiv1 = {
	.clk = {
		ANL_CLK_INIT("cpu4xdiv1", "cpupll4x", anl_div_ro_ops, 0, 0x010)
	},
	ANL_DIV_INIT(24, 0, CLK_DIVIDER_READ_ONLY),
};

static struct anl_div cpu4xdiv2 = {
	.clk = {
		ANL_CLK_INIT("cpu4xdiv2", "cpu4xdiv1", anl_div_ro_ops, 0, 0x014)
	},
	ANL_DIV_INIT(24, 0, CLK_DIVIDER_READ_ONLY),
};

static struct anl_div cpu4xdiv4 = {
	.clk = {
		ANL_CLK_INIT("cpu4xdiv4", "cpu4xdiv1", anl_div_ro_ops, 0, 0x018)
	},
	ANL_DIV_INIT(24, 0, CLK_DIVIDER_READ_ONLY),
};

static struct anl_div io400div2 = {
	.clk = {
		ANL_CLK_INIT("io400div2", "io400", anl_div_ops, 0, 0x020)
	},
	ANL_DIV_INIT(32, 0, 0),
};

static struct anl_div io400div4 = {
	.clk = {
		ANL_CLK_INIT("io400div4", "io400", anl_div_ops, 0, 0x024)
	},
	ANL_DIV_INIT(32, 0, 0),
};

static struct anl_div io400div8 = {
	.clk = {
		ANL_CLK_INIT("io400div8", "io400", anl_div_ops, 0, 0x028)
	},
	ANL_DIV_INIT(32, 0, 0),
};

static struct anl_div io400div16 = {
	.clk = {
		ANL_CLK_INIT("io400div16", "io400", anl_div_ops, 0, 0x02c)
	},
	ANL_DIV_INIT(32, 0, 0),
};

static const char * const cpu6x4x_pnames[] = {
	"cpu4xdiv1", "cpu6xdiv1"
};
static struct anl_sel cpu6x4x = {
	.clk = {
		ANL_CLK_INIT_PARENTS("cpu6x4x", cpu6x4x_pnames, anl_sel_ro_ops, 0, 0x040)
	},
	ANL_SEL_INIT(1, 0, 0, NULL),
};

static const char * const cpu2x_pnames[] = {
	"cpu4xdiv2", "cpu6xdiv3"
};
static struct anl_sel cpu2x = {
	.clk = {
		ANL_CLK_INIT_PARENTS("cpu2x", cpu2x_pnames, anl_sel_ro_ops, 0, 0x040)
	},
	ANL_SEL_INIT(1, 0, 0, NULL),
};

static const char * const cpu1x_pnames[] = {
	"cpu4xdiv4", "cpu6xdiv6"
};
static struct anl_sel cpu1x = {
	.clk = {
		ANL_CLK_INIT_PARENTS("cpu1x", cpu1x_pnames, anl_sel_ro_ops, 0, 0x040)
	},
	ANL_SEL_INIT(1, 0, 0, NULL),
};

static const char * const rpu4x2x_pnames[] = {
	"cpu4xdiv1", "cpu4xdiv2"
};
static const char * const rpusub_pnames[] = {
	"rpu4x2x", "cpu6xdiv3"
};
static struct anl_sel rpu4x2x = {
	.clk = {
		ANL_CLK_INIT_PARENTS("rpu4x2x", rpu4x2x_pnames, anl_sel_ro_ops, 0, 0x040)
	},
	ANL_SEL_INIT(1, 5, 0, NULL),
};
static struct anl_sel rpusub = {
	.clk = {
		ANL_CLK_INIT_PARENTS("rpusub", rpusub_pnames, anl_sel_ro_ops, 0, 0x040)
	},
	ANL_SEL_INIT(1, 0, 0, NULL),
};

static struct anl_div sdio = {
	.clk = {
		ANL_CLK_INIT("sdio", "io400", anl_div_ops, 0, 0x030)
	},
	ANL_DIV_INIT_VAL(6, 24, 0),
};

static struct anl_div pl0 = {
	.clk = {
		ANL_CLK_INIT("pl0", "io400", anl_div_ops, 0, 0x03c)
	},
	ANL_DIV_ZERO_GATE_INIT(6, 0, 0),
};

static struct anl_div pl1 = {
	.clk = {
		ANL_CLK_INIT("pl1", "io400", anl_div_ops, 0, 0x03c)
	},
	ANL_DIV_ZERO_GATE_INIT(6, 8, 0),
};

static struct anl_div pl2 = {
	.clk = {
		ANL_CLK_INIT("pl2", "io400", anl_div_ops, 0, 0x03c)
	},
	ANL_DIV_ZERO_GATE_INIT(6, 16, 0),
};

static struct anl_div pl3 = {
	.clk = {
		ANL_CLK_INIT("pl3", "io400", anl_div_ops, 0, 0x03c)
	},
	ANL_DIV_ZERO_GATE_INIT(6, 24, 0),
};

static struct anl_div adcpl = {
	.clk = {
		ANL_CLK_INIT("adcpl", "io400", anl_div_ops, 0, 0x03c)
	},
	ANL_DIV_FIXED_INIT(1, 8, 0),
};

static struct anl_div smc = {
	.clk = {
		ANL_CLK_INIT("smc", "io1000", anl_div_en_ops, 0, 0x030)
	},
	ANL_DIV_ZERO_GATE_INIT(6, 16, 0),
};

static struct anl_div spi = {
	.clk = {
		ANL_CLK_INIT("spi", "io1000", anl_div_en_ops, 0, 0x030)
	},
	ANL_DIV_ZERO_GATE_INIT(6, 8, 0),
};

static struct anl_div qspi = {
	.clk = {
		ANL_CLK_INIT("qspi", "io1000", anl_div_en_ops, 0, 0x030)
	},
	ANL_DIV_ZERO_GATE_INIT(6, 0, 0),
};

static struct anl_div gbe = {
	.clk = {
		ANL_CLK_INIT("gbe", "io1000", anl_div_ops, 0, 0x030)
	},
	ANL_DIV_FIXED_INIT(1, 2, 0),
};

static struct anl_div jpu = {
	.clk = {
		ANL_CLK_INIT("jpu", "io1000", anl_div_ops, 0, 0x030)
	},
	ANL_DIV_FIXED_INIT(1, 3, 0),
};

static const char * const wdt_pnames[] = {
	"oscdiv2", "wdtmio"
};
static struct anl_sel wdt = {
	.clk = {
		ANL_CLK_INIT_PARENTS("wdt", wdt_pnames, anl_sel_ops, 0, 0x040)
	},
	ANL_SEL_INIT(1, 1, 0, NULL),
};

static struct anl_div trng = {
	.clk = {
		ANL_CLK_INIT("trng", "io25", anl_div_en_ops, 0, 0x034)
	},
	ANL_DIV_ZERO_GATE_INIT(7, 24, 0),
};

static struct anl_div tvs = {
	.clk = {
		ANL_CLK_INIT("tvs", "io25", anl_div_en_ops, 0, 0x034)
	},
	ANL_DIV_ZERO_GATE_INIT(6, 16, 0),
};

static struct anl_div gpiodb = {
	.clk = {
		ANL_CLK_INIT("gpiodb", "io25", anl_div_en_ops, 0, 0x034)
	},
	ANL_DIV_ZERO_GATE_INIT(6, 0, 0),
};

static const char * const can_pnames[] = {
	"io80", "canmio"
};
static struct anl_sel can = {
	.clk = {
		ANL_CLK_INIT_PARENTS("can", can_pnames, anl_sel_ops, 0, 0x040)
	},
	ANL_SEL_INIT(1, 3, 0, NULL),
};

static struct anl_gate can0cgen = {
	.clk = {
		ANL_CLK_INIT("can0cgen", "can", anl_gate_ops, CLK_SET_RATE_PARENT, 0x08c)
	},
	ANL_GATE_INIT(20, CLK_GATE_SET_TO_DISABLE),
};

static struct anl_gate can1cgen = {
	.clk = {
		ANL_CLK_INIT("can1cgen", "can", anl_gate_ops, CLK_SET_RATE_PARENT, 0x08c)
	},
	ANL_GATE_INIT(21, CLK_GATE_SET_TO_DISABLE),
};

static struct anl_div pwm = {
	.clk = {
		ANL_CLK_INIT("pwm", "cryst_osc", anl_div_ops, 0, 0x038)
	},
	ANL_DIV_ZERO_GATE_INIT(12, 8, 0),
};

static struct anl_div divosc = {
	.clk = {
		ANL_CLK_INIT("divosc", "cryst_osc", anl_div_ops, 0, 0x038)
	},
	ANL_DIV_ZERO_GATE_INIT(6, 0, 0),
};

static struct anl_clk *anl_crp_clks[] = {
	&cpupll.clk,
	&iopll.clk,
	&cpupll6x.clk,
	&cpupll4x.clk,
	&io1000.clk,
	&io400.clk,
	&io25.clk,
	&io80.clk,
	&cpu6xdiv1.clk,
	&cpu4xdiv1.clk,
	&cpu6xdiv3.clk,
	&cpu4xdiv2.clk,
	&cpu6xdiv6.clk,
	&cpu4xdiv4.clk,
	&io400div2.clk,
	&io400div4.clk,
	&io400div8.clk,
	&io400div16.clk,
	&cpu6x4x.clk,
	&cpu2x.clk,
	&cpu1x.clk,
	&rpu4x2x.clk,
	&rpusub.clk,
	&sdio.clk,
	&pl0.clk,
	&pl1.clk,
	&pl2.clk,
	&pl3.clk,
	&adcpl.clk,
	&smc.clk,
	&spi.clk,
	&qspi.clk,
	&gbe.clk,
	&jpu.clk,
	&oscdiv2.clk,
	&wdt.clk,
	&trng.clk,
	&tvs.clk,
	&gpiodb.clk,
	&can.clk,
	&can0cgen.clk,
	&can1cgen.clk,
	&pwm.clk,
	&divosc.clk
};

static struct clk_hw_onecell_data anl_hw_clks = {
	.hws	= {
		[CLK_CPU1x] = &cpu1x.clk.hw,
		[CLK_CPU2x] = &cpu2x.clk.hw,
		[CLK_CPU6x4x] = &cpu6x4x.clk.hw,
		[CLK_UART] = &io400div8.clk.hw,
		[CLK_SMC] = &smc.clk.hw,
		[CLK_I2C] = &io400div4.clk.hw,
		[CLK_QSPI] = &qspi.clk.hw,
		[CLK_SPI] = &spi.clk.hw,
		[CLK_MMC_BUS] = &sdio.clk.hw,
		[CLK_CAN0] = &can0cgen.clk.hw,
		[CLK_CAN1] = &can1cgen.clk.hw,
		[CLK_WDT] = &wdt.clk.hw,
		[CLK_GPIO_DB] = &gpiodb.clk.hw,
		[CLK_ADCPL] = &adcpl.clk.hw,
		[CLK_GBE] = &gbe.clk.hw,
		[CLK_JPU] = &jpu.clk.hw,
	},
	.num	= CLK_NUMBER,
};

static struct anl_rstbl anl_crp_rstbl[] = {
	[RST_JPU] = {ANL_RSTBL_INIT(0x74, 13, 0)},
	[RST_NPU] = {ANL_RSTBL_INIT(0x74, 12, 0)},
	[RST_DMAC_AHB] = {ANL_RSTBL_INIT(0x74, 9, 0)},
	[RST_DMAC_AXI] = {ANL_RSTBL_INIT(0x74, 8, 0)},
	[RST_WDT] = {ANL_RSTBL_INIT(0x74, 7, 0)},
	[RST_SMC] = {ANL_RSTBL_INIT(0x74, 6, 0)},
	[RST_QSPI] = {ANL_RSTBL_INIT(0x74, 5, 0)},
	[RST_OCM] = {ANL_RSTBL_INIT(0x74, 4, 0)},
	[RST_IPC] = {ANL_RSTBL_INIT(0x78, 12, 0)},
	[RST_GPIO] = {ANL_RSTBL_INIT(0x78, 8, 0)},
	[RST_I2C1] = {ANL_RSTBL_INIT(0x7c, 29, 0)},
	[RST_I2C0] = {ANL_RSTBL_INIT(0x7c, 28, 0)},
	[RST_TTC1] = {ANL_RSTBL_INIT(0x7c, 25, 0)},
	[RST_TTC0] = {ANL_RSTBL_INIT(0x7c, 24, 0)},
	[RST_CAN1] = {ANL_RSTBL_INIT(0x7c, 21, 0)},
	[RST_CAN0] = {ANL_RSTBL_INIT(0x7c, 20, 0)},
	[RST_SPI1] = {ANL_RSTBL_INIT(0x7c, 17, 0)},
	[RST_SPI0] = {ANL_RSTBL_INIT(0x7c, 16, 0)},
	[RST_UART1] = {ANL_RSTBL_INIT(0x7c, 13, 0)},
	[RST_UART0] = {ANL_RSTBL_INIT(0x7c, 12, 0)},
	[RST_SDIO1] = {ANL_RSTBL_INIT(0x7c, 9, 0)},
	[RST_SDIO0] = {ANL_RSTBL_INIT(0x7c, 8, 0)},
	[RST_GBE1] = {ANL_RSTBL_INIT(0x7c, 5, 0)},
	[RST_GBE0] = {ANL_RSTBL_INIT(0x7c, 4, 0)},
	[RST_USB1] = {ANL_RSTBL_INIT(0x7c, 1, 0)},
	[RST_USB0] = {ANL_RSTBL_INIT(0x7c, 0, 0)}
};

static struct anl_crp_desc anl_desc = {
	.crp_clks = anl_crp_clks,
	.num_crp_clks = ARRAY_SIZE(anl_crp_clks),
	.hw_clks = &anl_hw_clks,
	.arstbl = anl_crp_rstbl,
	.num_resets = ARRAY_SIZE(anl_crp_rstbl),
};

static struct anl_clk *anl_ddr_clks[] = {
	&ddrgpll.clk,
	&ddrgpllctl0.clk,
	&ddrgpllphy0.clk,
	&ddrgpllctl1.clk,
	&ddrgpllphy1.clk,
	&ddroscctlsel.clk,
	&ddrgpllctlsel.clk,
	&ddrctl.clk,
	&ddroscphysel.clk,
	&ddrgpllphysel.clk,
	&ddrphy.clk
};

static struct anl_crp_desc anl_ddr_desc = {
	.crp_clks = anl_ddr_clks,
	.num_crp_clks = ARRAY_SIZE(anl_ddr_clks),
	.hw_clks = NULL,
	.arstbl = NULL,
	.num_resets = 0,
};

static int anl_dr1x90_crp_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	void __iomem *reg;
	int ret;

	anl_init_divtbl(anl_divtbl_6bits, 64);
	anl_init_divtbl(anl_divtbl_7bits, 128);
	anl_init_divtbl(anl_divtbl_12bits, 4096);

	reg = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(reg))
		return PTR_ERR(reg);

	ret = anl_crp_probe(dev, reg, &anl_desc);

	reg = devm_platform_ioremap_resource(pdev, 1);
	if (!IS_ERR(reg))
		anl_crp_probe(dev, reg, &anl_ddr_desc);

	return ret;
}

static const struct of_device_id anl_dr1x90_crp_ids[] = {
	{ .compatible = "anlogic,dr1x90-crp" },
	{ }
};
MODULE_DEVICE_TABLE(of, anl_dr1x90_crp_ids);

static struct platform_driver anl_dr1x90_crp_driver = {
	.probe	= anl_dr1x90_crp_probe,
	.driver	= {
		.name	= "anl-dr1x90-crp",
		.suppress_bind_attrs = true,
		.of_match_table	= anl_dr1x90_crp_ids,
	},
};

static int __init anl_dr1x90_crp_driver_init(void)
{
	return platform_driver_register(&anl_dr1x90_crp_driver);
}
subsys_initcall(anl_dr1x90_crp_driver_init);

MODULE_AUTHOR("fushan.zeng<fushan.zeng@anlogic.com>");
MODULE_DESCRIPTION("Anlogic dr1x90 crp");
MODULE_LICENSE("GPL");
