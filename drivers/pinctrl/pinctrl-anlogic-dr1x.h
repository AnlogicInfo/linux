/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Anlogic dr1x pin controller
 *
 * Copyright (C) 2024 Anlogic, Inc.
 */

#ifndef _PINCTRL_ANLOGIC_DR1X_H
#define _PINCTRL_ANLOGIC_DR1X_H

/* pinmux */
#define ANLOGIC_NUM_MIOS             54
#define ANLOGIC_PIN_MUX_OFFSET       0x00
#define ANLOGIC_PIN_EMIO_SEL_OFFSET  0x400
#define ANLOGIC_PINMUX_MUX_MASK      GENMASK(3, 0)

/* pinconfig */
#define ANLOGIC_PINCONF_SPEED		GENMASK(2, 0)
#define ANLOGIC_PINCONF_PULLUP		BIT(31)
#define ANLOGIC_PINCONF_PULLDOWN	BIT(30)
#define ANLOGIC_PINCONF_PULLUP_OHM	BIT(29)
#define ANLOGIC_PINCONF_PULLDOWN_OHM	BIT(28)
#define ANLOGIC_PIN_CFG_LOW_OFFSET	0x800
#define ANLOGIC_PIN_CFG_HIGH_OFFSET	0x804

#define PULL_RESISTOR_10KOHM	10000
#define PULL_RESISTOR_25KOHM	25000

/*
 * PIN_CONFIG_PULL_OHM: if the pin can select an IO pull up/down strength, the argument to
 *	this parameter (on a custom format) tells the driver which alternative bias Ohm to use.
 */
#define PIN_CONFIG_PULL_OHM	(PIN_CONFIG_END + 1)

#define DEFINE_ANLOGIC_PINCTRL_GRP(nm, emio_sel, is_support) \
	{ \
		.name = #nm "_grp", \
		.pins = nm ## _pins, \
		.npins = ARRAY_SIZE(nm ## _pins), \
		.emio_sel_offset = emio_sel, \
		.emio_supported = is_support, \
	}

#define DEFINE_ANLOGIC_PINMUX_FUNCTION(fname, mval)	\
	[ANLOGIC_PMUX_##fname] = {			\
		.name = #fname,				\
		.groups = fname##_groups,		\
		.ngroups = ARRAY_SIZE(fname##_groups),	\
		.mux_val = mval,			\
	}

/**
 * struct anlogic_pinctrl - driver data
 * @pctrl:		Pinctrl device
 * @groups:		Pingroups
 * @ngroups:		Number of @groups
 * @funcs:		Pinmux functions
 * @nfuncs:		Number of @funcs
 */
struct anlogic_pinctrl {
	struct pinctrl_dev *pctrl;
	void __iomem *iobase;
	const struct anlogic_pctrl_group *groups;
	unsigned int ngroups;
	const struct anlogic_pinmux_function *funcs;
	unsigned int nfuncs;
};

struct anlogic_pctrl_group {
	const char *name;
	const unsigned int *pins;
	const unsigned int npins;
	unsigned int emio_sel_offset;  // 表示EMIO选择寄存器的偏移
	unsigned int emio_supported;   // 表示是否支持EMIO
};

/**
 * struct anlogic_pinmux_function - a pinmux function
 * @name:	Name of the pinmux function.
 * @groups:	List of pingroups for this function.
 * @ngroups:	Number of entries in @groups.
 * @mux_val:	Selector for this function
 */
struct anlogic_pinmux_function {
	const char *name;
	const char * const *groups;
	unsigned int ngroups;
	unsigned int mux_val;
};

enum anlogic_pinmux_functions {
	ANLOGIC_PMUX_qspi0,
	ANLOGIC_PMUX_i2c0,
	ANLOGIC_PMUX_i2c1,
	ANLOGIC_PMUX_can0,
	ANLOGIC_PMUX_can1,
	ANLOGIC_PMUX_smc0,
	ANLOGIC_PMUX_uart0,
	ANLOGIC_PMUX_uart1,
	ANLOGIC_PMUX_gpio,
	ANLOGIC_PMUX_swdt0,
	ANLOGIC_PMUX_ttc0,
	ANLOGIC_PMUX_ttc1,
	ANLOGIC_PMUX_spi0,
	ANLOGIC_PMUX_spi1,
	ANLOGIC_PMUX_sdio0_cd,
	ANLOGIC_PMUX_sdio0,
	ANLOGIC_PMUX_sdio1,
	ANLOGIC_PMUX_ethernet0,
	ANLOGIC_PMUX_ethernet1,
	ANLOGIC_PMUX_mdio0,
	ANLOGIC_PMUX_mdio1,
	ANLOGIC_PMUX_usb0,
	ANLOGIC_PMUX_usb1,
	ANLOGIC_PMUX_MAX_FUNC
};

#endif
