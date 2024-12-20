// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Anlogic dr1x pin controller
 *
 * Copyright (C) 2024 Anlogic, Inc.
 */

#include <linux/io.h>
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pinctrl/pinctrl.h>
#include <linux/pinctrl/pinmux.h>
#include <linux/pinctrl/pinconf.h>
#include <linux/pinctrl/pinconf-generic.h>
#include <linux/regmap.h>
#include "pinctrl-utils.h"
#include "core.h"
#include "pinctrl-anlogic-dr1x.h"

static const struct pinctrl_pin_desc anlogic_pins[] = {
	PINCTRL_PIN(0,  "MIO0"),
	PINCTRL_PIN(1,  "MIO1"),
	PINCTRL_PIN(2,  "MIO2"),
	PINCTRL_PIN(3,  "MIO3"),
	PINCTRL_PIN(4,  "MIO4"),
	PINCTRL_PIN(5,  "MIO5"),
	PINCTRL_PIN(6,  "MIO6"),
	PINCTRL_PIN(7,  "MIO7"),
	PINCTRL_PIN(8,  "MIO8"),
	PINCTRL_PIN(9,  "MIO9"),
	PINCTRL_PIN(10, "MIO10"),
	PINCTRL_PIN(11, "MIO11"),
	PINCTRL_PIN(12, "MIO12"),
	PINCTRL_PIN(13, "MIO13"),
	PINCTRL_PIN(14, "MIO14"),
	PINCTRL_PIN(15, "MIO15"),
	PINCTRL_PIN(16, "MIO16"),
	PINCTRL_PIN(17, "MIO17"),
	PINCTRL_PIN(18, "MIO18"),
	PINCTRL_PIN(19, "MIO19"),
	PINCTRL_PIN(20, "MIO20"),
	PINCTRL_PIN(21, "MIO21"),
	PINCTRL_PIN(22, "MIO22"),
	PINCTRL_PIN(23, "MIO23"),
	PINCTRL_PIN(24, "MIO24"),
	PINCTRL_PIN(25, "MIO25"),
	PINCTRL_PIN(26, "MIO26"),
	PINCTRL_PIN(27, "MIO27"),
	PINCTRL_PIN(28, "MIO28"),
	PINCTRL_PIN(29, "MIO29"),
	PINCTRL_PIN(30, "MIO30"),
	PINCTRL_PIN(31, "MIO31"),
	PINCTRL_PIN(32, "MIO32"),
	PINCTRL_PIN(33, "MIO33"),
	PINCTRL_PIN(34, "MIO34"),
	PINCTRL_PIN(35, "MIO35"),
	PINCTRL_PIN(36, "MIO36"),
	PINCTRL_PIN(37, "MIO37"),
	PINCTRL_PIN(38, "MIO38"),
	PINCTRL_PIN(39, "MIO39"),
	PINCTRL_PIN(40, "MIO40"),
	PINCTRL_PIN(41, "MIO41"),
	PINCTRL_PIN(42, "MIO42"),
	PINCTRL_PIN(43, "MIO43"),
	PINCTRL_PIN(44, "MIO44"),
	PINCTRL_PIN(45, "MIO45"),
	PINCTRL_PIN(46, "MIO46"),
	PINCTRL_PIN(47, "MIO47"),
	PINCTRL_PIN(48, "MIO48"),
	PINCTRL_PIN(49, "MIO49"),
	PINCTRL_PIN(50, "MIO50"),
	PINCTRL_PIN(51, "MIO51"),
	PINCTRL_PIN(52, "MIO52"),
	PINCTRL_PIN(53, "MIO53"),
};

/* pin groups */
/*********************** qspi0 pin groups ***************************/
static const unsigned int qspi0_0_pins[] = {1, 2, 3, 4, 5, 6};
/*********************** i2c0 pin groups ***************************/
static const unsigned int i2c0_0_pins[] = {10, 11};
static const unsigned int i2c0_1_pins[] = {14, 15};
static const unsigned int i2c0_2_pins[] = {18, 19};
static const unsigned int i2c0_3_pins[] = {22, 23};
static const unsigned int i2c0_4_pins[] = {26, 27};
static const unsigned int i2c0_5_pins[] = {30, 31};
static const unsigned int i2c0_6_pins[] = {34, 35};
static const unsigned int i2c0_7_pins[] = {38, 39};
static const unsigned int i2c0_8_pins[] = {42, 43};
static const unsigned int i2c0_9_pins[] = {46, 47};
static const unsigned int i2c0_10_pins[] = {50, 51};
static const unsigned int i2c0_emio_pins[] = {};
/*********************** i2c1 pin groups ***************************/
static const unsigned int i2c1_0_pins[] = {12, 13};
static const unsigned int i2c1_1_pins[] = {16, 17};
static const unsigned int i2c1_2_pins[] = {20, 21};
static const unsigned int i2c1_3_pins[] = {24, 25};
static const unsigned int i2c1_4_pins[] = {28, 29};
static const unsigned int i2c1_5_pins[] = {32, 33};
static const unsigned int i2c1_6_pins[] = {36, 37};
static const unsigned int i2c1_7_pins[] = {40, 41};
static const unsigned int i2c1_8_pins[] = {44, 45};
static const unsigned int i2c1_9_pins[] = {48, 49};
static const unsigned int i2c1_10_pins[] = {52, 53};
static const unsigned int i2c1_emio_pins[] = {};
/*********************** can0 groups ***************************/
static const unsigned int can0_0_pins[] = {10, 11};
static const unsigned int can0_1_pins[] = {14, 15};
static const unsigned int can0_2_pins[] = {18, 19};
static const unsigned int can0_3_pins[] = {22, 23};
static const unsigned int can0_4_pins[] = {26, 27};
static const unsigned int can0_5_pins[] = {30, 31};
static const unsigned int can0_6_pins[] = {34, 35};
static const unsigned int can0_7_pins[] = {38, 39};
static const unsigned int can0_8_pins[] = {42, 43};
static const unsigned int can0_9_pins[] = {46, 47};
static const unsigned int can0_10_pins[] = {50, 51};
static const unsigned int can0_emio_pins[] = {};
/*********************** can1 groups ***************************/
static const unsigned int can1_0_pins[] = {8, 9};
static const unsigned int can1_1_pins[] = {12, 13};
static const unsigned int can1_2_pins[] = {16, 17};
static const unsigned int can1_3_pins[] = {20, 21};
static const unsigned int can1_4_pins[] = {24, 25};
static const unsigned int can1_5_pins[] = {28, 29};
static const unsigned int can1_6_pins[] = {32, 33};
static const unsigned int can1_7_pins[] = {36, 37};
static const unsigned int can1_8_pins[] = {40, 41};
static const unsigned int can1_9_pins[] = {44, 45};
static const unsigned int can1_10_pins[] = {48, 49};
static const unsigned int can1_11_pins[] = {52, 53};
static const unsigned int can1_emio_pins[] = {};
/*********************** smc0 nand 16bit groups ***************************/
static const unsigned int smc0_nand16_pins[] = {0, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11,
					      12, 13, 14, 16, 17, 18, 19, 20,
					      21, 22, 23};
/*********************** smc0 nand 8bit groups ***************************/
static const unsigned int smc0_nand8_pins[] = {0, 2, 3,  4,  5,  6,  7,
					       8, 9, 10, 11, 12, 13, 14};
/*********************** uart0 groups ***************************/
static const unsigned int uart0_0_pins[] = {10, 11};
static const unsigned int uart0_1_pins[] = {14, 15};
static const unsigned int uart0_2_pins[] = {18, 19};
static const unsigned int uart0_3_pins[] = {22, 23};
static const unsigned int uart0_4_pins[] = {26, 27};
static const unsigned int uart0_5_pins[] = {30, 31};
static const unsigned int uart0_6_pins[] = {34, 35};
static const unsigned int uart0_7_pins[] = {38, 39};
static const unsigned int uart0_8_pins[] = {42, 43};
static const unsigned int uart0_9_pins[] = {46, 47};
static const unsigned int uart0_10_pins[] = {50, 51};
static const unsigned int uart0_emio_pins[] = {};
/*********************** uart1 groups ***************************/
static const unsigned int uart1_0_pins[] = {8, 9};
static const unsigned int uart1_1_pins[] = {12, 13};
static const unsigned int uart1_2_pins[] = {16, 17};
static const unsigned int uart1_3_pins[] = {20, 21};
static const unsigned int uart1_4_pins[] = {24, 25};
static const unsigned int uart1_5_pins[] = {28, 29};
static const unsigned int uart1_6_pins[] = {32, 33};
static const unsigned int uart1_7_pins[] = {36, 37};
static const unsigned int uart1_8_pins[] = {40, 41};
static const unsigned int uart1_9_pins[] = {44, 45};
static const unsigned int uart1_10_pins[] = {48, 49};
static const unsigned int uart1_11_pins[] = {52, 53};
static const unsigned int uart1_emio_pins[] = {};
/*********************** SWDT0 groups ***************************/
static const unsigned int swdt0_0_pins[] = {14, 15};
static const unsigned int swdt0_1_pins[] = {26, 27};
static const unsigned int swdt0_2_pins[] = {38, 39};
static const unsigned int swdt0_3_pins[] = {50, 51};
static const unsigned int swdt0_4_pins[] = {52, 53};
static const unsigned int swdt0_emio_pins[] = {};
/*********************** ttc0 groups ***************************/
static const unsigned int ttc0_0_pins[] = {18, 19};
static const unsigned int ttc0_1_pins[] = {30, 31};
static const unsigned int ttc0_2_pins[] = {42, 43};
static const unsigned int ttc0_emio_pins[] = {};
/*********************** ttc1 groups ***************************/
static const unsigned int ttc1_0_pins[] = {16, 17};
static const unsigned int ttc1_1_pins[] = {28, 29};
static const unsigned int ttc1_2_pins[] = {40, 41};
static const unsigned int ttc1_emio_pins[] = {};
/*********************** spi0 groups ***************************/
static const unsigned int spi0_0_ss0_pins[] = {16, 17, 21, 18};
static const unsigned int spi0_0_ss1_pins[] = {16, 17, 21, 19};
static const unsigned int spi0_0_ss2_pins[] = {16, 17, 21, 20};
static const unsigned int spi0_1_ss0_pins[] = {28, 29, 33, 30};
static const unsigned int spi0_1_ss1_pins[] = {28, 29, 33, 31};
static const unsigned int spi0_1_ss2_pins[] = {28, 29, 33, 32};
static const unsigned int spi0_2_ss0_pins[] = {40, 41, 45, 42};
static const unsigned int spi0_2_ss1_pins[] = {40, 41, 45, 43};
static const unsigned int spi0_2_ss2_pins[] = {40, 41, 45, 44};
static const unsigned int spi0_emio_pins[] = {};
/*********************** spi1 groups ***************************/
static const unsigned int spi1_0_ss0_pins[] = {10, 11, 12, 13};
static const unsigned int spi1_0_ss1_pins[] = {10, 11, 12, 14};
static const unsigned int spi1_0_ss2_pins[] = {10, 11, 12, 15};
static const unsigned int spi1_1_ss0_pins[] = {22, 23, 24, 25};
static const unsigned int spi1_1_ss1_pins[] = {22, 23, 24, 26};
static const unsigned int spi1_1_ss2_pins[] = {22, 23, 24, 27};
static const unsigned int spi1_2_ss0_pins[] = {34, 35, 36, 37};
static const unsigned int spi1_2_ss1_pins[] = {34, 35, 36, 38};
static const unsigned int spi1_2_ss2_pins[] = {34, 35, 36, 39};
static const unsigned int spi1_3_ss0_pins[] = {46, 47, 48, 49};
static const unsigned int spi1_3_ss1_pins[] = {46, 47, 48, 50};
static const unsigned int spi1_3_ss2_pins[] = {46, 47, 48, 51};
static const unsigned int spi1_emio_pins[] = {};
/*********************** sdio0 cd pin groups ***************************/
static const unsigned int sdio0_cd_pins[] = {0};
/*********************** sdio0 pin groups ***************************/
static const unsigned int sdio0_0_pins[] = {16, 17, 18, 19, 20, 21};
static const unsigned int sdio0_1_pins[] = {28, 29, 30, 31, 32, 33};
static const unsigned int sdio0_2_pins[] = {40, 41, 42, 43, 44, 45};
static const unsigned int sdio0_emio_pins[] = {};
/*********************** sdio1 pin groups ***************************/
static const unsigned int sdio1_0_pins[] = {10, 11, 12, 13, 14, 15};
static const unsigned int sdio1_1_pins[] = {22, 23, 24, 25, 26, 27};
static const unsigned int sdio1_2_pins[] = {34, 35, 36, 37, 38, 39};
static const unsigned int sdio1_3_pins[] = {46, 47, 48, 49, 50, 51};
static const unsigned int sdio1_emio_pins[] = {};
/*********************** eth0 pin groups ***************************/
static const unsigned int ethernet0_0_pins[] = {16, 17, 18, 19, 20, 21, 22, 23,
						24, 25, 26, 27};
/*********************** eth1 pin groups ***************************/
static const unsigned int ethernet1_0_pins[] = {28, 29, 30, 31, 32, 33, 34, 35,
						36, 37, 38, 39};
/*********************** mdio0 pin groups ***************************/
static const unsigned int mdio0_0_pins[] = {52, 53};
static const unsigned int mdio0_emio_pins[] = {};
/*********************** mdio1 pin groups ***************************/
static const unsigned int mdio1_0_pins[] = {52, 53};
static const unsigned int mdio1_emio_pins[] = {};
/*********************** usb0 pin groups ***************************/
static const unsigned int usb0_0_pins[] = {28, 29, 30, 31, 32, 33, 34, 35, 36,
					   37, 38, 39};
/*********************** usb1 pin groups ***************************/
static const unsigned int usb1_0_pins[] = {40, 41, 42, 43, 44, 45, 46, 47, 48,
					   49, 50, 51};
/*********************** gpio groups ***************************/
static const unsigned int gpio_0_pins[] = {0};
static const unsigned int gpio_1_pins[] = {1};
static const unsigned int gpio_2_pins[] = {2};
static const unsigned int gpio_3_pins[] = {3};
static const unsigned int gpio_4_pins[] = {4};
static const unsigned int gpio_5_pins[] = {5};
static const unsigned int gpio_6_pins[] = {6};
static const unsigned int gpio_7_pins[] = {7};
static const unsigned int gpio_8_pins[] = {8};
static const unsigned int gpio_9_pins[] = {9};
static const unsigned int gpio_10_pins[] = {10};
static const unsigned int gpio_11_pins[] = {11};
static const unsigned int gpio_12_pins[] = {12};
static const unsigned int gpio_13_pins[] = {13};
static const unsigned int gpio_14_pins[] = {14};
static const unsigned int gpio_15_pins[] = {15};
static const unsigned int gpio_16_pins[] = {16};
static const unsigned int gpio_17_pins[] = {17};
static const unsigned int gpio_18_pins[] = {18};
static const unsigned int gpio_19_pins[] = {19};
static const unsigned int gpio_20_pins[] = {20};
static const unsigned int gpio_21_pins[] = {21};
static const unsigned int gpio_22_pins[] = {22};
static const unsigned int gpio_23_pins[] = {23};
static const unsigned int gpio_24_pins[] = {24};
static const unsigned int gpio_25_pins[] = {25};
static const unsigned int gpio_26_pins[] = {26};
static const unsigned int gpio_27_pins[] = {27};
static const unsigned int gpio_28_pins[] = {28};
static const unsigned int gpio_29_pins[] = {29};
static const unsigned int gpio_30_pins[] = {30};
static const unsigned int gpio_31_pins[] = {31};
static const unsigned int gpio_32_pins[] = {32};
static const unsigned int gpio_33_pins[] = {33};
static const unsigned int gpio_34_pins[] = {34};
static const unsigned int gpio_35_pins[] = {35};
static const unsigned int gpio_36_pins[] = {36};
static const unsigned int gpio_37_pins[] = {37};
static const unsigned int gpio_38_pins[] = {38};
static const unsigned int gpio_39_pins[] = {39};
static const unsigned int gpio_40_pins[] = {40};
static const unsigned int gpio_41_pins[] = {41};
static const unsigned int gpio_42_pins[] = {42};
static const unsigned int gpio_43_pins[] = {43};
static const unsigned int gpio_44_pins[] = {44};
static const unsigned int gpio_45_pins[] = {45};
static const unsigned int gpio_46_pins[] = {46};
static const unsigned int gpio_47_pins[] = {47};
static const unsigned int gpio_48_pins[] = {48};
static const unsigned int gpio_49_pins[] = {49};
static const unsigned int gpio_50_pins[] = {50};
static const unsigned int gpio_51_pins[] = {51};
static const unsigned int gpio_52_pins[] = {52};
static const unsigned int gpio_53_pins[] = {53};

static const struct anlogic_pctrl_group anlogic_pctrl_groups[] = {
/*********************** i2c0 groups ***************************/
	DEFINE_ANLOGIC_PINCTRL_GRP(i2c0_0, 0, true),
	DEFINE_ANLOGIC_PINCTRL_GRP(i2c0_1, 0, true),
	DEFINE_ANLOGIC_PINCTRL_GRP(i2c0_2, 0, true),
	DEFINE_ANLOGIC_PINCTRL_GRP(i2c0_3, 0, true),
	DEFINE_ANLOGIC_PINCTRL_GRP(i2c0_4, 0, true),
	DEFINE_ANLOGIC_PINCTRL_GRP(i2c0_5, 0, true),
	DEFINE_ANLOGIC_PINCTRL_GRP(i2c0_6, 0, true),
	DEFINE_ANLOGIC_PINCTRL_GRP(i2c0_7, 0, true),
	DEFINE_ANLOGIC_PINCTRL_GRP(i2c0_8, 0, true),
	DEFINE_ANLOGIC_PINCTRL_GRP(i2c0_9, 0, true),
	DEFINE_ANLOGIC_PINCTRL_GRP(i2c0_10, 0, true),
	DEFINE_ANLOGIC_PINCTRL_GRP(i2c0_emio, 0, true),
/*********************** i2c1 groups ***************************/
	DEFINE_ANLOGIC_PINCTRL_GRP(i2c1_0, 1, true),
	DEFINE_ANLOGIC_PINCTRL_GRP(i2c1_1, 1, true),
	DEFINE_ANLOGIC_PINCTRL_GRP(i2c1_2, 1, true),
	DEFINE_ANLOGIC_PINCTRL_GRP(i2c1_3, 1, true),
	DEFINE_ANLOGIC_PINCTRL_GRP(i2c1_4, 1, true),
	DEFINE_ANLOGIC_PINCTRL_GRP(i2c1_5, 1, true),
	DEFINE_ANLOGIC_PINCTRL_GRP(i2c1_6, 1, true),
	DEFINE_ANLOGIC_PINCTRL_GRP(i2c1_7, 1, true),
	DEFINE_ANLOGIC_PINCTRL_GRP(i2c1_8, 1, true),
	DEFINE_ANLOGIC_PINCTRL_GRP(i2c1_9, 1, true),
	DEFINE_ANLOGIC_PINCTRL_GRP(i2c1_10, 1, true),
	DEFINE_ANLOGIC_PINCTRL_GRP(i2c1_emio, 1, true),
/*********************** can0 groups ***************************/
	DEFINE_ANLOGIC_PINCTRL_GRP(can0_0, 2, true),
	DEFINE_ANLOGIC_PINCTRL_GRP(can0_1, 2, true),
	DEFINE_ANLOGIC_PINCTRL_GRP(can0_2, 2, true),
	DEFINE_ANLOGIC_PINCTRL_GRP(can0_3, 2, true),
	DEFINE_ANLOGIC_PINCTRL_GRP(can0_4, 2, true),
	DEFINE_ANLOGIC_PINCTRL_GRP(can0_5, 2, true),
	DEFINE_ANLOGIC_PINCTRL_GRP(can0_6, 2, true),
	DEFINE_ANLOGIC_PINCTRL_GRP(can0_7, 2, true),
	DEFINE_ANLOGIC_PINCTRL_GRP(can0_8, 2, true),
	DEFINE_ANLOGIC_PINCTRL_GRP(can0_9, 2, true),
	DEFINE_ANLOGIC_PINCTRL_GRP(can0_10, 2, true),
	DEFINE_ANLOGIC_PINCTRL_GRP(can0_emio, 2, true),
/*********************** can1 groups ***************************/
	DEFINE_ANLOGIC_PINCTRL_GRP(can1_0, 3, true),
	DEFINE_ANLOGIC_PINCTRL_GRP(can1_1, 3, true),
	DEFINE_ANLOGIC_PINCTRL_GRP(can1_2, 3, true),
	DEFINE_ANLOGIC_PINCTRL_GRP(can1_3, 3, true),
	DEFINE_ANLOGIC_PINCTRL_GRP(can1_4, 3, true),
	DEFINE_ANLOGIC_PINCTRL_GRP(can1_5, 3, true),
	DEFINE_ANLOGIC_PINCTRL_GRP(can1_6, 3, true),
	DEFINE_ANLOGIC_PINCTRL_GRP(can1_7, 3, true),
	DEFINE_ANLOGIC_PINCTRL_GRP(can1_8, 3, true),
	DEFINE_ANLOGIC_PINCTRL_GRP(can1_9, 3, true),
	DEFINE_ANLOGIC_PINCTRL_GRP(can1_10, 3, true),
	DEFINE_ANLOGIC_PINCTRL_GRP(can1_11, 3, true),
	DEFINE_ANLOGIC_PINCTRL_GRP(can1_emio, 3, true),
/*********************** uart0 groups ***************************/
	DEFINE_ANLOGIC_PINCTRL_GRP(uart0_0, 4, true),
	DEFINE_ANLOGIC_PINCTRL_GRP(uart0_1, 4, true),
	DEFINE_ANLOGIC_PINCTRL_GRP(uart0_2, 4, true),
	DEFINE_ANLOGIC_PINCTRL_GRP(uart0_3, 4, true),
	DEFINE_ANLOGIC_PINCTRL_GRP(uart0_4, 4, true),
	DEFINE_ANLOGIC_PINCTRL_GRP(uart0_5, 4, true),
	DEFINE_ANLOGIC_PINCTRL_GRP(uart0_6, 4, true),
	DEFINE_ANLOGIC_PINCTRL_GRP(uart0_7, 4, true),
	DEFINE_ANLOGIC_PINCTRL_GRP(uart0_8, 4, true),
	DEFINE_ANLOGIC_PINCTRL_GRP(uart0_9, 4, true),
	DEFINE_ANLOGIC_PINCTRL_GRP(uart0_10, 4, true),
	DEFINE_ANLOGIC_PINCTRL_GRP(uart0_emio, 4, true),
/*********************** uart1 groups ***************************/
	DEFINE_ANLOGIC_PINCTRL_GRP(uart1_0, 5, true),
	DEFINE_ANLOGIC_PINCTRL_GRP(uart1_1, 5, true),
	DEFINE_ANLOGIC_PINCTRL_GRP(uart1_2, 5, true),
	DEFINE_ANLOGIC_PINCTRL_GRP(uart1_3, 5, true),
	DEFINE_ANLOGIC_PINCTRL_GRP(uart1_4, 5, true),
	DEFINE_ANLOGIC_PINCTRL_GRP(uart1_5, 5, true),
	DEFINE_ANLOGIC_PINCTRL_GRP(uart1_6, 5, true),
	DEFINE_ANLOGIC_PINCTRL_GRP(uart1_7, 5, true),
	DEFINE_ANLOGIC_PINCTRL_GRP(uart1_8, 5, true),
	DEFINE_ANLOGIC_PINCTRL_GRP(uart1_9, 5, true),
	DEFINE_ANLOGIC_PINCTRL_GRP(uart1_10, 5, true),
	DEFINE_ANLOGIC_PINCTRL_GRP(uart1_11, 5, true),
	DEFINE_ANLOGIC_PINCTRL_GRP(uart1_emio, 5, true),
/*********************** ttc0 groups ***************************/
	DEFINE_ANLOGIC_PINCTRL_GRP(ttc0_0, 6, true),
	DEFINE_ANLOGIC_PINCTRL_GRP(ttc0_1, 6, true),
	DEFINE_ANLOGIC_PINCTRL_GRP(ttc0_2, 6, true),
	DEFINE_ANLOGIC_PINCTRL_GRP(ttc0_emio, 6, true),
/*********************** ttc1 groups ***************************/
	DEFINE_ANLOGIC_PINCTRL_GRP(ttc1_0, 7, true),
	DEFINE_ANLOGIC_PINCTRL_GRP(ttc1_1, 7, true),
	DEFINE_ANLOGIC_PINCTRL_GRP(ttc1_2, 7, true),
	DEFINE_ANLOGIC_PINCTRL_GRP(ttc1_emio, 7, true),
/*********************** swdt0 groups ***************************/
	DEFINE_ANLOGIC_PINCTRL_GRP(swdt0_0, 8, true),
	DEFINE_ANLOGIC_PINCTRL_GRP(swdt0_1, 8, true),
	DEFINE_ANLOGIC_PINCTRL_GRP(swdt0_2, 8, true),
	DEFINE_ANLOGIC_PINCTRL_GRP(swdt0_3, 8, true),
	DEFINE_ANLOGIC_PINCTRL_GRP(swdt0_emio, 8, true),
/*********************** spi0 groups ***************************/
	DEFINE_ANLOGIC_PINCTRL_GRP(spi0_0_ss0, 9, true),
	DEFINE_ANLOGIC_PINCTRL_GRP(spi0_0_ss1, 9, true),
	DEFINE_ANLOGIC_PINCTRL_GRP(spi0_0_ss2, 9, true),
	DEFINE_ANLOGIC_PINCTRL_GRP(spi0_1_ss0, 9, true),
	DEFINE_ANLOGIC_PINCTRL_GRP(spi0_1_ss1, 9, true),
	DEFINE_ANLOGIC_PINCTRL_GRP(spi0_1_ss2, 9, true),
	DEFINE_ANLOGIC_PINCTRL_GRP(spi0_2_ss0, 9, true),
	DEFINE_ANLOGIC_PINCTRL_GRP(spi0_2_ss1, 9, true),
	DEFINE_ANLOGIC_PINCTRL_GRP(spi0_2_ss2, 9, true),
	DEFINE_ANLOGIC_PINCTRL_GRP(spi0_emio, 9, true),
/*********************** spi1 groups ***************************/
	DEFINE_ANLOGIC_PINCTRL_GRP(spi1_0_ss0, 10, true),
	DEFINE_ANLOGIC_PINCTRL_GRP(spi1_0_ss1, 10, true),
	DEFINE_ANLOGIC_PINCTRL_GRP(spi1_0_ss2, 10, true),
	DEFINE_ANLOGIC_PINCTRL_GRP(spi1_1_ss0, 10, true),
	DEFINE_ANLOGIC_PINCTRL_GRP(spi1_1_ss1, 10, true),
	DEFINE_ANLOGIC_PINCTRL_GRP(spi1_1_ss2, 10, true),
	DEFINE_ANLOGIC_PINCTRL_GRP(spi1_2_ss0, 10, true),
	DEFINE_ANLOGIC_PINCTRL_GRP(spi1_2_ss1, 10, true),
	DEFINE_ANLOGIC_PINCTRL_GRP(spi1_2_ss2, 10, true),
	DEFINE_ANLOGIC_PINCTRL_GRP(spi1_emio, 10, true),
/*********************** sdio0 groups ***************************/
	DEFINE_ANLOGIC_PINCTRL_GRP(sdio0_0, 11, true),
	DEFINE_ANLOGIC_PINCTRL_GRP(sdio0_1, 11, true),
	DEFINE_ANLOGIC_PINCTRL_GRP(sdio0_2, 11, true),
	DEFINE_ANLOGIC_PINCTRL_GRP(sdio0_emio, 11, true),
/*********************** sdio0_cd groups ***************************/
	DEFINE_ANLOGIC_PINCTRL_GRP(sdio0_cd, false, false),
/*********************** sdio1 groups ***************************/
	DEFINE_ANLOGIC_PINCTRL_GRP(sdio1_0, 12, true),
	DEFINE_ANLOGIC_PINCTRL_GRP(sdio1_1, 12, true),
	DEFINE_ANLOGIC_PINCTRL_GRP(sdio1_2, 12, true),
	DEFINE_ANLOGIC_PINCTRL_GRP(sdio1_3, 12, true),
	DEFINE_ANLOGIC_PINCTRL_GRP(sdio1_emio, 12, true),
/*********************** mdio0 groups ***************************/
	DEFINE_ANLOGIC_PINCTRL_GRP(mdio0_0, 13, true),
	DEFINE_ANLOGIC_PINCTRL_GRP(mdio0_emio, 13, true),
/*********************** mdio1 groups ***************************/
	DEFINE_ANLOGIC_PINCTRL_GRP(mdio1_0, 14, true),
	DEFINE_ANLOGIC_PINCTRL_GRP(mdio1_emio, 14, true),
/*********************** qspi0 groups ***************************/
	DEFINE_ANLOGIC_PINCTRL_GRP(qspi0_0, false, false),
/*********************** usb0 groups ***************************/
	DEFINE_ANLOGIC_PINCTRL_GRP(usb0_0, false, false),
/*********************** usb1 groups ***************************/
	DEFINE_ANLOGIC_PINCTRL_GRP(usb1_0, false, false),
/*********************** eth0 groups ***************************/
	DEFINE_ANLOGIC_PINCTRL_GRP(ethernet0_0, false, false),
/*********************** eth1 groups ***************************/
	DEFINE_ANLOGIC_PINCTRL_GRP(ethernet1_0, false, false),
/*********************** nand 16 groups ***************************/
	DEFINE_ANLOGIC_PINCTRL_GRP(smc0_nand16, false, false),
/*********************** nand 8 groups ***************************/
	DEFINE_ANLOGIC_PINCTRL_GRP(smc0_nand8, false, false),
/*********************** gpio groups ***************************/
	DEFINE_ANLOGIC_PINCTRL_GRP(gpio_0, false, false),
	DEFINE_ANLOGIC_PINCTRL_GRP(gpio_1, false, false),
	DEFINE_ANLOGIC_PINCTRL_GRP(gpio_2, false, false),
	DEFINE_ANLOGIC_PINCTRL_GRP(gpio_3, false, false),
	DEFINE_ANLOGIC_PINCTRL_GRP(gpio_4, false, false),
	DEFINE_ANLOGIC_PINCTRL_GRP(gpio_5, false, false),
	DEFINE_ANLOGIC_PINCTRL_GRP(gpio_6, false, false),
	DEFINE_ANLOGIC_PINCTRL_GRP(gpio_7, false, false),
	DEFINE_ANLOGIC_PINCTRL_GRP(gpio_8, false, false),
	DEFINE_ANLOGIC_PINCTRL_GRP(gpio_9, false, false),
	DEFINE_ANLOGIC_PINCTRL_GRP(gpio_10, false, false),
	DEFINE_ANLOGIC_PINCTRL_GRP(gpio_11, false, false),
	DEFINE_ANLOGIC_PINCTRL_GRP(gpio_12, false, false),
	DEFINE_ANLOGIC_PINCTRL_GRP(gpio_13, false, false),
	DEFINE_ANLOGIC_PINCTRL_GRP(gpio_14, false, false),
	DEFINE_ANLOGIC_PINCTRL_GRP(gpio_15, false, false),
	DEFINE_ANLOGIC_PINCTRL_GRP(gpio_16, false, false),
	DEFINE_ANLOGIC_PINCTRL_GRP(gpio_17, false, false),
	DEFINE_ANLOGIC_PINCTRL_GRP(gpio_18, false, false),
	DEFINE_ANLOGIC_PINCTRL_GRP(gpio_19, false, false),
	DEFINE_ANLOGIC_PINCTRL_GRP(gpio_20, false, false),
	DEFINE_ANLOGIC_PINCTRL_GRP(gpio_21, false, false),
	DEFINE_ANLOGIC_PINCTRL_GRP(gpio_22, false, false),
	DEFINE_ANLOGIC_PINCTRL_GRP(gpio_23, false, false),
	DEFINE_ANLOGIC_PINCTRL_GRP(gpio_24, false, false),
	DEFINE_ANLOGIC_PINCTRL_GRP(gpio_25, false, false),
	DEFINE_ANLOGIC_PINCTRL_GRP(gpio_26, false, false),
	DEFINE_ANLOGIC_PINCTRL_GRP(gpio_27, false, false),
	DEFINE_ANLOGIC_PINCTRL_GRP(gpio_28, false, false),
	DEFINE_ANLOGIC_PINCTRL_GRP(gpio_29, false, false),
	DEFINE_ANLOGIC_PINCTRL_GRP(gpio_30, false, false),
	DEFINE_ANLOGIC_PINCTRL_GRP(gpio_31, false, false),
	DEFINE_ANLOGIC_PINCTRL_GRP(gpio_32, false, false),
	DEFINE_ANLOGIC_PINCTRL_GRP(gpio_33, false, false),
	DEFINE_ANLOGIC_PINCTRL_GRP(gpio_34, false, false),
	DEFINE_ANLOGIC_PINCTRL_GRP(gpio_35, false, false),
	DEFINE_ANLOGIC_PINCTRL_GRP(gpio_36, false, false),
	DEFINE_ANLOGIC_PINCTRL_GRP(gpio_37, false, false),
	DEFINE_ANLOGIC_PINCTRL_GRP(gpio_38, false, false),
	DEFINE_ANLOGIC_PINCTRL_GRP(gpio_39, false, false),
	DEFINE_ANLOGIC_PINCTRL_GRP(gpio_40, false, false),
	DEFINE_ANLOGIC_PINCTRL_GRP(gpio_41, false, false),
	DEFINE_ANLOGIC_PINCTRL_GRP(gpio_42, false, false),
	DEFINE_ANLOGIC_PINCTRL_GRP(gpio_43, false, false),
	DEFINE_ANLOGIC_PINCTRL_GRP(gpio_44, false, false),
	DEFINE_ANLOGIC_PINCTRL_GRP(gpio_45, false, false),
	DEFINE_ANLOGIC_PINCTRL_GRP(gpio_46, false, false),
	DEFINE_ANLOGIC_PINCTRL_GRP(gpio_47, false, false),
	DEFINE_ANLOGIC_PINCTRL_GRP(gpio_48, false, false),
	DEFINE_ANLOGIC_PINCTRL_GRP(gpio_49, false, false),
	DEFINE_ANLOGIC_PINCTRL_GRP(gpio_50, false, false),
	DEFINE_ANLOGIC_PINCTRL_GRP(gpio_51, false, false),
	DEFINE_ANLOGIC_PINCTRL_GRP(gpio_52, false, false),
	DEFINE_ANLOGIC_PINCTRL_GRP(gpio_53, false, false),
};

/* function groups */
static const char * const qspi0_groups[] = {"qspi0_0_grp"};

static const char * const i2c0_groups[] = {"i2c0_0_grp", "i2c0_1_grp",
		"i2c0_2_grp", "i2c0_3_grp", "i2c0_4_grp", "i2c0_5_grp",
		"i2c0_6_grp", "i2c0_7_grp", "i2c0_8_grp", "i2c0_9_grp",
		"i2c0_10_grp", "i2c0_emio_grp"};
static const char * const i2c1_groups[] = {"i2c1_0_grp", "i2c1_1_grp",
		"i2c1_2_grp", "i2c1_3_grp", "i2c1_4_grp", "i2c1_5_grp",
		"i2c1_6_grp", "i2c1_7_grp", "i2c1_8_grp", "i2c1_9_grp",
		"i2c1_10_grp", "i2c1_emio_grp"};

static const char * const can0_groups[] = {"can0_0_grp", "can0_1_grp",
		"can0_2_grp", "can0_3_grp", "can0_4_grp", "can0_5_grp",
		"can0_6_grp", "can0_7_grp", "can0_8_grp", "can0_9_grp",
		"can0_10_grp", "can0_emio_grp"};
static const char * const can1_groups[] = {"can1_0_grp", "can1_1_grp",
		"can1_2_grp", "can1_3_grp", "can1_4_grp", "can1_5_grp",
		"can1_6_grp", "can1_7_grp", "can1_8_grp", "can1_9_grp",
		"can1_10_grp", "can1_11_grp", "can1_emio_grp"};

static const char * const smc0_groups[] = {"smc0_nand8_grp", "smc0_nand16_grp"};

static const char * const uart0_groups[] = {"uart0_0_grp", "uart0_1_grp",
		"uart0_2_grp", "uart0_3_grp", "uart0_4_grp", "uart0_5_grp",
		"uart0_6_grp", "uart0_7_grp", "uart0_8_grp", "uart0_9_grp",
		"uart0_10_grp", "uart0_emio_grp"};
static const char * const uart1_groups[] = {"uart1_0_grp", "uart1_1_grp",
		"uart1_2_grp", "uart1_3_grp", "uart1_4_grp", "uart1_5_grp",
		"uart1_6_grp", "uart1_7_grp", "uart1_8_grp", "uart1_9_grp",
		"uart1_10_grp", "uart1_11_grp", "uart1_emio_grp"};

static const char * const gpio_groups[] = {"gpio_0_grp",
		"gpio_2_grp", "gpio_4_grp", "gpio_6_grp",
		"gpio_8_grp", "gpio_10_grp", "gpio_12_grp",
		"gpio_14_grp", "gpio_16_grp", "gpio_18_grp",
		"gpio_20_grp", "gpio_22_grp", "gpio_24_grp",
		"gpio_26_grp", "gpio_28_grp", "gpio_30_grp",
		"gpio_32_grp", "gpio_34_grp", "gpio_36_grp",
		"gpio_38_grp", "gpio_40_grp", "gpio_42_grp",
		"gpio_44_grp", "gpio_46_grp", "gpio_48_grp",
		"gpio_50_grp", "gpio_52_grp", "gpio_1_grp",
		"gpio_3_grp", "gpio_5_grp", "gpio_7_grp",
		"gpio_9_grp", "gpio_11_grp", "gpio_13_grp",
		"gpio_15_grp", "gpio_17_grp", "gpio_19_grp",
		"gpio_21_grp", "gpio_23_grp", "gpio_25_grp",
		"gpio_27_grp", "gpio_29_grp", "gpio_31_grp",
		"gpio_33_grp", "gpio_35_grp", "gpio_37_grp",
		"gpio_39_grp", "gpio_41_grp", "gpio_43_grp",
		"gpio_45_grp", "gpio_47_grp", "gpio_49_grp",
		"gpio_51_grp", "gpio_53_grp"};

static const char * const swdt0_groups[] = {"swdt0_0_grp", "swdt0_1_grp", "swdt0_2_grp",
					   "swdt0_3_grp", "swdt0_emio_grp"};

static const char * const ttc0_groups[] = {"ttc0_0_grp", "ttc0_1_grp",
					   "ttc0_2_grp", "ttc0_emio_grp"};
static const char * const ttc1_groups[] = {"ttc1_0_grp", "ttc1_1_grp",
					   "ttc1_2_grp", "ttc1_emio_grp"};

static const char * const spi0_groups[] = {"spi0_0_ss0_grp", "spi0_0_ss1_grp", "spi0_0_ss2_grp",
					   "spi0_1_ss0_grp", "spi0_1_ss1_grp", "spi0_1_ss2_grp",
					   "spi0_2_ss0_grp", "spi0_2_ss1_grp", "spi0_2_ss2_grp",
					   "spi0_emio_grp"};
static const char * const spi1_groups[] = {"spi1_0_ss0_grp", "spi1_0_ss1_grp", "spi1_0_ss2_grp",
					   "spi1_1_ss0_grp", "spi1_1_ss1_grp", "spi1_1_ss2_grp",
					   "spi1_2_ss0_grp", "spi1_2_ss1_grp", "spi1_2_ss2_grp",
					   "spi1_emio_grp"};

static const char * const sdio0_cd_groups[] = {"gpio_0_grp"};
static const char * const sdio0_groups[] = {"sdio0_0_grp", "sdio0_1_grp", "sdio0_2_grp",
					    "sdio0_emio_grp"};

static const char * const sdio1_groups[] = {"sdio1_0_grp", "sdio1_1_grp", "sdio1_2_grp",
					    "sdio1_3_grp", "sdio1_emio_grp"};

static const char * const ethernet0_groups[] = {"ethernet0_0_grp"};
static const char * const ethernet1_groups[] = {"ethernet1_0_grp"};

static const char * const mdio0_groups[] = {"mdio0_0_grp", "mdio0_emio_grp"};
static const char * const mdio1_groups[] = {"mdio1_0_grp", "mdio1_emio_grp"};

static const char * const usb0_groups[] = {"usb0_0_grp"};
static const char * const usb1_groups[] = {"usb1_0_grp"};

static const struct anlogic_pinmux_function anlogic_pmux_functions[] = {
	DEFINE_ANLOGIC_PINMUX_FUNCTION(qspi0, 0),
	DEFINE_ANLOGIC_PINMUX_FUNCTION(i2c0, 0),
	DEFINE_ANLOGIC_PINMUX_FUNCTION(i2c1, 0),
	DEFINE_ANLOGIC_PINMUX_FUNCTION(can0, 1),
	DEFINE_ANLOGIC_PINMUX_FUNCTION(can1, 1),
	DEFINE_ANLOGIC_PINMUX_FUNCTION(smc0, 2),
	DEFINE_ANLOGIC_PINMUX_FUNCTION(uart0, 3),
	DEFINE_ANLOGIC_PINMUX_FUNCTION(uart1, 3),
	DEFINE_ANLOGIC_PINMUX_FUNCTION(gpio, 4),
	DEFINE_ANLOGIC_PINMUX_FUNCTION(swdt0, 7),
	DEFINE_ANLOGIC_PINMUX_FUNCTION(ttc0, 7),
	DEFINE_ANLOGIC_PINMUX_FUNCTION(ttc1, 7),
	DEFINE_ANLOGIC_PINMUX_FUNCTION(spi0, 9),
	DEFINE_ANLOGIC_PINMUX_FUNCTION(spi1, 9),
	DEFINE_ANLOGIC_PINMUX_FUNCTION(sdio0_cd, 4),
	DEFINE_ANLOGIC_PINMUX_FUNCTION(sdio0, 10),
	DEFINE_ANLOGIC_PINMUX_FUNCTION(sdio1, 10),
	DEFINE_ANLOGIC_PINMUX_FUNCTION(ethernet0, 14),
	DEFINE_ANLOGIC_PINMUX_FUNCTION(ethernet1, 14),
	DEFINE_ANLOGIC_PINMUX_FUNCTION(mdio0, 14),
	DEFINE_ANLOGIC_PINMUX_FUNCTION(mdio1, 15),
	DEFINE_ANLOGIC_PINMUX_FUNCTION(usb0, 15),
	DEFINE_ANLOGIC_PINMUX_FUNCTION(usb1, 15),
};

/* pinctrl */
static int anlogic_pctrl_get_groups_count(struct pinctrl_dev *pctldev)
{
	struct anlogic_pinctrl *pctrl = pinctrl_dev_get_drvdata(pctldev);

	return pctrl->ngroups;
}

static const char *anlogic_pctrl_get_group_name(struct pinctrl_dev *pctldev,
					     unsigned int selector)
{
	struct anlogic_pinctrl *pctrl = pinctrl_dev_get_drvdata(pctldev);

	return pctrl->groups[selector].name;
}

static int anlogic_pctrl_get_group_pins(struct pinctrl_dev *pctldev,
				     unsigned int selector,
				     const unsigned int **pins,
				     unsigned int *num_pins)
{
	struct anlogic_pinctrl *pctrl = pinctrl_dev_get_drvdata(pctldev);

	*pins = pctrl->groups[selector].pins;
	*num_pins = pctrl->groups[selector].npins;

	return 0;
}

static const struct pinctrl_ops anlogic_pctrl_ops = {
	.get_groups_count = anlogic_pctrl_get_groups_count,
	.get_group_name = anlogic_pctrl_get_group_name,
	.get_group_pins = anlogic_pctrl_get_group_pins,
	.dt_node_to_map = pinconf_generic_dt_node_to_map_all,
	.dt_free_map = pinctrl_utils_free_map,
};

/* pinmux */
static int anlogic_pmux_get_functions_count(struct pinctrl_dev *pctldev)
{
	struct anlogic_pinctrl *pctrl = pinctrl_dev_get_drvdata(pctldev);

	return pctrl->nfuncs;
}

static const char *anlogic_pmux_get_function_name(struct pinctrl_dev *pctldev,
					       unsigned int selector)
{
	struct anlogic_pinctrl *pctrl = pinctrl_dev_get_drvdata(pctldev);

	return pctrl->funcs[selector].name;
}

static int anlogic_pmux_get_function_groups(struct pinctrl_dev *pctldev,
					 unsigned int selector,
					 const char * const **groups,
					 unsigned * const num_groups)
{
	struct anlogic_pinctrl *pctrl = pinctrl_dev_get_drvdata(pctldev);

	*groups = pctrl->funcs[selector].groups;
	*num_groups = pctrl->funcs[selector].ngroups;
	return 0;
}

static int anlogic_pinmux_set_mux(struct pinctrl_dev *pctldev,
			       unsigned int function,
			       unsigned int  group)
{
	int i;
	struct anlogic_pinctrl *pctrl = pinctrl_dev_get_drvdata(pctldev);
	const struct anlogic_pctrl_group *pgrp = &pctrl->groups[group];
	const struct anlogic_pinmux_function *func = &pctrl->funcs[function];

	for (i = 0; i < pgrp->npins; i++) {
		unsigned int pin = pgrp->pins[i];
		u32 reg = readl(pctrl->iobase + ANLOGIC_PIN_MUX_OFFSET + (4 * pin));

		reg &= ~ANLOGIC_PINMUX_MUX_MASK;
		reg |= func->mux_val;

		writel(reg, pctrl->iobase + ANLOGIC_PIN_MUX_OFFSET + (4 * pin));
	}

	if (pgrp->emio_supported) {
		if (pgrp->npins)
			/* If npins is not zero, set the EMIO selection to 1, use MIO. */
			writel(1, pctrl->iobase + ANLOGIC_PIN_EMIO_SEL_OFFSET +
							(4 * pgrp->emio_sel_offset));
		else
			/* If npins is zero, set the EMIO selection to 0, use EMIO. */
			writel(0, pctrl->iobase + ANLOGIC_PIN_EMIO_SEL_OFFSET +
							(4 * pgrp->emio_sel_offset));
	}

	return 0;
}

static const struct pinmux_ops anlogic_pinmux_ops = {
	.get_functions_count = anlogic_pmux_get_functions_count,
	.get_function_name = anlogic_pmux_get_function_name,
	.get_function_groups = anlogic_pmux_get_function_groups,
	.set_mux = anlogic_pinmux_set_mux,
};

/* pinconfig */
static const struct pinconf_generic_params anlogic_dt_params[] = {
	{"pull-ohm", PIN_CONFIG_PULL_OHM, PULL_RESISTOR_25KOHM},
};

#ifdef CONFIG_DEBUG_FS
static const struct pin_config_item anlogic_conf_items[ARRAY_SIZE(anlogic_dt_params)]
	= { PCONFDUMP(PIN_CONFIG_PULL_OHM, "pull-ohm", NULL, true),
};
#endif

static int get_pull_resistor_value(u32 reg_l, u32 pull_flag,
			       u32 pull_ohm_flag, unsigned int *arg)
{
	if (!(reg_l & pull_flag))
		return -EINVAL;

	*arg = (reg_l & pull_ohm_flag) ? PULL_RESISTOR_10KOHM : PULL_RESISTOR_25KOHM;

	return 0;
}

static int anlogic_pinconf_cfg_get(struct pinctrl_dev *pctldev,
				unsigned int pin,
				unsigned long *config)
{
	u32 reg;
	unsigned int arg = 0;
	unsigned int param = pinconf_to_config_param(*config);
	struct anlogic_pinctrl *pctrl = pinctrl_dev_get_drvdata(pctldev);

	if (pin >= ANLOGIC_NUM_MIOS)
		return -ENOTSUPP;

	reg = readl(pctrl->iobase + ANLOGIC_PIN_CFG_LOW_OFFSET + (8 * pin));

	switch (param) {
	case PIN_CONFIG_BIAS_PULL_UP:
		if (get_pull_resistor_value(reg, ANLOGIC_PINCONF_PULLUP,
					ANLOGIC_PINCONF_PULLUP_OHM, &arg))
			return -EINVAL;
		break;
	case PIN_CONFIG_BIAS_PULL_DOWN:
		if (get_pull_resistor_value(reg, ANLOGIC_PINCONF_PULLDOWN,
					ANLOGIC_PINCONF_PULLDOWN_OHM, &arg))
			return -EINVAL;
		break;
	case PIN_CONFIG_BIAS_HIGH_IMPEDANCE:
		if (!(reg & ANLOGIC_PINCONF_PULLUP) || !(reg & ANLOGIC_PINCONF_PULLDOWN))
			return -EINVAL;
		arg = 1;
		break;
	case PIN_CONFIG_SLEW_RATE:
		arg = reg & ANLOGIC_PINCONF_SPEED;
		break;
	case PIN_CONFIG_PULL_OHM:
		/* Pull-up/down resistance value is already obtained in PIN_CONFIG_BIAS_PULL_UP. */
	default:
		return -ENOTSUPP;
	}

	*config = pinconf_to_config_packed(param, arg);

	return 0;
}

static int anlogic_pinconf_cfg_set(struct pinctrl_dev *pctldev,
				unsigned int pin,
				unsigned long *configs,
				unsigned int num_configs)
{
	int i;
	u32 reg;
	struct anlogic_pinctrl *pctrl = pinctrl_dev_get_drvdata(pctldev);

	if (pin >= ANLOGIC_NUM_MIOS)
		return -ENOTSUPP;

	reg = readl(pctrl->iobase + ANLOGIC_PIN_CFG_LOW_OFFSET + (8 * pin));

	for (i = 0; i < num_configs; i++) {
		unsigned int param = pinconf_to_config_param(configs[i]);
		unsigned int arg = pinconf_to_config_argument(configs[i]);

		switch (param) {
		case PIN_CONFIG_BIAS_PULL_UP:
			reg &= ~(ANLOGIC_PINCONF_PULLDOWN);
			reg |= ANLOGIC_PINCONF_PULLUP;
			break;
		case PIN_CONFIG_BIAS_PULL_DOWN:
			reg &= ~(ANLOGIC_PINCONF_PULLUP);
			reg |= ANLOGIC_PINCONF_PULLDOWN;
			break;
		case PIN_CONFIG_BIAS_HIGH_IMPEDANCE:
			reg &= ~(ANLOGIC_PINCONF_PULLUP_OHM | ANLOGIC_PINCONF_PULLDOWN_OHM);
			reg |= (ANLOGIC_PINCONF_PULLUP | ANLOGIC_PINCONF_PULLDOWN);
			break;
		case PIN_CONFIG_PULL_OHM:
			/* The resistance value of the other resistor has no impact. */
			if (arg == PULL_RESISTOR_10KOHM)
				reg |= (ANLOGIC_PINCONF_PULLUP_OHM | ANLOGIC_PINCONF_PULLDOWN_OHM);
			else
				reg &= ~(ANLOGIC_PINCONF_PULLUP_OHM | ANLOGIC_PINCONF_PULLDOWN_OHM);
			break;
		case PIN_CONFIG_SLEW_RATE:
			reg &= ~ANLOGIC_PINCONF_SPEED;
			reg |= arg;
			break;
		default:
			dev_warn(pctldev->dev,
				 "unsupported configuration parameter '%u'\n",
				 param);
			continue;
		}
	}

	writel(reg, pctrl->iobase + ANLOGIC_PIN_CFG_LOW_OFFSET + (8 * pin));

	return 0;
}

static int anlogic_pinconf_group_set(struct pinctrl_dev *pctldev,
				  unsigned int selector,
				  unsigned long *configs,
				  unsigned int  num_configs)
{
	int i, ret;
	struct anlogic_pinctrl *pctrl = pinctrl_dev_get_drvdata(pctldev);
	const struct anlogic_pctrl_group *pgrp = &pctrl->groups[selector];

	for (i = 0; i < pgrp->npins; i++) {
		ret = anlogic_pinconf_cfg_set(pctldev, pgrp->pins[i], configs,
					   num_configs);
		if (ret)
			return ret;
	}

	return 0;
}

static const struct pinconf_ops anlogic_pinconf_ops = {
	.is_generic = true,
	.pin_config_get = anlogic_pinconf_cfg_get,
	.pin_config_set = anlogic_pinconf_cfg_set,
	.pin_config_group_set = anlogic_pinconf_group_set,
};

static struct pinctrl_desc anlogic_desc = {
	.name = "anlogic_pinctrl",
	.pins = anlogic_pins,
	.npins = ARRAY_SIZE(anlogic_pins),
	.pctlops = &anlogic_pctrl_ops,
	.pmxops = &anlogic_pinmux_ops,
	.confops = &anlogic_pinconf_ops,
	.num_custom_params = ARRAY_SIZE(anlogic_dt_params),
	.custom_params = anlogic_dt_params,
#ifdef CONFIG_DEBUG_FS
	.custom_conf_items = anlogic_conf_items,
#endif
	.owner = THIS_MODULE,
};

static int anlogic_pinctrl_probe(struct platform_device *pdev)

{
	struct anlogic_pinctrl *pctrl;

	pctrl = devm_kzalloc(&pdev->dev, sizeof(*pctrl), GFP_KERNEL);
	if (!pctrl)
		return -ENOMEM;

	pctrl->iobase = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(pctrl->iobase))
		return PTR_ERR(pctrl->iobase);

	pctrl->groups = anlogic_pctrl_groups;
	pctrl->ngroups = ARRAY_SIZE(anlogic_pctrl_groups);
	pctrl->funcs = anlogic_pmux_functions;
	pctrl->nfuncs = ARRAY_SIZE(anlogic_pmux_functions);

	pctrl->pctrl = devm_pinctrl_register(&pdev->dev, &anlogic_desc, pctrl);
	if (IS_ERR(pctrl->pctrl))
		return PTR_ERR(pctrl->pctrl);

	platform_set_drvdata(pdev, pctrl);

	dev_info(&pdev->dev, "anlogic dr1x pinctrl initialized\n");

	return 0;
}

static const struct of_device_id anlogic_pinctrl_of_match[] = {
	{ .compatible = "anlogic,pinctrl-dr1x" },
	{ }
};

static struct platform_driver anlogic_pinctrl_driver = {
	.driver = {
		.name = "anlogic-pinctrl",
		.of_match_table = anlogic_pinctrl_of_match,
	},
	.probe = anlogic_pinctrl_probe,
};

module_platform_driver(anlogic_pinctrl_driver);

MODULE_AUTHOR("Naiyuan Que <naiyuan.que@anlogic.com>");
MODULE_DESCRIPTION("anlogic dr1x pin Controller Driver");
MODULE_LICENSE("GPL");
