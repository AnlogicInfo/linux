// SPDX-License-Identifier: GPL-2.0+
/*
 * include/linux/scck9007_phy.h
 *
 * scck9007 PHY IDs
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 *
 */

#ifndef _SCCK9007_PHY_H
#define _SCCK9007_PHY_H

#define SCCK9007_PHY_ID_MASK	0x00000fff


#define PHY_ID_SCCK9007		0x0000011a

#define REG_PHY_SPEC_STATUS		0x11
#define REG_DEBUG_ADDR_OFFSET		0x1e
#define REG_DEBUG_DATA			0x1f




#define SCCK9007_EXTREG_SLEEP_CONTROL1	0x27
#define SCCK9007_EN_SLEEP_SW_BIT		15

#define SCCK9007_SPEED_MODE		0xc000
#define SCCK9007_DUPLEX			0x2000
#define SCCK9007_SPEED_MODE_BIT		14
#define SCCK9007_DUPLEX_BIT		13
#define SCCK9007_LINK_STATUS_BIT		10

#endif /* _SCCK9007_PHY_H */

