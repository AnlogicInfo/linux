/* SPDX-License-Identifier: GPL-2.0+ */

#ifndef _UAPI_LINUX_ANLTOP_H
#define _UAPI_LINUX_ANLTOP_H

#include <linux/ioctl.h>
#include <linux/types.h>

#define ANLTOP_MAGIC		'T'
struct anltop_ioc_secreg_query {
	uint64_t base;
	uint64_t len;
};
#define ANLTOP_IOCSQUERY	_IOR(ANLTOP_MAGIC, 0, struct anltop_ioc_secreg_query)

#endif
