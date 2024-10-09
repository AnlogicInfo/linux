/* SPDX-License-Identifier: GPL-2.0 */
/****************************************************************
 * Copyright (c) 2023 Anlogic, Inc.  All rights reserved.
 *
 ****************************************************************/

#ifdef __linux__
#ifndef __HARD_NPU_MODULE_H__
#define __HARD_NPU_MODULE_H__
#include <linux/fs.h>
#include <linux/types.h>
#ifndef __KERNEL__
#include <stdint.h>
#endif // !__KERNEL__

#define HARDNPU_IOCTL_MAGIC 'H'

#define HARDNPU_IOCTL_WAIT_INTERRUPT _IO(HARDNPU_IOCTL_MAGIC, 0)
#define HARDNPU_IOCTL_GET_REGISTER_INFO _IO(HARDNPU_IOCTL_MAGIC, 1)
#define HARDNPU_IOCTL_WRITE_REG _IO(HARDNPU_IOCTL_MAGIC, 2)
#define HARDNPU_IOCTL_READ_REG _IO(HARDNPU_IOCTL_MAGIC, 3)

struct hardnpu_intr_info_t {
	uint32_t timeout; // unit: ms
	uint32_t intr_flag;
	uint32_t work_status;
};

struct hardnpu_rw_info_t {
	uint32_t addr;
	uint32_t data;
};

struct hardnpu_reg_info_t {
	uint64_t addr;
	uint64_t size;
};

struct hard_npu_drv_buffer_t {
	unsigned int size;
	unsigned long phys_addr;
	void *virt_addr; /* virtual kernel space address */
};

#endif // !__HARD_NPU_MODULE_H__
#endif
