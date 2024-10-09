// SPDX-License-Identifier: GPL-2.0
/****************************************************************
 * Copyright (c) 2023 Anlogic, Inc.  All rights reserved.
 *
 ****************************************************************/

/***************************************************************
 *  @file This is hard npu driver using cdev driver.
 *  Last modified by
 *       30-10-2023   ritao.xiao
 ****************************************************************/

#include <linux/cdev.h>
#include <linux/io.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/platform_device.h>
#include <linux/types.h>
#include <linux/uaccess.h>
#include <linux/uio_driver.h>
#include <linux/kernel.h>
#include <linux/init.h>

#include "hard_npu_module.h"

static struct hard_npu_drv_buffer_t s_hard_npu_register;
#define ReadRegister(addr) readl((void __iomem *)(s_hard_npu_register.virt_addr + addr))
#define WriteRegister(addr, val)    \
	writel(val, (void __iomem *)(s_hard_npu_register.virt_addr + addr))

static int s_is_open;
static uint32_t s_intr_flag;
static int hard_npu_irq_num;
static uint32_t s_work_status;
static wait_queue_head_t s_intr_wait_q;
static int s_major;
static struct class *s_dev_class;
static struct device *s_devp;
static struct device *hardnpu_devp;
static struct cdev s_cdev;

#define CDEV_DEV_NAME "hard_npu"

#define ENABLE_REGISTER_ADDRESS 0x000001B8
#define SOFT_RESET_REGISTER_ADDRESS 0x00000074
#define SRAM_ENABLE_REGISTER_ADDRESS 0x00000030

static irqreturn_t my_hard_npu_interrupt(int irq, void *dev_id)
{
	s_intr_flag = ReadRegister(0x1018);
	s_work_status = ReadRegister(0x1014);
	WriteRegister(0x101c, s_intr_flag);
	wake_up_interruptible(&s_intr_wait_q);
	return IRQ_RETVAL(IRQ_HANDLED);
}

static int hard_npu_open(struct inode *inode, struct file *filp)
{
	s_intr_flag = 0;
	if (s_is_open == 0) {
		s_is_open = 1;
		return 0;
	}

	dev_err(hardnpu_devp, "hard_npu is obtained by another instance!\n");
	return -1;
}

static int hard_npu_release(struct inode *inode, struct file *filp)
{
	if (s_is_open == 0) {
		dev_err(hardnpu_devp, "hard_npu is not opened!\n");
		return -1;
	}

	s_is_open = 0;
	return 0;
}

static ssize_t hard_npu_read(struct file *filp, char __user *buf, size_t len, loff_t *ppos)
{
	dev_err(hardnpu_devp, "Not supported\n");
	return -1;
}

static ssize_t hard_npu_write(struct file *filp, const char __user *buf, size_t len, loff_t *ppos)
{
	dev_err(hardnpu_devp, "Not supported\n");
	return -1;
}

static long hard_npu_ioctl(struct file *filp, u_int cmd, u_long arg)
{
	long ret = 0;
	struct hardnpu_reg_info_t info;
	struct hardnpu_rw_info_t rw;
	struct hardnpu_intr_info_t intr;

	switch (cmd) {
	case HARDNPU_IOCTL_WAIT_INTERRUPT:
		// printk("HARDNPU_IOCTL_WAIT_INTERRUPT\n");
		ret = copy_from_user(&intr, (void __user *)arg, sizeof(struct hardnpu_intr_info_t));
		if (ret != 0) {
			ret = -EFAULT;
			dev_err(hardnpu_devp, "copy_from_user failed\n");
			break;
		}
		ret = wait_event_interruptible_timeout(s_intr_wait_q, s_intr_flag != 0,
				 msecs_to_jiffies(intr.timeout));
		if (!ret) {
			dev_err(hardnpu_devp, "hardnpu intr timeout!\n");
			ret = -ETIME;
			break;
		}
		if (signal_pending(current)) {
			ret = -ERESTARTSYS;
			dev_info(hardnpu_devp, "[HARD NPU] ERESTARTSYS\n");
			break;
		}
		// printk("s_intr_flag = %x\n", s_intr_flag);
		intr.intr_flag = s_intr_flag;
		intr.work_status = s_work_status;
		s_intr_flag = 0;
		ret = copy_to_user((void __user *)arg, &intr, sizeof(struct hardnpu_intr_info_t));
		if (ret != 0) {
			dev_err(hardnpu_devp, "copy_to_user failed\n");
			ret = -EFAULT;
			break;
		}
		break;
	case HARDNPU_IOCTL_GET_REGISTER_INFO:
		info.addr = s_hard_npu_register.phys_addr;
		info.size = s_hard_npu_register.size;
		ret = copy_to_user((void __user *)arg, &info, sizeof(struct hardnpu_reg_info_t));
		if (ret != 0) {
			dev_err(hardnpu_devp, "copy_to_user failed\n");
			ret = -EFAULT;
			break;
		}
		break;
	case HARDNPU_IOCTL_WRITE_REG:
		ret = copy_from_user(&rw, (void __user *)arg, sizeof(struct hardnpu_rw_info_t));
		if (ret != 0) {
			dev_err(hardnpu_devp, "copy_from_user failed\n");
			ret = -EFAULT;
			break;
		}
		WriteRegister(rw.addr, rw.data);
		break;
	case HARDNPU_IOCTL_READ_REG:
		ret = copy_from_user(&rw, (void __user *)arg, sizeof(struct hardnpu_rw_info_t));
		if (ret != 0) {
			dev_err(hardnpu_devp, "copy_from_user failed\n");
			ret = -EFAULT;
			break;
		}
		rw.data = ReadRegister(rw.addr);
		ret = copy_to_user((void __user *)arg, &rw, sizeof(struct hardnpu_rw_info_t));
		if (ret != 0) {
			dev_err(hardnpu_devp, "copy_to_user failed\n");
			ret = -EFAULT;
			break;
		}
		break;
	default:
		dev_err(hardnpu_devp, "No supported cmd!\n");
		break;
	}
	return ret;
}

static int hard_npu_mmap(struct file *fp, struct vm_area_struct *vm)
{
	if (vm->vm_pgoff != (s_hard_npu_register.phys_addr >> 12U)) {
		dev_err(hardnpu_devp, "No supported\n");
		return -EFAULT;
	}
	vm->vm_flags |= VM_IO | VM_DONTEXPAND | VM_DONTDUMP;
	vm->vm_page_prot = pgprot_noncached(vm->vm_page_prot);
	dev_info(hardnpu_devp, "map_to_register: phys_addr: %lx\n", s_hard_npu_register.phys_addr);
	return remap_pfn_range(vm, vm->vm_start, vm->vm_pgoff, vm->vm_end - vm->vm_start,
			 vm->vm_page_prot) ? -EAGAIN : 0;
}

const struct file_operations hard_npu_fops = {
	.owner = THIS_MODULE,
	.open = hard_npu_open,
	.release = hard_npu_release,
	.read = hard_npu_read,
	.write = hard_npu_write,
	.unlocked_ioctl = hard_npu_ioctl,
	.mmap = hard_npu_mmap,
};

static int drv_hard_npu_probe(struct platform_device *pdev);
static int drv_hard_npu_remove(struct platform_device *pdev);
static const struct of_device_id hard_npu_match_table[] = {
	{.compatible = "anlogic,hard_npu"},
};

static struct platform_driver hard_npu_driver = {.probe = drv_hard_npu_probe,
						 .remove = drv_hard_npu_remove,
						 .driver = {
							 .name = "hard_npu",
							 .owner = THIS_MODULE,
							 .of_match_table = hard_npu_match_table,
						 }};

static int drv_hard_npu_probe(struct platform_device *pdev)
{
	dev_info(&(pdev->dev), "%s( %p)\n", __func__, pdev);
	hardnpu_devp = &(pdev->dev);

	hard_npu_irq_num = platform_get_irq(pdev, 0);

	if (hard_npu_irq_num < 0) {
		dev_err(hardnpu_devp, "get irq resource failed in hard_npu device\n");
		return -EINVAL;
	}

	dev_info(hardnpu_devp, "find irq num: %d\n", hard_npu_irq_num);

	struct resource *mem_src_p = platform_get_resource(pdev, IORESOURCE_MEM, 0);

	if (mem_src_p == NULL) {
		dev_err(hardnpu_devp, "get mem resource failed in hard_npu device\n");
		return -EINVAL;
	}

	s_hard_npu_register.virt_addr = devm_ioremap_resource(&pdev->dev, mem_src_p);
	dev_info(hardnpu_devp, "devm_ioremap_resource return %p\n", s_hard_npu_register.virt_addr);

	s_hard_npu_register.phys_addr = mem_src_p->start;
	dev_info(hardnpu_devp, "mem start addr: %lx\n", s_hard_npu_register.phys_addr);

	s_hard_npu_register.size = (mem_src_p->end - mem_src_p->start + 1);
	dev_info(hardnpu_devp, "mem size: %d\n", s_hard_npu_register.size);

	/* get the major number of the character device */
	if ((alloc_chrdev_region(&s_major, 0, 1, CDEV_DEV_NAME)) < 0) {
		dev_err(hardnpu_devp, "could not allocate major number\n");
		return -EBUSY;
	}

	// get top config mem resource
	struct resource *top_mem_src_p = platform_get_resource(pdev, IORESOURCE_MEM, 1);

	if (top_mem_src_p == NULL) {
		dev_err(hardnpu_devp, "get top config mem resource failed in hard_npu device\n");
		return -EINVAL;
	}

	// enable hardnpu
	void __iomem *enable_register_ptr = ioremap((top_mem_src_p->start +
			ENABLE_REGISTER_ADDRESS), sizeof(u32));

	if (!enable_register_ptr) {
		dev_err(hardnpu_devp, "Failed to map enable register address for hardnpu\n");
		return -ENOMEM;
	}

	u32 tmp_val = 0x3;

	iowrite32(tmp_val, enable_register_ptr);
	iounmap(enable_register_ptr);

	// enable sram
	void __iomem *sram_register_ptr = ioremap((top_mem_src_p->start +
			SRAM_ENABLE_REGISTER_ADDRESS), sizeof(u32));

	if (!sram_register_ptr) {
		dev_err(hardnpu_devp, "Failed to map sram wakeup register address for hardnpu\n");
		return -ENOMEM;
	}

	tmp_val = ioread32(sram_register_ptr);
	tmp_val &= 0xC0FFFFFF;
	iowrite32(tmp_val, sram_register_ptr);
	iounmap(sram_register_ptr);

	// get crp config mem resource
	struct resource *crp_mem_src_p = platform_get_resource(pdev, IORESOURCE_MEM, 2);

	if (crp_mem_src_p == NULL) {
		dev_err(hardnpu_devp, "get crp config mem resource failed in hard_npu device\n");
		return -EINVAL;
	}

	// soft reset
	void __iomem *soft_reset_register_ptr = ioremap((crp_mem_src_p->start +
			SOFT_RESET_REGISTER_ADDRESS), sizeof(u32));

	if (!soft_reset_register_ptr) {
		dev_err(hardnpu_devp, "Failed to map soft reset register address for hardnpu\n");
		return -ENOMEM;
	}

	tmp_val = ioread32(soft_reset_register_ptr);
	tmp_val &= 0xFFFFEFFF;
	iowrite32(tmp_val, soft_reset_register_ptr);
	tmp_val = ioread32(soft_reset_register_ptr);
	tmp_val |= 0x00001000;
	iowrite32(tmp_val, soft_reset_register_ptr);
	iounmap(soft_reset_register_ptr);

	/* initialize the device structure and register the device with the kernel */
	cdev_init(&s_cdev, &hard_npu_fops);
	s_cdev.owner = THIS_MODULE;
	if ((cdev_add(&s_cdev, s_major, 1)) < 0) {
		dev_err(hardnpu_devp, "could not allocate chrdev\n");
		unregister_chrdev_region(s_major, 1);
		return -EBUSY;
	}

	// request handler
	int err = request_irq(hard_npu_irq_num, my_hard_npu_interrupt, IRQF_TRIGGER_HIGH,
			 "HARD_NPU_IRQ", (void *)NULL);

	if (err) {
		dev_err(hardnpu_devp, "fail to register interrupt handler\n");
		cdev_del(&s_cdev);
		unregister_chrdev_region(s_major, 1);
		return err;
	}

	// create device class
	s_dev_class = class_create(THIS_MODULE, CDEV_DEV_NAME);
	if (IS_ERR(s_dev_class)) {
		dev_err(hardnpu_devp, "fail to create device class\n");
		err = PTR_ERR(s_dev_class);
		free_irq(hard_npu_irq_num, (void *)NULL);
		cdev_del(&s_cdev);
		unregister_chrdev_region(s_major, 1);
		return err;
	}
	// create device
	s_devp = device_create(s_dev_class, NULL, s_major, NULL, "%s", CDEV_DEV_NAME);
	if (IS_ERR(s_devp)) {
		dev_err(hardnpu_devp, "fail to create device\n");
		err = PTR_ERR(s_devp);
		class_destroy(s_dev_class);
		free_irq(hard_npu_irq_num, (void *)NULL);
		cdev_del(&s_cdev);
		unregister_chrdev_region(s_major, 1);
		return err;
	}
	return 0;
}

static int drv_hard_npu_remove(struct platform_device *pdev)
{
	dev_info(hardnpu_devp, "%s( %p)\n", __func__, pdev);
	device_destroy(s_dev_class, s_major);
	class_destroy(s_dev_class);
	free_irq(hard_npu_irq_num, (void *)NULL);
	cdev_del(&s_cdev);
	unregister_chrdev_region(s_major, 1);

	if (s_hard_npu_register.virt_addr && 0) // auto release by linux
		iounmap((void *)s_hard_npu_register.virt_addr);

	return 0;
}

static int __init hard_npu_init(void)
{
	init_waitqueue_head(&s_intr_wait_q);
	return platform_driver_register(&hard_npu_driver);
}

static void __exit hard_npu_exit(void)
{
	platform_driver_unregister(&hard_npu_driver);
}

module_init(hard_npu_init);
module_exit(hard_npu_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("hard_npu driver");
