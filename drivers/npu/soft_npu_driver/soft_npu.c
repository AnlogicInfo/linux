// SPDX-License-Identifier: GPL-2.0
/****************************************************************
 * Copyright (c) 2023 - 2024 Anlogic, Inc.  All rights reserved.
 *
 ****************************************************************/

/***************************************************************
 *  @file This is soft npu driver using cdev driver.
 *  Last modified by
 *       30-01-2024   ritao.xiao
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

#include "soft_npu_module.h"

#define SOFTNPU__TOP__DEVICE_INTERRUPT_STATUS__ADDR (0x0000 + 0x8)  // ro
#define SOFTNPU__SOFT_NN__CLEAR_INTERRUPT__ADDR (0x1800 + 0xbc)	    // whp
#define SOFTNPU__SOFT_YOLO__CLEAR_INTERRUPT__ADDR (0x2000 + 0x40)   // whp
#define SOFTNPU__SOFT_RESIZE__CLEAR_INTERRUPT__ADDR (0x3000 + 0x44) // whp
#define SOFTNPU__SOFT_CSC__CLEAR_INTERRUPT__ADDR (0x2800 + 0x6c)    // whp

#define CDEV_DEV_NAME "soft_npu"

#define DMA_START0_REGISTER_ADDRESS_OFFSET 0x000000804
#define DMA_START1_REGISTER_ADDRESS_OFFSET 0x000001004

static struct soft_npu_drv_buffer_t s_soft_npu_register;
#define ReadRegister(addr) readl((void __iomem *)(s_soft_npu_register.virt_addr + addr))
#define WriteRegister(addr, val)          \
	writel(val, (void __iomem *)(s_soft_npu_register.virt_addr + addr))

static uint32_t nn_yolo_intr_flag;
static uint32_t csc_intr_flag;
static uint32_t resize_intr_flag;

static wait_queue_head_t nn_yolo_intr_wait_q;
static wait_queue_head_t csc_intr_wait_q;
static wait_queue_head_t resize_intr_wait_q;

static int s_major;
static struct class *s_dev_class;
static struct device *s_devp;
static struct device *softnpu_devp;
static struct cdev s_cdev;
static int soft_nn_irq_num;
static int s_is_open;

static DEFINE_SPINLOCK(lock);

static irqreturn_t my_soft_npu_interrupt(int irq, void *dev_id)
{
	spin_lock(&lock);

	uint32_t intr_flag = ReadRegister(SOFTNPU__TOP__DEVICE_INTERRUPT_STATUS__ADDR);

	if (intr_flag & (1U << 2U))
		WriteRegister(SOFTNPU__SOFT_NN__CLEAR_INTERRUPT__ADDR, 0x1U);
	if (intr_flag & (1U << 3U))
		WriteRegister(SOFTNPU__SOFT_YOLO__CLEAR_INTERRUPT__ADDR, 0x1U);
	if (intr_flag & (1U << 4U))
		WriteRegister(SOFTNPU__SOFT_CSC__CLEAR_INTERRUPT__ADDR, 0x1U);
	if (intr_flag & (1U << 5U))
		WriteRegister(SOFTNPU__SOFT_RESIZE__CLEAR_INTERRUPT__ADDR, 0x1U);
	spin_unlock(&lock);

	if ((intr_flag & (0b1111U << 2U)) == 0)
		dev_err(softnpu_devp, "[SOFT NPU]abnormal interrupt been triged,system may crash soon");

	// dev_err(softnpu_devp, "intr_flag=0x%x\r\n", intr_flag);

	if (intr_flag & ((1U << 2U) | (1U << 3U))) {
		nn_yolo_intr_flag = intr_flag;
		wake_up_interruptible(&nn_yolo_intr_wait_q);
	}
	if (intr_flag & (1U << 4U)) {
		csc_intr_flag = intr_flag;
		wake_up_interruptible(&csc_intr_wait_q);
	}
	if (intr_flag & (1U << 5U)) {
		resize_intr_flag = intr_flag;
		wake_up_interruptible(&resize_intr_wait_q);
	}

	// dev_err(softnpu_devp, "intr_flag=0x%x\r\n",
			// ReadRegister(SOFTNPU__TOP__DEVICE_INTERRUPT_STATUS__ADDR));
	return IRQ_RETVAL(IRQ_HANDLED);
}

static int soft_npu_open(struct inode *inode, struct file *filp)
{
	if (s_is_open == 0) {
		nn_yolo_intr_flag = 0;
		csc_intr_flag = 0;
		resize_intr_flag = 0;
		s_is_open = 1;
		return 0;
	}

	dev_err(softnpu_devp, "soft_npu is obtained by another instance!\n");
	return -1;
}
static int soft_npu_release(struct inode *inode, struct file *filp)
{
	if (s_is_open == 0) {
		dev_err(softnpu_devp, "soft_npu is not opened!\n");
		return -1;
	}

	s_is_open = 0;
	return 0;
}
static ssize_t soft_npu_read(struct file *filp, char __user *buf, size_t len, loff_t *ppos)
{
	dev_err(softnpu_devp, "Not supported\n");
	return -1;
}
static ssize_t soft_npu_write(struct file *filp, const char __user *buf, size_t len, loff_t *ppos)
{
	dev_err(softnpu_devp, "Not supported\n");
	return -1;
}

static long soft_npu_ioctl(struct file *filp, u_int cmd, u_long arg)
{
	long ret = 0;
	struct softnpu_reg_info_t info;
	struct softnpu_rw_info_t rw;
	struct softnpu_intr_info_t intr;
	wait_queue_head_t *s_intr_wait_q;
	uint32_t *s_intr_flag;

	switch (cmd) {
	case SOFTNPU_IOCTL_WAIT_INTERRUPT:
		// dev_info(softnpu_devp, "SOFTNPU_IOCTL_WAIT_INTERRUPT\n");
		ret = copy_from_user(&intr, (void __user *)arg, sizeof(struct softnpu_intr_info_t));
		if (ret != 0) {
			ret = -EFAULT;
			dev_err(softnpu_devp, "copy_from_user failed\n");
			break;
		}

		if (intr.intr_flag & ((1U << 2U) | (1U << 3U))) {
			s_intr_flag = &nn_yolo_intr_flag;
			s_intr_wait_q = (&nn_yolo_intr_wait_q);
		} else if (intr.intr_flag & (1U << 4U)) {
			s_intr_flag = &csc_intr_flag;
			s_intr_wait_q = (&csc_intr_wait_q);
		} else if (intr.intr_flag & (1U << 5U)) {
			s_intr_flag = &resize_intr_flag;
			s_intr_wait_q = (&resize_intr_wait_q);
		} else {
			ret = -EFAULT;
			dev_err(softnpu_devp, "no valid desirous intr_flag=0x%x\n", intr.intr_flag);
			break;
		}

		ret = wait_event_interruptible_timeout(*s_intr_wait_q, *s_intr_flag,
				msecs_to_jiffies(intr.timeout));
		if (!ret) {
			dev_err(softnpu_devp, "softnpu intr timeout!\n");
			ret = -ETIME;
			break;
		}
		if (signal_pending(current)) {
			ret = -ERESTARTSYS;
			dev_info(softnpu_devp, "[SOFT NPU] ERESTARTSYS\n");
			break;
		}
		// dev_info(softnpu_devp, "s_intr_flag = %x\n", s_intr_flag);
		intr.intr_flag = *s_intr_flag;
		*s_intr_flag = 0;
		ret = copy_to_user((void __user *)arg, &intr, sizeof(struct softnpu_intr_info_t));
		if (ret != 0) {
			dev_err(softnpu_devp, "copy_to_user failed\n");
			ret = -EFAULT;
			break;
		}
		break;
	case SOFTNPU_IOCTL_GET_REGISTER_INFO:
		info.addr = s_soft_npu_register.phys_addr;
		info.size = s_soft_npu_register.size;
		ret = copy_to_user((void __user *)arg, &info, sizeof(struct softnpu_reg_info_t));
		if (ret != 0) {
			dev_err(softnpu_devp, "copy_to_user failed\n");
			ret = -EFAULT;
			break;
		}
		break;
	case SOFTNPU_IOCTL_WRITE_REG:
		ret = copy_from_user(&rw, (void __user *)arg, sizeof(struct softnpu_rw_info_t));
		if (ret != 0) {
			dev_err(softnpu_devp, "copy_from_user failed\n");
			ret = -EFAULT;
			break;
		}
		WriteRegister(rw.addr, rw.data);
		break;
	case SOFTNPU_IOCTL_READ_REG:
		ret = copy_from_user(&rw, (void __user *)arg, sizeof(struct softnpu_rw_info_t));
		if (ret != 0) {
			dev_err(softnpu_devp, "copy_from_user failed\n");
			ret = -EFAULT;
			break;
		}
		rw.data = ReadRegister(rw.addr);
		ret = copy_to_user((void __user *)arg, &rw, sizeof(struct softnpu_rw_info_t));
		if (ret != 0) {
			dev_err(softnpu_devp, "copy_to_user failed\n");
			ret = -EFAULT;
			break;
		}
		break;
	default:
		dev_err(softnpu_devp, "No supported cmd!\n");
		break;
	}
	return ret;
}
static int soft_npu_mmap(struct file *fp, struct vm_area_struct *vm)
{
	if (vm->vm_pgoff != (s_soft_npu_register.phys_addr >> 12U)) {
		dev_err(softnpu_devp, "No supported\n");
		return -EFAULT;
	}
	vm->vm_flags |= VM_IO | VM_DONTEXPAND | VM_DONTDUMP;
	vm->vm_page_prot = pgprot_noncached(vm->vm_page_prot);
	dev_info(softnpu_devp, "map_to_register: phys_addr: %lx\n", s_soft_npu_register.phys_addr);
	return remap_pfn_range(vm, vm->vm_start, vm->vm_pgoff, vm->vm_end - vm->vm_start,
			vm->vm_page_prot) ? -EAGAIN : 0;
};

const struct file_operations soft_npu_fops = {
	.owner = THIS_MODULE,
	.open = soft_npu_open,
	.release = soft_npu_release,
	.read = soft_npu_read,
	.write = soft_npu_write,
	.unlocked_ioctl = soft_npu_ioctl,
	.mmap = soft_npu_mmap,
};

static int drv_soft_npu_probe(struct platform_device *pdev);
static int drv_soft_npu_remove(struct platform_device *pdev);
static const struct of_device_id soft_npu_match_table[] = {
	{.compatible = "anlogic,soft_npu"},
};

static struct platform_driver soft_npu_driver = {.probe = drv_soft_npu_probe,
						 .remove = drv_soft_npu_remove,
						 .driver = {
							 .name = "soft_npu",
							 .owner = THIS_MODULE,
							 .of_match_table = soft_npu_match_table,
						 }};

static int drv_soft_npu_probe(struct platform_device *pdev)
{
	dev_info(&(pdev->dev), "%s( %p)\n", __func__, pdev);

	softnpu_devp = &(pdev->dev);

	soft_nn_irq_num = platform_get_irq(pdev, 0);

	if (soft_nn_irq_num < 0) {
		dev_err(softnpu_devp, "get irq resource failed in soft_npu device\n");
		return -EINVAL;
	}

	dev_info(softnpu_devp, "find irq num: %d\n", soft_nn_irq_num);

	struct resource *mem_src_p = platform_get_resource(pdev, IORESOURCE_MEM, 0);

	if (mem_src_p == NULL) {
		dev_err(softnpu_devp, "get mem resource failed in soft_npu device\n");
		return -EINVAL;
	}
	s_soft_npu_register.virt_addr = devm_ioremap_resource(&pdev->dev, mem_src_p);
	dev_info(softnpu_devp, "devm_ioremap_resource return %p\n", s_soft_npu_register.virt_addr);

	WriteRegister(DMA_START0_REGISTER_ADDRESS_OFFSET, 0x1);
	WriteRegister(DMA_START1_REGISTER_ADDRESS_OFFSET, 0x1);

	s_soft_npu_register.phys_addr = mem_src_p->start;
	dev_info(softnpu_devp, "mem start addr: %lx\n", s_soft_npu_register.phys_addr);

	s_soft_npu_register.size = (mem_src_p->end - mem_src_p->start + 1);
	dev_info(softnpu_devp, "mem size: %u\n", s_soft_npu_register.size);

	int err = 0;
	/* get the major number of the character device */
	if ((alloc_chrdev_region(&s_major, 0, 1, CDEV_DEV_NAME)) < 0) {
		dev_err(softnpu_devp, "could not allocate major number\n");
		return -EBUSY;
	}

	/* initialize the device structure and register the device with the kernel */
	cdev_init(&s_cdev, &soft_npu_fops);
	s_cdev.owner = THIS_MODULE;
	if ((cdev_add(&s_cdev, s_major, 1)) < 0) {
		dev_err(softnpu_devp, "could not allocate chrdev\n");
		unregister_chrdev_region(s_major, 1);
		return -EBUSY;
	}

	// request handler
	err = request_irq(soft_nn_irq_num, my_soft_npu_interrupt, IRQF_TRIGGER_HIGH, "SOFT_NPU_IRQ",
			 (void *)NULL);
	if (err) {
		dev_err(softnpu_devp, "fail to register interrupt handler\n");
		cdev_del(&s_cdev);
		unregister_chrdev_region(s_major, 1);
		return err;
	}

	// create device class
	s_dev_class = class_create(THIS_MODULE, CDEV_DEV_NAME);
	if (IS_ERR(s_dev_class)) {
		dev_err(softnpu_devp, "fail to create device class\n");
		err = PTR_ERR(s_dev_class);
		free_irq(soft_nn_irq_num, (void *)NULL);
		cdev_del(&s_cdev);
		unregister_chrdev_region(s_major, 1);
		return err;
	}
	// create device
	s_devp = device_create(s_dev_class, NULL, s_major, NULL, "%s", CDEV_DEV_NAME);
	if (IS_ERR(s_devp)) {
		dev_err(softnpu_devp, "fail to create device\n");
		err = PTR_ERR(s_devp);
		class_destroy(s_dev_class);
		free_irq(soft_nn_irq_num, (void *)NULL);
		cdev_del(&s_cdev);
		unregister_chrdev_region(s_major, 1);
		return err;
	}
	return 0;
}

static int drv_soft_npu_remove(struct platform_device *pdev)
{
	dev_info(softnpu_devp, "%s( %p)\n", __func__, pdev);
	device_destroy(s_dev_class, s_major);
	class_destroy(s_dev_class);
	free_irq(soft_nn_irq_num, (void *)NULL);
	cdev_del(&s_cdev);
	unregister_chrdev_region(s_major, 1);

	if (s_soft_npu_register.virt_addr && 0) // auto release by linux
		iounmap((void *)s_soft_npu_register.virt_addr);

	return 0;
}

static int __init soft_npu_init(void)
{
	init_waitqueue_head(&nn_yolo_intr_wait_q);
	init_waitqueue_head(&csc_intr_wait_q);
	init_waitqueue_head(&resize_intr_wait_q);
	return platform_driver_register(&soft_npu_driver);
}

static void __exit soft_npu_exit(void)
{
	platform_driver_unregister(&soft_npu_driver);
}

module_init(soft_npu_init);
module_exit(soft_npu_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("soft_npu driver");
