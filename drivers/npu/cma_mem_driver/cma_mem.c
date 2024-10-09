// SPDX-License-Identifier: GPL-2.0
/****************************************************************
 * Copyright (c) 2023 - 2024 Anlogic, Inc.  All rights reserved.
 *
 ****************************************************************/

/***************************************************************
 *  @file This is cma driver.
 *  Last modified by
 *       30-01-2024   ritao.xiao
 ****************************************************************/

#include "linux/dev_printk.h"
#include "linux/nodemask.h"
#include "linux/rbtree.h"
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/dma-mapping.h>
#include <linux/err.h>
#include <linux/errno.h>
#include <linux/fs.h>
#include <linux/gpio.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/miscdevice.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/types.h>
#include <linux/uaccess.h>

#define IOCTL_MAGIC 'C'
#define CMA_MEM_WRITE _IO(IOCTL_MAGIC, 0)
#define CMA_MEM_SYNC_FOR_CPU _IO(IOCTL_MAGIC, 1)
#define CMA_MEM_SYNC_FOR_DEVICE _IO(IOCTL_MAGIC, 2)
#define CMA_MEM_ALLOCATE _IO(IOCTL_MAGIC, 3)
#define CMA_MEM_FREE _IO(IOCTL_MAGIC, 4)

struct cma_write_info_t {
	uint64_t phy_addr;
	uint64_t offset;
	void *buf;
	uint64_t len;
};

struct cma_sync_range_t {
	uint64_t phy_addr;
	uint64_t offset;
	uint64_t len;
};

struct cma_buffer_t {
	uint32_t size;	//
	uint32_t flags; // cache flags 0:cache_shared,1:writecombine,2:nocached
	phys_addr_t phys_addr;
	void *base;	 /* kernel logical address in use kernel */
	void *virt_addr; /* virtual user space address */
};

struct cma_buffer_node_t {
	struct rb_node node;
	struct cma_buffer_t buffer; // the key is buffer->phys_addr
};

static struct list_head s_buffer_pool_list_head = LIST_HEAD_INIT(s_buffer_pool_list_head);

static spinlock_t s_cma_lock = __SPIN_LOCK_UNLOCKED(s_cma_lock);
static DEFINE_SEMAPHORE(s_cma_sem);
static struct device *cma_dev;

struct cma_buffer_node_t *bf_node_search(struct rb_root *root, phys_addr_t phys_addr)
{
	struct rb_node *node = root->rb_node;

	while (node) {
		struct cma_buffer_node_t *bf_node = container_of(node, struct cma_buffer_node_t,
				node);

		if (phys_addr < bf_node->buffer.phys_addr)
			node = node->rb_left;
		else if (phys_addr > bf_node->buffer.phys_addr)
			node = node->rb_right;
		else
			return bf_node;
	}
	return NULL;
}

struct cma_buffer_node_t *bf_node_create(uint32_t size)
{
	struct cma_buffer_node_t *rtv = kzalloc(sizeof(struct cma_buffer_node_t), GFP_KERNEL);
	// cma_dev->coherent_dma_mask = ~0;
	rtv->buffer.size = PAGE_ALIGN(size);

	rtv->buffer.base = dma_alloc_coherent(cma_dev, rtv->buffer.size,
			&rtv->buffer.phys_addr, GFP_KERNEL);

	if (rtv->buffer.base == NULL) {
		_dev_err(cma_dev, "Physical memory allocation error with size=%u\n",
				rtv->buffer.size);
		kfree(rtv);
		rtv = NULL;
	}

	// _dev_info(cma_dev, "Alloc 0x%llx", rtv->buffer.phys_addr);

	return rtv;
}

bool bf_node_insert(struct rb_root *root, struct cma_buffer_node_t *bf_node)
{
	struct rb_node **new = &(root->rb_node), *parent = NULL;

	/* Figure out where to put new node */
	while (*new) {
		struct cma_buffer_node_t *this = container_of(*new, struct cma_buffer_node_t, node);

		parent = *new;
		if (bf_node->buffer.phys_addr < this->buffer.phys_addr)
			new = &((*new)->rb_left);
		else if (bf_node->buffer.phys_addr > this->buffer.phys_addr)
			new = &((*new)->rb_right);
		else
			return false;
	}
	/* Add new node and rebalance tree. */
	// _dev_info(cma_dev, "insert 0x%llx", bf_node->buffer.phys_addr);
	rb_link_node(&bf_node->node, parent, new);
	rb_insert_color(&bf_node->node, root);
	return true;
}

void bf_node_earse(struct rb_root *root, struct cma_buffer_node_t *bf_node)
{
	if (bf_node) {
		// _dev_info(cma_dev, "Free 0x%llx", bf_node->buffer.phys_addr);

		dma_free_coherent(cma_dev, bf_node->buffer.size, bf_node->buffer.base,
				bf_node->buffer.phys_addr);
		rb_erase(&bf_node->node, root);
		kfree(bf_node);
	}
}

void bf_node_clear(struct rb_root *root)
{
	struct rb_node **new = &(root->rb_node);

	while (*new) {
		struct cma_buffer_node_t *this = container_of(*new, struct cma_buffer_node_t, node);

		bf_node_earse(root, this);
	}
}

struct cma_buffer_pool_list_node_t {
	struct list_head list;
	struct file *filp;	     // filp->private_data point to this list
	struct rb_root buf_pool_rbt; // = RB_ROOT;
	uint64_t mem_sum;	     // total memory size used by this fd
};

#ifndef VM_RESERVED /*for kernel up to 3.7.0 version*/
#define VM_RESERVED (VM_DONTEXPAND | VM_DONTDUMP)
#endif

static int cma_mem_open(struct inode *inode, struct file *filp)
{
	spin_lock(&s_cma_lock);
	struct cma_buffer_pool_list_node_t *list_node = (struct cma_buffer_pool_list_node_t *)
			kzalloc(sizeof(struct cma_buffer_pool_list_node_t), GFP_KERNEL);
	list_node->filp = filp;
	filp->private_data = list_node;
	list_node->buf_pool_rbt = RB_ROOT;
	list_add(&list_node->list, &s_buffer_pool_list_head);
	spin_unlock(&s_cma_lock);
	return 0;
}

static int cma_mem_release(struct inode *inode, struct file *filp)
{
	int ret = down_interruptible(&s_cma_sem);

	if (ret == 0) {
		struct cma_buffer_pool_list_node_t *list_node = filp->private_data;
		// _dev_info(cma_dev, "bf_node_clear");
		bf_node_clear(&list_node->buf_pool_rbt);
		spin_lock(&s_cma_lock);
		list_del(&list_node->list);
		spin_unlock(&s_cma_lock);
		kfree(list_node);
		up(&s_cma_sem);
	}

	return 0;
}

static int cma_mem_mmap(struct file *filp, struct vm_area_struct *vma)
{
	struct cma_buffer_pool_list_node_t *list_node = filp->private_data;
	// _dev_info(cma_dev, "Mmap Phy_addr at 0x%lx\n", vma->vm_pgoff << PAGE_SHIFT);
	struct cma_buffer_node_t *bf_node = bf_node_search(&list_node->buf_pool_rbt,
			vma->vm_pgoff << PAGE_SHIFT);

	if (bf_node == NULL) {
		_dev_err(cma_dev, "Mmap Unmanaged Phy_addr at 0x%lx\n",
				vma->vm_pgoff << PAGE_SHIFT);
		return -ENXIO;
	}
	if (bf_node->buffer.size != vma->vm_end - vma->vm_start) {
		_dev_err(cma_dev, "Mmap Phy_addr with wrong size\n");
		return -ENXIO;
	}

	vma->vm_flags |= VM_IO | VM_RESERVED;
	switch (bf_node->buffer.flags) {
	default:
	case 0:
		vma->vm_page_prot = PAGE_SHARED;
		break;
	case 1:
		vma->vm_page_prot = pgprot_writecombine(vma->vm_page_prot);
		break;
	case 2:
		vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);
		break;
	}

	if (remap_pfn_range(vma, vma->vm_start, vma->vm_pgoff, vma->vm_end - vma->vm_start,
			vma->vm_page_prot)) {
		_dev_err(cma_dev, "%s: remap_pfn_range failed at [0x%lx  0x%lx]\n", __func__,
				vma->vm_start, vma->vm_end);
		return -ENXIO;
	}
	return 0;
}

loff_t cma_mem_llseek(struct file *filp, loff_t offset, int whence)
{
	return -1;
}
static ssize_t cma_mem_read(struct file *file, char __user *buf, size_t size, loff_t *ppos)
{
	return -1;
}
static ssize_t cma_mem_write(struct file *file, const char __user *buf, size_t size, loff_t *ppos)
{
	return -1;
}

static long cma_ioctl(struct file *filp, u_int cmd, u_long arg)
{
	//  _dev_info(cma_dev, "entering cma_ioctl\n");
	int ret = 0;
	struct cma_buffer_pool_list_node_t *list_node = filp->private_data;

	switch (cmd) {
	case CMA_MEM_WRITE: {
		struct cma_write_info_t w_info;

		ret = copy_from_user(&w_info, (struct cma_write_info_t *)(arg),
				sizeof(struct cma_write_info_t));
		if (ret)
			break;

		struct cma_buffer_node_t *bf_node = bf_node_search(&list_node->buf_pool_rbt,
				w_info.phy_addr);

		if (bf_node == NULL) {
			_dev_err(cma_dev, "Access Unmanaged Phy_addr at 0x%llx\n", w_info.phy_addr);
			return -EFAULT;
		}
		if (w_info.offset + w_info.len > bf_node->buffer.size) {
			_dev_err(cma_dev, "Access Over Range\n");
			return -EFAULT;
		}

		ret = copy_from_user(bf_node->buffer.base + w_info.offset, w_info.buf, w_info.len);
	} break;
	case CMA_MEM_SYNC_FOR_CPU: {
		struct cma_sync_range_t sync;

		ret = copy_from_user(&sync, (struct cma_write_info_t *)(arg),
				sizeof(struct cma_sync_range_t));
		if (ret)
			break;

		dma_sync_single_for_cpu(cma_dev, (dma_addr_t)(sync.phy_addr + sync.offset),
				sync.len, DMA_BIDIRECTIONAL);
	} break;
	case CMA_MEM_SYNC_FOR_DEVICE: {
		struct cma_sync_range_t sync;

		ret = copy_from_user(&sync, (struct cma_write_info_t *)(arg),
				sizeof(struct cma_sync_range_t));
		if (ret)
			break;

		dma_sync_single_for_device(cma_dev, (dma_addr_t)(sync.phy_addr + sync.offset),
				sync.len, DMA_BIDIRECTIONAL);
	} break;
	case CMA_MEM_ALLOCATE: {
		ret = down_interruptible(&s_cma_sem);
		if (ret == 0) {
			struct cma_buffer_t bf;

			ret = copy_from_user(&bf, (struct cma_buffer_t *)(arg),
					sizeof(struct cma_buffer_t));
			if (ret) {
				up(&s_cma_sem);
				break;
			}
			if (bf.size == 0) {
				_dev_err(cma_dev, "Wrong size");
				up(&s_cma_sem);
				return -EFAULT;
			}

			struct cma_buffer_node_t *bf_node = bf_node_create(bf.size);

			if (bf_node == NULL) {
				_dev_err(cma_dev, "Alloc fail,but %llu bytes Managed by this fd",
						list_node->mem_sum);
				up(&s_cma_sem);
				return -EFAULT;
			}

			bf_node->buffer.flags = bf.flags;

			spin_lock(&s_cma_lock);
			if (!bf_node_insert(&list_node->buf_pool_rbt, bf_node)) {
				_dev_err(cma_dev, "Phy_addr[0x%llx] be Managed Twice",
						bf_node->buffer.phys_addr);
				up(&s_cma_sem);
				return -EFAULT;
			}
			list_node->mem_sum += bf_node->buffer.size;
			spin_unlock(&s_cma_lock);

			ret = copy_to_user((struct cma_buffer_t *)(arg), &(bf_node->buffer),
					sizeof(struct cma_buffer_t));
			up(&s_cma_sem);
		}
	} break;
	case CMA_MEM_FREE: {
		ret = down_interruptible(&s_cma_sem);
		if (ret == 0) {
			struct cma_buffer_node_t *bf_node =
					bf_node_search(&list_node->buf_pool_rbt, arg);

			if (bf_node == NULL) {
				_dev_err(cma_dev, "Access Unmanaged Phy_addr at 0x%lx\n", arg);
				up(&s_cma_sem);
				return -EFAULT;
			}
			spin_lock(&s_cma_lock);
			list_node->mem_sum -= bf_node->buffer.size;
			bf_node_earse(&list_node->buf_pool_rbt, bf_node);
			spin_unlock(&s_cma_lock);
			up(&s_cma_sem);
		}
	} break;
	default: {
		_dev_err(cma_dev, KERN_ERR "No such IOCTL, cmd is %d\n", cmd);
	} break;
	}
	return ret;
}

static const struct file_operations cma_mem_fops = {
	.owner = THIS_MODULE,
	.open = cma_mem_open,
	.release = cma_mem_release,
	.read = cma_mem_read,
	.write = cma_mem_write,
	// .llseek = cma_mem_llseek,
	.mmap = cma_mem_mmap,
	.unlocked_ioctl = cma_ioctl,
};

struct miscdevice cma_mem_misc = {
	.name = "cma_mem",
	.fops = &cma_mem_fops,
};

static int __init cma_mem_init(void)
{
	int ret = misc_register(&cma_mem_misc);

	if (unlikely(ret)) {
		pr_err("failed to register cma test misc device!\n");
		return ret;
	}

	cma_dev = cma_mem_misc.this_device;
	cma_dev->coherent_dma_mask = ~0;

	_dev_info(cma_dev, "registered.\n");
	return ret;
}

module_init(cma_mem_init);

static void __exit cma_mem_exit(void)
{
	misc_deregister(&cma_mem_misc);
}

module_exit(cma_mem_exit);

MODULE_LICENSE("GPL");
MODULE_ALIAS("CMA mem driver");
