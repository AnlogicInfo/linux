// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2025 Anlogic, Inc.
 */
#include <linux/device.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/miscdevice.h>
#include <soc/anlogic/anl_top.h>

#if IS_ENABLED(CONFIG_ANLOGIC_TOP_USER_SACCESS)
static const struct file_operations secreg_fops = {
	.owner = THIS_MODULE,
	.write = secreg_write,
	.read = secreg_read,
	.unlocked_ioctl = secreg_ioctl,
	.compat_ioctl = compat_ptr_ioctl,
	.open = secreg_open,
	.release = secreg_release,
	.llseek = secreg_llseek,
};

static struct miscdevice secreg_miscdev = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "secreg",
	.fops = &secreg_fops,
};
#endif

static int __init anl_top_dev_init(void)
{
	int ret = 0;
#if IS_ENABLED(CONFIG_ANLOGIC_TOP_USER_SACCESS)
	ret = misc_register(&secreg_miscdev);
	if (ret < 0)
		pr_err("anl top register secreg fail\n");
#endif
	return ret;
}
module_init(anl_top_dev_init);

MODULE_AUTHOR("fushan.zeng<fushan.zeng@anlogic.com>");
MODULE_DESCRIPTION("anlogic top dev");
MODULE_LICENSE("GPL");
