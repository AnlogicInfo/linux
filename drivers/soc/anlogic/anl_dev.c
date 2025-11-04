// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2025 Anlogic, Inc.
 */
#include <linux/device.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/miscdevice.h>
#include <soc/anlogic/anl_top.h>

static ssize_t soc_info_show(struct device *dev,
			     struct device_attribute *attr, char *buf)
{
	int ret;

	ret = sprintf(buf, "soc sn\t: 0x%016lx\n", (ulong)anl_get_chipid());

	return ret;
}
static DEVICE_ATTR_RO(soc_info);

static struct attribute *anl_devinfo_attrs[] = {
	&dev_attr_soc_info.attr,
	NULL,
};
ATTRIBUTE_GROUPS(anl_devinfo);

static struct class anl_devinfo_class = {
	.name = "dev_info",
	.class_groups = anl_devinfo_groups,
};

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

	ret = class_register(&anl_devinfo_class);
	if (ret) {
		pr_err("anl top dev register class failed\n");
		goto err_exit;
	}

#if IS_ENABLED(CONFIG_ANLOGIC_TOP_USER_SACCESS)
	ret = misc_register(&secreg_miscdev);
	if (ret < 0) {
		pr_err("anl top register secreg fail\n");
		goto unregister_class;
	}
#endif

	return 0;
#if IS_ENABLED(CONFIG_ANLOGIC_TOP_USER_SACCESS)
unregister_class:
	class_unregister(&anl_devinfo_class);
#endif
err_exit:
	return ret;
}
module_init(anl_top_dev_init);

MODULE_AUTHOR("fushan.zeng<fushan.zeng@anlogic.com>");
MODULE_DESCRIPTION("anlogic top dev");
MODULE_LICENSE("GPL");
