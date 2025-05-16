#include <linux/module.h>
#include "rel_ver.h"

static int __init anl_display_ver(void)
{
	printk("*** anlogic linux version: %s ***\n", ANLOGIC_REL_VER);
	return 0;
}
early_initcall(anl_display_ver);
