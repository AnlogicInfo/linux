/* SPDX-License-Identifier: GPL-2.0+ */

#ifndef _ANL_TOP_H
#define _ANL_TOP_H
void anl_wdt_pause(int id, int pause);
void anl_spi_set_slave(int id, int slave);
int anl_fpga_prog_init(phys_addr_t addr, uint32_t len);
int anl_fpga_prog_start(void);
int anl_fpga_prog_load(phys_addr_t addr, uint32_t len);
int anl_fpga_prog_done(u32 done);
int anl_fpga_plclk_reset(u32 reset);
u64 anl_rproc_start_slave(u64 bootaddr, int slaveid);
u64 anl_rproc_stop_slave(int slaveid);
int anl_sec_readl(ulong off, u32 *val);
int anl_sec_writel(ulong off, u32 val);
int anl_sec_readq(ulong off, u64 *val);
int anl_sec_writeq(ulong off, u64 val);
int secreg_open(struct inode *inode, struct file *file);
int secreg_release(struct inode *inode, struct file *file);
long secreg_ioctl(struct file *file, unsigned int cmd,
		  unsigned long arg);
ssize_t secreg_read(struct file *file, char __user *data,
		    size_t len, loff_t *ppos);
ssize_t secreg_write(struct file *file, const char __user *data,
		     size_t len, loff_t *ppos);
loff_t secreg_llseek(struct file *file, loff_t off, int whence);
void al_misc_reboot(unsigned long entry);
void al_misc_cpu_die(int id);
int anl_wdt_softboot_enable(ulong wdtidx, ulong entry_point);
uint64_t anl_get_chipid(void);
#endif
