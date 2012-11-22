/*
 * linux/drivers/video/pxa168fb.c -- Marvell PXA168 LCD Controller
 *
 *  Copyright (C) 2008 Marvell International Ltd.
 *  All rights reserved.
 *
 *  2009-02-16  adapted from original version for PXA168
 *		Kevin Liu <kliu5@marvell.com>
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License. See the file COPYING in the main directory of this archive for
 * more details.
 */

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/errno.h>
#include <linux/string.h>
#include <linux/interrupt.h>
#include <linux/console.h>
#include <linux/slab.h>
#include <linux/fb.h>
#include <linux/delay.h>
#include <linux/init.h>
#include <linux/ioport.h>
#include <linux/cpufreq.h>
#include <linux/platform_device.h>
#include <linux/dma-mapping.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/uaccess.h>
#include <linux/proc_fs.h>

#ifdef CONFIG_PXA688_CMU

#include "pxa168fb.h"
#include <mach/io.h>
#include <mach/irqs.h>
#include <mach/pxa168fb.h>
#include <mach/hardware.h>
#include <mach/cputype.h>
#include <mach/gpio.h>

#include <asm/mach-types.h>
#include <mach/regs-apmu.h>
#include <mach/mfp-mmp2.h>
#include <mach/regs-mpmu.h>
#include <asm/mach-types.h>


static void pxa688fb_cmu_write(struct pxa168fb_info *fbi, unsigned int addr, unsigned int data)
{
	if (addr <= 0x88){
		writel(data, fbi->reg_base + 0x400 + addr);
	} else {
		writel(data & 0xff, fbi->reg_base + 0x400);
		writel(addr | 0x80000000, fbi->reg_base + 0x404);
	}
}

static unsigned int pxa688fb_cmu_read(struct pxa168fb_info *fbi, unsigned int addr)
{
	unsigned int val;
	if (addr <= 0x88)
		val = readl(fbi->reg_base + 0x400 + addr);
	else {
		writel(addr | 0x40000000, fbi->reg_base + 0x404);
		val = readl(fbi->reg_base + 0x400) & 0xff;
	}
	return val;
}

int pxa688_cmu_ioctl(struct fb_info *info, unsigned int cmd, unsigned long arg)
{
	struct pxa168fb_info *fbi = info->par;
	void __user *argp = (void __user *)arg;
	struct _pxa168fb_cmu_config cmu_config;
	struct _pxa168fb_cmu_pip rect;
	struct pxa168fb_mach_info *mi = fbi->dev->platform_data;
	struct cmu_res res;
	int val, val_top, val_io_overl, val_bln_ctl, vsync_enable, hsync_enable;
	unsigned int pip_h, pip_v, main_h, main_v;
	static int path = 0;
	unsigned hsync_len = gfx_info.fbi[path]->fb_info->var.hsync_len;
	unsigned vsync_len = gfx_info.fbi[path]->fb_info->var.vsync_len;
	unsigned left_margin = gfx_info.fbi[path]->fb_info->var.left_margin;
	unsigned upper_margin = gfx_info.fbi[path]->fb_info->var.upper_margin;
	unsigned yres = gfx_info.fbi[path]->fb_info->var.yres;
	unsigned xres = gfx_info.fbi[path]->fb_info->var.xres;

	switch (cmd) {
		case FB_IOCTL_CMU_GET_RES:
			if (copy_from_user(&res, argp, sizeof(res)))
				return -EFAULT;
			res.width = xres;
			res.height = yres;
			return copy_to_user(argp, &res, sizeof(res)) ? -EFAULT : 0;
			break;
		case FB_IOCTL_CMU_SWITCH:	/* FIXME */
			if (arg) {
				writel(0x1, fbi->reg_base + 0x400 + CMU_CTRL);
				val = readl(fbi->reg_base + LCD_TOP_CTRL);
				if (fbi->id == 0)
					val |= 0x1;
				else
					val |= 0x3;
				writel(val, fbi->reg_base + LCD_TOP_CTRL);
			} else {
				val = readl(fbi->reg_base + LCD_TOP_CTRL);
				val &= (~0x3);
				writel(val, fbi->reg_base + LCD_TOP_CTRL);
				writel(0x0, fbi->reg_base + 0x400 + CMU_CTRL);
			}
			break;
		case FB_IOCTL_CMU_SET_PIP:
			if ( !cpu_is_mmp2_z0() && !cpu_is_mmp2_z1()) {
				if(copy_from_user(&rect, argp, sizeof(rect)))
					return -EFAULT;
				/*
				*if vsync/hsync inverted,
				*exclude hsync_len and vsync_len
				*/
				hsync_enable = (readl(fbi->reg_base +
					intf_ctrl(fbi->id)) & HINVERT_MSK)
					>> HINVERT_LEN;
				vsync_enable = (readl(fbi->reg_base +
					intf_ctrl(fbi->id)) & VINVERT_MSK)
					>> VINVERT_LEN;
				if (hsync_enable)
					hsync_len = 0;
				if (vsync_enable)
					vsync_len = 0;
				/*calculate pip setting data*/
				pip_h = ((rect.left + left_margin + hsync_len
					+ mi->cmu_cal[path].left) & 0xfff) |
					((rect.right + left_margin + hsync_len
					+ mi->cmu_cal[path].right)<<12 &
					0xfff000);
				pip_v = ((rect.top + upper_margin + vsync_len
					+ mi->cmu_cal[path].top) & 0xfff) |
					((rect.bottom + upper_margin +
					vsync_len + mi->cmu_cal[path].bottom)
					<<12 & 0xfff000);
				main_h = ((left_margin + hsync_len) & 0xfff) |
					((xres + hsync_len + left_margin)<<12
					 & 0xfff000);
				main_v = ((upper_margin + vsync_len) & 0xfff) |
					((yres + upper_margin + vsync_len)<<12
					 & 0xfff000);
				/*set pip register*/
				writel(pip_h, fbi->reg_base + 0x400 + 0x08);// ACE_PIP_DE
				writel(pip_v, fbi->reg_base + 0x400 + 0x0c);
				writel(pip_h, fbi->reg_base + 0x400 + 0x28);// PIP_DE
				writel(pip_v, fbi->reg_base + 0x400 + 0x2c);
				writel(pip_h, fbi->reg_base + 0x400 + 0x10);// PRI
				writel(pip_v, fbi->reg_base + 0x400 + 0x14);

				writel(main_h, fbi->reg_base + 0x400 + 0x20);// ACE_MAIN_DE
				writel(main_v, fbi->reg_base + 0x400 + 0x24);
				writel(0x3f, fbi->reg_base + 0x400 + 0x30);
			} else {
				printk(KERN_INFO"Z stepping doesn't support\
					 PIP mode!\n");
			}
			break;
		case FB_IOCTL_CMU_SET_LETTER_BOX:
			if (!cpu_is_mmp2_z0() && !cpu_is_mmp2_z1()) {
				if (copy_from_user(&rect, argp, sizeof(rect)))
					return -EFAULT;
				/*
				* if vsync/hsync inverted,
				* exclude hsync_len and vsync_len
				*/
				hsync_enable = (readl(fbi->reg_base +
					intf_ctrl(fbi->id)) & HINVERT_MSK)
					>> HINVERT_LEN;
				vsync_enable = (readl(fbi->reg_base +
					intf_ctrl(fbi->id)) & VINVERT_MSK)
					 >> VINVERT_LEN;
				if (hsync_enable)
					hsync_len = 0;
				if (vsync_enable)
					vsync_len = 0;
				/* calculate letter box setting data */
				pip_h = ((rect.left + left_margin + hsync_len +
					mi->cmu_cal_letter_box[path].left)
					& 0xfff) |
					((rect.right + left_margin + hsync_len
					+ mi->cmu_cal_letter_box[path].right)
					<<12 & 0xfff000);
				pip_v = ((rect.top + upper_margin + vsync_len +
					mi->cmu_cal_letter_box[path].top)
					& 0xfff) |
					((rect.bottom + upper_margin + vsync_len
					+ mi->cmu_cal_letter_box[path].bottom)
					<<12 & 0xfff000);
				/* set letter box register */
				/* ACE_LETTER_BOX_DE */
				writel(pip_h, fbi->reg_base + 0x400 + 0x18);
				writel(pip_v, fbi->reg_base + 0x400 + 0x1c);
				val = readl(fbi->reg_base + 0x400 + 0x30);
				/* Enable Letter box setting */
				writel(val|0x3, fbi->reg_base + 0x400 + 0x30);
			} else {
				printk(KERN_INFO"Z stepping doesn't support\
					 letter box mode!\n");
			}
			break;
		case FB_IOCTL_CMU_SET_ROUTE:
			if ( !cpu_is_mmp2_z0() && !cpu_is_mmp2_z1()) {
				val = readl(fbi->reg_base + 0x400 + CMU_CTRL);
				val_top = readl(fbi->reg_base + LCD_TOP_CTRL);
				val_io_overl = 0;
				val_bln_ctl = 0;
				if (arg == ICR_DRV_ROUTE_OFF) {
					val &= ~CMU_CLK_CTRL_ENABLE;
					val_top &= ~LCD_TOP_CTRL_CMU_ENABLE;
				} else if (arg == ICR_DRV_ROUTE_TV) {
					val |= CMU_CLK_CTRL_ENABLE;
					val &= ~CMU_CTRL_A0_MSK;
					val |= CMU_CTRL_A0_TV << 1;

					val_top |= LCD_TOP_CTRL_CMU_ENABLE;

					val_io_overl &= ~LCD_IO_OVERL_MSK;
					val_io_overl |= LCD_IO_OVERL_TV;

					val_io_overl &= ~LCD_IO_CMU_IN_SEL_MSK;
					val_io_overl |= LCD_IO_CMU_IN_SEL_TV << 20;

					val_io_overl &= ~LCD_IO_TV_OUT_SEL_MSK;
					val_io_overl |= LCD_IO_TV_OUT_SEL_NON << 26;
					path = 1;
				} else if (arg == ICR_DRV_ROUTE_LCD1) {
					val |= CMU_CLK_CTRL_ENABLE;
					val &= ~CMU_CTRL_A0_MSK;
					val |= CMU_CTRL_A0_LCD1 << 1;

					val_top |= LCD_TOP_CTRL_CMU_ENABLE;
					val_io_overl &= ~LCD_IO_OVERL_MSK;
					val_io_overl |= LCD_IO_OVERL_LCD1;
					val_io_overl &= ~LCD_IO_CMU_IN_SEL_MSK;
					val_io_overl |= LCD_IO_CMU_IN_SEL_PN << 20;
					val_io_overl &= ~LCD_IO_PN_OUT_SEL_MSK;
					val_io_overl |= LCD_IO_PN_OUT_SEL_NON << 24;

					val_bln_ctl = 0x300;
					path = 0;
				} else if (arg == ICR_DRV_ROUTE_LCD2) {
					val |= CMU_CLK_CTRL_ENABLE;
					val &= ~CMU_CTRL_A0_MSK;
					val |= CMU_CTRL_A0_LCD2 << 1;

					val_top |= LCD_TOP_CTRL_CMU_ENABLE;
					val_io_overl &= ~LCD_IO_OVERL_MSK;
					val_io_overl |= LCD_IO_OVERL_LCD2;
					val_io_overl &= ~LCD_IO_CMU_IN_SEL_MSK;
					val_io_overl |= LCD_IO_CMU_IN_SEL_PN2 << 20;
					val_io_overl &= ~LCD_IO_PN2_OUT_SEL_MSK;
					val_io_overl |= LCD_IO_PN2_OUT_SEL_NON << 28;

					val_bln_ctl = 0xc00;
					path = 2;
				}
				writel(0x3e, fbi->reg_base + 0x400 + 0x30);// CMU_CTRL_EN_CFG
				writel(val_top, fbi->reg_base + LCD_TOP_CTRL);
				writel(val, fbi->reg_base + 0x400 + CMU_CTRL);
				writel(val_io_overl, fbi->reg_base + LCD_IO_OVERL_MAP_CTRL);
				writel(val_bln_ctl, fbi->reg_base + LCD_2ND_BLD_CTL);
			} else {
				val = readl(fbi->reg_base + 0x400 + CMU_CTRL);
				val_top = readl(fbi->reg_base + LCD_TOP_CTRL);
				if (arg == ICR_DRV_ROUTE_OFF) {
					val_top &= ~LCD_TOP_CTRL_CMU_ENABLE;
					val &= ~CMU_CLK_CTRL_ENABLE;
				} else if (arg == ICR_DRV_ROUTE_TV) {
					val_top &= ~LCD_TOP_CTRL_SEL_MSK;
					val_top |= LCD_TOP_CTRL_TV;
					val_top |= LCD_TOP_CTRL_CMU_ENABLE;

					val &= ~CMU_CLK_CTRL_MSK;
					val |= CMU_CLK_CTRL_TCLK;
					val |= CMU_CLK_CTRL_ENABLE;
				} else if (arg == ICR_DRV_ROUTE_LCD1) {
					val_top &= ~LCD_TOP_CTRL_SEL_MSK;
					val_top |= LCD_TOP_CTRL_PNL;
					val_top |= LCD_TOP_CTRL_CMU_ENABLE;

					val &= ~CMU_CLK_CTRL_MSK;
					val |= CMU_CLK_CTRL_SCLK;
					val |= CMU_CLK_CTRL_ENABLE;
				} else if (arg == ICR_DRV_ROUTE_LCD2) {
					val_top &= ~LCD_TOP_CTRL_SEL_MSK;
					val_top |= LCD_TOP_CTRL_TV;
					val_top |= LCD_TOP_CTRL_CMU_ENABLE;

					val &= ~CMU_CLK_CTRL_MSK;
					val |= CMU_CLK_CTRL_TCLK;
					val |= CMU_CLK_CTRL_ENABLE;
				}
				writel(val_top, fbi->reg_base + LCD_TOP_CTRL);
				writel(val, fbi->reg_base + 0x400 + CMU_CTRL);
			}
			break;


		case FB_IOCTL_CMU_WRITE:
			if (copy_from_user(&cmu_config, argp, sizeof(cmu_config)))
				return -EFAULT;
			pxa688fb_cmu_write(fbi, cmu_config.addr, cmu_config.data);
			break;

		case FB_IOCTL_CMU_READ:
			if (copy_from_user(&cmu_config, argp, sizeof(cmu_config)))
				return -EFAULT;
			cmu_config.data = pxa688fb_cmu_read(fbi, cmu_config.addr);
			return copy_to_user(argp, &cmu_config, sizeof(cmu_config)) ? -EFAULT : 0;
			break;

		default:
			printk("%s: unknown IOCTL 0x%x\n", __func__, cmd);
			return -EINVAL;
	}
	return 0;
}

static int cmu_store(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t size)
{
	struct pxa168fb_info *fbi = dev_get_drvdata(dev);
	void *cmu_base = fbi->reg_base + 0x400;
	unsigned res;
	unsigned addr = (unsigned)simple_strtoul(buf, NULL, 16);
	writel(0x40000000 | addr, cmu_base + 4);
	res = readl(cmu_base) & 0xff;
	printk("addr 0x%x is 0x%x \n", addr, res);
	return size;
}
static int cmu_show(struct device *dev, struct device_attribute *attr,
			char *buf)
{
	struct pxa168fb_info *fbi = dev_get_drvdata(dev);
	void *cmu_base = fbi->reg_base + 0x400;
	printk("cmu \n");
	printk("CMU_PIP_DE_H_CFG : %8x\n", readl(cmu_base + 0x8));
	printk("CMU_PIP_DE_V_CFG : %8x\n", readl(cmu_base + 0xc));
	printk("CMU_PRI_H_CFG : %8x\n", readl(cmu_base + 0x10));
	printk("CMU_PRI_V_CFG : %8x\n", readl(cmu_base + 0x14));
	printk("CMU_LETTER_BOX_H_CFG : %8x\n", readl(cmu_base + 0x18));
	printk("CMU_LETTER_BOX_V_CFG : %8x\n", readl(cmu_base + 0x1C));
	printk("CMU_ACE_MAIN_DE_H_CFG : %8x\n", readl(cmu_base + 0x20));
	printk("CMU_ACE_MAIN_DE_V_CFG : %8x\n", readl(cmu_base + 0x24));
	printk("CMU_ACE_PIP_DE_H_CFG : %8x\n", readl(cmu_base + 0x28));
	printk("CMU_ACE_PIP_DE_V_CFG : %8x\n", readl(cmu_base + 0x2c));
	printk("CMU_CTL_EN_CFG : %8x\n", readl(cmu_base + 0x30));
	printk("CMU_BAR_0_CFG : %8x\n", readl(cmu_base + 0x34));
	printk("CMU_BAR_1_CFG : %8x\n", readl(cmu_base + 0x38));
	printk("CMU_BAR_2_CFG : %8x\n", readl(cmu_base + 0x3c));
	printk("CMU_BAR_3_CFG : %8x\n", readl(cmu_base + 0x40));
	printk("CMU_BAR_4_CFG : %8x\n", readl(cmu_base + 0x44));
	printk("CMU_BAR_5_CFG : %8x\n", readl(cmu_base + 0x48));
	printk("CMU_BAR_6_CFG : %8x\n", readl(cmu_base + 0x4c));
	printk("CMU_BAR_7_CFG : %8x\n", readl(cmu_base + 0x50));
	printk("CMU_BAR_8_CFG : %8x\n", readl(cmu_base + 0x54));
	printk("CMU_BAR_9_CFG : %8x\n", readl(cmu_base + 0x58));
	printk("CMU_BAR_10_CFG : %8x\n", readl(cmu_base + 0x5c));
	printk("CMU_BAR_11_CFG : %8x\n", readl(cmu_base + 0x60));
	printk("CMU_BAR_12_CFG : %8x\n", readl(cmu_base + 0x64));
	printk("CMU_BAR_13_CFG : %8x\n", readl(cmu_base + 0x68));
	printk("CMU_BAR_14_CFG : %8x\n", readl(cmu_base + 0x6c));
	printk("CMU_BAR_15_CFG : %8x\n", readl(cmu_base + 0x70));
	printk("CMU_BAR_CTRL : %8x\n", readl(cmu_base + 0x74));
	printk("PATTERN_TOTAL : %8x\n", readl(cmu_base + 0x78));
	printk("PATTERN_ACTIVE : %8x\n", readl(cmu_base + 0x7c));
	printk("PATTERN_FRONT_PORCH : %8x\n", readl(cmu_base + 0x80));
	printk("PATTERN_BACK_PORCH : %8x\n", readl(cmu_base + 0x84));
	printk("CMU_CTRL : %8x\n", readl(cmu_base + 0x88));
	return 0;
}

static DEVICE_ATTR(cmu, S_IRUGO | S_IWUSR, cmu_show, cmu_store);

static int __devinit cmu_init(void)
{

	int ret;
	ret = device_create_file(gfx_info.fbi[0]->dev, &dev_attr_cmu);
	if (ret < 0) {
		pr_err("device attr create fail: %d\n", ret);
		return ret;
	}

	return 0;
}
module_init(cmu_init);

MODULE_AUTHOR("Yifan Zhang<zhangyf@marvell.com>");
MODULE_DESCRIPTION("CMU driver for PXA688");
MODULE_LICENSE("GPL");
#endif
