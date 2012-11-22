/*
 * drivers/char/audioGpoSet.c
 *
 * The headset detect driver based on micco
 *
 * Copyright (2008) Marvell International Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>

#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/types.h>
#include <linux/kdev_t.h>
#include <linux/interrupt.h>
#include <linux/spinlock.h>
#include <linux/uio.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/platform_device.h>
#include <linux/proc_fs.h>
#include <linux/miscdevice.h>
#include <linux/io.h>
#include <linux/mman.h>
#include <linux/uaccess.h>

#include <mach/hardware.h>
#include <asm/irq.h>
#include <asm/mach-types.h>
#include <mach/gpo.h>
#include <mach/bitfield.h>
#include <mach/mfp-pxa9xx.h>
#include <mach/gpio.h>

struct device *gpo_dev;
struct GPO_IOCTL gpo_detect;

#define A_DEBUG_MUST  KERN_ALERT

/*If following functions are needed,GPIOs must be set according to specific platform*/
static void setGpio(int gpio, int value)
{
	int err;

	err = gpio_request(gpio, "MONO_AMP_EN");
	if (err) {
		gpio_free(gpio);
		printk(KERN_ERR "Request GPIO failed,"
		       "gpio: %d return :%d\n", gpio, err);
		return;
	}

	gpio_direction_output(gpio, value);
	gpio_free(gpio);
}


void stereo_amp_en(int value)
{
	int err;

	err = gpio_request(MFP_PIN_GPIO84, "STEREO_AMP_EN");
	if (err) {
		gpio_free(MFP_PIN_GPIO84);
		printk(KERN_ERR "Request GPIO failed,"
				"gpio: %d return :%d\n", MFP_PIN_GPIO84, err);
		return;
	}
	gpio_direction_output(MFP_PIN_GPIO84, value);
	gpio_free(MFP_PIN_GPIO84);
}

void ext_mono_amp_en(int value)
{
	if (machine_is_saar())
		setGpio(MFP_PIN_GPIO63, value);
	else {
		printk(A_DEBUG_MUST "\nSet GPIO 78 to %d for EXT AMP\n", value);
		setGpio(MFP_PIN_GPIO78, value);
	}
}

/****************************************************************************
 * File System I/O operations
 ***************************************************************************/

static int gpo_open(struct inode *inode, struct file *file)
{
	return 0;
}

static ssize_t gpo_read(struct file *file, char __user *buf, size_t count,
			    loff_t *ppos)
{
	return 0;
}

/* We assume that the user buf will not larger than kbuf size */
static ssize_t gpo_write(struct file *file, const char __user *buf,
			      size_t count, loff_t *ppos)
{
	return count;
}

static int gpo_release(struct inode *inode, struct file *file)
{
	return 0;
}

static int gpo_ioctl(struct inode *inode,
					 struct file *file,
					 unsigned int cmd,
					 unsigned long arg)
{
	struct GPO_IOCTL gpo_ioctl;
	if (copy_from_user(&gpo_ioctl, (void *)arg, sizeof(struct GPO_IOCTL)))
		return -EFAULT;

	switch (cmd) {
	case GPO_EXT_HEADSET_STATUS:
		if (machine_is_saar())
			stereo_amp_en(gpo_ioctl.gpo_ext_hsdetect_status);
		else
			printk(A_DEBUG_MUST "GPO EXT HEADSET AMP is not being set ==> machine %d does NOT require it!\n", machine_arch_type);
		break;

	case GPO_EXT_MONO_AMP_STATUS:
		ext_mono_amp_en(gpo_ioctl.gpo_ext_mono_amp_status);
		break;

	default:
		 return -ENOTTY;

	}
	return copy_to_user((void *)arg, &gpo_ioctl, sizeof(struct GPO_IOCTL));
}

static const struct file_operations gpo_fops = {
	.owner		= THIS_MODULE,
	.open		= gpo_open,
	.release	= gpo_release,
	.ioctl		= gpo_ioctl,
	.write		= gpo_write,
	.read		= gpo_read,
};

static struct miscdevice gpo_miscdev = {
	.minor		= MISC_DYNAMIC_MINOR,
	.name		= "gpo",
	.fops		= &gpo_fops,
};

/****************************************************************************
 * Initialization / Registeration / Removal
 ***************************************************************************/
static int gpo_probe(struct platform_device *pdev)
{
	int ret;
	ret = misc_register(&gpo_miscdev);
	if (ret < 0)
		return ret;
	else
	return 0;
}

static int gpo_remove(struct platform_device *pdev)
{
	misc_deregister(&gpo_miscdev);
	return 0;
}

static struct platform_driver gpo_driver = {
	.driver = {
		.name	= "gpo",
	},
	.probe		= gpo_probe,
	.remove		= gpo_remove,
};

static int __init gpo_init(void)
{
	return misc_register(&gpo_miscdev);
}

static void __exit gpo_exit(void)
{
	platform_driver_unregister(&gpo_driver);
}

module_init(gpo_init);
module_exit(gpo_exit);
MODULE_LICENSE("GPL");

