/*
 * Tc35876x MIPI to Parallel Bridge Chip
 *
 *
 * Copyright (C) 2006, Marvell Corporation.
 *
 * This software program is licensed subject to the GNU General Public License
 * (GPL).Version 2,June 1991, available at http://www.fsf.org/copyleft/gpl.html
 */
#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/proc_fs.h>
#include <linux/suspend.h>
#include <linux/i2c.h>
#include <linux/mutex.h>
#include <linux/uaccess.h>
#include <mach/cputype.h>
#include <mach/mfp-mmp2.h>
#include <mach/gpio.h>
#include <mach/regs-mpmu.h>
#include <mach/tc35876x.h>

#define TC35876x_REG_NUM		(0x5a4)

/* Unique ID allocation */
static struct i2c_client *g_client;

static DEFINE_MUTEX(lock);

int tc35876x_read32(u16 reg, u32 *pval)
{
	int ret;
	int status;
	u8 address[2], data[4];

	if (g_client == NULL)	/* No global client pointer? */
		return -1;

	mutex_lock(&lock);
	memset(data, 0, 4);
	address[0] = (reg >> 8) & 0xff;
	address[1] = reg & 0xff;

	ret = i2c_master_send(g_client, address, 2);
	if (ret < 0) {
		status = -EIO;
		goto out_unlock;
	}
	ret = i2c_master_recv(g_client, data, 4);
	if (ret >= 0) {
		status = 0;
		*pval = data[0] | (data[1] << 8) | (data[2] << 16)
		    | (data[3] << 24);
	} else
		status = -EIO;

out_unlock:
	mutex_unlock(&lock);

	return status;
}
EXPORT_SYMBOL(tc35876x_read32);

int tc35876x_read16(u16 reg, u16 *pval)
{
	int ret;
	int status;
	u8 address[2], data[4];

	if (g_client == NULL)	/* No global client pointer? */
		return -1;

	mutex_lock(&lock);
	address[0] = (reg >> 8) & 0xff;
	address[1] = reg & 0xff;

	ret = i2c_master_send(g_client, address, 2);
	if (ret < 0) {
		status = -EIO;
		goto out_unlock;
	}

	ret = i2c_master_recv(g_client, data, 2);
	if (ret >= 0) {
		status = 0;
		*pval = data[0] | (data[1] << 8);
	} else
		status = -EIO;
out_unlock:
	mutex_unlock(&lock);

	return status;
}
EXPORT_SYMBOL(tc35876x_read16);

int tc35876x_write32(u16 reg, u32 val)
{
	int ret;
	int status;
	u8 data[6];

	if (g_client == NULL)	/* No global client pointer? */
		return -1;

	mutex_lock(&lock);
	data[0] = (reg >> 8) & 0xff;
	data[1] = reg & 0xff;
	data[2] = val & 0xff;
	data[3] = (val >> 8) & 0xff;
	data[4] = (val >> 16) & 0xff;
	data[5] = (val >> 24) & 0xff;
	ret = i2c_master_send(g_client, data, 6);
	if (ret >= 0)
		status = 0;
	else
		status = -EIO;
	mutex_unlock(&lock);

	return status;
}
EXPORT_SYMBOL(tc35876x_write32);

int tc35876x_write16(u16 reg, u16 val)
{
	int ret;
	int status;
	u8 data[4];

	if (g_client == NULL)	/* No global client pointer? */
		return -1;

	mutex_lock(&lock);
	data[0] = (reg >> 8) & 0xff;
	data[1] = reg & 0xff;
	data[2] = val & 0xff;
	data[3] = (val >> 8) & 0xff;

	ret = i2c_master_send(g_client, data, 4);
	if (ret >= 0)
		status = 0;
	else
		status = -EIO;
	mutex_unlock(&lock);

	return status;
}
EXPORT_SYMBOL(tc35876x_write16);

#ifdef	CONFIG_PROC_FS
#define	TC358762_PROC_FILE	"driver/tc35876x"
static struct proc_dir_entry *tc35876x_proc_file;
static int index;

static ssize_t tc35876x_proc_read(struct file *filp,
				  char *buffer, size_t length,
				  loff_t *offset)
{
	u32 reg_val;
	int ret;

	if ((index < 0) || (index > TC35876x_REG_NUM))
		return 0;

	ret = tc35876x_read32(index, &reg_val);
	if (ret < 0)
		printk(KERN_INFO "tc35876x read error!\n");
	else
		printk(KERN_INFO "register 0x%x: 0x%x\n", index, reg_val);
	return 0;
}

static ssize_t tc35876x_proc_write(struct file *filp,
				   const char *buff, size_t len,
				   loff_t *off)
{
	u32 reg_val;
	char messages[256], vol[256];

	if (len > 256)
		len = 256;

	if (copy_from_user(messages, buff, len))
		return -EFAULT;

	if ('-' == messages[0]) {
		/* set the register index */
		memcpy(vol, messages + 1, len - 1);
		index = (int) simple_strtoul(vol, NULL, 16);
	} else {
		/* set the register value */
		reg_val = (int) simple_strtoul(messages, NULL, 16);
		tc35876x_write32(index, reg_val);
	}

	return len;
}

static struct file_operations tc35876x_proc_ops = {
	.read = tc35876x_proc_read,
	.write = tc35876x_proc_write,
};

static void create_tc35876x_proc_file(void)
{
	tc35876x_proc_file =
	    create_proc_entry(TC358762_PROC_FILE, 0644, NULL);
	if (tc35876x_proc_file)
		tc35876x_proc_file->proc_fops = &tc35876x_proc_ops;
	else
		printk(KERN_INFO "proc file create failed!\n");
}

static void remove_tc35876x_proc_file(void)
{
	extern struct proc_dir_entry proc_root;
	remove_proc_entry(TC358762_PROC_FILE, &proc_root);
}

#endif

static int __devinit tc35876x_probe(struct i2c_client *client,
				    const struct i2c_device_id *id)
{
	struct tc35876x_platform_data *pdata;

	g_client = client;
	pdata = client->dev.platform_data;
	pdata->platform_init();


#ifdef	CONFIG_PROC_FS
	create_tc35876x_proc_file();
#endif

	return 0;
}

static int tc35876x_remove(struct i2c_client *client)
{
#ifdef	CONFIG_PROC_FS
	remove_tc35876x_proc_file();
#endif

	return 0;
}

static const struct i2c_device_id tc35876x_id[] = {
	{"tc35876x", 0},
	{}
};

static struct i2c_driver tc35876x_driver = {
	.driver = {
		   .name = "tc35876x",
		   },
	.id_table = tc35876x_id,
	.probe = tc35876x_probe,
	.remove = tc35876x_remove,
};

static int __init tc35876x_init(void)
{
	return i2c_add_driver(&tc35876x_driver);
}

static void __exit tc35876x_exit(void)
{
	i2c_del_driver(&tc35876x_driver);
}

subsys_initcall(tc35876x_init);
module_exit(tc35876x_exit);

MODULE_DESCRIPTION("Tc35876x Driver");
MODULE_LICENSE("GPL");
