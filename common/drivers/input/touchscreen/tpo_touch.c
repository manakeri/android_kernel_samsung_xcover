/*
 * * Copyright (C) 2009, Marvell Corporation(bin.yang@marvell.com).
 * *
 * * Author: Bin Yang <bin.yang@marvell.com>
 * *
 * * This software program is licensed subject to the GNU General Public License
 * * (GPL).Version 2,June 1991, available at http://www.fsf.org/copyleft/gpl.html
 * */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/interrupt.h>
#include <linux/kthread.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/proc_fs.h>
#include <linux/i2c.h>
#include <linux/suspend.h>
#include <linux/input.h>
#include <asm/uaccess.h>
#include <mach/gpio.h>
#include <plat/gpio.h>

static struct i2c_client *g_client;
static struct input_dev *tpo_ts_input_dev;
struct work_struct	tpo_ts_work;
static struct timer_list	tpo_timer;
#define TPO_LEN 12
static u8 tpo_buf[TPO_LEN];
#define TPO_PEN_UP	0
#define TPO_PEN_DOWN	1
static int pen_status = TPO_PEN_UP;

int tpo_touch_read_reg(u8 reg, u8 *pval)
{
	int ret;
	int status;

	if (g_client == NULL)	/* No global client pointer? */
		return -1;
	ret = i2c_smbus_read_byte_data(g_client, reg);
	if (ret >= 0) {
		*pval = ret;
		status = 0;
	} else {
		status = -EIO;
	}

	return status;
}

int tpo_touch_write_reg(u8 reg, u8 val)
{
	int ret;
	int status;

	if (g_client == NULL)	/* No global client pointer? */
		return -1;
	ret = i2c_smbus_write_byte_data(g_client, reg, val);
	if (ret == 0)
		status = 0;
	else
		status = -EIO;

	return status;
}

static int tpo_touch_read(char *buf, int count)
{
	int ret;

	ret = i2c_master_recv(g_client, (char *)buf, count);

	return ret;
}

#if 0
static int tpo_touch_write(char *buf, int count)
{
	int ret;

	ret = i2c_master_send(g_client, buf, count);

	return ret;
}
#endif

static int tpo_touch_recieve_data(void)
{
	return tpo_touch_read(tpo_buf, TPO_LEN);
}

#define tpo_16(x) ((((u16)tpo_buf[x] & 0xff) << 8) | (u16)tpo_buf[x+1])
static void tpo_touch_work(struct work_struct *work)
{
	u16 tem_x1 = 0xffff;
	u16 tem_y1 = 0xffff;
	u8 tmp;

	int ret = tpo_touch_recieve_data();
	if (ret < 0) {
		printk(KERN_ERR "%s: failed to receive data", __func__);
		return;
	}

	tmp = tpo_buf[10];

	if(tmp == 1) {
		if (pen_status == TPO_PEN_DOWN) {
			tem_x1 = tpo_16(2);
			tem_y1 = tpo_16(4);
			tem_y1 = 480 - tem_y1;
			input_report_abs(tpo_ts_input_dev, ABS_PRESSURE, 255);
			input_report_abs(tpo_ts_input_dev, ABS_X, tem_x1);
			input_report_abs(tpo_ts_input_dev, ABS_Y, tem_y1);
			input_report_key(tpo_ts_input_dev, BTN_TOUCH, 1);
			input_sync(tpo_ts_input_dev);
		}
		pen_status = TPO_PEN_DOWN;
		mod_timer(&tpo_timer, jiffies + msecs_to_jiffies(50));
	} else if (tmp == 0) {
		if(pen_status == TPO_PEN_DOWN) {
			pen_status = TPO_PEN_UP;
			input_report_abs(tpo_ts_input_dev, ABS_PRESSURE, 0);
			input_report_key(tpo_ts_input_dev, BTN_TOUCH, 0);
			input_sync(tpo_ts_input_dev);
		}
	}
}

static irqreturn_t tpo_touch_irq_handler(int irq, void *dev_id)
{
	schedule_work(&tpo_ts_work);
	return IRQ_HANDLED;
}

#ifdef	CONFIG_PM
static int tpo_touch_suspend(struct i2c_client *client, pm_message_t state)
{
	tpo_touch_write_reg(0, 0x10);   //sleep mode
	return 0;
}

static int tpo_touch_resume(struct i2c_client *client)
{
	tpo_touch_write_reg(0, 0);	//normal mode
	return 0;
}
#else
#define	tpo_touch_suspend		NULL
#define	tpo_touch_resume		NULL
#endif

#ifdef CONFIG_PROC_FS
#define	TPO_TOUCH_PROC_FILE	"driver/tpo_touch"
static struct proc_dir_entry *tpo_touch_proc_file;
static int index;

static ssize_t tpo_touch_proc_read(struct file *filp,
		char *buffer, size_t length, loff_t *offset)
{
	u8 reg_val;

	if ((index < 0) || (index > TPO_LEN))
		return 0;

	tpo_touch_read_reg(index, &reg_val);
	printk(KERN_INFO "register 0x%x: 0x%x\n", index, reg_val);
	return 0;
}

static ssize_t tpo_touch_proc_write(struct file *filp,
		const char *buff, size_t len, loff_t *off)
{
	u8 reg_val;
	char messages[256], vol[256];

	if (len > 256)
		len = 256;

	if (copy_from_user(messages, buff, len))
		return -EFAULT;

	if ('-' == messages[0]) {
		/* set the register index */
		memcpy(vol, messages+1, len-1);
		index = (int) simple_strtoul(vol, NULL, 16);
	} else {
		/* set the register value */
		reg_val = (int)simple_strtoul(messages, NULL, 16);
		tpo_touch_write_reg(index, reg_val & 0xFF);
	}

	return len;
}

static struct file_operations tpo_touch_proc_ops = {
	.read = tpo_touch_proc_read,
	.write = tpo_touch_proc_write,
};

static void create_tpo_touch_proc_file(void)
{
	tpo_touch_proc_file = create_proc_entry(TPO_TOUCH_PROC_FILE, 0644, NULL);
	if (tpo_touch_proc_file) {
		tpo_touch_proc_file->proc_fops = &tpo_touch_proc_ops;
	} else
		printk(KERN_INFO "proc file create failed!\n");
}

static void remove_tpo_touch_proc_file(void)
{
	extern struct proc_dir_entry proc_root;
	remove_proc_entry(TPO_TOUCH_PROC_FILE, &proc_root);
}
#endif

static int tpo_touch_open(struct input_dev *idev)
{
	return 0;
}

static void tpo_touch_close(struct input_dev *idev)
{
	return;
}

static void tpo_send_event_workaround(unsigned long p __maybe_unused)
{
	schedule_work(&tpo_ts_work);
}

static int __devinit tpo_touch_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	int ret;

	g_client = client;

	ret = tpo_touch_recieve_data();
	if (ret < 0) {
		printk(KERN_WARNING "tpo_touch unavailable!\n");
		g_client = NULL;
		return -ENXIO;
	} else {
		printk(KERN_INFO "tpo_touch(chip id:0x%02x) detected.\n", tpo_buf[0]);
	}

	/* register input device */
	tpo_ts_input_dev = input_allocate_device();
	if (tpo_ts_input_dev == NULL) {
		printk(KERN_ERR "%s: failed to allocate input dev\n",
				__FUNCTION__);
		return -ENOMEM;
	}

	tpo_ts_input_dev->name = "tpo-ts";
	tpo_ts_input_dev->phys = "tpo-ts/input0";
	tpo_ts_input_dev->open = tpo_touch_open;
	tpo_ts_input_dev->close = tpo_touch_close;

	__set_bit(EV_ABS, tpo_ts_input_dev->evbit);
	__set_bit(ABS_X, tpo_ts_input_dev->absbit);
	__set_bit(ABS_Y, tpo_ts_input_dev->absbit);
	__set_bit(ABS_PRESSURE, tpo_ts_input_dev->absbit);

	__set_bit(EV_SYN, tpo_ts_input_dev->evbit);
	__set_bit(EV_KEY, tpo_ts_input_dev->evbit);
	__set_bit(BTN_TOUCH, tpo_ts_input_dev->keybit);

	input_set_abs_params(tpo_ts_input_dev, ABS_X, 0, 320, 0, 0);
	input_set_abs_params(tpo_ts_input_dev, ABS_Y, 0, 480, 0, 0);
	input_set_abs_params(tpo_ts_input_dev, ABS_PRESSURE, 0, 255, 0, 0);

	ret = input_register_device(tpo_ts_input_dev);
	if (ret) {
		printk(KERN_ERR
				"%s: unabled to register input device, ret = %d\n",
				__FUNCTION__, ret);
		return ret;
	}

	INIT_WORK(&tpo_ts_work, tpo_touch_work);

	init_timer(&tpo_timer);
	tpo_timer.function = tpo_send_event_workaround;
	tpo_timer.data = (long)NULL;

	ret = request_irq(client->irq, tpo_touch_irq_handler, IRQF_DISABLED | IRQF_TRIGGER_FALLING,
			"tpo touch", client);

	if (ret) {
		printk(KERN_WARNING "Request IRQ for Bigstream touch failed, return:%d\n",
				ret);
		return ret;
	}

#ifdef	CONFIG_PROC_FS
	create_tpo_touch_proc_file();
#endif
	return 0;
}

static int tpo_touch_remove(struct i2c_client *client)
{
#ifdef CONFIG_PROC_FS
	remove_tpo_touch_proc_file();
#endif
	input_unregister_device(tpo_ts_input_dev);
	return 0;
}

static const struct i2c_device_id tpo_touch_id[] = {
	{ "tpo_touch", 0 },
	{ }
};

static struct i2c_driver tpo_touch_driver = {
	.driver = {
		.name	= "tpo_touch",
	},
	.id_table 	= tpo_touch_id,
	.probe		= tpo_touch_probe,
	.remove		= tpo_touch_remove,
	.suspend	= tpo_touch_suspend,
	.resume		= tpo_touch_resume,
};

static int __init tpo_touch_init(void)
{
	return i2c_add_driver(&tpo_touch_driver);
}

static void __exit tpo_touch_exit(void)
{
	i2c_del_driver(&tpo_touch_driver);
}

module_init(tpo_touch_init);
module_exit(tpo_touch_exit);

MODULE_DESCRIPTION("TPO touch Driver");
MODULE_LICENSE("GPL");

