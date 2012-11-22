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
#include <linux/earlysuspend.h>
#include <linux/i2c/elan_touch.h>

struct elan_touch {
	struct input_dev	*idev;
	struct i2c_client	*i2c;
	struct work_struct	work;
	struct elan_touch_platform_data *data;
	int pen_status;
	int irq;
};
static struct elan_touch *touch;

#define ELAN_LEN 9
static u8 elan_buf[ELAN_LEN];

#define ELAN_PEN_UP	0
#define ELAN_PEN_DOWN	1

static u8 elong_cmd[4] = { 0x53, 0x00, 0x00, 0x01 };
/*static u8 elong_mode_cmd_sensitivity[4] = { 0x54, 0x40, 0xff, 0x01 };*/
static u8 elong_mode_cmd_idle[4] = { 0x54, 0xdc, 0x00, 0x01 };
static u8 elong_reset[4] = { 0x77, 0x77, 0x77, 0x77 };
static u8 elong_mode_cmd_sleep[4] = { 0x54, 0x50, 0x00, 0x01 };

int elan_touch_read_reg(u8 reg, u8 * pval)
{
	int ret;
	int status;

	if (touch->i2c == NULL)
		return -1;
	ret = i2c_smbus_read_byte_data(touch->i2c, reg);
	if (ret >= 0) {
		*pval = ret;
		status = 0;
	} else {
		status = -EIO;
	}

	return status;
}

int elan_touch_write_reg(u8 reg, u8 val)
{
	int ret;
	int status;

	if (touch->i2c == NULL)
		return -1;
	ret = i2c_smbus_write_byte_data(touch->i2c, reg, val);
	if (ret == 0)
		status = 0;
	else
		status = -EIO;

	return status;
}

static int elan_touch_read(char *buf, int count)
{
	int ret;

	ret = i2c_master_recv(touch->i2c, (char *) buf, count);

	return ret;
}

static int elan_touch_write(char *buf, int count)
{
	int ret;

	ret = i2c_master_send(touch->i2c, buf, count);

	return ret;
}

static int elan_touch_recieve_data(void)
{
	return elan_touch_read(elan_buf, ELAN_LEN);
}

static void elan_touch_work(struct work_struct *work)
{
	u16 tem_x1 = 0;
	u16 tem_y1 = 0;
	u16 tem_x2 = 0;
	u16 tem_y2 = 0;
	u16 width1 = 0;
	u16 width2 = 0;
	u8 tmp;

	if (elan_touch_recieve_data() < 0) {
		printk(KERN_ERR "%s: failed to receive data", __func__);
		return;
	}
	if (elan_buf[0] == 0x55 && elan_buf[1] == 0x55 &&
	    elan_buf[2] == 0x55 && elan_buf[3] == 0x55) {
		printk(KERN_INFO "elan_touch detected ok.\n");
		elan_touch_write(elong_mode_cmd_idle, 4);
		return;
	}

	if (elan_buf[0] == 0x52) {
		int i;
		if (elan_buf[1] == 0) {
			printk(KERN_INFO
			       "ElanTouch firmware version:0x%x-0x%x\n",
			       elan_buf[2], elan_buf[3]);
		} else {
			for (i = 0; i < 4; i++) {
				if (i == 3)
					printk("0x%x \n", elan_buf[i]);
				else
					printk("0x%x ", elan_buf[i]);
			}
		}

		return;
	}

	tem_x1 = ((u16) (elan_buf[1] & 0xF0) << 4) | (u16) elan_buf[2];
	tem_y1 = ((u16) (elan_buf[1] & 0x0F) << 8) | (u16) elan_buf[3];
	tem_x2 = ((u16) (elan_buf[4] & 0xF0) << 4) | (u16) elan_buf[5];
	tem_y2 = ((u16) (elan_buf[4] & 0x0F) << 8) | (u16) elan_buf[6];
	width1 = (u16) (elan_buf[7] & 0x0F);
	width2 = ((u16) (elan_buf[7] & 0xF0) >> 4);
	tmp = elan_buf[8];
	if (tmp == 0x3) {
		/* One finger */
		if (touch->pen_status == ELAN_PEN_UP)
			touch->pen_status = ELAN_PEN_DOWN;
		tem_y1 = 704 - tem_y1;
		input_report_abs(touch->idev, ABS_MT_TRACKING_ID,
			0);
		input_report_abs(touch->idev, ABS_PRESSURE, 255);
		input_report_abs(touch->idev, ABS_X, tem_x1);
		input_report_abs(touch->idev, ABS_Y, tem_y1);
		input_report_abs(touch->idev, ABS_MT_TOUCH_MAJOR, width1);
		input_report_abs(touch->idev, ABS_MT_POSITION_X, tem_x1);
		input_report_abs(touch->idev, ABS_MT_POSITION_Y, tem_y1);
		input_report_key(touch->idev, BTN_TOUCH, 1);
		input_mt_sync(touch->idev);
		input_sync(touch->idev);
	} else if (tmp == 0x5) {
		/* Two fingers */
		if (touch->pen_status == ELAN_PEN_UP)
			touch->pen_status = ELAN_PEN_DOWN;

		tem_y1 = 704 - tem_y1;
		input_report_abs(touch->idev, ABS_MT_TRACKING_ID,
			0);
		input_report_abs(touch->idev, ABS_PRESSURE, 255);
		input_report_abs(touch->idev, ABS_X, tem_x1);
		input_report_abs(touch->idev, ABS_Y, tem_y1);
		input_report_abs(touch->idev, ABS_MT_TOUCH_MAJOR, width1);
		input_report_abs(touch->idev, ABS_MT_POSITION_X, tem_x1);
		input_report_abs(touch->idev, ABS_MT_POSITION_Y, tem_y1);
		input_report_key(touch->idev, BTN_TOUCH, 1);
		input_mt_sync(touch->idev);

		tem_y2 = 704 - tem_y2;
		input_report_abs(touch->idev, ABS_MT_TRACKING_ID,
			1);
		input_report_abs(touch->idev, ABS_PRESSURE, 255);
		input_report_abs(touch->idev, ABS_X, tem_x2);
		input_report_abs(touch->idev, ABS_Y, tem_y2);
		input_report_abs(touch->idev, ABS_MT_TOUCH_MAJOR, width2);
		input_report_abs(touch->idev, ABS_MT_POSITION_X, tem_x2);
		input_report_abs(touch->idev, ABS_MT_POSITION_Y, tem_y2);
		input_report_key(touch->idev,  BTN_2, 1);
		input_mt_sync(touch->idev);

		input_sync(touch->idev);
	} else if (tmp == 0x01) {
		/* No finger */
		if (touch->pen_status == ELAN_PEN_DOWN)
			touch->pen_status = ELAN_PEN_UP;

		input_report_abs(touch->idev, ABS_PRESSURE,
			0);
		input_report_key(touch->idev, BTN_TOUCH, 0);
		input_report_key(touch->idev, BTN_2, 0);
		input_report_abs(touch->idev, ABS_MT_TOUCH_MAJOR, 0);
		input_sync(touch->idev);
	}
}

static irqreturn_t elan_touch_irq_handler(int irq, void *dev_id)
{
	schedule_work(&touch->work);
	return IRQ_HANDLED;
}

#ifdef	CONFIG_PM
static int elan_touch_suspend(struct i2c_client *client,
			      pm_message_t state)
{
	return 0;
}

static int elan_touch_resume(struct i2c_client *client)
{
	return 0;
}
#else
#define	elan_touch_suspend		NULL
#define	elan_touch_resume		NULL
#endif

#ifdef CONFIG_PROC_FS
#define	ELAN_TOUCH_PROC_FILE	"driver/elan_touch"
static struct proc_dir_entry *elan_touch_proc_file;
static int index;

static ssize_t elan_touch_proc_read(struct file *filp,
				    char *buffer, size_t length,
				    loff_t * offset)
{
	u8 reg_val;

	if ((index < 0) || (index > ELAN_LEN))
		return 0;

	elan_touch_read_reg(index, &reg_val);
	printk(KERN_INFO "register 0x%x: 0x%x\n", index, reg_val);
	return 0;
}

static ssize_t elan_touch_proc_write(struct file *filp,
				     const char *buff, size_t len,
				     loff_t * off)
{
	char messages[256], vol[256] = { 0 };

	int  reg = 0, i;

	if (len > 256)
		len = 256;

	if (copy_from_user(messages, buff, len))
		return -EFAULT;

	if ('w' == messages[0]) {
		elong_cmd[0] = 0x54;
		for (i = 0; i < 2; i++) {
			memcpy(vol, messages + i * 4 + 1, 4);
			reg = (int) simple_strtoul(vol, NULL, 16);
			printk("reg:0x%x\n", reg);
			elong_cmd[i + 1] = reg;
		}
		elan_touch_write(elong_cmd, 4);
	} else if ('r' == messages[0]) {
		memcpy(vol, messages + 1, len - 1);
		elong_cmd[0] = 0x53;
		reg = (int) simple_strtoul(vol, NULL, 16);
		elong_cmd[1] = reg;
		printk("reg=0x%x \n", reg);
		elan_touch_write(elong_cmd, 4);
	} else if ('d' == messages[0]) {
		elong_cmd[0] = 0x53;
		elong_cmd[1] = 0x00;
		elan_touch_write(elong_cmd, 4);
		msleep(2);
		for (i = 0x20; i <= 0x60; i = i + 0x10) {
			elong_cmd[1] = i;
			elan_touch_write(elong_cmd, 4);
			msleep(2);
		}
		elong_cmd[1] = 0x63;
		elan_touch_write(elong_cmd, 4);
		msleep(2);
		elong_cmd[1] = 0x61;
		elan_touch_write(elong_cmd, 4);
		msleep(2);
		elong_cmd[1] = 0x081;
		elan_touch_write(elong_cmd, 4);
		msleep(2);
		elong_cmd[1] = 0x82;
		elan_touch_write(elong_cmd, 4);
		msleep(2);
		for (i = 0xd0; i <= 0xf0; i = i + 0x10) {
			elong_cmd[1] = i;
			elan_touch_write(elong_cmd, 4);
			msleep(2);
		}
	}
	return len;
}

static struct file_operations elan_touch_proc_ops = {
	.read = elan_touch_proc_read,
	.write = elan_touch_proc_write,
};

static void create_elan_touch_proc_file(void)
{
	elan_touch_proc_file =
	    create_proc_entry(ELAN_TOUCH_PROC_FILE, 0644, NULL);
	if (elan_touch_proc_file) {
		elan_touch_proc_file->proc_fops = &elan_touch_proc_ops;
	} else
		printk(KERN_INFO "proc file create failed!\n");
}

static void remove_elan_touch_proc_file(void)
{
	extern struct proc_dir_entry proc_root;
	remove_proc_entry(ELAN_TOUCH_PROC_FILE, &proc_root);
}
#endif

static int elan_touch_open(struct input_dev *idev)
{
	touch->data->power(1);
	return 0;
}

static void elan_touch_close(struct input_dev *idev)
{
	touch->data->power(0);
	return;
}

static void elan_touch_sleep_early_suspend(struct early_suspend *h)
{
	int ret, i = 0;

sleep_retry:
	ret = elan_touch_write(elong_mode_cmd_sleep, 4);
	if (ret < 0) {
		if (i < 50) {
			msleep(5);
			i++;
			printk(KERN_WARNING
			       "elan_touch can't enter sleep,retry %d\n",
			       i);
			goto sleep_retry;
		}
		printk(KERN_WARNING "elan_touch can't enter sleep\n");
		return;
	} else {
		printk(KERN_INFO "elan_touch enter sleep mode.\n");
	}
	touch->data->power(0);
}

static void elan_touch_normal_late_resume(struct early_suspend *h)
{
	int ret, i = 0;

	touch->data->power(1);
	msleep(10);
	
reset_retry:
	ret = elan_touch_write(elong_reset, 4);
	if (ret < 0) {
		if (i < 50) {
			msleep(5);
			i++;
			printk(KERN_WARNING
			       "elan_touch reset failed,retry %d\n", i);
			goto reset_retry;
		}
		printk(KERN_WARNING "elan_touch reset failed\n");
		return;
	} else {
		printk(KERN_INFO "elan_touch reset successful.\n");
	}
}

static struct early_suspend elan_touch_early_suspend_desc = {
	.level = EARLY_SUSPEND_LEVEL_STOP_DRAWING,
	.suspend = elan_touch_sleep_early_suspend,
	.resume = elan_touch_normal_late_resume,
};


static int __devinit elan_touch_probe(struct i2c_client *client,
				      const struct i2c_device_id *id)
{
	int ret;

	touch = kzalloc(sizeof(struct elan_touch), GFP_KERNEL);
	if (touch == NULL)
		return -ENOMEM;

	touch->data = client->dev.platform_data;
	if (touch->data == NULL) {
		dev_dbg(&client->dev, "no platform data\n");
		return -EINVAL;
	}
	touch->i2c = client;
	touch->irq = client->irq;
	touch->pen_status = ELAN_PEN_UP;
	touch->data->power(1);
	ret = elan_touch_write(elong_reset, 4);
	if (ret < 0) {
		printk(KERN_WARNING "elan detect fail!\n");
		touch->i2c = NULL;
		return -ENXIO;
	} else {
		printk(KERN_INFO "elan_touch reset.\n");
	}

	/* register input device */
	touch->idev = input_allocate_device();
	if (touch->idev == NULL) {
		printk(KERN_ERR "%s: failed to allocate input dev\n",
		       __FUNCTION__);
		ret = -ENOMEM;
		goto out;
	}

	touch->idev->name = "elan-ts";
	touch->idev->phys = "elan-ts/input0";
	touch->idev->open = elan_touch_open;
	touch->idev->close = elan_touch_close;

	__set_bit(EV_ABS, touch->idev->evbit);
	__set_bit(ABS_X, touch->idev->absbit);
	__set_bit(ABS_Y, touch->idev->absbit);
	__set_bit(ABS_MT_POSITION_X, touch->idev->absbit);
	__set_bit(ABS_MT_POSITION_Y, touch->idev->absbit);
	__set_bit(ABS_PRESSURE, touch->idev->absbit);

	__set_bit(EV_SYN, touch->idev->evbit);
	__set_bit(EV_KEY, touch->idev->evbit);
	__set_bit(BTN_TOUCH, touch->idev->keybit);
	__set_bit(BTN_2, touch->idev->keybit);

	input_set_abs_params(touch->idev, ABS_MT_TOUCH_MAJOR, 0, 16, 0, 0);
	input_set_abs_params(touch->idev, ABS_X, 0, 448, 0, 0);
	input_set_abs_params(touch->idev, ABS_Y, 0, 704, 0, 0);
	input_set_abs_params(touch->idev, ABS_PRESSURE, 0, 255, 0, 0);

	/*   muti touch */
	input_set_abs_params(touch->idev, ABS_MT_POSITION_X, 0, 448, 0, 0);
	input_set_abs_params(touch->idev, ABS_MT_POSITION_Y, 0, 704, 0, 0);

	ret = input_register_device(touch->idev);
	if (ret) {
		printk(KERN_ERR
		       "%s: unabled to register input device, ret = %d\n",
		       __FUNCTION__, ret);
		goto out_rg;
	}

	ret = request_irq(touch->irq, elan_touch_irq_handler,
			  IRQF_DISABLED | IRQF_TRIGGER_FALLING,
			  "elan touch", touch);
	if (ret < 0)
		goto out_irg;

	if (ret) {
		printk(KERN_WARNING
		       "Request IRQ for Bigstream touch failed, return:%d\n",
		       ret);
		goto out_rg;
	}
	INIT_WORK(&touch->work, elan_touch_work);
	register_early_suspend(&elan_touch_early_suspend_desc);

#ifdef	CONFIG_PROC_FS
	create_elan_touch_proc_file();
#endif

	return 0;

out_irg:
	free_irq(touch->irq, touch);
out_rg:
	input_free_device(touch->idev);
out:
	kfree(touch);
	return ret;

}

static int elan_touch_remove(struct i2c_client *client)
{
#ifdef CONFIG_PROC_FS
	remove_elan_touch_proc_file();
#endif
	input_unregister_device(touch->idev);
	return 0;
}

static const struct i2c_device_id elan_touch_id[] = {
	{"elan_touch", 0},
	{}
};

static struct i2c_driver elan_touch_driver = {
	.driver = {
		   .name = "elan_touch",
		   },
	.id_table = elan_touch_id,
	.probe = elan_touch_probe,
	.remove = elan_touch_remove,
	.suspend = elan_touch_suspend,
	.resume = elan_touch_resume,
};

static int __init elan_touch_init(void)
{
	return i2c_add_driver(&elan_touch_driver);
}

static void __exit elan_touch_exit(void)
{
	unregister_early_suspend(&elan_touch_early_suspend_desc);
	i2c_del_driver(&elan_touch_driver);
}

module_init(elan_touch_init);
module_exit(elan_touch_exit);

MODULE_DESCRIPTION("ELAN touch Driver");
MODULE_LICENSE("GPL");

