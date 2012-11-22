/*
 * Driver for the enhanced rotary controller on pxa930 and pxa935
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/input.h>
#include <linux/platform_device.h>
#include <linux/proc_fs.h>
#include <linux/earlysuspend.h>
#include <linux/i2c.h>
#include <plat/mfp.h>
#include <linux/gpio.h>
#include <mach/irqs.h>
#include <linux/uaccess.h>

#define OTP_INTTERUPT_GPIO			MFP_PIN_GPIO135
#define OTP_POWER_DOWN_GPIO			MFP_PIN_GPIO144
#define OTP_RESET_GPIO				MFP_PIN_GPIO136
#define OTP_OK_BTN_GPIO				MFP_PIN_GPIO8

#define OTP_USE_KEYS
/*reg and bit definisions*/
#define OTP_PRODUCT_ID_REG			(0x00)
#define OTP_REVISION_ID_REG			(0x01)
#define OTP_MOTION_REG				(0x02)
#define OPT_MOTION_OVF_BIT			(1<<4)
#define OTP_MOTION_MOT_BIT			(1<<7)
#define OTP_DELTA_X_REG				(0x03)
#define OTP_DELTA_Y_REG				(0x04)
#define OTP_SOFT_RESET_REG			(0x3A)
#define OTP_SOFT_RESET_BITMASK		(0x5A)
#define OTP_OJ_ORIENT_CTRL_REG		(0x77)
#define OPT_OJ_ORIENT_X_SWAP_BIT	(1<<5)
#define OPT_OJ_ORIENT_Y_SWAP_BIT	(1<<6)
#define OPT_OJ_ORIENT_XY_SWAP_BIT	(1<<7)


#define MIN_OFFSET_RIGHT		5
#define MIN_OFFSET_LEFT			(-MIN_OFFSET_RIGHT)
#define MIN_OFFSET_DOWN			5
#define MIN_OFFSET_UP			(-MIN_OFFSET_DOWN)

#define OTP_MAX_Y_VALUE			(400)
#define OTP_MIN_Y_VALUE			(-400)
#define OTP_MAX_X_VALUE			(240)
#define OTP_MIN_X_VALUE			(-240)

struct optic_tp_data {
	struct i2c_client	*client;
	struct input_dev	*input_dev;
	int					abs_x;
	int					abs_y;
	int					i_step;
};

struct optic_tp_data	*g_p_optic_tp_data/* = NULL*/;

/*i2c related functions */
static int optic_tp_i2c_write(u8 reg, u16 val)
{
	int ret;
	ret = i2c_smbus_write_byte_data(g_p_optic_tp_data->client, reg, val);
	if (ret < 0)
		dev_dbg(&(g_p_optic_tp_data->input_dev->dev), "failed to write to reg 0x%x, rc %d", reg, ret);
	return ret;
}

static int optic_tp_i2c_read_byte(u8 reg)
{
	return i2c_smbus_read_byte_data(g_p_optic_tp_data->client, reg);
}

static int optic_tp_i2c_read_burst(u8 reg, u8 *buff, int sz)
{
	int rc;
	rc = i2c_smbus_read_i2c_block_data(g_p_optic_tp_data->client,
		reg, sz, buff);
	if (rc != sz)
		printk(KERN_ERR"ctec OTP read burst failed to read all bytes\n");
	return rc;
}
/*end of i2c functions*/


#ifdef CONFIG_HAS_EARLYSUSPEND
static int optic_tp_resume(struct platform_device *pdev);
static int optic_tp_suspend(struct platform_device *pdev, pm_message_t state);

static void optic_tp_early_suspend(struct early_suspend *h)
{
	pm_message_t t;
	t.event = 0;
	if (optic_tp_suspend != NULL)
		optic_tp_suspend(NULL, t);
}
static void optic_tp_late_resume(struct early_suspend *h)
{
	if (optic_tp_resume != NULL)
		optic_tp_resume(NULL);
}

static struct early_suspend optic_tp_early_suspend_desc = {
	.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1,
	.suspend = optic_tp_early_suspend,
	.resume = optic_tp_late_resume,
};

static int optic_tp_resume(struct platform_device *pdev)
{
	dev_dbg(&(g_p_optic_tp_data->input_dev->dev), "++optic_tp_resume\n");
	/*enable_irq(g_p_optic_tp_data->client->irq);*/
	dev_dbg(&(g_p_optic_tp_data->input_dev->dev), "--optic_tp_resume\n");
	return 0;
}

static int optic_tp_suspend(struct platform_device *pdev, pm_message_t state)
{
	dev_dbg(&(g_p_optic_tp_data->input_dev->dev), "++optic_tp_suspend\n");
	/*disable_irq(g_p_optic_tp_data->client->irq);*/
	dev_dbg(&(g_p_optic_tp_data->input_dev->dev), "--optic_tp_suspend\n");
	return 0;
}
#endif

static int optic_tp_reset(void)
{
	int ret = 0;
	unsigned rst_gpio;
	unsigned pdown_gpio;
	unsigned inr_gpio;

	mfp_cfg_t config[] = {
		OTP_INTTERUPT_GPIO | MFP_LPM_EDGE_FALL,
		OTP_OK_BTN_GPIO | MFP_PULL_HIGH | MFP_LPM_EDGE_BOTH,
	};

	dev_dbg(&(g_p_optic_tp_data->input_dev->dev), "++optic_tp_reset\n");

	rst_gpio = mfp_to_gpio(OTP_RESET_GPIO);
	inr_gpio = mfp_to_gpio(OTP_INTTERUPT_GPIO);
	pdown_gpio = mfp_to_gpio(OTP_POWER_DOWN_GPIO);

	dev_dbg(&(g_p_optic_tp_data->input_dev->dev), "rst_gpio value is %d", rst_gpio);

	if (gpio_request(rst_gpio, "optic_tp_reset")) {
		printk(KERN_ERR "Request GPIO failed,"
			"gpio: %d\n", rst_gpio);
		return -EIO;
	}
	if (gpio_request(inr_gpio, "optic_tp_interrupt")) {
		printk(KERN_ERR "Request GPIO failed,"
			"gpio: %d\n", inr_gpio);
		gpio_free(rst_gpio);
		return -EIO;
	}
	if (gpio_request(pdown_gpio, "optic_tp_power_down")) {
		printk(KERN_ERR "Request GPIO failed,"
			"gpio: %d\n", pdown_gpio);
		gpio_free(rst_gpio);
		gpio_free(inr_gpio);
		return -EIO;
	}
	/*set interrupt as input*/
	gpio_direction_input(inr_gpio);

	/*set as output and level logic high*/
	gpio_direction_output(rst_gpio, 1);

	/*set as output and level logic low*/
	gpio_direction_output(pdown_gpio, 0);

	mfp_config(config, ARRAY_SIZE(config));

	gpio_free(rst_gpio);
	gpio_free(inr_gpio);
	gpio_free(pdown_gpio);
	dev_dbg(&(g_p_optic_tp_data->input_dev->dev), "--optic_tp_reset\n");
	return ret;
}

static void optic_tp_print_id(void)
{
	u16 value;
	u8	reg;
	/*print ID*/
	reg = OTP_PRODUCT_ID_REG;
	value = optic_tp_i2c_read_byte(reg);
	printk(KERN_INFO"optic track pad product ID is %d\n", value);

	reg = OTP_REVISION_ID_REG;
	value = optic_tp_i2c_read_byte(reg);
	printk(KERN_INFO"optic track pad revision ID is %d\n", value);
}
static int optic_tp_init(void)
{
	u16	value;
	u8	reg;

	/*perform soft reset by writing 0x5A to address 0x3a*/
	reg = OTP_SOFT_RESET_REG;
	value = OTP_SOFT_RESET_BITMASK;
	optic_tp_i2c_write(reg , value);

	/*Write 0xC0 to address 0x60*/
	reg = 0x60;
	value = 0xC0;
	optic_tp_i2c_write(reg, value);

	/*set oriantation by writing 0x80 to reg 0x77 (OTP_OJ_ORIENT_CTRL_REG)*/
	reg = OTP_OJ_ORIENT_CTRL_REG;
	value = OPT_OJ_ORIENT_X_SWAP_BIT;
	optic_tp_i2c_write(reg, value);

	/*write 0xff to reg 0x02 (OTP_MOTION_REG) to clear it*/
	value = 0xff;
	reg = OTP_MOTION_REG;
	optic_tp_i2c_write(reg, value);

	optic_tp_print_id();
	return 0;
}

void send_move(int key)
{
	input_report_key(g_p_optic_tp_data->input_dev,
		key, 1);/*press*/
	input_sync(g_p_optic_tp_data->input_dev);
	input_report_key(g_p_optic_tp_data->input_dev,
		key, 0);/*release*/
	input_sync(g_p_optic_tp_data->input_dev);
}

static void do_move(int x_delta, int y_delta)
{
	dev_dbg(&(g_p_optic_tp_data->input_dev->dev), "x = %d, y = %d\n", x_delta, y_delta);
#if defined(OTP_USE_KEYS)
	g_p_optic_tp_data->abs_x += x_delta;
	g_p_optic_tp_data->abs_y += y_delta;
	if (g_p_optic_tp_data->abs_x > g_p_optic_tp_data->i_step) {
		send_move(KEY_RIGHT);
		g_p_optic_tp_data->abs_x = 0;
	} else if (g_p_optic_tp_data->abs_x < -g_p_optic_tp_data->i_step) {
		send_move(KEY_LEFT);
		g_p_optic_tp_data->abs_x = 0;
	}
	if (g_p_optic_tp_data->abs_y > g_p_optic_tp_data->i_step) {
		send_move(KEY_DOWN);
		g_p_optic_tp_data->abs_y = 0;
	} else if (g_p_optic_tp_data->abs_y < -g_p_optic_tp_data->i_step) {
		send_move(KEY_UP);
		g_p_optic_tp_data->abs_y = 0;
	}
#else
	dev_dbg(&(g_p_optic_tp_data->input_dev->dev), "rel event\n");
	input_report_rel(g_p_optic_tp_data->input_dev,
		REL_X, x_delta);
	input_report_rel(g_p_optic_tp_data->input_dev,
		REL_Y, y_delta);
	g_p_optic_tp_data->abs_x += x_delta;
	if (g_p_optic_tp_data->abs_x > OTP_MAX_X_VALUE)
		g_p_optic_tp_data->abs_x = OTP_MAX_X_VALUE;
	if (g_p_optic_tp_data->abs_x < OTP_MIN_X_VALUE)
		g_p_optic_tp_data->abs_x = OTP_MIN_X_VALUE;
	g_p_optic_tp_data->abs_y += y_delta;
	if (g_p_optic_tp_data->abs_y > OTP_MAX_Y_VALUE)
		g_p_optic_tp_data->abs_y = OTP_MAX_Y_VALUE;
	if (g_p_optic_tp_data->abs_y < OTP_MIN_Y_VALUE)
		g_p_optic_tp_data->abs_y = OTP_MIN_Y_VALUE;

	dev_dbg(&(g_p_optic_tp_data->input_dev->dev), "abs x %d, abs y %d\n",
		g_p_optic_tp_data->abs_x,
		g_p_optic_tp_data->abs_y);
	input_report_abs(g_p_optic_tp_data->input_dev,
		ABS_X, g_p_optic_tp_data->abs_x);
	input_report_abs(g_p_optic_tp_data->input_dev,
		ABS_Y, g_p_optic_tp_data->abs_y);
	input_sync(g_p_optic_tp_data->input_dev);
#endif
}

static irqreturn_t optic_tp_irq_handler(int irq, void *dev_id)
{
	int status;
	/*char reg;*/
	char tmp;
	int x_move, y_move;

	status = optic_tp_i2c_read_byte(OTP_MOTION_REG);
	if ((status & OTP_MOTION_MOT_BIT) != OTP_MOTION_MOT_BIT)
		return IRQ_HANDLED;
	tmp = optic_tp_i2c_read_byte(OTP_DELTA_X_REG);
	x_move = (signed char)tmp;
	tmp = optic_tp_i2c_read_byte(OTP_DELTA_Y_REG);
	y_move = (signed char)tmp;
	do_move(x_move, y_move);
	/* this should clear the interrupts*/
	optic_tp_i2c_write(OTP_MOTION_REG, 0xff);

	return IRQ_HANDLED;
}

static int optic_tp_proc_write(struct file *file, const char __user *buffer,
						unsigned long count, void *data)
{
	static char kbuf[1024];
	int tmp;

	if (count >= 1024)
		return -EINVAL;
	if (copy_from_user(kbuf, buffer, count))
		return -EFAULT;
	if ('?' == kbuf[0]) {
		printk(KERN_INFO"\nHowTo: echo [dbgon|dbgoff|p|step] > /proc/driver/optic_tp\n");
		printk(KERN_INFO" p -\tprint version\n");
	} else if (strncmp(kbuf, "p", 1) == 0) {
		printk(KERN_INFO "\nprint version\n");
		optic_tp_print_id();
	} else if (strncmp(kbuf, "step", 4) == 0) {
		printk(KERN_INFO "change stepping TH\n");
		tmp = (int)strict_strtoul(kbuf+4, NULL, 10);
		if (tmp > 0 && tmp < 1000) {
			g_p_optic_tp_data->i_step = tmp;
			printk(KERN_INFO "changing stepping TH to %d\n", tmp);
		}
	} else {
		printk(KERN_ERR "unknown command\n");
	}
	return count;
}

static int __devinit
optic_tp_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	unsigned long flags = IRQF_TRIGGER_FALLING | IRQF_ONESHOT;
	struct proc_dir_entry *optic_tp_proc_entry;
	int ret;

	dev_dbg(&(g_p_optic_tp_data->input_dev->dev), "++optic_tp_probe\n");
	if (g_p_optic_tp_data != NULL) {
		printk(KERN_ERR"called optic_tp_probe but device data is not NULL!!");
		return -ENOMEM;
	}
	g_p_optic_tp_data = kzalloc(sizeof(struct optic_tp_data), GFP_KERNEL);
	if (!g_p_optic_tp_data)
		return -ENOMEM;

	/*init optic_tp_data struct*/
	g_p_optic_tp_data->client = client;
	g_p_optic_tp_data->abs_x = 0;
	g_p_optic_tp_data->abs_y = 0;
	g_p_optic_tp_data->i_step = 80;

	i2c_set_clientdata(client, g_p_optic_tp_data);

	ret = optic_tp_reset();
	if (ret < 0) {
		printk(KERN_ERR"failed to reset optic_track pad\n");
		goto err_no_dev;
	}

	ret = optic_tp_init();
	if (ret < 0) {
		printk(KERN_ERR"failed to init optic_track pad\n");
		goto err_no_dev;
	}

	g_p_optic_tp_data->input_dev = input_allocate_device();
	if (g_p_optic_tp_data->input_dev == NULL) {
		ret = -ENOMEM;
		printk(KERN_ERR"optic_tp_probe: failed to allocate input device\n");
		goto err_dev_alloc;
	}
	g_p_optic_tp_data->input_dev->name = "optic_tp";
#if defined(OTP_USE_KEYS)
	__set_bit(KEY_DOWN, g_p_optic_tp_data->input_dev->keybit);
	__set_bit(KEY_UP, g_p_optic_tp_data->input_dev->keybit);
	__set_bit(KEY_LEFT, g_p_optic_tp_data->input_dev->keybit);
	__set_bit(KEY_RIGHT, g_p_optic_tp_data->input_dev->keybit);
	__set_bit(EV_KEY, g_p_optic_tp_data->input_dev->evbit);
#else
	input_set_abs_params(g_p_optic_tp_data->input_dev,
		ABS_X, -240, 240, 0, 0);
	input_set_abs_params(g_p_optic_tp_data->input_dev,
		ABS_Y, -400, 400, 0, 0);
	__set_bit(ABS_X, g_p_optic_tp_data->input_dev->absbit);
	__set_bit(ABS_Y, g_p_optic_tp_data->input_dev->absbit);
	__set_bit(REL_X, g_p_optic_tp_data->input_dev->relbit);
	__set_bit(REL_Y, g_p_optic_tp_data->input_dev->relbit);
	__set_bit(EV_REL, g_p_optic_tp_data->input_dev->evbit);
#endif
	ret = input_register_device(g_p_optic_tp_data->input_dev);
	if (ret) {
		printk(KERN_ERR"optic_tp_probe: Unable to register input device\n");
		ret = -EXDEV;
		goto err_register_dev;
	}

	/*register irq*/
	g_p_optic_tp_data->client->irq = IRQ_GPIO(mfp_to_gpio(OTP_INTTERUPT_GPIO));
	dev_dbg(&(g_p_optic_tp_data->input_dev->dev), "received IRQ # %d\n", g_p_optic_tp_data->client->irq);

	ret = request_threaded_irq(g_p_optic_tp_data->client->irq,
		NULL, optic_tp_irq_handler,
		flags, "ctec_optic_tp", g_p_optic_tp_data);
	if (ret)
		printk(KERN_ERR"ssd2531 probe: failed to register to IRQ\n");

	optic_tp_proc_entry = create_proc_entry("driver/optic_tp", 0, NULL);
	if (optic_tp_proc_entry)
		optic_tp_proc_entry->write_proc = optic_tp_proc_write;
#ifdef CONFIG_HAS_EARLYSUSPEND
	register_early_suspend(&optic_tp_early_suspend_desc);
#endif

	printk(KERN_NOTICE"--optic_tp_probe - exit OK\n");
	return 0;

err_register_dev:
	input_free_device(g_p_optic_tp_data->input_dev);
	g_p_optic_tp_data->input_dev = NULL;
err_dev_alloc:
	/*release IRQ?*/
err_no_dev:
	if (g_p_optic_tp_data != NULL)
		kfree(g_p_optic_tp_data);
	g_p_optic_tp_data = NULL;
	printk(KERN_NOTICE"--optic_tp_probe - exit ERROR\n");
	return ret;
}

static int __devexit optic_tp_remove(struct i2c_client *client)
{
	struct optic_tp_data *otp = i2c_get_clientdata(client);
#ifdef CONFIG_HAS_EARLYSUSPEND
	unregister_early_suspend(&optic_tp_early_suspend_desc);
#endif
	free_irq(client->irq, otp);
	input_unregister_device(otp->input_dev);
	kfree(otp);
	g_p_optic_tp_data = NULL;
	return 0;
}

static const struct i2c_device_id optic_tp_i2c_id[] = {
	{ "ctec_optic_tp", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, optic_tp_i2c_id);

static struct i2c_driver optic_tp_i2c_driver = {
	.driver = {
		.name	= "ctec_optic_tp",
		.owner	= THIS_MODULE,
	},
	.probe		= optic_tp_probe,
	.remove		= __devexit_p(optic_tp_remove),
#ifndef CONFIG_HAS_EARLYSUSPEND
	.suspend	= optic_tp_suspend,
	.resume		= optic_tp_resume,
#endif
	.id_table	= optic_tp_i2c_id,
};

static int __init optic_tp_i2c_init(void)
{
	return i2c_add_driver(&optic_tp_i2c_driver);
}
module_init(optic_tp_i2c_init);

static void __exit optic_tp_i2c_exit(void)
{
	i2c_del_driver(&optic_tp_i2c_driver);
}
module_exit(optic_tp_i2c_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Driver for Crucial Tec optic track pad");
MODULE_AUTHOR("Chen Reichbach <creichba@marvell.com>");
