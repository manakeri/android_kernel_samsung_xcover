/*
 *  linux/drivers/input/touchscreen/SSD2531_touch.c
 *
 *  touch screen driver for Solomon systech SSD2531
 *
 *  Copyright (C) 2006, Marvell Corporation (fengwei.yin@Marvell.com)
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 */
#include <linux/module.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/workqueue.h>
#include <linux/wait.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/freezer.h>
#include <linux/proc_fs.h>
#include <linux/earlysuspend.h>

#include <asm/irq.h>
#include <mach/hardware.h>

#include <mach/regs-ost.h>
#include <mach/irqs.h>
#include <linux/uaccess.h>
#include <linux/i2c.h>
#include <plat/mfp.h>
#include <linux/gpio.h>

#define TOUCHSCREEN_X_AXIS_MIN				(0)
#define TOUCHSCREEN_X_AXIS_MAX				(479)
#define TOUCHSCREEN_Y_AXIS_MIN				(0)
#define TOUCHSCREEN_Y_AXIS_MAX				(800)
#define TOUCHSCREEN_Z_AXIS_MIN				(0)
#define TOUCHSCREEN_Z_AXIS_MAX				(0x0f)
#define TOUCHSCREEN_WIDTH_MIN				(0)
#define TOUCHSCREEN_WIDTH_MAX				(15)
#define TOUCHSCREEN_ID_MIN					(0)
#define TOUCHSCREEN_ID_MAX					(3)
#define TOUCHSCREEN__RESET_GPIO				MFP_PIN_GPIO70

/*////////////////////////////////////////////////////////////////////////*/
static int ssd2531_ts_reset(void);
static int ssd2531_ts_init(void);
static int ssd2531_ts_crl_all_events(void);

static int
 ssd2531_ts_proc_write(struct file *file, const char __user *buffer,
						unsigned long count, void *data)
{
	static char kbuf[1024];

	if (count >= 1024)
		return -EINVAL;
	if (copy_from_user(kbuf, buffer, count))
		return -EFAULT;

	if ('?' == kbuf[0]) {
		printk(KERN_INFO"\nHowTo: echo [dbgon|dbgoff|clr|rst] > /proc/driver/ssd2531_ts\n");
		printk(KERN_INFO" clr -\tclear pending events\n rst -\treset touch\n");
	} else if (strncmp(kbuf, "rst", 3) == 0) {
		printk(KERN_INFO "\nrunning reset\n");
		ssd2531_ts_reset();
		ssd2531_ts_init();
	} else if (strncmp(kbuf, "clr", 3) == 0) {
		printk(KERN_INFO "\nclearing all pending events\n");
		ssd2531_ts_crl_all_events();
	} else
		printk(KERN_ERR "unknown command\n");
	return count;
}

/*////////////////////////////////////////////////////////////////////////*/


/**************************************************************************/
/*SSD2531 i2c driver*/
#define MAX_NUM_OF_POINTS			4
#define POINT_DATA_SIZE				4
#define IS_FINGER_DOWN(i, status)	((0x01<<i)&(status))
/*////////////////////////////////////////////////////////////////////////*/
/*/ need to move to h file touch reg discription*/
#define SSD2531_BIT_0						((0x1)<<0)
#define SSD2531_BIT_1						((0x1)<<1)
#define SSD2531_BIT_2						((0x1)<<2)
#define SSD2531_BIT_3						((0x1)<<3)
#define SSD2531_BIT_4						((0x1)<<4)
#define SSD2531_BIT_5						((0x1)<<5)
#define SSD2531_BIT_6						((0x1)<<6)
#define SSD2531_BIT_7						((0x1)<<7)

#define SSD2531_SYSTEM_ENABLE_REG				0x23
#define SSD2531_SYSTEM_DISABLE_REG				0x24
#define SSD2531_EVENT_STATUS_REG				0x79
#define SSD2531_STATUS_FINGER_1_DETECTED_BIT	SSD2531_BIT_0
#define SSD2531_STATUS_FINGER_2_DETECTED_BIT	SSD2531_BIT_1
#define SSD2531_STATUS_FINGER_3_DETECTED_BIT	SSD2531_BIT_2
#define SSD2531_STATUS_FINGER_4_DETECTED_BIT	SSD2531_BIT_3
#define SSD2531_STATUS_FIFO_NOT_EMPTY_BIT		SSD2531_BIT_4
#define SSD2531_STATUS_FIFO_OVERFLOW_EMPTY_BIT	SSD2531_BIT_5
#define SSD2531_STATUS_LARGE_OBJECT_BIT			SSD2531_BIT_6
#define SSD2531_EVENT_MASK_REG					0x7A
#define SSD2531_IRQ_MASK_REG					0x7B

#define SSD2531_FINGER_1_REG					0x7C
#define SSD2531_FINGER_2_REG					0x7D
#define SSD2531_FINGER_3_REG					0x7E
#define SSD2531_FINGER_4_REG					0x7F

#define SSD2531_EVENT_STACK_REG					0x80
#define SSD2531_EVENT_STACK_CLEAR_REG			0x81



/*////////////////////////////////////////////////////////////////////////*/
struct finger_info {
	u16 x;
	u16 y;
	u16 weight;
	u16 f_id;
};

static struct ssd2531_ts_data {
	struct i2c_client	*client;
	struct input_dev	*input_dev;
	int					prev_finger_bitmask;
	struct finger_info	prev_finger_info[MAX_NUM_OF_POINTS];
	int					btn_info[MAX_NUM_OF_POINTS];
	int					b_is_suspended;
	int					mfp_gpio_pin_int;
	int					mfp_gpio_pin_reset;
/*	struct work_struct	work;*/
};

struct ssd2531_ts_data	*g_p_ssd2531_data;

static int ssd2531_i2c_read_byte(u8 reg)
{
	return i2c_smbus_read_byte_data(g_p_ssd2531_data->client, reg);
}
static int ssd2531_i2c_read_word(u8 reg)
{
	return i2c_smbus_read_word_data(g_p_ssd2531_data->client, reg);
}
static int ssd2531_i2c_read_burst(u8 reg, u8 *buff, int sz)
{
	int rc;
	rc = i2c_smbus_read_i2c_block_data(g_p_ssd2531_data->client,
							reg, sz, buff);
	if (rc != sz)
		printk(KERN_ERR"SSD2531 read burst failed to read all bytes\n");
	return rc;

}
static int ssd2531_i2c_write(u8 reg, u16 val)
{
	int ret;
	ret = i2c_smbus_write_byte_data(g_p_ssd2531_data->client, reg, val);
	dev_dbg(&(g_p_ssd2531_data->input_dev->dev),
		"ssd2531_i2c_w(reg=0x%x,val=0x%x)\n", reg, val);
	if (ret < 0)
		dev_dbg(&(g_p_ssd2531_data->input_dev->dev),
			"failed to write to reg 0x%x, rc %d", reg, ret);
	return ret;
}
/* Function for future use
static int ssd2531_i2c_write_word(u8 reg, u16 val)
{
	return i2c_smbus_write_word_data(g_p_ssd2531_data->client, reg, val);
}
*/

int ssd2531_ts_wakeup(int enable)
{
	u8 reg;
	printk(KERN_NOTICE"ssd2531_ts_wakeup");
	reg = ((enable == 1) ? SSD2531_SYSTEM_ENABLE_REG : SSD2531_SYSTEM_DISABLE_REG);
	return ssd2531_i2c_write(reg, 0x00);
}


#ifdef CONFIG_HAS_EARLYSUSPEND
static int ssd2531_ts_resume(struct platform_device *pdev);
static int ssd2531_ts_suspend(struct platform_device *pdev, pm_message_t state);

static void ssd2531_ts_early_suspend(struct early_suspend *h)
{
	pm_message_t t;
	t.event = 0;
	if (ssd2531_ts_suspend != NULL)
		ssd2531_ts_suspend(NULL, t);
}
static void ssd2531_ts_late_resume(struct early_suspend *h)
{
	if (ssd2531_ts_resume != NULL)
		ssd2531_ts_resume(NULL);
}

static struct early_suspend ssd2531_ts_early_suspend_desc = {
	.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1,
	.suspend = ssd2531_ts_early_suspend,
	.resume = ssd2531_ts_late_resume,
};

static int ssd2531_ts_resume(struct platform_device *pdev)
{
	dev_dbg(&(g_p_ssd2531_data->input_dev->dev), "++ssd2531_ts_resume\n");
/*	enable_irq(g_p_ssd2531_data->client->irq);*/
	if (g_p_ssd2531_data->b_is_suspended == 1) {
		ssd2531_ts_reset();
		ssd2531_ts_init();
		ssd2531_ts_crl_all_events();
	}
	g_p_ssd2531_data->b_is_suspended = 0;
	/*ssd2531_ts_wakeup(1);*/
	dev_dbg(&(g_p_ssd2531_data->input_dev->dev), "--ssd2531_ts_resume\n");
	return 0;
}

static int ssd2531_ts_suspend(struct platform_device *pdev, pm_message_t state)
{
	dev_dbg(&(g_p_ssd2531_data->input_dev->dev), "++ssd2531_ts_suspend\n");
	if (g_p_ssd2531_data->b_is_suspended == 0)
		ssd2531_ts_wakeup(0);
	g_p_ssd2531_data->b_is_suspended = 1;

	/*disable_irq(g_p_ssd2531_data->client->irq);*/
	dev_dbg(&(g_p_ssd2531_data->input_dev->dev), "--ssd2531_ts_suspend\n");
	return 0;
}
#endif

static void ssd2531_ts_work_func(struct work_struct *work)
{
	uint8_t read_buf[POINT_DATA_SIZE];	/*each point has 4 byte data*/
	struct finger_info info;
	int		i;
	int		status;
	int		reg;
	int		send_press;
	int		pressed_finger = -1;

	/*printk(KERN_NOTICE"ssd2531_ts_work_func ++\n");*/
	/*read number of point down*/
	status = ssd2531_i2c_read_byte(SSD2531_EVENT_STATUS_REG);
	if (status < 0) {
		printk(KERN_ERR"failed to read SSD2531 dava via i2c\n");
		return;
	}
	for (i = 0; i < MAX_NUM_OF_POINTS; ++i) {
		reg = SSD2531_FINGER_1_REG + i;
		send_press = 0;

		if (IS_FINGER_DOWN(i, status)) {
			ssd2531_i2c_read_burst(reg, read_buf, POINT_DATA_SIZE);
			info.x = read_buf[0] | ((read_buf[2]&0xF0)<<4);
			info.y = read_buf[1] | ((read_buf[2]&0x0F)<<8);
			info.weight = (read_buf[3]&0xF0)>>4;
			if (info.weight == 0)
				info.weight++;

			info.f_id = i;
			if (info.x > TOUCHSCREEN_X_AXIS_MAX ||
				info.y > TOUCHSCREEN_Y_AXIS_MAX)
				dev_dbg(&(g_p_ssd2531_data->input_dev->dev),
				"recieved index out of range value\n");
			/*update prev location array.
			may need to move this in case we use averaging*/
			g_p_ssd2531_data->prev_finger_info[i].x = info.x;
			g_p_ssd2531_data->prev_finger_info[i].y = info.y;

			dev_dbg(&(g_p_ssd2531_data->input_dev->dev),
				"finger %d x-%x, y-%x, weight-%x\n",
				i, info.x, info.y, info.weight);
			send_press = 1;
			if (pressed_finger < 0)
				pressed_finger = i;
		} else {
			if (IS_FINGER_DOWN(i, g_p_ssd2531_data->prev_finger_bitmask)) {
				/*was down and now not -> finger up event*/
				dev_dbg(&(g_p_ssd2531_data->input_dev->dev),
					"finger %d up\n", i);
				info.x = g_p_ssd2531_data->prev_finger_info[i].x;
				info.y = g_p_ssd2531_data->prev_finger_info[i].y;
				info.weight = 0; /*0 indicated release*/
				info.f_id = i;
				send_press = 1;
				/*update prev location array.
				may need to move this in case we use averaging*/
				g_p_ssd2531_data->prev_finger_info[i].x = 0;
				g_p_ssd2531_data->prev_finger_info[i].y = 0;
			}
		}
		if (send_press) {
			input_report_abs(g_p_ssd2531_data->input_dev,
				ABS_MT_POSITION_X, info.x);
			input_report_abs(g_p_ssd2531_data->input_dev,
				ABS_MT_POSITION_Y, info.y);
			input_report_abs(g_p_ssd2531_data->input_dev,
				ABS_MT_PRESSURE, info.weight);
			input_report_abs(g_p_ssd2531_data->input_dev,
				ABS_MT_TRACKING_ID, info.f_id);
			input_mt_sync(g_p_ssd2531_data->input_dev);
		}
	}
	if (pressed_finger < 0) {/*all fingers are up*/
		input_mt_sync(g_p_ssd2531_data->input_dev);
		input_sync(g_p_ssd2531_data->input_dev);
	} else {
		input_report_abs(g_p_ssd2531_data->input_dev, ABS_X,
			g_p_ssd2531_data->prev_finger_info[pressed_finger].x);
		input_report_abs(g_p_ssd2531_data->input_dev, ABS_Y,
			g_p_ssd2531_data->prev_finger_info[pressed_finger].y);
		input_report_abs(g_p_ssd2531_data->input_dev, ABS_PRESSURE,
			g_p_ssd2531_data->prev_finger_info[pressed_finger].weight);
		if (g_p_ssd2531_data->prev_finger_bitmask == 0) {
			/*first press*/
			input_report_key(g_p_ssd2531_data->input_dev,
				g_p_ssd2531_data->btn_info[pressed_finger], 1);
			dev_dbg(&(g_p_ssd2531_data->input_dev->dev), "send press\n");
		}
		input_sync(g_p_ssd2531_data->input_dev);
	}
	g_p_ssd2531_data->prev_finger_bitmask = status & 0x0F;
	if (status & (SSD2531_STATUS_FIFO_OVERFLOW_EMPTY_BIT |
				SSD2531_STATUS_FIFO_NOT_EMPTY_BIT))
		ssd2531_ts_crl_all_events();
}
/*#define ARRAY_AND_SIZE(x)	(x), ARRAY_SIZE(x)*/
static int ssd2531_ts_reset(void)
{
	int ret = 0;
	unsigned rst_gpio;
	unsigned inr_gpio;
	mfp_cfg_t config[] = {
		g_p_ssd2531_data->mfp_gpio_pin_int | MFP_LPM_EDGE_FALL,
	};

	dev_dbg(&(g_p_ssd2531_data->input_dev->dev), "++ssd2531_ts_reset\n");

	rst_gpio = mfp_to_gpio(g_p_ssd2531_data->mfp_gpio_pin_reset);
	dev_dbg(&(g_p_ssd2531_data->input_dev->dev), "rst_gpio value is %d", rst_gpio);
	inr_gpio = mfp_to_gpio(g_p_ssd2531_data->mfp_gpio_pin_int);
	if (gpio_request(rst_gpio, "ssd2531_reset")) {
		printk(KERN_ERR "Request GPIO failed,"
			"gpio: %d\n", rst_gpio);
		return -EIO;
	}
	if (gpio_request(inr_gpio, "ssd2531_interrupt")) {
		printk(KERN_ERR "Request GPIO failed,"
			"gpio: %d\n", inr_gpio);
		gpio_free(rst_gpio);
		return -EIO;
	}

	gpio_direction_input(inr_gpio); /*set as input*/
	/*set as output*/
	gpio_direction_output(rst_gpio, 1);
	udelay(30);
	gpio_direction_output(rst_gpio, 0);
	udelay(20);
	gpio_direction_output(rst_gpio, 1);

	mfp_config(config, ARRAY_SIZE(config));
	gpio_free(rst_gpio);
	gpio_free(inr_gpio);
	dev_dbg(&(g_p_ssd2531_data->input_dev->dev), "--ssd2531_ts_reset\n");
	udelay(10);
	return ret;
}
static int ssd2531_ts_init(void)
{
	int ret;
	u8 reg;
	u16 value;

	dev_dbg(&(g_p_ssd2531_data->input_dev->dev), "ssd2531_ts_init ++\n");
	/*Below commands must send first in order wake up the IC*/
	ret = ssd2531_ts_wakeup(1); /*Exit sleep mode*/
	if (ret < 0) {
		dev_dbg(&(g_p_ssd2531_data->input_dev->dev),
			"error writing to touch I2C, exiting");
		return -EIO;
	}
	udelay(100);
	reg = 0x2B;
	value = 0x02;
	ssd2531_i2c_write(reg, value); /*Enable DSP clock*/
	udelay(500);

	/* Touch panel configuration*/
	reg = 0x06;
	value = 0x0F;/* Set drive line no. = 21*/
	ssd2531_i2c_write(reg, value);
	reg = 0x07;
	value = 0x06;/* Set sense line no. = 12*/
	ssd2531_i2c_write(reg, value);

	/*set scaling to fit 480*800 resolution*/
	/*default resolution is 352*640 (# of lines * 32)*/
	reg = 0x66; /*X scaile*/
	value = 0x57;	/*need to change X from 352 to 480 ->
					factor is 1.363636,
					closest value is 1.359375.
					this gives res of 478*/
	ssd2531_i2c_write(reg, value);
	reg = 0x67;/*Y scaile*/
	value = 0x50;/*need to change Y from 640 to 800 ->
				 factor is 1.25*/
	ssd2531_i2c_write(reg, value);

	/*invert X coordinates*/
	reg = 0x65;
	value = 0x02;
	ssd2531_i2c_write(reg, value);

	/* SSD2531 analog setting*/
	reg = 0xC1;
	value = 0x02; /* Set charge bump x6*/
	ssd2531_i2c_write(reg, value);
	reg = 0xD5;
	value = 0x0F; /* Set Driving voltage ~15.5V*/
	ssd2531_i2c_write(reg, value);
	udelay(300);/*delay 300*/

	/* Touch detection setting*/
	reg = 0xD8;
	value = 0x03; /* Set sampling delay*/
	ssd2531_i2c_write(reg, value);
	reg = 0x2A;
	value = 0x03; /* Set sub-frame*/
	ssd2531_i2c_write(reg, value);
	reg = 0x59;
	value = 0x01; /* Enable move tolerance*/
	ssd2531_i2c_write(reg, value);
	reg = 0x5B;
	value = 0x01; /* Set Move tolerance to 1*/
	ssd2531_i2c_write(reg, value);
	reg = 0x5A;
	value = 0x01; /* Set maximum miss frame to 1*/
	ssd2531_i2c_write(reg, value);
	reg = 0x2C;
	value = 0x02; /* Set median filter to 1 tap*/
	ssd2531_i2c_write(reg, value);
	reg = 0x37;
	value = 0x03; /* Set Segmentation depth*/
	ssd2531_i2c_write(reg, value);
	reg = 0x39;
	value = 0x01; /* Finger tracking mode*/
	ssd2531_i2c_write(reg, value);
	reg = 0x56;
	value = 0x01; /* Moving average*/
	ssd2531_i2c_write(reg, value);
	reg = 0x53;
	value = 0x10; /* Set CG tolerance reg*/
	ssd2531_i2c_write(reg, value);
	reg = 0x54;
	value = 0x30; /* Set X coordinate tracking tolerance reg*/
	ssd2531_i2c_write(reg, value);
	reg = 0x55;
	value = 0x30; /* Set Y coordinate tracking tolerance reg*/
	ssd2531_i2c_write(reg, value);

	/* Finger recognition criteria*/
	reg = 0x33;
	value = 0x01; /* Set Min. Finger area = 1*/
	ssd2531_i2c_write(reg, value);
	reg = 0x34;
	value = 0x64; /* Set Min. Finger level = 100*/
	ssd2531_i2c_write(reg, value);

	reg = 0x35;
	value = 0x00;
	ssd2531_i2c_write(reg, value);
	value = 0x20;
	ssd2531_i2c_write(reg, value);
	reg = 0x36;
	value = 0x1E; /* Set Max. Finger area*/
	ssd2531_i2c_write(reg, value);
	udelay(100);/*delay 100*/
	reg = 0x25;
	value = 0x0B; /* Set scan mode*/
	ssd2531_i2c_write(reg, value);
	reg = 0xA2;
	value = 0x00; /* Reset Init Reference*/
	ssd2531_i2c_write(reg, value);
	udelay(100);/*delay 100*/

	reg = 0x02;
	value = ssd2531_i2c_read_word(reg);
	value = ((value&0xFF)<<8) | ((value&0xFF00)>>8);
	printk(KERN_INFO"ssd2531 chip Id is 0x%x\n", value);

	dev_dbg(&(g_p_ssd2531_data->input_dev->dev), "ssd2531_ts_init --\n");

	return 0;
}

static int ssd2531_ts_crl_all_events(void)
{
	u8 reg = SSD2531_EVENT_STACK_REG;
	uint8_t read_buf[POINT_DATA_SIZE];
	u16 value;
	int b_continue = 1;
	dev_dbg(&(g_p_ssd2531_data->input_dev->dev),
		"ssd2531_ts_crl_all_events ++\n");
	while (b_continue) {
		ssd2531_i2c_read_burst(reg, read_buf, POINT_DATA_SIZE);
		if (read_buf[0] != 0) {
			dev_dbg(&(g_p_ssd2531_data->input_dev->dev),
				"found event # %d in FIFO. flag = %d, x-%d, y-%d\n",
				(read_buf[0]&0x0F),
				((read_buf[0]&0xF0)>>4),
				(((read_buf[3]&0xF0)<<4) | read_buf[1]),
				(((read_buf[3]&0x0F)<<8) | read_buf[2]));
		}
		value = ssd2531_i2c_read_byte(SSD2531_EVENT_STATUS_REG);
		if (!(value&(SSD2531_STATUS_FIFO_OVERFLOW_EMPTY_BIT |
					SSD2531_STATUS_FIFO_NOT_EMPTY_BIT)))
			break;
	}

	ssd2531_i2c_read_byte(SSD2531_EVENT_STACK_CLEAR_REG);
	dev_dbg(&(g_p_ssd2531_data->input_dev->dev), "ssd2531_ts_crl_all_events --\n");
	return 0;
}

static int ssd2531_ts_interrupt_init(void)
{
	u8 reg;
	u16 value;

	dev_dbg(&(g_p_ssd2531_data->input_dev->dev),
		"++ssd2531_ts_interrupt_init\n");
	reg = 0x7B;
	value = 0xE0;
	ssd2531_i2c_write(reg, value);/*disable interrupts*/

	/*need to add GPIO settings if needed*/
	dev_dbg(&(g_p_ssd2531_data->input_dev->dev),
		"--ssd2531_ts_interrupt_init\n");
	return 0;
}

static irqreturn_t ssd2531_ts_irq_handler(int irq, void *dev_id)
{
	/*
	struct ssd2531_ts_data * ts = dev_id;
	disable_irq_nosync(ts->client->irq);
	schedule_work(&ts->work);
	*/
	ssd2531_ts_work_func(NULL);
	return IRQ_HANDLED;
}

static int __devinit
ssd2531_i2c_probe(struct i2c_client *client,
				  const struct i2c_device_id *id)
{
	int ret = 0;
	int *mfp_pins;
	struct proc_dir_entry *ssd2531_ts_proc_entry;
	unsigned long flags = IRQF_TRIGGER_FALLING | IRQF_ONESHOT;

	printk(KERN_NOTICE"++ssd2531_i2c_probe\n");

	if (g_p_ssd2531_data != NULL) {
		printk(KERN_ERR"called SSD2531_i2c_probe but device data is not NULL!!");
		return -ENOMEM;
	}

	mfp_pins = (int *)client->dev.platform_data;
	if (mfp_pins == NULL)
		return -EINVAL;

	g_p_ssd2531_data = kzalloc(sizeof(struct ssd2531_ts_data), GFP_KERNEL);
	if (!g_p_ssd2531_data)
		return -ENOMEM;

	g_p_ssd2531_data->mfp_gpio_pin_int = mfp_pins[0];
	g_p_ssd2531_data->mfp_gpio_pin_reset = mfp_pins[1];

	g_p_ssd2531_data->prev_finger_bitmask = 0;
	g_p_ssd2531_data->b_is_suspended = 0;
	g_p_ssd2531_data->btn_info[0] = BTN_TOUCH;
	g_p_ssd2531_data->btn_info[1] = BTN_2;
	g_p_ssd2531_data->btn_info[2] = BTN_3;
	g_p_ssd2531_data->btn_info[3] = BTN_4;

	g_p_ssd2531_data->client = client;

	i2c_set_clientdata(client, g_p_ssd2531_data);

	ret = ssd2531_ts_reset();
	if (ret < 0) {
		printk(KERN_ERR"failed to reset ssd2531 touch screen\n");
		goto err_no_dev;
	}
	ret = ssd2531_ts_init();
	if (ret < 0) {
		printk(KERN_ERR"failed to init ssd2531 touch screen\n");
		goto err_no_dev;
	}

	ret = ssd2531_ts_interrupt_init();
	if (ret < 0) {
		printk(KERN_ERR"failed to init ssd2531 interrupt\n");
		goto err_no_dev;
	}
	g_p_ssd2531_data->input_dev = input_allocate_device();
	if (g_p_ssd2531_data->input_dev == NULL) {
		ret = -ENOMEM;
		printk(KERN_ERR"failed to allocate input device\n");
		goto err_dev_alloc;
	}
	g_p_ssd2531_data->input_dev->name = "ssd2531-touch";
	set_bit(EV_SYN, g_p_ssd2531_data->input_dev->evbit);
	set_bit(EV_KEY, g_p_ssd2531_data->input_dev->evbit);
	set_bit(BTN_TOUCH, g_p_ssd2531_data->input_dev->keybit);
	set_bit(EV_ABS, g_p_ssd2531_data->input_dev->evbit);


	input_set_capability(g_p_ssd2531_data->input_dev, EV_KEY, BTN_TOUCH);
	input_set_capability(g_p_ssd2531_data->input_dev, EV_KEY, BTN_2);
	input_set_capability(g_p_ssd2531_data->input_dev, EV_KEY, BTN_3);
	input_set_capability(g_p_ssd2531_data->input_dev, EV_KEY, BTN_4);

	input_set_abs_params(g_p_ssd2531_data->input_dev,
		ABS_X, TOUCHSCREEN_X_AXIS_MIN,
		TOUCHSCREEN_X_AXIS_MAX, 0, 0);
	input_set_abs_params(g_p_ssd2531_data->input_dev,
		ABS_Y, TOUCHSCREEN_Y_AXIS_MIN,
		TOUCHSCREEN_Y_AXIS_MAX, 0, 0);
	input_set_abs_params(g_p_ssd2531_data->input_dev,
		ABS_PRESSURE, TOUCHSCREEN_Z_AXIS_MIN,
		TOUCHSCREEN_Z_AXIS_MAX, 0, 0);

	input_set_abs_params(g_p_ssd2531_data->input_dev,
		ABS_MT_POSITION_X, TOUCHSCREEN_X_AXIS_MIN,
		TOUCHSCREEN_X_AXIS_MAX, 0, 0);
	input_set_abs_params(g_p_ssd2531_data->input_dev,
		ABS_MT_POSITION_Y, TOUCHSCREEN_Y_AXIS_MIN,
		TOUCHSCREEN_Y_AXIS_MAX, 0, 0);
	input_set_abs_params(g_p_ssd2531_data->input_dev,
		ABS_MT_PRESSURE, TOUCHSCREEN_Z_AXIS_MIN,
		TOUCHSCREEN_Z_AXIS_MAX, 0, 0);
	input_set_abs_params(g_p_ssd2531_data->input_dev,
		ABS_MT_TRACKING_ID, TOUCHSCREEN_ID_MIN,
		TOUCHSCREEN_ID_MAX, 0, 0);

	ret = input_register_device(g_p_ssd2531_data->input_dev);
	if (ret) {
		printk(KERN_ERR"ssd2531 probe: Unable to register input device\n");
		ret = -EXDEV;
		goto err_register_dev;
	}
	/*register irq*/
	g_p_ssd2531_data->client->irq = IRQ_GPIO(mfp_to_gpio(g_p_ssd2531_data->mfp_gpio_pin_int));
	printk(KERN_NOTICE"received IRQ # %d\n", g_p_ssd2531_data->client->irq);

/*
	ret = request_irq(g_p_ssd2531_data->client->irq, ssd2531_ts_irq_handler,
		IRQF_TRIGGER_FALLING, "ssd2531_touch", g_p_ssd2531_data);
*/
	ret = request_threaded_irq(g_p_ssd2531_data->client->irq,
		NULL,
		ssd2531_ts_irq_handler,
		flags, "ssd2531_touch",
		g_p_ssd2531_data);
	if (ret)
		printk(KERN_ERR"ssd2531 probe: failed to register to IRQ\n");
	ssd2531_ts_crl_all_events();

	ssd2531_ts_proc_entry = create_proc_entry("driver/ssd2531_ts", 0, NULL);
	if (ssd2531_ts_proc_entry)
		ssd2531_ts_proc_entry->write_proc = ssd2531_ts_proc_write;

#ifdef CONFIG_HAS_EARLYSUSPEND
	register_early_suspend(&ssd2531_ts_early_suspend_desc);
#endif

	printk(KERN_NOTICE"--ssd2531_i2c_probe - exit OK\n");
	return 0;
err_register_dev:
	input_free_device(g_p_ssd2531_data->input_dev);
	g_p_ssd2531_data->input_dev = NULL;
err_dev_alloc:
	/*ssd2531_deinit_ts();
	release IRQ?*/
err_no_dev:
	if (g_p_ssd2531_data != NULL)
		kfree(g_p_ssd2531_data);
	g_p_ssd2531_data = NULL;
	printk(KERN_NOTICE"--ssd2531_i2c_probe - exit ERROR\n");
	return ret;
}

static int __devexit ssd2531_i2c_remove(struct i2c_client *client)
{
	struct ssd2531_ts_data *ts = i2c_get_clientdata(client);
#ifdef CONFIG_HAS_EARLYSUSPEND
	unregister_early_suspend(&ssd2531_ts_early_suspend_desc);
#endif
	free_irq(client->irq, ts);
	input_unregister_device(ts->input_dev);
	kfree(ts);
	g_p_ssd2531_data = NULL;
	return 0;
}

static const struct i2c_device_id SSD2531_i2c_id[] = {
	{ "ssd2531_ts", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, SSD2531_i2c_id);

static struct i2c_driver SSD2531_i2c_driver = {
	.driver = {
		.name	= "ssd2531",
		.owner	= THIS_MODULE,
	},
	.probe		= ssd2531_i2c_probe,
	.remove		= __devexit_p(ssd2531_i2c_remove),
#ifndef CONFIG_HAS_EARLYSUSPEND
	.suspend	= ssd2531_ts_suspend,
	.resume		= ssd2531_ts_resume,
#endif
	.id_table	= SSD2531_i2c_id,
};

static int __init ssd2531_i2c_init(void)
{
	return i2c_add_driver(&SSD2531_i2c_driver);
}
module_init(ssd2531_i2c_init);

static void __exit ssd2531_i2c_exit(void)
{
	i2c_del_driver(&SSD2531_i2c_driver);
}
module_exit(ssd2531_i2c_exit);

MODULE_AUTHOR("Chen Reichbach<creichba@marvell.com>");
MODULE_DESCRIPTION("ssd2531 touch screen driver");
MODULE_LICENSE("GPL");
