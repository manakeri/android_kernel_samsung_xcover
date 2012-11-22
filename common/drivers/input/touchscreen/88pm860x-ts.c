/*
 * Touchscreen driver for Marvell 88PM860x
 *
 * Copyright (C) 2009 Marvell International Ltd.
 * 	Haojian Zhuang <haojian.zhuang@marvell.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/mfd/88pm860x.h>
#include <mach/regs-ost.h>
#include <linux/earlysuspend.h>
#include <linux/proc_fs.h>
#include <asm/uaccess.h>

/* Internal Definitions */
#define TSI_SAMPLES_NUM		3
#define TSI_MAX_SAMPLE_DIF_X    250
#define TSI_MAX_SAMPLE_DIF_Y    250

#define PM8607_X_MES_TRIM	300 /*for debug*/
#define PM8607_Y_MES_TRIM	500 /*for debug*/
#define PM8607_X_AXIS_MAX	3500
#define PM8607_X_AXIS_MIN	0
#define PM8607_Y_AXIS_MAX	3500
#define PM8607_Y_AXIS_MIN	0
#define PM8607_PRESSURE_MAX	255
#define PM8607_PRESSURE_MIN	0
#define PM8607_TOOL_WIDTH_MAX	15
#define PM8607_TOOL_WIDTH_MIN	0
#define PM8607_XY_MES_CYC_MS	10


#define DEBUG	1

#define DIS_BET_2_POINTS 20 /* Pen Accuracy distance^2 = x^2 + y^2 */
#define TOUCHSCREEN_CONSOLE_PRINT_PENDETECT	pr_debug("levante_ts_interrupt  PEN-DETECT\n");
#define TOUCHSCREEN_CONSOLE_PRINT_XY		pr_debug("======> Touch-Screen:  X=%d    Y=%d\n", touch->tem_x, touch->tem_y);
#define TOUCHSCREEN_CONSOLE_PRINT_PENUP		pr_debug("======> Touch-Screen: PEN-UP\n");

#define WORKAROUND_FOR_FALSE_PEN_UP_DET

/*used for ram dump*/
#ifdef TSI_MIPS_RAM
	#ifdef CONFIG_PXA_MIPSRAM
	#include <linux/mipsram.h>
	#endif /* CONFIG_PXA_MIPSRAM */
#endif

DEFINE_MUTEX(tsi_lock);

enum {
	TSI_PEN_UNKNOW = 0,
	TSI_PEN_DOWN = 1,
	TSI_PEN_UP = 2
};

struct pm860x_touch {
	spinlock_t		ts_lock;
	struct input_dev	*idev;
	struct i2c_client	*i2c;
	struct pm860x_chip	*chip;
	int	irq;
	int	res_x;		/* resistor of Xplate */
	int	suspended;
	int	pen_state;
	int	use_count;
	u16	tem_x;
	u16	tem_y;
	u16	tem_x_pre;
	u16	tem_y_pre;
	struct delayed_work	*del_work;
};

static struct pm860x_touch *touch;

#ifdef CONFIG_HAS_EARLYSUSPEND
static int pm860x_ts_resume(struct platform_device *pdev);
static int pm860x_ts_suspend(struct platform_device *pdev, pm_message_t state);

static void pm860x_ts_early_suspend(struct early_suspend *h)
{
	pm_message_t t;
	t.event = 0;
	if (pm860x_ts_suspend != NULL)
		pm860x_ts_suspend(NULL, t);
}
static void pm860x_ts_late_resume(struct early_suspend *h)
{
	if (pm860x_ts_resume != NULL)
		pm860x_ts_resume(NULL);
}

static struct early_suspend pm860x_ts_early_suspend_desc = {
	.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1,
	.suspend = pm860x_ts_early_suspend,
	.resume = pm860x_ts_late_resume,
};
#endif

static int pm860x_ts_proc_write(struct file *file, const char __user *buffer,
			   unsigned long count, void *data)
{
	/*keep this interface for further useage*/
	return 0;
}

static int calc_avg(int* buff, int size, int max_diff)
{
	int result = 0, i;
	int max_sample = buff[0];
	int min_sample = buff[0];
	int used_samples = size;

	for (i=0; i< size; ++i)
	{
		if(buff[i] == -1)
		{
			used_samples--;
			continue;
		}

	result += buff[i];
	max_sample =  max_sample < buff[i] ? buff[i] : max_sample;
	min_sample =  min_sample > buff[i] ? buff[i] : min_sample;
	}
	if( (max_sample - min_sample) > max_diff ) /* this is not a valid sample there are big diffs */
	{
		pr_debug( "levante xy measurment averaging failed, sample diff is too big (%d)!\n", (max_sample - min_sample));
		return -EIO;
	}
	result /= used_samples;
	return result;
}

int pm860x_tsi_readxy(u16 *x, u16 *y, u8 *pen_state)
{
	int i, x_avg, y_avg, status;
	int x_arr[TSI_SAMPLES_NUM], y_arr[TSI_SAMPLES_NUM];
	u8 val[4];

	*x = *y = 0; /*initilize to zero*/

	for (i = 0; i < TSI_SAMPLES_NUM; ++i) {
		status = pm860x_bulk_read(touch->i2c, PM8607_MEAS_TSIX_1, sizeof(val), val);

		if (status < 0) {
			x_arr[0] = x_arr[1] = x_arr[2] = -1;
			y_arr[0] = y_arr[1] = y_arr[2] = -1;
			pr_debug("---->levante_tsi_readxy - fail sanremo_read_block \n");
			return -EIO;
		}

		*pen_state = (val[1] & (1<<6));

		x_arr[i] = ((val[0]<<4) | (val[1]&0x0f));
		y_arr[i] = ((val[2]<<4) | (val[3]&0x0f));
	}

	/*calculate the average*/
	x_avg = calc_avg(x_arr, TSI_SAMPLES_NUM, TSI_MAX_SAMPLE_DIF_X);
	y_avg = calc_avg(y_arr, TSI_SAMPLES_NUM, TSI_MAX_SAMPLE_DIF_Y);

	/*if the average value is not valid*/
	if ((x_avg <= 0) || (y_avg <= 0)) {
		pr_debug("------->levante_tsi_readxy - average value is not valid \n");
		return -EAGAIN;
	}

	*x = x_avg;
	*y = y_avg;

	pr_debug("------->levante_tsi_readxy pen_state[%x]cmp[%x] \n",*pen_state,val[1] );
	pr_debug("------->levante_tsi_readxy X[%d] Y[%d] \n", *x, *y );

	return 0;
}

int pm860x_tsi_disable_all_measurements(void)
{
	int status;
	/* disable TSI measurements */
	status = pm860x_set_bits(touch->i2c, PM8607_MEAS_EN3,PM8607_MEAS_EN3_PENDET | PM8607_MEAS_EN3_TSIX | PM8607_MEAS_EN3_TSIY, 0x00);
	if (status < 0)
		return -EIO;
	return 0;
}

int pm860x_ts_read(u16 *p_x,  u16 *p_y, int *p_pen_state)
{
	int ret;
	ret = pm860x_tsi_readxy( p_x, p_y, (u8*)p_pen_state);
	return ret;
}

/*(xd*xd + yd*yd) < (Diameter*Diameter)*/
static int pm860x_evaluate_point(int iX_Pre, int iY_Pre, int iX, int iY)
{
	int tempx,tempy,diff;
	int fRet=0;

	if ((iX_Pre < 0) || (iX < 0)){  /* not valid point*/
		return -EINVAL;
	}

	if(touch-> tem_x_pre == 0xffff && touch->tem_y_pre ==0xffff ){/*the first time no need to evaluate*/
		return 0;
	}

	tempx = abs( iX - iX_Pre);
	tempy = abs( iY - iY_Pre);

	diff = ((tempx ^ 2) + (tempy ^ 2));

	 /* if the dis between 2 points are less then DIS_BET_2_POINTS but
	  * greater then zero ( zero means its the same point which was reported)
	  * then it's valid
	  */
	fRet =  ((diff < (DIS_BET_2_POINTS * DIS_BET_2_POINTS)) ? 0 : -EINVAL);

	pr_debug("-------->levante_evaluate_point2 ok - (diff) = %d \n",diff);

	if (fRet < 0) {
		pr_debug("-------->!!!levante_evaluate_point skip!!! - diff [%d] pre[x=%d y=%d] current[x=%d y=%d]\n",diff,iX_Pre, iY_Pre, iX, iY);
	}
	return fRet;
}

/* setting the pen debounce 0ms=0x00/2ms=0x01/4ms=0x02/8ms=0x03*/
static int pm860x_tsi_set_pen_debounce(u8 debounce)
{
	/* change debounce to 0ms*/
	pm860x_set_bits(touch->i2c, PM8607_DEBOUNCE_REG,PM8607_PEN_DEBOUNCE_MASK, debounce);
	return 0;
}

int pm860x_tsi_enable_xy_measurements(void)
{
	int status;
	/*enable TSI measurements*/
	status = pm860x_set_bits(touch->i2c, PM8607_MEAS_EN3,PM8607_MEAS_EN3_TSIX | PM8607_MEAS_EN3_TSIY, PM8607_MEAS_EN3_TSIX | PM8607_MEAS_EN3_TSIY);
	if (status < 0)
		return -EIO;
	return 0;
}

int pm860x_enable_pen_down_irq(int enable)
{
	int ret;
	pr_debug("----->levante_enable_pen_down_irq begin\n");

	if (enable) {
		pr_debug("----->levante_enable_pen_down_irq:enable\n");

		{   /* before enabling the pen IRQ need to make sure pen measurment is enable and x,y measurment is disable */
			pm860x_set_bits(touch->i2c, PM8607_MEAS_EN3,PM8607_MEAS_EN3_TSIX | PM8607_MEAS_EN3_TSIY, 0x0);
			pm860x_set_bits(touch->i2c, PM8607_MEAS_EN3,PM8607_MEAS_EN3_PENDET, PM8607_MEAS_EN3_PENDET);
		}

		/* enable pen down IRQ */
		ret = pm860x_set_bits(touch->i2c, PM8607_INT_MASK_3,PM8607_INT_EN_PEN, PM8607_INT_EN_PEN);
		if(ret != 0)
			pr_debug( "----->levante_enable_pen_down_irq - fail \n");
		else
			pr_debug( "----->levante_enable_pen_down_irq - success \n");
	} else {
		/* disable pen down IRQ */
		pr_debug("----->levante_enable_pen_down_irq:disable\n");
		ret = pm860x_set_bits(touch->i2c, PM8607_INT_MASK_3,	PM8607_INT_EN_PEN, 0x00);
		if(ret != 0)
			pr_debug( "----->levante_disable_pen_down_irq - fail \n");
		else
			pr_debug("----->levante_disable_pen_down_irq - success \n");
	}
	pr_debug( "------->levante_enable_pen_down_irq end\n");

	return ret;
}

/* ===================== RUN-TIME HANDLING ==================================== */

/*  The context is SW-IRQ => I2C operations cannot be used here,
 *    activate the thread instead.
 */
static void pm860x_ts_timer_handler(unsigned long d)
{
	if (touch->use_count > 0) {
		schedule_work(&touch->del_work->work);
	}
}

static irqreturn_t pm860x_touch_handler(int irq, void *data)
{
	struct pm860x_touch *touch = data;
	u8 status = 0;

	status = pm860x_reg_read(touch->i2c, PM8607_STATUS_1);
	/* handle only pen-down events , pen-up is detected while measure x,y*/
	if (status & PM8607_STATUS_PEN) {
		if (touch->use_count > 0) {
			if (TSI_PEN_UP ==  touch->pen_state) {

				TOUCHSCREEN_CONSOLE_PRINT_PENDETECT;
				touch->pen_state = TSI_PEN_DOWN;
				/*Disable pen IRQ*/
				pm860x_enable_pen_down_irq(0);

				/* change to 0ms*/
				pm860x_tsi_set_pen_debounce(0x00);
				pm860x_tsi_enable_xy_measurements();

				/* activate timer for x,y readings
				 * for first mesurmant wait 5ms,in Tavor it will be 7.8msec since
				 * 1 jiffies = 7.8ms == 256 * 1/32K
				 */
				touch->del_work->timer.expires = jiffies + msecs_to_jiffies(5);
				/* check for pending before mod */
				if (timer_pending(&touch->del_work->timer) == 0)
					mod_timer(&touch->del_work->timer,touch->del_work->timer.expires);
				/* initilize for this pen-down event*/
				touch->tem_x_pre = touch->tem_y_pre = 0xffff;
			}
		}
		pr_debug("-------->levante_ts_interrupt - measure x,y start... \n");
	}else
		pr_debug("------->levante_ts_interrupt end - ignor pen-up \n");

	return IRQ_HANDLED;
}

static void pm860x_ts_worker(struct work_struct *work)
{
	int valid_point = -EINVAL;
	u8 pen_state;
	int retVal = 0;
	unsigned int diffmesc,next_delay;
	struct timeval start,end;
	/*we must report at least twice*/
#ifdef WORKAROUND_FOR_FALSE_PEN_UP_DET
	static u16 numOfReports = 2;
#endif

	pr_debug("-------->levante_ts_worker - start\n");
#ifdef TSI_MIPS_RAM
	#ifdef CONFIG_PXA_MIPSRAM
		MIPS_RAM_ADD_TRACE((unsigned int)0x70000);/*0x70000 is the trace ID need to make sure it is updated in MipsRam_linux.xls in event sheet*/
	#endif/* CONFIG_PXA_MIPSRAM */
#endif

	do_gettimeofday(&start);
	mutex_lock(&tsi_lock);
	retVal = pm860x_tsi_readxy(&touch->tem_x, &touch->tem_y, &pen_state);
	pr_debug("---->levante_ts_worker - pen state [0x%x][0:up 1:down] X = %d\t, Y = %d\n", pen_state,touch->tem_x,touch->tem_y);
	printk(KERN_DEBUG "---->levante_ts_worker - pen state [0x%x][0:up 1:down] X = %d\t, Y = %d\n", pen_state,touch->tem_x,touch->tem_y);
#ifdef WORKAROUND_FOR_FALSE_PEN_UP_DET
	if (pen_state || numOfReports >= 2) {
#else
	if (pen_state) {
#endif
		/*check current location vs. previous - if too far then measure again don't send to user.*/
		if (!retVal)
			valid_point = pm860x_evaluate_point(touch->tem_x_pre,touch->tem_y_pre,touch->tem_x,touch->tem_y);

		/* report the point only if xy meas is ok and eval is valid */
		if (!retVal && !valid_point) {
			/* send measure to user here. */
			input_report_abs(touch->idev, ABS_X, (touch->tem_x & 0xfff)-250/*12b vaild*/);
			input_report_abs(touch->idev, ABS_Y, 4096-(touch->tem_y & 0xfff)-250/*12b vaild*/);
			input_report_abs(touch->idev,ABS_PRESSURE, 255);
			input_report_key(touch->idev,BTN_TOUCH, 1);
			input_sync(touch->idev);

			touch->tem_x_pre = touch->tem_x;/* update for next points evaluate*/
			touch->tem_y_pre = touch->tem_y;/* update for next points evaluate*/

			pr_debug("--->levante_ts_worker-tem_x_pre[%d],tem_y_pre[%d] \n", touch->tem_x_pre, touch->tem_y_pre);
			pr_debug("--->levante_ts_worker-report pen down x[%d],y[%d]\n", touch->tem_x, touch->tem_y);
			TOUCHSCREEN_CONSOLE_PRINT_XY;
		} else
			pr_debug("levante_ts_worker - NOT VALID point don't report\n");

		/* we now know when to trigger the next measuremt in order to have 1 in 10msec*/
		do_gettimeofday(&end);
		diffmesc = ((start.tv_sec - end.tv_sec) * 1000000 + start.tv_usec - end.tv_usec)/1000;
		pr_debug("-------->levante_ts_worker - diffmesc[%d msec] \n",diffmesc);

		if( diffmesc < PM8607_XY_MES_CYC_MS)
			next_delay =  PM8607_XY_MES_CYC_MS - diffmesc;
		/* we want a mesurment every LEVNATE_XY_MES_CYC_MS (10ms)
		 * cycle,less might overloading the system
		 * WM standart normal 100 in a sec
		 */
		 else
			next_delay = 0;/*if diffmesc > LEVNATE_XY_MES_CYC_MS - it seems we spent lots of time calc -> don't delay next mesure*/

		pr_debug("-------->levante_ts_worker - next delay[%d] \n",next_delay);
		touch->del_work->timer.expires = jiffies + msecs_to_jiffies(next_delay);/* since OS tick is 7.8msec - next_delay usualy == 1 tick */
		/* check for pending beofre mod timer*/
		if (timer_pending(&touch->del_work->timer) == 0)
			mod_timer(&touch->del_work->timer,touch->del_work->timer.expires);
#ifdef WORKAROUND_FOR_FALSE_PEN_UP_DET
		if (numOfReports > 0) {
			numOfReports--;
		}
#endif
	} else {
		numOfReports = 2;
		/* Report a pen up event */
		pr_debug( "-------->levante_ts_worker - pen-up event \n");

		if( touch->pen_state == TSI_PEN_DOWN){/*Only do if the previous state was pen-down,if it is pen-up then nothing to do here*/

			touch->pen_state = TSI_PEN_UP;

			input_report_abs(touch->idev,ABS_PRESSURE, 0);
			input_report_abs(touch->idev,ABS_TOOL_WIDTH, 1);
			input_report_key(touch->idev,BTN_TOUCH, 0);
			input_sync(touch->idev);

			pr_debug("-------->levante_ts_worker-reported to user:pen-up event\n");

			/* change back to 8ms*/
			pm860x_tsi_set_pen_debounce(0x03);
			/* enable pen IRQ*/
			pm860x_enable_pen_down_irq(1);
			/* check for pending beofre delete*/
			if (timer_pending(&touch->del_work->timer) == 0)
				del_timer(&touch->del_work->timer);
		} else
			pr_debug( "levante_ts_worker - pen is already up - doing nothing \n");
	}

#ifdef TSI_MIPS_RAM
	#ifdef CONFIG_PXA_MIPSRAM
		MIPS_RAM_ADD_TRACE((unsigned int)0x70001);/*0x70001 is the trace ID need to make sure it is updated in MipsRam_linux.xls in event sheet*/
	#endif/* CONFIG_PXA_MIPSRAM */
#endif
	mutex_unlock(&tsi_lock);

	return;
}

int pm860x_tsi_measurements_config(void)
{
	int status;
	u8 val = 0;

	pr_debug("levante_tsi_measurements_config begin \n" );

	/*set measurements timing*/
	val = (PM8607_MEASOFFTIME1MEAS_OFFTIME1_B(0x00) | PM8607_MEASOFFTIME1MEAS_EN_SLP_B);
	status = pm860x_reg_write(touch->i2c,PM8607_MEAS_OFF_TIME1, val);
	if (status < 0)
		return -EIO;
	/* need 8ms(0x03) for BSP only*/
	status = pm860x_set_bits(touch->i2c, PM8607_DEBOUNCE_REG,PM8607_DEBOUNCE_PEN_DET(0x03),PM8607_DEBOUNCE_PEN_DET(0x03));
	if (status < 0)
		return -EIO;

	val = 0x20; /*for B0 SANREMO_MEAS_OFF_TIME2 in the next write.*/
	status = pm860x_reg_write(touch->i2c, PM8607_MEAS_OFF_TIME2, val);
	if (status < 0)
		return -EIO;

	val = 0x50;/* set TSI pre-bias (migth be different between EVB and SAAR depends on Touch panel HW )*/
	status = pm860x_reg_write(touch->i2c,PM8607_PD_PREBIAS, val);
	if (status < 0)
		return -EIO;

	val = 0x07;/*migth be different between EVB and SAAR depends on Touch panel HW*/
	status = pm860x_reg_write(touch->i2c,PM8607_TSI_PREBIAS, val);
	if (status < 0)
		return -EIO;

	pr_debug( "levante_tsi_measurements_config end \n" );

	return 0;
}



static int pm860x_touch_open(struct input_dev *dev)
{
	struct pm860x_touch *touch = input_get_drvdata(dev);
	unsigned long flags;
	pr_debug("%s: enter", __func__);
	spin_lock_irqsave(&touch->ts_lock,flags);
	if (touch->use_count++ == 0) {
		spin_unlock_irqrestore(&touch->ts_lock, flags);
		INIT_DELAYED_WORK(touch->del_work,pm860x_ts_worker);
		touch->del_work->timer.function = pm860x_ts_timer_handler;
		touch->del_work->timer.data = (unsigned long)touch;
	} else {
		spin_unlock_irqrestore(&touch->ts_lock, flags);
	}
	pm860x_enable_pen_down_irq(1);
	return 0;
}

static void pm860x_touch_close(struct input_dev *dev)
{
	struct pm860x_touch *touch = input_get_drvdata(dev);
	unsigned long flags;

	dev_dbg(&dev->dev, "levante_ts_close!\n");
	pr_debug("%s: enter with use count = %d", __func__,touch->use_count);

	pm860x_tsi_disable_all_measurements();
	pm860x_enable_pen_down_irq(0);

	spin_lock_irqsave(&touch->ts_lock, flags);
	if (--touch->use_count == 0) {
		spin_unlock_irqrestore(&touch->ts_lock, flags);
	} else {
		spin_unlock_irqrestore(&touch->ts_lock, flags);
	}
}

static int __devinit pm860x_touch_probe(struct platform_device *pdev)
{
	struct proc_dir_entry *pm860x_ts_proc_entry;
	struct pm860x_chip *chip = dev_get_drvdata(pdev->dev.parent);
	struct pm860x_platform_data *pm860x_pdata = pdev->dev.parent->platform_data;
	struct pm860x_touch_pdata *pdata = NULL;

	int irq, ret;
	irq = platform_get_irq(pdev, 0);
	if (irq < 0) {
		dev_err(&pdev->dev, "No IRQ resource!\n");
		return -EINVAL;
	}

	if (!pm860x_pdata) {
		dev_err(&pdev->dev, "platform data is missing\n");
		return -EINVAL;
	}

	pdata = pm860x_pdata->touch;
	if (!pdata) {
		dev_err(&pdev->dev, "touchscreen data is missing\n");
		return -EINVAL;
	}

	touch = kzalloc(sizeof(struct pm860x_touch), GFP_KERNEL);
	if (touch == NULL)
		return -ENOMEM;
	dev_set_drvdata(&pdev->dev, touch);

	spin_lock_init(&touch->ts_lock);
	touch->del_work = (struct delayed_work *)kzalloc(sizeof(struct delayed_work),GFP_KERNEL);
	if(!touch->del_work)
		return -ENOMEM;
	/*the initialization of del_work is in touch open function*/
	touch->pen_state = TSI_PEN_UP;
	touch->suspended = 0;
	touch->use_count = 0;

	touch->idev = input_allocate_device();
	if (touch->idev == NULL) {
		dev_err(&pdev->dev, "Failed to allocate input device!\n");
		ret = -ENOMEM;
		goto out;
	}

	touch->idev->name = "88pm860x-touch";
	touch->idev->phys = "88pm860x/input0";
	touch->idev->id.bustype = BUS_I2C;
	touch->idev->dev.parent = &pdev->dev;
	touch->idev->open = pm860x_touch_open;
	touch->idev->close = pm860x_touch_close;
	touch->chip = chip;
	touch->i2c = (chip->id == CHIP_PM8607) ? chip->client : chip->companion;
	touch->irq = irq + chip->irq_base;
	touch->res_x = pdata->res_x;
	input_set_drvdata(touch->idev, touch);
	touch->tem_x = 0xFFFF;
	touch->tem_y = 0xFFFF;
	touch->tem_x_pre = 0xFFFF;
	touch->tem_y_pre = 0xFFFF;
	ret = request_threaded_irq(touch->irq, NULL, pm860x_touch_handler,IRQF_ONESHOT, "touch", touch);
	if (ret < 0)
		goto out_irq;

	__set_bit(EV_ABS, touch->idev->evbit);
	__set_bit(ABS_X, touch->idev->absbit);
	__set_bit(ABS_Y, touch->idev->absbit);
	__set_bit(ABS_PRESSURE, touch->idev->absbit);
	__set_bit(EV_SYN, touch->idev->evbit);
	__set_bit(EV_KEY, touch->idev->evbit);
	__set_bit(BTN_TOUCH, touch->idev->keybit);

	input_set_abs_params(touch->idev, ABS_X, PM8607_X_AXIS_MIN, PM8607_X_AXIS_MAX, 0, 0);
	input_set_abs_params(touch->idev, ABS_Y, PM8607_Y_AXIS_MIN, PM8607_Y_AXIS_MAX, 0, 0);
	input_set_abs_params(touch->idev, ABS_PRESSURE, PM8607_PRESSURE_MIN, PM8607_PRESSURE_MAX,0, 0);

	ret = input_register_device(touch->idev);
	if (ret < 0) {
		dev_err(chip->dev, "Failed to register touch!\n");
		goto out_rg;
	}

	#ifdef CONFIG_HAS_EARLYSUSPEND
	register_early_suspend(&pm860x_ts_early_suspend_desc);
	#endif

	platform_set_drvdata(pdev, touch);

	pm860x_tsi_measurements_config(); /*for detecting pen-down*/
	pm860x_ts_proc_entry = create_proc_entry("driver/levante_ts", 0, NULL);
	if (pm860x_ts_proc_entry) {
		pm860x_ts_proc_entry->write_proc = pm860x_ts_proc_write;
	}

	return 0;
out_rg:
	free_irq(touch->irq, touch);
out_irq:
	input_free_device(touch->idev);
out:
	kfree(touch);
	return ret;
}

static int __devexit pm860x_touch_remove(struct platform_device *pdev)
{
	struct pm860x_touch *touch = platform_get_drvdata(pdev);

	#ifdef CONFIG_HAS_EARLYSUSPEND
	unregister_early_suspend(&pm860x_ts_early_suspend_desc);
	#endif

	input_unregister_device(touch->idev);
	free_irq(touch->irq, touch);
	platform_set_drvdata(pdev, NULL);
	kfree(touch);
	return 0;
}

#if CONFIG_PM
static int pm860x_ts_resume(struct platform_device *pdev)
{
	pm860x_enable_pen_down_irq(1);
	pm860x_tsi_measurements_config();
	touch->suspended = 0;

	return 0;
}
static int pm860x_ts_suspend(struct platform_device *pdev, pm_message_t state)
{
	touch->suspended = 1;
	pm860x_tsi_disable_all_measurements();
	pm860x_enable_pen_down_irq(0);

	return 0;
}
#else
#define pm860x_ts_resume NULL
#define pm860x_ts_suspend NULL
#endif


static struct platform_driver pm860x_touch_driver = {
	.driver	= {
		.name	= "88pm860x-touch",
		.owner	= THIS_MODULE,
	},
	.probe	= pm860x_touch_probe,
	.remove	= __devexit_p(pm860x_touch_remove),
	#ifndef CONFIG_HAS_EARLYSUSPEND
	.resume = pm860x_ts_resume,
	.suspend = pm860x_ts_suspend,
	#endif
};

static int __init pm860x_touch_init(void)
{
	return platform_driver_register(&pm860x_touch_driver);
}
module_init(pm860x_touch_init);

static void __exit pm860x_touch_exit(void)
{
	platform_driver_unregister(&pm860x_touch_driver);
}
module_exit(pm860x_touch_exit);

MODULE_DESCRIPTION("Touchscreen driver for Marvell Semiconductor 88PM860x");
MODULE_AUTHOR("Haojian Zhuang <haojian.zhuang@marvell.com>");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:88pm860x-touch");

