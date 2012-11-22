/*
 *	drivers/switch/88pm860x_headset.c
 *
 *	headset & hook detect driver for pm8607
 *
 *	Copyright (C) 2010, Marvell Corporation (xjian@Marvell.com)
 *	Author: Raul Xiong <xjian@marvell.com>
 *				 Mike Lockwood <lockwood@android.com>
 *
 *	This program is free software; you can redistribute it and/or modify
 *	it under the terms of the GNU General Public License version 2 as
 *	published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/mfd/88pm860x.h>
#include <linux/mfd/88pm860x-headset.h>
#include <linux/init.h>
#include <linux/input.h>
#include <linux/switch.h>
#include <linux/workqueue.h>
#include <linux/errno.h>
#include <asm/system.h>
#include <asm/irq.h>
#include <mach/hardware.h>
#include <linux/kthread.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/proc_fs.h>
#include <linux/uaccess.h>
#include <linux/miscdevice.h>
#include <mach/mfp.h>
#include <mach/gpio.h>
#include <mach/mfp-pxa9xx.h>
#include <linux/wakelock.h>
#include <linux/regulator/consumer.h>
#include <linux/regulator/machine.h>


/* define ADC value */
#define HEADSET_4POLE_HIGH		1780
#define HEADSET_4POLE_LOW		620
#define HEADSET_3POLE_HIGH		585
#define HEADSET_3POLE_LOW 		0 

#define SEC_JACK_SAMPLE_SIZE 5

struct sec_jack_buttons_zone {
	unsigned int code;
	unsigned int adc_low;
	unsigned int adc_high;
};

/* To support 3-buttons earjack */
static struct sec_jack_buttons_zone sec_jack_buttons_zones_rev03[] = {
	{
		.code		= KEY_MEDIA,
		.adc_low		= 0,
		.adc_high	= 54,
	},
	{
		.code		= KEY_VOLUMEUP,
		.adc_low		= 62,
		.adc_high	= 150,
	},
	{
		.code		= KEY_VOLUMEDOWN,
		.adc_low		= 154,
		.adc_high	= 350,
	},
};
static struct sec_jack_buttons_zone sec_jack_buttons_zones_rev05[] = {
	{
		.code		= KEY_MEDIA,
		.adc_low		= 0,
		.adc_high	= 84,
	},
	{
		.code		= KEY_VOLUMEUP,
		.adc_low		=100,
		.adc_high	= 215,
	},
	{
		.code		= KEY_VOLUMEDOWN,
		.adc_low		= 225,
		.adc_high	= 430,
	},
};


/* To support AT+FCESTEST=1 */
struct switch_dev switch_sendend = {
		.name = "h3w",
};

struct pm860x_headset_info {
	struct pm860x_chip	*chip;
	struct device		*dev;
	struct i2c_client	*i2c;
	int			irq_headset;
	int			irq_hook;
	int			headset_flag;
	unsigned long board_rev;
	unsigned int jack_fore_int_status;
	struct wake_lock jack_wake_lock;
	struct work_struct	work_headset, work_hook;
	struct delayed_work	delayed_work, work_ADC, work_PRESS, jack_deb_work;
	struct headset_switch_data *psw_data_headset, *psw_data_hook;
	struct input_dev *input;
	int hookINT_status; 
	struct input_device_id ids[2];
};

struct headset_switch_data {
	struct switch_dev	sdev;
	const char		*name_on;
	const char		*name_off;
	const char		*state_on;
	const char		*state_off;
	int			irq;
	int state;
};

struct dbg_last_adc
{
	unsigned int evType:3;
	unsigned int adcValue:29;
};
struct dbg_adc_info
{
	int last_key_adc;
	int last_jack_adc;
	unsigned int hist_count:6; // 2^6
};
struct dbg_adc_info dbg_adc;
struct dbg_last_adc dbg_adc_history[64]; // 2^6

static struct device *hsdetect_dev;
static struct PM860X_HS_IOCTL hs_detect;
static int pressed_code = KEY_MEDIA;
struct proc_dir_entry *hs_entry;
static struct regulator *ldo_mic_bias;	//MARVELL>> LDO for HEADSET
static int regulator_ena;				//MARVELL>> LDO for HEADSET

extern long get_board_id(void);

static void sec_hs_dbg_log(int adc, int evType)
{
	if(evType==0)dbg_adc.last_jack_adc=adc;
	if(evType==1)dbg_adc.last_key_adc=adc;

	dbg_adc_history[dbg_adc.hist_count].evType=evType;
	dbg_adc_history[dbg_adc.hist_count++].adcValue=adc;

	if(dbg_adc.hist_count >= 64) dbg_adc.hist_count=0;
}

/* on TD_DKB, the headset jack circuit logic is opposite to the
 * design of levante spec, so if headset status is connected in
 * levante spec, it's actually disconnected. */

static void jack_deb_work_func(struct work_struct *work)
{
	struct pm860x_headset_info	*info =
		container_of(work, struct pm860x_headset_info, jack_deb_work);
	int value;
	value=!gpio_get_value(mfp_to_gpio(MFP_PIN_GPIO79));

	if(value==info->jack_fore_int_status)
	{
		printk("%s : Signal(%d) has been kept by debounce time, continue to process \n", __func__, value);
		queue_work(info->chip->monitor_wqueue, &info->work_headset);
	}
	else
	{
		printk("%s : too short signal, ignore\n", __func__);
	}
}

/* 88PM860x gives us an interrupt when headset is plug-in/unplug */
static irqreturn_t pm860x_headset_handler(int irq, void *data)
{
	struct pm860x_headset_info *info = data;


	/* headset interrupt */
	if (irq == info->irq_headset) {
		wake_lock_timeout(&info->jack_wake_lock, msecs_to_jiffies(5000));	  
		if (info->psw_data_headset != NULL) {
			if( irq == info->irq_headset)
			{
				info->jack_fore_int_status=!gpio_get_value(mfp_to_gpio(MFP_PIN_GPIO79));
				cancel_delayed_work_sync(&info->jack_deb_work);
				//schedule_delayed_work(&info->jack_deb_work, msecs_to_jiffies(200));
				queue_delayed_work(info->chip->monitor_wqueue, &info->jack_deb_work, msecs_to_jiffies(200));
			}
			else
			queue_work(info->chip->monitor_wqueue, &info->work_headset);
		}
	} else if (irq == info->irq_hook) {
	/* hook interrupt */
		if (info->psw_data_hook != NULL) {
			queue_work(info->chip->monitor_wqueue, &info->work_hook);
		}
		else
		{
			/* do nothing */ 
			printk("WRONG INT \n ");
		}
	}

	return IRQ_HANDLED;
}


/* It was temperary source code for checking ADC value every 20 secs .
    However reading adc can consume current . */
#if 0
static void Headset_Adc_work(struct work_struct *work)
{
	struct pm860x_headset_info	*info =
		container_of(work, struct pm860x_headset_info, work_ADC);
	struct headset_switch_data	*switch_data;
	unsigned char value;

	int v_vf_adc;
	pm860x_read_tbat_adc(PM8607_GPADC2_MEAS1, &v_vf_adc);
	printk("SSONG ADC READING OVER AND OVER  ADC :  %d \n",v_vf_adc);

	schedule_delayed_work(&info->work_ADC, HZ*20);
	
}

#endif 

static int Headset_get_adc_data()
{
	int adc_data;
	int adc_max = 0;
	int adc_min = 0xFFFF;
	int adc_total = 0;
	int i;
	for (i = 0; i < SEC_JACK_SAMPLE_SIZE; i++) {
		
	pm860x_read_tbat_adc(PM8607_GPADC2_MEAS1, &adc_data);
		if (i != 0) {
			if (adc_data > adc_max)
				adc_max = adc_data;
			else if (adc_data < adc_min)
				adc_min = adc_data;
		} else {
			adc_max = adc_data;
			adc_min = adc_data;
		}
		adc_total += adc_data;
	}

	return (adc_total - adc_max - adc_min) / (SEC_JACK_SAMPLE_SIZE - 2);
}

static void Hookswitch_press_work(struct work_struct *work)
{
	struct pm860x_headset_info	*info =
		container_of(work, struct pm860x_headset_info, work_PRESS);
	struct headset_switch_data	*switch_data;
	unsigned char value;

	int v_vf_adc, i;

		v_vf_adc = Headset_get_adc_data();
		sec_hs_dbg_log(v_vf_adc, 1);

	if(info->board_rev==8)
	{
		for (i = 0; i < 3; i++)
			if (v_vf_adc >= sec_jack_buttons_zones_rev05[i].adc_low && v_vf_adc <= sec_jack_buttons_zones_rev05[i].adc_high) 
			{
				pressed_code = sec_jack_buttons_zones_rev05[i].code;
				input_report_key(info->input, sec_jack_buttons_zones_rev05[i].code, 1);
				input_sync(info->input);
				pr_info("%s: keycode=%d, is pressed\n", __func__,sec_jack_buttons_zones_rev05[i].code);
			}
	}
	else
	{
		for (i = 0; i < 3; i++)
			if (v_vf_adc >= sec_jack_buttons_zones_rev03[i].adc_low && v_vf_adc <= sec_jack_buttons_zones_rev03[i].adc_high) 
			{
				pressed_code = sec_jack_buttons_zones_rev03[i].code;
				input_report_key(info->input, sec_jack_buttons_zones_rev03[i].code, 1);
				input_sync(info->input);
				pr_info("%s: keycode=%d, is pressed\n", __func__,sec_jack_buttons_zones_rev03[i].code);
			}
			}

}


static int sec_hs_dbg_read_proc(char *buf, char **start, off_t offset, int count, int *eof, void *data)
{
	int len=0;
	int readed_adc=0;
	int i=0;

	readed_adc = Headset_get_adc_data();

	len= sprintf(buf+len, "current_readed_adc=%d\n",readed_adc);
	len+= sprintf(buf+len, "last jack adc=%d\n", dbg_adc.last_jack_adc);
	len+= sprintf(buf+len, "last key adc=%d\n\n", dbg_adc.last_key_adc);
	len+= sprintf(buf+len, "- ADC value Debug history -\n");
	len+= sprintf(buf+len, "- (index=%d, evType, adcValue) -\n", dbg_adc.hist_count);
	for(i=0; i < 64 ; i++)
	{
		len+= sprintf(buf+len, "(%d, %d, %d) - ", i, dbg_adc_history[i].evType, dbg_adc_history[i].adcValue);
	}

	len+= sprintf(buf+len, "\n--------------------------\n");
	return len;
}

static void headset_switch_work(struct work_struct *work)
{
	struct pm860x_headset_info	*info =
		container_of(work, struct pm860x_headset_info, work_headset);
	struct headset_switch_data	*switch_data;
#ifndef CONFIG_MACH_ALKON
	unsigned char value;
#else
	int value, ret;

	int v_vf_adc;
#endif 


#ifndef CONFIG_MACH_ALKON 
	value = (unsigned char)pm860x_reg_read(info->i2c, PM8607_STATUS_1);
	value &= PM8607_STATUS_HEADSET;
	/* on TD_DKB, the headset jack circuit logic is opposite to the
	 * design of levante spec, so if headset status is connected in
	 * levante spec, its actually disconnected.   */
	value = info->headset_flag? !value : value;
#else 
	value = gpio_get_value(mfp_to_gpio(MFP_PIN_GPIO79));
	if(value==0)
		 value = 1;
	else 
		value = 0; 
#endif 


	if (info == NULL) {
		pr_debug("Invalid headset info!\n");
		return;
	}

	/*headset was removed enable the audio-short interrupt*/
	pm860x_set_bits(info->i2c, PM8607_INT_MASK_3, PM8607_INT_EN_AS, 1);

	switch_data = info->psw_data_headset;
	if (switch_data == NULL) {
		pr_debug("Invalid headset switch data!\n");
		return;
	}

	msleep(10);

	if(info->jack_fore_int_status != value)
	{
		printk("%s : interrupt status mismatch\n", __func__);
	}
	
	/* headset detected */
	if (value) {
		switch_data->state = PM860X_HEADSET_ADD;

		if ((info->board_rev==8) && (!IS_ERR(ldo_mic_bias))) { //MARVELL>> LDO for HEADSET
			regulator_enable(ldo_mic_bias);
			regulator_ena = 1; //MARVELL>> LDO for HEADSET fix warning
			pr_info("[%s][%s]regulator_enable(ldo_mic_bias)\n",
			__FILE__, __func__);
			msleep(200);
		}
		else
		{
		/* enable MIC bias to enable hook detection, we must enable mic bias first
		* otherwise we may get false hook detection */
		pm860x_set_bits(info->i2c, PM8607_AUDIO_REG_BASE + PM8607_AUDIO_ADC_ANALOG_PROGRAM1, PM8607_ADC_EN_MIC2_BIAS, 0x60);
		
		//	if (!info->board_rev==8) //MARVELL>> LDO for HEADSET
		/* enable MIC detection to detect hook press*/
		pm860x_set_bits(info->i2c, PM8607_MIC_DECTION, PM8607_MIC_DET_EN_MIC_DET, 1);
		pm860x_set_bits(info->i2c, PM8607_INT_MASK_3, PM8607_INT_EN_HOOK, PM8607_INT_EN_HOOK);

		/* we need to wait some time before the status register goes stable */
		msleep(200);
		}
		
		/* for telephony*/
		kobject_uevent(&hsdetect_dev->kobj, KOBJ_ADD);
		hs_detect.hsdetect_status = PM860X_HEADSET_ADD;


#ifndef CONFIG_MACH_ALKON 
		switch_data->state = PM860X_HEADSET_ADD;


		
		value = (unsigned char)pm860x_reg_read(info->i2c, PM8607_STATUS_1);
		/* Levante issue: use hook status to detect MIC, if detected hook, it's
		 * without MIC, headphone; otherwise, it's headset. */
		value &= PM8607_STATUS_HOOK;
		if (value) {
			switch_data->state = PM860X_HEADPHONE_ADD;
			hs_detect.hsmic_status = PM860X_HS_MIC_REMOVE;
		}

		/* unmask hook interrupt only if the headset has a Mic */
		if (switch_data->state == PM860X_HEADSET_ADD) {
			pm860x_set_bits(info->i2c, PM8607_INT_MASK_3, PM8607_INT_EN_HOOK, PM8607_INT_EN_HOOK);
		} else {
			if (!info->board_rev==8) //MARVELL>> LDO for HEADSET
			/* disable MIC/hook detection if headset does not have a Mic */
			pm860x_set_bits(info->i2c, PM8607_MIC_DECTION, PM8607_MIC_DET_EN_MIC_DET, 0);
		}
#else /* CONFIG_MACH_ALKON */
		pm860x_read_tbat_adc(PM8607_GPADC2_MEAS1, &v_vf_adc);
		printk("HEADSET SWITCH ADC %d \n",v_vf_adc);
		sec_hs_dbg_log(v_vf_adc, 0);

		if( v_vf_adc >=HEADSET_3POLE_LOW && v_vf_adc <=HEADSET_3POLE_HIGH)
		{ // 3POLE
			switch_data->state = PM860X_HEADPHONE_ADD;
			hs_detect.hsmic_status = PM860X_HS_MIC_REMOVE;
			printk("SSONG headset adc is 3pole %d \n",v_vf_adc);
			/* disable mic bias */
			pm860x_set_bits(info->i2c, PM8607_AUDIO_REG_BASE + PM8607_AUDIO_ADC_ANALOG_PROGRAM1, PM8607_ADC_EN_MIC2_BIAS, 0);
			
			if (info->board_rev==8) //MARVELL>> LDO for HEADSET
			{ // rev0.5 disable the LDO
				regulator_disable(ldo_mic_bias);
				regulator_ena=0;
			}
			else //rev0.3 and rev0.4 
			{
				/* headset removed disable MIC/hook detection when headset is */
				pm860x_set_bits(info->i2c, PM8607_MIC_DECTION, PM8607_MIC_DET_EN_MIC_DET, 0);
				/* disable hook interrupt */
				pm860x_set_bits(info->i2c, PM8607_INT_MASK_3, PM8607_INT_EN_HOOK, 0);
				/* disable mic bias */
				pm860x_set_bits(info->i2c, PM8607_AUDIO_REG_BASE + PM8607_AUDIO_ADC_ANALOG_PROGRAM1, PM8607_ADC_EN_MIC2_BIAS, 0);
			}
		}
		else if(v_vf_adc >=HEADSET_4POLE_LOW && v_vf_adc<=HEADSET_4POLE_HIGH)
		{ // 4POLE
			switch_data->state = PM860X_HEADSET_ADD;
			hs_detect.hsmic_status = PM860X_HS_MIC_ADD;
			enable_irq(info->irq_hook);
			info->hookINT_status =1;
			printk("SSONG headset adc is 4pole %d \n",v_vf_adc);
		}
		else
		{
			switch_data->state = PM860X_HEADSET_ADD;
			hs_detect.hsmic_status = PM860X_HS_MIC_ADD;
			printk("SSONG headset debug , ADC is over not in range ..plug out . adc : %d\n" , v_vf_adc);
			
		}	

#endif  /* CONFIG_MACH_ALKON */

	} 
	else {
		if ((info->board_rev==8) && (!IS_ERR(ldo_mic_bias))) { //MARVELL>> LDO for HEADSET
		if (regulator_ena) { //MARVELL>> LDO for HEADSET fix warning
				regulator_disable(ldo_mic_bias);
				regulator_ena=0;
				pr_info("[%s][%s]regulator_disable(ldo_mic_bias)\n",
				__FILE__, __func__);
			}
		}
		else // others Rev0.3 and 0.4
		{
			/* headset removed disable MIC/hook detection when headset is */
			pm860x_set_bits(info->i2c, PM8607_MIC_DECTION, PM8607_MIC_DET_EN_MIC_DET, 0);
			/* disable hook interrupt */
			pm860x_set_bits(info->i2c, PM8607_INT_MASK_3, PM8607_INT_EN_HOOK, 0);
			/* disable mic bias */
			pm860x_set_bits(info->i2c, PM8607_AUDIO_REG_BASE + PM8607_AUDIO_ADC_ANALOG_PROGRAM1, PM8607_ADC_EN_MIC2_BIAS, 0);
		}

		switch_data->state = PM860X_HEADSET_REMOVE;

		/*for telephony*/
		kobject_uevent(&hsdetect_dev->kobj, KOBJ_REMOVE);
		hs_detect.hsdetect_status = PM860X_HEADSET_REMOVE;
		hs_detect.hsmic_status = PM860X_HS_MIC_ADD;
		if(info->hookINT_status)
			disable_irq(info->irq_hook); 

		info->hookINT_status = 0;

	}

	pr_info("headset_switch_work to %d\n", switch_data->state);
	switch_set_state(&switch_data->sdev, switch_data->state);
}

static void hook_switch_work(struct work_struct *work)
{
	struct pm860x_headset_info	*info =
		container_of(work, struct pm860x_headset_info, work_hook);
	struct headset_switch_data	*switch_data;
#ifndef CONFIG_MACH_ALKON
	unsigned char value;
#else /* CONFIG_MACH_ALKON */
	int value;
	int v_vf_adc;
	int headset_gpio;
	int i;
#endif  /* CONFIG_MACH_ALKON */


	if (info == NULL) {
		pr_debug("Invalid headset info!\n");
		return;
	}
	switch_data = info->psw_data_hook;
	if (switch_data == NULL) {
		pr_debug("Invalid hook switch data!\n");
		return;
	}
#ifndef CONFIG_MACH_ALKON

	value = (unsigned char)pm860x_reg_read(info->i2c, PM8607_STATUS_1);
	value &= PM8607_STATUS_HOOK;
#else
	headset_gpio =  gpio_get_value(mfp_to_gpio(MFP_PIN_GPIO79));
	value = gpio_get_value(mfp_to_gpio(MFP_PIN_GPIO80));
	if( (headset_gpio ==0)&&(value==0))
		{
		if(hs_detect.hsmic_status == PM860X_HS_MIC_ADD)
		 value = 1;
		else
			printk("WRONG !!!  \n");
		}
	else 
		value = 0; 
#endif 

	/* hook pressed */
	if (value) {

#ifndef CONFIG_MACH_ALKON
		switch_data->state = PM860X_HOOKSWITCH_PRESSED;
		/*for telephony*/
		kobject_uevent(&hsdetect_dev->kobj, KOBJ_ONLINE);
		hs_detect.hookswitch_status = PM860X_HOOKSWITCH_PRESSED;

	
#else  /* CONFIG_MACH_ALKON */
			switch_data->state = PM860X_HOOKSWITCH_PRESSED;
			cancel_delayed_work(&info->work_PRESS);

			schedule_delayed_work(&info->work_PRESS,msecs_to_jiffies(250));


#endif /*  CONFIG_MACH_ALKON */
		
		
	}


else {
	/* hook released */
		cancel_delayed_work(&info->work_PRESS);
		switch_data->state = PM860X_HOOKSWITCH_RELEASED;
		input_report_key(info->input, pressed_code, value);
		input_sync(info->input);
		hs_detect.hookswitch_status = PM860X_HOOKSWITCH_RELEASED;
		printk("HOOKSWITCH DEBUG GPIO is HIGH \n");

	}
	switch_set_state(&switch_data->sdev, switch_data->state);
}

static ssize_t switch_headset_print_state(struct switch_dev *sdev, char *buf)
{
	struct headset_switch_data	*switch_data =
		container_of(sdev, struct headset_switch_data, sdev);
	const char *state;
	if (switch_get_state(sdev))
		state = switch_data->state_on;
	else
		state = switch_data->state_off;

	if (state)
		return sprintf(buf, "%s\n", state);
	return -1;
}

static int headset_switch_probe(struct platform_device *pdev)
{
	struct pm860x_chip *chip = dev_get_drvdata(pdev->dev.parent);
	struct pm860x_headset_info *info;
	struct pm860x_platform_data *pm860x_pdata;
	struct gpio_switch_platform_data *pdata_headset = pdev->dev.platform_data;
	struct gpio_switch_platform_data *pdata_hook = pdata_headset + 1;
	struct headset_switch_data *switch_data_headset, *switch_data_hook;
	int irq_headset, irq_hook, ret = 0;

	if (pdev->dev.parent->platform_data) {
		pm860x_pdata = pdev->dev.parent->platform_data;
	} else {
		pr_debug("Invalid pm860x platform data!\n");
		return -EINVAL;
	}

	if (pdata_headset == NULL || pdata_hook == NULL) {
		pr_debug("Invalid gpio switch platform data!\n");
		return -EBUSY;
	}

#ifndef CONFIG_MACH_ALKON
	irq_headset = platform_get_irq(pdev, 0);
	if (irq_headset < 0) {
		dev_err(&pdev->dev, "No IRQ resource for headset!\n");
		return -EINVAL;
	}
	irq_hook = platform_get_irq(pdev, 1);
	if (irq_hook < 0) {
		dev_err(&pdev->dev, "No IRQ resource for hook!\n");
		return -EINVAL;
	}
 
#else /* CONFIG_MACH_ALKON */ 

	ret = gpio_request(mfp_to_gpio(MFP_PIN_GPIO79), "headset_detection");
	if (ret) {
		pr_err("%s : gpio_request failed for %d\n",
		       __func__, MFP_PIN_GPIO79);
		gpio_free(mfp_to_gpio(MFP_PIN_GPIO79));
	}

	ret = gpio_request(mfp_to_gpio(MFP_PIN_GPIO80), "hook_detection");
	if (ret) {
		pr_err("%s : gpio_request failed for %d\n",
		       __func__, MFP_PIN_GPIO80);
		gpio_free(mfp_to_gpio(MFP_PIN_GPIO80));
	}


	irq_headset = gpio_to_irq(mfp_to_gpio(MFP_PIN_GPIO79));
	if (irq_headset < 0) {
		pr_err("MFP_PIN_GPIO79 detection Fail to make irq with Headset detection \n");

	}

	irq_hook = gpio_to_irq(mfp_to_gpio(MFP_PIN_GPIO80));
		if (irq_hook < 0) {
		pr_err("MFP_PIN_GPIO80 detection Fail to make irq with Hookswitch detection \n");

	}

#endif /* CONFIG_MACH_ALKON */ 


	info = kzalloc(sizeof(struct pm860x_headset_info), GFP_KERNEL);
	if (!info)
		return -ENOMEM;
	info->chip = chip;
	info->dev = &pdev->dev;
#ifndef CONFIG_MACH_ALKON  /* alkon usd GPIO detection */
	info->irq_headset = irq_headset + chip->irq_base;
	info->irq_hook = irq_hook + chip->irq_base;
#else 
	info->irq_headset = irq_headset;
	info->irq_hook = irq_hook;
	
#endif   /* CONFIG_MACH_ALKON */
	info->headset_flag = pm860x_pdata->headset_flag;
	info->i2c = (chip->id == CHIP_PM8607) ? chip->client : chip->companion;

	info->board_rev=get_board_id();
	printk("%s : 88PM_GET_BOARD_ID = %d \n", __func__, info->board_rev);

	switch_data_headset = kzalloc(sizeof(struct headset_switch_data), GFP_KERNEL);
	if (!switch_data_headset)
		return -ENOMEM;
	switch_data_hook = kzalloc(sizeof(struct headset_switch_data), GFP_KERNEL);
	if (!switch_data_hook)
		return -ENOMEM;

	switch_data_headset->sdev.name = pdata_headset->name;
	switch_data_headset->name_on = pdata_headset->name_on;
	switch_data_headset->name_off = pdata_headset->name_off;
	switch_data_headset->state_on = pdata_headset->state_on;
	switch_data_headset->state_off = pdata_headset->state_off;
	switch_data_headset->sdev.print_state = switch_headset_print_state;
	info->psw_data_headset = switch_data_headset;

	switch_data_hook->sdev.name = pdata_hook->name;
	switch_data_hook->name_on = pdata_hook->name_on;
	switch_data_hook->name_off = pdata_hook->name_off;
	switch_data_hook->state_on = pdata_hook->state_on;
	switch_data_hook->state_off = pdata_hook->state_off;
	switch_data_hook->sdev.print_state = switch_headset_print_state;
	info->psw_data_hook = switch_data_hook;

	ret = switch_dev_register(&switch_data_headset->sdev);
	if (ret < 0)
		goto err_switch_dev_register;
	ret = switch_dev_register(&switch_data_hook->sdev);
	if (ret < 0)
		goto err_switch_dev_register;

#ifndef CONFIG_MACH_ALKON 

	ret = request_threaded_irq(info->irq_headset, NULL, pm860x_headset_handler,
				   IRQF_ONESHOT, "headset", info);
	if (ret < 0) {
		dev_err(chip->dev, "Failed to request IRQ: #%d: %d\n",
			info->irq_headset, ret);
		goto out_irq_headset;
	}
	ret = request_threaded_irq(info->irq_hook, NULL, pm860x_headset_handler,
				   IRQF_ONESHOT, "hook", info);
	if (ret < 0) {
		dev_err(chip->dev, "Failed to request IRQ: #%d: %d\n",
			info->irq_hook, ret);
		goto out_irq_hook;
	}

#else

	ret = request_threaded_irq(info->irq_headset, NULL, pm860x_headset_handler,
				  IRQF_TRIGGER_FALLING|IRQF_TRIGGER_RISING|IRQF_ONESHOT, "headset", info);
	if (ret < 0) {
		dev_err(chip->dev, "Failed to request IRQ: #%d: %d\n",
			info->irq_headset, ret);
		goto out_irq_headset;
	}
	enable_irq_wake(info->irq_headset);
	/* HOOK DETECTION  */
	ret = request_threaded_irq(info->irq_hook, NULL, pm860x_headset_handler,
		   		IRQF_TRIGGER_FALLING|IRQF_TRIGGER_RISING|IRQF_ONESHOT, "hook", info);
	/*  disable hoow switch interrupts */
	disable_irq(info->irq_hook);
	info->hookINT_status = 0;  // disable
	if (ret < 0) {
	printk( "Failed to request IRQ: #%d: %d\n",info->irq_hook, ret);
		free_irq(info->irq_hook,NULL);
	}

#endif 


	platform_set_drvdata(pdev, info);


/* to use earphone volume up/ down button  */

	info->input = input_allocate_device();
	if (info->input == NULL) {
		ret = -ENOMEM;
		pr_err("%s : Failed to allocate input device.\n", __func__);
		goto err_register_input_dev;
	}
	info->input->name = "sec_jack";
	info->input->phys = "headset/input0";
	

      set_bit(EV_KEY, info->input->evbit);


	/* define what keys are available */
	input_set_capability(info->input, EV_KEY, KEY_MEDIA );
	input_set_capability(info->input, EV_KEY, KEY_VOLUMEUP);
	input_set_capability(info->input, EV_KEY, KEY_VOLUMEDOWN);

	ret = input_register_device(info->input);
	if (ret) {
		pr_err("%s : Failed to register driver\n", __func__);
		goto err_register_input_dev;
	}
 
	set_bit(EV_KEY,info->ids[0].evbit);
	info->ids[0].flags = INPUT_DEVICE_ID_MATCH_EVBIT;
	

	/* set hook detection debounce time to 24ms, it's the best setting we experienced */
	pm860x_set_bits(info->i2c, PM8607_HEADSET_DECTION, PM8607_HEADSET_BTN_DBNC, 0x10);
	/* set headset period to 250msec */
	pm860x_set_bits(info->i2c, PM8607_HEADSET_DECTION, PM8607_HEADSET_PERIOD, 0x04);

	#define SANREMO_MIC_BUT_DET_MD_PUP		((0 << 7) | (1 << 6))
	#define SANREMO_MIC_BUT_DET_BTN_PER		((1 << 4) | (1 << 3))
	#define SANREMO_MIC_PERIOD_CONT			((1 << 2) | (1 << 1))

	/* set MIC detection parameter */
    pm860x_reg_write(info->i2c, PM8607_MIC_DECTION,
		(SANREMO_MIC_BUT_DET_MD_PUP |
		SANREMO_MIC_BUT_DET_BTN_PER |
		SANREMO_MIC_PERIOD_CONT));

	/* mask hook interrupt since we don't want the first false hook press down detection
	when inserting a headset without Mic */
	pm860x_set_bits(info->i2c, PM8607_INT_MASK_3, PM8607_INT_EN_HOOK, 0);

	/* enable headset detection */
	pm860x_set_bits(info->i2c, PM8607_HEADSET_DECTION, PM8607_HEADSET_EN_HS_DET, 1);

	wake_lock_init(&info->jack_wake_lock, WAKE_LOCK_SUSPEND, "sec_jack_wake");

	/* Setting MIC_BAIS1 & 2 current capability to max 575uA*/
	pm860x_set_bits(info->i2c, (PM8607_AUDIO_REG_BASE + PM8607_AUDIO_ADC_ANALOG_PROGRAM4),
			PM8607_MICBAIS_CURRENT, 0x03);

#ifdef CONFIG_MACH_ALKON 
	//if (info->board_rev==8) {
	// 8 means Rev0.5, 6 means Rev0.3
	if (info->board_rev == 8) {
		
		ldo_mic_bias = regulator_get(NULL, "v_mic_bias");
		
		pr_info("[%s][%s]ldo_mic_bias = 0x%x\n",
			__FILE__, __func__,(int)ldo_mic_bias);
		
		if (!IS_ERR(ldo_mic_bias))
			printk(KERN_ERR "ldo_mic_bias: success to get regulator\n");
		else
			printk(KERN_ERR "ldo_mic_bias: failed to get regulator\n");
			
		/* make sure the mic det ena is off */
		pm860x_set_bits(info->i2c, PM8607_MIC_DECTION, PM8607_MIC_DET_EN_MIC_DET, 0);
		/* make sure LDO8 is off - need to be done in the OBM*/
		pm860x_set_bits(info->i2c, PM8607_SUPPLIES_EN12, (1 << 2), 0);
		/*set LDO10 vlotage to 1.9V - need to be done in the OBM*/
		pm860x_reg_write(info->i2c, PM8607_LDO8, 0x12);
		
	}
#endif

	INIT_WORK(&info->work_headset, headset_switch_work);
	INIT_WORK(&info->work_hook, hook_switch_work);
	INIT_DELAYED_WORK(&info->work_PRESS, Hookswitch_press_work);
	INIT_DELAYED_WORK(&info->jack_deb_work, jack_deb_work_func);
/* It was temperary source code for checking ADC value every 20 secs .
    However reading adc can consume current . */
//	INIT_DELAYED_WORK(&info->work_ADC, Headset_Adc_work);


//	schedule_delayed_work(&info->work_ADC, 0);
// create proc entry for debugging
	hs_entry = create_proc_read_entry("sec_hs_dbg", S_IRUGO, NULL, sec_hs_dbg_read_proc, NULL);

	hsdetect_dev = &pdev->dev;

	hs_detect.hsdetect_status = 0;
	hs_detect.hookswitch_status = 0;
	/*default 4_POLES*/
	hs_detect.hsmic_status = PM860X_HS_MIC_ADD;

	/* Perform initial detection */
	headset_switch_work(&info->work_headset);
#ifndef CONFIG_MACH_ALKON
	hook_switch_work(&info->work_hook);
#endif 

	return 0;

err_switch_dev_register:
	kfree(switch_data_headset);
	kfree(switch_data_hook);
err_register_input_dev:
	input_free_device(info->input);  
out_irq_hook:
	free_irq(info->irq_headset, info);
out_irq_headset:
	kfree(info);
	return ret;
}

static int __devexit headset_switch_remove(struct platform_device *pdev)
{
	struct pm860x_headset_info *info = platform_get_drvdata(pdev);
	struct headset_switch_data *switch_data_headset = info->psw_data_headset;
	struct headset_switch_data *switch_data_hook = info->psw_data_hook;

	/* disable headset detection */
	pm860x_set_bits(info->i2c, PM8607_HEADSET_DECTION, PM8607_HEADSET_EN_HS_DET, 0);

	if ((info->board_rev==8) && (!IS_ERR(ldo_mic_bias))) //MARVELL>> LDO for HEADSET
		regulator_put(ldo_mic_bias);
	
	cancel_work_sync(&info->work_headset);
	cancel_work_sync(&info->work_hook);

	free_irq(info->irq_hook, info);
	free_irq(info->irq_headset, info);

	switch_dev_unregister(&switch_data_hook->sdev);
	switch_dev_unregister(&switch_data_headset->sdev);

	kfree(switch_data_hook);
	kfree(switch_data_headset);
	kfree(info);

	return 0;
}

static int headset_switch_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct pm860x_headset_info *info = platform_get_drvdata(pdev);
	struct headset_switch_data *switch_data = info->psw_data_headset;
	
	return 0;
}


static int headset_switch_resume(struct platform_device *pdev)
{
	struct pm860x_headset_info *info = platform_get_drvdata(pdev);
	struct headset_switch_data *switch_data = info->psw_data_headset;


	return 0;
}

static int pm860x_hsdetect_ioctl(struct inode *inode,
							struct file *file,
							unsigned int cmd,
							unsigned long arg)
{
	struct PM860X_HS_IOCTL hs_ioctl;

	if (copy_from_user(&hs_ioctl,
					(void *)arg,
					sizeof(struct PM860X_HS_IOCTL)))
		return -EFAULT;

	switch (cmd) {
	case PM860X_HSDETECT_STATUS:
			hs_ioctl.hsdetect_status = \
						hs_detect.hsdetect_status;
			hs_ioctl.hookswitch_status = \
						hs_detect.hookswitch_status;
			#if defined(ENABLE_HS_DETECT_POLES)
			hs_ioctl.hsmic_status =	\
						hs_detect.hsmic_status;
			#endif
			pr_info("hsdetect_ioctl PM860X_HSDETECT_STATUS\n");
	break;

	case PM860X_HOOKSWITCH_STATUS:
			hs_ioctl.hookswitch_status = \
						hs_detect.hookswitch_status;
			hs_ioctl.hsdetect_status = \
						hs_detect.hsdetect_status;
			#if defined(ENABLE_HS_DETECT_POLES)
			hs_ioctl.hsmic_status = \
						hs_detect.hsmic_status;
			#endif
			pr_info("hsdetect_ioctl PM860X_HOOKSWITCH_STATUS\n");
	break;

	default:
			 return -ENOTTY;

	}
	return copy_to_user((void *)arg, &hs_ioctl,
						sizeof(struct PM860X_HS_IOCTL));
}

static struct platform_driver headset_switch_driver = {
	.probe		= headset_switch_probe,
	.remove		= __devexit_p(headset_switch_remove),
	.suspend	= headset_switch_suspend,
	.resume	= headset_switch_resume,
	.driver		= {
		.name	= "88pm860x-headset",
		.owner	= THIS_MODULE,
	},
};

static const struct file_operations pm860x_hsdetect_fops = {
	.owner		= THIS_MODULE,
	.ioctl		= pm860x_hsdetect_ioctl,
};

static struct miscdevice pm860x_hsdetect_miscdev = {
	.minor		= MISC_DYNAMIC_MINOR,
	.name		= "pm860x_hsdetect",
	.fops		= &pm860x_hsdetect_fops,
};
static int __init headset_switch_init(void)
{
	int ret = -EINVAL;
	ret = misc_register(&pm860x_hsdetect_miscdev);
	if (ret < 0)
		return ret;
	ret = platform_driver_register(&headset_switch_driver);
	return ret;
}
// module_init(headset_switch_init);
late_initcall(headset_switch_init);

static void __exit headset_switch_exit(void)
{
	platform_driver_unregister(&headset_switch_driver);
	misc_deregister(&pm860x_hsdetect_miscdev);
}
module_exit(headset_switch_exit);

MODULE_DESCRIPTION("Marvell 88PM860x Headset driver");
MODULE_AUTHOR("Raul Xiong <xjian@marvell.com>");
MODULE_LICENSE("GPL");
