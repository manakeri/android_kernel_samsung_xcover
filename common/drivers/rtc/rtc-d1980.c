/*
 *	Real Time Clock driver for Dialog D1980
 *
 *	Copyright (C) 2011 Dialog Semiconductor Ltd.
 *
 *  	Author: D. Chen, A. Austin, D. Patel
 *
 *  	This program is free software; you can redistribute  it and/or modify it
 *  	under  the terms of  the GNU General  Public License as published by the
 *  	Free Software Foundation;  either version 2 of the  License, or (at your
 *  	option) any later version.
 *
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/time.h>
#include <linux/rtc.h>
#include <linux/bcd.h>
#include <linux/interrupt.h>
#include <linux/ioctl.h>
#include <linux/completion.h>
#include <linux/d1982/rtc.h>
#include <linux/d1982/d1980_reg.h> 
#include <linux/d1982/core.h>
#include <linux/delay.h>
#include <linux/platform_device.h>

#define DRIVER_NAME "d1980-rtc"

#define to_d1980_from_rtc_dev(d) container_of(d, struct d1980, rtc.pdev.dev)
#define RTCMON

#ifdef RTCMON
#include <mach/regs-rtc.h>
#include <linux/miscdevice.h>
DECLARE_WAIT_QUEUE_HEAD(rtc_update_head);

static struct platform_device *g_pdev = NULL;
#endif

#ifdef CONFIG_RTC_DRV_PXA3XX
static struct d1980             *d1980_rtc_g_client = NULL;
static struct setSrAlarmData_t  setSrAlarmData;
static bool                     wakeFromSRRTCAlarm;
static int                      rtcResetOccured;
static unsigned long            initRCNR;
#endif /* CONFIG_RTC_DRV_PXA3XX */

static int _d1980_rtc_readtime(struct device *dev, struct rtc_time *tm);

#ifdef RTCMON
static long d1980_rtcmon_ioctl(struct file *file,
							unsigned int cmd,
							unsigned long arg)
{
	void __user *uarg = (void __user *)arg;
	struct rtc_time tm = {0};
	int ret = 0;

	lock_kernel();

	switch(cmd) {

    	case RTC_CHANGE_SUB:
		{
			DEFINE_WAIT(wait);
			prepare_to_wait(&rtc_update_head,
							&wait,
							TASK_INTERRUPTIBLE);
			schedule();
			finish_wait(&rtc_update_head, &wait);

    		ret = _d1980_rtc_readtime(&g_pdev->dev, &tm);

    		if(ret < 0)
    			pr_info("pxa_rtcmon_ioctl::d1980_rtc_read_time fail]\n");
    		else
    			pr_info("pxa_rtcmon_ioctl::d1980_rtc_read_time=%d:%d:%d\n", \
                        			tm.tm_hour, tm.tm_min, tm.tm_sec);

			ret = copy_to_user(uarg, &tm, sizeof(struct rtc_time));
			if (ret)
				ret = -EFAULT;
    		break;
		}

    	default:
    		pr_info("pxa_rtcmon_ioctl:default\n");
    		ret = -ENOIOCTLCMD;
	}

	unlock_kernel();

	return ret;
}

static int d1980_rtcmon_open(struct inode *inode, struct file *file)
{
	pr_info( \
	"d1980_rtcmon_open::nothing done here\n");
	return 0;
}

static int d1980_rtcmon_release(struct inode *inode, struct file *file)
{
	pr_info( \
	"d1980_rtcmon_release::nothing done here\n");
	return 0;
}
#endif
///////////////////////////////////////////////////////////////////////////


#ifdef CONFIG_RTC_DRV_PXA3XX
/* Retrieve OBM information regarding:
 * Wake up from alarm, power down RTC reset */
static int __init get_SRST(char *p)
{
	u16 reg;
	reg = (u16)(simple_strtol(p, NULL, 16));
	rtcResetOccured = (bool)(reg & 0x1);
	wakeFromSRRTCAlarm = (bool)((reg & 0x4)>>2);
	return 1;
}
__setup("SRST=", get_SRST);

/* returns 1 if Sanremo RTC was woke system up due to alarm
/  returns 0 if not. In any way, after this call, this global var
/  is resetting to 0. */
bool d1980_get_rtc_wu_reason(void)
{
	bool temp = 0;
	temp = wakeFromSRRTCAlarm;
	wakeFromSRRTCAlarm = 0;

	printk(KERN_INFO "[D1980-RTC] %s >> reason = 0x%X\n", __func__, temp);
	return temp;
}

void d1980_set_rtc_last_alarm(unsigned long realAlarm, bool enabled)
{
	setSrAlarmData.realAlarm = realAlarm;
	setSrAlarmData.isEnabled = enabled;
}

void d1980_update_rcnr(unsigned long *rcnr)
{
	*rcnr = initRCNR;

}
EXPORT_SYMBOL(d1980_update_rcnr);
#endif /* CONFIG_RTC_DRV_PXA3XX */


static int d1980_rtc_check_param(struct rtc_time *rtc_tm)
{
	if ((rtc_tm->tm_sec > D1980_RTC_SECONDS_LIMIT) || (rtc_tm->tm_sec < 0))
		return -D1980_RTC_INVALID_SECONDS;

	if ((rtc_tm->tm_min > D1980_RTC_MINUTES_LIMIT) || (rtc_tm->tm_min < 0))
		return -D1980_RTC_INVALID_MINUTES;

	if ((rtc_tm->tm_hour > D1980_RTC_HOURS_LIMIT) || (rtc_tm->tm_hour < 0))
		return -D1980_RTC_INVALID_HOURS;

	if (rtc_tm->tm_mday == 0)
		return -D1980_RTC_INVALID_DAYS;

	if ((rtc_tm->tm_mon > D1980_RTC_MONTHS_LIMIT) || (rtc_tm->tm_mon <= 0))
		return -D1980_RTC_INVALID_MONTHS;

	if ((rtc_tm->tm_year > D1980_RTC_YEARS_LIMIT) || (rtc_tm->tm_year < 0))
		return -D1980_RTC_INVALID_YEARS;

	return 0;
}


/*
 * Read current time and date in RTC
 */
static int _d1980_rtc_readtime(struct device *dev, struct rtc_time *tm)
{
	struct d1980 *d1980 = dev_get_drvdata(dev);
	u8 rtc_time1[6], rtc_time2[6];
	int ret, retries = D1980_GET_TIME_RETRIES;

	/*
	 * Read the time twice and compare.
	 * If rtc_time1 == rtc_time2, then time is valid else retry.
	 */
        do {
		ret = d1980_block_read(d1980, D1980_COUNTS_REG, 6, rtc_time1);
		if (ret < 0)
			return ret;
		ret = d1980_block_read(d1980, D1980_COUNTS_REG, 6, rtc_time2);
		if (ret < 0)
			return ret;

		if (memcmp(rtc_time1, rtc_time2, sizeof(rtc_time1)) == 0) {
			tm->tm_sec = rtc_time1[0] & D1980_RTC_SECS_MASK;
			tm->tm_min = rtc_time1[1] & D1980_RTC_MINS_MASK;
			tm->tm_hour = rtc_time1[2] & D1980_RTC_HRS_MASK;
			tm->tm_mday = (rtc_time1[3] & D1980_RTC_DAY_MASK);
			tm->tm_mon = (rtc_time1[4] & D1980_RTC_MTH_MASK);
			tm->tm_year = (rtc_time1[5] & D1980_RTC_YRS_MASK);

			/* sanity checking */
			ret = d1980_rtc_check_param(tm);
			if (ret)
				return ret;


			tm->tm_yday = rtc_year_days(tm->tm_mday, tm->tm_mon,
							tm->tm_year);
			tm->tm_year += 100;
			tm->tm_mon -= 1;

			dev_dbg(dev, "Read (%d left): %02x %02x %02x %02x %02x %02x\n",
				retries, rtc_time1[0], rtc_time1[1], rtc_time1[2], 
				rtc_time1[3], rtc_time1[4], rtc_time1[5]);


			return 0;
	       }
	} while (retries--);

	dev_err(dev, "timed out reading RTC time\n");
	return -EIO;
}

#ifdef CONFIG_RTC_DRV_PXA3XX
/*
 * Set current time and date in RTC
 */
int d1980_rtc_readtime(struct rtc_time *tm)
{
    struct d1980 *d1980 = NULL;
    u8 rtc_time1[6], rtc_time2[6];
    int ret, retries = D1980_GET_TIME_RETRIES;

    printk(KERN_INFO "[D1980-RTC] %s ...!!\n", __func__);

    if(d1980_rtc_g_client)
    {
        d1980 = d1980_rtc_g_client;
    
        /*
         * Read the time twice and compare.
         * If rtc_time1 == rtc_time2, then time is valid else retry.
         */
        do {
            ret = d1980_block_read(d1980, D1980_COUNTS_REG, 6, rtc_time1);
            if (ret < 0)
                return ret;
    		ret = d1980_block_read(d1980, D1980_COUNTS_REG, 6, rtc_time2);
    		if (ret < 0)
    			return ret;

    		if (memcmp(rtc_time1, rtc_time2, sizeof(rtc_time1)) == 0) {
    			tm->tm_sec = rtc_time1[0] & D1980_RTC_SECS_MASK;
    			tm->tm_min = rtc_time1[1] & D1980_RTC_MINS_MASK;
    			tm->tm_hour = rtc_time1[2] & D1980_RTC_HRS_MASK;
    			tm->tm_mday = (rtc_time1[3] & D1980_RTC_DAY_MASK);
    			tm->tm_mon = (rtc_time1[4] & D1980_RTC_MTH_MASK);
    			tm->tm_year = (rtc_time1[5] & D1980_RTC_YRS_MASK);

    			/* sanity checking */
    			ret = d1980_rtc_check_param(tm);
    			if (ret)
    				return ret;

    			tm->tm_yday = rtc_year_days(tm->tm_mday, tm->tm_mon,
    							tm->tm_year);
    			tm->tm_year += 100;
    			tm->tm_mon -= 1;

                printk(KERN_INFO "[D1980-RTC] %s >> %02d-%02d-%02d %02d:%02d:%02d\n", __func__, \
                                    tm->tm_year, tm->tm_mon, tm->tm_mday, tm->tm_hour, tm->tm_min, tm->tm_sec);
            }
        } while (retries--);
        
		return 0;
    }
	printk(KERN_ERR "timed out reading RTC time\n");

	return -EIO;
}
EXPORT_SYMBOL(d1980_rtc_readtime);
#endif /* CONFIG_RTC_DRV_PXA3XX */


/*
 * Set current time and date in RTC
 */
static int _d1980_rtc_settime(struct device *dev, struct rtc_time *tm)
{
	struct d1980 *d1980 = dev_get_drvdata(dev);
	u8 rtc_time[6];
	u8 rtc_ctrl;
	int ret;

	if((tm->tm_year < 70) || (tm->tm_year > 138)) {
		dev_dbg(dev, "set time %d out of range, ( 1970 to 2038 ).\n", tm->tm_year);
		return -EINVAL;
	}

	tm->tm_year -= 100;
	tm->tm_mon += 1;

	ret = d1980_rtc_check_param(tm);
	if (ret < 0)
		return ret;

        rtc_ctrl = (d1980_reg_read(d1980, D1980_COUNTS_REG) & (~D1980_RTC_SECS_MASK));

	rtc_time[0] = rtc_ctrl | tm->tm_sec;
	rtc_time[1] = tm->tm_min;
	rtc_time[2] = tm->tm_hour;
	rtc_time[3] = tm->tm_mday;
	rtc_time[4] = tm->tm_mon;
	rtc_time[5] = tm->tm_year;

	dev_dbg(dev, "Setting: %02x %02x %02x %02x %02x %02x\n",
		rtc_time[0], rtc_time[1], rtc_time[2], rtc_time[3],
                rtc_time[4], rtc_time[5]);

	/* Write time to RTC */
	ret = d1980_block_write(d1980, D1980_COUNTS_REG, 6, rtc_time);
	if (ret < 0) {
		printk(KERN_CRIT" %s : ERROR\n", __func__);
		return ret;
	}
	
	if (!(d1980_reg_read(d1980, D1980_COUNTY_REG) & D1980_COUNTY_MONITOR))	
		d1980_set_bits(d1980, D1980_COUNTY_REG, D1980_COUNTY_MONITOR);

#ifdef RTCMON
	/* Update all subscribed about RTC set */
	wake_up_all(&rtc_update_head);
	printk(KERN_INFO "[RTC] UPDATE HEAD\n");
#endif

	return 0;
}

#ifdef CONFIG_RTC_DRV_PXA3XX
/*
 * Set current time and date in RTC
 */
int d1980_rtc_settime(struct rtc_time *tm)
{
	struct d1980 *d1980 = NULL;
	u8 rtc_time[6];
	u8 rtc_ctrl;
	int ret;

    printk(KERN_INFO, "D1980 RTC set TIME : %02d-%02d-%02d %02d:%02d:%02d\n", \
                		tm->tm_year, tm->tm_mon,tm->tm_mday, tm->tm_hour, \
                                tm->tm_min, tm->tm_sec);

    if(d1980_rtc_g_client)
    {
        d1980 = d1980_rtc_g_client;

    	tm->tm_year -= 100;
    	tm->tm_mon += 1;

    	ret = d1980_rtc_check_param(tm);
    	if (ret < 0)
    		return ret;

        rtc_ctrl = (d1980_reg_read(d1980, D1980_COUNTS_REG) & (~D1980_RTC_SECS_MASK));

    	rtc_time[0] = rtc_ctrl | tm->tm_sec;  // TODO: Check
    	rtc_time[0] = tm->tm_sec;
    	rtc_time[1] = tm->tm_min;
    	rtc_time[2] = tm->tm_hour;
    	rtc_time[3] = tm->tm_mday;
    	rtc_time[4] = tm->tm_mon;
    	rtc_time[5] = tm->tm_year;

    	printk(KERN_INFO, "[%s] Set date an time : %02d-%02d-%02d %02d:%02d:%02d\n", \
                    		rtc_time[5], rtc_time[4], rtc_time[3], rtc_time[2], \
                                    rtc_time[1], rtc_time[0]);

    	/* Write time to RTC */
    	ret = d1980_block_write(d1980, D1980_COUNTS_REG, 6, rtc_time);
    	if (ret < 0) {
    		printk(KERN_CRIT" %s : ERROR\n", __func__);
    		return ret;
    	}
    	
    	if (!(d1980_reg_read(d1980, D1980_COUNTY_REG) & D1980_COUNTY_MONITOR))	
    		d1980_set_bits(d1980, D1980_COUNTY_REG, D1980_COUNTY_MONITOR);
    }
    else
    {
        printk(KERN_ERR "Failed set RTC time to PMIC chip !\n");
        ret = -EIO;
    }
    
	return ret;
}
EXPORT_SYMBOL(d1980_rtc_settime);
#endif /* CONFIG_RTC_DRV_PXA3XX */

/*
 * Read alarm time and date in RTC
 */
static int d1980_rtc_readalarm(struct device *dev, struct rtc_wkalrm *alrm)
{
    struct d1980 *d1980 = dev_get_drvdata(dev);
	struct rtc_time *tm = &alrm->time;
	u8 time[6];
	int ret;

	ret = d1980_block_read(d1980, D1980_ALARMS_REG, 6, time);
	if (ret < 0)
		return ret;

	tm->tm_min = time[0] & D1980_RTC_ALMSECS_MASK;
	tm->tm_min = time[1] & D1980_RTC_ALMMINS_MASK;
	tm->tm_hour = time[2] & D1980_RTC_ALMHRS_MASK;
	tm->tm_mday = time[3] & D1980_RTC_ALMDAY_MASK;
	tm->tm_mon = time[4] & D1980_RTC_ALMMTH_MASK;
	tm->tm_year = time[5] & D1980_RTC_ALMYRS_MASK;
	
        /* sanity checking */
	ret = d1980_rtc_check_param(tm);
	if (ret < 0)
		return ret;
	
	alrm->enabled = (d1980_reg_read(d1980, D1980_ALARMY_REG) 
                            &  D1980_ALARMY_ALARMON);
  
	tm->tm_year += 100;
	tm->tm_mon -= 1;

	return 0;
}

static int _d1980_rtc_stop_alarm(struct d1980 *d1980)
{
	int ret;

	/* Set RTC_SET to stop the clock */
	ret = d1980_clear_bits(d1980, D1980_ALARMY_REG, D1980_ALARMY_ALARMON);
	if (ret < 0)
		return ret;

	return 0;      
}

#ifdef CONFIG_RTC_DRV_PXA3XX
int d1980_rtc_stop_alarm(void)
{
	int ret;
	struct d1980 *d1980 = NULL;

    if(d1980_rtc_g_client)
    {
        d1980 = d1980_rtc_g_client;

    	/* Set RTC_SET to stop the clock */
    	ret = d1980_clear_bits(d1980, D1980_ALARMY_REG, D1980_ALARMY_ALARMON);
    }
	return ret;
}
#endif /* CONFIG_RTC_DRV_PXA3XX */


static int d1980_rtc_start_alarm(struct d1980 *d1980)
{
	int ret;

    ret = d1980_set_bits(d1980, D1980_ALARMY_REG, D1980_ALARMY_ALARMON);
	if (ret < 0)
		return ret;
	return 0;
}

static void rtc_next_alarm_time(struct rtc_time *next, struct rtc_time *now,
				struct rtc_time *alrm)
{
	unsigned long next_time;
	unsigned long now_time;

	next->tm_year = now->tm_year;
	next->tm_mon = now->tm_mon;
	next->tm_mday = now->tm_mday;
	next->tm_hour = alrm->tm_hour;
	next->tm_min = alrm->tm_min;
	next->tm_sec = alrm->tm_sec;

	rtc_tm_to_time(now, &now_time);
	rtc_tm_to_time(next, &next_time);

	if (next_time < now_time) {
		/* Advance one day */
		next_time += 60 * 60 * 24;
		rtc_time_to_tm(next_time, next);
	}
}


static int _d1980_rtc_setalarm(struct device *dev, struct rtc_wkalrm *alrm)
{
	struct d1980 *d1980 = dev_get_drvdata(dev);
	struct rtc_time *tm = &alrm->time;
	struct rtc_time alarm_tm, now_tm;
	u8 time[6], rtc_ctrl;
	int ret;

	/* Set RTC_SET to stop the clock */
	ret = d1980_clear_bits(d1980, D1980_ALARMY_REG, D1980_ALARMY_ALARMON);
	if (ret < 0)
		return ret;

    _d1980_rtc_readtime(dev, &now_tm);
	rtc_next_alarm_time(&alarm_tm, &now_tm, &alrm->time);

    ret = d1980_rtc_check_param(&alarm_tm);
	if (ret < 0)
	   return ret;

    memset(time, 0, sizeof(time));

	alarm_tm.tm_year -= 100;
	alarm_tm.tm_mon += 1;

    time[0] = alarm_tm.tm_sec;
    rtc_ctrl = (d1980_reg_read(d1980, D1980_ALARMMI_REG) & (~D1980_RTC_ALMMINS_MASK));
    time[1] = rtc_ctrl | alarm_tm.tm_min;
    time[2] |= alarm_tm.tm_hour;
    time[3] |= alarm_tm.tm_mday;
    time[4] |= alarm_tm.tm_mon;
    rtc_ctrl = (d1980_reg_read(d1980, D1980_ALARMY_REG) & (~D1980_RTC_ALMYRS_MASK));
    time[5] = rtc_ctrl | alarm_tm.tm_year;

	/* Write time to RTC */
	ret = d1980_block_write(d1980, D1980_ALARMS_REG, 6, time);
	if (ret < 0)
		return ret;

	if (alrm->enabled)
		ret = d1980_rtc_start_alarm(d1980);

    printk(KERN_INFO "[D1980] RTC set alarm enabled [%d] %d-%d-%d %d:%d:%d\n",
                            alrm->enabled, tm->tm_year, tm->tm_mon, tm->tm_mday, tm->tm_hour, tm->tm_min, tm->tm_sec);

	return ret;
}



static int d1980_rtc_alarm_irq_enable(struct device *dev,
				  unsigned int enabled)
{
	struct d1980 *d1980 = dev_get_drvdata(dev);

    printk(KERN_INFO "[RTC] Alarm irq enable. enabled = %d\n", enabled);
	if (enabled)
		return d1980_rtc_start_alarm(d1980);
	else
		return _d1980_rtc_stop_alarm(d1980);
}

static int d1980_rtc_update_irq_enable(struct device *dev,
					unsigned int enabled)
{
	int ret;
	struct d1980 *d1980 = dev_get_drvdata(dev);

	if (enabled) {
		ret = d1980_set_bits(d1980, D1980_ALARMY_REG, D1980_ALARMY_TICKON);
		if (ret < 0)
			return ret;
	} else {
		ret = d1980_clear_bits(d1980, D1980_ALARMY_REG, D1980_ALARMY_TICKON);
		if (ret < 0)
			return ret;
        }
	return 0;
}

static irqreturn_t d1980_rtc_timer_alarm_handler(int irq, void *data)
{
	struct d1980 *d1980 = data;
	struct rtc_device *rtc = d1980->rtc.rtc;

    _d1980_rtc_stop_alarm(d1980);
	rtc_update_irq(rtc, 1, RTC_AF);

    printk(KERN_CRIT "\nRTC: TIMER ALARM\n");  

	return IRQ_HANDLED;
}

static irqreturn_t d1980_rtc_tick_alarm_handler(int irq, void *data)
{
	struct d1980 *d1980 = data;
	struct rtc_device *rtc = d1980->rtc.rtc;

	kobject_uevent(&rtc->dev.kobj,KOBJ_CHANGE);
	printk(KERN_CRIT "\nRTC: TICK ALARM\n");
	return IRQ_HANDLED;
}


static const struct rtc_class_ops d1980_rtc_ops = {
    .read_time = _d1980_rtc_readtime,
    .set_time = _d1980_rtc_settime,
    .read_alarm = d1980_rtc_readalarm,
    .set_alarm = _d1980_rtc_setalarm,
    .alarm_irq_enable = d1980_rtc_alarm_irq_enable,
    //.update_irq_enable = d1980_rtc_update_irq_enable,
};

#ifdef RTCMON
static const struct file_operations d1980_rtcmon_fops = {
	.owner				= THIS_MODULE,
	.open				= d1980_rtcmon_open,
	.release			= d1980_rtcmon_release,
	.unlocked_ioctl		= d1980_rtcmon_ioctl,
};
static struct miscdevice rtcmon_miscdev = {
	.minor		= MISC_DYNAMIC_MINOR,
	.name		= "rtcmon",
	.fops		= &d1980_rtcmon_fops,
};
#endif



static int d1980_rtc_probe(struct platform_device *pdev)
{
	struct d1980 *d1980 = platform_get_drvdata(pdev);
	struct d1980_rtc *dlg_rtc = &d1980->rtc;
	struct rtc_time tm;
	int ret = 0;

    dev_info(d1980->dev, "Starting RTC\n");

#ifdef CONFIG_RTC_DRV_PXA3XX
    if(d1980) {
        d1980_rtc_g_client = d1980;
        dev_info(d1980->dev, "Set RTC global client variable\n");
    } else {
        dev_err(d1980->dev, "Failed get platform driver data\n");
        return -EIO;
    }
	device_init_wakeup(&pdev->dev, 1);
#endif /* CONFIG_RTC_DRV_PXA3XX */

#ifdef RTCMON
    g_pdev = pdev;
#endif

	ret = d1980_clear_bits(d1980, D1980_WAITCONT_REG, 
			      D1980_WAITCONT_RTCCLOCK);
	if (ret != 0) {
		dev_err(&pdev->dev, "Failed No gating RTC: %d\n", ret);
	}   	
	
#ifdef RTCMON
	ret = misc_register(&rtcmon_miscdev);
	if	(ret < 0) {
		dev_err(&pdev->dev, "Failed to register rtcmon: %d\n", ret);
		goto out_rtc;
	}
#endif

#ifndef CONFIG_RTC_DRV_PXA3XX
	dlg_rtc->rtc = rtc_device_register("d1980", &pdev->dev,
					  &d1980_rtc_ops, THIS_MODULE);
	if (IS_ERR(dlg_rtc->rtc)) {
		ret = PTR_ERR(dlg_rtc->rtc);
		dev_err(&pdev->dev, "failed to register RTC: %d\n", ret);
		return ret;
	}
#endif /* CONFIG_RTC_DRV_PXA3XX */
        
	d1980_register_irq(d1980, D1980_IRQ_EALRAM, d1980_rtc_timer_alarm_handler, 
                            0, "RTC Timer Alarm", d1980);
	d1980_register_irq(d1980, D1980_IRQ_ETICK, d1980_rtc_tick_alarm_handler, 
                            0, "RTC Tick Alarm", d1980);
	dev_info(d1980->dev, "\nRTC registered\n");

	ret = _d1980_rtc_readtime(&pdev->dev,&tm);
	if (ret < 0) {
		dev_err(&pdev->dev, "Failed to read initial time.\n");
		goto out_rtc;
	}
	if((tm.tm_year < 70) || (tm.tm_year > 138)) {
		tm.tm_year = 111;
		tm.tm_mon = 0;
		tm.tm_mday = 1;
		tm.tm_hour = 0;
		tm.tm_min = 0;
		tm.tm_sec = 0;
		ret = _d1980_rtc_settime(&pdev->dev,&tm);
		if (ret < 0) {
			dev_err(&pdev->dev, "Failed to set initial time.\n");
			goto out_rtc;
		}
	}

	return 0;

out_rtc:
    return ret;
}

static int __devexit d1980_rtc_remove(struct platform_device *pdev)
{
	struct d1980 *d1980 = platform_get_drvdata(pdev);
	struct d1980_rtc *dlg_rtc = &d1980->rtc;

	d1980_free_irq(d1980, D1980_IRQ_EALRAM);

#ifndef CONFIG_RTC_DRV_PXA3XX
	rtc_device_unregister(dlg_rtc->rtc);
#endif /* CONFIG_RTC_DRV_PXA3XX */

	return 0;
}


#ifdef CONFIG_PM
static int d1980_rtc_suspend(struct device *dev)
{
	struct d1980 *d1980 = dev_get_drvdata(dev);

	if (device_may_wakeup(dev)) {
		// TODO:enable_irq_wake(info->chip->core_irq);
		// TODO:enable_irq_wake(info->irq);
	}
	return 0;
}

static int d1980_rtc_resume(struct device *dev)
{
	struct d1980 *d1980 = dev_get_drvdata(dev);

	if (device_may_wakeup(dev)) {
		// TODO:disable_irq_wake(info->chip->core_irq);
		// TODO:disable_irq_wake(info->irq);
	}
	return 0;
}

static struct dev_pm_ops d1980_rtc_pm_ops = {
	.suspend	= d1980_rtc_suspend,
	.resume		= d1980_rtc_resume,
};
#endif



static struct platform_driver d1980_rtc_driver = {
	.probe = d1980_rtc_probe,
	.remove = __devexit_p(d1980_rtc_remove),
	.driver = {
		.name = DRIVER_NAME,
		.owner = THIS_MODULE,
#ifdef CONFIG_PM
		.pm	= &d1980_rtc_pm_ops,
#endif
	},
};

static int __init d1980_rtc_init(void)
{
	return platform_driver_register(&d1980_rtc_driver);
}
module_init(d1980_rtc_init);

static void __exit d1980_rtc_exit(void)
{
	platform_driver_unregister(&d1980_rtc_driver);
#ifdef RTCMON
	misc_deregister(&rtcmon_miscdev);
#endif
}
module_exit(d1980_rtc_exit);

MODULE_AUTHOR("Dialog Semiconductor Ltd <divyang.patel@diasemi.com>");
MODULE_DESCRIPTION("RTC driver for the Dialog D1980 PMIC");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:" DRIVER_NAME);
