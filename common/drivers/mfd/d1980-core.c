/*
 * d1980-core.c  --  Device access for Dialog D1980
 *   
 * Copyright(c) 2011 Dialog Semiconductor Ltd.
 *  
 * Author: Dialog Semiconductor Ltd. D. Chen, A Austin, D. Patel
 *  
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>

#include <linux/bug.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/workqueue.h>
#include <linux/irq.h>
#include <linux/ioport.h>
#include <linux/kthread.h>

#include <linux/d1982/d1980_reg.h> 
#include <linux/d1982/core.h>
#include <linux/d1982/pmic.h>
#include <linux/d1982/rtc.h>
#include <linux/d1982/bat.h>
#include <mach/gpio.h>

#include <linux/proc_fs.h>

#undef CACHE_D1980_I2C
#if defined(CACHE_D1980_I2C)
#include "d1980-cache.h"
#else
#define pmic_cache_stat_print(RST)
#define pmic_cache_init(ID)
#define pmic_cache_hit_before_read(ID, REG, COUNT, PDATA)		(-1)
#define pmic_cache_hit_before_write(ID, REG, COUNT, PDATA)		(-1)
#define pmic_cache_save_after_readwrite(ID, I2C_RET, REG, COUNT, PDATA)
#define pmic_cache_invalidate(ID, REG)
#endif /* CACHE_D1980_I2C */


#define WILDCAT_DEBUG


#if defined(WILDCAT_DEBUG)
#include <asm/uaccess.h>

int wildcat_print = 0;
#endif

/*
 *   Static global variable
 */

/*
 * D1980 Device IO
 */
static DEFINE_MUTEX(io_mutex);



static int d1980_read(struct d1980 *d1980, u8 reg, int num_regs, u8 * dest)
{
	int bytes = num_regs;
#if defined(WILDCAT_DEBUG)
	int ret = 0;
#endif

	if (d1980->read_dev == NULL)
		return -ENODEV;

	if ((reg + num_regs - 1) > D1980_MAX_REGISTER_CNT) {
		dev_err(d1980->dev, "invalid reg %x\n", reg + num_regs - 1);
		return -EINVAL;
	}

	dev_dbg(d1980->dev, "%s R%d(0x%2.2x) %d regs\n", __func__, reg, reg, num_regs);

#if defined(WILDCAT_DEBUG)
    ret = d1980->read_dev(d1980, reg, bytes, (char *)dest);

    if(wildcat_print)
	    dev_info(d1980->dev, "Read R%d(0x%02x). 0x%02x \n", reg, reg, *dest);

    return ret;
#else
	/* Actually write it out */
	return d1980->read_dev(d1980, reg, bytes, (char *)dest);
#endif
}

static int d1980_write(struct d1980 *d1980, u8 reg, int num_regs, u8 * src)
{
	int bytes = num_regs;
#if defined(WILDCAT_DEBUG)
	int ret = 0;
#endif

	if (d1980->write_dev == NULL)
		return -ENODEV;

	if ((reg + num_regs - 1) > D1980_MAX_REGISTER_CNT) {
		dev_err(d1980->dev, "invalid reg %x\n",
			reg + num_regs - 1);
		return -EINVAL;
	}

#if defined(WILDCAT_DEBUG)
    ret = d1980->write_dev(d1980, reg, bytes, (char *)src);

    if(wildcat_print)
	    dev_info(d1980->dev, "Write R%d(0x%02x). 0x%02x\n", reg, reg, *src);

    return ret;
#else
	/* Actually write it out */
	return d1980->write_dev(d1980, reg, bytes, (char *)src);
#endif
}

/*
 * Safe read, modify, write methods
 */
int d1980_clear_bits(struct d1980 * const d1980, u8 const reg, u8 const mask)
{
	u8 data;
	int err;

	mutex_lock(&io_mutex);
	err = pmic_cache_hit_before_read(PMIC_ID, reg, 1, &data);
	if(err < 0) {
	err = d1980_read(d1980, reg, 1, &data);
	if (err) {
		dev_err(d1980->dev, "read from reg R%d failed\n", reg);
    		// commented 21/July/2011. goto out;
    	}
    	pmic_cache_save_after_readwrite(PMIC_ID, err, reg, 1, &data);
	}

    if(err >= 0) {
	data &= ~mask;
	err = d1980_write(d1980, reg, 1, &data);
	if (err)
		dev_err(d1980->dev, "write to reg R%d failed\n", reg);
        pmic_cache_save_after_readwrite(PMIC_ID, err, reg, 1, &data);
	}
out:
	mutex_unlock(&io_mutex);
	return err;
}
EXPORT_SYMBOL_GPL(d1980_clear_bits);



int d1980_set_bits(struct d1980 * const d1980, u8 const reg, u8 const mask)
{
	u8 data;
	int err;

	mutex_lock(&io_mutex);
	err = pmic_cache_hit_before_read(PMIC_ID, reg, 1, &data);
	if(err < 0) {
	err = d1980_read(d1980, reg, 1, &data);
	if (err) {
    		dev_err(d1980->dev, "### Read from reg R%d failed ### \n", reg);
    		// commented 21/July/2011. goto out;
        }
        pmic_cache_save_after_readwrite(PMIC_ID, err, reg, 1, &data);
	}

    if(err >= 0) {
	data |= mask;
	err = d1980_write(d1980, reg, 1, &data);
	if (err)
		dev_err(d1980->dev, "write to reg R%d failed\n", reg);
        pmic_cache_save_after_readwrite(PMIC_ID, err, reg, 1, &data);
    }
out:
	mutex_unlock(&io_mutex);
	return err;
}
EXPORT_SYMBOL_GPL(d1980_set_bits);


u8 d1980_reg_read(struct d1980 * const d1980, u8 const reg)
{
	u8 data;
	int err;


	mutex_lock(&io_mutex);
	err = pmic_cache_hit_before_read(PMIC_ID, reg, 1, &data);
	if(err < 0) {
	err = d1980_read(d1980, reg, 1, &data);
	if (err)
		dev_err(d1980->dev, "read from reg R%d failed\n", reg);
        pmic_cache_save_after_readwrite(PMIC_ID, err, reg, 1, &data);
	}
	mutex_unlock(&io_mutex);
	return data;
}
EXPORT_SYMBOL_GPL(d1980_reg_read);


int d1980_reg_write(struct d1980 * const d1980, u8 const reg, u8 const val)
{
	int ret;
	u8 data = val;

	mutex_lock(&io_mutex);
	ret = pmic_cache_hit_before_write(PMIC_ID, reg, 1, &data);
	if(ret < 0) {
	ret = d1980_write(d1980, reg, 1, &data);
	if (ret)
		dev_err(d1980->dev, "write to reg R%d failed\n", reg);
        pmic_cache_save_after_readwrite(PMIC_ID, ret, reg, 1, &data);
	}
	mutex_unlock(&io_mutex);
	return ret;
}
EXPORT_SYMBOL_GPL(d1980_reg_write);


int d1980_block_read(struct d1980 * const d1980, u8 const start_reg, u8 const regs,
		      u8 * const dest)
{
	int err = 0;

	mutex_lock(&io_mutex);
    err = pmic_cache_hit_before_read(PMIC_ID, start_reg, regs, dest);
    if( err < 0) {
	err = d1980_read(d1980, start_reg, regs, dest);
	if (err)
    		dev_err(d1980->dev, "block read starting from R%d failed\n", start_reg);
        pmic_cache_save_after_readwrite(PMIC_ID, err, start_reg, regs, dest);
    }
	mutex_unlock(&io_mutex);
	return err;
}
EXPORT_SYMBOL_GPL(d1980_block_read);


int d1980_block_write(struct d1980 * const d1980, u8 const start_reg, u8 const regs,
		       u8 * const src)
{
	int ret = 0;

	mutex_lock(&io_mutex);
	ret = pmic_cache_hit_before_write(PMIC_ID, start_reg, regs, src);
	if(ret < 0) {
	ret = d1980_write(d1980, start_reg, regs, src);
	if (ret)
    		dev_err(d1980->dev, "block write starting at R%d failed\n", start_reg);
        pmic_cache_save_after_readwrite(PMIC_ID, ret, start_reg, regs, src);
	}
	mutex_unlock(&io_mutex);
	return ret;
}
EXPORT_SYMBOL_GPL(d1980_block_write);

extern void d1980_pi2c_write2bulk1(int value);
void d1980_pi2c_set_bulk1_control(int value)
{
   // mutex_lock(&io_mutex);
    d1980_pi2c_write2bulk1(value);
   // mutex_unlock(&io_mutex);
}

EXPORT_SYMBOL_GPL(d1980_pi2c_set_bulk1_control);




/*
 * Register a client device.  This is non-fatal since there is no need to
 * fail the entire device init due to a single platform device failing.
 */
static void d1980_client_dev_register(struct d1980 *d1980,
				       const char *name,
				       struct platform_device **pdev)
{
	int ret;

	*pdev = platform_device_alloc(name, -1);
	if (*pdev == NULL) {
		dev_err(d1980->dev, "Failed to allocate %s\n", name);
		return;
	}

	(*pdev)->dev.parent = d1980->dev;
	platform_set_drvdata(*pdev, d1980);
	ret = platform_device_add(*pdev);
	if (ret != 0) {
		dev_err(d1980->dev, "Failed to register %s: %d\n", name, ret);
		platform_device_put(*pdev);
		*pdev = NULL;
	}
}

int d1980_adc_manual_conversion(struct d1980 * const d1980, const u8 chnl)
{ 
	u8 mux_sel=0, flag;
	int ret = 0, retries = D1980_MANUAL_READ_RETRIES;

	mutex_lock(&d1980->adc_mutex);

	if ((chnl >= 0) && (chnl < D1980_ADC_NUMBER_OF_CHANNELS)) {		
		switch (chnl)
		{
			case D1980_ADCMAN_MUXSEL_VBAT:			
				mux_sel = D1980_ADCMAN_MUXSEL_VBAT;
				break;
			case D1980_ADCMAN_MUXSEL_TEMP:
				mux_sel = D1980_ADCMAN_MUXSEL_TEMP;
				break;
			case D1980_ADCMAN_MUXSEL_VF:
				mux_sel = D1980_ADCMAN_MUXSEL_VF; 
				break;
			case D1980_ADCMAN_MUXSEL_ADCIN:
				mux_sel = D1980_ADCMAN_MUXSEL_ADCIN; 
				break;
			case D1980_ADCMAN_MUXSEL_TJUNC:
				mux_sel = D1980_ADCMAN_MUXSEL_TJUNC; 
				break;
			case D1980_ADCMAN_MUXSEL_VBBAT:
				mux_sel = D1980_ADCMAN_MUXSEL_VBBAT; 
				break;
			default:
				dev_err(d1980->dev, "\nChannel Selected is out of range \n");
				return -EIO;
				break;
		}
	} else {
		dev_err(d1980->dev, "\nChannel Selected is out of range \n");
		return -EIO;
	}

	d1980->adc_res[chnl].adc_flag = FALSE;
	
	mux_sel |= D1980_ADCMAN_MANCONV; 
	/* This will generate an event */
	d1980_reg_write(d1980, D1980_ADCMAN_REG, mux_sel);

	do {
	    // Change schedule time. 14/July/2011
		schedule_timeout_interruptible(msecs_to_jiffies(1));
		flag = d1980->adc_res[chnl].adc_flag;
		if (flag == TRUE) {
			ret = 0;
		}		
	} while (retries-- && (flag == FALSE));

	mutex_unlock(&d1980->adc_mutex);

	if (flag == FALSE) 
		ret = -EIO;
	return ret;
}
EXPORT_SYMBOL_GPL(d1980_adc_manual_conversion);

static irqreturn_t d1980_adc_event_handler(int irq, void *data)
{
	struct d1980 *d1980 = data;
	u16 result_data = 0, tmp_data = 0;
	u8 tmp;
	
	/* ADC has 12 bit resolution */
	tmp = d1980_reg_read(d1980, D1980_ADCRESH_REG);
	result_data = tmp;
	result_data = (result_data << 4);
	
	tmp = d1980_reg_read(d1980, D1980_ADCRESL_REG);
	tmp_data = tmp & 0x000F;
	result_data |= tmp_data;
	
	tmp = d1980_reg_read(d1980, D1980_ADCMAN_REG) & 0x0F;

    d1980->adc_res[tmp].adc_buf = result_data;
    d1980->adc_res[tmp].adc_flag = TRUE;

	return IRQ_HANDLED;
}



static int d1980_adc_start (struct d1980 *d1980)
{
	int i;

	mutex_init(&d1980->adc_mutex);
	
	for (i=0; i< D1980_ADC_NUMBER_OF_CHANNELS; i++) {
		d1980->adc_res[i].adc_buf = 0;
		d1980->adc_res[i].adc_flag = FALSE;
	}
	
	d1980_register_irq(d1980, D1980_IRQ_EADCEOM, d1980_adc_event_handler, 
				0, "ADC EOM", d1980);
				
	//Comment out below statement to use ADC-In 
	//d1980_reg_write(d1980, D1980_ADCCONT_REG, 0);
	dev_warn(d1980->dev, "d1980_adc_start [Exit] \n");
	return 0;
}


static int d1980_adc_stop (struct d1980 *d1980)
{
	d1980_free_irq(d1980, D1980_IRQ_EADCEOM);
	return 0;
}


static void d1980_worker_init(unsigned int irq)
{
	int err;
    unsigned int d1980_int_pin = IRQ_TO_GPIO(irq);

	err = gpio_request(d1980_int_pin, "D1980_IRQ_GPIO");
	if (err) {
		gpio_free(d1980_int_pin);
		printk(KERN_ERR "Request GPIO_%d failed with %d\n", d1980_int_pin, err);
		return;
	}
	gpio_direction_input(d1980_int_pin);
}


#if defined(CONFIG_PROC_FS) && defined(WILDCAT_DEBUG)

#define	D1980_PROC_FILE	"driver/wildcat"

static struct   d1980           *d1980_g_client = NULL;
static struct   proc_dir_entry  *d1980_proc_file;
static int      index;

/*
 * (1) cat /proc/driver/wildcat
 * will print the value of register located in address of index.
 */
static ssize_t d1980_proc_read(struct file *filp,
	char __user *buffer, size_t count, loff_t *offset)
{
	u8 reg_val;
	ssize_t len;
	const char fmt[] =    "Register 0x%02x: 0x%02x\n";
	const char fmtErr[] = "Register 0x%02x: failed\n";

	char buf[sizeof(fmt)];

	if ((index < 0) || (index > D1980_PAGE1_REG_END))
		return 0;
		
	pmic_cache_invalidate(PMIC_ID, index);
	if (reg_val = d1980_reg_read(d1980_g_client, index))
		len = sprintf(buf, fmt, index, reg_val);
	else
		len = sprintf(buf, fmtErr, index);

	return simple_read_from_buffer(buffer, count, offset, buf, len);
}

/*
 * (1) echo -0x0b > /proc/driver/wildcat
 * will set index to 0x0b such that next proc_read will be done from this new index value.
 *
 * (2) echo 0x0b > /proc/driver/wildcat
 * will set value of 0x0b for the register in address of index
 *
* */
static ssize_t d1980_proc_write(struct file *filp,
		const char *buff, size_t len, loff_t *off)
{
	int ret;
	u8 reg_val, i, j;
	char messages[256], vol[256];
    struct d1980 *d1980 = NULL;

    if(d1980_g_client)
        d1980 = d1980_g_client;

	if (len > 256)
		len = 256;

	if (copy_from_user(messages, buff, len))
		return -EFAULT;

	if ('-' == messages[0]) {
		/* set the register index */
		memcpy(vol, messages+1, len-1);
		index = (int) simple_strtoul(vol, NULL, 16);
		printk(KERN_INFO "\nwildcat_proc_write:register # was set = 0x%x\n", index);
	} else if ('?' == messages[0]) {
		printk(KERN_INFO "\n write: echo -0x56 > /proc/driver/wildcat \n");
		printk(KERN_INFO " read(reg from write): cat /proc/driver/wildcat \n");
		printk(KERN_INFO " read/write trace on: echo traceon > /proc/driver/wildcat \n");
		printk(KERN_INFO " BUCKs & LDOs status : echo power_status > /proc/driver/wildcat \n");
		printk(KERN_INFO " print cache statistic: echo stat\n\n");
		printk(KERN_INFO " print cache statistic: echo stat_reset\n\n");
	}else if (strncmp(messages, "traceon", 7) == 0){
		wildcat_print = 1;
		printk(KERN_INFO "\nwildcat_proc_write:enable wildcat read/write traces \n");
	} else if (strncmp(messages, "stat_reset", 5) == 0) {
    	pmic_cache_stat_print(1);
	} else if (strncmp(messages, "stat", 4) == 0) {
		pmic_cache_stat_print(0);
	}else if (strncmp(messages, "power_status", 12) == 0){
		wildcat_print = 0;

        printk(KERN_INFO "\n=============================================================\n");
		for(i = D1980_BUCK1_REG, j = 1; i <= D1980_BUCK4_REG; i++, j++)
		{
		    reg_val = d1980_reg_read(d1980, i);
            printk(KERN_INFO "[ BUCK%d(0x%x)] is turned %s\n",j, i, (reg_val & D1980_BUCK_LDO_EN) ? "ON" : "OFF");
	    }
		for(i = D1980_LDO1_REG, j = 1; i <= D1980_LDO13_REG; i++, j++)
		{
		    reg_val = d1980_reg_read(d1980, i);
            printk(KERN_INFO "[ LDO%d(0x%x)] is turned %s\n",j, i, (reg_val & D1980_BUCK_LDO_EN) ? "ON" : "OFF");
	    }
		for(i = D1980_LDO14_REG, j = 14; i <= D1980_LDO19_REG; i++, j++)
		{
		    reg_val = d1980_reg_read(d1980, i);
            printk(KERN_INFO "[ LDO%d(0x%x)] is turned %s\n",j, i, (reg_val & D1980_BUCK_LDO_EN) ? "ON" : "OFF");
	    }

        reg_val = d1980_reg_read(d1980, D1980_SUPPLY_REG);
        printk(KERN_INFO "[ AUDIO_A and B(0x%x)] is turned %s\n",D1980_SUPPLY_REG, (reg_val & D1980_SUPPLY_LDO15AEN) ? "ON" : "OFF");
        printk(KERN_INFO "[ AUDIO_A_VSEL (0x%x)] is %s V\n",D1980_SUPPLY_REG, (reg_val & D1980_SUPPLY_LDO15BEN) ? "1.6" : "1.2");
        printk(KERN_INFO "[CLASS D(0x%x)] is turned %s\n",D1980_AUDIO_CONF2_REG_B, (reg_val & D1980_AUDIOCONF_CLASSDEN) ? "ON" : "OFF");
        printk(KERN_INFO "=============================================================\n");
	} else {
		/* set the register value */
		reg_val = (int)simple_strtoul(messages, NULL, 16);
		reg_val &= 0xFF;
        pmic_cache_invalidate(PMIC_ID, index);
		d1980_reg_write(d1980, index, reg_val);
		printk(KERN_INFO "d1980_proc_write:register write 0x%x <- %x \n", index, (reg_val & 0xFF));
	}
	return len;
}

static struct file_operations d1980_proc_ops = {
	.read = d1980_proc_read,
	.write = d1980_proc_write,
};

static void create_d1980_proc_file(void)
{
	d1980_proc_file = create_proc_entry(D1980_PROC_FILE, 0644, NULL);
	if (d1980_proc_file) {
		d1980_proc_file->proc_fops = &d1980_proc_ops;
	} else
		printk(KERN_INFO "proc file create failed!\n");
}

static void remove_d1980_proc_file(void)
{
	remove_proc_entry(D1980_PROC_FILE, NULL);
}
#endif


int d1980_device_init(struct d1980 *d1980, int irq,
		       struct d1980_platform_data *pdata)
{
	int ret, tmp;
	struct regulator *regulator;

#if defined(WILDCAT_DEBUG)
    if(d1980)
        d1980_g_client = d1980;
#endif

	d1980->pmic.max_dcdc = 21;

	ret = d1980_set_bits(d1980, D1980_POWERCONT_REG, D1980_POWERCONT_nSLEEP);
    ret = d1980_reg_write(d1980, D1980_BBATCONT_REG, 0x1F);   // 3.3V, 100uA
	ret = d1980_clear_bits(d1980, D1980_CONTROLB_REG, D1980_CONTROLB_WRITEMODE);
	if (ret) 
		goto err;

	// 2011/09/06. LDO4. Enable Pull down Resistor for SIM poweroff imediately
	ret = d1980_clear_bits(d1980, D1980_PULLDOWN_REG_A, D1980_PULLDOWN_A_LDO4PDDIS);
  if (ret)
    goto err;


	if (pdata && pdata->irq_init) {
		printk(KERN_CRIT "\nD1980-core.c: IRQ PIN Configuration \n");
		ret = pdata->irq_init();
		if (ret != 0) {
			dev_err(d1980->dev, "Platform init() failed: %d\n", ret);
			goto err_irq;
		}
	}

	ret = d1980_irq_init(d1980, irq, pdata);
	if (ret < 0)
		goto err;
    d1980_worker_init(irq);
    
	if (pdata && pdata->init) {
		ret = pdata->init(d1980);
		if (ret != 0) {
			dev_err(d1980->dev, "Platform init() failed: %d\n", ret);
			goto err_irq;
		}
	}
	d1980_adc_start(d1980);	
    // Regulator Specific Init
	ret = d1980_platform_regulator_init(d1980);
	if (ret != 0) {
		dev_err(d1980->dev, "Platform Regulator init() failed: %d\n", ret);
		goto err_irq;
	}

	d1980_client_dev_register(d1980, "d1980-rtc", &(d1980->rtc.pdev));
	d1980_client_dev_register(d1980, "d1980-onkey", &(d1980->onkey.pdev));
	d1980_client_dev_register(d1980, "d1980-power", &(d1980->power.pdev));
	d1980_client_dev_register(d1980, "d1980-hsdetect", &(d1980->hsdetect.pdev));
	d1980_client_dev_register(d1980, "android-vibrator", &(d1980->vibrator.pdev));

#if defined(CONFIG_PROC_FS) && defined(WILDCAT_DEBUG)
	create_d1980_proc_file();
#endif

	return 0;

err_irq:
	d1980_irq_exit(d1980);
err:
	printk(KERN_CRIT "\n\nD1980-core.c: device init failed ! \n\n");
	return ret;
}
EXPORT_SYMBOL_GPL(d1980_device_init);

void d1980_device_exit(struct d1980 *d1980)
{
	int i;
	d1980_adc_stop(d1980);
	for (i = 0; i < ARRAY_SIZE(d1980->pmic.pdev); i++)
		platform_device_unregister(d1980->pmic.pdev[i]);

	platform_device_unregister(d1980->rtc.pdev);
	platform_device_unregister(d1980->onkey.pdev);
	platform_device_unregister(d1980->power.pdev);
	platform_device_unregister(d1980->hsdetect.pdev);
	platform_device_unregister(d1980->vibrator.pdev);

	d1980_irq_exit(d1980);

#if defined(CONFIG_PROC_FS) && defined(WILDCAT_DEBUG)
	remove_d1980_proc_file();
#endif

#ifdef D1980_INT_USE_THREAD
    if(d1980->irq_task)
	    kthread_stop(d1980->irq_task);
#endif
}
EXPORT_SYMBOL_GPL(d1980_device_exit);

MODULE_AUTHOR("Dialog Semiconductor Ltd <divyang.patel@diasemi.com>");
MODULE_DESCRIPTION("D1980 PMIC Core");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:" D1980_I2C);
