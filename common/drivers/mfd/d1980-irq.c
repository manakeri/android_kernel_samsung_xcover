/*
 * d1980-irq.c: IRQ support for Dialog D1980
 *   
 * Copyright(c) 2011 Dialog Semiconductor Ltd.
 *  
 * Author: Dialog Semiconductor Ltd. D. Chen, A. Austin, D. Patel
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
#include <linux/kthread.h>

#include <asm/mach/irq.h>
#include <asm/gpio.h>
 
#include <linux/d1982/d1980_reg.h> 
#include <linux/d1982/core.h>
#include <linux/d1982/pmic.h>
#include <linux/d1982/rtc.h>
#include <linux/d1982/bat.h>


#define D1980_NUM_IRQ_EVT_REGS			4

#define D1980_INT_OFFSET_1			0
#define D1980_INT_OFFSET_2			1
#define D1980_INT_OFFSET_3			2   
#define D1980_INT_OFFSET_4			3    

struct d1980_irq_data {
	int reg;
	int mask;
};

static struct d1980_irq_data d1980_irqs[] = {
	/* EVENT Register A start */
	[D1980_IRQ_EVBATLOW] = {
		.reg = D1980_INT_OFFSET_1,
		.mask = D1980_IRQMASKA_BIT4,
	},
	[D1980_IRQ_EALRAM] = {
		.reg = D1980_INT_OFFSET_1,
		.mask = D1980_IRQMASKA_MALRAM,
	},
	[D1980_IRQ_ESEQRDY] = {
		.reg = D1980_INT_OFFSET_1,
		.mask = D1980_IRQMASKA_MSEQRDY,		
	},
	[D1980_IRQ_ETICK] = {
		.reg = D1980_INT_OFFSET_1,
		.mask = D1980_IRQMASKA_MTICK,
	}, 
	/* EVENT Register B start */
	[D1980_IRQ_ENONKEY] = {
		.reg = D1980_INT_OFFSET_2,
		.mask = D1980_IRQMASKB_MNONKEY,		
	},
	[D1980_IRQ_ETBAT] = {
		.reg = D1980_INT_OFFSET_2,
		.mask = D1980_IRQMASKB_BIT4,		
	},
	[D1980_IRQ_EADCEOM] = {
		.reg = D1980_INT_OFFSET_2,
		.mask = D1980_IRQMASKB_MADCEOM,
	},
	/* EVENT Register C start */
 	[D1980_IRQ_EGPI0] = {
		.reg = D1980_INT_OFFSET_3,
		.mask = D1980_IRQMASKC_MGPI0,		
	},
	[D1980_IRQ_EGPI1] = {
		.reg = D1980_INT_OFFSET_3,
		.mask = D1980_IRQMASKC_MGPI1,
	},  	
	[D1980_IRQ_EAUDIO] = {
		.reg = D1980_INT_OFFSET_3,
		.mask = D1980_IRQMASKC_MAUDIO,		
	},
	[D1980_IRQ_ETA] = {
	    .reg = D1980_INT_OFFSET_3,
	    .mask = D1980_IRQMASKC_TA,
	},
	/* EVENT Register D start */
	[D1980_IRQ_EGPI2] = {
		.reg = D1980_INT_OFFSET_4,
		.mask = D1980_IRQMASKD_MGPI2,
	},
	[D1980_IRQ_EGPI3] = {
		.reg = D1980_INT_OFFSET_4,
		.mask = D1980_IRQMASKD_MGPI3,		
	},
	[D1980_IRQ_EGPI4] = {
		.reg = D1980_INT_OFFSET_4,
		.mask = D1980_IRQMASKD_MGPI4,
	},
	[D1980_IRQ_EGPI5] = {
		.reg = D1980_INT_OFFSET_4,
		.mask = D1980_IRQMASKD_MGPI5,		
	}, 
};



static void d1980_irq_call_handler(struct d1980 *d1980, int irq)
{
	mutex_lock(&d1980->irq_lock);

	if (d1980->irq[irq].handler) {
		d1980->irq[irq].handler(irq, d1980->irq[irq].data);

	} else {
		d1980_mask_irq(d1980, irq);
	}
	mutex_unlock(&d1980->irq_lock);
}

/*
 * This is a threaded IRQ handler so can access I2C/SPI.  Since all
 * interrupts are clear on read the IRQ line will be reasserted and
 * the physical IRQ will be handled again if another interrupt is
 * asserted while we run - in the normal course of events this is a
 * rare occurrence so we save I2C/SPI reads.
 */
void d1980_irq_worker(struct work_struct *work)
{
	struct d1980 *d1980 = container_of(work, struct d1980, irq_work); 	
	int reg_data;
	u8 sub_reg[D1980_NUM_IRQ_EVT_REGS];
	int read_done[D1980_NUM_IRQ_EVT_REGS];
	struct d1980_irq_data *data;
	int i;

	memset(&read_done, 0, sizeof(read_done));

	for (i = 0; i < ARRAY_SIZE(d1980_irqs); i++) {
		data = &d1980_irqs[i];

		if (!read_done[data->reg]) {
			sub_reg[data->reg] =
				d1980_reg_read(d1980, D1980_EVENTA_REG + data->reg);
			sub_reg[data->reg] &=
				~d1980_reg_read(d1980, D1980_IRQMASKA_REG + data->reg);
			read_done[data->reg] = 1;
		}

		if (sub_reg[data->reg] & data->mask)
			d1980_irq_call_handler(d1980, i);
	}

	/* Now clear EVENT registers */
	reg_data = 0xFFFFFFFF;
	d1980_block_write(d1980, D1980_EVENTA_REG, 4, (u8 *)&reg_data);	
	reg_data = 0;
	d1980_block_write(d1980, D1980_EVENTA_REG, 4, (u8 *)&reg_data);
	enable_irq(d1980->chip_irq);
	/* DLG Test Print */
	//dev_info(d1980->dev, "IRQ Generated [d1980_irq_worker EXIT]\n");
}


#ifdef D1980_INT_USE_THREAD
/*
 * This is a threaded IRQ handler so can access I2C/SPI.  Since all
 * interrupts are clear on read the IRQ line will be reasserted and
 * the physical IRQ will be handled again if another interrupt is
 * asserted while we run - in the normal course of events this is a
 * rare occurrence so we save I2C/SPI reads.
 */
void d1980_irq_thread(void *d1980_irq_tcb)
{
	struct d1980 *d1980 = (struct d1980 *)d1980_irq_tcb;
	int reg_data;
	u8 sub_reg[D1980_NUM_IRQ_EVT_REGS];
	int read_done[D1980_NUM_IRQ_EVT_REGS];
	struct d1980_irq_data *data;
	int i;


    //set_freezable();
    printk(KERN_ERR "d1980_irq_thread start  !!!!!! \n");
    while(!kthread_should_stop())
    {
        set_current_state(TASK_INTERRUPTIBLE);
    	memset(&read_done, 0, sizeof(read_done));

    	for (i = 0; i < ARRAY_SIZE(d1980_irqs); i++) {
    		data = &d1980_irqs[i];

    		if (!read_done[data->reg]) {
    			sub_reg[data->reg] =
    				d1980_reg_read(d1980, D1980_EVENTA_REG + data->reg);
    			sub_reg[data->reg] &=
    				~d1980_reg_read(d1980, D1980_IRQMASKA_REG + data->reg);
    			read_done[data->reg] = 1;
    		}

    		if (sub_reg[data->reg] & data->mask)
    			d1980_irq_call_handler(d1980, i);
    	}

    	/* Now clear EVENT registers */
    	reg_data = 0xFFFFFFFF;
    	d1980_block_write(d1980, D1980_EVENTA_REG, 4, (u8 *)&reg_data);	
    	reg_data = 0;
    	d1980_block_write(d1980, D1980_EVENTA_REG, 4, (u8 *)&reg_data);
    	enable_irq(d1980->chip_irq);
	}
	/* DLG Test Print */
	//dev_info(d1980->dev, "IRQ Generated [d1980_irq_worker EXIT]\n");
}
#endif /* D1980_INT_USE_THREAD */

#if 0
/* Kernel version 2.6.28 start */
static irqreturn_t d1980_irq(int irq, void *data)
{
	struct d1980 *d1980 = data;
        
	schedule_work(&d1980->irq_work);
	disable_irq_nosync(irq);
	/* DLG Test Print */
	return IRQ_HANDLED;
}
/* Kernel version 2.6.28 end */
#else
static irqreturn_t d1980_irq(int irq, void *data1)
{
	struct d1980 *d1980 = (struct d1980 *)data1; 	
	int reg_data;
	u8 sub_reg[D1980_NUM_IRQ_EVT_REGS];
	int read_done[D1980_NUM_IRQ_EVT_REGS];
	struct d1980_irq_data *data;
	int i;

	memset(&read_done, 0, sizeof(read_done));

	for (i = 0; i < ARRAY_SIZE(d1980_irqs); i++) {
		data = &d1980_irqs[i];

		if (!read_done[data->reg]) {
			sub_reg[data->reg] =
				d1980_reg_read(d1980, D1980_EVENTA_REG + data->reg);
			sub_reg[data->reg] &=
				~d1980_reg_read(d1980, D1980_IRQMASKA_REG + data->reg);
			read_done[data->reg] = 1;
		}

		if (sub_reg[data->reg] & data->mask)
			d1980_irq_call_handler(d1980, i);
	}

	/* Now clear EVENT registers */
	reg_data = 0xFFFFFFFF;
	d1980_block_write(d1980, D1980_EVENTA_REG, 4, (u8 *)&reg_data);	
	reg_data = 0;
	d1980_block_write(d1980, D1980_EVENTA_REG, 4, (u8 *)&reg_data);
	//enable_irq(d1980->chip_irq);
	/* DLG Test Print */
	//dev_info(d1980->dev, "IRQ Generated [d1980_irq_worker EXIT]\n");
	return IRQ_HANDLED;
}
#endif

int d1980_register_irq(struct d1980 *d1980, int irq,
			irq_handler_t handler, unsigned long flags,
			const char *name, void *data)
{
	if (irq < 0 || irq >= D1980_NUM_IRQ || !handler)
		return -EINVAL;

	if (d1980->irq[irq].handler)
		return -EBUSY;        
	mutex_lock(&d1980->irq_lock);	
	d1980->irq[irq].handler = handler;
	d1980->irq[irq].data = data;
	mutex_unlock(&d1980->irq_lock);
	/* DLG Test Print */
    //dev_info(d1980->dev, "\nIRQ After MUTEX UNLOCK\n");

	d1980_unmask_irq(d1980, irq);
	return 0;
}
EXPORT_SYMBOL_GPL(d1980_register_irq);

int d1980_free_irq(struct d1980 *d1980, int irq)
{
	if (irq < 0 || irq >= D1980_NUM_IRQ)
		return -EINVAL;

	d1980_mask_irq(d1980, irq);

	mutex_lock(&d1980->irq_lock);
	d1980->irq[irq].handler = NULL;
	mutex_unlock(&d1980->irq_lock);
	return 0;
}
EXPORT_SYMBOL_GPL(d1980_free_irq);

int d1980_mask_irq(struct d1980 *d1980, int irq)
{
	return d1980_set_bits(d1980, D1980_IRQMASKA_REG + d1980_irqs[irq].reg,
			       d1980_irqs[irq].mask);
}
EXPORT_SYMBOL_GPL(d1980_mask_irq);

int d1980_unmask_irq(struct d1980 *d1980, int irq)
{
      return d1980_clear_bits(d1980, D1980_IRQMASKA_REG + d1980_irqs[irq].reg,
				 d1980_irqs[irq].mask);
}
EXPORT_SYMBOL_GPL(d1980_unmask_irq);

int d1980_irq_init(struct d1980 *d1980, int irq,
		    struct d1980_platform_data *pdata)
{	
	int ret;
	int reg_data, maskbit = 0xFFFFFFFF;

	if (!irq) {
	    dev_err(d1980->dev, "No IRQ configured \n");
	    return -EINVAL;
	}
	reg_data = 0xFFFFFFFF;
	d1980_block_write(d1980, D1980_EVENTA_REG, 4, (u8 *)&reg_data);	
	reg_data = 0;
	d1980_block_write(d1980, D1980_EVENTA_REG, 4, (u8 *)&reg_data);

	dev_err(d1980->dev, "Test print IRQ \n");

	/* Clear Mask A register */
	maskbit = 0xFFFFFFFF;
	d1980_block_write(d1980, D1980_IRQMASKA_REG, 4, (u8 *)&maskbit);

	mutex_init(&d1980->irq_lock);

#ifdef D1980_INT_USE_THREAD
    d1980->irq_task = kthread_run(d1980_irq_thread, d1980, "d1980_irq_task");
#else
//	INIT_WORK(&d1980->irq_work, d1980_irq_worker);
#endif
	if (irq) {
		/* DLG Test Print */
 		dev_err(d1980->dev, "\n\n############## IRQ configured [%d] ###############\n\n", irq);
		//ret = request_irq(irq, d1980_irq, IRQF_TRIGGER_FALLING, //IRQ_TYPE_LEVEL_LOW,// 
	    //			  "d1980", d1980);
	    ret = request_threaded_irq(irq, NULL, d1980_irq, IRQF_TRIGGER_FALLING,
				   "d1980", d1980);
		if (ret != 0) {
			dev_err(d1980->dev, "Failed to request IRQ: %d\n", irq);
			return ret;
		}
	} else {
		dev_err(d1980->dev, "No IRQ configured\n");
		return ret;
	}

	d1980->chip_irq = irq;
	return ret;
}

int d1980_irq_exit(struct d1980 *d1980)
{
	free_irq(d1980->chip_irq, d1980);
	return 0;
}
