/*
 * 88pm860x VBus driver for Marvell USB
 *
 * Copyright (C) 2010 Marvell International Ltd.
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
#include <linux/mfd/88pm860x.h>
#include <linux/spinlock.h>
#include <linux/workqueue.h>
#include <plat/vbus.h>
#include <linux/slab.h>
#include <linux/delay.h>

/* Register Offset */
#define MEAS_ENABLE1		0x50

#define GPADC_MISC1		0x57

#define GPADC0_LOW_TH		0x5F
#define GPADC1_LOW_TH		0x60
#define GPADC2_LOW_TH		0x61
#define GPADC3_LOW_TH		0x62

#define GPADC0_UPP_TH		0x67
#define GPADC1_UPP_TH		0x68
#define GPADC2_UPP_TH		0x69
#define GPADC3_UPP_TH		0x70

#define GPADC0_MEAS1		0x75
#define GPADC0_MEAS2		0x76
#define GPADC1_MEAS1		0x77
#define GPADC1_MEAS2		0x78
#define GPADC2_MEAS1		0x79
#define GPADC2_MEAS2		0x7A
#define GPADC3_MEAS1		0x7B
#define GPADC3_MEAS2		0x7C

#define MEAS_GP0_EN		(1 << 4)
#define MEAS_GP1_EN		(1 << 5)
#define MEAS_GP2_EN		(1 << 6)
#define MEAS_GP3_EN		(1 << 7)

#define GPFSM_EN		(1 << 0)

#define STATUS2_VBUS		(1 << 4)

#define MISC1_GPIO1_DIR		(1 << 3)
#define MISC1_GPIO1_VAL		(1 << 4)
#define MISC1_GPIO2_DIR		(1 << 5)
#define MISC1_GPIO2_VAL		(1 << 6)

struct pm860x_vbus_info {
	struct pm860x_chip	*chip;
	struct i2c_client	*i2c;
	struct resource		*res;
	struct work_struct      wq;
	unsigned int		base;
	int			irq_status;
	int			irq_id;
	int			idpin;
	int			supply;
	int			work_data;
	int			work_mask;
};

static DEFINE_SPINLOCK(lock);
static struct pm860x_vbus_info *vbus_data = NULL;
static int otg_int_id = 0;
static int vbus_status = 0;
static pm860xVbusCbFunc vbusCbFun = NULL;

#ifdef CONFIG_CHARGER_88PM860X
extern irqreturn_t pm860x_charger_handler(int irq, void *data);
#endif
static irqreturn_t pm860x_vbus_handler(int irq, void *data)
{
	unsigned long flags;
	int ret;

	mdelay(20);
	ret = pm860x_reg_read(vbus_data->i2c, PM8607_STATUS_2);
	if (ret < 0)
		goto out;
	if (ret & STATUS2_VBUS)
		ret = VBUS_HIGH;
	else
		ret = VBUS_LOW;
	spin_lock_irqsave(&lock, flags);
	vbus_status = ret;
	spin_unlock_irqrestore(&lock, flags);

	/* notify usb driver if vbus event */
	pxa_vbus_handler(ret);
#ifdef CONFIG_CHARGER_88PM860X
	/* notify power driver charge in/out event*/
	if(pxa_query_usbdev() == USB_A_DEVICE){
		return IRQ_HANDLED;
	}
	pm860x_charger_handler(irq, data);
#endif
out:
	return IRQ_HANDLED;
}

static int pm860x_meas_adc(struct pm860x_vbus_info *info, int idpin)
{
	int meas1 = 0, meas2 = 0, low = 0, high = 0, data, ret;
	unsigned long flags;

	switch (idpin) {
	case PM860X_IDPIN_USE_GPADC0:
		meas1 = GPADC0_MEAS1;
		meas2 = GPADC0_MEAS2;
		low = GPADC0_LOW_TH;
		high = GPADC0_UPP_TH;
		break;
	case PM860X_IDPIN_USE_GPADC1:
		meas1 = GPADC1_MEAS1;
		meas2 = GPADC1_MEAS2;
		low = GPADC1_LOW_TH;
		high = GPADC1_UPP_TH;
		break;
	case PM860X_IDPIN_USE_GPADC2:
		meas1 = GPADC2_MEAS1;
		meas2 = GPADC2_MEAS2;
		low = GPADC2_LOW_TH;
		high = GPADC2_UPP_TH;
		break;
	case PM860X_IDPIN_USE_GPADC3:
		meas1 = GPADC3_MEAS1;
		meas2 = GPADC3_MEAS2;
		low = GPADC3_LOW_TH;
		high = GPADC3_UPP_TH;
		break;
	}
	ret = pm860x_reg_read(info->i2c, meas1);
	data = ret << 4;
	ret = pm860x_reg_read(info->i2c, meas2);
	data |= ret & 0x0F;

	if (data > 0x10) {
		ret = 1;
		pm860x_reg_write(info->i2c, low, 0x10);
		pm860x_reg_write(info->i2c, high, 0xff);
	} else {
		ret = 0;
		pm860x_reg_write(info->i2c, low, 0);
		pm860x_reg_write(info->i2c, high, 0x10);
	}
//#if defined(CONFIG_MACH_GFORCE) || defined(CONFIG_MACH_ALKON) || defined(CONFIG_MACH_JETTA)
//dh0318.lee 110412 TEMP_FIX
ret = 1;
//#endif

	spin_lock_irqsave(&lock, flags);
	otg_int_id = ret;
	spin_unlock_irqrestore(&lock, flags);

	return ret;
}

extern irqreturn_t mv_usb_dev_irq (int irq, void *data);
static irqreturn_t pm860x_idpin_handler(int irq, void *data)
{
	struct pm860x_vbus_info *info = (struct pm860x_vbus_info *)data;
	int ret;

	ret = pm860x_meas_adc(info, info->idpin);

	mv_usb_dev_irq(info->irq_id, NULL);

	return IRQ_HANDLED;
}

static void pm860x_set_work(struct work_struct *work)
{
	unsigned long flags;
	int ret;
	pm860x_set_bits(vbus_data->i2c, PM8607_A1_MISC1, vbus_data->work_mask,
			vbus_data->work_data);

	mdelay(20);
	ret = pm860x_reg_read(vbus_data->i2c, PM8607_STATUS_2);
	if (ret < 0)
		return;
	if (ret & STATUS2_VBUS)
		ret = VBUS_HIGH;
	else
		ret = VBUS_LOW;
	spin_lock_irqsave(&lock, flags);
	vbus_status = ret;
	spin_unlock_irqrestore(&lock, flags);
}

/* Supply VBUS power or not */
int pm860x_set_vbus(int enable, int srp)
{
	struct pm860x_chip *chip = vbus_data->chip;
	unsigned int data = 0, mask;

	if (enable) {
		/* output */
		data = MISC1_GPIO1_DIR | MISC1_GPIO2_DIR;
		data |= MISC1_GPIO1_VAL | MISC1_GPIO2_VAL;
	}
	switch (vbus_data->supply) {
	case PM860X_GPIO1_SUPPLY_VBUS:
		mask = MISC1_GPIO1_DIR | MISC1_GPIO1_VAL;
		break;
	case PM860X_GPIO2_SUPPLY_VBUS:
		mask = MISC1_GPIO2_DIR | MISC1_GPIO2_VAL;
		break;
	default:
		dev_err(chip->dev, "wrong vbus supplier is assigned\n");
		return -EINVAL;
	}
	data &= mask;
	vbus_data->work_data = data;
	vbus_data->work_mask = mask;
	schedule_work(&vbus_data->wq);

	return 0;
}

static int pm860x_query_vbus(void)
{
	unsigned long flags;
	int ret;

	spin_lock_irqsave(&lock, flags);
	ret = vbus_status;
	spin_unlock_irqrestore(&lock, flags);
	return ret;
}

static int pm860x_query_usbid(void)
{
	unsigned long flags;
	int event = 0;
	spin_lock_irqsave(&lock, flags);
	if (otg_int_id > 0)
		event |= OTG_INT_IDR;
	else if (otg_int_id == 0)
		event |= OTG_INT_IDF;
	/* otg_int_id = -1; */
	spin_unlock_irqrestore(&lock, flags);
	return event;
}
#ifdef CONFIG_ALTERNATE_CHARGER
static irqreturn_t pm860x_default_vbus_handler(int irq, void *data)
{
	pr_info( \
	"[%s]pm860x_default_vbus_handler\n",__FILE__);
	return IRQ_HANDLED;
}

int pm860x_registerVbusCb(pm860xVbusCbFunc callBack)
{
	if ( callBack ) {
		vbusCbFun = callBack;
		pr_info( \
		"88pm860x_vbus.c:pm860x_registerVbusCb[%x]\n",(int)vbusCbFun);
		return 0;
	} else { 
		pr_info( \
		"88pm860x_vbus.c:pm860x_registerVbusCb fail\n");
		return 1;
	}
}
#endif 
static int __devinit pm860x_vbus_probe(struct platform_device *pdev)
{
	struct pm860x_chip *chip = dev_get_drvdata(pdev->dev.parent);
	struct pm860x_platform_data *pm860x_pdata =		\
				pdev->dev.parent->platform_data;
	struct pm860x_vbus_pdata *pdata = NULL;
	struct pm860x_vbus_info *vbus;
	struct pxa_vbus_info info;
	unsigned int data, mask = 0;
	int ret = 0;

	if (!pm860x_pdata) {
		dev_err(&pdev->dev, "platform data is missing\n");
		return -EINVAL;
	}

	pdata = pm860x_pdata->vbus;
	if (!pdata) {
		dev_err(&pdev->dev, "vbus data is missing\n");
		return -EINVAL;
	}

	vbus = kzalloc(sizeof(struct pm860x_vbus_info), GFP_KERNEL);
	if (!vbus) {
		ret = -ENOMEM;
		goto out_mem;
	}
	dev_set_drvdata(&pdev->dev, vbus);
	vbus_data = vbus;

	vbus->res = kzalloc(sizeof(struct resource), GFP_KERNEL);
	if (!vbus->res) {
		ret = -ENOMEM;
		goto out_mem2;
	}
	vbus->res->start = pdata->reg_base;
	vbus->res->end = pdata->reg_end;

	INIT_WORK(&vbus_data->wq, pm860x_set_work);

	memset(&info, 0, sizeof(struct pxa_vbus_info));
	info.set_vbus = pm860x_set_vbus;
	info.query_vbus = pm860x_query_vbus;
	/* usbid check is from usb register */
#ifdef CONFIG_PXA95x
	info.query_usbid = pm860x_query_usbid;
#endif
	info.dev = chip->dev;
	info.res = vbus->res;
	pxa_vbus_init(&info);

	data = VBUS_A_VALID | VBUS_A_SESSION_VALID | VBUS_B_SESSION_VALID
		| VBUS_B_SESSION_END | VBUS_ID;
	pxa_unmask_vbus(data);

	vbus->chip = chip;

	vbus->irq_status = platform_get_irq(pdev, 0);
	if (vbus->irq_status < 0) {
		dev_err(&pdev->dev, "failed to get vbus irq\n");
		ret = -ENXIO;
		goto out_res;
	}

	vbus->idpin = pdata->idpin;

	vbus->i2c = (chip->id == CHIP_PM8607) ? chip->client : chip->companion;
	
#ifdef CONFIG_ALTERNATE_CHARGER
     pm860x_registerVbusCb(pm860x_default_vbus_handler);
	 /* make sure the call back will call 
	 pxa_vbus_handler(...) to notify all usb users*/
#else
	vbusCbFun = pm860x_vbus_handler;
#endif
	if (vbusCbFun) 
		ret = request_threaded_irq(vbus->irq_status, NULL, vbusCbFun,
					IRQF_ONESHOT, "vbus", vbus);

	if (ret < 0) {
		dev_err(&pdev->dev, "failed to request IRQ on vbus\n");
		dev_err(&pdev->dev, "irq:%d, ret:%d\n", vbus->irq_status, ret);
		ret = -EINVAL;
		goto out_res;
	}

	if (pdata->supply) {
		vbus->supply = pdata->supply;
		/* Disable VBUS output via PMIC */
		switch (vbus_data->supply) {
		case PM860X_GPIO1_SUPPLY_VBUS:
			mask = MISC1_GPIO1_DIR | MISC1_GPIO1_VAL;
			break;
		case PM860X_GPIO2_SUPPLY_VBUS:
			mask = MISC1_GPIO2_DIR | MISC1_GPIO2_VAL;
			break;
		}
		pm860x_set_bits(vbus->i2c, PM8607_A1_MISC1, mask, 0);
	}
	/* query vbus status */
	ret = pm860x_reg_read(vbus_data->i2c, PM8607_STATUS_2);
	if (ret & STATUS2_VBUS)
		vbus_status = VBUS_HIGH;
	else
		vbus_status = VBUS_LOW;
	dev_info(&pdev->dev, "VBUS is %s\n", vbus_status ? "high" : "low");

	switch (vbus->idpin) {
	case PM860X_IDPIN_USE_GPADC0:
		data = MEAS_GP0_EN;
		vbus->irq_id = PM8607_IRQ_GPADC0;
		break;
	case PM860X_IDPIN_USE_GPADC1:
		data = MEAS_GP1_EN;
		vbus->irq_id = PM8607_IRQ_GPADC1;
		break;
	case PM860X_IDPIN_USE_GPADC2:
		data = MEAS_GP2_EN;
		vbus->irq_id = PM8607_IRQ_GPADC2;
		break;
	case PM860X_IDPIN_USE_GPADC3:
		data = MEAS_GP3_EN;
		vbus->irq_id = PM8607_IRQ_GPADC3;
		break;
	case PM860X_IDPIN_NO_USE:
		goto handled;
	}
	pm860x_set_bits(vbus->i2c, MEAS_ENABLE1, data, data);
	pm860x_set_bits(vbus->i2c, GPADC_MISC1, GPFSM_EN, GPFSM_EN);
	pm860x_meas_adc(vbus, vbus->idpin);


	vbus->irq_id += chip->irq_base;
	ret = request_threaded_irq(vbus->irq_id, NULL,
				pm860x_idpin_handler, IRQF_ONESHOT,
				"idpin", vbus);
	if (ret < 0) {
		dev_err(&pdev->dev, "failed to request IRQ on id\n");
		dev_err(&pdev->dev, "irq:%d, ret:%d\n", vbus->idpin, ret);
		ret = -EINVAL;
		goto out_status;
	}
handled:
	platform_set_drvdata(pdev, vbus);
	device_init_wakeup(&pdev->dev, 1);

	return 0;

out_status:
	free_irq(vbus->irq_status, vbus);
out_res:
	kfree(vbus->res);
out_mem2:
	kfree(vbus);
	vbus_data = NULL;
out_mem:
	return ret;
}

static int __devexit pm860x_vbus_remove(struct platform_device *pdev)
{
	struct pm860x_vbus_info *vbus = platform_get_drvdata(pdev);
	unsigned int data;

	if (vbus) {
		data = VBUS_A_VALID | VBUS_A_SESSION_VALID
			| VBUS_B_SESSION_VALID
			| VBUS_B_SESSION_END;
		pxa_mask_vbus(data);

		free_irq(vbus->irq_id, vbus);
		free_irq(vbus->irq_status, vbus);
		platform_set_drvdata(pdev, NULL);
		kfree(vbus->res);
		kfree(vbus);
		vbus_data = NULL;
	}
	return 0;
}

#ifdef CONFIG_PM
static int pm860x_vbus_suspend(struct device *dev)
{
	struct pm860x_vbus_info *vbus = dev_get_drvdata(dev);

	if (device_may_wakeup(dev)) {
		enable_irq_wake(vbus->chip->core_irq);
		enable_irq_wake(vbus->irq_status);
		if (vbus->idpin != PM860X_IDPIN_NO_USE)
		enable_irq_wake(vbus->irq_id);
	}
	return 0;
}

static int pm860x_vbus_resume(struct device *dev)
{
	struct pm860x_vbus_info *vbus = dev_get_drvdata(dev);

	if (device_may_wakeup(dev)) {
		disable_irq_wake(vbus->chip->core_irq);
		disable_irq_wake(vbus->irq_status);
		if (vbus->idpin != PM860X_IDPIN_NO_USE)
		disable_irq_wake(vbus->irq_id);
	}
	return 0;
}

static struct dev_pm_ops pm860x_vbus_pm_ops = {
	.suspend	= pm860x_vbus_suspend,
	.resume		= pm860x_vbus_resume,
};
#endif

static struct platform_driver pm860x_vbus_driver = {
	.driver		= {
		.name	= "88pm860x-vbus",
		.owner	= THIS_MODULE,
#ifdef CONFIG_PM
		.pm	= &pm860x_vbus_pm_ops,
#endif
	},
	.probe		= pm860x_vbus_probe,
	.remove		= __devexit_p(pm860x_vbus_remove),
};

static int __init pm860x_vbus_init(void)
{
	return platform_driver_register(&pm860x_vbus_driver);
}
module_init(pm860x_vbus_init);

static void __exit pm860x_vbus_exit(void)
{
	platform_driver_unregister(&pm860x_vbus_driver);
}
module_exit(pm860x_vbus_exit);

MODULE_DESCRIPTION("VBUS driver for Marvell Semiconductor 88PM860x");
MODULE_AUTHOR("Haojian Zhuang <haojian.zhuang@marvell.com>");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:88pm860x-vbus");
