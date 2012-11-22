/*
 * 88pm860x_onkey.c - Marvell 88PM860x ONKEY driver
 *
 * Copyright (C) 2009-2010 Marvell International Ltd.
 *      Haojian Zhuang <haojian.zhuang@marvell.com>
 *
 * This file is subject to the terms and conditions of the GNU General
 * Public License. See the file "COPYING" in the main directory of this
 * archive for more details.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/mfd/88pm860x.h>
#include <linux/slab.h>

#define PM8607_WAKEUP		0x0b

#define LONG_ONKEY_EN		(1 << 1)
#define ONKEY_STATUS		(1 << 0)

#define SW_PDOWN		(1 << 7)

#define ONKEY_CHECK_TIME    500

struct pm860x_onkey_info {
	struct input_dev	*idev;
	struct pm860x_chip	*chip;
	struct i2c_client	*i2c;
	struct device		*dev;
	int			irq;
	
	//YJ : 2011,11,02 Onkey press debounce time
	struct timer_list onkey_timer;
};

static struct i2c_client *i2c = NULL;


/* Samsung YJ 2011 06 29 : For DFMS functionality */
static unsigned char gOnKeyState = 0;	//0 : Released, 1 : Pressed

//YJ : 2011,11,02 Onkey press debounce time
static unsigned char gOnkeyNotPassed = 0;

unsigned char get_Onkey_State(void)
{
	return gOnKeyState;
}
EXPORT_SYMBOL(get_Onkey_State);

/* 88PM860x poweroff function */
void pm860x_system_poweroff(void)
{
	u8 tmp;

	printk("turning off power....\n");

	tmp = pm860x_reg_read(i2c, PM8607_RESET_OUT);
	pm860x_reg_write(i2c, PM8607_RESET_OUT, tmp | SW_PDOWN);

	return;
}

static void onkey_timer_expired(unsigned long data)
{

}

/* 88PM860x gives us an interrupt when ONKEY is held */
static irqreturn_t pm860x_onkey_handler(int irq, void *data)
{
	struct pm860x_onkey_info *info = data;
	int ret;

	ret = pm860x_reg_read(info->i2c, PM8607_STATUS_2);
	ret &= ONKEY_STATUS;
	/* +Samsung YJ 2011 06 29 : For DFMS functionality */
	gOnKeyState = ret;		
	/* -Samsung YJ 2011 06 29 : For DFMS functionality */

	if(gOnKeyState == 1) {
		//YJ : 2011,11,02 Onkey press debounce time
		if(timer_pending(&info->onkey_timer)) {
			gOnkeyNotPassed = 1;
			return IRQ_HANDLED;
		}
		printk("onkey pressed..\n");
	}
	else
	{
	
	//YJ : 2011,11,02 Onkey press debounce time
	mod_timer(&info->onkey_timer, jiffies + msecs_to_jiffies(ONKEY_CHECK_TIME));
	
	if(gOnkeyNotPassed) {
			gOnkeyNotPassed = 0;
			return IRQ_HANDLED;
		}
		printk("onkey released..\n");

	}

	input_report_key(info->idev, KEY_POWER, ret);
	input_sync(info->idev);

	return IRQ_HANDLED;
}

#ifdef CONFIG_PM
static int pm860x_onkey_suspend(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct pm860x_chip *chip = dev_get_drvdata(pdev->dev.parent);
	int irq;

	irq = platform_get_irq(pdev, 0) + chip->irq_base;
	if (device_may_wakeup(dev)) {
		enable_irq_wake(chip->core_irq);
		enable_irq_wake(irq);
	}
	return 0;
}

static int pm860x_onkey_resume(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct pm860x_chip *chip = dev_get_drvdata(pdev->dev.parent);
	int irq;

	irq = platform_get_irq(pdev, 0) + chip->irq_base;
	if (device_may_wakeup(dev)) {
		disable_irq_wake(chip->core_irq);
		disable_irq_wake(irq);
	}
	return 0;
}

static struct dev_pm_ops pm860x_onkey_pm_ops = {
	.suspend	= pm860x_onkey_suspend,
	.resume		= pm860x_onkey_resume,
};
#endif

static int __devinit pm860x_onkey_probe(struct platform_device *pdev)
{
	struct pm860x_chip *chip = dev_get_drvdata(pdev->dev.parent);
	struct pm860x_onkey_info *info;
	int irq, ret;

	irq = platform_get_irq(pdev, 0);
	if (irq < 0) {
		dev_err(&pdev->dev, "No IRQ resource!\n");
		return -EINVAL;
	}

	info = kzalloc(sizeof(struct pm860x_onkey_info), GFP_KERNEL);
	if (!info)
		return -ENOMEM;
	info->chip = chip;
	info->i2c = (chip->id == CHIP_PM8607) ? chip->client : chip->companion;
	i2c = info->i2c;
	info->dev = &pdev->dev;
	info->irq = irq + chip->irq_base;

	info->idev = input_allocate_device();
	if (!info->idev) {
		dev_err(chip->dev, "Failed to allocate input dev\n");
		ret = -ENOMEM;
		goto out;
	}

	info->idev->name = "88pm860x_on";
	info->idev->phys = "88pm860x_on/input0";
	info->idev->id.bustype = BUS_I2C;
	info->idev->dev.parent = &pdev->dev;
	info->idev->evbit[0] = BIT_MASK(EV_KEY);
	info->idev->keybit[BIT_WORD(KEY_POWER)] = BIT_MASK(KEY_POWER);

	ret = input_register_device(info->idev);
	if (ret) {
		dev_err(chip->dev, "Can't register input device: %d\n", ret);
		goto out_reg;
	}

	ret = request_threaded_irq(info->irq, NULL, pm860x_onkey_handler,
				   IRQF_ONESHOT, "onkey", info);
	if (ret < 0) {
		dev_err(chip->dev, "Failed to request IRQ: #%d: %d\n",
			info->irq, ret);
		goto out_irq;
	}

	platform_set_drvdata(pdev, info);

	/* Enable 8-second long onkey detection */
	pm860x_set_bits(info->i2c, PM8607_WAKEUP, 3, LONG_ONKEY_EN);

	device_init_wakeup(&pdev->dev, 1);

	//YJ : 2011,11,02 Onkey press debounce time
	init_timer(&info->onkey_timer);
	info->onkey_timer.function = onkey_timer_expired;
	info->onkey_timer.data = (unsigned long)chip;

	return 0;

out_irq:
	input_unregister_device(info->idev);
	kfree(info);
	return ret;

out_reg:
	input_free_device(info->idev);
out:
	kfree(info);
	return ret;
}

static int __devexit pm860x_onkey_remove(struct platform_device *pdev)
{
	struct pm860x_onkey_info *info = platform_get_drvdata(pdev);

	free_irq(info->irq, info);
	input_unregister_device(info->idev);
	kfree(info);
	return 0;
}

static struct platform_driver pm860x_onkey_driver = {
	.driver		= {
		.name	= "88pm860x-onkey",
		.owner	= THIS_MODULE,
#ifdef CONFIG_PM
		.pm	= &pm860x_onkey_pm_ops,
#endif
	},
	.probe		= pm860x_onkey_probe,
	.remove		= __devexit_p(pm860x_onkey_remove),
};

static int __init pm860x_onkey_init(void)
{
	return platform_driver_register(&pm860x_onkey_driver);
}
module_init(pm860x_onkey_init);

static void __exit pm860x_onkey_exit(void)
{
	platform_driver_unregister(&pm860x_onkey_driver);
}
module_exit(pm860x_onkey_exit);

MODULE_DESCRIPTION("Marvell 88PM860x ONKEY driver");
MODULE_AUTHOR("Haojian Zhuang <haojian.zhuang@marvell.com>");
MODULE_LICENSE("GPL");
