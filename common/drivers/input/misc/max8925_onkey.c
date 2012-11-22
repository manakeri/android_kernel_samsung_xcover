/**
 * max8925_onkey.c - MAX8925 ONKEY driver
 *
 * Copyright (C) 2009 Marvell International Ltd.
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
#include <linux/mfd/max8925.h>
#include <linux/slab.h>

#define SW_INPUT		(1 << 7)	/* 0/1 -- up/down */
#define HARDRESET_EN		(1 << 7)

#define PWREN_EN		(1 << 7)
#define PWR_OFF			(1 << 6)
#define SFT_RESET		(1 << 5)
#define RSTIN_DELAY		(2 << 3)
#define SFT_DESERTION		(1 << 2)

struct max8925_onkey_info {
	struct input_dev	*idev;
	struct max8925_chip	*chip;
	struct i2c_client	*i2c;
	struct device		*dev;
	int			irq[2];
};

/* reserve this static structure for restart interface */
static struct i2c_client *i2c = NULL;

void max8925_system_restart(char mode, const char *cmd)
{
	if (i2c)
		max8925_reg_write(i2c, MAX8925_RESET_CNFG, SFT_RESET
				| RSTIN_DELAY | SFT_DESERTION);
}
EXPORT_SYMBOL(max8925_system_restart);

void max8925_system_poweroff(void)
{
	if (i2c) {
		max8925_set_bits(i2c, MAX8925_WLED_MODE_CNTL, 1, 0);
		max8925_set_bits(i2c, MAX8925_RESET_CNFG, PWR_OFF, PWR_OFF);
	}
}
EXPORT_SYMBOL(max8925_system_poweroff);

/* MAX8925 gives us an interrupt when ONKEY is held for 3 seconds. */
static irqreturn_t max8925_onkey_handler(int irq, void *data)
{
	struct max8925_onkey_info *info = data;
	int ret, event;

	ret = max8925_reg_read(info->i2c, MAX8925_ON_OFF_STATUS);
	if (ret & SW_INPUT)
		event = 1;
	else
		event = 0;
	input_report_key(info->idev, KEY_POWER, event);
	input_sync(info->idev);

	return IRQ_HANDLED;
}

#ifdef CONFIG_PM
static int max8925_onkey_suspend(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct max8925_chip *chip = dev_get_drvdata(pdev->dev.parent);
	int irq[2];

	irq[0] = platform_get_irq(pdev, 0) + chip->irq_base;
	irq[1] = platform_get_irq(pdev, 1) + chip->irq_base;
	if (device_may_wakeup(dev)) {
		enable_irq_wake(chip->core_irq);
		enable_irq_wake(irq[0]);
		enable_irq_wake(irq[1]);
	}
	return 0;
}

static int max8925_onkey_resume(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct max8925_chip *chip = dev_get_drvdata(pdev->dev.parent);
	int irq[2];

	irq[0] = platform_get_irq(pdev, 0) + chip->irq_base;
	irq[1] = platform_get_irq(pdev, 1) + chip->irq_base;
	if (device_may_wakeup(dev)) {
		disable_irq_wake(chip->core_irq);
		disable_irq_wake(irq[0]);
		disable_irq_wake(irq[1]);
	}
	return 0;
}

static struct dev_pm_ops max8925_onkey_pm_ops = {
	.suspend	= max8925_onkey_suspend,
	.resume		= max8925_onkey_resume,
};
#endif

static int __devinit max8925_onkey_probe(struct platform_device *pdev)
{
	struct max8925_chip *chip = dev_get_drvdata(pdev->dev.parent);
	struct max8925_onkey_info *info;
	int irq[2], ret;

	irq[0] = platform_get_irq(pdev, 0);
	if (irq[0] < 0) {
		dev_err(&pdev->dev, "No IRQ resource!\n");
		return -EINVAL;
	}
	irq[1] = platform_get_irq(pdev, 1);
	if (irq[1] < 0) {
		dev_err(&pdev->dev, "No IRQ resource!\n");
		return -EINVAL;
	}

	info = kzalloc(sizeof(struct max8925_onkey_info), GFP_KERNEL);
	if (!info)
		return -ENOMEM;

	info->chip = chip;
	info->i2c = chip->i2c;
	info->dev = &pdev->dev;
	irq[0] += chip->irq_base;
	irq[1] += chip->irq_base;

	ret = request_threaded_irq(irq[0], NULL, max8925_onkey_handler,
			IRQF_ONESHOT, "onkey-down", info);
	if (ret < 0) {
		dev_err(chip->dev, "Failed to request IRQ: #%d: %d\n",
				irq[0], ret);
		goto out;
	}
	ret = request_threaded_irq(irq[1], NULL, max8925_onkey_handler,
			IRQF_ONESHOT, "onkey-up", info);
	if (ret < 0) {
		dev_err(chip->dev, "Failed to request IRQ: #%d: %d\n",
				irq[1], ret);
		goto out_irq;
	}

	info->idev = input_allocate_device();
	if (!info->idev) {
		dev_err(chip->dev, "Failed to allocate input dev\n");
		ret = -ENOMEM;
		goto out_input;
	}

	info->idev->name = "max8925_on";
	info->idev->phys = "max8925_on/input0";
	info->idev->id.bustype = BUS_I2C;
	info->idev->dev.parent = &pdev->dev;
	info->irq[0] = irq[0];
	info->irq[1] = irq[1];
	info->idev->evbit[0] = BIT_MASK(EV_KEY);
	info->idev->keybit[BIT_WORD(KEY_POWER)] = BIT_MASK(KEY_POWER);

	ret = input_register_device(info->idev);
	if (ret) {
		dev_err(chip->dev, "Can't register input device: %d\n", ret);
		goto out_reg;
	}
	platform_set_drvdata(pdev, info);

	/* reserve this interface for reboot API */
	i2c = info->i2c;

	/*
	 * Enable hardreset, press onkey for 5 seconds, system will shutdown
	 * automatically
	 */
	max8925_set_bits(info->i2c, MAX8925_SYSENSEL,
			 HARDRESET_EN, HARDRESET_EN);

	device_init_wakeup(&pdev->dev, 1);
	return 0;

out_reg:
	input_free_device(info->idev);
out_input:
	free_irq(info->irq[1], info);
out_irq:
	free_irq(info->irq[0], info);
out:
	kfree(info);
	return ret;
}

static int __devexit max8925_onkey_remove(struct platform_device *pdev)
{
	struct max8925_onkey_info *info = platform_get_drvdata(pdev);

	if (info) {
		platform_set_drvdata(pdev, NULL);
		free_irq(info->irq[0], info);
		free_irq(info->irq[1], info);
		input_unregister_device(info->idev);
		kfree(info);
	}
	return 0;
}

static struct platform_driver max8925_onkey_driver = {
	.driver		= {
		.name	= "max8925-onkey",
		.owner	= THIS_MODULE,
#ifdef CONFIG_PM
		.pm	= &max8925_onkey_pm_ops,
#endif
	},
	.probe		= max8925_onkey_probe,
	.remove		= __devexit_p(max8925_onkey_remove),
};

static int __init max8925_onkey_init(void)
{
	return platform_driver_register(&max8925_onkey_driver);
}
module_init(max8925_onkey_init);

static void __exit max8925_onkey_exit(void)
{
	platform_driver_unregister(&max8925_onkey_driver);
}
module_exit(max8925_onkey_exit);

MODULE_DESCRIPTION("Maxim MAX8925 ONKEY driver");
MODULE_AUTHOR("Haojian Zhuang <haojian.zhuang@marvell.com>");
MODULE_LICENSE("GPL");
