/*
 * Virtual Regulators driver for PN544
 *
 * Copyright (C) 2010 Marvell International Ltd.
 *	Neil Zhang <zhangwm@marvell.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/err.h>
#include <linux/platform_device.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/machine.h>

#include <linux/regulator/vpmic.h>


struct vpmic_regulator_info {
	struct regulator_desc	desc;
	struct regulator_dev	*regulator;
};


static int vpmic_enable(struct regulator_dev *rdev)
{
	struct vpmic_regulator_info *info = rdev_get_drvdata(rdev);
	dev_dbg(&rdev->dev, "vpmic_enable: %s\n", info->desc.name);

	return 0;
}

static int vpmic_disable(struct regulator_dev *rdev)
{
	struct vpmic_regulator_info *info = rdev_get_drvdata(rdev);
	dev_dbg(&rdev->dev, "vpmic_disable: %s\n", info->desc.name);

	return 0;
}


static struct regulator_ops vpmic_regulator_ops = {
	.enable		= vpmic_enable,
	.disable	= vpmic_disable,
};


#define VPMIC_DVC(vreg)							\
{									\
	.desc	= {							\
		.name	= #vreg,					\
		.ops	= &vpmic_regulator_ops,				\
		.type	= REGULATOR_VOLTAGE,				\
		.id	= VPMIC_ID_##vreg,				\
		.owner	= THIS_MODULE,					\
	},								\
}


static struct vpmic_regulator_info vpmic_info[] = {
	VPMIC_DVC(Vdd_IO),
	VPMIC_DVC(VBat),
	VPMIC_DVC(VSim),
};


static int __devinit vpmic_regulator_probe(struct platform_device *pdev)
{
	struct vpmic_regulator_info *info = NULL;
	struct regulator_init_data *pdata = NULL;
	int i = 0;

	pdata = pdev->dev.platform_data;
	if (pdata == NULL)
		return -EINVAL;

	for (i = 0; i < ARRAY_SIZE(vpmic_info); i++) {
		info = &vpmic_info[i];
		if (!strcmp(info->desc.name, pdata->constraints.name))
			break;
	}

	if (i >= ARRAY_SIZE(vpmic_info)) {
		dev_err(&pdev->dev, "Failed to find regulator %s\n",
			pdata->constraints.name);
		return -EINVAL;
	}

	info->regulator = regulator_register(&info->desc, &pdev->dev,
						pdata, info);
	if (IS_ERR(info->regulator)) {
		dev_err(&pdev->dev, "failed to register regulator %s\n",
			info->desc.name);
		return PTR_ERR(info->regulator);
	}

	platform_set_drvdata(pdev, info);

	return 0;
}

static int __devexit vpmic_regulator_remove(struct platform_device *pdev)
{
	struct vpmic_regulator_info *info = platform_get_drvdata(pdev);

	platform_set_drvdata(pdev, NULL);
	regulator_unregister(info->regulator);
	return 0;
}

static struct platform_driver vpmic_regulator_driver = {
	.driver		= {
		.name	= "vpmic-regulator",
		.owner	= THIS_MODULE,
	},
	.probe		= vpmic_regulator_probe,
	.remove		= __devexit_p(vpmic_regulator_remove),
};

static int __init vpmic_regulator_init(void)
{
	return platform_driver_register(&vpmic_regulator_driver);
}
subsys_initcall(vpmic_regulator_init);

static void __exit vpmic_regulator_exit(void)
{
	platform_driver_unregister(&vpmic_regulator_driver);
}
module_exit(vpmic_regulator_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Regulator Driver for Marvell Virtual PMIC");
