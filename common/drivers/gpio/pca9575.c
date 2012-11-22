/*
 *  pca9575.c - 4/8/16 bit I/O ports
 *
 *  Copyright (C) 2005 Ben Gardner <bgardner@wabtec.com>
 *  Copyright (C) 2007 Marvell International Ltd.
 *
 *  Derived from drivers/i2c/chips/pca9539.c
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; version 2 of the License.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/workqueue.h>
#include <linux/i2c.h>
#include <linux/i2c/pca9575.h>
#include <linux/slab.h>

#include <asm/gpio.h>

#define PCA9575_IN	0
#define PCA9575_INVRT	1
#define PCA9575_BKEN	2
#define PCA9575_PUPD	3
#define PCA9575_CFG	4
#define PCA9575_OUT	5
#define PCA9575_MSK	6
#define PCA9575_INTS	7

static const struct i2c_device_id pca9575_id[] = {
	{ "pca9575", 16, },
	{ }
};
MODULE_DEVICE_TABLE(i2c, pca9575_id);

struct pca9575_chip *g_chip;
struct pca9575_chip {
	unsigned gpio_start;
	uint16_t reg_output;
	uint16_t reg_direction;

	struct i2c_client *client;
	struct gpio_chip gpio_chip;
#ifdef CONFIG_GPIO_PCA9575_GENERIC_IRQ
	uint16_t last_input;
	/*
	 * Note: Generic IRQ is not accessible within module code, the IRQ
	 * support will thus _only_ be available if the driver is built-in
	 */
	int irq;	/* IRQ for the chip itself */
	int irq_start;	/* starting IRQ for the on-chip GPIO lines */

	uint16_t irq_mask;
	uint16_t irq_falling_edge;
	uint16_t irq_rising_edge;

	struct irq_chip irq_chip;
	struct work_struct irq_work;
#endif
};

static int pca9575_write_reg(struct pca9575_chip *chip, int reg, uint16_t val)
{
	int ret;

	if (chip->gpio_chip.ngpio <= 8)
		ret = i2c_smbus_write_byte_data(chip->client, reg, val);
	else {
		reg = reg << 1;
		ret = i2c_smbus_write_byte_data(chip->client, reg, val & 0xff);
		ret = i2c_smbus_write_byte_data(chip->client, reg + 1, (val & 0xff00) >> 8);
	}

	if (ret < 0) {
		dev_err(&chip->client->dev, "failed writing register\n");
		return ret;
	}

	return 0;
}

static int pca9575_read_reg(struct pca9575_chip *chip, int reg, uint16_t *val)
{
	int ret, reth, retl;

	if (chip->gpio_chip.ngpio <= 8)
		ret = i2c_smbus_read_byte_data(chip->client, reg);
	else {
		reg = reg << 1;
		retl = i2c_smbus_read_byte_data(chip->client, reg);
		reth = i2c_smbus_read_byte_data(chip->client, reg + 1);
		ret = (retl & 0xff) | (reth << 8);
	}

	if (ret < 0) {
		dev_err(&chip->client->dev, "failed reading register\n");
		return ret;
	}

	*val = (uint16_t)ret;
	return 0;
}

static int pca9575_gpio_direction_input(struct gpio_chip *gc, unsigned off)
{
	struct pca9575_chip *chip;
	uint16_t reg_val;
	int ret;

	chip = container_of(gc, struct pca9575_chip, gpio_chip);

	reg_val = chip->reg_direction | (1u << off);
	ret = pca9575_write_reg(chip, PCA9575_CFG, reg_val);
	if (ret)
		return ret;

	chip->reg_direction = reg_val;
	return 0;
}

static int pca9575_gpio_direction_output(struct gpio_chip *gc,
		unsigned off, int val)
{
	struct pca9575_chip *chip;
	uint16_t reg_val;
	int ret;

	chip = container_of(gc, struct pca9575_chip, gpio_chip);

	/* set output level */
	if (val)
		reg_val = chip->reg_output | (1u << off);
	else
		reg_val = chip->reg_output & ~(1u << off);

	ret = pca9575_write_reg(chip, PCA9575_OUT, reg_val);
	if (ret)
		return ret;
	/*If output value equals to 1, pull up, or 0, pull down to save current eakage*/
	ret = pca9575_write_reg(chip, PCA9575_PUPD, reg_val);
	if (ret)
		return ret;

	chip->reg_output = reg_val;

	/* then direction */
	reg_val = chip->reg_direction & ~(1u << off);
	ret = pca9575_write_reg(chip, PCA9575_CFG, reg_val);
	if (ret)
		return ret;

	chip->reg_direction = reg_val;
	return 0;
}

static int pca9575_gpio_get_value(struct gpio_chip *gc, unsigned off)
{
	struct pca9575_chip *chip;
	uint16_t reg_val;
	int ret;

	chip = container_of(gc, struct pca9575_chip, gpio_chip);

	ret = pca9575_read_reg(chip, PCA9575_IN, &reg_val);
	if (ret < 0) {
		/* NOTE:  diagnostic already emitted; that's all we should
		 * do unless gpio_*_value_cansleep() calls become different
		 * from their nonsleeping siblings (and report faults).
		 */
		return 0;
	}

	return (reg_val & (1u << off)) ? 1 : 0;
}

static void pca9575_gpio_set_value(struct gpio_chip *gc, unsigned off, int val)
{
	struct pca9575_chip *chip;
	uint16_t reg_val;
	int ret;

	chip = container_of(gc, struct pca9575_chip, gpio_chip);

	if (val)
		reg_val = chip->reg_output | (1u << off);
	else
		reg_val = chip->reg_output & ~(1u << off);

	ret = pca9575_write_reg(chip, PCA9575_OUT, reg_val);
	if (ret)
		return;
	ret = pca9575_write_reg(chip, PCA9575_PUPD, reg_val);
	if (ret)
		return;

	chip->reg_output = reg_val;
}

static void pca9575_setup_gpio(struct pca9575_chip *chip, int gpios)
{
	struct gpio_chip *gc;

	gc = &chip->gpio_chip;

	gc->direction_input  = pca9575_gpio_direction_input;
	gc->direction_output = pca9575_gpio_direction_output;
	gc->get = pca9575_gpio_get_value;
	gc->set = pca9575_gpio_set_value;
	gc->can_sleep = 1;

	gc->base = chip->gpio_start;
	gc->ngpio = gpios;
	gc->label = chip->client->name;
	gc->dev = &chip->client->dev;
	gc->owner = THIS_MODULE;
}

#ifdef CONFIG_GPIO_PCA9575_GENERIC_IRQ
/* FIXME: change to schedule_delayed_work() here if reading out of
 * registers does not reflect the actual pin levels
 */

static void pca9575_irq_work(struct work_struct *work)
{
	struct pca9575_chip *chip;
	uint16_t input, mask, rising, falling;
	int ret, i;

	chip = container_of(work, struct pca9575_chip, irq_work);

	ret = pca9575_read_reg(chip, PCA9575_IN, &input);
	if (ret < 0)
		return;

	mask = (input ^ chip->last_input) & chip->irq_mask;
	rising = (input & mask) & chip->irq_rising_edge;
	falling = (~input & mask) & chip->irq_falling_edge;

	irq_enter();

	for (i = 0; i < chip->gpio_chip.ngpio; i++) {
		if ((rising | falling) & (1u << i)) {
			int irq = chip->irq_start + i;
			struct irq_desc *desc;

			desc = irq_desc + irq;
			desc_handle_irq(irq, desc);
		}
	}

	irq_exit();

	chip->last_input = input;
}

static void
pca9575_irq_demux(unsigned int irq, struct irq_desc *desc)
{
	struct pca9575_chip *chip = desc->handler_data;

	desc->chip->mask(chip->irq);
	desc->chip->ack(chip->irq);
	schedule_work(&chip->irq_work);
	desc->chip->unmask(chip->irq);
}

static void pca9575_irq_mask(unsigned int irq)
{
	struct irq_desc *desc = irq_desc + irq;
	struct pca9575_chip *chip = desc->chip_data;

	chip->irq_mask &= ~(1u << (irq - chip->irq_start));
}

static void pca9575_irq_unmask(unsigned int irq)
{
	struct irq_desc *desc = irq_desc + irq;
	struct pca9575_chip *chip = desc->chip_data;

	chip->irq_mask |= 1u << (irq - chip->irq_start);
}

static void pca9575_irq_ack(unsigned int irq)
{
	/* unfortunately, we have to provide an empty irq_chip.ack even
	 * if we do nothing here, Generic IRQ will complain otherwise
	 */
}

static int pca9575_irq_set_type(unsigned int irq, unsigned int type)
{
	struct irq_desc *desc = irq_desc + irq;
	struct pca9575_chip *chip = desc->chip_data;
	uint16_t mask = 1u << (irq - chip->irq_start);
	int gpio = irq_to_gpio(irq);

	if (type == IRQ_TYPE_PROBE) {
		if ((mask & chip->irq_rising_edge) ||
		    (mask & chip->irq_falling_edge) ||
		    (mask & ~chip->reg_direction))
			return 0;

		type = IRQ_TYPE_EDGE_RISING | IRQ_TYPE_EDGE_FALLING;
	}

	if (gpio_request(gpio, "gpio expander pin")) {
		printk(KERN_ERR "Request GPIO failed, gpio: 0x%x\n", gpio);
		return -1;
	}
	gpio_direction_input(gpio);
	gpio_free(gpio);

	if (type & IRQ_TYPE_EDGE_RISING)
		chip->irq_rising_edge |= mask;
	else
		chip->irq_rising_edge &= ~mask;

	if (type & IRQ_TYPE_EDGE_FALLING)
		chip->irq_falling_edge |= mask;
	else
		chip->irq_falling_edge &= ~mask;

	return 0;
}

static int pca9575_init_irq(struct pca9575_chip *chip)
{
	struct irq_chip *ic = &chip->irq_chip;
	int irq, irq_start = chip->irq_start;

	chip->irq = chip->client->irq;
	chip->irq_start = irq_start = gpio_to_irq(chip->gpio_start);

	/* do not install GPIO interrupts for the chip if
	 * 1. the PCA9575 interrupt line is not used
	 * 2. or the GPIO interrupt number exceeds NR_IRQS
	 */
	if (chip->irq <= 0 || irq_start + chip->gpio_chip.ngpio >= NR_IRQS)
		return -EINVAL;

	chip->irq_mask	= 0;
	chip->irq_rising_edge  = 0;
	chip->irq_falling_edge = 0;

	ic->ack = pca9575_irq_ack;
	ic->mask = pca9575_irq_mask;
	ic->unmask = pca9575_irq_unmask;
	ic->set_type = pca9575_irq_set_type;

	for (irq = irq_start; irq < irq_start + chip->gpio_chip.ngpio; irq++) {
		set_irq_chip(irq, ic);
		set_irq_chip_data(irq, chip);
		set_irq_handler(irq, handle_edge_irq);
		set_irq_flags(irq, IRQF_VALID | IRQF_PROBE);
	}

	set_irq_type(chip->irq, IRQ_TYPE_EDGE_FALLING);
	set_irq_data(chip->irq, chip);
	set_irq_chained_handler(chip->irq, pca9575_irq_demux);

	INIT_WORK(&chip->irq_work, pca9575_irq_work);
	return 0;
}
#else
static inline int pca9575_init_irq(struct pca9575_chip *chip)
{
	return 0;
}
#endif /* CONFIG_GPIO_PCA9575_GENERIC_IRQ */

static int __devinit pca9575_probe(struct i2c_client *client,
				   const struct i2c_device_id *id)
{
	struct pca9575_platform_data *pdata;
	struct pca9575_chip *chip;
	int ret;
	uint16_t val;

	pdata = client->dev.platform_data;
	if (pdata == NULL) {
		dev_dbg(&client->dev, "no platform data\n");
		return -EINVAL;
	}

	chip = kzalloc(sizeof(struct pca9575_chip), GFP_KERNEL);
	if (chip == NULL)
		return -ENOMEM;

	chip->client = client;

	chip->gpio_start = pdata->gpio_base;

	g_chip = chip;
	/* initialize cached registers from their original values.
	 * we can't share this chip with another i2c master.
	 */
	pca9575_setup_gpio(chip, id->driver_data);

	/*Let every port in proper state , that could save 0.3mA in buck3*/
	pca9575_write_reg(chip, PCA9575_PUPD, 0x0);
	pca9575_write_reg(chip, PCA9575_CFG, 0xffff);
	pca9575_write_reg(chip, PCA9575_OUT, 0x0);
	/* some chips need read first to validate address,
	 * if SCL SDA is used for address
	 */
	ret = pca9575_read_reg(chip, PCA9575_IN, &val);
	if (ret)
		goto out_failed;

	ret = pca9575_read_reg(chip, PCA9575_OUT, &chip->reg_output);
	if (ret)
		goto out_failed;

	ret = pca9575_read_reg(chip, PCA9575_CFG, &chip->reg_direction);
	if (ret)
		goto out_failed;

	/* set platform specific polarity inversion */
	ret = pca9575_write_reg(chip, PCA9575_INVRT, pdata->invert);
	if (ret)
		goto out_failed;

	/*To enable register 6, 7 to controll pull up and pull down*/
	pca9575_write_reg(chip, PCA9575_BKEN, 0x202);

	ret = gpiochip_add(&chip->gpio_chip);
	if (ret)
		goto out_failed;

	if (pdata->setup) {
		ret = pdata->setup(client, chip->gpio_chip.base,
				chip->gpio_chip.ngpio, pdata->context);
		if (ret < 0)
			dev_warn(&client->dev, "setup failed, %d\n", ret);
	}

	printk("%s:detected\n", __func__);

#if defined(CONFIG_WLAN_8688_SDIO)
	if (pdata->poweron) {
		ret = pdata->poweron();
		if (ret < 0)
			dev_warn(&client->dev, "poweron failed, %d\n", ret);
	}
#endif

	if(client->irq > 0){
		/* clear the interrupt */
		ret = pca9575_read_reg(chip, PCA9575_IN, &val);
		if (ret)
			goto out_failed;


		ret = pca9575_init_irq(chip);
		if (ret) {
			ret = gpiochip_remove(&chip->gpio_chip);
			goto out_failed;
		}

	}

	i2c_set_clientdata(client, chip);
	return 0;

out_failed:
	kfree(chip);
	return ret;
}

#ifdef CONFIG_GPIO_PCA9575_GENERIC_IRQ
static int pca9575_remove(struct i2c_client *client)
{
	printk(KERN_ERR "failed to unload the driver with IRQ support\n");
	return -EINVAL;
}
#else

static int pca9575_remove(struct i2c_client *client)
{
	struct pca9575_platform_data *pdata = client->dev.platform_data;
	struct pca9575_chip *chip = i2c_get_clientdata(client);
	int ret = 0;

	if (pdata->teardown) {
		ret = pdata->teardown(client, chip->gpio_chip.base,
				chip->gpio_chip.ngpio, pdata->context);
		if (ret < 0) {
			dev_err(&client->dev, "%s failed, %d\n",
					"teardown", ret);
			return ret;
		}
	}

	ret = gpiochip_remove(&chip->gpio_chip);
	if (ret) {
		dev_err(&client->dev, "%s failed, %d\n",
				"gpiochip_remove()", ret);
		return ret;
	}

	kfree(chip);
	return 0;
}
#endif /* CONFIG_GPIO_PCA9575_GENERIC_IRQ */

static struct i2c_driver pca9575_driver = {
	.driver = {
		.name	= "pca9575",
	},
	.probe		= pca9575_probe,
	.remove		= pca9575_remove,
	.id_table	= pca9575_id,
};

static int __init pca9575_init(void)
{
	return i2c_add_driver(&pca9575_driver);
}
/* register after i2c postcore initcall and before
 * subsys initcalls that may rely on these GPIOs
 */
subsys_initcall_sync(pca9575_init);

static void __exit pca9575_exit(void)
{
	i2c_del_driver(&pca9575_driver);
}
module_exit(pca9575_exit);

MODULE_AUTHOR("Jack Ren <jack.ren@marvell.com>");
MODULE_DESCRIPTION("GPIO expander driver for PCA9575");
MODULE_LICENSE("GPL");
