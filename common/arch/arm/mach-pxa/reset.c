/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/io.h>
#include <asm/proc-fns.h>

#include <mach/regs-ost.h>
#include <mach/pxa95x-regs.h>
#include <mach/reset.h>

#include <mach/ramdump_defs.h> /* common definitions reused in OSL */
#include <mach/spa.h>

unsigned int reset_status;
EXPORT_SYMBOL(reset_status);

/*PPS_SAMSUNG: added for power off charging */
extern struct rdc_area *rdc_va; 
#define REBOOT_INFORM_BOOTLOADER 0x55
/*PPS_SAMSUNG: Added for recovery */
#define RECOVERY_BOOT 0x01

static void do_hw_reset(void);

static int reset_gpio = -1;

int init_gpio_reset(int gpio, int output, int level)
{
	int rc;

	rc = gpio_request(gpio, "reset generator");
	if (rc) {
		printk(KERN_ERR "Can't request reset_gpio\n");
		goto out;
	}

	if (output)
		rc = gpio_direction_output(gpio, level);
	else
		rc = gpio_direction_input(gpio);
	if (rc) {
		printk(KERN_ERR "Can't configure reset_gpio\n");
		gpio_free(gpio);
		goto out;
	}

out:
	if (!rc)
		reset_gpio = gpio;

	return rc;
}

/*
 * Trigger GPIO reset.
 * This covers various types of logic connecting gpio pin
 * to RESET pins (nRESET or GPIO_RESET):
 */
static void do_gpio_reset(void)
{
	BUG_ON(reset_gpio == -1);

	/* drive it low */
	gpio_direction_output(reset_gpio, 0);
	mdelay(2);
	/* rising edge or drive high */
	gpio_set_value(reset_gpio, 1);
	mdelay(2);
	/* falling edge */
	gpio_set_value(reset_gpio, 0);

	/* give it some time */
	mdelay(10);

	WARN_ON(1);
	/* fallback */
	do_hw_reset();
}

static void do_hw_reset(void)
{
	if (cpu_is_pxa970()) {
		/* GPIO reset */
		PSPR = 0x5c014000;
		PMCR = (PMCR & PMCR_TIE) | PMCR_SWGR;
	} else if (cpu_is_pxa95x()) {
		/* GPIO reset */
		PSPR = 0x5c014000;
		PMCR = (PMCR & (PMCR_BIE | PMCR_TIE | PMCR_VIE))
			| PMCR_SWGR;
	} else {
		/* Initialize the watchdog and let it fire */
		OWER = OWER_WME;
		OSSR = OSSR_M3;
		OSMR3 = OSCR + 368640;  /* ... in 100 ms */
	}
}

void arch_reset(char mode, const char *cmd)
{
	/*PPS_SAMSUNG : write REBOOT_INFORM_BOOTLOADER to genral purpose register for power chraging */
	if( rdc_va!=NULL )
	{ 
		rdc_va->header.reserved[0] = REBOOT_INFORM_BOOTLOADER;
		if(cmd)
		{
			if(!strcmp(cmd, "recovery"))
			{
				/*PPS_SAMSUNG : write REBOOT cause to this field  */
				rdc_va->header.reserved[1] = RECOVERY_BOOT;
			}
			else
			{
				/*PPS_SAMSUNG : No REBOOT cause  */
				rdc_va->header.reserved[1] = 0;
			}
		}
	}
	

	clear_reset_status(RESET_STATUS_ALL);

	switch (mode) {
	case 's':
		/* Jump into ROM at address 0 */
		cpu_reset(0);
		break;
	case 'g':
		do_gpio_reset();
		break;
	case 'h':
	default:
		do_hw_reset();
		break;
	}
}


