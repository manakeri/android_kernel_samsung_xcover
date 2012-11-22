#ifndef __ASM_MACH_PXA910_H
#define __ASM_MACH_PXA910_H

#include <linux/i2c.h>
#include <mach/devices.h>
#include <plat/i2c.h>
#include <plat/pxa3xx_nand.h>
#include <plat/sdhci.h>
#include <plat/pxa27x_keypad.h>
#include <mach/pxa168fb.h>
#include <mach/pxa2xx_spi.h>

extern struct pxa_device_desc pxa910_device_uart0;
extern struct pxa_device_desc pxa910_device_uart1;
extern struct pxa_device_desc pxa910_device_uart2;
extern struct pxa_device_desc pxa910_device_pwm1;
extern struct pxa_device_desc pxa910_device_pwm2;
extern struct pxa_device_desc pxa910_device_pwm3;
extern struct pxa_device_desc pxa910_device_pwm4;
extern struct pxa_device_desc pxa910_device_nand;
extern struct platform_device pxa910_device_acipc;
extern struct pxa_device_desc pxa910_device_ire;
extern struct pxa_device_desc pxa910_device_fb;
extern struct pxa_device_desc pxa910_device_fb_ovly;
extern struct platform_device pxa910_device_freq;
extern struct pxa_device_desc pxa910_device_sdh0;
extern struct pxa_device_desc pxa910_device_sdh1;
extern struct pxa_device_desc pxa910_device_sdh2;

extern struct platform_device pxa910_device_rtc;
extern struct platform_device pxa910_device_1wire;
extern struct platform_device pxa910_device_twsi0;
extern struct platform_device pxa910_device_twsi1;
extern struct platform_device pxa_device_u2o;
extern struct platform_device pxa_device_u2h;
extern struct platform_device pxa_device_u2ootg;
extern struct platform_device pxa_device_u2oehci;
extern struct pxa_device_desc pxa910_device_camera;

extern struct pxa_device_desc pxa910_device_ssp0;
extern struct pxa_device_desc pxa910_device_ssp1;
extern struct pxa_device_desc pxa910_device_ssp2;

extern struct pxa_device_desc pxa910_device_keypad;
extern struct pxa_device_desc pxa910_device_cnm;

extern void pxa910_clear_keypad_wakeup(void);

int pxa_usb_phy_init(unsigned int base);
int pxa_usb_phy_deinit(unsigned int base);
void pxa910_sdh_specific_ctrl(struct sdhci_host *host,
		struct sdhci_pxa_platdata *pdata);
int pxa910_mmc_set_width(struct sdhci_host *host, int width);

static inline int pxa910_add_uart(int id)
{
	struct pxa_device_desc *d = NULL;

	switch (id) {
	case 0: d = &pxa910_device_uart0; break;
	case 1: d = &pxa910_device_uart1; break;
	case 2: d = &pxa910_device_uart2; break;
	}

	if (d == NULL)
		return -EINVAL;

	return pxa_register_device(d, NULL, 0);
}

static inline void pxa910_add_acipc(void)
{
	int ret;
	ret = platform_device_register(&pxa910_device_acipc);
	if (ret)
		dev_err(&pxa910_device_acipc.dev,
			"unable to register device: %d\n", ret);
}

static inline int pxa910_add_ire(void)
{
	return pxa_register_device(&pxa910_device_ire, NULL, 0);
}

static inline int pxa910_add_pwm(int id)
{
	struct pxa_device_desc *d = NULL;

	switch (id) {
	case 1: d = &pxa910_device_pwm1; break;
	case 2: d = &pxa910_device_pwm2; break;
	case 3: d = &pxa910_device_pwm3; break;
	case 4: d = &pxa910_device_pwm4; break;
	default:
		return -EINVAL;
	}

	return pxa_register_device(d, NULL, 0);
}

static inline int pxa910_add_sdh(int id, struct sdhci_pxa_platdata *data)
{
	struct pxa_device_desc *d = NULL;

	switch (id) {
	case 0: d = &pxa910_device_sdh0; break;
	case 1: d = &pxa910_device_sdh1; break;
	case 2: d = &pxa910_device_sdh2; break;
	default:
		return -EINVAL;
	}

	return pxa_register_device(d, data, sizeof(*data));
}

static inline int pxa910_add_nand(struct pxa3xx_nand_platform_data *info)
{
	return pxa_register_device(&pxa910_device_nand, info, sizeof(*info));
}

static inline int pxa910_add_fb(struct pxa168fb_mach_info *mi)
{
	return pxa_register_device(&pxa910_device_fb, mi, sizeof(*mi));
}

static inline int pxa910_add_fb_ovly(struct pxa168fb_mach_info *mi)
{
	return pxa_register_device(&pxa910_device_fb_ovly, mi, sizeof(*mi));
}

static inline void pxa910_add_freq(void)
{
        int ret;
        ret = platform_device_register(&pxa910_device_freq);
	if (ret)
		dev_err(&pxa910_device_freq.dev,
			"unable to register device: %d\n", ret);
}

static inline int pxa910_add_cam(void)
{
       return pxa_register_device(&pxa910_device_camera, NULL, 0);
}

static inline int pxa910_add_ssp(int id)
{
	struct pxa_device_desc *d = NULL;

	switch (id) {
		case 0: d = &pxa910_device_ssp0; break;
		case 1: d = &pxa910_device_ssp1; break;
		case 2: d = &pxa910_device_ssp2; break;
		default:
			return -EINVAL;
	}

	return pxa_register_device(d, NULL, 0);
}

static inline void pxa910_add_1wire(void)
{
	int ret;
	ret = platform_device_register(&pxa910_device_1wire);
	if (ret)
		dev_err(&pxa910_device_1wire.dev,
			"unable to register device: %d\n", ret);
}

static inline int pxa910_add_keypad(struct pxa27x_keypad_platform_data *data)
{
	data->clear_wakeup_event = pxa910_clear_keypad_wakeup;
	return pxa_register_device(&pxa910_device_keypad, data, sizeof(*data));
}

static inline int pxa910_add_cnm(void)
{
	return pxa_register_device(&pxa910_device_cnm, NULL, 0);
}

static inline int pxa910_add_spi(int id, struct pxa2xx_spi_master *pdata)
{
	struct platform_device *pd;

	pd = platform_device_alloc("pxa2xx-spi", id);
	if (pd == NULL) {
		pr_err("pxa2xx-spi: failed to allocate device (id=%d)\n", id);
		return -ENOMEM;
	}

	platform_device_add_data(pd, pdata, sizeof(*pdata));

	return platform_device_add(pd);
}

#endif /* __ASM_MACH_PXA910_H */
