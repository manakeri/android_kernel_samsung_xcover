#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/dma-mapping.h>

#include <mach/udc.h>
#include <mach/pxafb.h>
#include <mach/mmc.h>
#include <mach/irda.h>
#include <mach/ohci.h>
#include <plat/pxa27x_keypad.h>
#include <mach/pxa2xx_spi.h>
#include <mach/camera.h>
#include <mach/audio.h>
#include <mach/hardware.h>
#include <mach/pxa95x-regs.h>
#include <mach/pxa95x_dvfm.h>
#include <mach/soc_vmeta.h>
#include <plat/i2c.h>
#include <plat/pxa3xx_nand.h>
#include <plat/pxa3xx_onenand.h>
#include <plat/pxa_u2o.h>

#include "devices.h"
#include "generic.h"

void __init pxa_register_device(struct platform_device *dev, void *data)
{
	int ret;

	dev->dev.platform_data = data;

	ret = platform_device_register(dev);
	if (ret)
		dev_err(&dev->dev, "unable to register device: %d\n", ret);
}

static struct resource pxamci_resources[] = {
	[0] = {
		.start	= 0x41100000,
		.end	= 0x41100fff,
		.flags	= IORESOURCE_MEM,
	},
	[1] = {
		.start	= IRQ_MMC,
		.end	= IRQ_MMC,
		.flags	= IORESOURCE_IRQ,
	},
	[2] = {
		.start	= 21,
		.end	= 21,
		.flags	= IORESOURCE_DMA,
	},
	[3] = {
		.start	= 22,
		.end	= 22,
		.flags	= IORESOURCE_DMA,
	},
};

static u64 pxamci_dmamask = 0xffffffffUL;

struct platform_device pxa_device_mci = {
	.name		= "pxa2xx-mci",
	.id		= 0,
	.dev		= {
		.dma_mask = &pxamci_dmamask,
		.coherent_dma_mask = 0xffffffff,
	},
	.num_resources	= ARRAY_SIZE(pxamci_resources),
	.resource	= pxamci_resources,
};

void __init pxa_set_mci_info(struct pxamci_platform_data *info)
{
	pxa_register_device(&pxa_device_mci, info);
}


static struct pxa2xx_udc_mach_info pxa_udc_info = {
	.gpio_pullup = -1,
	.gpio_vbus   = -1,
};

void __init pxa_set_udc_info(struct pxa2xx_udc_mach_info *info)
{
	memcpy(&pxa_udc_info, info, sizeof *info);
}

static struct resource pxa2xx_udc_resources[] = {
	[0] = {
		.start	= 0x40600000,
		.end	= 0x4060ffff,
		.flags	= IORESOURCE_MEM,
	},
	[1] = {
		.start	= IRQ_USB,
		.end	= IRQ_USB,
		.flags	= IORESOURCE_IRQ,
	},
};

static u64 udc_dma_mask = ~(u32)0;

struct platform_device pxa25x_device_udc = {
	.name		= "pxa25x-udc",
	.id		= -1,
	.resource	= pxa2xx_udc_resources,
	.num_resources	= ARRAY_SIZE(pxa2xx_udc_resources),
	.dev		=  {
		.platform_data	= &pxa_udc_info,
		.dma_mask	= &udc_dma_mask,
	}
};

struct platform_device pxa27x_device_udc = {
	.name		= "pxa27x-udc",
	.id		= -1,
	.resource	= pxa2xx_udc_resources,
	.num_resources	= ARRAY_SIZE(pxa2xx_udc_resources),
	.dev		=  {
		.platform_data	= &pxa_udc_info,
		.dma_mask	= &udc_dma_mask,
	}
};

static struct resource pxafb_resources[] = {
	[0] = {
		.start	= 0x44000000,
		.end	= 0x4400ffff,
		.flags	= IORESOURCE_MEM,
	},
	[1] = {
		.start	= IRQ_LCD,
		.end	= IRQ_LCD,
		.flags	= IORESOURCE_IRQ,
	},
};

static u64 fb_dma_mask = ~(u64)0;

struct platform_device pxa_device_fb = {
	.name		= "pxa2xx-fb",
	.id		= -1,
	.dev		= {
		.dma_mask	= &fb_dma_mask,
		.coherent_dma_mask = 0xffffffff,
	},
	.num_resources	= ARRAY_SIZE(pxafb_resources),
	.resource	= pxafb_resources,
};

void __init set_pxa_fb_info(struct pxafb_mach_info *info)
{
	pxa_register_device(&pxa_device_fb, info);
}

void __init set_pxa_fb_parent(struct device *parent_dev)
{
	pxa_device_fb.dev.parent = parent_dev;
}

static struct resource pxa_resource_ffuart[] = {
	{
		.start	= 0x40100000,
		.end	= 0x40100023,
		.flags	= IORESOURCE_MEM,
	}, {
		.start	= IRQ_FFUART,
		.end	= IRQ_FFUART,
		.flags	= IORESOURCE_IRQ,
	}, {
		.start	= 6,
		.end	= 6,
		.flags	= IORESOURCE_DMA,
	}, {
		.start	= 7,
		.end	= 7,
		.flags	= IORESOURCE_DMA,
	},
};

struct platform_device pxa_device_ffuart = {
	.name		= "pxa2xx-uart",
	.id		= 0,
	.resource	= pxa_resource_ffuart,
	.num_resources	= ARRAY_SIZE(pxa_resource_ffuart),
};

void __init pxa_set_ffuart_info(void *info)
{
	pxa_register_device(&pxa_device_ffuart, info);
}

static struct resource pxa_resource_btuart[] = {
	{
		.start	= 0x40200000,
		.end	= 0x40200023,
		.flags	= IORESOURCE_MEM,
	}, {
		.start	= IRQ_BTUART,
		.end	= IRQ_BTUART,
		.flags	= IORESOURCE_IRQ,
	}, {
		.start	= 4,
		.end	= 4,
		.flags	= IORESOURCE_DMA,
	}, {
		.start	= 5,
		.end	= 5,
		.flags	= IORESOURCE_DMA,
	},
};

struct platform_device pxa_device_btuart = {
	.name		= "pxa2xx-uart",
	.id		= 1,
	.resource	= pxa_resource_btuart,
	.num_resources	= ARRAY_SIZE(pxa_resource_btuart),
};

void __init pxa_set_btuart_info(void *info)
{
	pxa_register_device(&pxa_device_btuart, info);
}

static struct resource pxa_resource_stuart[] = {
	{
		.start	= 0x40700000,
		.end	= 0x40700023,
		.flags	= IORESOURCE_MEM,
	}, {
		.start	= IRQ_STUART,
		.end	= IRQ_STUART,
		.flags	= IORESOURCE_IRQ,
	}, {
		.start	= 19,
		.end	= 19,
		.flags	= IORESOURCE_DMA,
	}, {
		.start	= 20,
		.end	= 20,
		.flags	= IORESOURCE_DMA,
	},
};

struct platform_device pxa_device_stuart = {
	.name		= "pxa2xx-uart",
	.id		= 2,
	.resource	= pxa_resource_stuart,
	.num_resources	= ARRAY_SIZE(pxa_resource_stuart),
};

void __init pxa_set_stuart_info(void *info)
{
	pxa_register_device(&pxa_device_stuart, info);
}

static struct resource pxa_resource_hwuart[] = {
	{
		.start	= 0x41600000,
		.end	= 0x4160002F,
		.flags	= IORESOURCE_MEM,
	}, {
		.start	= IRQ_HWUART,
		.end	= IRQ_HWUART,
		.flags	= IORESOURCE_IRQ,
	}
};

struct platform_device pxa_device_hwuart = {
	.name		= "pxa2xx-uart",
	.id		= 3,
	.resource	= pxa_resource_hwuart,
	.num_resources	= ARRAY_SIZE(pxa_resource_hwuart),
};

void __init pxa_set_hwuart_info(void *info)
{
	if (cpu_is_pxa255())
		pxa_register_device(&pxa_device_hwuart, info);
	else
		pr_info("UART: Ignoring attempt to register HWUART on non-PXA255 hardware");
}

static struct resource pxai2c_resources[] = {
	{
		.start	= 0x40301680,
		.end	= 0x403016a3,
		.flags	= IORESOURCE_MEM,
	}, {
		.start	= IRQ_I2C,
		.end	= IRQ_I2C,
		.flags	= IORESOURCE_IRQ,
	},
};

struct platform_device pxa_device_i2c = {
	.name		= "pxa2xx-i2c",
	.id		= 0,
	.resource	= pxai2c_resources,
	.num_resources	= ARRAY_SIZE(pxai2c_resources),
};

void __init pxa_set_i2c_info(struct i2c_pxa_platform_data *info)
{
	pxa_register_device(&pxa_device_i2c, info);
}

#ifdef CONFIG_PXA27x
static struct resource pxa27x_resources_i2c_power[] = {
	{
		.start	= 0x40f00180,
		.end	= 0x40f001a3,
		.flags	= IORESOURCE_MEM,
	}, {
		.start	= IRQ_PWRI2C,
		.end	= IRQ_PWRI2C,
		.flags	= IORESOURCE_IRQ,
	},
};

struct platform_device pxa27x_device_i2c_power = {
	.name		= "pxa2xx-i2c",
	.id		= 1,
	.resource	= pxa27x_resources_i2c_power,
	.num_resources	= ARRAY_SIZE(pxa27x_resources_i2c_power),
};
#endif

static struct resource pxai2s_resources[] = {
	{
		.start	= 0x40400000,
		.end	= 0x40400083,
		.flags	= IORESOURCE_MEM,
	}, {
		.start	= IRQ_I2S,
		.end	= IRQ_I2S,
		.flags	= IORESOURCE_IRQ,
	},
};

struct platform_device pxa_device_i2s = {
	.name		= "pxa2xx-i2s",
	.id		= -1,
	.resource	= pxai2s_resources,
	.num_resources	= ARRAY_SIZE(pxai2s_resources),
};

static u64 pxaficp_dmamask = ~(u32)0;

struct platform_device pxa_device_ficp = {
	.name		= "pxa2xx-ir",
	.id		= -1,
	.dev		= {
		.dma_mask = &pxaficp_dmamask,
		.coherent_dma_mask = 0xffffffff,
	},
};

void __init pxa_set_ficp_info(struct pxaficp_platform_data *info)
{
	pxa_register_device(&pxa_device_ficp, info);
}

static struct resource pxa_rtc_resources[] = {
	[0] = {
		.start  = 0x40900000,
		.end	= 0x40900000 + 0x3b,
		.flags  = IORESOURCE_MEM,
	},
	[1] = {
		.start  = IRQ_RTC1Hz,
		.end    = IRQ_RTC1Hz,
		.flags  = IORESOURCE_IRQ,
	},
	[2] = {
		.start  = IRQ_RTCAlrm,
		.end    = IRQ_RTCAlrm,
		.flags  = IORESOURCE_IRQ,
	},
};

struct platform_device sa1100_device_rtc = {
	.name		= "sa1100-rtc",
	.id		= -1,
};

struct platform_device pxa_device_rtc = {
	.name		= "pxa-rtc",
	.id		= -1,
	.num_resources  = ARRAY_SIZE(pxa_rtc_resources),
	.resource       = pxa_rtc_resources,
};

static struct resource pxa_ac97_resources[] = {
	[0] = {
		.start  = 0x40500000,
		.end	= 0x40500000 + 0xfff,
		.flags  = IORESOURCE_MEM,
	},
	[1] = {
		.start  = IRQ_AC97,
		.end    = IRQ_AC97,
		.flags  = IORESOURCE_IRQ,
	},
};

static u64 pxa_ac97_dmamask = 0xffffffffUL;

struct platform_device pxa_device_ac97 = {
	.name           = "pxa2xx-ac97",
	.id             = -1,
	.dev            = {
		.dma_mask = &pxa_ac97_dmamask,
		.coherent_dma_mask = 0xffffffff,
	},
	.num_resources  = ARRAY_SIZE(pxa_ac97_resources),
	.resource       = pxa_ac97_resources,
};

void __init pxa_set_ac97_info(pxa2xx_audio_ops_t *ops)
{
	pxa_register_device(&pxa_device_ac97, ops);
}

#ifdef CONFIG_PXA25x

static struct resource pxa25x_resource_pwm0[] = {
	[0] = {
		.start	= 0x40b00000,
		.end	= 0x40b0000f,
		.flags	= IORESOURCE_MEM,
	},
};

struct platform_device pxa25x_device_pwm0 = {
	.name		= "pxa25x-pwm",
	.id		= 0,
	.resource	= pxa25x_resource_pwm0,
	.num_resources	= ARRAY_SIZE(pxa25x_resource_pwm0),
};

static struct resource pxa25x_resource_pwm1[] = {
	[0] = {
		.start	= 0x40c00000,
		.end	= 0x40c0000f,
		.flags	= IORESOURCE_MEM,
	},
};

struct platform_device pxa25x_device_pwm1 = {
	.name		= "pxa25x-pwm",
	.id		= 1,
	.resource	= pxa25x_resource_pwm1,
	.num_resources	= ARRAY_SIZE(pxa25x_resource_pwm1),
};

static u64 pxa25x_ssp_dma_mask = DMA_BIT_MASK(32);

static struct resource pxa25x_resource_ssp[] = {
	[0] = {
		.start	= 0x41000000,
		.end	= 0x4100001f,
		.flags	= IORESOURCE_MEM,
	},
	[1] = {
		.start	= IRQ_SSP,
		.end	= IRQ_SSP,
		.flags	= IORESOURCE_IRQ,
	},
	[2] = {
		/* DRCMR for RX */
		.start	= 13,
		.end	= 13,
		.flags	= IORESOURCE_DMA,
	},
	[3] = {
		/* DRCMR for TX */
		.start	= 14,
		.end	= 14,
		.flags	= IORESOURCE_DMA,
	},
};

struct platform_device pxa25x_device_ssp = {
	.name		= "pxa25x-ssp",
	.id		= 0,
	.dev		= {
		.dma_mask = &pxa25x_ssp_dma_mask,
		.coherent_dma_mask = DMA_BIT_MASK(32),
	},
	.resource	= pxa25x_resource_ssp,
	.num_resources	= ARRAY_SIZE(pxa25x_resource_ssp),
};

static u64 pxa25x_nssp_dma_mask = DMA_BIT_MASK(32);

static struct resource pxa25x_resource_nssp[] = {
	[0] = {
		.start	= 0x41400000,
		.end	= 0x4140002f,
		.flags	= IORESOURCE_MEM,
	},
	[1] = {
		.start	= IRQ_NSSP,
		.end	= IRQ_NSSP,
		.flags	= IORESOURCE_IRQ,
	},
	[2] = {
		/* DRCMR for RX */
		.start	= 15,
		.end	= 15,
		.flags	= IORESOURCE_DMA,
	},
	[3] = {
		/* DRCMR for TX */
		.start	= 16,
		.end	= 16,
		.flags	= IORESOURCE_DMA,
	},
};

struct platform_device pxa25x_device_nssp = {
	.name		= "pxa25x-nssp",
	.id		= 1,
	.dev		= {
		.dma_mask = &pxa25x_nssp_dma_mask,
		.coherent_dma_mask = DMA_BIT_MASK(32),
	},
	.resource	= pxa25x_resource_nssp,
	.num_resources	= ARRAY_SIZE(pxa25x_resource_nssp),
};

static u64 pxa25x_assp_dma_mask = DMA_BIT_MASK(32);

static struct resource pxa25x_resource_assp[] = {
	[0] = {
		.start	= 0x41500000,
		.end	= 0x4150002f,
		.flags	= IORESOURCE_MEM,
	},
	[1] = {
		.start	= IRQ_ASSP,
		.end	= IRQ_ASSP,
		.flags	= IORESOURCE_IRQ,
	},
	[2] = {
		/* DRCMR for RX */
		.start	= 23,
		.end	= 23,
		.flags	= IORESOURCE_DMA,
	},
	[3] = {
		/* DRCMR for TX */
		.start	= 24,
		.end	= 24,
		.flags	= IORESOURCE_DMA,
	},
};

struct platform_device pxa25x_device_assp = {
	/* ASSP is basically equivalent to NSSP */
	.name		= "pxa25x-nssp",
	.id		= 2,
	.dev		= {
		.dma_mask = &pxa25x_assp_dma_mask,
		.coherent_dma_mask = DMA_BIT_MASK(32),
	},
	.resource	= pxa25x_resource_assp,
	.num_resources	= ARRAY_SIZE(pxa25x_resource_assp),
};
#endif /* CONFIG_PXA25x */

#if defined(CONFIG_PXA27x) || defined(CONFIG_PXA3xx) || defined(CONFIG_PXA93x)
static u64 pxa27x_ohci_dma_mask = DMA_BIT_MASK(32);

static struct resource pxa27x_resource_ohci[] = {
	[0] = {
		.start  = 0x4C000000,
		.end    = 0x4C00ff6f,
		.flags  = IORESOURCE_MEM,
	},
	[1] = {
		.start  = IRQ_USBH1,
		.end    = IRQ_USBH1,
		.flags  = IORESOURCE_IRQ,
	},
};

struct platform_device pxa27x_device_ohci = {
	.name		= "pxa27x-ohci",
	.id		= -1,
	.dev		= {
		.dma_mask = &pxa27x_ohci_dma_mask,
		.coherent_dma_mask = DMA_BIT_MASK(32),
	},
	.num_resources  = ARRAY_SIZE(pxa27x_resource_ohci),
	.resource       = pxa27x_resource_ohci,
};

void __init pxa_set_ohci_info(struct pxaohci_platform_data *info)
{
	pxa_register_device(&pxa27x_device_ohci, info);
}

static u64 pxa27x_dma_mask_camera = DMA_BIT_MASK(32);

static struct resource pxa27x_resource_camera[] = {
	[0] = {
		.start	= 0x50000000,
		.end	= 0x50000fff,
		.flags	= IORESOURCE_MEM,
	},
	[1] = {
		.start	= IRQ_CAMERA,
		.end	= IRQ_CAMERA,
		.flags	= IORESOURCE_IRQ,
	},
};

static struct platform_device pxa27x_device_camera = {
	.name		= "pxa27x-camera",
	.id		= 0, /* This is used to put cameras on this interface */
	.dev		= {
		.dma_mask      		= &pxa27x_dma_mask_camera,
		.coherent_dma_mask	= 0xffffffff,
	},
	.num_resources	= ARRAY_SIZE(pxa27x_resource_camera),
	.resource	= pxa27x_resource_camera,
};

void __init pxa_set_camera_info(struct pxacamera_platform_data *info)
{
	pxa_register_device(&pxa27x_device_camera, info);
}
#endif	/* CONFIG_PXA27x || CONFIG_PXA3xx || CONFIG_PXA93x */

#if defined(CONFIG_PXA27x) || defined(CONFIG_PXA3xx)		\
	|| defined(CONFIG_PXA93x) || defined(CONFIG_PXA95x)

static struct resource pxa27x_resource_keypad[] = {
	[0] = {
		.start	= 0x41500000,
		.end	= 0x4150004c,
		.flags	= IORESOURCE_MEM,
	},
	[1] = {
		.start	= IRQ_KEYPAD,
		.end	= IRQ_KEYPAD,
		.flags	= IORESOURCE_IRQ,
	},
};

struct platform_device pxa27x_device_keypad = {
	.name		= "pxa27x-keypad",
	.id		= -1,
	.resource	= pxa27x_resource_keypad,
	.num_resources	= ARRAY_SIZE(pxa27x_resource_keypad),
};

void __init pxa_set_keypad_info(struct pxa27x_keypad_platform_data *info)
{
	pxa_register_device(&pxa27x_device_keypad, info);
}

static u64 pxa27x_ssp1_dma_mask = DMA_BIT_MASK(32);

static struct resource pxa27x_resource_ssp1[] = {
	[0] = {
		.start	= 0x41000000,
		.end	= 0x41000043,
		.flags	= IORESOURCE_MEM,
	},
	[1] = {
		.start	= IRQ_SSP,
		.end	= IRQ_SSP,
		.flags	= IORESOURCE_IRQ,
	},
	[2] = {
		/* DRCMR for RX */
		.start	= 13,
		.end	= 13,
		.flags	= IORESOURCE_DMA,
	},
	[3] = {
		/* DRCMR for TX */
		.start	= 14,
		.end	= 14,
		.flags	= IORESOURCE_DMA,
	},
};

struct platform_device pxa27x_device_ssp1 = {
	.name		= "pxa27x-ssp",
	.id		= 0,
	.dev		= {
		.dma_mask = &pxa27x_ssp1_dma_mask,
		.coherent_dma_mask = DMA_BIT_MASK(32),
	},
	.resource	= pxa27x_resource_ssp1,
	.num_resources	= ARRAY_SIZE(pxa27x_resource_ssp1),
};

static u64 pxa27x_ssp2_dma_mask = DMA_BIT_MASK(32);

static struct resource pxa27x_resource_ssp2[] = {
	[0] = {
		.start	= 0x41700000,
		.end	= 0x41700043,
		.flags	= IORESOURCE_MEM,
	},
	[1] = {
		.start	= IRQ_SSP2,
		.end	= IRQ_SSP2,
		.flags	= IORESOURCE_IRQ,
	},
	[2] = {
		/* DRCMR for RX */
		.start	= 15,
		.end	= 15,
		.flags	= IORESOURCE_DMA,
	},
	[3] = {
		/* DRCMR for TX */
		.start	= 16,
		.end	= 16,
		.flags	= IORESOURCE_DMA,
	},
};

struct platform_device pxa27x_device_ssp2 = {
	.name		= "pxa27x-ssp",
	.id		= 1,
	.dev		= {
		.dma_mask = &pxa27x_ssp2_dma_mask,
		.coherent_dma_mask = DMA_BIT_MASK(32),
	},
	.resource	= pxa27x_resource_ssp2,
	.num_resources	= ARRAY_SIZE(pxa27x_resource_ssp2),
};

static u64 pxa27x_ssp3_dma_mask = DMA_BIT_MASK(32);

static struct resource pxa27x_resource_ssp3[] = {
	[0] = {
		.start	= 0x41900000,
		.end	= 0x41900043,
		.flags	= IORESOURCE_MEM,
	},
	[1] = {
		.start	= IRQ_SSP3,
		.end	= IRQ_SSP3,
		.flags	= IORESOURCE_IRQ,
	},
	[2] = {
		/* DRCMR for RX */
		.start	= 66,
		.end	= 66,
		.flags	= IORESOURCE_DMA,
	},
	[3] = {
		/* DRCMR for TX */
		.start	= 67,
		.end	= 67,
		.flags	= IORESOURCE_DMA,
	},
};

struct platform_device pxa27x_device_ssp3 = {
	.name		= "pxa27x-ssp",
	.id		= 2,
	.dev		= {
		.dma_mask = &pxa27x_ssp3_dma_mask,
		.coherent_dma_mask = DMA_BIT_MASK(32),
	},
	.resource	= pxa27x_resource_ssp3,
	.num_resources	= ARRAY_SIZE(pxa27x_resource_ssp3),
};

static struct resource pxa27x_resource_pwm0[] = {
	[0] = {
		.start	= 0x40b00000,
		.end	= 0x40b00008,
		.flags	= IORESOURCE_MEM,
	},
};

struct platform_device pxa27x_device_pwm0 = {
	.name		= "pxa27x-pwm",
	.id		= 0,
	.resource	= pxa27x_resource_pwm0,
	.num_resources	= ARRAY_SIZE(pxa27x_resource_pwm0),
};

static struct resource pxa27x_resource_pwm1[] = {
	[0] = {
		.start	= 0x40c00000,
		.end	= 0x40c00008,
		.flags	= IORESOURCE_MEM,
	},
};

struct platform_device pxa27x_device_pwm1 = {
	.name		= "pxa27x-pwm",
	.id		= 1,
	.resource	= pxa27x_resource_pwm1,
	.num_resources	= ARRAY_SIZE(pxa27x_resource_pwm1),
};

#if defined(CONFIG_PXA95x)
static struct resource pxa95x_resource_pwm4[] = {
	[0] = {
		.start	= 0x42404020,
		.end	= 0x42404028,
		.flags	= IORESOURCE_MEM,
	},
};

struct platform_device pxa95x_device_pwm4 = {
	.name		= "pxa95x-pwm",
	.id		= 4,
	.resource	= pxa95x_resource_pwm4,
	.num_resources	= ARRAY_SIZE(pxa95x_resource_pwm4),
};

static struct resource pxa95x_resource_pwm5[] = {
	[0] = {
		.start	= 0x42404030,
		.end	= 0x42404038,
		.flags	= IORESOURCE_MEM,
	},
};

struct platform_device pxa95x_device_pwm5 = {
	.name		= "pxa95x-pwm",
	.id		= 5,
	.resource	= pxa95x_resource_pwm5,
	.num_resources	= ARRAY_SIZE(pxa95x_resource_pwm5),
};

static struct resource pxa95x_resource_pwm6[] = {
	[0] = {
		.start	= 0x42404040,
		.end	= 0x42404048,
		.flags	= IORESOURCE_MEM,
	},
};

struct platform_device pxa95x_device_pwm6 = {
	.name		= "pxa95x-pwm",
	.id		= 6,
	.resource	= pxa95x_resource_pwm6,
	.num_resources	= ARRAY_SIZE(pxa95x_resource_pwm6),
};

static struct resource pxa95x_resource_pwm7[] = {
	[0] = {
		.start	= 0x42404050,
		.end	= 0x42404058,
		.flags	= IORESOURCE_MEM,
	},
};

struct platform_device pxa95x_device_pwm7 = {
	.name		= "pxa95x-pwm",
	.id		= 7,
	.resource	= pxa95x_resource_pwm7,
	.num_resources	= ARRAY_SIZE(pxa95x_resource_pwm7),
};

#endif

#endif /* CONFIG_PXA27x || CONFIG_PXA3xx || CONFIG_PXA93x */

#if defined(CONFIG_PXA9XX_ACIPC)
static u64 pxa930_acipc_dma_mask = DMA_BIT_MASK(32);

static struct resource pxa930_resource_acipc[] = {
	[0] = {
		.start	= 0x42403000,
		.end	= 0x424030ff,
		.flags	= IORESOURCE_MEM,
	},
	[1] = {
		.start	= IRQ_ACIPC0,
		.end	= IRQ_ACIPC0,
		.flags	= IORESOURCE_IRQ,
	},
	[2] = {
		.start	= IRQ_ACIPC1,
		.end	= IRQ_ACIPC1,
		.flags	= IORESOURCE_IRQ,
	},
	[3] = {
		.start	= IRQ_ACIPC2,
		.end	= IRQ_ACIPC2,
		.flags	= IORESOURCE_IRQ,
	},
};

struct platform_device pxa930_acipc_device = {
	.name		= "pxa9xx-acipc",
	.id		= -1,
	.dev		= {
		.dma_mask = &pxa930_acipc_dma_mask,
		.coherent_dma_mask = DMA_BIT_MASK(32),
	},
	.resource	= pxa930_resource_acipc,
	.num_resources	= ARRAY_SIZE(pxa930_resource_acipc),
};
#endif /*CONFIG_PXA9XX_ACIPC*/

#if defined(CONFIG_PXA3xx) || defined(CONFIG_PXA93x)
static struct resource pxa3xx_resources_mci2[] = {
	[0] = {
		.start	= 0x42000000,
		.end	= 0x42000fff,
		.flags	= IORESOURCE_MEM,
	},
	[1] = {
		.start	= IRQ_MMC2,
		.end	= IRQ_MMC2,
		.flags	= IORESOURCE_IRQ,
	},
	[2] = {
		.start	= 93,
		.end	= 93,
		.flags	= IORESOURCE_DMA,
	},
	[3] = {
		.start	= 94,
		.end	= 94,
		.flags	= IORESOURCE_DMA,
	},
};

struct platform_device pxa3xx_device_mci2 = {
	.name		= "pxa2xx-mci",
	.id		= 1,
	.dev		= {
		.dma_mask = &pxamci_dmamask,
		.coherent_dma_mask =	0xffffffff,
	},
	.num_resources	= ARRAY_SIZE(pxa3xx_resources_mci2),
	.resource	= pxa3xx_resources_mci2,
};

void __init pxa3xx_set_mci2_info(struct pxamci_platform_data *info)
{
	pxa_register_device(&pxa3xx_device_mci2, info);
}

static struct resource pxa3xx_resources_mci3[] = {
	[0] = {
		.start	= 0x42500000,
		.end	= 0x42500fff,
		.flags	= IORESOURCE_MEM,
	},
	[1] = {
		.start	= IRQ_MMC3,
		.end	= IRQ_MMC3,
		.flags	= IORESOURCE_IRQ,
	},
	[2] = {
		.start	= 100,
		.end	= 100,
		.flags	= IORESOURCE_DMA,
	},
	[3] = {
		.start	= 101,
		.end	= 101,
		.flags	= IORESOURCE_DMA,
	},
};

struct platform_device pxa3xx_device_mci3 = {
	.name		= "pxa2xx-mci",
	.id		= 2,
	.dev		= {
		.dma_mask = &pxamci_dmamask,
		.coherent_dma_mask = 0xffffffff,
	},
	.num_resources	= ARRAY_SIZE(pxa3xx_resources_mci3),
	.resource	= pxa3xx_resources_mci3,
};

void __init pxa3xx_set_mci3_info(struct pxamci_platform_data *info)
{
	pxa_register_device(&pxa3xx_device_mci3, info);
}

static struct resource pxa3xx_resources_gcu[] = {
	{
		.start	= 0x54000000,
		.end	= 0x54000fff,
		.flags	= IORESOURCE_MEM,
	},
	{
		.start	= IRQ_GCU,
		.end	= IRQ_GCU,
		.flags	= IORESOURCE_IRQ,
	},
};

static u64 pxa3xx_gcu_dmamask = DMA_BIT_MASK(32);

struct platform_device pxa3xx_device_gcu = {
	.name		= "pxa3xx-gcu",
	.id		= -1,
	.num_resources	= ARRAY_SIZE(pxa3xx_resources_gcu),
	.resource	= pxa3xx_resources_gcu,
	.dev		= {
		.dma_mask = &pxa3xx_gcu_dmamask,
		.coherent_dma_mask = 0xffffffff,
	},
};
#endif /* CONFIG_PXA3xx || CONFIG_PXA93x */

#if defined(CONFIG_PXA3xx) || defined(CONFIG_PXA93x) || defined(CONFIG_PXA95x)
static struct resource pxa3xx_resources_i2c_power[] = {
	{
		.start  = 0x40f500c0,
		.end    = 0x40f500d3,
		.flags	= IORESOURCE_MEM,
	}, {
		.start	= IRQ_PWRI2C,
		.end	= IRQ_PWRI2C,
		.flags	= IORESOURCE_IRQ,
	},
};

struct platform_device pxa3xx_device_i2c_power = {
	.name		= "pxa3xx-pwri2c",
	.id		= 1,
	.resource	= pxa3xx_resources_i2c_power,
	.num_resources	= ARRAY_SIZE(pxa3xx_resources_i2c_power),
};

static u64 pxa3xx_ssp4_dma_mask = DMA_BIT_MASK(32);

static struct resource pxa3xx_resource_ssp4[] = {
	[0] = {
		.start	= 0x41a00000,
		.end	= 0x41a00093,
		.flags	= IORESOURCE_MEM,
	},
	[1] = {
		.start	= IRQ_SSP4,
		.end	= IRQ_SSP4,
		.flags	= IORESOURCE_IRQ,
	},
	[2] = {
		/* DRCMR for RX */
		.start	= 2,
		.end	= 2,
		.flags	= IORESOURCE_DMA,
	},
	[3] = {
		/* DRCMR for TX */
		.start	= 3,
		.end	= 3,
		.flags	= IORESOURCE_DMA,
	},
};

struct platform_device pxa3xx_device_ssp4 = {
	/* PXA3xx SSP is basically equivalent to PXA27x */
	.name		= "pxa27x-ssp",
	.id		= 3,
	.dev		= {
		.dma_mask = &pxa3xx_ssp4_dma_mask,
		.coherent_dma_mask = DMA_BIT_MASK(32),
	},
	.resource	= pxa3xx_resource_ssp4,
	.num_resources	= ARRAY_SIZE(pxa3xx_resource_ssp4),
};

static struct resource pxa3xx_resources_nand[] = {
	[0] = {
		.start	= 0x43100000,
		.end	= 0x43100053,
		.flags	= IORESOURCE_MEM,
	},
	[1] = {
		.start	= IRQ_NAND,
		.end	= IRQ_NAND,
		.flags	= IORESOURCE_IRQ,
	},
	[2] = {
		/* DRCMR for Data DMA */
		.start	= 97,
		.end	= 97,
		.flags	= IORESOURCE_DMA,
	},
	[3] = {
		/* DRCMR for Command DMA */
		.start	= 99,
		.end	= 99,
		.flags	= IORESOURCE_DMA,
	},
};

static u64 pxa3xx_nand_dma_mask = DMA_BIT_MASK(32);

struct platform_device pxa3xx_device_nand = {
	.name		= "pxa3xx-nand",
	.id		= -1,
	.dev		= {
		.dma_mask = &pxa3xx_nand_dma_mask,
		.coherent_dma_mask = DMA_BIT_MASK(32),
	},
	.num_resources	= ARRAY_SIZE(pxa3xx_resources_nand),
	.resource	= pxa3xx_resources_nand,
};

void __init pxa3xx_set_nand_info(struct pxa3xx_nand_platform_data *info)
{
	pxa_register_device(&pxa3xx_device_nand, info);
}

/* pxa3xx onenand resource */
static u64 pxa3xx_onenand_dma_mask = DMA_BIT_MASK(32);

static struct resource pxa3xx_resources_onenand[] = {
	[0] = {
		.start  = 0x10000000,
		.end    = 0x100fffff,
		.flags  = IORESOURCE_MEM,
	},
};

struct platform_device pxa3xx_device_onenand = {
	.name		= "pxa3xx-onenand",
	.id		= -1,
	.dev		=  {
		.dma_mask	= &pxa3xx_onenand_dma_mask,
		.coherent_dma_mask = DMA_BIT_MASK(32),
	},
	.resource	= pxa3xx_resources_onenand,
	.num_resources	= ARRAY_SIZE(pxa3xx_resources_onenand),
};

void __init pxa3xx_set_onenand_info(struct pxa3xx_onenand_platform_data *info)
{
	pxa_register_device(&pxa3xx_device_onenand, info);
}

#endif	/* CONFIG_PXA3xx || CONFIG_PXA93x || CONFIG_PXA95x */

/* pxa2xx-spi platform-device ID equals respective SSP platform-device ID + 1.
 * See comment in arch/arm/mach-pxa/ssp.c::ssp_probe() */
void __init pxa2xx_set_spi_info(unsigned id, struct pxa2xx_spi_master *info)
{
	struct platform_device *pd;

	pd = platform_device_alloc("pxa2xx-spi", id);
	if (pd == NULL) {
		printk(KERN_ERR "pxa2xx-spi: failed to allocate device id %d\n",
		       id);
		return;
	}

	pd->dev.platform_data = info;
	platform_device_add(pd);
}

static struct resource pxa9xx_u2o_resources[] = {
	/* reg base */
	[0] = {
		.start	= PXA935_U2O_REGBASE,
		.end	= PXA935_U2O_REGBASE + USB_REG_RANGE,
		.flags	= IORESOURCE_MEM,
		.name	= "u2o",
	},
	/* phybase */
	[1] = {
		.start	= PXA935_U2O_PHYBASE,
		.end	= PXA935_U2O_PHYBASE + USB_PHY_RANGE,
		.flags	= IORESOURCE_MEM,
		.name	= "u2ophy",
	},
	[2] = {
		.start	= IRQ_U2O,
		.end	= IRQ_U2O,
		.flags	= IORESOURCE_IRQ,
	},
};

#if defined (CONFIG_USB) || defined (CONFIG_USB_GADGET)
/********************************************************************
 * The registers read/write routines
 ********************************************************************/

static u64 u2o_dma_mask = DMA_BIT_MASK(32);

struct platform_device pxa9xx_device_u2o = {
	.name		= "pxa-u2o",
	.id		= -1,
	.dev		= {
		.dma_mask	= &u2o_dma_mask,
		.coherent_dma_mask	= DMA_BIT_MASK(32),
	},

	.num_resources	= ARRAY_SIZE(pxa9xx_u2o_resources),
	.resource	= pxa9xx_u2o_resources,
};

unsigned u2o_get(unsigned base, unsigned offset)
{
        return readl(base + offset);
}

void u2o_set(unsigned base, unsigned offset, unsigned value)
{
        volatile unsigned int reg;

        reg = readl(base + offset);
        reg |= value;
        writel(reg, base + offset);
        __raw_readl(base + offset);

}

void u2o_clear(unsigned base, unsigned offset, unsigned value)
{
        volatile unsigned int reg;

        reg = readl(base + offset);
        reg &= ~value;
        writel(reg, base + offset);
        __raw_readl(base + offset);
}

void u2o_write(unsigned base, unsigned offset, unsigned value)
{
        writel(value, base + offset);
        __raw_readl(base + offset);

}

int pxa9xx_usb_phy_init(unsigned int base)
{
	/* linux kernel is not allowed to override usb phy settings */
	/* these settings configured only by obm (or bootrom)       */
	static int init_done;
	unsigned int ulTempAccr1;
	unsigned int ulTempCkenC;

	if (init_done)
		printk(KERN_DEBUG "re-init phy\n");

	ulTempAccr1 = ACCR1;
	ulTempCkenC = CKENC;

	ACCR1 |= (1<<10);
	CKENC |= (1<<10);

	/* Not safe. Sync risk */
	if ((ulTempAccr1 & (1<<10)) == 0)
		ACCR1 &= ~(1<<10);

	if ((ulTempCkenC & (1<<10)) == 0)
		CKENC &= ~(1<<10);

	/* override usb phy setting to mach values set by obm */
	u2o_write(base, U2PPLL, 0xfe819eeb);
	u2o_write(base, U2PTX, 0x41c10fc4);
	u2o_write(base, U2PRX, 0xe31d02e9);
	u2o_write(base, U2IVREF, 0x2000017e);

	if (cpu_is_pxa95x())
		u2o_write(base, U2PRS, 0x00008000);

        init_done = 1;
	return 0;
}
#endif

#if defined(CONFIG_USB_PXA_U2O) && defined(CONFIG_USB_OTG)
static struct resource pxa9xx_u2ootg_resources[] = {
	/* reg base */
	[0] = {
		.start	= PXA935_U2O_REGBASE,
		.end	= PXA935_U2O_REGBASE + USB_REG_RANGE,
		.flags	= IORESOURCE_MEM,
		.name	= "u2o",
	},
	/* phybase */
	[1] = {
		.start	= PXA935_U2O_PHYBASE,
		.end	= PXA935_U2O_PHYBASE + USB_PHY_RANGE,
		.flags	= IORESOURCE_MEM,
		.name	= "u2ophy",
	},
	[2] = {
		.start	= IRQ_U2O,
		.end	= IRQ_U2O,
		.flags	= IORESOURCE_IRQ,
	},
};

struct platform_device pxa9xx_device_u2ootg = {
	.name		= "pxa-otg",
	.id		= -1,
	.dev		= {
		.coherent_dma_mask	= DMA_BIT_MASK(32),
	},

	.num_resources	= ARRAY_SIZE(pxa9xx_u2ootg_resources),
	.resource	= pxa9xx_u2ootg_resources,
};

static struct resource pxa9xx_u2oehci_resources[] = {
	/* reg base */
	[0] = {
		.start	= PXA935_U2O_REGBASE,
		.end	= PXA935_U2O_REGBASE + USB_REG_RANGE,
		.flags	= IORESOURCE_MEM,
		.name	= "u2o",
	},
	/* phybase */
	[1] = {
		.start	= PXA935_U2O_PHYBASE,
		.end	= PXA935_U2O_PHYBASE + USB_PHY_RANGE,
		.flags	= IORESOURCE_MEM,
		.name	= "u2ophy",
	},
	[2] = {
		.start	= IRQ_U2O,
		.end	= IRQ_U2O,
		.flags	= IORESOURCE_IRQ,
	},
};

static u64 ehci_hcd_pxa_dmamask = DMA_BIT_MASK(32);
static void ehci_hcd_pxa_device_release(struct device *dev)
{
        /* Keep this function empty. */
}

struct platform_device pxa9xx_device_u2oehci = {
        .name = "pxau2o-ehci",
        .id = -1,
        .dev            = {
		.dma_mask = &ehci_hcd_pxa_dmamask,
		.coherent_dma_mask	= DMA_BIT_MASK(32),
		.release = ehci_hcd_pxa_device_release,

        },

        .num_resources = ARRAY_SIZE(pxa9xx_u2oehci_resources),
        .resource      = pxa9xx_u2oehci_resources,
};
#endif

#if defined(CONFIG_PXA95x)

/*mci0,1,2,3 corresponds to 1,2,3,4 in spec*/
static struct resource pxa95x_resources_mci0[] = {
	[0] = {
		.start	= 0x55000000,
		.end	= 0x550fffff,
		.flags	= IORESOURCE_MEM,
	},
	[1] = {
		.start	= IRQ_PXA935_MMC0,
		.end	= IRQ_PXA935_MMC0,
		.flags	= IORESOURCE_IRQ,
	},
};

struct platform_device pxa95x_device_mci0 = {
	.name		= "sdhci-pxa",
	.id		= 0,
	.dev		= {
		.dma_mask = &pxamci_dmamask,
		.coherent_dma_mask = 0xffffffff,
	},
	.num_resources	= ARRAY_SIZE(pxa95x_resources_mci0),
	.resource	= pxa95x_resources_mci0,
};

static struct resource pxa95x_resources_mci1[] = {
	[0] = {
		.start	= 0x55100000,
		.end	= 0x551fffff,
		.flags	= IORESOURCE_MEM,
	},
	[1] = {
		.start	= IRQ_PXA935_MMC1,
		.end	= IRQ_PXA935_MMC1,
		.flags	= IORESOURCE_IRQ,
	},
};

struct platform_device pxa95x_device_mci1 = {
	.name		= "sdhci-pxa",
	.id		= 1,
	.dev		= {
		.dma_mask = &pxamci_dmamask,
		.coherent_dma_mask = 0xffffffff,
	},
	.num_resources	= ARRAY_SIZE(pxa95x_resources_mci1),
	.resource	= pxa95x_resources_mci1,
};

static struct resource pxa95x_resources_mci2[] = {
	[0] = {
		.start	= 0x55200000,
		.end	= 0x552fffff,
		.flags	= IORESOURCE_MEM,
	},
	[1] = {
		.start	= IRQ_PXA935_MMC2,
		.end	= IRQ_PXA935_MMC2,
		.flags	= IORESOURCE_IRQ,
	},
};

struct platform_device pxa95x_device_mci2 = {
	.name		= "sdhci-pxa",
	.id		= 2,
	.dev		= {
		.dma_mask = &pxamci_dmamask,
		.coherent_dma_mask = 0xffffffff,
	},
	.num_resources	= ARRAY_SIZE(pxa95x_resources_mci2),
	.resource	= pxa95x_resources_mci2,
};

static struct resource pxa95x_resources_mci3[] = {
	[0] = {
		.start	= 0x55300000,
		.end	= 0x553fffff,
		.flags	= IORESOURCE_MEM,
	},
	[1] = {
		.start	= IRQ_PXA95x_MMC3,
		.end	= IRQ_PXA95x_MMC3,
		.flags	= IORESOURCE_IRQ,
	},
};

struct platform_device pxa95x_device_mci3 = {
	.name		= "sdhci-pxa",
	.id		= 3,
	.dev		= {
		.dma_mask = &pxamci_dmamask,
		.coherent_dma_mask = 0xffffffff,
	},
	.num_resources	= ARRAY_SIZE(pxa95x_resources_mci3),
	.resource	= pxa95x_resources_mci3,
};

void __init pxa95x_set_mci_info(int id, void *info)
{
	struct platform_device *d = NULL;

	switch (id) {
	case 0: d = &pxa95x_device_mci0; break;
	case 1: d = &pxa95x_device_mci1; break;
	case 2: d = &pxa95x_device_mci2; break;
	case 3: d = &pxa95x_device_mci3; break;
	default:
		return;
	}
	pxa_register_device(d, info);
}

static u64 pxa95x_i2c1_dma_mask = DMA_BIT_MASK(32);
static struct resource pxa95x_resources_i2c1[] = {
	{
		.start	= 0x40301680,
		.end	= 0x403016e3,
		.flags	= IORESOURCE_MEM,
	}, {
		.start	= IRQ_I2C,
		.end	= IRQ_I2C,
		.flags	= IORESOURCE_IRQ,
	}, {
		/* DRCMR for RX */
		.start	= 43,
		.end	= 43,
		.flags	= IORESOURCE_DMA,
	}, {
		/* DRCMR for TX */
		.start	= 44,
		.end	= 44,
		.flags	= IORESOURCE_DMA,
	},
};

struct platform_device pxa95x_device_i2c1 = {
	.name		= "pxa95x-i2c",
	.id		= 0,
	.dev		= {
		.dma_mask = &pxa95x_i2c1_dma_mask,
		.coherent_dma_mask = DMA_BIT_MASK(32),
	},
	.resource	= pxa95x_resources_i2c1,
	.num_resources	= ARRAY_SIZE(pxa95x_resources_i2c1),
};

static u64 pxa95x_i2c2_dma_mask = DMA_BIT_MASK(32);

static struct resource pxa95x_resources_i2c2[] = {
	{
		.start	= 0x40401680,
		.end	= 0x404016e3,
		.flags	= IORESOURCE_MEM,
	}, {
		.start	= IRQ_I2C2,
		.end	= IRQ_I2C2,
		.flags	= IORESOURCE_IRQ,
	}, {
		/* DRCMR for RX */
		.start	= 41,
		.end	= 41,
		.flags	= IORESOURCE_DMA,
	}, {
		/* DRCMR for TX */
		.start	= 42,
		.end	= 42,
		.flags	= IORESOURCE_DMA,
	},
};

struct platform_device pxa95x_device_i2c2 = {
	.name		= "pxa95x-i2c",
	.id		= 1,
	.dev		= {
		.dma_mask = &pxa95x_i2c2_dma_mask,
		.coherent_dma_mask = DMA_BIT_MASK(32),
	},
	.resource	= pxa95x_resources_i2c2,
	.num_resources	= ARRAY_SIZE(pxa95x_resources_i2c2),
};

static u64 pxa95x_i2c3_dma_mask = DMA_BIT_MASK(32);

static struct resource pxa95x_resources_i2c3[] = {
	{
		.start	= 0x40801680,
		.end	= 0x408016e3,
		.flags	= IORESOURCE_MEM,
	}, {
		.start	= IRQ_I2C3,
		.end	= IRQ_I2C3,
		.flags	= IORESOURCE_IRQ,
	}, {
		/* DRCMR for RX */
		.start	= 39,
		.end	= 39,
		.flags	= IORESOURCE_DMA,
	}, {
		/* DRCMR for TX */
		.start	= 40,
		.end	= 40,
		.flags	= IORESOURCE_DMA,
	},
};

struct platform_device pxa95x_device_i2c3 = {
	.name		= "pxa95x-i2c",
	.id		= 2,
	.dev		= {
		.dma_mask = &pxa95x_i2c3_dma_mask,
		.coherent_dma_mask = DMA_BIT_MASK(32),
	},
	.resource	= pxa95x_resources_i2c3,
	.num_resources	= ARRAY_SIZE(pxa95x_resources_i2c3),
};


static struct resource pxa95xfb_resources[] = {
	[0] = {
		.start  = 0x44100000,
		.end    = 0x4410ffff,
		.flags  = IORESOURCE_MEM,
	},
	[1] = {
		.start  = IRQ_LCDGLOBAL,
		.end    = IRQ_LCDGLOBAL,
		.flags  = IORESOURCE_IRQ,
	},
	[2] = {
		.start  = IRQ_LCDPARALLEL,
		.end    = IRQ_LCDPARALLEL,
		.flags  = IORESOURCE_IRQ,
	},
	[3] = {
		.start	= IRQ_DSI0,
		.end	= IRQ_DSI0,
		.flags	= IORESOURCE_IRQ,
	},
	[4] = {
		.start	= IRQ_DSI1,
		.end	= IRQ_DSI1,
		.flags	= IORESOURCE_IRQ,
	},
	[5] = {/*todo: */
		.start	= IRQ_DSI0,
		.end	= IRQ_DSI0,
		.flags	= IORESOURCE_IRQ,
	},
};

static struct resource pxa95xfb_ovly_resources[] = {
	[0] = {
		.start  = 0x44100000,
		.end    = 0x4410ffff,
		.flags  = IORESOURCE_MEM,
	},
	[1] = {
		.start  = IRQ_LCDGLOBAL,
		.end    = IRQ_LCDGLOBAL,
		.flags  = IORESOURCE_IRQ,
	},
	[2] = {
		.start  = IRQ_LCDPARALLEL,
		.end    = IRQ_LCDPARALLEL,
		.flags  = IORESOURCE_IRQ,
	},
	[3] = {
		.start	= IRQ_DSI0,
		.end	= IRQ_DSI0,
		.flags	= IORESOURCE_IRQ,
	},
	[4] = {
		.start	= IRQ_DSI1,
		.end	= IRQ_DSI1,
		.flags	= IORESOURCE_IRQ,
	},
	[5] = {/*todo: */
		.start	= IRQ_DSI0,
		.end	= IRQ_DSI0,
		.flags	= IORESOURCE_IRQ,
	},
};


struct platform_device pxa95x_device_fb = {
	.name           = "pxa95x-fb",
	.id             = -1,
	.dev            = {
		.dma_mask       = &fb_dma_mask,
		.coherent_dma_mask = 0xffffffff,
	},
	.num_resources  = ARRAY_SIZE(pxa95xfb_resources),
	.resource       = pxa95xfb_resources,
};

struct platform_device pxa95x_device_fb_ovly[] = {
	[0] = {
		.name           = "pxa95xfb-ovly",
		.id             = 0,
		.dev            = {
			.dma_mask       = &fb_dma_mask,
			.coherent_dma_mask = 0xffffffff,
		},
		.num_resources  = ARRAY_SIZE(pxa95xfb_ovly_resources),
		.resource       = pxa95xfb_ovly_resources,
	},
	[1] = {
		.name           = "pxa95xfb-ovly",
		.id             = 1,
		.dev            = {
			.dma_mask       = &fb_dma_mask,
			.coherent_dma_mask = 0xffffffff,
		},
		.num_resources  = ARRAY_SIZE(pxa95xfb_ovly_resources),
		.resource       = pxa95xfb_ovly_resources,
	},

};

void __init set_pxa95x_fb_info(void *info)
{
	pxa_register_device(&pxa95x_device_fb, info);
}

void __init set_pxa95x_fb_ovly_info(void *info, int id)
{
	pxa_register_device(&pxa95x_device_fb_ovly[id], info);
}


void __init set_pxa95x_fb_parent(struct device *parent_dev)
{
	pxa95x_device_fb.dev.parent = parent_dev;
}

static struct resource pxa95x_vmeta_resources[3] = {
       [0] = {
               .start = 0x58400000,
               .end   = 0x587fffff,
               .flags = IORESOURCE_MEM,
       },
       [1] = {
               .start = IRQ_VMETA_FUNC,
               .end   = IRQ_VMETA_FUNC,
               .flags = IORESOURCE_IRQ,
       },
       [2] = {
               .start = IRQ_VMETA_BUS,
               .end   = IRQ_VMETA_BUS,
               .flags = IORESOURCE_IRQ,
       },
};

static u64 pxa95x_vmeta_dma_mask = DMA_BIT_MASK(32);
struct platform_device pxa95x_device_vmeta = {
       .name           = UIO_VMETA_NAME,
       .id             = 0,
       .dev            = {
               .dma_mask = &pxa95x_vmeta_dma_mask,
               .coherent_dma_mask = DMA_BIT_MASK(32),
       },
       .resource       = pxa95x_vmeta_resources,
       .num_resources  = ARRAY_SIZE(pxa95x_vmeta_resources),
};

static struct resource pxa95x_resource_csi[] = {
	[0] = {
		.start	= 0x50020000,
		.end	= 0x50020fff,
		.flags	= IORESOURCE_MEM,
	},
	[1] = {
		.start	= 71,
		.end	= 71,
		.flags	= IORESOURCE_IRQ,
	},

};

struct platform_device pxa95x_device_csi = {
	.name		= "pxa95x-camera-csi",
	.id		= 0,
	.resource	= pxa95x_resource_csi,
	.num_resources	= ARRAY_SIZE(pxa95x_resource_csi),
};

/* two cam, to be continued */
/* IRQ: CI0, 76; CI1, 77*/
static struct resource pxa95x_resource_cam0[] = {
	[0] = {
		.start	= 0x50000000,
		.end	= 0x5000ffff,
		.flags	= IORESOURCE_MEM,
	},
	[1] = {
		.start	= 76,
		.end	= 76,
		.flags	= IORESOURCE_IRQ,
	},
};
static struct resource pxa95x_resource_cam1[] = {
	[0] = {
		.start	= 0x50010000,
		.end	= 0x5001ffff,
		.flags	= IORESOURCE_MEM,
	},
	[1] = {
		.start	= 77,
		.end	= 77,
		.flags	= IORESOURCE_IRQ,
	},
};

static u64 cam_dma_mask = ~(u32)0;
struct platform_device pxa95x_device_cam0 = {
	.name		= "pxa95x-camera",
	.id		= 0,
	.dev		= {
		.dma_mask	= &cam_dma_mask,
		.coherent_dma_mask = 0xffffffff,
	},
	.resource	= pxa95x_resource_cam0,
	.num_resources	= ARRAY_SIZE(pxa95x_resource_cam0),
};
struct platform_device pxa95x_device_cam1 = {
	.name		= "pxa95x-camera",
	.id		= 1,
	.dev		= {
		.dma_mask	= &cam_dma_mask,
		.coherent_dma_mask = 0xffffffff,
	},
	.resource	= pxa95x_resource_cam1,
	.num_resources	= ARRAY_SIZE(pxa95x_resource_cam1),
};

void __init pxa95x_set_vmeta_info(void* info)
{
	pxa_register_device(&pxa95x_device_vmeta,info);
}

static struct resource pxa95x_resource_freq[] = {
	[0] = {
		.name   = "clkmgr_regs",
		.start  = 0x41340000,
		.end    = 0x41350003,
		.flags  = IORESOURCE_MEM,
	},
	[1] = {
		.name   = "spmu_regs",
		.start  = 0x40f50000,
		.end    = 0x40f50103,
		.flags  = IORESOURCE_MEM,
	},
	[2] = {
		.name   = "bpmu_regs",
		.start  = 0x40f40000,
		.end    = 0x40f4003b,
		.flags  = IORESOURCE_MEM,
	},
	[3] = {
		.name   = "dmc_regs",
		.start  = 0x48100000,
		.end    = 0x4810012f,
		.flags  = IORESOURCE_MEM,
	},
	[4] = {
		.name   = "smc_regs",
		.start  = 0x4a000000,
		.end    = 0x4a00008f,
		.flags  = IORESOURCE_MEM,
	}
};

struct platform_device pxa95x_device_freq = {
	.name           = "pxa95x-freq",
	.id             = 0,
	.num_resources  = ARRAY_SIZE(pxa95x_resource_freq),
	.resource       = pxa95x_resource_freq,
};

void __init set_pxa95x_freq_info(struct pxa95x_freq_mach_info *info)
{
	pxa_register_device(&pxa95x_device_freq, info);
}

void __init set_pxa95x_freq_parent(struct device *parent_dev)
{
	pxa95x_device_freq.dev.parent = parent_dev;
}

static struct resource pxa95x_pmu_resources[] = {
	[0] = {
		.name   = "pmu_regs",
		.start = 0x4600ff00,
		.end   = 0x4600ffff,
		.flags = IORESOURCE_MEM,
	},
};

struct platform_device pxa95x_device_pmu = {
	.name           = "pxa95x-pmu",
	.id             = 0,
	.resource       = pxa95x_pmu_resources,
	.num_resources  = ARRAY_SIZE(pxa95x_pmu_resources),
};

void __init pxa95x_set_pmu_info(void *info)
{
	pxa_register_device(&pxa95x_device_pmu, info);
}

#if defined(CONFIG_CPU_PXA970)
static struct resource pxa970_ihdmi_resources[] = {
	[0] = {
		.start = 0x44108000,
		.end   = 0x441083ff,
		.flags = IORESOURCE_MEM,
	},
};

static u64 pxa970_ihdmi_dma_mask = DMA_BIT_MASK(32);
struct platform_device pxa970_device_ihdmi = {
	.name		= "pxa970-ihdmi",
	.id		= -1,
	.dev		= {
		.dma_mask = &pxa970_ihdmi_dma_mask,
		.coherent_dma_mask = DMA_BIT_MASK(32),
	},
	.resource	= pxa970_ihdmi_resources,
	.num_resources	= ARRAY_SIZE(pxa970_ihdmi_resources),
};

void __init pxa970_set_ihdmi_info(void* info)
{
	pxa_register_device(&pxa970_device_ihdmi, info);
}

#endif
#endif
