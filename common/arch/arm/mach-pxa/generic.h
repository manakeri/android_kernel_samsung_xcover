/*
 *  linux/arch/arm/mach-pxa/generic.h
 *
 * Author:	Nicolas Pitre
 * Copyright:	MontaVista Software Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

struct sys_timer;

extern struct sys_timer pxa_timer;
extern void __init pxa_init_irq(int irq_nr,
				int (*set_wake)(unsigned int, unsigned int));
extern void __init pxa25x_init_irq(void);
#ifdef CONFIG_CPU_PXA26x
extern void __init pxa26x_init_irq(void);
#endif
extern void __init pxa27x_init_irq(void);
extern void __init pxa3xx_init_irq(void);
extern void __init pxa93x_init_irq(void);
extern void __init pxa95x_init_irq(void);
extern void __init pxa_map_io(void);

extern unsigned int get_clk_frequency_khz(int info);

typedef enum {
	OBM_EVB_P_BOARD,
	OBM_EVB_PV_BOARD,
	OBM_EVB_PV2_BOARD,
	OBM_SAAR_2_5_BOARD,
	OBM_SAAR_2_6_BOARD,
	OBM_SAAR_PV_3_1_BOARD,
	OBM_SAAR_PV_3_2_BOARD,
	OBM_SAAR_PV_3_4_BOARD,
	OBM_SAAR_PV_3_5_BOARD,
	OBM_YARDEN_BOARD,
	OBM_EVB_PV2_3_0_BOARD,
	OBM_EVB_PV2_3_1_BOARD,
	OBM_EVB_PV2_JIL_BOARD,
	OBM_EVB_PV2_JIL_15_BOARD,
	OBM_SAAR_B_PV2_BOARD,
	OBM_EVB_PV2_3_2_BOARD,
	OBM_EVB_MG1_3_3_BOARD,
	OBM_SAAR_B_PV2_B0_BOARD,
	OBM_SAAR_B_PV2_B0_V1_BOARD,
	OBM_SAAR_B_MG1_C0_V12_BOARD,
	OBM_EVB_MG2_4_0_BOARD,
	OBM_SAAR_B_MG2_A0_V13_BOARD,
	OBM_SAAR_C_MG2_B0_V10_BOARD,
	OBM_SAAR_B_MG2_A0_V14_BOARD,
	OBM_SAAR_B_MG2_B0_V15_BOARD,
	OBM_EVB_ESHEL_1_0_BOARD,		/*25*/
	OBM_EVB_ESHEL_1_0_MCP_BOARD,
	OBM_EVB_ESHEL_SAGIE_BOARD,
	OBM_EVB_NEVO_1_0_BOARD,
	OBM_EVB_NEVO_1_1_BOARD,
	OBM_EVB_ESHEL_LTE_1_0_BOARD,		/*30*/
	OBM_SAAR_C2_NEVO_A0_V10_BOARD,
	OBM_SAAR_B_MG2_C0_V26_BOARD,
	OBM_EVB_MG2_4_1_BOARD,
	OBM_EVB_ESHEL_1_1_BOARD,
	OBM_SAAR_B_MG2_C0_V275_BOARD,		/*35*/
	MAX_BOARD_TYPE
} BOARD_ID_TYPE;
long get_board_id(void);

/* Flash Types */
enum {
	CS0_XIP_FLASH	= 1,
	CS2_XIP_FLASH	= 2,
	NAND_FLASH	= 3,
	ONENAND_FLASH	= 4,
	MSYS_DOC_FLASH	= 5,
	SDMMC_FLASH	= 6,
	SPI_FLASH	= 7
};

extern int is_uart_gpio(void);

#define SET_BANK(__nr,__start,__size) \
	mi->bank[__nr].start = (__start), \
	mi->bank[__nr].size = (__size), \
	mi->bank[__nr].node = (((unsigned)(__start) - PHYS_OFFSET) >> 27)

#define ARRAY_AND_SIZE(x)	(x), ARRAY_SIZE(x)

#ifdef CONFIG_PXA25x
extern unsigned pxa25x_get_clk_frequency_khz(int);
extern unsigned pxa25x_get_memclk_frequency_10khz(void);
#else
#define pxa25x_get_clk_frequency_khz(x)		(0)
#define pxa25x_get_memclk_frequency_10khz()	(0)
#endif

#ifdef CONFIG_PXA27x
extern unsigned pxa27x_get_clk_frequency_khz(int);
extern unsigned pxa27x_get_memclk_frequency_10khz(void);
#else
#define pxa27x_get_clk_frequency_khz(x)		(0)
#define pxa27x_get_memclk_frequency_10khz()	(0)
#endif

#if defined(CONFIG_PXA25x) || defined(CONFIG_PXA27x)
extern void pxa2xx_clear_reset_status(unsigned int);
#else
static inline void pxa2xx_clear_reset_status(unsigned int mask) {}
#endif

#ifdef CONFIG_PXA3xx
extern unsigned pxa3xx_get_clk_frequency_khz(int);
extern void pxa3xx_clear_reset_status(unsigned int);
#else
#define pxa3xx_get_clk_frequency_khz(x)		(0)
static inline void pxa3xx_clear_reset_status(unsigned int mask) {}
#endif

#ifdef CONFIG_PXA93x
extern void pxa93x_clear_reset_status(unsigned int);
#else
static inline void pxa93x_clear_reset_status(unsigned int mask) {}
#endif

#ifdef CONFIG_PXA95x
extern void pxa95x_clear_reset_status(unsigned int);
#else
static inline void pxa95x_clear_reset_status(unsigned int mask) {}
#endif

extern struct sysdev_class pxa_irq_sysclass;
extern struct sysdev_class pxa_gpio_sysclass;
extern struct sysdev_class pxa2xx_mfp_sysclass;
extern struct sysdev_class pxa3xx_mfp_sysclass;

void __init pxa_set_ffuart_info(void *info);
void __init pxa_set_btuart_info(void *info);
void __init pxa_set_stuart_info(void *info);
void __init pxa_set_hwuart_info(void *info);

#ifdef CONFIG_MTD_ONENAND_PXA3xx
#include <plat/pxa3xx_onenand.h>
extern void __init pxa3xx_set_onenand_info(struct pxa3xx_onenand_platform_data *info);
void onenand_init(int sync_enable);
#endif

#define WAKE_LOCK_CD_TIMEOUT (15*HZ)

void nand_init(void);
void pxa_boot_flash_init(int sync_mode);
