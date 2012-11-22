#ifndef __MACH_PXA688_DMA_H
#define __MACH_PXA688_DMA_H

#include <mach/addr-map.h>

#define __PXA688_DMA_REG(x, y)	(*((volatile u32 *)(AXI_VIRT_BASE + (x) + (y))))
#define __PXA688_ADMA_REG(x, y)	(*((volatile u32 *)(AUD_VIRT_BASE + (x) + (y))))

/* ADMA */
#define PXA688_ADSAR(base)			__PXA688_ADMA_REG(base, 0x10)
#define PXA688_ADDAR(base)			__PXA688_ADMA_REG(base, 0x20)
#define PXA688_ADNDPR(base)			__PXA688_ADMA_REG(base, 0x30)
#define PXA688_ADCR(base)			__PXA688_ADMA_REG(base, 0x40)
#define PXA688_ADCP(base)			__PXA688_ADMA_REG(base, 0x60)
#define PXA688_ADCDPR(base)			__PXA688_ADMA_REG(base, 0x70)
#define PXA688_ADIMR(base)			__PXA688_ADMA_REG(base, 0x80)
#define PXA688_ADISR(base)			__PXA688_ADMA_REG(base, 0xa0)

#define ADMA1_CH0_BASE		0x800
#define ADMA1_CH1_BASE		0x804
#define ADMA2_CH0_BASE		0x900
#define ADMA2_CH1_BASE		0x904
#define MDMA_CH0_BASE		0xA0a00
#define MDMA_CH1_BASE		0xA0a04
#define VDMA_CH0_BASE		0x0B300
#define VDMA_CH1_BASE		0x0B380

#define PXA688_DSAR(base)			__PXA688_DMA_REG(base, 0x10)
#define PXA688_DDAR(base)			__PXA688_DMA_REG(base, 0x20)
#define PXA688_DNDPR(base)			__PXA688_DMA_REG(base, 0x30)
#define PXA688_DCR(base)			__PXA688_DMA_REG(base, 0x40)
#define PXA688_DCP(base)			__PXA688_DMA_REG(base, 0x60)
#define PXA688_DCDPR(base)			__PXA688_DMA_REG(base, 0x70)
#define PXA688_DIMR(base)			__PXA688_DMA_REG(base, 0x80)
#define PXA688_DISR(base)			__PXA688_DMA_REG(base, 0xa0)
#define PXA688_VDCR(base)			__PXA688_DMA_REG(base, 0x28)

/*mapping according to ICU_DMA_IRQ1[16:23]*/
typedef enum {
	MDMA_CH_0 = 0,
	MDMA_CH_1,
	ADMA1_CH_0,
	ADMA1_CH_1,
	ADMA2_CH_0,
	ADMA2_CH_1,
	VDMA_CH_0,
	VDMA_CH_1,
	DMA_CH_NUM,
} pxa688_dma_channel_mapping;

/*
 * Descriptor structure for PXA688 ADMA and MDMA engine
 * Note: this structure must always be aligned to a 16-byte boundary.
 */
typedef struct pxa688_dma_desc {
	volatile u32 byte_cnt;	/* byte count */
	volatile u32 src_addr;	/* source address */
	volatile u32 dst_addr;	/* target address */
	volatile u32 nxt_desc;	/* next descriptor dress */
} pxa688_dma_desc;

int __init pxa688_init_dma(void);

int pxa688_request_dma(char *name,
		       pxa688_dma_channel_mapping dma_ch,
		       void (*irq_handler) (int, void *), void *data);

void pxa688_free_dma(int dma_ch);

u32 pxa688_find_dma_register_base(int dma_ch);

#endif /* __MACH_PXA688_DMA_H */
