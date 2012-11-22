#ifndef __MACH_SQU_H
#define __MACH_SQU_H

#include <mach/addr-map.h>

#define PXA910_SQU_REGS_VIRT	(AXI_VIRT_BASE + 0xA0000)
#define PXA910_SQU_REG(x)	(*((volatile u32 *)(PXA910_SQU_REGS_VIRT + (x))))
#define SQU_REG(x)		(PXA910_SQU_REGS_VIRT + (x))

/*SQU*/
#define SQU_CTRL_0			SQU_REG(0x000)
#define SQU_CTRL_1			SQU_REG(0x008)
#define SQU_CTRL_2			SQU_REG(0x030)
#define SQU_FMBIST_CTRL_0		SQU_REG(0x010)
#define SQU_FMBIST_CTRL_1		SQU_REG(0x018)
#define SQU_FMBIST_STATUS_0		SQU_REG(0x020)
#define SQU_RSVD			SQU_REG(0x028)
#define SQU_PERF_COUNT_CNTRL		SQU_REG(0x040)
#define SQU_PERF_COUNT_S1		SQU_REG(0x048)
#define SQU_PERF_COUNT_S4		SQU_REG(0x050)
#define SQU_PERF_COUNT_S8		SQU_REG(0x058)

#define SQU_CAM_ENT_BANK0		SQU_REG(0x200)
#define SQU_CAM_ENT_BANK1		SQU_REG(0x280)
#define SQU_CAM_ENT_BANK2		SQU_REG(0x300)
#define SQU_CAM_ENT_BANK3		SQU_REG(0x380)
#define SQU_CAM_ENT_BANK4		SQU_REG(0x400)
#define SQU_CAM_ENT_BANK5		SQU_REG(0x480)
#define SQU_LOGGER_ENT			SQU_REG(0x700)

#define SQU_CHAN_0_BYTE_CNT		SQU_REG(0x800)
#define SQU_CHAN_1_BYTE_CNT		SQU_REG(0x804)
#define SQU_CHAN_0_SRC_ADDR		SQU_REG(0x810)
#define SQU_CHAN_1_SRC_ADDR		SQU_REG(0x814)
#define SQU_CHAN_0_DEST_ADDR		SQU_REG(0x820)
#define SQU_CHAN_1_DEST_ADDR		SQU_REG(0x824)
#define SQU_CHAN_0_NEXT_DESC_PTR	SQU_REG(0x830)
#define SQU_CHAN_1_NEXT_DESC_PTR	SQU_REG(0x834)
#define SQU_CHAN_0_CTRL			SQU_REG(0x840)
#define SQU_CHAN_1_CTRL			SQU_REG(0x844)
#define SQU_CHAN_PRI			SQU_REG(0x860)
#define SQU_CHAN_0_CURR_DESC_PTR	SQU_REG(0x870)
#define SQU_CHAN_1_CURR_DESC_PTR	SQU_REG(0x874)
#define SQU_CHAN_0_INT_MASK		SQU_REG(0x880)
#define SQU_CHAN_1_INT_MASK		SQU_REG(0x884)
#define SQU_CHAN_0_INT_RST_SEL		SQU_REG(0x890)
#define SQU_CHAN_1_INT_RST_SEL		SQU_REG(0x894)
#define SQU_CHAN_0_INT_STATUS		SQU_REG(0x8a0)
#define SQU_CHAN_1_INT_STATUS		SQU_REG(0x8a4)

#define SDSAR(x)                PXA910_SQU_REG(0x810 + ((x) << 2))
#define SDDAR(x)                PXA910_SQU_REG(0x820 + ((x) << 2))
#define SDNDPR(x)               PXA910_SQU_REG(0x830 + ((x) << 2))
#define SDCR(x)                 PXA910_SQU_REG(0x840 + ((x) << 2))
#define SDCDPR(x)               PXA910_SQU_REG(0x870 + ((x) << 2))
#define SDIMR(x)                PXA910_SQU_REG(0x880 + ((x) << 2))
#define SDISR(x)                PXA910_SQU_REG(0x8a0 + ((x) << 2))

#define SDCR_SSPMOD             (1 << 21) /* SSPMod */
#define SDCR_ABR                (1 << 20) /* Channel Abort */
#define SDCR_CDE                (1 << 17)
#define SDCR_PACKMOD            (1 << 16)
#define SDCR_SDA                (1 << 15)
#define SDCR_CHANACT            (1 << 14) /* DMA Channel Active */
#define SDCR_FETCHND            (1 << 13)
#define SDCR_CHANEN             (1 << 12) /* Channel Enable */
#define SDCR_TRANSMOD           (1 << 11) /* TransMod */
#define SDCR_INTMODE            (1 << 10) /* Interrupt Mode */
#define SDCR_CHAINMOD           (1 << 9) /* Chain Mode */
#define SDCR_BURSTLIMIT_MSK     (0x7 << 6)
#define SDCR_DESTDIR_MSK        (0x3 << 4)
#define SDCR_SRCDIR_MSK         (0x3 << 2) /* Source Direction */
#define SDCR_DESTDESCCONT       (1 << 1)
#define SDCR_SRCDESTCONT        (1 << 0)
#define SDCR_DST_ADDR_INC       (0 << 4)
#define SDCR_DST_ADDR_HOLD      (0x2 << 4)
#define SDCR_SRC_ADDR_INC       (0 << 2)
#define SDCR_SRC_ADDR_HOLD      (0x2 << 2)
#define SDCR_DMA_BURST_4B       (0x0 << 6)
#define SDCR_DMA_BURST_8B       (0x1 << 6)
#define SDCR_DMA_BURST_16B      (0x3 << 6)
#define SDCR_DMA_BURST_1B       (0x5 << 6)
#define SDCR_DMA_BURST_2B       (0x6 << 6)

#define SDCR_DMA_BURST_32B      (0x7 << 6)
#define SDIMR_COMP              (1 << 0)


/*
 * Descriptor structure for PXA910's SQU engine
 * Note: this structure must always be aligned to a 16-byte boundary.
 */

typedef struct pxa910_squ_desc {
        volatile u32        byte_cnt;           // byte count
        volatile u32        src_addr;           // source address
        volatile u32        dst_addr;           // target address
        volatile u32        nxt_desc;          // next descriptor dress
} pxa910_squ_desc;


typedef enum {
	SQU_PRIO_HIGH = 0,
	SQU_PRIO_MEDIUM = 1,
	SQU_PRIO_LOW = 2
} pxa910_squ_prio;

/*
 * SQU registration
 */
int __init pxa910_init_squ(int num_ch);

int pxa910_request_squ (char *name,
			pxa910_squ_prio prio,
			void (*irq_handler)(int, void *),
			void *data);

void pxa910_free_squ (int squ_ch);


#endif /* __MACH_SQU_H */
