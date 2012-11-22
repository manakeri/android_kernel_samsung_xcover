/*
 *  i2c_adap_pxa.c
 *
 *  I2C adapter for the PXA I2C bus access.
 *
 *  Copyright (C) 2002 Intrinsyc Software Inc.
 *  Copyright (C) 2004-2005 Deep Blue Solutions Ltd.
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 *
 *  History:
 *    Apr 2002: Initial version [CS]
 *    Jun 2002: Properly seperated algo/adap [FB]
 *    Jan 2003: Fixed several bugs concerning interrupt handling [Kai-Uwe Bloem]
 *    Jan 2003: added limited signal handling [Kai-Uwe Bloem]
 *    Sep 2004: Major rework to ensure efficient bus handling [RMK]
 *    Dec 2004: Added support for PXA27x and slave device probing [Liam Girdwood]
 *    Feb 2005: Rework slave mode handling [RMK]
 *    Jul 2009 - [APSE]     Initial version for Linux-2.6.28 platform EVB3 PV2
 *                        with introduction to multiple instances supporting
 *    Jul 2009 - [PTK]      Use i2c_pxa_reset() in case of an error on I2C bus
 *    Aug 2009 - [PTK YANM] Reconsidering with sync to latest SAAR-platform code
 *		- Strict and multiple bus busy checking by (IBMR_IDLE ISR_EBB | ISR_IBB | ISR_UB)
 *		- i2c_pxa_abort() and i2c_pxa_unit_restart() to recover from error
 *		- udelay(I2C_ACCESS2_DELAY_US/2) after CLOCK setting !
 *		- do_xfer_retry with xfer_retries=1
 *		- "ADDR Arbitration Loss with MasterSTOP" handling
 *		- Delete cpu_is_pxa3xx related code and cpu_is_pxa9 differentiation in most cases except
 *		    No dvfm "CG" (Clock Gating mode) for cpu_is_pxa930 only
 *          Add differentiation for the IWCR
 *		- Adjust delays/retries for block/paging (more than one byte) read/write
 *		- Use ILCR/IWCR to adjust SCLK close to maxmium frequency
 *		- Parsing data-msg for debug and customization
 *		- CUSTOMIZATION_PER_SLAVE_DEVICE:
 *           SLOW, i2c_pxa_custom_nack_is_ok(), CUSTOM_BYTE2BYTE_DELAY
 *		- CONFIG_PROC_FS: to control slow/fast, customization and DEBUG_ERR_SIMULATE
 *
 *  NOTE: High-Speed Master codes are in range 0x8..0xF where
 *   the 8 is higest priority winning in arbitration with other masters on bus
 *   and 0xF is lowest priority. APPS processor should NOT use higest code 0x8
 *   The recommended value in i2c_pxa_platform_data .master_code =(8|4|2)=0xE
 *
 *  Sept 2011 - [PTK YANM] Do not printk ARBITRATION-Loss or BusERROR directly
 *    under IRQ context but "delay" it to task-context (these are bus-Warnings).
 *    Still keep "spurious" prints since these are real problem/error.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/i2c-id.h>
#include <linux/init.h>
#include <linux/time.h>
#include <linux/sched.h>
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/interrupt.h>
#include <linux/i2c-pxa.h>
#include <linux/platform_device.h>
#include <linux/err.h>
#include <linux/clk.h>

#include <linux/proc_fs.h>
#include <asm/uaccess.h>

#include <mach/regs-ost.h>

#include <asm/irq.h>
#include <asm/io.h>
#include <plat/i2c.h>
#include <mach/dma.h>

#include <mach/pxa95x-regs.h>
#include <mach/dvfm.h>

/* Incorrect "Successful" request may cause BUS-ERROR failure for next request
* So save the request message in debug buffer; it will be printed as "previous"
* with failed "current" request.
*/
#define SAVE_MSG_REQUEST_DEBUG_BUF

/*
 * I2C register offsets will be shifted 0 or 1 bit left, depending on
 * different SoCs
 */
#define REG_SHIFT_0	(0 << 0)
#define REG_SHIFT_1	(1 << 0)
#define REG_SHIFT(d)	((d) & 0x1)

static struct platform_device_id i2c_pxa_id_table[] = {
	{ "pxa2xx-i2c",		REG_SHIFT_1 },
	{ "pxa3xx-pwri2c",	REG_SHIFT_0 },
	{ "pxa95x-i2c", 	REG_SHIFT_1 },
	{ },
};
MODULE_DEVICE_TABLE(platform, i2c_pxa_id_table);

/*
 * I2C registers and bit definitions
 */
#define IBMR		(0x00)
#define IDBR		(0x08)
#define ICR		(0x10)
#define ISR		(0x18)
#define ISAR		(0x20)
#define IWCR            (0x30)
#define WFIFO           (0x40)
#define WFIFO_WPTR      (0x44)
#define WFIFO_RPTR      (0x48)
#define RFIFO           (0x50)
#define RFIFO_WPTR      (0x54)
#define RFIFO_RPTR      (0x58)
#define FIFO_TSHLD      (0x60)

#define ICR_START	(1 << 0)	   /* start bit */
#define ICR_STOP	(1 << 1)	   /* stop bit */
#define ICR_ACKNAK	(1 << 2)	   /* send ACK(0) or NAK(1) */
#define ICR_TB		(1 << 3)	   /* transfer byte bit */
#define ICR_MA		(1 << 4)	   /* master abort */
#define ICR_SCLE	(1 << 5)	   /* master clock enable */
#define ICR_IUE		(1 << 6)	   /* unit enable */
#define ICR_GCD		(1 << 7)	   /* general call disable */
#define ICR_ITEIE	(1 << 8)	   /* enable tx interrupts */
#define ICR_IRFIE	(1 << 9)	   /* enable rx interrupts */
#define ICR_BEIE	(1 << 10)	   /* enable bus error ints */
#define ICR_SSDIE	(1 << 11)	   /* slave STOP detected int enable */
#define ICR_ALDIE	(1 << 12)	   /* enable arbitration interrupt */
#define ICR_SADIE	(1 << 13)	   /* slave address detected int enable */
#define ICR_UR		(1 << 14)	   /* unit reset */
#define ICR_FM		(1 << 15)	   /* fast mode */
#define ICR_HS		(1 << 16)	   /* Hig Speed mode with/without ICR_FM*/
#define ICR_MSDIE	(1 << 17)	   /* master stop detected int enable */
#define ICR_MSDE	(1 << 18)	   /* master stop detected enable */
#define ICR_GPIOEN	(1 << 19)	   /* to drive the SCL by I2C in HS mode */
#define ICR_FIFOEN      (1 << 20)  /* to enable FIFO mode */
#define ICR_TXBEGIN     (1 << 21)  /* transaction begins */
#define ICR_TXDONE_IE   (1 << 22)  /* transaction done interrupt enable */
#define ICR_RXSR_IE     (1 << 23)  /* receive fifo half full interrupt enable */
#define ICR_TXSR_IE     (1 << 24)  /* transmit fifo service request interrupt enable */
#define ICR_RXF_IE      (1 << 25)  /* receive fifo full interrupt enable */
#define ICR_RXOV_IE     (1 << 26)  /* receive fifo overrun interrupt enable */
#define ICR_DMA_EN      (1 << 27)  /* DMA enable for both TX and RX FIFOs */
#define ICR_RXUN_IE     (1 << 28)  /* receive FIFO underrun interrupt enable */
#define ICR_TXOV_IE     (1 << 29)  /* transmit FIFO overrun interrupt enable */
#define ICR_SPIKE_FIX	(1 << 30)  /* apply fix for HW issue "fault spike*/

#define ISR_RWM		(1 << 0)	   /* read/write mode */
#define ISR_ACKNAK	(1 << 1)	   /* ack/nak status */
#define ISR_UB		(1 << 2)	   /* unit busy */
#define ISR_IBB		(1 << 3)	   /* bus busy */
#define ISR_SSD		(1 << 4)	   /* slave stop detected */
#define ISR_ALD		(1 << 5)	   /* arbitration loss detected */
#define ISR_ITE		(1 << 6)	   /* tx buffer empty */
#define ISR_IRF		(1 << 7)	   /* rx buffer full */
#define ISR_GCAD	(1 << 8)	   /* general call address detected */
#define ISR_SAD		(1 << 9)	   /* slave address detected */
#define ISR_BED		(1 << 10)	   /* bus error no ACK/NAK */
#define ISR_EBB		(1 << 11)	   /* early bus busy */
#define ISR_MSD		(1 << 12)	   /* master stop detected */
#define ISR_TXDONE      (1 << 13)  /* transaction done */
#define ISR_RXSR        (1 << 14)  /* receive fifo service request */
#define ISR_TXSR        (1 << 15)  /* transmit fifo sercice request */
#define ISR_RXF         (1 << 16)  /* receive fifo full */
#define ISR_RXOV        (1 << 17)  /* receive fifo overrun */
#define ISR_RXUN        (1 << 18)  /* receive fifo underrun */
#define ISR_TXOV        (1 << 19)  /* transmit fifo overrun */

enum pxa_i2c_active_dma {
	DMA_W = 0x1,
	DMA_R = 0x2,
};


struct i2c_algo_custom {
	u8	slave_addr;
	u8	clock_adj;
	u8	byte2byte_delay;
	u8	scream_silent;
	u8	spy_reg[4];
};

#ifdef SAVE_MSG_REQUEST_DEBUG_BUF
struct i2c_msg_debug {
	u16 addr;
	u16 flags;
	u16 len;
	u16 gap;
	u8  buf[4];
};
#endif


typedef enum
{
	IRQ_WARN_NONE,
	IRQ_WARN_ALD,		/*"wrong ISR_ALD"*/
	IRQ_WARN_ALD_BED,	/*"wrong ISR_ALD & ISR_BED"*/
	IRQ_WARN_ALD_RETRY,	/*"INFO but not error\ni2c: Arbitration Loss Detected, go with RETRY"*/
	IRQ_WARN_ALD_ADDR,	/*"ERROR: ADDR Arbitration Loss with MasterSTOP"*/
	IRQ_WARN_ALD_CONT,	/*"INFO but not error\ni2c: Arbitration Loss Detected, but transaction continued"*/
	IRQ_WARN_NO_ANSWER,	/*"Bus ERROR or no answer from slave-device"*/
	IRQ_WARN_BUS_ERR,	/*"Bus ERROR on Data stage"*/
	IRQ_WARN_UNKNOWN
}irq_warn_enum;

static char* i2c_irq_warn_str[IRQ_WARN_UNKNOWN+1] = {
	NULL,
	"wrong ISR_ALD",
	"wrong ISR_ALD & ISR_BED",
	"INFO but not error\ni2c: Arbitration Loss Detected, go with RETRY",
	"ERROR: ADDR Arbitration Loss with MasterSTOP",
	"INFO but not error\ni2c: Arbitration Loss Detected, but transaction continued",
	"Bus ERROR or no answer from slave-device",
	"Bus ERROR on Data stage",
	"Undefined Error on IRQ context"
};

struct pxa_i2c {
	wait_queue_head_t	wait;
	struct i2c_msg		*msg;
	unsigned int		msg_num;
	int			msg_idx;
	unsigned int		msg_ptr;
	unsigned int		slave_addr;  /* This is "my Master's" but not remote address*/
	unsigned int		icr_backup;

	struct i2c_adapter	adap;
	struct clk		*clk;
#ifdef CONFIG_I2C_PXA_SLAVE
	struct i2c_slave_client *slave;
#endif

	unsigned int		irqlogidx;
	u32			isrlog[32];
	u32			icrlog[32];
	int			debug_track;
#ifdef SAVE_MSG_REQUEST_DEBUG_BUF
    int						debug_num_msgs;
	struct i2c_msg_debug	debug_msgs[2];
#endif

	void __iomem		*reg_base;
	unsigned int		reg_shift;

	unsigned long		iobase;
	unsigned long		iosize;

	int			irq;
	unsigned int		use_pio;
	int			flags;	/* specify i2c speed mode */

	int		access2_delay;
	int		busbusy_delay;

	/* Parse data-msg and Customization per remote slave request */
	u8		req_slave_addr;
	char	req_read;			/* 'R' read or 'W' write */
	u8		req_num_data_bytes;

	irq_warn_enum  irq_warn;

	struct i2c_algo_custom *p_custom;
	bool		hs_enterring;
	u8		master_code;
	struct dvfm_lock	dvfm_lock;
	bool	fifo_transaction_down;
	u32		fifo_write_byte_num;
	u32		fifo_read_byte_num;
	unsigned int		dma_chans[2];
	void			*read_buffer;
	void			*write_buffer;
	dma_addr_t		dma_handles_read;
	dma_addr_t		dma_handles_write;
	bool			have_read_operation;
	char            dbg_trace;
	char            err_extend_trace;
};

#define _IBMR(i2c)	((i2c)->reg_base + (0x0 << (i2c)->reg_shift))
#define _IDBR(i2c)	((i2c)->reg_base + (0x4 << (i2c)->reg_shift))
#define _ICR(i2c)	((i2c)->reg_base + (0x8 << (i2c)->reg_shift))
#define _ISR(i2c)	((i2c)->reg_base + (0xc << (i2c)->reg_shift))
#define _ISAR(i2c)	((i2c)->reg_base + (0x10 << (i2c)->reg_shift))
#define _ILCR(i2c)	((i2c)->reg_base + (0x14 << (i2c)->reg_shift))
#define _IWCR(i2c)	((i2c)->reg_base + (0x18 << (i2c)->reg_shift))

#define _WFIFO(i2c)      ((i2c)->reg_base + (0x20 << (i2c)->reg_shift))
#define _WFIFO_WPTR(i2c) ((i2c)->reg_base + (0x22 << (i2c)->reg_shift))
#define _WFIFO_RPTR(i2c) ((i2c)->reg_base + (0x24 << (i2c)->reg_shift))
#define _RFIFO(i2c)      ((i2c)->reg_base + (0x28 << (i2c)->reg_shift))
#define _RFIFO_WPTR(i2c) ((i2c)->reg_base + (0x2A << (i2c)->reg_shift))
#define _RFIFO_RPTR(i2c) ((i2c)->reg_base + (0x2C << (i2c)->reg_shift))
#define _FIFO_TSHLD(i2c) ((i2c)->reg_base + (0x30 << (i2c)->reg_shift))

/*
 * FIFO entry and threshold infomation
 */
#define I2C_PXA_FIFO_TSHLD_RX	0x8
#define I2C_PXA_FIFO_TSHLD_TX	0x0
#define I2C_PXA_FIFO_ENTRY_RX	0x10
#define I2C_PXA_FIFO_ENTRY_TX	0x10

#define assert(expr) \
	if(unlikely(!(expr))) {				        \
	pr_err("Assertion failed! %s,%s,%s,line=%d\n",	\
	#expr, __FILE__, __func__, __LINE__);			\
	}

/*
 * I2C Slave mode address for this controller but not remote slave
 * The CPU I2C unit in fact never work in a slave but always in master mode.
 * Use any "invalid" address, but if high value used the detection
 * "not my address" would be faster (ZERO-bit wins on arbitration).
 */
#define I2C_PXA_SLAVE_ADDR      0xFF /* instead 1 */

/**************************** DEBUG ***********************************/
/** DBG_GPIO
 *
 *  The GPIOS are correct for SAAR platform only and may differe on otheer platforms
 *
 * GPIO:52 is used for the debugging on SAAR board (J9 B27)
 * For GPIOs <63:32>
 *    MFPR=0x40e102C4: 0xA840
 *    Registers are GPDRx where x=1
 *    Bit_in_reg=52-32=20   = (1<<20)
 *    GSDR1=0x40E00404  SET  Direction 1=Output
 *    GPDR1=0x40E00010       Direction to Read DIR status
 *    GPSR1=0x40E0001C  SET  Output to 1   (D10 led ON)
 *    GPCR1=0x40E00028  CLR  Output to 0   (D10 led OFF)
 *---------------
 * GPIO:51: bit 19 = (1<<19)  ; MFPR=0x40e102C0 (J9 B23)
 **/
/*#define DBG_GPIO*/
#ifdef  DBG_GPIO

void DBG_GPIO_CFG(void) {
	if(__raw_readl((void *)&(__REG(0x40e102C4))) != 0x0000A840)
		__raw_writel(0x0000A840, (void *)&(__REG(0x40e102C4)));  /*MFPR*/
	if(!(__raw_readl((void *)&(__REG(0x40E00010))) & (1<<20)))
		__raw_writel((1<<20), (void *)&(__REG(0x40E00404)));  /*Direction SET*/

	if((__raw_readl((void *)&(__REG(0x40e102C0))) != 0x0000A840)
			|| (! (__raw_readl((void *)&(__REG(0x40E00010))) & (1<<19))) )
	{
		__raw_writel(0x0000A840, (void *)&(__REG(0x40e102C0)));  /*MFPR*/
		__raw_writel((1<<19), (void *)&(__REG(0x40E00404)));  /*Direction SET*/
		__raw_writel((1<<19), (void *)&(__REG(0x40E00028)));  /*init-state off*/
	}
}

#define DBG_GPIO_DRV_IN()    /*__raw_writel((1<<20), (void *)&(__REG(0x40E00028))) off*/
#define DBG_GPIO_DRV_OUT()   /*__raw_writel((1<<20), (void *)&(__REG(0x40E0001C))) on*/
/*#define DBG_GPIO_IRQH_IN()   __raw_writel((1<<20), (void *)&(__REG(0x40E0001C))) on*/
/*#define DBG_GPIO_IRQH_OUT()  __raw_writel((1<<20), (void *)&(__REG(0x40E00028))) off*/

#define DBG_GPIO_IRQH_IN()   __raw_writel((1<<20), (void *)&(__REG(0x40E00028))) /*off*/
#define DBG_GPIO_IRQH_OUT()  __raw_writel((1<<20), (void *)&(__REG(0x40E0001C))) /*on*/

#define DBG_GPIO_ERR_ON()    __raw_writel((1<<19), (void *)&(__REG(0x40E0001C))) /*on*/
#define DBG_GPIO_ERR_OFF()   __raw_writel((1<<19), (void *)&(__REG(0x40E00028))) /*off*/
#define DBG_GPIO_WAIT_ON()  /* __raw_writel((1<<19), (void *)&(__REG(0x40E0001C))) on*/
#define DBG_GPIO_WAIT_OFF() /* __raw_writel((1<<19), (void *)&(__REG(0x40E00028))) off*/

#else/*DBG_GPIO*/

#define DBG_GPIO_CFG()
#define DBG_GPIO_DRV_IN()
#define DBG_GPIO_DRV_OUT()
#define DBG_GPIO_IRQH_IN()
#define DBG_GPIO_IRQH_OUT()
#define DBG_GPIO_ERR_ON()
#define DBG_GPIO_ERR_OFF()
#define DBG_GPIO_WAIT_ON()
#define DBG_GPIO_WAIT_OFF()

#endif/*DBG_GPIO*/

#ifdef DEBUG

struct bits {
	u32	mask;
	const char *set;
	const char *unset;
};
#define PXA_BIT(m, s, u)	{ .mask = m, .set = s, .unset = u }

static inline void
decode_bits(const char *prefix, const struct bits *bits, int num, u32 val)
{
	printk("%s %08x: ", prefix, val);
	while (num--) {
		const char *str = val & bits->mask ? bits->set : bits->unset;
		if (str)
			printk("%s ", str);
		bits++;
	}
}

static const struct bits isr_bits[] = {
	PXA_BIT(ISR_RWM,	"RX",		"TX"),
	PXA_BIT(ISR_ACKNAK,	"NAK",		"ACK"),
	PXA_BIT(ISR_UB,		"Bsy",		"Rdy"),
	PXA_BIT(ISR_IBB,	"BusBsy",	"BusRdy"),
	PXA_BIT(ISR_EBB,	"EarBusBsy",	"BusRdy"),
	PXA_BIT(ISR_SSD,	"SlaveStop",	NULL),
	PXA_BIT(ISR_ALD,	"ALD",		NULL),
	PXA_BIT(ISR_ITE,	"TxEmpty",	NULL),
	PXA_BIT(ISR_IRF,	"RxFull",	NULL),
	PXA_BIT(ISR_GCAD,	"GenCall",	NULL),
	PXA_BIT(ISR_SAD,	"SlaveAddr",	NULL),
	PXA_BIT(ISR_BED,	"BusErr",	NULL),
};

static void decode_ISR(unsigned int val)
{
	decode_bits(KERN_DEBUG "ISR", isr_bits, ARRAY_SIZE(isr_bits), val);
	printk("\n");
}

static const struct bits icr_bits[] = {
	PXA_BIT(ICR_START,  "START",	NULL),
	PXA_BIT(ICR_STOP,   "STOP",	NULL),
	PXA_BIT(ICR_ACKNAK, "ACKNAK",	NULL),
	PXA_BIT(ICR_TB,     "TB",	NULL),
	PXA_BIT(ICR_MA,     "MA",	NULL),
	PXA_BIT(ICR_SCLE,   "SCLE",	"scle"),
	PXA_BIT(ICR_IUE,    "IUE",	"iue"),
	PXA_BIT(ICR_GCD,    "GCD",	NULL),
	PXA_BIT(ICR_ITEIE,  "ITEIE",	NULL),
	PXA_BIT(ICR_IRFIE,  "IRFIE",	NULL),
	PXA_BIT(ICR_BEIE,   "BEIE",	NULL),
	PXA_BIT(ICR_SSDIE,  "SSDIE",	NULL),
	PXA_BIT(ICR_ALDIE,  "ALDIE",	NULL),
	PXA_BIT(ICR_SADIE,  "SADIE",	NULL),
	PXA_BIT(ICR_UR,     "UR",		"ur"),
};

#ifdef CONFIG_I2C_PXA_SLAVE
static void decode_ICR(unsigned int val)
{
	decode_bits(KERN_DEBUG "ICR", icr_bits, ARRAY_SIZE(icr_bits), val);
	printk("\n");
}
#endif

static unsigned int i2c_debug = DEBUG;

static void i2c_pxa_show_state(struct pxa_i2c *i2c, int lno, const char *fname)
{
	dev_dbg(&i2c->adap.dev, "state:%s:%d: ISR=%08x, ICR=%08x, IBMR=%02x\n", fname, lno,
			readl(_ISR(i2c)), readl(_ICR(i2c)), readl(_IBMR(i2c)));
}

#define show_state(i2c) i2c_pxa_show_state(i2c, __LINE__, __FUNCTION__)

#else /*DEBUG ************************************/

#define i2c_debug	0

#define show_state(i2c) do { } while (0)
#define decode_ISR(val) do { } while (0)
#define decode_ICR(val) do { } while (0)

#endif /*DEBUG ************************************/

static void i2c_pxa_scream_blue_murder(struct pxa_i2c *i2c, const char *whyTxt);
static int i2c_pxa_custom_spy_reg(struct pxa_i2c *i2c, struct i2c_msg *msg, int num);

/*****************************************************************
*         Configuration MODEs and Time/Retries
******************************************************************
*/
#define I2C_BUS_ERROR             (-6000)
#define I2C_NULL_BUFFER           (-4000)
#define I2C_TIMEOUT_DONT_RETRY    (-3000)
#define I2C_MASTER_ABORT          (-3010)

/*writing 1 to ISR status bits to clear them*/
#define I2C_ISR_CLEAR  (ISR_SSD | ISR_ALD | ISR_ITE | ISR_IRF | ISR_GCAD | ISR_SAD | ISR_BED | ISR_MSD | ISR_TXSR)
#define I2C_IBMR_IDLE   3 /* IBMR[SCL:1] and IBMR[SDA:0] are High */

/*===== The modes below are configured from upper layers*/
/* If the value is different from "0"-default we Force to specified mode*/
#define HANDLER_MODE__DFLT_PIO_IRQ     0 /* 0: Default ; 1: Polling only; 2: IRQ only */
#define HANDLER_MODE__DFLT_SLOW_FAST   0 /* 0: Default ; 1: Slow only ;   2: Fast only */


/* Delays are accounted for FAST 400kHz mode, for SLOW 100kHz the FAST_SLOW_RATE is used
 * More than one byte may be read/write by second Master in block/paging mode => MAX_TRANSACT_TIME_US
 */
#define MAX_XFER_TIMEOUT_US       80000   /* 80mSec guard after successfull start*/
#define MAX_TRANSACT_TIME_US       1200   /* 3+(4*2)+1=12 *10_SlowCycles = 120*10uS*/
#define FAST_SLOW_RATE            (400/100)
#define I2C_SLOW_DELAY_US         (20+1) /* 2 cycles * 1/100kHz  =  20usec*/
#define I2C_ACCESS2_DELAY_US      (5+1)	 /* 2 cycles * 1/400kHz  =  5usec*/
#define I2C_HS_DELAY_US            (5)   /* HS+FAST "Master" is on FAST clock*/
#define I2C_BUSBUSY_DELAY_US        80   /*RETRY: Shortest transaction 26bits. The time is 1/400kHz*26=65usec*/
#define NUM_RETRIES_ON_BUS_BUSY    125   /* (3ctrl+4data bytes)*9bits / 2bitsAccessDelay * FAST_SLOW_RATE*/
#define NUM_RETRIES_EXHAUSTED        3

#define OSCR_MASK		(~1UL)

#define CUSTOMIZATION_PER_SLAVE_DEVICE

#if !defined (CUSTOMIZATION_PER_SLAVE_DEVICE)
#define i2c_pxa_msg_parse(I2C,MSG,NUM)      /* */
#define i2c_pxa_custom_nack_is_ok(I2C)   (0)
#define i2c_pxa_custom_clock_adj(I2C)      I2C->icr_backup
#define CUSTOM_BYTE2BYTE_DELAY(I2C)         /* */
#else
/**********************************************************************
*        CUSTOMIZATION PER SLAVE DEVICE
***********************************************************************/
#define MAX_CUSTOMIZED_SLAVE_DEVICES   4
/********************* CUSTOMER DATA *****************/
struct i2c_algo_custom  i2c_pxa_custom[MAX_CUSTOMIZED_SLAVE_DEVICES+1] =
{
	/*	slave_addr	clock_adj	byte2byte_delay	scream_silent  spy_reg[4]*/
	{ 0x34,		0,			0,				0xFA,	{0xFA, 0xFB, 0xFC, 0xFF}},
	{0} /*ZERO line as END must be present, index=MAX_CUSTOMIZED_SLAVE_DEVICES */
};

int i2c_custom_ForceClck(u8 slave_addr, u8 set_none0_slow1_fast2)
{
	int i = 0;
	if(i2c_pxa_custom[MAX_CUSTOMIZED_SLAVE_DEVICES-1].slave_addr != 0) {
		printk("No free place for cutomization found\n");
		return -1;
	}
	while (i2c_pxa_custom[i].slave_addr != 0) {
		if(i2c_pxa_custom[i].slave_addr == slave_addr) {
			i2c_pxa_custom[i].clock_adj = set_none0_slow1_fast2;
			return 0;
		}
		i++;
	}
	i2c_pxa_custom[i].slave_addr = slave_addr;
	i2c_pxa_custom[i].clock_adj  = set_none0_slow1_fast2;
	return 0;
}
/****************END CUSTOMER DATA ****************/

static void i2c_pxa_msg_parse(struct pxa_i2c *i2c, struct i2c_msg *msg, int num)
{
	int i =  (num > 1)  ?  1 : 0;

	i2c->req_slave_addr     = msg[i].addr & 0x7f;
	i2c->req_read           =(msg[i].flags & I2C_M_RD) ? 'R' : 'W';
	i2c->req_num_data_bytes = msg[i].len;
}

#if !defined(SAVE_MSG_REQUEST_DEBUG_BUF)
#define i2c_pxa_msg_save(i2c, msg, num)
#else
static void i2c_pxa_msg_save(struct pxa_i2c *i2c, struct i2c_msg *msg, int num)
{
	struct i2c_msg_debug *p;

	i2c->debug_num_msgs = num;
	/* struct i2c_msg: (u16_addr, u16_flags), (u16_len, u16), u32_bufP */
	p = i2c->debug_msgs;
	p[0].addr  = msg[0].addr;
	p[0].flags = msg[0].flags;
	p[0].len   = msg[0].len;
	memcpy(&(p[0].buf[0]), (u8 *)msg[0].buf, 4);

	if (num > 1) {
		p[1].addr  = msg[1].addr;
		p[1].flags = msg[1].flags;
		p[1].len   = msg[1].len;
		memcpy(&(p[1].buf[0]), (u8 *)msg[1].buf, 4);
	}
}
#endif/*SAVE_MSG_REQUEST_DEBUG_BUF*/

static void i2c_pxa_msg_print(struct pxa_i2c *i2c, struct i2c_msg *msg, int num)
{
	u8  *bufP;
	char text[80];
	int  offs;
	int  i, k, len;

	printk(KERN_ERR "i2c: Current: num_messages=%d \n", num);
	num = (num > 1) ? 2 : 1;

	for (k = 0; k < num; k++) {
		bufP = msg[k].buf;
		offs = sprintf(text, "i2c:   slave_%02x, flags_%04x, len_%d:",
								msg[k].addr, msg[k].flags, msg[k].len);
		len = (msg[k].len <= 4) ? msg[k].len : 4;
		for (i = 0; i < len; i++)
			offs += sprintf(text+offs, " %02x", bufP[i]);
		printk(KERN_ERR "%s \n", text);
	}

#ifdef SAVE_MSG_REQUEST_DEBUG_BUF
  {
	struct i2c_msg_debug *msgD = i2c->debug_msgs;

	printk(KERN_ERR "i2c: Previous: num_messages=%d \n", i2c->debug_num_msgs);
	num = (i2c->debug_num_msgs > 1) ? 2 : 1;

	for (k = 0; k < num; k++) {
		bufP = &(i2c->debug_msgs[k].buf[0]);
		offs = sprintf(text, "i2c:   slave_%02x, flags_%04x, len_%d:",
								msgD[k].addr, msgD[k].flags, msgD[k].len);
		len = (msgD[k].len <= 4) ? msgD[k].len : 4;
		for (i = 0; i < len; i++)
			offs += sprintf(text+offs, " %02x", bufP[i]);
		printk(KERN_ERR "%s \n", text);
	}
  }
#endif/*SAVE_MSG_REQUEST_DEBUG_BUF*/
	printk(KERN_ERR "\n");
}


static struct i2c_algo_custom* i2c_pxa_custom_hit(struct pxa_i2c *i2c)
{
	struct i2c_algo_custom *p = i2c->p_custom;

	if(p != NULL) {
		while (p->slave_addr != 0) {
			if(p->slave_addr == i2c->req_slave_addr)
				break;
			p++;
		}
	}
	return p;
}

/* There are some specific addresses which do NOT give an ACK:
* SlaveAddr=0x34, the operation is write (SlaveAddr<<1), and the write applied into 0xFA..FF.
* Check them and do NOT print error report
*/
static int i2c_pxa_custom_nack_is_ok(struct pxa_i2c *i2c)
{
	struct i2c_algo_custom *p = i2c_pxa_custom_hit(i2c);

	if(p == NULL) return 0;

	if (i2c->msg->flags & (I2C_M_IGNORE_NAK | I2C_M_NO_RD_ACK))
			return 1;

	if(p->scream_silent) {
		if(i2c->msg->buf[0/*i2c->msg_ptr*/] >= p->scream_silent)
			return 1;	/* Action */
	}
	return 0;
}

static int i2c_pxa_custom_clock_adj(struct pxa_i2c *i2c)
{
	struct i2c_algo_custom *p = i2c_pxa_custom_hit(i2c);

	if(p == NULL) return i2c->icr_backup;

	if(p->clock_adj == 1) {
		return (i2c->icr_backup & ~((1 << 16) | (1 << 15)) ); /* set SLOW=100k */
	} else if(p->clock_adj == 2) {
		return (i2c->icr_backup | (1 << 15) );                /* set FAST=400k */
	}
	return i2c->icr_backup; /* not changed */
}

static int i2c_pxa_custom_spy_reg(struct pxa_i2c *i2c, struct i2c_msg *msg, int num)
{
	struct i2c_algo_custom *p = i2c_pxa_custom_hit(i2c);
	int i;
	u8 bulk_begin, bulk_end;
	int found = 0;

	if ((p == NULL) || (num == 0))
		return 0;

	/* There are 2 cases to be checked: */
	if ((num == 2) && (msg[1].len > 1)) {
		/* The register is not in Address, but accessed in Bulk-range*/
		bulk_begin = msg[0].buf[0];
		bulk_end   = msg[0].buf[0] + msg[1].len - 1;
	} else {
		/* "under spy" is accessed directly as Address */
		bulk_begin = msg[0].buf[0];
		bulk_end   = msg[0].buf[0];
	}

	for (i=0; i < 4/*(sizeof(p->spy_reg)/sizeof(p->spy_reg[0]))*/; i++) {
		if (p->spy_reg[i] == 0)
			break;
		if ((p->spy_reg[i] >= bulk_begin) &&
			(p->spy_reg[i] <= bulk_end)) {
				printk(KERN_INFO "SPY i2c slave_%x:reg_%x (op_%c), access by range [%x,%x]\n",
					i2c->req_slave_addr, msg[0].buf[0], i2c->req_read,
					bulk_begin, bulk_end);
					i2c_pxa_msg_print(i2c, msg, num);
				found = 1;
				break;
		}
	}
	return found;
}

#ifndef CUSTOM_BYTE2BYTE_DELAY_IN_USE
#define CUSTOM_BYTE2BYTE_DELAY(I2C)    /* */
#else
#define CUSTOM_BYTE2BYTE_DELAY(I2C)    i2c_pxa_custom_byte2byte_delay(I2C)
static void i2c_pxa_custom_byte2byte_delay(struct pxa_i2c *i2c)
{
	struct i2c_algo_custom *p = i2c_pxa_custom_hit(i2c);
	if(p == NULL) return;

	if(p->byte2byte_delay) {
		udelay(100);
	}
	return;
}
#endif/*CUSTOM_BYTE2BYTE_DELAY*/

/*** END CUSOTMIZATION DATA AND PROCEDURES *****************************/
#endif/*CUSTOMIZATION_PER_SLAVE_DEVICE*/

#ifdef	CONFIG_PROC_FS
#define	I2C_PXA_PROC_FILE          "driver/i2c-pxa"
#define	I2C_PXA_PROC_NUM_DEVS      3
static struct proc_dir_entry *i2c_pxa_proc_file;
static struct pxa_i2c *proc_i2c[I2C_PXA_PROC_NUM_DEVS];
static int    err_simulate;

#define DEBUG_ERR_SIMULATE       0xF9 /*Non-exist slave device address */
static void i2c_pxa_buserr_simulate(struct i2c_msg *msg, int num)
{
	if(err_simulate) {
		int i =  (num > 1)  ?  1 : 0;
		msg[i].addr = DEBUG_ERR_SIMULATE;
		err_simulate--;
	}
}

static ssize_t i2c_pxa_proc_read(struct file *filp,
		char *buffer, size_t count, loff_t *offset)
{
	return 0;
}

static void i2c_pxa_reset(struct pxa_i2c *i2c);

static ssize_t i2c_pxa_proc_write(struct file *filp,
		const char *buff, size_t len, loff_t *off)
{
	char messages[256];

	if (len > 256)
		len = 256;

	if (copy_from_user(messages, buff, len))
		return -EFAULT;

	if ('?' == messages[0]) {
		/* echo ? > /proc/driver/i2c-pxa */
		printk("\n possible commands:\n"
				"    i2c0slow .. i2c2slow  force I2C clock to 100kHz\n"
				"    i2c0fast .. i2c2fast  force I2C clock to 400kHz\n"
				"    i2c0hs                force I2C clock to HigSpeed+Fast\n"
				"    trace <bus 0/1/2> <level 0/err/all>  extended trace\n"
				"                          for example:   trace 0 err\n"
#if defined (CUSTOMIZATION_PER_SLAVE_DEVICE)
				"   Force clock only for specified NN hexAddress Slave-Device\n"
				"    slave.fast_NN   force  to 400kHz\n"
				"    slave.slow_NN   force  to 100kHz\n"
				"    slave.nocl_NN   NO force\n"
#endif
				"    err1       simulate one Bus-Error=accessToNonExist\n"
	  );
	} else if (strncmp(messages, "i2c0slow", 5) == 0) {
		proc_i2c[0]->icr_backup &= ~(1 << 15);
		proc_i2c[0]->flags &= ~PXA_I2C_HIGH_MODE;
		i2c_pxa_reset(proc_i2c[0]);
	} else if (strncmp(messages, "i2c0fast", 5) == 0) {
		proc_i2c[0]->icr_backup |= (1 << 15);
		proc_i2c[0]->flags &= ~PXA_I2C_HIGH_MODE;
		i2c_pxa_reset(proc_i2c[0]);
	} else if (strncmp(messages, "i2c0hs"  , 5) == 0) {
		proc_i2c[0]->icr_backup |= (1 << 16) | (1 << 15);
		proc_i2c[0]->flags |= PXA_I2C_HIGH_MODE;
		i2c_pxa_reset(proc_i2c[0]);
	} else if (strncmp(messages, "i2c1slow", 5) == 0) {
		proc_i2c[1]->icr_backup &= ~(1 << 15);
	} else if (strncmp(messages, "i2c1fast", 5) == 0) {
		proc_i2c[1]->icr_backup |= (1 << 15);
	} else if (strncmp(messages, "i2c2slow", 5) == 0) {
		proc_i2c[2]->icr_backup &= ~(1 << 15);
	} else if (strncmp(messages, "i2c2fast", 5) == 0) {
		proc_i2c[2]->icr_backup |= (1 << 15);

	} else if (strncmp(messages, "trace ", 6) == 0) {
		int bus = messages[6] - '0';
		if ((bus < 0) || (bus > (I2C_PXA_PROC_NUM_DEVS-1))) {
			printk(KERN_ERR "valid bus numbers are 0..%d\n", (I2C_PXA_PROC_NUM_DEVS-1));
			goto proc_end;
		}
		proc_i2c[bus]->dbg_trace = messages[6+2];

#if defined (CUSTOMIZATION_PER_SLAVE_DEVICE)
	} else if (strncmp(messages, "slave.",6) == 0) {
		u8   clck_op=0xFF; /*invalid*/
		if (strncmp(messages, "slave.fast_",11) == 0)
			clck_op = 2;
		else
			if (strncmp(messages, "slave.slow_",11) == 0)
				clck_op = 1;
			else
				if (strncmp(messages, "slave.nocl_",11) == 0)
					clck_op = 0;

		if (clck_op != 0xFF) {
			u8   slave_addr;
			char vol[256];
			memcpy(vol, messages+11, len-11);
			slave_addr = (u8)simple_strtoul(vol, NULL, 16);
			if(slave_addr == 0) {
				clck_op = 0xFF;
			} else {
				printk("\n Force clock for slave device address 0x%X\n", slave_addr);
				i2c_custom_ForceClck(slave_addr, clck_op);
			}
		}
		if (clck_op == 0xFF)
			printk("\n Incorrect format. Use ? for help\n");
#endif/*CUSTOMIZATION_PER_SLAVE_DEVICE*/
	} else if (strncmp(messages, "err1", 4) == 0) {
		err_simulate = 1;
	} else {
		printk("\n Unknown command\n");
	}
proc_end:
	return len;
}

static struct file_operations i2c_pxa_proc_ops = {
	.read  = i2c_pxa_proc_read,
	.write = i2c_pxa_proc_write,
};

static void create_i2c_pxa_proc_file(struct pxa_i2c *i2c, int id)
{
	if(id >= I2C_PXA_PROC_NUM_DEVS)
		return;

	proc_i2c[id] = i2c;

	if(i2c_pxa_proc_file != NULL) /* already done */
		return;

	i2c_pxa_proc_file = create_proc_entry(I2C_PXA_PROC_FILE, 0644, NULL);
	if (i2c_pxa_proc_file) {
		/*i2c_pxa_proc_file->owner = THIS_MODULE;*/
		i2c_pxa_proc_file->proc_fops = &i2c_pxa_proc_ops;
	} else
		printk(KERN_INFO "proc file create failed!\n");
}

static void remove_i2c_pxa_proc_file(int id)
{
	if((id != 0) || (i2c_pxa_proc_file == NULL))
		return;

	remove_proc_entry(I2C_PXA_PROC_FILE, NULL);
	i2c_pxa_proc_file = NULL;
}

#endif /*CONFIG_PROC_FS*/

static unsigned int read_time(void)
{
#ifdef CONFIG_PXA_32KTIMER
	return OSCR4;
#else
	return OSCR;
#endif
}

static void register_dvfm_constraint(struct pxa_i2c *i2c)
{
	int ret;
	i2c->dvfm_lock.lock    = SPIN_LOCK_UNLOCKED;
	i2c->dvfm_lock.count   = 0;
	i2c->dvfm_lock.dev_idx = -1;
	ret = dvfm_register(i2c->adap.name, &i2c->dvfm_lock.dev_idx);
	if (ret)
		printk(KERN_ERR "I2C: dvfm register error.\n");
}

static void unregister_dvfm_constraint(struct pxa_i2c *i2c)
{
	dvfm_unregister(i2c->adap.name, &i2c->dvfm_lock.dev_idx);
}

/*
 * dvfm_lock.count is used for releasing dvfm constraint immediately
 * when unset_dvfm_constraint is invoked, even though set_dvfm_constraint() is invoked many times
 */
static void set_dvfm_constraint(struct pxa_i2c *i2c)
{
	spin_lock_irqsave(&i2c->dvfm_lock.lock, i2c->dvfm_lock.flags);
	if (i2c->dvfm_lock.count++ == 0) {
		/* Disable Low power mode */
		dvfm_disable_op_name("D1", i2c->dvfm_lock.dev_idx);
		dvfm_disable_op_name("D2", i2c->dvfm_lock.dev_idx);
		dvfm_disable_op_name("CG", i2c->dvfm_lock.dev_idx);
	} else
		i2c->dvfm_lock.count--;
	spin_unlock_irqrestore(&i2c->dvfm_lock.lock, i2c->dvfm_lock.flags);
}

static void unset_dvfm_constraint(struct pxa_i2c *i2c)
{
	spin_lock_irqsave(&i2c->dvfm_lock.lock, i2c->dvfm_lock.flags);
	if (i2c->dvfm_lock.count == 0) {
		spin_unlock_irqrestore(&i2c->dvfm_lock.lock, i2c->dvfm_lock.flags);
		return;
	}
	if (--i2c->dvfm_lock.count == 0) {
	/* Enable Low power mode */
		dvfm_enable_op_name("D1", i2c->dvfm_lock.dev_idx);
		dvfm_enable_op_name("D2", i2c->dvfm_lock.dev_idx);
		dvfm_enable_op_name("CG", i2c->dvfm_lock.dev_idx);
	} else
		i2c->dvfm_lock.count++;
	spin_unlock_irqrestore(&i2c->dvfm_lock.lock, i2c->dvfm_lock.flags);
}

#define eedbg(lvl, x...) do { if ((lvl) < 1) { printk(KERN_DEBUG "" x); } } while(0)

#define ICR_LOG      i2c->icrlog[0]
#define ISR_LOG      i2c->isrlog[0]
#define IBMR_LOG     i2c->isrlog[2]
#define ENTRY_CNTR   i2c->icrlog[1]
#define ERROR_CNTR   i2c->isrlog[1]

static void i2c_pxa_master_complete(struct pxa_i2c *i2c, int ret);
static irqreturn_t i2c_pxa_handler(int this_irq, void *dev_id);

static void i2c_pxa_scream_blue_murder(struct pxa_i2c *i2c, const char *whyTxt)
{
	ERROR_CNTR++;

	printk(KERN_ERR "i2c: Time[%u] %s \n", read_time(), whyTxt);
	printk(KERN_ERR "i2c: %s:%x, slave_0x%X, %c-operation, %d data-bytes\n",
			i2c->adap.name,(unsigned int)i2c, i2c->req_slave_addr, i2c->req_read, i2c->req_num_data_bytes);
	printk(KERN_ERR "i2c: msg_num=%d, msg_idx=%d, msg_ptr=%d;  track=%d, Entry/Err=%d/%d\n",
			i2c->msg_num, i2c->msg_idx, i2c->msg_ptr,  i2c->debug_track, ENTRY_CNTR, ERROR_CNTR);
	printk(KERN_ERR "i2c: logged  icr=%08x isr=%08x ibmr=%x\n",
			ICR_LOG, ISR_LOG, IBMR_LOG);
	printk(KERN_ERR "i2c: current ICR=%08x ISR=%08x IBMR=%x\n",
			readl(_ICR(i2c)), readl(_ISR(i2c)), readl(_IBMR(i2c)) );
	printk(KERN_ERR "\n");
	/*i2c->debug_track=0;   - keep, don't reset */

	if ((i2c->dbg_trace == 'e') || (i2c->dbg_trace == 'a'))
		i2c->err_extend_trace = 1;	/* err / all */
}

static inline int i2c_pxa_is_slavemode(struct pxa_i2c *i2c)
{
	return !(readl(_ICR(i2c)) & ICR_SCLE);
}

static void i2c_pxa_abort(struct pxa_i2c *i2c, int full)
{
	unsigned long icr;

	if (i2c_pxa_is_slavemode(i2c)) {
		dev_dbg(&i2c->adap.dev, "%s: called in slave mode\n", __func__);
		return;
	}

	if (readl(_IBMR(i2c)) == I2C_IBMR_IDLE)
		return;

	if( readl(_ISR(i2c)) & ISR_MSD )
	{
		writel(ISR_MSD, _ISR(i2c));
		return;
	}

	if(readl(_ICR(i2c)) & ICR_START)
	{  /*Under START state the MA-bit cannot be used*/
		unsigned long timeout = jiffies + HZ/4;
		while (time_before(jiffies, timeout) && (readl(_IBMR(i2c)) & 0x1) == 0)
		{
			icr = readl(_ICR(i2c));

			icr &= ~ICR_START;
			icr |= ICR_ACKNAK | ICR_STOP | ICR_TB;

			writel(icr, _ICR(i2c));
			udelay(i2c->access2_delay);
		}
	}
	else
	{ /*MA Master Abort bit used*/
		icr = readl(_ICR(i2c));
		icr &= ~(ICR_START | ICR_ACKNAK | ICR_STOP | ICR_TB);
		icr |= ICR_MA;
		writel(I2C_ISR_CLEAR,  _ISR(i2c));
		writel(icr, _ICR(i2c));
		udelay(i2c->access2_delay);
	}
	writel(readl(_ICR(i2c)) & ~(ICR_MA | ICR_START | ICR_STOP), _ICR(i2c));
	if (full == 2)
		mdelay(1); /*udelay(MAX_TRANSACT_TIME_US);*/
}

/***== Check Bus-Busy - 2 procedures:********************/
/*  is_bus_busy()*/
/*  is_busy_now() =  is_bus_busy() + read_ISR - ISR_UB*/
/********************************************************/
#ifdef CONFIG_I2C_PXA_SLAVE
static int is_bus_busy(u32 isr)
{
	int ret = 0;

	if (isr & (ISR_IBB | ISR_UB | ISR_EBB))
		ret = 1;

	return ret;
}

static int is_busy_now(struct pxa_i2c *i2c)
{
	u32 isr;
	int ret = 0;

	isr = readl(_ISR(i2c));
	if (isr & (ISR_IBB | ISR_EBB))
		ret = 1;

	return ret;
}

static int i2c_pxa_set_master(struct pxa_i2c *i2c)
{
	u32 isr;

	if (i2c_debug)
		dev_dbg(&i2c->adap.dev, "setting to bus master\n");

	isr = readl(_ISR(i2c));
	if (is_bus_busy(isr)) {
		dev_dbg(&i2c->adap.dev, "%s: unit is busy\n", __func__);
		if (i2c_debug > 1)
			dev_dbg(&i2c->adap.dev, "%s: %ld: ISR=%08x, ICR=%08x, IBMR=%02x\n",
					__func__, (long)jiffies, readl(_ISR(i2c)),
					readl(_ICR(i2c)), readl(_IBMR(i2c)));

		if (isr & ISR_SAD) {
			if (i2c_debug > 0)
				dev_dbg(&i2c->adap.dev, "%s: Slave detected\n",
						__func__);
		}
		return I2C_RETRY;
	} else if (readl(_IBMR(i2c)) == I2C_IBMR_IDLE) {
		/* wait for unit and bus being not busy, and we also do a
		 * quick check of the i2c lines themselves to ensure they've
		 * gone high...
		 */
		if (i2c_debug > 0)
			dev_dbg(&i2c->adap.dev, "%s: done\n", __func__);
		/* set as master */
		writel(readl(_ICR(i2c)) | ICR_SCLE, _ICR(i2c));
		return 0;
	}
	if (i2c_debug > 0)
		dev_dbg(&i2c->adap.dev, "%s: did not free\n", __func__);

	return I2C_RETRY;
}

static int i2c_pxa_wait_slave(struct pxa_i2c *i2c)
{
	unsigned long timeout = jiffies + HZ*1;
	u32 isr;
	/* wait for stop */

	show_state(i2c);

	while (time_before(jiffies, timeout)) {
		if (i2c_debug > 1)
			dev_dbg(&i2c->adap.dev, "%s: %ld: ISR=%08x, ICR=%08x, IBMR=%02x\n",
					__func__, (long)jiffies, readl(_ISR(i2c)), readl(_ICR(i2c)), readl(_IBMR(i2c)));

		isr = readl(_ISR(i2c));
		if (!is_bus_busy(isr) || (isr & ISR_SAD)
				|| ((readl(_ICR(i2c)) & ICR_SCLE) == 0)) {
			if (i2c_debug > 1)
				dev_dbg(&i2c->adap.dev, "%s: done\n", __func__);
			return 1;
		}

		msleep(1);
	}

	if (i2c_debug > 0)
		dev_dbg(&i2c->adap.dev, "%s: did not free\n", __func__);
	return 0;
}

/*
 * clear the hold on the bus, and take of anything else
 * that has been configured
 */
static void i2c_pxa_set_slave(struct pxa_i2c *i2c, int errcode)
{
	show_state(i2c);

	if (errcode < 0) {
		udelay(i2c->access2_delay);   /* simple delay */
	} else {
		/* we need to wait for the stop condition to end */

		/* if we where in stop, then clear... */
		if (readl(_ICR(i2c)) & ICR_STOP) {
			udelay(i2c->access2_delay);
			writel(readl(_ICR(i2c)) & ~ICR_STOP, _ICR(i2c));
		}

		if (!i2c_pxa_wait_slave(i2c)) {
			dev_err(&i2c->adap.dev, "%s: wait timedout\n",
					__func__);
			return;
		}
	}

	writel(readl(_ICR(i2c)) & ~(ICR_STOP|ICR_ACKNAK|ICR_MA), _ICR(i2c));
	writel(readl(_ICR(i2c)) & ~ICR_SCLE, _ICR(i2c));

	if (i2c_debug) {
		dev_dbg(&i2c->adap.dev, "ICR now %08x, ISR %08x\n", readl(_ICR(i2c)), readl(_ISR(i2c)));
		decode_ICR(readl(_ICR(i2c)));
	}
}
#else
#define i2c_pxa_set_slave(i2c, err)	do { } while (0)
#endif/*CONFIG_I2C_PXA_SLAVE*/

/* Restart HW controller.
 * Depending upon full=2/1/0 calls or not for Master-Abort procedure.
 * Do NOT use "2" in Atomic or IRQ context since it has delay inside
 */
static void i2c_pxa_unit_restart(struct pxa_i2c *i2c, int full)
{
	if (full)
	{
		/* If bus is not IDLE, abort any transfer currently under way (should send STOP)*/
		i2c_pxa_abort(i2c, full);
	}
	/*Exec reset according to 9.8, but KEEP in reset*/
	writel(ICR_UR, _ICR(i2c));
	writel(I2C_ISR_CLEAR, _ISR(i2c));
	writel( 0 , _ICR(i2c));
	writel(i2c->slave_addr, _ISAR(i2c));

	/* Clear FIFO pointers (must for "aborted" transaction */
	writel(0, _WFIFO_WPTR(i2c));
	writel(0, _WFIFO_RPTR(i2c));
	writel(0, _RFIFO_WPTR(i2c));
	writel(0, _RFIFO_RPTR(i2c));

	if (full == 2)
		mdelay(1);
}



/* Set Functional clock 69/33 Mhz (outside of I2C-controller)
 * applied in ACCR1 register. Every I2C has its own bits
 *  NOTE_1:  protection is very important here!
 *  NOTE_2:  the HS clock may be used even for FAST/SLOW mode
 *           but the ILCR and IWCR should have appropriated value
*/
/*#define ALWAYS_USE_HIGH_FREQUENCY*/
    #define	ACCR1_I2C0		0x00000100
	#define	ACCR1_I2C1		0x00080000
	#define	ACCR1_I2C2		0x00100000
static int i2c_pxa_hs_clock(struct pxa_i2c *i2c)
{
    unsigned long cpsr;
    u32  accr_i2c_mask;
#if defined(ALWAYS_USE_HIGH_FREQUENCY)
	int flags = PXA_I2C_HIGH_MODE;
#else
	int flags = i2c->flags & PXA_I2C_HIGH_MODE;
#endif

	if (i2c->adap.nr == 0)
		accr_i2c_mask = ACCR1_I2C0;
	else
	if (i2c->adap.nr == 1)
		accr_i2c_mask = ACCR1_I2C1;
	else
		accr_i2c_mask = ACCR1_I2C2;

	local_irq_save(cpsr);

	if (flags) {
		if (!(ACCR1 & accr_i2c_mask))
			ACCR1 |= accr_i2c_mask;
	} else {
		if (ACCR1 & (accr_i2c_mask))
			ACCR1 &= ~accr_i2c_mask;
	}
	local_irq_restore(cpsr);
	return flags;
}

static void i2c_pxa_reset(struct pxa_i2c *i2c)
{
	u32 icr;

	pr_info("Resetting I2C Controller Unit.  ");

	i2c_pxa_unit_restart(i2c, 2);

	/* set control register values */
	writel((I2C_ICR_INIT & ~ICR_SCLE), _ICR(i2c));

	/* There are 2 multi-masters on the I2C bus - APPS and COMM
	 * It is important both uses the standard frequency
	 * Adjust clock by ILCR, IWCR is important
	 */
	if (i2c_pxa_hs_clock(i2c)) {
		writel(0x082CA356, _ILCR(i2c));
		writel(0x0000143A, _IWCR(i2c));
		/* ilcr &= ~0xfffc0000;                                        */
		/* ilcr |= ((0xc)<<18) | (1<<27);// 3.1Mhz-SCL, high speed mode*/
		udelay(2);
	} else {
		writel(0x082C45A3, _ILCR(i2c));
		writel(0x00001431, _IWCR(i2c));
	}

#ifdef CONFIG_I2C_PXA_SLAVE
	dev_info(&i2c->adap.dev, "Enabling slave mode\n");
	writel(readl(_ICR(i2c)) | ICR_SADIE | ICR_ALDIE | ICR_SSDIE, _ICR(i2c));
#endif

	i2c_pxa_set_slave(i2c, 0);

	icr = readl(_ICR(i2c));
	if (i2c->flags & PXA_I2C_HIGH_MODE) {
		icr |= (1 << 16);
		if (i2c->flags & PXA_I2C_FAST_MODE) {
			pr_info("I2C: Specify HIGH I2C speed with fast mode.\n");
			icr |= (1 << 15);
		} else if (i2c->flags & PXA_I2C_STANDARD_MODE) {
			icr &= ~(1 << 15);
			pr_info("I2C: Specify HIGH I2C speed with standard mode.\n");
		}
	} else if (i2c->flags & PXA_I2C_FAST_MODE) {
		icr |= (1 << 15);
		icr &= ~(1 << 16);
		pr_info("I2C: Specify FAST I2C speed.\n");
	} else if (i2c->flags & PXA_I2C_STANDARD_MODE) {
		icr &= ~(1 << 15);
		icr &= ~(1 << 16);
		pr_info("I2C: Specify SLOW I2C speed.\n");
	} else
		pr_info("I2C: unsupport i2c mode!!!\n");

	i2c->icr_backup = icr | ICR_IUE | ICR_SCLE;

	/* enable unit */
	writel(icr | ICR_IUE | ICR_SCLE, _ICR(i2c));
	udelay(100);
}

#ifdef CONFIG_I2C_PXA_SLAVE
/*
 * PXA I2C Slave mode
 */

static void i2c_pxa_slave_txempty(struct pxa_i2c *i2c, u32 isr)
{
	if (isr & ISR_BED) {
		/* what should we do here? */
		i2c_pxa_scream_blue_murder(i2c, "Bus ERROR i2c_pxa_slave_txempty");
	} else {
		int ret = 0;

		if (i2c->slave != NULL)
			ret = i2c->slave->read(i2c->slave->data);

		writel(ret, _IDBR(i2c));
		writel(readl(_ICR(i2c)) | ICR_TB, _ICR(i2c));   /* allow next byte */
	}
}

static void i2c_pxa_slave_rxfull(struct pxa_i2c *i2c, u32 isr)
{
	unsigned int byte = readl(_IDBR(i2c));

	if (i2c->slave != NULL)
		i2c->slave->write(i2c->slave->data, byte);

	writel(readl(_ICR(i2c)) | ICR_TB, _ICR(i2c));
}

static void i2c_pxa_slave_start(struct pxa_i2c *i2c, u32 isr)
{
	int timeout;

	if (i2c_debug > 0)
		dev_dbg(&i2c->adap.dev, "SAD, mode is slave-%cx\n",
				(isr & ISR_RWM) ? 'r' : 't');

	if (i2c->slave != NULL)
		i2c->slave->event(i2c->slave->data,
				(isr & ISR_RWM) ? I2C_SLAVE_EVENT_START_READ : I2C_SLAVE_EVENT_START_WRITE);

	/*
	 * slave could interrupt in the middle of us generating a
	 * start condition... if this happens, we'd better back off
	 * and stop holding the poor thing up
	 */
	writel(readl(_ICR(i2c)) & ~(ICR_START|ICR_STOP), _ICR(i2c));
	writel(readl(_ICR(i2c)) | ICR_TB, _ICR(i2c));

	timeout = 0x10000;

	while (1) {
		if ((readl(_IBMR(i2c)) & 2) == 2)
			break;

		timeout--;

		if (timeout <= 0) {
			dev_err(&i2c->adap.dev, "timeout waiting for SCL high\n");
			break;
		}
	}

	writel(readl(_ICR(i2c)) & ~ICR_SCLE, _ICR(i2c));
}

static void i2c_pxa_slave_stop(struct pxa_i2c *i2c)
{
	if (i2c_debug > 2)
		dev_dbg(&i2c->adap.dev, "ISR: SSD (Slave Stop)\n");

	if (i2c->slave != NULL)
		i2c->slave->event(i2c->slave->data, I2C_SLAVE_EVENT_STOP);

	if (i2c_debug > 2)
		dev_dbg(&i2c->adap.dev, "ISR: SSD (Slave Stop) acked\n");

	/*
	 * If we have a master-mode message waiting,
	 * kick it off now that the slave has completed.
	 */
	if (i2c->msg)
		i2c_pxa_master_complete(i2c, I2C_RETRY);
}
#else /*CONFIG_I2C_PXA_SLAVE*/
static void i2c_pxa_slave_txempty(struct pxa_i2c *i2c, u32 isr)
{
	if (isr & ISR_BED) {
		/* what should we do here? */
		i2c_pxa_scream_blue_murder(i2c, "Bus ERROR i2c_pxa_slave_txempty");
	} else {
		writel(0, _IDBR(i2c));
		writel(readl(_ICR(i2c)) | ICR_TB, _ICR(i2c));
	}
}

static void i2c_pxa_slave_rxfull(struct pxa_i2c *i2c, u32 isr)
{
	writel(readl(_ICR(i2c)) | ICR_TB | ICR_ACKNAK, _ICR(i2c));
}

static void i2c_pxa_slave_start(struct pxa_i2c *i2c, u32 isr)
{
	int timeout;

	/*
	 * slave could interrupt in the middle of us generating a
	 * start condition... if this happens, we'd better back off
	 * and stop holding the poor thing up
	 */
	writel(readl(_ICR(i2c)) & ~(ICR_START|ICR_STOP), _ICR(i2c));
	writel(readl(_ICR(i2c)) | ICR_TB | ICR_ACKNAK, _ICR(i2c));

	timeout = 0x10000;

	while (1) {
		if ((readl(_IBMR(i2c)) & 2) == 2)
			break;

		timeout--;

		if (timeout <= 0) {
			dev_err(&i2c->adap.dev, "timeout waiting for SCL high\n");
			break;
		}
	}

	writel(readl(_ICR(i2c)) & ~ICR_SCLE, _ICR(i2c));
}

static void i2c_pxa_slave_stop(struct pxa_i2c *i2c)
{
	if (i2c->msg)
		i2c_pxa_master_complete(i2c, I2C_RETRY);
}
#endif/*CONFIG_I2C_PXA_SLAVE*/

/*
 * PXA I2C Master mode
 */

static inline unsigned int i2c_pxa_addr_byte(struct i2c_msg *msg)
{
	unsigned int addr = (msg->addr & 0x7f) << 1;

	if (msg->flags & I2C_M_RD)
		addr |= 1;

	return addr;
}

static inline void i2c_pxa_start_message(struct pxa_i2c *i2c)
{
	u32 icr;

	/*
	 * Step 1: target slave address into IDBR
	 */
	writel(i2c_pxa_addr_byte(i2c->msg), _IDBR(i2c));

	/*
	 * Step 2: initiate the write.
	 *   Do NOT enable ICR_ALDIE so HW will resolve Arbitration automatically
	 */
	icr = readl(_ICR(i2c)) & ~(ICR_STOP | ICR_ALDIE);
	ICR_LOG = icr;
	icr |= ICR_START | ICR_TB | ICR_MSDE | ICR_SPIKE_FIX;

	writel(icr, _ICR(i2c));
}

static inline void i2c_pxa_stop_message(struct pxa_i2c *i2c)
{
	u32 icr, isr , i=5;

	icr = readl(_ICR(i2c));
	/* Workaround: Wait for a while before clearing ICR_STOP */
	if (icr & ICR_MSDE) {
		do {
			isr = readl(_ISR(i2c));
		} while ( ~(isr & ISR_MSD) & i--);
		writel(isr,_ISR(i2c));
	}
	/*
	 * Clear the STOP and ACK flags
	 */
	icr &= ~(ICR_STOP | ICR_ACKNAK);
	writel(icr, _ICR(i2c));
}

static int i2c_pxa_do_xfer_with_fifo(struct pxa_i2c *i2c, struct i2c_msg *msg, int num)
{
	unsigned long cpsr;
	u32 icr = 0xFFFFFFFF, isr, ibmr;
	u32 retry;
	int ret;
	long timeout;

	i2c->msg = msg;
	i2c->msg_num = num;
	i2c->msg_idx = 0;
	i2c->msg_ptr = 0;
	i2c->fifo_write_byte_num = 0;
	i2c->fifo_read_byte_num = 0;
	i2c->fifo_transaction_down = false;
	i2c->irq_warn = IRQ_WARN_NONE;

	writel(0, _WFIFO_WPTR(i2c));
	writel(0, _WFIFO_RPTR(i2c));
	writel(0, _RFIFO_WPTR(i2c));
	writel(0, _RFIFO_RPTR(i2c));

	/* Wait for bus free: check twice for the Bus status bits and IBMR to be sure.
	 * This still can't guaranty "no race condition with other master" but makes best efforts.
	 * If decided the bus is IDLE
	 *   do NOT enable ICR_ALDIE so possible arbitration will be fixed by HW
	 **/
	retry = 2;
	do
	{
		retry--;
		local_irq_save(cpsr);
		ibmr = readl(_IBMR(i2c));
		isr  = readl(_ISR(i2c));
		IBMR_LOG = ibmr;
		ISR_LOG  = isr;
		if( (ibmr != I2C_IBMR_IDLE) || (isr & (ISR_EBB | ISR_IBB | ISR_UB)) ) {
			local_irq_restore(cpsr);
			udelay(i2c->access2_delay);
			/* Do not print. BUS-BUSY is valid case in multi-mastering */
			/*printk(KERN_ERR "i2c: bus busy, go to retry, isr 0x%x\n",readl(_ISR(i2c)));*/
			return I2C_RETRY;
		}
		if(retry == 0)
			break;

		local_irq_restore(cpsr);

		/*"Delay" and Re-Check Bus IDLE*/
		icr = readl(_ICR(i2c));
		ICR_LOG = icr;
		isr |= I2C_ISR_CLEAR;
		writel(isr, _ISR(i2c));

	}while(retry>0);

	if (icr == 0xFFFFFFFF)
		icr = readl(_ICR(i2c));

	icr |= ICR_FIFOEN | ICR_TXSR_IE | ICR_RXSR_IE | ICR_RXF_IE | ICR_RXOV_IE | ICR_TXDONE_IE | ICR_SPIKE_FIX;
	icr &= ~(ICR_DMA_EN | ICR_ITEIE | (1 << 9));
	writel(icr, _ICR(i2c));
	i2c->debug_track = 1;

	/*local_irq_restore(cpsr) do NOT restore,
	 *the wait_event_timeout() should go into SUSPEND before IRQ-end-event wakes up this task
	 *The wait_event_timeout() enables the irq itself
	 *WORKAROUND:
	 *Sometimes wake_up() does not wake-up the task with wait_event_timeout()
	 *In that case the task runs again upon TimeOut but condition is already TRUE.
	 *Let's use wait_event_timeout() with short (7.8mS is minimum) and go to long TO only if condition still FALSE*/

	timeout = wait_event_timeout(i2c->wait, i2c->fifo_transaction_down == true, 1); /*7.8 mSec*/

	if(!i2c->fifo_transaction_down)
	{
		timeout = wait_event_timeout(i2c->wait, i2c->fifo_transaction_down == true,  MAX_XFER_TIMEOUT_US/1000/8+10); /*78 mSec*/
	}
	if (unlikely(i2c->irq_warn != IRQ_WARN_NONE)) {
		if (unlikely(i2c->irq_warn > IRQ_WARN_UNKNOWN))
			i2c_pxa_scream_blue_murder(i2c, i2c_irq_warn_str[IRQ_WARN_UNKNOWN]);
		else
			i2c_pxa_scream_blue_murder(i2c, i2c_irq_warn_str[i2c->irq_warn]);
	}
	ret = i2c->msg_idx;
	if(ret >= 0)
	{
		if(!i2c->fifo_transaction_down) {
			i2c_pxa_scream_blue_murder(i2c, "ERROR: XFER timeout - started but never finished");
			ret = I2C_TIMEOUT_DONT_RETRY;
		}
		else
		{
			i2c->msg_ptr = 0;
			i2c->msg = NULL;
			i2c->msg_idx++;
			i2c->msg_num = 0;
		}
	}
	/*else
	 *    error handling applied in i2c_pxa_xfer()
	 *i2c->fifo_transaction_down -- keep it for debug
	 */

	if (ret == I2C_RETRY)
		udelay(i2c->busbusy_delay);
	return ret;
} /*i2c_pxa_do_xfer_with_fifo*/

/*
 * We are protected by the adapter bus mutex.
 */
static int i2c_pxa_do_xfer(struct pxa_i2c *i2c, struct i2c_msg *msg, int num)
{
	unsigned long cpsr;
	int ret, retry;
	u32 icr, isr, ibmr;
	int  polling_delay;
	long timeout;

	i2c->msg = msg;
	i2c->msg_num = num;
	i2c->msg_idx = 0;
	i2c->msg_ptr = 0;
	i2c->irqlogidx = 0;
	i2c->irq_warn = IRQ_WARN_NONE;

	ICR_LOG = 0xCAFE;

	icr = 0; /*just avoids compile warning*/

	/* Wait for bus free: check twice for the Bus status bits and IBMR to be sure.
	* This still can't guaranty "no race condition with other master" but makes best efforts.
	* If decided the bus is IDLE
	*   do NOT enable ICR_ALDIE so possible arbitration will be fixed by HW
	* Check under UNPREEMPTABLE (by local_irq_save); this guaranty
	**/
	retry = 2;
	do 	{
		retry--;
		local_irq_save(cpsr);
		ibmr = readl(_IBMR(i2c));
		isr  = readl(_ISR(i2c));
		IBMR_LOG = ibmr;
		ISR_LOG  = isr;
		if ((ibmr != I2C_IBMR_IDLE) || (isr & (ISR_EBB | ISR_IBB | ISR_UB))) {
			local_irq_restore(cpsr);
			udelay(i2c->access2_delay);
			/* Do not print. BUS-BUSY is valid case in multi-mastering */
			/*printk(KERN_ERR "i2c: bus busy, go to retry, isr 0x%x\n",readl(_ISR(i2c)));*/
			return I2C_RETRY;
		}
		if (retry == 0)
			break; /* without local_irq_restore(), keep unpreemptive */

		local_irq_restore(cpsr);

		/*"Delay" and Re-Check Bus IDLE*/
		icr = readl(_ICR(i2c));
		ICR_LOG = icr;
		isr |= I2C_ISR_CLEAR;
		writel(isr, _ISR(i2c));

    }while(retry>0);

	/*a special sequency for HS mode:
	*1. Load master code in the IDBR.Note that for HS mode,
	*set ICR [GPIOEN].

	*2. Write the following fields in the I2C Control Register (ICR):
	*Set START, clear STOP, clear ALDIE, set ITEIE, set TB

	*3. When an IDBR transmit-empty interrupt occurs, read the ISR:
	*IDBR transmit empty = 1, unit busy = 1, ACKNAK = 1

	*4. Write 1 to the ISR[ITE] bit to clear the interrupt.
	*5. Send a repeated start and slave address.
	*Load the slave address and the R/W bit in the IDBR.
	*/

	if (i2c->flags & PXA_I2C_HIGH_MODE) {
		i2c->hs_enterring = true;

		icr = readl(_ICR(i2c));
		icr |= (1 << 16);  /* bit_15 clear or set already done by reset*/

		i2c->access2_delay = I2C_HS_DELAY_US;

		/*step 1*/
		icr |= ICR_GPIOEN;
		writel(i2c->master_code, _IDBR(i2c));

		/*step 2*/
		icr |= ICR_START | ICR_TB | ICR_ITEIE | ICR_SPIKE_FIX;
		icr &= ~ICR_STOP;
		icr &= ~ICR_ALDIE;
		writel(icr, _ICR(i2c));

		/*step 3*/
		/*step 4*/
		/*in i2c_pxa_handler(), read isr, clear interrupt bit, and wakeup event*/
		/*Bad! Bad! if we can not receive the response here!*/
		if (i2c->use_pio) {
			local_irq_restore(cpsr);
			polling_delay = 14 * i2c->access2_delay/2; //MIN: 3*8+2 cycles = 26 => use about half for polling
			timeout = (MAX_XFER_TIMEOUT_US/polling_delay)+1; //80mSec
			while(timeout-- && (i2c->hs_enterring != false) )
			{
				i2c_pxa_handler(0, i2c);
				udelay(polling_delay);
			}
		} else {
			/*"wait_event_timeout" is system call wich restores irq itself*/
			timeout = wait_event_timeout(i2c->wait, i2c->hs_enterring == false, MAX_XFER_TIMEOUT_US/1000/8);//78ms
		}
		if (unlikely(i2c->irq_warn != IRQ_WARN_NONE)) {
			if (unlikely(i2c->irq_warn > IRQ_WARN_UNKNOWN))
				i2c_pxa_scream_blue_murder(i2c, i2c_irq_warn_str[IRQ_WARN_UNKNOWN]);
			else
				i2c_pxa_scream_blue_murder(i2c, i2c_irq_warn_str[i2c->irq_warn]);
		}
		i2c->hs_enterring = false;
		if ((i2c->msg_idx == I2C_RETRY) || (i2c->msg_idx == I2C_BUS_ERROR) || (i2c->msg_idx == I2C_MASTER_ABORT))
			goto exit;
	}

	writel(i2c_pxa_addr_byte(i2c->msg), _IDBR(i2c)); /*target slave address into IDBR*/
	/* Bus is FREE, go ahead and start transaction*/
	i2c->debug_track=1;
	/* Do not use read-modify-write way, because we know the right value here
	 * should anything goes wrong inside ICR, overwriting the value will
	 * correct the problem */
	/* icr = readl(_ICR(i2c)); */
	icr |= ICR_START | ICR_TB | ICR_MSDE | ICR_SPIKE_FIX;
	writel(icr, _ICR(i2c));

	if (i2c->use_pio)
	{
		if ((i2c->flags & PXA_I2C_HIGH_MODE) == 0) /*was not restored*/
			local_irq_restore(cpsr);
		polling_delay = 14 * i2c->access2_delay/2; /*MIN: 3*8+2 cycles = 26 => use about half for polling*/
		timeout = (MAX_XFER_TIMEOUT_US/polling_delay)+1; /*80mSec*/
		while(timeout-- && (i2c->msg_num > 0) )
		{
			i2c_pxa_handler(0, i2c);
			udelay(polling_delay);
		}
	}
	else
	{
		/*local_irq_restore(cpsr) do NOT restore,
		the wait_event_timeout() should go into SUSPEND before IRQ-end-event wakes ip this task
		The wait_event_timeout() enables the irq itself
		WORKAROUND:
		Sometimes wake_up() does not wake-up the task with wait_event_timeout()
		In that case the task runs again upon TimeOut but condition is already TRUE.
		Let's use wait_event_timeout() with short (7.8mS is minimum) and go to long TO only if condition still FALSE*/
		timeout = wait_event_timeout(i2c->wait, i2c->msg_num == 0, 1); /*7.8 mSec*/
		if (unlikely(i2c->irq_warn != IRQ_WARN_NONE)) {
			if (unlikely(i2c->irq_warn > IRQ_WARN_UNKNOWN))
				i2c_pxa_scream_blue_murder(i2c, i2c_irq_warn_str[IRQ_WARN_UNKNOWN]);
			else
				i2c_pxa_scream_blue_murder(i2c, i2c_irq_warn_str[i2c->irq_warn]);
		}
		if(i2c->msg_num != 0)
			wait_event_timeout(i2c->wait, i2c->msg_num == 0, MAX_XFER_TIMEOUT_US/1000/8); /*78 mSec*/
	}
exit:
	i2c_pxa_stop_message(i2c);

	/*
	 * We place the return code in i2c->msg_idx.
	 */
	ret = i2c->msg_idx;

	if(i2c->msg_num != 0) {
		i2c_pxa_scream_blue_murder(i2c, "ERROR: XFER timeout - started but never finished");
		ret = I2C_TIMEOUT_DONT_RETRY;
	}
	if (ret == I2C_RETRY)
		udelay(i2c->busbusy_delay);

	return ret;
} /*i2c_pxa_do_xfer*/

/*
 * i2c_pxa_master_complete - complete the message and wake up.
 *
 * NOTE: the FIFO does NOT uses this procedure, but 2..3 lines:
 *			i2c->msg_idx =  error; if needed
 *			i2c->fifo_transaction_down = true;
 *			wake_up(&i2c->wait);
 */
static void i2c_pxa_master_complete(struct pxa_i2c *i2c, int ret)
{
	i2c->msg_ptr = 0;
	i2c->msg = NULL;
	i2c->msg_idx ++;
	i2c->msg_num = 0;
	if (ret)
		i2c->msg_idx = ret;
	if (!i2c->use_pio)
		wake_up(&i2c->wait);
	return;
}

static u32 i2c_pxa_irq_txempty_with_fifo(struct pxa_i2c *i2c, u32 isr)
{
	u32 dbgCntr = 0;
	u16 fifo_write_value;
	u32 icr = readl(_ICR(i2c));
	ICR_LOG = icr;

	if (isr & ISR_ALD) {
		if (isr & ISR_BED) {
			i2c->irq_warn = IRQ_WARN_ALD_BED;
			i2c->msg_idx = I2C_BUS_ERROR;
		} else
		if ((i2c->msg_ptr != 0) || (i2c->msg_idx != 0))
		{
			/* Still not started. Just go to retry */
			i2c->irq_warn = IRQ_WARN_ALD_RETRY;
			i2c->msg_idx = I2C_RETRY;
		} else {
			/* Arbitration on ADDRESS stage should be resolved by HW,
			 * just clear ALD-bit.
			 * But check the master is NOT stopped;
			 * if stopped go to retry
			 */
			if (isr & ISR_MSD) {
				i2c->irq_warn = IRQ_WARN_ALD_ADDR;
				i2c->msg_idx = I2C_MASTER_ABORT;
			} else {
				i2c->irq_warn = IRQ_WARN_ALD_CONT;
				i2c->debug_track = 7;
			}
		}
		writel((ISR_ALD | ISR_BED), _ISR(i2c));

		if (i2c->msg_idx < 0) { /* if error */
			i2c->fifo_transaction_down = true;
			wake_up(&i2c->wait);
			return 0;
		}
	}

	if (isr & ISR_BED) {
		if (!i2c_pxa_custom_nack_is_ok(i2c)) {
			i2c->irq_warn = IRQ_WARN_NO_ANSWER;
			i2c->msg_idx = I2C_BUS_ERROR;
		}
		writel(ISR_BED, _ISR(i2c));
		i2c->fifo_transaction_down = true;
		wake_up(&i2c->wait);
		return 0;
	}

	if (isr & (ISR_RXUN | ISR_RXOV | ISR_RXF | ISR_RXSR)) {

		if (isr & ISR_RXF)
		{
			u32 i;

			if(i2c->msg->flags & I2C_M_RD)
				for(i = 0; ((i < I2C_PXA_FIFO_ENTRY_RX) && (i2c->fifo_read_byte_num < i2c->msg->len));i++)
				{
					i2c->msg->buf[i2c->fifo_read_byte_num++]=readl(_RFIFO(i2c));
					/*printk("ISR_RXF, data[%d] = 0x%01x\n", (i2c->fifo_read_byte_num-1),i2c->msg->buf[i2c->fifo_read_byte_num-1]);*/
				}
			writel(ISR_RXF, _ISR(i2c));

		}
		if (isr & ISR_RXSR)
		{
			u32 i;
			if(i2c->msg->flags & I2C_M_RD)
				for(i = 0; ((i < I2C_PXA_FIFO_TSHLD_RX) && (i2c->fifo_read_byte_num < i2c->msg->len));i++)
				{
					i2c->msg->buf[i2c->fifo_read_byte_num++]=readl(_RFIFO(i2c));
					/*printk("ISR_RXSR, data[%d] = 0x%01x\n", (i2c->fifo_read_byte_num-1),i2c->msg->buf[i2c->fifo_read_byte_num-1]);*/
				}
			writel(ISR_RXSR, _ISR(i2c));

		}
		if (isr & ISR_RXOV)
		{
			writel(ISR_RXOV, _ISR(i2c));
		}
		if (isr & ISR_RXUN)
		{
			writel(ISR_RXUN, _ISR(i2c));
		}
	}

	if (isr & (ISR_TXOV | ISR_TXSR | ISR_TXDONE)) {

		if(isr & ISR_TXDONE)
		{
			i2c->msg_num = 0;
			writel((readl(_ICR(i2c)) & (~(ICR_START | ICR_STOP | ICR_ACKNAK | ICR_TB | ICR_TXSR_IE | ICR_RXSR_IE))), _ICR(i2c));
			writel(0xfffff, _ISR(i2c));

			if(i2c->msg->flags & I2C_M_RD)
				while (i2c->fifo_read_byte_num < i2c->msg->len)
				{
					i2c->msg->buf[i2c->fifo_read_byte_num++]=readl(_RFIFO(i2c));
					/*printk("ISR_TXDONE, data[%d] = 0x%01x\n", (i2c->fifo_read_byte_num-1),i2c->msg->buf[i2c->fifo_read_byte_num-1]);*/
				}
			i2c->fifo_transaction_down = true;
			wake_up(&i2c->wait);
			return 0;
		}

		if(isr & ISR_TXOV)
		{
			writel(ISR_TXOV, _ISR(i2c));
		}
		if(isr & ISR_TXSR)
		{
			/*use default fifo threshold value.*/
			i2c->fifo_write_byte_num = 0;
			/*write address*/
			if((i2c->msg_ptr == 0) && (i2c->msg_idx == 0))
			{

				if (i2c->flags & PXA_I2C_HIGH_MODE) {

					icr = readl(_ICR(i2c));
					icr |= (1 << 16);  /* bit_15 clear or set already done by reset*/

					/*step 1*/
					icr |= ICR_GPIOEN;
					writel(icr, _ICR(i2c));

					fifo_write_value = readl(_WFIFO(i2c));
					fifo_write_value |= (ICR_START | ICR_TB) << 8;
					fifo_write_value &= ~((ICR_STOP | ICR_ACKNAK) << 8);
					fifo_write_value |= i2c->master_code;
					writel(fifo_write_value, _WFIFO(i2c));
					i2c->fifo_write_byte_num++;
				}

				fifo_write_value = readl(_WFIFO(i2c));
				fifo_write_value |= (ICR_START | ICR_TB) << 8;
				fifo_write_value &= ~((ICR_STOP | ICR_ACKNAK) << 8);
				fifo_write_value |= i2c_pxa_addr_byte(i2c->msg);
				writel(fifo_write_value, _WFIFO(i2c));
				i2c->fifo_write_byte_num++;
			}

			while (1)
			{
			/*again:*/
				dbgCntr++;
				if(!(i2c->msg->flags & I2C_M_RD) && (i2c->fifo_write_byte_num < I2C_PXA_FIFO_ENTRY_TX))
				{
					if(i2c->msg_ptr < i2c->msg->len)
					{
						if(i2c->msg->buf == NULL)
						{
							i2c_pxa_scream_blue_murder(i2c, "NULL buffer for TX");
							i2c->msg_idx = I2C_NULL_BUFFER;
							i2c->fifo_transaction_down = true;
							wake_up(&i2c->wait);
							return dbgCntr;
						}
						fifo_write_value = readl(_WFIFO(i2c));
						fifo_write_value |= (ICR_TB) << 8;
						fifo_write_value &= ~((ICR_START | ICR_STOP | ICR_ACKNAK) << 8);

						fifo_write_value |= i2c->msg->buf[i2c->msg_ptr++];

						if ((i2c->msg_ptr == i2c->msg->len) && i2c->msg_idx == i2c->msg_num -1)
						{
							fifo_write_value |= (ICR_STOP) << 8;

							writel(fifo_write_value, _WFIFO(i2c));
							i2c->fifo_write_byte_num++;
							writel((readl(_ICR(i2c)) & (~(ICR_TXSR_IE))), _ICR(i2c));
							icr = readl(_ICR(i2c));

							break;
						}

						writel(fifo_write_value, _WFIFO(i2c));
						i2c->fifo_write_byte_num++;
					} else if (i2c->msg_idx < i2c->msg_num - 1) {
						/*next segment og the message */
						i2c->msg_ptr = 0;
						i2c->msg_idx ++;
						i2c->msg++;

						/* if we aren't doing a repeat start and address,
						 * go back and try to send the next byte. Note that
						 * we don't support switching the R/W direction here.
						 */
						if(i2c->msg->flags & I2C_M_NOSTART)
							continue; /*goto again;*/
						/*write the next address and trigger a repeat start*/
						fifo_write_value = readl(_WFIFO(i2c));
						fifo_write_value |= (ICR_START | ICR_TB) << 8;
						fifo_write_value &= ~((ICR_STOP | ICR_ACKNAK) << 8);
						fifo_write_value |= i2c_pxa_addr_byte(i2c->msg);
						writel(fifo_write_value, _WFIFO(i2c));
						i2c->fifo_write_byte_num++;

					} else {
						if(i2c->msg->len == 0)
						{
							/* device probes have a message length of zero
							 * and need the bus to be reset before it can be used again
							 */
							i2c_pxa_unit_restart(i2c, 1);
							i2c->debug_track = 51;
						} else {
							i2c->debug_track = 52;
							return dbgCntr;
						}
					}

				} else if((i2c->msg->flags & I2C_M_RD) && (i2c->fifo_write_byte_num < I2C_PXA_FIFO_ENTRY_TX)) {

					if(i2c->msg->buf == NULL)
					{
						i2c_pxa_scream_blue_murder(i2c, "NULL buffer for RX");
						i2c->msg_idx = I2C_NULL_BUFFER;
						i2c->fifo_transaction_down = true;
						wake_up(&i2c->wait);
						return dbgCntr;
					}
					if(i2c->msg_ptr < i2c->msg->len)
					{
						/*cmd+dummy data*/
						fifo_write_value = readl(_WFIFO(i2c));
						fifo_write_value |= (ICR_TB) << 8;
						fifo_write_value &= ~((ICR_START | ICR_STOP | ICR_ACKNAK) << 8);
						if (i2c->msg_ptr == i2c->msg->len - 1)
							fifo_write_value |= (ICR_STOP | ICR_ACKNAK) << 8;

						writel(fifo_write_value, _WFIFO(i2c));
						i2c->fifo_write_byte_num++;

						if (i2c->msg_ptr == i2c->msg->len - 1)
						{
							i2c->msg_ptr++;
							writel((readl(_ICR(i2c)) & (~(ICR_TXSR_IE))), _ICR(i2c));
							icr = readl(_ICR(i2c));

							break;
						}
						i2c->msg_ptr++;
					}
					else
					{
						/*printk("surplus txsr irq\n");*/
						break;
					}
				} else {
					break;
				}
			}
			writel(ISR_TXSR, _ISR(i2c));

		} /*while(1)*/
	} /*if (isr & (ISR_TXOV | ISR_TXSR | ISR_TXDONE))*/

	return dbgCntr;
} /*i2c_pxa_irq_txempty_with_fifo*/

static void i2c_pxa_irq_txempty_with_dma(struct pxa_i2c *i2c, u32 isr)
{
	u32 icr = readl(_ICR(i2c));
	ICR_LOG = icr;

	if(isr & (ISR_ALD)){
		i2c->irq_warn = IRQ_WARN_ALD;
		writel(ISR_ALD, _ISR(i2c));

		i2c->msg_idx = I2C_MASTER_ABORT;
		i2c->fifo_transaction_down = true;
		wake_up(&i2c->wait);
		return;
	}
	if (isr & ISR_BED) {
		if (!i2c_pxa_custom_nack_is_ok(i2c))
		{
			i2c->irq_warn = IRQ_WARN_NO_ANSWER;
			i2c->msg_idx = I2C_BUS_ERROR;
		}
		writel(ISR_BED, _ISR(i2c));
		i2c->fifo_transaction_down = true;
		wake_up(&i2c->wait);
		return;

	}

	if(isr & ISR_TXDONE)
	{
		i2c->msg_num = 0;
		/*printk("TXDONE\n");*/
		/*stop DMA channel*/
		DCSR(i2c->dma_chans[0]) &= ~DCSR_RUN;
		if (i2c->have_read_operation)
			DCSR(i2c->dma_chans[1]) &= ~DCSR_RUN;
		i2c->have_read_operation = false;
		writel(0xfffff, _ISR(i2c));
		i2c->fifo_transaction_down = true;
		wake_up(&i2c->wait);
		return;
	}
	return;
}

static void i2c_pxa_irq_txempty(struct pxa_i2c *i2c, u32 isr)
{
	u32 icr = readl(_ICR(i2c));
	ICR_LOG = icr;

	if (isr & ISR_ALD) {
		if (isr & ISR_BED) {
			i2c->irq_warn = IRQ_WARN_ALD_BED;
			writel((ISR_ALD | ISR_BED), _ISR(i2c));
			i2c_pxa_master_complete(i2c, I2C_BUS_ERROR);
			return;
		}
		if ((i2c->msg_ptr != 0) || (i2c->msg_idx != 0))
		{
			DBG_GPIO_ERR_ON();
			i2c->irq_warn = IRQ_WARN_ALD_RETRY;
			DBG_GPIO_ERR_OFF();
			i2c_pxa_master_complete(i2c, I2C_RETRY);
		} else {
			/* Arbitration on ADDRESS stage should be resolved by HW,
			 * just clear ALD-bit.
			 * But check the master is NOT stopped;
			 * if stopped go to retry
			 */
			if (isr & ISR_MSD) {
				i2c->irq_warn = IRQ_WARN_ALD_ADDR;
				i2c_pxa_master_complete(i2c, I2C_MASTER_ABORT);
			} else {
				i2c->debug_track=7;
			}
		}
		writel(ISR_ALD, _ISR(i2c));
		return;
	}
	if (isr & ISR_BED) {
		/*
		 * I2C bus error - either the device NAK'd us, or
		 * something more serious happened.  If we were NAK'd
		 * on the initial ADDRESS phase, we can retry.
		 */
		if (i2c->msg_ptr == 0 && i2c->msg_idx == 0)
		{
			DBG_GPIO_ERR_ON();
			i2c->irq_warn = IRQ_WARN_NO_ANSWER;
			DBG_GPIO_ERR_OFF();
		}
		else {
			if (i2c_pxa_custom_nack_is_ok(i2c)) {
				writel(ISR_BED, _ISR(i2c));
		        i2c_pxa_master_complete(i2c, 0);
				return;
			}
			i2c->irq_warn = IRQ_WARN_BUS_ERR;
		}
		writel(ISR_BED, _ISR(i2c));
		i2c_pxa_master_complete(i2c,I2C_BUS_ERROR);
		return;
	}

	icr &= ~(ICR_START|ICR_STOP|ICR_ACKNAK|ICR_TB);

again:
	/* for "if ((!i2c->hs_enterring))", if enable hs mode, only after i2c bus
	* has sent out master code, then can send the sequencial data, if do
	* not enable hs mode, hs_enterring is always "false", do not influence
	* the fast/standard mode transmitting sequence.
	*/
	if (!i2c->hs_enterring) {
		if (isr & ISR_RWM) {
			/*
			* Read mode.  We have just sent the address byte, and
			* now we must initiate the transfer.
			*/
			if (i2c->msg_ptr == i2c->msg->len - 1 &&
					i2c->msg_idx == i2c->msg_num - 1)
				icr |= ICR_STOP | ICR_ACKNAK;

			icr |= ICR_ALDIE | ICR_TB;
			i2c->debug_track=2;

		} else if (i2c->msg_ptr < i2c->msg->len) {
			/*
			 * Write mode.  Write the next data byte.
			 */
			if(i2c->msg->buf == NULL) {
				i2c_pxa_scream_blue_murder(i2c, "NULL buffer for TX");
				i2c_pxa_master_complete(i2c, I2C_NULL_BUFFER);
				return;
			}
			writel(i2c->msg->buf[i2c->msg_ptr++], _IDBR(i2c));

			icr |= ICR_ALDIE | ICR_TB;

			/*
			 * If this is the last byte of the last message, send
			 * a STOP.
			 */
			if (i2c->msg_ptr == i2c->msg->len &&
					i2c->msg_idx == i2c->msg_num - 1)
				icr |= ICR_STOP;
			i2c->debug_track=3;
		} else if (i2c->msg_idx < i2c->msg_num - 1) {
			/*
			 * Next segment of the message.
			 */
			i2c->msg_ptr = 0;
			i2c->msg_idx ++;
			i2c->msg++;

			/*
			 * If we aren't doing a repeated start and address,
			 * go back and try to send the next byte.  Note that
			 * we do not support switching the R/W direction here.
			 */
			if (i2c->msg->flags & I2C_M_NOSTART)
				goto again;

			/*
			 * Write the next address.
			 */
			writel(i2c_pxa_addr_byte(i2c->msg), _IDBR(i2c));

			/*
			 * And trigger a repeated start, and send the byte.
			 */
			icr &= ~ICR_ALDIE;
			/* Master stop detected enabled */
			icr |= ICR_START | ICR_TB | ICR_MSDE | ICR_SPIKE_FIX;
			i2c->debug_track=4;
		} else {
			if (i2c->msg->len == 0) {
				/*
				 * Device probes have a message length of zero
				 * and need the bus to be reset before it can
				 * be used again.
				 */
				i2c_pxa_unit_restart(i2c, 1);
				i2c->debug_track = 51;
			}
			else
				i2c->debug_track = 52;
			i2c_pxa_master_complete(i2c, 0);
		}
	}
	writel(icr, _ICR(i2c));
	show_state(i2c);
}

static void i2c_pxa_irq_rxfull(struct pxa_i2c *i2c, u32 isr)
{
	u32 icr = readl(_ICR(i2c)) & ~(ICR_START|ICR_STOP|ICR_ACKNAK|ICR_TB);
	ICR_LOG = icr;

	i2c->debug_track=6;
	/*
	 * Read the byte.
	 */
	if(i2c->msg->buf == NULL) {
		i2c_pxa_scream_blue_murder(i2c, "NULL buffer for RX");
		i2c_pxa_master_complete(i2c, I2C_NULL_BUFFER);
		return;
	}
	i2c->msg->buf[i2c->msg_ptr++] = readl(_IDBR(i2c));

	if (i2c->msg_ptr < i2c->msg->len) {
		/*
		 * If this is the last byte of the last
		 * message, send a STOP.
		 */
		if (i2c->msg_ptr == i2c->msg->len - 1)
			icr |= ICR_STOP | ICR_ACKNAK;

		icr |= ICR_ALDIE | ICR_TB;
		i2c->debug_track=61;
	} else {
		i2c->debug_track=62;
		i2c_pxa_master_complete(i2c, 0);
	}

	writel(icr, _ICR(i2c));
}

static irqreturn_t i2c_pxa_handler(int this_irq, void *dev_id)
{
	struct pxa_i2c *i2c = dev_id;
	u32 isr, dbgCntr = 0;

	CUSTOM_BYTE2BYTE_DELAY(i2c);

	DBG_GPIO_IRQH_IN();
	isr = readl(_ISR(i2c));
	ISR_LOG = isr;

/****
	if (i2c_debug > 2 && 0) {
		dev_dbg(&i2c->adap.dev, "%s: ISR=%08x, ICR=%08x, IBMR=%02x\n",
		__func__, isr, readl(_ICR(i2c)), readl(_IBMR(i2c)));
		decode_ISR(isr);
	}

	show_state(i2c);
*****/

	if (isr & ISR_SAD) {
		writel(ISR_SAD, _ISR(i2c));
		i2c_pxa_scream_blue_murder(i2c, "spurious SLAVE irq (ISR_SAD=1)");
		i2c_pxa_slave_start(i2c, isr);
	}
	if (isr & ISR_SSD) {
		writel(ISR_SSD, _ISR(i2c));
		i2c_pxa_scream_blue_murder(i2c, "spurious SLAVE irq (ISR_SSD=1)");
		i2c_pxa_slave_stop(i2c);
	}

	if (i2c_pxa_is_slavemode(i2c)) {
		i2c_pxa_scream_blue_murder(i2c, "spurious SLAVE irq");
		if (isr & ISR_ITE) {
			writel(ISR_ITE, _ISR(i2c));
			i2c_pxa_slave_txempty(i2c, isr);
		}
		if (isr & ISR_IRF) {
			writel(ISR_IRF, _ISR(i2c));
			i2c_pxa_slave_rxfull(i2c, isr);
		}
	} else if (i2c->msg) {
		if (i2c->flags & PXA_I2C_USING_FIFO_DMA_MODE) {
			i2c_pxa_irq_txempty_with_dma(i2c, isr);
		} else if (i2c->flags & PXA_I2C_USING_FIFO_PIO_MODE) {
			dbgCntr = i2c_pxa_irq_txempty_with_fifo(i2c, isr);
		} else {
			if (isr & (ISR_ITE | ISR_ALD | ISR_BED)) {
				writel(ISR_ITE, _ISR(i2c));
				i2c_pxa_irq_txempty(i2c, isr);
				/*if enable hs mode, sending maste code is done here,then wake-up*/
				if (i2c->hs_enterring)
				{
					i2c->hs_enterring = false;
					if (!i2c->use_pio)
					{
						wake_up(&i2c->wait);
					}
				}
			}
			if (isr & ISR_IRF) {
				writel(ISR_IRF, _ISR(i2c));
				i2c_pxa_irq_rxfull(i2c, isr);
			}
		}
	} else {
		i2c_pxa_scream_blue_murder(i2c, "spurious irq");
		i2c_pxa_master_complete(i2c, I2C_NULL_BUFFER);
	}

	DBG_GPIO_IRQH_OUT();
	return IRQ_HANDLED;
} /*i2c_pxa_handler*/

/**
 * pxa_dma_start_channels - start DMA channel for active buffer
 * @i2c: pxa i2c device
 *
 * Initialize DMA channels to the beginning of current transaction, and
 * start these channels.
 */

static void pxa_dma_start_channels(struct pxa_i2c *i2c, enum pxa_i2c_active_dma act_channel, int xfer_len)
{

	if (act_channel == DMA_W)
	{
		if (!(DCSR(i2c->dma_chans[0]) & DCSR_STOPSTATE))
			return;

		DCSR(i2c->dma_chans[0])  = DCSR_NODESC;

		if (i2c->adap.nr == 0)
		{
			DTADR(i2c->dma_chans[0]) = 0x403016c0; /*I2C1 WFIFO address*/
		}
		else if (i2c->adap.nr == 1)
		{
			DTADR(i2c->dma_chans[0]) = 0x404016c0; /*I2C2 WFIFO address*/
		}
		else if (i2c->adap.nr == 2)
		{
			DTADR(i2c->dma_chans[0]) = 0x408016c0; /*I2C3 WFIFO address*/
		}

		DSADR(i2c->dma_chans[0]) = i2c->dma_handles_write;
		DCMD(i2c->dma_chans[0]) = DCMD_INCSRCADDR | DCMD_FLOWTRG | DCMD_ENDIRQEN | DCMD_WIDTH2 | DCMD_BURST16 | (2 * xfer_len);
		DCSR(i2c->dma_chans[0]) |= DCSR_RUN;

	}
	else if (act_channel == DMA_R)
	{
		DCSR(i2c->dma_chans[1])  = DCSR_NODESC;
		if (i2c->adap.nr == 0)
		{
			DSADR(i2c->dma_chans[1]) = 0x403016d0; /*I2C1 RFIFO address*/
		}
		else if (i2c->adap.nr == 1)
		{
			DSADR(i2c->dma_chans[1]) = 0x404016d0; /*I2C2 RFIFO address*/
		}
		else if (i2c->adap.nr == 2)
		{
			DTADR(i2c->dma_chans[1]) = 0x408016d0; /*I2C3 RFIFO address*/
		}

		DTADR(i2c->dma_chans[1]) = i2c->dma_handles_read;
		DCMD(i2c->dma_chans[1]) = DCMD_INCTRGADDR | DCMD_FLOWSRC | DCMD_ENDIRQEN | DCMD_WIDTH1 | DCMD_BURST8 | (1 * xfer_len);
		DCSR(i2c->dma_chans[1]) |= DCSR_RUN;

	}

}

static void pxa_i2c_dma_irq(int channel, struct pxa_i2c *i2c,
				enum pxa_i2c_active_dma act_dma)
{
	volatile unsigned long dcsr;
	dcsr = DCSR(channel);
	DCSR(channel) &= ~DCSR_RUN;

	if (dcsr & DCSR_BUSERR) {
		DCSR(channel) |= DCSR_BUSERR;
		printk("%s(): DMA channel bus error\n", __func__);
	}

	if ((dcsr & DCSR_ENDINTR) || (dcsr & DCSR_STOPSTATE)) {
		if (dcsr & DCSR_STOPSTATE) {
			/*printk("DCSR_STOPSTATE, channel:%d\n",channel);*/
			DCSR(channel) &= ~DCSR_STOPSTATE;
		}

		if (dcsr & DCSR_ENDINTR) {
			/*printk("DCSR_ENDINTR, channel:%d\n",channel);*/
			DCSR(channel) |= DCSR_ENDINTR;

		}
	}
	return;
}

static int i2c_pxa_do_xfer_with_dma(struct pxa_i2c *i2c, struct i2c_msg *msg, int num)
{
	unsigned long cpsr;
	u32 icr = 0, isr, ibmr;
	u16 fifo_write_value;
	u32 retry;
	long timeout;
	int ret;

	int xfer_len = 0;
	int read_xfer_len = 0;
	unsigned short	*pwrite_buf = (unsigned short *)i2c->write_buffer;

	i2c->msg = msg;
	i2c->msg_num = num;
	i2c->msg_idx = 0;
	i2c->msg_ptr = 0;
	i2c->fifo_write_byte_num = 0;
	i2c->fifo_read_byte_num = 0;
	i2c->irq_warn = IRQ_WARN_NONE;

	writel(0, _WFIFO_WPTR(i2c));
	writel(0, _WFIFO_RPTR(i2c));
	writel(0, _RFIFO_WPTR(i2c));
	writel(0, _RFIFO_RPTR(i2c));

	/*printk("i2c_pxa_do_xfer_with_dma\n");*/

	i2c->have_read_operation = false;

	/* Wait for bus free: check twice for the Bus status bits and IBMR to be sure.
	 * This still can't guaranty "no race condition with other master" but makes best efforts.
	 * If decided the bus is IDLE
	 *   do NOT enable ICR_ALDIE so possible arbitration will be fixed by HW
	 **/
	retry = 2;
	do
	{
		retry--;
		local_irq_save(cpsr);
		ibmr = readl(_IBMR(i2c));
		isr  = readl(_ISR(i2c));
		IBMR_LOG = ibmr;
		ISR_LOG  = isr;
		if( (ibmr != I2C_IBMR_IDLE) || (isr & (ISR_EBB | ISR_IBB | ISR_UB)) ) {
			local_irq_restore(cpsr);
			udelay(i2c->access2_delay);
			/* Do not print. BUS-BUSY is valid case in multi-mastering */
			/*printk(KERN_ERR "i2c: bus busy, go to retry, isr 0x%x\n",readl(_ISR(i2c)));*/
			return I2C_RETRY;
		}
		if(retry == 0)
			break;

		local_irq_restore(cpsr);

		/*"Delay" and Re-Check Bus IDLE*/
		icr = readl(_ICR(i2c));
		ICR_LOG = icr;
		isr |= I2C_ISR_CLEAR;
		writel(isr, _ISR(i2c));

	}while(retry>0);

	if (i2c->flags & PXA_I2C_HIGH_MODE) {
		icr = readl(_ICR(i2c));
		icr |= (1 << 16);

		i2c->access2_delay = I2C_HS_DELAY_US;

		/*step 1*/
		icr |= ICR_GPIOEN;
		writel(icr, _ICR(i2c));

		fifo_write_value = readl(_WFIFO(i2c));
		fifo_write_value |= (ICR_START | ICR_TB) << 8;
		fifo_write_value &= ~((ICR_STOP | ICR_ACKNAK) << 8);
		fifo_write_value |= i2c->master_code;
		*(pwrite_buf++) = fifo_write_value;
		xfer_len++;

	}

		fifo_write_value = readl(_WFIFO(i2c));
		fifo_write_value |= (ICR_START | ICR_TB) << 8;
		fifo_write_value &= ~((ICR_STOP | ICR_ACKNAK) << 8);
		fifo_write_value |= i2c_pxa_addr_byte(i2c->msg);
		*(pwrite_buf++) = fifo_write_value;
		xfer_len++;

		while (1)
		{
		/*again:*/
			/*
			 *currently a PAGE_SIZE memory allocated for DMA operation. we think it is enough for all use cases.
			 *If a transaction needs to transfer more than one page size data, it can be split serveral transactions,
			 *or modify the code to allocate more memory dynamically.
			 */
			assert(xfer_len < PAGE_SIZE);

			if(!(i2c->msg->flags & I2C_M_RD))
			{
				if(i2c->msg_ptr < i2c->msg->len)
				{
					if(i2c->msg->buf == NULL)
					{
						i2c_pxa_scream_blue_murder(i2c, "NULL buffer for TX");
						i2c_pxa_unit_restart(i2c, 2);
						return I2C_NULL_BUFFER;
					}
					fifo_write_value = readl(_WFIFO(i2c));
					fifo_write_value |= (ICR_TB) << 8;
					fifo_write_value &= ~((ICR_START | ICR_STOP | ICR_ACKNAK) << 8);
					fifo_write_value |= i2c->msg->buf[i2c->msg_ptr++];

					if ((i2c->msg_ptr == i2c->msg->len) && i2c->msg_idx == i2c->msg_num -1)
					{
						fifo_write_value |= (ICR_STOP) << 8;

						*(pwrite_buf++) = fifo_write_value;
						xfer_len++;

						break;
					}

					*(pwrite_buf++) = fifo_write_value;
					xfer_len++;

				} else if (i2c->msg_idx < i2c->msg_num - 1) {
					/*next segment og the message */
					i2c->msg_ptr = 0;
					i2c->msg_idx ++;
					i2c->msg++;

					/* if we aren't doing a repeat start and address,
					 * go back and try to send the next byte. Note that
					 * we don't support switching the R/W direction here.
					 */
					if(i2c->msg->flags & I2C_M_NOSTART)
						continue; /*goto again;*/
					/*write the next address and trigger a repeat start*/
					fifo_write_value = readl(_WFIFO(i2c));
					fifo_write_value |= (ICR_START | ICR_TB) << 8;
					fifo_write_value &= ~((ICR_STOP | ICR_ACKNAK) << 8);
					fifo_write_value |= i2c_pxa_addr_byte(i2c->msg);
					*(pwrite_buf++) = fifo_write_value;
					xfer_len++;

				} else {
					if(i2c->msg->len == 0)
					{
						/*device probes have a message length of zero
						 * and need the bus to be reset before it can be used again
						 */
						i2c_pxa_unit_restart(i2c, 2);
						i2c->debug_track = 51;
					} else {
						i2c->debug_track = 52;
						return (I2C_RETRY);
					}
				}

			} else if(i2c->msg->flags & I2C_M_RD) {
				i2c->have_read_operation = true;
				read_xfer_len = i2c->msg->len;

				if(i2c->msg->buf == NULL)
				{
					i2c_pxa_scream_blue_murder(i2c, "NULL buffer for RX");
					i2c_pxa_unit_restart(i2c, 2);
					return (I2C_RETRY);
				}
				if(i2c->msg_ptr < i2c->msg->len)
				{
					/*cmd+dummy data*/
					fifo_write_value = readl(_WFIFO(i2c));
					fifo_write_value |= (ICR_TB) << 8;
					fifo_write_value &= ~((ICR_START | ICR_STOP | ICR_ACKNAK) << 8);
					if (i2c->msg_ptr == i2c->msg->len - 1)
						fifo_write_value |= (ICR_STOP | ICR_ACKNAK) << 8;

					*(pwrite_buf++) = fifo_write_value;
					xfer_len++;

					if (i2c->msg_ptr == i2c->msg->len - 1)
						break;
					i2c->msg_ptr++;
				}
				else
				{
					break;
				}
			} else {
				break;
			}
	}

	/* Do not use read-modify-write way, because we know the right value here
	 * should anything goes wrong inside ICR, overwriting the value will
	 * correct the problem */
	/* icr = readl(_ICR(i2c)); */

	icr |= ICR_FIFOEN | ICR_DMA_EN | ICR_TXDONE_IE | ICR_SPIKE_FIX;
	icr &= ~(ICR_TXSR_IE | ICR_RXSR_IE);

	icr &= ~ (ICR_ITEIE | (1 << 9));
	writel(icr, _ICR(i2c));

	pxa_dma_start_channels(i2c, DMA_W, xfer_len);
	if(i2c->have_read_operation)
		pxa_dma_start_channels(i2c, DMA_R,read_xfer_len);

	i2c->fifo_transaction_down = false;

	/*local_irq_restore(cpsr) do NOT restore,
	 *the wait_event_timeout() should go into SUSPEND before IRQ-end-event wakes ip this task
	 *The wait_event_timeout() enables the irq itself
	 *WORKAROUND:
	 *Sometimes wake_up() does not wake-up the task with wait_event_timeout()
	 *In that case the task runs again upon TimeOut but condition is already TRUE.
	 *Let's use wait_event_timeout() with short (7.8mS is minimum) and go to long TO only if condition still FALSE*/

	timeout = wait_event_timeout(i2c->wait, i2c->fifo_transaction_down == true, 1); /*7.8 mSec*/

	if(!i2c->fifo_transaction_down)
	{
		timeout = wait_event_timeout(i2c->wait, i2c->fifo_transaction_down == true,  MAX_XFER_TIMEOUT_US/1000/8+10); /*78 mSec*/
	}
	if (unlikely(i2c->irq_warn != IRQ_WARN_NONE)) {
		if (unlikely(i2c->irq_warn > IRQ_WARN_UNKNOWN))
			i2c_pxa_scream_blue_murder(i2c, i2c_irq_warn_str[IRQ_WARN_UNKNOWN]);
		else
			i2c_pxa_scream_blue_murder(i2c, i2c_irq_warn_str[i2c->irq_warn]);
	}
	ret = i2c->msg_idx;
	if(ret >= 0)
	{
		if(!i2c->fifo_transaction_down) {
			i2c_pxa_scream_blue_murder(i2c, "ERROR: XFER timeout - started but never finished");
			ret = I2C_TIMEOUT_DONT_RETRY;
		}
		else
		{
			i2c->msg = msg;

			for(i2c->msg_idx = 0; i2c->msg_idx < num; i2c->msg_idx++)
			{
				if(i2c->msg->flags & I2C_M_RD)
				{
					for(i2c->msg_ptr = ((i2c->msg->len/8)*8); i2c->msg_ptr < i2c->msg->len;)
					{
						*((unsigned char *)i2c->read_buffer+i2c->msg_ptr) = readl(_RFIFO(i2c));
						i2c->msg_ptr++;
					}
					memcpy(&i2c->msg->buf[0], i2c->read_buffer, i2c->msg->len);

				}
				i2c->msg++;
			}

			i2c->msg_ptr = 0;
			i2c->msg = NULL;
			i2c->msg_idx++;
			i2c->msg_num = 0;
		}
	}

	if (ret == I2C_RETRY)
		udelay(i2c->busbusy_delay);
	return ret;
}

static void pxa_i2c_dma_irq_w(int channel, void *data)
{
	struct pxa_i2c *pdev = data;
	pxa_i2c_dma_irq(channel, pdev, DMA_W);
}

static void pxa_i2c_dma_irq_r(int channel, void *data)
{
	struct pxa_i2c *pdev = data;
	pxa_i2c_dma_irq(channel, pdev, DMA_R);
}

static int i2c_pxa_xfer(struct i2c_adapter *adap, struct i2c_msg msgs[], int num)
{
	#define NUM_XFER_RETRIES  1
	struct pxa_i2c *i2c = adap->algo_data;
	int ret, i;
	int exh_retries = 0, xfer_retries = NUM_XFER_RETRIES;
	unsigned int use_pio	= i2c->use_pio; /* keep original PIO/IRQ*/
	int flags		= i2c->flags;   /* keep original flags */

	if (num <= 0)
		BUG();

	/* keep original modes (PIO/IRQ and flags) */
	use_pio = i2c->use_pio;
	flags   = i2c->flags;
	if (in_interrupt() || in_atomic()) {
		printk(KERN_ERR "i2c_pxa_xfer(%d) called in interrupt or atomic context\n",
			i2c->adap.nr);
		WARN_ON((i2c->dbg_trace != '0') && (i2c->dbg_trace != 0));
		/* force to use POLLING and non FIFO/DMA mode */
		i2c->use_pio = 1;
		i2c->flags &= ~(PXA_I2C_USING_FIFO_DMA_MODE | PXA_I2C_USING_FIFO_PIO_MODE);
	}

	/* Do not config ACCR1 frequency here; it is done by i2c_pxa_reset()*/

#if defined(DEBUG_ERR_SIMULATE)
	i2c_pxa_buserr_simulate(msgs, num);
#endif
	i2c_pxa_msg_parse(i2c, msgs, num);

	/* Special parse actions - spy or another... */
	if (i2c_pxa_custom_spy_reg(i2c, msgs, num)) {
		//WARN_ON(1);
		BUG_ON(1);
		return 0;
	}

	if (i2c->dbg_trace == 'a') {
		printk(KERN_ERR "\n I2C trace: ");
		i2c_pxa_msg_print(i2c, msgs, num); /* print once only */
	}

	set_dvfm_constraint(i2c);
	clk_enable(i2c->clk);   //Possible but not must

	ENTRY_CNTR++;
	DBG_GPIO_CFG();
	DBG_GPIO_DRV_IN();

	i2c_pxa_unit_restart(i2c, 0); /*Reset since we could be after D2*/

do_xfer_retry:
	i2c->debug_track=0;
	/* Unit on, Clock Master Mode, Fast/Slow mode...*/
	/*writel(i2c->icr_backup, _ICR(i2c));*/
	/* i2c_pxa_custom_clock_adj(i2c) may modify the clock*/
	writel(i2c_pxa_custom_clock_adj(i2c), _ICR(i2c));
	udelay(I2C_ACCESS2_DELAY_US/2);  /*WORKAROUND: wait a bit the clock is configured and stable*/

	while(1)
	{
		for (i = adap->retries; i >= 0; i--) {  /* NUM_RETRIES_ON_BUS_BUSY*/
			if (i2c->flags & PXA_I2C_USING_FIFO_DMA_MODE) {
				ret = i2c_pxa_do_xfer_with_dma(i2c, msgs, num);
			} else if (i2c->flags & PXA_I2C_USING_FIFO_PIO_MODE) {
				ret = i2c_pxa_do_xfer_with_fifo(i2c, msgs, num);
			} else {
				ret = i2c_pxa_do_xfer(i2c, msgs, num);
			}
			if (ret == I2C_RETRY)
				continue;	/*udelay() done in the i2c_pxa_do_xfer()*/
			goto out;
		}
		if (exh_retries++ < NUM_RETRIES_EXHAUSTED) {
			msleep(1); /* about 7.8 mS but non-blocking */
		} else{
			printk(KERN_ERR "i2c: ERROR: Bus Busy exhausted retries on Entry=%d  (ISR_%04x IBMR_%02x)\n",
			                            ENTRY_CNTR, readl(_ISR(i2c)), readl(_IBMR(i2c)) );
			ret = -EREMOTEIO;
			break;
		}
	}
out:
	if(ret<0)
	{
		if (xfer_retries == NUM_XFER_RETRIES)
			i2c_pxa_msg_print(i2c, msgs, num); /* print once only */

		i2c_pxa_unit_restart(i2c, 2);
		if(xfer_retries>0)
		{
			xfer_retries--;
			goto do_xfer_retry;
		}
		else
			ret = -EREMOTEIO;
	}
	else
		i2c_pxa_unit_restart(i2c, 0);

	i2c_pxa_msg_save(i2c, msgs, num);

	clk_disable(i2c->clk);   //Possible but not must
	unset_dvfm_constraint(i2c);
	DBG_GPIO_DRV_OUT();

	if (i2c->err_extend_trace) {
		i2c->err_extend_trace = 0;
		WARN_ON(1);
	}

	/* restore original flags and PIO/IRQ mode */
	i2c->use_pio = flags;
	i2c->use_pio = use_pio;
	return ret;
}

static u32 i2c_pxa_functionality(struct i2c_adapter *adap)
{
	return I2C_FUNC_I2C | I2C_FUNC_SMBUS_EMUL;
}

static const struct i2c_algorithm i2c_pxa_algorithm = {
	.master_xfer	= i2c_pxa_xfer,
	.functionality	= i2c_pxa_functionality,
};

static const struct i2c_algorithm i2c_pxa_pio_algorithm = {
	.master_xfer	= i2c_pxa_xfer,
	.functionality	= i2c_pxa_functionality,
};

extern void *dma_alloc_coherent(struct device *dev, size_t size,
				dma_addr_t *handle, gfp_t gfp);
extern void dma_free_coherent(struct device *dev, size_t size,
				void *cpu_addr, dma_addr_t handle);

static int i2c_pxa_probe(struct platform_device *dev)
{
	struct pxa_i2c *i2c;
	struct resource *res;
	struct i2c_pxa_platform_data *plat = dev->dev.platform_data;
	const struct platform_device_id *id = platform_get_device_id(dev);
	int ret;
	int irq;
	int err = 0;

	res = platform_get_resource(dev, IORESOURCE_MEM, 0);
	irq = platform_get_irq(dev, 0);
	if (res == NULL || irq < 0)
		return -ENODEV;

	if (!request_mem_region(res->start, resource_size(res), res->name))
		return -ENOMEM;

	i2c = kzalloc(sizeof(struct pxa_i2c), GFP_KERNEL);
	if (!i2c) {
		ret = -ENOMEM;
		goto emalloc;
	}

	i2c->adap.owner   = THIS_MODULE;

	init_waitqueue_head(&i2c->wait);

	/*
	 * If "dev->id" is negative we consider it as zero.
	 * The reason to do so is to avoid sysfs names that only make
	 * sense when there are multiple adapters.
	 */
	i2c->adap.nr = dev->id != -1 ? dev->id : 0;
	snprintf(i2c->adap.name, sizeof(i2c->adap.name), "pxa_i2c-i2c.%u",
			i2c->adap.nr);

	i2c->clk = clk_get(&dev->dev, "I2CCLK");
	if (IS_ERR(i2c->clk)) {
		ret = PTR_ERR(i2c->clk);
		goto eclk;
	}

	i2c->reg_base = ioremap(res->start, resource_size(res));
	if (!i2c->reg_base) {
		ret = -EIO;
		goto eremap;
	}
	i2c->reg_shift = REG_SHIFT(id->driver_data);

	i2c->iobase = res->start;
	i2c->iosize = resource_size(res);

	i2c->irq = irq;

	i2c->slave_addr = I2C_PXA_SLAVE_ADDR;

#ifdef CONFIG_I2C_PXA_SLAVE
	if (plat) {
		i2c->slave_addr = plat->slave_addr;
		i2c->slave = plat->slave;
	}
#endif

	clk_enable(i2c->clk);
	register_dvfm_constraint(i2c);

	if (plat) {
		i2c->adap.class = plat->class;
		i2c->use_pio = (plat->use_pio == 0) ? 0 : 1;
		i2c->flags = plat->flags;
		i2c->master_code = plat->master_code;
	}
/*   HANDLER_MODE__DFLT_SLOW_FAST == 0 - use default from upper configurator */
#if (HANDLER_MODE__DFLT_SLOW_FAST == 1)
	i2c->flags &= ~PXA_I2C_FAST_MODE;
#endif
#if (HANDLER_MODE__DFLT_SLOW_FAST == 2)
	i2c->flags |= PXA_I2C_FAST_MODE;
#endif

#if (HANDLER_MODE__DFLT_PIO_IRQ == 1)
	i2c->use_pio = 1;
#elif (HANDLER_MODE__DFLT_PIO_IRQ == 2)
	i2c->use_pio = 0;
#endif

	if (i2c->use_pio) {
		i2c->adap.algo = &i2c_pxa_pio_algorithm;
		pr_info("%s: PIO polling algo-mode configured\n", i2c->adap.name);
	} else {
		i2c->adap.algo = &i2c_pxa_algorithm;
		ret = request_irq(irq, i2c_pxa_handler, IRQF_DISABLED,
				i2c->adap.name, i2c);
		if (ret) {
			pr_info("%s: IRQ algo-mode configuration failed\n", i2c->adap.name);
			goto ereqirq;
		}
		pr_info("%s: IRQ mode configured\n", i2c->adap.name);
	}

	i2c->adap.retries = NUM_RETRIES_ON_BUS_BUSY;
	i2c->busbusy_delay = I2C_BUSBUSY_DELAY_US;
	if (i2c->flags & PXA_I2C_HIGH_MODE) {
		printk(KERN_INFO "I2C: hight speed mode enabled.\n");
		i2c->access2_delay = I2C_HS_DELAY_US;
	} else if (i2c->flags & PXA_I2C_FAST_MODE){
		printk(KERN_INFO "I2C: fast mode enabled.\n");
		i2c->access2_delay = I2C_ACCESS2_DELAY_US;
	} else if (i2c->flags & PXA_I2C_STANDARD_MODE) {
		printk(KERN_INFO "I2C: standard mode enabled.\n");
		i2c->access2_delay = I2C_SLOW_DELAY_US;
	} else {
		printk(KERN_INFO "I2C: un-support i2c mode!!! 0x%x\n",i2c->flags);
	}

	if (i2c->flags & PXA_I2C_USING_FIFO_DMA_MODE) {
		/* request dma */
		if(i2c->adap.nr == 0)
		{
			err = pxa_request_dma("I2C1_W", DMA_PRIO_LOW,
							pxa_i2c_dma_irq_w, i2c);
			if (err < 0) {
				printk("Can't request DMA for I2C1 write\n");
				ret = -ENOMEM;
				goto ereqirq;
			}
			i2c->dma_chans[0] = err;
			printk("got DMA channel (W) %u\n", i2c->dma_chans[0]);

			err = pxa_request_dma("I2C1_R", DMA_PRIO_LOW,
							pxa_i2c_dma_irq_r, i2c);
			if (err < 0) {
				printk("Can't request DMA for I2C1 read\n");
				ret = -ENOMEM;
				goto err_rxdma;
			}
			i2c->dma_chans[1] = err;
			printk("got I2C1 DMA channel (R) %u\n", i2c->dma_chans[1]);

			DRCMR(44) = i2c->dma_chans[0] | DRCMR_MAPVLD;
			DRCMR(43) = i2c->dma_chans[1] | DRCMR_MAPVLD;

		}
		else if(i2c->adap.nr == 1)
		{
			err = pxa_request_dma("I2C2_W", DMA_PRIO_LOW,
							pxa_i2c_dma_irq_w, i2c);
			if (err < 0) {
				printk("Can't request DMA for I2C2 write\n");
				ret = -ENOMEM;
				goto ereqirq;
			}
			i2c->dma_chans[0] = err;
			printk("got DMA channel (W) %u\n", i2c->dma_chans[0]);

			err = pxa_request_dma("I2C2_R", DMA_PRIO_LOW,
						pxa_i2c_dma_irq_r, i2c);
			if (err < 0) {
				printk("Can't request DMA for I2C2 read\n");
				ret = -ENOMEM;
				goto err_rxdma;
			}
			i2c->dma_chans[1] = err;
			printk("got I2C2 DMA channel (R) %u\n", i2c->dma_chans[1]);
			DRCMR(42) = i2c->dma_chans[0] | DRCMR_MAPVLD;
			DRCMR(41) = i2c->dma_chans[1] | DRCMR_MAPVLD;
		}
		else if(i2c->adap.nr == 2)
		{
			err = pxa_request_dma("I2C3_W", DMA_PRIO_LOW,
							pxa_i2c_dma_irq_w, i2c);
			if (err < 0) {
				printk("Can't request DMA for I2C3 write\n");
				ret = -ENOMEM;
				goto ereqirq;
			}
			i2c->dma_chans[0] = err;
			printk("got DMA channel (W) %u\n", i2c->dma_chans[0]);

			err = pxa_request_dma("I2C3_R", DMA_PRIO_LOW,
						pxa_i2c_dma_irq_r, i2c);
			if (err < 0) {
				printk("Can't request DMA for I2C3 read\n");
				ret = -ENOMEM;
				goto err_rxdma;
			}
			i2c->dma_chans[1] = err;
			printk("got I2C3 DMA channel (R) %u\n", i2c->dma_chans[1]);
			DRCMR(40) = i2c->dma_chans[0] | DRCMR_MAPVLD;
			DRCMR(39) = i2c->dma_chans[1] | DRCMR_MAPVLD;
		}

		if (NULL == i2c->read_buffer) {
			i2c->read_buffer = dma_alloc_coherent(NULL, PAGE_SIZE, &i2c->dma_handles_read, GFP_KERNEL);
			if (!i2c->read_buffer)
			{
				ret = -ENOMEM;
				goto rxdma_err_alloc;
			}
		}

		if (NULL == i2c->write_buffer) {
			i2c->write_buffer = dma_alloc_coherent(NULL, PAGE_SIZE, &i2c->dma_handles_write, GFP_KERNEL);
			if (!i2c->write_buffer)
			{
				ret = -ENOMEM;
				goto txdma_err_alloc;
			}
		}
	}

#if defined (CUSTOMIZATION_PER_SLAVE_DEVICE)
	if((i2c->adap.nr == 0) || (i2c->adap.nr == 1) || (i2c->adap.nr == 2))
		i2c->p_custom = &i2c_pxa_custom[0];
	else
		i2c->p_custom = NULL;
#endif
	i2c_pxa_reset(i2c);

	i2c->adap.algo_data = i2c;
	i2c->adap.dev.parent = &dev->dev;

	ret = i2c_add_numbered_adapter(&i2c->adap);
	if (ret < 0) {
		printk(KERN_INFO "I2C: Failed to add bus\n");
		goto eadapt;
	}

	platform_set_drvdata(dev, i2c);

#ifdef CONFIG_I2C_PXA_SLAVE
	printk(KERN_INFO "I2C: %s: PXA I2C adapter, slave address %d\n",
	       i2c->adap.dev.bus_id, i2c->slave_addr);
#else
	/*printk(KERN_INFO "I2C: %s: PXA I2C adapter\n",*/
	/*       i2c->adap.dev.bus_id);*/
#endif

#ifdef	CONFIG_PROC_FS
	create_i2c_pxa_proc_file(i2c, i2c->adap.nr);
#endif

	/*DBG_GPIO_CFG();*/
	clk_disable(i2c->clk);

	return 0;

eadapt:
	if (!i2c->use_pio)
		free_irq(irq, i2c);
	dma_free_coherent(NULL, PAGE_SIZE, i2c->write_buffer, i2c->dma_handles_write);
	i2c->write_buffer = NULL;
txdma_err_alloc:
	dma_free_coherent(NULL, PAGE_SIZE, i2c->read_buffer, i2c->dma_handles_read);
	i2c->read_buffer = NULL;
rxdma_err_alloc:
	pxa_free_dma(i2c->dma_chans[1]);
	i2c->dma_chans[1] = 0;
	if(i2c->adap.nr == 0)
		DRCMR(43) = 0;
	else if (i2c->adap.nr == 1)
		DRCMR(41) = 0;
	else if (i2c->adap.nr == 2)
		DRCMR(39) = 0;
err_rxdma:
	pxa_free_dma(i2c->dma_chans[0]);
	i2c->dma_chans[0] = 0;
	if(i2c->adap.nr == 0)
		DRCMR(44) = 0;
	else if (i2c->adap.nr == 1)
		DRCMR(42) = 0;
	else if (i2c->adap.nr == 2)
		DRCMR(40) = 0;
ereqirq:
	clk_disable(i2c->clk);
	iounmap(i2c->reg_base);
eremap:
	clk_put(i2c->clk);
eclk:
	kfree(i2c);
emalloc:
	release_mem_region(res->start, resource_size(res));
	return ret;
}

static int __exit i2c_pxa_remove(struct platform_device *dev)
{
	struct pxa_i2c *i2c = platform_get_drvdata(dev);

	clk_enable(i2c->clk);
	unregister_dvfm_constraint(i2c);

	if ( DCSR(i2c->dma_chans[1]) & DCSR_RUN)
		DCSR(i2c->dma_chans[1]) &= ~DCSR_RUN;

	if ( DCSR(i2c->dma_chans[0]) & DCSR_RUN)
		DCSR(i2c->dma_chans[0]) &= ~DCSR_RUN;

	if (i2c->write_buffer != NULL) {
		dma_free_coherent(NULL, PAGE_SIZE, i2c->write_buffer, i2c->dma_handles_write);
		i2c->write_buffer = NULL;
	}
	if (i2c->dma_chans[0] != 0) {
		pxa_free_dma(i2c->dma_chans[0]);
		if(i2c->adap.nr == 0)
			DRCMR(44) = 0;
		else if (i2c->adap.nr == 1)
			DRCMR(42) = 0;
		else if (i2c->adap.nr == 2)
			DRCMR(40) = 0;
		i2c->dma_chans[0] = 0;
	}

	if (i2c->read_buffer != NULL) {
		dma_free_coherent(NULL, PAGE_SIZE, i2c->read_buffer, i2c->dma_handles_read);
		i2c->read_buffer = NULL;
	}

	if (i2c->dma_chans[1] != 0) {
		pxa_free_dma(i2c->dma_chans[1]);
		if(i2c->adap.nr == 0)
			DRCMR(43) = 0;
		else if (i2c->adap.nr == 1)
			DRCMR(41) = 0;
		else if (i2c->adap.nr == 2)
			DRCMR(39) = 0;
		i2c->dma_chans[1] = 0;
	}

	platform_set_drvdata(dev, NULL);

#ifdef	CONFIG_PROC_FS
	remove_i2c_pxa_proc_file(i2c->adap.nr);
#endif

	i2c_del_adapter(&i2c->adap);
	if (!i2c->use_pio)
		free_irq(i2c->irq, i2c);

	clk_disable(i2c->clk);
	clk_put(i2c->clk);

	iounmap(i2c->reg_base);
	release_mem_region(i2c->iobase, i2c->iosize);
	kfree(i2c);

	return 0;
}

static struct platform_driver i2c_pxa_driver = {
	.probe		= i2c_pxa_probe,
	.remove		= __exit_p(i2c_pxa_remove),
	.driver		= {
		.name	= "pxa2xx-i2c",
		.owner	= THIS_MODULE,
	},
	.id_table	= i2c_pxa_id_table,
};

static int __init i2c_adap_pxa_init(void)
{
	return platform_driver_register(&i2c_pxa_driver);
}

static void __exit i2c_adap_pxa_exit(void)
{
	platform_driver_unregister(&i2c_pxa_driver);
}

MODULE_LICENSE("GPL");

subsys_initcall(i2c_adap_pxa_init);
module_exit(i2c_adap_pxa_exit);
