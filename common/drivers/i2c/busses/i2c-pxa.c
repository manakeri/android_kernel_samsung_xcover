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
 *    Jun 2002: Properly separated algo/adap [FB]
 *    Jan 2003: Fixed several bugs concerning interrupt handling [Kai-Uwe Bloem]
 *    Jan 2003: added limited signal handling [Kai-Uwe Bloem]
 *    Sep 2004: Major rework to ensure efficient bus handling [RMK]
 *    Dec 2004: Added support for PXA27x and slave device probing [Liam Girdwood]
 *    Feb 2005: Rework slave mode handling [RMK]
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
#include <linux/dma-mapping.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/slab.h>
#include <linux/io.h>

#include <asm/irq.h>
#include <plat/i2c.h>
#include <mach/dma.h>
#include <mach/dvfm.h>
#include <linux/wakelock.h>
/*
 * I2C register offsets will be shifted 0 or 1 bit left, depending on
 * different SoCs
 */
#define REG_SHIFT_0	(0 << 0)
#define REG_SHIFT_1	(1 << 0)
#define REG_SHIFT(d)	((d) & 0x1)

static const struct platform_device_id i2c_pxa_id_table[] = {
	{ "pxa2xx-i2c",		REG_SHIFT_1 },
	{ "pxa3xx-pwri2c",	REG_SHIFT_0 },
	{ "pxa95x-i2c",		REG_SHIFT_1 },
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

#define ICR_START	(1 << 0)	/* start bit */
#define ICR_STOP	(1 << 1)	/* stop bit */
#define ICR_ACKNAK	(1 << 2)	/* send ACK(0) or NAK(1) */
#define ICR_TB		(1 << 3)	/* transfer byte bit */
#define ICR_MA		(1 << 4)	/* master abort */
#define ICR_SCLE	(1 << 5)	/* master clock enable */
#define ICR_IUE		(1 << 6)	/* unit enable */
#define ICR_GCD		(1 << 7)	/* general call disable */
#define ICR_ITEIE	(1 << 8)	/* IDBR transmit empty int enable */
#define ICR_IRFIE	(1 << 9)	/* IDBR receive full int enable */
#define ICR_BEIE	(1 << 10)	/* enable bus error ints */
#define ICR_SSDIE	(1 << 11)	/* slave STOP detected int enable */
#define ICR_ALDIE	(1 << 12)	/* enable arbitration interrupt */
#define ICR_SADIE	(1 << 13)	/* slave address detected int enable */
#define ICR_UR		(1 << 14)	/* unit reset */
#define ICR_FM		(1 << 15)	/* fast mode */
#define ICR_HS_STD	(2 << 15)	/* high speed with standard mode */
#define ICR_HS_FAST	(3 << 15)	/* high speed with fast mode */
#define ICR_MSDIE	(1 << 17)	/* master stop detected int enable */
#define ICR_MSDE	(1 << 18)	/* master stop detected enable */
#define ICR_GPIOEN	(1 << 19)	/* drive SCL during HS mode */
#define ICR_FIFOEN	(1 << 20)	/* FIFO mode */
#define ICR_TXBEGIN	(1 << 21)	/* transaction begin (FIFO mode) */
#define ICR_TXDONE_IE	(1 << 22)	/* transaction done int enable (FIFO mode) */
#define ICR_RXSR_IE	(1 << 23)	/* receive FIFO half full int enable (FIFO mode) */
#define ICR_TXSR_IE	(1 << 24)	/* transmit FIFO service request int enable (FIFO mode) */
#define ICR_RXF_IE	(1 << 25)	/* receive FIFO full int enable (FIFO mode) */
#define ICR_RXOV_IE	(1 << 26)	/* receive FIFO overrun int enable (FIFO mode) */
#define ICR_DMA_EN	(1 << 27)	/* DMA enable (FIFO mode) */
#define ICR_RXUN_IE	(1 << 28)	/* receive FIFO underrun int enable (FIFO mode) */
#define ICR_TXOV_IE	(1 << 29)	/* transmit FIFO overrun int enable (FIFO mode) */
#define ICR_FIFO_MASK	(ICR_FIFOEN | ICR_TXBEGIN | ICR_TXDONE_IE	\
			| ICR_RXSR_IE | ICR_TXSR_IE | ICR_RXF_IE	\
			| ICR_RXOV_IE | ICR_DMA_EN | ICR_RXUN_IE	\
			| ICR_TXOV_IE)

#define ISR_RWM		(1 << 0)	/* read/write mode */
#define ISR_ACKNAK	(1 << 1)	/* ack/nak status */
#define ISR_UB		(1 << 2)	/* unit busy */
#define ISR_IBB		(1 << 3)	/* bus busy */
#define ISR_SSD		(1 << 4)	/* slave stop detected */
#define ISR_ALD		(1 << 5)	/* arbitration loss detected */
#define ISR_ITE		(1 << 6)	/* tx buffer empty */
#define ISR_IRF		(1 << 7)	/* rx buffer full */
#define ISR_GCAD	(1 << 8)	/* general call address detected */
#define ISR_SAD		(1 << 9)	/* slave address detected */
#define ISR_BED		(1 << 10)	/* bus error no ACK/NAK */
#define ISR_EBB		(1 << 11)	/* early bus busy */
#define ISR_MSD		(1 << 12)	/* master stop detected */
#define ISR_TXDONE	(1 << 13)	/* transaction done (FIFO mode) */
#define ISR_RXSR	(1 << 14)	/* receive FIFO service request (FIFO mode) */
#define ISR_TXSR	(1 << 15)	/* transmit FIFO service request (FIFO mode) */
#define ISR_RXF		(1 << 16)	/* receive FIFO full (FIFO mode) */
#define ISR_RXOV	(1 << 17)	/* receive FIFO overrun (FIFO mode) */
#define ISR_RXUN	(1 << 18)	/* receive FIFO underrun (FIFO mode) */
#define ISR_TXOV	(1 << 19)	/* transmit FIFO overrun (FIFO mode) */

#ifdef CONFIG_I2C_PXA_EBB_MSD_CHECK
#define ISR_BUSY	(ISR_IBB | ISR_UB | ISR_EBB)
#else
#define ISR_BUSY	(ISR_IBB | ISR_UB)
#endif

#define FIFO_ENTRY_TX		16
#define FIFO_ENTRY_RX		16
#define FIFO_THRESHOLD_TX	0
#define FIFO_THRESHOLD_RX	8

#define DMA_WRITE		0
#define DMA_READ		1

struct pxa_i2c {
	spinlock_t		lock;
	wait_queue_head_t	wait;
	struct i2c_msg		*msg;
	unsigned int		msg_num;
	unsigned int		msg_idx;
	unsigned int		msg_ptr;
	unsigned int		slave_addr;
	unsigned int		req_slave_addr;

	struct i2c_adapter	adap;
	struct clk		*clk;
#ifdef CONFIG_I2C_PXA_SLAVE
	struct i2c_slave_client *slave;
#endif

	unsigned int		irqlogidx;
	u32			isrlog[32];
	u32			icrlog[32];

	int			wfifo_cnt;
	int			rfifo_cnt;

	void __iomem		*reg_base;
	unsigned int		reg_shift;

	unsigned long		iobase;
	unsigned long		iosize;

	unsigned int		drcmr_rx;
	unsigned int		drcmr_tx;
	dma_addr_t		phys_rx;
	dma_addr_t		phys_tx;
	unsigned int		dma_chans[2];
	unsigned int		phys_wfifo;
	unsigned int		phys_rfifo;
	void			*read_buffer;
	void			*write_buffer;

	int			irq;
	unsigned int		mode;
	unsigned int		freq;
	unsigned int		master_code;
	unsigned int		ilcr;
	unsigned int		iwcr;
#ifdef CONFIG_PXA95x
	int                     dvfm_dev_idx;
#else
	struct wake_lock	idle_lock;
	struct wake_lock	suspend_lock;
#endif
};

#define _IBMR(i2c)	((i2c)->reg_base + (0x0 << (i2c)->reg_shift))
#define _IDBR(i2c)	((i2c)->reg_base + (0x4 << (i2c)->reg_shift))
#define _ICR(i2c)	((i2c)->reg_base + (0x8 << (i2c)->reg_shift))
#define _ISR(i2c)	((i2c)->reg_base + (0xc << (i2c)->reg_shift))
#define _ISAR(i2c)	((i2c)->reg_base + (0x10 << (i2c)->reg_shift))
#define _ILCR(i2c)	((i2c)->reg_base + (0x14 << (i2c)->reg_shift))
#define _IWCR(i2c)	((i2c)->reg_base + (0x18 << (i2c)->reg_shift))

#define _WFIFO(i2c)		((i2c)->reg_base + (0x20 << (i2c)->reg_shift))
#define _WFIFO_WPTR(i2c)	((i2c)->reg_base + (0x22 << (i2c)->reg_shift))
#define _WFIFO_RPTR(i2c)	((i2c)->reg_base + (0x24 << (i2c)->reg_shift))
#define _RFIFO(i2c)		((i2c)->reg_base + (0x28 << (i2c)->reg_shift))
#define _RFIFO_WPTR(i2c)	((i2c)->reg_base + (0x2a << (i2c)->reg_shift))
#define _RFIFO_RPTR(i2c)	((i2c)->reg_base + (0x2c << (i2c)->reg_shift))
#define _FIFO_TSHLD(i2c)	((i2c)->reg_base + (0x30 << (i2c)->reg_shift))

/*
 * I2C Slave mode address
 */
#define I2C_PXA_SLAVE_ADDR      0x1

#define DEBUG 0
static void i2c_pxa_reset(struct pxa_i2c *i2c);

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

#define show_state(i2c) i2c_pxa_show_state(i2c, __LINE__, __func__)

static void i2c_pxa_scream_blue_murder(struct pxa_i2c *i2c, const char *why)
{
	unsigned int i;
	struct i2c_pxa_platform_data *plat =
		(i2c->adap.dev.parent)->platform_data;
	printk(KERN_ERR "i2c: error: %s\n", why);
	printk(KERN_ERR"i2c: <%s> slave_0x%x error: %s\n", i2c->adap.name,
		i2c->req_slave_addr >> 1, why);
	printk(KERN_ERR "i2c: msg_num: %d msg_idx: %d msg_ptr: %d\n",
		i2c->msg_num, i2c->msg_idx, i2c->msg_ptr);
	printk(KERN_ERR "i2c: ICR: %08x ISR: %08x\n",
	       readl(_ICR(i2c)), readl(_ISR(i2c)));
	printk(KERN_DEBUG "i2c: log: ");
	for (i = 0; i < i2c->irqlogidx; i++)
		printk("[%08x:%08x] ", i2c->isrlog[i], i2c->icrlog[i]);
	printk("\n");
	if (strcmp(why, "exhausted retries") != 0) {
		if (plat && plat->i2c_bus_reset)
			plat->i2c_bus_reset();
		/*reset i2c contorler when it's fail*/
		i2c_pxa_reset(i2c);
	}
}

#else /* ifdef DEBUG */

#define i2c_debug	0

#define show_state(i2c) do { } while (0)
#define decode_ISR(val) do { } while (0)
#define decode_ICR(val) do { } while (0)
#define i2c_pxa_scream_blue_murder(i2c, why) do { } while (0)

#endif /* ifdef DEBUG / else */

static void i2c_pxa_master_complete(struct pxa_i2c *i2c, int ret);
static irqreturn_t i2c_pxa_handler(int this_irq, void *dev_id);

static inline int i2c_pxa_is_slavemode(struct pxa_i2c *i2c)
{
	return !(readl(_ICR(i2c)) & ICR_SCLE);
}

static void i2c_pxa_abort(struct pxa_i2c *i2c)
{
	int i = 250;
	unsigned long icr;

	if (i2c_pxa_is_slavemode(i2c)) {
		dev_dbg(&i2c->adap.dev, "%s: called in slave mode\n", __func__);
		return;
	}

	/* don't do abort if it's stopped */
	if (readl(_IBMR(i2c)) == 3)
		return;

#ifdef CONFIG_I2C_PXA_EBB_MSD_CHECK
	if( (readl(_ICR(i2c)) & ICR_MSDE) &&  (readl(_ISR(i2c)) & ISR_MSD)){
		writel(ISR_MSD, _ISR(i2c));
		return;
	}
#endif

	if (readl(_ICR(i2c)) & ICR_START) {
		/* under START state, could not use Master Abort */
		while ((i > 0) && (readl(_IBMR(i2c)) & 0x1) == 0) {
			icr = readl(_ICR(i2c));

			icr &= ~ICR_START;
			icr |= ICR_ACKNAK | ICR_STOP | ICR_TB;

			writel(icr, _ICR(i2c));

			show_state(i2c);

			mdelay(1);
			i--;
		}
	} else {
		/*MA Master Abort bit used*/
		icr = readl(_ICR(i2c));
		icr &= ~(ICR_START | ICR_ACKNAK | ICR_STOP | ICR_TB);
		icr |= ICR_MA;
		writel(I2C_ISR_INIT,  _ISR(i2c));
		writel(icr, _ICR(i2c));
		/* just use the largest delay */
		udelay(21);
	}

	writel(readl(_ICR(i2c)) & ~(ICR_MA | ICR_START | ICR_STOP),
	       _ICR(i2c));
}

static int i2c_pxa_wait_bus_not_busy(struct pxa_i2c *i2c)
{
	int timeout = DEF_TIMEOUT;

	while (timeout-- && readl(_ISR(i2c)) & ISR_BUSY) {
		if ((readl(_ISR(i2c)) & ISR_SAD) != 0)
			timeout += 4;

		msleep(2);
		show_state(i2c);
	}

	if (timeout < 0)
		show_state(i2c);

	return timeout < 0 ? I2C_RETRY : 0;
}

static int i2c_pxa_wait_master(struct pxa_i2c *i2c)
{
	unsigned long timeout = jiffies + HZ*4;

	while (time_before(jiffies, timeout)) {
		if (i2c_debug > 1)
			dev_dbg(&i2c->adap.dev, "%s: %ld: ISR=%08x, ICR=%08x, IBMR=%02x\n",
				__func__, (long)jiffies, readl(_ISR(i2c)), readl(_ICR(i2c)), readl(_IBMR(i2c)));

		if (readl(_ISR(i2c)) & ISR_SAD) {
			if (i2c_debug > 0)
				dev_dbg(&i2c->adap.dev, "%s: Slave detected\n", __func__);
			goto out;
		}

		/* wait for unit and bus being not busy, and we also do a
		 * quick check of the i2c lines themselves to ensure they've
		 * gone high...
		 */
		if ((readl(_ISR(i2c)) & ISR_BUSY) == 0 && readl(_IBMR(i2c)) == 3) {
			if (i2c_debug > 0)
				dev_dbg(&i2c->adap.dev, "%s: done\n", __func__);
			return 1;
		}

		msleep(1);
	}

	if (i2c_debug > 0)
		dev_dbg(&i2c->adap.dev, "%s: did not free\n", __func__);
 out:
	return 0;
}

static int i2c_pxa_set_master(struct pxa_i2c *i2c)
{
	if (i2c_debug)
		dev_dbg(&i2c->adap.dev, "setting to bus master\n");

	if ((readl(_ISR(i2c)) & ISR_BUSY) != 0) {
		dev_dbg(&i2c->adap.dev, "%s: unit is busy\n", __func__);
		if (!i2c_pxa_wait_master(i2c)) {
			dev_dbg(&i2c->adap.dev, "%s: error: unit busy\n", __func__);
			return I2C_RETRY;
		}
	}

	writel(readl(_ICR(i2c)) | ICR_SCLE, _ICR(i2c));
	return 0;
}

#ifdef CONFIG_I2C_PXA_SLAVE
static int i2c_pxa_wait_slave(struct pxa_i2c *i2c)
{
	unsigned long timeout = jiffies + HZ*1;

	/* wait for stop */

	show_state(i2c);

	while (time_before(jiffies, timeout)) {
		if (i2c_debug > 1)
			dev_dbg(&i2c->adap.dev, "%s: %ld: ISR=%08x, ICR=%08x, IBMR=%02x\n",
				__func__, (long)jiffies, readl(_ISR(i2c)), readl(_ICR(i2c)), readl(_IBMR(i2c)));

		if ((readl(_ISR(i2c)) & ISR_BUSY) == 0 ||
		    (readl(_ISR(i2c)) & ISR_SAD) != 0 ||
		    (readl(_ICR(i2c)) & ICR_SCLE) == 0) {
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
		udelay(100);   /* simple delay */
	} else {
		/* we need to wait for the stop condition to end */

		/* if we where in stop, then clear... */
		if (readl(_ICR(i2c)) & ICR_STOP) {
			udelay(100);
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
#endif

static void i2c_pxa_reset(struct pxa_i2c *i2c)
{
	u32 icr = I2C_ICR_INIT;

	pr_debug("Resetting I2C Controller Unit\n");

	/* abort any transfer currently under way */
	i2c_pxa_abort(i2c);

	/* reset according to 9.8 */
	writel(ICR_UR, _ICR(i2c));
	writel(I2C_ISR_INIT, _ISR(i2c));
	writel(readl(_ICR(i2c)) & ~ICR_UR, _ICR(i2c));

	writel(i2c->slave_addr, _ISAR(i2c));

	/* set control register values */
	if (i2c->freq == I2C_PXA_FREQ_FAST)
		icr |= ICR_FM;
	else if (i2c->freq == I2C_PXA_FREQ_HS_STD)
		icr |= ICR_HS_STD;
	else if (i2c->freq == I2C_PXA_FREQ_HS_FAST)
		icr |= ICR_HS_FAST;

	writel(icr, _ICR(i2c));

	/* There are 2 multi-masters on the I2C bus - APPS and COMM
	 * It is important both uses the standard frequency
	 * Adjust clock by ILCR, IWCR is important
	 */
	if(i2c->ilcr)
		writel(i2c->ilcr, _ILCR(i2c));
	if(i2c->iwcr)
		writel(i2c->iwcr, _IWCR(i2c));
	udelay(2);

#ifdef CONFIG_I2C_PXA_SLAVE
	dev_info(&i2c->adap.dev, "Enabling slave mode\n");
	writel(readl(_ICR(i2c)) | ICR_SADIE | ICR_ALDIE | ICR_SSDIE, _ICR(i2c));
#endif

	i2c_pxa_set_slave(i2c, 0);

	/* enable unit */
	writel(readl(_ICR(i2c)) | ICR_IUE, _ICR(i2c));
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
#else
static void i2c_pxa_slave_txempty(struct pxa_i2c *i2c, u32 isr)
{
	if (isr & ISR_BED) {
		/* what should we do here? */
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
#endif

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
	i2c->req_slave_addr = i2c_pxa_addr_byte(i2c->msg);

	/*
	 * Step 2: initiate the write.
	 */
	icr = readl(_ICR(i2c)) & ~(ICR_STOP | ICR_ALDIE);
#ifdef CONFIG_I2C_PXA_EBB_MSD_CHECK
	icr |= ICR_MSDE;
#endif
	writel(icr | ICR_START | ICR_TB, _ICR(i2c));
}

static inline void i2c_pxa_stop_message(struct pxa_i2c *i2c)
{
	u32 icr;

	/*
	 * Clear the STOP and ACK flags
	 */
	icr = readl(_ICR(i2c));
	icr &= ~(ICR_STOP | ICR_ACKNAK);
	writel(icr, _ICR(i2c));
}

static int i2c_pxa_pio_set_master(struct pxa_i2c *i2c)
{
	/* make timeout the same as for interrupt based functions */
	long timeout = 2 * DEF_TIMEOUT;

	/*
	 * Wait for the bus to become free.
	 */
	while (timeout-- && readl(_ISR(i2c)) & ISR_BUSY) {
		udelay(1000);
		show_state(i2c);
	}

	if (timeout < 0) {
		show_state(i2c);
		dev_err(&i2c->adap.dev,
			"i2c_pxa: timeout waiting for bus free\n");
		return I2C_RETRY;
	}

	/*
	 * Set master mode.
	 */
	writel(readl(_ICR(i2c)) | ICR_SCLE, _ICR(i2c));

	return 0;
}

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
#define I2C_PXA_HS_MASTERCODE  (0x08|0x06)
static int i2c_pxa_hs_send_mastercode(struct pxa_i2c *i2c)
{
	unsigned long timeout =(8000/35)+1; /*80mSec*/
	unsigned long icr = readl(_ICR(i2c)) & ~(ICR_START|ICR_STOP|ICR_ACKNAK|ICR_TB);

	writel(ISR_ITE, _ISR(i2c));

	/*step 1: write master code*/
	icr |= ICR_GPIOEN;
	writel((i2c->master_code?i2c->master_code: I2C_PXA_HS_MASTERCODE), _IDBR(i2c));

	/*step 2: start*/
	icr |= ICR_START | ICR_TB | ICR_ITEIE;
	icr &= ~ICR_ALDIE;
	writel(icr, _ICR(i2c));

	/*step 3: wait ack*/
	while(timeout-- ){
		unsigned long isr = readl(_ISR(i2c));
		if (isr & ISR_BED) {
			writel(ISR_BED, _ISR(i2c));
			i2c_pxa_master_complete(i2c, BUS_ERROR);
		}else if (isr & ISR_ITE){
			writel(ISR_ITE, _ISR(i2c));
			break;
		}
		udelay(35);
	}

	icr = readl(_ICR(i2c)) & ~(ICR_START|ICR_STOP|ICR_ACKNAK|ICR_TB);
	writel(icr, _ICR(i2c));
	show_state(i2c);

	return 0;
}

static int i2c_pxa_do_pio_xfer(struct pxa_i2c *i2c,
			       struct i2c_msg *msg, int num)
{
	unsigned long timeout = 500000; /* 5 seconds */
	int ret = 0;

	ret = i2c_pxa_pio_set_master(i2c);
	if (ret)
		goto out;

	i2c->msg = msg;
	i2c->msg_num = num;
	i2c->msg_idx = 0;
	i2c->msg_ptr = 0;
	i2c->irqlogidx = 0;

	if (i2c->freq & I2C_PXA_FREQ_HS_STD || i2c->freq & I2C_PXA_FREQ_HS_FAST) {
		if(i2c_pxa_hs_send_mastercode(i2c))
			goto out;
	}

#ifdef CONFIG_PXA95x
	dvfm_disable_lowpower(i2c->dvfm_dev_idx);
#else
	wake_lock(&i2c->idle_lock);
	wake_lock(&i2c->suspend_lock);
#endif
	i2c_pxa_start_message(i2c);

	while (i2c->msg_num > 0 && --timeout) {
		i2c_pxa_handler(0, i2c);
		udelay(10);
	}

	i2c_pxa_stop_message(i2c);
#ifdef CONFIG_PXA95x
	dvfm_enable_lowpower(i2c->dvfm_dev_idx);
#else
	wake_unlock(&i2c->idle_lock);
	wake_unlock(&i2c->suspend_lock);
#endif

	/*
	 * We place the return code in i2c->msg_idx.
	 */
	ret = i2c->msg_idx;

out:
	if (ret < 0)
		i2c_pxa_reset(i2c);
	if (timeout == 0)
		i2c_pxa_scream_blue_murder(i2c, "timeout");

	return ret;
}

/*
 * We are protected by the adapter bus mutex.
 */
static int i2c_pxa_do_xfer(struct pxa_i2c *i2c, struct i2c_msg *msg, int num)
{
	long timeout;
	int ret;

	/*
	 * Wait for the bus to become free.
	 */
	ret = i2c_pxa_wait_bus_not_busy(i2c);
	if (ret) {
		dev_err(&i2c->adap.dev, "i2c_pxa: timeout waiting for bus free\n");
		goto out;
	}

	/*
	 * Set master mode.
	 */
	ret = i2c_pxa_set_master(i2c);
	if (ret) {
		dev_err(&i2c->adap.dev, "i2c_pxa_set_master: error %d\n", ret);
		goto out;
	}

	spin_lock_irq(&i2c->lock);

	i2c->msg = msg;
	i2c->msg_num = num;
	i2c->msg_idx = 0;
	i2c->msg_ptr = 0;
	i2c->irqlogidx = 0;

	if (i2c->freq & I2C_PXA_FREQ_HS_STD || i2c->freq & I2C_PXA_FREQ_HS_FAST) {
		if(i2c_pxa_hs_send_mastercode(i2c)){
			spin_unlock_irq(&i2c->lock);
			goto out;
		}
	}

#ifdef CONFIG_PXA95x
	dvfm_disable_lowpower(i2c->dvfm_dev_idx);
#else
	wake_lock(&i2c->idle_lock);
	wake_lock(&i2c->suspend_lock);
#endif
	i2c_pxa_start_message(i2c);

	spin_unlock_irq(&i2c->lock);

	/*
	 * The rest of the processing occurs in the interrupt handler.
	 */
	timeout = wait_event_timeout(i2c->wait, i2c->msg_num == 0, HZ * 1);
	i2c_pxa_stop_message(i2c);
#ifdef CONFIG_PXA95x
	dvfm_enable_lowpower(i2c->dvfm_dev_idx);
#else
	wake_unlock(&i2c->idle_lock);
	wake_unlock(&i2c->suspend_lock);
#endif

	/*
	 * We place the return code in i2c->msg_idx.
	 */
	ret = i2c->msg_idx;

	if (timeout == 0)
		i2c_pxa_scream_blue_murder(i2c, "timeout");

 out:
	if (ret < 0)
		i2c_pxa_reset(i2c);
	return ret;
}

static int i2c_pxa_pio_xfer(struct i2c_adapter *adap,
			    struct i2c_msg msgs[], int num)
{
	struct pxa_i2c *i2c = adap->algo_data;
	int ret, i;

	/* If the I2C controller is disabled we need to reset it
	  (probably due to a suspend/resume destroying state). We do
	  this here as we can then avoid worrying about resuming the
	  controller before its users. */
	if (!(readl(_ICR(i2c)) & ICR_IUE))
		i2c_pxa_reset(i2c);

	for (i = adap->retries; i >= 0; i--) {
		ret = i2c_pxa_do_pio_xfer(i2c, msgs, num);
		if (ret != I2C_RETRY)
			goto out;

		if (i2c_debug)
			dev_dbg(&adap->dev, "Retrying transmission\n");
		udelay(100);
	}
	i2c_pxa_scream_blue_murder(i2c, "exhausted retries");
	ret = -EREMOTEIO;
 out:
	i2c_pxa_set_slave(i2c, ret);
	return ret;
}

/*
 * i2c_pxa_master_complete - complete the message and wake up.
 */
static void i2c_pxa_master_complete(struct pxa_i2c *i2c, int ret)
{
	i2c->msg_ptr = 0;
	i2c->msg = NULL;
	i2c->msg_idx ++;
	i2c->msg_num = 0;
	if (ret)
		i2c->msg_idx = ret;
	switch (i2c->mode) {
	case I2C_PXA_MODE_INT:
	case I2C_PXA_MODE_FIFO_INT:
	case I2C_PXA_MODE_FIFO_DMA:
		wake_up(&i2c->wait);
		break;
	}
}

static void i2c_pxa_irq_txempty(struct pxa_i2c *i2c, u32 isr)
{
	u32 icr = readl(_ICR(i2c)) & ~(ICR_START|ICR_STOP|ICR_ACKNAK|ICR_TB);

 again:
	/*If ISR_ALD is set, we lost arbitration. it may be due to multi-master*/
	if (isr & ISR_ALD) {
		i2c_pxa_master_complete(i2c, I2C_MASTER_ABORT);
		goto exit;
	}

	if (isr & ISR_BED) {
		int ret = BUS_ERROR;

		/*
		 * I2C bus error - either the device NAK'd us, or
		 * something more serious happened.  If we were NAK'd
		 * on the initial address phase, we can retry.
		 */
		if (isr & ISR_ACKNAK) {
			if (i2c->msg_ptr == 0 && i2c->msg_idx == 0)
				ret = I2C_RETRY;
			else
				ret = XFER_NAKED;
		}
		i2c_pxa_master_complete(i2c, ret);
	} else if (isr & ISR_RWM) {
		/*
		 * Read mode.  We have just sent the address byte, and
		 * now we must initiate the transfer.
		 */
		if (i2c->msg_ptr == i2c->msg->len - 1 &&
		    i2c->msg_idx == i2c->msg_num - 1)
			icr |= ICR_STOP | ICR_ACKNAK;

		icr |= ICR_ALDIE | ICR_TB;
	} else if (i2c->msg_ptr < i2c->msg->len) {
		/*
		 * Write mode.  Write the next data byte.
		 */
		writel(i2c->msg->buf[i2c->msg_ptr++], _IDBR(i2c));

		icr |= ICR_ALDIE | ICR_TB;

		/*
		 * If this is the last byte of the last message, send
		 * a STOP.
		 */
		if (i2c->msg_ptr == i2c->msg->len &&
		    i2c->msg_idx == i2c->msg_num - 1)
			icr |= ICR_STOP;
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
		i2c->req_slave_addr = i2c_pxa_addr_byte(i2c->msg);
		/*
		 * And trigger a repeated start, and send the byte.
		 */
		icr &= ~ICR_ALDIE;
#ifdef CONFIG_I2C_PXA_EBB_MSD_CHECK
		icr |= ICR_MSDE;
#endif
		icr |= ICR_START | ICR_TB;
	} else {
		if (i2c->msg->len == 0) {
			/*
			 * Device probes have a message length of zero
			 * and need the bus to be reset before it can
			 * be used again.
			 */
			i2c_pxa_reset(i2c);
		}
		i2c_pxa_master_complete(i2c, 0);
	}

exit:
	i2c->icrlog[i2c->irqlogidx-1] = icr;

	writel(icr, _ICR(i2c));
	show_state(i2c);
}

static void i2c_pxa_irq_rxfull(struct pxa_i2c *i2c, u32 isr)
{
	u32 icr = readl(_ICR(i2c)) & ~(ICR_START|ICR_STOP|ICR_ACKNAK|ICR_TB);

	/*
	 * Read the byte.
	 */
	i2c->msg->buf[i2c->msg_ptr++] = readl(_IDBR(i2c));

	if (i2c->msg_ptr < i2c->msg->len) {
		/*
		 * If this is the last byte of the last
		 * message, send a STOP.
		 */
		if (i2c->msg_ptr == i2c->msg->len - 1)
			icr |= ICR_STOP | ICR_ACKNAK;

		icr |= ICR_ALDIE | ICR_TB;
	} else {
		i2c_pxa_master_complete(i2c, 0);
	}

	i2c->icrlog[i2c->irqlogidx-1] = icr;

	writel(icr, _ICR(i2c));
}

static void setup_dma(struct pxa_i2c *i2c, int direction, int count)
{
	int chan = i2c->dma_chans[direction];
	u32 icr;

	/* clear ICR before DMA working */
	icr = readl(_ICR(i2c));
	icr &= ~(ICR_START | ICR_STOP | ICR_ACKNAK | ICR_TB);
	writel(icr, _ICR(i2c));

	if (direction == DMA_WRITE) {
		if (!(DCSR(chan) & DCSR_STOPSTATE))
			return;

		DRCMR(i2c->drcmr_tx) = chan | DRCMR_MAPVLD;
		DCSR(chan) = DCSR_NODESC;
		DTADR(chan) = i2c->phys_wfifo;
		DSADR(chan) = i2c->phys_tx;
		DCMD(chan) = DCMD_INCSRCADDR | DCMD_FLOWTRG | DCMD_ENDIRQEN
				| DCMD_WIDTH2 | DCMD_BURST8 | count << 1;
		DCSR(chan) |= DCSR_RUN;
	}
}

static int setup_wfifo(struct pxa_i2c *i2c, u32 icr, u16 *data)
{
	int i = 0, max, new_trans = 0, hs_set = 0;

	switch (i2c->mode) {
	case I2C_PXA_MODE_FIFO_INT:
		max = FIFO_ENTRY_TX - FIFO_THRESHOLD_TX;
		break;
	case I2C_PXA_MODE_FIFO_DMA:
		max = PAGE_SIZE;
		break;
	default:
		return -EINVAL;
	}

	while (i < max) {
		*data = 0;
		if (!i2c->msg_ptr && !i2c->msg_idx && !new_trans) {
			if((i2c->freq & I2C_PXA_FREQ_HS_STD || i2c->freq & I2C_PXA_FREQ_HS_FAST)
				&& !hs_set){
				icr |= ICR_GPIOEN;
				writel(icr, _ICR(i2c));

				*data |= (ICR_START | ICR_TB) << 8;
				*data |= (i2c->master_code? i2c->master_code: I2C_PXA_HS_MASTERCODE);
				hs_set ++;
			}else{
				/* start point of message */
				*data |= (ICR_START | ICR_TB) << 8;
				*data |= i2c_pxa_addr_byte(i2c->msg);
				new_trans++;
			}
		} else if (i2c->msg->flags & I2C_M_RD) {
			/* Write dummy data for read transaction */
			if (++i2c->msg_ptr == i2c->msg->len)
				*data |= (ICR_STOP | ICR_ACKNAK) << 8;
		} else if (i2c->msg_ptr < i2c->msg->len) {
			/* Write next byte for write transaction */
			*data |= i2c->msg->buf[i2c->msg_ptr++];

			if ((i2c->msg_ptr == i2c->msg->len)
				&& (i2c->msg_idx == i2c->msg_num - 1)) {
				/* end point of message */
				*data |= ICR_STOP << 8;
				/* ignore endless TXSR int */
				writel(icr & ~ICR_TXSR_IE, _ICR(i2c));
			}
		} else if (i2c->msg_idx < i2c->msg_num - 1) {
			/* Write next segment for write transaction */
			i2c->msg_ptr = 0;
			i2c->msg_idx++;
			i2c->msg++;
			if (i2c->msg->flags & I2C_M_NOSTART)
				continue;

			/* fill next address with repeated START */
			*data |= (ICR_START | ICR_TB) << 8;
			*data |= i2c_pxa_addr_byte(i2c->msg);
		} else {
			if (i2c->msg->len == 0)
				i2c_pxa_reset(i2c);
			break;
		}
		/* fill data to FIFO */
		*data |= ICR_TB << 8;
		i++;
		if (i2c->mode == I2C_PXA_MODE_FIFO_INT) {
			writel(*data, _WFIFO(i2c));

			/* start new transaction for TXDONE int */
			if ((i2c->msg_idx == 0) && (*data & (ICR_START << 8)))
				writel(icr | ICR_TXBEGIN, _ICR(i2c));
		}

		if (*data & (ICR_STOP << 8)) {
			/* ignore endless TXSR int */
			writel(icr & ~ICR_TXSR_IE, _ICR(i2c));
			break;
		}

		if (i2c->mode == I2C_PXA_MODE_FIFO_DMA)
			data++;
	}
	writel(ISR_TXSR, _ISR(i2c));

	if (i > 0 && i2c->mode == I2C_PXA_MODE_FIFO_DMA)
		setup_dma(i2c, DMA_WRITE, i);
	return 0;
}

static void i2c_fifo_irq(struct pxa_i2c *i2c, u32 isr)
{
	u32 icr = readl(_ICR(i2c)) & ~(ICR_START|ICR_STOP|ICR_ACKNAK|ICR_TB);
	u16 data;
	int i, ret;

	/*If ISR_ALD is set, we lost arbitration. it may be due to multi-master*/
	if (isr & ISR_ALD) {
		writel(ISR_ALD, _ISR(i2c));
		i2c_pxa_master_complete(i2c, I2C_MASTER_ABORT);
		return;
	}

	if (isr & ISR_BED) {
		int ret = BUS_ERROR;

		/*
		 * I2C bus error - either the device NAK'd us, or
		 * something more serious happened.  If we were NAK'd
		 * on the initial address phase, we can retry.
		 */
		if (isr & ISR_ACKNAK) {
			if (i2c->msg_ptr == 0 && i2c->msg_idx == 0)
				ret = I2C_RETRY;
			else
				ret = XFER_NAKED;
		}
		i2c_pxa_master_complete(i2c, ret);
		return;
	}

	if (isr & ISR_TXOV) {
		/* clear FIFO */
		writel(0, _WFIFO_WPTR(i2c));
		writel(0, _WFIFO_RPTR(i2c));
		ret = I2C_RETRY;
		i2c_pxa_master_complete(i2c, ret);
		writel(isr, _ISR(i2c));
		return;
	}

	if (isr & ISR_TXDONE) {
		if (i2c->msg->flags & I2C_M_RD) {
			/* read data from FIFO */
			i2c->rfifo_cnt = 0;
			for (i = 0; i < FIFO_ENTRY_RX; i++) {
				if (i2c->rfifo_cnt >= i2c->msg->len)
					break;
				i2c->msg->buf[i2c->rfifo_cnt++]
					= readl(_RFIFO(i2c));
			}
		}
		i2c_pxa_master_complete(i2c, 0);
		return;
	}

	if (isr & ISR_TXSR)
		setup_wfifo(i2c, icr, &data);

	if (isr & (ISR_RXF | ISR_RXSR)) {
		if (i2c->msg->flags & I2C_M_RD) {
			/* read data from FIFO */
			i2c->rfifo_cnt = 0;
			for (i = 0; i < FIFO_ENTRY_RX; i++) {
				if (i2c->rfifo_cnt >= i2c->msg->len)
					break;
				i2c->msg->buf[i2c->rfifo_cnt++]
					= readl(_RFIFO(i2c));
			}
		}
	}

	i2c->icrlog[i2c->irqlogidx-1] = icr;

	show_state(i2c);
}

static void i2c_fifo_dma(struct pxa_i2c *i2c, u32 isr)
{
	u32 icr = readl(_ICR(i2c)) & ~(ICR_START|ICR_STOP|ICR_ACKNAK|ICR_TB);
	int ret = 0;

	/*If ISR_ALD is set, we lost arbitration. it may be due to multi-master*/
	if (isr & ISR_ALD) {
		writel(ISR_ALD, _ISR(i2c));
		i2c_pxa_master_complete(i2c, I2C_MASTER_ABORT);
		return;
	}

	if (isr & ISR_BED) {
		ret = BUS_ERROR;

		/*
		 * I2C bus error - either the device NAK'd us, or
		 * something more serious happened.  If we were NAK'd
		 * on the initial address phase, we can retry.
		 */
		if (isr & ISR_ACKNAK) {
			if (i2c->msg_ptr == 0 && i2c->msg_idx == 0)
				ret = I2C_RETRY;
			else
				ret = XFER_NAKED;
		}
		i2c_pxa_master_complete(i2c, ret);
		return;
	}

	if (isr & ISR_TXDONE) {
		/* read FIFO for trailing bytes */
		if (i2c->msg->flags & I2C_M_RD) {
			i2c->msg_ptr = 0;
			while (i2c->msg_ptr < i2c->msg->len) {
				i2c->msg->buf[i2c->msg_ptr++]
					= readl(_RFIFO(i2c));
			}
		}
		writel(ISR_TXSR | ISR_TXDONE, _ISR(i2c));
		i2c_pxa_master_complete(i2c, 0);
		return;
	}

	i2c->icrlog[i2c->irqlogidx-1] = icr;

	show_state(i2c);
}

static void i2c_dma_irq(int channel, void *data)
{
	u32 dcsr;

	dcsr = DCSR(channel);
	DCSR(channel) = dcsr & ~DCSR_RUN;

	if (dcsr & DCSR_BUSERR)
		DCSR(channel) |= DCSR_BUSERR;

	if (dcsr & DCSR_ENDINTR)
		DCSR(channel) |= DCSR_ENDINTR;
}

static irqreturn_t i2c_pxa_handler(int this_irq, void *dev_id)
{
	struct pxa_i2c *i2c = dev_id;
	u32 isr = readl(_ISR(i2c));

	if (i2c_debug > 2) {
		dev_dbg(&i2c->adap.dev, "%s: ISR=%08x, ICR=%08x, IBMR=%02x\n",
			__func__, isr, readl(_ICR(i2c)), readl(_IBMR(i2c)));
		decode_ISR(isr);
	}

	if (i2c->irqlogidx < ARRAY_SIZE(i2c->isrlog))
		i2c->isrlog[i2c->irqlogidx++] = isr;

	show_state(i2c);

	/*
	 * Always clear all pending IRQs.
	 */
	writel(isr, _ISR(i2c));

	if (isr & ISR_SAD)
		i2c_pxa_slave_start(i2c, isr);
	if (isr & ISR_SSD)
		i2c_pxa_slave_stop(i2c);

	if (i2c_pxa_is_slavemode(i2c)) {
		if (isr & ISR_ITE)
			i2c_pxa_slave_txempty(i2c, isr);
		if (isr & ISR_IRF)
			i2c_pxa_slave_rxfull(i2c, isr);
	} else if (i2c->msg) {
		switch (i2c->mode) {
		case I2C_PXA_MODE_FIFO_INT:
			i2c_fifo_irq(i2c, isr);
			break;
		case I2C_PXA_MODE_FIFO_DMA:
			i2c_fifo_dma(i2c, isr);
			break;
		case I2C_PXA_MODE_INT:
		case I2C_PXA_MODE_POLL:
			if (isr & ISR_ITE)
				i2c_pxa_irq_txempty(i2c, isr);
			if (isr & ISR_IRF)
				i2c_pxa_irq_rxfull(i2c, isr);
			break;
		}
	} else {
		i2c_pxa_scream_blue_murder(i2c, "spurious irq");
	}

	return IRQ_HANDLED;
}


static int i2c_pxa_xfer(struct i2c_adapter *adap, struct i2c_msg msgs[], int num)
{
	struct pxa_i2c *i2c = adap->algo_data;
	int ret, i;

	enable_irq(i2c->irq);

	for (i = adap->retries; i >= 0; i--) {
		ret = i2c_pxa_do_xfer(i2c, msgs, num);
		if (ret != I2C_RETRY)
			goto out;

		if (i2c_debug)
			dev_dbg(&adap->dev, "Retrying transmission\n");
		udelay(100);
	}
	i2c_pxa_scream_blue_murder(i2c, "exhausted retries");
	ret = -EREMOTEIO;
 out:
	i2c_pxa_set_slave(i2c, ret);

	disable_irq(i2c->irq);
	return ret;
}

static void enable_fifo_int(struct pxa_i2c *i2c)
{
	u16 *w_buf;
	u32 icr;

	/* clear all pointers of FIFO */
	writel(0, _WFIFO_WPTR(i2c));
	writel(0, _WFIFO_RPTR(i2c));
	writel(0, _RFIFO_WPTR(i2c));
	writel(0, _RFIFO_RPTR(i2c));

	/* enable FIFO mode */
	icr = readl(_ICR(i2c));
	if (i2c->mode == I2C_PXA_MODE_FIFO_INT) {
		icr |= ICR_FIFOEN | ICR_TXSR_IE | ICR_RXSR_IE | ICR_RXF_IE
			| ICR_RXOV_IE | ICR_TXDONE_IE;
		icr &= ~(ICR_DMA_EN | ICR_ITEIE | ICR_IRFIE);
		writel(icr, _ICR(i2c));
	} else {
		icr &= ~(ICR_TXSR_IE | ICR_RXSR_IE | ICR_RXF_IE
			| ICR_RXOV_IE | ICR_ITEIE | ICR_IRFIE);
		icr |= ICR_FIFOEN | ICR_DMA_EN | ICR_TXDONE_IE;
		writel(icr, _ICR(i2c));

		w_buf = i2c->write_buffer;
		setup_wfifo(i2c, icr, w_buf);
	}

	/* set FIFO threshold (Rx/Tx -- half empty/empty) */
	writel(FIFO_THRESHOLD_RX << 16, _FIFO_TSHLD(i2c));
}

static int fifo_int_xfer(struct pxa_i2c *i2c, struct i2c_msg *msg, int num)
{
	long timeout;
	int ret;

	/* Wait for the bus to become free. */
	ret = i2c_pxa_wait_bus_not_busy(i2c);
	if (ret) {
		dev_err(&i2c->adap.dev, "i2c_pxa: timeout waiting for bus free\n");
		goto out;
	}

	/* Set master mode. */
	ret = i2c_pxa_set_master(i2c);
	if (ret) {
		dev_err(&i2c->adap.dev, "i2c_pxa_set_master: error %d\n", ret);
		goto out;
	}

	spin_lock_irq(&i2c->lock);

	i2c->msg = msg;
	i2c->msg_num = num;
	i2c->msg_idx = 0;
	i2c->msg_ptr = 0;
	i2c->irqlogidx = 0;

	enable_fifo_int(i2c);
	spin_unlock_irq(&i2c->lock);

	/* The rest of the processing occurs in the interrupt handler. */
	timeout = wait_event_timeout(i2c->wait, i2c->msg_num == 0, HZ * 5);

	/* We place the return code in i2c->msg_idx. */
	ret = i2c->msg_idx;

	if (timeout == 0)
		i2c_pxa_scream_blue_murder(i2c, "timeout");

out:
	if (ret < 0)
		i2c_pxa_reset(i2c);
	return ret;
}

static int i2c_pxa_fifo_int_xfer(struct i2c_adapter *adap,
				 struct i2c_msg msgs[], int num)
{
	struct pxa_i2c *i2c = adap->algo_data;
	int ret, i;

	for (i = adap->retries; i >= 0; i--) {
		ret = fifo_int_xfer(i2c, msgs, num);
		if (ret != I2C_RETRY)
			goto out;

		if (i2c_debug)
			dev_dbg(&adap->dev, "Retrying transmission\n");
		udelay(100);
	}
	i2c_pxa_scream_blue_murder(i2c, "exhausted retries");
	ret = -EREMOTEIO;
out:
	i2c_pxa_set_slave(i2c, ret);
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
	.master_xfer	= i2c_pxa_pio_xfer,
	.functionality	= i2c_pxa_functionality,
};

static const struct i2c_algorithm i2c_pxa_fifo_int_algorithm = {
	.master_xfer	= i2c_pxa_fifo_int_xfer,
	.functionality	= i2c_pxa_functionality,
};

static int i2c_pxa_adap_init(struct platform_device *dev,
			     struct pxa_i2c *i2c, int irq)
{
	struct resource *res;
	int ret;

	switch (i2c->mode) {
	case I2C_PXA_MODE_POLL:
		i2c->adap.algo = &i2c_pxa_pio_algorithm;
		break;
	case I2C_PXA_MODE_INT:
		i2c->adap.algo = &i2c_pxa_algorithm;
		ret = request_irq(irq, i2c_pxa_handler, IRQF_DISABLED,
				  i2c->adap.name, i2c);
		if (ret)
			goto out;
		break;
	case I2C_PXA_MODE_FIFO_INT:
		i2c->adap.algo = &i2c_pxa_fifo_int_algorithm;
		ret = request_irq(irq, i2c_pxa_handler, IRQF_DISABLED,
				  i2c->adap.name, i2c);
		if (ret)
			goto out;
		break;
	case I2C_PXA_MODE_FIFO_DMA:
		res = platform_get_resource(dev, IORESOURCE_DMA, 0);
		if (!res) {
			dev_err(&dev->dev, "no I2C Rx DMRCR definied\n");
			ret = -ENODEV;
			goto out;
		}
		i2c->drcmr_rx = res->start;

		res = platform_get_resource(dev, IORESOURCE_DMA, 1);
		if (!res) {
			dev_err(&dev->dev, "no I2C Tx DMRCR definied\n");
			ret = -ENODEV;
			goto out;
		}
		i2c->drcmr_tx = res->start;

		i2c->adap.algo = &i2c_pxa_fifo_int_algorithm;
		ret = request_irq(irq, i2c_pxa_handler, IRQF_DISABLED,
				  i2c->adap.name, i2c);
		if (ret)
			goto out;

		i2c->dma_chans[DMA_WRITE] = pxa_request_dma("i2c_w",
						DMA_PRIO_LOW, i2c_dma_irq, i2c);
		if (i2c->dma_chans[DMA_WRITE] < 0) {
			dev_err(&dev->dev, "can't request DMA for I2C write\n");
			goto out_irq;
		}

		i2c->dma_chans[DMA_READ] = pxa_request_dma("i2c_r",
						DMA_PRIO_LOW, i2c_dma_irq, i2c);
		if (i2c->dma_chans[DMA_READ] < 0) {
			dev_err(&dev->dev, "can't request DMA for I2C read\n");
			goto out_req;
		}

		i2c->read_buffer = dma_alloc_coherent(NULL, PAGE_SIZE,
						      &i2c->phys_rx,
						      GFP_KERNEL);
		if (!i2c->read_buffer) {
			dev_err(&dev->dev, "can't allocate dma memory\n");
			goto out_read;
		}

		i2c->write_buffer = dma_alloc_coherent(NULL, PAGE_SIZE,
						       &i2c->phys_tx,
						       GFP_KERNEL);
		if (!i2c->write_buffer) {
			dev_err(&dev->dev, "can't allocate dma memory\n");
			goto out_write;
		}
		break;
	}
	return 0;

out_write:
	dma_free_coherent(NULL, PAGE_SIZE, i2c->read_buffer, i2c->phys_rx);
out_read:
	pxa_free_dma(i2c->dma_chans[DMA_READ]);
out_req:
	pxa_free_dma(i2c->dma_chans[DMA_WRITE]);
out_irq:
	free_irq(irq, i2c);
	ret = -ENOMEM;
out:
	return ret;
}

static int i2c_pxa_probe(struct platform_device *dev)
{
	struct pxa_i2c *i2c;
	struct resource *res;
	struct i2c_pxa_platform_data *plat = dev->dev.platform_data;
	struct platform_device_id *id = (struct platform_device_id *)platform_get_device_id(dev);
	int ret;
	int irq;

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
	i2c->adap.retries = 3;

#ifdef CONFIG_PXA95x
	ret = dvfm_register((char *)i2c->adap.name, &i2c->dvfm_dev_idx);
	if (ret) {
		pr_err("Error %d: Fails to register %s into dvfm.\n",
				ret, i2c->adap.name);
		goto eadapt;
	}
#else
	wake_lock_init(&i2c->idle_lock, WAKE_LOCK_IDLE,
			(const char *)i2c->adap.name);
	wake_lock_init(&i2c->suspend_lock, WAKE_LOCK_SUSPEND,
			(const char *)i2c->adap.name);
#endif

	spin_lock_init(&i2c->lock);
	init_waitqueue_head(&i2c->wait);

	/*
	 * If "dev->id" is negative we consider it as zero.
	 * The reason to do so is to avoid sysfs names that only make
	 * sense when there are multiple adapters.
	 */
	i2c->adap.nr = dev->id != -1 ? dev->id : 0;
	snprintf(i2c->adap.name, sizeof(i2c->adap.name), "pxa_i2c-i2c.%u",
		 i2c->adap.nr);

	i2c->clk = clk_get(&dev->dev, NULL);
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
	i2c->phys_wfifo = _WFIFO(i2c) - i2c->reg_base + i2c->iobase;
	i2c->phys_rfifo = _RFIFO(i2c) - i2c->reg_base + i2c->iobase;

	i2c->irq = irq;

	i2c->slave_addr = I2C_PXA_SLAVE_ADDR;

#ifdef CONFIG_I2C_PXA_SLAVE
	if (plat) {
		i2c->slave_addr = plat->slave_addr;
		i2c->slave = plat->slave;
	}
#endif

	clk_enable(i2c->clk);

	if (plat) {
		i2c->adap.class = plat->class;
		i2c->mode = plat->mode;
		i2c->freq = plat->freq;
		i2c->master_code = plat->master_code;
		i2c->ilcr = plat->ilcr;
		i2c->iwcr = plat->iwcr;
		i2c->adap.hardware_lock = plat->hardware_lock;
		i2c->adap.hardware_unlock = plat->hardware_unlock;
	}

	ret = i2c_pxa_adap_init(dev, i2c, i2c->irq);
	if (ret < 0)
		goto ereqirq;

	i2c_pxa_reset(i2c);

	disable_irq(irq);

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
	       dev_name(&i2c->adap.dev), i2c->slave_addr);
#else
	printk(KERN_INFO "I2C: %s: PXA I2C adapter\n",
	       dev_name(&i2c->adap.dev));
#endif
	return 0;

eadapt:
	switch (i2c->mode) {
	case I2C_PXA_MODE_INT:
	case I2C_PXA_MODE_FIFO_INT:
		free_irq(irq, i2c);
		break;
	case I2C_PXA_MODE_FIFO_DMA:
		dma_free_coherent(NULL, PAGE_SIZE, i2c->read_buffer,
				  i2c->phys_rx);
		dma_free_coherent(NULL, PAGE_SIZE, i2c->write_buffer,
				  i2c->phys_tx);
		pxa_free_dma(i2c->dma_chans[DMA_READ]);
		pxa_free_dma(i2c->dma_chans[DMA_WRITE]);
		free_irq(irq, i2c);
		break;
	}
ereqirq:
	clk_disable(i2c->clk);
	iounmap(i2c->reg_base);
eremap:
	clk_put(i2c->clk);
eclk:
	kfree(i2c);
emalloc:
	release_mem_region(res->start, resource_size(res));
#ifdef CONFIG_PXA95x
	dvfm_unregister((char *)i2c->adap.name, &i2c->dvfm_dev_idx);
#else
	wake_lock_destroy(&i2c->idle_lock);
	wake_lock_destroy(&i2c->suspend_lock);
#endif
	return ret;
}

static int __exit i2c_pxa_remove(struct platform_device *dev)
{
	struct pxa_i2c *i2c = platform_get_drvdata(dev);

	platform_set_drvdata(dev, NULL);

#ifdef CONFIG_PXA95x
	dvfm_unregister((char *)i2c->adap.name, &i2c->dvfm_dev_idx);
#else
	wake_lock_destroy(&i2c->idle_lock);
	wake_lock_destroy(&i2c->suspend_lock);
#endif

	i2c_del_adapter(&i2c->adap);
	switch (i2c->mode) {
	case I2C_PXA_MODE_INT:
	case I2C_PXA_MODE_FIFO_INT:
		free_irq(i2c->irq, i2c);
		break;
	case I2C_PXA_MODE_FIFO_DMA:
		dma_free_coherent(NULL, PAGE_SIZE, i2c->read_buffer,
				  i2c->phys_rx);
		dma_free_coherent(NULL, PAGE_SIZE, i2c->write_buffer,
				  i2c->phys_tx);
		pxa_free_dma(i2c->dma_chans[DMA_READ]);
		pxa_free_dma(i2c->dma_chans[DMA_WRITE]);
		free_irq(i2c->irq, i2c);
		break;
	}

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
MODULE_ALIAS("platform:pxa2xx-i2c");

subsys_initcall(i2c_adap_pxa_init);
module_exit(i2c_adap_pxa_exit);
