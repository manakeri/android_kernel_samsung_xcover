/*
 * drivers/mtd/nand/pxa3xx_nand.c
 *
 * Copyright © 2005 Intel Corporation
 * Copyright © 2006 Marvell International Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/dma-mapping.h>
#include <linux/delay.h>
#include <linux/clk.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/nand.h>
#include <linux/mtd/partitions.h>
#include <linux/io.h>
#include <linux/irq.h>
#include <linux/slab.h>
#include <mach/dma.h>
#include <asm/highmem.h>
#include <asm/cacheflush.h>
#include <plat/pxa3xx_nand.h>
#ifdef CONFIG_PXA3XX_BBM
#include <plat/pxa3xx_bbm.h>
#endif
#include <mach/dvfm.h>
#include <linux/wakelock.h>
#include <linux/pm_qos_params.h>
#ifdef CONFIG_TRACEPOINTS
#define CREATE_TRACE_POINTS
#include "pxa3xx_nand_trace.h"
#endif

#define	CHIP_DELAY_TIMEOUT	(2 * HZ/10)
#define NAND_STOP_DELAY		(2 * HZ/50)
#define PAGE_CHUNK_SIZE		(2048)
#define OOB_CHUNK_SIZE		(64)
#define CMD_POOL_SIZE           (8)
#define READ_ID_BYTES		(4)
#define BCH_THRESHOLD           (8)
#define BCH_STRENGTH		(4)
#define HAMMING_STRENGTH	(1)

/* the max buff size should be large than the largest size
 * of page of NAND flash that currently controller support
 */
#define DMA_H_SIZE	(sizeof(struct pxa_dma_desc) * 2)
#define NAND_PGSZ	(NAND_MAX_OOBSIZE + NAND_MAX_PAGESIZE)

/* registers and bit definitions */
#define NDCR		(0x00) /* Control register */
#define NDTR0CS0	(0x04) /* Timing Parameter 0 for CS0 */
#define NDTR1CS0	(0x0C) /* Timing Parameter 1 for CS0 */
#define NDSR		(0x14) /* Status Register */
#define NDPCR		(0x18) /* Page Count Register */
#define NDBDR0		(0x1C) /* Bad Block Register 0 */
#define NDBDR1		(0x20) /* Bad Block Register 1 */
#define NDREDEL		(0x24) /* Read Enable Return Delay Register */
#define NDECCCTRL	(0x28) /* ECC Control Register */
#define NDBZCNT		(0x2C) /* Timer for NDRnB0 and NDRnB1 */
#define NDDB		(0x40) /* Data Buffer */
#define NDCB0		(0x48) /* Command Buffer0 */
#define NDCB1		(0x4C) /* Command Buffer1 */
#define NDCB2		(0x50) /* Command Buffer2 */

#define NDCR_SPARE_EN		(0x1 << 31)
#define NDCR_ECC_EN		(0x1 << 30)
#define NDCR_DMA_EN		(0x1 << 29)
#define NDCR_ND_RUN		(0x1 << 28)
#define NDCR_DWIDTH_C		(0x1 << 27)
#define NDCR_DWIDTH_M		(0x1 << 26)
#define NDCR_PAGE_SZ_MASK	(0x3 << 24)
#define NDCR_PAGE_SZ(x)		(((x) << 24) & NDCR_PAGE_SZ_MASK)
#define NDCR_SEQ_DIS		(0x1 << 23)
#define NDCR_ND_STOP		(0x1 << 22)
#define NDCR_FORCE_CSX		(0x1 << 21)
#define NDCR_CLR_PG_CNT		(0x1 << 20)
#define NDCR_STOP_ON_UNCOR	(0x1 << 19)
#define NDCR_RD_ID_CNT_MASK	(0x7 << 16)
#define NDCR_RD_ID_CNT(x)	(((x) << 16) & NDCR_RD_ID_CNT_MASK)

#define NDCR_RA_START		(0x1 << 15)
#define NDCR_PG_PER_BLK_MASK	(0x3 << 13)
#define NDCR_PG_PER_BLK(x)	(((x) << 13) & NDCR_PG_PER_BLK_MASK)
#define NDCR_ND_ARB_EN		(0x1 << 12)
#define NDCR_INT_MASK           (0xFFF)
#define NDCR_RDYM               (0x1 << 11)
#define NDCR_CS0_PAGEDM         (0x1 << 10)
#define NDCR_CS1_PAGEDM         (0x1 << 9)
#define NDCR_CS0_CMDDM          (0x1 << 8)
#define NDCR_CS1_CMDDM          (0x1 << 7)
#define NDCR_CS0_BBDM           (0x1 << 6)
#define NDCR_CS1_BBDM           (0x1 << 5)
#define NDCR_UNCERRM            (0x1 << 4)
#define NDCR_CORERRM            (0x1 << 3)
#define NDCR_WRDREQM            (0x1 << 2)
#define NDCR_RDDREQM            (0x1 << 1)
#define NDCR_WRCMDREQM          (0x1)

#define NDSR_MASK		(0xffffffff)
#define NDSR_ERR_CNT_MASK       (0x1F << 16)
#define NDSR_ERR_CNT(x)         (((x) << 16) & NDSR_ERR_CNT_MASK)
#define NDSR_RDY                (0x1 << 12)
#define NDSR_FLASH_RDY          (0x1 << 11)
#define NDSR_CS0_PAGED		(0x1 << 10)
#define NDSR_CS1_PAGED		(0x1 << 9)
#define NDSR_CS0_CMDD		(0x1 << 8)
#define NDSR_CS1_CMDD		(0x1 << 7)
#define NDSR_CS0_BBD		(0x1 << 6)
#define NDSR_CS1_BBD		(0x1 << 5)
#define NDSR_DBERR		(0x1 << 4)
#define NDSR_SBERR		(0x1 << 3)
#define NDSR_WRDREQ		(0x1 << 2)
#define NDSR_RDDREQ		(0x1 << 1)
#define NDSR_WRCMDREQ		(0x1)

#define NDCB0_CMD_XTYPE_MASK    (0x7 << 29)
#define NDCB0_CMD_XTYPE(x)      (((x) << 29) & NDCB0_CMD_XTYPE_MASK)
#define NDCB0_LEN_OVRD		(0x1 << 28)
#define NDCB0_ST_ROW_EN         (0x1 << 26)
#define NDCB0_AUTO_RS		(0x1 << 25)
#define NDCB0_CSEL		(0x1 << 24)
#define NDCB0_CMD_TYPE_MASK	(0x7 << 21)
#define NDCB0_CMD_TYPE(x)	(((x) << 21) & NDCB0_CMD_TYPE_MASK)
#define NDCB0_NC		(0x1 << 20)
#define NDCB0_DBC		(0x1 << 19)
#define NDCB0_ADDR_CYC_MASK	(0x7 << 16)
#define NDCB0_ADDR_CYC(x)	(((x) << 16) & NDCB0_ADDR_CYC_MASK)
#define NDCB0_CMD2_MASK		(0xff << 8)
#define NDCB0_CMD1_MASK		(0xff)
#define NDCB0_ADDR_CYC_SHIFT	(16)

/* ECC Control Register */
#define NDECCCTRL_ECC_SPARE_MSK (0xFF << 7)
#define NDECCCTRL_ECC_SPARE(x)  (((x) << 7) & NDECCCTRL_ECC_SPARE_MSK)
#define NDECCCTRL_ECC_THR_MSK   (0x3F << 1)
#define NDECCCTRL_ECC_THRESH(x) (((x) << 1) & NDECCCTRL_ECC_THR_MSK)
#define NDECCCTRL_BCH_EN        (0x1)

/* macros for registers read/write */
#define nand_writel(info, off, val)	\
	__raw_writel((val), (info)->mmio_base + (off))

#define nand_readl(info, off)		\
	__raw_readl((info)->mmio_base + (off))
#define get_mtd_by_info(info)		\
	(struct mtd_info *)((void *)info - sizeof(struct mtd_info))

/* error code and state */
enum {
	ERR_NONE	= 0,
	ERR_DMABUSERR	= -1,
	ERR_SENDCMD	= -2,
	ERR_DBERR	= -3,
	ERR_BBERR	= -4,
	ERR_SBERR	= -5,
};

enum {
	STATE_IDLE = 0,
	STATE_CMD_PREPARED,
	STATE_CMD_HANDLE,
	STATE_DMA_READING,
	STATE_DMA_WRITING,
	STATE_DMA_DONE,
	STATE_PIO_READING,
	STATE_PIO_WRITING,
	STATE_CMD_DONE,
	STATE_READY,
};

struct pxa3xx_nand_info {
	struct nand_chip	nand_chip;
	struct pxa3xx_nand_cmdset *cmdset;
	/* page size of attached chip */
	bool			is_partitioned;
	int			page_addr;
	uint32_t		page_size;
	unsigned int		chip_select;
	unsigned int		ecc_strength;

	/* calculated from pxa3xx_nand_flash data */
	unsigned int		col_addr_cycles;
	unsigned int		row_addr_cycles;

	/* cached register value */
	uint32_t		reg_ndcr;
	uint32_t		ndtr0cs0;
	uint32_t		ndtr1cs0;

	void			*nand_data;
};

struct pxa3xx_nand {
	struct clk		*clk;
	void __iomem		*mmio_base;
	unsigned long		mmio_phys;
	struct nand_hw_control	controller;
	struct completion 	cmd_complete;
	struct platform_device	 *pdev;

	/* DMA information */
	int			drcmr_dat;
	int			drcmr_cmd;
	int 			data_dma_ch;
	dma_addr_t 		dma_buff_phys;
	dma_addr_t 		data_offset;

	struct pxa3xx_nand_info *info[NUM_CHIP_SELECT];
	uint32_t		command;
	uint32_t		data_size;	/* data size in FIFO */
	uint32_t		oob_size;
	unsigned char		*dma_buff;
	unsigned char		*data_buff;
	unsigned char		*oob_buff;

	/* relate to the command */
	unsigned int		chip_select;
	unsigned int		total_cmds;
	unsigned int		state;
	unsigned int		ecc_strength;
	unsigned int		bad_count;
	unsigned int		buf_start;
	unsigned int		buf_count;
	unsigned int		data_column;
	unsigned int		oob_column;
	int			is_ready;
	int			use_dma;	/* use DMA ? */
	int 			retcode;

	/* cached register value */
	unsigned int		cmd_seqs;
	unsigned int		wait_ready[CMD_POOL_SIZE];
	uint32_t		ndcb0[CMD_POOL_SIZE];
	uint32_t		ndcb1[CMD_POOL_SIZE];
	uint32_t		ndcb2;
};

static int use_dma = 0;
module_param(use_dma, bool, 0444);
MODULE_PARM_DESC(use_dma, "enable DMA for data transfering to/from NAND HW");

static int use_polling = 0;
module_param(use_polling, bool, 0444);
MODULE_PARM_DESC(use_polling, "Use full polling mode");
static struct pxa3xx_nand_cmdset default_cmdset = {
	.read1		= 0x3000,
	.read2		= 0x0050,
	.program	= 0x1080,
	.read_status	= 0x0070,
	.read_id	= 0x0090,
	.erase		= 0xD060,
	.reset		= 0x00FF,
	.lock		= 0x002A,
	.unlock		= 0x2423,
	.lock_status	= 0x007A,
};

static struct pxa3xx_nand_timing timing[] = {
	{ 40, 80, 60, 100, 80, 100, 90000, 400, 40, },
	{ 10,  0, 20,  40, 30,  40, 11123, 110, 10, },
	{ 10, 25, 15,  25, 15,  30, 25000,  60, 10, },
	{ 10, 35, 15,  25, 15,  25, 25000,  60, 10, },
	{  5, 15, 10,  10, 10,  10,     0,  80,  5, },
	{  5, 20, 10,  12, 15,  12, 25000,  80, 10, },
	{ 10, 25, 15,  25, 15,  25, 25000,  80, 10, },
};

static struct pxa3xx_nand_flash builtin_flash_types[] = {
{ "DEFAULT FLASH", {0, 0},             0, 2048,  8,  8, 0,    0, &default_cmdset, &timing[0] },
{ "64MiB 16-bit",  {0x46ec, 0},       32,  512, 16, 16, 1, 4096, &default_cmdset, &timing[1] },
{ "256MiB 8-bit",  {0xdaec, 0},       64, 2048,  8,  8, 1, 2048, &default_cmdset, &timing[1] },
{ "1GiB 8-bit",    {0xd3ec, 0},      128, 2048,  8,  8, 4, 4096, &default_cmdset, &timing[1] },
{ "4GiB 8-bit",    {0xd7ec, 0xb655}, 128, 4096,  8,  8, 4, 8192, &default_cmdset, &timing[1] },
{ "4GiB 8-bit",    {0xd7ec, 0x29d5}, 128, 4096,  8,  8, 8, 8192, &default_cmdset, &timing[1] },
{ "512MiB 16-bit", {0xbcec, 0},       64, 4096, 16, 16, 4, 2048, &default_cmdset, &timing[1] },
{ "ST 1Gb 128MiB 8-bit", {0xa120, 0x1500}, 64, 2048,  8,  8, 1, 1024, &default_cmdset, &timing[6] },
{ "Micron 4bit ECC 1Gb 128MiB 8-bit 1.8V", {0xa12c, 0x1580}, 64, 2048,  8,  8, 4, 1024, &default_cmdset, &timing[5] },
{ "128MiB 8-bit",  {0xa12c, 0 },      64, 2048,  8,  8, 1, 1024, &default_cmdset, &timing[2] },
{ "128MiB 16-bit", {0xb12c, 0 },      64, 2048, 16, 16, 1, 1024, &default_cmdset, &timing[2] },
{ "256MiB 16-bit", {0xba2c, 0 },      64, 2048, 16, 16, 1, 2048, &default_cmdset, &timing[2] },
{ "512MiB 8-bit",  {0xdc2c, 0},       64, 2048,  8,  8, 1, 4096, &default_cmdset, &timing[2] },
{ "512MiB 16-bit", {0xcc2c, 0},       64, 2048, 16, 16, 1, 4096, &default_cmdset, &timing[2] },
{ "512MiB 16-bit BCH", {0xbc2c, 0x5590}, 64, 2048, 16, 16, 4, 4096, &default_cmdset, &timing[2] },
{ "1GiB 8-bit",    {0x382c, 0},      128, 4096,  8,  8, 4, 2048, &default_cmdset, &timing[2] },
{ "256MiB 16-bit", {0xba20, 0},       64, 2048, 16, 16, 1, 2048, &default_cmdset, &timing[3] },
{ "512MiB 16-bit", {0xbcad, 0x5510},  64, 2048, 16, 16, 1, 4096, &default_cmdset, &timing[4] },
{ "Hynix 1G 128M 8-bit", {0xa1ad, 0x1500}, 64, 2048,  8,  8, 1, 1024, &default_cmdset, &timing[3] },
};

static struct nand_ecclayout bch_nand_oob_64 = {
	.eccbytes = 32,
	.eccpos = {
		32, 33, 34, 35, 36, 37, 38, 39,
		40, 41, 42, 43, 44, 45, 46, 47,
		48, 49, 50, 51, 52, 53, 54, 55,
		56, 57, 58, 59, 60, 61, 62, 63},
	.oobfree = { {2, 30} }
};

static struct nand_ecclayout bch_nand_oob_128 = {
	.eccbytes = 64,
	.eccpos = {
		64, 65, 66, 67, 68, 69, 70, 71,
		72, 73, 74, 75, 76, 77, 78, 79,
		80, 81, 82, 83, 84, 85, 86, 87,
		88, 89, 90, 91, 92, 93, 94, 95,
		96, 97, 98, 99, 100, 101, 102, 103,
		104, 105, 106, 107, 108, 109, 110, 111,
		112, 113, 114, 115, 116, 117, 118, 119,
		120, 121, 122, 123, 124, 125, 126, 127},
	.oobfree = { {2, 62} }
};

/* Define a default flash type setting serve as flash detecting only */
#define DEFAULT_FLASH_TYPE (&builtin_flash_types[0])

static const char *mtd_names[] = {"pxa3xx_nand-0", "pxa3xx_nand-1", NULL};

#define NDTR0_tCH(c)	(min((c), 7) << 19)
#define NDTR0_tCS(c)	(min((c), 7) << 16)
#define NDTR0_tWH(c)	(min((c), 7) << 11)
#define NDTR0_tWP(c)	(min((c), 7) << 8)
#define NDTR0_tRH(c)	(min((c), 7) << 3)
#define NDTR0_tRP(c)	(min((c), 7) << 0)

#define NDTR1_tR(c)	(min((c), 65535) << 16)
#define NDTR1_tWHR(c)	(min((c), 15) << 4)
#define NDTR1_tAR(c)	(min((c), 15) << 0)

/* convert nano-seconds to nand flash controller clock cycles */
#define ns2cycle(ns, clk)	(int)((ns) * (clk / 1000000) / 1000)
#ifdef CONFIG_PXA95x
static int dvfm_dev_idx;
#else
static struct wake_lock idle_lock;
static struct wake_lock suspend_lock;
static struct pm_qos_request_list *pxa3xx_nand_qos_disable_cpufreq;
static struct pm_qos_request_list *pxa3xx_nand_qos_req_min;
#endif

static dma_addr_t map_addr(struct pxa3xx_nand *nand, void *buf,
			   size_t sz, int dir)
{
	struct device *dev = &nand->pdev->dev;
	/* if not cache aligned, don't use dma */
	if (((size_t)buf & 0x1f) || (sz & 0x1f))
		return ~0;
#ifdef CONFIG_HIGHMEM
	if ((size_t)buf >= PKMAP_ADDR(0) && (size_t)buf < PKMAP_ADDR(LAST_PKMAP)) {
		struct page *page = pte_page(pkmap_page_table[PKMAP_NR((size_t)buf)]);
		return dma_map_page(dev, page, (size_t)buf & (PAGE_SIZE - 1), sz, dir);
	}
#endif
	if (buf >= high_memory) {
		struct page *page;

		if (((size_t) buf & PAGE_MASK) !=
		    ((size_t) (buf + sz - 1) & PAGE_MASK))
			return ~0;

		page = vmalloc_to_page(buf);
		if (!page)
			return ~0;

		if (cache_is_vivt())
			dmac_map_area(buf, sz, dir);
		buf = page_address(page) + ((size_t)buf & ~PAGE_MASK);
	}else if ((size_t)buf >= PKMAP_ADDR(0) && (size_t)buf < PKMAP_ADDR(LAST_PKMAP)) {
		struct page *page = pte_page(pkmap_page_table[PKMAP_NR((size_t)buf)]);
		return dma_map_page(dev, page,(size_t)buf & (PAGE_SIZE - 1), sz, dir);
	}

	return dma_map_single(dev, buf, sz, dir);
}

static void unmap_addr(struct device *dev, dma_addr_t buf, void *orig_buf,
		       size_t sz, int dir)
{
	if (!buf)
		return;
#ifdef CONFIG_HIGHMEM
	if (orig_buf >= high_memory) {
		dma_unmap_page(dev, buf, sz, dir);
		return;
	} else if ((size_t)orig_buf >= PKMAP_ADDR(0) && (size_t)orig_buf < PKMAP_ADDR(LAST_PKMAP)) {
		dma_unmap_page(dev, buf, sz, dir);
		return;
	}
#endif
	dma_unmap_single(dev, buf, sz, dir);
}

static void pxa3xx_nand_set_timing(struct pxa3xx_nand_info *info,
				   const struct pxa3xx_nand_timing *t)
{
	struct pxa3xx_nand *nand = info->nand_data;
	unsigned long nand_clk;
	uint32_t ndtr0, ndtr1;

	nand_clk = clk_get_rate(nand->clk);
	ndtr0 = NDTR0_tCH(ns2cycle(t->tCH, nand_clk)) |
		NDTR0_tCS(ns2cycle(t->tCS, nand_clk)) |
		NDTR0_tWH(ns2cycle(t->tWH, nand_clk)) |
		NDTR0_tWP(ns2cycle(t->tWP, nand_clk)) |
		NDTR0_tRH(ns2cycle(t->tRH, nand_clk)) |
		NDTR0_tRP(ns2cycle(t->tRP, nand_clk));

	ndtr1 = NDTR1_tR(ns2cycle(t->tR, nand_clk)) |
		NDTR1_tWHR(ns2cycle(t->tWHR, nand_clk)) |
		NDTR1_tAR(ns2cycle(t->tAR, nand_clk));

	info->ndtr0cs0 = ndtr0;
	info->ndtr1cs0 = ndtr1;
	nand_writel(nand, NDTR0CS0, ndtr0);
	nand_writel(nand, NDTR1CS0, ndtr1);
}

static void pxa3xx_set_datasize(struct pxa3xx_nand_info *info)
{
	struct pxa3xx_nand *nand = info->nand_data;
	int oob_enable = info->reg_ndcr & NDCR_SPARE_EN;

	if (info->page_size < PAGE_CHUNK_SIZE) {
		nand->data_size = 512;
		if (!oob_enable) {
			nand->oob_size = 0;
			return;
		}

		switch (nand->ecc_strength) {
		case 0:
			nand->oob_size = 16;
			break;
		case HAMMING_STRENGTH:
			nand->oob_size = 8;
			break;
		default:
			printk("Don't support BCH on small page device!!!\n");
			BUG();
		}
		return;
	}
	nand->data_size = PAGE_CHUNK_SIZE;
	if (!oob_enable) {
		nand->oob_size = 0;
		return;
	}

	switch (nand->ecc_strength) {
	case 0:
		nand->oob_size = 64;
		break;
	case HAMMING_STRENGTH:
		nand->oob_size = 40;
		break;
	default:
		nand->oob_size = 32;
	}
}

/**
 * NOTE: it is a must to set ND_RUN firstly, then write
 * command buffer, otherwise, it does not work.
 * We enable all the interrupt at the same time, and
 * let pxa3xx_nand_irq to handle all logic.
 */
static void pxa3xx_nand_start(struct pxa3xx_nand *nand)
{
	struct pxa3xx_nand_info *info;
	uint32_t ndcr, ndeccctrl = 0;

	info = nand->info[nand->chip_select];
	ndcr = info->reg_ndcr;
	ndcr |= nand->use_dma ? NDCR_DMA_EN : 0;
	ndcr |= use_polling ? NDCR_INT_MASK : 0;
	ndcr |= NDCR_ND_RUN;

	switch (nand->ecc_strength) {
	default:
		ndeccctrl |= NDECCCTRL_BCH_EN;
		ndeccctrl |= NDECCCTRL_ECC_THRESH(BCH_THRESHOLD);
	case HAMMING_STRENGTH:
		ndcr |= NDCR_ECC_EN;
	case 0:
		break;
	}
	/* clear status bits and run */
	nand_writel(nand, NDCR, 0);
	nand_writel(nand, NDECCCTRL, ndeccctrl);
	nand_writel(nand, NDCR, ndcr);
	nand_writel(nand, NDSR, ~NDSR_WRCMDREQ);
}

static void pxa3xx_nand_stop(struct pxa3xx_nand *nand)
{
	uint32_t ndcr;
	int timeout = NAND_STOP_DELAY;

	/* wait RUN bit in NDCR become 0 */
	ndcr = nand_readl(nand, NDCR);
	while ((ndcr & NDCR_ND_RUN) && (timeout -- > 0)) {
		ndcr = nand_readl(nand, NDCR);
		udelay(1);
	}

	if (timeout <= 0) {
		ndcr &= ~NDCR_ND_RUN;
		nand_writel(nand, NDCR, ndcr);
	}
	/* clear status bits */
	nand_writel(nand, NDSR, NDSR_MASK);
}

static void enable_int(struct pxa3xx_nand *nand, uint32_t int_mask)
{
	uint32_t ndcr;

	ndcr = nand_readl(nand, NDCR);
	nand_writel(nand, NDCR, ndcr & ~int_mask);
}

static void disable_int(struct pxa3xx_nand *nand, uint32_t int_mask)
{
	uint32_t ndcr;

	ndcr = nand_readl(nand, NDCR);
	nand_writel(nand, NDCR, ndcr | int_mask);
}

static void nand_error_dump(struct pxa3xx_nand *nand)
{
	int i;

	printk(KERN_ERR "NAND controller state wrong!!!\n");
	printk(KERN_ERR "command %x, state %x, current seqs %d, errcode %x, bad count %d\n",
			nand->command, nand->state, nand->cmd_seqs,
			nand->retcode, nand->bad_count);
	printk(KERN_ERR "Totally %d command for sending\n",
			nand->total_cmds);
	for (i = 0; i < nand->total_cmds; i ++)
		printk(KERN_ERR "%d::NDCB0: %x, NDCB1: %x, NDCB2: %x\n",
			i, nand->ndcb0[i], nand->ndcb1[i], nand->ndcb2);

	printk(KERN_ERR "\nRegister DUMPing ##############\n");
	printk(KERN_ERR "NDCR %x\n"
			"NDSR %x\n"
			"NDCB0 %x\n"
			"NDCB1 %x\n"
			"NDCB2 %x\n"
			"NDTR0CS0 %x\n"
			"NDTR1CS0 %x\n"
			"NDBDR0 %x\n"
			"NDBDR1 %x\n"
			"NDREDEL %x\n"
			"NDECCCTRL %x\n"
			"NDBZCNT %x\n\n",
			nand_readl(nand, NDCR),
			nand_readl(nand, NDSR),
			nand_readl(nand, NDCB0),
			nand_readl(nand, NDCB1),
			nand_readl(nand, NDCB2),
			nand_readl(nand, NDTR0CS0),
			nand_readl(nand, NDTR1CS0),
			nand_readl(nand, NDBDR0),
			nand_readl(nand, NDBDR1),
			nand_readl(nand, NDREDEL),
			nand_readl(nand, NDECCCTRL),
			nand_readl(nand, NDBZCNT));
}

static void handle_data_pio(struct pxa3xx_nand *nand)
{
	unsigned int data_size, oob_size;

	data_size = DIV_ROUND_UP(nand->data_size, 4);
	oob_size = DIV_ROUND_UP(nand->oob_size, 4);
	switch (nand->state) {
	case STATE_PIO_WRITING:
		__raw_writesl(nand->mmio_base + NDDB,
			      nand->data_buff + nand->data_column, data_size);
		if (nand->oob_size > 0)
			__raw_writesl(nand->mmio_base + NDDB,
				      nand->oob_buff + nand->oob_column, oob_size);
		break;
	case STATE_PIO_READING:
		__raw_readsl(nand->mmio_base + NDDB,
				nand->data_buff + nand->data_column, data_size);
		if (nand->oob_size > 0)
			__raw_readsl(nand->mmio_base + NDDB,
				      nand->oob_buff + nand->oob_column, oob_size);
		break;
	default:
		printk(KERN_ERR "%s: invalid state %d\n", __func__,
				nand->state);
		BUG();
	}
	nand->data_column += (data_size << 2);
	nand->oob_column += (oob_size << 2);
}

static void start_data_dma(struct pxa3xx_nand *nand)
{
	struct pxa_dma_desc *desc, *desc_oob;
	dma_addr_t data_desc_addr;
	unsigned int data_len = ALIGN(nand->data_size, 32);
	unsigned int oob_len = ALIGN(nand->oob_size, 32);

	data_desc_addr = (dma_addr_t)((void *)nand->dma_buff_phys + NAND_PGSZ);
	desc = (struct pxa_dma_desc *)((void *)nand->dma_buff + NAND_PGSZ);

	desc->ddadr = data_desc_addr + sizeof(struct pxa_dma_desc);
	desc_oob = desc + 1;
	desc_oob->ddadr = DDADR_STOP;

	desc_oob->dcmd = desc->dcmd = DCMD_WIDTH4 | DCMD_BURST32;
	if (!use_polling) {
		desc_oob->dcmd |= DCMD_ENDIRQEN;
		desc->dcmd |= DCMD_ENDIRQEN;
		disable_int(nand, NDCR_INT_MASK);
	}

	switch (nand->state) {
	case STATE_DMA_WRITING:
		desc->dsadr = (volatile u32)nand->data_offset + nand->data_column;
		desc->dcmd |= DCMD_INCSRCADDR | DCMD_FLOWTRG | data_len;
		desc_oob->dsadr = nand->dma_buff_phys + NAND_MAX_PAGESIZE + nand->oob_column;
		desc_oob->dcmd |= DCMD_INCSRCADDR | DCMD_FLOWTRG | oob_len;
		desc_oob->dtadr = desc->dtadr = nand->mmio_phys + NDDB;
		break;
	case STATE_DMA_READING:
		desc->dtadr = (volatile u32)nand->data_offset + nand->data_column;
		desc->dcmd |= DCMD_INCTRGADDR | DCMD_FLOWSRC | data_len;
		desc_oob->dtadr = nand->dma_buff_phys + NAND_MAX_PAGESIZE + nand->oob_column;
		desc_oob->dcmd |= DCMD_INCTRGADDR | DCMD_FLOWSRC | oob_len;
		desc_oob->dsadr = desc->dsadr = nand->mmio_phys + NDDB;
		break;
	default:
		printk(KERN_ERR "%s: invalid state %d\n", __func__,
				nand->state);
		BUG();
	}

	DRCMR(nand->drcmr_dat) = DRCMR_MAPVLD | nand->data_dma_ch;
	DDADR(nand->data_dma_ch) = data_desc_addr;
	DCSR(nand->data_dma_ch) |= DCSR_RUN;
}

static inline void dma_complete(int channel, struct pxa3xx_nand *nand)
{
	uint32_t dcsr;

	dcsr = DCSR(channel);
	DCSR(channel) = dcsr;

	if (dcsr & DCSR_BUSERR) {
		nand->retcode = ERR_DMABUSERR;
	}

	nand->data_column += nand->data_size;
	nand->oob_column += nand->oob_size;
}

static void pxa3xx_nand_data_dma_irq(int channel, void *data)
{
	struct pxa3xx_nand *nand = data;

	dma_complete(channel, nand);

	if (!use_polling)
		enable_int(nand, NDCR_INT_MASK);
	nand_writel(nand, NDSR, NDSR_WRDREQ | NDSR_RDDREQ);
}

static int pxa3xx_nand_transaction(struct pxa3xx_nand *nand)
{
	struct pxa3xx_nand_info *info;
	unsigned int status, is_completed = 0, cs, cmd_seqs;
	unsigned int ready, cmd_done, page_done, badblock_detect, ndcb2;

	cs		= nand->chip_select;
	ready           = (cs) ? NDSR_RDY : NDSR_FLASH_RDY;
	cmd_done        = (cs) ? NDSR_CS1_CMDD : NDSR_CS0_CMDD;
	page_done       = (cs) ? NDSR_CS1_PAGED : NDSR_CS0_PAGED;
	badblock_detect = (cs) ? NDSR_CS1_BBD : NDSR_CS0_BBD;
	info            = nand->info[cs];
	cmd_seqs	= nand->cmd_seqs;

	status = nand_readl(nand, NDSR);
	if (!status)
		return 0;
	nand->bad_count = (status & NDSR_ERR_CNT_MASK) >> 16;
	if (status & badblock_detect) {
		nand->retcode = ERR_BBERR;
		is_completed = 1;
		goto IRQ_FORCE_EXIT;
	}
	if (status & NDSR_SBERR)
		nand->retcode = ERR_SBERR;
	if (status & NDSR_DBERR)
		nand->retcode = ERR_DBERR;
	if (status & (NDSR_RDDREQ | NDSR_WRDREQ)) {
		/* whether use dma to transfer data */
		if (nand->use_dma) {
			nand->state = (status & NDSR_RDDREQ) ?
				      STATE_DMA_READING : STATE_DMA_WRITING;
			start_data_dma(nand);
			if (use_polling) {
				while (!(DCSR(nand->data_dma_ch) & DCSR_STOPSTATE))
					;
				dma_complete(nand->data_dma_ch, nand);
			}
			else
				goto NORMAL_IRQ_EXIT;
		} else {
			nand->state = (status & NDSR_RDDREQ) ?
				      STATE_PIO_READING : STATE_PIO_WRITING;
			handle_data_pio(nand);
		}
	}
	if (status & ready) {
		nand->state = STATE_READY;
		nand->is_ready = 1;
		if (nand->wait_ready[cmd_seqs]) {
			if (!use_polling)
				enable_int(nand, NDCR_INT_MASK);
			if (cmd_seqs == nand->total_cmds)
				is_completed = 1;
		}
	}
	if (nand->wait_ready[cmd_seqs] && (nand->state != STATE_READY)) {
		if (!use_polling)
			disable_int(nand, NDCR_INT_MASK & ~NDCR_RDYM);
		goto NORMAL_IRQ_EXIT;
	}
	if (status & cmd_done) {
		nand->state = STATE_CMD_DONE;
		if (cmd_seqs == nand->total_cmds && !nand->wait_ready[cmd_seqs])
			is_completed = 1;
	}
	if (status & NDSR_WRCMDREQ) {
		status &= ~NDSR_WRCMDREQ;
		nand_writel(nand, NDSR, NDSR_WRCMDREQ);
		if (cmd_seqs < nand->total_cmds) {
			nand->cmd_seqs ++;
			if (cmd_seqs == 0)
				ndcb2 = nand->ndcb2;
			else
				ndcb2 = 0;
			nand->state = STATE_CMD_HANDLE;
			nand_writel(nand, NDCB0, nand->ndcb0[cmd_seqs]);
			nand_writel(nand, NDCB0, nand->ndcb1[cmd_seqs]);
			nand_writel(nand, NDCB0, ndcb2);
			if (nand->ndcb0[cmd_seqs] & NDCB0_LEN_OVRD)
				nand_writel(nand, NDCB0, nand->data_size
						+ nand->oob_size);
		}
		else
			is_completed = 1;
	}

	/* clear NDSR to let the controller exit the IRQ */
IRQ_FORCE_EXIT:
	nand_writel(nand, NDSR, status);
NORMAL_IRQ_EXIT:
	return is_completed;
}

static irqreturn_t pxa3xx_nand_irq(int irq, void *devid)
{
	struct pxa3xx_nand *nand = devid;
	if (pxa3xx_nand_transaction(nand))
		complete(&nand->cmd_complete);
	return IRQ_HANDLED;
}

static int pxa3xx_nand_polling(struct pxa3xx_nand *nand, unsigned long timeout)
{
	int i, ret = 0;

	for (i = 0; i < timeout; i++) {
		ret = pxa3xx_nand_transaction(nand);
		if (ret)
			break;
		udelay(10);
	}

	return ret;
}

static inline int is_buf_blank(uint8_t *buf, size_t len)
{
	for (; len > 0; len--)
		if (*buf++ != 0xff)
			return 0;
	return 1;
}

static int prepare_command_pool(struct pxa3xx_nand *nand, int command,
		uint16_t column, int page_addr)
{
	uint16_t cmd;
	int addr_cycle, exec_cmd, ndcb0, i, chunks = 0;
	struct mtd_info *mtd;
	struct pxa3xx_nand_info *info = nand->info[nand->chip_select];
	struct platform_device *pdev = nand->pdev;
	struct pxa3xx_nand_platform_data *pdata = pdev->dev.platform_data;

	mtd = get_mtd_by_info(info);
	ndcb0 = (nand->chip_select) ? NDCB0_CSEL : 0;;
	addr_cycle = 0;
	exec_cmd = 1;

	/* reset data and oob column point to handle data */
	nand->total_cmds	= 1;
	nand->buf_start		= column;

	switch (command) {
	case NAND_CMD_ERASE2:
		return 0;
	case NAND_CMD_PAGEPROG:
	case NAND_CMD_RNDOUT:
		pxa3xx_set_datasize(info);
		chunks = (info->page_size < PAGE_CHUNK_SIZE) ?
			1 : (info->page_size / PAGE_CHUNK_SIZE);
		if (info->ecc_strength > BCH_STRENGTH) {
			i = info->ecc_strength / BCH_STRENGTH;
			nand->data_size /= i;
			ndcb0 |= NDCB0_LEN_OVRD;
			chunks *= i;
		}
		break;
	case NAND_CMD_READOOB:
		if (info->ecc_strength > BCH_STRENGTH) {
			printk(KERN_ERR "we don't support oob command if use"
					" 8bit per 512bytes ecc feature!!\n");
			BUG();
		}
	default:
		i = (uint32_t)(&nand->state) - (uint32_t)nand;
		memset(&nand->state, 0, sizeof(struct pxa3xx_nand) - i);
		nand->data_size = nand->oob_size = 0;
		break;
	}

	for (i = 0; i < CMD_POOL_SIZE; i ++)
		nand->ndcb0[i] = ndcb0;
	addr_cycle = NDCB0_ADDR_CYC(info->row_addr_cycles
			+ info->col_addr_cycles);

	switch (command) {
	case NAND_CMD_READ0:
	case NAND_CMD_SEQIN:
		nand->buf_count = mtd->writesize;
	case NAND_CMD_READOOB:
		nand->buf_count += mtd->oobsize;
		nand->ecc_strength = info->ecc_strength;
		exec_cmd = 0;
		info->page_addr = page_addr;
		/* small page addr setting */
		if (unlikely(info->page_size < PAGE_CHUNK_SIZE))
			nand->ndcb1[0] = ((page_addr & 0xFFFFFF) << 8)
					| (column & 0xFF);
		else {
			nand->ndcb1[0] = ((page_addr & 0xFFFF) << 16)
					| (column & 0xFFFF);

			if (page_addr & 0xFF0000)
				nand->ndcb2 = (page_addr & 0xFF0000) >> 16;
		}
		break;

	case NAND_CMD_RNDOUT:
		cmd = info->cmdset->read1;
		if (unlikely(info->page_size < PAGE_CHUNK_SIZE)
		    || !(pdata->controller_attrs & PXA3XX_NAKED_CMD_EN)) {
			if (unlikely(info->page_size < PAGE_CHUNK_SIZE))
				nand->ndcb0[0] |= NDCB0_CMD_TYPE(0)
						| addr_cycle
						| (cmd & NDCB0_CMD1_MASK);
			else
				nand->ndcb0[0] |= NDCB0_CMD_TYPE(0)
						| NDCB0_DBC
						| addr_cycle
						| cmd;
			if (nand->command == NAND_CMD_READOOB) {
				nand->buf_start = mtd->writesize + column;
				nand->buf_count = mtd->oobsize;
			}
			break;
		}

		i = 0;
		nand->ndcb0[0] &= ~NDCB0_LEN_OVRD;
		nand->ndcb0[i ++] |= NDCB0_CMD_XTYPE(0x6)
				| NDCB0_CMD_TYPE(0)
				| NDCB0_DBC
				| NDCB0_NC
				| addr_cycle
				| cmd;

		ndcb0 = nand->ndcb0[i] | NDCB0_CMD_XTYPE(0x5) | NDCB0_NC;
		nand->total_cmds = chunks + i;
		for (; i <= nand->total_cmds - 1; i ++)
			nand->ndcb0[i] = ndcb0;
		nand->ndcb0[nand->total_cmds - 1] &= ~NDCB0_NC;

		/* we should wait RnB go high again
		 * before read out data*/
		nand->wait_ready[1] = 1;
		break;

	case NAND_CMD_PAGEPROG:
		if (nand->command == NAND_CMD_NONE) {
			exec_cmd = 0;
			break;
		}

		cmd = info->cmdset->program;
		if (unlikely(info->page_size < PAGE_CHUNK_SIZE)
		    || !(pdata->controller_attrs & PXA3XX_NAKED_CMD_EN)) {
			nand->ndcb0[0] |= NDCB0_CMD_TYPE(0x1)
					| NDCB0_AUTO_RS
					| NDCB0_ST_ROW_EN
					| NDCB0_DBC
					| cmd
					| addr_cycle;
			break;
		}

		nand->total_cmds = chunks + 1;
		nand->ndcb0[0] |= NDCB0_CMD_XTYPE(0x4)
				| NDCB0_CMD_TYPE(0x1)
				| NDCB0_NC
				| NDCB0_AUTO_RS
				| (cmd & NDCB0_CMD1_MASK)
				| addr_cycle;

		for (i = 1; i < chunks; i ++)
			nand->ndcb0[i] |= NDCB0_CMD_XTYPE(0x5)
					| NDCB0_NC
					| NDCB0_AUTO_RS
					| (cmd & NDCB0_CMD1_MASK)
					| NDCB0_CMD_TYPE(0x1);

		nand->ndcb0[chunks] |= NDCB0_CMD_XTYPE(0x3)
				| NDCB0_CMD_TYPE(0x1)
				| NDCB0_ST_ROW_EN
				| NDCB0_DBC
				| (cmd & NDCB0_CMD2_MASK)
				| NDCB0_CMD1_MASK;
		nand->ndcb0[chunks] &= ~NDCB0_LEN_OVRD;
		/* we should wait for RnB goes high which
		 * indicate the data has been written succesfully*/
		nand->wait_ready[nand->total_cmds] = 1;
		break;

	case NAND_CMD_READID:
		cmd = info->cmdset->read_id;
		nand->data_buff = nand->dma_buff;
		nand->buf_count = READ_ID_BYTES;
		nand->ndcb0[0] |= NDCB0_CMD_TYPE(3)
				| NDCB0_ADDR_CYC(1)
				| cmd;

		nand->data_size = 8;
		break;
	case NAND_CMD_STATUS:
		cmd = info->cmdset->read_status;
		nand->data_buff = nand->dma_buff;
		nand->buf_count = 1;
		nand->ndcb0[0] |= NDCB0_CMD_TYPE(4)
				| NDCB0_ADDR_CYC(1)
				| cmd;

		nand->data_size = 8;
		break;

	case NAND_CMD_ERASE1:
		cmd = info->cmdset->erase;
		nand->ndcb0[0] |= NDCB0_CMD_TYPE(2)
				| NDCB0_AUTO_RS
				| NDCB0_ADDR_CYC(3)
				| NDCB0_DBC
				| cmd;
		nand->ndcb1[0] = page_addr;

		break;
	case NAND_CMD_RESET:
		/* on some platform, it is stranger that when issue reset command,
		 * cmd done would not come till timeout cause irq exit.
		 * Force polling mode for reset command*/
		use_polling = 1;
		cmd = info->cmdset->reset;
		nand->ndcb0[0] |= NDCB0_CMD_TYPE(5)
				| cmd;

		break;

	case NAND_CMD_ERASE2:
		exec_cmd = 0;
		break;

	default:
		exec_cmd = 0;
		dev_err(&nand->pdev->dev, "non-supported command %x\n", command);
		break;
	}

	nand->command = command;
	return exec_cmd;
}


static DEFINE_MUTEX(nand_constraints_lock);
static struct work_struct release_constraints_work;
static int constraint_is_set;

void release_constraints_work_func(struct work_struct *work)
{
	mutex_lock(&nand_constraints_lock);

	constraint_is_set = 0;
#ifndef CONFIG_PXA95x
	pm_qos_update_request(pxa3xx_nand_qos_disable_cpufreq,
			PM_QOS_DEFAULT_VALUE);
	pm_qos_update_request(pxa3xx_nand_qos_req_min, PM_QOS_DEFAULT_VALUE);
	wake_unlock(&idle_lock);
	wake_unlock(&suspend_lock);
#else
	dvfm_enable_lowpower(dvfm_dev_idx);
#endif
	mutex_unlock(&nand_constraints_lock);
}

static void timer_nand_constraints(unsigned long data)
{
	schedule_work(&release_constraints_work);
	return;

}
static DEFINE_TIMER(constraints_timer, timer_nand_constraints, 0, 0);
static int constraint_timeout;


static void pxa3xx_nand_cmdfunc(struct mtd_info *mtd, unsigned command,
		int column, int page_addr)
{
	struct pxa3xx_nand_info *info = mtd->priv;
	struct pxa3xx_nand *nand = info->nand_data;
	int ret, exec_cmd, polling_mode;
#ifdef CONFIG_PXA3XX_BBM
	struct pxa3xx_bbm *pxa3xx_bbm = mtd->bbm;
	loff_t addr;

	if (pxa3xx_bbm && (command == NAND_CMD_READOOB
			|| command == NAND_CMD_READ0
			|| command == NAND_CMD_SEQIN
			|| command == NAND_CMD_ERASE1)) {

		addr = (loff_t)page_addr << mtd->writesize_shift;
		addr = pxa3xx_bbm->search(mtd, addr);
		page_addr = addr >> mtd->writesize_shift;
	}
#endif
	mutex_lock(&nand_constraints_lock);
	mod_timer(&constraints_timer, jiffies + constraint_timeout);

	if (constraint_is_set == 0) {
		constraint_is_set = 1;
#ifndef CONFIG_PXA95x
		wake_lock(&idle_lock);
		wake_lock(&suspend_lock);
		pm_qos_update_request(pxa3xx_nand_qos_req_min, 312);
		pm_qos_update_request(pxa3xx_nand_qos_disable_cpufreq, 1);
#else
		dvfm_disable_lowpower(dvfm_dev_idx);
#endif
	}

	/* if this is a x16 device ,then convert the input
	 * "byte" address into a "word" address appropriate
	 * for indexing a word-oriented device
	 */
	if (info->reg_ndcr & NDCR_DWIDTH_M)
		column /= 2;
	/* reset timing */
	if (nand->chip_select != info->chip_select) {
		nand->chip_select = info->chip_select;
		nand_writel(nand, NDTR0CS0, info->ndtr0cs0);
		nand_writel(nand, NDTR1CS0, info->ndtr1cs0);
	}

	polling_mode = use_polling;
	exec_cmd = prepare_command_pool(nand, command, column, page_addr);
	if (exec_cmd) {
		init_completion(&nand->cmd_complete);
		pxa3xx_nand_start(nand);

		if (!use_polling)
			ret = wait_for_completion_timeout(&nand->cmd_complete,
					CHIP_DELAY_TIMEOUT);
		else
			ret = pxa3xx_nand_polling(nand, CHIP_DELAY_TIMEOUT * 100);
		if (!ret) {
			printk(KERN_ERR "Wait time out!!!\n");
			nand_error_dump(nand);
			/* Stop State Machine for next command cycle */
			pxa3xx_nand_stop(nand);
		}
		nand->state = STATE_IDLE;
	}
	use_polling = polling_mode;

	mutex_unlock(&nand_constraints_lock);
}

static uint8_t pxa3xx_nand_read_byte(struct mtd_info *mtd)
{
	struct pxa3xx_nand_info *info = mtd->priv;
	struct pxa3xx_nand *nand = info->nand_data;
	char retval = 0xFF;

	if (nand->buf_start < nand->buf_count)
		/* Has just send a new command? */
		retval = nand->data_buff[nand->buf_start++];

	return retval;
}

static u16 pxa3xx_nand_read_word(struct mtd_info *mtd)
{
	struct pxa3xx_nand_info *info = mtd->priv;
	struct pxa3xx_nand *nand = info->nand_data;
	u16 retval = 0xFFFF;

	if (!(nand->buf_start & 0x01) && nand->buf_start < nand->buf_count) {
		retval = *((u16 *)(nand->data_buff+nand->buf_start));
		nand->buf_start += 2;
	}
	return retval;
}

static void pxa3xx_nand_read_buf(struct mtd_info *mtd, uint8_t *buf, int len)
{
	struct pxa3xx_nand_info *info = mtd->priv;
	struct pxa3xx_nand *nand = info->nand_data;
	int real_len = min_t(size_t, len, nand->buf_count - nand->buf_start);

	memcpy(buf, nand->data_buff + nand->buf_start, real_len);
	nand->buf_start += real_len;
}

static void pxa3xx_nand_write_buf(struct mtd_info *mtd,
		const uint8_t *buf, int len)
{
	struct pxa3xx_nand_info *info = mtd->priv;
	struct pxa3xx_nand *nand = info->nand_data;
	int real_len = min_t(size_t, len, nand->buf_count - nand->buf_start);

	memcpy(nand->data_buff + nand->buf_start, buf, real_len);
	nand->buf_start += real_len;
}

static int pxa3xx_nand_verify_buf(struct mtd_info *mtd,
		const uint8_t *buf, int len)
{
	return 0;
}

static void pxa3xx_nand_select_chip(struct mtd_info *mtd, int chip)
{
	return;
}

static int pxa3xx_nand_waitfunc(struct mtd_info *mtd, struct nand_chip *this)
{
	struct pxa3xx_nand_info *info = mtd->priv;
	struct pxa3xx_nand *nand = info->nand_data;


	if ((nand->command == NAND_CMD_PAGEPROG) && nand->use_dma)
		unmap_addr(&nand->pdev->dev, nand->data_offset,
			   nand->data_buff, mtd->writesize, DMA_TO_DEVICE);

	/* pxa3xx_nand_send_command has waited for command complete */
	if (this->state == FL_WRITING || this->state == FL_ERASING) {
		if (nand->retcode == ERR_NONE)
			return 0;
		else {
			/*
			 * any error make it return 0x01 which will tell
			 * the caller the erase and write fail
			 */
			return 0x01;
		}
	}

	return 0;
}

static int pxa3xx_nand_config_flash(struct pxa3xx_nand_info *info,
				    const struct pxa3xx_nand_flash *f)
{
	struct pxa3xx_nand *nand = info->nand_data;
	struct platform_device *pdev = nand->pdev;
	struct pxa3xx_nand_platform_data *pdata = pdev->dev.platform_data;
	uint32_t ndcr = 0x0; /* enable all interrupts */

	if (f->page_size != 4096 && f->page_size != 2048 && f->page_size != 512)
		return -EINVAL;

	if (f->flash_width != 16 && f->flash_width != 8)
		return -EINVAL;

	if (f->page_size > PAGE_CHUNK_SIZE
			&& !(pdata->controller_attrs & PXA3XX_NAKED_CMD_EN)) {
		printk(KERN_ERR "Your controller don't support 4k or larger "
			       "page NAND for don't support naked command\n");
		return -EINVAL;
	}
	/* calculate flash information */
	if (f->ecc_strength != 0 && f->ecc_strength != HAMMING_STRENGTH
	    && (f->ecc_strength % BCH_STRENGTH != 0)) {
		printk(KERN_ERR "ECC strength definition error, please recheck!!\n");
		return -EINVAL;
	}
	info->ecc_strength = f->ecc_strength;
	info->cmdset = f->cmdset;
	info->page_size = f->page_size;

	/* calculate addressing information */
	info->col_addr_cycles = (f->page_size >= 2048) ? 2 : 1;

	if (f->num_blocks * f->page_per_block > 65536)
		info->row_addr_cycles = 3;
	else
		info->row_addr_cycles = 2;

	ndcr |= (pdata->controller_attrs & PXA3XX_ARBI_EN) ? NDCR_ND_ARB_EN : 0;
	ndcr |= (info->col_addr_cycles == 2) ? NDCR_RA_START : 0;
	ndcr |= (f->flash_width == 16) ? NDCR_DWIDTH_M : 0;
	ndcr |= (f->dfc_width == 16) ? NDCR_DWIDTH_C : 0;
	switch (f->page_per_block) {
	case 32:
		ndcr |= NDCR_PG_PER_BLK(0x0);
		break;
	case 128:
		ndcr |= NDCR_PG_PER_BLK(0x1);
		break;
	case 256:
		ndcr |= NDCR_PG_PER_BLK(0x3);
		break;
	case 64:
	default:
		ndcr |= NDCR_PG_PER_BLK(0x2);
		break;
	}

	switch (f->page_size) {
	case 512:
		ndcr |= NDCR_PAGE_SZ(0x0);
		break;
	case 2048:
	default:
		ndcr |= NDCR_PAGE_SZ(0x1);
		ndcr |= NDCR_FORCE_CSX;
		break;
	}

	ndcr |= NDCR_RD_ID_CNT(READ_ID_BYTES);
	/* only enable spare area when ecc is lower than 8bits per 512 bytes */
	if (f->ecc_strength <= BCH_STRENGTH)
		ndcr |= NDCR_SPARE_EN;

	info->reg_ndcr = ndcr;

	pxa3xx_nand_set_timing(info, f->timing);
	return 0;
}

static int pxa3xx_nand_detect_config(struct pxa3xx_nand *nand)
{
	struct pxa3xx_nand_info *info = nand->info[nand->chip_select];
	uint32_t ndcr = nand_readl(nand, NDCR);
	uint32_t ndeccctrl = nand_readl(nand, NDECCCTRL);

	if (nand->chip_select > 0) {
		printk(KERN_ERR "We could not detect configure if two cs is supported!!\n");
		BUG();
	}
	info->page_size = ndcr & NDCR_PAGE_SZ_MASK ? 2048 : 512;
	/* set info fields needed to read id */
	info->reg_ndcr = ndcr;

	info->ndtr0cs0 = nand_readl(nand, NDTR0CS0);
	info->ndtr1cs0 = nand_readl(nand, NDTR1CS0);
	info->ecc_strength = (ndeccctrl & NDECCCTRL_BCH_EN) ? BCH_STRENGTH : HAMMING_STRENGTH;
	info->cmdset = &default_cmdset;

	return 0;
}

static void free_cs_resource(struct pxa3xx_nand_info *info, int cs)
{
	struct pxa3xx_nand *nand;
	struct mtd_info *mtd;

	if (!info)
		return;

	nand = info->nand_data;
	mtd = get_mtd_by_info(info);
	kfree(mtd);
	nand->info[cs] = NULL;
}

static void pxa3xx_read_page(struct mtd_info *mtd, uint8_t *buf)
{
	struct pxa3xx_nand_info *info = mtd->priv;
	struct nand_chip *chip = mtd->priv;
	struct pxa3xx_nand *nand = info->nand_data;
	dma_addr_t mapped_addr = 0;
	int buf_blank;

	nand->data_buff = (buf) ? buf : nand->dma_buff;
	nand->oob_buff = chip->oob_poi;
	nand->data_offset = 0;
	if (use_dma && buf) {
		mapped_addr = map_addr(nand, (void *)buf,
				mtd->writesize, DMA_FROM_DEVICE);
		if (dma_mapping_error(&nand->pdev->dev, mapped_addr))
			nand->use_dma = 0;
		else {
			nand->use_dma = 1;
			nand->data_offset = mapped_addr;
		}
	}

	pxa3xx_nand_cmdfunc(mtd, NAND_CMD_RNDOUT, 0, info->page_addr);
	if (nand->data_offset)
		unmap_addr(&nand->pdev->dev, nand->data_offset,
			   buf, mtd->writesize, DMA_FROM_DEVICE);
	switch (nand->retcode) {
	case ERR_SBERR:
		switch (nand->ecc_strength) {
		default:
			if (nand->bad_count > BCH_THRESHOLD)
				mtd->ecc_stats.corrected +=
					(nand->bad_count - BCH_THRESHOLD);
			break;
		case HAMMING_STRENGTH:
			mtd->ecc_stats.corrected ++;
		case 0:
			break;
		}
		break;
	case ERR_DBERR:
		buf_blank = is_buf_blank(nand->data_buff, mtd->writesize);
		if (!buf_blank)
			mtd->ecc_stats.failed++;
		break;
	case ERR_NONE:
		break;
	default:
		mtd->ecc_stats.failed++;
		break;
	}
}

static int pxa3xx_nand_read_page_hwecc(struct mtd_info *mtd,
		struct nand_chip *chip, uint8_t *buf, int page)
{
	pxa3xx_read_page(mtd, buf);
	return 0;
}

static int pxa3xx_nand_read_oob(struct mtd_info *mtd, struct nand_chip *chip,
		int page, int sndcmd)
{
	if (sndcmd) {
		pxa3xx_nand_cmdfunc(mtd, NAND_CMD_READOOB, 0, page);
		pxa3xx_read_page(mtd, NULL);
	}
	return 0;
}

static void pxa3xx_nand_write_page_hwecc(struct mtd_info *mtd,
		struct nand_chip *chip, const uint8_t *buf)
{
	struct pxa3xx_nand_info *info = mtd->priv;
	struct pxa3xx_nand *nand = info->nand_data;
	nand->data_buff = (uint8_t *)buf;
	nand->oob_buff = chip->oob_poi;
	if (is_buf_blank((uint8_t *)buf, mtd->writesize) &&
	    is_buf_blank(nand->oob_buff, mtd->oobsize)) {
		nand->command = NAND_CMD_NONE;
		return;
	}

	if (use_dma) {
		nand->data_offset = map_addr(nand, (void *)buf,
				      mtd->writesize, DMA_TO_DEVICE);
		if (dma_mapping_error(&nand->pdev->dev, nand->data_offset))
			nand->use_dma = 0;
		else
			nand->use_dma = 1;
	}
}

static int pxa3xx_nand_write_oob(struct mtd_info *mtd, struct nand_chip *chip,
		int page)
{
	struct pxa3xx_nand_info *info = mtd->priv;
	struct pxa3xx_nand *nand = info->nand_data;
	int status = 0;

	chip->cmdfunc(mtd, NAND_CMD_SEQIN, 0, page);
	memset(nand->dma_buff, 0xff, mtd->writesize);
	pxa3xx_nand_write_page_hwecc(mtd, chip, nand->dma_buff);
	/* Send command to program the OOB data */
	chip->cmdfunc(mtd, NAND_CMD_PAGEPROG, -1, -1);

	status = chip->waitfunc(mtd, chip);

	return status & NAND_STATUS_FAIL ? -EIO : 0;
}

static int pxa3xx_nand_sensing(struct pxa3xx_nand *nand)
{
	struct pxa3xx_nand_info *info = nand->info[nand->chip_select];
	struct mtd_info *mtd = get_mtd_by_info(info);

	/* use the common timing to make a try */
	if (pxa3xx_nand_config_flash(info, &builtin_flash_types[0]))
		return 0;
	pxa3xx_nand_cmdfunc(mtd, NAND_CMD_RESET, 0, 0);
	if (nand->is_ready)
		return 1;
	else
		return 0;
}

static int pxa3xx_nand_scan(struct mtd_info *mtd)
{
	struct pxa3xx_nand_info *info = mtd->priv;
	struct pxa3xx_nand *nand = info->nand_data;
	struct platform_device *pdev = nand->pdev;
	struct pxa3xx_nand_platform_data *pdata = pdev->dev.platform_data;
	struct nand_flash_dev pxa3xx_flash_ids[2] = {{NULL,}, {NULL,}};
	const struct pxa3xx_nand_flash *f = NULL;
	struct nand_chip *chip = mtd->priv;
	uint16_t *id;
	uint64_t chipsize;
	int i, ret, id_check;

	nand->chip_select = info->chip_select;
	if ((pdata->controller_attrs & PXA3XX_KEEP_CONFIG)
	    && !pxa3xx_nand_detect_config(nand))
		goto KEEP_CONFIG;

	ret = pxa3xx_nand_sensing(nand);
	if (!ret) {
		free_cs_resource(info, nand->chip_select);
		return -EINVAL;
	}

	chip->cmdfunc(mtd, NAND_CMD_READID, 0, 0);
	id = (uint16_t *)nand->data_buff;
	if (id[0] != 0) {
		dev_info(&nand->pdev->dev, "Detect a flash id: ");
		for (i = 0; i < ID_CHECK_RANGE; i ++)
			printk("%x ", id[i]);
		printk("\n");
	}
	else {
		printk(KERN_WARNING "Read out ID 0, potential timing set wrong!!\n");
		free_cs_resource(info, nand->chip_select);
		return -EINVAL;
	}

	for (i=0; i<ARRAY_SIZE(builtin_flash_types) + pdata->num_flash - 1; i++) {
		if (i < pdata->num_flash)
			f = pdata->flash + i;
		else
			f = &builtin_flash_types[i - pdata->num_flash + 1];

		/* find the chip in default list */
		for (id_check = 0; id_check < ID_CHECK_RANGE; id_check ++)
			if (f->chip_id[id_check] && f->chip_id[id_check] != id[id_check])
				break;
		if (id_check == ID_CHECK_RANGE)
			break;
	}

	if (i >= (ARRAY_SIZE(builtin_flash_types) + pdata->num_flash - 1)) {
		dev_err(&nand->pdev->dev, "ERROR!! flash not defined!!!\n");
		free_cs_resource(info, nand->chip_select);
		return -EINVAL;
	}

	if (pxa3xx_nand_config_flash(info, f))
		return -EINVAL;
	pxa3xx_flash_ids[0].name = f->name;
	pxa3xx_flash_ids[0].id = (f->chip_id[0] >> 8) & 0xffff;
	pxa3xx_flash_ids[0].pagesize = f->page_size;
	chipsize = (uint64_t)f->num_blocks * f->page_per_block * f->page_size;
	pxa3xx_flash_ids[0].chipsize = chipsize >> 20;
	pxa3xx_flash_ids[0].erasesize = f->page_size * f->page_per_block;
	pxa3xx_flash_ids[0].options = (f->flash_width == 16) ? NAND_BUSWIDTH_16 : 0;
	chip->options |= pxa3xx_flash_ids[0].options;

	if (f->ecc_strength > 1) {
		switch (f->page_size) {
		case 2048:
			chip->ecc.layout = &bch_nand_oob_64;
			break;
		case 4096:
			chip->ecc.layout = &bch_nand_oob_128;
			break;
		default:
			BUG();
		}
	}
KEEP_CONFIG:
	if (nand_scan_ident(mtd, 1, pxa3xx_flash_ids))
		return -ENODEV;
	/* calculate addressing information */
	info->col_addr_cycles = (mtd->writesize >= 2048) ? 2 : 1;
	if ((mtd->size >> chip->page_shift) > 65536)
		info->row_addr_cycles = 3;
	else
		info->row_addr_cycles = 2;
	info->page_size = mtd->writesize;
	mtd->name = mtd_names[nand->chip_select];
	chip->ecc.mode = NAND_ECC_HW;
	chip->ecc.size = mtd->writesize;
	chip->options |= (info->reg_ndcr & NDCR_DWIDTH_M) ? NAND_BUSWIDTH_16: 0;
	chip->options |= NAND_NO_READRDY;
	chip->options |= NAND_USE_FLASH_BBT;
#ifdef CONFIG_PXA3XX_BBM
	chip->options |= BBT_RELOCATION_IFBAD;
#endif
	mtd->writesize_shift = ffs(mtd->writesize) - 1;
	mtd->writesize_mask = (1 << mtd->writesize_shift) - 1;
	mtd->erasesize_shift = ffs(mtd->erasesize) - 1;
	mtd->erasesize_mask = (1 << mtd->erasesize_shift) - 1;
	ret = nand_scan_tail(mtd);
	chip->oob_poi = nand->dma_buff + NAND_MAX_PAGESIZE;

	return ret;
}

static struct pxa3xx_nand *alloc_nand_resource(struct platform_device *pdev)
{
	struct pxa3xx_nand_info *info;
	struct nand_chip *chip;
	struct mtd_info *mtd;
	struct pxa3xx_nand *nand;
	struct resource *r;
	int ret, irq, cs;

	nand = kzalloc(sizeof(struct pxa3xx_nand), GFP_KERNEL);
	if (!nand) {
		dev_err(&pdev->dev, "failed to allocate memory\n");
		return NULL;
	}

	nand->pdev = pdev;
	nand->clk = clk_get(&pdev->dev, NULL);
	if (IS_ERR(nand->clk)) {
		dev_err(&pdev->dev, "failed to get nand clock\n");
		ret = PTR_ERR(nand->clk);
		goto fail_alloc;
	}
	clk_enable(nand->clk);

	r = platform_get_resource(pdev, IORESOURCE_DMA, 0);
	if (r == NULL) {
		dev_err(&pdev->dev, "no resource defined for data DMA\n");
		ret = -ENXIO;
		goto fail_put_clk;
	}
	nand->drcmr_dat = r->start;

	r = platform_get_resource(pdev, IORESOURCE_DMA, 1);
	if (r == NULL) {
		dev_err(&pdev->dev, "no resource defined for command DMA\n");
		ret = -ENXIO;
		goto fail_put_clk;
	}
	nand->drcmr_cmd = r->start;

	r = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (r == NULL) {
		dev_err(&pdev->dev, "no IO memory resource defined\n");
		ret = -ENODEV;
		goto fail_put_clk;
	}

	r = request_mem_region(r->start, resource_size(r), pdev->name);
	if (r == NULL) {
		dev_err(&pdev->dev, "failed to request memory resource\n");
		ret = -EBUSY;
		goto fail_put_clk;
	}

	nand->mmio_base = ioremap(r->start, resource_size(r));
	if (nand->mmio_base == NULL) {
		dev_err(&pdev->dev, "ioremap() failed\n");
		ret = -ENODEV;
		goto fail_free_res;
	}

	nand->mmio_phys = r->start;
	/* initialize all interrupts to be disabled */
	irq = platform_get_irq(pdev, 0);
	disable_int(nand, NDCR_INT_MASK);
	if (irq < 0) {
		dev_err(&pdev->dev, "no IRQ resource defined\n");
		ret = -ENXIO;
		goto fail_put_clk;
	}

	ret = request_irq(irq, pxa3xx_nand_irq, IRQF_DISABLED,
			  pdev->name, nand);
	if (ret < 0) {
		dev_err(&pdev->dev, "failed to request IRQ\n");
		ret = -ENXIO;
		goto fail_free_irq;
	}

	platform_set_drvdata(pdev, nand);
	spin_lock_init(&nand->controller.lock);
	init_waitqueue_head(&nand->controller.wq);
	for (cs = 0; cs < NUM_CHIP_SELECT; cs++) {
		mtd = kzalloc(sizeof(struct mtd_info)
				+ sizeof(struct pxa3xx_nand_info),
				GFP_KERNEL);
		if (!mtd) {
			dev_err(&pdev->dev, "failed to allocate memory\n");
			ret = -ENOMEM;
			goto fail_free_irq;
		}

		info = (struct pxa3xx_nand_info *)(&mtd[1]);
		info->nand_data = nand;
		info->chip_select = cs;
		mtd->priv = info;
		mtd->owner = THIS_MODULE;
		nand->info[cs] = info;

		chip = (struct nand_chip *)(&mtd[1]);
		chip->controller        = &nand->controller;
		chip->ecc.read_page	= pxa3xx_nand_read_page_hwecc;
		chip->ecc.read_page_raw = pxa3xx_nand_read_page_hwecc;
		chip->ecc.read_oob      = pxa3xx_nand_read_oob;
		chip->ecc.write_page	= pxa3xx_nand_write_page_hwecc;
		chip->ecc.write_page_raw= pxa3xx_nand_write_page_hwecc;
		chip->ecc.write_oob     = pxa3xx_nand_write_oob;
		chip->waitfunc		= pxa3xx_nand_waitfunc;
		chip->select_chip	= pxa3xx_nand_select_chip;
		chip->cmdfunc		= pxa3xx_nand_cmdfunc;
		chip->read_word		= pxa3xx_nand_read_word;
		chip->read_byte		= pxa3xx_nand_read_byte;
		chip->read_buf		= pxa3xx_nand_read_buf;
		chip->write_buf		= pxa3xx_nand_write_buf;
		chip->verify_buf	= pxa3xx_nand_verify_buf;
#ifdef CONFIG_PXA3XX_BBM
		chip->scan_bbt		= pxa3xx_scan_bbt;
		chip->block_markbad	= pxa3xx_block_markbad;
		chip->block_bad		= pxa3xx_block_bad;
#endif
	}

	nand->dma_buff = dma_alloc_coherent(&pdev->dev, NAND_PGSZ + DMA_H_SIZE,
			&nand->dma_buff_phys, GFP_KERNEL);
	if (nand->dma_buff == NULL) {
		dev_err(&pdev->dev, "failed to allocate dma buffer\n");
		ret = -ENOMEM;
		goto fail_free_buf;
	}

	nand->data_dma_ch = pxa_request_dma("nand-data", DMA_PRIO_LOW,
			pxa3xx_nand_data_dma_irq, nand);
	if (nand->data_dma_ch < 0) {
		dev_err(&pdev->dev, "failed to request data dma\n");
		ret = -ENXIO;
		goto fail_free_buf;
	}
	return nand;

fail_free_buf:
	for (cs = 0; cs < NUM_CHIP_SELECT; cs++) {
		info = nand->info[cs];
		free_cs_resource(info, cs);
	}
fail_free_irq:
	free_irq(irq, nand);
	iounmap(nand->mmio_base);
fail_free_res:
	release_mem_region(r->start, resource_size(r));
fail_put_clk:
	clk_disable(nand->clk);
	clk_put(nand->clk);
fail_alloc:
	kfree(nand);
	return NULL;
}

static int pxa3xx_nand_remove(struct platform_device *pdev)
{
	struct pxa3xx_nand *nand = platform_get_drvdata(pdev);
	struct pxa3xx_nand_info *info = nand->info[nand->chip_select];
	struct mtd_info *mtd = get_mtd_by_info(info);
	struct resource *r;
	int irq, cs;

	platform_set_drvdata(pdev, NULL);

	irq = platform_get_irq(pdev, 0);
	if (irq >= 0)
		free_irq(irq, nand);

	iounmap(nand->mmio_base);
	r = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	release_mem_region(r->start, resource_size(r));

	clk_disable(nand->clk);
	clk_put(nand->clk);

	for (cs = 0; cs < NUM_CHIP_SELECT; cs ++) {
		info = nand->info[cs];
		if (!info)
			continue;
		mtd = get_mtd_by_info(info);
#ifdef CONFIG_PXA3XX_BBM
		if (mtd->bbm)
			((struct pxa3xx_bbm *)mtd->bbm)->uninit(mtd);
#endif
#ifdef CONFIG_MTD_PARTITIONS
		if (info->is_partitioned)
			del_mtd_partitions(mtd);
		del_mtd_device(mtd);
#endif
		free_cs_resource(info, cs);
	}
	if (nand->dma_buff_phys) {
		if (nand->data_dma_ch >= 0)
			pxa_free_dma(nand->data_dma_ch);
		if (nand->dma_buff)
			dma_free_coherent(&nand->pdev->dev, NAND_PGSZ + DMA_H_SIZE,
					nand->dma_buff, nand->dma_buff_phys);
		nand->dma_buff_phys = 0;
	}
	return 0;
}

static int pxa3xx_nand_probe(struct platform_device *pdev)
{
	struct pxa3xx_nand_platform_data *pdata;
	struct pxa3xx_nand_info *info;
	struct pxa3xx_nand *nand;
	struct mtd_info *mtd;
	static const char *probes[] = { "cmdlinepart", NULL };
	struct mtd_partition *parts = NULL, *tmp;
	int nr_parts, ret, cs, probe_success = 0, cs_to_scan;
	pdata = pdev->dev.platform_data;
	if (!pdata) {
		dev_err(&pdev->dev, "no platform data defined\n");
		return -ENODEV;
	}

	INIT_WORK(&release_constraints_work, release_constraints_work_func);
	constraint_timeout = usecs_to_jiffies(1000);

	if (pdata->controller_attrs & PXA3XX_POLLING_MODE)
		use_polling = 1;
	if (pdata->controller_attrs & PXA3XX_DMA_EN)
		use_dma = 1;
	nand = alloc_nand_resource(pdev);
	if (nand == NULL)
		return -ENOMEM;

	cs_to_scan = (pdata->controller_attrs & PXA3XX_TWO_CHIP_EN)? 2 : 1;
	for (cs = 0; cs < cs_to_scan; cs++) {
		info = nand->info[cs];
		info->is_partitioned = false;
		mtd = get_mtd_by_info(info);
		nand->chip_select = cs;
		if (pxa3xx_nand_scan(mtd)) {
			dev_err(&pdev->dev, "failed to scan cs#%d nand\n", cs);
			continue;
		}
		ret = nr_parts = 0;
#ifdef CONFIG_MTD_PARTITIONS
		if (mtd_has_cmdlinepart())
			nr_parts = parse_mtd_partitions(mtd, probes, &parts, 0);
		if (!nr_parts) {
			nr_parts = pdata->nr_parts[cs];
			parts = (struct mtd_partition *)pdata->parts[cs];
		}

		if (nr_parts) {
#ifdef CONFIG_PXA3XX_BBM
			struct pxa3xx_bbm *pxa3xx_bbm = mtd->bbm;
			tmp = pxa3xx_bbm->check_partition(mtd, parts, &nr_parts);
			if (!tmp)
				return -EINVAL;
			if (parts != pdata->parts[cs])
				kfree(parts);

			parts = tmp;
#endif
			ret = add_mtd_partitions(mtd, parts, nr_parts);
			if (parts && (parts != pdata->parts[cs]))
				kfree(parts);
			info->is_partitioned = true;
		}
#endif
		if (!nr_parts)
			ret = add_mtd_device(mtd);
		if (!ret)
			probe_success = 1;
	}

	if (!probe_success) {
		pxa3xx_nand_remove(pdev);
		return -ENODEV;
	}
	return 0;
}

#if (defined CONFIG_PM) && (!defined CONFIG_PXA95x_SUSPEND)
static int pxa3xx_nand_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct pxa3xx_nand *nand= platform_get_drvdata(pdev);
	struct pxa3xx_nand_platform_data *pdata;
	struct pxa3xx_nand_info *info;
	struct mtd_info *mtd;
	int ret = 0;
	uint8_t cs, cs_to_scan;

	pdata = pdev->dev.platform_data;
	cs_to_scan = (pdata->controller_attrs & PXA3XX_TWO_CHIP_EN)? 2 : 1;
	if (nand->state & STATE_CMD_PREPARED) {
		dev_err(&pdev->dev, "driver busy, state = %d\n", nand->state);
		return -EAGAIN;
	}

	for (cs = 0; cs < cs_to_scan; cs ++) {
		info = nand->info[cs];
		if (!info)
			continue;
		mtd = get_mtd_by_info(info);
		ret = mtd->suspend(mtd);
	}

	clk_disable(nand->clk);
	return ret;
}

static int pxa3xx_nand_resume(struct platform_device *pdev)
{
	struct pxa3xx_nand *nand= platform_get_drvdata(pdev);
	struct pxa3xx_nand_platform_data *pdata;
	struct pxa3xx_nand_info *info;
	struct mtd_info *mtd;
	uint8_t cs, cs_to_scan;

	disable_int(nand, NDCR_INT_MASK);
	/* set the controller cs to a invalid num to let driver
	 * reconfigure the timing when it call the cmdfunc */
	nand->chip_select = 0xff;
	clk_enable(nand->clk);
	/*
	 * As the spec, the NDSR would be updated to 0x1800 when
	 * do the nand_clk disable/enable.
	 * To prevent it damage state machine of the driver, clear
	 * all status before resume
	 */
	nand_writel(nand, NDSR, NDSR_MASK);

	pdata = pdev->dev.platform_data;
	cs_to_scan = (pdata->controller_attrs & PXA3XX_TWO_CHIP_EN)? 2 : 1;
	for (cs = 0; cs < cs_to_scan; cs ++) {
		info = nand->info[cs];
		if (!info)
			continue;
		mtd = get_mtd_by_info(info);
		mtd->resume(mtd);
	}

	return 0;
}
#else
#define pxa3xx_nand_suspend	NULL
#define pxa3xx_nand_resume	NULL
#endif

static struct platform_driver pxa3xx_nand_driver = {
	.driver = {
		.name	= "pxa3xx-nand",
	},
	.probe		= pxa3xx_nand_probe,
	.remove		= pxa3xx_nand_remove,
	.suspend	= pxa3xx_nand_suspend,
	.resume		= pxa3xx_nand_resume,
};

static int __init pxa3xx_nand_init(void)
{
#ifdef CONFIG_PXA95x
	dvfm_register("NAND", &dvfm_dev_idx);
#else
	wake_lock_init(&idle_lock, WAKE_LOCK_IDLE, "pxa3xx_nand_idle");
	wake_lock_init(&suspend_lock, WAKE_LOCK_SUSPEND, "pxa3xx_nand_idle");

	pxa3xx_nand_qos_req_min = pm_qos_add_request(PM_QOS_CPUFREQ_MIN,
			PM_QOS_DEFAULT_VALUE);
	pxa3xx_nand_qos_disable_cpufreq = pm_qos_add_request(
			PM_QOS_CPUFREQ_DISABLE, PM_QOS_DEFAULT_VALUE);
#endif

	return platform_driver_register(&pxa3xx_nand_driver);
}
module_init(pxa3xx_nand_init);

static void __exit pxa3xx_nand_exit(void)
{
#ifdef CONFIG_PXA95x
	dvfm_unregister("NAND", &dvfm_dev_idx);
#else
	wake_lock_destroy(&idle_lock);
	wake_lock_destroy(&suspend_lock);
#endif
	platform_driver_unregister(&pxa3xx_nand_driver);
}
module_exit(pxa3xx_nand_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("PXA3xx NAND controller driver");
