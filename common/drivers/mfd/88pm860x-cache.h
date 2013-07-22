/*
 * PMIC 88PM860x register pmic_cache
 *
 * Copyright (C) 2009 Marvell International Ltd.
 * Yan Markman <hymarkman@marvell.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * DESCRIPTION:
 *  The access to the PMIC registers is over I2C.
 *  To reduce the number of long external transactions
 *  some registers may be cacheable.
 *
 *     The APIs are:
 *   pmic_cache_init(id)
 * if( pmic_cache_hit_before_read(id, reg, count, *pdata)
 *     pmic_cache_save_after_readwrite(id, ret, reg, count, *pdata)
 * if( pmic_cache_hit_before_write(id, reg, count, *pdata)
 *     pmic_cache_save_after_readwrite(id, ret, reg, count, *pdata)
 *     pmic_cache_invalidate(id, reg)
 */
#if !defined(CACHE_88PM860X_I2C)
/* To reuse this module for another PMIC modify the
 *     pm8607_cache[ XXXX ].cachable = 1;
 * NOTE:
 *  88PM860X has special "test-page" mode.
 *  In this mode the same Reg-Offsest reference different registers.
 *  To make them cacheable the special array pm8607_cache_regs_page[]
 *  and procedures used.
 *  This may be irrelevant for other PMICs. Refer the PM8607_REG_NUM_PAGE
 */
#else

#include <linux/mfd/88pm860x.h>

#define PM8606_REG_NUM			0x20
#define PM8607_REG_NUM			256 /*0..0xff*/

#define PM8607_PAGE_ID			0
#define PM8607_PAGE_CACHE_BASE	0xB0
#define PM8607_PAGE_CACHE_END	0xB7
#define PM8607_REG_NUM_PAGE (PM8607_PAGE_CACHE_END-PM8607_PAGE_CACHE_BASE+1)


struct pmic_cache_entry_line {
	u8  hit;
	u8  cachable;
	u8  data;
	u8  data1; /*spare*/
};

static struct pmic_cache_entry_line   pm8606_cache[PM8606_REG_NUM];
static struct pmic_cache_entry_line   pm8607_cache[PM8607_REG_NUM];
static struct pmic_cache_entry_line   pm8607_cache_page[PM8607_REG_NUM_PAGE];

static struct pmic_cache_entry_line  *pmic_cache[] = /* [0],[1],[2] */
					{ pm8607_cache_page, pm8606_cache, pm8607_cache };
static int pmic_cache_initialized;

static void pmic_cache_invalidate(int id, int reg);

static u32 pmic_cache_stat_miss;
static u32 pmic_cache_stat_hit;

static void pmic_cache_stat_print(int reset_req)
{
	if (pmic_cache_stat_hit == 0)
		pmic_cache_stat_hit = 1;  /* avoids div0 */
	printk(KERN_INFO "\n88pm860x-pmic_cache: miss/hit=%u/%u = %u.%u\n",
		pmic_cache_stat_miss, pmic_cache_stat_hit,
		pmic_cache_stat_miss / pmic_cache_stat_hit,
		pmic_cache_stat_miss % pmic_cache_stat_hit);
	if (reset_req) {
		pmic_cache_stat_miss = 0;
		pmic_cache_stat_hit  = 0;
	}
}

static void __init pmic_cache_init(int id)
{
	int i, size;
	u8 pm8606_noncacheable_regs[] = {
		1, 7, 0x1A, 0x1B, 0x1C/*Protection & Protected*/,
		0x15, 0x16,
		0x18, 0x19/*Status*/,
		0x10, 0x11/*Reset outside*/,
		0xCB, 0xCC, 0xCD, 0xCE, 0xCF, 0xE2, 0xE3, 0xE4, 0xE5, 0xE6
		};

	u8 pm8607_noncacheable_regs[] = {
	0x01, 0x02, 0x03, 0x04, 0x05 /*:Status */, 0x0D/*:unused*/, 0x13/*0x1D LDO4-USIM Apps/Com*/,
		/* Measurement: */
	0x2B,0x6B, 0x6C, 0x6D, 0x6E, 0x6F,
	0x70, 0x71, 0x72, 0x73, 0x74, 0x75, 0x76, 0x77, 0x78, 0x79, 0x7A, 0x7B, 0x7C, 0x7D, 0x7E, 0x7F,
	0x80, 0x81, 0x82, 0x83, 0x84, 0x85, 0x86, 0x87, 0x88, 0x89, 0x8A, 0x8B, 0x8C, 0x8D, 0x8E, 0x8F,
	0x90, 0x91, 0x92, 0x93, 0x94, 0x95, 0x96, 0x97, 0x98, 0x99, 0x9A, 0x9B, 0x9C, 0x9D, 0x9E, 0x9F,
		/* RTC Counter: */
	0xA1, 0xA2, 0xA3, 0xA4,
		/* Unused or special: */
	0xEB, 0xEE, 0xEF,
	0xF0, 0xF1, 0xF2, 0xF3, 0xF4, 0xF5, 0xF6, 0xF7, 0xF8, 0xF9, 0xFA, 0xFB, 0xFC, 0xFD, 0xFE, 0xFF,

	0x3A /* hw issue fix from pm860x C0 unexpected BAT interrupt when chip-sleep */
	};
	/* pm8607_cache_page[] - all registers are B0, B1...B7 cacheable */

	if ((CHIP_PM8606 != 1) || (CHIP_PM8607 != 2)) {
		printk(KERN_ERR "ERROR: 88pm860x-pmic_cache disabled CHIP_PM8606=%d, CHIP_PM8607=%d\n",
				CHIP_PM8606, CHIP_PM8607);
		return;
	}
	if ((id != CHIP_PM8607) && (id != CHIP_PM8606)) {
		WARN_ON((id != CHIP_PM8607) && (id != CHIP_PM8606));
		return;
	}
	if (pmic_cache_initialized)
		return;

	/* Set all cachable */
	for (i = 0; i < PM8606_REG_NUM; i++)
		pm8606_cache[i].cachable = 1;
	for (i = 0; i < PM8607_REG_NUM; i++)
		pm8607_cache[i].cachable = 1;
	for (i = 0; i < PM8607_REG_NUM_PAGE; i++)
		pm8607_cache_page[i].cachable = 1;

	/* Update non-cachable */
	size = sizeof(pm8606_noncacheable_regs)/
			sizeof(pm8606_noncacheable_regs[0]);
	for (i = 0; i < size; i++)
		pm8607_cache[pm8606_noncacheable_regs[i]].cachable = 0;

	size = sizeof(pm8607_noncacheable_regs)/
			sizeof(pm8607_noncacheable_regs[0]);
	for (i = 0; i < size; i++)
		pm8607_cache[pm8607_noncacheable_regs[i]].cachable = 0;

	pmic_cache_stat_print(0);
}

/***************************************************************************
* Cache operations below are:
*  read  before - check HIT and return value, caller should not start I2C
*  read  after  - save read-value after I2C transaction
*  write before - check HIT, caller should not start I2C
*  write after  - save written-value after I2C transaction
*/
static int pmic_cache_hit_before_read(int id, int reg, u8 count, u8 *pdata)
{
	int i;
	if (unlikely(count == 0)) {
		pmic_cache_stat_miss++;
		return -1;
	}
	if (id == PM8607_PAGE_ID) {
		if ((reg < PM8607_PAGE_CACHE_BASE) ||
			(reg > PM8607_PAGE_CACHE_END) ||
			(count > PM8607_REG_NUM_PAGE)) {
			pmic_cache_stat_miss++;
			return -1;
		}
		reg -= PM8607_PAGE_CACHE_BASE;
	}

	/* The pmic_cache[].data should be updated ONLY if
	 *   ALL pmic_cache[].hit have been successful.
	 * So two for-loops are required:
	 *   one to check all hits and another to update data.
	 * Do NOT try to optimize with one for-loop!
	 */
	for (i = 0; i < count; i++) {
		if (!pmic_cache[id][reg+i].hit) {
			pmic_cache_stat_miss++;
			return -1;
		}
	}
	for (i = 0; i < count; i++)
		pdata[i] = pmic_cache[id][reg+i].data;
	pmic_cache_stat_hit++;
	return count;
}

static int pmic_cache_hit_before_write(int id, int reg, u8 count, u8 *pdata)
{
	int i;
	if (count == 0) {
		pmic_cache_stat_miss++;
		return -1;
	}
	if (id == PM8607_PAGE_ID) {
		if ((reg < PM8607_PAGE_CACHE_BASE) ||
			(reg > PM8607_PAGE_CACHE_END) ||
			(count > PM8607_REG_NUM_PAGE)) {
			pmic_cache_stat_miss++;
			return -1;
		}
		reg -= PM8607_PAGE_CACHE_BASE;
	}

	for (i = 0; i < count; i++) {
		if (!pmic_cache[id][reg+i].hit ||
			(pmic_cache[id][reg+i].data != pdata[i])) {
			pmic_cache_stat_miss++;
			return -1;
		}
	}
	pmic_cache_stat_hit++;
	return count;
}

static void pmic_cache_save_after_readwrite(
				int id, int i2c_ret, int reg, u8 count, u8 *pdata)
{
	int i;
	if (count == 0)
		return;
	if (id == PM8607_PAGE_ID) {
		if ((reg < PM8607_PAGE_CACHE_BASE) ||
			(reg > PM8607_PAGE_CACHE_END) ||
			(count > PM8607_REG_NUM_PAGE))
			return;
		reg -= PM8607_PAGE_CACHE_BASE;
	}

	if (i2c_ret >= 0) {
		for (i = 0; i < count; i++) {
			pmic_cache[id][reg+i].data = pdata[i];
			pmic_cache[id][reg+i].hit =
				pmic_cache[id][reg+i].cachable;
		}
	} else {
		for (i = 0; i < count; i++)
			pmic_cache[id][reg+i].hit = 0;
	}
}

static void pmic_cache_invalidate(int id, int reg)
{
	if (id == PM8607_PAGE_ID) {
		if ((reg < PM8607_PAGE_CACHE_BASE) ||
			(reg > PM8607_PAGE_CACHE_END))
			return;
		reg -= PM8607_PAGE_CACHE_BASE;
	}
	pmic_cache[id][reg].hit = 0;
}

#endif/*CACHE_88PM860X_I2C*/
