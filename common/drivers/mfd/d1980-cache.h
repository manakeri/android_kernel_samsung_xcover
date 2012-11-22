/*
 * PMIC D1980 register pmic_cache
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
#if !defined(CACHE_D1980_I2C)
/* To reuse this module for another PMIC modify the
 *     d1980_cache[ XXXX ].cachable = 1;
 */
#else

#include <linux/d1982/d1980_reg.h>

#define PMIC_ID                     0x0
#define D1980_REG_NUM               D1980_PAGE1_REG_END  /* 142(0x8E) */

struct pmic_cache_entry_line {
	u8  hit;
	u8  cachable;
	u8  data;
	u8  data1; /*spare*/
};

static struct pmic_cache_entry_line   d1980_cache[D1980_REG_NUM];

static struct pmic_cache_entry_line  *pmic_cache[] = /* [0] */
					{ d1980_cache };
static int pmic_cache_initialized;

static void pmic_cache_invalidate(int id, int reg);

static u32 pmic_cache_stat_miss;
static u32 pmic_cache_stat_hit;

static void pmic_cache_stat_print(int reset_req)
{
	if (pmic_cache_stat_hit == 0)
		pmic_cache_stat_hit = 1;  /* avoids div0 */
	printk(KERN_INFO "\nd1980-pmic_cache: miss/hit=%u/%u = %u.%u\n",
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

    // TODO: 21/Jul/2011. Check d1980_noncacheable_regs array.
    u8 d1980_noncacheable_regs[] = {
        0x00, 
        /* Status Register */
        0x01, 0x02, 0x03, 0x04,
        /* Event & Mask Register */
        0x05, 0x06, 0x07, 0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0F, 0x11,
        /* Interface & Etc */
        0x13, 0x46, 0x4E, 0x4F, 0x50, 0x51,
        /* Mesaurement */
        0x53, 0x54, 0x55, 0x57, 0x58, 0x59, 0x5A, 0x5C, 0x5F, 0x60, 0x61,
        0x62, 0x63, 0x64, 0x65, 0x66, 0x67, 0x68, 0x69, 0x6A, 0x6B, 0x6C,
        0x6D, 0x6E,
        /* Reserved */
        0x7B, 0x7C, 0x7D, 0x7E, 0x7F, 0x82
    };

	if(id != PMIC_ID) {
	    WARN_ON(id != PMIC_ID);
	    return;
    }
	if (pmic_cache_initialized)
		return;

    memset(d1980_cache, 0, sizeof(struct pmic_cache_entry_line) * D1980_REG_NUM);
	/* Set all cachable */
	for (i = 0; i < D1980_REG_NUM; i++)
		d1980_cache[i].cachable = 1;

    size = sizeof(d1980_noncacheable_regs) / sizeof(d1980_noncacheable_regs[0]);

    for(i = 0; i < size; i++)
        d1980_cache[d1980_noncacheable_regs[i]].cachable = 0;


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

	if(id != PMIC_ID || reg > D1980_REG_NUM || (count + reg) > D1980_REG_NUM) {
	    pmic_cache_stat_miss++;
		//printk(KERN_ERR "%s Register(0x%X) out of bounds.\n",
		//                    "# PMIC_CACHE - before_write : ", reg);
	    return -1;
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
			//printk(KERN_INFO "%s The register(0x%X) has been never hitted\n",
			//                    "# PMIC_CACHE - before_read : ", reg);
			return -1;
		}
	}
	for (i = 0; i < count; i++) {
		pdata[i] = pmic_cache[id][reg+i].data;
    }
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

	if(id != PMIC_ID || reg > D1980_REG_NUM || (count + reg) > D1980_REG_NUM) {
	    pmic_cache_stat_miss++;
		//printk(KERN_ERR "%s Register(0x%X) out of bounds.\n",
		//                    "# PMIC_CACHE - before_write : ", reg);
	    return -1;
    }

	for (i = 0; i < count; i++) {
		if (!pmic_cache[id][reg+i].hit ||
			(pmic_cache[id][reg+i].data != pdata[i])) {
			pmic_cache_stat_miss++;
    		//printk(KERN_ERR "%s The register(0x%X) has different value with a cache\n",
        	//	                    "# PMIC_CACHE - before_write : ", reg);
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

	if(id != PMIC_ID || reg > D1980_REG_NUM || (count + reg) > D1980_REG_NUM) {
		//printk(KERN_ERR "%s Register(0x%X) out of bounds.\n",
		//                    "# PMIC_CACHE - after_readwrite : ", reg);
	    return;
    }

	if (i2c_ret >= 0) {
		for (i = 0; i < count; i++) {
			pmic_cache[id][reg+i].data = pdata[i];
			pmic_cache[id][reg+i].hit =
			pmic_cache[id][reg+i].cachable;
		}
	} else {
	    //printk(KERN_INFO, "%s Cache invalidate.\n", "# PMIC_CACHE - after_readwrite : ");
		for (i = 0; i < count; i++)
			pmic_cache[id][reg+i].hit = 0;
	}
}

static void pmic_cache_invalidate(int id, int reg)
{
	if(id != PMIC_ID || reg > D1980_REG_NUM)
	    return;

	pmic_cache[id][reg].hit = 0;
}

#endif /* CACHE_D1980_I2C */

