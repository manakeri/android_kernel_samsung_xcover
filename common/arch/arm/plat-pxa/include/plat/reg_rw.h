/*
 *  linux/arch/arm/plat-pxa/include/plat/reg_rw.h
 *
 *  Support for write-only or read-only register access on PXA platforms
 *
 *  Copyright (C) 2011 Marvell International Ltd.
 *  Raymond Wu <xywu@marvell.com>
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  publishhed by the Free Software Foundation.
 */

#ifndef _PXA_REG_RW_H_
#define _PXA_REG_RW_H_

/**
 * pxa_reg_add - add "addr" into register list
 * @addr: register's kernel virtual address
 * @mask: mask the bit if it's write-only
 *
 * User must call this API before any read/write.
 */
int pxa_reg_add(u32 pa, u32 mask);

/**
 We don't support remove since there isn't a real case.
 */
int pxa_reg_remove(u32 pa);

/**
 * pxa_reg_read - read register value by mask
 * @pa: register's physical address
 * @*val: register value pointer to be read into
 * @mask: Only masked bits are valid.
 *
 * User should not suppose other bits are good, though in fact
 * the R/W and RO bits are correct.
 * User must call pxa_reg_add before this API.
 */
int pxa_reg_read(u32 pa, u32 *val, u32 mask);

/**
 * pxa_reg_write - write register value by mask
 * @pa: register's physical address
 * @val: register value to be written
 * @mask: Only masked bits will be written.
 *
 * User must call pxa_reg_add before this API.
 */
int pxa_reg_write(u32 pa, u32 val, u32 mask);

#define PXA_REG_ERR_ALREADY_ADD		1
#define PXA_REG_ERR_OK				0
#define PXA_REG_ERR_FAIL			(-1)

#endif
