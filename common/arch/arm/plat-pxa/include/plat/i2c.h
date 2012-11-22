/*
 *  i2c_pxa.h
 *
 *  Copyright (C) 2002 Intrinsyc Software Inc.
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 *
 */
#ifndef _I2C_PXA_H_
#define _I2C_PXA_H_

#if 0
#define DEF_TIMEOUT             3
#else
/* need a longer timeout if we're dealing with the fact we may well be
 * looking at a multi-master environment
*/
#define DEF_TIMEOUT             32
#endif

#define BUS_ERROR               (-EREMOTEIO)
#define XFER_NAKED              (-ECONNREFUSED)
#define I2C_RETRY               (-2000) /* an error has occurred retry transmit */
#define I2C_MASTER_ABORT        (-3010)

/* ICR initialize bit values
*
*  15. FM       0 (100 Khz operation)
*  14. UR       0 (No unit reset)
*  13. SADIE    0 (Disables the unit from interrupting on slave addresses
*                                       matching its slave address)
*  12. ALDIE    0 (Disables the unit from interrupt when it loses arbitration
*                                       in master mode)
*  11. SSDIE    0 (Disables interrupts from a slave stop detected, in slave mode)
*  10. BEIE     1 (Enable interrupts from detected bus errors, no ACK sent)
*  9.  IRFIE    1 (Enable interrupts from full buffer received)
*  8.  ITEIE    1 (Enables the I2C unit to interrupt when transmit buffer empty)
*  7.  GCD      1 (Disables i2c unit response to general call messages as a slave)
*  6.  IUE      0 (Disable unit until we change settings)
*  5.  SCLE     1 (Enables the i2c clock output for master mode (drives SCL)
*  4.  MA       0 (Only send stop with the ICR stop bit)
*  3.  TB       0 (We are not transmitting a byte initially)
*  2.  ACKNAK   0 (Send an ACK after the unit receives a byte)
*  1.  STOP     0 (Do not send a STOP)
*  0.  START    0 (Do not send a START)
*
*/
#define I2C_ICR_INIT	(ICR_BEIE | ICR_IRFIE | ICR_ITEIE | ICR_GCD | ICR_SCLE)

/* I2C status register init values
 *
 * 10. BED      1 (Clear bus error detected)
 * 9.  SAD      1 (Clear slave address detected)
 * 7.  IRF      1 (Clear IDBR Receive Full)
 * 6.  ITE      1 (Clear IDBR Transmit Empty)
 * 5.  ALD      1 (Clear Arbitration Loss Detected)
 * 4.  SSD      1 (Clear Slave Stop Detected)
 */
#define I2C_ISR_INIT	0x7FF  /* status register init */

#ifndef CONFIG_I2C_PXA95x

#define I2C_PXA_MODE_INT		0		/* default */
#define I2C_PXA_MODE_POLL		(1 << 0)	/* polling status */
#define I2C_PXA_MODE_FIFO_INT		(1 << 1)
#define I2C_PXA_MODE_FIFO_DMA		(1 << 2)

#define I2C_PXA_FREQ_STD		0		/* default, 100Kbps */
#define I2C_PXA_FREQ_FAST		(1 << 0)	/* 400Kbps */
#define I2C_PXA_FREQ_HS_STD		(1 << 1)	/* 3.4Mbps/100Kbps */
#define I2C_PXA_FREQ_HS_FAST		(1 << 2)	/* 3.4Mbps/400Kbps */

struct i2c_pxa_platform_data {
	unsigned int		slave_addr;
	struct i2c_slave_client	*slave;
	unsigned int		class;
	unsigned int		mode;
	unsigned int		freq;
	unsigned int		master_code;
	unsigned int		ilcr;
	unsigned int		iwcr;
	void (*hardware_lock) (void);
	void (*hardware_unlock) (void);
	void (*i2c_bus_reset) (void);
};

#else
#define PXA_I2C_STANDARD_MODE		(1 << 0)
#define PXA_I2C_FAST_MODE		(1 << 1)
/*
 * Up to 3.4Mbps high speed slave operation and up to 1.8Mbps high speed master
 * operation. When high speed mode isn't used, use standard mode instead.
 */
#define PXA_I2C_HIGH_MODE		(1 << 2)

/*
 * i2c transaction transfer mode. PXA_I2C_USING_FIFO_PIO_MODE means (FIFO+PIO);
 * PXA_I2C_USING_FIFO_DMA_MODE means (FIFO+DMA). If don't select these mode, means
 * Non-FIFO mode.
 */
#define PXA_I2C_USING_FIFO_PIO_MODE		(1 << 3)
#define PXA_I2C_USING_FIFO_DMA_MODE		(1 << 4)

struct i2c_pxa_platform_data {
	unsigned int		slave_addr;
	struct i2c_slave_client	*slave;
	unsigned int		class;
	unsigned int		use_pio :1;
	unsigned int		fast_mode :1;
	int			flags;
	u8			master_code;
};

#endif

struct i2c_slave_client;

extern void pxa_set_i2c_info(struct i2c_pxa_platform_data *info);

#ifdef CONFIG_PXA27x
extern void pxa27x_set_i2c_power_info(struct i2c_pxa_platform_data *info);
#endif

#ifdef CONFIG_PXA3xx
extern void pxa3xx_set_i2c_power_info(struct i2c_pxa_platform_data *info);
#endif

#endif
