/*
 * linux/sound/soc/pxa/pxa9xx-abu.h
 *
 * Copyright (C) 2007 Marvell International Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */
#ifndef _PXA9XX_ABU_H
#define _PXA9XX_ABU_H


/* ABU registers offset */
/* ABUCR0 - ABU control register 0 */
#define ABUCR0_PLAYBACK		(1 << 24)		/* Bit 24, 0=play/rec, 1 = play only */
#define ABUCR0_CODEC_MASK	(1 << 29)		/* Bit 29, 0=no mask, 1 = codec data mask  */
#define ABUCR0_SOFT_RESET	(1 << 31)		/* Bit 31, 0=disable, 1 = enable */
#define ABUCR0_DATA_PACK_BITS	(22)			/* Bits 22-23, packing. 00 = 16b */
#define ABUCR0_BUFFER_SZ_BITS	(12)			/* Bits 12-19, multiple of 80 packed samples  */
#define ABUCR0_PLAYBACK_WATERMARK_SZ_BITS   (4)		/* Bits 4-11, multiple of 80 packed samples */
#define ABUCR0_PLAYBACK_TIMEOUT_BITS	(25)		/* Bit 25 ~ 28,  */



/* ABUCR1 - ABU control register 1 */
#define ABUCR1_BYPASS_EN			(1 << 7)	/* Bit 7, 0= ABU, 1 = bypass = SSI only */
#define ABUCR1_INT_DRVN_XFR_EN	 		(1 << 6)	/* Bit 6, 1=interrupt, 0 = auto DMA */
#define ABU_AUTO_DMA_MODE			0		/* automatic DMA*/
#define ABU_INTRRUPT_MODE			1		/* interrupt driven*/
#define ABUCR1_TIMEOUT_INTR_EN			(1 << 5)	/* Bit 5, 1= enable, 0 = disable */
#define ABUCR1_ERROR_INTR_EN			(1 << 4)	/* Bit 4, 1= enable, 0 = disable */



/* ABURWR - ABU Record Watermark Register */
#define ABUCR0_RECORD_WATERMARK_SZ_BITS	   (0)   /* Bits 0-7, multiple of 80 packed samples */


/* ABUADSR - ABU Auto DMA Size Register */
#define ABUADSR_PLAYBACK_SZ_BITS	(0)		/* Bits 0-7, multiple of 80 packed samples */
#define ABUADSR_REC_SZ_BITS		(16)		/* Bits 16-23, multiple of 80 packed samples */

/* ABUADSR - ABU Status Register */
#define ABUSR_FIFO_OVERRUN		(1 << 0)	/* Bit 0 */
#define ABUSR_FIFO_UNDERRUN		(1 << 1)	/* Bit 1 */
#define ABUSR_PLAYBACKTIMEOUT		(1 << 3)	/* Bit 3 */
#define ABUSR_DATA_REQ			(1 << 4)	/* Bit 4 */
#define ABUSR_SSI_INT			(1 << 5)	/* Bit 5 */
#define ABUSR_DMA_TX_REQ		(1 << 6)	/* Bit 6 - read only */
#define ABUSR_DMA_RX_REQ		(1 << 7)	/* Bit 7 - read only */


#define ABU_BYTES_PER_SAMPLE_16BITS	(2)
#define ABU_BYTES_PER_SAMPLE_20BITS	(32.0/20.0)
#define ABU_BYTES_PER_SAMPLE_24BITS	(32.0/24.0)
#define ABU_SAMPLE_UNITS		(80)

#ifdef CONFIG_CPU_PXA970
#define ABU_BUFFER_SIZE 16240
#else
#define ABU_BUFFER_SIZE 32480
#endif

#define ABU_WMARk_SIZE_PLAY 640

/* the enum define for ABU logical working status */
typedef enum
{
	ABU_DATA_PACK_16BITS		= 0x0,
	ABU_DATA_PACK_20BITS		= 0x1,
	ABU_DATA_PACK_24BITS		= 0x2,
	ABU_DATA_PACK_NOT_DEFINE	= 0x3,

}ABU_DATA_PACK_BITS_ENUM;


/* inline function */
/* ABU codec data mask in case of error */
static inline void abu_reg_set_codec_mask(void __iomem *abu_reg_base, bool b_mask_flag)
{
	volatile u32 reg_value = __raw_readl(abu_reg_base + ABUCR0);

	if (b_mask_flag) {
		reg_value |= ABUCR0_CODEC_MASK;
	} else {
		reg_value &= ~ABUCR0_CODEC_MASK;
	}

	__raw_writel(reg_value, abu_reg_base + ABUCR0);
}

/* ABU do soft reset */
static inline void abu_reg_soft_reset(void __iomem *abu_reg_base)
{
	volatile u32 reg_value = __raw_readl(abu_reg_base + ABUCR0);
	reg_value |= ABUCR0_SOFT_RESET;
	__raw_writel(reg_value, abu_reg_base + ABUCR0);
	reg_value &= ~ABUCR0_SOFT_RESET;
	__raw_writel(reg_value, abu_reg_base + ABUCR0);
}

/* ABU playback mode setting */
static inline void abu_reg_set_playback_mode(void __iomem *abu_reg_base, bool b_play_flag)
{
	volatile u32 reg_value = __raw_readl(abu_reg_base + ABUCR0);

	if (b_play_flag) {
		reg_value |= ABUCR0_PLAYBACK;	/* playback only */
	} else {
		reg_value &= ~ABUCR0_PLAYBACK;   /* playback & recording */
	}

	__raw_writel(reg_value, abu_reg_base + ABUCR0);
}

/* ABU data pack bits setting */
static inline void abu_reg_set_data_pack_bits(void __iomem *abu_reg_base, ABU_DATA_PACK_BITS_ENUM data_pack_bits)
{
	volatile u32 reg_value = __raw_readl(abu_reg_base + ABUCR0);
	reg_value &= ~(0x3 << ABUCR0_DATA_PACK_BITS);
	reg_value |= data_pack_bits << ABUCR0_DATA_PACK_BITS;
	__raw_writel(reg_value, abu_reg_base + ABUCR0);
}

/* ABU get data pack bits */
static inline ABU_DATA_PACK_BITS_ENUM abu_reg_get_data_pack_bits(void __iomem *abu_reg_base)
{
	volatile u32 reg_value = __raw_readl(abu_reg_base + ABUCR0);
	reg_value = (reg_value & (0x3 << ABUCR0_DATA_PACK_BITS)) >> ABUCR0_DATA_PACK_BITS;
	return (ABU_DATA_PACK_BITS_ENUM)reg_value;
}

/* transfer bytes to the samples which will be used to register setting directly */
static inline u8 abu_reg_bytes_to_80samples(void __iomem *abu_reg_base, u32 bytes)
{
	u32 tmp = 1;
	ABU_DATA_PACK_BITS_ENUM enDataPackBits;

	enDataPackBits = abu_reg_get_data_pack_bits(abu_reg_base);

	switch (enDataPackBits)
	{
		case ABU_DATA_PACK_16BITS:
		{
			tmp = bytes/(ABU_SAMPLE_UNITS*ABU_BYTES_PER_SAMPLE_16BITS);
			break;
		}
		case ABU_DATA_PACK_20BITS:
		{
			/* tmp = bytes/(ABU_SAMPLE_UNITS*ABU_BYTES_PER_SAMPLE_20BITS); */
			break;
		}
		default: /* 24 bits */
		{
			/* tmp = bytes/(ABU_SAMPLE_UNITS*ABU_BYTES_PER_SAMPLE_24BITS); */
			break;
		}
	}

	return (u8)(tmp - 1);
}


/* ABU buffer size setting */
static inline void abu_reg_set_buffer_size(void __iomem *abu_reg_base, u32 buffer_bytes)
{
	u8  buffer_sz = abu_reg_bytes_to_80samples(abu_reg_base, buffer_bytes);
	volatile u32 reg_value = __raw_readl(abu_reg_base + ABUCR0);

	reg_value &= ~(0xFF << ABUCR0_BUFFER_SZ_BITS);
	reg_value |= buffer_sz << ABUCR0_BUFFER_SZ_BITS;
	__raw_writel(reg_value, abu_reg_base + ABUCR0);
}

/* ABU playback watermak setting */
static inline void abu_reg_set_playback_watermark(void __iomem *abu_reg_base, u32 playback_watermark_bytes)
{
	u8  playback_watermark = abu_reg_bytes_to_80samples(abu_reg_base, playback_watermark_bytes);
	volatile u32 reg_value = __raw_readl(abu_reg_base + ABUCR0);

	reg_value &= ~(0xFF << ABUCR0_PLAYBACK_WATERMARK_SZ_BITS);
	reg_value |= playback_watermark << ABUCR0_PLAYBACK_WATERMARK_SZ_BITS;
	__raw_writel(reg_value, abu_reg_base + ABUCR0);
}

/* ABU record watermak setting */
static inline void abu_reg_set_record_watermark(void __iomem *abu_reg_base, u32 record_watermark_bytes)
{
	u8  record_watermark = abu_reg_bytes_to_80samples(abu_reg_base, record_watermark_bytes);
	volatile u32 reg_value = __raw_readl(abu_reg_base + ABURWR);

	reg_value &= ~(0xFF << ABUCR0_RECORD_WATERMARK_SZ_BITS);
	reg_value |= record_watermark << ABUCR0_RECORD_WATERMARK_SZ_BITS;
	__raw_writel(reg_value, abu_reg_base + ABURWR);
}

/* ABU playback size setting */
static inline void abu_reg_set_playback_dma_size(void __iomem *abu_reg_base, u32 playback_bytes)
{
	u8  playback_sz = abu_reg_bytes_to_80samples(abu_reg_base, playback_bytes);
	volatile u32 reg_value = __raw_readl(abu_reg_base + ABUADSR);

	reg_value &= ~(0xFF << ABUADSR_PLAYBACK_SZ_BITS);
	reg_value |= playback_sz << ABUADSR_PLAYBACK_SZ_BITS;
	__raw_writel(reg_value, abu_reg_base + ABUADSR);
}

/* ABU record size setting */
static inline void abu_reg_set_record_dma_size(void __iomem *abu_reg_base, u32 record_bytes)
{
	u8  record_sz = abu_reg_bytes_to_80samples(abu_reg_base, record_bytes);
	volatile u32 reg_value = __raw_readl(abu_reg_base + ABUADSR);

	reg_value &= ~(0xFF << ABUADSR_REC_SZ_BITS);
	reg_value |= record_sz << ABUADSR_REC_SZ_BITS;
	__raw_writel(reg_value, abu_reg_base + ABUADSR);
}


/* ABU interrupt or auto DMA mode setting */
static inline void abu_reg_set_interrupt_dma_mode(void __iomem *abu_reg_base, u8 intr_dma_mode)
{
	volatile u32 reg_value = __raw_readl(abu_reg_base + ABUCR1);

	if (intr_dma_mode) {
		reg_value |= ABUCR1_INT_DRVN_XFR_EN;	 /* interrupt mode */
	} else {
		reg_value &= ~ABUCR1_INT_DRVN_XFR_EN;	/* auto DMA mode */
	}

	__raw_writel(reg_value, abu_reg_base + ABUCR1);
}

/* ABU enable/disable playback timeout error interrupt */
static inline void abu_reg_set_timeout_interrupt(void __iomem *abu_reg_base, bool b_flag)
{
	volatile u32 reg_value = __raw_readl(abu_reg_base + ABUCR1);

	if (b_flag) {
		reg_value |= ABUCR1_TIMEOUT_INTR_EN;	 /* enable */
	} else {
		reg_value &= ~ABUCR1_TIMEOUT_INTR_EN;	/* disable */
	}

	__raw_writel(reg_value, abu_reg_base + ABUCR1);
}

/* ABU enable/disable error interrupt */
static inline void abu_reg_set_error_interrupt(void __iomem *abu_reg_base, bool b_flag)
{
	volatile u32 reg_value = __raw_readl(abu_reg_base + ABUCR1);

	if (b_flag) {
		reg_value |= ABUCR1_ERROR_INTR_EN;	 /* enable */
	} else {
		reg_value &= ~ABUCR1_ERROR_INTR_EN;	/* disable */
	}

	__raw_writel(reg_value, abu_reg_base + ABUCR1);
}


/* ABU playback timeout settting */
static inline void abu_reg_set_playback_timeout(void __iomem *abu_reg_base, u8 time_out)
{
	volatile u32 reg_value = __raw_readl(abu_reg_base + ABUCR0);

	time_out &= 0xF;
	reg_value &= ~(0x0F << ABUCR0_PLAYBACK_TIMEOUT_BITS);
	reg_value |= time_out << ABUCR0_PLAYBACK_TIMEOUT_BITS;

	__raw_writel(reg_value, abu_reg_base + ABUCR0);
}

/* ABU enable/disable SSP */
static inline void abu_reg_enable_disable_ssp(void __iomem *abu_reg_base, bool b_flag)
{
	volatile u32 sscr0 = __raw_readl(abu_reg_base + SSCR0);

	if (b_flag) {

		if (0 == (sscr0 & SSCR0_SSE)) {
			sscr0 |= SSCR0_SSE;
			__raw_writel(sscr0, abu_reg_base + SSCR0);
		}

	} else {

		if (sscr0 & SSCR0_SSE) {
			sscr0 &= ~SSCR0_SSE;
			__raw_writel(sscr0, abu_reg_base + SSCR0);
		}
	}
}

/* ABU init SSI/SSP part */
static inline void abu_reg_init_ssp(void __iomem * p_abu_mmio_base)
{
	/* ABU SSI configure */
	/* SSI control register 0: network mode, on-chip clock is selected, 16-bit data */
	/* SSI control register 1: ABU SSI work as slave mode, both transimit/recevie mode */
	__raw_writel(0xC000003f, p_abu_mmio_base + SSCR0);
	__raw_writel(0x03301cc0, p_abu_mmio_base + SSCR1);
	__raw_writel(0x02000000, p_abu_mmio_base + SSPSP);
	__raw_writel(0x00000003, p_abu_mmio_base + SSTSA);
	__raw_writel(0x00000003, p_abu_mmio_base + SSRSA);
	__raw_writel(0x00000101, p_abu_mmio_base + SSICR);
	return;
}

/* ABU clear error status */
static inline void abu_reg_clear_error_status(void __iomem * p_abu_mmio_base)
{
	volatile u32 reg_value = __raw_readl(p_abu_mmio_base + ABUSR);

	/* check if it need to clear ABU error */
	if ((reg_value & ABUSR_FIFO_OVERRUN) || (reg_value & ABUSR_FIFO_UNDERRUN) || (reg_value & ABUSR_PLAYBACKTIMEOUT)) {

		reg_value &= (~ABUSR_FIFO_OVERRUN) & (~ABUSR_FIFO_UNDERRUN) & (~ABUSR_PLAYBACKTIMEOUT);
		__raw_writel(reg_value, p_abu_mmio_base + ABUSR);
	}

	return;
}

#endif

