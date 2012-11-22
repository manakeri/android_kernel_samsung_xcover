/*
 * d1981.h - D1981 audio codec interface
 *
 * Copyright 2010 Dialog Semiconductor Ltd.
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 */

#ifndef _D1981_REG_H
#define _D1981_REG_H

#define SET_BIT(value,mask)	((value)|(mask))
#define CLR_BIT(value,mask)	((value)&(mask^0xff))

/*
 * Register address.
 */
#define D1981_REG_STATUS_EXT    0x00
#define D1981_REG_STATUS		0x01
#define D1981_REG_REF1			0x02
#define D1981_REG_BIAS_EN		0x03
#define D1981_REG_BIAS1			0x04
#define D1981_REG_BIAS2			0x05
#define D1981_REG_BIAS3			0x06
#define D1981_REG_BIAS4			0x07
#define D1981_REG_MICBIAS2		0x0F
#define D1981_REG_MICBIAS1		0x10
#define D1981_REG_MICDET		0x11
#define D1981_REG_MIC1_PRE		0x12
#define D1981_REG_MIC1			0x13
#define D1981_REG_MIC2_PRE		0x14
#define D1981_REG_MIC2			0x15
#define D1981_REG_AUX1L			0x16
#define D1981_REG_AUX1R			0x17
#define D1981_REG_MIC3_PRE		0x18
#define D1981_REG_MIC3			0x19
#define D1981_REG_INP_PINBIAS		0x1A
#define D1981_REG_INP_ZC_EN		0x1B
#define D1981_REG_INP_MUX		0x1D
#define D1981_REG_HP_DET		0x20
#define D1981_REG_HPL_DAC_OFFSET	0x21
#define D1981_REG_HPL_DAC_OFF_CNTL	0x22
#define D1981_REG_HPL_OUT_OFFSET	0x23
#define D1981_REG_HPL			0x24
#define D1981_REG_HPL_VOL		0x25
#define D1981_REG_HPR_DAC_OFFSET	0x26
#define D1981_REG_HPR_DAC_OFF_CNTL	0x27
#define D1981_REG_HPR_OUT_OFFSET	0x28
#define D1981_REG_HPR			0x29
#define D1981_REG_HPR_VOL		0x2A
#define D1981_REG_LIN2  	0x2B
#define D1981_REG_LIN3  	0x2C
#define D1981_REG_LIN4  	0x2D
#define D1981_REG_OUT_ZC_EN			0x2E
//#define D1981_REG_LIN1L_VOL		0x2F
//#define D1981_REG_LIN1R_DAC_OFFSET	0x30
//#define D1981_REG_LIN1R_DAC_OFF_CNTL	0x31
//#define D1981_REG_LIN1R_OUT_OFFSET	0x32
//#define D1981_REG_LIN1R			0x33
//#define D1981_REG_LIN1R_VOL		0x34
//#define D1981_REG_lIN2			0x35
//#define D1981_REG_OUT_ZC_EN		0x36
#define D1981_REG_HP_LIN1_GNDSEL	0x37
//#define D1981_REG_PDM			0x38
#define D1981_REG_CP_HP1		0x3A
#define D1981_REG_CP_HP2		0x3B
//#define D1981_REG_CP_LIN1_1		0x3C
//#define D1981_REG_CP_LIN1_2		0x3D
#define D1981_REG_CP_CTRL1			0x40
#define D1981_REG_CP_CTRL2			0x41
#define D1981_REG_CP_CTRL3			0x42
#define D1981_REG_CP_LEVEL_MASK		0x43
#define D1981_REG_CP_DET		0x44
#define D1981_REG_CP_STATUS		0x45
#define D1981_REG_CP_THRESH1		0x46
#define D1981_REG_CP_THRESH2		0x47
#define D1981_REG_CP_THRESH3		0x48
#define D1981_REG_CP_THRESH4		0x49
#define D1981_REG_CP_THRESH5		0x4A
#define D1981_REG_CP_THRESH6		0x4B
#define D1981_REG_CP_THRESH7		0x4C
#define D1981_REG_CP_THRESH8		0x4D
#define D1981_REG_PLL_DIV_LO		0x50
#define D1981_REG_PLL_DIV_MID		0x51
#define D1981_REG_PLL_DIV_HI		0x52
#define D1981_REG_PLL_CTRL		0x53
#define D1981_REG_CLK_CTRL		0x54
#define D1981_REG_CLK_DSP		0x5A
#define D1981_REG_CLK_EN1		0x5B
#define D1981_REG_CLK_EN2		0x5C
#define D1981_REG_CLK_EN3		0x5D
#define D1981_REG_CLK_EN4		0x5E
#define D1981_REG_CLK_EN5		0x5F
#define D1981_REG_AIF_MCLK		0x60
#define D1981_REG_AIFA1			0x61
#define D1981_REG_AIFA2			0x62
#define D1981_REG_AIFA3			0x63
#define D1981_REG_AIFB1			0x64
#define D1981_REG_AIFB2			0x65
#define D1981_REG_AIFB3			0x66
#define D1981_REG_PC_CTRL		0x6A
#define D1981_REG_DATA_ROUTE		0x70
#define D1981_REG_DSP_CTRL		0x71
#define D1981_REG_CIF_CTRL2		0x74
#define D1981_REG_HANDSHAKE		0x75
#define D1981_REG_MBOX0			0x76
#define D1981_REG_MBOX1			0x77
#define D1981_REG_MBOX2			0x78
#define D1981_REG_MBOX_STATUS		0x79
#define D1981_REG_SPARE1_OUT		0x7D
#define D1981_REG_SPARE2_OUT		0x7E
#define D1981_REG_SPARE1_IN		0x7F
#define D1981_REG_ID			0x81
#define D1981_REG_ADC1_PD		0x90
#define D1981_REG_ADC1_HPF		0x93
#define D1981_REG_ADC1_SEL		0x94
#define D1981_REG_ADC1_EQ12		0x95
#define D1981_REG_ADC1_EQ34		0x96
#define D1981_REG_ADC1_EQ5		0x97
#define D1981_REG_ADC2_PD		0x98
#define D1981_REG_ADC2_HPF		0x9B
#define D1981_REG_ADC2_SEL		0x9C
#define D1981_REG_ADC2_EQ12		0x9D
#define D1981_REG_ADC2_EQ34		0x9E
#define D1981_REG_ADC2_EQ5		0x9F
#define D1981_REG_DAC1_2_HPF		0xA0
#define D1981_REG_DAC1_2_L_VOL		0xA1
#define D1981_REG_DAC1_2_R_VOL		0xA2
#define D1981_REG_DAC1_2_SEL		0xA3
#define D1981_REG_DAC1_2_SOFTMUTE	0xA4
#define D1981_REG_DAC1_2_EQ12		0xA5
#define D1981_REG_DAC1_2_EQ34		0xA6
#define D1981_REG_DAC1_2_EQ5		0xA7
#define D1981_REG_DAC3_4_HPF		0xB0
#define D1981_REG_DAC3_4_L_VOL		0xB1
#define D1981_REG_DAC3_4_R_VOL		0xB2
#define D1981_REG_DAC3_4_SEL		0xB3
#define D1981_REG_DAC3_4_SOFTMUTE	0xB4
#define D1981_REG_DAC3_4_EQ12		0xB5
#define D1981_REG_DAC3_4_EQ34		0xB6
#define D1981_REG_DAC3_4_EQ5		0xB7
#define D1981_REG_DAC5_HPF			0xC0
#define D1981_REG_DAC5_L_VOL		0xC1
#define D1981_REG_DAC5_SEL			0xC3
#define D1981_REG_DAC5_SOFTMUTE		0xC4
#define D1981_REG_DAC5_EQ12		0xC5
#define D1981_REG_DAC5_EQ34		0xC6
#define D1981_REG_DAC5_EQ5		0xC7
#define D1981_REG_BIQ_BYP		0xD2
#define D1981_REG_DMA_CMD		0xD3
#define D1981_REG_DMA_ADDR0		0xD4
#define D1981_REG_DMA_ADDR1		0xD5
#define D1981_REG_DMA_DATA0		0xD6
#define D1981_REG_DMA_DATA1		0xD7
#define D1981_REG_DMA_DATA2		0xD8
#define D1981_REG_DMA_DATA3		0xD9
#define D1981_REG_DMA_STATUS	0xDA
#define D1981_REG_BROWNOUT		0xDF
#define D1981_REG_UNLOCK		0xE0
#define D1981_REG_AMPCONFIG		0xF9

#define DMA_CMD	 	0xd3
#define DMA_ADDR0   0xd4
#define DMA_ADDR1   0xd5
#define DMA_DATA0   0xd6
#define DMA_DATA1   0xd7
#define DMA_DATA2   0xd8
#define DMA_DATA3   0xd9
#define DMA_STATUS  0xda


/*
 * Register values
 */
// D1981_REG_STATUS		(addr=0x01)
#define D1981_STATUS_PLL_LOCK		(1 << 0)
#define D1981_STATUS_PLL_MCLK_DET	(1 << 1)
#define D1981_STATUS_HPDET_OUT		(1 << 2)
#define D1981_STATUS_INP_MIXDET_1	(1 << 3)
#define D1981_STATUS_INP_MIXDET_2	(1 << 4)
#define D1981_STATUS_BO_STATUS		(1 << 5)

// D1981_REG_REF1		(addr=0x02)
#define D1981_VMID_FASTCHG		(1 << 1)
#define D1981_VMID_FASTDISCHG		(1 << 2)
#define D1981_REFBUFX2_EN		(1 << 6)

// D1981_REG_BIAS_EN		(addr=0x03)
#define D1981_BIAS_BOOST_MASK		(3 << 0)
#define D1981_BIAS_BOOST_100PC		(0 << 0)
#define D1981_BIAS_BOOST_133PC		(1 << 0)
#define D1981_BIAS_BOOST_88PC		(2 << 0)
#define D1981_BIAS_BOOST_50PC		(3 << 0)
#define D1981_BIAS_EN			(1 << 7)

// D1981_REG_BIAS1		(addr=0x04)
#define D1981_BIAS1_HP_DAC_BIAS_MASK	(3 << 0)
#define D1981_BIAS1_HP_DAC_BIAS_100PC	(0 << 0)
#define D1981_BIAS1_HP_DAC_BIAS_150PC	(1 << 0)
#define D1981_BIAS1_HP_DAC_BIAS_50PC	(2 << 0)
#define D1981_BIAS1_HP_DAC_BIAS_75PC	(3 << 0)
#define D1981_BIAS1_HP_OUT_BIAS_MASK	(7 << 4)
#define D1981_BIAS1_HP_OUT_BIAS_100PC	(0 << 4)
#define D1981_BIAS1_HP_OUT_BIAS_125PC	(1 << 4)
#define D1981_BIAS1_HP_OUT_BIAS_150PC	(2 << 4)
#define D1981_BIAS1_HP_OUT_BIAS_175PC	(3 << 4)
#define D1981_BIAS1_HP_OUT_BIAS_200PC	(4 << 4)
#define D1981_BIAS1_HP_OUT_BIAS_250PC	(5 << 4)
#define D1981_BIAS1_HP_OUT_BIAS_300PC	(6 << 4)
#define D1981_BIAS1_HP_OUT_BIAS_350PC	(7 << 4)

// D1981_REG_BIAS2		(addr=0x05)
#define D1981_BIAS2_LINE2_DAC_BIAS_MASK		(3 << 0)
#define D1981_BIAS2_LINE2_DAC_BIAS_100PC	(0 << 0)
#define D1981_BIAS2_LINE2_DAC_BIAS_150PC	(1 << 0)
#define D1981_BIAS2_LINE2_DAC_BIAS_50PC		(2 << 0)
#define D1981_BIAS2_LINE2_DAC_BIAS_75PC		(3 << 0)
#define D1981_BIAS2_LINE2_OUT_BIAS_MASK		(7 << 4)
#define D1981_BIAS2_LINE2_OUT_BIAS_100PC	(0 << 4)
#define D1981_BIAS2_LINE2_OUT_BIAS_125PC	(1 << 4)
#define D1981_BIAS2_LINE2_OUT_BIAS_150PC	(2 << 4)
#define D1981_BIAS2_LINE2_OUT_BIAS_175PC	(3 << 4)
#define D1981_BIAS2_LINE2_OUT_BIAS_200PC	(4 << 4)
#define D1981_BIAS2_LINE2_OUT_BIAS_250PC	(5 << 4)
#define D1981_BIAS2_LINE2_OUT_BIAS_300PC	(6 << 4)
#define D1981_BIAS2_LINE2_OUT_BIAS_350PC	(7 << 4)

// D1981_REG_BIAS3		(addr=0x06)
#define D1981_BIAS3_LINE3_DAC_BIAS_MASK		(3 << 0)
#define D1981_BIAS3_LINE3_DAC_BIAS_100PC	(0 << 0)
#define D1981_BIAS3_LINE3_DAC_BIAS_150PC	(1 << 0)
#define D1981_BIAS3_LINE3_DAC_BIAS_50PC		(2 << 0)
#define D1981_BIAS3_LINE3_DAC_BIAS_75PC		(3 << 0)
#define D1981_BIAS3_LINE3_OUT_BIAS_MASK		(7 << 4)
#define D1981_BIAS3_LINE3_OUT_BIAS_100PC	(0 << 4)
#define D1981_BIAS3_LINE3_OUT_BIAS_125PC	(1 << 4)
#define D1981_BIAS3_LINE3_OUT_BIAS_150PC	(2 << 4)
#define D1981_BIAS3_LINE3_OUT_BIAS_175PC	(3 << 4)
#define D1981_BIAS3_LINE3_OUT_BIAS_200PC	(4 << 4)
#define D1981_BIAS3_LINE3_OUT_BIAS_250PC	(5 << 4)
#define D1981_BIAS3_LINE3_OUT_BIAS_300PC	(6 << 4)
#define D1981_BIAS3_LINE3_OUT_BIAS_350PC	(7 << 4)

// D1981_REG_BIAS4		(addr=0x07)
#define D1981_BIAS4_LINE4_DAC_BIAS_MASK		(3 << 0)
#define D1981_BIAS4_LINE4_DAC_BIAS_100PC	(0 << 0)
#define D1981_BIAS4_LINE4_DAC_BIAS_150PC	(1 << 0)
#define D1981_BIAS4_LINE4_DAC_BIAS_50PC		(2 << 0)
#define D1981_BIAS4_LINE4_DAC_BIAS_75PC		(3 << 0)
#define D1981_BIAS4_LINE4_OUT_BIAS_MASK		(7 << 4)
#define D1981_BIAS4_LINE4_OUT_BIAS_100PC	(0 << 4)
#define D1981_BIAS4_LINE4_OUT_BIAS_125PC	(1 << 4)
#define D1981_BIAS4_LINE4_OUT_BIAS_150PC	(2 << 4)
#define D1981_BIAS4_LINE4_OUT_BIAS_175PC	(3 << 4)
#define D1981_BIAS4_LINE4_OUT_BIAS_200PC	(4 << 4)
#define D1981_BIAS4_LINE4_OUT_BIAS_250PC	(5 << 4)
#define D1981_BIAS4_LINE4_OUT_BIAS_300PC	(6 << 4)
#define D1981_BIAS4_LINE4_OUT_BIAS_350PC	(7 << 4)


// D1981_REG_SIF_VDD_SEL	(addr=0x08)
#define D1981_SIF_VDD_SEL_AIFA_VDD2		(1 << 0)
#define D1981_SIF_VDD_SEL_AIFB_VDD2		(1 << 1)
#define D1981_SIF_VDD_SEL_CIFA_VDD2		(1 << 4)

// D1981_REG_MICBIAS2		(addr=0x0F)
#define D1981_MICBIAS2_VOLTAGE_MASK		(0x0F << 0)
#define D1981_MICBIAS2_VOLTAGE_2V		(0x00 << 0)
#define D1981_MICBIAS2_VOLTAGE_2V05		(0x01 << 0)
#define D1981_MICBIAS2_VOLTAGE_2V1		(0x02 << 0)
#define D1981_MICBIAS2_VOLTAGE_2V15		(0x03 << 0)
#define D1981_MICBIAS2_VOLTAGE_2V2		(0x04 << 0)
#define D1981_MICBIAS2_VOLTAGE_2V25		(0x05 << 0)
#define D1981_MICBIAS2_VOLTAGE_2V3		(0x06 << 0)
#define D1981_MICBIAS2_VOLTAGE_2V35		(0x07 << 0)
#define D1981_MICBIAS2_VOLTAGE_2V4		(0x08 << 0)
#define D1981_MICBIAS2_VOLTAGE_2V45		(0x09 << 0)
#define D1981_MICBIAS2_VOLTAGE_2V5		(0x0A << 0)
#define D1981_MICBIAS2_EN			(1 << 7)

// D1981_REG_MICBIAS1		(addr=0x10)
#define D1981_MICBIAS1_VOLTAGE_MASK		(0x0F << 0)
#define D1981_MICBIAS1_VOLTAGE_2V		(0x00 << 0)
#define D1981_MICBIAS1_VOLTAGE_2V05		(0x01 << 0)
#define D1981_MICBIAS1_VOLTAGE_2V1		(0x02 << 0)
#define D1981_MICBIAS1_VOLTAGE_2V15		(0x03 << 0)
#define D1981_MICBIAS1_VOLTAGE_2V2		(0x04 << 0)
#define D1981_MICBIAS1_VOLTAGE_2V25		(0x05 << 0)
#define D1981_MICBIAS1_VOLTAGE_2V3		(0x06 << 0)
#define D1981_MICBIAS1_VOLTAGE_2V35		(0x07 << 0)
#define D1981_MICBIAS1_VOLTAGE_2V4		(0x08 << 0)
#define D1981_MICBIAS1_VOLTAGE_2V45		(0x09 << 0)
#define D1981_MICBIAS1_VOLTAGE_2V5		(0x0A << 0)
#define D1981_MICBIAS1_EN			(1 << 7)

// D1981_REG_MICDET		(addr=0x11)
#define D1981_MICDET_INP_MICRES			(1 << 0)
#define D1981_MICDET_INP_MICHOOK		(1 << 1)
#define D1981_MICDET_INP_DEBOUNCE_PRD_8MS		(0 << 0)
#define D1981_MICDET_INP_DEBOUNCE_PRD_16MS		(1 << 0)
#define D1981_MICDET_INP_DEBOUNCE_PRD_32MS		(2 << 0)
#define D1981_MICDET_INP_DEBOUNCE_PRD_64MS		(3 << 0)
#define D1981_MICDET_INP_MICDET_EN		(1 << 7)

// D1981_REG_MIC1_PRE		(addr=0x12)
#define D1981_MICL_PRE_VOL_VAL_SHIFT		0
#define D1981_MICL_PRE_VOL_VAL_MASK		(0x7 << D1981_MICL_PRE_VOL_VAL_SHIFT)
#define D1981_MICL_PRE_VOL_VAL_MIN		0x1	/* register value for minimum volume*/
#define D1981_MICL_PRE_VOL_MDB_MIN		0	/*minimum volume in mdB*/
#define D1981_MICL_PRE_VOL_MDB_INC		6000	/*volume increments in mdB*/
#define D1981_MICL_PRE_VOL_VAL_STEPS		6
#define D1981_MICL_INP_MIC1_MUTE		(1 << 6)
#define D1981_MICL_INP_MIC1_EN			(1 << 7)

// D1981_REG_MICL		(addr=0x13)
#define D1981_MICL_INP_MIC1_PGA_VOL_VAL_SHIFT	0
#define D1981_MICL_INP_MIC1_PGA_VOL_VAL_MASK	(0x1F << D1981_MICL_INP_MIC1_PGA_VOL_VAL_SHIFT)
#define D1981_MICL_INP_MIC1_PGA_VOL_VAL_MIN	0x7	/* register value for minimum volume*/
#define D1981_MICL_INP_MIC1_PGA_VOL_MDB_MIN	0	/*minimum volume in mdB*/
#define D1981_MICL_INP_MIC1_PGA_VOL_MDB_INC	500	/*volume increments in mdB*/
#define D1981_MICL_INP_MIC1_PGA_VOL_VAL_STEPS	24
#define D1981_MICL_INP_MIC1_MUTE		(1 << 6)
#define D1981_MICL_INP_MIC1_EN			(1 << 7)

// D1981_REG_MIC2_PRE		(addr=0x14)
#define D1981_MICR_PRE_VOL_VAL_SHIFT		0
#define D1981_MICR_PRE_VOL_VAL_MASK		(0x7 << D1981_MICR_PRE_VOL_VAL_SHIFT)
#define D1981_MICR_PRE_VOL_VAL_MIN		0x1	/* register value for minimum volume*/
#define D1981_MICR_PRE_VOL_MDB_MIN		0	/*minimum volume in mdB*/
#define D1981_MICR_PRE_VOL_MDB_INC		6000	/*volume increments in mdB*/
#define D1981_MICR_PRE_VOL_VAL_STEPS		6

// D1981_REG_MIC2		(addr=0x15)
#define D1981_MICR_INP_MIC1_PGA_VOL_VAL_SHIFT	0
#define D1981_MICR_INP_MIC1_PGA_VOL_VAL_MASK	(0x1F << D1981_MICR_INP_MIC1_PGA_VOL_VAL_SHIFT)
#define D1981_MICR_INP_MIC1_PGA_VOL_VAL_MIN	0x7	/* register value for minimum volume*/
#define D1981_MICR_INP_MIC1_PGA_VOL_MDB_MIN	0	/*minimum volume in mdB*/
#define D1981_MICR_INP_MIC1_PGA_VOL_MDB_INC	500	/*volume increments in mdB*/
#define D1981_MICR_INP_MIC1_PGA_VOL_VAL_STEPS	24
#define D1981_MICR_INP_MIC1_MUTE		(1 << 6)
#define D1981_MICR_INP_MIC1_EN			(1 << 7)

// D1981_REG_AUX1L		(addr=0x16)
#define D1981_AUX1L_INP_VOL_VAL_SHIFT		0
#define D1981_AUX1L_INP_VOL_VAL_MASK		(0x3F << D1981_AUX1L_INP_VOL_VAL_SHIFT)
#define D1981_AUX1L_INP_VOL_VAL_MIN		0xD	/* register value for minimum volume*/
#define D1981_AUX1L_INP_VOL_MDB_MIN		-60000	/*minimum volume in mdB*/
#define D1981_AUX1L_INP_VOL_MDB_INC		1500	/*volume increments in mdB*/
#define D1981_AUX1L_INP_VOL_VAL_STEPS		50
#define D1981_AUX1L_INP_MUTE			(1 << 6)
#define D1981_AUX1L_INP_EN			(1 << 7)

// D1981_REG_AUX1R		(addr=0x17)
#define D1981_AUX1R_INP_VOL_VAL_SHIFT		0
#define D1981_AUX1R_INP_VOL_VAL_MASK		(0x3F << D1981_AUX1R_INP_VOL_VAL_SHIFT)
#define D1981_AUX1R_INP_VOL_VAL_MIN		0xD	/* register value for minimum volume*/
#define D1981_AUX1R_INP_VOL_MDB_MIN		-60000	/*minimum volume in mdB*/
#define D1981_AUX1R_INP_VOL_MDB_INC		1500	/*volume increments in mdB*/
#define D1981_AUX1R_INP_VOL_VAL_STEPS		50
#define D1981_AUX1R_INP_MUTE			(1 << 6)
#define D1981_AUX1R_INP_EN			(1 << 7)

// D1981_REG_MIC3_PRE		(addr=0x18)
#define D1981_AUX2_PRE_VOL_VAL_SHIFT		0
#define D1981_AUX2_PRE_VOL_VAL_MASK		(0x7 << D1981_AUX2_PRE_VOL_VAL_SHIFT)
#define D1981_AUX2_PRE_VOL_VAL_MIN		0x1	/* register value for minimum volume*/
#define D1981_AUX2_PRE_VOL_MDB_MIN		0	/*minimum volume in mdB*/
#define D1981_AUX2_PRE_VOL_MDB_INC		6000	/*volume increments in mdB*/
#define D1981_AUX2_PRE_VOL_VAL_STEPS		6

// D1981_REG_MIC3		(addr=0x19)
#define D1981_AUX2_INP_VOL_VAL_SHIFT		0
#define D1981_AUX2_INP_VOL_VAL_MASK		(0x1F << D1981_AUX2_INP_VOL_VAL_SHIFT)
#define D1981_AUX2_INP_VOL_VAL_MIN		0x7	/* register value for minimum volume*/
#define D1981_AUX2_INP_VOL_MDB_MIN		0	/*minimum volume in mdB*/
#define D1981_AUX2_INP_VOL_MDB_INC		500	/*volume increments in mdB*/
#define D1981_AUX2_INP_VOL_VAL_STEPS		24
#define D1981_AUX2_INP_MUTE			(1 << 6)
#define D1981_AUX2_INP_EN			(1 << 7)

// D1981_REG_INP_PINBIAS	(addr=0x1A)
#define D1981_INP_MICL_PINBIAS_EN		(1 << 0)
#define D1981_INP_MICR_PINBIAS_EN		(1 << 1)
#define D1981_INP_AUX1L_PINBIAS_EN		(1 << 2)
#define D1981_INP_AUX1R_PINBIAS_EN		(1 << 3)
#define D1981_INP_AUX2_PINBIAS_EN		(1 << 4)

// D1981_REG_INP_ZC_EN		(addr=0x1B)
#define D1981_INP_MICL_PRE_ZC_EN		(1 << 0)
#define D1981_INP_MICL_PGA_ZC_EN		(1 << 1)
#define D1981_INP_MICR_PRE_ZC_EN		(1 << 2)
#define D1981_INP_MICR_PGA_ZC_EN		(1 << 3)
#define D1981_INP_AUX1L_ZC_EN			(1 << 4)
#define D1981_INP_AUX1R_ZC_EN			(1 << 5)
#define D1981_INP_AUX2_PRE_ZC_EN		(1 << 6)
#define D1981_INP_AUX2_PGA_ZC_EN		(1 << 7)

// D1981_REG_INP_MUX		(addr=0x1D)
#define D1981_INP_ADC1L_MUX_SEL_AUX1L		(0 << 0)
#define D1981_INP_ADC1L_MUX_SEL_MIC1		(1 << 0)
#define D1981_INP_ADC1R_MUX_SEL_MASK		(3 << 2)
#define D1981_INP_ADC1R_MUX_SEL_AUX1R		(0 << 2)
#define D1981_INP_ADC1R_MUX_SEL_MIC2		(1 << 2)
#define D1981_INP_ADC1R_MUX_SEL_MIC3		(2 << 2)
#define D1981_INP_ADC2L_MUX_SEL_AUX1L		(0 << 4)
#define D1981_INP_ADC2L_MUX_SEL_MICL		(1 << 4)
#define D1981_INP_ADC2R_MUX_SEL_MASK		(3 << 6)
#define D1981_INP_ADC2R_MUX_SEL_AUX1R		(0 << 6)
#define D1981_INP_ADC2R_MUX_SEL_MICR		(1 << 6)
#define D1981_INP_ADC2R_MUX_SEL_AUX2		(2 << 6)

// D1981_REG_HP_DET		(addr=0x20)
#define D1981_HP_DET_AZ				(1 << 0)
#define D1981_HP_DET_SEL1			(1 << 1)
#define D1981_HP_DET_IS_MASK		(3 << 2)
#define D1981_HP_DET_IS_0_5UA		(0 << 2)
#define D1981_HP_DET_IS_1UA			(1 << 2)
#define D1981_HP_DET_IS_2UA			(2 << 2)
#define D1981_HP_DET_IS_4UA			(3 << 2)
#define D1981_HP_DET_RS_MASK		(3 << 4)
#define D1981_HP_DET_RS_INFINITE	(0 << 4)
#define D1981_HP_DET_RS_100KOHM		(1 << 4)
#define D1981_HP_DET_RS_10KOHM		(2 << 4)
#define D1981_HP_DET_RS_1KOHM		(3 << 4)
#define D1981_HP_DET_EN				(1 << 7)

// D1981_REG_HPL_DAC_OFFSET	(addr=0x21)
#define D1981_HPL_DAC_OFFSET_TRIM_MASK		(0x3F << 0)
#define D1981_HPL_DAC_OFFSET_DAC_SIGN		(1 << 6)

// D1981_REG_HPL_DAC_OFF_CNTL	(addr=0x22)
#define D1981_HPL_DAC_OFF_CNTL_CONT_MASK	(7 << 0)
#define D1981_HPL_DAC_OFF_CNTL_COMPO		(1 << 3)

// D1981_REG_HPL_OUT_OFFSET	(addr=0x23)
#define D1981_HPL_OUT_OFFSET_MASK		(0xFF << 0)

// D1981_REG_HPL		(addr=0x24)
#define D1981_HPL_OUT_SIGN			(1 << 0)
#define D1981_HPL_OUT_COMP			(1 << 1)
#define D1981_HPL_OUT_RESERVED		(1 << 2)
#define D1981_HPL_OUT_COMPO			(1 << 3)
#define D1981_HPL_OUT_DAC_EN		(1 << 4)
#define D1981_HPL_OUT_HIZ_EN		(1 << 5)
#define D1981_HPL_OUT_MUTE			(1 << 6)
#define D1981_HPL_OUT_EN			(1 << 7)

// D1981_REG_HPL_VOL		(addr=0x25)
#define D1981_HPL_VOLSHIFT			0                                             				(1 << )
#define D1981_HPL_VOLMASK			(0x7 << D1981_AUX2_INP_VOL_VAL_SHIFT)        				(1 << )
//#define D1981_HPL_VOLMIN			0	/* register value for minimum volume*/				(1 << )
#define D1981_HPL_VOLMIN			-22500	/*minimum volume in mdB*/             				(1 << )
#define D1981_HPL_VOLINC			1500	/*volume increments in mdB*/          				(1 << )
#define D1981_HPL_VOLSTEPS			15                                            				(1 << )

// D1981_REG_HPR_DAC_OFFSET	(addr=0x26)
#define D1981_HPR_DAC_OFFSET_TRIM_MASK		(0x3F << 0)
#define D1981_HPR_DAC_OFFSET_DAC_SIGN		(1 << 6)

// D1981_REG_DAC_OFF_CNTL	(addr=0x27)
#define D1981_HPR_DAC_OFF_CNTL_CONT_MASK	(7 << 0)
#define D1981_HPR_DAC_OFF_CNTL_COMPO		(1 << 3)

// D1981_REG_HPR_OUT_OFFSET	(addr=0x28)
#define D1981_HPR_OUT_OFFSET_MASK		(0xFF << 0)

// D1981_REG_HPR		(addr=0x29)
#define D1981_HPR_OUT_SIGN			(1 << 0)
#define D1981_HPR_OUT_COMP			(1 << 1)
#define D1981_HPR_OUT_RESERVED		(1 << 2)
#define D1981_HPR_OUT_COMPO			(1 << 3)
#define D1981_HPR_OUT_DAC_EN		(1 << 4)
#define D1981_HPR_OUT_HIZ_EN		(1 << 5)
#define D1981_HPR_OUT_MUTE			(1 << 6)
#define D1981_HPR_OUT_EN			(1 << 7)

// D1981_REG_HPR_VOL		(addr=0x2A)
#define D1981_HPR_VOLSHIFT			0                                             				(1 << )
#define D1981_HPR_VOLMASK			(0x7 << D1981_AUX2_INP_VOL_VAL_SHIFT)        				(1 << )
//#define D1981_HPR_VOLMIN			0	/* register value for minimum volume*/				(1 << )
#define D1981_HPR_VOLMIN			-22500	/*minimum volume in mdB*/             				(1 << )
#define D1981_HPR_VOLINC			1500	/*volume increments in mdB*/          				(1 << )
#define D1981_HPR_VOLSTEPS			15                                            				(1 << )

// D1981_REG_LIN2	(addr=0x2B)
// D1981_REG_LIN3	(addr=0x2C)
// D1981_REG_LIN4	(addr=0x2D)
#define D1981_LIN_OUT_VOL_MASK	(0x0F << 0)
#define D1981_LIN_DAC_OFF		(0 << 4)
#define D1981_LIN_DAC_EN		(1 << 4)
#define D1981_LIN_OUT_HIZ_N_DIS	(0 << 5)
#define D1981_LIN_OUT_HIZ_N_EN		(1 << 5)
#define D1981_LIN_OUT_MUTE_NUMUTED		(0 << 6)
#define D1981_LIN_OUT_MUTE_MUTED		(1 << 6)
#define D1981_LIN_OUT_EN_OFF		(0 << 7)
#define D1981_LIN_OUT_EN_ON		(1 << 7)

// D1981_REG_OUT_ZC_EN		(addr=0x2E)
#define D1981_HPL_ZC_EN_1			(1 << 0)
#define D1981_HPL_ZC_EN_0			(0 << 0)
#define D1981_HPR_ZC_EN_1			(1 << 1)
#define D1981_HPR_ZC_EN_0			(0 << 1)
#define D1981_LIN2_ZC_EN_1		(1 << 2)
#define D1981_LIN2_ZC_EN_0		(0 << 2)
#define D1981_LIN3_ZC_EN_1		(1 << 3)
#define D1981_LIN3_ZC_EN_0		(0 << 3)
#define D1981_LIN4_ZC_EN_1		(1 << 4)
#define D1981_LIN4_ZC_EN_0		(0 << 4)

// D1981_REG_AIFA3		(addr=0x63)
// D1981_REG_AIFB3		(addr=0x66)
#define D1981_AIF_EN				(1<<7)

// Generic bit positions
#define D1981_DAC_ENABLE_BIT	 (1<<4)
#define D1981_ADC_ENABLE_BIT	 (1<<4)
#define D1981_PGA_ENABLE_BIT	 (1<<7)
#define D1981_MUTE_BIT	(1<<6)

/* mode */
#define PMEM	0x01
#define XMEM	0x02
#define YMEM	0x03
#define DMA_RD  0x10
#define DMA_BUSY	0x01

#define BYTE0(x) ((x)&0xff)
#define BYTE1(x) (((x)>>8)&0xff)
#define BYTE2(x) (((x)>>16)&0xff)
#define BYTE3(x) (((x)>>24)&0xff)

#endif
