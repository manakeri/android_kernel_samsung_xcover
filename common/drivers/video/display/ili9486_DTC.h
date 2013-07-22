#ifndef __ASM_MACH_ILI9486_DTC_H
#define __ASM_MACH_ILI9486_DTC_H

//#include <mach/tavor_evb3.h>

#define SMD_LCD_COMMAND 0x0 << 8
#define SMD_LCD_PARAMETERS 0x1 << 8

#define SMD_CMD(x) (SMD_LCD_COMMAND | x)
#define SMD_PARA(x) (SMD_LCD_PARAMETERS | x)


/* + SW reset */
static u16 smd_swreset_DTC[] = {

	SMD_CMD(0x01),
};
/* - SW reset */


/* + Power Setting Sequence */
static u16 smd_powersetseq1_DTC[] = {

	SMD_CMD(0xC0),
	SMD_PARA(0x18),
	SMD_PARA(0x19),
};

static u16 smd_powersetseq2_DTC[] = {

	SMD_CMD(0xC1),
	SMD_PARA(0x42),
};

static u16 smd_powersetseq3_DTC[] = {

	SMD_CMD(0xC2),
	SMD_PARA(0x22),
};

static u16 smd_powersetseq4_DTC[] = {

	SMD_CMD(0xC5),
	SMD_PARA(0x00),
	SMD_PARA(0x4E),
};
/* - Power Setting Sequence */


/* + Initializing Sequence */
static u16 smd_initseq1_DTC[] = {

	SMD_CMD(0x2A),
	SMD_PARA(0x00),
	SMD_PARA(0x00),
	SMD_PARA(0x01),
	SMD_PARA(0x3F),
};

static u16 smd_initseq2_DTC[] = {

	SMD_CMD(0x2B),
	SMD_PARA(0x00),
	SMD_PARA(0x00),
	SMD_PARA(0x01),
	SMD_PARA(0xDF),
};

static u16 smd_initseq3_DTC[] = {

	SMD_CMD(0x36),
	SMD_PARA(0x08),
};

static u16 smd_initseq4_DTC[] = {

	SMD_CMD(0x35),
	SMD_PARA(0x00),
	};


static u16 smd_initseq5_DTC[] = {

	SMD_CMD(0x3A),
	SMD_PARA(0x66),
};

static u16 smd_initseq6_DTC[] = {

	SMD_CMD(0xB0),  /* VPL, HPL, EPL, DPL setting */
	SMD_PARA(0x80),
};

static u16 smd_initseq7_DTC[] = {

	SMD_CMD(0xB4),
	SMD_PARA(0x02),
};

static u16 smd_initseq8_DTC[] = {

	SMD_CMD(0xB5),
	SMD_PARA(0x08),
	SMD_PARA(0x0C),
	SMD_PARA(0x10),
	SMD_PARA(0x14),
};

static u16 smd_initseq9_DTC[] = {

	SMD_CMD(0xB6),
	SMD_PARA(0x20),
	SMD_PARA(0x42),
	SMD_PARA(0x3B),
};

static u16 smd_initseq10_DTC[] = {

	SMD_CMD(0xB7),
	SMD_PARA(0x07),
};

static u16 smd_initseq11_DTC[] = {

	SMD_CMD(0xF9),
	SMD_PARA(0x00),
	SMD_PARA(0x08),
};


static u16 smd_normaldispon_DTC[] = {

	SMD_CMD(0x13),  /* Normal Display On */
};
/* - Initializing Sequence */


/* + Gamma Setting Sequence */
static u16 smd_gammaseq1_DTC[] = {

	SMD_CMD(0xE0),
	SMD_PARA(0x0F),SMD_PARA(0x1D),SMD_PARA(0x1A),SMD_PARA(0x0C),SMD_PARA(0x08),
	SMD_PARA(0x0A),SMD_PARA(0x4A),SMD_PARA(0x98),SMD_PARA(0x36),SMD_PARA(0x07),
	SMD_PARA(0x14),SMD_PARA(0x07),SMD_PARA(0x10),SMD_PARA(0x09),SMD_PARA(0x00),
};

static u16 smd_gammaseq2_DTC[] = {

	SMD_CMD(0xE1),
	SMD_PARA(0x0F),SMD_PARA(0x38),SMD_PARA(0x31),SMD_PARA(0x0B),SMD_PARA(0x0A),
	SMD_PARA(0x08),SMD_PARA(0x46),SMD_PARA(0x75),SMD_PARA(0x36),SMD_PARA(0x07),
	SMD_PARA(0x08),SMD_PARA(0x03),SMD_PARA(0x25),SMD_PARA(0x23),SMD_PARA(0x00),
};
		
static u16 smd_dispseq1_DTC[] = {

	SMD_CMD(0xF8),
	SMD_PARA(0x21),
	SMD_PARA(0x07),
	SMD_PARA(0x02),
};

static u16 smd_digitalgammacrtl1_DTC[] = {
	SMD_CMD(0xE2),
	SMD_PARA(0x00),SMD_PARA(0x00),SMD_PARA(0x00),SMD_PARA(0x09),SMD_PARA(0x09),
	SMD_PARA(0x09),SMD_PARA(0x09),SMD_PARA(0x09),SMD_PARA(0x09),SMD_PARA(0x09),
	SMD_PARA(0x09),SMD_PARA(0x09),SMD_PARA(0x09),SMD_PARA(0x09),SMD_PARA(0x09),
	SMD_PARA(0x00),
};

static u16 smd_digitalgammacrtl2_DTC[] = {
	SMD_CMD(0xE3),
	SMD_PARA(0x00),SMD_PARA(0x00),SMD_PARA(0x00),SMD_PARA(0x00),SMD_PARA(0x00),
	SMD_PARA(0x00),SMD_PARA(0x00),SMD_PARA(0x00),SMD_PARA(0x00),SMD_PARA(0x00),
	SMD_PARA(0x00),SMD_PARA(0x00),SMD_PARA(0x24),SMD_PARA(0x24),SMD_PARA(0x24),
	SMD_PARA(0x24),SMD_PARA(0x24),SMD_PARA(0x24),SMD_PARA(0x24),SMD_PARA(0x24),
	SMD_PARA(0x24),SMD_PARA(0x24),SMD_PARA(0x24),SMD_PARA(0x24),SMD_PARA(0x24),
	SMD_PARA(0x24),SMD_PARA(0x24),SMD_PARA(0x24),SMD_PARA(0x24),SMD_PARA(0x24),
	SMD_PARA(0x24),SMD_PARA(0x24),SMD_PARA(0x24),SMD_PARA(0x24),SMD_PARA(0x24),
	SMD_PARA(0x24),SMD_PARA(0x24),SMD_PARA(0x24),SMD_PARA(0x24),SMD_PARA(0x24),
	SMD_PARA(0x24),SMD_PARA(0x24),SMD_PARA(0x24),SMD_PARA(0x24),SMD_PARA(0x24),
	SMD_PARA(0x24),SMD_PARA(0x24),SMD_PARA(0x24),SMD_PARA(0x24),SMD_PARA(0x24),
	SMD_PARA(0x24),SMD_PARA(0x24),SMD_PARA(0x24),SMD_PARA(0x24),SMD_PARA(0x24),
	SMD_PARA(0x24),SMD_PARA(0x24),SMD_PARA(0x24),SMD_PARA(0x24),SMD_PARA(0x24),
	SMD_PARA(0x00),SMD_PARA(0x00),SMD_PARA(0x00),SMD_PARA(0x00),

	
	};
/* - Gamma Setting Sequence */




/* + Display Setting */

static u16 smd_dispseq2_DTC[] = {

	SMD_CMD(0xF1),
	SMD_PARA(0x36),
	SMD_PARA(0x04),
	SMD_PARA(0x00),
	SMD_PARA(0x3C),
       SMD_PARA(0x0F),
	SMD_PARA(0x0F),
	SMD_PARA(0xA4),
	SMD_PARA(0x02),
};

static u16 smd_dispseq3_DTC[] = {

	SMD_CMD(0xF7),
	SMD_PARA(0xA9),
	SMD_PARA(0x91),
	SMD_PARA(0x2D),
	SMD_PARA(0x0A),
	SMD_PARA(0x4F),

};

/*rbs-a new command added */
static u16 smd_dispseq5_DTC[] = {
	SMD_CMD(0xF4),
	SMD_PARA(0x00),
	SMD_PARA(0x00),
	SMD_PARA(0x08),
	SMD_PARA(0x91),
	SMD_PARA(0x04),
};

static u16 smd_dispseq4_DTC[] = {

	SMD_CMD(0xF2),
	SMD_PARA(0x18),SMD_PARA(0xA3),SMD_PARA(0x12),SMD_PARA(0x02),SMD_PARA(0x82),
    SMD_PARA(0x32),SMD_PARA(0xFF),SMD_PARA(0x10),SMD_PARA(0x00),
};



static u16 smd_dispseq6_DTC[] = {
	SMD_CMD(0xFC),
	SMD_PARA(0x00),
	SMD_PARA(0x00),
	SMD_PARA(0x83),
};


/* - Display Setting */


/* + CABC Setting Sequence */
static u16 smd_CABC1_DTC[] = {

	SMD_CMD(0xC8),  
      SMD_PARA(0x01),
};

static u16 smd_CABC2_DTC[] = {

	SMD_CMD(0x55),  
      SMD_PARA(0x00),  /* CABC OFF */
};

static u16 smd_CABC3_DTC[] = {

	SMD_CMD(0x53),  
      SMD_PARA(0x2C),
};

static u16 smd_CABC4_DTC[] = {

	SMD_CMD(0x51),  
      SMD_PARA(0x5D),
};

/* - CABC Setting Sequence */

/* + Sleep Out */
static u16 smd_sleepout_DTC[] = {

	SMD_CMD(0x11),
};
/* - Sleep Out */

/* + Display On */
static u16 smd_displayon_DTC[] = {

	SMD_CMD(0x29),
};
/* - Display On */


/* + Display Off */
static u16 smd_displayoff_DTC[] = {

	SMD_CMD(0x28),
};
/* + Display Off */

/* + Sleep Out */
static u16 smd_sleepin_DTC[] = {

	SMD_CMD(0x10),
};
/* - Sleep Out */


extern void lcd_init(void);

#endif

