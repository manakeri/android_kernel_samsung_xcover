#ifndef __ASM_MACH_ILI9486_H
#define __ASM_MACH_ILI9486_H

//#include <mach/tavor_evb3.h>

#define SMD_LCD_COMMAND 0x0 << 8
#define SMD_LCD_PARAMETERS 0x1 << 8

#define SMD_CMD(x) (SMD_LCD_COMMAND | x)
#define SMD_PARA(x) (SMD_LCD_PARAMETERS | x)


/* + SW reset */
static u16 smd_swreset[] = {

	SMD_CMD(0x01),
};
/* - SW reset */


/* + Power Setting Sequence */
static u16 smd_powersetseq1[] = {

	SMD_CMD(0xC0),
	SMD_PARA(0x15),
	SMD_PARA(0x15),
};

static u16 smd_powersetseq2[] = {

	SMD_CMD(0xC1),
	SMD_PARA(0x42),
};

static u16 smd_powersetseq3[] = {

	SMD_CMD(0xC2),
	SMD_PARA(0x22),
};

static u16 smd_powersetseq4[] = {

	SMD_CMD(0xC5),
	SMD_PARA(0x00),
	SMD_PARA(0x3A),
};
/* - Power Setting Sequence */


/* + Initializing Sequence */
static u16 smd_initseq1[] = {

	SMD_CMD(0x2A),
	SMD_PARA(0x00),
	SMD_PARA(0x00),
	SMD_PARA(0x01),
	SMD_PARA(0x3F),
};

static u16 smd_initseq2[] = {

	SMD_CMD(0x2B),
	SMD_PARA(0x00),
	SMD_PARA(0x00),
	SMD_PARA(0x01),
	SMD_PARA(0xDF),
};

static u16 smd_initseq3[] = {

	SMD_CMD(0x36),
	SMD_PARA(0x08),
};

static u16 smd_initseq4[] = {

	SMD_CMD(0x35),
	SMD_PARA(0x00),
	};


static u16 smd_initseq5[] = {

	SMD_CMD(0x3A),
	SMD_PARA(0x66),
};

static u16 smd_initseq6[] = {

	SMD_CMD(0xB0),  /* VPL, HPL, EPL, DPL setting */
	SMD_PARA(0x80),
};

static u16 smd_initseq7[] = {

	SMD_CMD(0xB4),
	SMD_PARA(0x02),
};

static u16 smd_initseq8[] = {

	SMD_CMD(0xB5),
	SMD_PARA(0x0C),
	SMD_PARA(0x08),
	SMD_PARA(0x2C),
	SMD_PARA(0x26),
};

static u16 smd_initseq9[] = {

	SMD_CMD(0xB6),
	SMD_PARA(0x20),
	SMD_PARA(0x42),
	SMD_PARA(0x3B),
};

static u16 smd_initseq10[] = {

	SMD_CMD(0xB7),
	SMD_PARA(0x07),
};

static u16 smd_initseq11[] = {

	SMD_CMD(0xF9),
	SMD_PARA(0x00),
	SMD_PARA(0x08),
};


static u16 smd_normaldispon[] = {

	SMD_CMD(0x13),  /* Normal Display On */
};
/* - Initializing Sequence */


/* + Gamma Setting Sequence */
static u16 smd_gammaseq1[] = {

	SMD_CMD(0xE0),
	SMD_PARA(0x1E),SMD_PARA(0x21),SMD_PARA(0x1F),SMD_PARA(0x0C),SMD_PARA(0x0B),
	SMD_PARA(0x0C),SMD_PARA(0x4E),SMD_PARA(0xB9),SMD_PARA(0x40),SMD_PARA(0x07),
	SMD_PARA(0x16),SMD_PARA(0x06),SMD_PARA(0x12),SMD_PARA(0x12),SMD_PARA(0x00),
};

static u16 smd_gammaseq2[] = {

	SMD_CMD(0xE1),
	SMD_PARA(0x1F),SMD_PARA(0x2A),SMD_PARA(0x25),SMD_PARA(0x0C),SMD_PARA(0x11),
	SMD_PARA(0x09),SMD_PARA(0x4C),SMD_PARA(0x42),SMD_PARA(0x35),SMD_PARA(0x08),
	SMD_PARA(0x09),SMD_PARA(0x02),SMD_PARA(0x26),SMD_PARA(0x23),SMD_PARA(0x00),
};
		
static u16 smd_dispseq1[] = {

	SMD_CMD(0xF8),
	SMD_PARA(0x21),
	SMD_PARA(0x07),
	SMD_PARA(0x02),
};

static u16 smd_digitalgammacrtl1[] = {
	SMD_CMD(0xE2),
	SMD_PARA(0x09),SMD_PARA(0x09),SMD_PARA(0x09),SMD_PARA(0x09),SMD_PARA(0x09),
	SMD_PARA(0x09),SMD_PARA(0x09),SMD_PARA(0x09),SMD_PARA(0x09),SMD_PARA(0x09),
	SMD_PARA(0x09),SMD_PARA(0x09),SMD_PARA(0x09),SMD_PARA(0x09),SMD_PARA(0x09),
	SMD_PARA(0x09),
};

static u16 smd_digitalgammacrtl2[] = {
	SMD_CMD(0xE3),
	SMD_PARA(0x00),SMD_PARA(0x00),SMD_PARA(0x00),SMD_PARA(0x00),SMD_PARA(0x00),
	SMD_PARA(0x00),SMD_PARA(0x00),SMD_PARA(0x00),SMD_PARA(0x00),SMD_PARA(0x00),
	SMD_PARA(0x00),SMD_PARA(0x00),SMD_PARA(0x00),SMD_PARA(0x00),SMD_PARA(0x00),
	SMD_PARA(0x00),SMD_PARA(0x00),SMD_PARA(0x00),SMD_PARA(0x00),SMD_PARA(0x00),
	SMD_PARA(0x00),SMD_PARA(0x00),SMD_PARA(0x00),SMD_PARA(0x00),SMD_PARA(0x00),
	SMD_PARA(0x00),SMD_PARA(0x00),SMD_PARA(0x00),SMD_PARA(0x00),SMD_PARA(0x00),
	SMD_PARA(0x00),SMD_PARA(0x00),SMD_PARA(0x00),SMD_PARA(0x00),SMD_PARA(0x00),
	SMD_PARA(0x00),SMD_PARA(0x00),SMD_PARA(0x00),SMD_PARA(0x00),SMD_PARA(0x00),
	SMD_PARA(0x00),SMD_PARA(0x00),SMD_PARA(0x00),SMD_PARA(0x00),SMD_PARA(0x00),
	SMD_PARA(0x00),SMD_PARA(0x00),SMD_PARA(0x00),SMD_PARA(0x00),SMD_PARA(0x00),
	SMD_PARA(0x00),SMD_PARA(0x00),SMD_PARA(0x00),SMD_PARA(0x00),SMD_PARA(0x00),
	SMD_PARA(0x00),SMD_PARA(0x00),SMD_PARA(0x00),SMD_PARA(0x00),SMD_PARA(0x00),
	SMD_PARA(0x00),SMD_PARA(0x00),SMD_PARA(0x00),SMD_PARA(0x00),

	
	};
/* - Gamma Setting Sequence */




/* + Display Setting */

static u16 smd_dispseq2[] = {

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

static u16 smd_dispseq3[] = {

	SMD_CMD(0xF7),
	SMD_PARA(0xA9),
	SMD_PARA(0x91),
	SMD_PARA(0x2D),
	SMD_PARA(0x0A),
	SMD_PARA(0x4F),

};

/*rbs-a new command added */
static u16 smd_dispseq5[] = {
	SMD_CMD(0xF4),
	SMD_PARA(0x00),
	SMD_PARA(0x00),
	SMD_PARA(0x08),
	SMD_PARA(0x91),
	SMD_PARA(0x04),
};

static u16 smd_dispseq4[] = {

	SMD_CMD(0xF2),
	SMD_PARA(0x18),SMD_PARA(0xA3),SMD_PARA(0x12),SMD_PARA(0x02),SMD_PARA(0x82),
    SMD_PARA(0x32),SMD_PARA(0xFF),SMD_PARA(0x10),SMD_PARA(0x00),
};



static u16 smd_dispseq6[] = {
	SMD_CMD(0xFC),
	SMD_PARA(0x00),
	SMD_PARA(0x00),
	SMD_PARA(0x83),
};


/* - Display Setting */


/* + CABC Setting Sequence */
static u16 smd_CABC1[] = {

	SMD_CMD(0xC8),  
      SMD_PARA(0x01),
};

static u16 smd_CABC2[] = {

	SMD_CMD(0x55),  
      SMD_PARA(0x00),  /* CABC OFF */
};

static u16 smd_CABC3[] = {

	SMD_CMD(0x53),  
      SMD_PARA(0x2C),
};

static u16 smd_CABC4[] = {

	SMD_CMD(0x51),  
      SMD_PARA(0x5D),
};

/* - CABC Setting Sequence */

/* + Sleep Out */
static u16 smd_sleepout[] = {

	SMD_CMD(0x11),
};
/* - Sleep Out */

/* + Display On */
static u16 smd_displayon[] = {

	SMD_CMD(0x29),
};
/* - Display On */


/* + Display Off */
static u16 smd_displayoff[] = {

	SMD_CMD(0x28),
};
/* + Display Off */

/* + Sleep Out */
static u16 smd_sleepin[] = {

	SMD_CMD(0x10),
};
/* - Sleep Out */


extern void lcd_init(void);

#endif

