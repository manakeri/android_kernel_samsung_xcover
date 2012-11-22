#ifndef __ASM_MACH_LMS365DF01_H
#define __ASM_MACH_LMS365DF01_H

//#include <mach/tavor_evb3.h>

#define SMD_LCD_COMMAND 0x0 << 8
#define SMD_LCD_PARAMETERS 0x1 << 8
#define SMD_LCD_COMMAND_SLPOUT 0x11
#define SMD_LCD_COMMAND_SLPIN 0x10
#define SMD_LCD_COMMAND_RAM_WRITE 0x2C
#define SMD_LCD_COMMAND_DISON 0x29
#define SMD_LCD_COMMAND_DISOFF 0x11
#define SMD_CMD(x) (SMD_LCD_COMMAND | x)
#define SMD_PARA(x) (SMD_LCD_PARAMETERS | x)


static u16 smd_slpout[] = {
	SMD_CMD(SMD_LCD_COMMAND_SLPOUT),
};

/* + Power Setting Sequence */
static u16 smd_passwd1[] = {
	SMD_CMD(0xEF),
	SMD_PARA(0x74),
	SMD_PARA(0x20),
};

static u16 smd_passwd2[] = {
	SMD_CMD(0xF1),
	SMD_PARA(0x02),	
};

static u16 smd_passwd3[] = {
	SMD_CMD(0xF2),
	SMD_PARA(0x00),	
	SMD_PARA(0x00),	
	SMD_PARA(0x00),	
	SMD_PARA(0x00),	
	SMD_PARA(0x0A),	
	SMD_PARA(0x00),	
};

static u16 smd_passwd4[] = {
	SMD_CMD(0xB1),
	SMD_PARA(0x01),	
	SMD_PARA(0x00),	
	SMD_PARA(0x22),	
	SMD_PARA(0x11),	
	SMD_PARA(0x73),	
	SMD_PARA(0x74),	
	SMD_PARA(0xEC),	
	SMD_PARA(0x15),	
	SMD_PARA(0x23),	
};

static u16 smd_passwd5[] = {
	SMD_CMD(0xB6),
	SMD_PARA(0x66),	
	SMD_PARA(0x66),	
	SMD_PARA(0x22),	
	SMD_PARA(0x00),	
	SMD_PARA(0x22),	
	SMD_PARA(0x00),	
	SMD_PARA(0x22),	
	SMD_PARA(0x00),	
};
/* - Power Setting Sequence */


/* + Initializing Sequence */
static u16 smd_initseq1[] = {
	SMD_CMD(0xB2),
	SMD_PARA(0x66),	
	SMD_PARA(0x06),	
	SMD_PARA(0xAA),	
	SMD_PARA(0x88),	
	SMD_PARA(0x88),	
	SMD_PARA(0x08),	
	SMD_PARA(0x08),	
	SMD_PARA(0x03),	
};

static u16 smd_initseq2[] = {
	SMD_CMD(0xB4),
	SMD_PARA(0x10),	
	SMD_PARA(0x00),	
	SMD_PARA(0x32),	
	SMD_PARA(0x32),	
	SMD_PARA(0x32),	
};

static u16 smd_initseq3[] = {
	SMD_CMD(0xD5),
	SMD_PARA(0x02),	
	SMD_PARA(0x43),	
	SMD_PARA(0x01),	
};

static u16 smd_initseq4[] = {
	SMD_CMD(0x36),
	SMD_PARA(0x08),	

};

static u16 smd_initseq5[] = {
	SMD_CMD(0x3A),
	SMD_PARA(0x77),	
};
/* - Initializing Sequence */


/* + Gamma Setting Sequence */
static u16 smd_gammaseq1[] = {
	SMD_CMD(0xE0),
	SMD_PARA(0x0F),	
	SMD_PARA(0x0D),	
	SMD_PARA(0x1D),	
	SMD_PARA(0x10),	
	SMD_PARA(0x20),	
	SMD_PARA(0x39),	
	SMD_PARA(0x28),	
	
      SMD_PARA(0x2E),	
	SMD_PARA(0x07),	
	SMD_PARA(0xCD),	
	SMD_PARA(0x93),	
	SMD_PARA(0xD4),	
	SMD_PARA(0x16),	
	SMD_PARA(0x95),	

      SMD_PARA(0x96),	
	SMD_PARA(0x1B),	
	SMD_PARA(0x16),	
	SMD_PARA(0x00),	
	SMD_PARA(0x09),	
	SMD_PARA(0x08),	
	SMD_PARA(0x0C),	

      SMD_PARA(0x1C),	
	SMD_PARA(0x39),	
	SMD_PARA(0x0E),	
	SMD_PARA(0x24),	
	SMD_PARA(0x06),	
	SMD_PARA(0xCD),	
	SMD_PARA(0x8E),	

      SMD_PARA(0x16),	
	SMD_PARA(0x19),	
	SMD_PARA(0x16),	
	SMD_PARA(0x96),	
	SMD_PARA(0x0D),	
	SMD_PARA(0x17),	
};

static u16 smd_gammaseq2[] = {
	SMD_CMD(0xE1),
	SMD_PARA(0x0F),	
	SMD_PARA(0x0D),	
	SMD_PARA(0x1D),	
	SMD_PARA(0x10),	
	SMD_PARA(0x20),	
	SMD_PARA(0x39),	
	SMD_PARA(0x28),	
	
      SMD_PARA(0x2E),	
	SMD_PARA(0x07),	
	SMD_PARA(0xCD),	
	SMD_PARA(0x93),	
	SMD_PARA(0xD4),	
	SMD_PARA(0x16),	
	SMD_PARA(0x95),	

      SMD_PARA(0x96),	
	SMD_PARA(0x1B),	
	SMD_PARA(0x16),	
	SMD_PARA(0x00),	
	SMD_PARA(0x09),	
	SMD_PARA(0x08),	
	SMD_PARA(0x0C),	

      SMD_PARA(0x1C),	
	SMD_PARA(0x39),	
	SMD_PARA(0x0E),	
	SMD_PARA(0x24),	
	SMD_PARA(0x06),	
	SMD_PARA(0xCD),	
	SMD_PARA(0x8E),	

      SMD_PARA(0x16),	
	SMD_PARA(0x19),	
	SMD_PARA(0x16),	
	SMD_PARA(0x96),	
	SMD_PARA(0x0D),	
	SMD_PARA(0x17),	
};

static u16 smd_gammaseq3[] = {
	SMD_CMD(0xE3),
	SMD_PARA(0x0F),	
	SMD_PARA(0x0D),	
	SMD_PARA(0x1D),	
	SMD_PARA(0x10),	
	SMD_PARA(0x20),	
	SMD_PARA(0x39),	
	SMD_PARA(0x28),	
	
      SMD_PARA(0x2E),	
	SMD_PARA(0x07),	
	SMD_PARA(0xCD),	
	SMD_PARA(0x93),	
	SMD_PARA(0xD4),	
	SMD_PARA(0x16),	
	SMD_PARA(0x95),	

      SMD_PARA(0x96),	
	SMD_PARA(0x1B),	
	SMD_PARA(0x16),	
	SMD_PARA(0x00),	
	SMD_PARA(0x09),	
	SMD_PARA(0x08),	
	SMD_PARA(0x0C),	

      SMD_PARA(0x1C),	
	SMD_PARA(0x39),	
	SMD_PARA(0x0E),	
	SMD_PARA(0x24),	
	SMD_PARA(0x06),	
	SMD_PARA(0xCD),	
	SMD_PARA(0x8E),	

      SMD_PARA(0x16),	
	SMD_PARA(0x19),	
	SMD_PARA(0x16),	
	SMD_PARA(0x96),	
	SMD_PARA(0x0D),	
	SMD_PARA(0x17),	
};

/* - Gamma Setting Sequence */


/* + Display On */
static u16 smd_dison[] = {
	SMD_CMD(SMD_LCD_COMMAND_DISON),
};

/* - Display On */


/* + Backlight On */
static u16 brightness01[] = {
	SMD_CMD(0x51),
	SMD_PARA(0xFF), //org  
};

static u16 brightness02[] = {
	SMD_CMD(0x52),
	SMD_PARA(0xFF), //org  
};
/* - Backlight On */


/* + Display Off */
static u16 smd_disoff[] = {
	SMD_CMD(SMD_LCD_COMMAND_DISOFF),
};
/* - Display Off */


/* + Sleep In */
static u16 smd_slpin[] = {
	SMD_CMD(SMD_LCD_COMMAND_SLPIN),
};
/* - Sleep In */


static u16 smd_ramwrite[] = {
	SMD_CMD(SMD_LCD_COMMAND_RAM_WRITE),
};








static u16 smd_disctl[] = {
	SMD_CMD(0xF2),
	SMD_PARA(0x3B),
	SMD_PARA(0x3A),
	SMD_PARA(0x03),
	SMD_PARA(0x04),
	SMD_PARA(0x02),
	SMD_PARA(0x08),
	SMD_PARA(0x08),
	SMD_PARA(0x00),
	SMD_PARA(0x08),
	SMD_PARA(0x08),
	SMD_PARA(0x00),
	SMD_PARA(0x00),
	SMD_PARA(0x00),
	SMD_PARA(0x00),
	SMD_PARA(0x54),
	SMD_PARA(0x08),
	SMD_PARA(0x08),
	SMD_PARA(0x08),
	SMD_PARA(0x08),
};

static u16 smd_pwrctl[] = {
	SMD_CMD(0xF4),
	SMD_PARA(0x0A),
	SMD_PARA(0x00),
	SMD_PARA(0x00),
	SMD_PARA(0x00),
	SMD_PARA(0x00),
	SMD_PARA(0x00),
	SMD_PARA(0x00),
	SMD_PARA(0x00),
	SMD_PARA(0x15),
	SMD_PARA(0x6B),
	SMD_PARA(0x03),
};

static u16 smd_vcmctll[] = {
	SMD_CMD(0xF5),
	SMD_PARA(0x00),
	SMD_PARA(0x47),
	SMD_PARA(0x75),
	SMD_PARA(0x00),
	SMD_PARA(0x00),
	SMD_PARA(0x04),
	SMD_PARA(0x00),
	SMD_PARA(0x00),
	SMD_PARA(0x04),
};

static u16 smd_pixel_format[] = {
	SMD_CMD(0x3A),
	SMD_PARA(0x77),
};

static u16 smd_tearing_lineon[] = {
	SMD_CMD(0x35),
	SMD_PARA(0x00),
};

static u16 smd_memory_data[] = {
	SMD_CMD(0x36),
	SMD_PARA(0xC0),
};

static u16 smd_column_addset[] = {
	SMD_CMD(0x2A),
	SMD_PARA(0x00),
	SMD_PARA(0x00),
	SMD_PARA(0x01),
	SMD_PARA(0x3F),
};

static u16 smd_page_addset[] = {
	SMD_CMD(0x2B),
	SMD_PARA(0x00),
	SMD_PARA(0x00),
	SMD_PARA(0x01),
	SMD_PARA(0xDF),
};

static u16 smd_srcctl[] = {
	SMD_CMD(0xF6),
	SMD_PARA(0x04),
	SMD_PARA(0x00),
	SMD_PARA(0x08),
	SMD_PARA(0x03),
	SMD_PARA(0x01),
	SMD_PARA(0x00),
};

static u16 smd_ifctl[] = {
	SMD_CMD(0xF7),
	SMD_PARA(0x48),
	SMD_PARA(0x01),
	SMD_PARA(0x10),
	SMD_PARA(0x14),
	SMD_PARA(0x00),
};

static u16 smd_panelctl[] = {
	SMD_CMD(0xF8),
	SMD_PARA(0x11),
	SMD_PARA(0x00),
};

static u16 smd_gammasel1[] = {
	SMD_CMD(0xF9),
	SMD_PARA(0x24),
};

static u16 smd_pgammactl1[] = {
	SMD_CMD(0xFA),
	SMD_PARA(0x23),
	SMD_PARA(0x00),
	SMD_PARA(0x0A),
	SMD_PARA(0x18),
	SMD_PARA(0x1E),
	SMD_PARA(0x22),
	SMD_PARA(0x29),
	SMD_PARA(0x1D),
	SMD_PARA(0x2A),
	SMD_PARA(0x2F),
	SMD_PARA(0x3A),
	SMD_PARA(0x3C),
	SMD_PARA(0x30),
	SMD_PARA(0x00),
	SMD_PARA(0x2A),
	SMD_PARA(0x00),
};

static u16 smd_gammasel2[] = {
	SMD_CMD(0xF9),
	SMD_PARA(0x22),
};

static u16 smd_pgammactl2[] = {
	SMD_CMD(0xFA),
	SMD_PARA(0x30),
	SMD_PARA(0x10),
	SMD_PARA(0x08),
	SMD_PARA(0x1B),
	SMD_PARA(0x1B),
	SMD_PARA(0x1F),
	SMD_PARA(0x25),
	SMD_PARA(0x1A),
	SMD_PARA(0x26),
	SMD_PARA(0x24),
	SMD_PARA(0x25),
	SMD_PARA(0x22),
	SMD_PARA(0x2C),
	SMD_PARA(0x00),
	SMD_PARA(0x2A),
	SMD_PARA(0x00),
};

static u16 smd_gammasel3[] = {
	SMD_CMD(0xF9),
	SMD_PARA(0x21),
};

static u16 smd_pgammactl3[] = {
	SMD_CMD(0xFA),
	SMD_PARA(0x30),
	SMD_PARA(0x10),
	SMD_PARA(0x0A),
	SMD_PARA(0x21),
	SMD_PARA(0x31),
	SMD_PARA(0x33),
	SMD_PARA(0x32),
	SMD_PARA(0x10),
	SMD_PARA(0x1D),
	SMD_PARA(0x20),
	SMD_PARA(0x21),
	SMD_PARA(0x21),
	SMD_PARA(0x20),
	SMD_PARA(0x00),
	SMD_PARA(0x2A),
	SMD_PARA(0x00),
};

static u16 smd_wrctrld_on[] = {
	SMD_CMD(0x53),
	SMD_PARA(0x2C),
};



static u16 brightness_off[] = {
	SMD_CMD(0x51),
	SMD_PARA(0x00), //org  
};


extern void lcd_init(void);

#endif

