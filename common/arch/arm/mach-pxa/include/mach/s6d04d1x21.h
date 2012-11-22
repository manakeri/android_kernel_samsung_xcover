#ifndef __ASM_MACH_S6D04D1X21_H
#define __ASM_MACH_S6D04D1X21_H

//#include <mach/tavor_evb3.h>

#define SMD_LCD_COMMAND 0x0 << 8
#define SMD_LCD_PARAMETERS 0x1 << 8
#define SMD_LCD_COMMAND_SLPOUT 0x11
#define SMD_LCD_COMMAND_SLPIN 0x10
#define SMD_LCD_COMMAND_RAM_WRITE 0x2C
#define SMD_LCD_COMMAND_DISON 0x29
#define SMD_LCD_COMMAND_DISOFF 0x28
#define SMD_CMD(x) (SMD_LCD_COMMAND | x)
#define SMD_PARA(x) (SMD_LCD_PARAMETERS | x)


static u16 smd_slpout[] = {
	SMD_CMD(SMD_LCD_COMMAND_SLPOUT),
};

static u16 smd_dison[] = {
	SMD_CMD(SMD_LCD_COMMAND_DISON),
};

static u16 smd_ramwrite[] = {
	SMD_CMD(SMD_LCD_COMMAND_RAM_WRITE),
};

static u16 smd_disoff[] = {
	SMD_CMD(SMD_LCD_COMMAND_DISOFF),
};

static u16 smd_slpin[] = {
	SMD_CMD(SMD_LCD_COMMAND_SLPIN),
};

static u16 smd_passwd1[] = {
	/* DSTB OUT */
	SMD_CMD(0xF0),
	SMD_PARA(0x5A),
	SMD_PARA(0x5A),
};

static u16 smd_passwd2[] = {
	/* DSTB OUT */
	SMD_CMD(0xF1),
	SMD_PARA(0x5A),
	SMD_PARA(0x5A),
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
	SMD_PARA(0x02),
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
	SMD_PARA(0x98),
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
       SMD_PARA(0x00),
	SMD_PARA(0x01),
	SMD_PARA(0x10),
	SMD_PARA(0x12),
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

static u16 brightness01[] = {
	SMD_CMD(0x51),
	SMD_PARA(0x80), //org  
};

static u16 brightness_off[] = {
	SMD_CMD(0x51),
	SMD_PARA(0x01), //org  
};


extern void lcd_init(void);

#endif

