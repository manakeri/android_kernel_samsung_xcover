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
static u16 smd_PNL_Manufacturer[] = {

	SMD_CMD(0xB0),
	SMD_PARA(0x00),
};

static u16 smd_PNL_SET_POR_Mode[] = {

	SMD_CMD(0x51),
	SMD_PARA(0x00),
};

static u16 smd_PNL_Enter_Invert_Mode[] = {

	SMD_CMD(0x21),
};

static u16 smd_PNL_SET_Address_Mode[] = {

	SMD_CMD(0x36),
	SMD_PARA(0x0A),
	//SMD_PARA(0x09),
};

static u16 smd_PNL_Panel_Driving[] = {

	SMD_CMD(0xC0),
	SMD_PARA(0x14),
	SMD_PARA(0x08),
};

static u16 smd_PNL_Source_Control[] = {

	SMD_CMD(0xC1),
	SMD_PARA(0x01),
	SMD_PARA(0x30),
	SMD_PARA(0x14),
	SMD_PARA(0x02),
	SMD_PARA(0x55),
};

static u16 smd_PNL_Gate_Interface[] = {

	SMD_CMD(0xC4),
	SMD_PARA(0x00),
	SMD_PARA(0x01),
	SMD_PARA(0x00),
};

static u16 smd_PNL_Display_H_Timming[] = {

	SMD_CMD(0xC5),
	SMD_PARA(0x33),
	SMD_PARA(0x7A),
	SMD_PARA(0x05),
	SMD_PARA(0x07),
	SMD_PARA(0x0B),
	SMD_PARA(0x33),
	SMD_PARA(0x00),
	SMD_PARA(0x01),
	SMD_PARA(0x03),
};

static u16 smd_PNL_RGB_Sync_Option[] = {

	SMD_CMD(0xC6),
	SMD_PARA(0x01),
};

static u16 smd_PNL_Logic_Test_Option[] = {

	SMD_CMD(0xF4),
	SMD_PARA(0x00),
	SMD_PARA(0x00),
	SMD_PARA(0x00),
	SMD_PARA(0x03),
	SMD_PARA(0x00),
};

static u16 smd_Power_Bias_Current[] = {

	SMD_CMD(0xD1),
	SMD_PARA(0x33),
	SMD_PARA(0x33),
};

static u16 smd_Power_DDV[] = {

	SMD_CMD(0xD2),
	SMD_PARA(0x22),
	SMD_PARA(0x00),
	SMD_PARA(0x00),
};

static u16 smd_Power_Gamma_Control_Ref[] = {

	SMD_CMD(0xD3),
	SMD_PARA(0x5C),
	SMD_PARA(0x5C),
};

static u16 smd_Power_DCDC[] = {

	SMD_CMD(0xD5),
	SMD_PARA(0x1F),
	SMD_PARA(0x11),
	SMD_PARA(0x19),
	SMD_PARA(0x41),
};

static u16 smd_Power_VCL[] = {

	SMD_CMD(0xD6),
	SMD_PARA(0x11),
	SMD_PARA(0x0A),
};


/* + Gamma Setting Sequence */
static u16 smd_gammaseq_RED[] = {

	SMD_CMD(0xC8),
	SMD_PARA(0x00),
	SMD_PARA(0x05),
	SMD_PARA(0x43),
	SMD_PARA(0x59),
	SMD_PARA(0x7F),

	SMD_PARA(0x79),
	SMD_PARA(0x7F),
	SMD_PARA(0x7F),
	SMD_PARA(0x7E),
	SMD_PARA(0x7F),

	SMD_PARA(0x7F),
	SMD_PARA(0x7B),
	SMD_PARA(0x75),
	SMD_PARA(0x66),
	SMD_PARA(0x5B),

	SMD_PARA(0x54),
	SMD_PARA(0x62),
	SMD_PARA(0x61),
	SMD_PARA(0x56),
	SMD_PARA(0x00),

	SMD_PARA(0x05),
	SMD_PARA(0x25),
	SMD_PARA(0x3B),
	SMD_PARA(0x25),
	SMD_PARA(0x6D),

	SMD_PARA(0x63),
	SMD_PARA(0x63),
	SMD_PARA(0x60),
	SMD_PARA(0x61),
	SMD_PARA(0x5F),

	SMD_PARA(0x5B),
	SMD_PARA(0x55),
	SMD_PARA(0x4C),
	SMD_PARA(0x3D),
	SMD_PARA(0x2C),

	SMD_PARA(0x30),
	SMD_PARA(0x3B),
	SMD_PARA(0x00)

};

static u16 smd_gammaseq_GREEN[] = {

	SMD_CMD(0xC9),
	SMD_PARA(0x00),
	SMD_PARA(0x05),
	SMD_PARA(0x43),
	SMD_PARA(0x59),
	SMD_PARA(0x7F),

	SMD_PARA(0x79),
	SMD_PARA(0x7F),
	SMD_PARA(0x7F),
	SMD_PARA(0x7E),
	SMD_PARA(0x7F),

	SMD_PARA(0x7F),
	SMD_PARA(0x7B),
	SMD_PARA(0x75),
	SMD_PARA(0x66),
	SMD_PARA(0x5B),

	SMD_PARA(0x54),
	SMD_PARA(0x62),
	SMD_PARA(0x61),
	SMD_PARA(0x56),
	SMD_PARA(0x00),

	SMD_PARA(0x05),
	SMD_PARA(0x25),
	SMD_PARA(0x3B),
	SMD_PARA(0x25),
	SMD_PARA(0x6D),

	SMD_PARA(0x63),
	SMD_PARA(0x63),
	SMD_PARA(0x60),
	SMD_PARA(0x61),
	SMD_PARA(0x5F),

	SMD_PARA(0x5B),
	SMD_PARA(0x55),
	SMD_PARA(0x4C),
	SMD_PARA(0x3D),
	SMD_PARA(0x2C),

	SMD_PARA(0x30),
	SMD_PARA(0x3B),
	SMD_PARA(0x00)

};

static u16 smd_gammaseq_BLUE[] = {

	SMD_CMD(0xCA),
	SMD_PARA(0x00),
	SMD_PARA(0x05),
	SMD_PARA(0x43),
	SMD_PARA(0x59),
	SMD_PARA(0x7F),

	SMD_PARA(0x79),
	SMD_PARA(0x7F),
	SMD_PARA(0x7F),
	SMD_PARA(0x7E),
	SMD_PARA(0x7F),

	SMD_PARA(0x7F),
	SMD_PARA(0x7B),
	SMD_PARA(0x75),
	SMD_PARA(0x66),
	SMD_PARA(0x5B),

	SMD_PARA(0x54),
	SMD_PARA(0x62),
	SMD_PARA(0x61),
	SMD_PARA(0x56),
	SMD_PARA(0x00),

	SMD_PARA(0x05),
	SMD_PARA(0x25),
	SMD_PARA(0x3B),
	SMD_PARA(0x25),
	SMD_PARA(0x6D),

	SMD_PARA(0x63),
	SMD_PARA(0x63),
	SMD_PARA(0x60),
	SMD_PARA(0x61),
	SMD_PARA(0x5F),

	SMD_PARA(0x5B),
	SMD_PARA(0x55),
	SMD_PARA(0x4C),
	SMD_PARA(0x3D),
	SMD_PARA(0x2C),

	SMD_PARA(0x30),
	SMD_PARA(0x3B),
	SMD_PARA(0x00)
};

/* - Gamma Setting Sequence */

/* + Sleep Out */
static u16 smd_sleepout[] = {

	SMD_CMD(0x11),
};
/* - Sleep Out */

/* + Initializing Sequence Vcom Voltage Set */
static u16 smd_Vcom_Voltage_Set[] = {

	SMD_CMD(0xD4),
	SMD_PARA(0x37),
	SMD_PARA(0x37),
};
/* - Initializing Sequence Vcom Voltage Set */

/* + CABC & Backlight Control Set */
static u16 smd_CABC_Turn_on1[] = {
	SMD_CMD(0xB4),
	SMD_PARA(0x0F),
	SMD_PARA(0x00),
	SMD_PARA(0x50),
};

static u16 smd_CABC_Turn_on2[] = {
	SMD_CMD(0xB5),
	SMD_PARA(0x80),
};

static u16 smd_CABC_Turn_on3[] = {
	SMD_CMD(0xB7),
	SMD_PARA(0x24),
};

static u16 smd_CABC_Turn_on4[] = {
	SMD_CMD(0xB8),
	SMD_PARA(0x00),
};
/* - CABC & Backlight Control Set */


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

