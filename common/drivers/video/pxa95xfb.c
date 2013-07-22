/*
 *  linux/drivers/video/pxa95xfb.c
 */
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/errno.h>
#include <linux/string.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/fb.h>
#include <linux/delay.h>
#include <linux/init.h>
#include <linux/ioport.h>
#include <linux/cpufreq.h>
#include <linux/platform_device.h>
#include <linux/dma-mapping.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/uaccess.h>
#include <linux/kthread.h>
#include <linux/earlysuspend.h>
#include <mach/io.h>
#include <mach/irqs.h>
#include <mach/regs-ost.h>
#include <mach/pxa95x-regs.h>
#include <mach/pxa95xfb.h>
#include <mach/dvfm.h>

#include "pxa95xfb.h"

extern void enable_oscc_tout_s0(void);
extern void disable_oscc_tout_s0(void);
extern void set_mipi_reference_control(void);
extern void clear_mipi_reference_control(void);
#if defined(CONFIG_MACH_GFORCE) || defined(CONFIG_MACH_ALKON) || defined(CONFIG_MACH_JETTA)
extern unsigned char lcdPanelOnOff_status; //pps-a
#endif
static void controller_enable_disable(struct pxa95xfb_info *fbi, int onoff);

struct pxa95xfb_info * pxa95xfbi[PXA95xFB_FB_NUM];
struct pxa95xfb_conv_info pxa95xfb_conv[4] = {
	[0] = {
		.name		= "LCD_PARALLEL",
		.converter	= LCD_M2PARALELL_CONVERTER,
		.on			= 0,
		.inited		= 0,
		.ref_count	= 0,
	},
	[1] = {
		.name		= "LCD_DSI0",
		.converter	= LCD_M2DSI0,
		.on			= 0,
		.inited		= 0,
		.ref_count	= 0,
	},
	[2] = {
		.name		= "LCD_DSI1",
		.converter	= LCD_M2DSI1,
		.on 		= 0,
		.inited 	= 0,
		.ref_count	= 0,
	},
	[3] = {
		.name		= "LCD_HDMI",
		.converter	= LCD_M2HDMI,
		.on		= 0,
		.inited		= 0,
		.ref_count	= 0,
	},
};

int display_enabled = 0;
static int display_update_ongoing[3] = {0, 0, 0};

#define DSI_CMDLINE_TIMEOUT 0
#define DSI_CMDLINE_CMD 1
#define DSI_CMDLINE_PARM 2

/* cmd line: 0: timeout; 1 type: is loop, | is cmd or data; 2: cmd(or data); 3- parms*/
static u8 init_board [][BOARD_INIT_MAX_DATA_BYTES] = {
	{ 0xFF,LCD_Controller_DCS_SHORT_WRITE_NO_PARAMETER,LCD_Controller_DCS_SOFT_RESET}, /*Soft reset*/
	{ 10,LCD_Controller_DCS_SHORT_WRITE_NO_PARAMETER,LCD_Controller_DCS_NOP}, /*Nop*/
	/*wait 10ms*/
	{ 0xFF,LCD_Controller_DCS_SHORT_WRITE_NO_PARAMETER,LCD_Controller_DCS_EXIT_SLEEP_MODE}, /* Exit sleep*/
	{ 20,LCD_Controller_DCS_SHORT_WRITE_NO_PARAMETER,LCD_Controller_DCS_NOP}, /*Nop*/
	/*wait 20ms*/
	{ 1,LCD_Controller_DCS_SHORT_WRITE_WITH_PARAMETER,LCD_Controller_DCS_SET_OUTPUT_PIXEL_CLOCK_FREQUENCY,0x52}, /* SET_OUTPUT_CLOCK_FREQUENCY*/
	{ 1,LCD_Controller_DCS_LONG_WRITE,0x05,0x00,LCD_Controller_DCS_ENABLE_SET_SPECIAL_COMMAND,0x03,0x7F,0x5C,0x33}, /* Enable_Set_Special_command*/
	{ 1,LCD_Controller_DCS_SHORT_WRITE_WITH_PARAMETER,LCD_Controller_DCS_DISPLAY_BUFFER_IO_CONTROL,0x05}, /*2 THSSI channels and RGB16*/
	/* { 1,LCD_Controller_DCS_SHORT_WRITE_WITH_PARAMETER,LCD_Controller_DCS_SET_TEAR_ON,0x00}, //Set TEAR ON */
	{ 1,LCD_Controller_DCS_LONG_WRITE,0x07,0x00,LCD_Controller_DCS_SET_OUTPUT_VERTICAL_TIMINGS,0x04,0x09,0x09,0x00,0x02,0x80}, /*set_output_vertical_timing*/
	{ 1,LCD_Controller_DCS_LONG_WRITE,0x07,0x00,LCD_Controller_DCS_SET_OUTPUT_HORIZONTAL_TIMINGS,0x28,0x13,0x3B,0x00,0x01,0xE0}, /*set_output_horizontal_timing*/
	/*wait 1ms*/
	{ 1, LCD_Controller_DCS_LONG_WRITE,0x05,0x00 , LCD_Controller_DCS_WRITE_EDISCO_REGISTER , 0xCC, 0x00, 0x00, 0x0E}, /* MRESET*/
	{ 1, LCD_Controller_DCS_LONG_WRITE,0x05,0x00 , LCD_Controller_DCS_WRITE_EDISCO_REGISTER , 0xC8, 0x00, 0x72, 0x42}, /* IOCTRL*/
	{ 1, LCD_Controller_DCS_LONG_WRITE,0x05,0x00 , LCD_Controller_DCS_WRITE_EDISCO_REGISTER , 0x44, 0x00, 0xFF, 0x00}, /* SSITIM1*/
	{ 1, LCD_Controller_DCS_LONG_WRITE,0x05,0x00 , LCD_Controller_DCS_WRITE_EDISCO_REGISTER , 0x40, 0x14, 0x00, 0x1A}, /* SSICTL*/
	/*wait 1ms*/
	{ 1,LCD_Controller_DCS_SHORT_WRITE_WITH_PARAMETER, LCD_Controller_DCS_GET_PIXEL_FORMAT , 0x05}, /* FIXME: hardcode here: set format*/
	{ 1,LCD_Controller_DCS_SHORT_WRITE_NO_PARAMETER,LCD_Controller_DCS_SET_DISPLAY_ON}, /* Display on*/
	{ 1,LCD_Controller_DCS_LONG_WRITE,0x05,0x00,LCD_Controller_DCS_WRITE_EDISCO_REGISTER,0x20,0x00,0x00,0x24}, /*LCDCTRL*/
	/*wait 1ms*/
	{0xFF,0xFF},
};

static u8 enter_sleep [][BOARD_INIT_MAX_DATA_BYTES] = {
	{0xff,LCD_Controller_DCS_SHORT_WRITE_NO_PARAMETER,LCD_Controller_DCS_SET_DISPLAY_OFF},
	{0x10,LCD_Controller_DCS_SHORT_WRITE_NO_PARAMETER,LCD_Controller_DCS_NOP},
	/*wait 10ms*/
	{0xff,LCD_Controller_DCS_SHORT_WRITE_NO_PARAMETER,LCD_Controller_DCS_ENTER_SLEEP_MODE},
	{0x10,LCD_Controller_DCS_SHORT_WRITE_NO_PARAMETER,LCD_Controller_DCS_NOP},
	/*wait 10ms*/
	{0xff,0xff},
};

static void lcdc_writel(unsigned int val, unsigned int *reg_base, unsigned int offset)
{
	int retry = 0;
	/* if display update is ongoing, register write should be avoided */
	if (display_enabled) {
		while ((display_update_ongoing[0]
					|| display_update_ongoing[1]
					|| display_update_ongoing[2])
				&& retry++ < 50)
			msleep(1);
		if(retry >= 50)
			printk("ERROR: pxa95xfb write %x -> %x failed.\n", val, offset);
	}
	if ((offset == LCD_MIXER0_CTL0)
				&& (val & LCD_MIXERx_CTL0_DISP_UPDATE)
				&& display_enabled)
			display_update_ongoing[0] = 1;
	if ((offset == LCD_MIXER1_CTL0)
				&& (val & LCD_MIXERx_CTL0_DISP_UPDATE)
				&& display_enabled)
			display_update_ongoing[1] = 1;
	if ((offset == LCD_MIXER2_CTL0)
				&& (val & LCD_MIXERx_CTL0_DISP_UPDATE)
				&& display_enabled)
			display_update_ongoing[2] = 1;

	writel(val, (unsigned int *)((unsigned int)reg_base + offset));
	return;
}

static int dvfm_dev_idx;
static void set_dvfm_constraint(void)
{
	/* Disable Lowpower mode */
	dvfm_disable_op_name("D0CS", dvfm_dev_idx);
	dvfm_disable_op_name("D1", dvfm_dev_idx);
	dvfm_disable_op_name("D2", dvfm_dev_idx);
	dvfm_disable_op_name("CG", dvfm_dev_idx);
}

static void set_dvfm_constraint_hdmi(void)
{
	/* Disable Lowpower mode */
	dvfm_disable_op_name("156M", dvfm_dev_idx);
	dvfm_disable_op_name("156M_HF", dvfm_dev_idx);
	dvfm_disable_op_name("416M", dvfm_dev_idx);
}

static void unset_dvfm_constraint(void)
{
	/* Enable Lowpower mode */
	dvfm_enable_op_name("D0CS", dvfm_dev_idx);
	dvfm_enable_op_name("D1", dvfm_dev_idx);
	dvfm_enable_op_name("D2", dvfm_dev_idx);
	dvfm_enable_op_name("CG", dvfm_dev_idx);
}

static void unset_dvfm_constraint_hdmi(void)
{
	/* Enable Lowpower mode */
	dvfm_enable_op_name("156M", dvfm_dev_idx);
	dvfm_enable_op_name("156M_HF", dvfm_dev_idx);
	dvfm_enable_op_name("416M", dvfm_dev_idx);
}

static void dsi_set_time(struct pxa95xfb_conv_info *conv, int freq)
{
	u32 THS_prep_min, THS_prep_max;
	u32 THS_trail_min, THS_trail_max, THS_zero_min, THS_lpx_min, CL_zero_min;
	u32 CL_trail_min, THS_exit_min;
	u8	DSI_HSPREP, DSI_HSTRAIL, DSI_HSZERO, DSI_HSEXIT, DSI_LPX, DSI_CLZERO, DSI_CLTRAIL;

	void *conv_base = conv->conv_base;

	THS_prep_min = 20;
	THS_prep_max = 50;
	THS_trail_min = 50;
	THS_trail_max = 75;
	THS_zero_min = 70;
	THS_exit_min = 100;
	THS_lpx_min = 50;
	CL_zero_min = 150;
	CL_trail_min = 60;

	DSI_HSPREP = (THS_prep_min + THS_prep_max-THS_prep_min/2)*52/1000;
	if (freq > 156)
		DSI_HSTRAIL = (THS_trail_min + THS_trail_max-THS_trail_min/2)*52/1000;
	else
		DSI_HSTRAIL = (THS_trail_min + THS_trail_max-THS_trail_min/2 + freq*8/1000)*52/1000;
	DSI_HSZERO = (THS_zero_min - freq*3/1000)*52/1000;
	DSI_HSEXIT = THS_exit_min*52/1000;
	DSI_LPX = THS_lpx_min*52/1000;
	DSI_CLZERO = CL_zero_min*52/1000;
	DSI_CLTRAIL = CL_trail_min*52/1000;

	/*Set timing configuration according to PXA95x docs*/

	/*Clear previous states*/
	writel(0, conv_base + LCD_DSI_DxPHY_TIM0_OFFSET);
	writel(0, conv_base + LCD_DSI_DxPHY_TIM1_OFFSET);
	writel(0, conv_base + LCD_DSI_DxPHY_TIM2_OFFSET);

	writel(LCD_DSI_DxPHY_TIM0_HSTRAIL(DSI_HSTRAIL)
		|LCD_DSI_DxPHY_TIM0_HSZERO(DSI_HSZERO)
		|LCD_DSI_DxPHY_TIM0_HSPREP(DSI_HSPREP)
		|LCD_DSI_DxPHY_TIM0_LPX(DSI_LPX),
		conv_base + LCD_DSI_DxPHY_TIM0_OFFSET);

	writel(LCD_DSI_DxPHY_TIM1_HSEXIT(DSI_HSEXIT)
		|LCD_DSI_DxPHY_TIM1_CLTRAIL(DSI_CLTRAIL)
		|LCD_DSI_DxPHY_TIM1_CLZERO(DSI_CLZERO)
		|LCD_DSI_DxPHY_TIM1_TAGO(12),
		conv_base + LCD_DSI_DxPHY_TIM1_OFFSET);

	writel(LCD_DSI_DxPHY_TIM2_REQ_RDY_DELAY(15)
		|LCD_DSI_DxPHY_TIM2_TAGET(15)
		|LCD_DSI_DxPHY_TIM2_WAKEUP(0xFFFF),
	   conv_base + LCD_DSI_DxPHY_TIM2_OFFSET);

	writel(4,
	   conv_base + LCD_DSI_DxPHY_CAL_OFFSET);

}


static u8 dsi_enable_disable(struct pxa95xfb_conv_info * conv, int enable_disable)
{
	u8 ret = 1;
	u32 x;
	void *conv_base = conv->conv_base;

	switch(enable_disable)
	{
		case DSI_DISABLE:
			x = readl(conv_base + LCD_DSI_DxSCR1_OFFSET) &
						(~LCD_DSI_DxSCR1_DSI_EN);
			writel(x, conv_base + LCD_DSI_DxSCR1_OFFSET);
			break;
		case DSI_ENABLE:
			x = readl(conv_base + LCD_DSI_DxSCR1_OFFSET)
							|LCD_DSI_DxSCR1_DSI_EN;
			writel(x, conv_base + LCD_DSI_DxSCR1_OFFSET);
			break;
		default:
			ret = 0;
			break;
	}

	return ret;
}

static u8 dsi_set_lanes_number(struct pxa95xfb_conv_info * conv, int nol)
{
	u8 ret = 1;
	u32 x;
	void *conv_base = conv->conv_base;

	if(nol>LCD_Controller_DSI_4LANE){
		/*only 4 lanes available*/
		ret = 0;
	}else{
		/*Clear previous state*/
		x = readl(conv_base + LCD_DSI_DxSCR1_OFFSET)
					& (~LCD_DSI_DxSCR1_NOL_MASK);
		writel(x, conv_base + LCD_DSI_DxSCR1_OFFSET);
		/*Set number of lanes*/
		x |= LCD_DSI_DxSCR1_NOL(nol);
		writel(x, conv_base + LCD_DSI_DxSCR1_OFFSET);
	}

	return ret;
}

static u8 dsi_set_power_mode(struct pxa95xfb_conv_info * conv, int power_mode)
{
	u8 ret = 1;
	u8 rihs_mode,ulps_mode,lpdt_mode,bta_mode;
	u32 x;
	void *conv_base = conv->conv_base;

	/* TODO: issue with panel??*/
	if(conv->output == OUTPUT_PANEL)
		rihs_mode = 0;
	else
		rihs_mode = power_mode&LCD_Controller_DSI_RIHS;
	ulps_mode = (power_mode&LCD_Controller_DSI_ULPS)>>1;
	lpdt_mode = (power_mode&LCD_Controller_DSI_LPDT)>>2;
	bta_mode = (power_mode&LCD_Controller_DSI_BTA)>>3;

	/*Clear previous state*/
	x = readl(conv_base + LCD_DSI_DxSCR1_OFFSET)
					& (~LCD_DSI_DxSCR1_POWER_MASK);
	writel(x, conv_base + LCD_DSI_DxSCR1_OFFSET);

	/*Set power mode*/
	x |= LCD_DSI_DxSCR1_RIHS(rihs_mode)|LCD_DSI_DxSCR1_ULPS(ulps_mode)
		|LCD_DSI_DxSCR1_LPDT(lpdt_mode)|LCD_DSI_DxSCR1_BTA(bta_mode);
	writel(x, conv_base + LCD_DSI_DxSCR1_OFFSET);

	return ret;
}

static u8 dsi_wait_interrupt(struct pxa95xfb_conv_info * conv, u32 time_to_wait)
{
	u8 ret = 1;
	u32 x;
	int i = 0;
	void *conv_base = conv->conv_base;

	/*Wait until we received interrupt from the command fifo*/
	x = readl(conv_base + LCD_DSI_DxINST0_OFFSET);
	while ((x & LCD_DSI_DxINST0_SP_GEN_INT_STS) !=
						LCD_DSI_DxINST0_SP_GEN_INT_STS){
		i++;
		msleep(100);
		x = readl(conv_base + LCD_DSI_DxINST0_OFFSET);
		if (i>50){
			ret = 0;
			break;
		}
	}
	if (i > 50)
		printk("%s: failed to receive DSI interrupt!!!\n", __func__);
	mdelay(time_to_wait);
	writel(x|LCD_DSI_DxINST0_SP_GEN_INT_STS,
				conv_base + LCD_DSI_DxINST0_OFFSET);
	return ret;

}

static inline void dsi_add_cmd(struct pxa95xfb_conv_info * conv, u8 dsi_cmd, u8 dsi_data, int dsi_loop)
{
	conv->dsi_cmd_buf[conv->dsi_cmd_index++] = LCD_DSI_COMMAND_DATA(dsi_data)
		|LCD_DSI_COMMAND_CMD(dsi_cmd)
		|LCD_DSI_COMMAND_LOOP(dsi_loop);
}

static void dsi_send_cmd(struct pxa95xfb_conv_info * conv)
{
	int i;

	if(conv->dsi_cmd_index%2==1)
		dsi_add_cmd(conv, LCD_Controller_NO_OPERATION,0,0);

	/* avoid interrupted when send cmd, time cost is 2-3 us*/
	local_irq_disable();
	/*send commands to the FIFO to enable frame readings*/
	for(i=0;i<conv->dsi_cmd_index-1;i=i+2){
		writel(((u32)conv->dsi_cmd_buf[i])|((u32)conv->dsi_cmd_buf[i+1]<<16),
			conv->conv_base + LCD_DSI_DxCFIF_OFFSET);
	}
	local_irq_enable();
	conv->dsi_cmd_index = 0;
}

static u32 dsi_send_cmd_array(struct pxa95xfb_conv_info * conv, u8* cmd_array)
{
	int i, line_index = 0;
	u8 parm1,parm2;
	u8 command, timeout;
	u8 *cmd_line;
	u16 param_count;

	conv->dsi_cmd_index=0;

	/*Get board parameters*/
	cmd_line = cmd_array + line_index* BOARD_INIT_MAX_DATA_BYTES;

	command = cmd_line[DSI_CMDLINE_CMD];
	while(command != 0xFF){
		parm1 = cmd_line[DSI_CMDLINE_PARM];
		parm2 = cmd_line[DSI_CMDLINE_PARM + 1];
		timeout = cmd_line[DSI_CMDLINE_TIMEOUT];

		switch(command){
			case LCD_Controller_DCS_READ_NO_PARAMETER:
			case LCD_Controller_DCS_SHORT_WRITE_NO_PARAMETER:
				dsi_add_cmd(conv, LCD_Controller_COMMAND_WRITE, LCD_Controller_DCS_SHORT_WRITE_NO_PARAMETER, 0);
				dsi_add_cmd(conv, LCD_Controller_COMMAND_WRITE, parm1, 0);
				dsi_add_cmd(conv, LCD_Controller_COMMAND_WRITE, 0, 0);
				break;
			case LCD_Controller_DCS_SHORT_WRITE_WITH_PARAMETER:
			case LCD_Controller_GENERIC_SHORT_WRITE_TWO_PARAMETERS:
			case LCD_Controller_SET_MAXIMUM_RETURN_PACKET_SIZE:
			case LCD_Controller_TURN_ON_PERIPHERAL:
				dsi_add_cmd(conv, LCD_Controller_COMMAND_WRITE, command, 0);
				dsi_add_cmd(conv, LCD_Controller_COMMAND_WRITE, parm1, 0);
				dsi_add_cmd(conv, LCD_Controller_COMMAND_WRITE, parm2, 0);
				break;
			case LCD_Controller_DCS_LONG_WRITE:
			case LCD_Controller_GENERIC_LONG_WRITE:
			case LCD_Controller_LONG_PACKET_TBD:
				dsi_add_cmd(conv, LCD_Controller_COMMAND_WRITE, command, 0);
				param_count = ((u16)parm1|((u16)parm2<<8));
				for(i=0;i<param_count+2;i++)
					dsi_add_cmd(conv, LCD_Controller_COMMAND_WRITE,cmd_line[i+DSI_CMDLINE_PARM],0);
				break;
			default:
				break;
		}

		/*Get timeout - if not 0 - add interrupt and exit. if 0xFF - dont add a nop*/
		if(!timeout){
			dsi_add_cmd(conv, LCD_Controller_NO_OPERATION,10,0);
		}else if (timeout !=0xFF){
			/*Interrupt the proccesor for finishing*/
			dsi_add_cmd(conv, LCD_Controller_INTERRUPT_THE_PROCESSOR,0,0);

			/*Send the commands*/
			dsi_send_cmd(conv);

			/*wait until the processor finish the DSI fifo sequence*/
			if(!dsi_wait_interrupt(conv,timeout)){
				printk("%s: dsi_wait_interrupt failed at commandline %d!!\n", __func__, line_index);
				return 0;
			}

			conv->dsi_cmd_index = 0;
		}

		/*Get the next command*/
		line_index ++;
		cmd_line = cmd_array + line_index* BOARD_INIT_MAX_DATA_BYTES;
		command = cmd_line[DSI_CMDLINE_CMD];
	}

	/*end with out send? send out cmds*/
	if(conv->dsi_cmd_index)
		dsi_send_cmd(conv);

	return 1;
}

static void __attribute__ ((unused)) dsi_frame_update_dcs(struct pxa95xfb_conv_info * conv)
{
	u16 vlines;
	u16 x ;

	switch (conv->pix_fmt_out){
		case PIX_FMTOUT_16_RGB565:
			x = (u16)(conv->xres *2 +1);
			break;
		case PIX_FMTOUT_24_RGB888:
			x = (u16)(conv->xres *3 +1);
			break;
		default:
			printk(KERN_ERR "%s: format %d not supported\n", __func__, conv->pix_fmt_out);
			return;
	}

	conv->dsi_cmd_index=0;

	dsi_add_cmd(conv, LCD_Controller_NO_OPERATION,0x22,0);
	dsi_add_cmd(conv, LCD_Controller_NO_OPERATION,0x22,0);

	/*WRITE_MEMORY_START with line from the LCD controller*/
	dsi_add_cmd(conv, LCD_Controller_COMMAND_WRITE_HOLD, LCD_Controller_DCS_LONG_WRITE, 0);
	dsi_add_cmd(conv, LCD_Controller_COMMAND_WRITE, (u8)(x & 0xff), 0);
	dsi_add_cmd(conv, LCD_Controller_COMMAND_WRITE, (u8)((x & 0xff00) >> 0x8), 0);
	dsi_add_cmd(conv, LCD_Controller_COMMAND_WRITE, LCD_Controller_DCS_WRITE_MEMORY_START, 0);
	dsi_add_cmd(conv, LCD_Controller_LINE_DATA_WRITE,0,0);

	/*Invalidate all loop buffer contents*/
	dsi_add_cmd(conv, LCD_Controller_FLUSH_LOOP_BUFFER,0,0);

	/*WRITE_MEMORY_CONTINUE with line from the LCD controller*/
	dsi_add_cmd(conv, LCD_Controller_COMMAND_WRITE_HOLD, LCD_Controller_DCS_LONG_WRITE, 1);
	dsi_add_cmd(conv, LCD_Controller_COMMAND_WRITE, (u8)(x & 0xff), 1);
	dsi_add_cmd(conv, LCD_Controller_COMMAND_WRITE, (u8)((x & 0xff00) >> 0x8), 1);
	dsi_add_cmd(conv, LCD_Controller_COMMAND_WRITE, LCD_Controller_DCS_WRITE_MEMORY_CONTINUE, 1);
	dsi_add_cmd(conv, LCD_Controller_LINE_DATA_WRITE,0,1);

	/*Execute loop according to Vertical resolution */
	vlines = conv->yres -2;
	while(vlines>0x100){
		dsi_add_cmd(conv, LCD_Controller_EXECUTE_LOOP_BUFFER,0xff,0);
		vlines -= 0x100;
	}
	dsi_add_cmd(conv, LCD_Controller_EXECUTE_LOOP_BUFFER,(u8)(vlines-1),0);

	dsi_add_cmd(conv, LCD_Controller_NO_OPERATION,0x22,0);

	/*Invalidate all loop buffer contents*/
	dsi_add_cmd(conv, LCD_Controller_FLUSH_LOOP_BUFFER,0,0);
	/*Send end of transmition*/
	dsi_add_cmd(conv, LCD_Controller_COMMAND_WRITE, LCD_Controller_END_OF_TRANSMITION, 0);
	dsi_add_cmd(conv, LCD_Controller_COMMAND_WRITE, 0xf, 0);
	dsi_add_cmd(conv, LCD_Controller_COMMAND_WRITE, 0xf, 0);

	dsi_send_cmd(conv);
}

static void dsi_frame_update_packet(struct pxa95xfb_conv_info * conv)
{
	u16 vlines;
	u16 x = (u16)(conv->xres);

	conv->dsi_cmd_index=0;

	dsi_add_cmd(conv, LCD_Controller_NO_OPERATION,0x22,0);
	dsi_add_cmd(conv, LCD_Controller_NO_OPERATION,0x22,0);

	/*WRITE_MEMORY_START with line from the LCD controller*/
	dsi_add_cmd(conv, LCD_Controller_COMMAND_WRITE_HOLD, LCD_Controller_DCS_SHORT_WRITE_NO_PARAMETER, 0);
	dsi_add_cmd(conv, LCD_Controller_COMMAND_WRITE, LCD_Controller_DCS_WRITE_MEMORY_START, 0);
	dsi_add_cmd(conv, LCD_Controller_COMMAND_WRITE, 0, 0);

	/*workaround due d-phy SI issue*/
	dsi_add_cmd(conv, LCD_Controller_COMMAND_WRITE_HOLD, LCD_Controller_DCS_SHORT_WRITE_NO_PARAMETER, 0);
	dsi_add_cmd(conv, LCD_Controller_COMMAND_WRITE, LCD_Controller_DCS_WRITE_MEMORY_START, 0);
	dsi_add_cmd(conv, LCD_Controller_COMMAND_WRITE, 0, 0);

#if 0
	/*if such code is not removed, panel would be floating*/
	/*RGB packet with loop*/
	dsi_add_cmd(conv, LCD_Controller_COMMAND_WRITE, dsi_rgb_mode, 0);
	dsi_add_cmd(conv, LCD_Controller_COMMAND_WRITE, (u8)(x & 0xff), 0);
	dsi_add_cmd(conv, LCD_Controller_COMMAND_WRITE, (u8)((x & 0xff00) >> 0x8), 0);

	/*Line data write with loop - send a line from memory*/
	dsi_add_cmd(conv, LCD_Controller_LINE_DATA_WRITE,0,0);
#endif
	/*Invalidate all loop buffer contents*/
	dsi_add_cmd(conv, LCD_Controller_FLUSH_LOOP_BUFFER,0,0);

	/*WRITE_MEMORY_CONTINUE with line from the LCD controller*/
	dsi_add_cmd(conv, LCD_Controller_COMMAND_WRITE_HOLD, LCD_Controller_DCS_SHORT_WRITE_NO_PARAMETER, 1);
	dsi_add_cmd(conv, LCD_Controller_COMMAND_WRITE, LCD_Controller_DCS_WRITE_MEMORY_CONTINUE, 1);
	dsi_add_cmd(conv,LCD_Controller_COMMAND_WRITE, 0, 1);
	/*RGB packet with loop*/
	dsi_add_cmd(conv,LCD_Controller_COMMAND_WRITE, LCD_Controller_RGB_565_PACKET, 1);
	dsi_add_cmd(conv,LCD_Controller_COMMAND_WRITE, (u8)(x & 0xff), 1);
	dsi_add_cmd(conv,LCD_Controller_COMMAND_WRITE, (u8)((x & 0xff00) >> 0x8), 1);
	/*Line data write with loop - send a line from memory*/
	dsi_add_cmd(conv,LCD_Controller_LINE_DATA_WRITE,0,1);

	/*Execute loop according to Vertical resolution */
	vlines = conv->yres -1;
	while(vlines>0x100){
		dsi_add_cmd(conv,LCD_Controller_EXECUTE_LOOP_BUFFER,0xff,0);
		vlines -= 0x100;
	}
	dsi_add_cmd(conv,LCD_Controller_EXECUTE_LOOP_BUFFER,(u8)(vlines-1),0);

	dsi_add_cmd(conv,LCD_Controller_NO_OPERATION,0x22,0);

	/*Invalidate all loop buffer contents*/
	dsi_add_cmd(conv,LCD_Controller_FLUSH_LOOP_BUFFER,0,0);
	/*Send end of transmition*/
	dsi_add_cmd(conv,LCD_Controller_COMMAND_WRITE, LCD_Controller_END_OF_TRANSMITION, 0);
	dsi_add_cmd(conv,LCD_Controller_COMMAND_WRITE, 0xf, 0);
	dsi_add_cmd(conv,LCD_Controller_COMMAND_WRITE, 0xf, 0);

	dsi_send_cmd(conv);

}

static void dsi_init_video_burst(struct pxa95xfb_conv_info * conv)
{
	u16 vlines;
	u16 x = (u16)(conv->xres);

	conv->dsi_cmd_index=0;

	dsi_add_cmd(conv,LCD_Controller_START_LABEL,0,0);

	/*Set delay for Nop - for line*/
	dsi_add_cmd(conv,LCD_Controller_SET_DLY_MULT,20,0);

	/*vsync start*/
	dsi_add_cmd(conv,LCD_Controller_COMMAND_WRITE, LCD_Controller_VSYNC_START, 0);
	dsi_add_cmd(conv,LCD_Controller_COMMAND_WRITE, 0, 0);
	dsi_add_cmd(conv,LCD_Controller_COMMAND_WRITE, 0, 0);

	/*vsync end*/
	dsi_add_cmd(conv,LCD_Controller_COMMAND_WRITE, LCD_Controller_VSYNC_END, 0);
	dsi_add_cmd(conv,LCD_Controller_COMMAND_WRITE, 0, 0);
	dsi_add_cmd(conv,LCD_Controller_COMMAND_WRITE, 0, 0);

	/*Invalidate all loop buffer contents*/
	dsi_add_cmd(conv,LCD_Controller_FLUSH_LOOP_BUFFER,0,0);

	/*hsync start*/
	dsi_add_cmd(conv,LCD_Controller_COMMAND_WRITE, LCD_Controller_HSYNC_START, 1);
	dsi_add_cmd(conv,LCD_Controller_COMMAND_WRITE, 0, 1);
	dsi_add_cmd(conv,LCD_Controller_COMMAND_WRITE, 0, 1);

	/*hsync end*/
	dsi_add_cmd(conv,LCD_Controller_COMMAND_WRITE, LCD_Controller_HSYNC_END, 1);
	dsi_add_cmd(conv,LCD_Controller_COMMAND_WRITE, 0, 1);
	dsi_add_cmd(conv,LCD_Controller_COMMAND_WRITE, 0, 1);

	/*write RGB packet*/
	dsi_add_cmd(conv,LCD_Controller_COMMAND_WRITE, conv->dsi_rgb_mode, 1);
	dsi_add_cmd(conv,LCD_Controller_COMMAND_WRITE, (u8)(x & 0xff), 1);
	dsi_add_cmd(conv,LCD_Controller_COMMAND_WRITE, (u8)((x & 0xff00) >> 0x8), 1);
	/*Line data write - send a line from memory*/
	dsi_add_cmd(conv,LCD_Controller_LINE_DATA_WRITE,0,1);

	/*Execute loop according to Vertical resolution */
	vlines = conv->yres -1;
	while(vlines>0x100){
		dsi_add_cmd(conv,LCD_Controller_EXECUTE_LOOP_BUFFER,0xff,0);
		vlines -= 0x100;
	}
	dsi_add_cmd(conv,LCD_Controller_EXECUTE_LOOP_BUFFER,(u8)(vlines-1),0);

	/*Invalidate all loop buffer contents*/
	dsi_add_cmd(conv,LCD_Controller_FLUSH_LOOP_BUFFER,0,0);

	/*go to start*/
	dsi_add_cmd(conv,LCD_Controller_GOTO_START, 0, 0);

	dsi_send_cmd(conv);
}

static void dsi_init_video_non_burst(struct pxa95xfb_conv_info * conv)
{
	u16 vlines;
	u8 hsync_blank = conv->hsync_len;
	int active_hsw_compensation = 3,active_blw_compensation = 5, active_elw_compensation = 4, blank_pixel_compensation = 8;
	u32 blank_per_line_no_hsw = conv->left_margin + conv->xres + conv->right_margin - blank_pixel_compensation;

	conv->dsi_cmd_index=0;

	/* Begin Subroutine */
	dsi_add_cmd(conv, LCD_Controller_START_LABEL, 0, 0);

	/*Assert VSYNC*/
	/* VSYNC start */
	dsi_add_cmd(conv, LCD_Controller_COMMAND_WRITE, LCD_Controller_VSYNC_START, 0);
	dsi_add_cmd(conv, LCD_Controller_COMMAND_WRITE, 0, 0);
	dsi_add_cmd(conv, LCD_Controller_COMMAND_WRITE, 0, 0);

	/* HSW delay */
	dsi_add_cmd(conv, LCD_Controller_COMMAND_WRITE, LCD_Controller_BLANKING_PACKET, 0);
	dsi_add_cmd(conv, LCD_Controller_COMMAND_WRITE, hsync_blank, 0);
	dsi_add_cmd(conv, LCD_Controller_COMMAND_WRITE, 0, 0);

	/* Line data write - send a blank data from memory */
	dsi_add_cmd(conv, LCD_Controller_LINE_DATA_WRITE, 0, 0);

	/* HSYNC end */
	dsi_add_cmd(conv, LCD_Controller_COMMAND_WRITE, LCD_Controller_HSYNC_END, 0);
	dsi_add_cmd(conv, LCD_Controller_COMMAND_WRITE, 0, 0);
	dsi_add_cmd(conv, LCD_Controller_COMMAND_WRITE, 0, 0);

	/* Rest of the line delay */
	dsi_add_cmd(conv, LCD_Controller_COMMAND_WRITE, LCD_Controller_BLANKING_PACKET, 0);
	dsi_add_cmd(conv, LCD_Controller_COMMAND_WRITE, blank_per_line_no_hsw & 0xFF, 0);
	dsi_add_cmd(conv, LCD_Controller_COMMAND_WRITE, (blank_per_line_no_hsw & 0xFF00)>>8, 0);

	/* Line data write - send a blank data from memory */
	dsi_add_cmd(conv, LCD_Controller_LINE_DATA_WRITE, 0, 0);

	/* Add DSI phy compensation for the next line will start from byte 0 */
	dsi_add_cmd(conv, LCD_Controller_COMMAND_WRITE, LCD_Controller_SET_MAXIMUM_RETURN_PACKET_SIZE, 0);
	dsi_add_cmd(conv, LCD_Controller_COMMAND_WRITE, 1, 0);
	dsi_add_cmd(conv, LCD_Controller_COMMAND_WRITE, 0, 0);

	/* Invalidate all loop buffer contents */
	dsi_add_cmd(conv, LCD_Controller_FLUSH_LOOP_BUFFER, 0, 0);

	/* HSYNC start */
	dsi_add_cmd(conv, LCD_Controller_COMMAND_WRITE, LCD_Controller_HSYNC_START, 1);
	dsi_add_cmd(conv, LCD_Controller_COMMAND_WRITE, 0, 1);
	dsi_add_cmd(conv, LCD_Controller_COMMAND_WRITE, 0, 1);

	/* HSW delay */
	dsi_add_cmd(conv, LCD_Controller_COMMAND_WRITE, LCD_Controller_BLANKING_PACKET, 1);
	dsi_add_cmd(conv, LCD_Controller_COMMAND_WRITE, hsync_blank, 1);
	dsi_add_cmd(conv, LCD_Controller_COMMAND_WRITE, 0, 1);

	/* Line data write - send a blank data from memory */
	dsi_add_cmd(conv, LCD_Controller_LINE_DATA_WRITE, 0, 1);

	/* HSYNC end */
	dsi_add_cmd(conv, LCD_Controller_COMMAND_WRITE, LCD_Controller_HSYNC_END, 1);
	dsi_add_cmd(conv, LCD_Controller_COMMAND_WRITE, 0, 1);
	dsi_add_cmd(conv, LCD_Controller_COMMAND_WRITE, 0, 1);

	/* Rest of the line delay */
	dsi_add_cmd(conv, LCD_Controller_COMMAND_WRITE, LCD_Controller_BLANKING_PACKET, 1);
	dsi_add_cmd(conv, LCD_Controller_COMMAND_WRITE, blank_per_line_no_hsw & 0xFF, 1);
	dsi_add_cmd(conv, LCD_Controller_COMMAND_WRITE, (blank_per_line_no_hsw & 0xFF00)>>8, 1);

	/* Line data write - send a blank data from memory */
	dsi_add_cmd(conv, LCD_Controller_LINE_DATA_WRITE, 0, 1);

	/* Add DSI phy compensation for the next line will start from byte 0 */
	dsi_add_cmd(conv, LCD_Controller_COMMAND_WRITE, LCD_Controller_SET_MAXIMUM_RETURN_PACKET_SIZE, 1);
	dsi_add_cmd(conv, LCD_Controller_COMMAND_WRITE, 1, 1);
	dsi_add_cmd(conv, LCD_Controller_COMMAND_WRITE, 0, 1);

	/* loop according to VSW */
	if(conv->vsync_len > 2)
		dsi_add_cmd(conv, LCD_Controller_EXECUTE_LOOP_BUFFER, conv->vsync_len - 3, 0);

	/* Invalidate all loop buffer contents */
	dsi_add_cmd(conv, LCD_Controller_FLUSH_LOOP_BUFFER, 0, 0);

	/* VSYNC END */
	dsi_add_cmd(conv, LCD_Controller_COMMAND_WRITE, LCD_Controller_VSYNC_END, 0);
	dsi_add_cmd(conv, LCD_Controller_COMMAND_WRITE, 0, 0);
	dsi_add_cmd(conv, LCD_Controller_COMMAND_WRITE, 0, 0);

	/* HSW delay */
	dsi_add_cmd(conv, LCD_Controller_COMMAND_WRITE, LCD_Controller_BLANKING_PACKET, 0);
	dsi_add_cmd(conv, LCD_Controller_COMMAND_WRITE, hsync_blank, 0);
	dsi_add_cmd(conv, LCD_Controller_COMMAND_WRITE, 0, 0);

	/* Line data write - send a blank data from memory */
	dsi_add_cmd(conv, LCD_Controller_LINE_DATA_WRITE, 0, 0);

	/* HSYNC end */
	dsi_add_cmd(conv, LCD_Controller_COMMAND_WRITE, LCD_Controller_HSYNC_END, 0);
	dsi_add_cmd(conv, LCD_Controller_COMMAND_WRITE, 0, 0);
	dsi_add_cmd(conv, LCD_Controller_COMMAND_WRITE, 0, 0);

	/* Rest of the line delay */
	dsi_add_cmd(conv, LCD_Controller_COMMAND_WRITE, LCD_Controller_BLANKING_PACKET, 0);
	dsi_add_cmd(conv, LCD_Controller_COMMAND_WRITE, blank_per_line_no_hsw & 0xFF, 0);
	dsi_add_cmd(conv, LCD_Controller_COMMAND_WRITE, (blank_per_line_no_hsw & 0xFF00)>>8, 0);

	/* Line data write - send a blank data from memory */
	dsi_add_cmd(conv, LCD_Controller_LINE_DATA_WRITE, 0, 0);

	/* Add DSI phy compensation for the next line will start from byte 0 */
	dsi_add_cmd(conv, LCD_Controller_COMMAND_WRITE, LCD_Controller_SET_MAXIMUM_RETURN_PACKET_SIZE, 0);
	dsi_add_cmd(conv, LCD_Controller_COMMAND_WRITE, 1, 0);
	dsi_add_cmd(conv, LCD_Controller_COMMAND_WRITE, 0, 0);

	/*Frame back porch*/
	/* HSYNC start */
	dsi_add_cmd(conv, LCD_Controller_COMMAND_WRITE, LCD_Controller_HSYNC_START, 1);
	dsi_add_cmd(conv, LCD_Controller_COMMAND_WRITE, 0, 1);
	dsi_add_cmd(conv, LCD_Controller_COMMAND_WRITE, 0, 1);

	/* HSW delay */
	dsi_add_cmd(conv, LCD_Controller_COMMAND_WRITE, LCD_Controller_BLANKING_PACKET, 1);
	dsi_add_cmd(conv, LCD_Controller_COMMAND_WRITE, hsync_blank, 1);
	dsi_add_cmd(conv, LCD_Controller_COMMAND_WRITE, 0, 1);

	/* Line data write - send a blank data from memory */
	dsi_add_cmd(conv, LCD_Controller_LINE_DATA_WRITE, 0, 1);

	/* HSYNC end */
	dsi_add_cmd(conv, LCD_Controller_COMMAND_WRITE, LCD_Controller_HSYNC_END, 1);
	dsi_add_cmd(conv, LCD_Controller_COMMAND_WRITE, 0, 1);
	dsi_add_cmd(conv, LCD_Controller_COMMAND_WRITE, 0, 1);

	/* Rest of the line delay */
	dsi_add_cmd(conv, LCD_Controller_COMMAND_WRITE, LCD_Controller_BLANKING_PACKET, 1);
	dsi_add_cmd(conv, LCD_Controller_COMMAND_WRITE, blank_per_line_no_hsw & 0xFF, 1);
	dsi_add_cmd(conv, LCD_Controller_COMMAND_WRITE, (blank_per_line_no_hsw & 0xFF00)>>8, 1);

	/* Line data write - send a blank data from memory */
	dsi_add_cmd(conv, LCD_Controller_LINE_DATA_WRITE, 0, 1);

	/* Add DSI phy compensation for the next line will start from byte 0 */
	dsi_add_cmd(conv, LCD_Controller_COMMAND_WRITE, LCD_Controller_SET_MAXIMUM_RETURN_PACKET_SIZE, 1);
	dsi_add_cmd(conv, LCD_Controller_COMMAND_WRITE, 1, 1);
	dsi_add_cmd(conv, LCD_Controller_COMMAND_WRITE, 0, 1);

	/* loop according to BFW */
	if(conv->upper_margin > 2)
		dsi_add_cmd(conv, LCD_Controller_EXECUTE_LOOP_BUFFER, conv->upper_margin - 3, 0);

	/* Invalidate all loop buffer contents */
	dsi_add_cmd(conv, LCD_Controller_FLUSH_LOOP_BUFFER, 0, 0);

	/*Output actual frame*/
	/* HSYNC start */
	dsi_add_cmd(conv, LCD_Controller_COMMAND_WRITE, LCD_Controller_HSYNC_START, 1);
	dsi_add_cmd(conv, LCD_Controller_COMMAND_WRITE, 0, 1);
	dsi_add_cmd(conv, LCD_Controller_COMMAND_WRITE, 0, 1);

	/* HSW delay */
	dsi_add_cmd(conv, LCD_Controller_COMMAND_WRITE, LCD_Controller_BLANKING_PACKET, 1);
	dsi_add_cmd(conv, LCD_Controller_COMMAND_WRITE, hsync_blank - active_hsw_compensation, 1);
	dsi_add_cmd(conv, LCD_Controller_COMMAND_WRITE, 0, 1);

	/* Line data write - send a blank data from memory */
	dsi_add_cmd(conv, LCD_Controller_LINE_DATA_WRITE, 0, 1);

	/* HSYNC end */
	dsi_add_cmd(conv, LCD_Controller_COMMAND_WRITE, LCD_Controller_HSYNC_END, 1);
	dsi_add_cmd(conv, LCD_Controller_COMMAND_WRITE, 0, 1);
	dsi_add_cmd(conv, LCD_Controller_COMMAND_WRITE, 0, 1);

	/* Add DSI phy compensation for the next line will start from byte 0 */
	dsi_add_cmd(conv, LCD_Controller_COMMAND_WRITE, LCD_Controller_SET_MAXIMUM_RETURN_PACKET_SIZE, 1);
	dsi_add_cmd(conv, LCD_Controller_COMMAND_WRITE, 1, 1);
	dsi_add_cmd(conv, LCD_Controller_COMMAND_WRITE, 0, 1);

	/* HBP delay */
	dsi_add_cmd(conv, LCD_Controller_COMMAND_WRITE, LCD_Controller_BLANKING_PACKET, 1);
	dsi_add_cmd(conv, LCD_Controller_COMMAND_WRITE, conv->left_margin - active_blw_compensation, 1);
	dsi_add_cmd(conv, LCD_Controller_COMMAND_WRITE, 0, 1);

	/* Line data write - send a blank data from memory */
	dsi_add_cmd(conv, LCD_Controller_LINE_DATA_WRITE, 0, 1);

	/* Send Line */
	dsi_add_cmd(conv, LCD_Controller_COMMAND_WRITE, conv->dsi_rgb_mode, 1);
	dsi_add_cmd(conv, LCD_Controller_COMMAND_WRITE, (u8)(conv->xres & 0xff), 1);
	dsi_add_cmd(conv, LCD_Controller_COMMAND_WRITE, (u8)((conv->xres & 0xff00) >> 0x8), 1);

	/* Line data write - send a blank data from memory */
	dsi_add_cmd(conv, LCD_Controller_LINE_DATA_WRITE, 0, 1);

	/* HFP delay */
	dsi_add_cmd(conv, LCD_Controller_COMMAND_WRITE, LCD_Controller_BLANKING_PACKET, 1);
	dsi_add_cmd(conv, LCD_Controller_COMMAND_WRITE, (conv->right_margin - active_elw_compensation) & 0xFF, 1);
	dsi_add_cmd(conv, LCD_Controller_COMMAND_WRITE, ((conv->right_margin - active_elw_compensation) & 0xFF00) >> 8, 1);

	/* Line data write - send a blank data from memory */
	dsi_add_cmd(conv, LCD_Controller_LINE_DATA_WRITE, 0, 1);

	/* Execute loop according to Vertical resolution - 1 (above) */
	vlines = conv->yres - 1;
	while(vlines > 0x100) {
		dsi_add_cmd(conv, LCD_Controller_EXECUTE_LOOP_BUFFER, 0xFF, 0);
		vlines -= 0x100;
	}
	dsi_add_cmd(conv, LCD_Controller_EXECUTE_LOOP_BUFFER, (u8)(vlines - 1), 0);

	/* Invalidate all loop buffer contents */
	dsi_add_cmd(conv, LCD_Controller_FLUSH_LOOP_BUFFER, 0, 0);

	/*Frame front porch*/
	/* HSYNC start */
	dsi_add_cmd(conv, LCD_Controller_COMMAND_WRITE, LCD_Controller_HSYNC_START, 1);
	dsi_add_cmd(conv, LCD_Controller_COMMAND_WRITE, 0, 1);
	dsi_add_cmd(conv, LCD_Controller_COMMAND_WRITE, 0, 1);

	/* HSW delay */
	dsi_add_cmd(conv, LCD_Controller_COMMAND_WRITE, LCD_Controller_BLANKING_PACKET, 1);
	dsi_add_cmd(conv, LCD_Controller_COMMAND_WRITE, hsync_blank, 1);
	dsi_add_cmd(conv, LCD_Controller_COMMAND_WRITE, 0, 1);

	/* Line data write - send a blank data from memory */
	dsi_add_cmd(conv, LCD_Controller_LINE_DATA_WRITE, 0, 1);

	/* HSYNC end */
	dsi_add_cmd(conv, LCD_Controller_COMMAND_WRITE, LCD_Controller_HSYNC_END, 1);
	dsi_add_cmd(conv, LCD_Controller_COMMAND_WRITE, 0, 1);
	dsi_add_cmd(conv, LCD_Controller_COMMAND_WRITE, 0, 1);

	/* Rest of the line delay */
	dsi_add_cmd(conv, LCD_Controller_COMMAND_WRITE, LCD_Controller_BLANKING_PACKET, 1);
	dsi_add_cmd(conv, LCD_Controller_COMMAND_WRITE, blank_per_line_no_hsw & 0xFF, 1);
	dsi_add_cmd(conv, LCD_Controller_COMMAND_WRITE, (blank_per_line_no_hsw & 0xFF00)>>8, 1);

	/* Line data write - send a blank data from memory */
	dsi_add_cmd(conv, LCD_Controller_LINE_DATA_WRITE, 0, 1);

	/* Add DSI phy compensation for the next line will start from byte 0 */
	dsi_add_cmd(conv, LCD_Controller_COMMAND_WRITE, LCD_Controller_SET_MAXIMUM_RETURN_PACKET_SIZE, 1);
	dsi_add_cmd(conv, LCD_Controller_COMMAND_WRITE, 1, 1);
	dsi_add_cmd(conv, LCD_Controller_COMMAND_WRITE, 0, 1);

	/* loop according to EFW */
	if(conv->lower_margin > 1)
		dsi_add_cmd(conv, LCD_Controller_EXECUTE_LOOP_BUFFER, conv->lower_margin - 2, 0);

	/* Invalidate all loop buffer contents */
	dsi_add_cmd(conv, LCD_Controller_FLUSH_LOOP_BUFFER, 0, 0);

	/*END - go to start for a new frame*/
	dsi_add_cmd(conv, LCD_Controller_GOTO_START, 0, 0);

	dsi_send_cmd(conv);
}

static void dsi_send_frame(struct pxa95xfb_conv_info * conv)
{
	u32 x;
	if(CONVERTER_IS_DSI(conv->converter)){
		x = readl(conv->conv_base + LCD_DSI_DxSSR_OFFSET);
		while((LCD_DSI_DxSSR_STOP_ST | LCD_DSI_DxSSR_STOP_ST_ALL) !=
				(x & (LCD_DSI_DxSSR_STOP_ST | LCD_DSI_DxSSR_STOP_ST_ALL))){
			msleep(1);
			x = readl(conv->conv_base + LCD_DSI_DxSSR_OFFSET);
		}
		dsi_frame_update_packet(conv);
	}
}

static u32 dsi_init(struct pxa95xfb_conv_info * conv)
{
	u8 ret = 1;

	/*Set default values - Low Power and 2 lanes*/
	dsi_set_lanes_number(conv, conv->dsi_lanes);
	dsi_set_power_mode(conv, LCD_Controller_DSI_LPDT);

	/*Enable the DSI*/
	if(!dsi_enable_disable(conv, DSI_ENABLE)){
		printk("%s: dsi_enable_disable failed!!\n", __func__);
		return 0;
	}

	/*Enable the LCD controller*/
	controller_enable_disable(pxa95xfbi[0], LCD_Controller_Enable);
	msleep(100);

	/*End of inialization - init board*/
	printk("%s: Init_board\n", __func__);

	/*Handle board - only support this currently */
	if (conv->dsi_init_cmds && !dsi_send_cmd_array(conv, conv->dsi_init_cmds)){
		printk("%s: dsi_init_board failed!!\n", __func__);
		return 0;
	}

	/*Set power mode*/
	dsi_set_power_mode(conv, LCD_Controller_DSI_RIHS);

	return ret;

}

static u32 dsi_enter_sleep(struct pxa95xfb_conv_info *conv)
{
	u32 x;
	/*for video mode, we need to disable FIFO*/
	if (conv->conf_dsi_video_mode) {
		x = readl(conv->conv_base + LCD_DSI_DxSCR0_OFFSET);
		writel(x | LCD_DSI_DxSCR0_SP_BREAK_INT_EN | LCD_DSI_DxSCR0_BREAK,
			conv->conv_base + LCD_DSI_DxSCR0_OFFSET);
		/* TODO: checking DxINST0 would always fail
		* and only SP_UNDRUN_INT_STS is set, just do wait now*/
		msleep(50);
		pr_info("%s: disable fifo when video mode done, DxINST0 = %x!!!\n",
			__func__, readl(conv->conv_base + LCD_DSI_DxINST0_OFFSET));
		writel(LCD_DSI_DxINST0_BREAK_INT_STS,
			conv->conv_base + LCD_DSI_DxINST0_OFFSET);
	}

	dsi_set_power_mode(conv, LCD_Controller_DSI_LPDT);
	msleep(10);
	if (conv->dsi_sleep_cmds && !dsi_send_cmd_array(conv, conv->dsi_sleep_cmds)) {
		printk("%s: dsi_send_enter_sleep failed!!\n", __func__);
		return 0;
	}
	return 1;
}

static int loop_kthread_body(void *arg)
{
	struct loop_kthread * thread = arg;
	int save_status = thread->is_run;
	int timeout = save_status ? thread->interval: MAX_SCHEDULE_TIMEOUT;
	u32 s=0, c=0, t;
	while(1) {
		if(wait_event_interruptible_timeout(thread->wq_main, (save_status != thread->is_run), timeout))	{
			mutex_lock(&thread->mutex);
			save_status = thread->is_run;
			timeout = save_status ? thread->interval: MAX_SCHEDULE_TIMEOUT;
			if(!save_status)
				complete(&thread->stopped);
			mutex_unlock(&thread->mutex);
		}
		if (save_status)
			thread->op(thread->par);

		if(c)
			s += (OSCR - t)*4 /13000;
		c++;
		t = OSCR;
		/*if(c %100 == 0)
			printk("%s: count %d, average dur %d\n", __func__, c, s/c);*/
	}
	return 0;
}

static int loop_kthread_init(struct loop_kthread * thread,  const char * name, void *par, void *op, int interval)
{
	thread->is_run = 0;
	thread->op = op;
	thread->interval = interval;
	thread->par = par;
	init_waitqueue_head(&thread->wq_main);
	mutex_init(&thread->mutex);
	init_completion(&thread->stopped);

	thread->thread = kthread_run(loop_kthread_body, (void *)thread, name);
	if (IS_ERR(thread->thread)) {
		pr_err("%s: unable to create kernel thread %s\n", __func__, name);
		return 0;
	}
	return 1;
}

static void loop_kthread_resume(struct loop_kthread * thread)
{
	mutex_lock(&thread->mutex);
	thread->is_run = 1;
	wake_up(&thread->wq_main);
	mutex_unlock(&thread->mutex);
}

static void loop_kthread_pause(struct loop_kthread * thread)
{
	mutex_lock(&thread->mutex);
	thread->is_run = 0;
	wake_up(&thread->wq_main);
	mutex_unlock(&thread->mutex);
	wait_for_completion(&thread->stopped);
}

static void converter_set_gamma(struct pxa95xfb_conv_info *conv)
{
	int i, x;
	u32 RedLinearLUT[] = {0x30201000, 0x70605040, 0xb0a09080, 0xf0e0d0c0, 0x000000ff};
	u32 GreenLinearLUT[] = {0x30201000, 0x70605040, 0xb0a09080, 0xf0e0d0c0, 0x000000ff};
	u32 BlueLinearLUT[] = {0x30201000, 0x70605040, 0xb0a09080, 0xf0e0d0c0, 0x000000ff};
	void *conv_base = conv->conv_base;

	for(i=0;i<5;i++){
		lcdc_writel(LCD_CONVx_G_CTL_ADD_PTR_R(i)
			 | LCD_CONVx_G_CTL_ADD_PTR_G(i)
			| LCD_CONVx_G_CTL_ADD_PTR_B(i),
			 conv_base, LCD_DSI_Dx_G_CTL_OFFSET);
		lcdc_writel(RedLinearLUT[i],
			 conv_base, LCD_DSI_Dx_G_DAT_RED_OFFSET);
		lcdc_writel(GreenLinearLUT[i],
			 conv_base, LCD_DSI_Dx_G_DAT_GREEN_OFFSET);
		lcdc_writel(BlueLinearLUT[i],
			 conv_base, LCD_DSI_Dx_G_DAT_BLUE_OFFSET);
	}
	x = readl(conv_base + LCD_DSI_Dx_G_CTL_OFFSET);
	x |= LCD_CONVx_G_CTL_Q4_C(0)|LCD_CONVx_G_CTL_Q3_C(0)
		|LCD_CONVx_G_CTL_Q2_C(0)|LCD_CONVx_G_CTL_Q1_C(0);
	lcdc_writel(x, conv_base, LCD_DSI_Dx_G_CTL_OFFSET);
}

static void set_clock_divider(struct pxa95xfb_info *fbi)
{
	int divider_int;
	int needed_pixclk;
	int lcd_src_clk;
	u64 div_result;
	u32 x = 0;
	struct fb_videomode * m = &fbi->mode;

	/*
	 * Calc divider according to refresh rate.
	 */
	div_result = 1000000000000ll;
	do_div(div_result, m->pixclock);
	needed_pixclk = (u32)div_result;

	x = ACCR1 & (0x1<<9);
	if(x)
		lcd_src_clk = 120000000ll;
	else
		lcd_src_clk = 208000000ll;

	divider_int = lcd_src_clk / needed_pixclk;

	/* check whether divisor is too small. */
	if (divider_int < 2) {
		printk(KERN_WARNING "Warning: clock source is too slow."
				"Try smaller resolution\n");
		divider_int = 2;
	}

	/* Set setting to reg. */
	x = readl(fbi->reg_base + LCD_CTL);
	x &= 0xfffffe00;
	x |= divider_int;
	lcdc_writel(x, fbi->reg_base, LCD_CTL);
	printk(KERN_ERR "%s: LCD_CTL=0x%x, divider_int=0x%x\n", __func__, \
	x, divider_int);
}

static void converter_set_parallel(struct pxa95xfb_info *fbi)
{
	u32 x;
	struct pxa95xfb_conv_info *conv = &pxa95xfb_conv[fbi->converter - 1];
	struct fb_videomode * m = &fbi->mode;

	/* gamma is in converter set*/
	converter_set_gamma(conv);
	if (conv->output == OUTPUT_HDMI) {
		/*Hezi - Temp W/A for fixing HDMI Color issue*/
		x = LCD_MIXERx_TIM0_HSW(m->hsync_len)
			|LCD_MIXERx_TIM0_ELW(m->left_margin)
			|LCD_MIXERx_TIM0_BLW(m->right_margin);
	}else{
		x = ((m->sync & FB_SYNC_VERT_HIGH_ACT) ? 0 : (1 << 2))
			|((m->sync & FB_SYNC_HOR_HIGH_ACT) ? 0 : (1 << 3))
			|(1 << 6)	/* VSYNC delay to remove the bottom white line */
			|(conv->invert_pixclock ? 1 : 0)
			|LCD_MIXERx_TIM0_HSW(m->hsync_len)
			|LCD_MIXERx_TIM0_ELW(m->left_margin)
			|LCD_MIXERx_TIM0_BLW(m->right_margin);
	}
	lcdc_writel(x, fbi->reg_base, LCD_MIXER0_TIM0);

	x = LCD_MIXERx_TIM1_VSW(m->vsync_len)
		|LCD_MIXERx_TIM1_EFW(m->upper_margin)
		|LCD_MIXERx_TIM1_BFW(m->lower_margin);
	lcdc_writel(x,fbi->reg_base, LCD_MIXER0_TIM1);

	/* set converter registers */
	x = LCD_CONVx_CTL_DISP_URUN_INT_EN
		|LCD_CONVx_CTL_CONV_EN
		|LCD_CONVx_CTL_TV_FOR
		|LCD_CONVx_CTL_DISP_EOF_INT_EN
		|LCD_CONVx_CTL_OP_FOR(conv->pix_fmt_out)
		|LCD_CONVx_CTL_DISP_TYPE(conv->panel_type);
	lcdc_writel(x, fbi->reg_base, LCD_CONV0_CTL);

	/* set clock */
	set_clock_divider(fbi);

	/* enable PCLK, PV2 B0 default is disabled */
	x = readl(fbi->reg_base + LCD_CTL);
	lcdc_writel(x | LCD_CTL_LCD_PCLK_EN_BO, fbi->reg_base, LCD_CTL);
	x = readl(fbi->reg_base + LCD_CTL);
}

static void converter_set_dsi(struct pxa95xfb_conv_info *conv)
{
	void *conv_base = conv->conv_base;
	u32 x, format = 0;

	/*Set Gamma correction*/
	converter_set_gamma(conv);

	/*Set DSI timings according to the frequency*/
	clk_enable(conv->clk);
	clk_set_rate(conv->clk, conv->dsi_clock_val * 1000000);
	dsi_set_time(conv, conv->dsi_clock_val);

	/*Enable the DSI converter with primary channel*/
	lcdc_writel(
		LCD_DSI_DxINEN_LP_CONT_EN
		|LCD_DSI_DxINEN_PORT_ERR_EN
		|LCD_DSI_DxINEN_TIMEOUT_EN
		|LCD_DSI_DxINEN_ACK_ERR_EN
		|LCD_DSI_DxINEN_PHY_ERR_EN
		,conv_base, LCD_DSI_DxINEN_OFFSET);

	lcdc_writel(
		LCD_DSI_DxCONV_FIFO_THRESH_PIXEL_FIFO_THRESH(400)
		|LCD_DSI_DxCONV_FIFO_THRESH_PIXEL_FIFO_SIZE(1280)
		,conv_base, LCD_DSI_DxCONV_FIFO_THRESH_OFFSET);

	lcdc_writel(
		LCD_DSI_DxCONV_FIFO_THRESH_INT_THRESH_INT_EN
		,conv_base, LCD_DSI_DxCONV_FIFO_THRESH_INT_OFFSET);

	switch (conv->pix_fmt_out){
		case PIX_FMTOUT_16_RGB565:
			format = 0;
			conv->dsi_rgb_mode = LCD_Controller_RGB_565_PACKET;
			break;
		case PIX_FMTOUT_24_RGB888:
			format = 3;
			conv->dsi_rgb_mode = LCD_Controller_RGB_888_PACKET;
			break;
		default:
			printk(KERN_ERR "%s: format %d not supported\n", __func__, conv->pix_fmt_out);
	}

	x = readl(conv_base + LCD_DSI_DxSCR1_OFFSET);
	x &= (~LCD_DSI_DxSCR1_BLANK_NULL_FRMT_MASK);
	x |= LCD_DSI_DxSCR1_BLANK_NULL_FRMT(format);/*24bpp*/
	lcdc_writel(x, conv_base, LCD_DSI_DxSCR1_OFFSET);

	lcdc_writel(DxCONV_GEN_NULL_BLANK_NULL_BLANK_DATA(0)
			|DxCONV_GEN_NULL_BLANK_GEN_PXL_FORMAT(format)
		   |DxCONV_GEN_NULL_BLANK_GEN_PXL_FORMAT2(format)
		   |DxCONV_GEN_NULL_BLANK_GEN_PXL_FORMAT3(format),
		conv_base, LCD_DSI_DxCONV_GEN_NULL_BLANK_OFFSET);

	lcdc_writel(
		LCD_DSI_DxSCR0_PRIM_VC
		|LCD_DSI_DxSCR0_FLOW_DIS
		|LCD_DSI_DxSCR0_CONV_EN
		|LCD_DSI_DxSCR0_DISP_URUN_INT_EN
		|LCD_DSI_DxSCR0_DISP_EOF_INT_EN
		,conv_base, LCD_DSI_DxSCR0_OFFSET);

	/*Send DSI commands and enable dsi*/
	if(!dsi_init(conv))
		printk(KERN_ERR "%s: dsi_init failed!!\n", __func__);

	if(conv->conf_dsi_video_mode == DSI_VIDEO_MODE)
		dsi_init_video_burst(conv);
	else if(conv->conf_dsi_video_mode == DSI_VIDEO_MODE_NON_BURST)
		dsi_init_video_non_burst(conv);
}

static void interlacer_enable(int enable)
{
	static u8 *enable_interlacer;
	if (!enable_interlacer)
		enable_interlacer =
			(u8 *)ioremap_nocache(0x4410C000, sizeof(u32));

	if (enable)
		*enable_interlacer |= 0x1;
	else
		*enable_interlacer &= 0xFE;
}

static void converter_set_hdmi(struct pxa95xfb_info *fbi)
{
	struct pxa95xfb_conv_info *conv = &pxa95xfb_conv[fbi->converter - 1];
	struct fb_videomode *m = &fbi->mode;

	void *conv_base = conv->conv_base;
	u32 x, hdmi_format, hdmi_freq;

	hdmi_format = 4; /*Todo: */

	/*Init HDMI Phy*/
	hdmi_freq = ((74170000 * HDMI_PLL_DIV_VALUE)/1000000) + 1;
	/* Set HDMI clock */
	clk_enable(conv->clk);
	if (clk_set_rate(conv->clk, hdmi_freq * 1000000)) {
		printk(KERN_ERR "HDMI PLL set failed!\n\r");
		return;
	}

	msleep(10);

	/*Set Gamma correction*/
	converter_set_gamma(conv);

	x = readl(conv_base + HDMI_MIXER_TIM0);
	if ((hdmi_format == 4)
		|| (hdmi_format == 5)
		|| (hdmi_format == 16)
		|| (hdmi_format == 19)
		|| (hdmi_format == 20))
		x &= ~(HDMI_MIXERx_TIM0_HSP|HDMI_MIXERx_TIM0_VSP);
	else
		x |= HDMI_MIXERx_TIM0_HSP|HDMI_MIXERx_TIM0_VSP;

	x |= HDMI_MIXERx_TIM0_PCP | HDMI_MIXERx_TIM0_VSYNC_DEL;
	writel(x, conv_base + HDMI_MIXER_TIM0);

	x = HDMI_MIXERx_TIM2_BLW(m->left_margin - 1)
		|HDMI_MIXERx_TIM2_ELW(m->right_margin - 1);
	writel(x, conv_base + HDMI_MIXER_TIM2);

	x = HDMI_MIXERx_TIM3_HSW(m->hsync_len - 1)
		|HDMI_MIXERx_TIM3_BFW(m->upper_margin);
	writel(x, conv_base + HDMI_MIXER_TIM3);

	x = HDMI_MIXERx_TIM4_EFW(m->lower_margin)
		|HDMI_MIXERx_TIM4_VSW(m->vsync_len - 1);
	writel(x, conv_base + HDMI_MIXER_TIM4);

	/* Enable the converter */
	x = HDMI_CONVx_CTL_WRITE_INTERLACER_REG_EN
		|HDMI_CONVx_CTL_OP_FOR(conv->pix_fmt_out)
		|HDMI_CONVx_CTL_DISP_TYPE(conv->panel_type);
	writel(x, conv_base + HDMI_CONV_CTL);

	/* Set clock dividers */
	x = HDMI_CLK_DIV_PCLK_DIV(HDMI_PLL_DIV_VALUE)
		|HDMI_CLK_DIV_CEC_REFCLK_DIV(HDMI_PLL_DIV_VALUE)
		|HDMI_CLK_DIV_PR_CLK_DIV(HDMI_PLL_DIV_VALUE);
	writel(x, conv_base + HDMI_CLK_DIV);

	if (hdmi_format == 5 || hdmi_format == 10 || hdmi_format == 11)
		interlacer_enable(1);
	else
		interlacer_enable(0);

}

static void converter_onoff(struct pxa95xfb_info *fbi, int on)
{
	struct pxa95xfb_conv_info *conv = &pxa95xfb_conv[fbi->converter - 1];
	if(conv->on == on){
		printk(KERN_INFO "converter %s is already %s\n", conv->name, conv->on?"On":"Off");
	}else if(on){
		if(conv->output == OUTPUT_HDMI) {
			set_dvfm_constraint_hdmi();
			msleep(100);
		}

		//if(conv->reset)
		//	conv->reset();

		if(CONVERTER_IS_DSI(conv->converter))
			converter_set_dsi(conv);
		else if(LCD_M2PARALELL_CONVERTER == conv->converter){
			converter_set_parallel(fbi);
			controller_enable_disable(fbi, LCD_Controller_Enable);
		} else if (LCD_M2HDMI == conv->converter) {
			converter_set_hdmi(fbi);
			controller_enable_disable(fbi, LCD_Controller_Enable);
		}
		if (CONVERTER_IS_DSI(conv->converter) && !conv->conf_dsi_video_mode){
			loop_kthread_resume(&conv->thread);
		}
		
		if(conv->power)
			conv->power(1);

		conv->on = on;
		printk(KERN_INFO "converter %s is On\n", conv->name);
	}else{
		conv->on = on;

		if (CONVERTER_IS_DSI(conv->converter) && !conv->conf_dsi_video_mode){
			loop_kthread_pause(&conv->thread);
		}

		if(conv->power)
			conv->power(0);

		if (CONVERTER_IS_DSI(conv->converter))
			dsi_enter_sleep(conv);

		if (conv->clk)
			clk_disable(conv->clk);

		/* reset it to save 1mA more */
		//if(conv->reset)
		//	conv->reset();

		if(conv->output == OUTPUT_HDMI)
			unset_dvfm_constraint_hdmi();

		printk(KERN_INFO "converter %s is Off\n", conv->name);
	}
}

static void converter_power(struct pxa95xfb_info *fbi, int on)
{
	struct pxa95xfb_conv_info *conv = &pxa95xfb_conv[fbi->converter - 1];

	if((on && conv->ref_count) || (!on) )
		converter_onoff(fbi, on);
}

#ifdef CONFIG_PXA95x_DVFM
extern void update_hss(void); /* FIXME: workaround for hss change issue*/
#else
static void update_hss(void) {}
#endif

/* only mixer0 needs this update */
static int mixer0_update;
static void wr_mixer0_update(struct pxa95xfb_conv_info *conv)
{
	u32 ctrl;
	u32 mixer_addr, mixer_sts_addr;
	if (!mixer0_update)
		return;

	pr_info("mixer0 update in eof +\n");

	mixer_addr = (u32)(pxa95xfbi[0]->reg_base) + conv->mixer * 0x100 + LCD_MIXER0_CTL0;
	mixer_sts_addr = (u32)(pxa95xfbi[0]->reg_base) + conv->mixer * 0x100 + LCD_MIXER0_INT_STS;

	/*disable MIXER*/
	ctrl = readl(mixer_addr);
	ctrl &= ~LCD_MIXERx_CTL0_MIX_EN;
	writel(ctrl, mixer_addr);
	while (!(readl(mixer_sts_addr) & LCD_MIXERx_STS0_MIX_EN_STS)) {};
	writel(LCD_MIXERx_STS0_MIX_EN_STS, mixer_sts_addr);

	mixer0_update = 0;

	/* enable MIXER*/
	ctrl = readl(mixer_addr);
	ctrl |= LCD_MIXERx_CTL0_MIX_EN;
	writel(ctrl, mixer_addr);
	while (!(readl(mixer_sts_addr) & LCD_MIXERx_STS0_MIX_EN_STS)) {};
	writel(LCD_MIXERx_STS0_MIX_EN_STS, mixer_sts_addr);

	pr_info("mixer0 update in eof -\n");

}

static void dump_regs_base(struct pxa95xfb_info *fbi);

static void wr_mixer0_disable(struct pxa95xfb_info *fbi)
{

	u32 x, stat;
	int retry;


	/* mask all interrupts */
	x = readl(fbi->reg_base + LCD_CTL);
	x &= ~(LCD_CTL_GMIX_INT_EN | LCD_CTL_GFETCH_INT_EN | LCD_CTL_GWIN_INT_EN);
	writel(x, fbi->reg_base + LCD_CTL);
 

	/* clear status */
	stat = readl(fbi->reg_base + LCD_MIXER0_INT_STS);
	writel(stat, fbi->reg_base + LCD_MIXER0_INT_STS);
 
	
	x = readl(fbi->reg_base + LCD_MIXER0_CTL0);
	x &= ~LCD_MIXERx_CTL0_MIX_EN;
	writel(x, fbi->reg_base + LCD_MIXER0_CTL0);

	retry = 50;
	stat = readl(fbi->reg_base + LCD_MIXER0_INT_STS);
	while(!(stat & LCD_MIXERx_CTL1_DISP_EN_INT_EN) && retry--) {
		msleep(1);	
		stat = readl(fbi->reg_base + LCD_MIXER0_INT_STS);
	}

	if (retry <= 0) {
		//pr_info("mixer_disable failed: mixer int status %x\n", stat);
		dump_regs_base(fbi);
		BUG_ON(1);
	} else

		pr_info("mixer_disable done, %d ms...", 50-retry);
}

static void wr_mixer0_enable(struct pxa95xfb_info *fbi)
{
	u32 x, stat;
	int retry;

	/* clear status */
	stat = readl(fbi->reg_base + LCD_MIXER0_INT_STS);
	writel(stat, fbi->reg_base + LCD_MIXER0_INT_STS);
 

	x = readl(fbi->reg_base + LCD_MIXER0_CTL0);
	x |= LCD_MIXERx_CTL0_MIX_EN;
	writel(x, fbi->reg_base + LCD_MIXER0_CTL0);

	retry = 50;
	stat = readl(fbi->reg_base + LCD_MIXER0_INT_STS);
	while(!(stat & LCD_MIXERx_CTL1_DISP_EN_INT_EN) && retry--) {
		msleep(1);
		stat = readl(fbi->reg_base + LCD_MIXER0_INT_STS);
}

	if (retry <= 0) {
		pr_info("mixer_enable failed: mixer int status %x\n", stat);
		dump_regs_base(fbi);
		BUG_ON(1);
	} else
		pr_info("mixer_enable done, %d ms...", 50-retry);

	/* unmask all interrupts */
	x = readl(fbi->reg_base + LCD_CTL);
	x |= (LCD_CTL_GMIX_INT_EN | LCD_CTL_GFETCH_INT_EN | LCD_CTL_GWIN_INT_EN);
	writel(x, fbi->reg_base + LCD_CTL);
 }


static irqreturn_t converter_handle_irq(int irq, void *dev_id)
{
	struct pxa95xfb_conv_info *conv = (struct pxa95xfb_conv_info *)dev_id;
	u32	inst0, inst1, inthresh;
	void *conv_base;

	if (conv->converter == LCD_MIXER_DISABLE)
		return IRQ_HANDLED;
	/* int sts for dpi/dsi0/dsi1 is 0x4410 1028/2028/3028, the eof bit are same 1<< 1*/
	conv_base = conv->conv_base;
	inst0 = readl(conv_base + LCD_DSI_DxINST0_OFFSET);
	writel(inst0, conv_base + LCD_DSI_DxINST0_OFFSET);
	if (inst0 & LCD_CONVx_CTL_DISP_URUN_INT_EN)
		printk(KERN_ALERT "%s: LCD underrun observed!\n", __func__);
	if (inst0 & LCD_CONVx_CTL_DISP_EOF_INT_EN){
		/*workaround here: hss change when dvfm is only permitted when converter eof*/
		if(conv->output == OUTPUT_PANEL)
			update_hss();
		/*printk(KERN_ALERT "%s: DSI Converter eof observed!\n", __func__);*/
		if (conv->mixer == 0)
			wr_mixer0_update(conv);
	}
	if (CONVERTER_IS_DSI(conv->converter)){
		/*if (inst0 & LCD_CONVx_CTL_DISP_SOF_INT_EN)
			printk(KERN_ALERT "%s: DSI Converter sof observed!\n", __func__);*/
		inst1 = readl(conv_base + LCD_DSI_DxINST1_OFFSET);
		writel(inst1, conv_base + LCD_DSI_DxINST1_OFFSET);
		/*printk(KERN_INFO "%s: INT STS0= %x, STS1= %x\n", __func__, inst0, inst1);*/

		inthresh = readl(conv_base + LCD_DSI_DxCONV_FIFO_THRESH_INT_OFFSET);
		writel(inthresh |LCD_DSI_DxCONV_FIFO_THRESH_INT_THRESH_INT_ST,
			conv_base + LCD_DSI_DxCONV_FIFO_THRESH_INT_OFFSET);
		/*printk(KERN_INFO "%s: thresh int= %x\n", __func__, inthresh);*/
	}
	return IRQ_HANDLED;
}

void converter_openclose(struct pxa95xfb_info *fbi, int open)
{
	struct pxa95xfb_conv_info *conv = &pxa95xfb_conv[fbi->converter - 1];

	if(open){
		conv->ref_count ++;
		if(!pxa95xfbi[0]->suspend && conv->ref_count == 1)
			converter_onoff(fbi,1);
		printk(KERN_INFO "converter open: %s refer count = %d", conv->name, conv->ref_count);
	}else{
		conv->ref_count --;
		if(!pxa95xfbi[0]->suspend && conv->ref_count == 0)
			converter_onoff(fbi, 0);
		printk(KERN_INFO "converter close: %s refer count = %d", conv->name, conv->ref_count);
	}
}

void converter_init(struct pxa95xfb_info *fbi)
{
	struct pxa95xfb_conv_info *conv = &pxa95xfb_conv[fbi->converter - 1];
	int ret;
	conv->conv_base = CONVERTER_BASE_ADDRESS(fbi->reg_base, conv->converter);
	conv->xres = fbi->mode.xres;
	conv->yres = fbi->mode.yres;
	conv->left_margin = fbi->mode.left_margin;
	conv->right_margin = fbi->mode.right_margin;
	conv->upper_margin = fbi->mode.upper_margin;
	conv->lower_margin = fbi->mode.lower_margin;
	conv->hsync_len = fbi->mode.hsync_len;
	conv->vsync_len = fbi->mode.vsync_len;
	conv->mixer = fbi->mixer_id;
	/* TODO : hard code here: if DSI + HDMI, it's adv chip*/
	if(CONVERTER_IS_DSI(conv->converter) && conv->output == OUTPUT_PANEL) {
		conv->dsi_clock_val = 156;
		conv->conf_dsi_video_mode = DSI_COMMAND_MODE;
		conv->dsi_init_cmds = &init_board[0][0];
		conv->dsi_sleep_cmds = &enter_sleep[0][0];
		conv->dsi_lanes = LCD_Controller_DSI_2LANE;
	} else if(CONVERTER_IS_DSI(conv->converter) && conv->output == OUTPUT_HDMI){
		conv->dsi_clock_val = (((27000000) * 8)/1000000)+1;
		conv->conf_dsi_video_mode = DSI_VIDEO_MODE_NON_BURST;
		conv->dsi_lanes = LCD_Controller_DSI_3LANE;
	}

	ret = request_irq(conv->irq, converter_handle_irq, IRQF_SHARED, conv->name, conv);
	if (ret < 0) {
		printk(KERN_INFO "unable to request IRQ\n");
	}

	if (CONVERTER_IS_DSI(conv->converter) && !conv->conf_dsi_video_mode){
		ret = loop_kthread_init(&conv->thread, "lcd_refresh_thread", (void *)conv, (void *)dsi_send_frame,28 * HZ / 1000);
		if(!ret ){
			printk(KERN_INFO "pxa95xfb: DSI update thread init failed\n");
		}
	}
}

static void dump_regs_base(struct pxa95xfb_info *fbi)
{
	printk("LCD_CTL = %x\n", readl(fbi->reg_base + LCD_CTL));
	printk("LCD_CTL_STS = %x\n", readl(fbi->reg_base + LCD_CTL_STS));
	printk("LCD_NXT_DESC_ADDR0 = %x\n", readl(fbi->reg_base + LCD_NXT_DESC_ADDR0));
	printk("LCD_FETCH_CTL0 = %x\n", readl(fbi->reg_base + LCD_FETCH_CTL0));
	printk("LCD_WIN0_CTL = %x\n", readl(fbi->reg_base + LCD_WIN0_CTL));
	printk("LCD_CH0_ALPHA = %x\n", readl(fbi->reg_base + LCD_CH0_ALPHA));
	printk("LCD_FR_ADDR0 = %x\n", readl(fbi->reg_base + LCD_FR_ADDR0));
	printk("LCD_CH0_CMD = %x\n", readl(fbi->reg_base + 0x0214));
	printk("LCD_MIXER0_CTL0 = %x\n", readl(fbi->reg_base + LCD_MIXER0_CTL0));
	printk("LCD_MIXER0_STS = %x\n", readl(fbi->reg_base + LCD_MIXER0_STS));
	printk("LCD_MIXER0_CTL1 = %x\n", readl(fbi->reg_base + LCD_MIXER0_CTL1));
	printk("LCD_MIXER0_CTL2 = %x\n", readl(fbi->reg_base + LCD_MIXER0_CTL2));
	printk("LCD_MIXER0_OL1_CFG0 = %x\n", readl(fbi->reg_base + LCD_MIXER0_OL1_CFG0));
	printk("LCD_MIXER0_OL1_CFG1 = %x\n", readl(fbi->reg_base + LCD_MIXER0_OL1_CFG1));
	printk("LCD_CONV0_CTL = 0x%x\n", readl(fbi->reg_base + LCD_CONV0_CTL));
}

static void __attribute__ ((unused)) dump_regs_ovly(struct pxa95xfb_info *fbi)
{
	if (fbi->pix_fmt == 7 || fbi->pix_fmt < 3) {
		printk("LCD_NXT_DESC_ADDR4 = %x\n", readl(fbi->reg_base + LCD_NXT_DESC_ADDR4));
		printk("LCD_FETCH_CTL4 = %x\n", readl(fbi->reg_base + LCD_FETCH_CTL4));
		printk("LCD_WIN4_CTL = %x\n", readl(fbi->reg_base + LCD_WIN4_CTL));
		printk("LCD_WINx_CFG = %x\n", readl(fbi->reg_base + LCD_WIN4_CFG));
		printk("LCD_CH4_ALPHA = %x\n", readl(fbi->reg_base + LCD_CH4_ALPHA));
	} else{
		printk("LCD_NXT_DESC_ADDR4 = %x\n", readl(fbi->reg_base + LCD_NXT_DESC_ADDR4));
		printk("LCD_NXT_DESC_ADDR5 = %x\n", readl(fbi->reg_base + LCD_NXT_DESC_ADDR5));
		printk("LCD_NXT_DESC_ADDR6 = %x\n", readl(fbi->reg_base + LCD_NXT_DESC_ADDR6));
		printk("LCD_FETCH_CTL4 = %x\n", readl(fbi->reg_base + LCD_FETCH_CTL4));
		printk("LCD_FETCH_CTL5 = %x\n", readl(fbi->reg_base + LCD_FETCH_CTL5));
		printk("LCD_FETCH_CTL6 = %x\n", readl(fbi->reg_base + LCD_FETCH_CTL6));
		printk("LCD_WIN0_CTL = %x\n", readl(fbi->reg_base + LCD_WIN0_CTL));
		printk("LCD_WIN0_CFG = %x\n", readl(fbi->reg_base + LCD_WIN0_CFG));
		printk("LCD_WIN1_CTL = %x\n", readl(fbi->reg_base + LCD_WIN1_CTL));
		printk("LCD_WIN1_CFG = %x\n", readl(fbi->reg_base + LCD_WIN1_CFG));
		printk("LCD_WIN2_CTL = %x\n", readl(fbi->reg_base + LCD_WIN2_CTL));
		printk("LCD_WIN2_CFG = %x\n", readl(fbi->reg_base + LCD_WIN2_CFG));
		printk("LCD_WIN3_CTL = %x\n", readl(fbi->reg_base + LCD_WIN3_CTL));
		printk("LCD_WIN3_CFG = %x\n", readl(fbi->reg_base + LCD_WIN3_CFG));
		printk("LCD_WIN4_CTL = %x\n", readl(fbi->reg_base + LCD_WIN4_CTL));
		printk("LCD_WIN4_CFG = %x\n", readl(fbi->reg_base + LCD_WIN4_CFG));
		printk("LCD_CH0_ALPHA = %x\n", readl(fbi->reg_base + LCD_CH0_ALPHA));
		printk("LCD_CH4_ALPHA = %x\n", readl(fbi->reg_base + LCD_CH4_ALPHA));
	}
	printk("LCD_CTL = %x\n", readl(fbi->reg_base + LCD_CTL));
	printk("LCD_CTL_STS = %x\n", readl(fbi->reg_base + LCD_CTL_STS));
	printk("LCD_MIXER1_BP_CFG0 = %x\n", readl(fbi->reg_base + LCD_MIXER1_BP_CFG0));
	printk("LCD_MIXER1_BP_CFG1 = %x\n", readl(fbi->reg_base + LCD_MIXER1_BP_CFG1));
	printk("LCD_MIXER1_OL1_CFG0 = %x\n", readl(fbi->reg_base + LCD_MIXER1_OL1_CFG0));
	printk("LCD_MIXER1_OL1_CFG1 = %x\n", readl(fbi->reg_base + LCD_MIXER1_OL1_CFG1));
	printk("LCD_MIXER1_CTL0 = %x\n", readl(fbi->reg_base + LCD_MIXER1_CTL0));
	printk("LCD_MIXER1_CTL1 = %x\n", readl(fbi->reg_base + LCD_MIXER1_CTL1));
	printk("DSI D1SCR0 = 0x%x\n", readl(fbi->reg_base + 0x3000));
	printk("DSI D1SCR1 = 0x%x\n", readl(fbi->reg_base + 0x3100));
	printk("DSI D1SSR  = 0x%x\n", readl(fbi->reg_base + 0x3104));

}

static void __attribute__ ((unused)) dump_regs_dsi(struct pxa95xfb_info *fbi)
{
	/*TBD - pass converter as an argumnet for dynamic choice*/
	void *conv_base =
		CONVERTER_BASE_ADDRESS(fbi->reg_base, LCD_M2DSI1);

	printk(KERN_INFO "LCD_DSI_D0SCR0 = %x\n",
	readl(conv_base + LCD_DSI_DxSCR0_OFFSET));
	printk(KERN_INFO "LCD_DSI_D0SCR1 = %x\n",
	readl(conv_base + LCD_DSI_DxSCR1_OFFSET));
	printk(KERN_INFO "LCD_DSI_D0SSR = %x\n",
	readl(conv_base + LCD_DSI_DxSSR_OFFSET));

	printk(KERN_INFO "LCD_DSI_DxCONV_FIFO_THRESH = %x\n",
	readl(conv_base + LCD_DSI_DxCONV_FIFO_THRESH_OFFSET));
	printk(KERN_INFO "LCD_DSI_DxCONV_FIFO_THRESH_INT = %x\n",
	readl(conv_base + 0x44));

	printk(KERN_INFO "LCD_DSI_D0TRIG = %x\n",
	readl(conv_base + LCD_DSI_DxTRIG_OFFSET));
	printk(KERN_INFO "LCD_DSI_D0INEN = %x\n",
	readl(conv_base + LCD_DSI_DxINEN_OFFSET));
	printk(KERN_INFO "LCD_DSI_D0INST1 = %x\n",
	readl(conv_base + LCD_DSI_DxINST1_OFFSET));
	printk(KERN_INFO "LCD_DSI_D0TEINTCNT = %x\n",
	readl(conv_base + LCD_DSI_DxTEINTCNT_OFFSET));
	printk(KERN_INFO "LCD_DSI_D0_G_CTL = %x\n",
	readl(conv_base + LCD_DSI_Dx_G_CTL_OFFSET));
	printk(KERN_INFO "LCD_DSI_D0_G_DAT_RED = %x\n",
	readl(conv_base + LCD_DSI_Dx_G_DAT_RED_OFFSET));
	printk(KERN_INFO "LCD_DSI_D0_G_DAT_GREEN = %x\n",
	readl(conv_base + LCD_DSI_Dx_G_DAT_GREEN_OFFSET));
	printk(KERN_INFO "LCD_DSI_D0_G_DAT_BLUE = %x\n",
	readl(conv_base + LCD_DSI_Dx_G_DAT_BLUE_OFFSET));
	printk(KERN_INFO "LCD_DSI_D0PRSR = %x\n",
	readl(conv_base + LCD_DSI_DxPRSR_OFFSET));
	printk(KERN_INFO "LCD_DSI_D0INST0 = %x\n",
	readl(conv_base + LCD_DSI_DxINST0_OFFSET));
	printk(KERN_INFO "LCD_DSI_D0ADAT = %x\n",
	readl(conv_base + LCD_DSI_DxADAT_OFFSET));
	printk(KERN_INFO "LCD_DSI_D0CFIF = %x\n",
	readl(conv_base + LCD_DSI_DxCFIF_OFFSET));
	printk(KERN_INFO "LCD_DSI_D0TIM0 = %x\n",
	readl(conv_base + LCD_DSI_DxTIM0_OFFSET));
	printk(KERN_INFO "LCD_DSI_D0TIM1 = %x\n",
	readl(conv_base + LCD_DSI_DxTIM1_OFFSET));
	printk(KERN_INFO "LCD_DSI_D0PHY_TIM0 = %x\n",
	readl(conv_base + LCD_DSI_DxPHY_TIM0_OFFSET));
	printk(KERN_INFO "LCD_DSI_D0PHY_TIM1 = %x\n",
	readl(conv_base + LCD_DSI_DxPHY_TIM1_OFFSET));
	printk(KERN_INFO "LCD_DSI_D0PHY_TIM2 = %x\n",
	readl(conv_base + LCD_DSI_DxPHY_TIM2_OFFSET));
}

static void controller_enable_disable(struct pxa95xfb_info *fbi, int onoff)
{
	int ctrl, stat, retry=50;

	switch(onoff)
	{
		case LCD_Controller_Enable:
			display_enabled = 1;
			ctrl = readl(fbi->reg_base + LCD_CTL);
			if (!(ctrl & LCD_CTL_AXI32_EN))
				printk(KERN_ERR "%s: AXI32_EN is zero which should not happen!!!\n", __func__);
			ctrl |= LCD_CTL_LCD_EN | LCD_CTL_GMIX_INT_EN | LCD_CTL_AXI32_EN | LCD_CTL_GFETCH_INT_EN | LCD_CTL_GWIN_INT_EN;
			/*make sure quick disable is cleared*/
			ctrl &= ~LCD_CTL_LCD_QD;
			lcdc_writel(ctrl, fbi->reg_base, LCD_CTL);

			stat = readl(fbi->reg_base + LCD_CTL_INT_STS);
			while (!(stat & LCD_CTL_INT_STS_LCD_EN_INT_STS) && retry-- ) {
				stat = readl(fbi->reg_base + LCD_CTL_INT_STS);
				msleep(1);
			}
			if(retry <= 0)
				printk(KERN_ERR "%s: lcd en sts not set when enable!!!\n", __func__);
			printk("enabling 1: lcd init stst %x, lcd ctl %08x, lcd mixer status %x\n",
				readl(fbi->reg_base + LCD_CTL_INT_STS),
				readl(fbi->reg_base + LCD_CTL),
				readl(fbi->reg_base + fbi->mixer_id*0x100 + LCD_MIXER0_INT_STS));
			lcdc_writel(LCD_CTL_INT_STS_LCD_EN_INT_STS, fbi->reg_base, LCD_CTL_INT_STS);
			break;
		case LCD_Controller_Disable:
			ctrl = readl(fbi->reg_base + LCD_CTL);
			ctrl &= ~(LCD_CTL_LCD_EN);
			lcdc_writel(ctrl, fbi->reg_base, LCD_CTL);
			/* after disable, the LCD_CTL_INT_STS[LCD_CTL_INT_STS_LCD_DIS_INT_STS]
			 * is not cleared
			 * -- now we use quick disable when display off according to BBU team
			 */
			stat = readl(fbi->reg_base + LCD_CTL_INT_STS);
			while ((stat & LCD_CTL_INT_STS_LCD_DIS_INT_STS) && retry--) {
				stat = readl(fbi->reg_base + LCD_CTL_INT_STS);
				msleep(1);
			}
			if(retry <= 0)
				printk(KERN_ERR "%s: lcd en sts not cleared when disable!!!\n", __func__);
			lcdc_writel(LCD_CTL_INT_STS_LCD_DIS_INT_STS, fbi->reg_base, LCD_CTL_INT_STS);
			display_enabled = 0;
			break;
		case LCD_Controller_Quick_Disable:
			ctrl = readl(fbi->reg_base + LCD_CTL);
			ctrl |= LCD_CTL_LCD_QD;
			lcdc_writel(ctrl, fbi->reg_base, LCD_CTL);
			/* move to LCD_CTL_INT_STS reg to check*/
			stat = readl(fbi->reg_base + LCD_CTL_INT_STS);
			while (!(stat & LCD_CTL_INT_STS_LCD_QD_INT_STS) && retry--){
				stat = readl(fbi->reg_base + LCD_CTL_INT_STS);
				msleep(1);
			}
			if(retry <= 0)
				printk(KERN_ERR "%s: lcd qd sts not set when quick disable!!!\n", __func__);

			printk("quick disabling 1: lcd init stst %x, lcd ctl %08x, lcd mixer status %x\n",
				readl(fbi->reg_base + LCD_CTL_INT_STS),
				readl(fbi->reg_base + LCD_CTL),
				readl(fbi->reg_base + fbi->mixer_id*0x100 + LCD_MIXER0_INT_STS));
			lcdc_writel(LCD_CTL_INT_STS_LCD_QD_INT_STS, fbi->reg_base, LCD_CTL_INT_STS);
			msleep(20);

			ctrl = readl(fbi->reg_base + LCD_CTL);
			ctrl &= ~(LCD_CTL_LCD_QD | LCD_CTL_LCD_EN);
			lcdc_writel(ctrl, fbi->reg_base, LCD_CTL);
			msleep(50);
			printk("quick disabling 2: lcd init stst %x, lcd ctl %08x, lcd mixer status %x\n",
				readl(fbi->reg_base + LCD_CTL_INT_STS),
				readl(fbi->reg_base + LCD_CTL),
				readl(fbi->reg_base + fbi->mixer_id*0x100 + LCD_MIXER0_INT_STS));
			lcdc_writel(LCD_CTL_INT_STS_LCD_DIS_INT_STS, fbi->reg_base, LCD_CTL_INT_STS);
			msleep(20);

			display_enabled = 0;
			break;
		default:
			printk(KERN_ERR "%s: invalid parameter\n", __func__);
			break;
	}
}

static int determine_best_pix_fmt(struct fb_var_screeninfo *var)
{
	/*
	 * Pseudocolor mode?
	 */
	if (var->bits_per_pixel == 8)
		return PIX_FMT_PSEUDOCOLOR;

	/*
	 * Check for YUV422PLANAR.
	 */
	if (var->bits_per_pixel == 16 && var->red.length == 8 &&
			var->green.length == 4 && var->blue.length == 4) {
		if (var->red.offset >= var->blue.offset)
			return PIX_FMTIN_YUV422;
		else
			printk("%s: YVU422PLANAR format is not supported!\n", __func__);
	}

	/*
	 * Check for YUV420PLANAR.
	 */
	if (var->bits_per_pixel == 12 && var->red.length == 8 &&
			var->green.length == 2 && var->blue.length == 2) {
		if (var->red.offset >= var->blue.offset)
			return PIX_FMTIN_YUV420;
		else
			printk("%s: YVU420PLANAR format is not supported!\n", __func__);
	}

	/*
	 * Check for YUV444PLANAR.
	 */
	if (var->bits_per_pixel == 24 && var->red.length == 16 &&
			var->green.length == 16 && var->blue.length == 16) {
		if (var->red.offset >= var->blue.offset)
			return PIX_FMTIN_YUV444;
		else
			printk("%s: YVU420PLANAR format is not supported!\n", __func__);
	}
	/*
	 * Check for YUV422PACK.
	 */
	if (var->bits_per_pixel == 16 && var->red.length == 16 &&
			var->green.length == 16 && var->blue.length == 16) {
		if ((var->red.offset >= var->blue.offset) && (var->red.offset == 8))
			return PIX_FMTIN_YUV422IL;
		else
			printk("%s: YVU422PACK or UYVY format is not supported!\n", __func__);
	}

	/*
	 * Check for 565/1555.
	 */
	if (var->bits_per_pixel == 16 && var->red.length <= 5 &&
			var->green.length <= 6 && var->blue.length <= 5) {
		if (var->transp.length == 0) {
			if (var->red.offset >= var->blue.offset)
				return PIX_FMTIN_RGB_16;
			else
				printk("%s: BGR565 format is not supported!\n", __func__);
		}
	}

	/*
	 * Check for 888/A888.
	 */
	if (var->bits_per_pixel <= 32 && var->red.length <= 8 &&
			var->green.length <= 8 && var->blue.length <= 8) {
		if (var->bits_per_pixel == 24 && var->transp.length == 0) {
			if (var->red.offset >= var->blue.offset)
				return PIX_FMTIN_RGB_24_PACK;
			else
				printk("%s: BGR888 packed format is not supported!\n", __func__);
		}

		if (var->bits_per_pixel == 32 && var->transp.length == 8) {
			if (var->red.offset >= var->blue.offset)
				return PIX_FMTIN_RGB_32;
			else
				printk("%s: BGRA888 format is not supported!\n", __func__);
		} else {
			if (var->red.offset >= var->blue.offset)
				return PIX_FMTIN_RGB_24;
			else
				printk("%s: BGR888 format is not supported!\n", __func__);;
		}

		/* fall through */
	}

	return -EINVAL;


}


static void set_fetch(struct pxa95xfb_info *fbi)
{
	u32 x;
	struct fb_videomode * m = &fbi->mode;
	/* set DMA descriptor register
	 *  NOT set LCD_FR_ADDRx, set it in set_graphics_start function
	 */
	DisplayControllerFrameDescriptor *dmadesc_cpu;
	u32 dmadesc_dma;
	int start_channel, end_channel, channel;
	/* for normal format, set channel = window, for YUV, set 4,5,6 channels*/
	start_channel = fbi->window;
	end_channel = (fbi->pix_fmt >= PIX_FMTIN_YUV420 && fbi->pix_fmt <= PIX_FMTIN_YUV444 && start_channel == 4)?
		6 : start_channel;

	for(channel = start_channel; channel <= end_channel; channel ++){
		dmadesc_cpu = (DisplayControllerFrameDescriptor *)((u32)fbi->fb_start- 16*(channel+1));
		dmadesc_dma = fbi->fb_start_dma - 16*(channel+1);
		dmadesc_cpu->LCD_FR_IDx = 0;
		/*FIXME: for yuv, this value might be not correct*/
		dmadesc_cpu->LCD_CH_CMDx = (fbi->user_addr)?
			fbi->surface.viewPortInfo.srcHeight * fbi->surface.viewPortInfo.ycPitch:
			m->yres * m->xres * fbi->bpp /8;
		dmadesc_cpu->LCD_NXT_DESC_ADDRx = dmadesc_dma;
		lcdc_writel(dmadesc_dma, fbi->reg_base, LCD_NXT_DESC_ADDR0 + channel * 0x40);

		/* set format */
		x = readl(fbi->reg_base + LCD_FETCH_CTL0 + fbi->window * 0x40);
		x &= ~(LCD_FETCH_CTLx_SRC_FOR(0x7));
		x |= LCD_FETCH_CTLx_CHAN_EN|LCD_FETCH_CTLx_SRC_FOR(fbi->pix_fmt) |LCD_FETCH_CTLx_BUS_ERR_INT_EN;
		if(fbi->eof_intr_en && channel == start_channel)
			x |= LCD_FETCH_CTLx_END_FR_INT_EN;
		lcdc_writel(x, fbi->reg_base, LCD_FETCH_CTL0 + channel * 0x40);
	}

}

static void set_window(struct pxa95xfb_info *fbi)
{
	u32 x;
	struct fb_videomode * m = &fbi->mode;
	u32 xsrc, ysrc, xdst, ydst, xscale, yscale;
	u32 rcrop;
	u32 ctl_offset = (fbi->window == 4)? LCD_WIN4_CTL: LCD_WIN0_CTL + fbi->window * 0x40;
	u32 crop1_offset = (fbi->window == 4) ? LCD_WIN4_CROP1:
		LCD_WIN0_CROP1 + fbi->window * 0x40;

	xsrc = (fbi->surface.viewPortInfo.ycPitch)?
		fbi->surface.viewPortInfo.ycPitch * 8 / fbi->bpp: m->xres;
	ysrc = (fbi->surface.viewPortInfo.srcHeight)?
		fbi->surface.viewPortInfo.srcHeight : m->yres;

	/* set input image resolution*/
	x = LCD_WINx_CTL_WIN_XRES(xsrc/4)|LCD_WINx_CTL_WIN_YRES(ysrc/4);
	x |= LCD_WINx_CTL_WIN_URUN_INT_EN;
	lcdc_writel(x, fbi->reg_base, ctl_offset);

	/* set cropping */
	rcrop = (fbi->surface.viewPortInfo.ycPitch * 8 / fbi->bpp -
			fbi->surface.viewPortInfo.srcWidth);
	x = LCD_WINx_CROP1_RC(rcrop/4);
	lcdc_writel(x, fbi->reg_base, crop1_offset);

	/*scale is supported when wge = 4, and in fb0, surface is always not set*/
	if(4 != fbi->window)
		return;

	xdst = (fbi->surface.viewPortInfo.zoomXSize)?
		fbi->surface.viewPortInfo.zoomXSize : m->xres;
	ydst = (fbi->surface.viewPortInfo.zoomYSize)?
		fbi->surface.viewPortInfo.zoomYSize : m->yres;

	x = readl(fbi->reg_base + LCD_WIN0_CFG + fbi->window * 0x40);
	if(xdst < xsrc){
		x &= ~(1<<24);
		xscale = (100000 - xdst*100000/(xsrc))/3125 - 1;
	}else{
		x |= (1<<24);
		xscale = (xdst*1000/(xsrc) - 1000)/125;
	}

	if(ydst < ysrc){
		x &= ~(1<<18);
		yscale = (100000 - ydst*100000/(ysrc))/3125 - 1;
	}else{
		x |= (1<<18);
		yscale = (ydst*1000/(ysrc) - 1000)/125;
	}
	x &= ~(LCD_WINx_CFG_WIN_XSCALE(0x1f)|LCD_WINx_CFG_WIN_YSCALE(0x1f));
	x |= LCD_WINx_CFG_WIN_XSCALE(xscale)|LCD_WINx_CFG_WIN_YSCALE(yscale);
	lcdc_writel(x, fbi->reg_base, LCD_WIN4_CFG);
}

static void set_mixer(struct pxa95xfb_info *fbi)
{
	u32 x, ctl0_wge_id;
	u32 update, en;
	struct fb_videomode * m = &fbi->mode;

	u32 ctl0_off = LCD_MIXER0_CTL0 + fbi->mixer_id* 0x100;
	u32 ctl1_off = LCD_MIXER0_CTL1 + fbi->mixer_id* 0x100;
	u32 ctl2_off = LCD_MIXER0_CTL2 + fbi->mixer_id* 0x100;
	u32 cfg0_off = LCD_MIXER0_BP_CFG0 + fbi->mixer_id* 0x100 + fbi->zorder* 0x8;
	u32 cfg1_off = LCD_MIXER0_BP_CFG1 + fbi->mixer_id* 0x100 + fbi->zorder* 0x8;
	u32 converter = fbi->converter;

	u32 xsize, ysize, xpos, ypos;
	xsize = (fbi->surface.viewPortInfo.zoomXSize)?
		fbi->surface.viewPortInfo.zoomXSize : m->xres;
	ysize = (fbi->surface.viewPortInfo.zoomYSize)?
		fbi->surface.viewPortInfo.zoomYSize : m->yres;
	/* in fb0, surface is always set to 0*/
	xpos = fbi->surface.viewPortOffset.xOffset;
	ypos = fbi->surface.viewPortOffset.yOffset;

	if (converter == LCD_MIXER_DISABLE)
		return;

	x = readl(fbi->reg_base + cfg0_off);
	x &= ~(LCD_MIXERx_CFG0_XPOS(0x7ff) | LCD_MIXERx_CFG0_YPOS(0x7ff));
	x |= LCD_MIXERx_CFG0_XPOS(xpos)
		| LCD_MIXERx_CFG0_YPOS(ypos);
	lcdc_writel(x, fbi->reg_base, cfg0_off);

	x = readl(fbi->reg_base + cfg1_off);
	x &= ~(LCD_MIXERx_CFG1_XRES(0x7ff) | LCD_MIXERx_CFG1_YRES(0x7ff));
	x |= LCD_MIXERx_CFG1_XRES(xsize)| LCD_MIXERx_CFG1_YRES(ysize);
	lcdc_writel(x, fbi->reg_base, cfg1_off);

	x = readl(fbi->reg_base + ctl1_off);
	x &= ~(LCD_MIXERx_CTL1_DISP_XRES(0x7ff) | LCD_MIXERx_CTL1_DISP_YRES(0x7ff));
	x |= LCD_MIXERx_CTL1_DISP_XRES(m->xres)| LCD_MIXERx_CTL1_DISP_YRES(m->yres)
		| LCD_MIXERx_CTL1_DISP_UPDATE_INT_EN;
	lcdc_writel(x, fbi->reg_base,ctl1_off);

	x = LCD_MIXERx_CTL2_CONV_ID(LCD_Controller_P_CONVERTER0 - LCD_M2PARALELL_CONVERTER
		+ converter);
	lcdc_writel(x, fbi->reg_base,ctl2_off);

	/* LCD_MIXERx_CTL0_DISP_UPDATE should be set as the last step */
	x = readl(fbi->reg_base + ctl0_off);
	ctl0_wge_id = (fbi->on)?
		LCD_MIXERx_CTL0_OLx_ID(fbi->zorder, LCD_Controller_WGe0 + fbi->window):
		LCD_MIXERx_CTL0_OLx_ID_MASK(fbi->zorder);
	x &= ~ LCD_MIXERx_CTL0_OLx_ID_MASK(fbi->zorder);
	x |= ctl0_wge_id;
	en = ((x & LCD_MIXERx_CTL0_ALL_OL_MASK) != LCD_MIXERx_CTL0_ALL_OL_DISABLE)? LCD_MIXERx_CTL0_MIX_EN: 0;
	/* workaround: for mixer 0, always not update here */
	if (fbi->mixer_id == 0) {
		/* if mixer not enabled, no need to update */
		mixer0_update = en && display_enabled && (fbi->on || fbi->active);
		if (mixer0_update)
			pr_info("lcd mixer update set!\n");
		x &= ~LCD_MIXERx_CTL0_MIX_EN;
		x |= en;
	} else {
		update = (display_enabled && (fbi->on || fbi->active))? LCD_MIXERx_CTL0_DISP_UPDATE: 0;
		x &= ~LCD_MIXERx_CTL0_MIX_EN;
		x |= (en)?(en | update): 0;
	}

	lcdc_writel(x, fbi->reg_base, ctl0_off);

	fbi->active = fbi->on;
}

static void set_scale(struct pxa95xfb_info *fbi)
{
	u32 scale_lut[] = {0x00004000, 0xff0940f8, 0xfe0f40f3, 0xfc163ef0, 0xfa1c3cee,
		0xf7223aed, 0xf42837ed, 0xf22c34ee, 0xf03030f0, 0xee342cf2, 0xed3728f4,
		0xed3a22f7, 0xee3c1cfa, 0xf03e16fc, 0xf3400ffe, 0xf84009ff};
	int i;
	for (i = 0; i < 16; i++){
		lcdc_writel(i, fbi->reg_base, LCD_WIN4_SCALE_PTR);
		lcdc_writel(scale_lut[i], fbi->reg_base, LCD_WIN4_SCALE_RW);
	}
}

static ssize_t vsync_show(struct device *dev, struct device_attribute *attr,
		char *buf)
{
	struct pxa95xfb_info *fbi = dev_get_drvdata(dev);

	return sprintf(buf, "fbi %d wait vsync: %d\n", fbi->id, fbi->vsync_en);
}

static ssize_t vsync_store(
		struct device *dev, struct device_attribute *attr,
		const char *buf, size_t size)
{
	struct pxa95xfb_info *fbi = dev_get_drvdata(dev);

	sscanf(buf, "%d", &fbi->vsync_en);

	return size;
}

static DEVICE_ATTR(vsync, S_IRUGO | S_IWUSR, vsync_show, vsync_store);


/*
 * The hardware clock divider has an integer and a fractional
 * stage:
 *
 *	clk2 = clk_in / integer_divider
 *	clk_out = clk2 * (1 - (fractional_divider >> 12))
 *
 * Calculate integer and fractional divider for given clk_in
 * and clk_out.
 */
#define DEFAULT_REFRESH		60	/* Hz */

void lcdc_correct_pixclock(struct fb_videomode * m)
{
	u64 div_result;
	u32 total_w, total_h, refresh;

	refresh = (m->refresh)? m->refresh: DEFAULT_REFRESH;
	total_w = m->xres + m->left_margin + m->right_margin + m->hsync_len;
	total_h = m->yres + m->upper_margin + m->lower_margin + m->vsync_len;
	div_result = 1000000000000ll;
	do_div(div_result, total_w * total_h * refresh);
	m->pixclock = div_result;

	printk(KERN_INFO "LCD %s: pixclock %d\n", __func__, m->pixclock);
}

static inline int check_colorkeyalpha_changed(struct _sColorKeyNAlpha *a, struct _sColorKeyNAlpha *b)
{
	return (a->alpha_method != b->alpha_method
		|| a->color_match != b->color_match
		|| a->default_alpha_val!= b->default_alpha_val
		|| a->win_alpha_en != b->win_alpha_en);
}


u32 lcdc_set_colorkeyalpha(struct pxa95xfb_info *fbi)
{
	unsigned int x = 0;
	static struct _sColorKeyNAlpha color_a;

	if(!check_colorkeyalpha_changed(&color_a,  &fbi->ckey_alpha))
		return 0;
	memcpy(&color_a, &fbi->ckey_alpha, sizeof(struct _sColorKeyNAlpha));

	dev_dbg(fbi->fb_info->dev, "Enter %s\n", __FUNCTION__);

	if (color_a.win_alpha_en)
		/* YCbCr formate can only select window specific alpha */
		x = LCD_CHx_ALPHA_W_ALPHA_EN | LCD_CHx_ALPHA_CH_ALPHA(color_a.default_alpha_val);
	else {
		/*
		 * color_a->alpha_method = 0: color match alpha method (only for RGB16 and RGB24)
		 * color_a->alpha_method = 1: lower bit alpha method (only for RGB16 and RGB24)
		 * color_a->alpha_method = 2: source alpha method (only for RGB32)
		 */
		if ((LCD_CHx_ALPHA_CLR_KEY_EN(color_a.alpha_method)) == 0) {
			x = LCD_CHx_ALPHA_CH_ALPHA(color_a.default_alpha_val);
			lcdc_writel(color_a.color_match, fbi->reg_base, LCD_CH0_CLR_MATCH  + 4*fbi->window);
		} else
			x = LCD_CHx_ALPHA_CLR_KEY_EN(color_a.alpha_method);
	}
	lcdc_writel(x, fbi->reg_base, LCD_CH0_ALPHA  + 4*fbi->window);

	return 0;
}

void lcdc_set_pix_fmt(struct fb_var_screeninfo *var, int pix_fmt)
{
	switch (pix_fmt) {
		case PIX_FMTIN_RGB_16:
			var->bits_per_pixel = 16;
			var->red.offset = 11;	var->red.length = 5;
			var->green.offset = 5;   var->green.length = 6;
			var->blue.offset = 0;	var->blue.length = 5;
			var->transp.offset = 0;  var->transp.length = 0;
			break;
		case PIX_FMTIN_RGB_24:
			var->bits_per_pixel = 32;
			var->red.offset = 16;	var->red.length = 8;
			var->green.offset = 8;   var->green.length = 8;
			var->blue.offset = 0;	var->blue.length = 8;
			var->transp.offset = 0;  var->transp.length = 0;
			break;
		case PIX_FMTIN_RGB_32:
			var->bits_per_pixel = 32;
			var->red.offset = 16;	var->red.length = 8;
			var->green.offset = 8;   var->green.length = 8;
			var->blue.offset = 0;	var->blue.length = 8;
			var->transp.offset = 24; var->transp.length = 8;
			break;
		case PIX_FMTIN_RGB_24_PACK:
			var->bits_per_pixel = 24;
			var->red.offset = 16;	var->red.length = 8;
			var->green.offset = 8;   var->green.length = 8;
			var->blue.offset = 0;	var->blue.length = 8;
			var->transp.offset = 0;  var->transp.length = 0;
			break;
		case PIX_FMTIN_YUV420:
			var->bits_per_pixel = 12;
			var->red.offset = 4;	 var->red.length = 8;
			var->green.offset = 2;   var->green.length = 2;
			var->blue.offset = 0;   var->blue.length = 2;
			var->transp.offset = 0;  var->transp.length = 0;
			break;
		case PIX_FMTIN_YUV422:
			var->bits_per_pixel = 16;
			var->red.offset = 8;	 var->red.length = 8;
			var->green.offset = 4;   var->green.length = 4;
			var->blue.offset = 0;   var->blue.length = 4;
			var->transp.offset = 0;  var->transp.length = 0;
			break;
		case PIX_FMTIN_YUV444:
			var->bits_per_pixel = 24;
			var->red.offset = 16;	  var->red.length = 16;
			var->green.offset = 8;	 var->green.length = 16;
			var->blue.offset = 0;	var->blue.length = 16;
			var->transp.offset = 0;  var->transp.length = 0;
			break;
		case PIX_FMTIN_YUV422IL:
			var->bits_per_pixel = 16;
			var->red.offset = 8;	 var->red.length = 16;
			var->green.offset = 4;   var->green.length = 16;
			var->blue.offset = 0;   var->blue.length = 16;
			var->transp.offset = 0;  var->transp.length = 0;
			break;
		case PIX_FMT_PSEUDOCOLOR:
			var->bits_per_pixel = 8;
			var->red.offset = 0;	 var->red.length = 8;
			var->green.offset = 0;   var->green.length = 8;
			var->blue.offset = 0;	var->blue.length = 8;
			var->transp.offset = 0;  var->transp.length = 0;
			break;
	}
}

void lcdc_set_mode_to_var(struct pxa95xfb_info *fbi, struct fb_var_screeninfo *var,
			 const struct fb_videomode *mode)
{
	dev_dbg(fbi->fb_info->dev, "Enter %s\n", __FUNCTION__);

	var->xres = mode->xres;
	var->yres = mode->yres;
	var->xres_virtual = var->xres;
	var->yres_virtual = (var->xres * var->yres * 4 > fbi->fb_size)? (var->yres): (var->yres*2);
	var->grayscale = 0;
	var->accel_flags = FB_ACCEL_NONE;
	var->pixclock = mode->pixclock;
	var->left_margin = mode->left_margin;
	var->right_margin = mode->right_margin;
	var->upper_margin = mode->upper_margin;
	var->lower_margin = mode->lower_margin;
	var->hsync_len = mode->hsync_len;
	var->vsync_len = mode->vsync_len;
	var->sync = mode->sync;
	var->vmode = mode->vmode;
	var->rotate = FB_ROTATE_UR;
}

u32 lcdc_set_fr_addr(struct pxa95xfb_info *fbi, struct fb_var_screeninfo * var)
{
	DisplayControllerFrameDescriptor *dmadesc_cpu;
	u32 addr = fbi->user_addr ? fbi->user_addr: fbi->fb_start_dma;
	/* assert user pointer uses no var->x/y offset*/
	u32 pixel_offset = fbi->user_addr ? 0:
		(var->yoffset * var->xres_virtual + var->xoffset);

	if(fbi->pix_fmt >= PIX_FMTIN_YUV420 && fbi->pix_fmt <= PIX_FMTIN_YUV444){
		/* set three channels: 456 for plannar YUV channels: assert fbi->window = 4*/
		/* y size is for YUV plannar only: when yres_virtual  > yres, still use yres to make sure YUV is continous*/
		u32 y_size = fbi->user_addr? (fbi->surface.viewPortInfo.ycPitch * 8 / fbi->bpp
				* fbi->surface.viewPortInfo.srcHeight) : (var->xres * var->yres);
		dmadesc_cpu = (DisplayControllerFrameDescriptor *)((u32)fbi->fb_start - 16*5);
		dmadesc_cpu->LCD_FR_ADDRx = addr + pixel_offset;

		if (fbi->pix_fmt == PIX_FMTIN_YUV420){
			dmadesc_cpu = (DisplayControllerFrameDescriptor *)((u32)fbi->fb_start - 16*6);
			dmadesc_cpu->LCD_FR_ADDRx = addr + y_size + pixel_offset /4;
			dmadesc_cpu = (DisplayControllerFrameDescriptor *)((u32)fbi->fb_start - 16*7);
			dmadesc_cpu->LCD_FR_ADDRx = addr + y_size * 5/4 + pixel_offset /4 ;
		}
		if (fbi->pix_fmt == PIX_FMTIN_YUV422){
			dmadesc_cpu = (DisplayControllerFrameDescriptor *)((u32)fbi->fb_start - 16*6);
			dmadesc_cpu->LCD_FR_ADDRx = addr + y_size + pixel_offset /2;
			dmadesc_cpu = (DisplayControllerFrameDescriptor *)((u32)fbi->fb_start - 16*7);
			dmadesc_cpu->LCD_FR_ADDRx = addr + y_size * 3/2 + pixel_offset /2 ;
		}
		if (fbi->pix_fmt == PIX_FMTIN_YUV444){
			dmadesc_cpu = (DisplayControllerFrameDescriptor *)((u32)fbi->fb_start - 16*6);
			dmadesc_cpu->LCD_FR_ADDRx = addr + y_size + pixel_offset;
			dmadesc_cpu = (DisplayControllerFrameDescriptor *)((u32)fbi->fb_start - 16*7);
			dmadesc_cpu->LCD_FR_ADDRx = addr + y_size * 2 + pixel_offset;
		}
	}else{
		dmadesc_cpu = (DisplayControllerFrameDescriptor *)((u32)fbi->fb_start- 16*(fbi->window + 1));
		dmadesc_cpu->LCD_FR_ADDRx = addr + pixel_offset* var->bits_per_pixel /8;
	}
	dmadesc_cpu = (DisplayControllerFrameDescriptor *)((u32)fbi->fb_start- 16*(fbi->window + 1));
	return dmadesc_cpu->LCD_FR_ADDRx;
}

u32 lcdc_get_fr_addr(struct pxa95xfb_info *fbi)
{
	if(pxa95xfbi[0]->suspend)
		return 0;

	/*according to spec, fr_adr is 4-31 bit, but actually it's 0-31bit */
	return (readl(fbi->reg_base + LCD_FR_ADDR0 + 0x40*fbi->window));
}


void lcdc_set_lcd_controller(struct pxa95xfb_info *fbi)
{
	/* set fetch registers */
	set_fetch(fbi);
	/* set window registers */
	set_window(fbi);

	/* set mixer registers */
	set_mixer(fbi);
}

int lcdc_wait_for_vsync(struct pxa95xfb_info *fbi)
{
	u32 t, ret = 0;
	static u32 c = 0, s = 0;

	if (fbi && fbi->vsync_en) {
		t = OSCR;
		atomic_set(&fbi->w_intr, 0);
		ret = wait_event_interruptible_timeout(fbi->w_intr_wq,
				atomic_read(&fbi->w_intr), 60 * HZ / 1000);
		t = (OSCR - t) *4/ 13000;
		c++;
		s += t;
		/*if(c %100 == 0)
			printk(KERN_INFO "%s: %d times, average = %dms\n", __func__, c, s/c);*/
		/*printk(KERN_INFO "%s: ch %d time %d ms\n", __func__, fbi->window, t);*/
		if(!ret)
			printk(KERN_WARNING "warning %s:failed\n",__func__);
	}
	return ret;
}

int pxa95xfb_check_var(struct fb_var_screeninfo *var,
		struct fb_info *info)
{
	struct pxa95xfb_info *fbi = info->par;

	if (var->bits_per_pixel == 8)
		return -EINVAL;
	/*
	 * Basic geometry sanity checks.
	 */
	if (var->xoffset + var->xres > var->xres_virtual)
		return -EINVAL;
	if (var->yoffset + var->yres > var->yres_virtual)
		return -EINVAL;
	if (var->xres + var->right_margin +
			var->hsync_len + var->left_margin > 2048)
		return -EINVAL;
	if (var->yres + var->lower_margin +
			var->vsync_len + var->upper_margin > 2048)
		return -EINVAL;

	/*
	 * Check size of framebuffer.
	 */
	if (var->xres_virtual * var->yres_virtual *
			(var->bits_per_pixel >> 3) > fbi->fb_size)
		return -EINVAL;

	return 0;
}

static unsigned int chan_to_field(unsigned int chan, struct fb_bitfield *bf)
{
	return ((chan & 0xffff) >> (16 - bf->length)) << bf->offset;
}

static u32 to_rgb(u16 red, u16 green, u16 blue)
{
	red >>= 8;
	green >>= 8;
	blue >>= 8;

	return (red << 16) | (green << 8) | blue;
}

int pxa95xfb_setcolreg(unsigned int regno, unsigned int red, unsigned int green,
		unsigned int blue, unsigned int trans, struct fb_info *info)
{
	struct pxa95xfb_info *fbi = info->par;
	u32 val;

	if (info->fix.visual == FB_VISUAL_TRUECOLOR && regno < 16) {
		val =  chan_to_field(red,   &info->var.red);
		val |= chan_to_field(green, &info->var.green);
		val |= chan_to_field(blue , &info->var.blue);
		fbi->pseudo_palette[regno] = val;
	}

	if (info->fix.visual == FB_VISUAL_PSEUDOCOLOR && regno < 256) {
		val = to_rgb(red, green, blue);
		/* TODO */
	}

	return 0;
}

int pxa95xfb_pan_display(struct fb_var_screeninfo *var,
		struct fb_info *info)
{
	struct pxa95xfb_info *fbi = (struct pxa95xfb_info *)info->par;

	lcdc_set_fr_addr(fbi, var);

	lcdc_wait_for_vsync(fbi);

	return 0;
}

int pxa95xfb_set_par(struct fb_info *info)
{
	struct pxa95xfb_info *fbi = info->par;
	struct fb_var_screeninfo *var = &info->var;
	struct fb_videomode * m;
	int pix_fmt;

	/* set pix_fmt */
	pix_fmt = determine_best_pix_fmt(var);
	if (pix_fmt < 0)
		return -EINVAL;
	lcdc_set_pix_fmt(var, pix_fmt);
	fbi->pix_fmt = pix_fmt;
	fbi->bpp = var->bits_per_pixel;

	/* set var according to best video mode*/
	m = (struct fb_videomode *)fb_match_mode(var, &info->modelist);
	if(!m){
		printk(KERN_WARNING "set par: no match mode, turn to best mode\n");
		m = (struct fb_videomode *)fb_find_best_mode(var,&info->modelist);
	}
	lcdc_set_mode_to_var(fbi, var, m);
	memcpy(&fbi->mode, m, sizeof(struct fb_videomode));

	info->fix.visual = (pix_fmt == PIX_FMT_PSEUDOCOLOR)?
		FB_VISUAL_PSEUDOCOLOR: FB_VISUAL_TRUECOLOR;
	info->fix.line_length = var->xres_virtual * var->bits_per_pixel / 8;
	info->fix.ypanstep = var->yres;


	/* configure graphics dma always*/
	lcdc_set_fr_addr(fbi, var);

	if(!pxa95xfbi[0]->suspend)
		lcdc_set_lcd_controller(fbi);

	return 0;
}

static int pxa95xfb_gfx_ioctl(struct fb_info *fi, unsigned int cmd,
		unsigned long arg)
{
	void __user *argp = (void __user *)arg;
	struct pxa95xfb_info *fbi = (struct pxa95xfb_info *)fi->par;
	int on;

	/*This API will work on both Mixers, if enabled*/
	switch (cmd) {
		case FB_IOCTL_SWITCH_GRA_OVLY:
			if (copy_from_user(&on, argp, sizeof(int))) {
				return -EFAULT;
			}
			if(on == fbi->on){
				printk(KERN_INFO "PXA95xfb: graphics already: %s\n", fbi->on?"on":"off");
			}else{
				fbi->on = on;
				printk(KERN_INFO "PXA95xfb: graphics switch: %s\n", fbi->on?"on":"off");
				converter_openclose(fbi, on);
				if(!fbi->suspend)
					lcdc_set_lcd_controller(fbi);
			}
			break;
		case FB_IOCTL_SET_COLORKEYnALPHA:
			if (copy_from_user(&fbi->ckey_alpha, argp,
						sizeof(struct _sColorKeyNAlpha)))
				return -EFAULT;
			if(!fbi->suspend)
				lcdc_set_colorkeyalpha(fbi);
			break;
		case FB_IOCTL_GET_COLORKEYnALPHA:
			if (copy_to_user(argp, &fbi->ckey_alpha,
						sizeof(struct _sColorKeyNAlpha)))
				return -EFAULT;
			break;
		case FB_IOCTL_WAIT_VSYNC:
			lcdc_wait_for_vsync(fbi);
			break;
		case FB_IOCTL_WAIT_VSYNC_ON:
			fbi->vsync_en = 1;
			break;
		case FB_IOCTL_WAIT_VSYNC_OFF:
			fbi->vsync_en = 0;
			break;
		default:
			break;
	}
	return 0;
}

static void pxa95xfb_gfx_power(struct pxa95xfb_info *fbi, int on)
{
	static int i;
	struct pxa95xfb_conv_info *conv = &pxa95xfb_conv[fbi->converter - 1];

	printk(KERN_INFO "pxa95xfb_gfx_power() start \n");


	mutex_lock(&fbi->access_ok);
	if(on == (!fbi->suspend)){
		printk(KERN_INFO "LCD power already %s\n", on?"up":"down");
	} else if(!on){
		fbi->suspend = 1;

		//wr_mixer0_disable(fbi);
		if(conv->power)
		conv->power(0);

		wr_mixer0_disable(fbi);

		msleep(20);
		//converter_power(fbi, on);

		//controller_enable_disable(fbi, LCD_Controller_Quick_Disable);

		//clk_disable(fbi->clk_lcd);
		//clk_disable(fbi->clk_axi);

		display_enabled = 0;

		unset_dvfm_constraint();

		disable_oscc_tout_s0();
		printk(KERN_INFO "LCD power down %d, wo ckendisable\n", i);
	}else{
		if(conv->reset)
			conv->reset();
		
		//mdelay(100);
	
		enable_oscc_tout_s0();

		/* to turn on the display */
		set_dvfm_constraint();
		display_enabled = 1;

		//clk_enable(fbi->clk_axi);
		//clk_enable(fbi->clk_lcd);

		msleep(20);

		//lcdc_set_lcd_controller(fbi);

		/* set scale registers for fb1*/
		//set_scale(fbi);

		//msleep(10);

		//converter_power(fbi, on);
		wr_mixer0_enable(fbi);

		msleep(10);

		if(conv->power)
			conv->power(1);

		fbi->suspend = 0;
		printk(KERN_INFO "LCD power up %d wo ckendisable + Mixer disable only\n", i++);
	}
	mutex_unlock(&fbi->access_ok);

	printk(KERN_INFO "pxa95xfb_gfx_power() end \n");	

}

int pxa95xfb_pclk_modification(unsigned long int pclk,
				unsigned int blw, unsigned int elw,
				unsigned int hsw)
{

	struct fb_videomode * m = &(pxa95xfbi[0]->mode);
	u32 lcd_ctrl, lcd_mixer;

	mutex_lock(&pxa95xfbi[0]->access_ok);

	m->left_margin = blw;
	m->right_margin = elw;
	m->hsync_len = hsw;

	if (pxa95xfbi[0]->suspend) {
		printk(KERN_ERR "pxa95xfb_pclk_modification in SUSPEND state, \
		will save pclk value\n");
		mutex_unlock(&pxa95xfbi[0]->access_ok);
		return 0;
	}

	lcdc_correct_pixclock(m); /* will calculate diveder for new PCLK value */
	converter_set_parallel(pxa95xfbi[0]); /* will update LCD_MIXER0_TIM0 and call for set_clock_divider which will update LCD_CTL */
	
	lcd_ctrl = readl(pxa95xfbi[0]->reg_base + LCD_CTL);
	lcd_mixer = readl(pxa95xfbi[0]->reg_base + LCD_MIXER0_TIM0);

	/* DEBUG  */
	printk (KERN_ERR "%s: read reg values lcd_ctrl=0x%x, lcd_mixer=0x%x\n", __func__, readl(pxa95xfbi[0]->reg_base + LCD_CTL), readl(pxa95xfbi[0]->reg_base + LCD_MIXER0_TIM0));
	/* DEBUG  */

	mutex_unlock(&pxa95xfbi[0]->access_ok);
	return 0; /* =OK */
}
EXPORT_SYMBOL(pxa95xfb_pclk_modification);


#ifdef CONFIG_EARLYSUSPEND
static void pxa95xfb_gfx_earlysuspend_handler(struct early_suspend *h)
{
	pxa95xfb_gfx_power(pxa95xfbi[0], 0);
}

static void pxa95xfb_gfx_lateresume_handler(struct early_suspend *h)
{
	pxa95xfb_gfx_power(pxa95xfbi[0], 1);
}

static struct early_suspend pxa95xfb_gfx_earlysuspend = {
	.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN,
	.suspend = pxa95xfb_gfx_earlysuspend_handler,
	.resume  = pxa95xfb_gfx_lateresume_handler,
};

#endif

static int pxa95xfb_gfx_blank(int blank, struct fb_info *info)
{
	struct pxa95xfb_info *fbi = info->par;

	fbi->is_blanked = (blank == FB_BLANK_UNBLANK) ? 0 : 1;
	pxa95xfb_gfx_power(fbi, (blank == FB_BLANK_UNBLANK));

	return 0;
}

static irqreturn_t pxa95xfb_gfx_handle_irq_ctl(int irq, void *dev_id)
{
	struct pxa95xfb_info *fbi = (struct pxa95xfb_info *)dev_id;
	u32	 g, x;
	int i;

	g = readl(fbi->reg_base + LCD_CTL_INT_STS);
	/*do nothing with LCD EN/DIS/Q_DIS: we don't enable these intr*/

	/* fetch intr: wait fetch eof*/
	if(g & LCD_CTL_INT_STS_GFETCH_INT_STS){
		writel(LCD_CTL_INT_STS_GFETCH_INT_STS, fbi->reg_base + LCD_CTL_INT_STS);

		x = readl(fbi->reg_base + LCD_FETCH_INT_STS1);
		writel(x, fbi->reg_base + LCD_FETCH_INT_STS1);
		for(i = 0; i < PXA95xFB_FB_NUM; i++){
			if(pxa95xfbi[i] &&
				(x & LCD_FETCH_INT_STS1_BUS_ERRx(pxa95xfbi[i]->window)) )
				printk(KERN_ALERT "%s: fetch %d intr Bus Err happen? %x\n",
					__func__, pxa95xfbi[i]->window, x);
		}

		x = readl(fbi->reg_base + LCD_FETCH_INT_STS0);
		writel(x, fbi->reg_base + LCD_FETCH_INT_STS0);

		for(i = 0; i < PXA95xFB_FB_NUM; i++){
			if(pxa95xfbi[i] && pxa95xfbi[i]->eof_intr_en){
				if(x & LCD_FETCH_INT_STS0_END_FRx(pxa95xfbi[i]->window) ){
					/*printk(KERN_INFO "%s: fetch %d EOF intr: fr= %x\n",__func__, pxa95xfbi[i]->window, lcdc_get_fr_addr(pxa95xfbi[i]));*/
					if(pxa95xfbi[i]->vsync_en){
						atomic_set(&pxa95xfbi[i]->w_intr, 1);
						wake_up(&pxa95xfbi[i]->w_intr_wq);
					}
					if(pxa95xfbi[i]->eof_handler){
						pxa95xfbi[i]->eof_handler(pxa95xfbi[i]);
					}
				}
			}
		}
	}

	if(g & LCD_CTL_INT_STS_GWIN_INT_STS){
		writel(LCD_CTL_INT_STS_GWIN_INT_STS, fbi->reg_base + LCD_CTL_INT_STS); 
		printk(KERN_WARNING "%s: windows intr happen?\n", __func__);
	}

	if(g & LCD_CTL_INT_STS_GMIX_INT_STS){
		writel(LCD_CTL_INT_STS_GMIX_INT_STS, fbi->reg_base + LCD_CTL_INT_STS);
		for(i = 0; i < PXA95xFB_FB_NUM; i++){
			if (pxa95xfbi[i] && pxa95xfbi[i]->converter != LCD_MIXER_DISABLE) {
				x = readl(fbi->reg_base + LCD_MIXER0_INT_STS + pxa95xfbi[i]->mixer_id*0x100);
				writel(x, fbi->reg_base + LCD_MIXER0_INT_STS + pxa95xfbi[i]->mixer_id*0x100);
				if ((x & LCD_MIXERx_CTL1_DISP_UPDATE_INT_EN)) {
					display_update_ongoing[pxa95xfbi[i]->mixer_id] = 0;
				}
			}
		}
	}

	return IRQ_HANDLED;
}

#if defined(CONFIG_MACH_GFORCE) || defined(CONFIG_MACH_ALKON) || defined(CONFIG_MACH_JETTA)
//Copy bootloader data from bootloader frambuffer to kernel frambuffer
static int copy_bootimage_in_lcdfb(struct pxa95xfb_info *fbi)
{
	unsigned short  *ObmLokefb = 0x8f274040;  //lcd frambuffer address in bootloader
	unsigned int  *kernelframeBuffer = NULL; 
        unsigned short  temp=0;
	unsigned int  writeInKernelFb=0;
        unsigned int length = 0 ;
	if(fbi && fbi->fb_start && fbi->fb_size )
	{

                ObmLokefb = ioremap_nocache(ObmLokefb,(fbi->fb_size - PAGE_SIZE)/4);  //mapped physical address in virtual address space
		if (ObmLokefb == NULL) 
		{
			printk(KERN_ALERT "\n*****ObmLokefb is NULL");
                        return 0;
		}
                kernelframeBuffer = (unsigned int  *)fbi->fb_start;  // get kernel fb address
                while(length != (fbi->fb_size - PAGE_SIZE)/8)
		{
                 
		  temp = ObmLokefb[length];
		  writeInKernelFb = 0;
		  writeInKernelFb = (unsigned int)((temp & 0x1f) << 3);  //convert rgb16 to rgb24
		  writeInKernelFb |= (unsigned int)((temp & 0x7E0) << 5);
		  writeInKernelFb |= (unsigned int)((temp & 0xF100) << 8);
                  kernelframeBuffer[length ]= writeInKernelFb;   // copy in kernel buffer from bootloader buffer
                  length++;
		}
		iounmap(ObmLokefb);

	}
	else
	{
		printk(KERN_ALERT "copy_bootimage_in_lcdfb has null poiter\n");
		return 0;
	}

	return 1;
}
//checking for lcd controller is enabled /disabled
static unsigned int check_lcd_state(struct pxa95xfb_info *fbi)
{
	unsigned int ctrl=0xFFFFFFFF;
	if(fbi && fbi->reg_base)
	{
		ctrl = readl(fbi->reg_base + LCD_CTL);
		ctrl = (ctrl & LCD_CTL_LCD_EN) >> 31;
	}
	return ctrl;
}
#endif /*MACH_GFORCE || MACH_ALKON || MACH_JETTA */

static struct fb_ops pxa95xfb_gfx_ops = {
	.owner		= THIS_MODULE,
	.fb_blank		= pxa95xfb_gfx_blank,
	.fb_ioctl		= pxa95xfb_gfx_ioctl,
	.fb_check_var	= pxa95xfb_check_var,
	.fb_set_par 	= pxa95xfb_set_par,
	.fb_setcolreg	= pxa95xfb_setcolreg,	/* TODO */
	.fb_pan_display	= pxa95xfb_pan_display,
	.fb_fillrect	= cfb_fillrect,
	.fb_copyarea	= cfb_copyarea,
	.fb_imageblit	= cfb_imageblit,
};

#if 1 // SEC_GET_LOG //{{ Mark for GetLog - 1/2

struct struct_frame_buf_mark {

	u32 special_mark_1;

	u32 special_mark_2;

	u32 special_mark_3;

	u32 special_mark_4;

	void *p_fb; // it must be physical address

	u32 resX;

	u32 resY;

	u32 bpp;    // color depth : 16 or 24

	u32 frames; // frame buffer count : 2

};

static struct struct_frame_buf_mark  frame_buf_mark = {

	.special_mark_1 = (('*' << 24) | ('^' << 16) | ('^' << 8) | ('*' << 0)),

	.special_mark_2 = (('I' << 24) | ('n' << 16) | ('f' << 8) | ('o' << 0)),

	.special_mark_3 = (('H' << 24) | ('e' << 16) | ('r' << 8) | ('e' << 0)),

	.special_mark_4 = (('f' << 24) | ('b' << 16) | ('u' << 8) | ('f' << 0)),

	.p_fb   = 0,

	.resX   = 240,    // it has dependency on h/w

	.resY   = 320,     // it has dependency on h/w

	.bpp    = 16,      // it has dependency on h/w

	.frames = 2

};

#endif // SEC_GET_LOG //}} Mark for GetLog - 1/2


static int __devinit pxa95xfb_gfx_probe(struct platform_device *pdev)
{
	struct pxa95xfb_mach_info *mi;
	struct fb_info *info = 0;
	struct pxa95xfb_info *fbi = 0;
	struct pxa95xfb_conv_info *conv;
	struct resource *res;
	struct clk *clk_lcd, *clk = NULL, *clk_axi;
	int irq_ctl, irq_conv, ret;
	int i;

	mi = pdev->dev.platform_data;
	if (mi == NULL) {
		dev_err(&pdev->dev, "no platform data defined\n");
		return -EINVAL;
	}
	if (mi->regulator_initialize)
		mi->regulator_initialize(pdev);
	clk_lcd = clk_get(&pdev->dev, "PXA95x_LCDCLK");
	if (IS_ERR(clk_lcd)) {
		dev_err(&pdev->dev, "unable to get LCDCLK");
		return PTR_ERR(clk_lcd);
	}

	clk_axi = clk_get(NULL, "AXICLK");
	if (IS_ERR(clk_axi)) {
		dev_err(&pdev->dev, "unable to get axi bus clock");
		return PTR_ERR(clk_axi);
	}

	if (mi->converter == LCD_M2DSI0) {
		clk = clk_get(&pdev->dev, "PXA95x_DSI0CLK");
		if (IS_ERR(clk)) {
			dev_err(&pdev->dev, "unable to get DSI0 CLK");
			return PTR_ERR(clk);
		}
	} else if (mi->converter == LCD_M2DSI1) {
		clk = clk_get(&pdev->dev, "PXA95x_DSI1CLK");
		if (IS_ERR(clk)) {
			dev_err(&pdev->dev, "unable to get DSI1 CLK");
			return PTR_ERR(clk);
		}
	} else if (mi->converter == LCD_M2HDMI) {
		clk = clk_get(&pdev->dev, "PXA95x_iHDMICLK");
		if (IS_ERR(clk)) {
			dev_err(&pdev->dev, "unable to get internal HDMI CLK");
			return PTR_ERR(clk);
		}
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (res == NULL) {
		dev_err(&pdev->dev, "no IO memory defined\n");
		return -ENOENT;
	}

	irq_ctl = platform_get_irq(pdev, 0);
	if (irq_ctl < 0) {
		dev_err(&pdev->dev, "no IRQ defined\n");
		return -ENOENT;
	}

	if(mi->converter != LCD_MIXER_DISABLE){
		irq_conv = platform_get_irq(pdev, mi->converter);
		if (irq_conv < 0) {
			dev_err(&pdev->dev, "no IRQ defined\n");
			return -ENOENT;
		}
	}else{
		dev_err(&pdev->dev, "no converter defined\n");
		return -ENOENT;
	}

	info = framebuffer_alloc(sizeof(struct pxa95xfb_info), &pdev->dev);
	if (info == NULL)
		return -ENOMEM;

	/* Initialize private data */
	fbi = info->par;
	if(!fbi) {
		ret = -EINVAL;
		goto failed_free_clk;
	}
	fbi->fb_info = info;
	platform_set_drvdata(pdev, fbi);
	fbi->clk_lcd = clk_lcd;
	fbi->clk_axi = clk_axi;
	fbi->dev = &pdev->dev;
	fbi->on = 1;
	fbi->active = 0;
	fbi->open_count = fbi->on;/*if fbi on as default, open count +1 as default*/
	fbi->is_blanked = 0;
	fbi->suspend = 0;
	fbi->debug = 0;
	fbi->window = mi->window;
	fbi->zorder = mi->zorder;
	fbi->mixer_id = mi->mixer_id;
	fbi->converter = mi->converter;
	fbi->user_addr = 0;

	fbi->eof_intr_en = 1;
	fbi->vsync_en = 1;
	fbi->eof_handler = NULL;

	mutex_init(&fbi->access_ok);
	init_waitqueue_head(&fbi->w_intr_wq);

	/*set surface related to 0: fb0 not use it*/
	memset(&fbi->surface, 0, sizeof(fbi->surface));
	memset(&fbi->mode, 0, sizeof(struct fb_videomode));

	/* Map registers.*/
	fbi->reg_base = ioremap_nocache(res->start, res->end - res->start);
	if (fbi->reg_base == NULL) {
		ret = -ENOMEM;
		goto failed;
	}

	/* Allocate framebuffer memory: size = modes xy *4 .*/
	fbi->fb_size = PAGE_ALIGN(mi->modes[0].xres * mi->modes[0].yres * 8 + PAGE_SIZE);
	fbi->fb_start = dma_alloc_writecombine(fbi->dev, fbi->fb_size + PAGE_SIZE,
			&fbi->fb_start_dma,
			GFP_KERNEL);
	if (fbi->fb_start == NULL) {
		ret = -ENOMEM;
		goto failed;
	}
	memset(fbi->fb_start, 0, fbi->fb_size);
	fbi->fb_start = fbi->fb_start + PAGE_SIZE;
	fbi->fb_start_dma = fbi->fb_start_dma + PAGE_SIZE;

	/* Initialise static fb parameters.*/
	info->flags = FBINFO_DEFAULT | FBINFO_PARTIAL_PAN_OK |
		FBINFO_HWACCEL_XPAN | FBINFO_HWACCEL_YPAN;
	info->node = -1;
	strcpy(info->fix.id, mi->id);
	info->fix.type = FB_TYPE_PACKED_PIXELS;
	info->fix.type_aux = 0;
	info->fix.xpanstep = 0;
	info->fix.ypanstep = 0;
	info->fix.ywrapstep = 0;
	info->fix.mmio_start = res->start;
	info->fix.mmio_len = res->end - res->start + 1;
	info->fix.accel = FB_ACCEL_NONE;
	info->fbops = &pxa95xfb_gfx_ops;
	info->pseudo_palette = fbi->pseudo_palette;
	info->fix.smem_start = fbi->fb_start_dma;
	info->fix.smem_len = fbi->fb_size;
	info->screen_base = fbi->fb_start;
	info->screen_size = fbi->fb_size;

	/* Set video mode and init var*/
	for(i = 0; i < mi->num_modes; i++)
		lcdc_correct_pixclock(&mi->modes[i]);
	fb_videomode_to_modelist(mi->modes, mi->num_modes, &info->modelist);

	/* init var: according to modes[0] */
	lcdc_set_pix_fmt(&info->var, mi->pix_fmt_in);
	lcdc_set_mode_to_var(fbi, &info->var, &mi->modes[0]);
	memcpy(&fbi->mode, &mi->modes[0], sizeof(struct fb_videomode));

	info->var.xoffset = info->var.yoffset = 0;
	/* set global var at last*/
	pxa95xfbi[0] = fbi;

	/* Register irq handler. */
	ret = request_irq(irq_ctl, pxa95xfb_gfx_handle_irq_ctl, IRQF_SHARED, mi->id, fbi);
	if (ret < 0) {
		dev_err(&pdev->dev, "unable to request IRQ\n");
		ret = -ENXIO;
		goto failed_free_clk;
	}

	set_dvfm_constraint();

	/* enable clocks: clk_tout, controller, dsi*/
	enable_oscc_tout_s0();

	clk_enable(fbi->clk_axi);
	clk_enable(fbi->clk_lcd);

	/* Enable AXI32 before modifying the controller registers */
#if defined(CONFIG_MACH_GFORCE) || defined(CONFIG_MACH_ALKON) || defined(CONFIG_MACH_JETTA)
	if(check_lcd_state(fbi))  //pps-a : if lcd is not init then init it
	{
		/* lcdPanelOnOff_status should not be handle out of the internal driver - s6d04d1x21.c */
		/* lcdPanelOnOff_status = 1; */
		copy_bootimage_in_lcdfb(fbi);
	}
	else
#endif
               lcdc_writel(LCD_CTL_AXI32_EN, fbi->reg_base, LCD_CTL);
	controller_enable_disable(fbi, LCD_Controller_Disable);
	
        /* SAMSUNG_PPS : set alpha register to default value , it may be modified by lower layer */
        /*TODO: Need to set more register to their default value */
        lcdc_writel(0x00, fbi->reg_base, LCD_CH0_ALPHA);

	/* set scale registers for fb1*/
	set_scale(fbi);

	/*set ch4 as un-transparent for YVU use*/
	lcdc_writel(0x800000ff, fbi->reg_base, LCD_CH4_ALPHA);

	pxa95xfb_set_par(info);

	/*init converter*/
	conv = &pxa95xfb_conv[fbi->converter -1];
	if(!conv->inited){
		conv->inited = 1;
		conv->output = mi->output;
		conv->clk = clk;
		conv->irq = irq_conv;
		conv->pix_fmt_out = mi->pix_fmt_out;
		conv->active = mi->active;
		conv->invert_pixclock = mi->invert_pixclock;
		conv->panel_rbswap = mi->panel_rbswap;
		conv->panel_type = mi->panel_type;
		conv->power = mi->panel_power;
		conv->reset = mi->reset;

		converter_init(fbi);
	}

	if(fbi->on)
		converter_openclose(fbi, 1);

	/* For FB framework: Allocate color map and Register framebuffer*/
	if (fb_alloc_cmap(&info->cmap, 256, 0) < 0) {
		ret = -ENOMEM;
		goto failed_free_irq_conv;
	}
	ret = register_framebuffer(info);
	if (ret < 0) {
		dev_err(&pdev->dev, "Failed to register pxa95x-fb: %d\n", ret);
		ret = -ENXIO;
		goto failed_free_cmap;
	}
	printk(KERN_INFO "pxa95xfb: frame buffer device %s was loaded"
			" to /dev/fb%d <%s>.\n", conv->output?"HDMI":"PANEL", info->node, info->fix.id);

	ret = device_create_file(&pdev->dev, &dev_attr_vsync);
	if (ret < 0) {
		printk(KERN_INFO "%s, device attr create fail: %d\n", __func__, ret);
		goto failed_free_cmap;
	}

#if 1 // SEC_GET_LOG //{{ Mark for GetLog - 2/2

	frame_buf_mark.p_fb = pxa95xfbi[0]->fb_info->fix.smem_start;

	frame_buf_mark.bpp = 16; // pxa95xfbi[0]->bpp;

#endif // SEC_GET_LOG //}} Mark for GetLog - 2/2

#ifdef CONFIG_EARLYSUSPEND
	register_early_suspend(&pxa95xfb_gfx_earlysuspend);
#endif

	return 0;

failed_free_cmap:
	fb_dealloc_cmap(&info->cmap);
failed_free_irq_conv:
	free_irq(irq_conv, fbi);
	free_irq(irq_ctl, fbi);
failed_free_clk:
	clk_disable(fbi->clk_lcd);
	clk_disable(clk);
	clk_disable(fbi->clk_axi);
	clk_put(fbi->clk_lcd);
	clk_put(clk);
	clk_put(fbi->clk_axi);
failed:
	pr_err("pxa95x-fb: frame buffer device init failed\n");
	platform_set_drvdata(pdev, NULL);

	if (fbi && fbi->reg_base) {
		iounmap(fbi->reg_base);
		kfree(fbi);
	}

	return ret;
}

static struct platform_driver pxa95xfb_gfx_driver = {
	.driver		= {
		.name	= "pxa95x-fb",
		.owner	= THIS_MODULE,
	},
	.probe		= pxa95xfb_gfx_probe,
};

static int __devinit pxa95xfb_gfx_init(void)
{
	dvfm_register("pxa95x-fb", &dvfm_dev_idx);
	return platform_driver_register(&pxa95xfb_gfx_driver);
}
device_initcall_sync(pxa95xfb_gfx_init);

MODULE_AUTHOR("Lennert Buytenhek <buytenh@marvell.com>");
MODULE_DESCRIPTION("Framebuffer driver for PXA95x");
MODULE_LICENSE("GPL");
