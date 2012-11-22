/*
 * Marvell HDMI driver
 * Jiangang Jing
 * Copyright (c) 2009, Marvell International Ltd (jgjing@marvell.com)
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * (C) Copyright 2009 Marvell International Ltd.
 * All Rights Reserved
*/

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/param.h>
#include <linux/jiffies.h>
#include <linux/platform_device.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/proc_fs.h>
#include "si9226_hdmi.h"

#define si_HDMI_RES               SI_HDMI_1280_720
static unsigned int hdmi_res = si_HDMI_RES;
static unsigned int hdmi_type;

static struct i2c_client *g_i2c_client[2] = {NULL, NULL};

static SII92XX_VIDEO Video_Mode[] = {
/*PixelClock, Vfreq,          Pixels,          Lines*/
{0x0000, 0x0000, 0x0000, 0x0000},/*NULL*/
{0x0A28, 0x1770, 0x0339, 0x020D},/*640x480*/
{0x0A28, 0x1388, 0x0344, 0x0271},/*720x576*/
{0x1B15, 0x19C8, 0x0517, 0x0326},/*1024x768*/
{0x0A28, 0x1770, 0x0339, 0x020D},/*640x480*/
{0x1B15, 0x19C8, 0x0517, 0x0326},/*1024x768*/
{0x0D8A, 0x15F9, 0x0400, 0x0271},/*800x600*/
{0x0A28, 0x1388, 0x03C0, 0x0226},/*800x480*/
{0x1B15, 0x1388, 0x073A, 0x02EE},/*1280x720*/
};

static SII92XX_DE Data_Enable[] = {
/*DE_DLY,    DE_CNT,      DE_LIN,      VSYNC_POLARITY, HSYNC_POLARITY, DE_TOP*/
{0x0000, 0x0000, 0x0000, 0, 0, 0x00},/*NULL*/
{0x00B4, 0x0280, 0x01E0, SiL92xx_DE_REG_SYNC_POLARITY_NEGETIVE, SiL92xx_DE_REG_SYNC_POLARITY_NEGETIVE, 0x2C},/*640x480*/
{0x005A, 0x02D0, 0x0240, SiL92xx_DE_REG_SYNC_POLARITY_NEGETIVE, SiL92xx_DE_REG_SYNC_POLARITY_NEGETIVE, 0x2B},/*720x576*/
{0x0117, 0x0400, 0x0300, SiL92xx_DE_REG_SYNC_POLARITY_NEGETIVE, SiL92xx_DE_REG_SYNC_POLARITY_NEGETIVE, 0x22},/*1024x768*/
{0x00B4, 0x0280, 0x01E0, SiL92xx_DE_REG_SYNC_POLARITY_NEGETIVE, SiL92xx_DE_REG_SYNC_POLARITY_NEGETIVE, 0x2C},/*640x480*/
{0x0117, 0x0400, 0x0300, SiL92xx_DE_REG_SYNC_POLARITY_NEGETIVE, SiL92xx_DE_REG_SYNC_POLARITY_NEGETIVE, 0x22},/*1024x768*/
{0x007D, 0x0320, 0x0258, SiL92xx_DE_REG_SYNC_POLARITY_NEGETIVE, SiL92xx_DE_REG_SYNC_POLARITY_NEGETIVE, 0x17},/*800x600*/
{0x0070, 0x0320, 0x01E0, SiL92xx_DE_REG_SYNC_POLARITY_NEGETIVE, SiL92xx_DE_REG_SYNC_POLARITY_NEGETIVE, 0x28},/*800x480*/
{0x0140, 0x0500, 0x02D0, SiL92xx_DE_REG_SYNC_POLARITY_POSITIVE, SiL92xx_DE_REG_SYNC_POLARITY_POSITIVE, 0x08},/*1280x720*/
};

static SII92XX_AUDIO Audio_Mode[] = {
/*audio_interface, coding_type, mclk, sample_edge, ws_polarity, sd_justify, sd_direction, ws_to_sd, i2s_map, i2s_sd, auto_downsample, swap_left_right, sample_size, freq, channels ...
...CBIT{byte0,byte1,source_number,channel_number,sampling_frequency,clock_accuracy,sample_length,original_fs}*/
{SII92XX_AUDIO_INTERFACE_I2S, SII92XX_AUDIO_CODING_TYPE_PCM, SII92XX_I2S_INTERFACE_MCLK_128,
 1, 0, 0, 0, 0, SII92XX_I2S_INTERFACE_MAP_FIFO0, SII92XX_I2S_INTERFACE_SD_SD0,
 0, 0, SII92XX_AUDIO_SAMPLE_SIZE_16BIT, SII92XX_AUDIO_SAMPLE_FREQ_44KHZ, SII92XX_AUDIO_CHANNEL_COUNT_2_CHANNELS,
{0, 0, 0, 0, 0, 0, 0x2, 0xf} },
/*TBD - other defenitions*/
};

static SII92XX_FORMAT io_format[] = {
/*Input_Format,    Input_Video_Quantization,      Output_Format,      Output_Video_Quantization, Dither_Enable, Extended_Bit, Color_Space */
{SII92XX_INPUT_FORMAT_RGB, SII92XX_INPUT_VIDEO_QUANTIZATION_AUTO, SII92XX_OUTPUT_FORMAT_HDMI_RGB,
 SII92XX_OUTPUT_VIDEO_QUANTIZATION_AUTO, 0, SII92XX_INPUT_FORMAT_ALL_8_BIT, SII92XX_OUTPUT_FORMAT_BT601},/*HDMI*/
{SII92XX_INPUT_FORMAT_RGB, SII92XX_INPUT_VIDEO_QUANTIZATION_AUTO, SII92XX_OUTPUT_FORMAT_DVI_RGB,
 SII92XX_OUTPUT_VIDEO_QUANTIZATION_AUTO, 0, SII92XX_INPUT_FORMAT_ALL_8_BIT, SII92XX_OUTPUT_FORMAT_BT601},/*DVI*/
};

static u8 si_I2C_SEQ_BUF[256];

static int hdmi_read(u8 addr)
{
	return i2c_smbus_read_byte_data(g_i2c_client[0], addr);
}

static int hdmi_write(u8 addr, u8 value)
{
	return i2c_smbus_write_byte_data(g_i2c_client[0], addr, value);
}

static int hdmi_write_burst(u8 addr, u32 length, u8 *values)
{
	return i2c_smbus_write_i2c_block_data(g_i2c_client[0], addr, length, values);
}

static int __attribute__ ((unused)) hdmi_ctrl_read(u8 addr)
{
	return i2c_smbus_read_byte_data(g_i2c_client[1], addr);
}

static int hdmi_ctrl_write(u8 addr, u8 value)
{
	return i2c_smbus_write_byte_data(g_i2c_client[1], addr, value);
}

static int __attribute__ ((unused)) hdmi_ctrl_write_burst(u8 addr, u32 length, u8 *values)
{
	return i2c_smbus_write_i2c_block_data(g_i2c_client[1], addr, length, values);
}

static int  si_hdmi_read_ID(void)
{
	unsigned int id = 0;

	id = hdmi_read(SiL92xx_DEVICE_ID_REG);
	if (id == SiL9226_DEVICE_ID) {
		printk(KERN_NOTICE "[HDMI]: Found SiI9226\n");
		return 0;
	} else if (id == SiL92xx_DEVICE_ID_OTHER_FAMILY) {
		/*Enter first page*/
		if (hdmi_write(SiL92xx_INTERNAL_PAGE, SiL92xx_FIRST_PAGE))
			return -EIO;

		/*Set 2nd register*/
		if (hdmi_write(SiL92xx_SET_INDEXED_REGISTER, SiL92xx_FIRST_PAGE_SECOND_REGISTER))
			return -EIO;

		/*Read the register*/
		id = hdmi_read(SiL92xx_READ_WRITE_ACCESS);
		if (id == SiL9226_OTHER_DEVICE_ID) {
			printk(KERN_NOTICE "[HDMI]: Found SiI9226\n");
			return 0;
		} else if (id == SiL9222_DEVICE_ID) {
			printk(KERN_NOTICE "[HDMI]: Found SiI9222\n");
			return 0;
		} else if (id == SiL922x_DEVICE_ID_OTHER_FAMILY) {
			/*Set 2nd register*/
			if (hdmi_write(SiL92xx_SET_INDEXED_REGISTER, SiL92xx_FIRST_PAGE_THIRD_REGISTER))
				return -EIO;

			/*Read the register*/
			id = hdmi_read(SiL92xx_READ_WRITE_ACCESS);
			if (id == SiL9220_MIPI_DEVICE_ID) {
				printk(KERN_NOTICE "[HDMI]: Found SiI9220 MIPI\n");
				return 0;
			}
		}
	}

	return -EIO;
}

static int si_hdmi_read_ID_rev(void)
{
	unsigned int rev;

	rev = hdmi_read(SiL92xx_DEVICE_REVISION_ID_REG);
	if (rev < 0)
		return -EIO;

	printk(KERN_DEBUG "[HDMI]: Chip version: 0x%x\n", rev);
	return 0;
}

static int si_hdmi_set_video_mode(SII92XX_VIDEO video_mode)
{
	si_I2C_SEQ_BUF[0] = (u8)(video_mode.PixelClock&0xFF);
	si_I2C_SEQ_BUF[1] = (u8)((video_mode.PixelClock&0xFF00)>>8);

	si_I2C_SEQ_BUF[2] = (u8)(video_mode.VFreq&0xFF);
	si_I2C_SEQ_BUF[3] = (u8)((video_mode.VFreq&0xFF00)>>8);

	si_I2C_SEQ_BUF[4] = (u8)(video_mode.Pixels&0xFF);
	si_I2C_SEQ_BUF[5] = (u8)((video_mode.Pixels&0xFF00)>>8);

	si_I2C_SEQ_BUF[6] = (u8)(video_mode.Lines&0xFF);
	si_I2C_SEQ_BUF[7] = (u8)((video_mode.Lines&0xFF00)>>8);

	return hdmi_write_burst(SiL92xx_VIDEO_REG, 8, si_I2C_SEQ_BUF);
}

static int si_hdmi_set_pixel_repetition(SII92XX_PIXEL_REPETITION repetition, bool edge, bool input_bus)
{
	u8 value;

	value = SiL92xx_PIXEL_REPETITION_REG_PIXEL_REPETITION_FACTOR(repetition)
		| SiL92xx_PIXEL_REPETITION_REG_EDGE(edge)
		| SiL92xx_PIXEL_REPETITION_REG_INPUT_BUS(input_bus);

	return hdmi_write(SiL92xx_PIXEL_REPETITION_REG, value);
}

static int si_hdmi_set_audio(bool enable, SII92XX_AUDIO audio)
{
	u8 value, value_bak;
	int ret;

	if (!enable) {
		return hdmi_write(SiL92xx_AUDIO_ENABLE_REG, 0);
	} else {	/*enable audio according to User definitions*/

		/*Enabled Audio Interface and mute the audio*/
		value = SiL92xx_AUDIO_ENABLE_REG_AUDIO_INTERFACE(audio.audio_interface)
				|SiL92xx_AUDIO_ENABLE_REG_CODING_TYPE(audio.coding_type);

		value_bak = value;	/*Save the value for later*/

		value |= SiL92xx_AUDIO_ENABLE_REG_MUTE;

		ret = hdmi_write(SiL92xx_AUDIO_ENABLE_REG, value);
		if (ret)
			return ret;

		switch (audio.audio_interface) {
		case SII92XX_AUDIO_INTERFACE_I2S:
			/*Assemble I2S input configuration Register value*/
			value = SiL92xx_I2S_INPUT_CONFIGURATION_REG_MCLK(audio.mclk);

			if (audio.sample_edge)
				value |= SiL92xx_I2S_INPUT_CONFIGURATION_REG_SCK_SAMPLE_EDGE;
			if (audio.ws_polarity)
				value |= SiL92xx_I2S_INPUT_CONFIGURATION_REG_WS_POLARITY;
			if (audio.sd_justify)
				value |= SiL92xx_I2S_INPUT_CONFIGURATION_REG_SD_JUSTIFY;
			if (audio.sd_direction)
				value |= SiL92xx_I2S_INPUT_CONFIGURATION_REG_SD_DIRECTION;
			if (audio.ws_to_sd)
				value |= SiL92xx_I2S_INPUT_CONFIGURATION_REG_WS_TO_SD;

			ret = hdmi_write (SiL92xx_I2S_INPUT_CONFIGURATION_REG, value);
			if (ret)
				return ret;

			/* Assemble I2S Enable and Mapping Register value*/
			value = SiL92xx_I2S_ENABLE_AND_MAPPING_REG_MAP(audio.i2s_map)
					|SiL92xx_I2S_ENABLE_AND_MAPPING_REG_SD(audio.i2s_sd)
					|SiL92xx_I2S_ENABLE_AND_MAPPING_REG_SD_CHANNEL_INPUT;
			if (audio.auto_downsample)
				value |= SiL92xx_I2S_ENABLE_AND_MAPPING_REG_AUTO_DOWNSAMPLE;
			if (audio.swap_left_right)
				value |= SiL92xx_I2S_ENABLE_AND_MAPPING_REG_SWAP_LEFT_RIGHT;

			ret = hdmi_write(SiL92xx_I2S_ENABLE_AND_MAPPING_REG, value);
			if (ret)
				return ret;

			/* Assemble Audio sample register*/
			value = SiL92xx_AUDIO_SAMPLE_REG_AUDIO_SAMPLE_SIZE(audio.sample_size)
					|SiL92xx_AUDIO_SAMPLE_REG_AUDIO_SAMPLE_FREQ(audio.freq)
					|SiL92xx_AUDIO_SAMPLE_REG_AUDIO_SAMPLE_FREQ(audio.channels);

			ret = hdmi_write(SiL92xx_AUDIO_SAMPLE_REG, value);
			if (ret)
				return ret;

			/* Assemble Stream Header Settings for I2S*/
			value = audio.cbit.byte0;
			ret = hdmi_write(SiL92xx_I2S_CHANNEL_STATUS_BYTE0, value);
			if (ret)
				return ret;

			value = audio.cbit.byte1;
			ret = hdmi_write(SiL92xx_I2S_CHANNEL_STATUS_BYTE1, value);
			if (ret)
				return ret;

			value = SiL92xx_I2S_CHANNEL_STATUS_BYTE2_SOURCE_NUMBER(audio.cbit.source_number)
				|SiL92xx_I2S_CHANNEL_STATUS_BYTE2_CHANNEL_NUMBER(audio.cbit.channel_number);

			ret = hdmi_write (SiL92xx_I2S_CHANNEL_STATUS_BYTE2, value);
			if (ret)
				return ret;

			value = SiL92xx_I2S_CHANNEL_STATUS_BYTE3_SAMPLING_FREQUENCY(audio.cbit.sampling_frequency)
					|SiL92xx_I2S_CHANNEL_STATUS_BYTE3_CLOCK_ACCURACY(audio.cbit.clock_accuracy);

			ret = hdmi_write (SiL92xx_I2S_CHANNEL_STATUS_BYTE3, value);
			if (ret)
				return ret;

			value = SiL92xx_I2S_CHANNEL_STATUS_BYTE4_SAMPLE_LENGTH(audio.cbit.sample_length)
					|SiL92xx_I2S_CHANNEL_STATUS_BYTE4_ORIGINAL_FS(audio.cbit.original_fs);

			ret = hdmi_write(SiL92xx_I2S_CHANNEL_STATUS_BYTE4, value);
			if (ret)
				return ret;
			break;

		case SII92XX_AUDIO_INTERFACE_SPDIF:
			break;
		default:
			break;
		}
	}

	mdelay(100);

	/*0x26, bit5, set to 0, 2-channel*/
	ret = hdmi_write(SiL92xx_INTERNAL_PAGE, SiL92xx_SECOND_PAGE);
	if (ret)
		return ret;

	/*Set Audio register*/
	ret = hdmi_write(SiL92xx_SET_INDEXED_REGISTER, SiL92xx_SECOND_PAGE_AUDIO_REGISTER);
	if (ret)
		return ret;

	/*Write to the register - TBD value*/
	value = hdmi_read(SiL92xx_READ_WRITE_ACCESS);
	value &= ~0x2;
	hdmi_write(SiL92xx_READ_WRITE_ACCESS, value);
	if (ret)
		return ret;

	/*0x25, word length is 16bit*/
	ret = hdmi_write(SiL92xx_INTERNAL_PAGE, SiL92xx_SECOND_PAGE);
	if (ret)
		return ret;
	ret = hdmi_write(SiL92xx_SET_INDEXED_REGISTER, 0x24);
	if (ret)
		return ret;

	value = hdmi_read(SiL92xx_READ_WRITE_ACCESS);
	value &= 0xf0;
	value |= 0x02;
	ret = hdmi_write(SiL92xx_READ_WRITE_ACCESS, value);
	if (ret)
		return ret;

	/* Enable I2S and un-mute the audio*/
	value = value_bak;
	ret = hdmi_write(SiL92xx_AUDIO_ENABLE_REG, value);
	if (ret)
		return ret;

	return 0;
}

static int si_hdmi_set_data_enable(bool enable, SII92XX_DE de)
{
	if (enable) {
		si_I2C_SEQ_BUF[0] = (u8)(de.DE_DLY&0xFF);
		si_I2C_SEQ_BUF[1] = (u8)((de.DE_DLY&0x0300)>>8)
							| SiL92xx_DE_REG_ENABLE(enable)
							| SiL92xx_DE_REG_VSYNC_POLARITY(de.VSYNC_POLARITY)
							| SiL92xx_DE_REG_HSYNC_POLARITY(de.HSYNC_POLARITY);

		si_I2C_SEQ_BUF[2] = (u8)(de.DE_TOP&0x7F);
		si_I2C_SEQ_BUF[3] = 0;

		si_I2C_SEQ_BUF[4] = (u8)(de.DE_CNT&0xFF);
		si_I2C_SEQ_BUF[5] = (u8)((de.DE_CNT&0x0F00)>>8);

		si_I2C_SEQ_BUF[6] = (u8)(de.DE_LIN&0xFF);
		si_I2C_SEQ_BUF[7] = (u8)((de.DE_LIN&0x0700)>>8);
	} else
		memset(si_I2C_SEQ_BUF, 0, 8);

	return hdmi_write_burst(SiL92xx_DE_REG, 8, si_I2C_SEQ_BUF);
}

static int si_hdmi_set_io_format(SII92XX_FORMAT format)
{
	si_I2C_SEQ_BUF[0] = SiL92xx_INPUT_FORMAT_REG_FORMAT(format.Input_Format)
						| SiL92xx_INPUT_FORMAT_REG_QUANTIZATION(format.Input_Video_Quantization)
						| SiL92xx_INPUT_FORMAT_REG_10BIT_TO_8BIT(format.Dither_Enable)
						| SiL92xx_INPUT_FORMAT_REG_EXTENDED_BIT(format.Extended_Bit);

	si_I2C_SEQ_BUF[1] = SiL92xx_OUTPUT_FORMAT_REG_FORMAT(format.Output_Format)
						| SiL92xx_OUTPUT_FORMAT_REG_QUANTIZATION(format.Output_Video_Quantization)
						| SiL92xx_OUTPUT_FORMAT_REG_COLOR_SPACE(format.Color_Space);

	return hdmi_write_burst(SiL92xx_INPUT_FORMAT_REG, 2, si_I2C_SEQ_BUF);
}

static int  si_hdmi_set_system_control(bool output_mode, bool ddc, bool mute, bool power_down)
{
	u8 value;

	value = SiL92xx_SYSTEM_CONTROL_OUTPUT_MODE(output_mode)
			|SiL92xx_SYSTEM_CONTROL_DDC(ddc)
			|SiL92xx_SYSTEM_CONTROL_MUTE(mute)
			|SiL92xx_SYSTEM_CONTROL_TMDS(power_down);

	return hdmi_write(SiL92xx_SYSTEM_CONTROL, value);
}

static int si_hdmi_set_ctrl_channel(void)
{
	if (hdmi_ctrl_write(SiL92xx_PVT_ADDRESS, SiL92xx_CTRL_BUS_SET_CEC_LADDR))
		return -EIO;

	if (hdmi_ctrl_write(SiL92xx_PVT_CONTROL, SiL92xx_PVT_CONTROL_START))
		return -EIO;

	if (hdmi_ctrl_write(SiL92xx_PVT_ADDRESS, SiL92xx_CTRL_BUS_GET_CEC_LADDR))
		return -EIO;

	if (hdmi_ctrl_write(SiL92xx_PVT_CONTROL, SiL92xx_PVT_CONTROL_START))
		return -EIO;

	return 0;
}

static int si_hdmi_set_power_state(SII92XX_POWER_STATE state, bool ctrl_pin)
{
	u8 value;
	value = SiL92xx_POWER_STATE_TX_POWER_STATE(state)
			|SiL92xx_POWER_STATE_CTRL_PIN(ctrl_pin);

	return hdmi_write(SiL92xx_POWER_STATE, value);
}

static int	si_hdmi_init(void)
{
	hdmi_type = ((hdmi_res < 4) | (hdmi_res == SI_HDMI_1280_720)) ? SI_HDMI : SI_DVI;

	if (!g_i2c_client[0] || !g_i2c_client[1]) {
		printk(KERN_ERR "[HDMI]:I2C client error\n");
		return -EIO;
	}

	/*Read HDMI device ID*/
	if (si_hdmi_read_ID()) {
		printk(KERN_ERR "[HDMI] Invalid HDMI ID\n");
		return -EIO;
	}

	/*Read HDMI device version*/
	if (si_hdmi_read_ID_rev()) {
		printk(KERN_ERR "[HDMI] Invalid HDMI version\n");
		return -EIO;
	}

	/*Set video mode*/
	if (si_hdmi_set_video_mode(Video_Mode[hdmi_res])) {
		printk(KERN_ERR "[HDMI] Set video mode fail\n");
		return -EIO;
	}

	/*Set pixel repetition*/
	if (si_hdmi_set_pixel_repetition(SII92XX_PIXEL_REPETITION_NOT_REPLICATED, PIXEL_REPETITION_FALLING_EDGE, PIXEL_REPETITION_24_BITS_WIDE)) {
		printk(KERN_ERR "[HDMI] Set pixel repetition fail\n");
		return -EIO;
	}

	/*Set audio config*/
	if (si_hdmi_set_audio(1, Audio_Mode[0])) {
		printk(KERN_ERR "[HDMI] Set audio fail\n");
		return -EIO;
	}

	/*Set data enable*/
	if (si_hdmi_set_data_enable(1, Data_Enable[hdmi_res])) {
		printk(KERN_ERR "[HDMI] Set data enable fail\n");
		return -EIO;
	}

	/*Set input output format*/
	if (si_hdmi_set_io_format(io_format[hdmi_type])) {
		printk(KERN_ERR "[HDMI] Set io format fail\n");
		return -EIO;
	}

	/*Set input output format*/
	if (si_hdmi_set_system_control(hdmi_type, 0, 0, 1)) {
		printk(KERN_ERR "[HDMI] Set system control fail\n");
		return -EIO;
	}

	mdelay(128);

	/*Set control channel*/
	if (si_hdmi_set_ctrl_channel()) {
		printk(KERN_ERR "[HDMI] Set ctrl channel fail\n");
		return -EIO;
	}

	return 0;
}

static int si_hdmi_start_streaming(void)
{

	/*Set power state to D0*/
	if (si_hdmi_set_power_state(SII92XX_POWER_STATE_D0_STATE, 1)) {
		printk(KERN_ERR "[HDMI] Set power state fail\n");
		return -EIO;
	}

	/*Enable system control*/
	if (si_hdmi_set_system_control(hdmi_type, 0, 0, 0)) {
		printk(KERN_ERR "[HDMI] Set system control enable fail\n");
		return -EIO;
	}

	return 0;
}


static void (*hdmi_power)(struct device *dev, int on);
static bool is_enable;

static ssize_t sii9226_enable_store(struct device *dev,
				      struct device_attribute *attr,
				      const char *buf, size_t count)
{
	u8 value;
	unsigned long enable = simple_strtoul(buf, NULL, 10);

	if (enable == 1) {
		if (is_enable == 0) {
			if (hdmi_power)
				hdmi_power(dev, 1);
			msleep(10);

			si_hdmi_init();
			si_hdmi_start_streaming();
			is_enable = enable;

			printk(KERN_NOTICE "[HDMI]: power up HDMI\n");
		} else {

			printk(KERN_ERR "[HDMI]: it is already enabled\n");
		}
	} else if (enable == 0) {

		if (is_enable == 1) {

			/*enable Rx-sense and hot-plug before enter to D3*/
			value = hdmi_read(0x3d);
			value |= 0x3;
			hdmi_write(0x3d, value);

			/*enter D3 power state*/
			if (si_hdmi_set_power_state(SII92XX_POWER_STATE_D3_STATE, 0))
				printk(KERN_ERR "[HDMI]: Set power state to D3 done, ignore i2c error\n");
			is_enable = enable;

			if (hdmi_power)
				hdmi_power(dev, 0);

			printk(KERN_ERR "[HDMI]: power down HDMI\n");
		} else {
			printk(KERN_ERR "[HDMI]: it is already disable\n");
		}
	} else
		printk(KERN_ERR "[HDMI]: invalid parameter, please set 1-enable, 0-disable\n");
	return count;
}

static ssize_t sii9226_enable_show(struct device *dev,
				     struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", is_enable);
}

static ssize_t sii9226_detect_show(struct device *dev,
				     struct device_attribute *attr, char *buf)
{
	int value, detect = 0;

	if (is_enable) {
		value = hdmi_read(0x3d);
		/* bit2 is set means cable is pulged*/
		if (value & 1 << 2) {
			detect = 1;
			printk(KERN_NOTICE "[HDMI]: hdmi cable is pluged\n");
		} else {
			printk(KERN_NOTICE "[HDMI]: hdmi cable is un-pluged\n");
		}

	} else {
		printk(KERN_ERR "[HDMI]: hdmi has been powered down\n");
	}
	return sprintf(buf, "%d\n", detect);
}

static DEVICE_ATTR(enable, S_IRUGO|S_IWUGO,
		   sii9226_enable_show, sii9226_enable_store);
static DEVICE_ATTR(detect, S_IRUGO,
		   sii9226_detect_show, NULL);

static struct attribute *sii9226_attributes[] = {
	&dev_attr_enable.attr,
	&dev_attr_detect.attr,
	NULL
};

static struct attribute_group sii9226_attribute_group = {
	.attrs = sii9226_attributes
};

static int __devinit pxa95x_hdmi_probe(struct i2c_client *client, const struct i2c_device_id *client_id)
{
	int ret;

	g_i2c_client[client_id->driver_data] = client;
	if (client_id->driver_data == 0)
		hdmi_power = client->dev.platform_data;

	if (client_id->driver_data == 0) {
		ret = sysfs_create_group(&client->dev.kobj, &sii9226_attribute_group);
		if (ret < 0) {
			printk(KERN_ERR "[HDMI]: failed to create file node!\n");
			return -EIO;
		}
	}

	return 0;
}

static int pxa95x_hdmi_command(struct i2c_client *client, unsigned int cmd,
		void *arg)
{
	return 0;
}

static const struct i2c_device_id hdmi_id[] = {
	{ "SI9226", 0 },
	{ "SI9226-ctrl", 1 },
	{ }
};

static struct i2c_driver pxa95x_hdmi_driver = {
	.driver		= {
		.name	= "pxa95xhdmi",
		.owner	= THIS_MODULE,
	},
	.id_table	= hdmi_id,
	.probe		= pxa95x_hdmi_probe,
	.command	= pxa95x_hdmi_command,
};


static int __init hdmi_init(void)
{
	return i2c_add_driver(&pxa95x_hdmi_driver);
}
late_initcall(hdmi_init);

MODULE_AUTHOR("Tianxf");
MODULE_DESCRIPTION("MHL driver for PXA95x");
MODULE_LICENSE("GPL");

