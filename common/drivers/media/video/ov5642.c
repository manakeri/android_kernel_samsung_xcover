/*
 * ov5642 Camera Driver
 *
 * Copyright (c) 2010 Marvell Ltd.
 * Qing Xu <qingx@marvell.com>
 * Kassey Lee <ygli@marvell.com>
 * Angela Wan <jwan@marvell.com>
 * Jiaquan Su <jqsu@marvell.com>
 *
 * Based on linux/drivers/media/video/mt9m001.c
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/videodev2.h>

#include "ov5642.h"

#define REG_PIDH    0x300a
#define REG_PIDL    0x300b

struct i2c_client *g_i2c_client;

static const struct ov5642_datafmt ov5642_colour_fmts[] = {
	{V4L2_MBUS_FMT_YUYV8_2X8_BE, V4L2_COLORSPACE_JPEG},
	{V4L2_MBUS_FMT_JPEG_1X8, V4L2_COLORSPACE_JPEG},
	{V4L2_MBUS_FMT_RGB565_2X8_LE, V4L2_COLORSPACE_SRGB}
};

static const struct v4l2_queryctrl ov5642_controls[] = {
	{
		.id = V4L2_CID_FOCUS_AUTO,
		.type = V4L2_CTRL_TYPE_INTEGER,
		.name = "auto focus",
	}, {
		.id = V4L2_CID_BRIGHTNESS,
		.type = V4L2_CTRL_TYPE_INTEGER,
		.name = "brightness",
	}

};

static struct ov5642 *to_ov5642(const struct i2c_client
					     *client)
{
	return container_of(i2c_get_clientdata(client),
			    struct ov5642, subdev);
}

int ov5642_read(struct i2c_client *c, u16 reg, unsigned char *value)
{
	u8 data;
	u8 address[2];
	int ret = 0;
	address[0] = reg >> 8;
	address[1] = reg;
	ret = i2c_master_send(c, address, 2);
	if (ret < 0)
		goto out;
	ret = i2c_master_recv(c, &data, 1);
	if (ret < 0)
		goto out;
	*value = data;
out:
	return (ret < 0) ? ret : 0;
}

int ov5642_write(struct i2c_client *c, u16 reg, unsigned char value)
{
	u8 data[3];
	int ret = 0;
	data[0] = reg >> 8;
	data[1] = reg;
	data[2] = value;
	ret = i2c_master_send(c, data, 3);
	return (ret < 0) ? ret : 0;
}

/*
 * Write a list of register settings; ff/ff stops the process.
 */
static int ov5642_write_array(struct i2c_client *c, struct regval_list * vals)
{
	int i = 0;
	int ret = 0;
	while (vals->reg_num != OV5642_END_ADDR
	       || vals->value != OV5642_END_VAL) {
		ret = ov5642_write(c, vals->reg_num, vals->value);
		if (ret < 0)
			return ret;
		vals++;
		i++;
	}
	return ret;
}

int to_mipi(u32 code, u32 width, u32 height)
{
struct regval_list *fmt_reg = NULL, *res_reg = NULL;
int ret;
	/* ov5642 works as a mipi converter, switch dvp signal to mipi signal*/
	ret = select_bus_type("pxa955-mipi-bridge");
	if (ret) {
		dev_err(&g_i2c_client->dev, \
			"mipi bridge settings undefined\n");
		return ret;
	}

	fmt_reg = get_fmt_default_setting(code);
	res_reg = get_yuv_size_regs(width, height);

	if ((fmt_reg == NULL) || (res_reg == NULL)) {
		dev_err(&g_i2c_client->dev, \
			"mipi bridge format or resolution not supported \n");
		return -EINVAL;
	}

	ov5642_write_array(g_i2c_client, fmt_reg);
	ov5642_write_array(g_i2c_client, res_reg);

	msleep(1);
	return 0;
}
EXPORT_SYMBOL(to_mipi);

static int ov5642_detect(struct i2c_client *client)
{
	unsigned char v = 0;
	int ret = 0;

	ret = ov5642_read(client, REG_PIDH, &v);

	if (ret < 0)
		return ret;
	if (v != 0x56)
		return -ENODEV;

	ret = ov5642_read(client, REG_PIDL, &v);
	if (ret < 0)
		return ret;
	if (v != 0x42)
		return -ENODEV;
	dev_err(&client->dev, "camera: ov5642 detected 0x%x\n", v);
	return 0;
}

static int ov5642_g_chip_ident(struct v4l2_subdev *sd,
				   struct v4l2_dbg_chip_ident *id)
{
	struct i2c_client *client = sd->priv;
	struct ov5642 *ov5642 = to_ov5642(client);

	id->ident = ov5642->model;
	id->revision = 0;

	return 0;
}

#ifdef CONFIG_VIDEO_ADV_DEBUG
static int ov5642_g_register(struct v4l2_subdev *sd,
				 struct v4l2_dbg_register *reg)
{
	struct i2c_client *client = sd->priv;
	return ov5642_read(client, (u16) reg->reg,
			       (unsigned char *)&(reg->val));
}

static int ov5642_s_register(struct v4l2_subdev *sd,
				 struct v4l2_dbg_register *reg)
{
	struct i2c_client *client = sd->priv;
	return ov5642_write(client, (u16) reg->reg,
				(unsigned char)reg->val);
}
#endif

static int ov5642_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct i2c_client *client = sd->priv;
	int ret = 0;
	ret = set_stream(client, enable);
	if (ret < 0)
		dev_err(&client->dev, "ov5642 set stream error\n");
	return ret;
}

static int ov5642_enum_fmt(struct v4l2_subdev *sd,
		unsigned int index,
		enum v4l2_mbus_pixelcode *code)
{
	if (index >= ARRAY_SIZE(ov5642_colour_fmts))
		return -EINVAL;
	*code = ov5642_colour_fmts[index].code;
	return 0;
}

static int ov5642_try_fmt(struct v4l2_subdev *sd,
			   struct v4l2_mbus_framefmt *mf)
{
	struct i2c_client *client = sd->priv;
	struct ov5642 *ov5642 = to_ov5642(client);

	ov5642->regs_fmt = get_fmt_regs(mf->code);
	if (!ov5642->regs_fmt) {
		dev_err(&client->dev, "ov5642 unsupported color format!!\n");
		return -EINVAL;
	}

	ov5642->regs_default = get_fmt_default_setting(mf->code);

	mf->field = V4L2_FIELD_NONE;

	switch (mf->code) {
	case V4L2_MBUS_FMT_RGB565_2X8_LE:
	case V4L2_MBUS_FMT_YUYV8_2X8_LE:
	case V4L2_MBUS_FMT_YVYU8_2X8_LE:
	case V4L2_MBUS_FMT_YUYV8_2X8_BE:
	case V4L2_MBUS_FMT_YVYU8_2X8_BE:
		/* enum the supported sizes*/
		ov5642->regs_size = get_yuv_size_regs(mf->width, mf->height);
		if (!ov5642->regs_size) {
			dev_err(&client->dev, "ov5642 unsupported yuv resolution!\n");
			return -EINVAL;
		}
		if (mf->code == V4L2_MBUS_FMT_RGB565_2X8_LE)
			mf->colorspace = V4L2_COLORSPACE_SRGB;
		else
			mf->colorspace = V4L2_COLORSPACE_JPEG;
		break;
	case V4L2_MBUS_FMT_JPEG_1X8:
		/* enum the supported sizes for JPEG*/
		ov5642->regs_size = get_jpg_size_regs(mf->width, mf->height);
		if (!ov5642->regs_size) {
			dev_err(&client->dev, "ov5642 unsupported yuv resolution!\n");
			return -EINVAL;
		}
		mf->colorspace = V4L2_COLORSPACE_JPEG;
		break;
	default:
		dev_err(&client->dev, "ov5642 doesn't support code %d\n", mf->code);
		break;
	}

	return 0;
}

static int ov5642_s_fmt(struct v4l2_subdev *sd,
			 struct v4l2_mbus_framefmt *mf)
{
	int ret = 0;
	struct i2c_client *client = sd->priv;
	struct ov5642 *ov5642 = to_ov5642(client);

	if (ov5642->regs_default) {
		ret = ov5642_write_array(client, ov5642->regs_default);
		if (ret)
			return ret;
	}

	if (ov5642->regs_fmt) {
		ret = ov5642_write_array(client, ov5642->regs_fmt);
		if (ret)
			return ret;
	}

	if (ov5642->regs_size) {
		ret = ov5642_write_array(client, ov5642->regs_size);
		if (ret)
			return ret;
	}

	return ret;
}

static int ov5642_g_fmt(struct v4l2_subdev *sd, struct v4l2_mbus_framefmt *mf)
{
	struct i2c_client *client = sd->priv;
	struct ov5642 *ov5642 = to_ov5642(client);

	mf->width		= ov5642->rect.width;
	mf->height		= ov5642->rect.height;
	mf->code	= V4L2_MBUS_FMT_YUYV8_2X8_BE;
	mf->field		= V4L2_FIELD_NONE;
	mf->colorspace		= V4L2_COLORSPACE_JPEG;
	return 0;
}

static int ov5642_enum_fsizes(struct v4l2_subdev *sd,
 struct v4l2_frmsizeenum *fsize)
{
	struct i2c_client *client = sd->priv;
	struct ov5642_win_size *res = NULL;
	int size = 0;

	if (!fsize)
		return -EINVAL;

	fsize->type = V4L2_FRMSIZE_TYPE_DISCRETE;

	/* abuse pixel_format, in fact, it is xlate->code*/
	switch (fsize->pixel_format) {
	case V4L2_MBUS_FMT_YUYV8_2X8_BE:
	case V4L2_MBUS_FMT_RGB565_2X8_LE:
		size = get_yuv_res_array(&res);
		if (fsize->index >= size)
			return -EINVAL;

		if (res) {
			fsize->discrete.height = res[fsize->index].height;
			fsize->discrete.width = res[fsize->index].width;
		}
		break;
	case V4L2_MBUS_FMT_JPEG_1X8:
		size = get_jpg_res_array(&res);
		if (fsize->index >= size)
			return -EINVAL;

		if (res) {
			fsize->discrete.height = res[fsize->index].height;
			fsize->discrete.width = res[fsize->index].width;
		}
		break;
	default:
		dev_err(&client->dev, "ov5642 unsupported format!\n");
		return -EINVAL;
}
	return 0;
}

static unsigned long ov5642_query_bus_param(struct soc_camera_device
						*icd)
{
	struct soc_camera_link *icl = to_soc_camera_link(icd);
	/* ov5642 mipi setting: support 1 lane or 2 lane */
	/* ov5642 dvp setting: default value = HSYNC_ACTIVE_LOW
	 *		| VSYNC_ACTIVE_LOW | PCLK_SAMPLE_FALLING */
	unsigned long flags = SOCAM_MIPI | SOCAM_MIPI_1LANE | SOCAM_MIPI_2LANE \
			| SOCAM_HSYNC_ACTIVE_LOW | SOCAM_VSYNC_ACTIVE_LOW \
			| SOCAM_PCLK_SAMPLE_FALLING;
	return soc_camera_apply_sensor_flags(icl, flags);
}

static int ov5642_set_bus_param(struct soc_camera_device *icd,
				    unsigned long flags)
{
	return 0;
}

static struct soc_camera_ops ov5642_ops = {
	.query_bus_param = ov5642_query_bus_param,
	.set_bus_param = ov5642_set_bus_param,
	.controls			= ov5642_controls,
	.num_controls		= ARRAY_SIZE(ov5642_controls),
};

static int ov5642_firmware_download(struct i2c_client *client)
{
	int ret = 0, i, size, j;
	char data[258];
	OV5642_REG_ARRAY *firmware_regs = NULL;

	dev_info(&client->dev, "AF firmware downloader\n");
	size = get_firmware(&firmware_regs);
	for (i = 0; i < size; i++) {
		data[0] = firmware_regs[i].reg_base >> 8;
		data[1] = firmware_regs[i].reg_base;
		for (j = 0; j < firmware_regs[i].len; j++)
			data[j+2] = firmware_regs[i].value[j];
		ret = i2c_master_send(client, data, firmware_regs[i].len+2);
		if (ret < 0) {
			dev_err(&client->dev, "i2c error %s %d\n",
				__func__, __LINE__);
			break;
		}
	}
	return (ret > 0) ? 0 : ret;

}

static int ov5642_set_brightness(struct i2c_client *client, int value)
{
	return 0;
}

static int ov5642_s_ctrl(struct v4l2_subdev *sd, struct v4l2_control *ctrl)
{
	struct i2c_client *client = sd->priv;
	const struct v4l2_queryctrl *qctrl;
	int ret = 0;

	qctrl = soc_camera_find_qctrl(&ov5642_ops, ctrl->id);
	if (!qctrl)
		return -EINVAL;
	switch (ctrl->id) {
	case V4L2_CID_BRIGHTNESS:
		ret = ov5642_set_brightness(client, ctrl->value);
		break;
	case V4L2_CID_FOCUS_AUTO:
		//if (ctrl->value == 0)
		ret = ov5642_firmware_download(client);
		break;
	default:
		ret = -EINVAL;
	}
	return ret;
}

static int ov5642_load_fw(struct v4l2_subdev *sd)
{
	struct i2c_client *client = sd->priv;
	struct ov5642 *ov5642 = to_ov5642(client);
	int ret = 0;

	ov5642->init = get_global_init_regs();
	if (ov5642->init)
		ret = ov5642_write_array(client, ov5642->init);
	return ret;
}

static struct v4l2_subdev_core_ops ov5642_subdev_core_ops = {
	.g_chip_ident = ov5642_g_chip_ident,
	.load_fw = ov5642_load_fw,
	.s_ctrl	= ov5642_s_ctrl,
#ifdef CONFIG_VIDEO_ADV_DEBUG
	.g_register = ov5642_g_register,
	.s_register = ov5642_s_register,
#endif
};

static struct v4l2_subdev_video_ops ov5642_subdev_video_ops = {
	.s_stream = ov5642_s_stream,
	.g_mbus_fmt = ov5642_g_fmt,
	.s_mbus_fmt = ov5642_s_fmt,
	.try_mbus_fmt = ov5642_try_fmt,
	.enum_mbus_fsizes = ov5642_enum_fsizes,
	.enum_mbus_fmt = ov5642_enum_fmt,
};

static struct v4l2_subdev_ops ov5642_subdev_ops = {
	.core = &ov5642_subdev_core_ops,
	.video = &ov5642_subdev_video_ops,
};

static int ov5642_video_probe(struct soc_camera_device *icd,
				   struct i2c_client *client)
{
	struct ov5642 *ov5642 = to_ov5642(client);
	int ret = 0;

	/*
	 * We must have a parent by now. And it cannot be a wrong one.
	 * So this entire test is completely redundant.
	 */
	if (!icd->dev.parent ||
	    to_soc_camera_host(icd->dev.parent)->nr != icd->iface)
		return -ENODEV;
	ret = ov5642_detect(client);

	if (ret)
		goto out;
	dev_err(&client->dev, "OmniVision ov5642 sensor detected\n");

	ov5642->model = V4L2_IDENT_OV5642;

out:
	return ret;

}

static int ov5642_probe(struct i2c_client *client,
			    const struct i2c_device_id *did)
{
	struct ov5642 *ov5642;
	struct soc_camera_device *icd = client->dev.platform_data;
	struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);
	struct soc_camera_link *icl;
	int ret;

	g_i2c_client = client;

	if (!icd) {
		dev_err(&client->dev, "ov5642: missing soc-camera data!\n");
		return -EINVAL;
	}

	icl = to_soc_camera_link(icd);
	if (!icl) {
		dev_err(&client->dev, "ov5642 driver needs platform data\n");
		return -EINVAL;
	}
	if (icl->priv) {
		ret = select_bus_type((char *)icl->priv);
		if (ret) {
			dev_err(&client->dev, "ov5642 driver need know bus type\n");
			return ret;
		}
	} else {
		dev_err(&client->dev, "ov5642 driver need know priv\n");
		return -EINVAL;
	}

	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_WORD_DATA)) {
		dev_warn(&adapter->dev,
			 "I2C-Adapter doesn't support I2C_FUNC_SMBUS_WORD\n");
		return -EIO;
	}

	ov5642 = kzalloc(sizeof(struct ov5642), GFP_KERNEL);
	if (!ov5642)
		return -ENOMEM;

	v4l2_i2c_subdev_init(&ov5642->subdev, client, &ov5642_subdev_ops);
	icd->ops = &ov5642_ops;
	ov5642->rect.left = 0;
	ov5642->rect.top = 0;
	ov5642->rect.width = 640;
	ov5642->rect.height = 480;
	ov5642->pixfmt = V4L2_PIX_FMT_YUV422P;

	ret = ov5642_video_probe(icd, client);
	if (ret) {
		icd->ops = NULL;
		kfree(ov5642);
	}

	return ret;
}

static int ov5642_remove(struct i2c_client *client)
{
	struct ov5642 *ov5642 = to_ov5642(client);
	struct soc_camera_device *icd = client->dev.platform_data;

	icd->ops = NULL;
	kfree(ov5642);
	return 0;
}

static const struct i2c_device_id ov5642_idtable[] = {
	{"ov5642", 0},
	{}
};

MODULE_DEVICE_TABLE(i2c, ov5642_idtable);

static struct i2c_driver ov5642_driver = {
	.driver = {
		   .name = "ov5642",
		   },
	.probe = ov5642_probe,
	.remove = ov5642_remove,
	.id_table = ov5642_idtable,
};

static int __init ov5642_mod_init(void)
{
	int ret = 0;
	ret = i2c_add_driver(&ov5642_driver);
	return ret;
}

static void __exit ov5642_mod_exit(void)
{
	i2c_del_driver(&ov5642_driver);
}

module_init(ov5642_mod_init);
module_exit(ov5642_mod_exit);

MODULE_DESCRIPTION("OmniVision OV5642 Camera Driver");
MODULE_LICENSE("GPL");
