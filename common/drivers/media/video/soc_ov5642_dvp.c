/*
 * ov5642_dvp Camera Driver
 *
 * Copyright (c) 2010 Marvell Ltd.
 * Kassey Lee <ygli@marvell.com>
 *
 * Based on  mt9t111 driver
 *
 * Copyright (C) 2008 Kuninori Morimoto <morimoto.kuninori@renesas.com>
 * Copyright (C) 2008, Robert Jarzmik <robert.jarzmik@free.fr>
 * Copyright 2006-7 Jonathan Corbet <corbet@lwn.net>
 * Copyright (C) 2008 Magnus Damm
 * Copyright (C) 2008, Guennadi Liakhovetski <kernel@pengutronix.de>
 *
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

#include "ov5642_dvp.h"
#include <media/soc_camera.h>
#include <media/soc_mediabus.h>
#include <media/v4l2-chip-ident.h>
#include <media/v4l2-common.h>

/************************************************************************

			macro

************************************************************************/
/*
 * frame size
 */

#define VGA_WIDTH   640
#define VGA_HEIGHT  480

#define pixfmtstr(x) (x) & 0xff, ((x) >> 8) & 0xff, ((x) >> 16) & 0xff, \
		((x) >> 24) & 0xff

#define REG_PIDH    0x300a
#define REG_PIDL    0x300b

/************************************************************************

			struct

************************************************************************/
struct ov5642_dvp_frame_size {
	u16 width;
	u16 height;
};


struct ov5642_dvp_priv {
	struct v4l2_subdev subdev;
	struct ov5642_dvp_camera_info *info;
	struct i2c_client *client;
	struct soc_camera_device icd;
	struct ov5642_dvp_frame_size frame;
	int model;
	u32 flags;
};

/* ov5642 has only one fixed colorspace per pixelcode */
struct ov5642_datafmt {
	enum v4l2_mbus_pixelcode	code;
	enum v4l2_colorspace		colorspace;
};


/************************************************************************

			supported format

************************************************************************/
static const struct ov5642_datafmt ov5642_colour_fmts[] = {
	/* TODO: add supported format*/
	{V4L2_MBUS_FMT_YUYV8_2X8_LE, V4L2_COLORSPACE_JPEG},
	{V4L2_MBUS_FMT_JPEG_1X8, V4L2_COLORSPACE_JPEG},
};

static struct ov5642_dvp_win_size {
	int width;
	int height;
} ov5642_dvp_win_sizes[] = {
	/* QCIF */
	{
		.width = QCIF_WIDTH,
		.height = QCIF_HEIGHT,
	},
	/* QVGA */
	{
		.width = QVGA_WIDTH,
		.height = QVGA_HEIGHT,
	},
	/* VGA */
	{
		.width = VGA_WIDTH,
		.height = VGA_HEIGHT,
	},
	/* D1 */
	{
		.width = D1_WIDTH,
		.height = D1_HEIGHT,
	},
	/* 3M */
	{
		.width = 2048,
		.height = 1536,
	}
};

/* capture jpeg size */
static struct ov5642_dvp_win_size ov5642_dvp_win_sizes_jpeg[] = {
	/* full */
	{
		.width = 2592,
		.height = 1944,
	},
	/* 3M */
	{
		.width = 2048,
		.height = 1536,
	},
	/* QVGA */
	{
		.width = QVGA_WIDTH,
		.height = QVGA_HEIGHT,
	},
	/* VGA */
	{
		.width = VGA_WIDTH,
		.height = VGA_HEIGHT,
	},
};

static const struct v4l2_queryctrl ov5642_dvp_controls[] = {
	{
		.id = V4L2_CID_AF_FIRMWARE_DOWNLOAD,
		.type = V4L2_CTRL_TYPE_BOOLEAN,
		.name = "auto focus",
	}
};

/************************************************************************

  general function

 ************************************************************************/

static int ov5642_dvp_read(struct i2c_client *c, u16 reg, unsigned char *value)
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

static int ov5642_dvp_write(struct i2c_client *c, u16 reg, unsigned char value)
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
static int ov5642_dvp_write_array(struct i2c_client *c, OV5642_WREG * vals)
{
	int i = 0;
	int ret = 0;
	while (vals->reg_num != OV5642_DVP_END_ADDR
	       || vals->value != OV5642_DVP_END_VAL) {
		ret = ov5642_dvp_write(c, vals->reg_num, vals->value);
		if (ret < 0)
			return ret;
		vals++;
		i++;
	}
	return ret;
}

static int ov5642_dvp_set_jpegdata_size(struct i2c_client *c, u16 width,
					u16 height)
{
	int ret = 0;
	ret = ov5642_dvp_write(c, 0x4602, width >> 8);
	if (ret < 0)
		goto out;
	ret = ov5642_dvp_write(c, 0x4603, width & 0xff);
	if (ret < 0)
		goto out;
out:
	return ret;
}

static struct ov5642_dvp_priv *to_ov5642_dvp(const struct i2c_client
					     *client)
{
	return container_of(i2c_get_clientdata(client),
			    struct ov5642_dvp_priv, subdev);
}

/************************************************************************

  soc_camera_ops

 ************************************************************************/
static int ov5642_dvp_set_bus_param(struct soc_camera_device *icd,
				    unsigned long flags)
{
	return 0;
}

static unsigned long ov5642_dvp_query_bus_param(struct soc_camera_device
						*icd)
{
#if 1
	return 0;
#else
	return SOCAM_MIPI | SOCAM_MIPI_2LANE;
#endif
}

static struct soc_camera_ops ov5642_dvp_ops = {
	.set_bus_param = ov5642_dvp_set_bus_param,
	.query_bus_param = ov5642_dvp_query_bus_param,
	.controls			= ov5642_dvp_controls,
	.num_controls		= ARRAY_SIZE(ov5642_dvp_controls),
};

/************************************************************************

  v4l2_subdev_core_ops

 ************************************************************************/
static int ov5642_dvp_g_chip_ident(struct v4l2_subdev *sd,
				   struct v4l2_dbg_chip_ident *id)
{
	struct i2c_client *client = sd->priv;
	struct ov5642_dvp_priv *priv = to_ov5642_dvp(client);

	id->ident = priv->model;
	id->revision = 0;

	return 0;
}

#ifdef CONFIG_VIDEO_ADV_DEBUG
static int ov5642_dvp_g_register(struct v4l2_subdev *sd,
				 struct v4l2_dbg_register *reg)
{
	struct i2c_client *client = sd->priv;
	return ov5642_dvp_read(client, (u16) reg->reg,
			       (unsigned char *)&(reg->val));
}

static int ov5642_dvp_s_register(struct v4l2_subdev *sd,
				 struct v4l2_dbg_register *reg)
{
	struct i2c_client *client = sd->priv;
	return ov5642_dvp_write(client, (u16) reg->reg,
				(unsigned char)reg->val);
}
#endif

static int ov5642_dvp_firmware_download(struct i2c_client *client,
			 __s32  value)
{
	int ret, i, size, j;
	char data[258];

	size = ARRAY_SIZE(firmware_regs);
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

static int ov5642_s_ctrl(struct v4l2_subdev *sd, struct v4l2_control *ctrl)
{
	struct i2c_client *client = sd->priv;
	const struct v4l2_queryctrl *qctrl;
	int ret = 0;

	qctrl = soc_camera_find_qctrl(&ov5642_dvp_ops, ctrl->id);
	if (!qctrl)
		return -EINVAL;

	switch (ctrl->id) {
	case V4L2_CID_AF_FIRMWARE_DOWNLOAD:
		ret = ov5642_dvp_firmware_download(client, ctrl->value);
		break;
	default:
		ret = -EINVAL;
	}
	return ret;
}

static int ov5642_dvp_load_fw(struct v4l2_subdev *sd)
{
	struct i2c_client *client = sd->priv;
	int ret = ov5642_dvp_write_array(client, ov5642_dvp_fmt_global_init);
	return ret;
}

static struct v4l2_subdev_core_ops ov5642_dvp_subdev_core_ops = {
	.g_chip_ident = ov5642_dvp_g_chip_ident,
	.load_fw = ov5642_dvp_load_fw,
	.s_ctrl	= ov5642_s_ctrl,
#ifdef CONFIG_VIDEO_ADV_DEBUG
	.g_register = ov5642_dvp_g_register,
	.s_register = ov5642_dvp_s_register,
#endif
};

/************************************************************************

  v4l2_subdev_video_ops

 ************************************************************************/
static int ov5642_dvp_streamoff(struct i2c_client *client)
{
	int ret = 0;
	ret = ov5642_dvp_write(client, 0x4201, 0x01);
	if (ret < 0)
		goto out;
	ret = ov5642_dvp_write(client, 0x4202, 0x00);
	if (ret < 0)
		goto out;
out:
	return ret;
}

static int ov5642_dvp_streamon(struct i2c_client *client)
{
	int ret = 0;
	ret = ov5642_dvp_write(client, 0x4201, 0x00);
	if (ret < 0)
		goto out;
	ret = ov5642_dvp_write(client, 0x4202, 0x00);
	if (ret < 0)
		goto out;
out:
	return ret;
}

static int ov5642_dvp_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct i2c_client *client = sd->priv;
	struct ov5642_dvp_priv *priv = to_ov5642_dvp(client);
	int ret = 0;

	if (!enable) {
		/* FIXME
		 *
		 * If user selected large output size,
		 * and used it long time,
		 * ov5642_dvp camera will be very warm.
		 *
		 * But current driver can not stop ov5642_dvp camera.
		 * So, set small size here to solve this problem.
		 */
		ret = ov5642_dvp_streamoff(client);
		return ret;
	}

	dev_dbg(&client->dev, "size   : %d x %d\n",
		priv->frame.width, priv->frame.height);
	ret = ov5642_dvp_streamon(client);

	return ret;
}

static int ov5642_enum_mbus_fsizes(struct v4l2_subdev *sd,
			struct v4l2_frmsizeenum *fsize)
{
	struct i2c_client *client = sd->priv;

	fsize->type = V4L2_FRMSIZE_TYPE_DISCRETE;
	/* abuse pixel_format, in fact, it is xlate->code*/
	switch (fsize->pixel_format) {
		case V4L2_MBUS_FMT_YUYV8_2X8_LE:
			if (fsize->index >= ARRAY_SIZE(ov5642_dvp_win_sizes)) {
				dev_warn(&client->dev,
					"ov5642 unsupported size %d!\n", fsize->index);
				return -EINVAL;
			}
			fsize->discrete.height = ov5642_dvp_win_sizes[fsize->index].height;
			fsize->discrete.width = ov5642_dvp_win_sizes[fsize->index].width;
			break;
		case V4L2_MBUS_FMT_JPEG_1X8:
			if (fsize->index >= ARRAY_SIZE(ov5642_dvp_win_sizes_jpeg)) {
				dev_warn(&client->dev,
					"ov5642 unsupported jpeg size %d!\n", fsize->index);
				return -EINVAL;
			}
			fsize->discrete.height = ov5642_dvp_win_sizes_jpeg[fsize->index].height;
			fsize->discrete.width = ov5642_dvp_win_sizes_jpeg[fsize->index].width;
			break;
		default:
			dev_err(&client->dev, "ov5642 unsupported format!\n");
			return -EINVAL;
	}

	return 0;
}

static int ov5642_dvp_s_fmt(struct v4l2_subdev *sd, struct v4l2_format *fmt)
{
	int ret = 0;
	OV5642_WREG *pregs = NULL;
	struct v4l2_pix_format *pix = &fmt->fmt.pix;
	struct i2c_client *client = sd->priv;

	switch (pix->pixelformat) {
	case V4L2_PIX_FMT_YUYV:
	case V4L2_PIX_FMT_YUV422P:
	case V4L2_PIX_FMT_YUV420:
		pregs = ov5642_dvp_jpeg_to_yuv;
		ret = ov5642_dvp_write_array(client, pregs);
		if (ret < 0)
			goto out;
		switch (pix->width) {
		case QCIF_WIDTH:
			pregs = ov5642_dvp_fmt_yuv422_qcif;
			break;
		case QVGA_WIDTH:
			pregs = ov5642_dvp_fmt_yuv422_qvga;
			break;
		case D1_WIDTH:
			pregs = ov5642_dvp_fmt_yuv422_d1;
			break;
		case VGA_WIDTH:
			pregs = ov5642_dvp_fmt_yuv422_vga;
			break;
		case 2048:
			pregs = ov5642_dvp_fmt_yuv422_qxga;
			break;

		default:
			dev_err(&client->dev, "\n unsupported "
				"size for preview! %s %d w=%d h=%d\n",
			     __func__, __LINE__,
				pix->width, pix->height);
			goto out;
			break;
		}
		break;
	case V4L2_PIX_FMT_JPEG:
		/* FIXME need to check with OV */
		pix->bytesperline = 2048;
		switch (pix->width) {

		case 2592:
			pregs = ov5642_dvp_fmt_jpeg_5M;
			ret = ov5642_dvp_set_jpegdata_size(client,
							   pix->bytesperline,
							   pix->sizeimage /
							   pix->bytesperline);
			if (ret < 0)
				goto out;

			dev_err(&client->dev, "choose 5M jpeg setting\n");
			dev_err(&client->dev, " bytesperline %d height %d\n",
			       pix->bytesperline,
			       pix->sizeimage / pix->bytesperline);
			break;
		case 640:
			pregs = ov5642_dvp_fmt_jpeg_vga;
			ret = ov5642_dvp_set_jpegdata_size(client,
							   pix->bytesperline,
							   pix->sizeimage /
							   pix->bytesperline);
			if (ret < 0)
				goto out;
			dev_err(&client->dev, "choose vga jpeg setting\n");
			dev_err(&client->dev, " bytesperline %d height %d\n",
			       pix->bytesperline,
			       pix->sizeimage / pix->bytesperline);
			break;

		case 320:
			pregs = ov5642_dvp_fmt_jpeg_qvga;
			ret = ov5642_dvp_set_jpegdata_size(client,
							   pix->bytesperline,
							   pix->sizeimage /
							   pix->bytesperline);
			if (ret < 0)
				goto out;

			dev_err(&client->dev, "choose qvga jpeg setting\n");
			dev_err(&client->dev, " bytesperline %d height %d\n",
			       pix->bytesperline,
			       pix->sizeimage / pix->bytesperline);
			break;

		case 2048:
			pregs = ov5642_dvp_fmt_jpeg_qxga;
			ret = ov5642_dvp_set_jpegdata_size(client,
							   pix->bytesperline,
							   pix->sizeimage /
							   pix->bytesperline);
			if (ret < 0)
				goto out;

			dev_err(&client->dev, "choose qxga jpeg setting\n");
			dev_err(&client->dev, " bytesperline %d height %d\n",
			       pix->bytesperline,
			       pix->sizeimage / pix->bytesperline);
			break;

		default:
			dev_err(&client->dev,
			       "unsupported JPEG format ! %s %d\n",
			       __func__, __LINE__);
			ret = -EINVAL;
			goto out;
			break;
		}
		break;

	default:
		dev_err(&client->dev, "\n unsupported format! %s %d\n",
				 __func__, __LINE__);
		break;
	}
	ret = ov5642_dvp_write_array(client, pregs);
out:
	return ret;
}

static int ov5642_dvp_g_mbus_fmt(struct v4l2_subdev *sd,
				 struct v4l2_mbus_framefmt *mf)
{
	mf->width = 640;
	mf->height = 480;
	mf->code = V4L2_PIX_FMT_YUYV;

	return 0;
}

static int ov5642_dvp_s_mbus_fmt(struct v4l2_subdev *sd,
				 struct v4l2_mbus_framefmt *fmt)
{
	return 0;
}

static int ov5642_dvp_enum_mbus_fmt(struct v4l2_subdev *sd,
		unsigned int index,
		enum v4l2_mbus_pixelcode *code)
{
	if (index >= ARRAY_SIZE(ov5642_colour_fmts))
		return -EINVAL;
	*code = ov5642_colour_fmts[index].code;
	return 0;
}

static int ov5642_dvp_try_mbus_fmt(struct v4l2_subdev *sd,
			   struct v4l2_mbus_framefmt *mf)
{
	int i;
	struct i2c_client *client = sd->priv;

	/* enum the supported formats*/
	for (i = 0; i <ARRAY_SIZE(ov5642_colour_fmts); i++) {
		if (ov5642_colour_fmts[i].code == mf->code){
			break;
		}
	}
	if (i >= ARRAY_SIZE(ov5642_colour_fmts)){
		dev_err(&client->dev, "cam: ov5642 unsupported color format!\n");
		return -EINVAL;
	}
	mf->field = V4L2_FIELD_NONE;
	switch (mf->code) {
	case V4L2_MBUS_FMT_YUYV8_2X8_LE:
		/* enum the supported sizes*/
		for (i = 0; i < ARRAY_SIZE(ov5642_dvp_win_sizes); i++) {
			if (mf->width == ov5642_dvp_win_sizes[i].width
				&& mf->height == ov5642_dvp_win_sizes[i].height) {
				break;
			}
		}
		if (i >= ARRAY_SIZE(ov5642_dvp_win_sizes)){
			dev_err(&client->dev, "cam: ov5642 unsupported window size, w%d, h%d!\n",
					mf->width, mf->height);
			return -EINVAL;
		}

		mf->colorspace = V4L2_COLORSPACE_JPEG;
		break;

	case V4L2_MBUS_FMT_JPEG_1X8:
		/* enum the supported sizes for JPEG*/
		for (i = 0; i < ARRAY_SIZE(ov5642_dvp_win_sizes_jpeg); i++) {
			if (mf->width == ov5642_dvp_win_sizes_jpeg[i].width
				&& mf->height == ov5642_dvp_win_sizes_jpeg[i].height) {
				break;
			}
		}
		if (i >= ARRAY_SIZE(ov5642_dvp_win_sizes_jpeg)){
				dev_err(&client->dev, "cam: ov5642 unsupported jpeg size!\n");
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

static struct v4l2_subdev_video_ops ov5642_dvp_subdev_video_ops = {
	.s_stream = ov5642_dvp_s_stream,
	.s_fmt = ov5642_dvp_s_fmt,
	.try_mbus_fmt	= ov5642_dvp_try_mbus_fmt,
	.g_mbus_fmt = ov5642_dvp_g_mbus_fmt,
	.s_mbus_fmt = ov5642_dvp_s_mbus_fmt,
	.enum_mbus_fmt = ov5642_dvp_enum_mbus_fmt,
	.enum_mbus_fsizes = ov5642_enum_mbus_fsizes,
};

/************************************************************************

  i2c driver

 ************************************************************************/
static struct v4l2_subdev_ops ov5642_dvp_subdev_ops = {
	.core = &ov5642_dvp_subdev_core_ops,
	.video = &ov5642_dvp_subdev_video_ops,
};

static int ov5642_dvp_detect(struct i2c_client *client)
{
	unsigned char v = 0;
	int ret = 0;

	ret = ov5642_dvp_read(client, REG_PIDH, &v);

	if (ret < 0)
		return ret;

	if (v != 0x56)
		return -ENODEV;
	client->addr = 0x3C;
	ret = ov5642_dvp_read(client, REG_PIDL, &v);
	if (ret < 0)
		return ret;
	if (v != 0x42)
		return -ENODEV;
	dev_err(&client->dev, "camera: ov5642 detected 0x%x\n", v);
	return 0;
}

static int ov5642_dvp_camera_probe(struct soc_camera_device *icd,
				   struct i2c_client *client)
{
	struct ov5642_dvp_priv *priv = to_ov5642_dvp(client);
	const char *devname;
	int chipid;
	int ret = 0;

	/*
	 * We must have a parent by now. And it cannot be a wrong one.
	 * So this entire test is completely redundant.
	 */
	if (!icd->dev.parent ||
	    to_soc_camera_host(icd->dev.parent)->nr != icd->iface)
		return -ENODEV;
	ret = ov5642_dvp_detect(client);

	if (ret)
		goto out;
	dev_err(&client->dev, "OmniVision ov5642_dvp sensor detected\n");

	devname = "ov5642_dvp";
	priv->model = V4L2_IDENT_OV5642;

	dev_info(&client->dev, "%s chip ID %04x\n", devname, chipid);
out:
	return ret;
}

static int ov5642_dvp_probe(struct i2c_client *client,
			    const struct i2c_device_id *did)
{
	struct ov5642_dvp_priv *priv;
	struct soc_camera_device *icd = client->dev.platform_data;
	struct soc_camera_link *icl;
	int ret;

	if (!icd) {
		dev_err(&client->dev, "ov5642_dvp: missing soc-camera data!\n");
		return -EINVAL;
	}

	icl = to_soc_camera_link(icd);
	if (!icl)
		return -EINVAL;

	priv = kzalloc(sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	v4l2_i2c_subdev_init(&priv->subdev, client, &ov5642_dvp_subdev_ops);

	icd->ops = &ov5642_dvp_ops;

	ret = ov5642_dvp_camera_probe(icd, client);
	if (ret) {
		icd->ops = NULL;
		kfree(priv);
	}

	return ret;
}

static int ov5642_dvp_remove(struct i2c_client *client)
{
	struct ov5642_dvp_priv *priv = to_ov5642_dvp(client);
	struct soc_camera_device *icd = client->dev.platform_data;

	icd->ops = NULL;
	kfree(priv);
	return 0;
}

static const struct i2c_device_id ov5642_dvp_id[] = {
	{"ov5642_dvp", 0},
	{}
};

MODULE_DEVICE_TABLE(i2c, ov5642_dvp_id);

static struct i2c_driver ov5642_dvp_i2c_driver = {
	.driver = {
		   .name = "ov5642_dvp",
		   },
	.probe = ov5642_dvp_probe,
	.remove = ov5642_dvp_remove,
	.id_table = ov5642_dvp_id,
};

/************************************************************************

  module function

 ************************************************************************/
static int __init ov5642_dvp_module_init(void)
{
	return i2c_add_driver(&ov5642_dvp_i2c_driver);
}

static void __exit ov5642_dvp_module_exit(void)
{
	i2c_del_driver(&ov5642_dvp_i2c_driver);
}

module_init(ov5642_dvp_module_init);
module_exit(ov5642_dvp_module_exit);

MODULE_DESCRIPTION("OmniVision OV5642 Camera Driver");
MODULE_AUTHOR("Kassey Lee");
MODULE_LICENSE("GPL");
