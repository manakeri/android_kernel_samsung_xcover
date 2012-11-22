/*-------------------------------------------------------

* Driver for OmniVision CMOS Image Sensor
*
* Copyright (C) 2010, Marvell International Ltd.
*				Qing Xu <qingx@marvell.com>
*
* Based on linux/drivers/media/video/mt9m001.c
*
* This program is free software; you can redistribute it and/or modify
* it under the terms of the GNU General Public License version 2 as
* published by the Free Software Foundation.

-------------------------------------------------------*/

#include <linux/init.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/i2c.h>

#include <linux/videodev2.h>
#include <media/v4l2-common.h>
#include <media/v4l2-chip-ident.h>
#include <media/soc_camera.h>

//#include "ov7690.h"
struct regval_list {
	u8 reg_num;
	unsigned char value;
};

static struct regval_list ov7690_default[] = {
	/* 320 x 240 */
	/*@@ OV5642_Br_ov7690_vga_yuv_1lane*/
	/*;OV7690 vga_raw*/

	{0x12, 0x80},
	{0x0c, 0x16},
	{0x48, 0x42},
	{0x41, 0x43},
	{0x81, 0xff},
	{0x21, 0x44},
	{0x16, 0x03},
	{0x39, 0x80},
	{0x12, 0x01},
	{0x82, 0x00},
	{0xd0, 0x48},
	{0x80, 0x7f},
	{0x3e, 0x20},
	{0x22, 0x00},
	{0x17, 0x69},
	{0x18, 0xa4},
	{0x19, 0x0c},
	{0x1a, 0xf6},
	/*VGA*/

	{0xc8, 0x02},
	{0xc9, 0x80},
	{0xca, 0x01},
	{0xcb, 0xe0},
	{0xcc, 0x02},
	{0xcd, 0x80},
	{0xce, 0x01},
	{0xcf, 0xe0},

	/*;;QVGA*/
	/*
	{0xc8, 0x02},
	{0xc9, 0x80},
	{0xca, 0x01},
	{0xcb, 0xe0},
	{0xcc, 0x01},
	{0xcd, 0x40},
	{0xce, 0x00},
	{0xcf, 0xf0},
	*/

	{0x85, 0x90},
	{0x86, 0x00},
	{0x87, 0x00},
	{0x88, 0x10},
	{0x89, 0x30},
	{0x8a, 0x29},
	{0x8b, 0x26},
	{0xbb, 0x80},
	{0xbc, 0x62},
	{0xbd, 0x1e},
	{0xbe, 0x26},
	{0xbf, 0x7b},
	{0xc0, 0xac},
	{0xc1, 0x1e},
	{0xb7, 0x0c},
	{0xb8, 0x04},
	{0xb9, 0x00},
	{0xba, 0x04},
	{0x5a, 0x14},
	{0x5b, 0xa2},
	{0x5c, 0x70},
	{0x5d, 0x20},
	{0x24, 0x38},
	{0x25, 0x30},
	{0x26, 0x71},
	{0xa3, 0x0b},
	{0xa4, 0x15},
	{0xa5, 0x2a},
	{0xa6, 0x51},
	{0xa7, 0x63},
	{0xa8, 0x74},
	{0xa9, 0x83},
	{0xaa, 0x91},
	{0xab, 0x9e},
	{0xac, 0xaa},
	{0xad, 0xbe},
	{0xae, 0xce},
	{0xaf, 0xe5},
	{0xb0, 0xf3},
	{0xb1, 0xfb},
	{0xb2, 0x06},
	{0x8c, 0x5d},
	{0x8d, 0x11},
	{0x8e, 0x12},
	{0x8f, 0x11},
	{0x90, 0x50},
	{0x91, 0x22},
	{0x92, 0xd1},
	{0x93, 0xa7},
	{0x94, 0x23},
	{0x95, 0x3b},
	{0x96, 0xff},
	{0x97, 0x00},
	{0x98, 0x4a},
	{0x99, 0x46},
	{0x9a, 0x3d},
	{0x9b, 0x3a},
	{0x9c, 0xf0},
	{0x9d, 0xf0},
	{0x9e, 0xf0},
	{0x9f, 0xff},
	{0xa0, 0x56},
	{0xa1, 0x55},
	{0xa2, 0x13},
	{0x14, 0x20},
	{0x13, 0xf7},
	{0x11, 0x01},

	{0xFF, 0xFF}
};

static struct regval_list ov7690_fmt_yuv422[] = {

	{0xFF, 0xFF}
};

static struct regval_list ov7690_res_qcif[] = {
	/* 176 x 144 */
	{0xc8, 0x02},
	{0xc9, 0x80},
	{0xca, 0x01},
	{0xcb, 0xe0},
	{0xcc, 0x00},
	{0xcd, 0xB0},
	{0xce, 0x00},
	{0xcf, 0x90},

	{0xFF, 0xFF}
};
static struct regval_list ov7690_res_cif[] = {
	/* 352 x 288 */
	{0xc8, 0x02},
	{0xc9, 0x80},
	{0xca, 0x01},
	{0xcb, 0xe0},
	{0xcc, 0x01},
	{0xcd, 0x60},
	{0xce, 0x01},
	{0xcf, 0x20},

	{0xFF, 0xFF}
};

static struct regval_list ov7690_res_qvga[] = {
	/* 320 x 240 */
	{0xc8, 0x02},
	{0xc9, 0x80},
	{0xca, 0x01},
	{0xcb, 0xe0},
	{0xcc, 0x01},
	{0xcd, 0x40},
	{0xce, 0x00},
	{0xcf, 0xf0},

	{0xFF, 0xFF}
};

static struct regval_list ov7690_res_vga[] = {
	/* 640 x 480 */

	{0xc8, 0x02},
	{0xc9, 0x80},
	{0xca, 0x01},
	{0xcb, 0xe0},
	{0xcc, 0x02},
	{0xcd, 0x80},
	{0xce, 0x01},
	{0xcf, 0xe0},

	{0xFF, 0xFF}
};

struct ov7690_datafmt {
	enum v4l2_mbus_pixelcode	code;
	enum v4l2_colorspace		colorspace;
};

struct ov7690 {
	struct v4l2_subdev subdev;
	int model;	/* V4L2_IDENT_OV7690* codes from v4l2-chip-ident.h */
	struct v4l2_rect rect;
	u32 pixfmt;
	const struct ov7690_datafmt *curfmt;
	const struct ov7690_datafmt *fmts;
	int num_fmts;

	struct regval_list *regs_fmt;
	struct regval_list *regs_size;
};

static const struct ov7690_datafmt ov7690_colour_fmts[] = {
	{V4L2_MBUS_FMT_YUYV8_2X8_BE, V4L2_COLORSPACE_JPEG},
};

static struct ov7690_format_struct {
	enum v4l2_mbus_pixelcode	code;
	struct regval_list	*regs;
} ov7690_fmts[] = {
	{
		.code	=	V4L2_MBUS_FMT_YUYV8_2X8_LE,
		.regs	=	ov7690_fmt_yuv422,
	},{
		.code	=	V4L2_MBUS_FMT_YVYU8_2X8_LE,
		.regs	=	ov7690_fmt_yuv422,
	},{
		.code	=	V4L2_MBUS_FMT_YUYV8_2X8_BE,
		.regs	=	ov7690_fmt_yuv422,
	},{
		.code	=	V4L2_MBUS_FMT_YVYU8_2X8_BE,
		.regs	=	ov7690_fmt_yuv422,
	}
};

static struct ov7690_win_size {
	int	width;
	int	height;
	struct regval_list *regs;
} ov7690_sizes[] = {
	{
		.width	= 176,
		.height	= 144,
		.regs	= ov7690_res_qcif,/* QCIF */
	}, {
		.width	= 352,
		.height	= 288,
		.regs	= ov7690_res_cif,/* CIF */
	}, {
		.width	= 320,
		.height	= 240,
		.regs	= ov7690_res_qvga,/* QVGA */
	}, {
		.width	= 640,
		.height	= 480,
		.regs	= ov7690_res_vga,/* VGA */
	},
};

#define N_OV7690_SIZES (ARRAY_SIZE(ov7690_sizes))
#define N_OV7690_FMTS (ARRAY_SIZE(ov7690_fmts))
extern int to_mipi(u32 code, u32 width, u32 height);

static struct ov7690 *to_ov7690(const struct i2c_client *client)
{
	return container_of(i2c_get_clientdata(client), struct ov7690, subdev);
}

static int ov7690_read(struct i2c_client *c, unsigned char reg,
		unsigned char *value)
{
	u8 data;
	int ret;

	ret = i2c_smbus_write_byte(c,reg);
	data = i2c_smbus_read_byte(c);
	*value = data;
	return 0;
}

static int ov7690_write(struct i2c_client *c, unsigned char reg,
		unsigned char value)
{
	u8 data[2];
	data[0] = reg;
	data[1]=  value;
	i2c_master_send(c, data, 2);
	msleep(1);
	return 0;
}

static int ov7690_write_array(struct i2c_client *c, struct regval_list *vals)
{
	while (vals->reg_num != 0xff) {
		int ret = ov7690_write(c, vals->reg_num, vals->value);
		if (ret < 0)
		{
			/*abnormal state occur.*/
			return ret;
		}
		vals++;
	}
	return 0;
}

static int ov7690_detect(struct i2c_client *client)
{
	u8 v = 0;
	int ret = 0;

	ret = ov7690_read(client, 0x0a, &v);
	printk(KERN_NOTICE "cam: ov7690_detect 0x%x\n",v);
	if (ret < 0)
		return ret;
	if (v != 0x76)
		return -ENODEV;

	ret = ov7690_read(client, 0x0b, &v);
	printk(KERN_NOTICE "cam: ov7690_detect 0x%x\n",v);
	if (ret < 0)
		return ret;
	if (v != 0x91)
		return -ENODEV;

	return 0;
}

static int ov7690_get_awb(struct i2c_client *client, __s32 *value)
{
	return 0;
}

static int ov7690_set_awb(struct i2c_client *client, int value)
{
	return 0;
}

static int ov7690_get_ae(struct i2c_client *client, __s32 *value)
{
	return 0;
}

static int ov7690_set_ae(struct i2c_client *client, int value)
{
	return 0;
}

static int ov7690_get_af(struct i2c_client *client, __s32 *value)
{
	return 0;
}

static int ov7690_set_af(struct i2c_client *client, int value)
{
	return 0;
}

static const struct v4l2_queryctrl ov7690_controls[] = {
	{
		.id = V4L2_CID_FOCUS_AUTO,
		.type = V4L2_CTRL_TYPE_BOOLEAN,
		.name = "auto focus",
		.minimum = 0,
		.maximum = 1,
		.step = 1,
		.default_value = 0,
	},{
		.id = V4L2_CID_AUTO_WHITE_BALANCE,
		.type = V4L2_CTRL_TYPE_BOOLEAN,
		.name = "auto white balance",
		.minimum = 0,
		.maximum = 1,
		.step = 1,
		.default_value = 0,
	},{
		.id = V4L2_CID_EXPOSURE_AUTO,
		.type = V4L2_CTRL_TYPE_BOOLEAN,
		.name = "auto exposure",
		.minimum = 0,
		.maximum = 1,
		.step = 1,
		.default_value = 0,
	}
};

static unsigned long ov7690_query_bus_param(struct soc_camera_device *icd)
{
	struct soc_camera_link *icl = to_soc_camera_link(icd);
	unsigned long flags = SOCAM_MIPI | SOCAM_MIPI_1LANE;

	return soc_camera_apply_sensor_flags(icl, flags);
}

static int ov7690_set_bus_param(struct soc_camera_device *icd, unsigned long f)
{
#if 0/*TODO: add mipi and parallel different setting*/
	if (f & SOCAM_MIPI) /* mipi setting*/
		ov7690_write_array(client, ov7690_mipi);
	else /* parallel setting*/
		ov7690_write_array(client, ov7690_mipi);
#endif
	return 0;
}

static struct soc_camera_ops ov7690_ops = {
	.query_bus_param	= ov7690_query_bus_param,
	.set_bus_param		= ov7690_set_bus_param,
	.controls			= ov7690_controls,
	.num_controls		= ARRAY_SIZE(ov7690_controls),
};

static int ov7690_s_ctrl(struct v4l2_subdev *sd, struct v4l2_control *ctrl)
{
	struct i2c_client *client = sd->priv;
	const struct v4l2_queryctrl *qctrl;
	int ret;

	qctrl = soc_camera_find_qctrl(&ov7690_ops, ctrl->id);
	if (!qctrl)
		return -EINVAL;

	switch (ctrl->id) {
	case V4L2_CID_AUTO_WHITE_BALANCE:
		ret = ov7690_set_awb(client, ctrl->value);
		break;
	case V4L2_CID_FOCUS_AUTO:
		ret = ov7690_set_af(client, ctrl->value);
		break;
	case V4L2_CID_EXPOSURE_AUTO:
		ret = ov7690_set_ae(client, ctrl->value);
		break;
	default:
		ret = -EINVAL;
	}

	return ret;
}

static int ov7690_g_ctrl(struct v4l2_subdev *sd, struct v4l2_control *ctrl)
{
	struct i2c_client *client = sd->priv;
	int ret;

	switch (ctrl->id) {
	case V4L2_CID_AUTO_WHITE_BALANCE:
		ret = ov7690_get_awb(client, &ctrl->value);
		break;
	case V4L2_CID_FOCUS_AUTO:
		ret = ov7690_get_af(client, &ctrl->value);
		break;
	case V4L2_CID_EXPOSURE_AUTO:
		ret = ov7690_get_ae(client, &ctrl->value);
		break;
	default:
		ret = -EINVAL;
	}

	return ret;
}

static int ov7690_g_chip_ident(struct v4l2_subdev *sd,
				struct v4l2_dbg_chip_ident *id)
{
	struct i2c_client *client = sd->priv;
	struct ov7690 *ov7690 = to_ov7690(client);
	id->ident		= ov7690->model;
	id->revision	= 0;

	return 0;
}

static int ov7690_g_register(struct v4l2_subdev *sd,
			      struct v4l2_dbg_register *reg)
{
	struct i2c_client *client = sd->priv;
	return ov7690_read(client, reg->reg, (unsigned char *)&(reg->val));
}

static int ov7690_s_register(struct v4l2_subdev *sd,
			      struct v4l2_dbg_register *reg)
{
	struct i2c_client *client = sd->priv;
	return ov7690_write(client, reg->reg, (unsigned char)reg->val);
}

static int ov7690_s_stream(struct v4l2_subdev *sd, int enable)
{
	return 0;
}

static int ov7690_g_fmt(struct v4l2_subdev *sd,
			 struct v4l2_mbus_framefmt *mf)
{
	struct i2c_client *client = sd->priv;
	struct ov7690 *ov7690 = to_ov7690(client);

	mf->width		= ov7690->rect.width;
	mf->height		= ov7690->rect.height;
	mf->code	= V4L2_MBUS_FMT_YUYV8_2X8_BE;
	mf->field		= V4L2_FIELD_NONE;
	mf->colorspace		= V4L2_COLORSPACE_JPEG;
	return 0;
}

static int ov7690_s_fmt(struct v4l2_subdev *sd,
			 struct v4l2_mbus_framefmt *mf)
{
	int ret = 0;
	u8 val;
	struct i2c_client *client = sd->priv;

	switch (mf->code) {
		case V4L2_MBUS_FMT_YUYV8_2X8_LE:
		case V4L2_MBUS_FMT_YVYU8_2X8_LE:
		case V4L2_MBUS_FMT_YUYV8_2X8_BE:
		case V4L2_MBUS_FMT_YVYU8_2X8_BE:
			ov7690_read(client, 0x49, &val);
			val = 0x0c;
			ov7690_write(client, 0x49, val);
			/*read DOVDD IO voltage setting*/
			ov7690_read(client, 0x49, &val);

			/*Per OV suggest, config OV7690 first, then configure OV5642 bridge mode setting*/
			ov7690_write_array(client, ov7690_default);
			msleep(1);
			to_mipi(mf->code, mf->width, mf->height);
			break;

		default:
			printk(KERN_ERR "cam: not supported fmt!\n");
			ret = -EINVAL;
			break;
	}

	return ret;
}

static int ov7690_enum_fmt(struct v4l2_subdev *sd, unsigned int index,
			    enum v4l2_mbus_pixelcode *code)
{
	struct i2c_client *client = sd->priv;
	struct ov7690 *ov7690 = to_ov7690(client);

	if (index >= ov7690->num_fmts)
		return -EINVAL;

	*code = ov7690->fmts[index].code;
	return 0;
}

static int ov7690_try_fmt(struct v4l2_subdev *sd,
			   struct v4l2_mbus_framefmt *mf)
{
	int i;
	struct i2c_client *client = sd->priv;
	struct ov7690 *ov7690 = to_ov7690(client);

	/* enum the supported formats*/
	for (i = 0; i < N_OV7690_FMTS; i++) {
		if (ov7690_fmts[i].code == mf->code){
			ov7690->regs_fmt = ov7690_fmts[i].regs;
			break;
		}
	}
	if (i >= N_OV7690_FMTS){
		printk(KERN_ERR "cam: ov7690 unsupported color format!\n");
		return -EINVAL;
	}

	/* enum the supported sizes*/
	for (i = 0; i < N_OV7690_SIZES; i++) {
		if (mf->width == ov7690_sizes[i].width
			&& mf->height == ov7690_sizes[i].height) {
			ov7690->regs_size = ov7690_sizes[i].regs;
			break;
		}
	}
	if (i >= N_OV7690_SIZES){
		printk(KERN_ERR "cam: ov7690 unsupported window size, w%d, h%d!\n",
				mf->width, mf->height);
		return -EINVAL;
	}

	mf->field = V4L2_FIELD_NONE;
	mf->colorspace = V4L2_COLORSPACE_JPEG;
	return 0;
}

static struct v4l2_subdev_core_ops ov7690_subdev_core_ops = {
	.g_ctrl		= ov7690_g_ctrl,
	.s_ctrl		= ov7690_s_ctrl,
	.g_chip_ident	= ov7690_g_chip_ident,
#ifdef CONFIG_VIDEO_ADV_DEBUG
	.g_register	= ov7690_g_register,
	.s_register	= ov7690_s_register,
#endif
};

static struct v4l2_subdev_video_ops ov7690_subdev_video_ops = {
	.s_stream	= ov7690_s_stream,
	.s_mbus_fmt	= ov7690_s_fmt,
	.g_mbus_fmt	= ov7690_g_fmt,
	.try_mbus_fmt	= ov7690_try_fmt,
	.enum_mbus_fmt	= ov7690_enum_fmt,
};

static struct v4l2_subdev_ops ov7690_subdev_ops = {
	.core	= &ov7690_subdev_core_ops,
	.video	= &ov7690_subdev_video_ops,
};

static int ov7690_probe(struct i2c_client *client,
			 const struct i2c_device_id *did)
{
	struct ov7690 *ov7690;
	struct soc_camera_device *icd = client->dev.platform_data;
	struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);
	struct soc_camera_link *icl;
	int ret;

	if (!icd) {
		dev_err(&client->dev, "ov7690 missing soc-camera data!\n");
		return -EINVAL;
	}
	if (!icd->dev.parent ||
	    to_soc_camera_host(icd->dev.parent)->nr != icd->iface)
		return -ENODEV;

	icl = to_soc_camera_link(icd);
	if (!icl) {
		dev_err(&client->dev, "ov7690 driver needs platform data\n");
		return -EINVAL;
	}

	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_BYTE_DATA)) {
		dev_warn(&adapter->dev,
			 "I2C-Adapter doesn't support I2C_FUNC_SMBUS_WORD\n");
		return -EIO;
	}

	ov7690 = kzalloc(sizeof(struct ov7690), GFP_KERNEL);
	if (!ov7690) {
		dev_err(&client->dev, "ov7690 failed to alloc struct!\n");
		return -ENOMEM;
	}

	ov7690->rect.left = 0;
	ov7690->rect.top = 0;
	ov7690->rect.width = 320;
	ov7690->rect.height = 240;
	ov7690->pixfmt = V4L2_PIX_FMT_UYVY;

	icd->ops = &ov7690_ops;

	ov7690->model = V4L2_IDENT_OV7690;
	ov7690->fmts = ov7690_colour_fmts;
	ov7690->num_fmts = ARRAY_SIZE(ov7690_colour_fmts);

	v4l2_i2c_subdev_init(&ov7690->subdev, client, &ov7690_subdev_ops);

	ret = ov7690_detect(client);
	if (!ret) {
		printk(KERN_NOTICE "cam: OmniVision ov7690 sensor detected!\n");
		return 0;
	}
	printk(KERN_ERR "cam: failed to detect OmniVision ov7690!\n");

	icd->ops = NULL;
	i2c_set_clientdata(client, NULL);
	if (ov7690)
		kfree(ov7690);

	return -ENODEV;;
}

static int ov7690_remove(struct i2c_client *client)
{
	struct ov7690 *ov7690 = to_ov7690(client);
	struct soc_camera_device *icd = client->dev.platform_data;
	struct soc_camera_link *icl = to_soc_camera_link(icd);

	icd->ops = NULL;
	if (icl->free_bus)
		icl->free_bus(icl);
	icl->power(icd->pdev, 0);

	i2c_set_clientdata(client, NULL);
	client->driver = NULL;
	kfree(ov7690);
	return 0;
}

static struct i2c_device_id ov7690_idtable[] = {
	{ "ov7690", 0 },
	{ }
};

MODULE_DEVICE_TABLE(i2c, ov7690_idtable);

static struct i2c_driver ov7690_driver = {
	.driver = {
		.name	= "ov7690",
	},
	.id_table	= ov7690_idtable,
	.probe		= ov7690_probe,
	.remove		= ov7690_remove,
};

static int __init ov7690_mod_init(void)
{
	int ret = 0;
	ret = i2c_add_driver(&ov7690_driver);
	return ret;
}

static void __exit ov7690_mod_exit(void)
{
	i2c_del_driver(&ov7690_driver);
}

module_init(ov7690_mod_init);
module_exit(ov7690_mod_exit);

MODULE_DESCRIPTION("OmniVision OV7690 Camera Driver");
MODULE_AUTHOR("Qing Xu");
MODULE_LICENSE("GPL");

