#include <linux/init.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/videodev.h>
#include <media/v4l2-common.h>
#include <media/v4l2-chip-ident.h>
#include <linux/i2c.h>
#include <mach/hardware.h>
#include <mach/camera.h>
#include <mach/mfp.h>

#include <linux/clk.h>
#include "pxa910_camera.h"

#include "ov5642_mipi.h"

MODULE_AUTHOR("ygli@marvell.com");
MODULE_DESCRIPTION("OmniVision ov5642 sensors");
MODULE_LICENSE("GPL");

struct ov5642_mipi_info
{
  struct ov5642_mipi_format_struct *fmt;  /* Current format */
  unsigned char sat;		/* Saturation value */
  int hue;			/* Hue value */
};

static int ov5642_mipi_read(struct i2c_client *c, u16 reg,unsigned char *value)
{
	u8 data;
	u8 address[2];
	int ret = 0;
	address[0] = reg>>8;
	address[1] = reg;
	ret = i2c_master_send(c, address, 2);
	if(ret < 0)
		goto out;
	ret = i2c_master_recv(c, &data, 1);
	if(ret < 0)
		goto out;
	*value = data;
out:
	return (ret < 0) ? ret: 0;

}

static int ov5642_mipi_write(struct i2c_client *c, u16 reg,unsigned char value)
{
	u8 data[3];
	int ret = 0;
	data[0] = reg>>8;
	data[1] = reg;
	data[2]=  value;
	ret = i2c_master_send(c, data, 3);
	return (ret < 0) ? ret: 0;
}

/*
 * Write a list of register settings; ff/ff stops the process.
 */
static int ov5642_mipi_write_array(struct i2c_client *c, OV5642_WREG   *vals)
{
	int i = 0;
	int ret = 0;
	while (vals->reg_num != OV5642_DVP_END_ADDR|| vals->value != OV5642_DVP_END_VAL)
	{
		ret = ov5642_mipi_write(c, vals->reg_num, vals->value);
		if (ret < 0)
			return ret;
		vals++;
		i++;
	}
	return ret;

}
static void ov5642_mipi_reset(struct i2c_client *client)
{
	msleep(1);
}

static int ov5642_mipi_detect(struct i2c_client *client)
{
	unsigned char v = 0;
	int ret = 0;

	/*
	 * no MID register found. OK, we know we have an OmniVision chip...but which one?
	 */
	ret = ov5642_mipi_read(client, 0x300a, &v);
	if (ret < 0)
		return ret;
	if (v != 0x56)
		return -ENODEV;

	ret = ov5642_mipi_read(client, 0x300b, &v);

	if (ret < 0)
		return ret;
	if (v != 0x42)
		return -ENODEV;
	printk(KERN_NOTICE "camera: ov5642 detected 0x%x\n",v);
	return 0;
}

static struct ov5642_mipi_format_struct
{
	__u8 *desc;
	__u32 pixelformat;
	int bpp;   /* bits per pixel */
} ov5642_mipi_formats[] = {

	{
		.desc		= "UYVY 4:2:2",
		.pixelformat	= V4L2_PIX_FMT_UYVY,
		.bpp		= 16,
	},
	{
		.desc		= "YUYV422 planar",
		.pixelformat	= V4L2_PIX_FMT_YUV422P,
		.bpp		= 16,
	},
	{
		.desc           = "YUYV 4:2:0",
		.pixelformat    = V4L2_PIX_FMT_YUV420,
		.bpp            = 12,
	},
	{
		.desc           = "JFIF JPEG",
		.pixelformat    = V4L2_PIX_FMT_JPEG,
		.bpp            = 16,
	}
};

/* preview size */
static struct ov5642_mipi_win_size {
	int	width;
	int	height;
} ov5642_mipi_win_sizes[] = {

	/* QCIF */
	{
		.width		= QCIF_WIDTH,
		.height 	= QCIF_HEIGHT,
	},

	/* QVGA */
	{
		.width		= QVGA_WIDTH,
		.height		= QVGA_HEIGHT,
	},

	/* VGA */
	{
		.width		= VGA_WIDTH,
		.height		= VGA_HEIGHT,
	},
	/* D1 size  */
	{
		.width          = 720,
		.height         = 480,

	},
};


/* capture jpeg size */
static struct ov5642_mipi_win_size ov5642_mipi_win_sizes_jpeg[] = {
	/* full */
	{
		.width = 2592,
		.height = 1944,
	},
	{
		.width = 640,
		.height = 480,
	},
};

static int ov5642_mipi_querycap(struct i2c_client *c, struct v4l2_capability *argp)
{
	if(!argp){
		printk(KERN_ERR" argp is NULL %s %d \n", __FUNCTION__, __LINE__);
		return -EINVAL;
	}
	strcpy(argp->driver, "ov5642");
	strcpy(argp->card, "TD/TTC");
	return 0;
}

static int ov5642_mipi_enum_fmt(struct i2c_client *c, struct v4l2_fmtdesc *fmt)
{
	struct ov5642_mipi_format_struct *ofmt;

	if (fmt->index >= ARRAY_SIZE(ov5642_mipi_formats))
        {
		printk("NO such fmt->index\n");
		return -EINVAL;
	}
	ofmt = ov5642_mipi_formats + fmt->index;
	fmt->flags = 0;
	strcpy(fmt->description, ofmt->desc);
	fmt->pixelformat = ofmt->pixelformat;
	return 0;
}

static int ov5642_mipi_enum_fmsize(struct i2c_client *c, struct v4l2_frmsizeenum *argp)
{
	struct v4l2_frmsizeenum frmsize;

	if (copy_from_user(&frmsize, argp, sizeof(frmsize)))
		   return -EFAULT;

	if (frmsize.pixel_format == V4L2_PIX_FMT_YUV420 ||
	frmsize.pixel_format == V4L2_PIX_FMT_UYVY ||
	frmsize.pixel_format == V4L2_PIX_FMT_YUV422P){
		if (frmsize.index >= (ARRAY_SIZE(ov5642_mipi_win_sizes))){
			printk(KERN_ERR" \n max index for preview is %d \n", ARRAY_SIZE(ov5642_mipi_win_sizes));
		    return -EINVAL;
		}
		frmsize.type = V4L2_FRMSIZE_TYPE_DISCRETE;
		frmsize.discrete.height = ov5642_mipi_win_sizes[frmsize.index].height;
		frmsize.discrete.width = ov5642_mipi_win_sizes[frmsize.index].width;
	}else if(frmsize.pixel_format == V4L2_PIX_FMT_JPEG){
		if (frmsize.index >= ARRAY_SIZE(ov5642_mipi_win_sizes_jpeg)){
			   printk(KERN_ERR" \n max index for jpeg  is %d \n", ARRAY_SIZE(ov5642_mipi_win_sizes_jpeg));
			   return -EINVAL;
		}
		frmsize.type = V4L2_FRMSIZE_TYPE_DISCRETE;
		frmsize.discrete.height = ov5642_mipi_win_sizes_jpeg[frmsize.index].height;
		frmsize.discrete.width = ov5642_mipi_win_sizes_jpeg[frmsize.index].width;

	}else
	   return -EINVAL;

	if (copy_to_user(argp, &frmsize, sizeof(frmsize)))
		   return -EFAULT;
	return 0;
}

static int ov5642_mipi_try_fmt(struct i2c_client *c, struct v4l2_format *fmt)
{
	int index = 0;
	int i = 0;
	struct v4l2_pix_format *pix = &fmt->fmt.pix;
	for (index = 0; index < ARRAY_SIZE(ov5642_mipi_formats); index++)
		if (ov5642_mipi_formats[index].pixelformat == pix->pixelformat)
			break;
	if (index >= ARRAY_SIZE(ov5642_mipi_formats))
	{
		printk(KERN_ERR"unsupported format! %s %d \n", __FUNCTION__, __LINE__);
		return -EINVAL;
	}

	if (pix->field == V4L2_FIELD_ANY)
		pix->field = V4L2_FIELD_NONE;
	else if (pix->field != V4L2_FIELD_NONE)
	{
		printk(KERN_ERR"pix->filed != V4l2_FIELD_NONE\n");
		return -EINVAL;
	}
	if(pix->pixelformat == V4L2_PIX_FMT_JPEG){
		for (i = 0; i < ARRAY_SIZE(ov5642_mipi_win_sizes_jpeg); i++)
			if (pix->width == ov5642_mipi_win_sizes_jpeg[i].width && pix->height == ov5642_mipi_win_sizes_jpeg[i].height)
				break;

		if (i >= ARRAY_SIZE(ov5642_mipi_win_sizes_jpeg)){
			printk(KERN_ERR"invalid size request for jpeg! %d %d  \n",
				pix->width, pix->height);
				return -EINVAL;
			}
		/* for OV5642, HSYNC contains 2048bytes */
		pix->bytesperline = 2048;
	}else{
		for (i = 0; i < (ARRAY_SIZE(ov5642_mipi_win_sizes));i ++)
			if (pix->width == ov5642_mipi_win_sizes[i].width && pix->height == ov5642_mipi_win_sizes[i].height)
				break;

		if (i>= (ARRAY_SIZE(ov5642_mipi_win_sizes))){
			printk(KERN_ERR"invalid size request for preview! %d %d  \n",
				pix->width, pix->height);
				return -EINVAL;
			}
		pix->bytesperline = pix->width*ov5642_mipi_formats[index].bpp/8;
		pix->sizeimage = pix->height*pix->bytesperline;
	}
	return 0;
}

/*
 * Set a format.
 */
static int ov5642_mipi_s_fmt(struct i2c_client *c, struct v4l2_format *fmt)
{
	int ret;
	OV5642_WREG    *pregs;
	struct v4l2_pix_format *pix = &fmt->fmt.pix;
	ret = ov5642_mipi_try_fmt(c, fmt);
	if (ret == -EINVAL)
	{
		printk("try fmt error\n");
		return ret;
	}

	switch(pix->pixelformat)
	{
		case V4L2_PIX_FMT_UYVY:
		case V4L2_PIX_FMT_YUV422P:
		case V4L2_PIX_FMT_YUV420:
			switch (pix->width )
			{
				case QCIF_WIDTH:
					pregs = yuv_QCIF_tab;
					printk("choose qcif setting \n");
					break;
				case QVGA_WIDTH:
					pregs = yuv_QVGA_tab;
					printk("choose qvga setting \n");
					break;
				case VGA_WIDTH:
					pregs = yuv_VGA_tab;
					printk("choose vga setting \n");
					break;
				case 720:
					pregs = yuv_D1_tab;
					printk("choose D1 setting \n");
					break;
				default:
					printk("\n unsupported YUV size %s %d !\n", __FUNCTION__, __LINE__);
					ret = -EINVAL;
					goto out;
					break;
			}
			break;
		case V4L2_PIX_FMT_JPEG:
			switch (pix->width)
			{
				case VGA_WIDTH:
					pregs = jpg_VGA_tab;
					printk("choose vga jpeg setting \n");
					break;
				case 2592:
					pregs = jpg_FULL_tab;
					printk("choose 5M jpeg setting \n");
					break;
				default:
					printk("\n unsupported JPEG size!\n");
					ret = -EINVAL;
					goto out;
					break;
			}
			break;
		default:
			printk("\n unsupported format! %s %d\n", __FUNCTION__, __LINE__);
			ret = -EINVAL;
			break;
	}

	ret = ov5642_mipi_write_array(c, pregs);
out:
	return ret;
}

static int ov5642_mipi_g_parm(struct i2c_client *c, struct v4l2_streamparm *parms)
{
	return 0;
}

static int ov5642_firmware_download(struct i2c_client *client,  __s32  value)
{
	int ret, i, size, j;
	char data[258];
	printk("OV5642 AF firmware download\n");
	size = ARRAY_SIZE(firmware_regs);
	for (i = 0; i < size; i++){
		data[0] = firmware_regs[i].reg_base>>8;
		data[1] = firmware_regs[i].reg_base;
		for (j = 0; j < firmware_regs[i].len; j++)
			data[j+2]= firmware_regs[i].value[j];
		ret = i2c_master_send(client, data, firmware_regs[i].len+2);
		if (ret < 0) {
			printk("I2C send error: i: %d--ret : %d\n",i, ret);
			break;
		}
	}
	return (ret > 0) ? 0 : ret;
}

static int ov5642_mipi_s_parm(struct i2c_client *c, struct v4l2_streamparm *parms)
{
	return 0;
}

static int ov5642_mipi_s_input(struct i2c_client *c, int *id)
{
	return ov5642_mipi_write_array(c, init_global_tab);
}


/*
 * Hue also requires messing with the color matrix.  It also requires
 * trig functions, which tend not to be well supported in the kernel.
 * So here is a simple table of sine values, 0-90 degrees, in steps
 * of five degrees.  Values are multiplied by 1000.
 *
 * The following naive approximate trig functions require an argument
 * carefully limited to -180 <= theta <= 180.
 */

static int g_brightness;
static int g_hue;
static int g_saturation;
static int g_contrast;
static int g_whitebalance;
static int g_exposure;

static int ov5642_mipi_t_sat(struct i2c_client *client, int value)
{
	return 0;
}

static int ov5642_mipi_q_sat(struct i2c_client *client, __s32 *value)
{
	*value = g_saturation;
	return 0;
}

static int ov5642_mipi_t_hue(struct i2c_client *client, int value)
{
	return 0;
}


static int ov5642_mipi_q_hue(struct i2c_client *client, __s32 *value)
{
	*value = g_hue;
	return 0;
}

static int ov5642_mipi_t_brightness(struct i2c_client *client, int value)
{
	return 0;
}

static int ov5642_mipi_q_brightness(struct i2c_client *client, __s32 *value)
{
	*value = g_brightness;
	return 0;
}

static int ov5642_mipi_t_contrast(struct i2c_client *client, int value)
{
	return 0;
}

static int ov5642_mipi_q_contrast(struct i2c_client *client, __s32 *value)
{
	*value = g_contrast;
	return 0;
}

static int ov5642_mipi_t_whitebalance(struct i2c_client *client, int value)
{
	return 0;
}

static int ov5642_mipi_q_whitebalance(struct i2c_client *client, __s32 *value)
{
	*value = g_whitebalance;
	return 0;
}

static int ov5642_mipi_t_exposure(struct i2c_client *client, int value)
{
	return 0;
}

static int ov5642_mipi_q_exposure(struct i2c_client *client, __s32 *value)
{
	*value = g_exposure;
	return 0;
}


static struct ov5642_mipi_control {
	struct v4l2_queryctrl qc;
	int (*query)(struct i2c_client *c, __s32 *value);
	int (*tweak)(struct i2c_client *c, int value);
} ov5642_mipi_controls[] =
{
	{
		.qc = {
			.id = V4L2_CID_BRIGHTNESS,
			.type = V4L2_CTRL_TYPE_INTEGER,
			.name = "Brightness",
			.minimum = 0,
			.maximum = 4,
			.step = 1,
			.default_value = 2,
			.flags = V4L2_CTRL_FLAG_SLIDER
		},
		.tweak = ov5642_mipi_t_brightness,
		.query = ov5642_mipi_q_brightness,
	},
	{
		.qc = {
			.id = V4L2_CID_CONTRAST,
			.type = V4L2_CTRL_TYPE_INTEGER,
			.name = "Contrast",
			.minimum = 0,
			.maximum = 4,
			.step = 1,
			.default_value = 2,   /* XXX ov5642_mipi spec */
			.flags = V4L2_CTRL_FLAG_SLIDER
		},
		.tweak = ov5642_mipi_t_contrast,
		.query = ov5642_mipi_q_contrast,
	},
	{
		.qc = {
			.id = V4L2_CID_SATURATION,
			.type = V4L2_CTRL_TYPE_INTEGER,
			.name = "Saturation",
			.minimum = 0,
			.maximum = 4,
			.step = 1,
			.default_value = 2,
			.flags = V4L2_CTRL_FLAG_SLIDER
		},
		.tweak = ov5642_mipi_t_sat,
		.query = ov5642_mipi_q_sat,
	},
	{
		.qc = {
			.id = V4L2_CID_HUE,
			.type = V4L2_CTRL_TYPE_INTEGER,
			.name = "HUE",
			.minimum = 0,
			.maximum = 3,
			.step = 1,
			.default_value = 0,
			.flags = V4L2_CTRL_FLAG_SLIDER
		},
		.tweak = ov5642_mipi_t_hue,
		.query = ov5642_mipi_q_hue,
	},
	{
		.qc = {
			.id = V4L2_CID_DO_WHITE_BALANCE,
			.type = V4L2_CTRL_TYPE_INTEGER,
			.name = "white balance",
			.minimum = 0,
			.maximum = 4,
			.step = 1,
			.default_value = 2,
			.flags = V4L2_CTRL_FLAG_SLIDER
		},
		.tweak = ov5642_mipi_t_whitebalance,
		.query = ov5642_mipi_q_whitebalance,
	},
	{
		.qc = {
			.id = V4L2_CID_EXPOSURE,
			.type = V4L2_CTRL_TYPE_INTEGER,
			.name = "exposure",
			.minimum = 0,
			.maximum = 4,
			.step = 1,
			.default_value = 2,
			.flags = V4L2_CTRL_FLAG_SLIDER
		},
		.tweak = ov5642_mipi_t_exposure,
		.query = ov5642_mipi_q_exposure,
	},
	{
		.qc = {
			.id = V4L2_CID_AF_FIRMWARE_DOWNLOAD,
		},
		.tweak = ov5642_firmware_download,
	},
};
#define N_CONTROLS (ARRAY_SIZE(ov5642_mipi_controls))

static struct ov5642_mipi_control *ov5642_mipi_find_control(__u32 id)
{
	int i;

	for (i = 0; i < N_CONTROLS; i++)
		if (ov5642_mipi_controls[i].qc.id == id)
			return ov5642_mipi_controls + i;
	return NULL;
}


static int ov5642_mipi_queryctrl(struct i2c_client *client,
		struct v4l2_queryctrl *qc)
{
	struct ov5642_mipi_control *ctrl = ov5642_mipi_find_control(qc->id);

	if (ctrl == NULL)
		return -EINVAL;
	*qc = ctrl->qc;
	return 0;
}

static int ov5642_mipi_g_ctrl(struct i2c_client *client, struct v4l2_control *ctrl)
{
	struct ov5642_mipi_control *octrl = ov5642_mipi_find_control(ctrl->id);
	int ret;

	if (octrl == NULL)
		return -EINVAL;
	ret = octrl->query(client, &ctrl->value);
	if (ret >= 0)
		return 0;
	return ret;
}

static int ov5642_mipi_s_ctrl(struct i2c_client *client, struct v4l2_control *ctrl)
{
	struct ov5642_mipi_control *octrl = ov5642_mipi_find_control(ctrl->id);
	int ret;

	if (octrl == NULL)
		return -EINVAL;
	ret =  octrl->tweak(client, ctrl->value);
	if (ret >= 0)
		return 0;
	return ret;
}

/*
 * Basic i2c stuff.
 */
static int __devinit ov5642_mipi_probe(struct i2c_client *client, const struct i2c_device_id * i2c_id)
{
	int ret = 0;
	struct ov5642_mipi_info *info;
	struct sensor_platform_data *pdata;
	pdata = client->dev.platform_data;
	client->addr = 0x3C;

	ccic_set_clock_mipi();

	pdata->power_on(1, 1);
	/*
	 * Set up our info structure.
	 */
	info = kzalloc(sizeof (struct ov5642_mipi_info), GFP_KERNEL);
	if (! info) {
		ret = -ENOMEM;
		goto out_free;
	}
	info->fmt = &ov5642_mipi_formats[1];
	info->sat = 128;
	i2c_set_clientdata(client, info);

	ret = ov5642_mipi_detect(client);
	if (ret)
		goto out_free;
	printk(KERN_ERR "OmniVision ov5642_mipi sensor detected\n");
	ret = ccic_sensor_attach(client);
	if (ret)
		goto out_free;

out_free:
	pdata->power_on(0, 1);
	ccic_disable_clock();

	if(ret)
		kfree(info);
	return ret;
}


static int ov5642_mipi_remove(struct i2c_client *client)
{
	return 0;	//TODO
}

static int ov5642_mipi_streamon(struct i2c_client *client)
{
	int ret = 0;
	ret = ov5642_mipi_write(client, 0x4201, 0x00);
	if(ret < 0)
		goto out;
	ret = ov5642_mipi_write(client, 0x4202, 0x00);
	if(ret < 0)
		goto out;
out:
	return ret;
}


static int ov5642_mipi_streamoff(struct i2c_client *client)
{
	int ret = 0;
	ret = ov5642_mipi_write(client, 0x4201, 0x01);
	if(ret < 0)
		goto out;
	ret = ov5642_mipi_write(client, 0x4202, 0x00);
	if(ret < 0)
		goto out;
out:
	return ret;
}

static int ov5642_mipi_g_register(struct i2c_client *client, struct v4l2_dbg_register * reg)
{
	return ov5642_mipi_read(client, (u16)reg->reg, (unsigned char *)&(reg->val));
}

static int ov5642_mipi_s_register(struct i2c_client *client, struct v4l2_dbg_register * reg)
{
	return ov5642_mipi_write(client, (u16)reg->reg, (unsigned char)reg->val);
}

static int ov5642_mipi_command(struct i2c_client *client, unsigned int cmd,	void *arg)
{
	switch (cmd)
	{
		case VIDIOC_DBG_G_CHIP_IDENT:
			return v4l2_chip_ident_i2c_client(client, arg, V4L2_IDENT_OV5642, 0);
		case VIDIOC_INT_RESET:
			ov5642_mipi_reset(client);
			return 0;
		case VIDIOC_ENUM_FRAMESIZES:
			return ov5642_mipi_enum_fmsize(client, (struct v4l2_frmsizeenum *) arg);
		case VIDIOC_QUERYCAP:
			return ov5642_mipi_querycap(client, (struct v4l2_capability *) arg);
		//case VIDIOC_INT_INIT:

		//	return 0;//ov5642_mipi_init(client);		//TODO - should get 5642 default register values
		case VIDIOC_ENUM_FMT:
			return ov5642_mipi_enum_fmt(client, (struct v4l2_fmtdesc *) arg);
		case VIDIOC_TRY_FMT:
			return ov5642_mipi_try_fmt(client, (struct v4l2_format *) arg);
		case VIDIOC_S_FMT:
			return ov5642_mipi_s_fmt(client, (struct v4l2_format *) arg);
		case VIDIOC_QUERYCTRL:
			return ov5642_mipi_queryctrl(client, (struct v4l2_queryctrl *) arg);
		case VIDIOC_S_CTRL:
			return ov5642_mipi_s_ctrl(client, (struct v4l2_control *) arg);
		case VIDIOC_G_CTRL:
			return ov5642_mipi_g_ctrl(client, (struct v4l2_control *) arg);
		case VIDIOC_S_PARM:
			return ov5642_mipi_s_parm(client, (struct v4l2_streamparm *) arg);
		case VIDIOC_G_PARM:
			return ov5642_mipi_g_parm(client, (struct v4l2_streamparm *) arg);
		case VIDIOC_S_INPUT:
			return ov5642_mipi_s_input(client, (int *) arg);
		case VIDIOC_STREAMON:
			return ov5642_mipi_streamon(client);
		case VIDIOC_STREAMOFF:
			return ov5642_mipi_streamoff(client);
		case VIDIOC_DBG_G_REGISTER:
			return ov5642_mipi_g_register(client, (struct v4l2_dbg_register *) arg);
		case VIDIOC_DBG_S_REGISTER:
			return ov5642_mipi_s_register(client, (struct v4l2_dbg_register *) arg);
	}
	return -EINVAL;
}

static struct i2c_device_id ov5642_mipi_idtable[] = {
	{ "ov5642", 0 },
	{ }
};

MODULE_DEVICE_TABLE(i2c, ov5642_mipi_idtable);

static struct i2c_driver ov5642_mipi_driver =
{
	.driver = {
		.name	= "ov5642",
	},
	.id_table         = ov5642_mipi_idtable,
	.command	= ov5642_mipi_command,
	.probe		= ov5642_mipi_probe,
	.remove		= ov5642_mipi_remove,
};

/*
 * Module initialization
 */
static int __init ov5642_mipi_mod_init(void)
{
	printk(KERN_ERR "OmniVision ov5642_mipi sensor driver at your service\n");
	return i2c_add_driver(&ov5642_mipi_driver);
}

static void __exit ov5642_mipi_mod_exit(void)
{
	i2c_del_driver(&ov5642_mipi_driver);
}

late_initcall(ov5642_mipi_mod_init);
module_exit(ov5642_mipi_mod_exit);
