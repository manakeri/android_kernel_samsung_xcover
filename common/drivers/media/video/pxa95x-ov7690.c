/*
 * A V4L2 driver for OmniVision OV7690 cameras.
 */
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
MODULE_DESCRIPTION("A low-level driver for OmniVision ov7690 sensors");
MODULE_LICENSE("GPL");


/*
 * Basic window sizes.  These probably belong somewhere more globally
 * useful.
 */

/*for OV7690 porting*/
#define REG_PIDH	0x0a
#define REG_PIDL	0x0b
#define REG_SYS		0x12
#define SYS_RESET	0x80

/*
 * Information we maintain about a known sensor.
 */
struct ov7690_format_struct;  /* coming later */
struct ov7690_info {
	struct ov7690_format_struct *fmt;  /* Current format */
	unsigned char sat;		/* Saturation value */
	int hue;			/* Hue value */
};

/*
 * The default register settings, as obtained from OmniVision.  There
 * is really no making sense of most of these - lots of "reserved" values
 * and such.
 *
 * These settings give VGA YUYV.
 */


struct regval_list {
	u8 reg_num;
	unsigned char value;
};

static struct regval_list ov7690_Set[] = {
	{0x12, 0x80},
	{0x0c, 0x16},
	{0x48, 0x42},
	{0x41, 0x43},
	{0x4c, 0x73},
	{0x81, 0xff},
	{0xD8, 0x70},
	{0xD9, 0x70},
	{0xD2, 0x02},
	{0x21, 0x44},
	{0x16, 0x03},
	{0x39, 0x80},
	{0x1e, 0xb1},
	{0x12, 0x00},
	{0x82, 0x03},
	{0xd0, 0x48},
	{0x80, 0x7f},
	{0x3e, 0x30},
	{0x22, 0x00},
	{0x17, 0x69},
	{0x18, 0xa4},
	{0x19, 0x0c},
	{0x1a, 0xf6},
	{0xc8, 0x02},
	{0xc9, 0x80},
	{0xca, 0x01},
	{0xcb, 0xe0},
	{0xcc, 0x02},
	{0xcd, 0x80},
	{0xce, 0x01},
	{0xcf, 0xe0},
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
	{0xb7, 0x05},
	{0xb8, 0x09},
	{0xb9, 0x00},
	{0xba, 0x18},
	{0x5A, 0x4A},
	{0x5B, 0x9F},
	{0x5C, 0x48},
	{0x5d, 0x32},
	{0x24, 0x80},
	{0x25, 0x70},
	{0x26, 0xa4},
	{0xa3, 0x04},
	{0xa4, 0x09},
	{0xa5, 0x16},
	{0xa6, 0x30},
	{0xa7, 0x3E},
	{0xa8, 0x4B},
	{0xa9, 0x59},
	{0xaa, 0x67},
	{0xab, 0x72},
	{0xac, 0x7C},
	{0xad, 0x8E},
	{0xae, 0x9E},
	{0xaf, 0xB6},
	{0xb0, 0xCC},
	{0xb1, 0xE2},
	{0xb2, 0x28},
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
	{0x50, 0x4c},
	{0x51, 0x3f},
	{0x21, 0x57},
	{0x20, 0x00},
	{0x14, 0x29},
	{0x13, 0xf7},
	{0x11, 0x00},

	{0xFF, 0xFF}
};


static struct regval_list ov7690_fmt_yuv422[] = {

	{0xFF, 0xFF}
};

static struct regval_list __attribute((unused)) ov7690_res_qcif_v[] = {

	/* 144 x 176 */
	{0xc8, 0x02},
	{0xc9, 0x80},
	{0xca, 0x01},
	{0xcb, 0xe0},
	{0xcc, 0x00},
	{0xcd, 0x90},
	{0xce, 0x00},
	{0xcf, 0xB0},

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

static struct regval_list __attribute((unused)) ov7690_res_qvga_v[] = {

	/* 240 x 320 */
	{0xc8, 0x02},
	{0xc9, 0x80},
	{0xca, 0x01},
	{0xcb, 0xe0},
	{0xcc, 0x00},
	{0xcd, 0xf0},
	{0xce, 0x01},
	{0xcf, 0x40},

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

static int ov7690_read(struct i2c_client *c, unsigned char reg,
		unsigned char *value)
{
	u8 data;
	u8 address[1];
	address[0] = reg;
	i2c_smbus_write_byte(c,reg);
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
	if (reg == REG_SYS && (value & SYS_RESET))
		msleep(2);  /* Wait for reset to run */

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

static int ov7690_g_register(struct i2c_client *client, struct v4l2_dbg_register * reg)
{
       return ov7690_read(client, (unsigned char)reg->reg, (unsigned char *)&(reg->val));
}

static int ov7690_s_register(struct i2c_client *client, struct v4l2_dbg_register * reg)
{
       return ov7690_write(client, (unsigned char)reg->reg, (unsigned char)reg->val);
}


static int ov7690_detect(struct i2c_client *client)
{
	unsigned char v = 0;
	int ret = 0;
	/*
	 * no MID register found. OK, we know we have an OmniVision chip...but which one?
	 */
	ret = ov7690_read(client, 0x0a, &v);

	printk(KERN_NOTICE "*ov7690_detect 0x%x\n",v);
	if (ret < 0)
		return ret;
	if (v != 0x76)
		return -ENODEV;

	ret = ov7690_read(client, 0x0b, &v);

	printk(KERN_NOTICE "*ov7690_detect 0x%x\n",v);
	if (ret < 0)
		return ret;
	if (v != 0x91)
		return -ENODEV;
	return 0;
}


/*
 * Store information about the video data format.  The color matrix
 * is deeply tied into the format, so keep the relevant values here.
 * The magic matrix nubmers come from OmniVision.
 */
static struct ov7690_format_struct {
	__u8 *desc;
	__u32 pixelformat;
	struct regval_list *regs;
	int bpp;   /* bits per pixel */
} ov7690_formats[] = {
	{
		.desc		= "YUYV422 planar",
		.pixelformat	= V4L2_PIX_FMT_YUV422P,
		.regs 		= ov7690_fmt_yuv422,
		.bpp		= 16,
	},
	{
		.desc           = "YUYV 4:2:0",
		.pixelformat    = V4L2_PIX_FMT_YUV420,
		.regs           = ov7690_fmt_yuv422,
		.bpp            = 12,
	},
	{
		.desc           = "YUYV422 packet",
		.pixelformat    = V4L2_PIX_FMT_UYVY,
		.regs           = ov7690_fmt_yuv422,
		.bpp            = 16,
	},
};
#define N_OV7690_FMTS ARRAY_SIZE(ov7690_formats)


/*
 * Then there is the issue of window sizes.  Try to capture the info here.
 */
#if 1
static struct ov7690_win_size {
	int	width;
	int	height;
	struct regval_list *regs;
} ov7690_win_sizes[] = {
	/* QCIF */
	{
		.width		= 176,
		.height		= 144,
		.regs           = ov7690_res_qcif,
	},
	/* CIF */
	{
		.width          = 352,
		.height         = 288,
		.regs           = ov7690_res_cif,
	},
	/* QVGA */
	{
		.width		= 320,
		.height		= 240,
		.regs           = ov7690_res_qvga,
	},
	/* VGA */
	{
		.width		= 640,
		.height		= 480,
		.regs           = ov7690_res_vga,
	},
};
#endif
#define N_WIN_SIZES (ARRAY_SIZE(ov7690_win_sizes))

static int ov7690_querycap(struct i2c_client *c, struct v4l2_capability *argp)
{
	if (unlikely(argp == NULL)) {
		printk(KERN_ERR"cam: argp is NULL in %s line%d \n", __FUNCTION__, __LINE__);
		return -EINVAL;
	}

	strcpy(argp->driver, "ov7690");
	return 0;
}

static int ov7690_enum_fmsize(struct i2c_client *c, struct v4l2_frmsizeenum *argp)
{
	struct v4l2_frmsizeenum frmsize;
	int ret = 0;

	if (copy_from_user(&frmsize, argp, sizeof(frmsize)))
		   return -EFAULT;

	if (    (frmsize.pixel_format == V4L2_PIX_FMT_YUV422P)
             || (frmsize.pixel_format == V4L2_PIX_FMT_YUV420)
             || (frmsize.pixel_format == V4L2_PIX_FMT_UYVY) ) {
		if (frmsize.index >= N_WIN_SIZES){
			/*printk(KERN_NOTICE"cam: max index is %d \n", N_WIN_SIZES);*/
		    return -EINVAL;
		}
		frmsize.type = V4L2_FRMSIZE_TYPE_DISCRETE;
		frmsize.discrete.height = ov7690_win_sizes[frmsize.index].height;
		frmsize.discrete.width = ov7690_win_sizes[frmsize.index].width;
		goto exit;
	}
	ret = -EINVAL;
exit:
	if (copy_to_user(argp, &frmsize, sizeof(frmsize)))
		return -EFAULT;
	return ret;
}

static int ov7690_q_awb(struct i2c_client *client, __s32 *value)
{
	return 0;
}

static int ov7690_t_awb(struct i2c_client *client, int value)
{
	return 0;
}

static int ov7690_q_ae(struct i2c_client *client, __s32 *value)
{
	return 0;
}

static int ov7690_t_ae(struct i2c_client *client, int value)
{
	return 0;
}

static int ov7690_q_af(struct i2c_client *client, __s32 *value)
{
	return 0;
}

static int ov7690_t_af(struct i2c_client *client, int value)
{
	return 0;
}

static struct ov7690_control {
	struct v4l2_queryctrl qc;
	int (*query)(struct i2c_client *c, __s32 *value);
	int (*tweak)(struct i2c_client *c, int value);
} ov7690_controls[] =
{
	{
		.qc = {
			.id = V4L2_CID_FOCUS_AUTO,
			.type = V4L2_CTRL_TYPE_BOOLEAN,
			.name = "auto focus",
			.minimum = 0,
			.maximum = 1,
			.step = 1,
			.default_value = 0,
		},
		.tweak = ov7690_t_af,
		.query = ov7690_q_af,
	},
	{
		.qc = {
			.id = V4L2_CID_AUTO_WHITE_BALANCE,
			.type = V4L2_CTRL_TYPE_BOOLEAN,
			.name = "auto white balance",
			.minimum = 0,
			.maximum = 1,
			.step = 1,
			.default_value = 0,
		},
		.tweak = ov7690_t_awb,
		.query = ov7690_q_awb,
	},
	{
		.qc = {
			.id = V4L2_CID_EXPOSURE_AUTO,
			.type = V4L2_CTRL_TYPE_BOOLEAN,
			.name = "auto exposure",
			.minimum = 0,
			.maximum = 1,
			.step = 1,
			.default_value = 0,
		},
		.tweak = ov7690_t_ae,
		.query = ov7690_q_ae,
	},
};
#define N_CONTROLS (ARRAY_SIZE(ov7690_controls))

static int ov7690_enum_fmt(struct i2c_client *c, struct v4l2_fmtdesc *fmt)
{

	struct ov7690_format_struct *ofmt;
	if (fmt->index >= N_OV7690_FMTS)
		return -EINVAL;

	ofmt = ov7690_formats + fmt->index;
	fmt->flags = 0;
	strncpy(fmt->description, ofmt->desc, min(sizeof(fmt->description), strlen(ofmt->desc)+1));
	fmt->description[sizeof(fmt->description)-1] = 0;
	fmt->pixelformat = ofmt->pixelformat;
	return 0;
}

static int ov7690_try_fmt(struct i2c_client *c, struct v4l2_format *fmt,
		struct ov7690_format_struct **ret_fmt,
		struct ov7690_win_size **ret_wsize)
{
	int index;
	struct v4l2_pix_format *pix = &fmt->fmt.pix;

	for (index = 0; index < N_OV7690_FMTS; index++)
		if (ov7690_formats[index].pixelformat == pix->pixelformat)
			break;
	if (index >= N_OV7690_FMTS){
		return -EINVAL;
	}

	if (ret_fmt != NULL)
		*ret_fmt = &ov7690_formats[index];

	/*
	 * Note the size we'll actually handle.
	 */

	pix->bytesperline = pix->width*ov7690_formats[index].bpp/8;
	pix->sizeimage = pix->height*pix->bytesperline;
	if(V4L2_PIX_FMT_JPEG == pix->pixelformat){
		/*printk(KERN_NOTICE "cam: ov7690_try_fmt: pix->sizeimage = JPEG_SIZE!\n");*/
		/*pix->sizeimage = JPEG_SIZE;*/
		printk(KERN_ERR "V4L2_PIX_FMT_JPEG format is not supported for OV7690!\n");
		return -EINVAL;
	}
	/*
	 * Round requested image size down to the nearest
	 * we support, but not below the smallest.
	 */
	for (index = 0; index < N_WIN_SIZES; index++)
		 if (pix->width == ov7690_win_sizes[index].width
			&& pix->height == ov7690_win_sizes[index].height)
			break;
	if (index >= N_WIN_SIZES){
		return -EINVAL;
	}

	if (ret_wsize != NULL)
		*ret_wsize = ov7690_win_sizes + index;

	return 0;
}


static struct ov7690_control *ov7690_find_control(__u32 id)
{
	int i;

	for (i = 0; i < N_CONTROLS; i++)
		if (ov7690_controls[i].qc.id == id)
			return ov7690_controls + i;
	return NULL;
}

static int ov7690_s_ctrl(struct i2c_client *client, struct v4l2_control *ctrl)
{
	struct ov7690_control *octrl = ov7690_find_control(ctrl->id);
	int ret;

	if (octrl == NULL)
		return -EINVAL;
	ret =  octrl->tweak(client, ctrl->value);
	if (ret >= 0)
		return 0;
	return ret;
}

static int ov7690_q_ctrl(struct i2c_client *client,
		struct v4l2_queryctrl *qc)
{
	struct ov7690_control *ctrl = ov7690_find_control(qc->id);

	if (ctrl == NULL)
		return -EINVAL;
	*qc = ctrl->qc;
	return 0;
}

static int ov7690_g_ctrl(struct i2c_client *client, struct v4l2_control *ctrl)
{
	struct ov7690_control *octrl = ov7690_find_control(ctrl->id);
	int ret;

	if (octrl == NULL)
		return -EINVAL;
	ret = octrl->query(client, &ctrl->value);

	if (ret >= 0)
		return 0;
	return ret;
}


/*
 * Set a format.
 */
static int ov7690_s_fmt(struct i2c_client *c, struct v4l2_format *fmt)
{
	int ret = 0;
	struct ov7690_format_struct *ovfmt = NULL;
	struct ov7690_win_size *wsize = NULL;
	struct v4l2_pix_format *pix = &fmt->fmt.pix;

	ret = ov7690_try_fmt(c, fmt, &ovfmt, &wsize);
	if (ret)
		return ret;

	printk(KERN_NOTICE "cam: ov7690 YUV resolution(%d, %d)\n", \
				wsize->width, wsize->height);

	if(V4L2_PIX_FMT_JPEG == pix->pixelformat) {
		/*No JPEG support on OV7690 currently*/
		printk(KERN_NOTICE "No JPEG support on OV7690 currently\n");
		return ret;
	} else {
#if 0
/*TavorEVBIII is using 1.8V DOVDD. Per spec suggestion, write 0x49[3:0] with value 0x0c*/

		ov7690_read(c, 0x49, &val);
		/* printk(KERN_NOTICE "DOVDD register: [0x49] = %x\n",val); */
		val=0x0c;
		ov7690_write(c, 0x49, val);
/*read DOVDD IO voltage setting*/
		ov7690_read(c, 0x49, &val);
		/* printk(KERN_NOTICE "DOVDD register: [0x49] = %x\n",val); */
#endif
/*Per OV suggest, config OV7690 first, then configure OV5642 bridge mode setting*/
		ov7690_write_array(c, ov7690_Set);
		ov7690_write_array(c, wsize->regs);
	}

	return ret;
}


/*
 * Implement G/S_PARM.  There is a "high quality" mode we could try
 * to do someday; for now, we just do the frame rate tweak.
 */
static int ov7690_g_parm(struct i2c_client *c, struct v4l2_streamparm *parms)
{
	return 0;
}

static int ov7690_s_parm(struct i2c_client *c, struct v4l2_streamparm *parms)
{
	return 0;
}

static int ov7690_s_input(struct i2c_client *c, int *id)
{
	printk(KERN_NOTICE "cam: ov7690_s_input!\n");
	return 0;
}

extern struct cam_platform_data *init_platform_ops;

/*
 * Basic i2c stuff.
 */
static int __devinit ov7690_probe(struct i2c_client *client, const struct i2c_device_id * id)
{
	int ret;
	struct ov7690_info *info;
	struct sensor_platform_data *pdata;
	pdata = client->dev.platform_data;
	printk(KERN_NOTICE "cam-ov7690: pdata->id =%d!\n",pdata->id);
	pdata->power_set(pdata->id, SENSOR_OPEN);
	/*
	 * Set up our info structure.
	 */
	info = kzalloc(sizeof (struct ov7690_info), GFP_KERNEL);
	if (! info) {
		ret = -ENOMEM;
		goto out_free;
	}
	info->fmt = &ov7690_formats[1];
	info->sat = 128;	/* Review this */
	i2c_set_clientdata(client, info);

	/*
	 * Make sure it's an ov7690
	 */
	ret = ov7690_detect(client);

	if (ret){
		printk(KERN_NOTICE "failed to detect OmniVision ov7690!\n");
		ret = -ENODEV;
		goto out_free_info;
	}
	else {
		printk(KERN_NOTICE "OmniVision ov7690 sensor detected!\n");
	}

	ccic_sensor_attach(client);
	pdata->power_set(pdata->id, SENSOR_CLOSE);
	return 0;

out_free_info:
	kfree(info);
out_free:
	return ret;
}


static int ov7690_remove(struct i2c_client *client)
{
	struct ov7690_info *info;
	struct sensor_platform_data *pdata;

	pdata = client->dev.platform_data;
	pdata->power_set(pdata->id, SENSOR_CLOSE);
	ccic_sensor_detach(client);

	info = i2c_get_clientdata(client);
	i2c_set_clientdata(client, NULL);
	kfree(info);
	return 0;
}

static int ov7690_streamon(struct i2c_client *client)
{
	return 0;
}
static int ov7690_streamoff(struct i2c_client *client)
{
	return 0;
}

static int ov7690_command(struct i2c_client *client, unsigned int cmd,
		void *arg)
{
	switch (cmd) {
		case VIDIOC_DBG_G_CHIP_IDENT:
			return v4l2_chip_ident_i2c_client(client, arg, V4L2_IDENT_OV7690, 0);
		case VIDIOC_INT_RESET:
			return 0;
		case VIDIOC_QUERYCAP:
			return ov7690_querycap(client, (struct v4l2_capability *)arg);
		case VIDIOC_ENUM_FMT:
			return ov7690_enum_fmt(client, (struct v4l2_fmtdesc *) arg);
		case VIDIOC_TRY_FMT:
			return ov7690_try_fmt(client, (struct v4l2_format *) arg, NULL, NULL);
		case VIDIOC_S_FMT:
			return ov7690_s_fmt(client, (struct v4l2_format *) arg);
		case VIDIOC_ENUM_FRAMESIZES:
                        return ov7690_enum_fmsize(client, (struct v4l2_frmsizeenum *) arg);
		case VIDIOC_QUERYCTRL:
			return ov7690_q_ctrl(client, (struct v4l2_queryctrl *) arg);
		case VIDIOC_S_CTRL:
			return ov7690_s_ctrl(client, (struct v4l2_control *) arg);
		case VIDIOC_G_CTRL:
			return ov7690_g_ctrl(client, (struct v4l2_control *) arg);
		case VIDIOC_S_PARM:
			return ov7690_s_parm(client, (struct v4l2_streamparm *) arg);
		case VIDIOC_G_PARM:
			return ov7690_g_parm(client, (struct v4l2_streamparm *) arg);
		case VIDIOC_S_INPUT:
			return ov7690_s_input(client, (int *) arg);
		case VIDIOC_STREAMON:
			return ov7690_streamon(client);
		case VIDIOC_STREAMOFF:
			return ov7690_streamoff(client);
                case VIDIOC_DBG_G_REGISTER:
                        return ov7690_g_register(client, (struct v4l2_dbg_register *) arg);
                case VIDIOC_DBG_S_REGISTER:
                        return ov7690_s_register(client, (struct v4l2_dbg_register *) arg);
	}
	return -EINVAL;
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
	.id_table       = ov7690_idtable,
	.command	= ov7690_command,
	.probe		= ov7690_probe,
	.remove		= ov7690_remove,
};

/*
 * Module initialization
 */
static int __init ov7690_mod_init(void)
{
	int ret = 0;
	ret = i2c_add_driver(&ov7690_driver);
	printk(KERN_NOTICE "cam: try to init OmniVision ov7690 sensor driver!\n");

	return ret;

}

static void __exit ov7690_mod_exit(void)
{
	i2c_del_driver(&ov7690_driver);
}

late_initcall(ov7690_mod_init);
module_exit(ov7690_mod_exit);

