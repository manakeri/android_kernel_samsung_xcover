#ifndef OV5642_H_
#define OV5642_H_

#include <linux/types.h>
#include <media/v4l2-chip-ident.h>
#include <media/v4l2-common.h>
#include <media/soc_camera.h>

#define OV5642_END_ADDR		0xFFFF
#define OV5642_END_VAL		0xFF
#define END_SYMBOL		{OV5642_END_ADDR, OV5642_END_VAL}

struct regval_list {
	u16 reg_num;
	unsigned char value;
};

struct ov5642_format {
	enum v4l2_mbus_pixelcode	code;
	struct regval_list	*regs;
	struct regval_list	*def_set;
};

struct ov5642_win_size {
	int width;
	int height;
	struct regval_list *regs;
};

struct ov5642_config {
	const char name[32];
	struct ov5642_format *fmt;
	int fmt_size;
	struct ov5642_win_size *yuv_res;
	int yuv_res_size;
	struct ov5642_win_size *jpg_res;
	int jpg_res_size;
	struct regval_list	*init;
};

struct ov5642 {
	struct v4l2_subdev subdev;
	int model;	/* V4L2_IDENT_OV5642* codes from v4l2-chip-ident.h */
	struct v4l2_rect rect;
	u32 pixfmt;
	struct i2c_client *client;
	struct soc_camera_device icd;
	struct regval_list *init;
	struct regval_list *regs_fmt;
	struct regval_list *regs_size;
	struct regval_list *regs_default;
};

/* ov5642 has only one fixed colorspace per pixelcode */
struct ov5642_datafmt {
	enum v4l2_mbus_pixelcode	code;
	enum v4l2_colorspace		colorspace;
};

typedef struct {
	u16 reg_base;
	char value[256];
	int len;
}OV5642_REG_ARRAY;


extern int ov5642_read(struct i2c_client *c, u16 reg, unsigned char *value);
extern int ov5642_write(struct i2c_client *c, u16 reg, unsigned char value);
extern int select_bus_type(const char *name);
extern int set_stream(struct i2c_client *client, int enable);
extern int get_firmware(OV5642_REG_ARRAY **firmware_array);
extern int get_yuv_res_array(struct ov5642_win_size **res);
extern int get_jpg_res_array(struct ov5642_win_size **res);
extern struct regval_list *get_global_init_regs(void);
extern struct regval_list *get_fmt_regs(enum v4l2_mbus_pixelcode code);
extern struct regval_list *get_fmt_default_setting(enum v4l2_mbus_pixelcode code);
extern struct regval_list *get_yuv_size_regs(int width, int height);
extern struct regval_list *get_jpg_size_regs(int width, int length);
extern struct ov5642_win_size *get_yuv_size_array(void);
extern struct ov5642_win_size *get_jpg_size_array(void);
#endif

