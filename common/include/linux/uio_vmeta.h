/*
 * This software program is licensed subject to the GNU General Public License
 * (GPL).Version 2,June 1991, available at http://www.fsf.org/copyleft/gpl.html

 * (C) Copyright 2010 Marvell International Ltd.
 * All Rights Reserved
 */

#ifndef __UIO_VMETA_H
#define __UIO_VMETA_H

typedef unsigned int vmeta_instance_status;

typedef struct _vmeta_user_info {
	int usertype;		/*0:dec, 1:enc*/
	int strm_fmt;		/*0:mpeg1, 1:mpeg2, 2:mpeg4, 3:h261, 4:h263, 5:h264, 6:vc1 ap, 7:jpeg, 8:mjpeg, 10:vc1 sp&mp*/
	int width;
	int height;
	int perf_req;		/*-99: expect lowest perf, -1: expect lower perf, 0: default perf, 1: expect higher perf, 99: expect highest perf*/
	int curr_op;		/*Filled by driver, inform the high-level the current OP*/
}vmeta_user_info_t;

typedef struct _id_instance
{
	vmeta_instance_status	status;
	vmeta_user_info_t		info;
	int						frame_rate;
	pid_t					pid;
	unsigned int			pt;//pthread_t
}id_instance;

#define MAX_VMETA_INSTANCE 32

typedef enum _VMETA_LOCK_FLAG{
	VMETA_LOCK_OFF = 0,
	VMETA_LOCK_ON,
	VMETA_LOCK_FORCE_INIT
}VMETA_LOCK_FLAG;

/* This struct should be aligned with user space API */
typedef struct _kernel_share
{
	int ref_count;
	VMETA_LOCK_FLAG lock_flag;
	int active_user_id;
	struct timeval lock_start_tv;
	id_instance user_id_list[MAX_VMETA_INSTANCE];
}kernel_share;

#define IOP_MAGIC	'v'

#define VMETA_CMD_POWER_ON		_IO(IOP_MAGIC, 0)
#define VMETA_CMD_POWER_OFF		_IO(IOP_MAGIC, 1)
#define VMETA_CMD_CLK_ON		_IO(IOP_MAGIC, 2)
#define VMETA_CMD_CLK_OFF		_IO(IOP_MAGIC, 3)
#define VMETA_CMD_CLK_SWITCH		_IO(IOP_MAGIC, 4)
#define VMETA_CMD_LOCK			_IO(IOP_MAGIC, 5)
#define VMETA_CMD_UNLOCK		_IO(IOP_MAGIC, 6)
#define VMETA_CMD_PRIV_LOCK		_IO(IOP_MAGIC, 7)
#define VMETA_CMD_PRIV_UNLOCK		_IO(IOP_MAGIC, 8)
#define VMETA_CMD_REG_UNREG		_IO(IOP_MAGIC, 9)

#ifndef VMETA_OP_MAX
#define VMETA_OP_MAX		15
#define VMETA_OP_MIN		0
#define VMETA_OP_VGA		1
#define VMETA_OP_720P		8
#define VMETA_OP_1080P		14
#define VMETA_OP_VGA_MAX	(VMETA_OP_720P-1)
#define VMETA_OP_720P_MAX	(VMETA_OP_1080P-1)
#define	VMETA_OP_1080P_MAX	VMETA_OP_1080P
#define VMETA_OP_VGA_ENC	VMETA_OP_720P
#define VMETA_OP_VGA_ENC_MAX	VMETA_OP_720P_MAX
#define VMETA_OP_INVALID -1
#endif

/*
This API is exposed in kernel space to inform other componets, like GC
return: -1 failure; 0 power off; 1 power on
*/
int vmeta_power_status(void);

#endif /* __UIO_VMETA_H */
