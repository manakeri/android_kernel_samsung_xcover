/*
 * D1980_hsdetect.h
 *
 * The headset detect driver based on D1980
 *
 * Copyright (2008) Marvell International Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#ifndef _LINUX_D1980_HSDETECT_H_
#define _LINUX_D1980_HSDETECT_H_

#include <linux/switch.h>
#include <linux/miscdevice.h>

#define D1980_HEADSET_REMOVE	      0
#define D1980_HEADSET_ADD	      1
#define D1980_HEADSET_MODE_STEREO		0
#define D1980_HEADSET_MODE_MONO		1
#define D1980_HS_MIC_ADD				1
#define D1980_HS_MIC_REMOVE			0
#define D1980_HOOKSWITCH_PRESSED    1
#define D1980_HOOKSWITCH_RELEASED   0

#define D1980_HEADSET_DETECT	       0x20
#define D1980_HOOKSWITCH_DETECT   0x10

struct D1981_HS_IOCTL {
	int hsdetect_status;
	int hsdetect_mode; /* for future stereo/mono */
	int hsmic_status;
	int hookswitch_status;
	};

struct d1981_hs_data {
	struct switch_dev sdev;
	struct miscdevice miscdev;
	int mic_status;
	int hs_status;
};
/*
 * ioctl calls that are permitted to the /dev/micco_hsdetect interface.
 */

#define D1980_HSDETECT_STATUS		_IO('L', 0x01)	/* Headset detection status*/
#define D1980_HOOKSWITCH_STATUS	_IO('L', 0x02)	/* Hook switch status */

#define ENABLE_HS_DETECT_POLES

int d1981_hs_handler(struct kobject *kobj, int event);
int d1981_get_hook_state(void);

#endif /* _LINUX_D1980_HSDETECT_H_ */
