/*
 * 88pm860x-headset.h
 *
 * The headset detect driver based on levante
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

#ifndef __LINUX_MFD_88PM860X_HEADSET_H
#define __LINUX_MFD_88PM860X_HEADSET_H

#define PM860X_HEADSET_REMOVE			0
#define PM860X_HEADSET_ADD				1
#define PM860X_HEADPHONE_ADD			2
#define PM860X_HEADSET_MODE_STEREO		0
#define PM860X_HEADSET_MODE_MONO		1
#define PM860X_HS_MIC_ADD				1
#define PM860X_HS_MIC_REMOVE			0
#define PM860X_HOOKSWITCH_PRESSED		1
#define PM860X_HOOKSWITCH_RELEASED		0

struct PM860X_HS_IOCTL {
	int hsdetect_status;
	int hsdetect_mode; /* for future stereo/mono */
	int hsmic_status;
	int hookswitch_status;
	};

/*
 * ioctl calls that are permitted to the /dev/micco_hsdetect interface.
 */

#define PM860X_HSDETECT_STATUS		_IO('L', 0x01)	/* Headset detection status*/
#define PM860X_HOOKSWITCH_STATUS	_IO('L', 0x02)	/* Hook switch status */

#define ENABLE_HS_DETECT_POLES

#endif /* __LINUX_MFD_88PM860X_HEADSET_H */
