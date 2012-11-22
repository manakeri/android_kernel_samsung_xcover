/*
 * gpo.h
 *
 * The headset detect driver based on micco
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
#ifndef _LINUX_GPO_H_
#define _LINUX_GPO_H_

#ifdef __KERNEL__
#include <linux/ioctl.h>
#endif

#define GPO_EXT_HEADSET_OFF	0
#define GPO_EXT_HEADSET_ON	1
#define GPO_EXT_MONO_AMP_ON    1
#define GPO_EXT_MONO_AMP_OFF   0

struct GPO_IOCTL{
	int gpo_ext_hsdetect_status;
	int gpo_ext_mono_amp_status;
	};


/*
 * ioctl calls that are permitted to the /dev/micco_hsdetect interface.
 */

/* GPO Headset detection status*/
#define GPO_EXT_HEADSET_STATUS		_IO('e', 0x01)
/* GPO mono amp */
#define GPO_EXT_MONO_AMP_STATUS	_IO('e', 0x02)

#endif /* _LINUX_GPO_H_ */
