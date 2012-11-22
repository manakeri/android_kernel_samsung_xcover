/*******************************************************************************

This software file (the "File") is distributed by Marvell International Ltd.
or its affiliate(s) under the terms of the GNU General Public License Version 2,
June 1991 (the "License").  You may use, redistribute and/or modify this File
in accordance with the terms and conditions of the License, a copy of which
is available along with the File in the license.txt file or by writing to the
Free Software Foundation, Inc.,59 Temple Place, Suite 330, Boston, MA 02111-1307
or on the worldwide web at http://www.gnu.org/licenses/gpl.txt.

THE FILE IS DISTRIBUTED AS-IS, WITHOUT WARRANTY OF ANY KIND, AND THE IMPLIED
WARRANTIES OF MERCHANTABILITY OR FITNESS FOR A PARTICULAR PURPOSE ARE EXPRESSLY
DISCLAIMED.  The GPL License provides additional details about this warranty
disclaimer.

Copyright:	(C) Copyright 2008 Marvell International Ltd.
*******************************************************************************/

#ifndef __IF228_H__
#define __IF228_H__

#include <linux/videodev2.h>

#define READ_AHBM2	0x71	/*type2*/
#define WRITE_AHBM2	0x73	/*type2*/
#define READ_AHBM1	0x70	/*type3*/
#define WRITE_AHBM1	0x72	/*type3*/

#define SPI_CID					V4L2_CID_PRIVACY
#define SPI_CID_W_ONE_BYTE		(V4L2_CID_PRIVACY + 1)
#define SPI_CID_R_ONE_BYTE		(V4L2_CID_PRIVACY + 2)
#define SPI_CID_W_BYTE_TYPE2	(V4L2_CID_PRIVACY + 3)
#define SPI_CID_R_BYTE_TYPE2	(V4L2_CID_PRIVACY + 4)
#define SPI_CID_W_WORD_TYPE2	(V4L2_CID_PRIVACY + 5)
#define SPI_CID_R_WORD_TYPE2	(V4L2_CID_PRIVACY + 6)
#define SPI_CID_W_BYTES_TYPE3	(V4L2_CID_PRIVACY + 7)
#define SPI_CID_R_BYTES_TYPE3	(V4L2_CID_PRIVACY + 8)
#define SPI_CID_W_BYTES_TYPE4	(V4L2_CID_PRIVACY + 9)
#define SPI_CID_R_BYTES_TYPE4	(V4L2_CID_PRIVACY + 10)
#define SPI_CID_R_BYTES			(V4L2_CID_PRIVACY + 11)

/* assemble an ioctl command */
#define CMMB_IOCTL(addr, data)	(((addr) << 16) | (data))

/* disassemble an ioctl command */
#define CMMB_IOCTL_ADDR(val)	((val) >> 16)
#define CMMB_IOCTL_DATA(val)	((val) & 0xffff)

#endif
