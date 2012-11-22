/*
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

#ifndef _IRE_H
#define _IRE_H

#include <asm/ioctl.h>

#define MAX_IRE_BUF_COUNT	3

#define IREIO_REQUEST_MEM	_IOW('2', 1, struct ire_mem_req *)
#define IREIO_RELEASE_MEM	_IOW('2', 2, unsigned long)
#define IREIO_FLUSH_MEM		_IOW('2', 3, unsigned long)
#define IREIO_S_FMT		_IOW('2', 4, struct ire_fmt *)
#define IREIO_G_FMT		_IOW('2', 5, struct ire_fmt *)
#define IREIO_SUBMIT		_IOW('2', 6, struct ire_submit_req *)
#define IREIO_ENQUEUE		_IOW('2', 7, int)
#define IREIO_DEQUEUE		_IOW('2', 8, int)
#define IREIO_ENABLE		_IOW('2', 10, unsigned long)
#define IREIO_DISABLE		_IOW('2', 11, unsigned long)
#define IREIO_DESUBMIT		_IOW('2', 12, struct ire_submit_req *)

#define IRE_GRAPHICS_MEM	0
#define IRE_FRAME_BUFFER	1

#define IRE_ATTR_COHERENT	0x00
#define IRE_ATTR_WRITECOMBINE	0x10
#define IRE_ATTR_CACHEABLE	0x20

#define IRE_MEM_REQ_TYPE(f)		(f & 0x0f)
#define IRE_MEM_REQ_ATTR(f)		(f & 0xf0)

#define IRE_PIXFMT(endian, yuv, mode, bits) \
	(((endian & 0x3) << 9) | ((yuv & 0x3) << 7) |\
	 ((mode & 0x3) << 5) | ((bits & 0x7) << 2))

#define IRE_PIXFMT_BITS(fmt)	(((fmt >> 2) & 0x7) == 4 ? 16 :\
				((fmt >> 2) & 0x7) == 6 ? 32 :\
				((fmt >> 7) & 0x3) == 1 ? 16 : \
				((fmt >> 7) & 0x3) == 2 ? 12 : 0)

#define IRE_PIXFMT_UV_FACTOR(fmt)	(((fmt >> 5) & 0x3) == 0 ? 0 : \
				((fmt >> 7) & 0x3) == 1 ? 4 : \
				((fmt >> 7) & 0x3) == 2 ? 2 : 0)

typedef enum {
	IRE_PIXFMT_RGB565		= IRE_PIXFMT(0, 0, 0, 4),
	IRE_PIXFMT_RGB888		= IRE_PIXFMT(0, 0, 0, 6),
	IRE_PIXFMT_YUV420_PLANAR	= IRE_PIXFMT(0, 2, 2, 3),
	IRE_PIXFMT_YUV422_PACKED_VYUV	= IRE_PIXFMT(0, 1, 1, 3),
	IRE_PIXFMT_YUV422_PACKED_UYVY	= IRE_PIXFMT(1, 1, 1, 3),
	IRE_PIXFMT_YUV422_PACKED_YUYV	= IRE_PIXFMT(2, 1, 1, 3),
	IRE_PIXFMT_YUV422_PACKED_YVYU	= IRE_PIXFMT(3, 1, 1, 3),
} IRE_PIXEL_FORMAT;

typedef enum {
	IRE_ROT_90	= 0,
	IRE_ROT_270	= 1,
	IRE_ROT_180	= 2,
} IRE_ROTATION;

struct ire_fmt {
	unsigned int	fmt;
	unsigned int	rot_angle;
	unsigned long	width, height;
	unsigned long	y_pitch, uv_pitch;
	unsigned long	yp0_pitch, uvp0_pitch;
};

struct ire_mem_req {
	unsigned int	req_type;
	unsigned int	req_size;
	unsigned long	phys_addr;
	unsigned long	mmap_addr;
	unsigned long	mmap_size;
};

#define IRE_SUBMIT_MODE_NDELAY	(1 << 0)
#define IRE_SUBMIT_MODE_DEQUEUE	(1 << 1)

struct ire_buffer {
	unsigned long	y_vaddr, u_vaddr, v_vaddr;
};

struct ire_buffer_phy {
	size_t size;
	unsigned long	y_paddr, u_paddr, v_paddr;
};

struct ire_submit_req {
	size_t size;
	unsigned int buf_id;
	struct ire_buffer src;
	struct ire_buffer dst;
};

#endif