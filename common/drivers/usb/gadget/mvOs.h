/*******************************************************************************
*
*   Marvell Serial ATA Linux Driver
*   Copyright 2004 
*   Marvell International Ltd.
*
* This software program (the "Program") is distributed by Marvell International
* ltd. under the terms of the GNU General Public License Version 2, June 1991 
* (the "License").  You may use, redistribute and/or modify this Program in 
* accordance with the terms and conditions of the License, a copy of which is
* available along with the Program in the license.txt file or by writing to the 
* Free Software Foundation, Inc., 59 Temple Place, Suite 330, Boston, 
* MA 02111-1307 or on the worldwide web at http://www.gnu.org/licenses/gpl.txt.
*
* THE PROGRAM IS DISTRIBUTED AS-IS, WITHOUT WARRANTY OF ANY KIND, AND THE 
* IMPLIED WARRANTIES OF MERCHANTABILITY OR FITNESS FOR A PARTICULAR PURPOSE ARE
* EXPRESSLY DISCLAIMED.  The License provides additional details about this 
* warranty disclaimer.
*
* For more information about the Program or the License as it applies to the
* Program, please contact Marvell International Ltd. via its affiliate, Marvell
* Semiconductor, Inc., 700 First Avenue, Sunnyvale, CA 94010
*
********************************************************************************
* mvOsLinux.h - O.S. interface header file for Linux  
*
* DESCRIPTION:
*       This header file contains OS dependent definition under Linux
*
* DEPENDENCIES:
*       Linux kernel header files.
*
* FILE REVISION NUMBER:
*       $Revision: 1.1.2.1 $
*******************************************************************************/

#ifndef __INCmvOsLinuxh
#define __INCmvOsLinuxh

/* Includes */
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/ioport.h>
#include <linux/types.h>
#include <linux/version.h>
#include <linux/errno.h>
#include <linux/delay.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/timer.h>
#include <linux/list.h>
#include <linux/interrupt.h>
#include <linux/proc_fs.h>
#include <linux/mm.h>
#include <linux/device.h>
#include <linux/dma-mapping.h>
#include <linux/platform_device.h>
#include <linux/clk.h>

#include <asm/byteorder.h>
#include <asm/dma.h>
#include <asm/io.h>
#include <asm/irq.h>
#include <asm/system.h>
#include <asm/mach-types.h>
#include <asm/unaligned.h>
#include <plat/pxa_u2o.h>
#include <asm/uaccess.h>

#include <linux/usb.h>
#include <linux/usb/ch9.h>
#include <linux/usb/cdc.h>
#include <linux/usb/gadget.h>
#include <linux/usb/otg.h>


/* Definitions */

#define MV_DEFAULT_QUEUE_DEPTH 2
/* System dependent macro for flushing CPU write cache */
#define MV_CPU_WRITE_BUFFER_FLUSH()     wmb()

/* System dependent little endian from / to CPU conversions */
#define MV_CPU_TO_LE16(x)   cpu_to_le16(x)
#define MV_CPU_TO_LE32(x)   cpu_to_le32(x)

#define MV_LE16_TO_CPU(x)   le16_to_cpu(x)
#define MV_LE32_TO_CPU(x)   le32_to_cpu(x)

/* System dependent register read / write in byte/word/dword variants */
#define MV_REG_WRITE_BYTE(base, offset, val)    writeb(val, base + offset)
#define MV_REG_WRITE_WORD(base, offset, val)    writew(val, base + offset)
#define MV_REG_WRITE_DWORD(base, offset, val)   writel(val, base + offset)
#define MV_REG_READ_BYTE(base, offset)          readb(base + offset)
#define MV_REG_READ_WORD(base, offset)          readw(base + offset)
#define MV_REG_READ_DWORD(base, offset)         readl(base + offset)


/* Typedefs    */
typedef enum mvBoolean{MV_FALSE, MV_TRUE} MV_BOOLEAN;

/* System dependant typedefs */
typedef void            MV_VOID;
typedef u32             MV_U32;
typedef u16             MV_U16;
typedef u8              MV_U8;
typedef void            *MV_VOID_PTR;
typedef u32             *MV_U32_PTR;
typedef u16             *MV_U16_PTR;
typedef u8              *MV_U8_PTR;
typedef char            *MV_CHAR_PTR;
typedef void            *MV_BUS_ADDR_T;
typedef unsigned long   MV_CPU_FLAGS;


/* Structures  */
/* System dependent structure */
typedef struct mvOsSemaphore
{
  int notUsed;
} MV_OS_SEMAPHORE;


/* Functions (User implemented)*/

/* Semaphore init, take and release */
#define mvOsSemInit(x) MV_TRUE
#define mvOsSemTake(x)
#define mvOsSemRelease(x)

/* Interrupt masking and unmasking functions */
MV_CPU_FLAGS mvOsSaveFlagsAndMaskCPUInterrupts(MV_VOID);
MV_VOID      mvOsRestoreFlags(MV_CPU_FLAGS);

/* Delay function in micro seconds resolution */
void mvMicroSecondsDelay(MV_VOID_PTR, MV_U32);

/* System logging function */
#include "mvLog.h"
/* Enable READ/WRITE Long SCSI command only when driver is compiled for debugging */
#ifdef MV_LOGGER
#define MV_SATA_SUPPORT_READ_WRITE_LONG
#endif

#define MV_IAL_LOG_ID       3

#endif /* __INCmvOsLinuxh */

