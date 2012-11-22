/** @file mbt_char.h
  *
  * @brief This file contains mbtchar driver specific defines etc
  * 
  * Copyright (C) 2010-2011, Marvell International Ltd.  
  *
  * This software file (the "File") is distributed by Marvell International 
  * Ltd. under the terms of the GNU General Public License Version 2, June 1991 
  * (the "License").  You may use, redistribute and/or modify this File in 
  * accordance with the terms and conditions of the License, a copy of which 
  * is available by writing to the Free Software Foundation, Inc.,
  * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA or on the
  * worldwide web at http://www.gnu.org/licenses/old-licenses/gpl-2.0.txt.
  *
  * THE FILE IS DISTRIBUTED AS-IS, WITHOUT WARRANTY OF ANY KIND, AND THE 
  * IMPLIED WARRANTIES OF MERCHANTABILITY OR FITNESS FOR A PARTICULAR PURPOSE 
  * ARE EXPRESSLY DISCLAIMED.  The License provides additional details about 
  * this warranty disclaimer.
  *
  */
#ifndef __MBT_CHAR_H__
#define __MBT_CHAR_H__

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/version.h>
#include <linux/fs.h>
#include <linux/cdev.h>

#include <include/bluetooth.h>
#include <include/hci_core.h>

/** Driver release version */
#define DRIVER_VERSION		"M2614001"

#define MBTCHAR_MAJOR_NUM    (0)
/** BIT value */
#define MBIT(x)    (((u32)1) << (x))

/** Debug Macro definition*/
extern ulong drvdbg;

/** Debug message control bit definition */
#define	MMSG        MBIT(0)
#define MFATAL      MBIT(1)
#define MERROR      MBIT(2)
#define MDATA       MBIT(3)
#define MINFO       MBIT(4)
#define MDAT_D      MBIT(5)
#define MENTRY      MBIT(6)

#define	PRINTM_MENTRY(msg...) do {if (drvdbg & MENTRY) printk(KERN_DEBUG msg);} while(0)
#define	PRINTM_MINFO(msg...)  do {if (drvdbg & MINFO) printk(KERN_DEBUG msg);} while(0)
#define	PRINTM_MDATA(msg...)  do {if (drvdbg & MDATA) printk(KERN_DEBUG msg);} while(0)
#define	PRINTM_MERROR(msg...) do {if (drvdbg & MERROR) printk(KERN_DEBUG msg);} while(0)
#define	PRINTM_MFATAL(msg...) do {if (drvdbg & MFATAL) printk(KERN_DEBUG msg);} while(0)
#define	PRINTM_MMSG(msg...)   do {if (drvdbg & MMSG) printk(KERN_ALERT msg);} while(0)
#define	PRINTM(level,msg...) PRINTM_##level(msg)

/** Log entry point for debugging */
#define	ENTER()			PRINTM(MENTRY, "Enter: %s\n", \
                                    __FUNCTION__)
/** Log exit point for debugging */
#define	LEAVE()			PRINTM(MENTRY, "Leave: %s\n", \
                                    __FUNCTION__)

#define DBG_DUMP_BUF_LEN 	64
#define MAX_DUMP_PER_LINE	16

static inline void
hexdump(char *prompt, u8 * buf, int len)
{
    int i;
    char dbgdumpbuf[DBG_DUMP_BUF_LEN];
    char *ptr = dbgdumpbuf;

    printk(KERN_DEBUG "%s:\n", prompt);
    for (i = 1; i <= len; i++) {
        ptr += snprintf(ptr, 4, "%02x ", *buf);
        buf++;
        if (i % MAX_DUMP_PER_LINE == 0) {
            *ptr = 0;
            printk(KERN_DEBUG "%s\n", dbgdumpbuf);
            ptr = dbgdumpbuf;
        }
    }
    if (len % MAX_DUMP_PER_LINE) {
        *ptr = 0;
        printk(KERN_DEBUG "%s\n", dbgdumpbuf);
    }
}

#define DBG_HEXDUMP_MDAT_D(x,y,z)    do {if (drvdbg & MDAT_D) hexdump(x,y,z);} while(0)
#define	DBG_HEXDUMP(level,x,y,z)    DBG_HEXDUMP_##level(x,y,z)

/** functions export to hci_wrapper */
int mbtchar_register_dev(struct hci_dev *hdev);
int mbtchar_unregister_dev(struct hci_dev *hdev);

/** functions export to mbtchar */
int hci_wrapper_open(struct hci_dev *hdev);
int hci_wrapper_close(struct hci_dev *hdev);
int hci_wrapper_send(struct hci_dev *hdev, struct sk_buff *skb);
#endif /*__MBT_CHAR_H__*/
