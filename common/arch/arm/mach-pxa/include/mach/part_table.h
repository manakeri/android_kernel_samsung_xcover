#ifndef	__ASMARM_ARCH_PART_TABLE_H__
#define	__ASMARM_ARCH_PART_TABLE_H__
#include <linux/mtd/partitions.h>
#include <mtd/mtd-abi.h>

static struct mtd_partition __attribute__((unused))
	android_256m_4k_page_partitions[] = {
	[0] = {
		.name        = "init",
		.offset      = 0xe00000,
		.size        = 0x40000,
	},
	[1] = {	/* Note, that this must be in partition#1 */
			/* Please don't change this!!! */
		.name        = "MRD",
		.offset      = 0x100000,
		.size        = 0xC0000,
	},
	[2] = {
		.name        = "Arbel and Greyback Image",
		.offset      = 0x800000,
		.size        = 0x200000,
		.mask_flags  = MTD_WRITEABLE,  /* force read-only */
	},
	[3] = {
		.name        = "Kernel",
		.offset      = 0xa00000,
		.size        = 0x400000,
		.mask_flags  = MTD_WRITEABLE,  /* force read-only */
	},
	[4] = {
		.name        = "system",
		.offset      = 0x0e40000,
		.size        = 0x8300000,     /* mount fs */
	},
	[5] = {
		.name        = "userdata",
		.offset      = 0x9140000,
		.size        = 0x4ec0000,     /* mount fs */
	},
	[6] = {
		.name        = "telephony",
		.offset      = 0xE000000,
		.size        = 0x1400000,     /* mount fs */
	},
	[7] = {
		.name        = "NVMFS",
		.offset      = 0xF400000,
		.size        = 0x100000,     /* mount fs */
	},
	[8] = {
		.name        = "Linux BBT",	/*Must not exceed 0xfb00000 (Mvrl BBT)*/
		.offset      = 0xFA00000,
		.size        = 0x100000,
		.mask_flags  = MTD_WRITEABLE,  /* force read-only */
	},
	/* NOTES: We reserve some blocks for PXA3xx BBM at the end of NAND.
	 * And the max relocation blocks is not same on different platform.
	 * Please take care it when define the partition table.
	 */
};


static struct mtd_partition __attribute__((unused))
	android_512m_4k_page_partitions[] = {
	[0] = {
		.name        = "init",
		.offset      = 0xe00000,
		.size        = 0x40000,
	},
	[1] = { /* Note, that this must be in partition#1 */
			/* Please don't change this!!! */
		.name        = "MRD",
		.offset      = 0x100000,
		.size        = 0xC0000,
	},
	[2] = {
		.name        = "Arbel and Greyback Image",
		.offset      = 0x800000,
		.size        = 0x200000,
		.mask_flags  = MTD_WRITEABLE,  /* force read-only */
	},
	[3] = {
		.name        = "Kernel",
		.offset      = 0xa00000,
		.size        = 0x400000,
		.mask_flags  = MTD_WRITEABLE,  /* force read-only */
	},
	[4] = {
		.name        = "system",
		.offset      = 0x0e40000,
		.size        = 0x8300000,     /* mount fs */
	},
	[5] = {
		.name        = "userdata",
		.offset      = 0x9140000,
		.size        = 0x14bc0000,     /* mount fs */
	},
	[6] = {
		.name        = "telephony",
		.offset      = 0x1dd00000,
		.size        = 0x1400000,     /* mount fs */
	},
	[7] = {
		.name        = "NVMFS",
		.offset      = 0x1f100000,
		.size        = 0x400000,     /* mount fs */
	},
	[8] = {
		.name        = "Linux BBT",	/*Must not exceed 0xfb00000 (Mvrl BBT)*/
		.offset      = 0x1f500000,
		.size        = 0x0100000,
		.mask_flags  = MTD_WRITEABLE,  /* force read-only */
	},
	/* NOTES: We reserve some blocks for PXA3xx BBM at the end of NAND.
	 * And the max relocation blocks is not same on different platform.
	 * Please take care it when define the partition table.
	 */
};

#endif

