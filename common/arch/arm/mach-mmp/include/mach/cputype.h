#ifndef __ASM_MACH_CPUTYPE_H
#define __ASM_MACH_CPUTYPE_H

#include <asm/io.h>
#include <asm/cputype.h>
#include <mach/addr-map.h>

#define CHIP_ID		(AXI_VIRT_BASE + 0x82c00)

#define BLOCK7_RESEVED_2	(AXI_VIRT_BASE + 0x1498)
#define FUSE_ID		BLOCK7_RESEVED_2
/*
 *  CPU   Stepping   CPU_ID      CHIP_ID
 *
 * PXA168    S0    0x56158400   0x0000C910
 * PXA168    A0    0x56158400   0x00A0A168
 * PXA910    Y1    0x56158400   0x00F2C920
 * PXA910    A0    0x56158400   0x00F2C910
 * PXA910    A1    0x56158400   0x00A0C910
 * PXA920    Y0    0x56158400   0x00F2C920
 * PXA920    A0    0x56158400   0x00A0C920
 * PXA920    A1    0x56158400   0x00A1C920
 * MMP2	     Z0	   0x560f5811   0x00F00410
 * MMP2      Z1    0x560f5811   0x00E00410
 * MMP2      A0    0x560f5811   0x00A0A610
 */

#define MMP_CHIPID		(AXI_VIRT_BASE + 0x82c00)

#ifdef CONFIG_CPU_PXA168
#define __cpu_is_pxa168(id)	\
	({ unsigned int _id = ((id) >> 8) & 0xff; _id == 0x84; })
#else
#define __cpu_is_pxa168(id)	(0)
#endif

/* cpu_is_pxa910() is shared on both pxa910 and pxa920 */
#ifdef CONFIG_CPU_PXA910
#define __cpu_is_pxa910(id)	\
	({ unsigned int _id = ((id) >> 8) & 0xff;		\
	   unsigned int _mmp_id = (__raw_readl(MMP_CHIPID) & 0xfff);	\
	   (_id == 0x84 && (_mmp_id == 0x910 || _mmp_id == 0x920)); })
#else
#define __cpu_is_pxa910(id)	(0)
#endif

#ifdef CONFIG_CPU_MMP2
#define __cpu_is_mmp2(id)	\
	({ unsigned int _id = ((id) >> 8) & 0xff; \
	_id == 0x58 || _id == 0xb7 || _id == 0xc0; })
#else
#define __cpu_is_mmp2(id)	(0)
#endif

#ifdef CONFIG_CPU_MMP3
#define __cpu_is_mmp3(id)       \
	({ unsigned int _id = ((id) >> 8) & 0xff; _id == 0x58; })
#else
#define __cpu_is_mmp3(id)	(0)
#endif

#define cpu_is_pxa168()		({ __cpu_is_pxa168(read_cpuid_id()); })
#define cpu_is_pxa910()		({ __cpu_is_pxa910(read_cpuid_id()); })
#define cpu_is_mmp2()		({ __cpu_is_mmp2(read_cpuid_id()); })
#define cpu_is_mmp3()		({ __cpu_is_mmp3(read_cpuid_id()); })

#define CHIP_ID         (AXI_VIRT_BASE + 0x82c00)
#define CPU_CONF        (AXI_VIRT_BASE + 0x82c08)
#define SOC_STEPPING    (BOOTROM_VIRT_BASE + 0x30)

static inline int cpu_is_pxa910_Ax(void)
{
	unsigned int revision = (__raw_readl(CHIP_ID) >> 16) & 0xff;

	if (cpu_is_pxa910()) {
		if ((revision >= 0xa0) && (revision < 0xb0))
			return 1;
	}
	return 0;
}

extern int is_pxa921;
static inline int cpu_is_pxa921(void)
{
#if 0
	unsigned int fuse_id = __raw_readl(FUSE_ID);

	if (cpu_is_pxa910() && ((fuse_id & 0xa000) == 0))
		return 1;
	return 0;
#endif
	return is_pxa921;
}

static inline int cpu_is_pxa918(void)
{
	unsigned int fuse_id = __raw_readl(FUSE_ID);

	if (cpu_is_pxa910() && ((fuse_id & 0x3000000) == 0x3000000))
		return 1;
	return 0;
}

static inline int cpu_is_pxa910_c910(void)
{
	unsigned int chip_id = __raw_readl(CHIP_ID);

	if (cpu_is_pxa910() && ((chip_id & 0xffff) == 0xc910))
		return 1;
	return 0;
}

static inline int cpu_is_pxa910_c920(void)
{
	unsigned int chip_id = __raw_readl(CHIP_ID);

	if (cpu_is_pxa910() && ((chip_id & 0xffff) == 0xc920))
		return 1;
	return 0;
}

static inline int cpu_is_mmp2_z0(void)
{
	unsigned int chip_id = __raw_readl(CHIP_ID);
	if (cpu_is_mmp2() && ((chip_id & 0x00ff0000) == 0x00f00000))
		return 1;
	else
		return 0;
}

static inline int cpu_is_mmp2_z1(void)
{
	unsigned int chip_id = __raw_readl(CHIP_ID);
	if (cpu_is_mmp2() && ((chip_id & 0x00ff0000) == 0x00e00000))
		return 1;
	else
		return 0;
}

static inline int cpu_is_mmp2_a0(void)
{
	unsigned int chip_id = __raw_readl(CHIP_ID);
	unsigned int soc_stepping = __raw_readl(SOC_STEPPING);
	if (cpu_is_mmp2() && ((chip_id & 0x00ff0000) == 0x00a00000) \
	 && (soc_stepping == 0x4130))
		return 1;
	else
		return 0;
}

static inline int cpu_is_mmp2_a1(void)
{
	unsigned int chip_id = __raw_readl(CHIP_ID);
	unsigned int soc_stepping = __raw_readl(SOC_STEPPING);
	if (cpu_is_mmp2() && ((chip_id & 0x00ff0000) == 0x00a00000) \
	 && (soc_stepping == 0x4131))
		return 1;
	else
		return 0;
}

static inline int cpu_is_mmp3_z0(void)
{
	unsigned int chip_id = __raw_readl(CHIP_ID);
	if (cpu_is_mmp3() && ((chip_id & 0x00ff0000) == 0x00f00000))
		return 1;
	else
		return 0;
}

static inline int cpu_is_mmp3_z1(void)
{
	unsigned int chip_id = __raw_readl(CHIP_ID);
	if (cpu_is_mmp3() && ((chip_id & 0x00ff0000) == 0x00e00000))
		return 1;
	else
		return 0;
}

static inline int cpu_is_mmp3_a0(void)
{
	unsigned int chip_id = __raw_readl(CHIP_ID);
	if (cpu_is_mmp3() && ((chip_id & 0x00ff0000) == 0x00a00000))
		return 1;
	else
		return 0;
}

#endif /* __ASM_MACH_CPUTYPE_H */
