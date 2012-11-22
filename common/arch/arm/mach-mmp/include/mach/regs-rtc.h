#ifndef __ASM_MACH_REGS_RTC_H
#define __ASM_MACH_REGS_RTC_H

#include <mach/hardware.h>

/*
 * Real Time Clock
 */
#define RTC_REG(offset)	(*((volatile u32 *)(rtc_base+(offset))))

#define RCNR		RTC_REG(0x00)  /* RTC Count Register */
#define RTAR		RTC_REG(0x04)  /* RTC Alarm Register */
#define RTSR		RTC_REG(0x08)  /* RTC Status Register */
#define RTTR		RTC_REG(0x0c)  /* RTC Timer Trim Register */
#define PIAR		RTC_REG(0x38)  /* Periodic Interrupt Alarm Register */

#define RTSR_PICE	(1 << 15)	/* Periodic interrupt count enable */
#define RTSR_PIALE	(1 << 14)	/* Periodic interrupt Alarm enable */
#define RTSR_HZE	(1 << 3)	/* HZ interrupt enable */
#define RTSR_ALE	(1 << 2)	/* RTC alarm interrupt enable */
#define RTSR_HZ		(1 << 1)	/* HZ rising-edge detected */
#define RTSR_AL		(1 << 0)	/* RTC alarm detected */

#endif /* __ASM_MACH_REGS_RTC_H */
