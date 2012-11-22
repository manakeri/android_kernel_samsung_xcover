#ifndef __ASM_MACH_REGS_RIPC_H
#define __ASM_MACH_REGS_RIPC_H

#include <mach/addr-map.h>

/*
 * Resource IPC
 */
#define RIPC0_VIRT_BASE		(APB_VIRT_BASE + 0x03D000)
#define RIPC0_REG(offset)	(*((volatile u32 *)(RIPC0_VIRT_BASE+(offset))))

#define RIPC0_STATUS	RIPC0_REG(0x00)  /* RIPC0 Status Register */

#endif /* __ASM_MACH_REGS_RIPC_H */
