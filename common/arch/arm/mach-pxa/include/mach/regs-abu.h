#ifndef __ASM_ARCH_REGS_ABU_H
#define __ASM_ARCH_REGS_ABU_H


/* ABU & SSI */
#define	SSICR		(0x48)  /* was SSFL -ESSI FIFO level 			SSP4Base + 0x48 */
#define	SSWSS		(0x4c)  /* filler - was ESSI watermark step		SSP4Base + 0x4c */
#define	ABUCR0		(0x50)  /* ABU	Control Register			SSP4Base + 0x50 */
#define	ABUSR		(0x54)  /* ABU	Statusl Register			SSP4Base + 0x54 */
#define	ABURWR		(0x58)  /* ABU	Record Watermark Register		SSP4Base + 0x58 */
#define	ABUCR1		(0x5c)  /* ABU	Control Register			SSP4Base + 0x5c */
#define	ABUSPR		(0x84)  /* {00, system_read_pointer, 00, system_write_pointer}	SSP4Base + 0x84 */
#define	ABUCPR		(0x88)  /* {00, codec_read_pointer, 00, codec_write_pointer}	SSP4Base + 0x88 */
#define	ABUADSR		(0x8c)  /* ABU	Auto DMA Size Register				SSP4Base + 0x8c */
#define	ABUDR		(0x90)  /* ABU	Data Register					SSP4Base + 0x90 */


#endif /* __ASM_ARCH_REGS_ABU_H */
