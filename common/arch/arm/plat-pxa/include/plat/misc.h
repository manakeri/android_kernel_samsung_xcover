#ifndef __PLAT_MISC_H
#define __PLAT_MISC_H

#define MARVELL_BONNELL_REV1		0x06
#define MARVELL_BONNELL_REV2		0x02

#define MARVELL_BROWNSTONE_REV1		0x04
#define MARVELL_BROWNSTONE_REV2		0x03

extern int get_board_version(void);
extern void set_board_version(int);

#endif
