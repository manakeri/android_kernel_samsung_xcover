/*
 * (C) Copyright 2010 Marvell International Ltd.
 * All Rights Reserved
*/

#ifndef _SEH_LINUX_KERNEL_H
#define _SEH_LINUX_KERNEL_H

#include "eeh_ioctl.h"

#define GCWSBR	0x42404008 // General Register (CWSBR) 
#define TAI 0x48100F00

#define XLLP_ICMR2_BIT3				( 1u << 3 ) // Set to 1 for unmasking interrupt in ICU; 0 for masking interrupt in ICU
#define XLLP_CWSBR_CWSBV			( 1u << 2 ) // Configure for BPMU Wakeups
#define XLLP_CWSBR_CWSB				( 1u << 1 ) // CWSB: 0--ready to capture Comm WDT event; 1-- held in reset and Sticky bit's output is cleared
#define XLLP_CWSBR_CWRE				( 1u << 0 ) //CWRE: 0 --once Comm WDT IRQ asserted, not reset the Comm;1--once Comm WDT IRQ asserted, reset the Comm

#define XLLP_OIER_RESERVED_BITS (0xFFFFF000)

struct seh_dev {
	EehMsgStruct msg;
	struct platform_device *dev;
	struct semaphore read_sem;		  /* mutual exclusion semaphore */
	wait_queue_head_t readq;			  /* read queue */
	void __iomem *cwsbr;
};

#if !defined (ADDR_CONVERT)    /* May be defined in EE_Postmortem.h or loadTable.h */
#define TAVOR_ADDR_SELECT_MASK            0xFF000000
#define PHY_OFFSET(addr) ((addr)&~TAVOR_ADDR_SELECT_MASK)

#if defined (OSA_LINUX)
/* Get the CP area address from kernel: this should be the top 16MB
of the DDR bank#0 LOWEST alias, i.e. 0x80000000+actual bank size-16MB */
extern unsigned cp_area_addr(void); /* arch/arm/mach-pxa/pxa930.c */
/* USE OF THIS IN USER SPACE IS PROHIBITED,
 will result in unresolved external error during link */
#define CP_AREA_PHYSICAL_ADDRESS	(cp_area_addr())
#else
#define CP_AREA_PHYSICAL_ADDRESS      0xBF000000
#endif

#define BOERNE_ADDR_EXEC_REGION_BASE      CP_AREA_PHYSICAL_ADDRESS
#define BOERNE_ADDR_EXEC_REGION_BASE_ORIG  BOERNE_ADDR_EXEC_REGION_BASE
#define HARBELL_ADDR_EXEC_REGION_BASE     0xD0000000

#define RESET_ROUTINE_ARBEL_ADDR   0xD0FFFFE0
#define ARBEL_PHY_MEM_ADDR ((CP_AREA_PHYSICAL_ADDRESS)+0xA00000)
#define ARBEL_IMAGE_SIZE 0x500000
#define GRAYBACK_PHY_MEM_ADDR ((CP_AREA_PHYSICAL_ADDRESS)+0xF00000)
#define GRAYBACK_IMAGE_SIZE 1048512
#define ARBEL_GRAYBACK_OFFSET 0x700000

#if defined (_TAVOR_HARBELL_)
#define ADDR_CONVERT(aDDR)    ((UINT8*)(((UINT32)(aDDR) &  ~TAVOR_ADDR_SELECT_MASK) | HARBELL_ADDR_EXEC_REGION_BASE))
#else /* _TAVOR_BOERNE_) */
#define ADDR_CONVERT(aDDR)    ((UINT8*)(((UINT32)(aDDR) &  ~TAVOR_ADDR_SELECT_MASK) | BOERNE_ADDR_EXEC_REGION_BASE)) //+
#endif

#endif //ADDR_CONVERT

/* Hard-coded address of COMM image RO segment start: cannot fetch this through COMM reset vector at 0xd0ffffc0
   as upper COMM DDR area might be not accessible from AP due to TAI register setting */
#define COMM_RO_ADDRESS 0xd0900000
#define COMM_RW_ADDRESS 0xd0000000	/* the COMM RW starts in 0xd0400000.
We use the OBM and uboot area for diag-POOL in comm -
to get larger pool for offline trace collection */

#define SPECIAL_APP_POOL_SIZE     (3 *  128 * 1024)     //SHMEM memory (APPS to COMM) = 368k
#define SPECIAL_COM_POOL_SIZE     (9 * 1024 * 1024) /*was
(5 * 1024 * 1024) now we map 9Meg of memory
(the 5M RW of Comm + 4M for diag-pool (OBM, Uboot area) */
#define APP_POOL_START_ADDR	      (COMM_RO_ADDRESS - SPECIAL_APP_POOL_SIZE)
#define LOAD_TABLE_FIX_ADDR       0xd0400000 /* was COMM_RW_ADDRESS
- used in the Driver's code */

#define LOAD_TABLE_OFFSET       0
#define LOAD_TABLE_SIGN_OFFSET  0x01C0
#define LT_APP2COM_DATA_LEN     48

typedef struct
{
	UINT32 b2init;                         /* branch to init routine */
	UINT32 init;                           /* image init routine */
	UINT32 addrLoadTableRWcopy;            /* This CONST table is copied into RW area pointed by this*/
	UINT32 ee_postmortem;                  /* EE_PostmortemDesc addr (should be on offset 0xC */
	UINT32 numOfLife;                      /* Increment for every restart (life) */

    UINT32 diag_p_diagIntIfQPtrData;       //Pointer to DIAG-Pointer
    UINT32 diag_p_diagIntIfReportsList;    //Pointer to DIAG-Pointer
    UINT32 spare_CCCC;                     //spare currently used with "CCCC" pattern
                                           // one-direction Apps2com channel for generic command/data
	UINT32 initState;                      //address of BSP_InitStateE COMM state
	UINT32 mipsRamBuffer;                  //address of BSP_InitStateE COMM state
    //UINT16 apps2commDataFormat;
    UINT32 apps2commDataLen;
    UINT8  apps2commDataBuf[LT_APP2COM_DATA_LEN];

    UINT8  filler[LOAD_TABLE_SIGN_OFFSET-(4*11)-LT_APP2COM_DATA_LEN];  //backwards compatible filler to meet 0x1c0 OBM load_table address

	UINT32 imageBegin;                     // image addresses are in HARBELL address space 0xD0??.????
    UINT32 imageEnd;                       // for BOERNE use conversion to 0xBF??.????
    char   Signature[16];
    UINT32 sharedFreeBegin;
    UINT32 sharedFreeEnd;
    UINT32 ramRWbegin;
    UINT32 ramRWend;
	UINT32 spare_EEEE;                     //spare currently used with "EEEE" pattern
    UINT32 ReliableDataBegin;
    UINT32 ReliableDataEnd;

    char   OBM_VerString[8];  //All "OBM" here are temp solution
    UINT32 OBM_Data32bits;
}LoadTableType; /*total size 512bytes */

#define LOAD_TABLE_SIGNATURE_STR  "LOAD_TABLE_SIGN"  /*15 + zeroEnd */
#define INVALID_ADDRESS           0xBAD0DEAD

extern UINT32 getCommImageBaseAddr(void);
extern void   getAppComShareMemAddr(UINT32* begin, UINT32* end);

#if defined (OSA_WINCE) || defined (OSA_LINUX)
UINT32 ConvertPhysicalAddrToVirtualAddr(UINT32 PhysicalAddr);
void   commImageTableInit(void);
void commImageTableFree(void);

#define MAP_PHYSICAL_TO_VIRTUAL_ADDRESS(pHYaddr)    /*Returns (UINT32) virtual*/ \
			       ConvertPhysicalAddrToVirtualAddr((UINT32)(pHYaddr))
#else

#define MAP_PHYSICAL_TO_VIRTUAL_ADDRESS((pHYaddr))    (pHYaddr)
#endif

typedef enum
{
	RESET_BASIC_NONE = 0,
	RESET_BASIC_1    = 0x7CAFE001, //while(1)
	RESET_BASIC_2    = 0x7CAFE002, //LogStream
	RESET_BASIC_3    = 0x7CAFE003  //for future use
}enumResetType;

#ifndef _LOAD_TABLE_C_
extern LoadTableType  loadTable;   /* do NOT use "const" in EXTERN prototype */
#endif

extern LoadTableType  *pLoadTable;


#endif

