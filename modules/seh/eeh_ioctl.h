/*
 * (C) Copyright 2010 Marvell International Ltd.
 * All Rights Reserved
 */

#ifndef __EEH_IOCTL__
#define __EEH_IOCTL__

#include "linux_types.h"

/* The retun value of the EEH API finction calls. */
typedef	UINT32	EEH_STATUS;

/* Error code */
#define	EEH_NO_ERROR							0
#define EEH_SUCCESS                             EEH_NO_ERROR
#define	EEH_ERROR								(EEH_NO_ERROR-1)
#define	EEH_ERROR_INIT							(EEH_NO_ERROR-2)
#define EEH_MSG_DESC_LEN						512

#define SEH_IOC_MAGIC                   'Y'
#define SEH_IOCTL_API                   _IOW(SEH_IOC_MAGIC, 1, int)
#define SEH_IOCTL_TEST                  _IOW(SEH_IOC_MAGIC, 2, int)
#define SEH_IOCTL_ATC_OK                _IOW(SEH_IOC_MAGIC, 3, int)
#define SEH_IOCTL_APP_ASSERT            _IOW(SEH_IOC_MAGIC, 4, int)
#define SEH_IOCTL_AUDIOSERVER_REL_COMP  _IOW(SEH_IOC_MAGIC, 5, int)
#define SEH_IOCTL_DONT_SLEEP            _IOW(SEH_IOC_MAGIC, 6, int)
#define SEH_IOCTL_SLEEP_ALLOWED         _IOW(SEH_IOC_MAGIC, 7, int)
#define SEH_IOCTL_EEH_TIMER_EXPIRED     _IOW(SEH_IOC_MAGIC, 8, int)
#define SEH_IOCTL_MIPSRAM_STOP  	    _IOW(SEH_IOC_MAGIC, 9, int)
#define SEH_IOCTL_MIPSRAM_START         _IOW(SEH_IOC_MAGIC, 10, int)
#define SEH_IOCTL_MIPSRAM_ADD_TRACE     _IOW(SEH_IOC_MAGIC, 11, int)
#define SEH_IOCTL_MIPSRAM_DUMP			_IOW(SEH_IOC_MAGIC, 12, int)
#define SEH_IOCTL_EMULATE_PANIC			_IOW(SEH_IOC_MAGIC, 13, int)
#define SEH_IOCTL_SET_ERROR_INFO		_IOW(SEH_IOC_MAGIC, 14, int)
#define SEH_IOCTL_GET_GPIO_SD_DETECT_INSERTED	_IOW(SEH_IOC_MAGIC, 15, int)


typedef enum _EehApiId
{
    _EehInit	= 0,
    _EehDeInit,
	_EehInsertComm2Reset,
	_EehReleaseCommFromReset,
	_EehExecReset,
	_EehExecGpioReset,
    _EehCopyCommImageFromFlash,
    _EehGetTid,
    _EehTestAssertKernel,
	_EehTestAssertKernelIRQ,
	_EehTestCpAssert,
	_EehExecRFICReset
}EehApiId;

typedef struct _EehApiParams
{
    int eehApiId;
    UINT32  status;
    void *params;
} EehApiParams;

typedef struct _EehCopyCommImageFromFlashParam
{
	UINT32 ArbelAddr;
	UINT32 ArbelSize;
	UINT32 GraybackAddr;
	UINT32 GraybackSize;
	UINT32 ArbelGraybackOffset;
}EehCopyCommImageFromFlashParam;

typedef enum _EehMsgInd
{
    EEH_INVALID_MSG	= 0,
    EEH_WDT_INT_MSG,
    EEH_ATC_OK_MSG,
    EEH_AP_ASSERT_MSG,
    EEH_AUDIOSERVER_REL_COMP_MSG,
    EEH_TIMER_EXPIRED_MSG,
	EEH_MIPSRAM_DUMP
}EehMsgInd;

typedef enum _EehAssertType
{
	EEH_NONE_ASSERT = 0,
	EEH_AP_ASSERT,
	EEH_CP_ASSERT,
	EEH_CP_WDT
}EehAssertType;

typedef struct _EehInsertComm2ResetParam
{
	UINT32 AssertType;
}EehInsertComm2ResetParam;

typedef struct _EehMsgStruct
{
	UINT32 msgId;
	char msgDesc[EEH_MSG_DESC_LEN];
}EehMsgStruct;

typedef struct _EehAppAssertParam
{
	char msgDesc[EEH_MSG_DESC_LEN];
}EehAppAssertParam;

typedef struct _EehErrorInfo
{
	unsigned err;
	char	 *str;
	unsigned char *regs;
} EehErrorInfo;

/* err: SEH converts these into what defined in kernel ramdump_defs.h file */
#define ERR_EEH_CP 0
#define ERR_EEH_AP 1

#endif


