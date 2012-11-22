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
* mvLog.h - Header File for CORE driver logger
*
* DESCRIPTION:
*       None.
*
* DEPENDENCIES:
*
*******************************************************************************/
#ifndef __INCmvLogh
#define __INCmvLogh

#ifdef __cplusplus
extern "C" /*{*/
#endif /* __cplusplus */

/*-------------H file-----------------------------*/
#define MV_DEBUG_FATAL_ERROR                    0x01
#define MV_DEBUG_ERROR                          0x02
#define MV_DEBUG_INIT                           0x04
#define MV_DEBUG_INTERRUPTS                     0x08
#define MV_DEBUG_SATA_LINK                      0x10
#define MV_DEBUG_UDMA_COMMAND                   0x20
#define MV_DEBUG_NON_UDMA_COMMAND               0x40
#define MV_DEBUG_PM                             0x80
#define MV_DEBUG                                0x100



#define MV_MAX_LOG_MODULES         16
#define MV_MAX_MESSAGE_TYPE        9
#define MV_RAW_MSG_ID              0xF

typedef struct
{
    MV_BOOLEAN      used;
    MV_U32          filterMask;
    char            *name;
    char            *filters;
} MV_LOG_FILTER_HEADER;


#if defined (MV_LOG_DEBUG) || defined (MV_LOG_ERROR)
    #define MV_LOGGER       1
    #if defined (WIN32)
ULONG
_cdecl
DbgPrint(
        PCH Format,
        ...
        );
        #define MV_LOG_PRINT    DbgPrint
    #elif defined (LINUX)
        #define MV_LOG_PRINT    printk
    #else
        #define MV_LOG_PRINT	printf
    #endif


MV_BOOLEAN mvLogRegisterModule(MV_U8 moduleId, MV_U32 filterMask, char* name);
MV_BOOLEAN mvLogSetModuleFilter(MV_U8 moduleId, MV_U32 filterMask);
MV_U32 mvLogGetModuleFilter(MV_U8 moduleId);
void mvLogMsg(MV_U8 moduleId, MV_U32 type, char* format, ...);

#else /*defined (MV_LOG_DEBUG) || defined (MV_LOG_ERROR)*/

    #undef MV_LOGGER

    #if defined (WIN32)
        #define MV_LOG_PRINT
        #define mvLogRegisterModule
        #define mvLogGetModuleFilter
        #define mvLogRegisterAllModules
        #define mvLogMsg

    #elif defined (LINUX)
        #define MV_LOG_PRINT(x...)
        #define mvLogRegisterModule(x...)
        #define mvLogSetModuleFilter(x...)
        #define mvLogGetModuleFilter(x...)
        #define mvLogRegisterAllModules(x...)
        #define mvLogMsg(x...)

    #else
        #define MV_LOG_PRINT
        #define mvLogRegisterModule
        #define mvLogSetModuleFilter
        #define mvLogGetModuleFilter
        #define mvLogRegisterAllModules
        #define mvLogMsg
    #endif

#endif /*!defined (MV_LOG_DEBUG) && !defined (MV_LOG_ERROR)*/

#ifdef __cplusplus

/*}*/
#endif /* __cplusplus */

#endif


