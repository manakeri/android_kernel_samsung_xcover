/*
** (C) Copyright 2009 Marvell International Ltd.
**  		All Rights Reserved

** This software file (the "File") is distributed by Marvell International Ltd. 
** under the terms of the GNU General Public License Version 2, June 1991 (the "License"). 
** You may use, redistribute and/or modify this File in accordance with the terms and 
** conditions of the License, a copy of which is available along with the File in the 
** license.txt file or by writing to the Free Software Foundation, Inc., 59 Temple Place, 
** Suite 330, Boston, MA 02111-1307 or on the worldwide web at http://www.gnu.org/licenses/gpl.txt.
** THE FILE IS DISTRIBUTED AS-IS, WITHOUT WARRANTY OF ANY KIND, AND THE IMPLIED WARRANTIES 
** OF MERCHANTABILITY OR FITNESS FOR A PARTICULAR PURPOSE ARE EXPRESSLY DISCLAIMED.  
** The License provides additional details about this warranty disclaimer.
*/

/* (C) Copyright 2009 Marvell International Ltd. All Rights Reserved */

#ifndef __VDK_IO_CODE_H__
#define __VDK_IO_CODE_H__

#include <asm/ioctl.h>

#define PX_MAGIC 'P'
/* io control code of Counter Monitor profiler kernel driver */
#define PX_CM_CMD_ALLOC_BUFFER                        _IOWR(PX_MAGIC, 0, unsigned int)
#define PX_CM_CMD_FREE_BUFFER                         _IO(PX_MAGIC, 1)
#define PX_CM_CMD_SET_COUNTER                         _IOW(PX_MAGIC, 2, struct CMCounterConfigs)
#define PX_CM_CMD_START_KERNEL_DRIVER                 _IOW(PX_MAGIC, 3, bool)
#define PX_CM_CMD_STOP_KERNEL_DRIVER                  _IO(PX_MAGIC, 4)
#define PX_CM_CMD_PAUSE_KERNEL_DRIVER                 _IO(PX_MAGIC, 5)
#define PX_CM_CMD_RESUME_KERNEL_DRIVER                _IO(PX_MAGIC, 6)
#define PX_CM_CMD_QUERY_REQUEST                       _IOR(PX_MAGIC, 7, unsigned int)
#define PX_CM_CMD_GET_CPU_ID                          _IOR(PX_MAGIC, 8, int)
#define PX_CM_CMD_GET_TIMESTAMP_FREQ                  _IOR(PX_MAGIC, 9, unsigned long long)
#define PX_CM_CMD_GET_OS_TIMER_FREQ                   _IOR(PX_MAGIC, 10, unsigned long long)
#define PX_CM_CMD_READ_COUNTER                        _IOWR(PX_MAGIC, 11, struct read_counter_data)
#define PX_CM_CMD_GET_TIMESTAMP                       _IOR(PX_MAGIC, 12, unsigned long long)
#define PX_CM_CMD_SET_MODE                            _IOW(PX_MAGIC, 13, struct set_mode_data)
#define PX_CM_CMD_GET_CPU_FREQ                        _IOR(PX_MAGIC, 14, unsigned long)
#define PX_CM_CMD_SET_AUTO_LAUNCH_APP_PID             _IOW(PX_MAGIC, 15, pid_t)
#define PX_CM_CMD_RESET_BUFFER_FULL                   _IOW(PX_MAGIC, 16, unsigned int)
#define PX_CM_CMD_READ_COUNTER_IN_SPECIFIC_MODE       _IOR(PX_MAGIC, 17, unsigned long long)

#define KDR_PROCESS_CREATE_BUFFER_FULL   0x00000001
#define KDR_THREAD_CREATE_BUFFER_FULL    0x00000002
#define KDR_THREAD_SWITCH_BUFFER_FULL    0x00000004
#define KDR_LAUNCHED_APP_EXIT            0x00000008

/* structure for PX_CM_CMD_READ_COUNTER */
struct read_counter_data 
{
	unsigned int        cid;
	unsigned long long  value;
};

/* structure for PX_CM_CMD_SET_MODE */
struct set_mode_data
{
    unsigned long   mode;
    unsigned long   specific_pid;
    unsigned long   specific_tid;
};

/* io control code of Hotspot profiler kernel driver */
#define PX_HS_CMD_START_SAMPLING                       _IOW(PX_MAGIC, 0, bool)
#define PX_HS_CMD_STOP_PROFILING                       _IO(PX_MAGIC, 1)
#define PX_HS_CMD_PAUSE_PROFILING                      _IO(PX_MAGIC, 2)
#define PX_HS_CMD_RESUME_PROFILING                     _IO(PX_MAGIC, 3)
#define PX_HS_CMD_ALLOC_SAMPLE_BUFFER                  _IOWR(PX_MAGIC, 4, unsigned int)
#define PX_HS_CMD_ALLOC_MODULE_BUFFER                  _IOWR(PX_MAGIC, 5, unsigned int)
#define PX_HS_CMD_FREE_SAMPLE_BUFFER                   _IO(PX_MAGIC, 6)
#define PX_HS_CMD_FREE_MODULE_BUFFER                   _IO(PX_MAGIC, 7)
#define PX_HS_CMD_QUERY_REQUEST                        _IOR(PX_MAGIC, 8, unsigned int)
#define PX_HS_CMD_SET_AUTO_LAUNCH_APP_PID              _IOW(PX_MAGIC, 9, pid_t)
#define PX_HS_CMD_SET_WAIT_IMAGE_LOAD_NAME             _IOW(PX_MAGIC, 10, char)
#define PX_HS_CMD_SET_TBS_SETTINGS                     _IOW(PX_MAGIC, 11, struct HSTimerSettings)
#define PX_HS_CMD_SET_EBS_SETTINGS                     _IOW(PX_MAGIC, 12, struct HSEventSettings)
#define PX_HS_CMD_GET_CALIBRATION_RESULT               _IOWR(PX_MAGIC, 13, struct calibration_result)
#define PX_HS_CMD_GET_CPU_ID                           _IOR(PX_MAGIC, 14, unsigned int)
#define PX_HS_CMD_GET_CPU_FREQ                         _IOR(PX_MAGIC, 15, unsigned int)
#define PX_HS_CMD_SET_CALIBRATION_MODE                 _IOW(PX_MAGIC, 16, bool)
#define PX_HS_CMD_GET_TIMESTAMP_FREQ                   _IOR(PX_MAGIC, 17, unsigned long)
#define PX_HS_CMD_ADD_MODULE_RECORD                    _IOW(PX_MAGIC, 18, struct add_module_data)
#define PX_HS_CMD_RESET_SAMPLE_BUFFER_FULL             _IOW(PX_MAGIC, 19, bool)
#define PX_HS_CMD_RESET_MODULE_BUFFER_FULL             _IOW(PX_MAGIC, 20, bool)
#define PX_HS_CMD_START_MODULE_TRACKING                _IO(PX_MAGIC, 21)

/* request from Hotspot profiler kernel driver*/
#define KDR_HS_SAMPLE_BUFFER_FULL      0x00000001
#define KDR_HS_MODULE_BUFFER_FULL      0x00000002
#define KDR_HS_LAUNCHED_APP_EXIT       0x00000004
#define KDR_HS_WAIT_IMAGE_LOADED       0x00000008

/* structure for PX_HS_CMD_SET_DATA_COLLECTOR_PID */
struct calibration_result
{
	int                register_id;
	unsigned long long event_count;
};

/* structure for PX_HS_CMD_ADD_MODULE_RECORD */
struct add_module_data
{
	//char         name[PATH_MAX];
	char *       name;
	unsigned int name_offset;
	unsigned int address;
	unsigned int size;
	unsigned int pid;
	unsigned int lsc;
	unsigned int flag;
};

/* io control code of Call Stack Sampling profiler kernel driver */
#define PX_CSS_CMD_START_SAMPLING                       _IOW(PX_MAGIC, 0, bool)
#define PX_CSS_CMD_STOP_PROFILING                       _IO(PX_MAGIC, 1)
#define PX_CSS_CMD_PAUSE_PROFILING                      _IO(PX_MAGIC, 2)
#define PX_CSS_CMD_RESUME_PROFILING                     _IO(PX_MAGIC, 3)
#define PX_CSS_CMD_ALLOC_SAMPLE_BUFFER                  _IOWR(PX_MAGIC, 4, unsigned int)
#define PX_CSS_CMD_ALLOC_MODULE_BUFFER                  _IOWR(PX_MAGIC, 5, unsigned int)
#define PX_CSS_CMD_FREE_SAMPLE_BUFFER                   _IO(PX_MAGIC, 6)
#define PX_CSS_CMD_FREE_MODULE_BUFFER                   _IO(PX_MAGIC, 7)
#define PX_CSS_CMD_QUERY_REQUEST                        _IOR(PX_MAGIC, 8, unsigned int)
#define PX_CSS_CMD_SET_AUTO_LAUNCH_APP_PID              _IOW(PX_MAGIC, 9, pid_t)
#define PX_CSS_CMD_SET_WAIT_IMAGE_LOAD_NAME             _IOW(PX_MAGIC, 10, char)
#define PX_CSS_CMD_SET_TBS_SETTINGS                     _IOW(PX_MAGIC, 11, struct CSSTimerSettings)
#define PX_CSS_CMD_SET_EBS_SETTINGS                     _IOW(PX_MAGIC, 12, struct CSSEventSettings)
#define PX_CSS_CMD_GET_CALIBRATION_RESULT               _IOWR(PX_MAGIC, 13, struct calibration_result)
#define PX_CSS_CMD_GET_CPU_ID                           _IOR(PX_MAGIC, 14, unsigned int)
#define PX_CSS_CMD_GET_CPU_FREQ                         _IOR(PX_MAGIC, 15, unsigned int)
#define PX_CSS_CMD_SET_CALIBRATION_MODE                 _IOW(PX_MAGIC, 16, bool)
#define PX_CSS_CMD_GET_TIMESTAMP_FREQ                   _IOR(PX_MAGIC, 17, unsigned long)
#define PX_CSS_CMD_ADD_MODULE_RECORD                    _IOW(PX_MAGIC, 18, struct add_module_data)
#define PX_CSS_CMD_RESET_SAMPLE_BUFFER_FULL             _IOW(PX_MAGIC, 19, bool)
#define PX_CSS_CMD_RESET_MODULE_BUFFER_FULL             _IOW(PX_MAGIC, 20, bool)
#define PX_CSS_CMD_START_MODULE_TRACKING                _IO(PX_MAGIC, 21)

/* request from Hotspot profiler kernel driver*/
#define KDR_CSS_SAMPLE_BUFFER_FULL      0x00000001
#define KDR_CSS_MODULE_BUFFER_FULL      0x00000002
#define KDR_CSS_LAUNCHED_APP_EXIT       0x00000004
#define KDR_CSS_WAIT_IMAGE_LOADED       0x00000008

#define PX_HS_DRV_NAME            "/dev/px_hs_d"
#define PX_HS_SAMPLE_DRV_NAME     "/dev/pxhs_sample_d"
#define PX_HS_MODULE_DRV_NAME     "/dev/pxhs_module_d"
#define PX_HS_DSA_DRV_NAME        "/dev/pxhs_dsa_d"

#define PX_CM_DRV_NAME            "/dev/px_cm_d"
#define PX_CM_PC_DRV_NAME         "/dev/pxcm_pc_d"
#define PX_CM_TC_DRV_NAME         "/dev/pxcm_tc_d"
#define PX_CM_TS_DRV_NAME         "/dev/pxcm_ts_d"

#define PX_CSS_DRV_NAME            "/dev/px_css_d"
#define PX_CSS_SAMPLE_DRV_NAME     "/dev/pxcss_sample_d"
#define PX_CSS_MODULE_DRV_NAME     "/dev/pxcss_module_d"
#define PX_CSS_DSA_DRV_NAME        "/dev/pxcss_dsa_d"

#endif /* __VDK_IO_CODE_H__ */
