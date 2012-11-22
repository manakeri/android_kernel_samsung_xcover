/* 
 *
 * Zinitix touch driver
 *
 * Copyright (C) 2009 Zinitix, Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
// Version 2.0.0 : using reg data file (2010/11/05)
// Version 2.0.1 : syntxt bug fix (2010/11/09)
// Version 2.0.2 : Save status cmd delay bug (2010/11/10)
// Version 2.0.3 : modify delay 10ms -> 50ms for clear hw calibration bit
//		: modify ZINITIX_TOTAL_NUMBER_OF_Y register (read only -> read/write )
//		: modify SUPPORTED FINGER NUM register (read only -> read/write )
// Version 2.0.4 : [20101116]
//	Modify Firmware Upgrade routine.
// Version 2.0.5 : [20101118]
//	add esd timer function & some bug fix.
//	you can select request_threaded_irq or request_irq, setting USE_THREADED_IRQ.
// Version 2.0.6 : [20101123]
//	add ABS_MT_WIDTH_MAJOR Report
// Version 2.0.7 : [20101201]
//	Modify zinitix_early_suspend() / zinitix_late_resume() routine.
// Version 2.0.8 : [20101216]
//	add using spin_lock option
// Version 2.0.9 : [20101216]
//	Test Version
// Version 2.0.10 : [20101217]
//	add USE_THREAD_METHOD option. if  USE_THREAD_METHOD = 0, you use workqueue
// Version 2.0.11 : [20101229]
//	add USE_UPDATE_SYSFS option for update firmware. && TOUCH_MODE == 1 mode.
// Version 2.0.13 : [20110125]
//	modify esd timer routine
// Version 2.0.14 : [20110217]
//	esd timer bug fix. (kernel panic)
//	sysfs bug fix.
// Version 2.0.15 : [20110315]
//	add power off delay ,250ms
// Version 2.0.16 : [20110316]
//	add upgrade method using isp
// Version 2.0.17 : [20110406]
//	change naming rule : sain -> zinitix
//    (add) pending interrupt skip
//	add isp upgrade mode
//	remove warning message when complile

// Version 3.0.2 : [20110711]
//   support bt4x3 series
// Version 3.0.3 : [20110720]
//   add raw data monitoring func.
//   add the h/w calibration skip option.
// Version 3.0.4 : [20110728]
//   use threaded irq
// Version 3.0.5 : [20110801]
//   fix some bugs.


#include <linux/module.h>
#include <linux/input.h>
#include <linux/i2c.h>		// I2C_M_NOSTART
#include <linux/miscdevice.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/hrtimer.h>
#include <linux/ioctl.h>
#include <linux/earlysuspend.h>
#include <linux/string.h>
#include <linux/semaphore.h>
#include <linux/kthread.h>
#include <linux/timer.h>
#include <linux/workqueue.h>
#include <linux/firmware.h>
#include <linux/regulator/consumer.h>
#include <linux/regulator/machine.h>
#include <linux/slab.h>
#include <mach/mfp-pxa9xx.h>
#include <asm/io.h>
#include <linux/delay.h>
#include <mach/gpio.h>


#include "zinitix_touch.h"

#if	BT4x2_Series
#include "zinitix_touch_firmware.h"
#include "zinitix_touch_reg_data.h"
#endif


#ifdef CONFIG_PXA95x_DVFM
#include <mach/dvfm.h>
#include <mach/pxa95x_dvfm.h>
#endif

#include <mach/hardware.h>

#define	ZINITIX_DEBUG		0

#define	SYSTEM_MAX_X_RESOLUTION	319//4000	//480
#define	SYSTEM_MAX_Y_RESOLUTION	479 //3900	//800

#if USE_THREADED_IRQ
/* Do Nothing */
#else
static struct workqueue_struct *zinitix_workqueue;
#endif

#if	ZINITIX_ESD_TIMER_INTERVAL
static struct workqueue_struct *zinitix_tmr_workqueue;
#endif

#if	ZINITIX_DEBUG
#define	zinitix_debug_msg(fmt, args...)	printk(KERN_INFO "[TSP: %-18s:%5d]" fmt, __FUNCTION__, __LINE__, ## args)
#else
#define	zinitix_debug_msg(fmt, args...)	do{}while(0)
#endif

//-------------------------------------------------------
typedef	struct	
{
	u16	x;
	u16	y;
	u8	width;	
	u8	sub_status;
}_ts_zinitix_coord;

typedef	struct	
{
	u16	status;
#if (TOUCH_MODE == 1)
	u16	event_flag;
#else
	u8	finger_cnt;
	u8	time_stamp;
#endif	
	_ts_zinitix_coord	coord[SUPPORTED_FINGER_NUM];

}_ts_zinitix_point_info;


#define	TOUCH_V_FLIP	0x01
#define	TOUCH_H_FLIP	0x02
#define	TOUCH_XY_SWAP	0x04

typedef	struct
{
	u16 chip_revision; 
	u16 chip_firmware_version;
	u16 chip_reg_data_version;		  
	u32 chip_fw_size;	
	u32 MaxX;
	u32 MaxY;
	u32 MinX;
	u32 MinY;
	u32 Orientation;
	u8 gesture_support;
	u8 multi_fingers;
#if	USE_TEST_RAW_TH_DATA_MODE
	u16	x_node_num;
	u16	y_node_num;
	u16	total_node_num;
#endif	
}_ts_capa_info;

typedef	enum
{
	TS_NO_WORK = 0,
	TS_NORMAL_WORK,
	TS_ESD_TIMER_WORK,
	TS_IN_EALRY_SUSPEND,
	TS_IN_SUSPEND,
	TS_IN_RESUME,
	TS_IN_LATE_RESUME,
	TS_IN_UPGRADE,
	TS_REMOVE_WORK,
	TS_SET_MODE,
}_ts_work_proceedure;


typedef struct
{
	struct input_dev *input_dev;
	struct task_struct *task;
	wait_queue_head_t	wait;
	struct work_struct  work;
	struct work_struct  tmr_work;
	struct i2c_client *client;
	struct semaphore update_lock;    
	u32 i2c_dev_addr;
	_ts_capa_info	cap_info;
	char	phys[32];
	
	bool is_valid_event;
	_ts_zinitix_point_info touch_info;
	_ts_zinitix_point_info reported_touch_info;    	
	u16 icon_event_reg;
	u16 chip_int_mask;    
	u16 event_type;
	u32 int_gpio_num;
	u32 irq;
	u8 button[SUPPORTED_BUTTON_NUM];

	u8	work_proceedure;	
	struct semaphore	work_proceedure_lock;
	
	u8	use_esd_timer;
#if	ZINITIX_ESD_TIMER_INTERVAL
	bool in_esd_timer;
	struct timer_list esd_timeout_tmr;		//for repeated card detecting work
	struct timer_list *p_esd_timeout_tmr;		//for repeated card detecting work
#endif

#ifdef CONFIG_HAS_EARLYSUSPEND	
	struct early_suspend early_suspend;
#endif

#if	USE_TEST_RAW_TH_DATA_MODE
	struct semaphore	raw_data_lock;
	u8	raw_mode_flag;
	s16	ref_data[MAX_TEST_RAW_DATA];
	s16 cur_data[MAX_RAW_DATA];
#endif
	unsigned long board_rev;
} zinitix_touch_dev;


#define ZINITIX_DRIVER_NAME        "zinitix_touch"

#if	TOUCH_USING_ISP_METHOD
#define ZINITIX_ISP_NAME        "zinitix_isp"	
struct i2c_client *m_isp_client = NULL;
#endif


static struct i2c_device_id zinitix_idtable[] = {
    {ZINITIX_DRIVER_NAME, 0},  	// i2c register.
#if	TOUCH_USING_ISP_METHOD
    {ZINITIX_ISP_NAME, 0},
#endif
    { }   
};


u32 BUTTON_MAPPING_KEY[SUPPORTED_BUTTON_NUM];	//={KEY_SEARCH, KEY_BACK, KEY_HOME, KEY_MENU}; //<= you must set key button mapping

extern long get_board_id();
// define sub functions
//==================================================================

/* + SKC on110725  porting BT402B TSP*/
#define GPIO_TOUCH_PIN_NUM  23

struct regulator *tsp_regulator; 
struct regulator *tsp_regulator2; 
static int tsp_ldo_en =0;
zinitix_touch_dev* g_touch_dev;
#ifdef CONFIG_PXA95x_DVFM
static int dvfm_dev_idx;
static void set_dvfm_constraint(void)
{
	/* Disable Lowpower mode */
	dvfm_disable_op_name("D0CS", dvfm_dev_idx);
	dvfm_disable_op_name("D1", dvfm_dev_idx);
	dvfm_disable_op_name("D2", dvfm_dev_idx);
	dvfm_disable_op_name("CG", dvfm_dev_idx);
	/* Disable below frequency since they only support DSI clock up to 156Mhz */
	dvfm_disable_op_name("156M", dvfm_dev_idx);
	dvfm_disable_op_name("208M", dvfm_dev_idx);
	dvfm_disable_op_name("156M_HF", dvfm_dev_idx);

}

static void unset_dvfm_constraint(void)
{
	/* Enable Lowpower mode */
	dvfm_enable_op_name("D0CS", dvfm_dev_idx);
	dvfm_enable_op_name("D1", dvfm_dev_idx);
	dvfm_enable_op_name("D2", dvfm_dev_idx);
	dvfm_enable_op_name("CG", dvfm_dev_idx);
	/* Enable frequency */
	dvfm_enable_op_name("156M", dvfm_dev_idx);
	dvfm_enable_op_name("208M", dvfm_dev_idx);
	dvfm_enable_op_name("156M_HF", dvfm_dev_idx);
}



static void set_tsp_dvfm_constraint(void)
{
	/* Disable Lowpower mode */
	dvfm_disable_op_name("D0CS", dvfm_dev_idx);
	dvfm_disable_op_name("D1", dvfm_dev_idx);
	dvfm_disable_op_name("D2", dvfm_dev_idx);
	dvfm_disable_op_name("CG", dvfm_dev_idx);
}

static void unset_tsp_dvfm_constraint(void)
{
	/* Enable Lowpower mode */
	dvfm_enable_op_name("D0CS", dvfm_dev_idx);
	dvfm_enable_op_name("D1", dvfm_dev_idx);
	dvfm_enable_op_name("D2", dvfm_dev_idx);
	dvfm_enable_op_name("CG", dvfm_dev_idx);

}


#else
static void set_dvfm_constraint(void) {}
static void unset_dvfm_constraint(void) {}
#endif

static int touch_power_control(int enable)
{
	int ret;
       static u8 icotl_suppliesEnable;  /* static is '0' initialized */

	if (enable) {
    
         zinitix_debug_msg("touch_power_control ON \n");
		 if(g_touch_dev->board_rev==6)
		 {
			 if(regulator_is_enabled(tsp_regulator2) == 0){
				 ret = regulator_enable(tsp_regulator2);
				 zinitix_debug_msg("regulator_enable 3.3V LDO6 = %d \n",ret);
			 }
			 if(regulator_is_enabled(tsp_regulator) == 0){
				 ret = regulator_enable(tsp_regulator);
				 zinitix_debug_msg("regulator_enable 1.8V LDO8 = %d \n",ret);
			 }
		 }
         
         gpio_direction_output(tsp_ldo_en,1);  
         zinitix_debug_msg("touch_power_control ON : %d \n", ret );
           
	} else {

         zinitix_debug_msg("touch_power_control OFF \n");
		 if(g_touch_dev->board_rev==6)
		 {
         if(regulator_is_enabled(tsp_regulator)){
              ret = regulator_disable(tsp_regulator);
              zinitix_debug_msg("regulator_disable1 = %d \n",ret);
         }

         if(regulator_is_enabled(tsp_regulator2)){
              ret = regulator_disable(tsp_regulator2);
              zinitix_debug_msg("regulator_disable2 = %d \n",ret);
         }   
		 }
         gpio_direction_output(tsp_ldo_en,0);  
         zinitix_debug_msg("touch_power_control OFF : %d \n", ret);
            
	}

	return 0;
}


/* sys fs */
struct class *touch_class;
EXPORT_SYMBOL(touch_class);
struct device *firmware_dev;
EXPORT_SYMBOL(firmware_dev);

static ssize_t firmware_show(struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t firmware_store( struct device *dev, struct device_attribute *attr, const char *buf, size_t size);
static ssize_t firmware_ret_show(struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t firmware_ret_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size);
static ssize_t fimware_show_versname(struct device *dev, struct device_attribute *attr, char *buf);
static DEVICE_ATTR(firmware	, 0444, firmware_show, firmware_store);
static DEVICE_ATTR(firmware_ret	, 0444, firmware_ret_show, firmware_ret_store);
static DEVICE_ATTR(versname, 0444, fimware_show_versname, NULL) ;

static ssize_t raw_show_zinitix(struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t raw_store_zinitix(struct device *dev, struct device_attribute *attr, char *buf, size_t size);
static ssize_t raw_enable_zinitix(struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t raw_disable_zinitix(struct device *dev, struct device_attribute *attr, char *buf);

static ssize_t read_rawdata(struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t read_difference(struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t rawdata_pass_fail_zinitix(struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t diffdata_zinitix(struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t refdata_zinitix(struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t scantime_zinitix(struct device *dev, struct device_attribute *attr, char *buf);

static DEVICE_ATTR(raw, 0444, raw_show_zinitix, NULL) ;
static DEVICE_ATTR(raw_enable, 0444, raw_enable_zinitix, NULL) ;
static DEVICE_ATTR(raw_disable, 0444, raw_disable_zinitix, NULL) ;

static DEVICE_ATTR(rawdata_read, 0444, read_rawdata, NULL) ;
static DEVICE_ATTR(difference_read, 0444, read_difference, NULL) ;
static DEVICE_ATTR(raw_value, 0444, rawdata_pass_fail_zinitix, NULL) ;
static DEVICE_ATTR(diff_value, 0444, diffdata_zinitix, NULL) ;
static DEVICE_ATTR(ref_value, 0444, refdata_zinitix, NULL) ;
static DEVICE_ATTR(tsp_scantime, 0444, scantime_zinitix, NULL) ;
bool    ts_hw_calibration(zinitix_touch_dev* touch_dev);

static int tsp_testmode = 0;
static void	zinitix_clear_report_data(zinitix_touch_dev *touch_dev);

u8 ts_upgrade_firmware(zinitix_touch_dev* touch_dev, const u8 *firmware_data, u32 size);
bool ts_init_touch(zinitix_touch_dev* touch_dev);
/* sys fs */
int firm_update( void )
{	
     return 0;
}

static ssize_t firmware_show(struct device *dev, struct device_attribute *attr, char *buf)
{
     int HW_ver = 2;
     int phone_ver;
     int ic_ver;

     phone_ver = m_firmware_data[0];
     ic_ver =   0x00ff & g_touch_dev->cap_info.chip_firmware_version;
     HW_ver = 0x00ff & g_touch_dev->cap_info.chip_revision;

      zinitix_debug_msg("chip_revision : %d, HW_ver = %d\n", g_touch_dev->cap_info.chip_revision, HW_ver);  // 0xb , 11
      zinitix_debug_msg("chip_firmware_version : %d, phone_ver = %d\n", g_touch_dev->cap_info.chip_firmware_version, phone_ver);  //0x102, 258
      zinitix_debug_msg("phone(m_firmware_data[0]) = %d\n", m_firmware_data[0]); 

     // sprintf(buf, "0%d\n", (HW_ver*100000) + (phone_ver*1000)+(ic_ver*10));
     sprintf(buf, "%02x%02x%02x0\n", HW_ver,phone_ver, ic_ver);

	return sprintf(buf, "%s", buf );
}

static ssize_t fimware_show_versname(struct device *dev, struct device_attribute *attr, char *buf)
{
      int ic_ver;

      ic_ver = 0x00ff & g_touch_dev->cap_info.chip_firmware_version;
      
      sprintf(buf, "0x%d", ic_ver);

	return sprintf(buf, "%s", buf );
}

static ssize_t firmware_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
      char *after;

	unsigned long value = simple_strtoul(buf, &after, 10);	
	zinitix_debug_msg( "firmware_store value = %d, size = %d\n", value, size);
/*
	if ( value == 1 ){
        
		zinitix_debug_msg("firmware update start!!\n" );

            down(&g_touch_dev->work_proceedure_lock);	
	      g_touch_dev->work_proceedure = TS_IN_UPGRADE;	

             disable_irq(g_touch_dev->irq);
             
             set_dvfm_constraint();
		size = ts_upgrade_firmware(g_touch_dev, &m_firmware_data[2], g_touch_dev->cap_info.chip_fw_size);
             unset_dvfm_constraint();

             ts_init_touch(g_touch_dev);

	      g_touch_dev->work_proceedure = TS_NO_WORK;	
	      up(&g_touch_dev->work_proceedure_lock); 

             enable_irq(g_touch_dev->irq);

             zinitix_debug_msg("firmware update end!!\n" );
		return size;
	}
*/
	return size;
}

static ssize_t firmware_ret_show(struct device *dev, struct device_attribute *attr, char *buf)
{
     int size =0;
     
     zinitix_debug_msg("firmware update start!!\n" );

     down(&g_touch_dev->work_proceedure_lock);	
     g_touch_dev->work_proceedure = TS_IN_UPGRADE;	

     disable_irq(g_touch_dev->irq);
             
     set_dvfm_constraint();
     size = ts_upgrade_firmware(g_touch_dev, &m_firmware_data[2], g_touch_dev->cap_info.chip_fw_size);
     unset_dvfm_constraint();

    ts_hw_calibration(g_touch_dev);
    ts_init_touch(g_touch_dev);

     g_touch_dev->work_proceedure = TS_NO_WORK;	
     up(&g_touch_dev->work_proceedure_lock); 

     enable_irq(g_touch_dev->irq);

     zinitix_debug_msg("firmware update end!!\n" );

     return sprintf(buf, "%d", 1 );
}

static ssize_t firmware_ret_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
    zinitix_debug_msg("return = %d\n", size );
    return size;
}

/* - SKC on110725  porting BT402B TSP*/

#if BT4x2_Series
inline s32 ts_write_cmd(struct i2c_client *client, u8 reg)
{
	s32 ret;
	ret = i2c_smbus_write_byte(client, reg);
	udelay(DELAY_FOR_POST_TRANSCATION);
	return ret;
}

inline s32 ts_write_reg(struct i2c_client *client, u8 reg, u16 value)
{
	s32 ret;
	ret = i2c_smbus_write_word_data(client, reg, value);
	udelay(DELAY_FOR_POST_TRANSCATION);
	return ret;
}

inline s32 ts_read_data(struct i2c_client *client, u8 reg, u8 *values, u16 length)
{
	s32 ret;
	if((ret = i2c_master_send(client , &reg , 1)) < 0)	return ret;	// select register
	udelay(DELAY_FOR_TRANSCATION);		// for setup tx transaction.
	if((ret = i2c_master_recv(client , values , length)) < 0)	return ret;
	udelay(DELAY_FOR_POST_TRANSCATION);
	return length;
}

inline s32 ts_write_data(struct i2c_client *client, u8 reg, u8 *values, u16 length)
{
	s32 ret;
	ret = i2c_smbus_write_i2c_block_data(client, reg, length, values);
	udelay(DELAY_FOR_POST_TRANSCATION);
	return ret;
}

#endif


#if	TOUCH_USING_ISP_METHOD
inline s32 ts_read_firmware_data(struct i2c_client *client, char *addr, u8 *values, u16 length)
{
	s32 ret;
	if((ret = i2c_master_send(client , addr , 2)) < 0)	return ret;	// select register
	mdelay(1);		// for setup tx transaction.
	if((ret = i2c_master_recv(client , values , length)) < 0)	return ret;
	udelay(DELAY_FOR_POST_TRANSCATION);
	return length;
}

inline s32 ts_write_firmware_data(struct i2c_client *client, u8 *values, u16 length)
{
	s32 ret;
	if((ret = i2c_master_send(client , values , length)) < 0)	return ret;	
	udelay(DELAY_FOR_POST_TRANSCATION);
	return length;
}
#else
inline s32 ts_read_firmware_data(struct i2c_client *client, u8 reg, u8 *values, u16 length)
{
	s32 ret;
	if((ret = i2c_master_send(client , &reg , 1)) < 0)	return ret;	// select register
	mdelay(1);		// for setup tx transaction
	if((ret = i2c_master_recv(client , values , length)) < 0)	return ret;
	udelay(DELAY_FOR_POST_TRANSCATION);	
	return length;
}
#endif

//==================================================================


static int zinitix_touch_probe(struct i2c_client *client, const struct i2c_device_id *i2c_id);
static int zinitix_touch_remove(struct i2c_client *client);
bool ts_init_touch(zinitix_touch_dev* touch_dev);


#if (TOUCH_MODE == 1)
static void	zinitix_report_data(zinitix_touch_dev *touch_dev, int id);
#endif

#ifdef CONFIG_HAS_EARLYSUSPEND
static void zinitix_early_suspend(struct early_suspend *h);
static void zinitix_late_resume(struct early_suspend *h);
#endif

#if	ZINITIX_ESD_TIMER_INTERVAL
static void ts_esd_timer_start(u16 sec, zinitix_touch_dev* touch_dev);
static void ts_esd_timer_stop(zinitix_touch_dev* touch_dev);
static void ts_esd_timer_init(zinitix_touch_dev* touch_dev);
static void ts_esd_timeout_handler(unsigned long data);
#endif


// id -> include/linux/i2c-id.h 
static struct i2c_driver zinitix_touch_driver = {
	.probe     = zinitix_touch_probe,
	.remove    = zinitix_touch_remove,
	.id_table  = zinitix_idtable,
	.driver    = {
		.name  = ZINITIX_DRIVER_NAME,
	},
};


static bool ts_get_samples (zinitix_touch_dev* touch_dev)
{
	int i;	

	zinitix_debug_msg("ts_get_samples+\r\n");

	if (gpio_get_value(touch_dev->int_gpio_num))
	{
        //interrupt pin is high, not valid data.
		zinitix_debug_msg("woops... inturrpt pin is high\r\n");
       	return false;
    }

#if	USE_TEST_RAW_TH_DATA_MODE
	down(&touch_dev->raw_data_lock);
	if(touch_dev->raw_mode_flag == TOUCH_TEST_RAW_MODE)
	{
		if(ts_read_firmware_data(touch_dev->client,ZINITIX_RAWDATA_REG,(u8 *)touch_dev->cur_data, touch_dev->cap_info.total_node_num*2 + MAX_TEST_POINT_INFO*2)<0)
		{
				printk(KERN_INFO "error : read zinitix tc raw data\n");
				up(&touch_dev->raw_data_lock); 
				return false;
		}		
		if(!zinitix_bit_test(touch_dev->cur_data[MAX_TEST_RAW_DATA], BIT_ICON_EVENT) && 
			!zinitix_bit_test(touch_dev->cur_data[MAX_TEST_RAW_DATA], BIT_PT_EXIST))		// no point, so update ref_data
		{
			memcpy((u8*)touch_dev->ref_data, (u8*)touch_dev->cur_data, touch_dev->cap_info.total_node_num*2);
		}
		up(&touch_dev->raw_data_lock); 
		return true;		
	}
	up(&touch_dev->raw_data_lock); 
#endif

#if (TOUCH_MODE == 1)
	
	memset(&touch_dev->touch_info, 0x0, sizeof(_ts_zinitix_point_info));	

	if (ts_read_data (touch_dev->client, ZINITIX_POINT_STATUS_REG, (u8*)(&touch_dev->touch_info), 4)< 0)
	{
		zinitix_debug_msg("error read point info using i2c.-\r\n");
       	return false;
	}
	zinitix_debug_msg("status reg = 0x%x , event_flag = 0x%04x\r\n", touch_dev->touch_info.status, touch_dev->touch_info.event_flag);

	if(touch_dev->touch_info.status == 0x0)
	{
		zinitix_debug_msg("periodical esd repeated int occured\r\n");
		return true;
	}

	if(zinitix_bit_test(touch_dev->touch_info.status, BIT_ICON_EVENT))
	{
		udelay(20);
		if (ts_read_data (touch_dev->client, ZINITIX_ICON_STATUS_REG, (u8*)(&touch_dev->icon_event_reg), 2) < 0)
		{
			printk(KERN_INFO "error read icon info using i2c.\n");
       		return false;
		}
		return true;
	}

	if(!zinitix_bit_test(touch_dev->touch_info.status, BIT_PT_EXIST))
	{
		//ts_write_cmd(touch_dev->client, ZINITIX_CLEAR_INT_STATUS_CMD);
		for(i=0; i < SUPPORTED_FINGER_NUM; i++)
		{		
			if(zinitix_bit_test(touch_dev->reported_touch_info.coord[i].sub_status, SUB_BIT_EXIST))
			{				
				input_report_abs(touch_dev->input_dev, ABS_MT_TOUCH_MAJOR, 0);
				input_report_abs(touch_dev->input_dev, ABS_MT_WIDTH_MAJOR, 0);	
				input_report_abs(touch_dev->input_dev, ABS_MT_POSITION_X, touch_dev->reported_touch_info.coord[i].x);
				input_report_abs(touch_dev->input_dev, ABS_MT_POSITION_Y, touch_dev->reported_touch_info.coord[i].y);
				input_mt_sync(touch_dev->input_dev);	
				touch_dev->reported_touch_info.coord[i].sub_status = 0;
			}
		}	
		input_sync(touch_dev->input_dev);
		return true;
	}	

	
	for(i=0; i < SUPPORTED_FINGER_NUM; i++)
	{
		if(zinitix_bit_test(touch_dev->touch_info.event_flag, i))
		{			
			udelay(20);
			if (ts_read_data (touch_dev->client, ZINITIX_POINT_STATUS_REG+2+i, (u8*)(&touch_dev->touch_info.coord[i]), sizeof(_ts_zinitix_coord))< 0)
			{
				zinitix_debug_msg("error read point info using i2c.-\r\n");
		        	return false;
			}
			zinitix_bit_clr(touch_dev->touch_info.event_flag, i);
			if(touch_dev->touch_info.event_flag == 0)	
			{
				//ts_write_cmd(touch_dev->client, ZINITIX_CLEAR_INT_STATUS_CMD);
				zinitix_report_data(touch_dev, i);
				return true;
			}
			else
				zinitix_report_data(touch_dev, i);
		}		
	}	


#else
	if (ts_read_data (touch_dev->client, ZINITIX_POINT_STATUS_REG, (u8*)(&touch_dev->touch_info), sizeof(_ts_zinitix_point_info))< 0)
	{
		zinitix_debug_msg("error read point info using i2c.-\r\n");
        return false;
	}
	zinitix_debug_msg("status reg = 0x%x , point cnt = %d, time stamp = %d\r\n", touch_dev->touch_info.status, 
		touch_dev->touch_info.finger_cnt, touch_dev->touch_info.time_stamp);

	if(touch_dev->touch_info.status == 0x0 && touch_dev->touch_info.finger_cnt == 100)
	{
		zinitix_debug_msg("periodical esd repeated int occured\r\n");
		return true;
	}


	if(zinitix_bit_test(touch_dev->touch_info.status, BIT_ICON_EVENT))
	{
		udelay(20);
		if (ts_read_data (touch_dev->client, ZINITIX_ICON_STATUS_REG, (u8*)(&touch_dev->icon_event_reg), 2) < 0)
		{
			printk(KERN_INFO "error read icon info using i2c.\n");
        	return false;
		}
	}
#endif		
	zinitix_debug_msg("ts_get_samples-\r\n");
	
	return true;
}


static irqreturn_t ts_int_handler(int irq, void *dev)
{

#if USE_THREADED_IRQ	
      return IRQ_WAKE_THREAD;
#else
	zinitix_touch_dev* touch_dev = (zinitix_touch_dev*)dev;

	zinitix_debug_msg("interrupt occured +\r\n");
      
	if (gpio_get_value(touch_dev->int_gpio_num))	// remove pending interrupt
	{
		zinitix_debug_msg("invalid interrupt occured +\r\n");
       	return IRQ_HANDLED;
	}

	if (down_trylock(&touch_dev->work_proceedure_lock)) 
	{
		disable_irq_nosync(irq);
		zinitix_debug_msg(KERN_WARNING "interrupt occured when other process occupied sema\r\n");
		return IRQ_HANDLED;
	}
		
	if(touch_dev->work_proceedure != TS_NO_WORK)
	{
		zinitix_debug_msg(KERN_WARNING "interrupt occured in suspend or esd timer.(%d)\r\n", touch_dev->work_proceedure);
		up(&touch_dev->work_proceedure_lock);
		return IRQ_HANDLED;
	}
	
	touch_dev->work_proceedure = TS_NORMAL_WORK;
      disable_irq_nosync(irq);
	queue_work(zinitix_workqueue, &touch_dev->work);
      
	return IRQ_HANDLED;
#endif
}

static bool ts_read_coord (zinitix_touch_dev * hDevice)
{
	zinitix_touch_dev* touch_dev = (zinitix_touch_dev*)hDevice;
	//zinitix_debug_msg("ts_read_coord+\r\n");

	if(ts_get_samples(touch_dev)==false)
	{
		return false;
	}
	
	ts_write_cmd(touch_dev->client, ZINITIX_CLEAR_INT_STATUS_CMD);
#if DELAY_FOR_SIGNAL_DELAY	
	udelay(DELAY_FOR_SIGNAL_DELAY);
#endif
	return true;
}


static void ts_power_control(zinitix_touch_dev *touch_dev, u8 ctl)
{
	if(ctl == POWER_OFF){
          touch_power_control(0);
	}
	else if(ctl == POWER_ON){
          touch_power_control(1);
	}
	else if(ctl == RESET_LOW){
          /* Do nothing */
	}
	else if(ctl == RESET_HIGH){
          /* Do nothing */
	}
}

static bool ts_mini_init_touch(zinitix_touch_dev * touch_dev)
{
	if(touch_dev == NULL)	
	{
		printk(KERN_INFO "ts_mini_init_touch : error (touch_dev == NULL?)\r\n");
		return false;		
	}	

	zinitix_debug_msg("clear all reported points\r\n");

	ts_init_touch(touch_dev);

#if	ZINITIX_ESD_TIMER_INTERVAL	
	if(touch_dev->use_esd_timer)
	{
		ts_esd_timer_start(ZINITIX_ESD_TIMER_INTERVAL*2, touch_dev);
		zinitix_debug_msg("esd timer start\r\n");
	}
#endif		
	
	return true;
}


#if	ZINITIX_ESD_TIMER_INTERVAL
static void zinitix_touch_tmr_work(struct work_struct *work)
{
	zinitix_touch_dev *touch_dev = container_of(work, zinitix_touch_dev, tmr_work);	
	
	printk(KERN_INFO "tmr queue work ++\r\n");
	if(touch_dev == NULL)
	{
		printk(KERN_INFO "touch dev == NULL ?\r\n");
		goto fail_time_out_init;
	}	
	disable_irq(touch_dev->irq);
	printk(KERN_INFO "error. timeout occured. maybe ts device dead. so reset & reinit.\r\n");
	mdelay(CHIP_POWER_OFF_DELAY);
	ts_power_control(touch_dev, RESET_LOW); //reset pin low
	ts_power_control(touch_dev, POWER_OFF); //power off
	mdelay(CHIP_POWER_OFF_DELAY);
	ts_power_control(touch_dev, POWER_ON); //power on
	ts_power_control(touch_dev, RESET_HIGH);	//reset pin high
	mdelay(CHIP_ON_DELAY);
	
	if(ts_mini_init_touch(touch_dev) == false)
		goto fail_time_out_init;

	touch_dev->work_proceedure = TS_NO_WORK;
	enable_irq(touch_dev->irq);
	up(&touch_dev->work_proceedure_lock);
	printk(KERN_INFO "tmr queue work ----\r\n");
	return;
fail_time_out_init:
	printk(KERN_INFO "tmr work : restart error\r\n");	
#if	ZINITIX_ESD_TIMER_INTERVAL	
	ts_esd_timer_start(ZINITIX_ESD_TIMER_INTERVAL*2, touch_dev);
#endif		
	touch_dev->work_proceedure = TS_NO_WORK;
	enable_irq(touch_dev->irq);
	up(&touch_dev->work_proceedure_lock);
}

static void ts_esd_timer_start(u16 sec, zinitix_touch_dev* touch_dev)
{	
	if(touch_dev->p_esd_timeout_tmr != NULL)	del_timer(touch_dev->p_esd_timeout_tmr);
	touch_dev->p_esd_timeout_tmr = NULL;

	init_timer(&(touch_dev->esd_timeout_tmr));
	touch_dev->esd_timeout_tmr.data = (unsigned long)(touch_dev);
   	touch_dev->esd_timeout_tmr.function = ts_esd_timeout_handler;			
	touch_dev->esd_timeout_tmr.expires = jiffies + HZ*sec;	
	touch_dev->p_esd_timeout_tmr = &touch_dev->esd_timeout_tmr;
	add_timer(&touch_dev->esd_timeout_tmr);
}

static void ts_esd_timer_stop(zinitix_touch_dev* touch_dev)
{
	if(touch_dev->p_esd_timeout_tmr) del_timer(touch_dev->p_esd_timeout_tmr);
	touch_dev->p_esd_timeout_tmr = NULL;
}


static void ts_esd_timer_init(zinitix_touch_dev* touch_dev)
{
	init_timer(&(touch_dev->esd_timeout_tmr));
	touch_dev->esd_timeout_tmr.data = (unsigned long)(touch_dev);
   	touch_dev->esd_timeout_tmr.function = ts_esd_timeout_handler;		
	touch_dev->p_esd_timeout_tmr=NULL;
}

static void ts_esd_timeout_handler(unsigned long data)
{
	zinitix_touch_dev* touch_dev = (zinitix_touch_dev*)data;
	touch_dev->p_esd_timeout_tmr=NULL;	
	down(&touch_dev->work_proceedure_lock);	
	if(touch_dev->work_proceedure != TS_NO_WORK)
	{
		up(&touch_dev->work_proceedure_lock);
		return;	
	}
	touch_dev->work_proceedure = TS_ESD_TIMER_WORK;	
	queue_work(zinitix_tmr_workqueue, &touch_dev->tmr_work);
}
#endif


bool ts_check_need_upgrade(u16 curVersion, u16 curRegVersion)
{
	u16	newVersion;
	newVersion = (u16) (m_firmware_data[0] | (m_firmware_data[1]<<8));

	printk(KERN_INFO "cur Version = 0x%x, new Version = 0x%x\n", curVersion, newVersion);

      //return false;  /* SKC to block initial firmware upgrade */
	
	if(curVersion < newVersion)		return true;
/*	else if(curVersion > newVersion)	return false;

	if(m_firmware_data[0x3FFE] == 0xff && m_firmware_data[0x3FFF] == 0xff)	return false;
	newVersion = (u16) (m_firmware_data[0x3FFE] | (m_firmware_data[0x3FFF]<<8));	
			
	if(curRegVersion < newVersion)	return true;
*/
	return false;
}


#define	TC_PAGE_SZ		64
#define	TC_SECTOR_SZ		8

u8 ts_upgrade_firmware(zinitix_touch_dev* touch_dev, const u8 *firmware_data, u32 size)
{
	u16 flash_addr;
	u8  * verify_data;
#if	(TOUCH_USING_ISP_METHOD==0 && BT4x2_Series == 1)	
	int	retry_cnt = 0;
	u32 i;
#endif	
	u8	i2c_buffer[TC_PAGE_SZ+2];
	
	verify_data = (u8*)kzalloc(size, GFP_KERNEL);
	if(verify_data == NULL)
	{
		printk(KERN_ERR "cannot alloc verify buffer\n");
		return false;
	}
	
#if	(TOUCH_USING_ISP_METHOD==0 && BT4x2_Series == 1)
	do{
		printk(KERN_INFO "reset command\n");
		if (ts_write_cmd(touch_dev->client, ZINITIX_SWRESET_CMD)!=I2C_SUCCESS)
		{
			printk(KERN_INFO "failed to reset\n");
			goto fail_upgrade;
		}
		
#if USE_HW_CALIBRATION
		printk(KERN_INFO "Erase Flash\n");
		if (ts_write_reg(touch_dev->client, ZINITIX_ERASE_FLASH, 0xaaaa)!=I2C_SUCCESS)
		{
			printk(KERN_INFO "failed to erase flash\n");
			goto fail_upgrade;
		}		

		mdelay(500);
#else
		if (ts_write_reg(touch_dev->client, ZINITIX_TOUCH_MODE, 0x06)!=I2C_SUCCESS)
		{
			printk(KERN_INFO "failed to erase flash\n");
			goto fail_upgrade;
		}		
		ts_write_cmd(touch_dev->client, ZINITIX_CLEAR_INT_STATUS_CMD);
		mdelay(10);
		ts_write_cmd(touch_dev->client, ZINITIX_CLEAR_INT_STATUS_CMD);
		mdelay(10);
#endif	

		printk(KERN_INFO "writing firmware data\n");			
			
		for(flash_addr= 0; flash_addr< size; )
		{
				
			for(i=0; i< TC_PAGE_SZ/TC_SECTOR_SZ; i++)
			{
				printk(KERN_INFO "addr = %04x, len = %d\n", flash_addr, TC_SECTOR_SZ);
				if(ts_write_data(touch_dev->client,ZINITIX_WRITE_FLASH, &firmware_data[flash_addr],TC_SECTOR_SZ)<0)
				{
					printk(KERN_INFO"error : write zinitix tc firmare\n");
					goto fail_upgrade;
				}		
				flash_addr+= TC_SECTOR_SZ;	
				udelay(100);
			}
			mdelay(20);	
#if !USE_HW_CALIBRATION
			if(flash_addr >= CALIBRATION_AREA)	break;
#endif			
		}

		printk(KERN_INFO "read firmware data\n");					
		for(flash_addr= 0; flash_addr< size; )
		{
				
			for(i=0; i< TC_PAGE_SZ/TC_SECTOR_SZ; i++)
			{
				printk(KERN_INFO "addr = %04x, len = %d\n", flash_addr, TC_SECTOR_SZ);
				if(ts_read_firmware_data(touch_dev->client,ZINITIX_READ_FLASH,&verify_data[flash_addr],TC_SECTOR_SZ)<0)
				{
					printk(KERN_INFO "error : read zinitix tc firmare\n");
					goto fail_upgrade;
				}		
				flash_addr+= TC_SECTOR_SZ;			
			}
#if !USE_HW_CALIBRATION
			if(flash_addr >= CALIBRATION_AREA)
			{
				memcpy((u8*)&verify_data[CALIBRATION_AREA], (u8*)&firmware_data[CALIBRATION_AREA], size-CALIBRATION_AREA);
				break;				
			}
#endif						
		}
		// verify
		printk(KERN_INFO "verify firmware data\n");
		if(memcmp((u8*)&firmware_data[0], (u8*)&verify_data[0], size) == 0)
		{
			printk(KERN_INFO "upgrade finished\n");
			kfree(verify_data);		
			ts_power_control(touch_dev, RESET_LOW); //reset pin low
			ts_power_control(touch_dev, POWER_OFF); //power off
			mdelay(CHIP_POWER_OFF_DELAY);
			ts_power_control(touch_dev, POWER_ON); //power on
			ts_power_control(touch_dev, RESET_HIGH);	//reset pin high	
			mdelay(CHIP_ON_DELAY);
			return true;
		}
		printk(KERN_INFO "upgrade fail : so retry... (%d)\n", ++retry_cnt);
		
	}while(1);

	
#elif	(TOUCH_USING_ISP_METHOD==1)	// isp


	if(m_isp_client == NULL)
	{
		printk(KERN_ERR "i2c client for isp is not register \r\n");
		return false;
	}
	
#if BT4x2_Series
	//must be reset pin low
	//+++++++++++++++++++++++++++++++++++++++++++++++++++++
	ts_power_control(touch_dev, RESET_LOW);	//reset pin low
	//+++++++++++++++++++++++++++++++++++++++++++++++++++++	
	mdelay(100);	
#endif	

	for(flash_addr= 0; flash_addr< size; flash_addr+=TC_PAGE_SZ)
	{

		printk(KERN_INFO "firmware write : addr = %04x, len = %d\n", flash_addr, TC_PAGE_SZ);
		i2c_buffer[0] = (flash_addr>>8)&0xff;	//addr_h
		i2c_buffer[1] = (flash_addr)&0xff;	//addr_l
		memcpy(&i2c_buffer[2], &firmware_data[flash_addr], TC_PAGE_SZ);

		if(ts_write_firmware_data(m_isp_client, i2c_buffer, TC_PAGE_SZ+2)<0)	
		{
			printk(KERN_INFO"error : write zinitix tc firmare\n");
			goto fail_upgrade;
		}	
		else
			break;	
		mdelay(20);	
#if !USE_HW_CALIBRATION
		if(flash_addr >= CALIBRATION_AREA)			break;
#endif								
	}

	printk(KERN_INFO "read firmware data\n");					
	for(flash_addr= 0; flash_addr< size; flash_addr+=TC_PAGE_SZ)
	{
		i2c_buffer[0] = (flash_addr>>8)&0xff;	//addr_h
		i2c_buffer[1] = (flash_addr)&0xff;	//addr_l
		if(ts_read_firmware_data(m_isp_client, i2c_buffer,&verify_data[flash_addr],TC_PAGE_SZ)<0)
		{
			printk(KERN_INFO "error : read zinitix tc firmare: addr = %04x, len = %d\n", flash_addr, TC_PAGE_SZ);
			goto fail_upgrade;
		}				
		if(memcmp((u8*)&firmware_data[flash_addr], (u8*)&verify_data[flash_addr], TC_PAGE_SZ) != 0)			
		{
			printk(KERN_INFO "error : verify error : addr = %04x, len = %d\n", flash_addr, TC_PAGE_SZ);
			goto fail_upgrade;
		}
#if !USE_HW_CALIBRATION
		if(flash_addr >= CALIBRATION_AREA)			break;
#endif									
	}
	// verify
	mdelay(CHIP_POWER_OFF_DELAY);
	ts_power_control(touch_dev, RESET_LOW); //reset pin low
	ts_power_control(touch_dev, POWER_OFF); //power off
	mdelay(CHIP_POWER_OFF_DELAY);
	ts_power_control(touch_dev, POWER_ON); //power on
	ts_power_control(touch_dev, RESET_HIGH);	//reset pin high
	mdelay(CHIP_ON_DELAY);
	printk(KERN_INFO "upgrade finished\n");
	kfree(verify_data);
	return true;

#endif

fail_upgrade:	
	if(verify_data != NULL)		kfree(verify_data);
	mdelay(CHIP_POWER_OFF_DELAY);
	ts_power_control(touch_dev, RESET_LOW); //reset pin low
	ts_power_control(touch_dev, POWER_OFF); //power off
	mdelay(CHIP_POWER_OFF_DELAY);
	ts_power_control(touch_dev, POWER_ON); //power on
	ts_power_control(touch_dev, RESET_HIGH);	//reset pin high
	mdelay(CHIP_ON_DELAY);
	printk(KERN_INFO "upgrade fail..\n");
	return false;

}

bool    ts_hw_calibration(zinitix_touch_dev* touch_dev)
{
    u16 chip_eeprom_info;
    u16 hw_cal_cnt;
    u16 i;
    
	if (ts_read_data(touch_dev->client, ZINITIX_EEPROM_INFO_REG, (u8*)&chip_eeprom_info, 2)<0) goto fail_hw_calibration;
	printk(KERN_INFO"touch eeprom info = 0x%04X\r\n", chip_eeprom_info);
	
#if	USE_HW_CALIBRATION	
	if(zinitix_bit_test(chip_eeprom_info, 0)){

		udelay(100); 		
         
		for(i=0;i<10;i++){
			if (ts_write_reg(touch_dev->client, ZINITIX_TOUCH_MODE, 0x07)==I2C_SUCCESS) break;
		}
        
		if(i==10)
			goto fail_hw_calibration;

             udelay(200);
             
		for(i=0;i<10;i++){
            
			chip_eeprom_info = 1;
			ts_write_cmd(touch_dev->client, ZINITIX_CALIBRATE_CMD);
	
			hw_cal_cnt = 0;
            
		do{
			msleep(500); 
                        
			if (ts_read_data(touch_dev->client, ZINITIX_EEPROM_INFO_REG, (u8*)&chip_eeprom_info, 2)<0) goto fail_hw_calibration;
                        
                        printk(KERN_INFO "[TSP] touch eeprom info = 0x%04X\r\n", chip_eeprom_info);
                        
			if(!zinitix_bit_test(chip_eeprom_info, 0))	break;

                        hw_cal_cnt++;

                        if(hw_cal_cnt>20){
					printk(KERN_INFO "[TSP] could not calibrated retry \r\n");
					mdelay(100);
					break;
                        }
		}while(1);
			
			if(!zinitix_bit_test(chip_eeprom_info, 0))	break;
		}
        
		if(i==10)
			goto fail_hw_calibration;
		
		udelay(100);
		if (ts_write_reg(touch_dev->client, ZINITIX_TOUCH_MODE, 0x00)!=I2C_SUCCESS) goto fail_hw_calibration;
		if (ts_write_cmd(touch_dev->client, ZINITIX_SWRESET_CMD)!=I2C_SUCCESS) goto fail_hw_calibration;
	}
#endif
	return true;

fail_hw_calibration:
	ts_power_control(touch_dev, RESET_LOW); //reset pin low
	ts_power_control(touch_dev, POWER_OFF); //power off
	mdelay(CHIP_POWER_OFF_DELAY);
	ts_power_control(touch_dev, POWER_ON); //power on
	ts_power_control(touch_dev, RESET_HIGH);	//reset pin high
	mdelay(CHIP_ON_DELAY);
	printk(KERN_INFO "ts hw calibraton fail..\n");
        return false;
}

bool ts_init_touch(zinitix_touch_dev* touch_dev)
{
	u16		reg_val;
	int		i;
	u16 	SetMaxX = SYSTEM_MAX_X_RESOLUTION; //Max Position range from 0x0002 to 0x1fff
	u16 	SetMaxY = SYSTEM_MAX_Y_RESOLUTION; //Max Position range from 0x0002 to 0x1fff  
	u16		SupportedFingerNum = SUPPORTED_FINGER_NUM;

	u16 	CurMaxX = 1024;
	u16 	CurMaxY = 1920;      
	u16 	chip_revision = 0;
	u16 	chip_firmware_version = 0;
	u16		chip_reg_data_version = 0;
	u16		chip_eeprom_info = 0;
	s16		stmp = 0;

        u16 hw_cal_cnt;

	if(touch_dev == NULL)
	{
		printk(KERN_ERR "error touch_dev == null?\r\n");
		zinitix_debug_msg("failed initiallize\r\n");
		return false;
	}

	zinitix_debug_msg("disable interrupt\r\n");
	for(i=0; i<ZINITIX_INIT_RETRY_CNT; i++)
	{
		if (ts_write_reg(touch_dev->client, ZINITIX_INT_ENABLE_FLAG, 0x0)==I2C_SUCCESS)	break;
		mdelay(10);
	}

	if(i==ZINITIX_INIT_RETRY_CNT)
	{
		printk(KERN_INFO "fail to write interrupt register\r\n");
		goto fail_init;
	}

	zinitix_debug_msg("send reset command\r\n");
	if (ts_write_cmd(touch_dev->client, ZINITIX_SWRESET_CMD)!=I2C_SUCCESS)	goto fail_init;

	/* get chip revision id */
	if (ts_read_data(touch_dev->client, ZINITIX_CHIP_REVISION, (u8*)&chip_revision, 2)<0)
	{
		printk(KERN_INFO "fail to read chip revision\r\n");
		goto fail_init;
	}	
	zinitix_debug_msg("zinitix touch chip revision id = %x\r\n", chip_revision);

	touch_dev->cap_info.chip_fw_size = 16*1024;

#if	USE_TEST_RAW_TH_DATA_MODE
	if (ts_read_data(touch_dev->client, ZINITIX_TOTAL_NUMBER_OF_X, (u8*)&touch_dev->cap_info.x_node_num, 2)<0) goto fail_init;
	printk(KERN_INFO "zinitix touch chip x node num = %d\r\n", touch_dev->cap_info.x_node_num);
	if (ts_read_data(touch_dev->client, ZINITIX_TOTAL_NUMBER_OF_Y, (u8*)&touch_dev->cap_info.y_node_num, 2)<0) goto fail_init;
	printk(KERN_INFO "zinitix touch chip y node num = %d\r\n", touch_dev->cap_info.y_node_num);

	touch_dev->cap_info.total_node_num = touch_dev->cap_info.x_node_num*touch_dev->cap_info.y_node_num;
	printk(KERN_INFO "zinitix touch chip total node num = %d\r\n", touch_dev->cap_info.total_node_num);	
#endif

	touch_dev->cap_info.multi_fingers = SUPPORTED_FINGER_NUM;	

	/* get chip firmware version */
	if (ts_read_data(touch_dev->client, ZINITIX_FIRMWARE_VERSION, (u8*)&chip_firmware_version, 2)<0) goto fail_init;
	zinitix_debug_msg("zinitix touch chip firmware version = %x\r\n", chip_firmware_version);
	
#if	TOUCH_ONESHOT_UPGRADE
	chip_reg_data_version = 0xffff;

	if (ts_read_data(touch_dev->client, ZINITIX_DATA_VERSION_REG, (u8*)&chip_reg_data_version, 2)<0) goto fail_init;
	zinitix_debug_msg("touch reg data version = %d\r\n", chip_reg_data_version);
	
 	if(ts_check_need_upgrade(chip_firmware_version, chip_reg_data_version)==true)
   	{
   		zinitix_debug_msg("start upgrade firmware\n");

		ts_upgrade_firmware(touch_dev, &m_firmware_data[2], touch_dev->cap_info.chip_fw_size);
		
	
		/* get chip revision id */
		if (ts_read_data(touch_dev->client, ZINITIX_CHIP_REVISION, (u8*)&chip_revision, 2)<0)
		{
			printk(KERN_INFO "fail to read chip revision\r\n");
			goto fail_init;
		}	
        
		zinitix_debug_msg("zinitix touch chip revision id = %x\r\n", chip_revision);
		
		/* get chip firmware version */
		if (ts_read_data(touch_dev->client, ZINITIX_FIRMWARE_VERSION, (u8*)&chip_firmware_version, 2)<0) goto fail_init;		
		printk(KERN_INFO "zinitix touch chip renewed firmware version = %x\r\n", chip_firmware_version);

	  }


	if (ts_read_data(touch_dev->client, ZINITIX_DATA_VERSION_REG, (u8*)&chip_reg_data_version, 2)<0) goto fail_init;
	zinitix_debug_msg("touch reg data version = %d\r\n", chip_reg_data_version);

#if 0
	if(chip_reg_data_version < m_reg_data[ZINITIX_DATA_VERSION_REG].reg_val)
	{
		zinitix_debug_msg("write new reg data( %d < %d)\r\n", chip_reg_data_version, m_reg_data[ZINITIX_DATA_VERSION_REG].reg_val);
		for(i=0; i < MAX_REG_COUNT; i++)
		{
			if(m_reg_data[i].valid == 1)
			{
				if(ts_write_reg(touch_dev->client, (u16)i, (u16)(m_reg_data[i].reg_val))!=I2C_SUCCESS) goto fail_init;
				if(i == ZINITIX_TOTAL_NUMBER_OF_X || i == ZINITIX_TOTAL_NUMBER_OF_Y)	mdelay(50);	//for clear hw calibration bit
				if(ts_read_data(touch_dev->client, (u16)i, (u8*)&stmp, 2)<0) goto fail_init;
				if(memcmp((char*)&m_reg_data[i].reg_val, (char*)&stmp, 2)!=0)		//if(m_reg_data[i].reg_val != stmp)
					printk(KERN_WARNING "register data is different. (addr = 0x%02X , %d != %d)\r\n", i, m_reg_data[i].reg_val, stmp);					
			}
		}
		zinitix_debug_msg("done new reg data( %d < %d)\r\n", chip_reg_data_version, m_reg_data[ZINITIX_DATA_VERSION_REG].reg_val);		
		if (ts_write_cmd(touch_dev->client, ZINITIX_SAVE_STATUS_CMD)!=I2C_SUCCESS) goto fail_init;			
		mdelay(1000);	// for fusing eeprom
	}
#endif

	if (ts_read_data(touch_dev->client, ZINITIX_EEPROM_INFO_REG, (u8*)&chip_eeprom_info, 2)<0) goto fail_init;
	printk(KERN_INFO"touch eeprom info = 0x%04X\r\n", chip_eeprom_info);
	
#if	USE_HW_CALIBRATION	
	if(zinitix_bit_test(chip_eeprom_info, 0)){

		udelay(100); 		
         
		for(i=0;i<10;i++){
			if (ts_write_reg(touch_dev->client, ZINITIX_TOUCH_MODE, 0x07)==I2C_SUCCESS) break;
		}
        
		if(i==10)
			goto fail_init;

             udelay(200);
             
		for(i=0;i<10;i++){
            
			chip_eeprom_info = 1;
			ts_write_cmd(touch_dev->client, ZINITIX_CALIBRATE_CMD);
	
			hw_cal_cnt = 0;
            
		do{
			msleep(500); 
                        
			if (ts_read_data(touch_dev->client, ZINITIX_EEPROM_INFO_REG, (u8*)&chip_eeprom_info, 2)<0) goto fail_init;
                        
                        printk(KERN_INFO "[TSP] touch eeprom info = 0x%04X\r\n", chip_eeprom_info);
                        
			if(!zinitix_bit_test(chip_eeprom_info, 0))	break;

                        hw_cal_cnt++;

                        if(hw_cal_cnt>20){
					printk(KERN_INFO "[TSP] could not calibrated retry \r\n");
					mdelay(100);
					break;
                        }
		}while(1);
			
			if(!zinitix_bit_test(chip_eeprom_info, 0))	break;
		}
        
		if(i==10)
			goto fail_init;
		
		udelay(100);
		if (ts_write_reg(touch_dev->client, ZINITIX_TOUCH_MODE, 0x00)!=I2C_SUCCESS) goto fail_init;
		if (ts_write_cmd(touch_dev->client, ZINITIX_SWRESET_CMD)!=I2C_SUCCESS) goto fail_init;
	}
#endif

#endif

	touch_dev->cap_info.chip_revision = (u16)chip_revision;
	touch_dev->cap_info.chip_firmware_version = (u16)chip_firmware_version;
	touch_dev->cap_info.chip_reg_data_version = (u16)chip_reg_data_version;
 
	/* initialize */	
	if (ts_write_reg(touch_dev->client, ZINITIX_X_RESOLUTION, (u16)(SetMaxX))!=I2C_SUCCESS) goto fail_init;
	if (ts_write_reg(touch_dev->client, ZINITIX_Y_RESOLUTION, (u16)(SetMaxY))!=I2C_SUCCESS) goto fail_init;
    
	if (ts_read_data(touch_dev->client, ZINITIX_X_RESOLUTION, (u8*)&CurMaxX, 2)<0) goto fail_init;
	zinitix_debug_msg("touch max x = %d\r\n", CurMaxX);
	if (ts_read_data(touch_dev->client, ZINITIX_Y_RESOLUTION, (u8*)&CurMaxY, 2)<0) goto fail_init;
	zinitix_debug_msg("touch max y = %d\r\n", CurMaxY);    

	touch_dev->cap_info.MinX = (u32)0;
	touch_dev->cap_info.MinY = (u32)0;
	touch_dev->cap_info.MaxX = (u32)CurMaxX;
	touch_dev->cap_info.MaxY = (u32)CurMaxY;

	if (ts_write_reg(touch_dev->client, ZINITIX_SUPPORTED_FINGER_NUM, (u16)SupportedFingerNum)!=I2C_SUCCESS) goto fail_init;
	if (ts_read_data(touch_dev->client, ZINITIX_SUPPORTED_FINGER_NUM, (u8*)&SupportedFingerNum, 2)<0) goto fail_init;
	zinitix_debug_msg("supported finger num = %d\r\n", SupportedFingerNum); 
	
		
	touch_dev->cap_info.gesture_support = 0;
	touch_dev->cap_info.multi_fingers = SupportedFingerNum;
    		
	zinitix_debug_msg("set other configuration\r\n");

#if	USE_TEST_RAW_TH_DATA_MODE
	if(touch_dev->raw_mode_flag == TOUCH_TEST_RAW_MODE)	// test mode
	{
		reg_val = TOUCH_TEST_RAW_MODE;
		if (ts_write_reg(touch_dev->client, ZINITIX_TOUCH_MODE, reg_val)!=I2C_SUCCESS)	goto fail_init; 	
	}
	else
#endif
{
	reg_val = TOUCH_MODE;
	if (ts_write_reg(touch_dev->client, ZINITIX_TOUCH_MODE, reg_val)!=I2C_SUCCESS)	goto fail_init;		
}
	// soft calibration
	if (ts_write_cmd(touch_dev->client, ZINITIX_CALIBRATE_CMD)!=I2C_SUCCESS)		goto fail_init;	

		
	reg_val = 0;
	zinitix_bit_set(reg_val, BIT_PT_CNT_CHANGE);			
	zinitix_bit_set(reg_val, BIT_DOWN);
	zinitix_bit_set(reg_val, BIT_MOVE);
	zinitix_bit_set(reg_val, BIT_UP);
		
	if(SUPPORTED_BUTTON_NUM > 0)		zinitix_bit_set(reg_val, BIT_ICON_EVENT);
	touch_dev->chip_int_mask = reg_val;

	if (ts_write_reg(touch_dev->client, ZINITIX_INT_ENABLE_FLAG, touch_dev->chip_int_mask)!=I2C_SUCCESS)	goto fail_init;		
	//---------------------------------------------------------------------
	// read garbage data
	for(i=0; i<10; i++)
	{
		ts_write_cmd(touch_dev->client, ZINITIX_CLEAR_INT_STATUS_CMD);
		udelay(10);
	}
	
#if	USE_TEST_RAW_TH_DATA_MODE
	if(touch_dev->raw_mode_flag == TOUCH_TEST_RAW_MODE)	// test mode
	{
		if (ts_write_reg(touch_dev->client, ZINITIX_PERIODICAL_INTERRUPT_INTERVAL, ZINITIX_SCAN_RATE_HZ*ZINITIX_RAW_DATA_ESD_TIMER_INTERVAL)!=I2C_SUCCESS)
			printk(KERN_INFO "[zinitix_touch] Fail to set ZINITIX_RAW_DATA_ESD_TIMER_INTERVAL.\r\n");
	}
	else
#endif
{
#if	ZINITIX_ESD_TIMER_INTERVAL	
	if (ts_write_reg(touch_dev->client, ZINITIX_PERIODICAL_INTERRUPT_INTERVAL, ZINITIX_SCAN_RATE_HZ*ZINITIX_ESD_TIMER_INTERVAL)!=I2C_SUCCESS)	goto fail_init;
	if (ts_read_data(touch_dev->client, ZINITIX_PERIODICAL_INTERRUPT_INTERVAL, (u8*)&reg_val, 2)<0) goto fail_init;
	zinitix_debug_msg("esd timer register = %d\r\n", reg_val); 
#endif
}



	zinitix_debug_msg("successfully initialized\r\n");
	return true;
	
fail_init:
	zinitix_debug_msg("failed initiallize\r\n");
	
	if (ts_write_cmd(touch_dev->client, ZINITIX_SWRESET_CMD)!=I2C_SUCCESS);
	return false;

}


#if (TOUCH_MODE == 1)
static void	zinitix_report_data(zinitix_touch_dev *touch_dev, int id)
{
	int i;
	u32 x, y;
	u32 tmp;

	if(id >= SUPPORTED_FINGER_NUM || id < 0)
	{
		return;
	}
	
	x = touch_dev->touch_info.coord[id].x;
	y = touch_dev->touch_info.coord[id].y;
					
	 /* transformation from touch to screen orientation */
	if (touch_dev->cap_info.Orientation & TOUCH_V_FLIP)
	{
		y = touch_dev->cap_info.MaxY + touch_dev->cap_info.MinY - y;			               
	}
	if (touch_dev->cap_info.Orientation & TOUCH_H_FLIP)
	{
		x = touch_dev->cap_info.MaxX + touch_dev->cap_info.MinX - x;			               
	}
	if (touch_dev->cap_info.Orientation & TOUCH_XY_SWAP)
	{					
		zinitix_swap_v(x, y, tmp);
	}
	zinitix_debug_msg("x = %d, y = %d, w = %d\r\n", x, y, touch_dev->touch_info.coord[id].width);

	touch_dev->reported_touch_info.coord[id].x = x;
	touch_dev->reported_touch_info.coord[id].y = y;				
	touch_dev->reported_touch_info.coord[id].width = touch_dev->touch_info.coord[id].width;
	touch_dev->reported_touch_info.coord[id].sub_status = touch_dev->touch_info.coord[id].sub_status;
	

	for(i=0; i< SUPPORTED_FINGER_NUM; i++)
	{
		if(zinitix_bit_test(touch_dev->reported_touch_info.coord[i].sub_status, SUB_BIT_EXIST)
			||zinitix_bit_test(touch_dev->reported_touch_info.coord[i].sub_status, SUB_BIT_DOWN)
			||zinitix_bit_test(touch_dev->reported_touch_info.coord[i].sub_status, SUB_BIT_MOVE))			
		{
	
			if(touch_dev->reported_touch_info.coord[i].width == 0)	touch_dev->reported_touch_info.coord[i].width = 5;
			input_report_abs(touch_dev->input_dev, ABS_MT_TOUCH_MAJOR, (u32)touch_dev->reported_touch_info.coord[i].width);
			input_report_abs(touch_dev->input_dev, ABS_MT_WIDTH_MAJOR, (u32)touch_dev->reported_touch_info.coord[i].width);					
			input_report_abs(touch_dev->input_dev, ABS_MT_POSITION_X, touch_dev->reported_touch_info.coord[i].x);
			input_report_abs(touch_dev->input_dev, ABS_MT_POSITION_Y, touch_dev->reported_touch_info.coord[i].y);
			input_mt_sync(touch_dev->input_dev);	
		}
		else if(zinitix_bit_test(touch_dev->reported_touch_info.coord[i].sub_status, SUB_BIT_UP))			
		{
			input_report_abs(touch_dev->input_dev, ABS_MT_TOUCH_MAJOR, 0);
			input_report_abs(touch_dev->input_dev, ABS_MT_WIDTH_MAJOR, 0);	
			input_report_abs(touch_dev->input_dev, ABS_MT_POSITION_X, touch_dev->reported_touch_info.coord[i].x);
			input_report_abs(touch_dev->input_dev, ABS_MT_POSITION_Y, touch_dev->reported_touch_info.coord[i].y);
			input_mt_sync(touch_dev->input_dev);	
			touch_dev->reported_touch_info.coord[i].sub_status = 0;
		}
		else
			touch_dev->reported_touch_info.coord[i].sub_status = 0;
	}

	input_sync(touch_dev->input_dev);
}
#endif	// TOUCH_MODE == 1


static void	zinitix_clear_report_data(zinitix_touch_dev *touch_dev)
{
	int i;
	u8	reported = 0;

	for(i=0; i < touch_dev->cap_info.multi_fingers; i++)
	{		
		if(zinitix_bit_test(touch_dev->reported_touch_info.coord[i].sub_status, SUB_BIT_EXIST))
		{				
			input_report_abs(touch_dev->input_dev, ABS_MT_TOUCH_MAJOR, 0);
			input_report_abs(touch_dev->input_dev, ABS_MT_WIDTH_MAJOR, 0);	
			input_report_abs(touch_dev->input_dev, ABS_MT_POSITION_X, touch_dev->reported_touch_info.coord[i].x);
			input_report_abs(touch_dev->input_dev, ABS_MT_POSITION_Y, touch_dev->reported_touch_info.coord[i].y);
			input_mt_sync(touch_dev->input_dev);	
			reported = 1;
		}
		touch_dev->reported_touch_info.coord[i].sub_status = 0;
	}	
	if(reported)	input_sync(touch_dev->input_dev);
}


extern void push_touch_trace(int coord_x, int coord_y, int result);
#if USE_THREADED_IRQ
static irqreturn_t zinitix_touch_work(int irq, void *dev_id)
#else
static void zinitix_touch_work(struct work_struct *work)
#endif
{
	bool read_coord_continued;
	int i;
	u8 reported = false;
#if (TOUCH_MODE == 0)
	u32 x, y;
	u32 tmp;
#endif

#if USE_THREADED_IRQ

      zinitix_touch_dev *touch_dev = dev_id;

	zinitix_debug_msg("interrupt occured +\r\n");
      
	if (gpio_get_value(touch_dev->int_gpio_num))	// remove pending interrupt
	{
		printk("invalid interrupt occured +\r\n");
       	return IRQ_HANDLED;
	}

	down(&touch_dev->work_proceedure_lock);
		
	if(touch_dev->work_proceedure != TS_NO_WORK)
	{
		printk(KERN_WARNING "interrupt occured in suspend or esd timer.(%d)\r\n", touch_dev->work_proceedure);
		up(&touch_dev->work_proceedure_lock);
		return IRQ_HANDLED;
	}
	
	touch_dev->work_proceedure = TS_NORMAL_WORK;

#else
	zinitix_touch_dev *touch_dev = container_of(work, zinitix_touch_dev, work);
#endif

		//touch_dev->work_proceedure = TS_NORMAL_WORK;

	zinitix_debug_msg("zinitix_touch_thread : semaphore signalled\r\n");
		
#if	ZINITIX_ESD_TIMER_INTERVAL	
		if(touch_dev->use_esd_timer){			
			ts_esd_timer_stop(touch_dev);	
			zinitix_debug_msg("esd timer stop\r\n");
		}
#endif				
		read_coord_continued = true;
		do{
			if (ts_read_coord(touch_dev)==false){
				zinitix_debug_msg("couldn't read touch_dev sample\r\n");
				goto continue_read_samples;
			}

#if (TOUCH_MODE == 1)
			// invalid : maybe periodical repeated int.
			if(touch_dev->touch_info.status == 0x0)			
				goto continue_read_samples;
#else
			// invalid : maybe periodical repeated int.
			if(touch_dev->touch_info.status == 0x0 && touch_dev->touch_info.finger_cnt == 100)			
				goto continue_read_samples;
#endif	//TOUCH_MODE == 1
			reported = false;

			if(zinitix_bit_test(touch_dev->touch_info.status, BIT_ICON_EVENT)){

				for(i=0; i<SUPPORTED_BUTTON_NUM; i++){
					if(zinitix_bit_test(touch_dev->icon_event_reg, (BIT_O_ICON0_DOWN+i))){
						touch_dev->button[i] = ICON_BUTTON_DOWN;
						input_report_key(touch_dev->input_dev, BUTTON_MAPPING_KEY[i], 1);					
						reported = true;						
						zinitix_debug_msg("button down = %d \r\n", i);
					}
				}

				for(i=0; i<SUPPORTED_BUTTON_NUM; i++){
					if(zinitix_bit_test(touch_dev->icon_event_reg, (BIT_O_ICON0_UP+i))){
						touch_dev->button[i] = ICON_BUTTON_UP;	
						input_report_key(touch_dev->input_dev, BUTTON_MAPPING_KEY[i], 0);					
						reported = true;		
						zinitix_debug_msg("button up = %d \r\n", i);
					}
				}
			}

			// if button press or up event occured...
			if(reported == true){
#if (TOUCH_MODE == 1)
				//input_sync(touch_dev->input_dev);
				for(i=0; i< SUPPORTED_FINGER_NUM; i++){

					if(zinitix_bit_test(touch_dev->reported_touch_info.coord[i].sub_status, SUB_BIT_EXIST)){
						input_report_abs(touch_dev->input_dev, ABS_MT_TOUCH_MAJOR, 0);
						input_report_abs(touch_dev->input_dev, ABS_MT_WIDTH_MAJOR, 0);	
						input_report_abs(touch_dev->input_dev, ABS_MT_POSITION_X, touch_dev->reported_touch_info.coord[i].x);
						input_report_abs(touch_dev->input_dev, ABS_MT_POSITION_Y, touch_dev->reported_touch_info.coord[i].y);												
						input_mt_sync(touch_dev->input_dev);	
					}
					touch_dev->reported_touch_info.coord[i].sub_status = 0;
				}
				input_sync(touch_dev->input_dev);
				//goto continue_read_samples;
			}
#else					
				for(i=0; i< SUPPORTED_FINGER_NUM; i++){

					if(zinitix_bit_test(touch_dev->reported_touch_info.coord[i].sub_status, SUB_BIT_EXIST)){

						//input_report_abs(touch_dev->input_dev,ABS_MT_TRACKING_ID,i);
						input_report_abs(touch_dev->input_dev, ABS_MT_TOUCH_MAJOR, 0);
						input_report_abs(touch_dev->input_dev, ABS_MT_WIDTH_MAJOR, 0);	
						input_report_abs(touch_dev->input_dev, ABS_MT_POSITION_X, touch_dev->reported_touch_info.coord[i].x);
						input_report_abs(touch_dev->input_dev, ABS_MT_POSITION_Y, touch_dev->reported_touch_info.coord[i].y);												
						input_mt_sync(touch_dev->input_dev);	
					}
				}
				memset(&touch_dev->reported_touch_info, 0x0, sizeof(_ts_zinitix_point_info));				
				input_sync(touch_dev->input_dev);
				udelay(100);				
				goto continue_read_samples;				
			}


			if (touch_dev->touch_info.finger_cnt > SUPPORTED_FINGER_NUM)
				touch_dev->touch_info.finger_cnt = SUPPORTED_FINGER_NUM;

			if(!zinitix_bit_test(touch_dev->touch_info.status, BIT_PT_EXIST)){
			
				for(i=0; i< SUPPORTED_FINGER_NUM; i++){
					if(zinitix_bit_test(touch_dev->reported_touch_info.coord[i].sub_status, SUB_BIT_EXIST)){
						printk("[TSP] finger exist up \r\n");
						push_touch_trace(touch_dev->reported_touch_info.coord[i].x, touch_dev->reported_touch_info.coord[i].y, 1);
						//input_report_abs(touch_dev->input_dev,ABS_MT_TRACKING_ID,i);
						input_report_abs(touch_dev->input_dev, ABS_MT_TOUCH_MAJOR, 0);
						input_report_abs(touch_dev->input_dev, ABS_MT_WIDTH_MAJOR, 0);	
						input_report_abs(touch_dev->input_dev, ABS_MT_POSITION_X, touch_dev->reported_touch_info.coord[i].x);
						input_report_abs(touch_dev->input_dev, ABS_MT_POSITION_Y, touch_dev->reported_touch_info.coord[i].y);						
						input_mt_sync(touch_dev->input_dev);	
					}
				}				
				memset(&touch_dev->reported_touch_info, 0x0, sizeof(_ts_zinitix_point_info));					
				input_sync(touch_dev->input_dev);
				goto continue_read_samples;
			}
		

			for(i=0; i< SUPPORTED_FINGER_NUM; i++){
					
				if(zinitix_bit_test(touch_dev->touch_info.coord[i].sub_status, SUB_BIT_DOWN)
					|| zinitix_bit_test(touch_dev->touch_info.coord[i].sub_status, SUB_BIT_MOVE)
					|| zinitix_bit_test(touch_dev->touch_info.coord[i].sub_status, SUB_BIT_EXIST)){
					
					x = touch_dev->touch_info.coord[i].x;
					y = touch_dev->touch_info.coord[i].y;
					
					 /* transformation from touch to screen orientation */
					if (touch_dev->cap_info.Orientation & TOUCH_V_FLIP){
						y = touch_dev->cap_info.MaxY + touch_dev->cap_info.MinY - y;			               
					}
                    
					if (touch_dev->cap_info.Orientation & TOUCH_H_FLIP){
						x = touch_dev->cap_info.MaxX + touch_dev->cap_info.MinX - x;			               
					}
                    
					if (touch_dev->cap_info.Orientation & TOUCH_XY_SWAP){					
						zinitix_swap_v(x, y, tmp);
					}
                    
					touch_dev->touch_info.coord[i].x = x;
					touch_dev->touch_info.coord[i].y = y;

				if(zinitix_bit_test(touch_dev->touch_info.coord[i].sub_status, SUB_BIT_DOWN))
				{
					printk("[TSP] finger down \r\n");
					push_touch_trace(touch_dev->touch_info.coord[i].x, touch_dev->touch_info.coord[i].y, 0);
				}

			
					//input_report_abs(touch_dev->input_dev,ABS_MT_TRACKING_ID,i);
					if(touch_dev->touch_info.coord[i].width == 0)	
                                     touch_dev->touch_info.coord[i].width = 5;
                    
					input_report_abs(touch_dev->input_dev, ABS_MT_TOUCH_MAJOR, (u32)touch_dev->touch_info.coord[i].width);
					input_report_abs(touch_dev->input_dev, ABS_MT_WIDTH_MAJOR, (u32)touch_dev->touch_info.coord[i].width);					
					input_report_abs(touch_dev->input_dev, ABS_MT_POSITION_X, x);
					input_report_abs(touch_dev->input_dev, ABS_MT_POSITION_Y, y);
					input_mt_sync(touch_dev->input_dev);	


				}
				else if(zinitix_bit_test(touch_dev->touch_info.coord[i].sub_status, SUB_BIT_UP)){
                    
					printk("[TSP] finger sub up \r\n");
					push_touch_trace(touch_dev->reported_touch_info.coord[i].x, touch_dev->reported_touch_info.coord[i].y, 2);
					memset(&touch_dev->touch_info.coord[i], 0x0, sizeof(_ts_zinitix_coord));	
					//input_report_abs(touch_dev->input_dev, ABS_MT_TRACKING_ID,i);
					input_report_abs(touch_dev->input_dev, ABS_MT_TOUCH_MAJOR, 0);
					input_report_abs(touch_dev->input_dev, ABS_MT_WIDTH_MAJOR, 0);	
					input_report_abs(touch_dev->input_dev, ABS_MT_POSITION_X, touch_dev->reported_touch_info.coord[i].x);
					input_report_abs(touch_dev->input_dev, ABS_MT_POSITION_Y, touch_dev->reported_touch_info.coord[i].y);
					input_mt_sync(touch_dev->input_dev);

				}else
					memset(&touch_dev->touch_info.coord[i], 0x0, sizeof(_ts_zinitix_coord));	
	

			}		
            
			memcpy((char*)&touch_dev->reported_touch_info, (char*)&touch_dev->touch_info, sizeof(_ts_zinitix_point_info));	
			input_sync(touch_dev->input_dev);
			
#endif	// TOUCH_MODE == 1
		continue_read_samples:
			
			//check_interrupt_pin, if high, enable int & wait signal
#if USE_THREADED_IRQ
			if(1){
#else
			if (gpio_get_value(touch_dev->int_gpio_num)){
#endif                
				read_coord_continued = false;				
				//enable_irq(touch_dev->irq);
#if	ZINITIX_ESD_TIMER_INTERVAL					
				if(touch_dev->use_esd_timer){
					ts_esd_timer_start(ZINITIX_ESD_TIMER_INTERVAL*2, touch_dev);	
					zinitix_debug_msg("esd timer start\r\n");
				}
				
#endif		
                          #if USE_THREADED_IRQ
                          touch_dev->work_proceedure = TS_NO_WORK;
                          up(&touch_dev->work_proceedure_lock);
                          #else
                          touch_dev->work_proceedure = TS_NO_WORK;
				up(&touch_dev->work_proceedure_lock);
				enable_irq(touch_dev->irq);
                          #endif
			} else{
				zinitix_debug_msg("interrupt pin is still low, so continue read \r\n");			
			}
				
		}while(read_coord_continued);
#if USE_THREADED_IRQ
             return IRQ_HANDLED;
#endif
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static void zinitix_late_resume(struct early_suspend *h)
{
       int *mfpr_value = 0;

	zinitix_touch_dev * touch_dev;
	touch_dev = container_of(h, zinitix_touch_dev, early_suspend);
	printk(KERN_INFO "[TSP] Resume START\r\n");
    
	if(touch_dev == NULL)	return;


       mfpr_value = io_p2v(0x40e1044c);

       *(mfpr_value) = 0x8940; 
       
       touch_power_control(1);
       msleep(500);
       
	down(&touch_dev->work_proceedure_lock);
    
	if(touch_dev->work_proceedure != TS_IN_RESUME && touch_dev->work_proceedure != TS_IN_EALRY_SUSPEND){
             printk(KERN_INFO "[TSP] Resume touch_dev->work_proceedure = %d\r\n", touch_dev->work_proceedure);
		up(&touch_dev->work_proceedure_lock);
		return;
	}	
    
	ts_write_cmd(touch_dev->client, ZINITIX_WAKEUP_CMD);
	msleep(10);	
    
	if(ts_mini_init_touch(touch_dev)==false)	
            goto fail_resume;
    
	zinitix_debug_msg("wakeup\n");	

      #if USE_THREADED_IRQ
      //enable_irq(touch_dev->irq);
      	touch_dev->work_proceedure = TS_NO_WORK;	
	up(&touch_dev->work_proceedure_lock);
      #else
	enable_irq(touch_dev->irq);
	touch_dev->work_proceedure = TS_NO_WORK;	
	up(&touch_dev->work_proceedure_lock);
      #endif

      printk(KERN_INFO "[TSP] Resume END\r\n");
      
	return;	
fail_resume:
	printk(KERN_ERR "failed to wakeup\n");

      #if USE_THREADED_IRQ
      //enable_irq(touch_dev->irq);
      	touch_dev->work_proceedure = TS_NO_WORK;	
	up(&touch_dev->work_proceedure_lock);
      #else
	enable_irq(touch_dev->irq);
	touch_dev->work_proceedure = TS_NO_WORK;	
	up(&touch_dev->work_proceedure_lock);
      #endif
      
	return;
}


static void zinitix_early_suspend(struct early_suspend *h)
{
       int *mfpr_value =0;
	zinitix_touch_dev * touch_dev;

      printk(KERN_INFO "[TSP] Suspend START\r\n");

	touch_dev = container_of(h, zinitix_touch_dev, early_suspend);
    
       mfpr_value = io_p2v(0x40e1044c);

       *(mfpr_value) = 0x8840; 
    
	if(touch_dev == NULL)	
           goto	early_suspend_force_poweroff;

	down(&touch_dev->work_proceedure_lock);

      if(g_touch_dev->work_proceedure == TS_IN_UPGRADE){

           printk(KERN_INFO "[TSP] Firmware upgrading...\r\n");
           return;
      }
      
      zinitix_debug_msg(" START\r\n");

	if(touch_dev->work_proceedure != TS_NO_WORK){
             printk(KERN_INFO "[TSP] touch_dev->work_proceedure = %d\r\n", touch_dev->work_proceedure);
		up(&touch_dev->work_proceedure_lock);
		return;
	}
    
	touch_dev->work_proceedure = TS_IN_EALRY_SUSPEND;
	zinitix_clear_report_data(touch_dev);
	
#if	ZINITIX_ESD_TIMER_INTERVAL	
	if(touch_dev->use_esd_timer){			
		ts_write_reg(touch_dev->client, ZINITIX_PERIODICAL_INTERRUPT_INTERVAL, 0);
		ts_esd_timer_stop(touch_dev);
		printk(KERN_INFO "[TSP] ts_esd_timer_stop\n");			
	}
#endif		

	#if USE_THREADED_IRQ
	#else
	disable_irq(touch_dev->irq);
	#endif
	
	ts_write_reg(touch_dev->client, ZINITIX_INT_ENABLE_FLAG, 0x0);
	udelay(100);
	ts_write_cmd(touch_dev->client, ZINITIX_CLEAR_INT_STATUS_CMD);
    
	if (ts_write_cmd(touch_dev->client, ZINITIX_SLEEP_CMD)!=I2C_SUCCESS){
		printk(KERN_ERR "failed to enter into sleep mode\n");
		up(&touch_dev->work_proceedure_lock);
	           goto	early_suspend_force_poweroff;
	}	
    
	up(&touch_dev->work_proceedure_lock);

      touch_power_control(0);
      mdelay(250);

      printk(KERN_INFO "[TSP] Suspend END\r\n");
	return;

early_suspend_force_poweroff:
      printk(KERN_INFO "[TSP] Suspend force Power Off\r\n");
      touch_power_control(0);
      mdelay(250);
}

#endif	// CONFIG_HAS_EARLYSUSPEND



static ssize_t raw_store_zinitix(struct device *dev, struct device_attribute *attr, char *buf, size_t size)
{
	printk("[TSP] %s start. line : %d, \n", __func__,__LINE__);
	if(strncasecmp(buf, "start", 5) == 0)
	{
		printk("[TSP] %s start. line : %d, \n", __func__,__LINE__);
	}
	else if(strncasecmp(buf, "stop", 4) == 0)
	{
		printk("[TSP] %s end. line : %d, \n", __func__,__LINE__);
	}
      else
      {
      		printk("[TSP] %s error-unknown command. line : %d, \n", __func__,__LINE__);
      }

    return size ;
}

static ssize_t raw_show_zinitix(struct device *dev, struct device_attribute *attr, char *buf)
{

	int i,j;
	int k = 0;

	int written_bytes = 0 ;  /* & error check */

	int16_t raw_data[12][16]={{0,},};
	int16_t diff_data[12][16]={{0,},};

	uint8_t buf1[192]={0,};
	uint8_t buf2[192]={0,};
	uint8_t buf3[192]={0,};
	uint8_t buf4[192]={0,};

    
	down(&g_touch_dev->raw_data_lock);
	


	for(j = 0 ; j < 16 ; j++)
	{
		for(i = 11 ; i >= 0 ; i-- )
		{
			raw_data[i][j] =(int)(g_touch_dev->ref_data[k]&0xffff);
			diff_data[i][j] =(int)(((s16)(g_touch_dev->cur_data[k]-g_touch_dev->ref_data[k]))&0xffff);
			k++;
		}
	}
	
	up(&g_touch_dev->raw_data_lock);	

	
	for(i = 0 ; i < 12 ; i++)
	{
		printk("[TSP]");
		for(j = 0 ; j < 16 ; j++ )
		{
			printk(" %5d", raw_data[i][j]);
		}
		printk("\n");
	}

	
	for(i = 0 ; i < 12 ; i++)
	{
		printk("[TSP]");
		for(j = 0 ; j < 16 ; j++ )
		{
			printk(" %5d", diff_data[i][j]);
		}
		printk("\n");
	}
	
    for (i = 0; i < 12 ; i++)
    {
        for(j = 0 ; j < 16 ; j++)
        {
		written_bytes += sprintf(buf+written_bytes, "%d %d\n", raw_data[i][j], diff_data[i][j]) ;
        }
    }

    if (written_bytes > 0)
        return written_bytes ;

    return sprintf(buf, "-1") ;
 
}


static ssize_t diffdata_zinitix(struct device *dev, struct device_attribute *attr, char *buf)
{

	int i,j;
	int k = 0;

	int written_bytes = 0 ;  /* & error check */

	int16_t diff_data[12][16]={{0,},};
	int16_t raw_data[12][16]={{0,},};

    
	down(&g_touch_dev->raw_data_lock);
	


	for(j = 0 ; j < 16 ; j++)
	{
		for(i = 11 ; i >= 0 ; i-- )
		{
			raw_data[i][j] =(int)(g_touch_dev->ref_data[k]&0xffff);
			diff_data[i][j] =(int)(((s16)(g_touch_dev->cur_data[k]-g_touch_dev->ref_data[k]))&0xffff);
			k++;
		}
	}
	
	up(&g_touch_dev->raw_data_lock);	

	
	for(i = 0 ; i < 12 ; i++)
	{
		printk("[TSP]");
		for(j = 0 ; j < 16 ; j++ )
		{
			printk(" %5d", diff_data[i][j]);
		}
		printk("\n");
	}
	
    for (i = 0; i < 12 ; i++)
    {
        for(j = 0 ; j < 16 ; j++)
        {
			sprintf(buf+written_bytes, "%d\n",diff_data[i][j]);
		 	written_bytes += 4;
			
        }
    }

    if (written_bytes > 0)
        return written_bytes ;

    return sprintf(buf, "-1") ;
 
}





static ssize_t raw_enable_zinitix(struct device *dev, struct device_attribute *attr, char *buf)
{
	unsigned char value = 0;

	//struct input_dev *input = to_input_dev(dev);
    //zinitix_touch_dev *touch_dev = input_get_drvdata(input);

	printk(KERN_INFO "[zinitix_touch] zinitix_set_testmode, buf = %d\r\n", *buf);

	//if(touch_dev == NULL)
	//	zinitix_debug_msg("device NULL : NULL\n");
	
//	sscanf(buf, "%c", &value);
	down(&g_touch_dev->raw_data_lock);
	g_touch_dev->raw_mode_flag = TOUCH_TEST_RAW_MODE;

	printk(KERN_INFO "[zinitix_touch] zinitix_set_testmode, touchkey_testmode = %d\r\n", g_touch_dev->raw_mode_flag);

	//if(touch_dev->raw_mode_flag == 0) // 테스트 모드 진입시
	{
  	    printk(KERN_INFO "[zinitix_touch] TEST Mode Enter\r\n");

		if (ts_write_reg(g_touch_dev->client, ZINITIX_PERIODICAL_INTERRUPT_INTERVAL, 40)!=I2C_SUCCESS)
			printk(KERN_INFO "[zinitix_touch] Fail to set ZINITIX_RAW_DATA_ESD_TIMER_INTERVAL.\r\n");

	    if (ts_write_reg(g_touch_dev->client, ZINITIX_TOUCH_MODE, TOUCH_TEST_RAW_MODE)!=I2C_SUCCESS)
		{
			printk(KERN_INFO "[zinitix_touch] TEST Mode : Fail to set ZINITX_TOUCH_MODE %d.\r\n", TOUCH_TEST_RAW_MODE);
		}
		ts_write_cmd(g_touch_dev->client, ZINITIX_CLEAR_INT_STATUS_CMD);
		// clear garbage data
		msleep(10);
		ts_write_cmd(g_touch_dev->client, ZINITIX_CLEAR_INT_STATUS_CMD);		
		memset(&g_touch_dev->reported_touch_info, 0x0, sizeof(_ts_zinitix_point_info));
		memset(&g_touch_dev->touch_info, 0x0, sizeof(_ts_zinitix_point_info));
	}
    
	up(&g_touch_dev->raw_data_lock);

	tsp_testmode = 1;
	printk("[TSP] %s start. line : %d, \n", __func__,__LINE__);

	msleep(100); 

	return 1;
}


static ssize_t raw_disable_zinitix(struct device *dev, struct device_attribute *attr, char *buf)
{

	unsigned char value = 0;

	//struct input_dev *input = to_input_dev(dev);
//	zinitix_touch_dev *touch_dev = input_get_drvdata(input);

	printk(KERN_INFO "[zinitix_touch] zinitix_set_testmode, buf = %d\r\n", *buf);

//	if(touch_dev == NULL)
//		zinitix_debug_msg("device NULL : NULL\n");
	
//	sscanf(buf, "%c", &value);
	down(&g_touch_dev->raw_data_lock);
	g_touch_dev->raw_mode_flag = TOUCH_NORMAL_MODE;

	printk(KERN_INFO "[zinitix_touch] zinitix_set_testmode, touchkey_testmode = %d\r\n", g_touch_dev->raw_mode_flag);

	//if(touch_dev->raw_mode_flag == 48)	// 테스트 모드 해제시
	{	
		printk(KERN_INFO "[zinitix_touch] TEST Mode Exit\r\n");

		if (ts_write_reg(g_touch_dev->client, ZINITIX_PERIODICAL_INTERRUPT_INTERVAL, ZINITIX_SCAN_RATE_HZ*ZINITIX_ESD_TIMER_INTERVAL)!=I2C_SUCCESS)
			printk(KERN_INFO "[zinitix_touch] Fail to set ZINITIX_PERIODICAL_INTERRUPT_INTERVAL.\r\n");

		if (ts_write_reg(g_touch_dev->client, ZINITIX_TOUCH_MODE, TOUCH_MODE)!=I2C_SUCCESS)
		{
			printk(KERN_INFO "[zinitix_touch] Fail to set ZINITX_TOUCH_MODE %d.\r\n", TOUCH_MODE);
		}
		// clear garbage data
		ts_write_cmd(g_touch_dev->client, ZINITIX_CLEAR_INT_STATUS_CMD);
		msleep(10);
		ts_write_cmd(g_touch_dev->client, ZINITIX_CLEAR_INT_STATUS_CMD);		
	}
    
	up(&g_touch_dev->raw_data_lock);
	
	tsp_testmode = 0;
	printk("[TSP] %s stop. line : %d, \n", __func__,__LINE__);

	return 1;
}


static ssize_t read_rawdata(struct device *dev, struct device_attribute *attr, char *buf)//AT+TSPPTEST=1,1,1
{
	printk("[TSP] %s entered. line : %d, \n", __func__,__LINE__);


	return sprintf(buf, "-1") ;
}


static ssize_t read_difference(struct device *dev, struct device_attribute *attr, char *buf)//AT+TSPPTEST=1,2,1
{
	printk("[TSP] %s entered. line : %d, \n", __func__,__LINE__);
	

	return sprintf(buf, "-1") ;
}


static ssize_t rawdata_pass_fail_zinitix(struct device *dev, struct device_attribute *attr, char *buf)
{
	int time_out = 0;
	int i,j;
	int interrup_detecting;

	int phone_ver;
	int ic_ver;

	int16_t raw_data[12][16]={{0,},};
	//char temp_buff[384];

    printk(KERN_INFO "[zinitix_touch] zinitix_read_raw_data( )\r\n");
	disable_irq(g_touch_dev->client->irq); 

	if (ts_write_reg(g_touch_dev->client, ZINITIX_TOUCH_MODE,  7)!=I2C_SUCCESS)
	{
		printk(KERN_INFO "[zinitix_touch] ZINITIX_TOUCH_MODE fail\r\n");
		goto fail_read_raw; 
	}
	udelay(10);
	
	for(i=0; i<50; i++)
	{
		ts_write_cmd(g_touch_dev->client, ZINITIX_CLEAR_INT_STATUS_CMD);
		time_out = 0;
		interrup_detecting = 1;

		while(gpio_get_value(g_touch_dev->int_gpio_num))
		{
			mdelay(1);
			if(time_out++ > 20)
			{
			    printk(KERN_INFO "[zinitix_touch] interrupt disable timed out\r\n");
				interrup_detecting = 0;
				break;
			}
		}
		if(i==50)
		{
			printk(KERN_INFO "[zinitix_touch] interrupt disable timed out\r\n");
			goto fail_read_raw;  
		}

		if( (interrup_detecting==1)&&(time_out<20) )
		{
			if(ts_read_firmware_data(g_touch_dev->client, ZINITIX_RAWDATA_REG, (char *)raw_data, MAX_TEST_RAW_DATA*2)<0)
			{
				printk(KERN_INFO "[zinitix_touch] ZINITIX_RAWDATA_REG fail\r\n");
				goto fail_read_raw;
			}
#if 0
            memcpy(temp_buff, raw_data, 384);
			for(j=0; j<384; j++)
			{
				printk(KERN_INFO "[zinitix_touch] temp_buff[%d] = %x\r\n", j, temp_buff[j]);
			}
#endif
			break;
		}
	}

	ts_write_cmd(g_touch_dev->client, ZINITIX_CLEAR_INT_STATUS_CMD);
	udelay(10);
	
	if (ts_write_reg(g_touch_dev->client, ZINITIX_TOUCH_MODE, TOUCH_MODE)!=I2C_SUCCESS)
	{
		printk(KERN_INFO "[zinitix_touch] ZINITIX_TOUCH_MODE fail\r\n");
		goto fail_read_raw;  
	}
	ts_write_cmd(g_touch_dev->client, ZINITIX_CLEAR_INT_STATUS_CMD);
	udelay(10);
	
	enable_irq(g_touch_dev->client->irq);
	//return true;

	for(i = 0 ; i < 12 ; i++)
	{
		printk("[TSP]");
		for(j = 0 ; j < 16 ; j++ )
		{
			printk(" %5d", raw_data[i][j]);
		}
		printk("\n");
	}

	for(i = 0 ; i < 12 ; i++)
	{
		for(j = 0 ; j < 16 ; j++ )
		{
			if(raw_data[i][j]<700) 
				return sprintf(buf, "0"); // fail

			if(raw_data[i][j]>1020)
				return sprintf(buf, "0"); // fail
		}
	}

	phone_ver = m_firmware_data[0];
	ic_ver =   0x00ff & g_touch_dev->cap_info.chip_firmware_version;

	printk("phone_ver = %d, ic_ver = %d\n",phone_ver,ic_ver);

	if(phone_ver != ic_ver)
		return sprintf(buf, "0"); // fail

    return sprintf(buf, "1"); // success

fail_read_raw:
	if (ts_write_reg(g_touch_dev->client, ZINITIX_TOUCH_MODE, TOUCH_MODE)!=I2C_SUCCESS)
	{
		printk(KERN_INFO "[zinitix_touch] ZINITIX_TOUCH_MODE fail\r\n");
		goto fail_read_raw;
	}
	ts_write_cmd(g_touch_dev->client, ZINITIX_CLEAR_INT_STATUS_CMD);
	udelay(10);
	
	enable_irq(g_touch_dev->client->irq);
	return sprintf(buf, "-1");
  
 }

static ssize_t refdata_zinitix(struct device *dev, struct device_attribute *attr, char *buf)
{
	int time_out = 0;
	int i,j;
	int interrup_detecting;

	int phone_ver;
	int ic_ver;

	int written_bytes = 0 ;  /* & error check */

	int16_t raw_data[12][16]={{0,},};
	//char temp_buff[384];

    printk(KERN_INFO "[zinitix_touch] refdata_zinitix( )\r\n");
	disable_irq(g_touch_dev->client->irq); 

	if (ts_write_reg(g_touch_dev->client, ZINITIX_TOUCH_MODE,  7)!=I2C_SUCCESS)
	{
		printk(KERN_INFO "[zinitix_touch] ZINITIX_TOUCH_MODE fail\r\n");
		goto fail_read_raw; 
	}
	udelay(10);
	
	for(i=0; i<50; i++)
	{
		ts_write_cmd(g_touch_dev->client, ZINITIX_CLEAR_INT_STATUS_CMD);
		time_out = 0;
		interrup_detecting = 1;

		while(gpio_get_value(g_touch_dev->int_gpio_num))
		{
			mdelay(1);
			if(time_out++ > 20)
			{
			    printk(KERN_INFO "[zinitix_touch] interrupt disable timed out\r\n");
				interrup_detecting = 0;
				break;
			}
		}
		if(i==50)
		{
			printk(KERN_INFO "[zinitix_touch] interrupt disable timed out\r\n");
			goto fail_read_raw;  
		}

		if( (interrup_detecting==1)&&(time_out<20) )
		{
			if(ts_read_firmware_data(g_touch_dev->client, ZINITIX_RAWDATA_REG, (char *)raw_data, MAX_TEST_RAW_DATA*2)<0)
			{
				printk(KERN_INFO "[zinitix_touch] ZINITIX_RAWDATA_REG fail\r\n");
				goto fail_read_raw;
			}
#if 0
            memcpy(temp_buff, raw_data, 384);
			for(j=0; j<384; j++)
			{
				printk(KERN_INFO "[zinitix_touch] temp_buff[%d] = %x\r\n", j, temp_buff[j]);
			}
#endif
			break;
		}
	}

	ts_write_cmd(g_touch_dev->client, ZINITIX_CLEAR_INT_STATUS_CMD);
	udelay(10);
	
	if (ts_write_reg(g_touch_dev->client, ZINITIX_TOUCH_MODE, TOUCH_MODE)!=I2C_SUCCESS)
	{
		printk(KERN_INFO "[zinitix_touch] ZINITIX_TOUCH_MODE fail\r\n");
		goto fail_read_raw;  
	}
	ts_write_cmd(g_touch_dev->client, ZINITIX_CLEAR_INT_STATUS_CMD);
	udelay(10);
	
	enable_irq(g_touch_dev->client->irq);
	//return true;

	for(i = 0 ; i < 12 ; i++)
	{
		printk("[TSP]");
		for(j = 0 ; j < 16 ; j++ )
		{
			printk(" %5d", raw_data[i][j]);
		}
		printk("\n");
	}

	for (i = 0; i < 12 ; i++)
	{
		for(j = 0 ; j < 16 ; j++)
        	{
			sprintf(buf+written_bytes, "%d\n", raw_data[i][j]) ;
			written_bytes += 4;
        	}
    	}

    	if (written_bytes > 0)
			return written_bytes ;

	return sprintf(buf, "-1") ;



fail_read_raw:
	if (ts_write_reg(g_touch_dev->client, ZINITIX_TOUCH_MODE, TOUCH_MODE)!=I2C_SUCCESS)
	{
		printk(KERN_INFO "[zinitix_touch] ZINITIX_TOUCH_MODE fail\r\n");
		goto fail_read_raw;
	}
	ts_write_cmd(g_touch_dev->client, ZINITIX_CLEAR_INT_STATUS_CMD);
	udelay(10);
	
	enable_irq(g_touch_dev->client->irq);
	return sprintf(buf, "-1");
  
 }

static ssize_t scantime_zinitix(struct device *dev, struct device_attribute *attr, char *buf)
{
	int time_out = 0;
	int i,j;
	int interrup_detecting;

 	int written_bytes = 0 ;  /* & error check */
	u16	max_n_cnt = 0;
 
	u8 raw_data[20][16]={{0,},};
	u8 scantime_data[16][12]={{0,},};

 	printk(KERN_INFO "[zinitix_touch] scantime_zinitix( )\r\n");
	disable_irq(g_touch_dev->client->irq); 
 
	if (ts_read_data(g_touch_dev->client, 0x34, (u8*)&max_n_cnt, 2)<0)
	{
		 printk(KERN_INFO "fail to read Max N count\r\n");
		 goto fail_read_raw;
	}

 	if (ts_write_reg(g_touch_dev->client, ZINITIX_TOUCH_MODE,  8)!=I2C_SUCCESS)
	{
		printk(KERN_INFO "[zinitix_touch] ZINITIX_TOUCH_MODE fail\r\n");
		goto fail_read_raw; 
	}
	udelay(10);
	ts_write_cmd(g_touch_dev->client, ZINITIX_CLEAR_INT_STATUS_CMD);

  	//if(touch_dev->raw_mode_flag == TOUCH_ZINITIX_CAL_N_MODE)
  
 	if(ts_read_firmware_data(g_touch_dev->client,ZINITIX_RAWDATA_REG,(char *)raw_data, 320)<0)//(unsigned int)((unsigned int)161*2 + sizeof(_ts_zinitix_point_info)))<0)
	{
	printk(KERN_INFO "error : read zinitix tc Cal N raw data\n");
	goto fail_read_raw;
	}  
  
	ts_write_cmd(g_touch_dev->client, ZINITIX_CLEAR_INT_STATUS_CMD);
 	udelay(10);
  
 	if (ts_write_reg(g_touch_dev->client, ZINITIX_TOUCH_MODE, TOUCH_MODE)!=I2C_SUCCESS)
	{
	printk(KERN_INFO "[zinitix_touch] ZINITIX_TOUCH_MODE fail\r\n");
	goto fail_read_raw;  
	}
	ts_write_cmd(g_touch_dev->client, ZINITIX_CLEAR_INT_STATUS_CMD);
	udelay(10);
  
	enable_irq(g_touch_dev->client->irq);
  	//return true;

	for(j = 0 ; j < 12 ; j++) 
	{
		printk("[TSP]");
       	for(i = 0 ; i < 16 ; i++ ) 
       	{
			scantime_data[i][j] = ((u8)max_n_cnt) - raw_data[i][j+2];   //x_lsb
			printk(" %d", scantime_data[i][j]);
		}
  		printk("\n");
	}

 	for (j = 0 ; j < 12 ; j++)
	{
		for(i = 0 ; i < 16 ; i++ )
        	{
			sprintf(buf+written_bytes, "%d\n", scantime_data[i][j]) ;
			written_bytes += 4;
        	}
    	}

    	if (written_bytes > 0)
			return written_bytes ;

	return sprintf(buf, "-1") ;


 
fail_read_raw:
 	if (ts_write_reg(g_touch_dev->client, ZINITIX_TOUCH_MODE, TOUCH_MODE)!=I2C_SUCCESS)
 	{
  		printk(KERN_INFO "[zinitix_touch] ZINITIX_TOUCH_MODE fail\r\n");
  		goto fail_read_raw;
 	}
	ts_write_cmd(g_touch_dev->client, ZINITIX_CLEAR_INT_STATUS_CMD);
	udelay(10);
 
	enable_irq(g_touch_dev->client->irq);
	return sprintf(buf, "-1");
  
 }



#if 0
#if	USE_TEST_RAW_TH_DATA_MODE


static ssize_t zinitix_get_test_raw_data(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct input_dev *input = to_input_dev(dev);
	zinitix_touch_dev *touch_dev = input_get_drvdata(input);

	int i;

	if(touch_dev == NULL)
	{
		zinitix_debug_msg("device NULL : NULL\n");
		return 0;
	}
	down(&touch_dev->raw_data_lock);
	
	if(zinitix_bit_test(touch_dev->cur_data[touch_dev->cap_info.total_node_num], BIT_PT_EXIST))
	{
		buf[20] = (touch_dev->cur_data[touch_dev->cap_info.total_node_num+1]&0xff);			//x_lsb
		buf[21] = ((touch_dev->cur_data[touch_dev->cap_info.total_node_num+1]>>8)&0xff);		//x_msb
		buf[22] = (touch_dev->cur_data[touch_dev->cap_info.total_node_num+2]&0xff);			//y_lsb
		buf[23] = ((touch_dev->cur_data[touch_dev->cap_info.total_node_num+2]>>8)&0xff);		//y_msb
	}
	else	
	{
		buf[20] = 0; //x_lsb
		buf[21] = 0; //x_msb
		buf[22] = 0; //y_lsb
		buf[23] = 0; //y_msb
	}
		
	buf[0] = (char)(touch_dev->ref_data[22]&0xff);		//lsb
	buf[1] = (char)((touch_dev->ref_data[22]>>8)&0xff); 	//msb
	buf[2] = (char)(((s16)(touch_dev->cur_data[22]-touch_dev->ref_data[22]))&0xff);			//delta lsb
	buf[3] = (char)((((s16)(touch_dev->cur_data[22]-touch_dev->ref_data[22]))>>8)&0xff); 	//delta msb	
	buf[4] = (char)(touch_dev->ref_data[51]&0xff);		//lsb
	buf[5] = (char)((touch_dev->ref_data[51]>>8)&0xff); 	//msb
	buf[6] = (char)(((s16)(touch_dev->cur_data[51]-touch_dev->ref_data[51]))&0xff);			//delta lsb
	buf[7] = (char)((((s16)(touch_dev->cur_data[51]-touch_dev->ref_data[51]))>>8)&0xff); 	//delta msb

	buf[8] = (char)(touch_dev->ref_data[102]&0xff); 	 //lsb
	buf[9] = (char)((touch_dev->ref_data[102]>>8)&0xff);	 //msb
	buf[10] = (char)(((s16)(touch_dev->cur_data[102]-touch_dev->ref_data[102]))&0xff);		 //delta lsb
	buf[11] = (char)((((s16)(touch_dev->cur_data[102]-touch_dev->ref_data[102]))>>8)&0xff);	 //delta msb

	buf[12] = (char)(touch_dev->ref_data[169]&0xff); 	 //lsb
	buf[13] = (char)((touch_dev->ref_data[169]>>8)&0xff);	 //msb
	buf[14] = (char)(((s16)(touch_dev->cur_data[169]-touch_dev->ref_data[169]))&0xff);		 //delta lsb
	buf[15] = (char)((((s16)(touch_dev->cur_data[169]-touch_dev->ref_data[169]))>>8)&0xff);	 //delta msb	

	buf[16] = (char)(touch_dev->ref_data[178]&0xff);	  //lsb
	buf[17] = (char)((touch_dev->ref_data[178]>>8)&0xff);	  //msb
	buf[18] = (char)(((s16)(touch_dev->cur_data[178]-touch_dev->ref_data[178]))&0xff); 	  //delta lsb
	buf[19] = (char)((((s16)(touch_dev->cur_data[178]-touch_dev->ref_data[178]))>>8)&0xff);  //delta msb  
	up(&touch_dev->raw_data_lock);
 
		for(i = 0 ; i < 20 ; i++)
        	{
		printk(" %5d", buf[i]);
        	}
 
 return 24; //NOISE_TEST_THRESHOLD_SIZE; // 20 Byte
}


ssize_t zinitix_set_testmode(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	unsigned char value = 0;

	struct input_dev *input = to_input_dev(dev);
    zinitix_touch_dev *touch_dev = input_get_drvdata(input);

	printk(KERN_INFO "[zinitix_touch] zinitix_set_testmode, buf = %d\r\n", *buf);

	if(touch_dev == NULL)
		zinitix_debug_msg("device NULL : NULL\n");
	
	sscanf(buf, "%c", &value);

	if(value !=	TOUCH_TEST_RAW_MODE && value !=	TOUCH_NORMAL_MODE)
	{
		printk(KERN_WARNING "[zinitix ts] test mode setting value error. you must set %d[=normal] or %d[=raw mode]\r\n", TOUCH_NORMAL_MODE, TOUCH_TEST_RAW_MODE);
		return 1;
	}
	
	down(&touch_dev->raw_data_lock);
	touch_dev->raw_mode_flag = value;

	printk(KERN_INFO "[zinitix_touch] zinitix_set_testmode, touchkey_testmode = %d\r\n", touch_dev->raw_mode_flag);

	if(touch_dev->raw_mode_flag == TOUCH_NORMAL_MODE){	// 테스트 모드 해제시
		printk(KERN_INFO "[zinitix_touch] TEST Mode Exit\r\n");

		if (ts_write_reg(touch_dev->client, ZINITIX_PERIODICAL_INTERRUPT_INTERVAL, ZINITIX_SCAN_RATE_HZ*ZINITIX_ESD_TIMER_INTERVAL)!=I2C_SUCCESS)
			printk(KERN_INFO "[zinitix_touch] Fail to set ZINITIX_PERIODICAL_INTERRUPT_INTERVAL.\r\n");

		if (ts_write_reg(touch_dev->client, ZINITIX_TOUCH_MODE, TOUCH_MODE)!=I2C_SUCCESS){
			printk(KERN_INFO "[zinitix_touch] Fail to set ZINITX_TOUCH_MODE %d.\r\n", TOUCH_MODE);
		}
		// clear garbage data
		ts_write_cmd(touch_dev->client, ZINITIX_CLEAR_INT_STATUS_CMD);
		mdelay(100);
		ts_write_cmd(touch_dev->client, ZINITIX_CLEAR_INT_STATUS_CMD);		
	}else{ // 테스트 모드 진입시
  	      printk(KERN_INFO "[zinitix_touch] TEST Mode Enter\r\n");
  
		if(ts_write_reg(touch_dev->client, ZINITIX_PERIODICAL_INTERRUPT_INTERVAL, ZINITIX_SCAN_RATE_HZ*ZINITIX_RAW_DATA_ESD_TIMER_INTERVAL)!=I2C_SUCCESS)
			printk(KERN_INFO "[zinitix_touch] Fail to set ZINITIX_RAW_DATA_ESD_TIMER_INTERVAL.\r\n");

	       if(ts_write_reg(touch_dev->client, ZINITIX_TOUCH_MODE, TOUCH_TEST_RAW_MODE)!=I2C_SUCCESS){
			printk(KERN_INFO "[zinitix_touch] TEST Mode : Fail to set ZINITX_TOUCH_MODE 51.\r\n");
		}
           
		ts_write_cmd(touch_dev->client, ZINITIX_CLEAR_INT_STATUS_CMD);
		// clear garbage data
		mdelay(100);
		ts_write_cmd(touch_dev->client, ZINITIX_CLEAR_INT_STATUS_CMD);		
		memset(&touch_dev->reported_touch_info, 0x0, sizeof(_ts_zinitix_point_info));
		memset(&touch_dev->touch_info, 0x0, sizeof(_ts_zinitix_point_info));
	}
    
	up(&touch_dev->raw_data_lock);
	return 1;             

}

static DEVICE_ATTR(get_touch_test_raw_data, S_IRUGO | S_IWUSR | S_IWOTH | S_IXOTH, zinitix_get_test_raw_data, NULL);
static DEVICE_ATTR(set_touch_testmode, S_IRUGO | S_IWUSR | S_IWOTH | S_IXOTH, NULL, zinitix_set_testmode);

#endif
#endif

static int zinitix_touch_probe(struct i2c_client *client, const struct i2c_device_id *i2c_id)
{
	int ret;
	zinitix_touch_dev* touch_dev;
	int i;
    int *mfpr_value =0;

  	zinitix_debug_msg("zinitix_touch_probe+\r\n");
#if	BT4x2_Series  	
  	zinitix_debug_msg("BT4x2 Driver\r\n");		
#endif

  	zinitix_debug_msg("driver version = %s\r\n", TS_DRVIER_VERSION);	

#if	TOUCH_USING_ISP_METHOD	
	if(strcmp(client->name, ZINITIX_ISP_NAME) == 0){
		printk(KERN_INFO "isp client probe \r\n");
		m_isp_client = client;
		return 0;
	}
#endif

  	zinitix_debug_msg("touch data alloc \r\n");	
	touch_dev = kzalloc(sizeof(zinitix_touch_dev), GFP_KERNEL);

    
	if (!touch_dev) {
	  	printk(KERN_ERR "unabled to allocate touch data \r\n");	
		ret = -ENOMEM;
		goto err_alloc_dev_data;
	}

	touch_dev->board_rev=get_board_id();
      g_touch_dev = touch_dev;
      /* + to get touch regulator */
	if(g_touch_dev->board_rev==6)
	{
		//tsp_regulator = regulator_get(&(client->dev), "v_touch1");
		tsp_regulator = regulator_get(&(client->dev), "v_mic_bias");

      if (IS_ERR(tsp_regulator)) {	
		tsp_regulator = NULL;
		zinitix_debug_msg("failed to get regulator\n");
      }

      tsp_regulator2 = regulator_get(&(client->dev), "v_touch2");

      if (IS_ERR(tsp_regulator2)) {	
		tsp_regulator2 = NULL;
		zinitix_debug_msg("failed to get regulator\n");
      }
	}
      /* + for TSP_LDO_EN */ 
      tsp_ldo_en = mfp_to_gpio(MFP_PIN_GPIO124);
      
	if (gpio_request(tsp_ldo_en, "tsp_ldo_en"))
	    printk(KERN_ERR "tsp_ldo_en Request GPIO_%d failed!\n", tsp_ldo_en);    
      /* - for TSP_LDO_EN */ 
      
       mfpr_value = io_p2v(0x40e1044c);

       *(mfpr_value) = 0x8940;
      
      touch_power_control(1);
      mdelay(CHIP_ON_DELAY);
      /* - to get touch regulator */

  	zinitix_debug_msg("i2c check function \r\n");	
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		printk(KERN_ERR "error : not compatible i2c function \r\n");
		ret = -ENODEV;
		goto err_check_functionality;
	}

      
	touch_dev->client = client;
	i2c_set_clientdata(client, touch_dev);

      #if USE_THREADED_IRQ
      /* Do Nothing */
      #else
	INIT_WORK(&touch_dev->work, zinitix_touch_work);
      #endif
      
	zinitix_debug_msg("touch thread create \r\n");

       #if USE_THREADED_IRQ
       /* Do Nothing */
       #else
	zinitix_workqueue = create_singlethread_workqueue("zinitix_workqueue");
       
	if (!zinitix_workqueue)
	{
		printk(KERN_ERR "unabled to create touch thread \r\n");
		ret = -1;
		goto err_kthread_create_failed;
	}
      #endif


	//wake_up_process( touch_dev->task );
	zinitix_debug_msg("allocate input device \r\n");
	touch_dev->input_dev = input_allocate_device();
	if (touch_dev->input_dev == 0) {
		printk(KERN_ERR "unabled to allocate input device \r\n");
		ret = -ENOMEM;
		goto err_input_allocate_device;
	}

	//initialize zinitix touch ic
	touch_dev->int_gpio_num = GPIO_TOUCH_PIN_NUM;	// for upgrade	

	memset(&touch_dev->reported_touch_info, 0x0, sizeof(_ts_zinitix_point_info));

#if	USE_TEST_RAW_TH_DATA_MODE
	touch_dev->raw_mode_flag = TOUCH_NORMAL_MODE;			// not test mode
#endif	

	ts_init_touch(touch_dev);	

	touch_dev->use_esd_timer = 0;

#if	ZINITIX_ESD_TIMER_INTERVAL	
	INIT_WORK(&touch_dev->tmr_work, zinitix_touch_tmr_work);
	
	zinitix_tmr_workqueue = create_singlethread_workqueue("zinitix_tmr_workqueue")
        ;
	if (!zinitix_tmr_workqueue){
		printk(KERN_ERR "unabled to create touch tmr work queue \r\n");
		ret = -1;
		goto err_kthread_create_failed;
	}

	touch_dev->use_esd_timer = 1;
	ts_esd_timer_init(touch_dev);	
	ts_esd_timer_start(ZINITIX_ESD_TIMER_INTERVAL*2, touch_dev);	
	printk(KERN_INFO " ts_esd_timer_start\n");	
#endif	

	sprintf(touch_dev->phys, "input(ts)");
	touch_dev->input_dev->name = ZINITIX_DRIVER_NAME;
	//touch_dev->input_dev->phys = "zinitix_touch/input0";	// <- for compatability
   	touch_dev->input_dev->id.bustype = BUS_I2C;
   	touch_dev->input_dev->id.vendor = 0x0001;
	touch_dev->input_dev->phys = touch_dev->phys;
   	touch_dev->input_dev->id.product = 0x0002;
   	touch_dev->input_dev->id.version = 0x0100;
	//touch_dev->input_dev->dev.parent = &client->dev;
    
	set_bit(EV_SYN, touch_dev->input_dev->evbit);
	set_bit(EV_KEY, touch_dev->input_dev->evbit);
	set_bit(BTN_TOUCH, touch_dev->input_dev->keybit);
	set_bit(EV_ABS, touch_dev->input_dev->evbit);

	if(SUPPORTED_BUTTON_NUM > 0){
		for(i=0; i< SUPPORTED_BUTTON_NUM; i++)	
	    	set_bit(BUTTON_MAPPING_KEY[i], touch_dev->input_dev->keybit);
	}	


	if (touch_dev->cap_info.Orientation & TOUCH_XY_SWAP){
		input_set_abs_params(touch_dev->input_dev, ABS_MT_POSITION_Y, touch_dev->cap_info.MinX, SYSTEM_MAX_X_RESOLUTION+1, 0, 0);		
		input_set_abs_params(touch_dev->input_dev, ABS_MT_POSITION_X, touch_dev->cap_info.MinY, SYSTEM_MAX_Y_RESOLUTION+1, 0, 0);
	}else{
		input_set_abs_params(touch_dev->input_dev, ABS_MT_POSITION_X, touch_dev->cap_info.MinX, SYSTEM_MAX_X_RESOLUTION+1, 0, 0);		
		input_set_abs_params(touch_dev->input_dev, ABS_MT_POSITION_Y, touch_dev->cap_info.MinY, SYSTEM_MAX_Y_RESOLUTION+1, 0, 0);
	}

	input_set_abs_params(touch_dev->input_dev, ABS_TOOL_WIDTH, 0, 255, 0, 0);				
	input_set_abs_params(touch_dev->input_dev, ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0);
	input_set_abs_params(touch_dev->input_dev, ABS_MT_WIDTH_MAJOR, 0, 255, 0, 0);

	zinitix_debug_msg("register %s input device \r\n", touch_dev->input_dev->name);
	ret = input_register_device(touch_dev->input_dev);
    
	if(ret) {
		printk(KERN_ERR "unable to register %s input device\r\n", touch_dev->input_dev->name);
        goto err_input_register_device;
	}
	
	/* configure touchscreen interrupt gpio */
	ret = gpio_request(GPIO_TOUCH_PIN_NUM, "zinitix_irq_gpio");
    
	if (ret) {
		printk(KERN_ERR "unable to request gpio.(%s)\r\n", touch_dev->input_dev->name);
		goto err_request_irq;
	}

	ret = gpio_direction_input(GPIO_TOUCH_PIN_NUM);

      zinitix_debug_msg("gpio_direction_input ret = %d\n", ret);

	touch_dev->int_gpio_num = GPIO_TOUCH_PIN_NUM;

#ifdef	GPIO_TOUCH_IRQ
	touch_dev->irq = GPIO_TOUCH_IRQ;
#else
	touch_dev->irq = gpio_to_irq(touch_dev->int_gpio_num);

      zinitix_debug_msg("touch_dev->irq = %d\n", touch_dev->irq);
      
	
#endif
	zinitix_debug_msg("request irq (irq = %d, pin = %d) \r\n", touch_dev->irq, touch_dev->int_gpio_num);

	touch_dev->work_proceedure = TS_NO_WORK;
	sema_init(&touch_dev->work_proceedure_lock, 1); 

	if (touch_dev->irq) {
        
             #if USE_THREADED_IRQ
             ret = request_threaded_irq(touch_dev->irq, ts_int_handler, zinitix_touch_work, IRQF_TRIGGER_FALLING | IRQF_ONESHOT , ZINITIX_DRIVER_NAME, touch_dev);
             #else
		ret = request_irq(touch_dev->irq, ts_int_handler, IRQF_TRIGGER_FALLING, ZINITIX_DRIVER_NAME, touch_dev);
             #endif
             
		if (ret) {
			printk(KERN_ERR "unable to register irq.(%s)\r\n", touch_dev->input_dev->name);
			goto err_request_irq;
		}	
	}
	dev_info(&client->dev, "zinitix touch probe.\r\n");

#ifdef CONFIG_HAS_EARLYSUSPEND
	touch_dev->early_suspend.level = EARLY_SUSPEND_LEVEL_STOP_DRAWING + 1;
	touch_dev->early_suspend.suspend = zinitix_early_suspend;
	touch_dev->early_suspend.resume = zinitix_late_resume;
	register_early_suspend(&touch_dev->early_suspend);
#endif


       /* sys fs */
	touch_class = class_create(THIS_MODULE, "touch");

	if (IS_ERR(touch_class))
		pr_err("Failed to create class(touch)!\n");

	firmware_dev = device_create(touch_class, NULL, 0, NULL, "firmware");
    
	if (IS_ERR(firmware_dev))
		pr_err("Failed to create device(firmware)!\n");

	if (device_create_file(firmware_dev, &dev_attr_firmware) < 0)
		pr_err("Failed to create device file(%s)!\n", dev_attr_firmware.attr.name);
	if (device_create_file(firmware_dev, &dev_attr_firmware_ret) < 0)
		pr_err("Failed to create device file(%s)!\n", dev_attr_firmware_ret.attr.name);
	if (device_create_file(firmware_dev, &dev_attr_versname) < 0)
		pr_err("Failed to create device file(%s)!\n", dev_attr_versname.attr.name);

      if (device_create_file(firmware_dev, &dev_attr_raw) < 0)
		pr_err("Failed to create device file(%s)!\n", dev_attr_raw.attr.name);
	if (device_create_file(firmware_dev, &dev_attr_raw_enable) < 0)
		pr_err("Failed to create device file(%s)!\n", dev_attr_raw_enable.attr.name);
	if (device_create_file(firmware_dev, &dev_attr_raw_disable) < 0)
		pr_err("Failed to create device file(%s)!\n", dev_attr_raw_disable.attr.name);

	if (device_create_file(firmware_dev, &dev_attr_rawdata_read) < 0)
	   pr_err("Failed to create device file(%s)!\n", dev_attr_rawdata_read.attr.name);
	if (device_create_file(firmware_dev, &dev_attr_difference_read) < 0)
	   pr_err("Failed to create device file(%s)!\n", dev_attr_difference_read.attr.name);
	if (device_create_file(firmware_dev, &dev_attr_raw_value) < 0)
		pr_err("Failed to create device file(%s)!\n", dev_attr_raw_value.attr.name);	
	if (device_create_file(firmware_dev, &dev_attr_diff_value) < 0)
		pr_err("Failed to create device file(%s)!\n", dev_attr_diff_value.attr.name);
	if (device_create_file(firmware_dev, &dev_attr_ref_value) < 0)
		pr_err("Failed to create device file(%s)!\n", dev_attr_ref_value.attr.name);
	if (device_create_file(firmware_dev, &dev_attr_tsp_scantime) < 0)
		pr_err("Failed to create device file(%s)!\n", dev_attr_tsp_scantime.attr.name);	

	/* sys fs */

	sema_init(&touch_dev->raw_data_lock, 1);

#if 0
#if	USE_TEST_RAW_TH_DATA_MODE
	sema_init(&touch_dev->raw_data_lock, 1);

	if (device_create_file(firmware_dev, &dev_attr_get_touch_test_raw_data) < 0)
		printk("Failed to create device file(%s)!\n", dev_attr_get_touch_test_raw_data.attr.name);
	if (device_create_file(firmware_dev, &dev_attr_set_touch_testmode) < 0)
		printk("Failed to create device file(%s)!\n", dev_attr_set_touch_testmode.attr.name);
#endif
#endif


	return 0;

err_request_irq:
	input_unregister_device(touch_dev->input_dev);
err_input_register_device:
	input_free_device(touch_dev->input_dev);
err_kthread_create_failed:	
err_input_allocate_device:	
	kfree(touch_dev);
err_alloc_dev_data:	
err_check_functionality:

	return ret;
}


static int zinitix_touch_remove(struct i2c_client *client)
{
	zinitix_touch_dev *touch_dev = i2c_get_clientdata(client);

	zinitix_debug_msg("zinitix_touch_remove+ \r\n");	
	down(&touch_dev->work_proceedure_lock);
    
	if(touch_dev->work_proceedure != TS_NO_WORK){
		flush_work(&touch_dev->work);
	}
    
	touch_dev->work_proceedure = TS_REMOVE_WORK;
	
#if	ZINITIX_ESD_TIMER_INTERVAL	
	if(touch_dev->use_esd_timer){	
		flush_work(&touch_dev->tmr_work);		
		ts_write_reg(touch_dev->client, ZINITIX_PERIODICAL_INTERRUPT_INTERVAL, 0);
		ts_esd_timer_stop(touch_dev);			
		zinitix_debug_msg(KERN_INFO " ts_esd_timer_stop\n");			
	}
#endif
	if (touch_dev->irq) {
		free_irq(touch_dev->irq, touch_dev);
	}

#ifdef CONFIG_HAS_EARLYSUSPEND
	unregister_early_suspend(&touch_dev->early_suspend);
#endif

      #if USE_THREADED_IRQ
      /* Do Nothing */
      #else
	destroy_workqueue(zinitix_workqueue);
      #endif

#if	ZINITIX_ESD_TIMER_INTERVAL	
	destroy_workqueue(zinitix_tmr_workqueue);
#endif

	if(gpio_is_valid(touch_dev->int_gpio_num))
		gpio_free(touch_dev->int_gpio_num);

	input_unregister_device(touch_dev->input_dev);
	input_free_device(touch_dev->input_dev);
	up(&touch_dev->work_proceedure_lock);	
	kfree(touch_dev);

	return 0;
}

static int __devinit zinitix_touch_init(void)
{
      zinitix_debug_msg("START!! \n");      
	return i2c_add_driver(&zinitix_touch_driver);    
}

static void __exit zinitix_touch_exit(void)
{
	i2c_del_driver(&zinitix_touch_driver);
}

module_init(zinitix_touch_init);
module_exit(zinitix_touch_exit);

MODULE_DESCRIPTION("touch-screen device driver using i2c interface");
MODULE_AUTHOR("sohnet <swjang@zinitix.com>");
MODULE_LICENSE("GPL");


