/*.......................................................................................................
. COPYRIGHT (C)  SAMSUNG Electronics CO., LTD (Suwon, Korea). 2009           
. All rights are reserved. Reproduction and redistiribution in whole or 
. in part is prohibited without the written consent of the copyright owner.
. 
.   Developer:
.   Date:
.   Description:  
..........................................................................................................
*/

/*******************************************************************************************
 #  Display resolution standards #

	QCIF: 176 x 144
	CIF: 352 x 288
	QVGA: 320 x 240
	VGA: 640 x 480 
	SVGA: 800 x 600 
	XGA: 1024 x 768 
	WXGA: 1280 x 800 
	QVGA: 1280 x 960 
	SXGA: 1280 x 1024 
	SXGA+: 1400 x 1050 
	WSXGA+: 1680 x 1050 
	UXGA: 1600 x 1200 
	WUXGA: 1920 x 1200 
	QXGA: 2048 x 1536
********************************************************************************************/

#if !defined(_CAMACQ_MT9V114_MIPI_H_)
#define _CAMACQ_MT9V114_MIPI_H_

/* Include */
#include "camacq_api.h"

/* Global */
#undef GLOBAL

#if !defined(_CAMACQ_API_C_)
#define GLOBAL extern
#else
#define GLOBAL
#endif /* _CAMACQ_API_C_ */

/* Include */
#if defined(WIN32)
#include "cmmfile.h"
#endif /* WIN32 */

/* Definition */
#define _MT9V114_MIPI_    //sensor option

#define CAMACQ_SUB_NAME         "mt9v114"
#define CAMACQ_SUB_I2C_ID       0x3D                    // 0x7A
#define CAMACQ_SUB_RES_TYPE   	 CAMACQ_SENSOR_SUB     // sub sensor

#define CAMACQ_SUB_ISPROBED     0
#define CAMACQ_SUB_CLOCK        0               
#define CAMACQ_SUB_YUVORDER     0
#define CAMACQ_SUB_V_SYNCPOL    0
#define CAMACQ_SUB_H_SYNCPOL    0
#define CAMACQ_SUB_SAMPLE_EDGE  0
#define CAMACQ_SUB_FULL_RANGE   0

#define CAMACQ_SUB_RST 
#define CAMACQ_SUB_RST_MUX 
#define CAMACQ_SUB_EN 
#define CAMACQ_SUB_EN_MUX 

#define CAMACQ_SUB_RST_ON          1
#define CAMACQ_SUB_RST_OFF         0
#define CAMACQ_SUB_EN_ON           1
#define CAMACQ_SUB_EN_OFF          0
#define CAMACQ_SUB_STANDBY_ON      1
#define CAMACQ_SUB_STANDBY_OFF	    0


#define CAMACQ_SUB_POWER_CTRL(onoff)
 
#define CAMACQ_SUB_2BYTE_SENSOR    0
#define CAMACQ_SUB_AF              0
#define CAMACQ_SUB_INT_MODE        CAMACQ_EXT_LEN_2BYTE_INT
#define CAMACQ_SUB_FS_MODE         0
#define CAMACQ_SUB_PATH_SET_FILE   "/sdcard/sensor/sub/%s.dat"

#if (CAMACQ_SUB_2BYTE_SENSOR)	
#define CAMACQ_SUB_BURST_MODE 0
#else
#define CAMACQ_SUB_BURST_MODE 0
#endif /* CAMACQ_SUB2BYTE_SENSOR */

#define CAMACQ_SUB_BURST_I2C_TRACE 0
#define CAMACQ_SUB_BURST_MAX 100

#define CAMACQ_SUB_REG_FLAG_CNTS 	0x0F12
#define CAMACQ_SUB_REG_DELAY 		0xFFFF
#define CAMACQ_SUB_BTM_OF_DATA 	{0xFFFF, 0xFFFF},
#define CAMACQ_SUB_END_MARKER 		0xFF
#define CAMACQ_SUB_REG_SET_SZ 		2	// {0xFFFFFFFF} is 1, {0xFFFF,0xFFFF} is 2, {0xFF,0XFF} is 2, {0xFF,0xFF,0xFF,0xFF} is 4, {0xFFFF} is 1
#define CAMACQ_SUB_REG_DAT_SZ 		2 // {0xFFFFFFFF} is 4, {0xFFFF,0xFFFF} is 2, {0xFF,0XFF} is 1, {0xFF,0xFF,0xFF,0xFF} is 1, {0xFFFF} is 2

#define CAMACQ_SUB_FRATE_MIN  5
#define CAMACQ_SUB_FRATE_MAX  30

// MACRO FUNCTIONS BEGIN //////////////////////////////////////////////////////////// 
#if (CAMACQ_SUB_2BYTE_SENSOR)
#define CAMACQ_SUB_EXT_RD_SZ 1
#else
#define CAMACQ_SUB_EXT_RD_SZ 2
#endif /* CAMACQ_SUB_2BYTE_SENSOR */

#if CAMACQ_SUB_2BYTE_SENSOR
#define CAMACQ_SUB_EXT_REG_IS_BTM_OF_DATA(A)		(((A[0]==CAMACQ_SUB_END_MARKER) && (A[1]==CAMACQ_SUB_END_MARKER))? 1:0)
#define CAMACQ_SUB_EXT_REG_IS_DELAY(A)				((A[0]==CAMACQ_SUB_REG_DELAY)? 1:0)

#if (CAMACQ_SUB_FS_MODE==1)
#define CAMACQ_SUB_EXT_REG_GET_DATA(dest,srce,idx)\
memcpy(dest, &(srce[idx*CAMACQ_SUB_REG_DAT_SZ*CAMACQ_SUB_REG_SET_SZ]), CAMACQ_SUB_REG_DAT_SZ*CAMACQ_SUB_REG_SET_SZ);
#elif (CAMACQ_SUB_REG_DAT_SZ==1)
#define CAMACQ_SUB_EXT_REG_GET_DATA(dest,srce,idx)	dest[0] = (srce[idx][0] & 0xFF); dest[1] = (srce[idx][1] & 0xFF);
#elif (CAMACQ_SUB_REG_DAT_SZ==2)
#define CAMACQ_SUB_EXT_REG_GET_DATA(dest,srce,idx)	dest[0] = ((U8)(srce[idx] >> 8) & 0xFF); dest[1] = ((U8)(srce[idx]) & 0xFF);
#endif

#else // CAMACQ_SUB_2BYTE_SENSOR

#define CAMACQ_SUB_EXT_REG_IS_BTM_OF_DATA(A)		(((A[0]==CAMACQ_SUB_END_MARKER) && (A[1]==CAMACQ_SUB_END_MARKER) && \
(A[2]==CAMACQ_SUB_END_MARKER) && (A[3]==CAMACQ_SUB_END_MARKER))? 1:0)
#define CAMACQ_SUB_EXT_REG_IS_DELAY(A)				(((A[0]==((CAMACQ_SUB_REG_DELAY>>8) & 0xFF)) && (A[1]==(CAMACQ_SUB_REG_DELAY & 0xFF)))? 1:0)
#define CAMACQ_SUB_EXT_REG_IS_CNTS(A)				(((A[0]==((CAMACQ_SUB_REG_FLAG_CNTS>>8) & 0xFF)) && (A[1]==(CAMACQ_SUB_REG_FLAG_CNTS & 0xFF)))? 1:0)

#if (CAMACQ_SUB_FS_MODE==1)
#define CAMACQ_SUB_EXT_REG_GET_DATA(dest,srce,idx)\
memcpy(dest, &(srce[idx*CAMACQ_SUB_REG_DAT_SZ*CAMACQ_SUB_REG_SET_SZ]), CAMACQ_SUB_REG_DAT_SZ*CAMACQ_SUB_REG_SET_SZ);
#elif (CAMACQ_SUB_REG_DAT_SZ==2)
#define CAMACQ_SUB_EXT_REG_GET_DATA(dest,srce,idx)	dest[0]=((srce[idx][0] >> 8) & 0xFF); dest[1]=(srce[idx][0] & 0xFF); \
dest[2]=((srce[idx][1] >> 8) & 0xFF); dest[3]=(srce[idx][1] & 0xFF);
#elif (CAMACQ_SUB_REG_DAT_SZ==1)
#define CAMACQ_SUB_EXT_REG_GET_DATA(dest,srce,idx)	dest[0]=srce[idx][0]; dest[1]=srce[idx][1]; \
dest[2]=srce[idx][2]; dest[3]=srce[idx][3];
#elif (CAMACQ_SUB_REG_DAT_SZ==4)
#define CAMACQ_SUB_EXT_REG_GET_DATA(dest,srce,idx)	dest[0] = ((U8)(srce[idx] >> 24) & 0xFF); dest[1] = ((U8)(srce[idx] >> 16) & 0xFF); \
dest[2] = ((U8)(srce[idx] >> 8) & 0xFF); dest[3] = ((U8)(srce[idx]) & 0xFF);			
#endif
#endif /* CAMACQ_2BYTE_SENSOR */
// MACRO FUNCTIONS END /////////////////////////////////////////////////////////// 


/* DEFINE for sensor regs*/
#if( CAMACQ_SUB_FS_MODE )
#define CAMACQ_SUB_REG_INIT                     "reg_sub_init"
#define CAMACQ_SUB_REG_PLL                      "reg_sub_pll"
#define CAMACQ_SUB_REG_WORKAROUND               "reg_sub_workaround"
#define CAMACQ_SUB_REG_DTP_ON                   "reg_sub_dtp_on"
#define CAMACQ_SUB_REG_DTP_OFF                  "reg_sub_dtp_off"
#define CAMACQ_SUB_REG_CAMCORDER_ON             "reg_sub_camcorder_on"
#else
#define CAMACQ_SUB_REG_INIT                     reg_sub_init
#define CAMACQ_SUB_REG_PLL                      reg_sub_pll 
#define CAMACQ_SUB_REG_WORKAROUND               reg_sub_workaround
#define CAMACQ_SUB_REG_DTP_ON                   reg_sub_dtp_on
#define CAMACQ_SUB_REG_DTP_OFF                  reg_sub_dtp_off
#define CAMACQ_SUB_REG_CAMCORDER_ON             reg_sub_camcorder_on
#endif

#define CAMACQ_SUB_REG_PREVIEW                 reg_sub_preview
#define CAMACQ_SUB_REG_SNAPSHOT                reg_sub_snapshot
#define CAMACQ_SUB_REG_MIDLIGHT                reg_sub_midlight
#define CAMACQ_SUB_REG_LOWLIGHT                reg_sub_lowlight
#define CAMACQ_SUB_REG_NIGHTSHOT_ON            reg_sub_nightshot_on
#define CAMACQ_SUB_REG_NIGHTSHOT_OFF           reg_sub_nightshot_off
#define CAMACQ_SUB_REG_NIGHTSHOT               reg_sub_nightshot

#define CAMACQ_SUB_REG_WB_AUTO                 reg_sub_wb_auto
#define CAMACQ_SUB_REG_WB_DAYLIGHT             reg_sub_wb_daylight
#define CAMACQ_SUB_REG_WB_CLOUDY               reg_sub_wb_cloudy
#define CAMACQ_SUB_REG_WB_INCANDESCENT         reg_sub_wb_incandescent
#define CAMACQ_SUB_REG_WB_FLUORESCENT          reg_sub_wb_fluorescent
#define CAMACQ_SUB_REG_WB_TWILIGHT             reg_sub_wb_twilight
#define CAMACQ_SUB_REG_WB_TUNGSTEN             reg_sub_wb_tungsten
#define CAMACQ_SUB_REG_WB_OFF                  reg_sub_wb_off
#define CAMACQ_SUB_REG_WB_HORIZON              reg_sub_wb_horizon
#define CAMACQ_SUB_REG_WB_SHADE                reg_sub_wb_shade

#define CAMACQ_SUB_REG_EFFECT_NONE             reg_sub_effect_none
#define CAMACQ_SUB_REG_EFFECT_GRAY             reg_sub_effect_gray // mono, blackwhite
#define CAMACQ_SUB_REG_EFFECT_NEGATIVE         reg_sub_effect_negative
#define CAMACQ_SUB_REG_EFFECT_SOLARIZE         reg_sub_effect_solarize
#define CAMACQ_SUB_REG_EFFECT_SEPIA            reg_sub_effect_sepia
#define CAMACQ_SUB_REG_EFFECT_POSTERIZE        reg_sub_effect_posterize
#define CAMACQ_SUB_REG_EFFECT_WHITEBOARD       reg_sub_effect_whiteboard
#define CAMACQ_SUB_REG_EFFECT_BLACKBOARD       reg_sub_effect_blackboard
#define CAMACQ_SUB_REG_EFFECT_AQUA             reg_sub_effect_aqua
#define CAMACQ_SUB_REG_EFFECT_SHARPEN          reg_sub_effect_sharpen
#define CAMACQ_SUB_REG_EFFECT_VIVID            reg_sub_effect_vivid
#define CAMACQ_SUB_REG_EFFECT_GREEN            reg_sub_effect_green
#define CAMACQ_SUB_REG_EFFECT_SKETCH           reg_sub_effect_sketch

#define CAMACQ_SUB_REG_METER_MATRIX            reg_sub_meter_matrix
#define CAMACQ_SUB_REG_METER_CW                reg_sub_meter_cw
#define CAMACQ_SUB_REG_METER_SPOT              reg_sub_meter_spot

#define CAMACQ_SUB_REG_FLIP_NONE               reg_sub_flip_none
#define CAMACQ_SUB_REG_FLIP_WATER              reg_sub_flip_water
#define CAMACQ_SUB_REG_FLIP_MIRROR             reg_sub_flip_mirror
#define CAMACQ_SUB_REG_FLIP_WATER_MIRROR       reg_sub_flip_water_mirror


#define CAMACQ_SUB_REG_FPS_FIXED_5             reg_sub_fps_fixed_5
#define CAMACQ_SUB_REG_FPS_FIXED_7             reg_sub_fps_fixed_7
#define CAMACQ_SUB_REG_FPS_FIXED_10            reg_sub_fps_fixed_10
#define CAMACQ_SUB_REG_FPS_FIXED_15            reg_sub_fps_fixed_15
#define CAMACQ_SUB_REG_FPS_FIXED_20            reg_sub_fps_fixed_20
#define CAMACQ_SUB_REG_FPS_FIXED_25            reg_sub_fps_fixed_25
#define CAMACQ_SUB_REG_FPS_FIXED_30            reg_sub_fps_fixed_30
#define CAMACQ_SUB_REG_FPS_VAR_15              reg_sub_fps_var_15

#define CAMACQ_SUB_REG_BRIGHTNESS_LEVEL_0      reg_sub_brightness_level_0
#define CAMACQ_SUB_REG_BRIGHTNESS_LEVEL_1      reg_sub_brightness_level_1
#define CAMACQ_SUB_REG_BRIGHTNESS_LEVEL_2      reg_sub_brightness_level_2
#define CAMACQ_SUB_REG_BRIGHTNESS_LEVEL_3      reg_sub_brightness_level_3
#define CAMACQ_SUB_REG_BRIGHTNESS_LEVEL_4      reg_sub_brightness_level_4
#define CAMACQ_SUB_REG_BRIGHTNESS_LEVEL_5      reg_sub_brightness_level_5
#define CAMACQ_SUB_REG_BRIGHTNESS_LEVEL_6      reg_sub_brightness_level_6
#define CAMACQ_SUB_REG_BRIGHTNESS_LEVEL_7      reg_sub_brightness_level_7
#define CAMACQ_SUB_REG_BRIGHTNESS_LEVEL_8      reg_sub_brightness_level_8

#define CAMACQ_SUB_REG_EXPCOMPENSATION_LEVEL_0 reg_sub_expcompensation_level_0
#define CAMACQ_SUB_REG_EXPCOMPENSATION_LEVEL_1 reg_sub_expcompensation_level_1
#define CAMACQ_SUB_REG_EXPCOMPENSATION_LEVEL_2 reg_sub_expcompensation_level_2
#define CAMACQ_SUB_REG_EXPCOMPENSATION_LEVEL_3 reg_sub_expcompensation_level_3
#define CAMACQ_SUB_REG_EXPCOMPENSATION_LEVEL_4 reg_sub_expcompensation_level_4
#define CAMACQ_SUB_REG_EXPCOMPENSATION_LEVEL_5 reg_sub_expcompensation_level_5
#define CAMACQ_SUB_REG_EXPCOMPENSATION_LEVEL_6 reg_sub_expcompensation_level_6
#define CAMACQ_SUB_REG_EXPCOMPENSATION_LEVEL_7 reg_sub_expcompensation_level_7
#define CAMACQ_SUB_REG_EXPCOMPENSATION_LEVEL_8 reg_sub_expcompensation_level_8

#define CAMACQ_SUB_REG_SET_AF                  reg_sub_set_af  // start af
#define CAMACQ_SUB_REG_OFF_AF                  reg_sub_off_af
#define CAMACQ_SUB_REG_CHECK_AF                reg_sub_check_af
#define CAMACQ_SUB_REG_RESET_AF                reg_sub_reset_af
#define CAMACQ_SUB_REG_CANCEL_MACRO_AF         reg_sub_cancel_macro_af
#define CAMACQ_SUB_REG_CANCEL_MANUAL_AF        reg_sub_cancel_manual_af
#define CAMACQ_SUB_REG_MANUAL_AF               reg_sub_manual_af    // normal_af
#define CAMACQ_SUB_REG_MACRO_AF                reg_sub_macro_af
#define CAMACQ_SUB_REG_RETURN_MANUAL_AF        reg_sub_return_manual_af
#define CAMACQ_SUB_REG_RETURN_MACRO_AF         reg_sub_return_macro_af
#define CAMACQ_SUB_REG_SET_AF_NLUX             reg_sub_set_af_nlux
#define CAMACQ_SUB_REG_SET_AF_LLUX             reg_sub_set_af_llux
#define CAMACQ_SUB_REG_SET_AF_NORMAL_MODE_1    reg_sub_set_af_normal_mode_1
#define CAMACQ_SUB_REG_SET_AF_NORMAL_MODE_2    reg_sub_set_af_normal_mode_2
#define CAMACQ_SUB_REG_SET_AF_NORMAL_MODE_3    reg_sub_set_af_normal_mode_3
#define CAMACQ_SUB_REG_SET_AF_MACRO_MODE_1     reg_sub_set_af_macro_mode_1
#define CAMACQ_SUB_REG_SET_AF_MACRO_MODE_2     reg_sub_set_af_macro_mode_2
#define CAMACQ_SUB_REG_SET_AF_MACRO_MODE_3     reg_sub_set_af_macro_mode_3

#define CAMACQ_SUB_REG_ISO_AUTO                reg_sub_iso_auto
#define CAMACQ_SUB_REG_ISO_50                  reg_sub_iso_50
#define CAMACQ_SUB_REG_ISO_100                 reg_sub_iso_100
#define CAMACQ_SUB_REG_ISO_200                 reg_sub_iso_200
#define CAMACQ_SUB_REG_ISO_400                 reg_sub_iso_400
#define CAMACQ_SUB_REG_ISO_800                 reg_sub_iso_800
#define CAMACQ_SUB_REG_ISO_1600                reg_sub_iso_1600
#define CAMACQ_SUB_REG_ISO_3200                reg_sub_iso_3200

#define CAMACQ_SUB_REG_SCENE_AUTO              reg_sub_scene_auto  // auto, off
#define CAMACQ_SUB_REG_SCENE_NIGHT             reg_sub_scene_night
#define CAMACQ_SUB_REG_SCENE_LANDSCAPE         reg_sub_scene_landscape
#define CAMACQ_SUB_REG_SCENE_SUNSET            reg_sub_scene_sunset
#define CAMACQ_SUB_REG_SCENE_PORTRAIT          reg_sub_scene_portrait
#define CAMACQ_SUB_REG_SCENE_SUNRISE           reg_sub_scene_sunrise    // dawn
#define CAMACQ_SUB_REG_SCENE_INDOOR            reg_sub_scene_indoor
#define CAMACQ_SUB_REG_SCENE_PARTY             reg_sub_scene_party
#define CAMACQ_SUB_REG_SCENE_SPORTS            reg_sub_scene_sports
#define CAMACQ_SUB_REG_SCENE_BEACH             reg_sub_scene_beach
#define CAMACQ_SUB_REG_SCENE_SNOW              reg_sub_scene_snow
#define CAMACQ_SUB_REG_SCENE_FALLCOLOR         reg_sub_scene_fallcolor
#define CAMACQ_SUB_REG_SCENE_FIREWORKS         reg_sub_scene_fireworks
#define CAMACQ_SUB_REG_SCENE_CANDLELIGHT       reg_sub_scene_candlelight
#define CAMACQ_SUB_REG_SCENE_AGAINSTLIGHT      reg_sub_scene_againstlight  // backlight
#define CAMACQ_SUB_REG_SCENE_TEXT              reg_sub_scene_text
#define CAMACQ_SUB_REG_SCENE_AQUA              reg_sub_scene_aqua

#define CAMACQ_SUB_REG_ADJUST_CONTRAST_M2              reg_sub_adjust_contrast_m2
#define CAMACQ_SUB_REG_ADJUST_CONTRAST_M1              reg_sub_adjust_contrast_m1
#define CAMACQ_SUB_REG_ADJUST_CONTRAST_DEFAULT         reg_sub_adjust_contrast_default
#define CAMACQ_SUB_REG_ADJUST_CONTRAST_P1              reg_sub_adjust_contrast_p1
#define CAMACQ_SUB_REG_ADJUST_CONTRAST_P2              reg_sub_adjust_contrast_p2

#define CAMACQ_SUB_REG_ADJUST_SHARPNESS_M2             reg_sub_adjust_sharpness_m2
#define CAMACQ_SUB_REG_ADJUST_SHARPNESS_M1             reg_sub_adjust_sharpness_m1
#define CAMACQ_SUB_REG_ADJUST_SHARPNESS_DEFAULT        reg_sub_adjust_sharpness_default
#define CAMACQ_SUB_REG_ADJUST_SHARPNESS_P1             reg_sub_adjust_sharpness_p1
#define CAMACQ_SUB_REG_ADJUST_SHARPNESS_P2             reg_sub_adjust_sharpness_p2

#define CAMACQ_SUB_REG_ADJUST_SATURATION_M2            reg_sub_adjust_saturation_m2
#define CAMACQ_SUB_REG_ADJUST_SATURATION_M1            reg_sub_adjust_saturation_m1
#define CAMACQ_SUB_REG_ADJUST_SATURATION_DEFAULT       reg_sub_adjust_saturation_default
#define CAMACQ_SUB_REG_ADJUST_SATURATION_P1            reg_sub_adjust_saturation_p1
#define CAMACQ_SUB_REG_ADJUST_SATURATION_P2            reg_sub_adjust_saturation_p2

#define CAMACQ_SUB_REG_QCIF                     reg_sub_qcif
#define CAMACQ_SUB_REG_QVGA                     reg_sub_qvga
#define CAMACQ_SUB_REG_CIF                      reg_sub_cif
#define CAMACQ_SUB_REG_VGA                      reg_sub_vga
#define CAMACQ_SUB_REG_1080P		            reg_sub_1080P
#define CAMACQ_SUB_REG_720P			            reg_sub_720P
#define CAMACQ_SUB_REG_800_480			        reg_sub_800_480
#define CAMACQ_SUB_REG_720_480			        reg_sub_720_480
#define CAMACQ_SUB_REG_NOT                      reg_sub_not

#define CAMACQ_SUB_REG_JPEG_5M                 reg_sub_jpeg_5m        //2560X1920
#define CAMACQ_SUB_REG_JPEG_5M_2               reg_sub_jpeg_5m_2      // 2592X1944
#define CAMACQ_SUB_REG_JPEG_W4M                 reg_sub_jpeg_w4m      // 2560x1536
#define CAMACQ_SUB_REG_JPEG_3M                 reg_sub_jpeg_3m        // QXGA 2048X1536
#define CAMACQ_SUB_REG_JPEG_2M                 reg_sub_jpeg_2m        // UXGA 1600x1200
#define CAMACQ_SUB_REG_JPEG_W1_5M               reg_sub_jpeg_w1_5m    // 1280x960
#define CAMACQ_SUB_REG_JPEG_1M                 reg_sub_jpeg_1m
#define CAMACQ_SUB_REG_JPEG_VGA                reg_sub_jpeg_vga   //640x480
#define CAMACQ_SUB_REG_JPEG_WQVGA              reg_sub_jpeg_wqvga //420x240
#define CAMACQ_SUB_REG_JPEG_QVGA               reg_sub_jpeg_qvga  //320x240

#define CAMACQ_SUB_REG_FLICKER_DISABLED        reg_sub_flicker_disabled
#define CAMACQ_SUB_REG_FLICKER_50HZ            reg_sub_flicker_50hz
#define CAMACQ_SUB_REG_FLICKER_60HZ            reg_sub_flicker_60hz
#define CAMACQ_SUB_REG_FLICKER_AUTO            reg_sub_flicker_auto

// image quality
#define CAMACQ_SUB_REG_JPEG_QUALITY_SUPERFINE reg_sub_jpeg_quality_superfine
#define CAMACQ_SUB_REG_JPEG_QUALITY_FINE      reg_sub_jpeg_quality_fine
#define CAMACQ_SUB_REG_JPEG_QUALITY_NORMAL    reg_sub_jpeg_quality_normal

// Format
#define CAMACQ_SUB_REG_FMT_YUV422 	            reg_sub_fmt_yuv422
#define CAMACQ_SUB_REG_FMT_JPG		            reg_sub_fmt_jpg


// NEW
#define CAMACQ_SUB_REG_SLEEP                   reg_sub_sleep
#define CAMACQ_SUB_REG_WAKEUP                  reg_sub_wakeup

/* Enumeration */

/* Global Value */
GLOBAL const U16 reg_sub_workaround[][2]
#if defined(_CAMACQ_API_C_)
={
// [Leakage_current_workaround]
//{0x0018, 0x4028}, 	    // STANDBY_CONTROL
//{0xFFFF, 0x0064},       // DELAY=100
//{0x001A, 0x0013}, 	    // RESET_AND_MISC_CONTROL
//{0xFFFF, 0x000A},       // DELAY=10
//{0x001A, 0x0010}, 	    // RESET_AND_MISC_CONTROL
//{0xFFFF, 0x000A},       // DELAY=10

{0x0018, 0x4028}, 	    // STANDBY_CONTROL
{0xFFFF, 0x0064},       // DELAY=100
{0x3400, 0x7A3C}, 	    // MIPI_CONTROL
{0x098C, 0x02F0}, 	    // MCU_ADDRESS
{0x0990, 0x0000}, 	    // MCU_DATA_0
{0x098C, 0x02F2}, 	    // MCU_ADDRESS
{0x0990, 0x0210}, 	    // MCU_DATA_0
{0x098C, 0x02F4}, 	    // MCU_ADDRESS
{0x0990, 0x001A}, 	    // MCU_DATA_0
{0x098C, 0x2145}, 	    // MCU_ADDRESS [SEQ_ADVSEQ_CALLLIST_5]
{0x0990, 0x02F4}, 	    // MCU_DATA_0
{0x098C, 0xA134}, 	    // MCU_ADDRESS [SEQ_ADVSEQ_STACKOPTIONS]
{0x0990, 0x0001}, 	    // MCU_DATA_0
{0x31E0, 0x0001}, 	    // PIX_DEF_ID
{0x001A, 0x0210}, 	    // RESET_AND_MISC_CONTROL
{0x001E, 0x0777}, 	    // PAD_SLEW
{0x0016, 0x42DF}, 	    // CLOCKS_CONTROL
{0x0014, 0x2145}, 	    // PLL_CONTROL
{0x0014, 0x2145},	    // PLL_CONTROL
{0x0010, 0x0231},	    // PLL_DIVIDERS
{0x0012, 0x0000}, 	    // PLL_P_DIVIDERS
{0x0014, 0x244B}, 	    // PLL_CONTROL
{0xFFFF, 0x000A},       // DELAY=10
{0x0014, 0x304B}, 	    // PLL_CONTROL
{0xFFFF, 0x000A},       // DELAY=10
{0x0014, 0xB04A}, 	    // PLL_CONTROL
{0x301A, 0x021C}, 	    // RESET_REGISTER
{0xFFFF, 0x0064},	// DELAY=100

CAMACQ_SUB_BTM_OF_DATA
}
#endif /* _CAMACQ_API_C_ */
;

GLOBAL const U16 reg_sub_pll[][2]
#if defined(_CAMACQ_API_C_)
={

CAMACQ_SUB_BTM_OF_DATA
}
#endif /* _CAMACQ_API_C_ */
;

GLOBAL const U16 reg_sub_dtp_on[][2]
#if defined(_CAMACQ_API_C_)
={
// {0x3070, 0x0002},      // TEST_PATTERN_MODE_
{0x3070, 0x0001},      // TEST_PATTERN_MODE_

CAMACQ_SUB_BTM_OF_DATA
}
#endif /* _CAMACQ_API_C_ */
;

GLOBAL const U16 reg_sub_dtp_off[][2]
#if defined(_CAMACQ_API_C_)
={
{0x3070, 0x0000},      // TEST_PATTERN_MODE_

CAMACQ_SUB_BTM_OF_DATA
}
#endif /* _CAMACQ_API_C_ */
;

GLOBAL const U16 reg_sub_sleep[][2] 
#if defined(_CAMACQ_API_C_)
={

CAMACQ_SUB_BTM_OF_DATA
}
#endif /* _CAMACQ_API_C_ */
;

GLOBAL const U16 reg_sub_wakeup[][2] 
#if defined(_CAMACQ_API_C_)
={

CAMACQ_SUB_BTM_OF_DATA
}
#endif /* _CAMACQ_API_C_ */
;

GLOBAL const U16 reg_sub_init[][2]
#if defined(_CAMACQ_API_C_)
={
#if 1
{0x0018, 0x4028},
{0xFFFF, 0x0032},
{0x098C, 0x02F0},
{0x0990, 0x0000},
{0x098C, 0x02F2},
{0x0990, 0x0210},
{0x098C, 0x02F4},
{0x0990, 0x001A},
{0x098C, 0x2145},
{0x0990, 0x02F4},
{0x098C, 0xA134},
{0x0990, 0x0001},
{0x31E0, 0x0001},
{0x3400, 0x7A20},
{0x001A, 0x0210},
{0x001E, 0x0777},
{0x0016, 0x42DF},
{0x0014, 0x2145},
{0x0014, 0x2145},
{0x0010, 0x0110},
{0x0012, 0x0000},
{0x0014, 0x244B},
{0xFFFF, 0x0002},
{0x0014, 0x304B},
{0xFFFF, 0x0032},
{0x0014, 0xB04A},
{0x001A, 0x0010},
{0x001A, 0x0018},
{0x321C, 0x0003},
{0x098C, 0x2703},
{0x0990, 0x0280},
{0x098C, 0x2705},
{0x0990, 0x01E0},
{0x098C, 0x2707},
{0x0990, 0x0280},
{0x098C, 0x2709},
{0x0990, 0x01E0},
{0x098C, 0x270D},
{0x0990, 0x0000},
{0x098C, 0x270F},
{0x0990, 0x0000},
{0x098C, 0x2711},
{0x0990, 0x01E7},
{0x098C, 0x2713},
{0x0990, 0x0287},
{0x098C, 0x2715},
{0x0990, 0x0001},
{0x098C, 0x2717},
{0x0990, 0x0026},
{0x098C, 0x2719},
{0x0990, 0x001A},
{0x098C, 0x271B},
{0x0990, 0x006B},
{0x098C, 0x271D},
{0x0990, 0x006B},
{0x098C, 0x271F},
{0x0990, 0x0378},
{0x098C, 0x2721},
{0x0990, 0x03CF},
{0x098C, 0x2723},
{0x0990, 0x0000},
{0x098C, 0x2725},
{0x0990, 0x0000},
{0x098C, 0x2727},
{0x0990, 0x01E7},
{0x098C, 0x2729},
{0x0990, 0x0287},
{0x098C, 0x272B},
{0x0990, 0x0001},
{0x098C, 0x272D},
{0x0990, 0x0026},
{0x098C, 0x272F},
{0x0990, 0x001A},
{0x098C, 0x2731},
{0x0990, 0x006B},
{0x098C, 0x2733},
{0x0990, 0x006B},
{0x098C, 0x2735},
{0x0990, 0x0378},
{0x098C, 0x2737},
{0x0990, 0x03CF},
{0x098C, 0x2739},
{0x0990, 0x0000},
{0x098C, 0x273B},
{0x0990, 0x027F},
{0x098C, 0x273D},
{0x0990, 0x0000},
{0x098C, 0x273F},
{0x0990, 0x01DF},
{0x098C, 0x2747},
{0x0990, 0x0000},
{0x098C, 0x2749},
{0x0990, 0x027F},
{0x098C, 0x274B},
{0x0990, 0x0000},
{0x098C, 0x274D},
{0x0990, 0x01DF},
{0x098C, 0x222D},
{0x0990, 0x006F},
{0x098C, 0xA408},
{0x0990, 0x0024},
{0x098C, 0xA409},
{0x0990, 0x0026},
{0x098C, 0xA40A},
{0x0990, 0x002B},
{0x098C, 0xA40B},
{0x0990, 0x002D},
{0x098C, 0x2411},
{0x0990, 0x006F},
{0x098C, 0x2413},
{0x0990, 0x0085},
{0x098C, 0x2415},
{0x0990, 0x006F},
{0x098C, 0x2417},
{0x0990, 0x0085},
{0x098C, 0xA404},
{0x0990, 0x0010},
{0x098C, 0xA40D},
{0x0990, 0x0002},
{0x098C, 0xA40E},
{0x0990, 0x0003},
{0x098C, 0xA410},
{0x0990, 0x000A},
{0x3210, 0x09B0},
{0x364E, 0x0090},
{0x3650, 0x48E7},
{0x3652, 0x29F3},
{0x3654, 0xF42E},
{0x3656, 0xEA94},
{0x3658, 0x7ECF},
{0x365A, 0x08CC},
{0x365C, 0x3B73},
{0x365E, 0x100D},
{0x3660, 0xE314},
{0x3662, 0x7F6F},
{0x3664, 0x3DCC},
{0x3666, 0x20F3},
{0x3668, 0x146D},
{0x366A, 0x8555},
{0x366C, 0x7F0F},
{0x366E, 0x8B2C},
{0x3670, 0x2BF3},
{0x3672, 0x84CF},
{0x3674, 0xE994},
{0x3676, 0x364C},
{0x3678, 0xF38F},
{0x367A, 0x5F52},
{0x367C, 0x86B1},
{0x367E, 0xF614},
{0x3680, 0x20AE},
{0x3682, 0xEC0F},
{0x3684, 0x32D3},
{0x3686, 0xD2B1},
{0x3688, 0xC3F5},
{0x368A, 0x3EAE},
{0x368C, 0x9D4F},
{0x368E, 0x73B2},
{0x3690, 0xFEF0},
{0x3692, 0xB695},
{0x3694, 0x7B6C},
{0x3696, 0xFCEF},
{0x3698, 0x7F32},
{0x369A, 0xB171},
{0x369C, 0xA855},
{0x369E, 0x7B53},
{0x36A0, 0xD971},
{0x36A2, 0x8C36},
{0x36A4, 0x3633},
{0x36A6, 0x2057},
{0x36A8, 0x0754},
{0x36AA, 0xAE31},
{0x36AC, 0x8E35},
{0x36AE, 0x9AF3},
{0x36B0, 0x8996},
{0x36B2, 0x6893},
{0x36B4, 0xB9B1},
{0x36B6, 0xB556},
{0x36B8, 0xA731},
{0x36BA, 0x24D8},
{0x36BC, 0x7373},
{0x36BE, 0x9DF2},
{0x36C0, 0xE095},
{0x36C2, 0x68B4},
{0x36C4, 0x4F34},
{0x36C6, 0xC16F},
{0x36C8, 0xD7AF},
{0x36CA, 0xF8D4},
{0x36CC, 0x6ED4},
{0x36CE, 0x89D8},
{0x36D0, 0x12B1},
{0x36D2, 0x9C10},
{0x36D4, 0xD7B5},
{0x36D6, 0x25D5},
{0x36D8, 0xFB97},
{0x36DA, 0x75F1},
{0x36DC, 0x5D30},
{0x36DE, 0xB635},
{0x36E0, 0xA8F4},
{0x36E2, 0xA2D8},
{0x36E4, 0x2EAE},
{0x36E6, 0xCBF1},
{0x36E8, 0x9C15},
{0x36EA, 0x0616},
{0x36EC, 0x9B97},
{0x36EE, 0xA715},
{0x36F0, 0x3CB4},
{0x36F2, 0x70B6},
{0x36F4, 0x1296},
{0x36F6, 0x999B},
{0x36F8, 0x8F15},
{0x36FA, 0x1532},
{0x36FC, 0x9938},
{0x36FE, 0x45B8},
{0x3700, 0x30F9},
{0x3702, 0xC1B5},
{0x3704, 0x3154},
{0x3706, 0x2A38},
{0x3708, 0x3817},
{0x370A, 0xE33B},
{0x370C, 0x8775},
{0x370E, 0x7D94},
{0x3710, 0xEAF6},
{0x3712, 0xEF15},
{0x3714, 0xAAB5},
{0x3644, 0x0140},
{0x3642, 0x0100},
{0x3210, 0x09B8},
{0x098C, 0x2306},
{0x0990, 0x0194},
{0x098C, 0x231C},
{0x0990, 0x003B},
{0x098C, 0x2308},
{0x0990, 0xFFA2},
{0x098C, 0x231E},
{0x0990, 0xFF5D},
{0x098C, 0x230A},
{0x0990, 0xFFCA},
{0x098C, 0x2320},
{0x0990, 0x0068},
{0x098C, 0x230C},
{0x0990, 0xFF8C},
{0x098C, 0x2322},
{0x0990, 0x004D},
{0x098C, 0x230E},
{0x0990, 0x016B},
{0x098C, 0x2324},
{0x0990, 0x0002},
{0x098C, 0x2310},
{0x0990, 0x0009},
{0x098C, 0x2326},
{0x0990, 0xFFB2},
{0x098C, 0x2312},
{0x0990, 0xFF90},
{0x098C, 0x2328},
{0x0990, 0x0076},
{0x098C, 0x2314},
{0x0990, 0xFE85},
{0x098C, 0x232A},
{0x0990, 0x00C4},
{0x098C, 0x2316},
{0x0990, 0x02EB},
{0x098C, 0x232C},
{0x0990, 0xFEC6},
{0x098C, 0x2318},
{0x0990, 0x0013},
{0x098C, 0x231A},
{0x0990, 0x003F},
{0x098C, 0x232E},
{0x0990, 0x000F},
{0x098C, 0x2330},
{0x0990, 0xFFF5},
{0x098C, 0xA34A},
{0x0990, 0x0059},
{0x098C, 0xA34B},
{0x0990, 0x00FB},
{0x098C, 0xA34C},
{0x0990, 0x0059},
{0x098C, 0xA34D},
{0x0990, 0x0086},
{0x098C, 0xA302},
{0x0990, 0x0012},
{0x098C, 0xA303},
{0x0990, 0x00BC},
{0x098C, 0xA351},
{0x0990, 0x0000},
{0x098C, 0xA352},
{0x0990, 0x007F},
{0x098C, 0xA355},
{0x0990, 0x000A},
{0x098C, 0xA35D},
{0x0990, 0x007C},
{0x098C, 0xA35E},
{0x0990, 0x0084},
{0x098C, 0xA35F},
{0x0990, 0x007E},
{0x098C, 0xA360},
{0x0990, 0x0082},
{0x098C, 0x2361},
{0x0990, 0x0140},
{0x098C, 0xA353},
{0x0990, 0x0070},
{0x098C, 0xA34E},
{0x0990, 0x00BC},
{0x098C, 0xA34F},
{0x0990, 0x0080},
{0x098C, 0xA350},
{0x0990, 0x006A},
{0x098C, 0xA363},
{0x0990, 0x00DA},
{0x098C, 0xA364},
{0x0990, 0x00EA},
{0x098C, 0xA366},
{0x0990, 0x00A9},
{0x098C, 0xA367},
{0x0990, 0x0088},
{0x098C, 0xA368},
{0x0990, 0x0069},
{0x098C, 0xA369},
{0x0990, 0x0080},
{0x098C, 0xA36A},
{0x0990, 0x0080},
{0x098C, 0xA36B},
{0x0990, 0x0080},
{0x3240, 0xEB14},
{0x3244, 0x0370},
{0x098C, 0x275F},
{0x0990, 0x0594},
{0x098C, 0x2761},
{0x0990, 0x00AA},
{0x098C, 0xA117},
{0x0990, 0x0001},
{0x098C, 0xA119},
{0x0990, 0x0000},
{0x098C, 0xA11A},
{0x0990, 0x0000},
{0x098C, 0xA11D},
{0x0990, 0x0002},
{0x098C, 0xA11E},
{0x0990, 0x0001},
{0x098C, 0xA11F},
{0x0990, 0x0001},
{0x098C, 0xA120},
{0x0990, 0x0001},
{0x098C, 0xA129},
{0x0990, 0x0002},
{0x098C, 0xA20C},
{0x0990, 0x000F},
{0x098C, 0xA215},
{0x0990, 0x0008},
{0x098C, 0xA216},
{0x0990, 0x0040},
{0x098C, 0xA228},
{0x0990, 0x0080},
{0x098C, 0xA20D},
{0x0990, 0x001C},
{0x098C, 0xA20E},
{0x0990, 0x0080},
{0x098C, 0x2212},
{0x0990, 0x00FE},
{0x098C, 0xA244},
{0x0990, 0x0008},
{0x098C, 0xA24F},
{0x0990, 0x003F},
{0x098C, 0xA207},
{0x0990, 0x0007},
{0x098C, 0xA202},
{0x0990, 0x0034},
{0x098C, 0xA203},
{0x0990, 0x0097},
{0x098C, 0xA208},
{0x0990, 0x0003},
{0x098C, 0xAB1F},
{0x0990, 0x00C7},
{0x098C, 0x274F},
{0x0990, 0x0004},
{0x098C, 0x2741},
{0x0990, 0x0004},
{0x098C, 0xAB20},
{0x0990, 0x0090},
{0x098C, 0xAB21},
{0x0990, 0x001A},
{0x098C, 0xAB22},
{0x0990, 0x0003},
{0x098C, 0xAB23},
{0x0990, 0x0012},
{0x098C, 0xAB24},
{0x0990, 0x0055},
{0x098C, 0xAB25},
{0x0990, 0x0045},
{0x098C, 0xAB26},
{0x0990, 0x0001},
{0x098C, 0xAB27},
{0x0990, 0x0040},
{0x098C, 0x2B28},
{0x0990, 0x0400},
{0x098C, 0x2B2A},
{0x0990, 0x2200},
{0x098C, 0xAB2C},
{0x0990, 0x0010},
{0x098C, 0xAB2D},
{0x0990, 0x000A},
{0x098C, 0xAB2E},
{0x0990, 0x0010},
{0x098C, 0xAB2F},
{0x0990, 0x0008},
{0x098C, 0xAB30},
{0x0990, 0x0030},
{0x098C, 0xAB31},
{0x0990, 0x0020},
{0x098C, 0xAB32},
{0x0990, 0x0030},
{0x098C, 0xAB33},
{0x0990, 0x0020},
{0x098C, 0xAB34},
{0x0990, 0x0030},
{0x098C, 0xAB35},
{0x0990, 0x0080},
{0x098C, 0xAB36},
{0x0990, 0x0036},
{0x098C, 0xAB04},
{0x0990, 0x0010},
{0x098C, 0xAB06},
{0x0990, 0x000A},
{0x326C, 0x1605},
{0x098C, 0xAB37},
{0x0990, 0x0003},
{0x098C, 0x2B38},
{0x0990, 0x17A0},
{0x098C, 0x2B3A},
{0x0990, 0x1F00},
{0x098C, 0xAB3C},
{0x0990, 0x0000},
{0x098C, 0xAB3D},
{0x0990, 0x000E},
{0x098C, 0xAB3E},
{0x0990, 0x001B},
{0x098C, 0xAB3F},
{0x0990, 0x002B},
{0x098C, 0xAB40},
{0x0990, 0x004C},
{0x098C, 0xAB41},
{0x0990, 0x006B},
{0x098C, 0xAB42},
{0x0990, 0x0089},
{0x098C, 0xAB43},
{0x0990, 0x00A1},
{0x098C, 0xAB44},
{0x0990, 0x00B3},
{0x098C, 0xAB45},
{0x0990, 0x00C2},
{0x098C, 0xAB46},
{0x0990, 0x00CD},
{0x098C, 0xAB47},
{0x0990, 0x00D7},
{0x098C, 0xAB48},
{0x0990, 0x00DF},
{0x098C, 0xAB49},
{0x0990, 0x00E6},
{0x098C, 0xAB4A},
{0x0990, 0x00EC},
{0x098C, 0xAB4B},
{0x0990, 0x00F1},
{0x098C, 0xAB4C},
{0x0990, 0x00F6},
{0x098C, 0xAB4D},
{0x0990, 0x00FB},
{0x098C, 0xAB4E},
{0x0990, 0x00FF},
{0x098C, 0xAB4F},
{0x0990, 0x0012},
{0x098C, 0xAB50},
{0x0990, 0x0020},
{0x098C, 0xAB51},
{0x0990, 0x0030},
{0x098C, 0xAB52},
{0x0990, 0x0040},
{0x098C, 0xAB53},
{0x0990, 0x0060},
{0x098C, 0xAB54},
{0x0990, 0x0080},
{0x098C, 0xAB55},
{0x0990, 0x00A1},
{0x098C, 0xAB56},
{0x0990, 0x00B0},
{0x098C, 0xAB57},
{0x0990, 0x00BC},
{0x098C, 0xAB58},
{0x0990, 0x00C7},
{0x098C, 0xAB59},
{0x0990, 0x00D0},
{0x098C, 0xAB5A},
{0x0990, 0x00D8},
{0x098C, 0xAB5B},
{0x0990, 0x00DF},
{0x098C, 0xAB5C},
{0x0990, 0x00E6},
{0x098C, 0xAB5D},
{0x0990, 0x00EC},
{0x098C, 0xAB5E},
{0x0990, 0x00F1},
{0x098C, 0xAB5F},
{0x0990, 0x00F6},
{0x098C, 0xAB60},
{0x0990, 0x00FB},
{0x098C, 0xAB61},
{0x0990, 0x00FF},
{0x322A, 0x0004},
{0x098C, 0xAB1F},
{0x0990, 0x00C7},
{0x098C, 0xA103},
{0x0990, 0x0006},
{0xFFFF, 0x012C},
{0x098C, 0xA103},
{0x0990, 0x0005},
{0xFFFF, 0x0096},

#else //20110714
//Devware 에서만 필요한 명령어들
//XMCLK=24000000
//STATE=Sensor Reset, 1
//DELAY=1
//STATE=Sensor Reset, 0
//DELAY=1
//위쪽 영역은 Devware 에서 사용하는 셋팅으로 실제 폰 적용에서는 필요 없는 부분입니다.
//Phone setting 에서는 Power up 후 Sensor Reset 만 한번 주신 후 아래 I2C 셋팅만 진행 해 주시면 됩니다.
// {0x0018, 0x4028},	// STANDBY_CONTROL
// {0xFFFF, 0x0096},	// DELAY=150
// {0x001A, 0x0013},	// RESET_AND_MISC_CONTROL
// {0xFFFF, 0x000A},	// DELAY=10
// {0x001A, 0x0010},	// RESET_AND_MISC_CONTROL
// {0xFFFF, 0x000A},	// DELAY=10
{0x0018, 0x4028}, 	// STANDBY_CONTROL
{0xFFFF, 0x0096},	// DELAY=150 
{0x098C, 0x02F0},	// MCU_ADDRESS
{0x0990, 0x0000},	// MCU_DATA_0
{0x098C, 0x02F2},	// MCU_ADDRESS
{0x0990, 0x0210},	// MCU_DATA_0
{0x098C, 0x02F4},	// MCU_ADDRESS
{0x0990, 0x001A},	// MCU_DATA_0
{0x098C, 0x2145}, 	// MCU_ADDRESS [SEQ_ADVSEQ_CALLLIST_5]
{0x0990, 0x02F4},	// MCU_DATA_0
{0x098C, 0xA134}, 	// MCU_ADDRESS [SEQ_ADVSEQ_STACKOPTIONS]
{0x0990, 0x0001},	// MCU_DATA_0
{0x31E0, 0x0001},	// PIX_DEF_ID
{0x3400, 0x7A20}, 	// MIPI_CONTROL
{0x001A, 0x0210},	// RESET_AND_MISC_CONTROL
{0x001E, 0x0777},	// PAD_SLEW
{0x0016, 0x42DF},	// CLOCKS_CONTROL
{0x0014, 0x2145},	// PLL_CONTROL
{0x0014, 0x2145}, 	// PLL_CONTROL
{0x0010, 0x0110},	// PLL_DIVIDERS
{0x0012, 0x0000}, 	// PLL_P_DIVIDERS
{0x0014, 0x244B},	// PLL_CONTROL
{0xFFFF, 0x000A},	// DELAY=10
{0x0014, 0x304B},	// PLL_CONTROL

{0xFFFF, 0x000A},	// DELAY=10
{0x0014, 0xB04A},	// PLL_CONTROL

{0x001A, 0x0010},	// RESET_AND_MISC_CONTROL
{0x321C, 0x0003},	// OFIFO_CONTROL_STATUS
{0x001A, 0x0010},	// RESET_AND_MISC_CONTROL
{0x001A, 0x0018},	// RESET_AND_MISC_CONTROL

{0x098C, 0x2703},	// MCU_ADDRESS
{0x0990, 0x0280},	// MCU_DATA_0
//{0x0990, 0x00B0},	// MCU_DATA_0

{0x098C, 0x2705},	// MCU_ADDRESS
{0x0990, 0x01E0},	// MCU_DATA_0
//{0x0990, 0x0090},	// MCU_DATA_0

{0x098C, 0x2707}, 	// MCU_ADDRESS
{0x0990, 0x0280},	// MCU_DATA_0
{0x098C, 0x2709}, 	// MCU_ADDRESS
{0x0990, 0x01E0},	// MCU_DATA_0
{0x098C, 0x270D},	// MCU_ADDRESS
{0x0990, 0x0000},	// MCU_DATA_0
{0x098C, 0x270F},	// MCU_ADDRESS
{0x0990, 0x0000},	// MCU_DATA_0
{0x098C, 0x2711},	// MCU_ADDRESS
{0x0990, 0x01E7}, 	// MCU_DATA_0
{0x098C, 0x2713},	// MCU_ADDRESS
{0x0990, 0x0287}, 	// MCU_DATA_0
{0x098C, 0x2715},	// MCU_ADDRESS
{0x0990, 0x0001},	// MCU_DATA_0
{0x098C, 0x2717},	// MCU_ADDRESS
{0x0990, 0x0026},	// MCU_DATA_0
{0x098C, 0x2719},	// MCU_ADDRESS
{0x0990, 0x001A},	// MCU_DATA_0
{0x098C, 0x271B}, 	// MCU_ADDRESS
{0x0990, 0x006B},	// MCU_DATA_0
{0x098C, 0x271D}, 	// MCU_ADDRESS
{0x0990, 0x006B},	// MCU_DATA_0
{0x098C, 0x271F},	// MCU_ADDRESS
{0x0990, 0x01FB},	// MCU_DATA_0
{0x098C, 0x2721},	// MCU_ADDRESS
{0x0990, 0x0356},	// MCU_DATA_0
{0x098C, 0x2723},	// MCU_ADDRESS
{0x0990, 0x0000}, 	// MCU_DATA_0
{0x098C, 0x2725},	// MCU_ADDRESS
{0x0990, 0x0000},	// MCU_DATA_0
{0x098C, 0x2727},	// MCU_ADDRESS
{0x0990, 0x01E7},	// MCU_DATA_0
{0x098C, 0x2729},	// MCU_ADDRESS
{0x0990, 0x0287},	// MCU_DATA_0
{0x098C, 0x272B},	// MCU_ADDRESS
{0x0990, 0x0001}, 	// MCU_DATA_0
{0x098C, 0x272D},	// MCU_ADDRESS
{0x0990, 0x0026}, 	// MCU_DATA_0
{0x098C, 0x272F},	// MCU_ADDRESS
{0x0990, 0x001A},	// MCU_DATA_0
{0x098C, 0x2731},	// MCU_ADDRESS
{0x0990, 0x006B},	// MCU_DATA_0
{0x098C, 0x2733},	// MCU_ADDRESS
{0x0990, 0x006B},	// MCU_DATA_0
{0x098C, 0x2735}, 	// MCU_ADDRESS
{0x0990, 0x01FB},	// MCU_DATA_0
{0x098C, 0x2737}, 	// MCU_ADDRESS
{0x0990, 0x0356},	// MCU_DATA_0
{0x098C, 0x2739},	// MCU_ADDRESS
{0x0990, 0x0000},	// MCU_DATA_0
{0x098C, 0x273B},	// MCU_ADDRESS
{0x0990, 0x027F},	// MCU_DATA_0
{0x098C, 0x273D},	// MCU_ADDRESS
{0x0990, 0x0000}, 	// MCU_DATA_0
{0x098C, 0x273F},	// MCU_ADDRESS
{0x0990, 0x01DF}, 	// MCU_DATA_0
{0x098C, 0x2747},	// MCU_ADDRESS
{0x0990, 0x0000},	// MCU_DATA_0
{0x098C, 0x2749},	// MCU_ADDRESS
{0x0990, 0x027F},	// MCU_DATA_0
{0x098C, 0x274B},	// MCU_ADDRESS
{0x0990, 0x0000},	// MCU_DATA_0
{0x098C, 0x274D}, 	// MCU_ADDRESS
{0x0990, 0x01DF},	// MCU_DATA_0
{0x098C, 0x222D},	// MCU_ADDRESS
{0x0990, 0x007F},	// MCU_DATA_0
{0x098C, 0xA408},	// MCU_ADDRESS
{0x0990, 0x001E},	// MCU_DATA_0
{0x098C, 0xA409},	// MCU_ADDRESS
{0x0990, 0x0020},	// MCU_DATA_0
{0x098C, 0xA40A}, 	// MCU_ADDRESS
{0x0990, 0x0025},	// MCU_DATA_0
{0x098C, 0xA40B}, 	// MCU_ADDRESS
{0x0990, 0x0027},	// MCU_DATA_0
{0x098C, 0x2411},	// MCU_ADDRESS
{0x0990, 0x007F},	// MCU_DATA_0
{0x098C, 0x2413},	// MCU_ADDRESS
{0x0990, 0x0098},	// MCU_DATA_0
{0x098C, 0x2415},	// MCU_ADDRESS
{0x0990, 0x007F}, 	// MCU_DATA_0
{0x098C, 0x2417},	// MCU_ADDRESS
{0x0990, 0x0098}, 	// MCU_DATA_0
{0x098C, 0xA404},	// MCU_ADDRESS
{0x0990, 0x0010},	// MCU_DATA_0
{0x098C, 0xA40D},	// MCU_ADDRESS
{0x0990, 0x0002},	// MCU_DATA_0
{0x098C, 0xA40E},	// MCU_ADDRESS
{0x0990, 0x0003},	// MCU_DATA_0
{0x098C, 0xA410}, 	// MCU_ADDRESS
{0x0990, 0x000A},	// MCU_DATA_0
{0x098C, 0xA103}, 	// MCU_ADDRESS
{0x0990, 0x0006},	// MCU_DATA_0
{0xFFFF, 0x0064},	// DELAY=100
{0x3210, 0x09B0},	// COLOR_PIPELINE_CONTROL
{0x364E, 0x0250},	// P_GR_P0Q0
{0x3650, 0x6C4C},	// P_GR_P0Q1
{0x3652, 0x28B3},	// P_GR_P0Q2
{0x3654, 0x57F0}, 	// P_GR_P0Q3
{0x3656, 0xC5F4},	// P_GR_P0Q4
{0x3658, 0x00B0},	// P_RD_P0Q0
{0x365A, 0x1E4C},	// P_RD_P0Q1
{0x365C, 0x3013},	// P_RD_P0Q2
{0x365E, 0x25B0},	// P_RD_P0Q3
{0x3660, 0x94B4},	// P_RD_P0Q4
{0x3662, 0x0010},	// P_BL_P0Q0
{0x3664, 0x720C}, 	// P_BL_P0Q1
{0x3666, 0x1593},	// P_BL_P0Q2
{0x3668, 0x3970}, 	// P_BL_P0Q3
{0x366A, 0xB714},	// P_BL_P0Q4
{0x366C, 0x0070},	// P_GB_P0Q0
{0x366E, 0x504A},	// P_GB_P0Q1
{0x3670, 0x2753},	// P_GB_P0Q2
{0x3672, 0x5F70},	// P_GB_P0Q3
{0x3674, 0xB814},	// P_GB_P0Q4
{0x3676, 0x8D8F}, 	// P_GR_P1Q0
{0x3678, 0xA04F},	// P_GR_P1Q1
{0x367A, 0xA5B0}, 	// P_GR_P1Q2
{0x367C, 0x36F1},	// P_GR_P1Q3
{0x367E, 0x35F4},	// P_GR_P1Q4
{0x3680, 0xCBEE},	// P_RD_P1Q0
{0x3682, 0x8C6F},	// P_RD_P1Q1
{0x3684, 0x8830},	// P_RD_P1Q2
{0x3686, 0x7011},	// P_RD_P1Q3
{0x3688, 0x6234}, 	// P_RD_P1Q4
{0x368A, 0xEE6D},	// P_BL_P1Q0
{0x368C, 0xD62F}, 	// P_BL_P1Q1
{0x368E, 0xDC70},	// P_BL_P1Q2
{0x3690, 0x2993},	// P_BL_P1Q3
{0x3692, 0x2E94},	// P_BL_P1Q4
{0x3694, 0x800F},	// P_GB_P1Q0
{0x3696, 0x892F},	// P_GB_P1Q1
{0x3698, 0xA20E},	// P_GB_P1Q2
{0x369A, 0x45CF}, 	// P_GB_P1Q3
{0x369C, 0x6D72},	// P_GB_P1Q4
{0x369E, 0x0594},	// P_GR_P2Q0
{0x36A0, 0x2EF1},	// P_GR_P2Q1
{0x36A2, 0xCA14},	// P_GR_P2Q2
{0x36A4, 0x8116},	// P_GR_P2Q3
{0x36A6, 0x9CD8},	// P_GR_P2Q4
{0x36A8, 0x0B54},	// P_RD_P2Q0
{0x36AA, 0x3791}, 	// P_RD_P2Q1
{0x36AC, 0xFECF},	// P_RD_P2Q2
{0x36AE, 0xA2F6}, 	// P_RD_P2Q3
{0x36B0, 0xDA38},	// P_RD_P2Q4
{0x36B2, 0x6153},	// P_BL_P2Q0
{0x36B4, 0x48F1},	// P_BL_P2Q1
{0x36B6, 0xA9D4},	// P_BL_P2Q2
{0x36B8, 0x8D56},	// P_BL_P2Q3
{0x36BA, 0xAEF8},	// P_BL_P2Q4
{0x36BC, 0x01B4}, 	// P_GB_P2Q0
{0x36BE, 0x7971},	// P_GB_P2Q1
{0x36C0, 0xACD4}, 	// P_GB_P2Q2
{0x36C2, 0xCA76},	// P_GB_P2Q3
{0x36C4, 0xAF18},	// P_GB_P2Q4
{0x36C6, 0xA293},	// P_GR_P3Q0
{0x36C8, 0x2D72},	// P_GR_P3Q1
{0x36CA, 0x7F76},	// P_GR_P3Q2
{0x36CC, 0x9C56},	// P_GR_P3Q3
{0x36CE, 0xB835}, 	// P_GR_P3Q4
{0x36D0, 0x8D13},	// P_RD_P3Q0
{0x36D2, 0x29F3}, 	// P_RD_P3Q1
{0x36D4, 0x04B7},	// P_RD_P3Q2
{0x36D6, 0xBFB7},	// P_RD_P3Q3
{0x36D8, 0x9636},	// P_RD_P3Q4
{0x36DA, 0xD412},	// P_BL_P3Q0
{0x36DC, 0x2973},	// P_BL_P3Q1
{0x36DE, 0x4E56},	// P_BL_P3Q2
{0x36E0, 0xC757}, 	// P_BL_P3Q3
{0x36E2, 0x5336},	// P_BL_P3Q4
{0x36E4, 0xB833},	// P_GB_P3Q0
{0x36E6, 0x1891},	// P_GB_P3Q1
{0x36E8, 0x5ED6},	// P_GB_P3Q2
{0x36EA, 0xB395},	// P_GB_P3Q3
{0x36EC, 0x1959},	// P_GB_P3Q4
{0x36EE, 0xA8D5},	// P_GR_P4Q0
{0x36F0, 0x85D4}, 	// P_GR_P4Q1
{0x36F2, 0xC6B9},	// P_GR_P4Q2
{0x36F4, 0x81B9}, 	// P_GR_P4Q3
{0x36F6, 0x1EDC},	// P_GR_P4Q4
{0x36F8, 0x9F95},	// P_RD_P4Q0
{0x36FA, 0xD175},	// P_RD_P4Q1
{0x36FC, 0xEF79},	// P_RD_P4Q2
{0x36FE, 0x7117},	// P_RD_P4Q3
{0x3700, 0x575C},	// P_RD_P4Q4
{0x3702, 0xA2D5}, 	// P_BL_P4Q0
{0x3704, 0x97D5},	// P_BL_P4Q1
{0x3706, 0x89B9}, 	// P_BL_P4Q2
{0x3708, 0x6DB6},	// P_BL_P4Q3
{0x370A, 0x52FB},	// P_BL_P4Q4
{0x370C, 0x9015},	// P_GB_P4Q0
{0x370E, 0xB275},	// P_GB_P4Q1
{0x3710, 0xBED9},	// P_GB_P4Q2
{0x3712, 0x32F8},	// P_GB_P4Q3
{0x3714, 0x079C}, 	// P_GB_P4Q4
{0x3644, 0x0144},	// POLY_ORIGIN_C
{0x3642, 0x00E0}, 	// POLY_ORIGIN_R
{0x3210, 0x09B8},	// COLOR_PIPELINE_CONTROL
{0x098C, 0x2306},	// MCU_ADDRESS [AWB_CCM_L_0]
{0x0990, 0x0194},	// MCU_DATA_0
{0x098C, 0x2308},	// MCU_ADDRESS [AWB_CCM_L_1]
{0x0990, 0xFFA2},	// MCU_DATA_0
{0x098C, 0x230A},	// MCU_ADDRESS [AWB_CCM_L_2]
{0x0990, 0xFFCA}, 	// MCU_DATA_0
{0x098C, 0x230C},	// MCU_ADDRESS [AWB_CCM_L_3]
{0x0990, 0xFF8C},	// MCU_DATA_0
{0x098C, 0x230E},	// MCU_ADDRESS [AWB_CCM_L_4]
{0x0990, 0x016B},	// MCU_DATA_0
{0x098C, 0x2310},	// MCU_ADDRESS [AWB_CCM_L_5]
{0x0990, 0x0009},	// MCU_DATA_0
{0x098C, 0x2312},	// MCU_ADDRESS [AWB_CCM_L_6]
{0x0990, 0xFF90}, 	// MCU_DATA_0
{0x098C, 0x2314},	// MCU_ADDRESS [AWB_CCM_L_7]
{0x0990, 0xFE85}, 	// MCU_DATA_0
{0x098C, 0x2316},	// MCU_ADDRESS [AWB_CCM_L_8]
{0x0990, 0x02EB},	// MCU_DATA_0
{0x098C, 0x231C},	// MCU_ADDRESS [AWB_CCM_RL_0]
{0x0990, 0x0038},	// MCU_DATA_0
{0x098C, 0x231E},	// MCU_ADDRESS [AWB_CCM_RL_1]
{0x0990, 0xFF52},	// MCU_DATA_0
{0x098C, 0x2320}, 	// MCU_ADDRESS [AWB_CCM_RL_2]
{0x0990, 0x0076},	// MCU_DATA_0
{0x098C, 0x2322}, 	// MCU_ADDRESS [AWB_CCM_RL_3]
{0x0990, 0x0030},	// MCU_DATA_0
{0x098C, 0x2324},	// MCU_ADDRESS [AWB_CCM_RL_4]
{0x0990, 0x001F},	// MCU_DATA_0
{0x098C, 0x2326},	// MCU_ADDRESS [AWB_CCM_RL_5]
{0x0990, 0xFFB1},	// MCU_DATA_0
{0x098C, 0x2328},	// MCU_ADDRESS [AWB_CCM_RL_6]
{0x0990, 0x0063}, 	// MCU_DATA_0
{0x098C, 0x232A},	// MCU_ADDRESS [AWB_CCM_RL_7]
{0x0990, 0x00E2}, 	// MCU_DATA_0
{0x098C, 0x232C},	// MCU_ADDRESS [AWB_CCM_RL_8]
{0x0990, 0xFEBB},	// MCU_DATA_0
{0x098C, 0x2318},	// MCU_ADDRESS [AWB_CCM_L_9]
{0x0990, 0x0013},	// MCU_DATA_0
{0x098C, 0x231A},	// MCU_ADDRESS [AWB_CCM_L_10]
{0x0990, 0x003F},	// MCU_DATA_0
{0x098C, 0x232E}, 	// MCU_ADDRESS [AWB_CCM_RL_9]
{0x0990, 0x000F},	// MCU_DATA_0
{0x098C, 0x2330},	// MCU_ADDRESS [AWB_CCM_RL_10]
{0x0990, 0xFFF5},	// MCU_DATA_0
{0x098C, 0xA34A},	// MCU_ADDRESS [AWB_GAIN_MIN]
{0x0990, 0x0095},	// MCU_DATA_0
{0x098C, 0xA34B},	// MCU_ADDRESS [AWB_GAIN_MAX]
{0x0990, 0x00FB},	// MCU_DATA_0
{0x098C, 0xA34C}, 	// MCU_ADDRESS [AWB_GAINMIN_B]
{0x0990, 0x0039},	// MCU_DATA_0
{0x098C, 0xA34D}, 	// MCU_ADDRESS [AWB_GAINMAX_B]
{0x0990, 0x0075},	// MCU_DATA_0
{0x098C, 0xA302},	// MCU_ADDRESS [AWB_WINDOW_POS]
{0x0990, 0x0000},	// MCU_DATA_0
{0x098C, 0xA303},	// MCU_ADDRESS [AWB_WINDOW_SIZE]
{0x0990, 0x00CF},	// MCU_DATA_0
{0x098C, 0xA351},	// MCU_ADDRESS [AWB_CCM_POSITION_MIN]
{0x0990, 0x0000}, 	// MCU_DATA_0
{0x098C, 0xA352},	// MCU_ADDRESS [AWB_CCM_POSITION_MAX]
{0x0990, 0x007F}, 	// MCU_DATA_0
{0x098C, 0xA355},	// MCU_ADDRESS [AWB_MODE]
{0x0990, 0x000A},	// MCU_DATA_0
{0x098C, 0xA35F},	// MCU_ADDRESS [AWB_STEADY_BGAIN_IN_MIN]
{0x0990, 0x007E},	// MCU_DATA_0
{0x098C, 0xA360},	// MCU_ADDRESS [AWB_STEADY_BGAIN_IN_MAX]
{0x0990, 0x0082},	// MCU_DATA_0
{0x098C, 0x2361}, 	// MCU_ADDRESS [AWB_CNT_PXL_TH]
{0x0990, 0x0140},	// MCU_DATA_0
{0x098C, 0xA353}, 	// MCU_ADDRESS [AWB_CCM_POSITION]
{0x0990, 0x007F},	// MCU_DATA_0
{0x098C, 0xA34E},	// MCU_ADDRESS [AWB_GAIN_R]
{0x0990, 0x00A2},	// MCU_DATA_0
{0x098C, 0xA34F},	// MCU_ADDRESS [AWB_GAIN_G]
{0x0990, 0x0080},	// MCU_DATA_0
{0x098C, 0xA350},	// MCU_ADDRESS [AWB_GAIN_B]
{0x0990, 0x0070}, 	// MCU_DATA_0
{0x098C, 0xA363},	// MCU_ADDRESS [AWB_TG_MIN0]
{0x0990, 0x00E2},	// MCU_DATA_0
{0x098C, 0xA364},	// MCU_ADDRESS [AWB_TG_MAX0]
{0x0990, 0x00EA},	// MCU_DATA_0
{0x098C, 0xA366},	// MCU_ADDRESS [AWB_KR_L]
{0x0990, 0x0098},	// MCU_DATA_0
{0x098C, 0xA367},	// MCU_ADDRESS [AWB_KG_L]
{0x0990, 0x008D}, 	// MCU_DATA_0
{0x098C, 0xA368},	// MCU_ADDRESS [AWB_KB_L]
{0x0990, 0x007E}, 	// MCU_DATA_0
{0x098C, 0xA369},	// MCU_ADDRESS [AWB_KR_R]
{0x0990, 0x0080},	// MCU_DATA_0
{0x098C, 0xA36A},	// MCU_ADDRESS [AWB_KG_R]
{0x0990, 0x0080},	// MCU_DATA_0
{0x098C, 0xA36B},	// MCU_ADDRESS [AWB_KB_R]
{0x0990, 0x0080},	// MCU_DATA_0
{0x3240, 0xF23C}, 	// LUM_LIMITS_WB_STATS
{0x3244, 0x031A},	// AWB_CONFIG4
{0x098C, 0x275F}, 	// MCU_ADDRESS [MODE_COMMONMODESETTINGS_BRIGHT_COLOR_KILL]
{0x0990, 0x0594},	// MCU_DATA_0
{0x098C, 0x2761},	// MCU_ADDRESS [MODE_COMMONMODESETTINGS_DARK_COLOR_KILL]
{0x0990, 0x00AA},	// MCU_DATA_0
{0x098C, 0xA117},	// MCU_ADDRESS [SEQ_PREVIEW_0_AE]
{0x0990, 0x0001},	// MCU_DATA_0
{0x098C, 0xA119},	// MCU_ADDRESS [SEQ_PREVIEW_0_AWB]
{0x0990, 0x0000}, 	// MCU_DATA_0
{0x098C, 0xA11A},	// MCU_ADDRESS [SEQ_PREVIEW_0_HG]
{0x0990, 0x0000}, 	// MCU_DATA_0
{0x098C, 0xA11D},	// MCU_ADDRESS [SEQ_PREVIEW_1_AE]
{0x0990, 0x0002},	// MCU_DATA_0
{0x098C, 0xA11E},	// MCU_ADDRESS [SEQ_PREVIEW_1_FD]
{0x0990, 0x0001},	// MCU_DATA_0
{0x098C, 0xA11F},	// MCU_ADDRESS [SEQ_PREVIEW_1_AWB]
{0x0990, 0x0001},	// MCU_DATA_0
{0x098C, 0xA120}, 	// MCU_ADDRESS [SEQ_PREVIEW_1_HG]
{0x0990, 0x0001},	// MCU_DATA_0
{0x098C, 0xA129},	// MCU_ADDRESS [SEQ_PREVIEW_3_AE]
{0x0990, 0x0002},	// MCU_DATA_0
{0x098C, 0xA20C},	// MCU_ADDRESS [AE_MAX_INDEX]
{0x0990, 0x000F},	// MCU_DATA_0
{0x098C, 0xA215},	// MCU_ADDRESS [AE_INDEX_TH23]
{0x0990, 0x0008},	// MCU_DATA_0
{0x098C, 0xA216}, 	// MCU_ADDRESS [AE_MAXGAIN23]
{0x0990, 0x0040},	// MCU_DATA_0
{0x098C, 0xA228}, 	// MCU_ADDRESS [AE_GAINR12]
{0x0990, 0x0080},	// MCU_DATA_0
{0x098C, 0xA20D},	// MCU_ADDRESS [AE_MIN_VIRTGAIN]
{0x0990, 0x0018},	// MCU_DATA_0
{0x098C, 0xA20E},	// MCU_ADDRESS [AE_MAX_VIRTGAIN]
{0x0990, 0x0080},	// MCU_DATA_0
{0x098C, 0x2212},	// MCU_ADDRESS [AE_MAX_DGAIN_AE1]
{0x0990, 0x00C0}, 	// MCU_DATA_0
{0x098C, 0xA244},	// MCU_ADDRESS [AE_DRTFEATURECTRL]
{0x0990, 0x0008}, 	// MCU_DATA_0
{0x098C, 0xA24F},	// MCU_ADDRESS [AE_BASETARGET]
{0x0990, 0x0038},	// MCU_DATA_0
{0x098C, 0xA207},	// MCU_ADDRESS [AE_GATE]
{0x0990, 0x0007},	// MCU_DATA_0
{0x098C, 0xA202},	// MCU_ADDRESS [AE_WINDOW_POS]
{0x0990, 0x0034},	// MCU_DATA_0
{0x098C, 0xA203}, 	// MCU_ADDRESS [AE_WINDOW_SIZE]
{0x0990, 0x0097},	// MCU_DATA_0
{0x098C, 0xA208}, 	// MCU_ADDRESS [AE_SKIP_FRAMES]
{0x0990, 0x0003},	// MCU_DATA_0
{0x098C, 0xAB1F},	// MCU_ADDRESS [HG_LLMODE]
{0x0990, 0x00C7},	// MCU_DATA_0
{0x098C, 0x274F},	// MCU_ADDRESS [MODE_DEC_CTRL_B]
{0x0990, 0x0004},	// MCU_DATA_0
{0x098C, 0x2741},	// MCU_ADDRESS [MODE_DEC_CTRL_A]
{0x0990, 0x0004}, 	// MCU_DATA_0
{0x098C, 0xAB20},	// MCU_ADDRESS [HG_LL_SAT1]
{0x0990, 0x0088},	// MCU_DATA_0
{0x098C, 0xAB21},	// MCU_ADDRESS [HG_LL_INTERPTHRESH1]
{0x0990, 0x0022},	// MCU_DATA_0
{0x098C, 0xAB22},	// MCU_ADDRESS [HG_LL_APCORR1]
{0x0990, 0x0003},	// MCU_DATA_0
{0x098C, 0xAB23},	// MCU_ADDRESS [HG_LL_APTHRESH1]
{0x0990, 0x000B}, 	// MCU_DATA_0
{0x098C, 0xAB24},	// MCU_ADDRESS [HG_LL_SAT2]
{0x0990, 0x0041}, 	// MCU_DATA_0
{0x098C, 0xAB25},	// MCU_ADDRESS [HG_LL_INTERPTHRESH2]
{0x0990, 0x0030},	// MCU_DATA_0
{0x098C, 0xAB26},	// MCU_ADDRESS [HG_LL_APCORR2]
{0x0990, 0x0001},	// MCU_DATA_0
{0x098C, 0xAB27},	// MCU_ADDRESS [HG_LL_APTHRESH2]
{0x0990, 0x0015},	// MCU_DATA_0
{0x098C, 0x2B28}, 	// MCU_ADDRESS [HG_LL_BRIGHTNESSSTART]
{0x0990, 0x1520},	// MCU_DATA_0
{0x098C, 0x2B2A}, 	// MCU_ADDRESS [HG_LL_BRIGHTNESSSTOP]
{0x0990, 0x2000},	// MCU_DATA_0
{0x098C, 0xAB2C},	// MCU_ADDRESS [HG_NR_START_R]
{0x0990, 0x0012},	// MCU_DATA_0
{0x098C, 0xAB2D},	// MCU_ADDRESS [HG_NR_START_G]
{0x0990, 0x0014},	// MCU_DATA_0
{0x098C, 0xAB2E},	// MCU_ADDRESS [HG_NR_START_B]
{0x0990, 0x0012}, 	// MCU_DATA_0
{0x098C, 0xAB2F},	// MCU_ADDRESS [HG_NR_START_OL]
{0x0990, 0x0014}, 	// MCU_DATA_0
{0x098C, 0xAB30},	// MCU_ADDRESS [HG_NR_STOP_R]
{0x0990, 0x001F},	// MCU_DATA_0
{0x098C, 0xAB31},	// MCU_ADDRESS [HG_NR_STOP_G]
{0x0990, 0x001C},	// MCU_DATA_0
{0x098C, 0xAB32},	// MCU_ADDRESS [HG_NR_STOP_B]
{0x0990, 0x001F},	// MCU_DATA_0
{0x098C, 0xAB33}, 	// MCU_ADDRESS [HG_NR_STOP_OL]
{0x0990, 0x001C},	// MCU_DATA_0
{0x098C, 0xAB34},	// MCU_ADDRESS [HG_NR_GAINSTART]
{0x0990, 0x0020},	// MCU_DATA_0
{0x098C, 0xAB35},	// MCU_ADDRESS [HG_NR_GAINSTOP]
{0x0990, 0x0080},	// MCU_DATA_0
{0x098C, 0xAB04},	// MCU_ADDRESS [HG_MAX_DLEVEL]
{0x0990, 0x0010},	// MCU_DATA_0
{0x098C, 0xAB06}, 	// MCU_ADDRESS [HG_PERCENT]
{0x0990, 0x000A},	// MCU_DATA_0
{0x326C, 0x1605}, 	// APERTURE_PARAMETERS
{0x098C, 0xAB37},	// MCU_ADDRESS [HG_GAMMA_MORPH_CTRL]
{0x0990, 0x0003},	// MCU_DATA_0
{0x098C, 0x2B38},	// MCU_ADDRESS [HG_GAMMASTARTMORPH]
{0x0990, 0x1F00},	// MCU_DATA_0
{0x098C, 0x2B3A},	// MCU_ADDRESS [HG_GAMMASTOPMORPH]
{0x0990, 0x2200},	// MCU_DATA_0
{0x098C, 0xAB3C}, 	// MCU_ADDRESS [HG_GAMMA_TABLE_A_0]
{0x0990, 0x0005},	// MCU_DATA_0
{0x098C, 0xAB3D}, 	// MCU_ADDRESS [HG_GAMMA_TABLE_A_1]
{0x0990, 0x000D},	// MCU_DATA_0
{0x098C, 0xAB3E},	// MCU_ADDRESS [HG_GAMMA_TABLE_A_2]
{0x0990, 0x0027},	// MCU_DATA_0
{0x098C, 0xAB3F},	// MCU_ADDRESS [HG_GAMMA_TABLE_A_3]
{0x0990, 0x0048},	// MCU_DATA_0
{0x098C, 0xAB40},	// MCU_ADDRESS [HG_GAMMA_TABLE_A_4]
{0x0990, 0x0070}, 	// MCU_DATA_0
{0x098C, 0xAB41},	// MCU_ADDRESS [HG_GAMMA_TABLE_A_5]
{0x0990, 0x008E}, 	// MCU_DATA_0
{0x098C, 0xAB42},	// MCU_ADDRESS [HG_GAMMA_TABLE_A_6]
{0x0990, 0x00A4},	// MCU_DATA_0
{0x098C, 0xAB43},	// MCU_ADDRESS [HG_GAMMA_TABLE_A_7]
{0x0990, 0x00B4},	// MCU_DATA_0
{0x098C, 0xAB44},	// MCU_ADDRESS [HG_GAMMA_TABLE_A_8]
{0x0990, 0x00C1},	// MCU_DATA_0
{0x098C, 0xAB45}, 	// MCU_ADDRESS [HG_GAMMA_TABLE_A_9]
{0x0990, 0x00CC},	// MCU_DATA_0
{0x098C, 0xAB46},	// MCU_ADDRESS [HG_GAMMA_TABLE_A_10]
{0x0990, 0x00D5},	// MCU_DATA_0
{0x098C, 0xAB47},	// MCU_ADDRESS [HG_GAMMA_TABLE_A_11]
{0x0990, 0x00DC},	// MCU_DATA_0
{0x098C, 0xAB48},	// MCU_ADDRESS [HG_GAMMA_TABLE_A_12]
{0x0990, 0x00E3},	// MCU_DATA_0
{0x098C, 0xAB49}, 	// MCU_ADDRESS [HG_GAMMA_TABLE_A_13]
{0x0990, 0x00E9},	// MCU_DATA_0
{0x098C, 0xAB4A}, 	// MCU_ADDRESS [HG_GAMMA_TABLE_A_14]
{0x0990, 0x00EE},	// MCU_DATA_0
{0x098C, 0xAB4B},	// MCU_ADDRESS [HG_GAMMA_TABLE_A_15]
{0x0990, 0x00F3},	// MCU_DATA_0
{0x098C, 0xAB4C},	// MCU_ADDRESS [HG_GAMMA_TABLE_A_16]
{0x0990, 0x00F7},	// MCU_DATA_0
{0x098C, 0xAB4D},	// MCU_ADDRESS [HG_GAMMA_TABLE_A_17]
{0x0990, 0x00FB}, 	// MCU_DATA_0
{0x098C, 0xAB4E},	// MCU_ADDRESS [HG_GAMMA_TABLE_A_18]
{0x0990, 0x00FF}, 	// MCU_DATA_0
{0x098C, 0xAB4F},	// MCU_ADDRESS [HG_GAMMA_TABLE_B_0]
{0x0990, 0x0012},	// MCU_DATA_0
{0x098C, 0xAB50},	// MCU_ADDRESS [HG_GAMMA_TABLE_B_1]
{0x0990, 0x0012},	// MCU_DATA_0
{0x098C, 0xAB51},	// MCU_ADDRESS [HG_GAMMA_TABLE_B_2]
{0x0990, 0x002B},	// MCU_DATA_0
{0x098C, 0xAB52}, 	// MCU_ADDRESS [HG_GAMMA_TABLE_B_3]
{0x0990, 0x004C},	// MCU_DATA_0
{0x098C, 0xAB53}, 	// MCU_ADDRESS [HG_GAMMA_TABLE_B_4]
{0x0990, 0x0072},	// MCU_DATA_0
{0x098C, 0xAB54},	// MCU_ADDRESS [HG_GAMMA_TABLE_B_5]
{0x0990, 0x008D},	// MCU_DATA_0
{0x098C, 0xAB55},	// MCU_ADDRESS [HG_GAMMA_TABLE_B_6]
{0x0990, 0x00A1},	// MCU_DATA_0
{0x098C, 0xAB56},	// MCU_ADDRESS [HG_GAMMA_TABLE_B_7]
{0x0990, 0x00B0}, 	// MCU_DATA_0
{0x098C, 0xAB57},	// MCU_ADDRESS [HG_GAMMA_TABLE_B_8]
{0x0990, 0x00BC},	// MCU_DATA_0
{0x098C, 0xAB58},	// MCU_ADDRESS [HG_GAMMA_TABLE_B_9]
{0x0990, 0x00C7},	// MCU_DATA_0
{0x098C, 0xAB59},	// MCU_ADDRESS [HG_GAMMA_TABLE_B_10]
{0x0990, 0x00D0},	// MCU_DATA_0
{0x098C, 0xAB5A},	// MCU_ADDRESS [HG_GAMMA_TABLE_B_11]
{0x0990, 0x00D8}, 	// MCU_DATA_0
{0x098C, 0xAB5B},	// MCU_ADDRESS [HG_GAMMA_TABLE_B_12]
{0x0990, 0x00DF},	// MCU_DATA_0
{0x098C, 0xAB5C},	// MCU_ADDRESS [HG_GAMMA_TABLE_B_13]
{0x0990, 0x00E6},	// MCU_DATA_0
{0x098C, 0xAB5D},	// MCU_ADDRESS [HG_GAMMA_TABLE_B_14]
{0x0990, 0x00EC},	// MCU_DATA_0
{0x098C, 0xAB5E},	// MCU_ADDRESS [HG_GAMMA_TABLE_B_15]
{0x0990, 0x00F1}, 	// MCU_DATA_0
{0x098C, 0xAB5F},	// MCU_ADDRESS [HG_GAMMA_TABLE_B_16]
{0x0990, 0x00F6},	// MCU_DATA_0
{0x098C, 0xAB60},	// MCU_ADDRESS [HG_GAMMA_TABLE_B_17]
{0x0990, 0x00FB},	// MCU_DATA_0
{0x098C, 0xAB61},	// MCU_ADDRESS [HG_GAMMA_TABLE_B_18]
{0x0990, 0x00FF},	// MCU_DATA_0
{0x322A, 0x0004},	// DECIMATOR_CONTROL
{0x098C, 0xAB1F}, 	// MCU_ADDRESS [HG_LLMODE]
{0x0990, 0x00C7},	// MCU_DATA_0
{0x301A, 0x021C},	// RESET_REGISTER
{0x098C, 0xA103},	// MCU_ADDRESS
{0x0990, 0x0006},	// MCU_DATA_0
{0xFFFF, 0x012C},	// DELAY=300
{0x098C, 0xA103},	// MCU_ADDRESS
{0x0990, 0x0005},	// MCU_DATA_0
{0xFFFF, 0x012C},	// DELAY=300
#endif

CAMACQ_SUB_BTM_OF_DATA
}
#endif /* _CAMACQ_API_C_ */
;

GLOBAL const U16 reg_sub_fmt_jpg[][2]
#if defined(_CAMACQ_API_C_)
={

CAMACQ_SUB_BTM_OF_DATA
}
#endif /* _CAMACQ_API_C_ */
;

GLOBAL const U16 reg_sub_fmt_yuv422[][2]
#if defined(_CAMACQ_API_C_)
={

CAMACQ_SUB_BTM_OF_DATA
}
#endif /* _CAMACQ_API_C_ */
;

//==========================================================
// CAMERA PREVIEW(640*480 / FULL)
//==========================================================
GLOBAL const U16 reg_sub_preview[][2]
#if defined(_CAMACQ_API_C_)
={

CAMACQ_SUB_BTM_OF_DATA
}
#endif /* _CAMACQ_API_C_ */
;

//==========================================================
// SNAPSHOT
//==========================================================

GLOBAL const U16 reg_sub_snapshot[][2]
#if defined(_CAMACQ_API_C_)
={

CAMACQ_SUB_BTM_OF_DATA
}
#endif /* _CAMACQ_API_C_ */
;

//==========================================================
// Camcorder ON
//==========================================================

GLOBAL const U16 reg_sub_camcorder_on[][2]
#if defined(_CAMACQ_API_C_)
={

CAMACQ_SUB_BTM_OF_DATA
}
#endif /* _CAMACQ_API_C_ */
;


//==========================================================
// MIDLIGHT SNAPSHOT =======> Flash Low light snapshot
//==========================================================
GLOBAL const U16 reg_sub_midlight[][2]
#if defined(_CAMACQ_API_C_)
={

CAMACQ_SUB_BTM_OF_DATA
}
#endif /* _CAMACQ_API_C_ */
;

//==========================================================
// LOWLIGHT SNAPSHOT
//==========================================================
GLOBAL const U16 reg_sub_lowlight[][2]
#if defined(_CAMACQ_API_C_)
={

CAMACQ_SUB_BTM_OF_DATA
}
#endif /* _CAMACQ_API_C_ */
;



/*****************************************************************/
/*********************** N I G H T  M O D E **********************/
/*****************************************************************/

//==========================================================
// CAMERA NIGHTMODE ON
//==========================================================
GLOBAL const U16 reg_sub_nightshot_on[][2]
#if defined(_CAMACQ_API_C_)
={

CAMACQ_SUB_BTM_OF_DATA
}
#endif /* _CAMACQ_API_C_ */
;

//==========================================================
// CAMERA NIGHTMODE OFF
//==========================================================
GLOBAL const U16 reg_sub_nightshot_off[][2]
#if defined(_CAMACQ_API_C_)
={

CAMACQ_SUB_BTM_OF_DATA
}
#endif /* _CAMACQ_API_C_ */
;

//==========================================================
// NIGHT-MODE SNAPSHOT
//==========================================================
GLOBAL const U16 reg_sub_nightshot[][2]
#if defined(_CAMACQ_API_C_)
={

CAMACQ_SUB_BTM_OF_DATA
}
#endif /* _CAMACQ_API_C_ */
;

//==========================================================
// CAMERA_WB_AUTO (1/10)
//==========================================================
GLOBAL const U16 reg_sub_wb_auto[][2]
#if defined(_CAMACQ_API_C_)
={
0xFCFCD000,
0x00287000,
               // 0x002A04E6 S/W Program
               // 0x0F12077F

CAMACQ_SUB_BTM_OF_DATA
}
#endif /* _CAMACQ_API_C_ */
;

//==========================================================
// CAMERA_WB_DAYLIGHT (2/10)
//==========================================================
GLOBAL const U16 reg_sub_wb_daylight[][2]
#if defined(_CAMACQ_API_C_)
={

CAMACQ_SUB_BTM_OF_DATA
}
#endif /* _CAMACQ_API_C_ */
;

//==========================================================
// CAMERA_WB_CLOUDY  (3/10)
//==========================================================
GLOBAL const U16 reg_sub_wb_cloudy[][2]
#if defined(_CAMACQ_API_C_)
={

CAMACQ_SUB_BTM_OF_DATA
}
#endif /* _CAMACQ_API_C_ */
;

//==========================================================
// CAMERA_WB_INCANDESCENT (4/10)
//==========================================================
GLOBAL const U16 reg_sub_wb_incandescent[][2]
#if defined(_CAMACQ_API_C_)
={

CAMACQ_SUB_BTM_OF_DATA
}
#endif /* _CAMACQ_API_C_ */
;

//==========================================================
// CAMERA_WB_FLUORESCENT (5/10)
//==========================================================
GLOBAL const U16 reg_sub_wb_fluorescent[][2]
#if defined(_CAMACQ_API_C_)
={

CAMACQ_SUB_BTM_OF_DATA
}
#endif /* _CAMACQ_API_C_ */
;

//==========================================================
// CAMERA_WB_FLUORESCENT (6/10)
//==========================================================
GLOBAL const U16 reg_sub_wb_twilight[][2]
#if defined(_CAMACQ_API_C_)
={

CAMACQ_SUB_BTM_OF_DATA
}
#endif /* _CAMACQ_API_C_ */
;

//==========================================================
// CAMERA_WB_FLUORESCENT (7/10)
//==========================================================
GLOBAL const U16 reg_sub_wb_tungsten[][2]
#if defined(_CAMACQ_API_C_)
={

CAMACQ_SUB_BTM_OF_DATA
}
#endif /* _CAMACQ_API_C_ */
;

//==========================================================
// CAMERA_WB_FLUORESCENT (8/10)
//==========================================================
GLOBAL const U16 reg_sub_wb_off[][2]
#if defined(_CAMACQ_API_C_)
={

CAMACQ_SUB_BTM_OF_DATA
}
#endif /* _CAMACQ_API_C_ */
;

//==========================================================
// CAMERA_WB_FLUORESCENT (9/10)
//==========================================================
GLOBAL const U16 reg_sub_wb_horizon[][2]
#if defined(_CAMACQ_API_C_)
={

CAMACQ_SUB_BTM_OF_DATA
}
#endif /* _CAMACQ_API_C_ */
;

//==========================================================
// CAMERA_WB_FLUORESCENT (10/10)
//==========================================================
GLOBAL const U16 reg_sub_wb_shade[][2]
#if defined(_CAMACQ_API_C_)
={

CAMACQ_SUB_BTM_OF_DATA
}
#endif /* _CAMACQ_API_C_ */
;


//==========================================================
// CAMERA_EFFECT_OFF (1/13)
//==========================================================
GLOBAL const U16 reg_sub_effect_none[][2]
#if defined(_CAMACQ_API_C_)
={

CAMACQ_SUB_BTM_OF_DATA
}
#endif /* _CAMACQ_API_C_ */
;

//==========================================================
// CAMERA_EFFECT_MONO (2/13)
//==========================================================
GLOBAL const U16 reg_sub_effect_gray[][2]
#if defined(_CAMACQ_API_C_)
={

CAMACQ_SUB_BTM_OF_DATA
}
#endif /* _CAMACQ_API_C_ */
;

//==========================================================
// CAMERA_EFFECT_NEGATIVE (3/13)
//==========================================================
GLOBAL const U16 reg_sub_effect_negative[][2]
#if defined(_CAMACQ_API_C_)
={
0xFCFCD000,
0x00287000,
0x002A023C,
0x0F120003,

CAMACQ_SUB_BTM_OF_DATA
}
#endif /* _CAMACQ_API_C_ */
;

//==========================================================
// CAMERA_EFFECT_SOLARIZE (4/13)
//==========================================================
GLOBAL const U16 reg_sub_effect_solarize[][2]
#if defined(_CAMACQ_API_C_)
={

CAMACQ_SUB_BTM_OF_DATA
}
#endif /* _CAMACQ_API_C_ */
;

//==========================================================
// CAMERA_EFFECT_SEPIA (5/13)
//==========================================================
GLOBAL const U16 reg_sub_effect_sepia[][2]
#if defined(_CAMACQ_API_C_)
={


CAMACQ_SUB_BTM_OF_DATA
}
#endif /* _CAMACQ_API_C_ */
;

//==========================================================
// CAMERA_EFFECT_POSTERIZE (6/13)
//==========================================================
GLOBAL const U16 reg_sub_effect_posterize[][2]
#if defined(_CAMACQ_API_C_)
={

CAMACQ_SUB_BTM_OF_DATA
}
#endif /* _CAMACQ_API_C_ */
;

//==========================================================
// CAMERA_EFFECT_WHITEBOARD (7/13)
//==========================================================
GLOBAL const U16 reg_sub_effect_whiteboard[][2]
#if defined(_CAMACQ_API_C_)
={

CAMACQ_SUB_BTM_OF_DATA
}
#endif /* _CAMACQ_API_C_ */
;

//==========================================================
// CAMERA_EFFECT_BLACKBOARD (8/13)
//==========================================================
GLOBAL const U16 reg_sub_effect_blackboard[][2]
#if defined(_CAMACQ_API_C_)
={

CAMACQ_SUB_BTM_OF_DATA
}
#endif /* _CAMACQ_API_C_ */
;

//==========================================================
// CAMERA_EFFECT_AQUA (9/13)
//==========================================================
GLOBAL const U16 reg_sub_effect_aqua[][2]
#if defined(_CAMACQ_API_C_)
={

CAMACQ_SUB_BTM_OF_DATA
}
#endif /* _CAMACQ_API_C_ */
;

//==========================================================
// CAMERA_EFFECT_SHARPEN (10/13)
//==========================================================
GLOBAL const U16 reg_sub_effect_sharpen[][2]
#if defined(_CAMACQ_API_C_)
={

CAMACQ_SUB_BTM_OF_DATA
}
#endif /* _CAMACQ_API_C_ */
;

//==========================================================
// CAMERA_EFFECT_AQUA (11/13)
//==========================================================
GLOBAL const U16 reg_sub_effect_vivid[][2]
#if defined(_CAMACQ_API_C_)
={

CAMACQ_SUB_BTM_OF_DATA
}
#endif /* _CAMACQ_API_C_ */
;

//==========================================================
// CAMERA_EFFECT_GREEN (12/13)
//==========================================================
GLOBAL const U16 reg_sub_effect_green[][2]
#if defined(_CAMACQ_API_C_)
={

CAMACQ_SUB_BTM_OF_DATA
}
#endif /* _CAMACQ_API_C_ */
;

//==========================================================
// CAMERA_EFFECT_SKETCH (13/13)
//==========================================================
GLOBAL const U16 reg_sub_effect_sketch[][2]
#if defined(_CAMACQ_API_C_)
={

CAMACQ_SUB_BTM_OF_DATA
}
#endif /* _CAMACQ_API_C_ */
;


//==========================================================
// CAMERA_AEC_MATRIX_METERING (2/2)
//==========================================================
GLOBAL const U16 reg_sub_meter_matrix[][2]
#if defined(_CAMACQ_API_C_)
={

CAMACQ_SUB_BTM_OF_DATA
}
#endif /* _CAMACQ_API_C_ */
;

//==========================================================
// CAMERA_AEC_CENTERWEIGHTED_METERING (2/2)
//==========================================================
GLOBAL const U16 reg_sub_meter_cw[][2]
#if defined(_CAMACQ_API_C_)
={

CAMACQ_SUB_BTM_OF_DATA
}
#endif /* _CAMACQ_API_C_ */
;

//==========================================================
// CAMERA_AEC_SPOT_METERING (1/2)
//==========================================================
GLOBAL const U16 reg_sub_meter_spot[][2]
#if defined(_CAMACQ_API_C_)
={


CAMACQ_SUB_BTM_OF_DATA
}
#endif /* _CAMACQ_API_C_ */
;

//==========================================================
// CAMERA_AEC_FRAME_AVERAGE (2/2)
//==========================================================
GLOBAL const U16 reg_sub_meter_frame[][2]
#if defined(_CAMACQ_API_C_)
={

CAMACQ_SUB_BTM_OF_DATA
}
#endif /* _CAMACQ_API_C_ */
;

//==========================================================
// CAMERA_FLIP_NONE (1/4)
//==========================================================
GLOBAL const U16 reg_sub_flip_none[][2]
#if defined(_CAMACQ_API_C_)
={

CAMACQ_SUB_BTM_OF_DATA
}
#endif /* _CAMACQ_API_C_ */
;

//==========================================================
// CAMERA_FLIP_VERTICAL (2/4)
//==========================================================
GLOBAL const U16 reg_sub_flip_water[][2]
#if defined(_CAMACQ_API_C_)
={

CAMACQ_SUB_BTM_OF_DATA
}
#endif /* _CAMACQ_API_C_ */
;

//==========================================================
// CAMERA_FLIP_HORIZ (3/4)
//==========================================================
GLOBAL const U16 reg_sub_flip_mirror[][2]
#if defined(_CAMACQ_API_C_)
={

CAMACQ_SUB_BTM_OF_DATA
}
#endif /* _CAMACQ_API_C_ */
;


//==========================================================
// CAMERA_FLIP_SYMMETRIC (4/4)
//==========================================================
GLOBAL const U16 reg_sub_flip_water_mirror[][2]
#if defined(_CAMACQ_API_C_)
={

CAMACQ_SUB_BTM_OF_DATA
}
#endif /* _CAMACQ_API_C_ */
;

//==========================================================
// CAMERA_FRAMERATE_5FPS
//==========================================================
GLOBAL const U16 reg_sub_fps_fixed_5[][2]
#if defined(_CAMACQ_API_C_)
={

CAMACQ_SUB_BTM_OF_DATA
}
#endif /* _CAMACQ_API_C_ */
;


//==========================================================
// CAMERA_FRAMERATE_7FPS
//==========================================================
GLOBAL const U16 reg_sub_fps_fixed_7[][2]
#if defined(_CAMACQ_API_C_)
={


CAMACQ_SUB_BTM_OF_DATA
}
#endif /* _CAMACQ_API_C_ */
;

//==========================================================
// CAMERA_FRAMERATE_10FPS
//==========================================================
GLOBAL const U16 reg_sub_fps_fixed_10[][2]
#if defined(_CAMACQ_API_C_)
={

CAMACQ_SUB_BTM_OF_DATA
}
#endif /* _CAMACQ_API_C_ */
;

//==========================================================
// CAMERA_FRAMERATE_15FPS
//==========================================================
GLOBAL const U16 reg_sub_fps_fixed_15[][2]
#if defined(_CAMACQ_API_C_)
={


CAMACQ_SUB_BTM_OF_DATA
}
#endif /* _CAMACQ_API_C_ */
;

//==========================================================
// CAMERA_FRAMERATE_20FPS
//==========================================================
GLOBAL const U16 reg_sub_fps_fixed_20[][2]
#if defined(_CAMACQ_API_C_)
={


CAMACQ_SUB_BTM_OF_DATA
}
#endif /* _CAMACQ_API_C_ */
;

//==========================================================
// CAMERA_FRAMERATE_25FPS
//==========================================================
GLOBAL const U16 reg_sub_fps_fixed_25[][2]
#if defined(_CAMACQ_API_C_)
={


CAMACQ_SUB_BTM_OF_DATA
}
#endif /* _CAMACQ_API_C_ */
;

//==========================================================
// CAMERA_FRAMERATE_30FPS
//==========================================================
GLOBAL const U16 reg_sub_fps_fixed_30[][2]
#if defined(_CAMACQ_API_C_)
={

CAMACQ_SUB_BTM_OF_DATA
}
#endif /* _CAMACQ_API_C_ */
;

//==========================================================
// CAMERA_FRAMERATE_AUTO_MAX15(5~15fps)
//==========================================================
GLOBAL const U16 reg_sub_fps_var_15[][2]
#if defined(_CAMACQ_API_C_)
={

CAMACQ_SUB_BTM_OF_DATA
}
#endif /* _CAMACQ_API_C_ */
;


//==========================================================
// CAMERA_BRIGHTNESS_LEVEL1 (1/9) : 
//==========================================================
GLOBAL const U16 reg_sub_brightness_level_0[][2]
#if defined(_CAMACQ_API_C_)
={

CAMACQ_SUB_BTM_OF_DATA
}
#endif /* _CAMACQ_API_C_ */
;

//==========================================================
// CAMERA_BRIGHTNESS_LEVEL2 (2/9)
//==========================================================
GLOBAL const U16 reg_sub_brightness_level_1[][2]
#if defined(_CAMACQ_API_C_)
={

CAMACQ_SUB_BTM_OF_DATA
}
#endif /* _CAMACQ_API_C_ */
;

//==========================================================
// CAMERA_BRIGHTNESS_LEVEL3 (3/9)
//==========================================================
GLOBAL const U16 reg_sub_brightness_level_2[][2]
#if defined(_CAMACQ_API_C_)
={

CAMACQ_SUB_BTM_OF_DATA
}
#endif /* _CAMACQ_API_C_ */
;

//==========================================================
// CAMERA_BRIGHTNESS_LEVEL4 (4/9)
//==========================================================
GLOBAL const U16 reg_sub_brightness_level_3[][2]
#if defined(_CAMACQ_API_C_)
={

CAMACQ_SUB_BTM_OF_DATA
}
#endif /* _CAMACQ_API_C_ */
;

//==========================================================
// CAMERA_BRIGHTNESS_LEVEL5 (5/9)
//==========================================================
GLOBAL const U16 reg_sub_brightness_level_4[][2]
#if defined(_CAMACQ_API_C_)
={

CAMACQ_SUB_BTM_OF_DATA
}
#endif /* _CAMACQ_API_C_ */
;

//==========================================================
// CAMERA_BRIGHTNESS_LEVEL6 (6/9)
//==========================================================
GLOBAL const U16 reg_sub_brightness_level_5[][2]
#if defined(_CAMACQ_API_C_)
={

CAMACQ_SUB_BTM_OF_DATA
}
#endif /* _CAMACQ_API_C_ */
;

//==========================================================
// CAMERA_BRIGHTNESS_LEVEL7 (7/9)
//==========================================================
GLOBAL const U16 reg_sub_brightness_level_6[][2]
#if defined(_CAMACQ_API_C_)
={

CAMACQ_SUB_BTM_OF_DATA
}
#endif /* _CAMACQ_API_C_ */
;

//==========================================================
// CAMERA_BRIGHTNESS_LEVEL8 (8/9)
//==========================================================
GLOBAL const U16 reg_sub_brightness_level_7[][2]
#if defined(_CAMACQ_API_C_)
={

CAMACQ_SUB_BTM_OF_DATA
}
#endif /* _CAMACQ_API_C_ */
;

//==========================================================
// CAMERA_BRIGHTNESS_LEVEL9 (9/9)
//==========================================================
GLOBAL const U16 reg_sub_brightness_level_8[][2]
#if defined(_CAMACQ_API_C_)
={

CAMACQ_SUB_BTM_OF_DATA
}
#endif /* _CAMACQ_API_C_ */
;

//==========================================================
// CAMERA_EXPOSURE_COMPENSATION_LEVEL1 (1/9) : 
//==========================================================
GLOBAL const U16 reg_sub_expcompensation_level_0[][2]
#if defined(_CAMACQ_API_C_)
={


CAMACQ_SUB_BTM_OF_DATA
}
#endif /* _CAMACQ_API_C_ */
;

//==========================================================
// CAMERA_EXPOSURE_COMPENSATION_LEVEL2 (2/9)
//==========================================================
GLOBAL const U16 reg_sub_expcompensation_level_1[][2]
#if defined(_CAMACQ_API_C_)
={


CAMACQ_SUB_BTM_OF_DATA
}
#endif /* _CAMACQ_API_C_ */
;

//==========================================================
// CAMERA_EXPOSURE_COMPENSATION_LEVEL3 (3/9)
//==========================================================
GLOBAL const U16 reg_sub_expcompensation_level_2[][2]
#if defined(_CAMACQ_API_C_)
={


CAMACQ_SUB_BTM_OF_DATA
}
#endif /* _CAMACQ_API_C_ */
;

//==========================================================
// CAMERA_EXPOSURE_COMPENSATION_LEVEL4 (4/9)
//==========================================================
GLOBAL const U16 reg_sub_expcompensation_level_3[][2]
#if defined(_CAMACQ_API_C_)
={


CAMACQ_SUB_BTM_OF_DATA
}
#endif /* _CAMACQ_API_C_ */
;

//==========================================================
// CAMERA_EXPOSURE_COMPENSATION_LEVEL5 (5/9)
//==========================================================
GLOBAL const U16 reg_sub_expcompensation_level_4[][2]
#if defined(_CAMACQ_API_C_)
={

CAMACQ_SUB_BTM_OF_DATA
}
#endif /* _CAMACQ_API_C_ */
;

//==========================================================
// CAMERA_EXPOSURE_COMPENSATION_LEVEL6 (6/9)
//==========================================================
GLOBAL const U16 reg_sub_expcompensation_level_5[][2]
#if defined(_CAMACQ_API_C_)
={


CAMACQ_SUB_BTM_OF_DATA
}
#endif /* _CAMACQ_API_C_ */
;

//==========================================================
// CAMERA_EXPOSURE_COMPENSATION_LEVEL7 (7/9)
//==========================================================
GLOBAL const U16 reg_sub_expcompensation_level_6[][2]
#if defined(_CAMACQ_API_C_)
={

CAMACQ_SUB_BTM_OF_DATA
}
#endif /* _CAMACQ_API_C_ */
;

//==========================================================
// CAMERA_EXPOSURE_COMPENSATION_LEVEL8 (8/9)
//==========================================================
GLOBAL const U16 reg_sub_expcompensation_level_7[][2]
#if defined(_CAMACQ_API_C_)
={

CAMACQ_SUB_BTM_OF_DATA
}
#endif /* _CAMACQ_API_C_ */
;

//==========================================================
// CAMERA_EXPOSURE_COMPENSATION_LEVEL9 (9/9)
//==========================================================
GLOBAL const U16 reg_sub_expcompensation_level_8[][2]
#if defined(_CAMACQ_API_C_)
={

CAMACQ_SUB_BTM_OF_DATA
}
#endif /* _CAMACQ_API_C_ */
;

//==========================================================
// CAMERA_AF
//==========================================================
GLOBAL const U16 reg_sub_reset_af [][2]
#if defined(_CAMACQ_API_C_)
={

CAMACQ_SUB_BTM_OF_DATA
}
#endif /* _CAMACQ_API_C_ */
;

GLOBAL const U16 reg_sub_cancel_macro_af [][2]
#if defined(_CAMACQ_API_C_)
={

CAMACQ_SUB_BTM_OF_DATA
}
#endif /* _CAMACQ_API_C_ */
;

GLOBAL const U16 reg_sub_cancel_manual_af [][2]
#if defined(_CAMACQ_API_C_)
={

CAMACQ_SUB_BTM_OF_DATA
}
#endif /* _CAMACQ_API_C_ */
;


GLOBAL const U16 reg_sub_set_af_nlux [][2]
#if defined(_CAMACQ_API_C_)
={

CAMACQ_SUB_BTM_OF_DATA
}
#endif /* _CAMACQ_API_C_ */
;

GLOBAL const U16 reg_sub_set_af_llux [][2]
#if defined(_CAMACQ_API_C_)
={

CAMACQ_SUB_BTM_OF_DATA
}
#endif /* _CAMACQ_API_C_ */
;

GLOBAL const U16 reg_sub_set_af[][2] // start_af
#if defined(_CAMACQ_API_C_)
={

CAMACQ_SUB_BTM_OF_DATA
}
#endif /* _CAMACQ_API_C_ */
;

GLOBAL const U16 reg_sub_off_af[][2]
#if defined(_CAMACQ_API_C_)
={

CAMACQ_SUB_BTM_OF_DATA
}
#endif /* _CAMACQ_API_C_ */
;


GLOBAL const U16 reg_sub_check_af[][2]
#if defined(_CAMACQ_API_C_)
={

CAMACQ_SUB_BTM_OF_DATA
}
#endif /* _CAMACQ_API_C_ */
;

GLOBAL const U16 reg_sub_manual_af[][2]
#if defined(_CAMACQ_API_C_)
={

CAMACQ_SUB_BTM_OF_DATA
}
#endif /* _CAMACQ_API_C_ */
;

GLOBAL const U16 reg_sub_macro_af[][2]
#if defined(_CAMACQ_API_C_)
={

CAMACQ_SUB_BTM_OF_DATA
}
#endif /* _CAMACQ_API_C_ */
;

GLOBAL const U16 reg_sub_return_manual_af[][2]
#if defined(_CAMACQ_API_C_)
={

CAMACQ_SUB_BTM_OF_DATA
}
#endif /* _CAMACQ_API_C_ */
;

GLOBAL const U16 reg_sub_return_macro_af[][2]
#if defined(_CAMACQ_API_C_)
={

CAMACQ_SUB_BTM_OF_DATA
}
#endif /* _CAMACQ_API_C_ */
;

GLOBAL const U16 reg_sub_set_af_normal_mode_1[][2]
#if defined(_CAMACQ_API_C_)
={

CAMACQ_SUB_BTM_OF_DATA
}
#endif /* _CAMACQ_API_C_ */
;

GLOBAL const U16 reg_sub_set_af_normal_mode_2[][2]
#if defined(_CAMACQ_API_C_)
={

CAMACQ_SUB_BTM_OF_DATA
}
#endif /* _CAMACQ_API_C_ */
;

GLOBAL const U16 reg_sub_set_af_normal_mode_3[][2]
#if defined(_CAMACQ_API_C_)
={

CAMACQ_SUB_BTM_OF_DATA
}
#endif /* _CAMACQ_API_C_ */
;

GLOBAL const U16 reg_sub_set_af_macro_mode_1[][2]
#if defined(_CAMACQ_API_C_)
={

CAMACQ_SUB_BTM_OF_DATA
}
#endif /* _CAMACQ_API_C_ */
;

GLOBAL const U16 reg_sub_set_af_macro_mode_2[][2]
#if defined(_CAMACQ_API_C_)
={

CAMACQ_SUB_BTM_OF_DATA
}
#endif /* _CAMACQ_API_C_ */
;

GLOBAL const U16 reg_sub_set_af_macro_mode_3[][2]
#if defined(_CAMACQ_API_C_)
={

CAMACQ_SUB_BTM_OF_DATA
}
#endif /* _CAMACQ_API_C_ */
;


//==========================================================
// CAMERA_ISO_AUTO
//==========================================================
GLOBAL const U16 reg_sub_iso_auto[][2]
#if defined(_CAMACQ_API_C_)
={
    

CAMACQ_SUB_BTM_OF_DATA
}
#endif /* _CAMACQ_API_C_ */
;


//==========================================================
// CAMERA_ISO_50
//==========================================================
GLOBAL const U16 reg_sub_iso_50[][2]
#if defined(_CAMACQ_API_C_)
={

CAMACQ_SUB_BTM_OF_DATA
}
#endif /* _CAMACQ_API_C_ */
;


//==========================================================
// CAMERA_ISO_100
//==========================================================
GLOBAL const U16 reg_sub_iso_100[][2]
#if defined(_CAMACQ_API_C_)
={

CAMACQ_SUB_BTM_OF_DATA
}
#endif /* _CAMACQ_API_C_ */
;


//==========================================================
// CAMERA_ISO_200
//==========================================================
GLOBAL const U16 reg_sub_iso_200[][2]
#if defined(_CAMACQ_API_C_)
={

CAMACQ_SUB_BTM_OF_DATA
}
#endif /* _CAMACQ_API_C_ */
;

//==========================================================
// CAMERA_ISO_400
//==========================================================
GLOBAL const U16 reg_sub_iso_400[][2]
#if defined(_CAMACQ_API_C_)
={


CAMACQ_SUB_BTM_OF_DATA
}
#endif /* _CAMACQ_API_C_ */
;

//==========================================================
// CAMERA_ISO_800
//==========================================================
GLOBAL const U16 reg_sub_iso_800[][2]
#if defined(_CAMACQ_API_C_)
={

CAMACQ_SUB_BTM_OF_DATA
}
#endif /* _CAMACQ_API_C_ */
;

//==========================================================
// CAMERA_ISO_1600
//==========================================================
GLOBAL const U16 reg_sub_iso_1600[][2]
#if defined(_CAMACQ_API_C_)
={

CAMACQ_SUB_BTM_OF_DATA
}
#endif /* _CAMACQ_API_C_ */
;

//==========================================================
// CAMERA_ISO_3200
//==========================================================
GLOBAL const U16 reg_sub_iso_3200[][2]
#if defined(_CAMACQ_API_C_)
={

CAMACQ_SUB_BTM_OF_DATA
}
#endif /* _CAMACQ_API_C_ */
;

//==========================================================
// CAMERA_SCENE_AUTO (OFF)
//==========================================================
GLOBAL const U16 reg_sub_scene_auto[][2]
#if defined(_CAMACQ_API_C_)
={

CAMACQ_SUB_BTM_OF_DATA
}
#endif /* _CAMACQ_API_C_ */
;

//==========================================================
// CAMERA_SCENE_NIGHT
//==========================================================
GLOBAL const U16 reg_sub_scene_night[][2]
#if defined(_CAMACQ_API_C_)
={

CAMACQ_SUB_BTM_OF_DATA
}
#endif /* _CAMACQ_API_C_ */
;

//==========================================================
// CAMERA_SCENE_LANDSCAPE
//==========================================================
GLOBAL const U16 reg_sub_scene_landscape[][2]
#if defined(_CAMACQ_API_C_)
={

CAMACQ_SUB_BTM_OF_DATA
}
#endif /* _CAMACQ_API_C_ */
;

//==========================================================
// CAMERA_SCENE_SUNSET
//==========================================================
GLOBAL const U16 reg_sub_scene_sunset[][2]
#if defined(_CAMACQ_API_C_)
={

CAMACQ_SUB_BTM_OF_DATA
}
#endif /* _CAMACQ_API_C_ */
;

//==========================================================
// CAMERA_SCENE_PORTRAIT
//==========================================================
GLOBAL const U16 reg_sub_scene_portrait[][2]
#if defined(_CAMACQ_API_C_)
={

CAMACQ_SUB_BTM_OF_DATA
}
#endif /* _CAMACQ_API_C_ */
;

//==========================================================
// CAMERA_SCENE_SUNRISE
//==========================================================
GLOBAL const U16 reg_sub_scene_sunrise[][2]
#if defined(_CAMACQ_API_C_)
={

CAMACQ_SUB_BTM_OF_DATA
}
#endif /* _CAMACQ_API_C_ */
;

//==========================================================
// CAMERA_SCENE_INDOOR // == PARTY
//==========================================================
GLOBAL const U16 reg_sub_scene_indoor[][2]
#if defined(_CAMACQ_API_C_)
={

CAMACQ_SUB_BTM_OF_DATA
}
#endif /* _CAMACQ_API_C_ */
;

//==========================================================
// CAMERA_SCENE_PARTY // == INDOOR
//==========================================================
GLOBAL const U16 reg_sub_scene_party[][2] 
#if defined(_CAMACQ_API_C_)
={

CAMACQ_SUB_BTM_OF_DATA
}
#endif /* _CAMACQ_API_C_ */
;

//==========================================================
// CAMERA_SCENE_SPORTS
//==========================================================
GLOBAL const U16 reg_sub_scene_sports[][2]
#if defined(_CAMACQ_API_C_)
={

CAMACQ_SUB_BTM_OF_DATA
}
#endif /* _CAMACQ_API_C_ */
;

//==========================================================
// CAMERA_SCENE_BEACH
//==========================================================
GLOBAL const U16 reg_sub_scene_beach[][2]
#if defined(_CAMACQ_API_C_)
={

CAMACQ_SUB_BTM_OF_DATA
}
#endif /* _CAMACQ_API_C_ */
;

//==========================================================
// CAMERA_SCENE_SNOW
//==========================================================
GLOBAL const U16 reg_sub_scene_snow[][2]
#if defined(_CAMACQ_API_C_)
={

CAMACQ_SUB_BTM_OF_DATA
}
#endif /* _CAMACQ_API_C_ */
;

//==========================================================
// CAMERA_SCENE_FALLCOLOR
//==========================================================
GLOBAL const U16 reg_sub_scene_fallcolor[][2]
#if defined(_CAMACQ_API_C_)
={

CAMACQ_SUB_BTM_OF_DATA
}
#endif /* _CAMACQ_API_C_ */
;

//==========================================================
// CAMERA_SCENE_FIREWORKS
//==========================================================
GLOBAL const U16 reg_sub_scene_fireworks[][2]
#if defined(_CAMACQ_API_C_)
={

CAMACQ_SUB_BTM_OF_DATA
}
#endif /* _CAMACQ_API_C_ */
;

//==========================================================
// CAMERA_SCENE_CANDLELIGHT
//==========================================================
GLOBAL const U16 reg_sub_scene_candlelight[][2]
#if defined(_CAMACQ_API_C_)
={

CAMACQ_SUB_BTM_OF_DATA
}
#endif /* _CAMACQ_API_C_ */
;

//==========================================================
// CAMERA_SCENE_AGAINSTLIGHT (BACKLight??)
//==========================================================
GLOBAL const U16 reg_sub_scene_againstlight[][2]
#if defined(_CAMACQ_API_C_)
={

CAMACQ_SUB_BTM_OF_DATA
}
#endif /* _CAMACQ_API_C_ */
;

//==========================================================
// CAMERA_SCENE_TEXT
//==========================================================
GLOBAL const U16 reg_sub_scene_text[][2]
#if defined(_CAMACQ_API_C_)
={

CAMACQ_SUB_BTM_OF_DATA
}
#endif /* _CAMACQ_API_C_ */
;

//==========================================================
// CAMERA_SCENE_AQUA
//==========================================================
GLOBAL const U16 reg_sub_scene_aqua[][2]
#if defined(_CAMACQ_API_C_)
={

CAMACQ_SUB_BTM_OF_DATA
}
#endif /* _CAMACQ_API_C_ */
;

//==========================================================
// CAMERA_ADJUST_CONTRAST_M2
//==========================================================
GLOBAL const U16 reg_sub_adjust_contrast_m2[][2]
#if defined(_CAMACQ_API_C_)
={

CAMACQ_SUB_BTM_OF_DATA
}
#endif /* _CAMACQ_API_C_ */
;

//==========================================================
// CAMERA_ADJUST_CONTRAST_M1
//==========================================================
GLOBAL const U16 reg_sub_adjust_contrast_m1[][2]
#if defined(_CAMACQ_API_C_)
={

CAMACQ_SUB_BTM_OF_DATA
}
#endif /* _CAMACQ_API_C_ */
;

//==========================================================
// CAMERA_ADJUST_CONTRAST_DEFAULT
//==========================================================
GLOBAL const U16 reg_sub_adjust_contrast_default[][2]
#if defined(_CAMACQ_API_C_)
={


CAMACQ_SUB_BTM_OF_DATA
}
#endif /* _CAMACQ_API_C_ */
;

//==========================================================
// CAMERA_ADJUST_CONTRAST_P1
//==========================================================
GLOBAL const U16 reg_sub_adjust_contrast_p1[][2]
#if defined(_CAMACQ_API_C_)
={


CAMACQ_SUB_BTM_OF_DATA
}
#endif /* _CAMACQ_API_C_ */
;

//==========================================================
// CAMERA_ADJUST_CONTRAST_P2
//==========================================================
GLOBAL const U16 reg_sub_adjust_contrast_p2[][2]
#if defined(_CAMACQ_API_C_)
={

CAMACQ_SUB_BTM_OF_DATA
}
#endif /* _CAMACQ_API_C_ */
;

//==========================================================
// CAMERA_ADJUST_SHARPNESS_M2
//==========================================================
GLOBAL const U16 reg_sub_adjust_sharpness_m2[][2]
#if defined(_CAMACQ_API_C_)
={


CAMACQ_SUB_BTM_OF_DATA
}
#endif /* _CAMACQ_API_C_ */
;

//==========================================================
// CAMERA_ADJUST_SHARPNESS_M1
//==========================================================
GLOBAL const U16 reg_sub_adjust_sharpness_m1[][2]
#if defined(_CAMACQ_API_C_)
={


CAMACQ_SUB_BTM_OF_DATA
}
#endif /* _CAMACQ_API_C_ */
;


//==========================================================
// CAMERA_ADJUST_SHARPNESS_DEFAULT
//==========================================================
GLOBAL const U16 reg_sub_adjust_sharpness_default[][2]
#if defined(_CAMACQ_API_C_)
={

CAMACQ_SUB_BTM_OF_DATA
}
#endif /* _CAMACQ_API_C_ */
;


//==========================================================
// CAMERA_ADJUST_SHARPNESS_P1
//==========================================================
GLOBAL const U16 reg_sub_adjust_sharpness_p1[][2]
#if defined(_CAMACQ_API_C_)
={


CAMACQ_SUB_BTM_OF_DATA
}
#endif /* _CAMACQ_API_C_ */
;


//==========================================================
// CAMERA_ADJUST_SHARPNESS_P2
//==========================================================
GLOBAL const U16 reg_sub_adjust_sharpness_p2[][2]
#if defined(_CAMACQ_API_C_)
={


CAMACQ_SUB_BTM_OF_DATA
}
#endif /* _CAMACQ_API_C_ */
;

//==========================================================
// CAMERA_ADJUST_SATURATION_M2
//==========================================================
GLOBAL const U16 reg_sub_adjust_saturation_m2[][2]
#if defined(_CAMACQ_API_C_)
={


CAMACQ_SUB_BTM_OF_DATA
}
#endif /* _CAMACQ_API_C_ */
;

//==========================================================
// CAMERA_ADJUST_SATURATION_M1
//==========================================================
GLOBAL const U16 reg_sub_adjust_saturation_m1[][2]
#if defined(_CAMACQ_API_C_)
={


CAMACQ_SUB_BTM_OF_DATA
}
#endif /* _CAMACQ_API_C_ */
;

//==========================================================
// CAMERA_ADJUST_SATURATION_DEFAULT
//==========================================================
GLOBAL const U16 reg_sub_adjust_saturation_default[][2]
#if defined(_CAMACQ_API_C_)
={


CAMACQ_SUB_BTM_OF_DATA
}
#endif /* _CAMACQ_API_C_ */
;

//==========================================================
// CAMERA_ADJUST_SATURATION_P1
//==========================================================
GLOBAL const U16 reg_sub_adjust_saturation_p1[][2]
#if defined(_CAMACQ_API_C_)
={


CAMACQ_SUB_BTM_OF_DATA
}
#endif /* _CAMACQ_API_C_ */
;

//==========================================================
// CAMERA_ADJUST_SATURATION_P2
//==========================================================
GLOBAL const U16 reg_sub_adjust_saturation_p2[][2]
#if defined(_CAMACQ_API_C_)
={


CAMACQ_SUB_BTM_OF_DATA
}
#endif /* _CAMACQ_API_C_ */
;

//==========================================================
// CAMERA_JPEG_output_5M
//==========================================================
GLOBAL const U16 reg_sub_jpeg_5m[][2]
#if defined(_CAMACQ_API_C_)
={


CAMACQ_SUB_BTM_OF_DATA
}
#endif /* _CAMACQ_API_C_ */
;

//==========================================================
// CAMERA_JPEG_output_5M_2
//==========================================================
GLOBAL const U16 reg_sub_jpeg_5m_2[][2]
#if defined(_CAMACQ_API_C_)
={


CAMACQ_SUB_BTM_OF_DATA
}
#endif /* _CAMACQ_API_C_ */
;

//==========================================================
// CAMERA_JPEG_output_4M
//==========================================================
GLOBAL const U16 reg_sub_jpeg_w4m[][2]
#if defined(_CAMACQ_API_C_)
={


CAMACQ_SUB_BTM_OF_DATA
}
#endif /* _CAMACQ_API_C_ */
;

//==========================================================
// CAMERA_JPEG_output_3M
//==========================================================
GLOBAL const U16 reg_sub_jpeg_3m[][2]
#if defined(_CAMACQ_API_C_)
={


CAMACQ_SUB_BTM_OF_DATA
}
#endif /* _CAMACQ_API_C_ */
;

//==========================================================
// CAMERA_JPEG_output_2M
//==========================================================
GLOBAL const U16 reg_sub_jpeg_2m[][2]
#if defined(_CAMACQ_API_C_)
={

CAMACQ_SUB_BTM_OF_DATA
}
#endif /* _CAMACQ_API_C_ */
;

//==========================================================
// CAMERA_JPEG_output_w1_5M
//==========================================================
GLOBAL const U16 reg_sub_jpeg_w1_5m[][2]
#if defined(_CAMACQ_API_C_)
={

CAMACQ_SUB_BTM_OF_DATA
}
#endif /* _CAMACQ_API_C_ */
;

//==========================================================
// CAMERA_JPEG_output_1M
//==========================================================
GLOBAL const U16 reg_sub_jpeg_1m[][2]
#if defined(_CAMACQ_API_C_)
={


CAMACQ_SUB_BTM_OF_DATA
}
#endif /* _CAMACQ_API_C_ */
;

//==========================================================
// CAMERA_JPEG_output_VGA
//==========================================================
GLOBAL const U16 reg_sub_jpeg_vga[][2]
#if defined(_CAMACQ_API_C_)
={


CAMACQ_SUB_BTM_OF_DATA
}
#endif /* _CAMACQ_API_C_ */
;

//==========================================================
// CAMERA_JPEG_output_WQVGA
//==========================================================
GLOBAL const U16 reg_sub_jpeg_wqvga[][2]
#if defined(_CAMACQ_API_C_)
={

CAMACQ_SUB_BTM_OF_DATA
}
#endif /* _CAMACQ_API_C_ */
;

//==========================================================
// CAMERA_JPEG_output_QVGA
//==========================================================
GLOBAL const U16 reg_sub_jpeg_qvga[][2]
#if defined(_CAMACQ_API_C_)
={


CAMACQ_SUB_BTM_OF_DATA
}
#endif /* _CAMACQ_API_C_ */
;

//==========================================================
// CAMERA_output_qcif
//==========================================================
GLOBAL const U16 reg_sub_qcif[][2]
#if defined(_CAMACQ_API_C_)
={
// [VGA to QCIF]
{0x098C, 0x2739}, 	    // MCU_ADDRESS [MODE_CROP_X0_A]
{0x0990, 0x0000}, 	    // MCU_DATA_0
{0x098C, 0x273B}, 	    // MCU_ADDRESS [MODE_CROP_X1_A]
{0x0990, 0x027F}, 	    // MCU_DATA_0
{0x098C, 0x273D}, 	    // MCU_ADDRESS [MODE_CROP_Y0_A]
{0x0990, 0x0000}, 	    // MCU_DATA_0
{0x098C, 0x273F}, 	    // MCU_ADDRESS [MODE_CROP_Y1_A]
{0x0990, 0x01DF}, 	    // MCU_DATA_0
{0x098C, 0x2703},	    // MCU_ADDRESS [MODE_OUTPUT_WIDTH_A]
{0x0990, 0x00B0}, 	    // MCU_DATA_0
{0x098C, 0x2705}, 	    // MCU_ADDRESS [MODE_OUTPUT_HEIGHT_A]
{0x0990, 0x0090}, 	    // MCU_DATA_0
{0x098C, 0xA103}, 	    // MCU_ADDRESS [SEQ_CMD]
{0x0990, 0x0005}, 	    // MCU_DATA_0

{0xFFFF, 0x012C},

CAMACQ_SUB_BTM_OF_DATA
}
#endif /* _CAMACQ_API_C_ */
;

//==========================================================
// CAMERA_output_qvga
//==========================================================
GLOBAL const U16 reg_sub_qvga[][2]
#if defined(_CAMACQ_API_C_)
={
// [VGA to QVGA]
{0x098C, 0x2739}, 	    // MCU_ADDRESS [MODE_CROP_X0_A]
{0x0990, 0x0000}, 	    // MCU_DATA_0
{0x098C, 0x273B}, 	    // MCU_ADDRESS [MODE_CROP_X1_A]
{0x0990, 0x027F}, 	    // MCU_DATA_0
{0x098C, 0x273D}, 	    // MCU_ADDRESS [MODE_CROP_Y0_A]
{0x0990, 0x0000}, 	    // MCU_DATA_0
{0x098C, 0x273F}, 	    // MCU_ADDRESS [MODE_CROP_Y1_A]
{0x0990, 0x01DF}, 	    // MCU_DATA_0
{0x098C, 0x2703},	    // MCU_ADDRESS [MODE_OUTPUT_WIDTH_A]
{0x0990, 0x0140}, 	    // MCU_DATA_0
{0x098C, 0x2705}, 	    // MCU_ADDRESS [MODE_OUTPUT_HEIGHT_A]
{0x0990, 0x00F0}, 	    // MCU_DATA_0
{0x098C, 0xA103}, 	    // MCU_ADDRESS [SEQ_CMD]
{0x0990, 0x0005}, 	    // MCU_DATA_0

{0xFFFF, 0x012C},

CAMACQ_SUB_BTM_OF_DATA
}
#endif /* _CAMACQ_API_C_ */
;

//==========================================================
// CAMERA_output_cif
//==========================================================
GLOBAL const U16 reg_sub_cif[][2]
#if defined(_CAMACQ_API_C_)
={

CAMACQ_SUB_BTM_OF_DATA
}
#endif /* _CAMACQ_API_C_ */
;

//==========================================================
// CAMERA_output_vga
//==========================================================
GLOBAL const U16 reg_sub_vga[][2]
#if defined(_CAMACQ_API_C_)
={
// [QCIF to VGA]
{0x098C, 0x2739}, 	// MCU_ADDRESS [MODE_CROP_X0_A]
{0x0990, 0x0000}, 	// MCU_DATA_0
{0x098C, 0x273B}, 	// MCU_ADDRESS [MODE_CROP_X1_A]
{0x0990, 0x027F}, 	// MCU_DATA_0
{0x098C, 0x273D}, 	// MCU_ADDRESS [MODE_CROP_Y0_A]
{0x0990, 0x0000},	// MCU_DATA_0
{0x098C, 0x273F}, 	// MCU_ADDRESS [MODE_CROP_Y1_A]
{0x0990, 0x01DF}, 	// MCU_DATA_0
{0x098C, 0x2703}, 	// MCU_ADDRESS [MODE_OUTPUT_WIDTH_A]
{0x0990, 0x0280}, 	// MCU_DATA_0
{0x098C, 0x2705}, 	// MCU_ADDRESS [MODE_OUTPUT_HEIGHT_A]
{0x0990, 0x01E0}, 	// MCU_DATA_0
{0x098C, 0xA103}, 	// MCU_ADDRESS [SEQ_CMD]
{0x0990, 0x0005}, 	// MCU_DATA_0

{0xFFFF, 0x012C},

CAMACQ_SUB_BTM_OF_DATA
}
#endif /* _CAMACQ_API_C_ */
;

GLOBAL const U16 reg_sub_1080P[][2]
#if defined(_CAMACQ_API_C_)
={

CAMACQ_SUB_BTM_OF_DATA
}
#endif /* _CAMACQ_API_C_ */
;

GLOBAL const U16 reg_sub_720P[][2]
#if defined(_CAMACQ_API_C_)
={

CAMACQ_SUB_BTM_OF_DATA
}
#endif /* _CAMACQ_API_C_ */
;

GLOBAL const U16 reg_sub_800_480[][2]
#if defined(_CAMACQ_API_C_)
={

CAMACQ_SUB_BTM_OF_DATA
}
#endif /* _CAMACQ_API_C_ */
;


GLOBAL const U16 reg_sub_720_480[][2]
#if defined(_CAMACQ_API_C_)
={

CAMACQ_SUB_BTM_OF_DATA
}
#endif /* _CAMACQ_API_C_ */
;


GLOBAL const U16 reg_sub_not[][2]
#if defined(_CAMACQ_API_C_)
={

CAMACQ_SUB_BTM_OF_DATA
}
#endif /* _CAMACQ_API_C_ */
;

GLOBAL const U16 reg_sub_flicker_disabled[][2]
#if defined(_CAMACQ_API_C_)
={

CAMACQ_SUB_BTM_OF_DATA
}
#endif /* _CAMACQ_API_C_ */
;

GLOBAL const U16 reg_sub_flicker_50hz[][2]
#if defined(_CAMACQ_API_C_)
={

CAMACQ_SUB_BTM_OF_DATA
}
#endif /* _CAMACQ_API_C_ */
;

GLOBAL const U16 reg_sub_flicker_60hz[][2]
#if defined(_CAMACQ_API_C_)
={

CAMACQ_SUB_BTM_OF_DATA
}
#endif /* _CAMACQ_API_C_ */
;

GLOBAL const U16 reg_sub_flicker_auto[][2]
#if defined(_CAMACQ_API_C_)
={

CAMACQ_SUB_BTM_OF_DATA
}
#endif /* _CAMACQ_API_C_ */
;

GLOBAL const U16 reg_sub_jpeg_quality_superfine[][2]
#if defined(_CAMACQ_API_C_)
={

CAMACQ_SUB_BTM_OF_DATA
}
#endif /* _CAMACQ_API_C_ */
;

GLOBAL const U16 reg_sub_jpeg_quality_fine[][2]
#if defined(_CAMACQ_API_C_)
={


CAMACQ_SUB_BTM_OF_DATA
}
#endif /* _CAMACQ_API_C_ */
;

GLOBAL const U16 reg_sub_jpeg_quality_normal[][2]
#if defined(_CAMACQ_API_C_)
={

CAMACQ_SUB_BTM_OF_DATA
}
#endif /* _CAMACQ_API_C_ */
;

#undef GLOBAL

#endif /* _CAMACQ_MT9V114_MIPI_H_ */
