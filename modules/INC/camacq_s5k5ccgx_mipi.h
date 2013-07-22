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

#if !defined(_CAMACQ_S5K5CCGX_MIPI_H_)
#define _CAMACQ_S5K5CCGX_MIPI_H_

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
#define _S5K5CCGX_EVT1_MIPI_    //sensor option

#define CAMACQ_MAIN_NAME         "s5k5ccgx"
#define CAMACQ_MAIN_I2C_ID       0x3C	                // 0x78
#define CAMACQ_MAIN_RES_TYPE   	 CAMACQ_SENSOR_MAIN     // main sensor

#define CAMACQ_MAIN_ISPROBED     0
#define CAMACQ_MAIN_CLOCK        0               
#define CAMACQ_MAIN_YUVORDER     0
#define CAMACQ_MAIN_V_SYNCPOL    0
#define CAMACQ_MAIN_H_SYNCPOL    0
#define CAMACQ_MAIN_SAMPLE_EDGE  0
#define CAMACQ_MAIN_FULL_RANGE   0

#define CAMACQ_MAIN_RST 
#define CAMACQ_MAIN_RST_MUX 
#define CAMACQ_MAIN_EN 
#define CAMACQ_MAIN_EN_MUX 

#define CAMACQ_MAIN_RST_ON          1
#define CAMACQ_MAIN_RST_OFF         0
#define CAMACQ_MAIN_EN_ON           1
#define CAMACQ_MAIN_EN_OFF          0
#define CAMACQ_MAIN_STANDBY_ON      1
#define CAMACQ_MAIN_STANDBY_OFF	    0

#define CAMACQ_MAIN_POWER_CTRL(onoff)
 
#define CAMACQ_MAIN_2BYTE_SENSOR    0
#define CAMACQ_MAIN_AF              1
#define CAMACQ_MAIN_INT_MODE        CAMACQ_EXT_LEN_2BYTE_INT
#define CAMACQ_MAIN_FS_MODE         0
#define CAMACQ_MAIN_PATH_SET_FILE   "/sdcard/sensor/main/%s.dat"

#if (CAMACQ_MAIN_2BYTE_SENSOR)	
#define CAMACQ_MAIN_BURST_MODE 1
#else
#define CAMACQ_MAIN_BURST_MODE 0
#endif /* CAMACQ_MAIN2BYTE_SENSOR */

#define CAMACQ_MAIN_BURST_I2C_TRACE 0
#define CAMACQ_MAIN_BURST_MAX 500

#define CAMACQ_MAIN_REG_FLAG_CNTS 	0x0F12
#define CAMACQ_MAIN_REG_DELAY 		0xFFFF
#define CAMACQ_MAIN_BTM_OF_DATA 	{0xFFFF, 0xFFFF},
#define CAMACQ_MAIN_END_MARKER 		0xFF
#define CAMACQ_MAIN_REG_SET_SZ 		2	// {0xFFFFFFFF} is 1, {0xFFFF,0xFFFF} is 2, {0xFF,0XFF} is 2, {0xFF,0xFF,0xFF,0xFF} is 4, {0xFFFF} is 1
#define CAMACQ_MAIN_REG_DAT_SZ 		2 // {0xFFFFFFFF} is 4, {0xFFFF,0xFFFF} is 2, {0xFF,0XFF} is 1, {0xFF,0xFF,0xFF,0xFF} is 1, {0xFFFF} is 2

#define CAMACQ_MAIN_FRATE_MIN  5
#define CAMACQ_MAIN_FRATE_MAX  30

// MACRO FUNCTIONS BEGIN //////////////////////////////////////////////////////////// 
#if (CAMACQ_MAIN_2BYTE_SENSOR)
#define CAMACQ_MAIN_EXT_RD_SZ 1
#else
#define CAMACQ_MAIN_EXT_RD_SZ 2
#endif /* CAMACQ_MAIN_2BYTE_SENSOR */

#if CAMACQ_MAIN_2BYTE_SENSOR
#define CAMACQ_MAIN_EXT_REG_IS_BTM_OF_DATA(A)		(((A[0]==CAMACQ_MAIN_END_MARKER) && (A[1]==CAMACQ_MAIN_END_MARKER))? 1:0)
#define CAMACQ_MAIN_EXT_REG_IS_DELAY(A)				((A[0]==CAMACQ_MAIN_REG_DELAY)? 1:0)

#if (CAMACQ_MAIN_FS_MODE==1)
#define CAMACQ_MAIN_EXT_REG_GET_DATA(dest,srce,idx)\
memcpy(dest, &(srce[idx*CAMACQ_MAIN_REG_DAT_SZ*CAMACQ_MAIN_REG_SET_SZ]), CAMACQ_MAIN_REG_DAT_SZ*CAMACQ_MAIN_REG_SET_SZ);
#elif (CAMACQ_MAIN_REG_DAT_SZ==1)
#define CAMACQ_MAIN_EXT_REG_GET_DATA(dest,srce,idx)	dest[0] = (srce[idx][0] & 0xFF); dest[1] = (srce[idx][1] & 0xFF);
#elif (CAMACQ_MAIN_REG_DAT_SZ==2)
#define CAMACQ_MAIN_EXT_REG_GET_DATA(dest,srce,idx)	dest[0] = ((U8)(srce[idx] >> 8) & 0xFF); dest[1] = ((U8)(srce[idx]) & 0xFF);
#endif

#else // CAMACQ_MAIN_2BYTE_SENSOR

#define CAMACQ_MAIN_EXT_REG_IS_BTM_OF_DATA(A)		(((A[0]==CAMACQ_MAIN_END_MARKER) && (A[1]==CAMACQ_MAIN_END_MARKER) && \
(A[2]==CAMACQ_MAIN_END_MARKER) && (A[3]==CAMACQ_MAIN_END_MARKER))? 1:0)
#define CAMACQ_MAIN_EXT_REG_IS_DELAY(A)				(((A[0]==((CAMACQ_MAIN_REG_DELAY>>8) & 0xFF)) && (A[1]==(CAMACQ_MAIN_REG_DELAY & 0xFF)))? 1:0)
#define CAMACQ_MAIN_EXT_REG_IS_CNTS(A)				(((A[0]==((CAMACQ_MAIN_REG_FLAG_CNTS>>8) & 0xFF)) && (A[1]==(CAMACQ_MAIN_REG_FLAG_CNTS & 0xFF)))? 1:0)

#if (CAMACQ_MAIN_FS_MODE==1)
#define CAMACQ_MAIN_EXT_REG_GET_DATA(dest,srce,idx)\
memcpy(dest, &(srce[idx*CAMACQ_MAIN_REG_DAT_SZ*CAMACQ_MAIN_REG_SET_SZ]), CAMACQ_MAIN_REG_DAT_SZ*CAMACQ_MAIN_REG_SET_SZ);
#elif (CAMACQ_MAIN_REG_DAT_SZ==2)
#define CAMACQ_MAIN_EXT_REG_GET_DATA(dest,srce,idx)	dest[0]=((srce[idx][0] >> 8) & 0xFF); dest[1]=(srce[idx][0] & 0xFF); \
dest[2]=((srce[idx][1] >> 8) & 0xFF); dest[3]=(srce[idx][1] & 0xFF);
#elif (CAMACQ_MAIN_REG_DAT_SZ==1)
#define CAMACQ_MAIN_EXT_REG_GET_DATA(dest,srce,idx)	dest[0]=srce[idx][0]; dest[1]=srce[idx][1]; \
dest[2]=srce[idx][2]; dest[3]=srce[idx][3];
#elif (CAMACQ_MAIN_REG_DAT_SZ==4)
#define CAMACQ_MAIN_EXT_REG_GET_DATA(dest,srce,idx)	dest[0] = ((U8)(srce[idx] >> 24) & 0xFF); dest[1] = ((U8)(srce[idx] >> 16) & 0xFF); \
dest[2] = ((U8)(srce[idx] >> 8) & 0xFF); dest[3] = ((U8)(srce[idx]) & 0xFF);			
#endif
#endif /* CAMACQ_2BYTE_SENSOR */
// MACRO FUNCTIONS END /////////////////////////////////////////////////////////// 


/* DEFINE for sensor regs*/
#if( CAMACQ_MAIN_FS_MODE )
#define CAMACQ_MAIN_REG_PLL                     "reg_main_pll"
#define CAMACQ_MAIN_REG_INIT                    "reg_main_init"
#define CAMACQ_MAIN_REG_PREVIEW                 "reg_main_preview"
#define CAMACQ_MAIN_REG_SNAPSHOT                "reg_main_snapshot"
#define CAMACQ_MAIN_REG_MIDLIGHT                "reg_main_midlight"
#define CAMACQ_MAIN_REG_HIGHLIGHT               "reg_main_highlight"
#define CAMACQ_MAIN_REG_LOWLIGHT                "reg_main_lowlight"
#define CAMACQ_MAIN_REG_NIGHTSHOT_ON            "reg_main_nightshot_on"
#define CAMACQ_MAIN_REG_NIGHTSHOT_OFF           "reg_main_nightshot_off"
#define CAMACQ_MAIN_REG_NIGHTSHOT               "reg_main_nightshot"
#define CAMACQ_MAIN_REG_CAMCORDER_ON            "reg_main_camcorder_on"

#define CAMACQ_MAIN_REG_WB_AUTO                 "reg_main_wb_auto"
#define CAMACQ_MAIN_REG_WB_DAYLIGHT             "reg_main_wb_daylight"
#define CAMACQ_MAIN_REG_WB_CLOUDY               "reg_main_wb_cloudy"
#define CAMACQ_MAIN_REG_WB_INCANDESCENT         "reg_main_wb_incandescent"
#define CAMACQ_MAIN_REG_WB_FLUORESCENT          "reg_main_wb_fluorescent"
#define CAMACQ_MAIN_REG_WB_TWILIGHT             "reg_main_wb_twilight"
#define CAMACQ_MAIN_REG_WB_TUNGSTEN             "reg_main_wb_tungsten"
#define CAMACQ_MAIN_REG_WB_OFF                  "reg_main_wb_off"
#define CAMACQ_MAIN_REG_WB_HORIZON              "reg_main_wb_horizon"
#define CAMACQ_MAIN_REG_WB_SHADE                "reg_main_wb_shade"

#define CAMACQ_MAIN_REG_EFFECT_NONE             "reg_main_effect_none"
#define CAMACQ_MAIN_REG_EFFECT_GRAY             "reg_main_effect_gray" // mono, blackwhite
#define CAMACQ_MAIN_REG_EFFECT_NEGATIVE         "reg_main_effect_negative"
#define CAMACQ_MAIN_REG_EFFECT_SOLARIZE         "reg_main_effect_solarize"
#define CAMACQ_MAIN_REG_EFFECT_SEPIA            "reg_main_effect_sepia"
#define CAMACQ_MAIN_REG_EFFECT_POSTERIZE        "reg_main_effect_posterize"
#define CAMACQ_MAIN_REG_EFFECT_WHITEBOARD       "reg_main_effect_whiteboard"
#define CAMACQ_MAIN_REG_EFFECT_BLACKBOARD       "reg_main_effect_blackboard"
#define CAMACQ_MAIN_REG_EFFECT_AQUA             "reg_main_effect_aqua"
#define CAMACQ_MAIN_REG_EFFECT_SHARPEN          "reg_main_effect_sharpen"
#define CAMACQ_MAIN_REG_EFFECT_VIVID            "reg_main_effect_vivid"
#define CAMACQ_MAIN_REG_EFFECT_GREEN            "reg_main_effect_green"
#define CAMACQ_MAIN_REG_EFFECT_SKETCH           "reg_main_effect_sketch"

#define CAMACQ_MAIN_REG_METER_MATRIX            "reg_main_meter_matrix"
#define CAMACQ_MAIN_REG_METER_CW                "reg_main_meter_cw"
#define CAMACQ_MAIN_REG_METER_SPOT              "reg_main_meter_spot"

#define CAMACQ_MAIN_REG_FLIP_NONE               "reg_main_flip_none"
#define CAMACQ_MAIN_REG_FLIP_WATER              "reg_main_flip_water"
#define CAMACQ_MAIN_REG_FLIP_MIRROR             "reg_main_flip_mirror"
#define CAMACQ_MAIN_REG_FLIP_WATER_MIRROR       "reg_main_flip_water_mirror"


#define CAMACQ_MAIN_REG_FPS_FIXED_5             "reg_main_fps_fixed_5"
#define CAMACQ_MAIN_REG_FPS_FIXED_7             "reg_main_fps_fixed_7"
#define CAMACQ_MAIN_REG_FPS_FIXED_10            "reg_main_fps_fixed_10"
#define CAMACQ_MAIN_REG_FPS_FIXED_15            "reg_main_fps_fixed_15"
#define CAMACQ_MAIN_REG_FPS_FIXED_20            "reg_main_fps_fixed_20"
#define CAMACQ_MAIN_REG_FPS_FIXED_25            "reg_main_fps_fixed_25"
#define CAMACQ_MAIN_REG_FPS_FIXED_30            "reg_main_fps_fixed_30"
#define CAMACQ_MAIN_REG_FPS_VAR_15              "reg_main_fps_var_15"

#define CAMACQ_MAIN_REG_BRIGHTNESS_LEVEL_0      "reg_main_brightness_level_0"
#define CAMACQ_MAIN_REG_BRIGHTNESS_LEVEL_1      "reg_main_brightness_level_1"
#define CAMACQ_MAIN_REG_BRIGHTNESS_LEVEL_2      "reg_main_brightness_level_2"
#define CAMACQ_MAIN_REG_BRIGHTNESS_LEVEL_3      "reg_main_brightness_level_3"
#define CAMACQ_MAIN_REG_BRIGHTNESS_LEVEL_4      "reg_main_brightness_level_4"
#define CAMACQ_MAIN_REG_BRIGHTNESS_LEVEL_5      "reg_main_brightness_level_5"
#define CAMACQ_MAIN_REG_BRIGHTNESS_LEVEL_6      "reg_main_brightness_level_6"
#define CAMACQ_MAIN_REG_BRIGHTNESS_LEVEL_7      "reg_main_brightness_level_7"
#define CAMACQ_MAIN_REG_BRIGHTNESS_LEVEL_8      "reg_main_brightness_level_8"

#define CAMACQ_MAIN_REG_EXPCOMPENSATION_LEVEL_0 "reg_main_expcompensation_level_0"
#define CAMACQ_MAIN_REG_EXPCOMPENSATION_LEVEL_1 "reg_main_expcompensation_level_1"
#define CAMACQ_MAIN_REG_EXPCOMPENSATION_LEVEL_2 "reg_main_expcompensation_level_2"
#define CAMACQ_MAIN_REG_EXPCOMPENSATION_LEVEL_3 "reg_main_expcompensation_level_3"
#define CAMACQ_MAIN_REG_EXPCOMPENSATION_LEVEL_4 "reg_main_expcompensation_level_4"
#define CAMACQ_MAIN_REG_EXPCOMPENSATION_LEVEL_5 "reg_main_expcompensation_level_5"
#define CAMACQ_MAIN_REG_EXPCOMPENSATION_LEVEL_6 "reg_main_expcompensation_level_6"
#define CAMACQ_MAIN_REG_EXPCOMPENSATION_LEVEL_7 "reg_main_expcompensation_level_7"
#define CAMACQ_MAIN_REG_EXPCOMPENSATION_LEVEL_8 "reg_main_expcompensation_level_8"

#define CAMACQ_MAIN_REG_SET_AF                  "reg_main_set_af"  // start af
#define CAMACQ_MAIN_REG_OFF_AF                  "reg_main_off_af"
#define CAMACQ_MAIN_REG_CHECK_AF                "reg_main_check_af"
#define CAMACQ_MAIN_REG_RESET_AF                "reg_main_reset_af"
#define CAMACQ_MAIN_REG_CANCEL_MACRO_AF         "reg_main_cancel_macro_af"
#define CAMACQ_MAIN_REG_CANCEL_MANUAL_AF        "reg_main_cancel_manual_af"
#define CAMACQ_MAIN_REG_MANUAL_AF               "reg_main_manual_af"    // normal_af
#define CAMACQ_MAIN_REG_MACRO_AF                "reg_main_macro_af"
#define CAMACQ_MAIN_REG_RETURN_MACRO_AF         "reg_main_return_macro_af"
#define CAMACQ_MAIN_REG_RETURN_MANUAL_AF        "reg_main_return_manual_af"
#define CAMACQ_MAIN_REG_SET_AF_NLUX             "reg_main_set_af_nlux"
#define CAMACQ_MAIN_REG_SET_AF_LLUX             "reg_main_set_af_llux"
#define CAMACQ_MAIN_REG_SET_AF_NORMAL_MODE_1    "reg_main_set_af_normal_mode_1"
#define CAMACQ_MAIN_REG_SET_AF_NORMAL_MODE_2    "reg_main_set_af_normal_mode_2"
#define CAMACQ_MAIN_REG_SET_AF_NORMAL_MODE_3    "reg_main_set_af_normal_mode_3"
#define CAMACQ_MAIN_REG_SET_AF_MACRO_MODE_1     "reg_main_set_af_macro_mode_1"
#define CAMACQ_MAIN_REG_SET_AF_MACRO_MODE_2     "reg_main_set_af_macro_mode_2"
#define CAMACQ_MAIN_REG_SET_AF_MACRO_MODE_3     "reg_main_set_af_macro_mode_3"

#define CAMACQ_MAIN_REG_ISO_AUTO                "reg_main_iso_auto"
#define CAMACQ_MAIN_REG_ISO_50                  "reg_main_iso_50"
#define CAMACQ_MAIN_REG_ISO_100                 "reg_main_iso_100"
#define CAMACQ_MAIN_REG_ISO_200                 "reg_main_iso_200"
#define CAMACQ_MAIN_REG_ISO_400                 "reg_main_iso_400"
#define CAMACQ_MAIN_REG_ISO_800                 "reg_main_iso_800"
#define CAMACQ_MAIN_REG_ISO_1600                "reg_main_iso_1600"
#define CAMACQ_MAIN_REG_ISO_3200                "reg_main_iso_3200"

#define CAMACQ_MAIN_REG_SCENE_AUTO              "reg_main_scene_auto"  // auto, off
#define CAMACQ_MAIN_REG_SCENE_NIGHT             "reg_main_scene_night"
#define CAMACQ_MAIN_REG_SCENE_LANDSCAPE         "reg_main_scene_landscape"
#define CAMACQ_MAIN_REG_SCENE_SUNSET            "reg_main_scene_sunset"
#define CAMACQ_MAIN_REG_SCENE_PORTRAIT          "reg_main_scene_portrait"
#define CAMACQ_MAIN_REG_SCENE_SUNRISE           "reg_main_scene_sunrise"   // dawn
#define CAMACQ_MAIN_REG_SCENE_INDOOR            "reg_main_scene_indoor"
#define CAMACQ_MAIN_REG_SCENE_PARTY             "reg_main_scene_party"
#define CAMACQ_MAIN_REG_SCENE_SPORTS            "reg_main_scene_sports"
#define CAMACQ_MAIN_REG_SCENE_BEACH             "reg_main_scene_beach"
#define CAMACQ_MAIN_REG_SCENE_SNOW              "reg_main_scene_snow"
#define CAMACQ_MAIN_REG_SCENE_FALLCOLOR         "reg_main_scene_fallcolor"
#define CAMACQ_MAIN_REG_SCENE_FIREWORKS         "reg_main_scene_fireworks"
#define CAMACQ_MAIN_REG_SCENE_CANDLELIGHT       "reg_main_scene_candlelight"
#define CAMACQ_MAIN_REG_SCENE_AGAINSTLIGHT      "reg_main_scene_againstlight"  // backlight
#define CAMACQ_MAIN_REG_SCENE_TEXT              "reg_main_scene_text"
#define CAMACQ_MAIN_REG_SCENE_AQUA              "reg_main_scene_aqua"

#define CAMACQ_MAIN_REG_ADJUST_CONTRAST_M2              "reg_main_adjust_contrast_m2"
#define CAMACQ_MAIN_REG_ADJUST_CONTRAST_M1              "reg_main_adjust_contrast_m1"
#define CAMACQ_MAIN_REG_ADJUST_CONTRAST_DEFAULT         "reg_main_adjust_contrast_default"
#define CAMACQ_MAIN_REG_ADJUST_CONTRAST_P1              "reg_main_adjust_contrast_p1"
#define CAMACQ_MAIN_REG_ADJUST_CONTRAST_P2              "reg_main_adjust_contrast_p2"

#define CAMACQ_MAIN_REG_ADJUST_SHARPNESS_M2             "reg_main_adjust_sharpness_m2"
#define CAMACQ_MAIN_REG_ADJUST_SHARPNESS_M1             "reg_main_adjust_sharpness_m1"
#define CAMACQ_MAIN_REG_ADJUST_SHARPNESS_DEFAULT        "reg_main_adjust_sharpness_default"
#define CAMACQ_MAIN_REG_ADJUST_SHARPNESS_P1             "reg_main_adjust_sharpness_p1"
#define CAMACQ_MAIN_REG_ADJUST_SHARPNESS_P2             "reg_main_adjust_sharpness_p2"

#define CAMACQ_MAIN_REG_ADJUST_SATURATION_M2            "reg_main_adjust_saturation_m2"
#define CAMACQ_MAIN_REG_ADJUST_SATURATION_M1            "reg_main_adjust_saturation_m1"
#define CAMACQ_MAIN_REG_ADJUST_SATURATION_DEFAULT       "reg_main_adjust_saturation_default"
#define CAMACQ_MAIN_REG_ADJUST_SATURATION_P1            "reg_main_adjust_saturation_p1"
#define CAMACQ_MAIN_REG_ADJUST_SATURATION_P2            "reg_main_adjust_saturation_p2"

#define CAMACQ_MAIN_REG_QCIF                            "reg_main_qcif"
#define CAMACQ_MAIN_REG_QVGA                            "reg_main_qvga"
#define CAMACQ_MAIN_REG_CIF                             "reg_main_cif"
#define CAMACQ_MAIN_REG_VGA                             "reg_main_vga"
#define CAMACQ_MAIN_REG_1080P		                    "reg_main_1080P"
#define CAMACQ_MAIN_REG_720P			                "reg_main_720P"
#define CAMACQ_MAIN_REG_800_480			                "reg_main_800_480"
#define CAMACQ_MAIN_REG_720_480			                "reg_main_720_480"
#define CAMACQ_MAIN_REG_NOT                             "reg_main_not"

#define CAMACQ_MAIN_REG_JPEG_5M                         "reg_main_jpeg_5m"        //2560X1920
#define CAMACQ_MAIN_REG_JPEG_5M_2                       "reg_main_jpeg_5m_2"      // 2592X1944
#define CAMACQ_MAIN_REG_JPEG_W4M                        "reg_main_jpeg_w4m"     // 2560x1536
#define CAMACQ_MAIN_REG_JPEG_3M                         "reg_main_jpeg_3m"        // QXGA 2048X1536
#define CAMACQ_MAIN_REG_JPEG_2M                         "reg_main_jpeg_2m"        // UXGA 1600x1200
#define CAMACQ_MAIN_REG_JPEG_W1_5M                      "reg_main_jpeg_w1_5m"    // 1280x960
#define CAMACQ_MAIN_REG_JPEG_1M                         "reg_main_jpeg_1m"
#define CAMACQ_MAIN_REG_JPEG_VGA                        "reg_main_jpeg_vga"   //640x480
#define CAMACQ_MAIN_REG_JPEG_WQVGA                      "reg_main_jpeg_wqvga" //420x240
#define CAMACQ_MAIN_REG_JPEG_QVGA                       "reg_main_jpeg_qvga"  //320x240

#define CAMACQ_MAIN_REG_FLICKER_DISABLED                "reg_main_flicker_disabled"
#define CAMACQ_MAIN_REG_FLICKER_50HZ                    "reg_main_flicker_50hz"
#define CAMACQ_MAIN_REG_FLICKER_60HZ                    "reg_main_flicker_60hz"
#define CAMACQ_MAIN_REG_FLICKER_AUTO                    "reg_main_flicker_auto"

#define CAMACQ_MAIN_REG_AE_LOCK                         "reg_main_ae_lock"
#define CAMACQ_MAIN_REG_AE_UNLOCK                       "reg_main_ae_unlock"             
#define CAMACQ_MAIN_REG_AWB_LOCK                        "reg_main_awb_lock"
#define CAMACQ_MAIN_REG_AWB_UNLOCK                      "reg_main_awb_unlock"

#define CAMACQ_MAIN_REG_PRE_FLASH_START                 "reg_main_pre_flash_start"
#define CAMACQ_MAIN_REG_PRE_FLASH_END                   "reg_main_pre_flash_end"
#define CAMACQ_MAIN_REG_MAIN_FLASH_START                "reg_main_main_flash_start"
#define CAMACQ_MAIN_REG_MAIN_FLASH_END                  "reg_main_main_flash_end"

#define CAMACQ_MAIN_REG_FLASH_AE_SET                    "reg_main_flash_ae_set"
#define CAMACQ_MAIN_REG_FLASH_AE_CLEAR                  "reg_main_flash_ae_clear"

//denis_flash_2
#define CAMACQ_MAIN_REG_FLASH_ON_SET            "reg_main_flash_on_set"             

// image quality
#define CAMACQ_MAIN_REG_JPEG_QUALITY_SUPERFINE          "reg_main_jpeg_quality_superfine"
#define CAMACQ_MAIN_REG_JPEG_QUALITY_FINE               "reg_main_jpeg_quality_fine"
#define CAMACQ_MAIN_REG_JPEG_QUALITY_NORMAL             "reg_main_jpeg_quality_normal"

// Format
#define CAMACQ_MAIN_REG_FMT_YUV422 	                    "reg_main_fmt_yuv422"
#define CAMACQ_MAIN_REG_FMT_JPG		                    "reg_main_fmt_jpg"

// NEW
#define CAMACQ_MAIN_REG_SLEEP                           "reg_main_sleep"
#define CAMACQ_MAIN_REG_WAKEUP                          "reg_main_wakeup"

#define CAMACQ_MAIN_REG_DTP_ON                          "reg_main_dtp_on"
#define CAMACQ_MAIN_REG_DTP_OFF                         "reg_main_dtp_off"

#define CAMACQ_MAIN_REG_1_28x_ZOOM_0                          "reg_main_1_28x_zoom_0"
#define CAMACQ_MAIN_REG_1_28x_ZOOM_1                          "reg_main_1_28x_zoom_1"
#define CAMACQ_MAIN_REG_1_28x_ZOOM_2                          "reg_main_1_28x_zoom_2"
#define CAMACQ_MAIN_REG_1_28x_ZOOM_3                          "reg_main_1_28x_zoom_3"
#define CAMACQ_MAIN_REG_1_28x_ZOOM_4                          "reg_main_1_28x_zoom_4"
#define CAMACQ_MAIN_REG_1_28x_ZOOM_5                          "reg_main_1_28x_zoom_5"
#define CAMACQ_MAIN_REG_1_28x_ZOOM_6                          "reg_main_1_28x_zoom_6"
#define CAMACQ_MAIN_REG_1_28x_ZOOM_7                          "reg_main_1_28x_zoom_7"
#define CAMACQ_MAIN_REG_1_28x_ZOOM_8                          "reg_main_1_28x_zoom_8"

#define CAMACQ_MAIN_REG_1_60x_ZOOM_0                          "reg_main_1_60x_zoom_0"
#define CAMACQ_MAIN_REG_1_60x_ZOOM_1                          "reg_main_1_60x_zoom_1"
#define CAMACQ_MAIN_REG_1_60x_ZOOM_2                          "reg_main_1_60x_zoom_2"
#define CAMACQ_MAIN_REG_1_60x_ZOOM_3                          "reg_main_1_60x_zoom_3"
#define CAMACQ_MAIN_REG_1_60x_ZOOM_4                          "reg_main_1_60x_zoom_4"
#define CAMACQ_MAIN_REG_1_60x_ZOOM_5                          "reg_main_1_60x_zoom_5"
#define CAMACQ_MAIN_REG_1_60x_ZOOM_6                          "reg_main_1_60x_zoom_6"
#define CAMACQ_MAIN_REG_1_60x_ZOOM_7                          "reg_main_1_60x_zoom_7"
#define CAMACQ_MAIN_REG_1_60x_ZOOM_8                          "reg_main_1_60x_zoom_8"

#define CAMACQ_MAIN_REG_2_00x_ZOOM_0                          "reg_main_2_00x_zoom_0"
#define CAMACQ_MAIN_REG_2_00x_ZOOM_1                          "reg_main_2_00x_zoom_1"
#define CAMACQ_MAIN_REG_2_00x_ZOOM_2                          "reg_main_2_00x_zoom_2"
#define CAMACQ_MAIN_REG_2_00x_ZOOM_3                          "reg_main_2_00x_zoom_3"
#define CAMACQ_MAIN_REG_2_00x_ZOOM_4                          "reg_main_2_00x_zoom_4"
#define CAMACQ_MAIN_REG_2_00x_ZOOM_5                          "reg_main_2_00x_zoom_5"
#define CAMACQ_MAIN_REG_2_00x_ZOOM_6                          "reg_main_2_00x_zoom_6"
#define CAMACQ_MAIN_REG_2_00x_ZOOM_7                          "reg_main_2_00x_zoom_7"
#define CAMACQ_MAIN_REG_2_00x_ZOOM_8                          "reg_main_2_00x_zoom_8"


#else
#define CAMACQ_MAIN_REG_PLL                     reg_main_pll
#define CAMACQ_MAIN_REG_INIT                    reg_main_init
#define CAMACQ_MAIN_REG_PREVIEW                 reg_main_preview
#define CAMACQ_MAIN_REG_SNAPSHOT                reg_main_snapshot
#define CAMACQ_MAIN_REG_MIDLIGHT                reg_main_midlight
#define CAMACQ_MAIN_REG_HIGHLIGHT               reg_main_highlight
#define CAMACQ_MAIN_REG_LOWLIGHT                reg_main_lowlight
#define CAMACQ_MAIN_REG_NIGHTSHOT_ON            reg_main_nightshot_on
#define CAMACQ_MAIN_REG_NIGHTSHOT_OFF           reg_main_nightshot_off
#define CAMACQ_MAIN_REG_NIGHTSHOT               reg_main_nightshot
#define CAMACQ_MAIN_REG_CAMCORDER_ON            reg_main_camcorder_on

#define CAMACQ_MAIN_REG_WB_AUTO                 reg_main_wb_auto
#define CAMACQ_MAIN_REG_WB_DAYLIGHT             reg_main_wb_daylight
#define CAMACQ_MAIN_REG_WB_CLOUDY               reg_main_wb_cloudy
#define CAMACQ_MAIN_REG_WB_INCANDESCENT         reg_main_wb_incandescent
#define CAMACQ_MAIN_REG_WB_FLUORESCENT          reg_main_wb_fluorescent
#define CAMACQ_MAIN_REG_WB_TWILIGHT             reg_main_wb_twilight
#define CAMACQ_MAIN_REG_WB_TUNGSTEN             reg_main_wb_tungsten
#define CAMACQ_MAIN_REG_WB_OFF                  reg_main_wb_off
#define CAMACQ_MAIN_REG_WB_HORIZON              reg_main_wb_horizon
#define CAMACQ_MAIN_REG_WB_SHADE                reg_main_wb_shade

#define CAMACQ_MAIN_REG_EFFECT_NONE             reg_main_effect_none
#define CAMACQ_MAIN_REG_EFFECT_GRAY             reg_main_effect_gray // mono, blackwhite
#define CAMACQ_MAIN_REG_EFFECT_NEGATIVE         reg_main_effect_negative
#define CAMACQ_MAIN_REG_EFFECT_SOLARIZE         reg_main_effect_solarize
#define CAMACQ_MAIN_REG_EFFECT_SEPIA            reg_main_effect_sepia
#define CAMACQ_MAIN_REG_EFFECT_POSTERIZE        reg_main_effect_posterize
#define CAMACQ_MAIN_REG_EFFECT_WHITEBOARD       reg_main_effect_whiteboard
#define CAMACQ_MAIN_REG_EFFECT_BLACKBOARD       reg_main_effect_blackboard
#define CAMACQ_MAIN_REG_EFFECT_AQUA             reg_main_effect_aqua
#define CAMACQ_MAIN_REG_EFFECT_SHARPEN          reg_main_effect_sharpen
#define CAMACQ_MAIN_REG_EFFECT_VIVID            reg_main_effect_vivid
#define CAMACQ_MAIN_REG_EFFECT_GREEN            reg_main_effect_green
#define CAMACQ_MAIN_REG_EFFECT_SKETCH           reg_main_effect_sketch

#define CAMACQ_MAIN_REG_METER_MATRIX            reg_main_meter_matrix
#define CAMACQ_MAIN_REG_METER_CW                reg_main_meter_cw
#define CAMACQ_MAIN_REG_METER_SPOT              reg_main_meter_spot

#define CAMACQ_MAIN_REG_FLIP_NONE               reg_main_flip_none
#define CAMACQ_MAIN_REG_FLIP_WATER              reg_main_flip_water
#define CAMACQ_MAIN_REG_FLIP_MIRROR             reg_main_flip_mirror
#define CAMACQ_MAIN_REG_FLIP_WATER_MIRROR       reg_main_flip_water_mirror


#define CAMACQ_MAIN_REG_FPS_FIXED_5             reg_main_fps_fixed_5
#define CAMACQ_MAIN_REG_FPS_FIXED_7             reg_main_fps_fixed_7
#define CAMACQ_MAIN_REG_FPS_FIXED_10            reg_main_fps_fixed_10
#define CAMACQ_MAIN_REG_FPS_FIXED_15            reg_main_fps_fixed_15
#define CAMACQ_MAIN_REG_FPS_FIXED_20            reg_main_fps_fixed_20
#define CAMACQ_MAIN_REG_FPS_FIXED_25            reg_main_fps_fixed_25
#define CAMACQ_MAIN_REG_FPS_FIXED_30            reg_main_fps_fixed_30
#define CAMACQ_MAIN_REG_FPS_VAR_15              reg_main_fps_var_15

#define CAMACQ_MAIN_REG_BRIGHTNESS_LEVEL_0      reg_main_brightness_level_0
#define CAMACQ_MAIN_REG_BRIGHTNESS_LEVEL_1      reg_main_brightness_level_1
#define CAMACQ_MAIN_REG_BRIGHTNESS_LEVEL_2      reg_main_brightness_level_2
#define CAMACQ_MAIN_REG_BRIGHTNESS_LEVEL_3      reg_main_brightness_level_3
#define CAMACQ_MAIN_REG_BRIGHTNESS_LEVEL_4      reg_main_brightness_level_4
#define CAMACQ_MAIN_REG_BRIGHTNESS_LEVEL_5      reg_main_brightness_level_5
#define CAMACQ_MAIN_REG_BRIGHTNESS_LEVEL_6      reg_main_brightness_level_6
#define CAMACQ_MAIN_REG_BRIGHTNESS_LEVEL_7      reg_main_brightness_level_7
#define CAMACQ_MAIN_REG_BRIGHTNESS_LEVEL_8      reg_main_brightness_level_8

#define CAMACQ_MAIN_REG_EXPCOMPENSATION_LEVEL_0 reg_main_expcompensation_level_0
#define CAMACQ_MAIN_REG_EXPCOMPENSATION_LEVEL_1 reg_main_expcompensation_level_1
#define CAMACQ_MAIN_REG_EXPCOMPENSATION_LEVEL_2 reg_main_expcompensation_level_2
#define CAMACQ_MAIN_REG_EXPCOMPENSATION_LEVEL_3 reg_main_expcompensation_level_3
#define CAMACQ_MAIN_REG_EXPCOMPENSATION_LEVEL_4 reg_main_expcompensation_level_4
#define CAMACQ_MAIN_REG_EXPCOMPENSATION_LEVEL_5 reg_main_expcompensation_level_5
#define CAMACQ_MAIN_REG_EXPCOMPENSATION_LEVEL_6 reg_main_expcompensation_level_6
#define CAMACQ_MAIN_REG_EXPCOMPENSATION_LEVEL_7 reg_main_expcompensation_level_7
#define CAMACQ_MAIN_REG_EXPCOMPENSATION_LEVEL_8 reg_main_expcompensation_level_8

#define CAMACQ_MAIN_REG_SET_AF                  reg_main_set_af  // start af
#define CAMACQ_MAIN_REG_OFF_AF                  reg_main_off_af
#define CAMACQ_MAIN_REG_CHECK_AF                reg_main_check_af
#define CAMACQ_MAIN_REG_RESET_AF                reg_main_reset_af
#define CAMACQ_MAIN_REG_CANCEL_MACRO_AF         reg_main_cancel_macro_af
#define CAMACQ_MAIN_REG_CANCEL_MANUAL_AF        reg_main_cancel_manual_af
#define CAMACQ_MAIN_REG_MANUAL_AF               reg_main_manual_af    // normal_af
#define CAMACQ_MAIN_REG_MACRO_AF                reg_main_macro_af
#define CAMACQ_MAIN_REG_RETURN_MANUAL_AF        reg_main_return_manual_af
#define CAMACQ_MAIN_REG_RETURN_MACRO_AF         reg_main_return_macro_af
#define CAMACQ_MAIN_REG_SET_AF_NLUX             reg_main_set_af_nlux
#define CAMACQ_MAIN_REG_SET_AF_LLUX             reg_main_set_af_llux
#define CAMACQ_MAIN_REG_SET_AF_NORMAL_MODE_1    reg_main_set_af_normal_mode_1
#define CAMACQ_MAIN_REG_SET_AF_NORMAL_MODE_2    reg_main_set_af_normal_mode_2
#define CAMACQ_MAIN_REG_SET_AF_NORMAL_MODE_3    reg_main_set_af_normal_mode_3
#define CAMACQ_MAIN_REG_SET_AF_MACRO_MODE_1     reg_main_set_af_macro_mode_1
#define CAMACQ_MAIN_REG_SET_AF_MACRO_MODE_2     reg_main_set_af_macro_mode_2
#define CAMACQ_MAIN_REG_SET_AF_MACRO_MODE_3     reg_main_set_af_macro_mode_3

#define CAMACQ_MAIN_REG_ISO_AUTO                reg_main_iso_auto
#define CAMACQ_MAIN_REG_ISO_50                  reg_main_iso_50
#define CAMACQ_MAIN_REG_ISO_100                 reg_main_iso_100
#define CAMACQ_MAIN_REG_ISO_200                 reg_main_iso_200
#define CAMACQ_MAIN_REG_ISO_400                 reg_main_iso_400
#define CAMACQ_MAIN_REG_ISO_800                 reg_main_iso_800
#define CAMACQ_MAIN_REG_ISO_1600                reg_main_iso_1600
#define CAMACQ_MAIN_REG_ISO_3200                reg_main_iso_3200

#define CAMACQ_MAIN_REG_SCENE_AUTO              reg_main_scene_auto  // auto, off
#define CAMACQ_MAIN_REG_SCENE_NIGHT             reg_main_scene_night
#define CAMACQ_MAIN_REG_SCENE_LANDSCAPE         reg_main_scene_landscape
#define CAMACQ_MAIN_REG_SCENE_SUNSET            reg_main_scene_sunset
#define CAMACQ_MAIN_REG_SCENE_PORTRAIT          reg_main_scene_portrait
#define CAMACQ_MAIN_REG_SCENE_SUNRISE           reg_main_scene_sunrise    // dawn
#define CAMACQ_MAIN_REG_SCENE_INDOOR            reg_main_scene_indoor
#define CAMACQ_MAIN_REG_SCENE_PARTY             reg_main_scene_party
#define CAMACQ_MAIN_REG_SCENE_SPORTS            reg_main_scene_sports
#define CAMACQ_MAIN_REG_SCENE_BEACH             reg_main_scene_beach
#define CAMACQ_MAIN_REG_SCENE_SNOW              reg_main_scene_snow
#define CAMACQ_MAIN_REG_SCENE_FALLCOLOR         reg_main_scene_fallcolor
#define CAMACQ_MAIN_REG_SCENE_FIREWORKS         reg_main_scene_fireworks
#define CAMACQ_MAIN_REG_SCENE_CANDLELIGHT       reg_main_scene_candlelight
#define CAMACQ_MAIN_REG_SCENE_AGAINSTLIGHT      reg_main_scene_againstlight  // backlight
#define CAMACQ_MAIN_REG_SCENE_TEXT              reg_main_scene_text
#define CAMACQ_MAIN_REG_SCENE_AQUA              reg_main_scene_aqua

#define CAMACQ_MAIN_REG_ADJUST_CONTRAST_M2              reg_main_adjust_contrast_m2
#define CAMACQ_MAIN_REG_ADJUST_CONTRAST_M1              reg_main_adjust_contrast_m1
#define CAMACQ_MAIN_REG_ADJUST_CONTRAST_DEFAULT         reg_main_adjust_contrast_default
#define CAMACQ_MAIN_REG_ADJUST_CONTRAST_P1              reg_main_adjust_contrast_p1
#define CAMACQ_MAIN_REG_ADJUST_CONTRAST_P2              reg_main_adjust_contrast_p2

#define CAMACQ_MAIN_REG_ADJUST_SHARPNESS_M2             reg_main_adjust_sharpness_m2
#define CAMACQ_MAIN_REG_ADJUST_SHARPNESS_M1             reg_main_adjust_sharpness_m1
#define CAMACQ_MAIN_REG_ADJUST_SHARPNESS_DEFAULT        reg_main_adjust_sharpness_default
#define CAMACQ_MAIN_REG_ADJUST_SHARPNESS_P1             reg_main_adjust_sharpness_p1
#define CAMACQ_MAIN_REG_ADJUST_SHARPNESS_P2             reg_main_adjust_sharpness_p2

#define CAMACQ_MAIN_REG_ADJUST_SATURATION_M2            reg_main_adjust_saturation_m2
#define CAMACQ_MAIN_REG_ADJUST_SATURATION_M1            reg_main_adjust_saturation_m1
#define CAMACQ_MAIN_REG_ADJUST_SATURATION_DEFAULT       reg_main_adjust_saturation_default
#define CAMACQ_MAIN_REG_ADJUST_SATURATION_P1            reg_main_adjust_saturation_p1
#define CAMACQ_MAIN_REG_ADJUST_SATURATION_P2            reg_main_adjust_saturation_p2

// camcorder-preview
#define CAMACQ_MAIN_REG_QCIF                    reg_main_qcif
#define CAMACQ_MAIN_REG_QVGA                    reg_main_qvga
#define CAMACQ_MAIN_REG_CIF                     reg_main_cif
#define CAMACQ_MAIN_REG_VGA                     reg_main_vga
#define CAMACQ_MAIN_REG_1080P		            reg_main_1080P
#define CAMACQ_MAIN_REG_720P			        reg_main_720P
#define CAMACQ_MAIN_REG_800_480			        reg_main_800_480
#define CAMACQ_MAIN_REG_720_480			        reg_main_720_480
#define CAMACQ_MAIN_REG_NOT                     reg_main_not

#define CAMACQ_MAIN_REG_JPEG_5M                 reg_main_jpeg_5m        //2560X1920
#define CAMACQ_MAIN_REG_JPEG_5M_2               reg_main_jpeg_5m_2      // 2592X1944
#define CAMACQ_MAIN_REG_JPEG_W4M                 reg_main_jpeg_w4m      // 2560x1536
#define CAMACQ_MAIN_REG_JPEG_3M                 reg_main_jpeg_3m        // QXGA 2048X1536
#define CAMACQ_MAIN_REG_JPEG_2M                 reg_main_jpeg_2m        // UXGA 1600x1200
#define CAMACQ_MAIN_REG_JPEG_W1_5M               reg_main_jpeg_w1_5m    // 1280x960
#define CAMACQ_MAIN_REG_JPEG_1M                 reg_main_jpeg_1m
#define CAMACQ_MAIN_REG_JPEG_VGA                reg_main_jpeg_vga   //640x480
#define CAMACQ_MAIN_REG_JPEG_WQVGA              reg_main_jpeg_wqvga //420x240
#define CAMACQ_MAIN_REG_JPEG_QVGA               reg_main_jpeg_qvga  //320x240

#define CAMACQ_MAIN_REG_FLICKER_DISABLED        reg_main_flicker_disabled
#define CAMACQ_MAIN_REG_FLICKER_50HZ            reg_main_flicker_50hz
#define CAMACQ_MAIN_REG_FLICKER_60HZ            reg_main_flicker_60hz
#define CAMACQ_MAIN_REG_FLICKER_AUTO            reg_main_flicker_auto

#define CAMACQ_MAIN_REG_AE_LOCK                 reg_main_ae_lock
#define CAMACQ_MAIN_REG_AE_UNLOCK               reg_main_ae_unlock               
#define CAMACQ_MAIN_REG_AWB_LOCK                reg_main_awb_lock
#define CAMACQ_MAIN_REG_AWB_UNLOCK              reg_main_awb_unlock

#define CAMACQ_MAIN_REG_PRE_FLASH_START         reg_main_pre_flash_start
#define CAMACQ_MAIN_REG_PRE_FLASH_END           reg_main_pre_flash_end
#define CAMACQ_MAIN_REG_MAIN_FLASH_START        reg_main_main_flash_start
#define CAMACQ_MAIN_REG_MAIN_FLASH_END          reg_main_main_flash_end

#define CAMACQ_MAIN_REG_FLASH_AE_SET            reg_main_flash_ae_set
#define CAMACQ_MAIN_REG_FLASH_AE_CLEAR          reg_main_flash_ae_clear

//denis_flash_2
#define CAMACQ_MAIN_REG_FLASH_ON_SET            reg_main_flash_on_set     

// image quality
#define CAMACQ_MAIN_REG_JPEG_QUALITY_SUPERFINE reg_main_jpeg_quality_superfine
#define CAMACQ_MAIN_REG_JPEG_QUALITY_FINE      reg_main_jpeg_quality_fine
#define CAMACQ_MAIN_REG_JPEG_QUALITY_NORMAL    reg_main_jpeg_quality_normal

// Format
#define CAMACQ_MAIN_REG_FMT_YUV422 	            reg_main_fmt_yuv422
#define CAMACQ_MAIN_REG_FMT_JPG		            reg_main_fmt_jpg

// NEW
#define CAMACQ_MAIN_REG_SLEEP                   reg_main_sleep
#define CAMACQ_MAIN_REG_WAKEUP                  reg_main_wakeup

#define CAMACQ_MAIN_REG_DTP_ON                  reg_main_dtp_on
#define CAMACQ_MAIN_REG_DTP_OFF                 reg_main_dtp_off

#define CAMACQ_MAIN_REG_1_28x_ZOOM_0                          reg_main_1_28x_zoom_0
#define CAMACQ_MAIN_REG_1_28x_ZOOM_1                          reg_main_1_28x_zoom_1
#define CAMACQ_MAIN_REG_1_28x_ZOOM_2                          reg_main_1_28x_zoom_2
#define CAMACQ_MAIN_REG_1_28x_ZOOM_3                          reg_main_1_28x_zoom_3
#define CAMACQ_MAIN_REG_1_28x_ZOOM_4                          reg_main_1_28x_zoom_4
#define CAMACQ_MAIN_REG_1_28x_ZOOM_5                          reg_main_1_28x_zoom_5
#define CAMACQ_MAIN_REG_1_28x_ZOOM_6                          reg_main_1_28x_zoom_6
#define CAMACQ_MAIN_REG_1_28x_ZOOM_7                          reg_main_1_28x_zoom_7
#define CAMACQ_MAIN_REG_1_28x_ZOOM_8                          reg_main_1_28x_zoom_8

#define CAMACQ_MAIN_REG_1_60x_ZOOM_0                          reg_main_1_60x_zoom_0
#define CAMACQ_MAIN_REG_1_60x_ZOOM_1                          reg_main_1_60x_zoom_1
#define CAMACQ_MAIN_REG_1_60x_ZOOM_2                          reg_main_1_60x_zoom_2
#define CAMACQ_MAIN_REG_1_60x_ZOOM_3                          reg_main_1_60x_zoom_3
#define CAMACQ_MAIN_REG_1_60x_ZOOM_4                          reg_main_1_60x_zoom_4
#define CAMACQ_MAIN_REG_1_60x_ZOOM_5                          reg_main_1_60x_zoom_5
#define CAMACQ_MAIN_REG_1_60x_ZOOM_6                          reg_main_1_60x_zoom_6
#define CAMACQ_MAIN_REG_1_60x_ZOOM_7                          reg_main_1_60x_zoom_7
#define CAMACQ_MAIN_REG_1_60x_ZOOM_8                          reg_main_1_60x_zoom_8

#define CAMACQ_MAIN_REG_2_00x_ZOOM_0                          reg_main_2_00x_zoom_0
#define CAMACQ_MAIN_REG_2_00x_ZOOM_1                          reg_main_2_00x_zoom_1
#define CAMACQ_MAIN_REG_2_00x_ZOOM_2                          reg_main_2_00x_zoom_2
#define CAMACQ_MAIN_REG_2_00x_ZOOM_3                          reg_main_2_00x_zoom_3
#define CAMACQ_MAIN_REG_2_00x_ZOOM_4                          reg_main_2_00x_zoom_4
#define CAMACQ_MAIN_REG_2_00x_ZOOM_5                          reg_main_2_00x_zoom_5
#define CAMACQ_MAIN_REG_2_00x_ZOOM_6                          reg_main_2_00x_zoom_6
#define CAMACQ_MAIN_REG_2_00x_ZOOM_7                          reg_main_2_00x_zoom_7
#define CAMACQ_MAIN_REG_2_00x_ZOOM_8                          reg_main_2_00x_zoom_8

#endif

/* Enumeration */

/* Global Value */

GLOBAL const U16 reg_main_sleep[][2]
#if defined(_CAMACQ_API_C_)
={

CAMACQ_MAIN_BTM_OF_DATA
}
#endif /* _CAMACQ_API_C_ */
;


GLOBAL const U16 reg_main_wakeup[][2] 
#if defined(_CAMACQ_API_C_)
={

CAMACQ_MAIN_BTM_OF_DATA
}
#endif /* _CAMACQ_API_C_ */
;

GLOBAL const U16 reg_main_pll[][2] 
#if defined(_CAMACQ_API_C_)
={

CAMACQ_MAIN_BTM_OF_DATA
}
#endif /* _CAMACQ_API_C_ */
;

GLOBAL const U16 reg_main_dtp_on[][2] 
#if defined(_CAMACQ_API_C_)
={
//****************************************/
{0xFCFC, 0xD000},
//****************************************/
//===================================================================
// History
//===================================================================
//20100717 : 1st release
//20100806 : 2nd release for EVT0.1
//20101028 : 3rd release for EVT1
//WRITE #awbb_otp_disable  0000 //awb otp use
//==========================================================================================
//-->The below registers are for FACTORY ONLY. if you change them without prior notification,
//   YOU are RESPONSIBLE for the FAILURE that will happen in the future.
//==========================================================================================
//===================================================================
// Reset & Trap and Patch
//===================================================================

// Start of Trap and Patch
//  2010-08-11 13:53:35
{0x0010, 0x0001},
{0x1030, 0x0000},
{0x0014, 0x0001},

{0xFFFF, 0x000A},						//Delay 10ms

// Start of Patch data
{0x0028, 0x7000},
{0x002A, 0x352C},
{0x0F12, 0xB510},    // 7000352C 
{0x0F12, 0x4A2C},    // 7000352E 
{0x0F12, 0x213F},    // 70003530 
{0x0F12, 0x482C},    // 70003532 
{0x0F12, 0x4B2C},    // 70003534 
{0x0F12, 0x2400},    // 70003536 
{0x0F12, 0x801C},    // 70003538 
{0x0F12, 0xC004},    // 7000353A 
{0x0F12, 0x6001},    // 7000353C 
{0x0F12, 0x492B},    // 7000353E 
{0x0F12, 0x482B},    // 70003540 
{0x0F12, 0xF000},    // 70003542 
{0x0F12, 0xFBE9},    // 70003544 
{0x0F12, 0x2115},    // 70003546 
{0x0F12, 0x482A},    // 70003548 
{0x0F12, 0x01C9},    // 7000354A 
{0x0F12, 0xF000},    // 7000354C 
{0x0F12, 0xF88E},    // 7000354E 
{0x0F12, 0x4828},    // 70003550 
{0x0F12, 0x210B},    // 70003552 
{0x0F12, 0x0189},    // 70003554 
{0x0F12, 0x3018},    // 70003556 
{0x0F12, 0xF000},    // 70003558 
{0x0F12, 0xF888},    // 7000355A 
{0x0F12, 0x4825},    // 7000355C 
{0x0F12, 0x4926},    // 7000355E 
{0x0F12, 0x300C},    // 70003560 
{0x0F12, 0xF000},    // 70003562 
{0x0F12, 0xF883},    // 70003564 
{0x0F12, 0x4823},    // 70003566 
{0x0F12, 0x4924},    // 70003568 
{0x0F12, 0x3010},    // 7000356A 
{0x0F12, 0xF000},    // 7000356C 
{0x0F12, 0xF87E},    // 7000356E 
{0x0F12, 0x4923},    // 70003570 
{0x0F12, 0x4824},    // 70003572 
{0x0F12, 0xF000},    // 70003574 
{0x0F12, 0xFBD0},    // 70003576 
{0x0F12, 0x4923},    // 70003578 
{0x0F12, 0x4824},    // 7000357A 
{0x0F12, 0xF000},    // 7000357C 
{0x0F12, 0xFBCC},    // 7000357E 
{0x0F12, 0x4923},    // 70003580 
{0x0F12, 0x4824},    // 70003582 
{0x0F12, 0xF000},    // 70003584 
{0x0F12, 0xFBC8},    // 70003586 
{0x0F12, 0x4923},    // 70003588 
{0x0F12, 0x4824},    // 7000358A 
{0x0F12, 0xF000},    // 7000358C 
{0x0F12, 0xFBC4},    // 7000358E 
{0x0F12, 0x4923},    // 70003590 
{0x0F12, 0x4824},    // 70003592 
{0x0F12, 0xF000},    // 70003594 
{0x0F12, 0xFBC0},    // 70003596 
{0x0F12, 0x4823},    // 70003598 
{0x0F12, 0x4924},    // 7000359A 
{0x0F12, 0x6408},    // 7000359C 
{0x0F12, 0x4924},    // 7000359E 
{0x0F12, 0x4824},    // 700035A0 
{0x0F12, 0xF000},    // 700035A2 
{0x0F12, 0xFBB9},    // 700035A4 
{0x0F12, 0x4924},    // 700035A6 
{0x0F12, 0x4824},    // 700035A8 
{0x0F12, 0xF000},    // 700035AA 
{0x0F12, 0xFBB5},    // 700035AC 
{0x0F12, 0x4924},    // 700035AE 
{0x0F12, 0x4824},    // 700035B0 
{0x0F12, 0xF000},    // 700035B2 
{0x0F12, 0xFBB1},    // 700035B4 
{0x0F12, 0x4924},    // 700035B6 
{0x0F12, 0x4824},    // 700035B8 
{0x0F12, 0xF000},    // 700035BA 
{0x0F12, 0xFBAD},    // 700035BC 
{0x0F12, 0x4924},    // 700035BE 
{0x0F12, 0x4824},    // 700035C0 
{0x0F12, 0xF000},    // 700035C2 
{0x0F12, 0xFBA9},    // 700035C4 
{0x0F12, 0x4824},    // 700035C6 
{0x0F12, 0x8104},    // 700035C8 
{0x0F12, 0x4924},    // 700035CA 
{0x0F12, 0x4824},    // 700035CC 
{0x0F12, 0xF000},    // 700035CE 
{0x0F12, 0xFBA3},    // 700035D0 
{0x0F12, 0x4924},    // 700035D2 
{0x0F12, 0x4824},    // 700035D4 
{0x0F12, 0xF000},    // 700035D6 
{0x0F12, 0xFB9F},    // 700035D8 
{0x0F12, 0xBC10},    // 700035DA 
{0x0F12, 0xBC08},    // 700035DC 
{0x0F12, 0x4718},    // 700035DE 
{0x0F12, 0x00BA},    // 700035E0 
{0x0F12, 0x5CC1},    // 700035E2 
{0x0F12, 0x1C08},    // 700035E4 
{0x0F12, 0x7000},    // 700035E6 
{0x0F12, 0x3290},    // 700035E8 
{0x0F12, 0x7000},    // 700035EA 
{0x0F12, 0x368B},    // 700035EC 
{0x0F12, 0x7000},    // 700035EE 
{0x0F12, 0xD9E7},    // 700035F0 
{0x0F12, 0x0000},    // 700035F2 
{0x0F12, 0x6FC0},    // 700035F4 
{0x0F12, 0x0000},    // 700035F6 
{0x0F12, 0x0A91},    // 700035F8 
{0x0F12, 0x0000},    // 700035FA 
{0x0F12, 0x02C9},    // 700035FC 
{0x0F12, 0x0000},    // 700035FE 
{0x0F12, 0x36C3},    // 70003600 
{0x0F12, 0x7000},    // 70003602 
{0x0F12, 0xA607},    // 70003604 
{0x0F12, 0x0000},    // 70003606 
{0x0F12, 0x3733},    // 70003608 
{0x0F12, 0x7000},    // 7000360A 
{0x0F12, 0x7C0D},    // 7000360C 
{0x0F12, 0x0000},    // 7000360E 
{0x0F12, 0x374F},    // 70003610 
{0x0F12, 0x7000},    // 70003612 
{0x0F12, 0x7C2B},    // 70003614 
{0x0F12, 0x0000},    // 70003616 
{0x0F12, 0x376B},    // 70003618 
{0x0F12, 0x7000},    // 7000361A 
{0x0F12, 0x9E89},    // 7000361C 
{0x0F12, 0x0000},    // 7000361E 
{0x0F12, 0x394F},    // 70003620 
{0x0F12, 0x7000},    // 70003622 
{0x0F12, 0x395D},    // 70003624 
{0x0F12, 0x0000},    // 70003626 
{0x0F12, 0x39DB},    // 70003628 
{0x0F12, 0x7000},    // 7000362A 
{0x0F12, 0x0000},    // 7000362C 
{0x0F12, 0x7000},    // 7000362E 
{0x0F12, 0x3B01},    // 70003630 
{0x0F12, 0x7000},    // 70003632 
{0x0F12, 0xF903},    // 70003634 
{0x0F12, 0x0000},    // 70003636 
{0x0F12, 0x37B3},    // 70003638 
{0x0F12, 0x7000},    // 7000363A 
{0x0F12, 0x495F},    // 7000363C 
{0x0F12, 0x0000},    // 7000363E 
{0x0F12, 0x380D},    // 70003640 
{0x0F12, 0x7000},    // 70003642 
{0x0F12, 0xE421},    // 70003644 
{0x0F12, 0x0000},    // 70003646 
{0x0F12, 0x38BB},    // 70003648 
{0x0F12, 0x7000},    // 7000364A 
{0x0F12, 0x216D},    // 7000364C 
{0x0F12, 0x0000},    // 7000364E 
{0x0F12, 0x392F},    // 70003650 
{0x0F12, 0x7000},    // 70003652 
{0x0F12, 0x0179},    // 70003654 
{0x0F12, 0x0001},    // 70003656 
{0x0F12, 0x3FC8},    // 70003658 
{0x0F12, 0x7000},    // 7000365A 
{0x0F12, 0x3CE3},    // 7000365C 
{0x0F12, 0x7000},    // 7000365E 
{0x0F12, 0x04C9},    // 70003660 
{0x0F12, 0x0000},    // 70003662 
{0x0F12, 0x3C2F},    // 70003664 
{0x0F12, 0x7000},    // 70003666 
{0x0F12, 0x5027},    // 70003668 
{0x0F12, 0x0000},    // 7000366A 
{0x0F12, 0xB570},    // 7000366C 
{0x0F12, 0x000D},    // 7000366E 
{0x0F12, 0x4CFF},    // 70003670 
{0x0F12, 0x8821},    // 70003672 
{0x0F12, 0xF000},    // 70003674 
{0x0F12, 0xFB58},    // 70003676 
{0x0F12, 0x8820},    // 70003678 
{0x0F12, 0x4AFE},    // 7000367A 
{0x0F12, 0x0081},    // 7000367C 
{0x0F12, 0x5055},    // 7000367E 
{0x0F12, 0x1C40},    // 70003680 
{0x0F12, 0x8020},    // 70003682 
{0x0F12, 0xBC70},    // 70003684 
{0x0F12, 0xBC08},    // 70003686 
{0x0F12, 0x4718},    // 70003688 
{0x0F12, 0x6801},    // 7000368A 
{0x0F12, 0x0409},    // 7000368C 
{0x0F12, 0x0C09},    // 7000368E 
{0x0F12, 0x6840},    // 70003690 
{0x0F12, 0x0400},    // 70003692 
{0x0F12, 0x0C00},    // 70003694 
{0x0F12, 0x4AF8},    // 70003696 
{0x0F12, 0x8992},    // 70003698 
{0x0F12, 0x2A00},    // 7000369A 
{0x0F12, 0xD00D},    // 7000369C 
{0x0F12, 0x2300},    // 7000369E 
{0x0F12, 0x1A80},    // 700036A0 
{0x0F12, 0xD400},    // 700036A2 
{0x0F12, 0x0003},    // 700036A4 
{0x0F12, 0x0418},    // 700036A6 
{0x0F12, 0x0C00},    // 700036A8 
{0x0F12, 0x4BF4},    // 700036AA 
{0x0F12, 0x1851},    // 700036AC 
{0x0F12, 0x891B},    // 700036AE 
{0x0F12, 0x428B},    // 700036B0 
{0x0F12, 0xD300},    // 700036B2 
{0x0F12, 0x000B},    // 700036B4 
{0x0F12, 0x0419},    // 700036B6 
{0x0F12, 0x0C09},    // 700036B8 
{0x0F12, 0x4AF1},    // 700036BA 
{0x0F12, 0x8151},    // 700036BC 
{0x0F12, 0x8190},    // 700036BE 
{0x0F12, 0x4770},    // 700036C0 
{0x0F12, 0xB510},    // 700036C2 
{0x0F12, 0x48EF},    // 700036C4 
{0x0F12, 0x4CF0},    // 700036C6 
{0x0F12, 0x88C1},    // 700036C8 
{0x0F12, 0x8061},    // 700036CA 
{0x0F12, 0x2101},    // 700036CC 
{0x0F12, 0x8021},    // 700036CE 
{0x0F12, 0x8840},    // 700036D0 
{0x0F12, 0xF000},    // 700036D2 
{0x0F12, 0xFB31},    // 700036D4 
{0x0F12, 0x88E0},    // 700036D6 
{0x0F12, 0x4AEC},    // 700036D8 
{0x0F12, 0x2800},    // 700036DA 
{0x0F12, 0xD003},    // 700036DC 
{0x0F12, 0x49EC},    // 700036DE 
{0x0F12, 0x8849},    // 700036E0 
{0x0F12, 0x2900},    // 700036E2 
{0x0F12, 0xD009},    // 700036E4 
{0x0F12, 0x2001},    // 700036E6 
{0x0F12, 0x03C0},    // 700036E8 
{0x0F12, 0x8050},    // 700036EA 
{0x0F12, 0x80D0},    // 700036EC 
{0x0F12, 0x2000},    // 700036EE 
{0x0F12, 0x8090},    // 700036F0 
{0x0F12, 0x8110},    // 700036F2 
{0x0F12, 0xBC10},    // 700036F4 
{0x0F12, 0xBC08},    // 700036F6 
{0x0F12, 0x4718},    // 700036F8 
{0x0F12, 0x8050},    // 700036FA 
{0x0F12, 0x8920},    // 700036FC 
{0x0F12, 0x80D0},    // 700036FE 
{0x0F12, 0x8960},    // 70003700 
{0x0F12, 0x0400},    // 70003702 
{0x0F12, 0x1400},    // 70003704 
{0x0F12, 0x8090},    // 70003706 
{0x0F12, 0x89A1},    // 70003708 
{0x0F12, 0x0409},    // 7000370A 
{0x0F12, 0x1409},    // 7000370C 
{0x0F12, 0x8111},    // 7000370E 
{0x0F12, 0x89E3},    // 70003710 
{0x0F12, 0x8A24},    // 70003712 
{0x0F12, 0x2B00},    // 70003714 
{0x0F12, 0xD104},    // 70003716 
{0x0F12, 0x17C3},    // 70003718 
{0x0F12, 0x0F5B},    // 7000371A 
{0x0F12, 0x1818},    // 7000371C 
{0x0F12, 0x10C0},    // 7000371E 
{0x0F12, 0x8090},    // 70003720 
{0x0F12, 0x2C00},    // 70003722 
{0x0F12, 0xD1E6},    // 70003724 
{0x0F12, 0x17C8},    // 70003726 
{0x0F12, 0x0F40},    // 70003728 
{0x0F12, 0x1840},    // 7000372A 
{0x0F12, 0x10C0},    // 7000372C 
{0x0F12, 0x8110},    // 7000372E 
{0x0F12, 0xE7E0},    // 70003730 
{0x0F12, 0xB510},    // 70003732 
{0x0F12, 0x0004},    // 70003734 
{0x0F12, 0x49D5},    // 70003736 
{0x0F12, 0x2204},    // 70003738 
{0x0F12, 0x6820},    // 7000373A 
{0x0F12, 0x5E8A},    // 7000373C 
{0x0F12, 0x0140},    // 7000373E 
{0x0F12, 0x1A80},    // 70003740 
{0x0F12, 0x0280},    // 70003742 
{0x0F12, 0x8849},    // 70003744 
{0x0F12, 0xF000},    // 70003746 
{0x0F12, 0xFAFF},    // 70003748 
{0x0F12, 0x6020},    // 7000374A 
{0x0F12, 0xE7D2},    // 7000374C 
{0x0F12, 0xB510},    // 7000374E 
{0x0F12, 0x0004},    // 70003750 
{0x0F12, 0x49CE},    // 70003752 
{0x0F12, 0x2208},    // 70003754 
{0x0F12, 0x6820},    // 70003756 
{0x0F12, 0x5E8A},    // 70003758 
{0x0F12, 0x0140},    // 7000375A 
{0x0F12, 0x1A80},    // 7000375C 
{0x0F12, 0x0280},    // 7000375E 
{0x0F12, 0x88C9},    // 70003760 
{0x0F12, 0xF000},    // 70003762 
{0x0F12, 0xFAF1},    // 70003764 
{0x0F12, 0x6020},    // 70003766 
{0x0F12, 0xE7C4},    // 70003768 
{0x0F12, 0xB510},    // 7000376A 
{0x0F12, 0x0004},    // 7000376C 
{0x0F12, 0x49C7},    // 7000376E 
{0x0F12, 0x48C8},    // 70003770 
{0x0F12, 0x884A},    // 70003772 
{0x0F12, 0x8B43},    // 70003774 
{0x0F12, 0x435A},    // 70003776 
{0x0F12, 0x2304},    // 70003778 
{0x0F12, 0x5ECB},    // 7000377A 
{0x0F12, 0x0A92},    // 7000377C 
{0x0F12, 0x18D2},    // 7000377E 
{0x0F12, 0x02D2},    // 70003780 
{0x0F12, 0x0C12},    // 70003782 
{0x0F12, 0x88CB},    // 70003784 
{0x0F12, 0x8B80},    // 70003786 
{0x0F12, 0x4343},    // 70003788 
{0x0F12, 0x0A98},    // 7000378A 
{0x0F12, 0x2308},    // 7000378C 
{0x0F12, 0x5ECB},    // 7000378E 
{0x0F12, 0x18C0},    // 70003790 
{0x0F12, 0x02C0},    // 70003792 
{0x0F12, 0x0C00},    // 70003794 
{0x0F12, 0x0411},    // 70003796 
{0x0F12, 0x0400},    // 70003798 
{0x0F12, 0x1409},    // 7000379A 
{0x0F12, 0x1400},    // 7000379C 
{0x0F12, 0x1A08},    // 7000379E 
{0x0F12, 0x49BC},    // 700037A0 
{0x0F12, 0x3980},    // 700037A2 
{0x0F12, 0x6348},    // 700037A4 
{0x0F12, 0x0020},    // 700037A6 
{0x0F12, 0xC80F},    // 700037A8 
{0x0F12, 0xF000},    // 700037AA 
{0x0F12, 0xFAD3},    // 700037AC 
{0x0F12, 0x6020},    // 700037AE 
{0x0F12, 0xE7A0},    // 700037B0 
{0x0F12, 0xB510},    // 700037B2 
{0x0F12, 0x4CB8},    // 700037B4 
{0x0F12, 0x48B9},    // 700037B6 
{0x0F12, 0x78A1},    // 700037B8 
{0x0F12, 0x2900},    // 700037BA 
{0x0F12, 0xD101},    // 700037BC 
{0x0F12, 0x87C1},    // 700037BE 
{0x0F12, 0xE004},    // 700037C0 
{0x0F12, 0x7AE1},    // 700037C2 
{0x0F12, 0x2900},    // 700037C4 
{0x0F12, 0xD001},    // 700037C6 
{0x0F12, 0x2101},    // 700037C8 
{0x0F12, 0x87C1},    // 700037CA 
{0x0F12, 0xF000},    // 700037CC 
{0x0F12, 0xFACA},    // 700037CE 
{0x0F12, 0x49B3},    // 700037D0 
{0x0F12, 0x8B08},    // 700037D2 
{0x0F12, 0x06C2},    // 700037D4 
{0x0F12, 0xD50A},    // 700037D6 
{0x0F12, 0x7AA2},    // 700037D8 
{0x0F12, 0x0652},    // 700037DA 
{0x0F12, 0xD507},    // 700037DC 
{0x0F12, 0x2210},    // 700037DE 
{0x0F12, 0x4390},    // 700037E0 
{0x0F12, 0x8308},    // 700037E2 
{0x0F12, 0x48AF},    // 700037E4 
{0x0F12, 0x7AE1},    // 700037E6 
{0x0F12, 0x6B00},    // 700037E8 
{0x0F12, 0xF000},    // 700037EA 
{0x0F12, 0xFAC3},    // 700037EC 
{0x0F12, 0x48A2},    // 700037EE 
{0x0F12, 0x89C0},    // 700037F0 
{0x0F12, 0x2801},    // 700037F2 
{0x0F12, 0xD109},    // 700037F4 
{0x0F12, 0x78A0},    // 700037F6 
{0x0F12, 0x2800},    // 700037F8 
{0x0F12, 0xD006},    // 700037FA 
{0x0F12, 0x7AE0},    // 700037FC 
{0x0F12, 0x2800},    // 700037FE 
{0x0F12, 0xD003},    // 70003800 
{0x0F12, 0x7AA0},    // 70003802 
{0x0F12, 0x2140},    // 70003804 
{0x0F12, 0x4308},    // 70003806 
{0x0F12, 0x72A0},    // 70003808 
{0x0F12, 0xE773},    // 7000380A 
{0x0F12, 0xB570},    // 7000380C 
{0x0F12, 0x4DA4},    // 7000380E 
{0x0F12, 0x4CA4},    // 70003810 
{0x0F12, 0x8B28},    // 70003812 
{0x0F12, 0x0701},    // 70003814 
{0x0F12, 0xD507},    // 70003816 
{0x0F12, 0x2108},    // 70003818 
{0x0F12, 0x4388},    // 7000381A 
{0x0F12, 0x8328},    // 7000381C 
{0x0F12, 0x49A2},    // 7000381E 
{0x0F12, 0x6B20},    // 70003820 
{0x0F12, 0x6B89},    // 70003822 
{0x0F12, 0xF000},    // 70003824 
{0x0F12, 0xFAAE},    // 70003826 
{0x0F12, 0x8B28},    // 70003828 
{0x0F12, 0x06C1},    // 7000382A 
{0x0F12, 0xD50A},    // 7000382C 
{0x0F12, 0x499A},    // 7000382E 
{0x0F12, 0x7A8A},    // 70003830 
{0x0F12, 0x0652},    // 70003832 
{0x0F12, 0xD406},    // 70003834 
{0x0F12, 0x2210},    // 70003836 
{0x0F12, 0x4390},    // 70003838 
{0x0F12, 0x8328},    // 7000383A 
{0x0F12, 0x7AC9},    // 7000383C 
{0x0F12, 0x6B20},    // 7000383E 
{0x0F12, 0xF000},    // 70003840 
{0x0F12, 0xFA98},    // 70003842 
{0x0F12, 0xE71E},    // 70003844 
{0x0F12, 0xB5F8},    // 70003846 
{0x0F12, 0x4C98},    // 70003848 
{0x0F12, 0x26FF},    // 7000384A 
{0x0F12, 0x8820},    // 7000384C 
{0x0F12, 0x4D98},    // 7000384E 
{0x0F12, 0x1C76},    // 70003850 
{0x0F12, 0x2702},    // 70003852 
{0x0F12, 0x2803},    // 70003854 
{0x0F12, 0xD112},    // 70003856 
{0x0F12, 0x8860},    // 70003858 
{0x0F12, 0x2800},    // 7000385A 
{0x0F12, 0xD10F},    // 7000385C 
{0x0F12, 0x88E0},    // 7000385E 
{0x0F12, 0x2800},    // 70003860 
{0x0F12, 0xD10C},    // 70003862 
{0x0F12, 0xF000},    // 70003864 
{0x0F12, 0xFA96},    // 70003866 
{0x0F12, 0x2800},    // 70003868 
{0x0F12, 0xD008},    // 7000386A 
{0x0F12, 0x8F28},    // 7000386C 
{0x0F12, 0x2800},    // 7000386E 
{0x0F12, 0xD001},    // 70003870 
{0x0F12, 0x80E6},    // 70003872 
{0x0F12, 0x80A7},    // 70003874 
{0x0F12, 0x2001},    // 70003876 
{0x0F12, 0x7260},    // 70003878 
{0x0F12, 0xF000},    // 7000387A 
{0x0F12, 0xFA93},    // 7000387C 
{0x0F12, 0x8820},    // 7000387E 
{0x0F12, 0x2802},    // 70003880 
{0x0F12, 0xD10E},    // 70003882 
{0x0F12, 0x8860},    // 70003884 
{0x0F12, 0x2800},    // 70003886 
{0x0F12, 0xD10B},    // 70003888 
{0x0F12, 0x88E0},    // 7000388A 
{0x0F12, 0x2800},    // 7000388C 
{0x0F12, 0xD108},    // 7000388E 
{0x0F12, 0x8F28},    // 70003890 
{0x0F12, 0x2800},    // 70003892 
{0x0F12, 0xD001},    // 70003894 
{0x0F12, 0x80E6},    // 70003896 
{0x0F12, 0x80A7},    // 70003898 
{0x0F12, 0x2001},    // 7000389A 
{0x0F12, 0x7260},    // 7000389C 
{0x0F12, 0xF000},    // 7000389E 
{0x0F12, 0xFA81},    // 700038A0 
{0x0F12, 0x88E0},    // 700038A2 
{0x0F12, 0x2800},    // 700038A4 
{0x0F12, 0xD006},    // 700038A6 
{0x0F12, 0x1FC1},    // 700038A8 
{0x0F12, 0x39FD},    // 700038AA 
{0x0F12, 0xD003},    // 700038AC 
{0x0F12, 0x2001},    // 700038AE 
{0x0F12, 0xBCF8},    // 700038B0 
{0x0F12, 0xBC08},    // 700038B2 
{0x0F12, 0x4718},    // 700038B4 
{0x0F12, 0x2000},    // 700038B6 
{0x0F12, 0xE7FA},    // 700038B8 
{0x0F12, 0xB570},    // 700038BA 
{0x0F12, 0x4C7D},    // 700038BC 
{0x0F12, 0x8860},    // 700038BE 
{0x0F12, 0x2800},    // 700038C0 
{0x0F12, 0xD00C},    // 700038C2 
{0x0F12, 0x8820},    // 700038C4 
{0x0F12, 0x4D74},    // 700038C6 
{0x0F12, 0x2800},    // 700038C8 
{0x0F12, 0xD009},    // 700038CA 
{0x0F12, 0x0029},    // 700038CC 
{0x0F12, 0x31A0},    // 700038CE 
{0x0F12, 0x7AC9},    // 700038D0 
{0x0F12, 0x2900},    // 700038D2 
{0x0F12, 0xD004},    // 700038D4 
{0x0F12, 0x7AA8},    // 700038D6 
{0x0F12, 0x2180},    // 700038D8 
{0x0F12, 0x4308},    // 700038DA 
{0x0F12, 0x72A8},    // 700038DC 
{0x0F12, 0xE6D1},    // 700038DE 
{0x0F12, 0x2800},    // 700038E0 
{0x0F12, 0xD003},    // 700038E2 
{0x0F12, 0xF7FF},    // 700038E4 
{0x0F12, 0xFFAF},    // 700038E6 
{0x0F12, 0x2800},    // 700038E8 
{0x0F12, 0xD1F8},    // 700038EA 
{0x0F12, 0x2000},    // 700038EC 
{0x0F12, 0x8060},    // 700038EE 
{0x0F12, 0x8820},    // 700038F0 
{0x0F12, 0x2800},    // 700038F2 
{0x0F12, 0xD003},    // 700038F4 
{0x0F12, 0x2008},    // 700038F6 
{0x0F12, 0xF000},    // 700038F8 
{0x0F12, 0xFA5C},    // 700038FA 
{0x0F12, 0xE00B},    // 700038FC 
{0x0F12, 0x486D},    // 700038FE 
{0x0F12, 0x3020},    // 70003900 
{0x0F12, 0x8880},    // 70003902 
{0x0F12, 0x2800},    // 70003904 
{0x0F12, 0xD103},    // 70003906 
{0x0F12, 0x7AA8},    // 70003908 
{0x0F12, 0x2101},    // 7000390A 
{0x0F12, 0x4308},    // 7000390C 
{0x0F12, 0x72A8},    // 7000390E 
{0x0F12, 0x2010},    // 70003910 
{0x0F12, 0xF000},    // 70003912 
{0x0F12, 0xFA4F},    // 70003914 
{0x0F12, 0x8820},    // 70003916 
{0x0F12, 0x2800},    // 70003918 
{0x0F12, 0xD1E0},    // 7000391A 
{0x0F12, 0x4856},    // 7000391C 
{0x0F12, 0x89C0},    // 7000391E 
{0x0F12, 0x2801},    // 70003920 
{0x0F12, 0xD1DC},    // 70003922 
{0x0F12, 0x7AA8},    // 70003924 
{0x0F12, 0x21BF},    // 70003926 
{0x0F12, 0x4008},    // 70003928 
{0x0F12, 0x72A8},    // 7000392A 
{0x0F12, 0xE6AA},    // 7000392C 
{0x0F12, 0x6800},    // 7000392E 
{0x0F12, 0x4961},    // 70003930 
{0x0F12, 0x8188},    // 70003932 
{0x0F12, 0x4861},    // 70003934 
{0x0F12, 0x2201},    // 70003936 
{0x0F12, 0x8981},    // 70003938 
{0x0F12, 0x4861},    // 7000393A 
{0x0F12, 0x0252},    // 7000393C 
{0x0F12, 0x4291},    // 7000393E 
{0x0F12, 0xD902},    // 70003940 
{0x0F12, 0x2102},    // 70003942 
{0x0F12, 0x8181},    // 70003944 
{0x0F12, 0x4770},    // 70003946 
{0x0F12, 0x2101},    // 70003948 
{0x0F12, 0x8181},    // 7000394A 
{0x0F12, 0x4770},    // 7000394C 
{0x0F12, 0xB5F1},    // 7000394E 
{0x0F12, 0x4E51},    // 70003950 
{0x0F12, 0x8834},    // 70003952 
{0x0F12, 0x2C00},    // 70003954 
{0x0F12, 0xD03C},    // 70003956 
{0x0F12, 0x2001},    // 70003958 
{0x0F12, 0x2C08},    // 7000395A 
{0x0F12, 0xD000},    // 7000395C 
{0x0F12, 0x2000},    // 7000395E 
{0x0F12, 0x70B0},    // 70003960 
{0x0F12, 0x4D50},    // 70003962 
{0x0F12, 0x2700},    // 70003964 
{0x0F12, 0x2800},    // 70003966 
{0x0F12, 0xD009},    // 70003968 
{0x0F12, 0xF000},    // 7000396A 
{0x0F12, 0xFA2B},    // 7000396C 
{0x0F12, 0x0028},    // 7000396E 
{0x0F12, 0x38F0},    // 70003970 
{0x0F12, 0x6328},    // 70003972 
{0x0F12, 0x7AB0},    // 70003974 
{0x0F12, 0x217E},    // 70003976 
{0x0F12, 0x4008},    // 70003978 
{0x0F12, 0x72B0},    // 7000397A 
{0x0F12, 0xE00C},    // 7000397C 
{0x0F12, 0x484C},    // 7000397E 
{0x0F12, 0x8F00},    // 70003980 
{0x0F12, 0x2800},    // 70003982 
{0x0F12, 0xD003},    // 70003984 
{0x0F12, 0xF000},    // 70003986 
{0x0F12, 0xFA25},    // 70003988 
{0x0F12, 0x4849},    // 7000398A 
{0x0F12, 0x8707},    // 7000398C 
{0x0F12, 0x2000},    // 7000398E 
{0x0F12, 0xF000},    // 70003990 
{0x0F12, 0xFA28},    // 70003992 
{0x0F12, 0x484B},    // 70003994 
{0x0F12, 0x6328},    // 70003996 
{0x0F12, 0x78B1},    // 70003998 
{0x0F12, 0x0038},    // 7000399A 
{0x0F12, 0x2900},    // 7000399C 
{0x0F12, 0xD008},    // 7000399E 
{0x0F12, 0x4944},    // 700039A0 
{0x0F12, 0x3920},    // 700039A2 
{0x0F12, 0x8ACA},    // 700039A4 
{0x0F12, 0x2A00},    // 700039A6 
{0x0F12, 0xD003},    // 700039A8 
{0x0F12, 0x8B09},    // 700039AA 
{0x0F12, 0x2900},    // 700039AC 
{0x0F12, 0xD000},    // 700039AE 
{0x0F12, 0x2001},    // 700039B0 
{0x0F12, 0x7170},    // 700039B2 
{0x0F12, 0x2C02},    // 700039B4 
{0x0F12, 0xD102},    // 700039B6 
{0x0F12, 0x483A},    // 700039B8 
{0x0F12, 0x3860},    // 700039BA 
{0x0F12, 0x6328},    // 700039BC 
{0x0F12, 0x2201},    // 700039BE 
{0x0F12, 0x2C02},    // 700039C0 
{0x0F12, 0xD000},    // 700039C2 
{0x0F12, 0x2200},    // 700039C4 
{0x0F12, 0x4834},    // 700039C6 
{0x0F12, 0x2110},    // 700039C8 
{0x0F12, 0x300A},    // 700039CA 
{0x0F12, 0xF000},    // 700039CC 
{0x0F12, 0xFA12},    // 700039CE 
{0x0F12, 0x8037},    // 700039D0 
{0x0F12, 0x9900},    // 700039D2 
{0x0F12, 0x0020},    // 700039D4 
{0x0F12, 0x600C},    // 700039D6 
{0x0F12, 0xE76A},    // 700039D8 
{0x0F12, 0xB538},    // 700039DA 
{0x0F12, 0x4837},    // 700039DC 
{0x0F12, 0x4669},    // 700039DE 
{0x0F12, 0x3848},    // 700039E0 
{0x0F12, 0xF000},    // 700039E2 
{0x0F12, 0xFA0F},    // 700039E4 
{0x0F12, 0x4A32},    // 700039E6 
{0x0F12, 0x4834},    // 700039E8 
{0x0F12, 0x8F51},    // 700039EA 
{0x0F12, 0x2400},    // 700039EC 
{0x0F12, 0x3020},    // 700039EE 
{0x0F12, 0x2900},    // 700039F0 
{0x0F12, 0xD00A},    // 700039F2 
{0x0F12, 0x8754},    // 700039F4 
{0x0F12, 0x6941},    // 700039F6 
{0x0F12, 0x6451},    // 700039F8 
{0x0F12, 0x6491},    // 700039FA 
{0x0F12, 0x466B},    // 700039FC 
{0x0F12, 0x8819},    // 700039FE

{0x0028, 0x7000},
{0x002A, 0x3A00}, 
{0x0F12, 0x87D1},    // 70003A00 
{0x0F12, 0x885B},    // 70003A02 
{0x0F12, 0x0011},    // 70003A04 
{0x0F12, 0x3140},    // 70003A06 
{0x0F12, 0x800B},    // 70003A08 
{0x0F12, 0x8F91},    // 70003A0A 
{0x0F12, 0x2900},    // 70003A0C 
{0x0F12, 0xD002},    // 70003A0E 
{0x0F12, 0x8794},    // 70003A10 
{0x0F12, 0x6940},    // 70003A12 
{0x0F12, 0x6490},    // 70003A14 
{0x0F12, 0xF000},    // 70003A16 
{0x0F12, 0xF9FD},    // 70003A18 
{0x0F12, 0xBC38},    // 70003A1A 
{0x0F12, 0xBC08},    // 70003A1C 
{0x0F12, 0x4718},    // 70003A1E 
{0x0F12, 0xB5F8},    // 70003A20 
{0x0F12, 0x4C29},    // 70003A22 
{0x0F12, 0x89E0},    // 70003A24 
{0x0F12, 0xF000},    // 70003A26 
{0x0F12, 0xF9FD},    // 70003A28 
{0x0F12, 0x0006},    // 70003A2A 
{0x0F12, 0x8A20},    // 70003A2C 
{0x0F12, 0xF000},    // 70003A2E 
{0x0F12, 0xFA01},    // 70003A30 
{0x0F12, 0x0007},    // 70003A32 
{0x0F12, 0x4821},    // 70003A34 
{0x0F12, 0x4D1E},    // 70003A36 
{0x0F12, 0x3020},    // 70003A38 
{0x0F12, 0x6CA9},    // 70003A3A 
{0x0F12, 0x6940},    // 70003A3C 
{0x0F12, 0x1809},    // 70003A3E 
{0x0F12, 0x0200},    // 70003A40 
{0x0F12, 0xF000},    // 70003A42 
{0x0F12, 0xF9FF},    // 70003A44 
{0x0F12, 0x0400},    // 70003A46 
{0x0F12, 0x0C00},    // 70003A48 
{0x0F12, 0x002A},    // 70003A4A 
{0x0F12, 0x326E},    // 70003A4C 
{0x0F12, 0x0011},    // 70003A4E 
{0x0F12, 0x390A},    // 70003A50 
{0x0F12, 0x2305},    // 70003A52 
{0x0F12, 0xF000},    // 70003A54 
{0x0F12, 0xF9FC},    // 70003A56 
{0x0F12, 0x4C14},    // 70003A58 
{0x0F12, 0x61A0},    // 70003A5A 
{0x0F12, 0x8FEB},    // 70003A5C 
{0x0F12, 0x0002},    // 70003A5E 
{0x0F12, 0x0031},    // 70003A60 
{0x0F12, 0x0018},    // 70003A62 
{0x0F12, 0xF000},    // 70003A64 
{0x0F12, 0xF9FC},    // 70003A66 
{0x0F12, 0x466B},    // 70003A68 
{0x0F12, 0x0005},    // 70003A6A 
{0x0F12, 0x8018},    // 70003A6C 
{0x0F12, 0xE02D},    // 70003A6E 
{0x0F12, 0x3290},    // 70003A70 
{0x0F12, 0x7000},    // 70003A72 
{0x0F12, 0x3294},    // 70003A74 
{0x0F12, 0x7000},    // 70003A76 
{0x0F12, 0x04A8},    // 70003A78 
{0x0F12, 0x7000},    // 70003A7A 
{0x0F12, 0x15DC},    // 70003A7C 
{0x0F12, 0x7000},    // 70003A7E 
{0x0F12, 0x5000},    // 70003A80 
{0x0F12, 0xD000},    // 70003A82 
{0x0F12, 0x064C},    // 70003A84 
{0x0F12, 0x7000},    // 70003A86 
{0x0F12, 0xA000},    // 70003A88 
{0x0F12, 0xD000},    // 70003A8A 
{0x0F12, 0x2468},    // 70003A8C 
{0x0F12, 0x7000},    // 70003A8E 
{0x0F12, 0x11DC},    // 70003A90 
{0x0F12, 0x7000},    // 70003A92 
{0x0F12, 0x2828},    // 70003A94 
{0x0F12, 0x7000},    // 70003A96 
{0x0F12, 0x1E84},    // 70003A98 
{0x0F12, 0x7000},    // 70003A9A 
{0x0F12, 0x1BE4},    // 70003A9C 
{0x0F12, 0x7000},    // 70003A9E 
{0x0F12, 0x2EA8},    // 70003AA0 
{0x0F12, 0x7000},    // 70003AA2 
{0x0F12, 0x21A4},    // 70003AA4 
{0x0F12, 0x7000},    // 70003AA6 
{0x0F12, 0x0100},    // 70003AA8 
{0x0F12, 0x7000},    // 70003AAA 
{0x0F12, 0x31A0},    // 70003AAC 
{0x0F12, 0x7000},    // 70003AAE 
{0x0F12, 0x3F48},    // 70003AB0 
{0x0F12, 0x7000},    // 70003AB2 
{0x0F12, 0x01E8},    // 70003AB4 
{0x0F12, 0x7000},    // 70003AB6 
{0x0F12, 0xF2A0},    // 70003AB8 
{0x0F12, 0xD000},    // 70003ABA 
{0x0F12, 0x2A44},    // 70003ABC 
{0x0F12, 0x7000},    // 70003ABE 
{0x0F12, 0xF400},    // 70003AC0 
{0x0F12, 0xD000},    // 70003AC2 
{0x0F12, 0x2024},    // 70003AC4 
{0x0F12, 0x7000},    // 70003AC6 
{0x0F12, 0x1650},    // 70003AC8 
{0x0F12, 0x7000},    // 70003ACA 
{0x0F12, 0x4888},    // 70003ACC 
{0x0F12, 0x69A2},    // 70003ACE 
{0x0F12, 0x8800},    // 70003AD0 
{0x0F12, 0x0039},    // 70003AD2 
{0x0F12, 0xF000},    // 70003AD4 
{0x0F12, 0xF9C4},    // 70003AD6 
{0x0F12, 0x466B},    // 70003AD8 
{0x0F12, 0x0006},    // 70003ADA 
{0x0F12, 0x8058},    // 70003ADC 
{0x0F12, 0x0021},    // 70003ADE 
{0x0F12, 0x9800},    // 70003AE0 
{0x0F12, 0x311C},    // 70003AE2 
{0x0F12, 0xF000},    // 70003AE4 
{0x0F12, 0xF9C4},    // 70003AE6 
{0x0F12, 0x4981},    // 70003AE8 
{0x0F12, 0x3140},    // 70003AEA 
{0x0F12, 0x808D},    // 70003AEC 
{0x0F12, 0x80CE},    // 70003AEE 
{0x0F12, 0x8BA1},    // 70003AF0 
{0x0F12, 0x4880},    // 70003AF2 
{0x0F12, 0x8001},    // 70003AF4 
{0x0F12, 0x8BE1},    // 70003AF6 
{0x0F12, 0x8041},    // 70003AF8 
{0x0F12, 0x8C21},    // 70003AFA 
{0x0F12, 0x8081},    // 70003AFC 
{0x0F12, 0xE6D7},    // 70003AFE 
{0x0F12, 0xB5F8},    // 70003B00 
{0x0F12, 0x4E7B},    // 70003B02 
{0x0F12, 0x3E40},    // 70003B04 
{0x0F12, 0x6C70},    // 70003B06 
{0x0F12, 0x6CB1},    // 70003B08 
{0x0F12, 0x0200},    // 70003B0A 
{0x0F12, 0xF000},    // 70003B0C 
{0x0F12, 0xF99A},    // 70003B0E 
{0x0F12, 0x0400},    // 70003B10 
{0x0F12, 0x0C00},    // 70003B12 
{0x0F12, 0x2401},    // 70003B14 
{0x0F12, 0x0364},    // 70003B16 
{0x0F12, 0x42A0},    // 70003B18 
{0x0F12, 0xD200},    // 70003B1A 
{0x0F12, 0x0004},    // 70003B1C 
{0x0F12, 0x4A74},    // 70003B1E 
{0x0F12, 0x0020},    // 70003B20 
{0x0F12, 0x323E},    // 70003B22 
{0x0F12, 0x1F91},    // 70003B24 
{0x0F12, 0x2303},    // 70003B26 
{0x0F12, 0xF000},    // 70003B28 
{0x0F12, 0xF992},    // 70003B2A 
{0x0F12, 0x0405},    // 70003B2C 
{0x0F12, 0x0C2D},    // 70003B2E 
{0x0F12, 0x4A6F},    // 70003B30 
{0x0F12, 0x0020},    // 70003B32 
{0x0F12, 0x321A},    // 70003B34 
{0x0F12, 0x0011},    // 70003B36 
{0x0F12, 0x390A},    // 70003B38 
{0x0F12, 0x2305},    // 70003B3A 
{0x0F12, 0xF000},    // 70003B3C 
{0x0F12, 0xF988},    // 70003B3E 
{0x0F12, 0x496B},    // 70003B40 
{0x0F12, 0x3940},    // 70003B42 
{0x0F12, 0x64C8},    // 70003B44 
{0x0F12, 0x496C},    // 70003B46 
{0x0F12, 0x4E6A},    // 70003B48 
{0x0F12, 0x88C8},    // 70003B4A 
{0x0F12, 0x2701},    // 70003B4C 
{0x0F12, 0x3620},    // 70003B4E 
{0x0F12, 0x2800},    // 70003B50 
{0x0F12, 0xD009},    // 70003B52 
{0x0F12, 0x4C69},    // 70003B54 
{0x0F12, 0x38FF},    // 70003B56 
{0x0F12, 0x1E40},    // 70003B58 
{0x0F12, 0xD00A},    // 70003B5A 
{0x0F12, 0x2804},    // 70003B5C 
{0x0F12, 0xD01D},    // 70003B5E 
{0x0F12, 0x2806},    // 70003B60 
{0x0F12, 0xD101},    // 70003B62 
{0x0F12, 0x2000},    // 70003B64 
{0x0F12, 0x80C8},    // 70003B66 
{0x0F12, 0x82B7},    // 70003B68 
{0x0F12, 0x2001},    // 70003B6A 
{0x0F12, 0xF000},    // 70003B6C 
{0x0F12, 0xF988},    // 70003B6E 
{0x0F12, 0xE69E},    // 70003B70 
{0x0F12, 0x000D},    // 70003B72 
{0x0F12, 0x724F},    // 70003B74 
{0x0F12, 0x2001},    // 70003B76 
{0x0F12, 0xF000},    // 70003B78 
{0x0F12, 0xF98A},    // 70003B7A 
{0x0F12, 0xF000},    // 70003B7C 
{0x0F12, 0xF990},    // 70003B7E 
{0x0F12, 0x485B},    // 70003B80 
{0x0F12, 0x3840},    // 70003B82 
{0x0F12, 0x6C81},    // 70003B84 
{0x0F12, 0x6CC0},    // 70003B86 
{0x0F12, 0x4341},    // 70003B88 
{0x0F12, 0x0A08},    // 70003B8A 
{0x0F12, 0x6160},    // 70003B8C 
{0x0F12, 0x20FF},    // 70003B8E 
{0x0F12, 0x1D40},    // 70003B90 
{0x0F12, 0x80E8},    // 70003B92 
{0x0F12, 0x4858},    // 70003B94 
{0x0F12, 0x3040},    // 70003B96 
{0x0F12, 0x7707},    // 70003B98 
{0x0F12, 0xE7E5},    // 70003B9A 
{0x0F12, 0x4856},    // 70003B9C 
{0x0F12, 0x7247},    // 70003B9E 
{0x0F12, 0x21FF},    // 70003BA0 
{0x0F12, 0x1DC9},    // 70003BA2 
{0x0F12, 0x80C1},    // 70003BA4 
{0x0F12, 0xF000},    // 70003BA6 
{0x0F12, 0xF983},    // 70003BA8 
{0x0F12, 0x4952},    // 70003BAA 
{0x0F12, 0x3940},    // 70003BAC 
{0x0F12, 0x2800},    // 70003BAE 
{0x0F12, 0xD007},    // 70003BB0 
{0x0F12, 0x684A},    // 70003BB2 
{0x0F12, 0x0001},    // 70003BB4 
{0x0F12, 0x436A},    // 70003BB6 
{0x0F12, 0x0010},    // 70003BB8 
{0x0F12, 0xF000},    // 70003BBA 
{0x0F12, 0xF943},    // 70003BBC 
{0x0F12, 0x6160},    // 70003BBE 
{0x0F12, 0xE002},    // 70003BC0 
{0x0F12, 0x6848},    // 70003BC2 
{0x0F12, 0x4368},    // 70003BC4 
{0x0F12, 0x6160},    // 70003BC6 
{0x0F12, 0x8BF0},    // 70003BC8 
{0x0F12, 0x2800},    // 70003BCA 
{0x0F12, 0xD001},    // 70003BCC 
{0x0F12, 0xF7FF},    // 70003BCE 
{0x0F12, 0xFF27},    // 70003BD0 
{0x0F12, 0x2000},    // 70003BD2 
{0x0F12, 0xF000},    // 70003BD4 
{0x0F12, 0xF95C},    // 70003BD6 
{0x0F12, 0x4947},    // 70003BD8 
{0x0F12, 0x20FF},    // 70003BDA 
{0x0F12, 0x1DC0},    // 70003BDC 
{0x0F12, 0x80C8},    // 70003BDE 
{0x0F12, 0xE7C2},    // 70003BE0 
{0x0F12, 0xB5F8},    // 70003BE2 
{0x0F12, 0x2400},    // 70003BE4 
{0x0F12, 0x4D46},    // 70003BE6 
{0x0F12, 0x4846},    // 70003BE8 
{0x0F12, 0x210E},    // 70003BEA 
{0x0F12, 0x8041},    // 70003BEC 
{0x0F12, 0x2101},    // 70003BEE 
{0x0F12, 0x8001},    // 70003BF0 
{0x0F12, 0xF000},    // 70003BF2 
{0x0F12, 0xF965},    // 70003BF4 
{0x0F12, 0x4844},    // 70003BF6 
{0x0F12, 0x8840},    // 70003BF8 
{0x0F12, 0xF000},    // 70003BFA 
{0x0F12, 0xF89D},    // 70003BFC 
{0x0F12, 0x4E3C},    // 70003BFE 
{0x0F12, 0x270D},    // 70003C00 
{0x0F12, 0x073F},    // 70003C02 
{0x0F12, 0x3E40},    // 70003C04 
{0x0F12, 0x19E8},    // 70003C06 
{0x0F12, 0x8803},    // 70003C08 
{0x0F12, 0x00E2},    // 70003C0A 
{0x0F12, 0x1991},    // 70003C0C 
{0x0F12, 0x804B},    // 70003C0E 
{0x0F12, 0x8843},    // 70003C10 
{0x0F12, 0x52B3},    // 70003C12 
{0x0F12, 0x8882},    // 70003C14 
{0x0F12, 0x80CA},    // 70003C16 
{0x0F12, 0x88C0},    // 70003C18 
{0x0F12, 0x8088},    // 70003C1A 
{0x0F12, 0x3508},    // 70003C1C 
{0x0F12, 0x042D},    // 70003C1E 
{0x0F12, 0x0C2D},    // 70003C20 
{0x0F12, 0x1C64},    // 70003C22 
{0x0F12, 0x0424},    // 70003C24 
{0x0F12, 0x0C24},    // 70003C26 
{0x0F12, 0x2C07},    // 70003C28 
{0x0F12, 0xD3EC},    // 70003C2A 
{0x0F12, 0xE640},    // 70003C2C 
{0x0F12, 0xB5F0},    // 70003C2E 
{0x0F12, 0xB085},    // 70003C30 
{0x0F12, 0x6801},    // 70003C32 
{0x0F12, 0x9103},    // 70003C34 
{0x0F12, 0x6881},    // 70003C36 
{0x0F12, 0x040A},    // 70003C38 
{0x0F12, 0x0C12},    // 70003C3A 
{0x0F12, 0x4933},    // 70003C3C 
{0x0F12, 0x8B89},    // 70003C3E 
{0x0F12, 0x2900},    // 70003C40 
{0x0F12, 0xD001},    // 70003C42 
{0x0F12, 0x0011},    // 70003C44 
{0x0F12, 0xE000},    // 70003C46 
{0x0F12, 0x2100},    // 70003C48 
{0x0F12, 0x9102},    // 70003C4A 
{0x0F12, 0x6840},    // 70003C4C 
{0x0F12, 0x0401},    // 70003C4E 
{0x0F12, 0x9803},    // 70003C50 
{0x0F12, 0x0C09},    // 70003C52 
{0x0F12, 0xF000},    // 70003C54 
{0x0F12, 0xF93C},    // 70003C56 
{0x0F12, 0x4825},    // 70003C58 
{0x0F12, 0x3040},    // 70003C5A 
{0x0F12, 0x8900},    // 70003C5C 
{0x0F12, 0x2800},    // 70003C5E 
{0x0F12, 0xD03B},    // 70003C60 
{0x0F12, 0x2100},    // 70003C62 
{0x0F12, 0x4825},    // 70003C64 
{0x0F12, 0x4D2A},    // 70003C66 
{0x0F12, 0x30C0},    // 70003C68 
{0x0F12, 0x4684},    // 70003C6A 
{0x0F12, 0x4B29},    // 70003C6C 
{0x0F12, 0x4C20},    // 70003C6E 
{0x0F12, 0x88DA},    // 70003C70 
{0x0F12, 0x3C40},    // 70003C72 
{0x0F12, 0x0048},    // 70003C74 
{0x0F12, 0x00D7},    // 70003C76 
{0x0F12, 0x193E},    // 70003C78 
{0x0F12, 0x197F},    // 70003C7A 
{0x0F12, 0x183F},    // 70003C7C 
{0x0F12, 0x5A36},    // 70003C7E 
{0x0F12, 0x8AFF},    // 70003C80 
{0x0F12, 0x437E},    // 70003C82 
{0x0F12, 0x00B6},    // 70003C84 
{0x0F12, 0x0C37},    // 70003C86 
{0x0F12, 0x1906},    // 70003C88 
{0x0F12, 0x3680},    // 70003C8A 
{0x0F12, 0x8177},    // 70003C8C 
{0x0F12, 0x1C52},    // 70003C8E 
{0x0F12, 0x00D2},    // 70003C90 
{0x0F12, 0x1914},    // 70003C92 
{0x0F12, 0x1952},    // 70003C94 
{0x0F12, 0x1812},    // 70003C96 
{0x0F12, 0x5A24},    // 70003C98 
{0x0F12, 0x8AD2},    // 70003C9A 
{0x0F12, 0x4354},    // 70003C9C 
{0x0F12, 0x00A2},    // 70003C9E 
{0x0F12, 0x0C12},    // 70003CA0 
{0x0F12, 0x8272},    // 70003CA2 
{0x0F12, 0x891C},    // 70003CA4 
{0x0F12, 0x895B},    // 70003CA6 
{0x0F12, 0x4367},    // 70003CA8 
{0x0F12, 0x435A},    // 70003CAA 
{0x0F12, 0x1943},    // 70003CAC 
{0x0F12, 0x3340},    // 70003CAE 
{0x0F12, 0x89DB},    // 70003CB0 
{0x0F12, 0x9C02},    // 70003CB2 
{0x0F12, 0x18BA},    // 70003CB4 
{0x0F12, 0x4363},    // 70003CB6 
{0x0F12, 0x18D2},    // 70003CB8 
{0x0F12, 0x0212},    // 70003CBA 
{0x0F12, 0x0C12},    // 70003CBC 
{0x0F12, 0x466B},    // 70003CBE 
{0x0F12, 0x521A},    // 70003CC0 
{0x0F12, 0x4663},    // 70003CC2 
{0x0F12, 0x7DDB},    // 70003CC4 
{0x0F12, 0x435A},    // 70003CC6 
{0x0F12, 0x9B03},    // 70003CC8 
{0x0F12, 0x0252},    // 70003CCA 
{0x0F12, 0x0C12},    // 70003CCC 
{0x0F12, 0x521A},    // 70003CCE 
{0x0F12, 0x1C49},    // 70003CD0 
{0x0F12, 0x0409},    // 70003CD2 
{0x0F12, 0x0C09},    // 70003CD4 
{0x0F12, 0x2904},    // 70003CD6 
{0x0F12, 0xD3C8},    // 70003CD8 
{0x0F12, 0xB005},    // 70003CDA 
{0x0F12, 0xBCF0},    // 70003CDC 
{0x0F12, 0xBC08},    // 70003CDE 
{0x0F12, 0x4718},    // 70003CE0 
{0x0F12, 0xB510},    // 70003CE2 
{0x0F12, 0xF7FF},    // 70003CE4 
{0x0F12, 0xFF7D},    // 70003CE6 
{0x0F12, 0xF000},    // 70003CE8 
{0x0F12, 0xF8FA},    // 70003CEA 
{0x0F12, 0xE502},    // 70003CEC 
{0x0F12, 0x0000},    // 70003CEE 
{0x0F12, 0x3F88},    // 70003CF0 
{0x0F12, 0x7000},    // 70003CF2 
{0x0F12, 0x2A24},    // 70003CF4 
{0x0F12, 0x7000},    // 70003CF6 
{0x0F12, 0x31A0},    // 70003CF8 
{0x0F12, 0x7000},    // 70003CFA 
{0x0F12, 0x2A64},    // 70003CFC 
{0x0F12, 0x7000},    // 70003CFE 
{0x0F12, 0xA006},    // 70003D00 
{0x0F12, 0x0000},    // 70003D02 
{0x0F12, 0xA000},    // 70003D04 
{0x0F12, 0xD000},    // 70003D06 
{0x0F12, 0x064C},    // 70003D08 
{0x0F12, 0x7000},    // 70003D0A 
{0x0F12, 0x07C4},    // 70003D0C 
{0x0F12, 0x7000},    // 70003D0E 
{0x0F12, 0x07E8},    // 70003D10 
{0x0F12, 0x7000},    // 70003D12 
{0x0F12, 0x1FA0},    // 70003D14 
{0x0F12, 0x7000},    // 70003D16 
{0x0F12, 0x4778},    // 70003D18 
{0x0F12, 0x46C0},    // 70003D1A 
{0x0F12, 0xC000},    // 70003D1C 
{0x0F12, 0xE59F},    // 70003D1E 
{0x0F12, 0xFF1C},    // 70003D20 
{0x0F12, 0xE12F},    // 70003D22 
{0x0F12, 0x1F63},    // 70003D24 
{0x0F12, 0x0001},    // 70003D26 
{0x0F12, 0x4778},    // 70003D28 
{0x0F12, 0x46C0},    // 70003D2A 
{0x0F12, 0xC000},    // 70003D2C 
{0x0F12, 0xE59F},    // 70003D2E 
{0x0F12, 0xFF1C},    // 70003D30 
{0x0F12, 0xE12F},    // 70003D32 
{0x0F12, 0x1EDF},    // 70003D34 
{0x0F12, 0x0001},    // 70003D36 
{0x0F12, 0x4778},    // 70003D38 
{0x0F12, 0x46C0},    // 70003D3A 
{0x0F12, 0xC000},    // 70003D3C 
{0x0F12, 0xE59F},    // 70003D3E 
{0x0F12, 0xFF1C},    // 70003D40 
{0x0F12, 0xE12F},    // 70003D42 
{0x0F12, 0xFDAF},    // 70003D44 
{0x0F12, 0x0000},    // 70003D46 
{0x0F12, 0x4778},    // 70003D48 
{0x0F12, 0x46C0},    // 70003D4A 
{0x0F12, 0xF004},    // 70003D4C 
{0x0F12, 0xE51F},    // 70003D4E 
{0x0F12, 0x2328},    // 70003D50 
{0x0F12, 0x0001},    // 70003D52 
{0x0F12, 0x4778},    // 70003D54 
{0x0F12, 0x46C0},    // 70003D56 
{0x0F12, 0xC000},    // 70003D58 
{0x0F12, 0xE59F},    // 70003D5A 
{0x0F12, 0xFF1C},    // 70003D5C 
{0x0F12, 0xE12F},    // 70003D5E 
{0x0F12, 0x9E89},    // 70003D60 
{0x0F12, 0x0000},    // 70003D62 
{0x0F12, 0x4778},    // 70003D64 
{0x0F12, 0x46C0},    // 70003D66 
{0x0F12, 0xC000},    // 70003D68 
{0x0F12, 0xE59F},    // 70003D6A 
{0x0F12, 0xFF1C},    // 70003D6C 
{0x0F12, 0xE12F},    // 70003D6E 
{0x0F12, 0x495F},    // 70003D70 
{0x0F12, 0x0000},    // 70003D72 
{0x0F12, 0x4778},    // 70003D74 
{0x0F12, 0x46C0},    // 70003D76 
{0x0F12, 0xC000},    // 70003D78 
{0x0F12, 0xE59F},    // 70003D7A 
{0x0F12, 0xFF1C},    // 70003D7C 
{0x0F12, 0xE12F},    // 70003D7E 
{0x0F12, 0xE403},    // 70003D80 
{0x0F12, 0x0000},    // 70003D82 
{0x0F12, 0x4778},    // 70003D84 
{0x0F12, 0x46C0},    // 70003D86 
{0x0F12, 0xC000},    // 70003D88 
{0x0F12, 0xE59F},    // 70003D8A 
{0x0F12, 0xFF1C},    // 70003D8C 
{0x0F12, 0xE12F},    // 70003D8E 
{0x0F12, 0x24B3},    // 70003D90 
{0x0F12, 0x0001},    // 70003D92 
{0x0F12, 0x4778},    // 70003D94 
{0x0F12, 0x46C0},    // 70003D96 
{0x0F12, 0xC000},    // 70003D98 
{0x0F12, 0xE59F},    // 70003D9A 
{0x0F12, 0xFF1C},    // 70003D9C 
{0x0F12, 0xE12F},    // 70003D9E 
{0x0F12, 0xEECD},    // 70003DA0 
{0x0F12, 0x0000},    // 70003DA2 
{0x0F12, 0x4778},    // 70003DA4 
{0x0F12, 0x46C0},    // 70003DA6 
{0x0F12, 0xC000},    // 70003DA8 
{0x0F12, 0xE59F},    // 70003DAA 
{0x0F12, 0xFF1C},    // 70003DAC 
{0x0F12, 0xE12F},    // 70003DAE 
{0x0F12, 0xF049},    // 70003DB0 
{0x0F12, 0x0000},    // 70003DB2 
{0x0F12, 0x4778},    // 70003DB4 
{0x0F12, 0x46C0},    // 70003DB6 
{0x0F12, 0xC000},    // 70003DB8 
{0x0F12, 0xE59F},    // 70003DBA 
{0x0F12, 0xFF1C},    // 70003DBC 
{0x0F12, 0xE12F},    // 70003DBE 
{0x0F12, 0x12DF},    // 70003DC0 
{0x0F12, 0x0000},    // 70003DC2 
{0x0F12, 0x4778},    // 70003DC4 
{0x0F12, 0x46C0},    // 70003DC6 
{0x0F12, 0xC000},    // 70003DC8 
{0x0F12, 0xE59F},    // 70003DCA 
{0x0F12, 0xFF1C},    // 70003DCC 
{0x0F12, 0xE12F},    // 70003DCE 
{0x0F12, 0xF05B},    // 70003DD0 
{0x0F12, 0x0000},    // 70003DD2 
{0x0F12, 0x4778},    // 70003DD4 
{0x0F12, 0x46C0},    // 70003DD6 
{0x0F12, 0xC000},    // 70003DD8 
{0x0F12, 0xE59F},    // 70003DDA 
{0x0F12, 0xFF1C},    // 70003DDC 
{0x0F12, 0xE12F},    // 70003DDE 
{0x0F12, 0xF07B},    // 70003DE0 
{0x0F12, 0x0000},    // 70003DE2 
{0x0F12, 0x4778},    // 70003DE4 
{0x0F12, 0x46C0},    // 70003DE6 
{0x0F12, 0xC000},    // 70003DE8 
{0x0F12, 0xE59F},    // 70003DEA 
{0x0F12, 0xFF1C},    // 70003DEC 
{0x0F12, 0xE12F},    // 70003DEE 
{0x0F12, 0xFE6D},    // 70003DF0 
{0x0F12, 0x0000},    // 70003DF2 
{0x0F12, 0x4778},    // 70003DF4 
{0x0F12, 0x46C0},    // 70003DF6 
{0x0F12, 0xC000},    // 70003DF8 
{0x0F12, 0xE59F},    // 70003DFA 
{0x0F12, 0xFF1C},    // 70003DFC 
{0x0F12, 0xE12F},    // 70003DFE 
{0x0F12, 0x3295},    // 70003E00 
{0x0F12, 0x0000},    // 70003E02 
{0x0F12, 0x4778},    // 70003E04 
{0x0F12, 0x46C0},    // 70003E06 
{0x0F12, 0xC000},    // 70003E08 
{0x0F12, 0xE59F},    // 70003E0A 
{0x0F12, 0xFF1C},    // 70003E0C 
{0x0F12, 0xE12F},    // 70003E0E 
{0x0F12, 0x234F},    // 70003E10 
{0x0F12, 0x0000},    // 70003E12 
{0x0F12, 0x4778},    // 70003E14 
{0x0F12, 0x46C0},    // 70003E16 
{0x0F12, 0xC000},    // 70003E18 
{0x0F12, 0xE59F},    // 70003E1A 
{0x0F12, 0xFF1C},    // 70003E1C 
{0x0F12, 0xE12F},    // 70003E1E 
{0x0F12, 0x4521},    // 70003E20 
{0x0F12, 0x0000},    // 70003E22 
{0x0F12, 0x4778},    // 70003E24 
{0x0F12, 0x46C0},    // 70003E26 
{0x0F12, 0xC000},    // 70003E28 
{0x0F12, 0xE59F},    // 70003E2A 
{0x0F12, 0xFF1C},    // 70003E2C 
{0x0F12, 0xE12F},    // 70003E2E 
{0x0F12, 0x7C0D},    // 70003E30 
{0x0F12, 0x0000},    // 70003E32 
{0x0F12, 0x4778},    // 70003E34 
{0x0F12, 0x46C0},    // 70003E36 
{0x0F12, 0xC000},    // 70003E38 
{0x0F12, 0xE59F},    // 70003E3A 
{0x0F12, 0xFF1C},    // 70003E3C 
{0x0F12, 0xE12F},    // 70003E3E 
{0x0F12, 0x7C2B},    // 70003E40 
{0x0F12, 0x0000},    // 70003E42 
{0x0F12, 0x4778},    // 70003E44 
{0x0F12, 0x46C0},    // 70003E46 
{0x0F12, 0xF004},    // 70003E48 
{0x0F12, 0xE51F},    // 70003E4A 
{0x0F12, 0x24C4},    // 70003E4C 
{0x0F12, 0x0001},    // 70003E4E 
{0x0F12, 0x4778},    // 70003E50 
{0x0F12, 0x46C0},    // 70003E52 
{0x0F12, 0xC000},    // 70003E54 
{0x0F12, 0xE59F},    // 70003E56 
{0x0F12, 0xFF1C},    // 70003E58 
{0x0F12, 0xE12F},    // 70003E5A 
{0x0F12, 0x3183},    // 70003E5C 
{0x0F12, 0x0000},    // 70003E5E 
{0x0F12, 0x4778},    // 70003E60 
{0x0F12, 0x46C0},    // 70003E62 
{0x0F12, 0xC000},    // 70003E64 
{0x0F12, 0xE59F},    // 70003E66 
{0x0F12, 0xFF1C},    // 70003E68 
{0x0F12, 0xE12F},    // 70003E6A 
{0x0F12, 0x302F},    // 70003E6C 
{0x0F12, 0x0000},    // 70003E6E 
{0x0F12, 0x4778},    // 70003E70 
{0x0F12, 0x46C0},    // 70003E72 
{0x0F12, 0xC000},    // 70003E74 
{0x0F12, 0xE59F},    // 70003E76 
{0x0F12, 0xFF1C},    // 70003E78 
{0x0F12, 0xE12F},    // 70003E7A 
{0x0F12, 0xEF07},    // 70003E7C 
{0x0F12, 0x0000},    // 70003E7E 
{0x0F12, 0x4778},    // 70003E80 
{0x0F12, 0x46C0},    // 70003E82 
{0x0F12, 0xC000},    // 70003E84 
{0x0F12, 0xE59F},    // 70003E86 
{0x0F12, 0xFF1C},    // 70003E88 
{0x0F12, 0xE12F},    // 70003E8A 
{0x0F12, 0x48FB},    // 70003E8C 
{0x0F12, 0x0000},    // 70003E8E 
{0x0F12, 0x4778},    // 70003E90 
{0x0F12, 0x46C0},    // 70003E92 
{0x0F12, 0xC000},    // 70003E94 
{0x0F12, 0xE59F},    // 70003E96 
{0x0F12, 0xFF1C},    // 70003E98 
{0x0F12, 0xE12F},    // 70003E9A 
{0x0F12, 0xF0B1},    // 70003E9C 
{0x0F12, 0x0000},    // 70003E9E 
{0x0F12, 0x4778},    // 70003EA0 
{0x0F12, 0x46C0},    // 70003EA2 
{0x0F12, 0xC000},    // 70003EA4 
{0x0F12, 0xE59F},    // 70003EA6 
{0x0F12, 0xFF1C},    // 70003EA8 
{0x0F12, 0xE12F},    // 70003EAA 
{0x0F12, 0xEEDF},    // 70003EAC 
{0x0F12, 0x0000},    // 70003EAE 
{0x0F12, 0x4778},    // 70003EB0 
{0x0F12, 0x46C0},    // 70003EB2 
{0x0F12, 0xC000},    // 70003EB4 
{0x0F12, 0xE59F},    // 70003EB6 
{0x0F12, 0xFF1C},    // 70003EB8 
{0x0F12, 0xE12F},    // 70003EBA 
{0x0F12, 0xAEF1},    // 70003EBC 
{0x0F12, 0x0000},    // 70003EBE 
{0x0F12, 0x4778},    // 70003EC0 
{0x0F12, 0x46C0},    // 70003EC2 
{0x0F12, 0xC000},    // 70003EC4 
{0x0F12, 0xE59F},    // 70003EC6 
{0x0F12, 0xFF1C},    // 70003EC8 
{0x0F12, 0xE12F},    // 70003ECA 
{0x0F12, 0xFD21},    // 70003ECC 
{0x0F12, 0x0000},    // 70003ECE 
{0x0F12, 0x4778},    // 70003ED0 
{0x0F12, 0x46C0},    // 70003ED2 
{0x0F12, 0xC000},    // 70003ED4 
{0x0F12, 0xE59F},    // 70003ED6 
{0x0F12, 0xFF1C},    // 70003ED8 
{0x0F12, 0xE12F},    // 70003EDA 
{0x0F12, 0x5027},    // 70003EDC 
{0x0F12, 0x0000},    // 70003EDE 
{0x0F12, 0x4778},    // 70003EE0 
{0x0F12, 0x46C0},    // 70003EE2 
{0x0F12, 0xC000},    // 70003EE4 
{0x0F12, 0xE59F},    // 70003EE6 
{0x0F12, 0xFF1C},    // 70003EE8 
{0x0F12, 0xE12F},    // 70003EEA 
{0x0F12, 0x04C9},    // 70003EEC 
{0x0F12, 0x0000},    // 70003EEE 
// End of Patch Data(Last : 70003EEEh)
// Total Size 2500 (s09C4)
// Addr : 352C , Size : 2498(9C2h) 
{0x1000, 0x0001},

{0x0028, 0xD000},
{0x002A, 0x0070},
{0x0F12, 0x0007},// clks_src_gf_force_enable

//TNP_USER_MBCV_CONTROL	
//TNP_AWB_MODUL_COMP	
//TNP_SINGLE_FRAME_CAPTURE	// "M_REG_ELF_Reserved3"  register was used for Option register					
//TNP_CAPTURE_DONE_INFO		// "Mon_DBG_GenInfo[15]" register was used for Capture Infor
//TNP_5CC_SENSOR_TUNE	

//added DTP 1st
{0x0028, 0x7000},
{0x002A, 0x2A5A},
{0x0F12, 0x0000},
{0x002A, 0x2A62},
{0x0F12, 0x0000},
{0x0028, 0xD000},
{0x002A, 0xF2AC},
{0x0F12, 0x0020},
{0x002A, 0xC200},
{0x0F12, 0x0000},
{0x002A, 0xC202},
{0x0F12, 0x0000},
//added DTP 1st end

//MBCV Control
{0x0028, 0x7000},
{0x002A, 0x04B4},
{0x0F12, 0x0064},

// AFIT by Normalized Brightness Tuning parameter
{0x0028, 0x7000},
{0x002A, 0x3302},
{0x0F12, 0x0000},	// on/off AFIT by NB option

{0x0F12, 0x0005},	// NormBR[0]
{0x0F12, 0x0019},	// NormBR[1]
{0x0F12, 0x0050},	// NormBR[2]
{0x0F12, 0x0300},	// NormBR[3]
{0x0F12, 0x0375},	// NormBR[4]

// Flash
{0x002A, 0x3F82},
{0x0F12, 0x0000},		// TNP_Regs_PreflashStart
{0x0F12, 0x0000},		// TNP_Regs_PreflashEnd
{0x0F12, 0x0260},		// TNP_Regs_PreWP_r
{0x0F12, 0x0240},	// TNP_Regs_PreWP_b

{0x002A, 0x3F98},		// BR Tuning
{0x0F12, 0x0100},		// TNP_Regs_BrRatioIn_0_
{0x0F12, 0x0150},
{0x0F12, 0x0200},
{0x0F12, 0x0300},
{0x0F12, 0x0400},

{0x0F12, 0x0100},		// TNP_Regs_BrRatioOut_0_
{0x0F12, 0x00A0},
{0x0F12, 0x0080},
{0x0F12, 0x0040},
{0x0F12, 0x0020},

{0x0F12, 0x0030},		// WP Tuning
{0x0F12, 0x0040},		// TNP_Regs_WPThresTbl_0_
{0x0F12, 0x0048},
{0x0F12, 0x0050},
{0x0F12, 0x0060},

{0x0F12, 0x0100},		// TNP_Regs_WPWeightTbl_0_
{0x0F12, 0x00C0},
{0x0F12, 0x0080},
{0x0F12, 0x000A},
{0x0F12, 0x0000},

{0x0F12, 0x0120},		// T_BR tune	
{0x0F12, 0x0150},		// TNP_Regs_FlBRIn_0_
{0x0F12, 0x0200},
 
{0x0F12, 0x003C},		//TNP_Regs_FlBRInOut_0_
{0x0F12, 0x003B},
{0x0F12, 0x002C},
 
{0x002A, 0x0430},		//REG_TC_FLS_Mode
{0x0F12, 0x0002},		
{0x002A, 0x3F80},		//TNP_Regs_FastFlashAlg
{0x0F12, 0x0000},

{0x002A, 0x165E},
{0x0F12, 0x0258}, //AWB R point
{0x0F12, 0x022D}, //AWB B point


///////////////////////////////////////////////////////////////////////////////////
// Analog & APS settings///////////////////////////////////////////////////////////
// This register is for FACTORY ONLY. If you change it without prior notification//
// YOU are RESPONSIBLE for the FAILURE that will happen in the future//////////////
///////////////////////////////////////////////////////////////////////////////////

//========================================================================================
// 5CC EVT0 analog register setting
// '10.07.14. Initial Draft
// '10.07.24. sE404=0000 -> 1FC0 (Depedestal 0 -> -64d)
// '10.08.16. sF410=0001 -> 0000 (for SHBN)
// '10.08.25. sF438=0020 -> 0002 (VTGSL=2.96V) by APS
//            sF43A=0020 -> 0001 (VRG=2.83V) by APS
// '10.09.28. sF402=1F02 -> 3F02 ([13]: pixel bias powerdown according to HADR) for Darkshading
//		    sF416=0000 -> 0001 (AAC_EN enable) for Darkshading
//========================================================================================
//============================= Analog & APS Control =====================================
{0x0028, 0xD000},
{0x002A, 0xF2AC},
{0x0F12, 0x0100},	// analog gain; 0200 x16, 0100 x8, 0080 x4, 0040 x2, 0020 x1
{0x002A, 0xF400},
{0x0F12, 0x001D},	// ldb_en[4], ld_en[3], clp_en[2](N/A), smp_en[1], dshut_en[0]
{0x0F12, 0x3F02},	// cds_test[15:0]; refer to the ATOP_TEST_INFORMATION.

{0x002A, 0xF40A},
{0x0F12, 0x0054},	// adc_sat[7:0]=84d (500mV)
{0x0F12, 0x0002},	// ms[2:0]; 2h@Normal, 2h@PLA, 1h@CNT.AVG
{0x0F12, 0x0008},	// rmp_option[7:0]; [3]SL_Low_PWR_SAVE On
{0x0F12, 0x0000},	// msoff_en; No MS if gain gain is lower than x2
{0x0F12, 0x00A4},	// rmp_init[7:0]

{0x002A, 0xF416},
{0x0F12, 0x0001},	// dbs_option[11:4], dbs_mode[3:2], dbs_bist_en[1], aac_en[0]

{0x002A, 0xF41E},
{0x0F12, 0x0065},	// comp2_bias[7:4], comp1_bias[3:0]

{0x002A, 0xF422},
{0x0F12, 0x0005},	// pix_bias[3:0]

{0x002A, 0xF426},
{0x0F12, 0x00D4},	// clp_lvl[7:0]

{0x002A, 0xF42A},
{0x0F12, 0x0001},	// ref_option[7:0]; [4]OB_PIX monit en, [3]Clamp monit en, [2]Monit amp en, [1]Clamp power-down, [0]CDS power-down during SL=low

{0x002A, 0xF42E},
{0x0F12, 0x0406},	// fb_lv[11:10], pd_fblv[9], capa_ctrl_en[8], pd_inrush_ctrl[7], pd_reg_ntg[6], pd_reg_tgsl[5], pd_reg_rg[4], pd_reg_pix[3], pd_ncp_rosc[2], pd_cp_rosc[1], pd_cp[0]

{0x002A, 0xF434},
{0x0F12, 0x0003},	// dbr_clk_sel[1:0]; PLL_mode=3h, ROSC_mode=0h
{0x0F12, 0x0004},	// reg_tune_pix[7:0]
{0x0F12, 0x0002},	// reg_tune_tgsl[7:0] (2.96V)
{0x0F12, 0x0001},	// reg_tune_rg[7:0] (2.83V)
{0x0F12, 0x0004},	// reg_tune_ntg[7:0]

{0x002A, 0xF446},
{0x0F12, 0x0000},	// blst_en_cintr[15:0]

{0x002A, 0xF466},
{0x0F12, 0x0000},	// srx_en[0]

{0x002A, 0x0054},
{0x0F12, 0x0028},	// pll_pd[10](0:enable, 1:disable), div_clk_en[0](0:enable, 1:disable)
{0x0F12, 0x8888},	// div_dbr[7:4]

{0x002A, 0xF132},
{0x0F12, 0x0206},	// tgr_frame_decription 4
{0x002A, 0xF152},
{0x0F12, 0x0206},	// tgr_frame_decription 7
{0x002A, 0xF1A2},
{0x0F12, 0x0200},	// tgr_frame_params_descriptor_3
{0x002A, 0xF1B2},
{0x0F12, 0x0202},	// tgr_frame_params_descriptor_6
//===========================================================================================

//============================= Line-ADLC Tuning ============================================
{0x002A, 0xE412},
{0x0F12, 0x0008},	// adlc_tune_offset_gr[7:0]
{0x0F12, 0x0008},	// adlc_tune_offset_r[7:0]
{0x0F12, 0x0010},	// adlc_tune_offset_b[7:0]
{0x0F12, 0x0010},	// adlc_tune_offset_gb[7:0]
{0x002A, 0xE42E},
{0x0F12, 0x0004},	// adlc_qec[2:0]
//===========================================================================================

//===================================================================
// AWB white locus setting - Have to be written after TnP
//===================================================================
{0x0028, 0x7000},
{0x002A, 0x1014},
{0x0F12, 0x0132},	//0138	//awbb_IntcR
{0x0F12, 0x010A},	//011C	//awbb_IntcB

//===================================================================
// AF
//===================================================================
//1. AF interface setting
{0x002A, 0x01A2},
{0x0F12, 0x0003}, //REG_TC_IPRM_CM_Init_AfModeType            // VCM_I2C actuator
{0x0F12, 0x0000}, //REG_TC_IPRM_CM_Init_PwmConfig1           // No PWM
{0x0F12, 0x0000}, //REG_TC_IPRM_CM_Init_PwmConfig2
{0x0F12, 0x0041}, //REG_TC_IPRM_CM_Init_GpioConfig1            // Use GPIO_4 for enable port
{0x0F12, 0x0000}, //REG_TC_IPRM_CM_Init_GpioConfig2
{0x0F12, 0x2A0C}, //REG_TC_IPRM_CM_Init_Mi2cBits            // Use GPIO_5 for SCL, GPIO_6 for SDA
{0x0F12, 0x0190}, //REG_TC_IPRM_CM_Init_Mi2cRateKhz            // MI2C Speed : 400KHz

//2. AF window setting
{0x002A, 0x022C},
{0x0F12, 0x0100},	//REG_TC_AF_FstWinStartX 
{0x0F12, 0x00E3},	//REG_TC_AF_FstWinStartY
{0x0F12, 0x0200},	//REG_TC_AF_FstWinSizeX 
{0x0F12, 0x0238},	//REG_TC_AF_FstWinSizeY
{0x0F12, 0x018C},	//REG_TC_AF_ScndWinStartX
{0x0F12, 0x0166},	//REG_TC_AF_ScndWinStartY
{0x0F12, 0x00E6},	//REG_TC_AF_ScndWinSizeX
{0x0F12, 0x0132},	//REG_TC_AF_ScndWinSizeY
{0x0F12, 0x0001},	//REG_TC_AF_WinSizesUpdated

//3. AF Fine Search Settings
{0x002A, 0x063A},
{0x0F12, 0x00C0}, //#skl_af_StatOvlpExpFactor
{0x002A, 0x064A},
{0x0F12, 0x0000},	//0000 //#skl_af_bAfStatOff
{0x002A, 0x1488},
{0x0F12, 0x0000}, //#af_search_usAeStable
{0x002A, 0x1494},
{0x0F12, 0x1002},	//#af_search_usSingleAfFlags, 1000- fine search disable, 1002- fine search on
{0x002A, 0x149E},
{0x0F12, 0x0003}, //#af_search_usFinePeakCount
{0x0F12, 0x0000}, //#af_search_usFineMaxScale
{0x002A, 0x142C},
{0x0F12, 0x0602},	//#af_pos_usFineStepNumSize
{0x002A, 0x14A2},
{0x0F12, 0x0000}, //#af_search_usCapturePolicy 0000 : Shutter_Priority_Current

//4.  AF Peak Threshold Setting
{0x002A, 0x1498},
{0x0F12, 0x0003},	//#af_search_usMinPeakSamples
{0x002A, 0x148A},
{0x0F12, 0x00CC},	//#af_search_usPeakThr  for 80%
{0x0F12, 0x00A0}, //#af_search_usPeakThrLow

//5.  AF Default Position
{0x002A, 0x1420},
{0x0F12, 0x0000},	//#af_pos_usHomePos
{0x0F12, 0x952F},	//#af_pos_usLowConfPos

//6. AF statistics
{0x002A, 0x14B4},
{0x0F12, 0x0280}, //#af_search_usConfThr_4_  LowEdgeBoth GRAD
{0x002A, 0x14C0},
{0x0F12, 0x03A0}, //#af_search_usConfThr_10_ LowLight HPF
{0x0F12, 0x0320}, //#af_search_usConfThr_11_
{0x002A, 0x14F4},
{0x0F12, 0x0030}, //#af_stat_usMinStatVal
{0x002A, 0x1514},
{0x0F12, 0x0060}, //#af_scene_usSceneLowNormBrThr
// AF Scene Settings
{0x002A, 0x151E},
{0x0F12, 0x0003}, //#af_scene_usSaturatedScene

//7. AF Lens Position Table Settings
{0x002A, 0x1434},
{0x0F12, 0x0010},  //#af_pos_usTableLastInd, 10h + 1h = 17 Steps 

{0x0F12, 0x0030},  //#af_pos_usTable_0_  48  
{0x0F12, 0x0033},  //#af_pos_usTable_1_  51   
{0x0F12, 0x0036},  //#af_pos_usTable_2_  54	
{0x0F12, 0x0039},  //#af_pos_usTable_3_  57	
{0x0F12, 0x003D},  //#af_pos_usTable_4_  61   
{0x0F12, 0x0041},  //#af_pos_usTable_5_  65	
{0x0F12, 0x0045},  //#af_pos_usTable_6_  69	
{0x0F12, 0x0049},  //#af_pos_usTable_7_  73	
{0x0F12, 0x004E},  //#af_pos_usTable_8_  78	
{0x0F12, 0x0053},  //#af_pos_usTable_9_  83	
{0x0F12, 0x0059},  //#af_pos_usTable_10_ 89	
{0x0F12, 0x0060},  //#af_pos_usTable_11_ 104	
{0x0F12, 0x0068},  //#af_pos_usTable_12_ 109	
{0x0F12, 0x0072},  //#af_pos_usTable_13_ 114	
{0x0F12, 0x007D},  //#af_pos_usTable_14_ 125
{0x0F12, 0x0089},  //#af_pos_usTable_15_ 137	
{0x0F12, 0x0096},  //#af_pos_usTable_16_ 150

//8. VCM AF driver with PWM/I2C 
{0x002A, 0x1558},
{0x0F12, 0x8000}, //#afd_usParam[0]  I2C power down command
{0x0F12, 0x0006}, //#afd_usParam[1]  Position Right Shift 
{0x0F12, 0x3FF0}, //#afd_usParam[2]  I2C Data Mask
{0x0F12, 0x03E8}, //#afd_usParam[3]  PWM Period
{0x0F12, 0x0000}, //#afd_usParam[4]  PWM Divider
{0x0F12, 0x0020}, //#afd_usParam[5]  SlowMotion Delay    reduce lens collision noise.
{0x0F12, 0x0010}, //#afd_usParam[6]  SlowMotion Threshold
{0x0F12, 0x0008}, //#afd_usParam[7]  Signal Shaping
{0x0F12, 0x0040}, //#afd_usParam[8]  Signal Shaping level 
{0x0F12, 0x0080}, //#afd_usParam[9]  Signal Shaping level
{0x0F12, 0x00C0}, //#afd_usParam[10] Signal Shaping level
{0x0F12, 0x00E0}, //#afd_usParam[11] Signal Shaping level

{0x002A, 0x0224},
{0x0F12, 0x0003},	//REG_TC_AF_AfCmd	//Initialize AF subsystem (AF driver, AF algorithm)

//===================================================================
// Flash setting
//===================================================================
{0x002A, 0x018C},
{0x0F12, 0x0001},	//REG_TC_IPRM_AuxConfig	// bit[0] : Flash is in use, bit[1] : Mechanical shutter is in use // 0 : do not use, 1 : use
{0x0F12, 0x0003},	//REG_TC_IPRM_AuxPolarity	// bit[0] : Flash polarity (1 is active high), bit[1] : Mechanical shutter polarity (1 is active high)
{0x0F12, 0x0003},	//REG_TC_IPRM_AuxGpios	//1-4 : Flash GPIO number, If GPIO number is overaped with AF GPIO, F/W could be stop.

//===================================================================
// 1-H timing setting
//===================================================================
{0x002A, 0x1686},
{0x0F12, 0x005C},	//senHal_uAddColsBin
{0x0F12, 0x005C},	//senHal_uAddColsNoBin
{0x0F12, 0x085C},	//senHal_uMinColsHorBin
{0x0F12, 0x005C},	//senHal_uMinColsNoHorBin
{0x0F12, 0x025A},	//senHal_uMinColsAddAnalogBin

//===================================================================
// Forbidden area setting
//===================================================================
{0x002A, 0x1844},
{0x0F12, 0x0000},	//senHal_bSRX    //SRX off

{0x002A, 0x1680},
{0x0F12, 0x0002},	//senHal_NExpLinesCheckFine	//0004    //extend Forbidden area line

//===================================================================
// Preview subsampling mode
//===================================================================
{0x002A, 0x18F8},
{0x0F12, 0x0001},	//senHal_bAACActiveWait2Start
{0x002A, 0x18F6},
{0x0F12, 0x0001},	//senHal_bAlwaysAAC
{0x002A, 0x182C},
{0x0F12, 0x0001},	//senHal_bSenAAC
{0x002A, 0x0EE4},
{0x0F12, 0x0001},	//setot_bUseDigitalHbin
{0x002A, 0x1674},
{0x0F12, 0x0002},	//senHal_SenBinFactor	// 2:2x2, 4:4x4
{0x0F12, 0x0002},	//senHal_SamplingType	// 0:Full, 1:digital, 2:PLA, 3:CA
{0x0F12, 0x0000},	//senHal_SamplingMode	// 0:2x2,	1:4x4

//===================================================================
// PLL setting for Max frequency (EVT0.1) 2010.08.05 - Do not remove
//===================================================================
{0x002A, 0x19AE},
{0x0F12, 0xEA60},	//pll_uMaxSysFreqKhz
{0x0F12, 0x7530},	//pll_uMaxPVIFreq4KH
{0x002A, 0x19C2},
{0x0F12, 0x7530},	//pll_uMaxMIPIFreq4KH
{0x002A, 0x0244},
{0x0F12, 0x7530},	//REG_0TC_PCFG_usMaxOut4KHzRate
{0x002A, 0x0336},
{0x0F12, 0x7530},	//REG_0TC_CCFG_usMaxOut4KHzRate

//===================================================================
// Init Parameters
//===================================================================
//MCLK
{0x002A, 0x0188},
{0x0F12, 0x6590},// - MCLKL 26Mhz s5DC0-24Mhz	//REG_TC_IPRM_InClockLSBs
{0x0F12, 0x0000},	//REG_TC_IPRM_InClockMSBs
{0x002A, 0x01B2},
{0x0F12, 0x0000},	//REG_TC_IPRM_UseNPviClocks
{0x0F12, 0x0001},	//REG_TC_IPRM_UseNMipiClocks
{0x002A, 0x01B8},
{0x0F12, 0x0000},	//REG_TC_IPRM_bBlockInternalPllCalc	//1:pll bypass

//SCLK & PCLK
{0x0F12, 0x32C8},	//REG_TC_IPRM_OpClk4KHz_0	//52Mhz	//2EE0	//48Mhz 1F40	//32MHz
{0x0F12, 0x36A4}, //s32C8//s34BC	//REG_TC_IPRM_MinOutRate4KHz_0	//53.936Mhz	//32A8	//51.872MHz
{0x0F12, 0x3AA4}, //s36B0//s34BC	//REG_TC_IPRM_MaxOutRate4KHz_0	//54.064Mhz	//32E8	//52.128MHz

//SCLK & PCLK
{0x0F12, 0x1F40},	//REG_TC_IPRM_OpClk4KHz_1	//52Mhz	//2EE0	//48Mhz 1F40	//32MHz     
{0x0F12, 0x36A4}, //s32C8//s34BC	//REG_TC_IPRM_MinOutRate4KHz_1	//53.936Mhz	//32A8	//51.872MHz                
{0x0F12, 0x3AA4}, //s36B0//s34BC	//REG_TC_IPRM_MaxOutRate4KHz_1	//54.064Mhz	//32E8	//52.128MHz                

{0x002A, 0x01CC},
{0x0F12, 0x0001},	//REG_TC_IPRM_InitParamsUpdated

//===================================================================
// JPEG Thumbnail Setting
//===================================================================
{0x002A, 0x0428},
//{0x0F12, 0x0001},		//REG_TC_THUMB_Thumb_bActive			Thumbnail Enable
{0x0F12, 0x0000},
{0x0F12, 0x0140},		//REG_TC_THUMB_Thumb_uWidth				Thumbnail Width
{0x0F12, 0x00F0},		//REG_TC_THUMB_Thumb_uHeight			Thumbnail Height
{0x0F12, 0x0005},		//REG_TC_THUMB_Thumb_Format				Thumbnail Output Format 5:YUV

//===================================================================
// Input Width & Height
//===================================================================
{0x002A, 0x01F6},
{0x0F12, 0x0800},	//REG_TC_GP_PrevReqInputWidth		//Sensor Crop Width	2048
{0x0F12, 0x0600},	//REG_TC_GP_PrevReqInputHeight	//Sensor Crop Height 1536
{0x0F12, 0x0000},	//REG_TC_GP_PrevInputWidthOfs		//Sensor HOffset 0
{0x0F12, 0x0000},	//REG_TC_GP_PrevInputHeightOfs	//Sensor VOffset 0
{0x0F12, 0x0800},	//REG_TC_GP_CapReqInputWidth		//Sensor Crop Width	2048
{0x0F12, 0x0600},	//REG_TC_GP_CapReqInputHeight		//Sensor Crop Height 1536
{0x0F12, 0x0000},	//REG_TC_GP_CapInputWidthOfs		//Sensor HOffset 0
{0x0F12, 0x0000},	//REG_TC_GP_CapInputHeightOfs		//Sensor VOffset 0

{0x002A, 0x0216},
{0x0F12, 0x0001},	//REG_TC_GP_bUseReqInputInPre
{0x0F12, 0x0001},	//REG_TC_GP_bUseReqInputInCap

{0x002A, 0x043C},
{0x0F12, 0x0800},	//REG_TC_PZOOM_ZoomInputWidth
{0x0F12, 0x0600},	//REG_TC_PZOOM_ZoomInputHeight
{0x0F12, 0x0000},	//REG_TC_PZOOM_ZoomInputWidthOfs
{0x0F12, 0x0000},	//REG_TC_PZOOM_ZoomInputHeightOfs

//===================================================================
// Preview 0 64s480 system 52M PCLK 54M
//===================================================================
{0x002A, 0x023E},
{0x0F12, 0x0280},	//REG_0TC_PCFG_usWidth
{0x0F12, 0x01E0},	//REG_0TC_PCFG_usHeight
{0x0F12, 0x0005},	//REG_0TC_PCFG_Format
{0x0F12, 0x3AA4}, //s36B0//s34BC	//REG_0TC_PCFG_usMaxOut4KHzRate
{0x0F12, 0x36A4}, //s32C8//s34BC	//REG_0TC_PCFG_usMinOut4KHzRate

{0x002A, 0x024C},
{0x0F12, 0x0012},	//REG_0TC_PCFG_PVIMask    => cmk 2010.10.29 s0042 => s0052 Invert Y, C order
{0x0F12, 0x0010},	//REG_0TC_PCFG_OIFMask

{0x002A, 0x0254},
{0x0F12, 0x0000},	//REG_0TC_PCFG_uClockInd
{0x0F12, 0x0000},	//REG_0TC_PCFG_usFrTimeType
{0x0F12, 0x0001},	//REG_0TC_PCFG_FrRateQualityType
{0x0F12, 0x03E8},	//REG_0TC_PCFG_usMaxFrTimeMsecMult10	//max frame time : 30fps 014D 15fps 029a; a6a - 3.75 fps; 0535 - 7.5FPS
{0x0F12, 0x014D},	//REG_0TC_PCFG_usMinFrTimeMsecMult10	//min frame time : 30fps 014D 15fps 029a; a6a - 3.75 fps; 0535 - 7.5FPS

{0x0F12, 0x0000},	//REG_0TC_PCFG_bSmearOutput
{0x0F12, 0x0000},	//REG_0TC_PCFG_sSaturation
{0x0F12, 0x0000},	//REG_0TC_PCFG_sSharpBlur
{0x0F12, 0x0000},	//REG_0TC_PCFG_sColorTemp
{0x0F12, 0x0000},	//REG_0TC_PCFG_uDeviceGammaIndex
{0x0F12, 0x0000},	//REG_0TC_PCFG_uPrevMirror
{0x0F12, 0x0000},	//REG_0TC_PCFG_uCaptureMirror
{0x0F12, 0x0000},	//REG_0TC_PCFG_uRotation

//===================================================================
// Capture 0 2048x1536 system 52M PCLK 54M
//===================================================================
{0x002A, 0x032E},
{0x0F12, 0x0000},	//REG_0TC_CCFG_uCaptureMode

{0x0F12, 0x0800},	//REG_0TC_CCFG_usWidth
{0x0F12, 0x0600},	//REG_0TC_CCFG_usHeight
{0x0F12, 0x0009},	//REG_0TC_CCFG_Format   //PCAM 5:YUV   9:JPEG
{0x0F12, 0x3AA4}, //s36B0//s34BC	//REG_0TC_CCFG_usMaxOut4KHzRate
{0x0F12, 0x36A4}, //s32C8//s34BC	//REG_0TC_CCFG_usMinOut4KHzRate

{0x002A, 0x033E},
{0x0F12, 0x0012},	//REG_0TC_CCFG_PVIMask        => cmk 2010.10.29 s0042 => s0052 Invert Y, C order
{0x0F12, 0x0010},	//REG_0TC_CCFG_OIFMask
//{0x0F12, 0x03C0},	//REG_0TC_CCFG_usJpegPacketSize
//{0x0F12, 0x0C00},       //REG_0TC_CCFG_usJpegTotalPackets

{0x002A, 0x0346},
{0x0F12, 0x0000},	//REG_0TC_CCFG_uClockInd
{0x0F12, 0x0002},	//REG_0TC_CCFG_usFrTimeType
{0x0F12, 0x0002},	//REG_0TC_CCFG_FrRateQualityType
{0x0F12, 0x0535},	//REG_0TC_CCFG_usMaxFrTimeMsecMult10
{0x0F12, 0x029A},	//REG_0TC_CCFG_usMinFrTimeMsecMult10
{0x0F12, 0x0000},	//REG_0TC_CCFG_bSmearOutput
{0x0F12, 0x0000},	//REG_0TC_CCFG_sSaturation
{0x0F12, 0x0000},	//REG_0TC_CCFG_sSharpBlur
{0x0F12, 0x0000},	//REG_0TC_CCFG_sColorTemp
{0x0F12, 0x0000},	//REG_0TC_CCFG_uDeviceGammaIndex

{0x002A, 0x0426},
{0x0F12, 0x0055},	//REG_TC_BRC_usCaptureQuality

//===================================================================
// AFC
//===================================================================
//Auto
{0x002A, 0x0F08},
{0x0F12, 0x0000},	//AFC_Default60Hz   01:60hz 00:50Hz
{0x002A, 0x04A4},
{0x0F12, 0x067F},	//REG_TC_DBG_AutoAlgEnBits, 065f : Manual AFC on   067f : Manual AFC off



//===================================================================
// AE - shutter
//===================================================================
//****************************************/
// AE 2009 03 08 - based on TN
//****************************************/
//============================================================
// Frame rate setting
//============================================================
// How to set
// 1. Exposure value
// dec2hex((1 / (frame rate you want(ms))) * 100d * 5d)
//
//
// 2. Analog Digital gain
// dec2hex((Analog gain you want) * 256d)
//              Ex1) Simple Caculation for x3.25?:   3.25x256 = 832[dec] = 0340[hex]
//============================================================
//MBR
{0x002A, 0x01DE},
{0x0F12, 0x0000},	//REG_TC_bUseMBR	//MBR off
//MBR off is needed to prevent a shorter integration time when the scene has blurring in Night shot

//AE_Target
{0x002A, 0x1308},
{0x0F12, 0x003E},	//TVAR_ae_BrAve
{0x002A, 0x130E},
{0x0F12, 0x000F},	//ae_StatMode
//ae_StatMode bit[3] BLC has to be bypassed to prevent AE weight change, especially backlight scene

//added
{0x002A, 0x2A5A},
{0x0F12, 0x0000},
{0x002A, 0x2A62},
{0x0F12, 0x0000},
{0x0028, 0xD000},
{0x002A, 0xF2AC},
{0x0F12, 0x0020},
{0x002A, 0xC200},
{0x0F12, 0x0000},
{0x002A, 0xC202},
{0x0F12, 0x0000},
{0x0028, 0x7000},


//AE_state
{0x002A, 0x04EE},
{0x0F12, 0x010E},	//#lt_uLimitHigh
{0x0F12, 0x00F5},	//#lt_uLimitLow

//PREVIEW
{0x002A, 0x0208},
{0x0F12, 0x0000},	//REG_TC_GP_ActivePrevConfig
{0x002A, 0x0210},
{0x0F12, 0x0000},	//REG_TC_GP_ActiveCapConfig
{0x002A, 0x020C},
{0x0F12, 0x0001},	//REG_TC_GP_PrevOpenAfterChange
{0x002A, 0x01F4},
{0x0F12, 0x0001},	//REG_TC_GP_NewConfigSync
{0x002A, 0x020A},
{0x0F12, 0x0001},	//REG_TC_GP_PrevConfigChanged
{0x002A, 0x0212},
{0x0F12, 0x0001},	//REG_TC_GP_CapConfigChanged
{0x002A, 0x01E8},
{0x0F12, 0x0000},	//REG_TC_GP_EnableCapture
{0x0F12, 0x0001},	//REG_TC_GP_EnableCaptureChanged

{0xFFFF, 0x0064},   //Delay 100ms


//===================================================================
// AFIT_DTP
//===================================================================

//	param_start	afit_uNoiseIndInDoor
{0x002A, 0x085C},
{0x0F12, 0x004A}, //0049//#afit_uNoiseIndInDoor_0_
{0x0F12, 0x004E}, //005F//#afit_uNoiseIndInDoor_1_
{0x0F12, 0x00CB}, //00CB//#afit_uNoiseIndInDoor_2_
{0x0F12, 0x01C0}, //01E0//#afit_uNoiseIndInDoor_3_
{0x0F12, 0x0200}, //0220//#afit_uNoiseIndInDoor_4_

{0x002A, 0x08C0},
{0x0F12, 0x0000}, //0007//700008C0//AFIT16_BRIGHTNESS
{0x0F12, 0x0000}, //0000//700008C2           //AFIT16_CONTRAST
{0x0F12, 0x0000}, //0000//700008C4           //AFIT16_SATURATION
{0x0F12, 0x0000}, //0000//700008C6           //AFIT16_SHARP_BLUR
{0x0F12, 0x0000}, //0000//700008C8           //AFIT16_GLAMOUR
{0x0F12, 0x00C1}, //00C1//700008CA           //AFIT16_sddd8a_edge_high
{0x0F12, 0x0000}, //0000//700008CC        
{0x0F12, 0x03FF}, //03FF//700008CE           //AFIT16_Demosaicing_iSatVal
{0x0F12, 0x009C}, //009C//700008D0           //AFIT16_Sharpening_iReduceEdgeThresh
{0x0F12, 0x017C}, //017C//700008D2           //AFIT16_demsharpmix1_iRGBOffset
{0x0F12, 0x03FF}, //03FF//700008D4           //AFIT16_demsharpmix1_iDemClamp
{0x0F12, 0x000C}, //000C//700008D6           //AFIT16_demsharpmix1_iLowThreshold
{0x0F12, 0x0010}, //0010//700008D8           //AFIT16_demsharpmix1_iHighThreshold
{0x0F12, 0x012C}, //012C//700008DA           //AFIT16_demsharpmix1_iLowBright
{0x0F12, 0x03E8}, //03E8//700008DC           //AFIT16_demsharpmix1_iHighBright
{0x0F12, 0x0046}, //0046//700008DE           //AFIT16_demsharpmix1_iLowSat
{0x0F12, 0x005A}, //005A//700008E0           //AFIT16_demsharpmix1_iHighSat
{0x0F12, 0x0070}, //0070//700008E2           //AFIT16_demsharpmix1_iTune
{0x0F12, 0x0001}, //0010//700008E4//AFIT16_demsharpmix1_iHystThLow
{0x0F12, 0x0000}, //0010//700008E6//AFIT16_demsharpmix1_iHystThHigh
{0x0F12, 0x0320}, //01F4//700008E8//AFIT16_demsharpmix1_iHystCenter
{0x0F12, 0x006E}, //003C//700008EA           //AFIT16_Sharpening_iLowSharpClamp
{0x0F12, 0x0014}, //0008//700008EC           //AFIT16_Sharpening_iHighSharpClamp
{0x0F12, 0x003C}, //003C//700008EE           //AFIT16_Sharpening_iLowSharpClamp_Bin
{0x0F12, 0x001E}, //001E//700008F0           //AFIT16_Sharpening_iHighSharpClamp_Bin
{0x0F12, 0x003C}, //003C//700008F2           //AFIT16_Sharpening_iLowSharpClamp_sBin
{0x0F12, 0x001E}, //001E//700008F4           //AFIT16_Sharpening_iHighSharpClamp_sBin
{0x0F12, 0x0A24}, //0A24//700008F6           //AFIT8_sddd8a_edge_low [7:0] AFIT8_sddd8a_repl_thresh [15:8]
{0x0F12, 0x1701}, //1701//700008F8           //AFIT8_sddd8a_repl_force [7:0] AFIT8_sddd8a_sat_level [15:8]
{0x0F12, 0x0229}, //0229//700008FA           //AFIT8_sddd8a_sat_thr[7:0] AFIT8_sddd8a_sat_mpl [15:8]
{0x0F12, 0x1403}, //1403//700008FC           //AFIT8_sddd8a_sat_noise[7:0] AFIT8_sddd8a_iMaxSlopeAllowed [15:8]
{0x0F12, 0x0004}, //0004//700008FE           //AFIT8_sddd8a_iHotThreshHigh[7:0] AFIT8_sddd8a_iHotThreshLow [15:8]
{0x0F12, 0x0300}, //0300//70000900           //AFIT8_sddd8a_iColdThreshHigh[7:0] AFIT8_sddd8a_iColdThreshLow [15:8]
{0x0F12, 0x0000}, //0000//70000902           //AFIT8_sddd8a_AddNoisePower1[7:0] AFIT8_sddd8a_AddNoisePower2 [15:8]
{0x0F12, 0x02FF}, //02FF//70000904           //AFIT8_sddd8a_iSatSat[7:0] AFIT8_sddd8a_iRadialTune [15:8]
{0x0F12, 0x052D}, //09E8//70000906           //AFIT8_sddd8a_iRadialLimit [7:0] AFIT8_sddd8a_iRadialPower [15:8]
{0x0F12, 0x1414}, //1414//70000908           //AFIT8_sddd8a_iLowMaxSlopeAllowed [7:0] AFIT8_sddd8a_iHighMaxSlopeAllowed [15:8]
{0x0F12, 0x0301}, //0301//7000090A           //AFIT8_sddd8a_iLowSlopeThresh[7:0] AFIT8_sddd8a_iHighSlopeThresh [15:8]
{0x0F12, 0x0007}, //0007//7000090C           //AFIT8_sddd8a_iSquaresRounding [7:0]
{0x0F12, 0x1000}, //4000//7000090E        
{0x0F12, 0x2003}, //7803//70000910        
{0x0F12, 0x101A}, //3C50//70000912        
{0x0F12, 0x0010}, //003C//70000914        
{0x0F12, 0x1E80}, //1E80//70000916           //AFIT8_Demosaicing_iCentGrad[7:0] AFIT8_Demosaicing_iMonochrom[15:8]
{0x0F12, 0x1E08}, //1E08//70000918           //AFIT8_Demosaicing_iDecisionThresh[7:0] AFIT8_Demosaicing_iDesatThresh[15:8]
{0x0F12, 0x000A}, //000A//7000091A           //AFIT8_Demosaicing_iEnhThresh[7:0] AFIT8_Demosaicing_iGRDenoiseVal[15:8]
{0x0F12, 0x0000}, //0000//7000091C           //AFIT8_Demosaicing_iGBDenoiseVal[7:0] AFIT8_Demosaicing_iNearGrayDesat[15:8]
{0x0F12, 0x120A}, //120A//7000091E           //AFIT8_Demosaicing_iDFD_ReduceCoeff[7:0] AFIT8_Sharpening_iMSharpen[15:8]
{0x0F12, 0x1400}, //0F00//70000920           //AFIT8_Sharpening_iMShThresh[7:0] AFIT8_Sharpening_iWSharpen[15:8]
{0x0F12, 0x0200}, //0200//70000922           //AFIT8_Sharpening_iWShThresh[7:0] AFIT8_Sharpening_nSharpWidth[15:8]
{0x0F12, 0xFF00}, //FF00//70000924           //AFIT8_Sharpening_iReduceNegative[7:0] AFIT8_Sharpening_iShDespeckle[15:8]
{0x0F12, 0x0200}, //0200//70000926           //AFIT8_demsharpmix1_iRGBMultiplier[7:0] AFIT8_demsharpmix1_iFilterPower[15:8]
{0x0F12, 0x1B11}, //1B11//70000928           //AFIT8_demsharpmix1_iBCoeff[7:0] AFIT8_demsharpmix1_iGCoeff[15:8]
{0x0F12, 0x0000}, //0000//7000092A           //AFIT8_demsharpmix1_iWideMult[7:0] AFIT8_demsharpmix1_iNarrMult[15:8]
{0x0F12, 0x0009}, //0009//7000092C           //AFIT8_demsharpmix1_iHystFalloff[7:0] AFIT8_demsharpmix1_iHystMinMult[15:8]
{0x0F12, 0x0406}, //0406//7000092E           //AFIT8_demsharpmix1_iHystWidth[7:0] AFIT8_demsharpmix1_iHystFallLow[15:8]
{0x0F12, 0x0605}, //0605//70000930           //AFIT8_demsharpmix1_iHystFallHigh[7:0] AFIT8_demsharpmix1_iHystTune[15:8]
{0x0F12, 0x0307}, //0307//70000932        
{0x0F12, 0x0609}, //0609//70000934        
{0x0F12, 0x1C07}, //2C07//70000936        
{0x0F12, 0x1414}, //142C//70000938        
{0x0F12, 0x0510}, //0718//7000093A//[15:8]iUVNRStrengthL [7:0]iMaxThreshH
{0x0F12, 0x8005}, //8007//7000093C//[7:0]iUVNRStrengthH AFIT8_byr_cgras_iShadingPower[15:8]
{0x0F12, 0x0080}, //0880//7000093E           //AFIT8_RGBGamma2_iLinearity [7:0]  AFIT8_RGBGamma2_iDarkReduce [15:8]
{0x0F12, 0x0080}, //0B50//70000940           //AFIT8_ccm_oscar_iSaturation[7:0]   AFIT8_RGB2YUV_iYOffset [15:8]
{0x0F12, 0x0080}, //0080//70000942           //AFIT8_RGB2YUV_iRGBGain [7:0]   AFIT8_RGB2YUV_iSaturation [15:8]
{0x0F12, 0x0101}, //0101//70000944           //AFIT8_sddd8a_iClustThresh_H [7:0]  AFIT8_sddd8a_iClustThresh_C [15:8]
{0x0F12, 0x0707}, //0707//70000946           //AFIT8_sddd8a_iClustMulT_H [7:0]   AFIT8_sddd8a_iClustMulT_C [15:8]
{0x0F12, 0x4B01}, //4601//70000948           //AFIT8_sddd8a_nClustLevel_H [7:0]   AFIT8_sddd8a_DispTH_Low [15:8]
{0x0F12, 0x184B}, //C844//7000094A           //AFIT8_sddd8a_DispTH_High [7:0]   AFIT8_sddd8a_iDenThreshLow [15:8]
{0x0F12, 0x500C}, //50C8//7000094C           //AFIT8_sddd8a_iDenThreshHigh[7:0]   AFIT8_Demosaicing_iEdgeDesat [15:8]
{0x0F12, 0x0500}, //0500//7000094E           //AFIT8_Demosaicing_iEdgeDesatThrLow [7:0]   AFIT8_Demosaicing_iEdgeDesatThrHigh [15:8]
{0x0F12, 0x1C03}, //0003//70000950           //AFIT8_Demosaicing_iEdgeDesatLimit[7:0]  AFIT8_Demosaicing_iDemSharpenLow [15:8]
{0x0F12, 0x0A08}, //1C01//70000952           //AFIT8_Demosaicing_iDemSharpenHigh[7:0]   AFIT8_Demosaicing_iDemSharpThresh [15:8]
{0x0F12, 0x070C}, //0714//70000954           //AFIT8_Demosaicing_iDemShLowLimit [7:0]   AFIT8_Demosaicing_iDespeckleForDemsharp [15:8]
{0x0F12, 0x1428}, //1464//70000956           //AFIT8_Demosaicing_iDemBlurLow[7:0]   AFIT8_Demosaicing_iDemBlurHigh [15:8]
{0x0F12, 0x6401}, //5A04//70000958           //AFIT8_Demosaicing_iDemBlurRange[7:0]   AFIT8_Sharpening_iLowSharpPower [15:8]                         
{0x0F12, 0x281E}, //3C1E//7000095A           //AFIT8_Sharpening_iHighSharpPower[7:0]   AFIT8_Sharpening_iLowShDenoise [15:8]
{0x0F12, 0x200F}, //400F//7000095C           //AFIT8_Sharpening_iHighShDenoise [7:0]   AFIT8_Sharpening_iReduceEdgeMinMult [15:8]
{0x0F12, 0x0204}, //0204//7000095E           //AFIT8_Sharpening_iReduceEdgeSlope [7:0]  AFIT8_demsharpmix1_iWideFiltReduce [15:8]
{0x0F12, 0x4603}, //1403//70000960           //AFIT8_demsharpmix1_iNarrFiltReduce [7:0]  AFIT8_sddd8a_iClustThresh_H_Bin [15:8]
{0x0F12, 0x0146}, //0114//70000962           //AFIT8_sddd8a_iClustThresh_C_Bin [7:0]   AFIT8_sddd8a_iClustMulT_H_Bin [15:8]
{0x0F12, 0x0101}, //0101//70000964           //AFIT8_sddd8a_iClustMulT_C_Bin [7:0]   AFIT8_sddd8a_nClustLevel_H_Bin [15:8]
{0x0F12, 0x1C46}, //4446//70000966           //AFIT8_sddd8a_DispTH_Low_Bin [7:0]   AFIT8_sddd8a_DispTH_High_Bin [15:8]
{0x0F12, 0x1819}, //646E//70000968           //AFIT8_sddd8a_iDenThreshLow_Bin [7:0]   AFIT8_sddd8a_iDenThreshHigh_Bin [15:8]
{0x0F12, 0x0028}, //0028//7000096A           //AFIT8_Demosaicing_iEdgeDesat_Bin[7:0]   AFIT8_Demosaicing_iEdgeDesatThrLow_Bin [15:8]
{0x0F12, 0x030A}, //030A//7000096C           //AFIT8_Demosaicing_iEdgeDesatThrHigh_Bin [7:0]  AFIT8_Demosaicing_iEdgeDesatLimit_Bin [15:8]
{0x0F12, 0x0000}, //0000//7000096E           //AFIT8_Demosaicing_iDemSharpenLow_Bin [7:0]  AFIT8_Demosaicing_iDemSharpenHigh_Bin [15:8]
{0x0F12, 0x141E}, //141E//70000970           //AFIT8_Demosaicing_iDemSharpThresh_Bin [7:0]  AFIT8_Demosaicing_iDemShLowLimit_Bin [15:8]
{0x0F12, 0xFF07}, //FF07//70000972           //AFIT8_Demosaicing_iDespeckleForDemsharp_Bin [7:0]  AFIT8_Demosaicing_iDemBlurLow_Bin [15:8]
{0x0F12, 0x0432}, //0432//70000974           //AFIT8_Demosaicing_iDemBlurHigh_Bin [7:0]  AFIT8_Demosaicing_iDemBlurRange_Bin [15:8]
{0x0F12, 0x0000}, //0000//70000976           //AFIT8_Sharpening_iLowSharpPower_Bin [7:0]  AFIT8_Sharpening_iHighSharpPower_Bin [15:8]
{0x0F12, 0x0F0F}, //0F0F//70000978           //AFIT8_Sharpening_iLowShDenoise_Bin [7:0]  AFIT8_Sharpening_iHighShDenoise_Bin [15:8]
{0x0F12, 0x0440}, //0440//7000097A           //AFIT8_Sharpening_iReduceEdgeMinMult_Bin [7:0]  AFIT8_Sharpening_iReduceEdgeSlope_Bin [15:8]
{0x0F12, 0x0302}, //0302//7000097C           //AFIT8_demsharpmix1_iWideFiltReduce_Bin [7:0]  AFIT8_demsharpmix1_iNarrFiltReduce_Bin [15:8]
{0x0F12, 0x1414}, //1414//7000097E           //AFIT8_sddd8a_iClustThresh_H_sBin[7:0]   AFIT8_sddd8a_iClustThresh_C_sBin [15:8]
{0x0F12, 0x0101}, //0101//70000980           //AFIT8_sddd8a_iClustMulT_H_sBin [7:0]   AFIT8_sddd8a_iClustMulT_C_sBin [15:8]
{0x0F12, 0x1801}, //4601//70000982           //AFIT8_sddd8a_nClustLevel_H_sBin [7:0]   AFIT8_sddd8a_DispTH_Low_sBin [15:8]
{0x0F12, 0x191C}, //6E44//70000984           //AFIT8_sddd8a_DispTH_High_sBin [7:0]   AFIT8_sddd8a_iDenThreshLow_sBin [15:8]
{0x0F12, 0x2818}, //2864//70000986           //AFIT8_sddd8a_iDenThreshHigh_sBin[7:0]   AFIT8_Demosaicing_iEdgeDesat_sBin [15:8]
{0x0F12, 0x0A00}, //0A00//70000988           //AFIT8_Demosaicing_iEdgeDesatThrLow_sBin [7:0]  AFIT8_Demosaicing_iEdgeDesatThrHigh_sBin [15:8]
{0x0F12, 0x1403}, //0003//7000098A           //AFIT8_Demosaicing_iEdgeDesatLimit_sBin [7:0]  AFIT8_Demosaicing_iDemSharpenLow_sBin [15:8]
{0x0F12, 0x1400}, //1E00//7000098C           //AFIT8_Demosaicing_iDemSharpenHigh_sBin [7:0]  AFIT8_Demosaicing_iDemSharpThresh_sBin [15:8]
{0x0F12, 0x0714}, //0714//7000098E           //AFIT8_Demosaicing_iDemShLowLimit_sBin [7:0]  AFIT8_Demosaicing_iDespeckleForDemsharp_sBin [15:8]
{0x0F12, 0x32FF}, //32FF//70000990           //AFIT8_Demosaicing_iDemBlurLow_sBin [7:0]  AFIT8_Demosaicing_iDemBlurHigh_sBin [15:8]
{0x0F12, 0x0004}, //0004//70000992           //AFIT8_Demosaicing_iDemBlurRange_sBin [7:0]  AFIT8_Sharpening_iLowSharpPower_sBin [15:8]
{0x0F12, 0x0F00}, //0F00//70000994           //AFIT8_Sharpening_iHighSharpPower_sBin [7:0]  AFIT8_Sharpening_iLowShDenoise_sBin [15:8]
{0x0F12, 0x400F}, //400F//70000996           //AFIT8_Sharpening_iHighShDenoise_sBin [7:0]  AFIT8_Sharpening_iReduceEdgeMinMult_sBin [15:8]
{0x0F12, 0x0204}, //0204//70000998           //AFIT8_Sharpening_iReduceEdgeSlope_sBin [7:0]  AFIT8_demsharpmix1_iWideFiltReduce_sBin [15:8]
{0x0F12, 0x0003}, //0003//7000099A           //AFIT8_demsharpmix1_iNarrFiltReduce_sBin [7:0]
{0x0F12, 0x0001}, //0001//7000099C        
{0x0F12, 0x0000}, //0000//7000099E//AFIT16_BRIGHTNESS
{0x0F12, 0x0000}, //0000//700009A0           //AFIT16_CONTRAST
{0x0F12, 0x0000}, //0000//700009A2           //AFIT16_SATURATION
{0x0F12, 0x0000}, //0000//700009A4           //AFIT16_SHARP_BLUR
{0x0F12, 0x0000}, //0000//700009A6           //AFIT16_GLAMOUR
{0x0F12, 0x00C1}, //00C1//700009A8           //AFIT16_sddd8a_edge_high
{0x0F12, 0x0000}, //0000//700009AA        
{0x0F12, 0x03FF}, //03FF//700009AC           //AFIT16_Demosaicing_iSatVal
{0x0F12, 0x009C}, //009C//700009AE           //AFIT16_Sharpening_iReduceEdgeThresh
{0x0F12, 0x017C}, //017C//700009B0           //AFIT16_demsharpmix1_iRGBOffset
{0x0F12, 0x03FF}, //03FF//700009B2           //AFIT16_demsharpmix1_iDemClamp
{0x0F12, 0x000C}, //000C//700009B4           //AFIT16_demsharpmix1_iLowThreshold
{0x0F12, 0x0010}, //0010//700009B6           //AFIT16_demsharpmix1_iHighThreshold
{0x0F12, 0x012C}, //012C//700009B8           //AFIT16_demsharpmix1_iLowBright
{0x0F12, 0x03E8}, //03E8//700009BA           //AFIT16_demsharpmix1_iHighBright
{0x0F12, 0x0046}, //0046//700009BC           //AFIT16_demsharpmix1_iLowSat
{0x0F12, 0x005A}, //005A//700009BE           //AFIT16_demsharpmix1_iHighSat
{0x0F12, 0x0070}, //0070//700009C0           //AFIT16_demsharpmix1_iTune
{0x0F12, 0x0001}, //0001//700009C2//AFIT16_demsharpmix1_iHystThLow
{0x0F12, 0x0000}, //0000//700009C4//AFIT16_demsharpmix1_iHystThHigh
{0x0F12, 0x0320}, //0320//700009C6//AFIT16_demsharpmix1_iHystCenter
{0x0F12, 0x006E}, //006E//700009C8           //AFIT16_Sharpening_iLowSharpClamp
{0x0F12, 0x0014}, //0014//700009CA           //AFIT16_Sharpening_iHighSharpClamp
{0x0F12, 0x003C}, //003C//700009CC           //AFIT16_Sharpening_iLowSharpClamp_Bin
{0x0F12, 0x001E}, //001E//700009CE           //AFIT16_Sharpening_iHighSharpClamp_Bin
{0x0F12, 0x003C}, //003C//700009D0           //AFIT16_Sharpening_iLowSharpClamp_sBin
{0x0F12, 0x001E}, //001E//700009D2           //AFIT16_Sharpening_iHighSharpClamp_sBin
{0x0F12, 0x0A24}, //0A24//700009D4           //AFIT8_sddd8a_edge_low [7:0] AFIT8_sddd8a_repl_thresh [15:8]
{0x0F12, 0x1701}, //1701//700009D6           //AFIT8_sddd8a_repl_force [7:0] AFIT8_sddd8a_sat_level [15:8]
{0x0F12, 0x0229}, //0229//700009D8           //AFIT8_sddd8a_sat_thr[7:0] AFIT8_sddd8a_sat_mpl [15:8]
{0x0F12, 0x1403}, //1403//700009DA           //AFIT8_sddd8a_sat_noise[7:0] AFIT8_sddd8a_iMaxSlopeAllowed [15:8]
{0x0F12, 0x0004}, //0004//700009DC           //AFIT8_sddd8a_iHotThreshHigh[7:0] AFIT8_sddd8a_iHotThreshLow [15:8]
{0x0F12, 0x0300}, //0300//700009DE           //AFIT8_sddd8a_iColdThreshHigh[7:0] AFIT8_sddd8a_iColdThreshLow [15:8]
{0x0F12, 0x0000}, //0000//700009E0           //AFIT8_sddd8a_AddNoisePower1[7:0] AFIT8_sddd8a_AddNoisePower2 [15:8]
{0x0F12, 0x02FF}, //02FF//700009E2           //AFIT8_sddd8a_iSatSat[7:0] AFIT8_sddd8a_iRadialTune [15:8]
{0x0F12, 0x052D}, //05E8//700009E4           //AFIT8_sddd8a_iRadialLimit [7:0] AFIT8_sddd8a_iRadialPower [15:8]
{0x0F12, 0x1414}, //1414//700009E6           //AFIT8_sddd8a_iLowMaxSlopeAllowed [7:0] AFIT8_sddd8a_iHighMaxSlopeAllowed [15:8]
{0x0F12, 0x0301}, //0301//700009E8           //AFIT8_sddd8a_iLowSlopeThresh[7:0] AFIT8_sddd8a_iHighSlopeThresh [15:8]
{0x0F12, 0x0007}, //0007//700009EA           //AFIT8_sddd8a_iSquaresRounding [7:0]
{0x0F12, 0x1000}, //2000//700009EC        
{0x0F12, 0x2003}, //5003//700009EE        
{0x0F12, 0x101A}, //3228//700009F0        
{0x0F12, 0x0010}, //0032//700009F2        
{0x0F12, 0x1E80}, //1E80//700009F4           //AFIT8_Demosaicing_iCentGrad[7:0] AFIT8_Demosaicing_iMonochrom[15:8]
{0x0F12, 0x1E08}, //1E08//700009F6           //AFIT8_Demosaicing_iDecisionThresh[7:0] AFIT8_Demosaicing_iDesatThresh[15:8]
{0x0F12, 0x000A}, //000A//700009F8           //AFIT8_Demosaicing_iEnhThresh[7:0] AFIT8_Demosaicing_iGRDenoiseVal[15:8]
{0x0F12, 0x0000}, //0000//700009FA           //AFIT8_Demosaicing_iGBDenoiseVal[7:0] AFIT8_Demosaicing_iNearGrayDesat[15:8]
{0x0F12, 0x120A}, //120A//700009FC           //AFIT8_Demosaicing_iDFD_ReduceCoeff[7:0] AFIT8_Sharpening_iMSharpen[15:8]
{0x0F12, 0x1400}, //1400//700009FE           //AFIT8_Sharpening_iMShThresh[7:0] AFIT8_Sharpening_iWSharpen[15:8]
{0x0F12, 0x0200}, //0200//70000A00           //AFIT8_Sharpening_iWShThresh[7:0] AFIT8_Sharpening_nSharpWidth[15:8]
{0x0F12, 0xFF00}, //FF00//70000A02           //AFIT8_Sharpening_iReduceNegative[7:0] AFIT8_Sharpening_iShDespeckle[15:8]
{0x0F12, 0x0200}, //0200//70000A04           //AFIT8_demsharpmix1_iRGBMultiplier[7:0] AFIT8_demsharpmix1_iFilterPower[15:8]
{0x0F12, 0x1B11}, //1B11//70000A06           //AFIT8_demsharpmix1_iBCoeff[7:0] AFIT8_demsharpmix1_iGCoeff[15:8]
{0x0F12, 0x0000}, //0000//70000A08           //AFIT8_demsharpmix1_iWideMult[7:0] AFIT8_demsharpmix1_iNarrMult[15:8]
{0x0F12, 0x0009}, //0009//70000A0A           //AFIT8_demsharpmix1_iHystFalloff[7:0] AFIT8_demsharpmix1_iHystMinMult[15:8]
{0x0F12, 0x0406}, //0406//70000A0C           //AFIT8_demsharpmix1_iHystWidth[7:0] AFIT8_demsharpmix1_iHystFallLow[15:8]
{0x0F12, 0x0605}, //0605//70000A0E           //AFIT8_demsharpmix1_iHystFallHigh[7:0] AFIT8_demsharpmix1_iHystTune[15:8]
{0x0F12, 0x0307}, //0307//70000A10        
{0x0F12, 0x0609}, //0609//70000A12        
{0x0F12, 0x1C07}, //2C07//70000A14        
{0x0F12, 0x1414}, //142C//70000A16        
{0x0F12, 0x0510}, //0518//70000A18//[15:8]iUVNRStrengthL [7:0]iMaxThreshH        
{0x0F12, 0x8005}, //8005//70000A1A//[7:0]iUVNRStrengthH AFIT8_byr_cgras_iShadingPower[15:8]
{0x0F12, 0x0080}, //0580//70000A1C           //AFIT8_RGBGamma2_iLinearity [7:0]  AFIT8_RGBGamma2_iDarkReduce [15:8]
{0x0F12, 0x0080}, //0080//70000A1E           //AFIT8_ccm_oscar_iSaturation[7:0]   AFIT8_RGB2YUV_iYOffset [15:8]
{0x0F12, 0x0080}, //0080//70000A20           //AFIT8_RGB2YUV_iRGBGain [7:0]   AFIT8_RGB2YUV_iSaturation [15:8]
{0x0F12, 0x0101}, //0101//70000A22           //AFIT8_sddd8a_iClustThresh_H [7:0]  AFIT8_sddd8a_iClustThresh_C [15:8]
{0x0F12, 0x0707}, //0707//70000A24           //AFIT8_sddd8a_iClustMulT_H [7:0]   AFIT8_sddd8a_iClustMulT_C [15:8]
{0x0F12, 0x4B01}, //4B01//70000A26           //AFIT8_sddd8a_nClustLevel_H [7:0]   AFIT8_sddd8a_DispTH_Low [15:8]
{0x0F12, 0x184B}, //444B 494B//70000A28           //AFIT8_sddd8a_DispTH_High [7:0]   AFIT8_sddd8a_iDenThreshLow [15:8]
{0x0F12, 0x500C}, //503C 5044//70000A2A           //AFIT8_sddd8a_iDenThreshHigh[7:0]   AFIT8_Demosaicing_iEdgeDesat [15:8]
{0x0F12, 0x0500}, //0500//70000A2C           //AFIT8_Demosaicing_iEdgeDesatThrLow [7:0]   AFIT8_Demosaicing_iEdgeDesatThrHigh [15:8]
{0x0F12, 0x1C03}, //0503//70000A2E           //AFIT8_Demosaicing_iEdgeDesatLimit[7:0]  AFIT8_Demosaicing_iDemSharpenLow [15:8]
{0x0F12, 0x0A08}, //0D02//70000A30           //AFIT8_Demosaicing_iDemSharpenHigh[7:0]   AFIT8_Demosaicing_iDemSharpThresh [15:8]
{0x0F12, 0x070C}, //071E//70000A32           //AFIT8_Demosaicing_iDemShLowLimit [7:0]   AFIT8_Demosaicing_iDespeckleForDemsharp [15:8]
{0x0F12, 0x1428}, //1432//70000A34           //AFIT8_Demosaicing_iDemBlurLow[7:0]   AFIT8_Demosaicing_iDemBlurHigh [15:8]
{0x0F12, 0x6401}, //5A01//70000A36           //AFIT8_Demosaicing_iDemBlurRange[7:0]   AFIT8_Sharpening_iLowSharpPower [15:8]                         
{0x0F12, 0x281E}, //281E//70000A38           //AFIT8_Sharpening_iHighSharpPower[7:0]   AFIT8_Sharpening_iLowShDenoise [15:8]
{0x0F12, 0x200F}, //200F//70000A3A           //AFIT8_Sharpening_iHighShDenoise [7:0]   AFIT8_Sharpening_iReduceEdgeMinMult [15:8]
{0x0F12, 0x0204}, //0204//70000A3C           //AFIT8_Sharpening_iReduceEdgeSlope [7:0]  AFIT8_demsharpmix1_iWideFiltReduce [15:8]
{0x0F12, 0x4603}, //1E03//70000A3E           //AFIT8_demsharpmix1_iNarrFiltReduce [7:0]  AFIT8_sddd8a_iClustThresh_H_Bin [15:8]
{0x0F12, 0x0146}, //011E//70000A40           //AFIT8_sddd8a_iClustThresh_C_Bin [7:0]   AFIT8_sddd8a_iClustMulT_H_Bin [15:8]
{0x0F12, 0x0101}, //0101//70000A42           //AFIT8_sddd8a_iClustMulT_C_Bin [7:0]   AFIT8_sddd8a_nClustLevel_H_Bin [15:8]
{0x0F12, 0x1C46}, //3A3C//70000A44           //AFIT8_sddd8a_DispTH_Low_Bin [7:0]   AFIT8_sddd8a_DispTH_High_Bin [15:8]
{0x0F12, 0x1819}, //585A//70000A46           //AFIT8_sddd8a_iDenThreshLow_Bin [7:0]   AFIT8_sddd8a_iDenThreshHigh_Bin [15:8]
{0x0F12, 0x0028}, //0028//70000A48           //AFIT8_Demosaicing_iEdgeDesat_Bin[7:0]   AFIT8_Demosaicing_iEdgeDesatThrLow_Bin [15:8]
{0x0F12, 0x030A}, //030A//70000A4A           //AFIT8_Demosaicing_iEdgeDesatThrHigh_Bin [7:0]  AFIT8_Demosaicing_iEdgeDesatLimit_Bin [15:8]
{0x0F12, 0x0000}, //0000//70000A4C           //AFIT8_Demosaicing_iDemSharpenLow_Bin [7:0]  AFIT8_Demosaicing_iDemSharpenHigh_Bin [15:8]
{0x0F12, 0x141E}, //141E//70000A4E           //AFIT8_Demosaicing_iDemSharpThresh_Bin [7:0]  AFIT8_Demosaicing_iDemShLowLimit_Bin [15:8]
{0x0F12, 0xFF07}, //FF07//70000A50           //AFIT8_Demosaicing_iDespeckleForDemsharp_Bin [7:0]  AFIT8_Demosaicing_iDemBlurLow_Bin [15:8]
{0x0F12, 0x0432}, //0432//70000A52           //AFIT8_Demosaicing_iDemBlurHigh_Bin [7:0]  AFIT8_Demosaicing_iDemBlurRange_Bin [15:8]
{0x0F12, 0x0000}, //0000//70000A54           //AFIT8_Sharpening_iLowSharpPower_Bin [7:0]  AFIT8_Sharpening_iHighSharpPower_Bin [15:8]
{0x0F12, 0x0F0F}, //0F0F//70000A56           //AFIT8_Sharpening_iLowShDenoise_Bin [7:0]  AFIT8_Sharpening_iHighShDenoise_Bin [15:8]
{0x0F12, 0x0440}, //0440//70000A58           //AFIT8_Sharpening_iReduceEdgeMinMult_Bin [7:0]  AFIT8_Sharpening_iReduceEdgeSlope_Bin [15:8]
{0x0F12, 0x0302}, //0302//70000A5A           //AFIT8_demsharpmix1_iWideFiltReduce_Bin [7:0]  AFIT8_demsharpmix1_iNarrFiltReduce_Bin [15:8]
{0x0F12, 0x1414}, //1E1E//70000A5C           //AFIT8_sddd8a_iClustThresh_H_sBin[7:0]   AFIT8_sddd8a_iClustThresh_C_sBin [15:8]
{0x0F12, 0x0101}, //0101//70000A5E           //AFIT8_sddd8a_iClustMulT_H_sBin [7:0]   AFIT8_sddd8a_iClustMulT_C_sBin [15:8]
{0x0F12, 0x1801}, //3C01//70000A60           //AFIT8_sddd8a_nClustLevel_H_sBin [7:0]   AFIT8_sddd8a_DispTH_Low_sBin [15:8]
{0x0F12, 0x191C}, //5A3A//70000A62           //AFIT8_sddd8a_DispTH_High_sBin [7:0]   AFIT8_sddd8a_iDenThreshLow_sBin [15:8]
{0x0F12, 0x2818}, //2858//70000A64           //AFIT8_sddd8a_iDenThreshHigh_sBin[7:0]   AFIT8_Demosaicing_iEdgeDesat_sBin [15:8]
{0x0F12, 0x0A00}, //0A00//70000A66           //AFIT8_Demosaicing_iEdgeDesatThrLow_sBin [7:0]  AFIT8_Demosaicing_iEdgeDesatThrHigh_sBin [15:8]
{0x0F12, 0x1403}, //0003//70000A68           //AFIT8_Demosaicing_iEdgeDesatLimit_sBin [7:0]  AFIT8_Demosaicing_iDemSharpenLow_sBin [15:8]
{0x0F12, 0x1400}, //1E00//70000A6A           //AFIT8_Demosaicing_iDemSharpenHigh_sBin [7:0]  AFIT8_Demosaicing_iDemSharpThresh_sBin [15:8]
{0x0F12, 0x0714}, //0714//70000A6C           //AFIT8_Demosaicing_iDemShLowLimit_sBin [7:0]  AFIT8_Demosaicing_iDespeckleForDemsharp_sBin [15:8]
{0x0F12, 0x32FF}, //32FF//70000A6E           //AFIT8_Demosaicing_iDemBlurLow_sBin [7:0]  AFIT8_Demosaicing_iDemBlurHigh_sBin [15:8]
{0x0F12, 0x0004}, //0004//70000A70           //AFIT8_Demosaicing_iDemBlurRange_sBin [7:0]  AFIT8_Sharpening_iLowSharpPower_sBin [15:8]
{0x0F12, 0x0F00}, //0F00//70000A72           //AFIT8_Sharpening_iHighSharpPower_sBin [7:0]  AFIT8_Sharpening_iLowShDenoise_sBin [15:8]
{0x0F12, 0x400F}, //400F//70000A74           //AFIT8_Sharpening_iHighShDenoise_sBin [7:0]  AFIT8_Sharpening_iReduceEdgeMinMult_sBin [15:8]
{0x0F12, 0x0204}, //0204//70000A76           //AFIT8_Sharpening_iReduceEdgeSlope_sBin [7:0]  AFIT8_demsharpmix1_iWideFiltReduce_sBin [15:8]
{0x0F12, 0x0003}, //0003//70000A78           //AFIT8_demsharpmix1_iNarrFiltReduce_sBin [7:0]
{0x0F12, 0x0001}, //0001//70000A7A        
{0x0F12, 0x0000}, //0000//70000A7C//AFIT16_BRIGHTNESS
{0x0F12, 0x0000}, //0000//70000A7E           //AFIT16_CONTRAST
{0x0F12, 0x0000}, //0000//70000A80           //AFIT16_SATURATION
{0x0F12, 0x0000}, //0000//70000A82           //AFIT16_SHARP_BLUR
{0x0F12, 0x0000}, //0000//70000A84           //AFIT16_GLAMOUR
{0x0F12, 0x00C1}, //00C1//70000A86           //AFIT16_sddd8a_edge_high
{0x0F12, 0x0000}, //0000//70000A88        
{0x0F12, 0x03FF}, //03FF//70000A8A           //AFIT16_Demosaicing_iSatVal
{0x0F12, 0x009C}, //009E//70000A8C           //AFIT16_Sharpening_iReduceEdgeThresh
{0x0F12, 0x017C}, //017C//70000A8E           //AFIT16_demsharpmix1_iRGBOffset
{0x0F12, 0x03FF}, //03FF//70000A90           //AFIT16_demsharpmix1_iDemClamp
{0x0F12, 0x000C}, //000C//70000A92           //AFIT16_demsharpmix1_iLowThreshold
{0x0F12, 0x0010}, //0010//70000A94           //AFIT16_demsharpmix1_iHighThreshold
{0x0F12, 0x012C}, //012C//70000A96           //AFIT16_demsharpmix1_iLowBright
{0x0F12, 0x03E8}, //03E8//70000A98           //AFIT16_demsharpmix1_iHighBright
{0x0F12, 0x0046}, //0046//70000A9A           //AFIT16_demsharpmix1_iLowSat
{0x0F12, 0x005A}, //005A//70000A9C           //AFIT16_demsharpmix1_iHighSat
{0x0F12, 0x0070}, //0070//70000A9E           //AFIT16_demsharpmix1_iTune
{0x0F12, 0x0001}, //0001//70000AA0//AFIT16_demsharpmix1_iHystThLow
{0x0F12, 0x0000}, //0000//70000AA2//AFIT16_demsharpmix1_iHystThHigh
{0x0F12, 0x0320}, //0320//70000AA4//AFIT16_demsharpmix1_iHystCenter
{0x0F12, 0x006E}, //008C//70000AA6           //AFIT16_Sharpening_iLowSharpClamp
{0x0F12, 0x0014}, //0014//70000AA8           //AFIT16_Sharpening_iHighSharpClamp
{0x0F12, 0x003C}, //003C//70000AAA           //AFIT16_Sharpening_iLowSharpClamp_Bin
{0x0F12, 0x001E}, //001E//70000AAC           //AFIT16_Sharpening_iHighSharpClamp_Bin
{0x0F12, 0x003C}, //003C//70000AAE           //AFIT16_Sharpening_iLowSharpClamp_sBin
{0x0F12, 0x001E}, //001E//70000AB0           //AFIT16_Sharpening_iHighSharpClamp_sBin
{0x0F12, 0x0A24}, //0A24//70000AB2           //AFIT8_sddd8a_edge_low [7:0] AFIT8_sddd8a_repl_thresh [15:8]
{0x0F12, 0x1701}, //1701//70000AB4           //AFIT8_sddd8a_repl_force [7:0] AFIT8_sddd8a_sat_level [15:8]
{0x0F12, 0x0229}, //0229//70000AB6           //AFIT8_sddd8a_sat_thr[7:0] AFIT8_sddd8a_sat_mpl [15:8]
{0x0F12, 0x1403}, //1403//70000AB8           //AFIT8_sddd8a_sat_noise[7:0] AFIT8_sddd8a_iMaxSlopeAllowed [15:8]
{0x0F12, 0x0004}, //0004//70000ABA           //AFIT8_sddd8a_iHotThreshHigh[7:0] AFIT8_sddd8a_iHotThreshLow [15:8]
{0x0F12, 0x0300}, //0300//70000ABC           //AFIT8_sddd8a_iColdThreshHigh[7:0] AFIT8_sddd8a_iColdThreshLow [15:8]
{0x0F12, 0x0000}, //0000//70000ABE           //AFIT8_sddd8a_AddNoisePower1[7:0] AFIT8_sddd8a_AddNoisePower2 [15:8]
{0x0F12, 0x02FF}, //02FF//70000AC0           //AFIT8_sddd8a_iSatSat[7:0] AFIT8_sddd8a_iRadialTune [15:8]
{0x0F12, 0x052D}, //05DE//70000AC2           //AFIT8_sddd8a_iRadialLimit [7:0] AFIT8_sddd8a_iRadialPower [15:8]
{0x0F12, 0x1414}, //1414//70000AC4           //AFIT8_sddd8a_iLowMaxSlopeAllowed [7:0] AFIT8_sddd8a_iHighMaxSlopeAllowed [15:8]
{0x0F12, 0x0301}, //0301//70000AC6           //AFIT8_sddd8a_iLowSlopeThresh[7:0] AFIT8_sddd8a_iHighSlopeThresh [15:8]
{0x0F12, 0x0007}, //0007//70000AC8           //AFIT8_sddd8a_iSquaresRounding [7:0]
{0x0F12, 0x1000}, //1000//70000ACA        
{0x0F12, 0x2003}, //2803//70000ACC        
{0x0F12, 0x101A}, //261E//70000ACE        
{0x0F12, 0x0010}, //0026//70000AD0        
{0x0F12, 0x1E80}, //1E80//70000AD2           //AFIT8_Demosaicing_iCentGrad[7:0] AFIT8_Demosaicing_iMonochrom[15:8]
{0x0F12, 0x1E08}, //1E08//70000AD4           //AFIT8_Demosaicing_iDecisionThresh[7:0] AFIT8_Demosaicing_iDesatThresh[15:8]
{0x0F12, 0x000A}, //010A//70000AD6           //AFIT8_Demosaicing_iEnhThresh[7:0] AFIT8_Demosaicing_iGRDenoiseVal[15:8]
{0x0F12, 0x0000}, //0001//70000AD8           //AFIT8_Demosaicing_iGBDenoiseVal[7:0] AFIT8_Demosaicing_iNearGrayDesat[15:8]
{0x0F12, 0x120A}, //3C0A//70000ADA           //AFIT8_Demosaicing_iDFD_ReduceCoeff[7:0] AFIT8_Sharpening_iMSharpen[15:8]
{0x0F12, 0x1400}, //2300//70000ADC           //AFIT8_Sharpening_iMShThresh[7:0] AFIT8_Sharpening_iWSharpen[15:8]
{0x0F12, 0x0200}, //0200//70000ADE           //AFIT8_Sharpening_iWShThresh[7:0] AFIT8_Sharpening_nSharpWidth[15:8]
{0x0F12, 0xFF00}, //FF00//70000AE0           //AFIT8_Sharpening_iReduceNegative[7:0] AFIT8_Sharpening_iShDespeckle[15:8]
{0x0F12, 0x0200}, //0200//70000AE2           //AFIT8_demsharpmix1_iRGBMultiplier[7:0] AFIT8_demsharpmix1_iFilterPower[15:8]
{0x0F12, 0x1B11}, //1B11//70000AE4           //AFIT8_demsharpmix1_iBCoeff[7:0] AFIT8_demsharpmix1_iGCoeff[15:8]
{0x0F12, 0x0000}, //0000//70000AE6           //AFIT8_demsharpmix1_iWideMult[7:0] AFIT8_demsharpmix1_iNarrMult[15:8]
{0x0F12, 0x0009}, //0009//70000AE8           //AFIT8_demsharpmix1_iHystFalloff[7:0] AFIT8_demsharpmix1_iHystMinMult[15:8]
{0x0F12, 0x0406}, //0406//70000AEA           //AFIT8_demsharpmix1_iHystWidth[7:0] AFIT8_demsharpmix1_iHystFallLow[15:8]
{0x0F12, 0x0605}, //0605//70000AEC           //AFIT8_demsharpmix1_iHystFallHigh[7:0] AFIT8_demsharpmix1_iHystTune[15:8]
{0x0F12, 0x0307}, //0307//70000AEE        
{0x0F12, 0x0609}, //0609//70000AF0        
{0x0F12, 0x1C07}, //1C07//70000AF2        
{0x0F12, 0x1414}, //1014//70000AF4        
{0x0F12, 0x0510}, //0510//70000AF6//[15:8]iUVNRStrengthL [7:0]iMaxThreshH        
{0x0F12, 0x8005}, //8005//70000AF8//[7:0]iUVNRStrengthH AFIT8_byr_cgras_iShadingPower[15:8]
{0x0F12, 0x0080}, //0080//70000AFA           //AFIT8_RGBGamma2_iLinearity [7:0]  AFIT8_RGBGamma2_iDarkReduce [15:8]
{0x0F12, 0x0080}, //0080//70000AFC           //AFIT8_ccm_oscar_iSaturation[7:0]   AFIT8_RGB2YUV_iYOffset [15:8]
{0x0F12, 0x0080}, //0080//70000AFE           //AFIT8_RGB2YUV_iRGBGain [7:0]   AFIT8_RGB2YUV_iSaturation [15:8]
{0x0F12, 0x0101}, //0101//70000B00           //AFIT8_sddd8a_iClustThresh_H [7:0]  AFIT8_sddd8a_iClustThresh_C [15:8]
{0x0F12, 0x0707}, //0707//70000B02           //AFIT8_sddd8a_iClustMulT_H [7:0]   AFIT8_sddd8a_iClustMulT_C [15:8]
{0x0F12, 0x4B01}, //4B01//70000B04           //AFIT8_sddd8a_nClustLevel_H [7:0]   AFIT8_sddd8a_DispTH_Low [15:8]
{0x0F12, 0x184B}, //2A4B//70000B06           //AFIT8_sddd8a_DispTH_High [7:0]   AFIT8_sddd8a_iDenThreshLow [15:8]
{0x0F12, 0x500C}, //5020//70000B08           //AFIT8_sddd8a_iDenThreshHigh[7:0]   AFIT8_Demosaicing_iEdgeDesat [15:8]
{0x0F12, 0x0500}, //0500//70000B0A           //AFIT8_Demosaicing_iEdgeDesatThrLow [7:0]   AFIT8_Demosaicing_iEdgeDesatThrHigh [15:8]
{0x0F12, 0x1C03}, //1C03//70000B0C           //AFIT8_Demosaicing_iEdgeDesatLimit[7:0]  AFIT8_Demosaicing_iDemSharpenLow [15:8]
{0x0F12, 0x0A08}, //0D0C//70000B0E           //AFIT8_Demosaicing_iDemSharpenHigh[7:0]   AFIT8_Demosaicing_iDemSharpThresh [15:8]
{0x0F12, 0x070C}, //0823//70000B10           //AFIT8_Demosaicing_iDemShLowLimit [7:0]   AFIT8_Demosaicing_iDespeckleForDemsharp [15:8]
{0x0F12, 0x1428}, //1428//70000B12           //AFIT8_Demosaicing_iDemBlurLow[7:0]   AFIT8_Demosaicing_iDemBlurHigh [15:8]
{0x0F12, 0x6401}, //6401//70000B14           //AFIT8_Demosaicing_iDemBlurRange[7:0]   AFIT8_Sharpening_iLowSharpPower [15:8]
{0x0F12, 0x281E}, //282D//70000B16           //AFIT8_Sharpening_iHighSharpPower[7:0]   AFIT8_Sharpening_iLowShDenoise [15:8]
{0x0F12, 0x200F}, //2012//70000B18           //AFIT8_Sharpening_iHighShDenoise [7:0]   AFIT8_Sharpening_iReduceEdgeMinMult [15:8]
{0x0F12, 0x0204}, //0204//70000B1A           //AFIT8_Sharpening_iReduceEdgeSlope [7:0]  AFIT8_demsharpmix1_iWideFiltReduce [15:8]
{0x0F12, 0x4603}, //2803//70000B1C           //AFIT8_demsharpmix1_iNarrFiltReduce [7:0]  AFIT8_sddd8a_iClustThresh_H_Bin [15:8]
{0x0F12, 0x0146}, //0128//70000B1E           //AFIT8_sddd8a_iClustThresh_C_Bin [7:0]   AFIT8_sddd8a_iClustMulT_H_Bin [15:8]
{0x0F12, 0x0101}, //0101//70000B20           //AFIT8_sddd8a_iClustMulT_C_Bin [7:0]   AFIT8_sddd8a_nClustLevel_H_Bin [15:8]
{0x0F12, 0x1C46}, //2224//70000B22           //AFIT8_sddd8a_DispTH_Low_Bin [7:0]   AFIT8_sddd8a_DispTH_High_Bin [15:8]
{0x0F12, 0x1819}, //3236//70000B24           //AFIT8_sddd8a_iDenThreshLow_Bin [7:0]   AFIT8_sddd8a_iDenThreshHigh_Bin [15:8]
{0x0F12, 0x0028}, //0028//70000B26           //AFIT8_Demosaicing_iEdgeDesat_Bin[7:0]   AFIT8_Demosaicing_iEdgeDesatThrLow_Bin [15:8]
{0x0F12, 0x030A}, //030A//70000B28           //AFIT8_Demosaicing_iEdgeDesatThrHigh_Bin [7:0]  AFIT8_Demosaicing_iEdgeDesatLimit_Bin [15:8]
{0x0F12, 0x0000}, //0410//70000B2A           //AFIT8_Demosaicing_iDemSharpenLow_Bin [7:0]  AFIT8_Demosaicing_iDemSharpenHigh_Bin [15:8]
{0x0F12, 0x141E}, //141E//70000B2C           //AFIT8_Demosaicing_iDemSharpThresh_Bin [7:0]  AFIT8_Demosaicing_iDemShLowLimit_Bin [15:8]
{0x0F12, 0xFF07}, //FF07//70000B2E           //AFIT8_Demosaicing_iDespeckleForDemsharp_Bin [7:0]  AFIT8_Demosaicing_iDemBlurLow_Bin [15:8]
{0x0F12, 0x0432}, //0432//70000B30           //AFIT8_Demosaicing_iDemBlurHigh_Bin [7:0]  AFIT8_Demosaicing_iDemBlurRange_Bin [15:8]
{0x0F12, 0x0000}, //4050//70000B32           //AFIT8_Sharpening_iLowSharpPower_Bin [7:0]  AFIT8_Sharpening_iHighSharpPower_Bin [15:8]
{0x0F12, 0x0F0F}, //0F0F//70000B34           //AFIT8_Sharpening_iLowShDenoise_Bin [7:0]  AFIT8_Sharpening_iHighShDenoise_Bin [15:8]
{0x0F12, 0x0440}, //0440//70000B36           //AFIT8_Sharpening_iReduceEdgeMinMult_Bin [7:0]  AFIT8_Sharpening_iReduceEdgeSlope_Bin [15:8]
{0x0F12, 0x0302}, //0302//70000B38           //AFIT8_demsharpmix1_iWideFiltReduce_Bin [7:0]  AFIT8_demsharpmix1_iNarrFiltReduce_Bin [15:8]
{0x0F12, 0x1414}, //2828//70000B3A           //AFIT8_sddd8a_iClustThresh_H_sBin[7:0]   AFIT8_sddd8a_iClustThresh_C_sBin [15:8]
{0x0F12, 0x0101}, //0101//70000B3C           //AFIT8_sddd8a_iClustMulT_H_sBin [7:0]   AFIT8_sddd8a_iClustMulT_C_sBin [15:8]
{0x0F12, 0x1801}, //2401//70000B3E           //AFIT8_sddd8a_nClustLevel_H_sBin [7:0]   AFIT8_sddd8a_DispTH_Low_sBin [15:8]
{0x0F12, 0x191C}, //3622//70000B40           //AFIT8_sddd8a_DispTH_High_sBin [7:0]   AFIT8_sddd8a_iDenThreshLow_sBin [15:8]
{0x0F12, 0x2818}, //2832//70000B42           //AFIT8_sddd8a_iDenThreshHigh_sBin[7:0]   AFIT8_Demosaicing_iEdgeDesat_sBin [15:8]
{0x0F12, 0x0A00}, //0A00//70000B44           //AFIT8_Demosaicing_iEdgeDesatThrLow_sBin [7:0]  AFIT8_Demosaicing_iEdgeDesatThrHigh_sBin [15:8]
{0x0F12, 0x1403}, //1003//70000B46           //AFIT8_Demosaicing_iEdgeDesatLimit_sBin [7:0]  AFIT8_Demosaicing_iDemSharpenLow_sBin [15:8]
{0x0F12, 0x1400}, //1E04//70000B48           //AFIT8_Demosaicing_iDemSharpenHigh_sBin [7:0]  AFIT8_Demosaicing_iDemSharpThresh_sBin [15:8]
{0x0F12, 0x0714}, //0714//70000B4A           //AFIT8_Demosaicing_iDemShLowLimit_sBin [7:0]  AFIT8_Demosaicing_iDespeckleForDemsharp_sBin [15:8]
{0x0F12, 0x32FF}, //32FF//70000B4C           //AFIT8_Demosaicing_iDemBlurLow_sBin [7:0]  AFIT8_Demosaicing_iDemBlurHigh_sBin [15:8]
{0x0F12, 0x0004}, //5004//70000B4E           //AFIT8_Demosaicing_iDemBlurRange_sBin [7:0]  AFIT8_Sharpening_iLowSharpPower_sBin [15:8]
{0x0F12, 0x0F00}, //0F40//70000B50           //AFIT8_Sharpening_iHighSharpPower_sBin [7:0]  AFIT8_Sharpening_iLowShDenoise_sBin [15:8]
{0x0F12, 0x400F}, //400F//70000B52           //AFIT8_Sharpening_iHighShDenoise_sBin [7:0]  AFIT8_Sharpening_iReduceEdgeMinMult_sBin [15:8]
{0x0F12, 0x0204}, //0204//70000B54           //AFIT8_Sharpening_iReduceEdgeSlope_sBin [7:0]  AFIT8_demsharpmix1_iWideFiltReduce_sBin [15:8]
{0x0F12, 0x0003}, //0003//70000B56           //AFIT8_demsharpmix1_iNarrFiltReduce_sBin [7:0]
{0x0F12, 0x0001}, //0001//70000B58        
{0x0F12, 0x0000}, //0000//70000B5A//AFIT16_BRIGHTNESS
{0x0F12, 0x0000}, //0000//70000B5C           //AFIT16_CONTRAST
{0x0F12, 0x0000}, //0000//70000B5E           //AFIT16_SATURATION
{0x0F12, 0x0000}, //0000//70000B60           //AFIT16_SHARP_BLUR
{0x0F12, 0x0000}, //0000//70000B62           //AFIT16_GLAMOUR
{0x0F12, 0x00C1}, //00C1//70000B64           //AFIT16_sddd8a_edge_high
{0x0F12, 0x0000}, //0000//70000B66        
{0x0F12, 0x03FF}, //03FF//70000B68           //AFIT16_Demosaicing_iSatVal
{0x0F12, 0x009C}, //009E//70000B6A           //AFIT16_Sharpening_iReduceEdgeThresh
{0x0F12, 0x017C}, //017C//70000B6C           //AFIT16_demsharpmix1_iRGBOffset
{0x0F12, 0x03FF}, //03FF//70000B6E           //AFIT16_demsharpmix1_iDemClamp
{0x0F12, 0x000C}, //000C//70000B70           //AFIT16_demsharpmix1_iLowThreshold
{0x0F12, 0x0010}, //0010//70000B72           //AFIT16_demsharpmix1_iHighThreshold
{0x0F12, 0x012C}, //00C8//70000B74           //AFIT16_demsharpmix1_iLowBright
{0x0F12, 0x03E8}, //03E8//70000B76           //AFIT16_demsharpmix1_iHighBright
{0x0F12, 0x0046}, //0046//70000B78           //AFIT16_demsharpmix1_iLowSat
{0x0F12, 0x005A}, //0050//70000B7A           //AFIT16_demsharpmix1_iHighSat
{0x0F12, 0x0070}, //0070//70000B7C           //AFIT16_demsharpmix1_iTune
{0x0F12, 0x0001}, //0001//70000B7E//AFIT16_demsharpmix1_iHystThLow
{0x0F12, 0x0000}, //0000//70000B80//AFIT16_demsharpmix1_iHystThHigh
{0x0F12, 0x0320}, //0320//70000B82//AFIT16_demsharpmix1_iHystCenter
{0x0F12, 0x006E}, //008C//70000B84           //AFIT16_Sharpening_iLowSharpClamp
{0x0F12, 0x0014}, //0014//70000B86           //AFIT16_Sharpening_iHighSharpClamp
{0x0F12, 0x003C}, //002D//70000B88           //AFIT16_Sharpening_iLowSharpClamp_Bin
{0x0F12, 0x001E}, //0019//70000B8A           //AFIT16_Sharpening_iHighSharpClamp_Bin
{0x0F12, 0x003C}, //002D//70000B8C           //AFIT16_Sharpening_iLowSharpClamp_sBin
{0x0F12, 0x001E}, //0019//70000B8E           //AFIT16_Sharpening_iHighSharpClamp_sBin
{0x0F12, 0x0A24}, //0A24//70000B90           //AFIT8_sddd8a_edge_low [7:0] AFIT8_sddd8a_repl_thresh [15:8]
{0x0F12, 0x1701}, //1701//70000B92           //AFIT8_sddd8a_repl_force [7:0] AFIT8_sddd8a_sat_level [15:8]
{0x0F12, 0x0229}, //0229//70000B94           //AFIT8_sddd8a_sat_thr[7:0] AFIT8_sddd8a_sat_mpl [15:8]
{0x0F12, 0x1403}, //1403//70000B96           //AFIT8_sddd8a_sat_noise[7:0] AFIT8_sddd8a_iMaxSlopeAllowed [15:8]
{0x0F12, 0x0004}, //0004//70000B98           //AFIT8_sddd8a_iHotThreshHigh[7:0] AFIT8_sddd8a_iHotThreshLow [15:8]
{0x0F12, 0x0300}, //0300//70000B9A           //AFIT8_sddd8a_iColdThreshHigh[7:0] AFIT8_sddd8a_iColdThreshLow [15:8]
{0x0F12, 0x0000}, //0000//70000B9C           //AFIT8_sddd8a_AddNoisePower1[7:0] AFIT8_sddd8a_AddNoisePower2 [15:8]
{0x0F12, 0x02FF}, //02FF//70000B9E           //AFIT8_sddd8a_iSatSat[7:0] AFIT8_sddd8a_iRadialTune [15:8]
{0x0F12, 0x052D}, //05DE//70000BA0           //AFIT8_sddd8a_iRadialLimit [7:0] AFIT8_sddd8a_iRadialPower [15:8]
{0x0F12, 0x1414}, //1414//70000BA2           //AFIT8_sddd8a_iLowMaxSlopeAllowed [7:0] AFIT8_sddd8a_iHighMaxSlopeAllowed [15:8]
{0x0F12, 0x0301}, //0301//70000BA4           //AFIT8_sddd8a_iLowSlopeThresh[7:0] AFIT8_sddd8a_iHighSlopeThresh [15:8]
{0x0F12, 0x0007}, //0007//70000BA6           //AFIT8_sddd8a_iSquaresRounding [7:0]
{0x0F12, 0x1000}, //1000//70000BA8        
{0x0F12, 0x2003}, //2303//70000BAA        
{0x0F12, 0x101A}, //231A//70000BAC        
{0x0F12, 0x0010}, //0023//70000BAE        
{0x0F12, 0x1E80}, //1E80//70000BB0           //AFIT8_Demosaicing_iCentGrad[7:0] AFIT8_Demosaicing_iMonochrom[15:8]
{0x0F12, 0x1E08}, //1E08//70000BB2           //AFIT8_Demosaicing_iDecisionThresh[7:0] AFIT8_Demosaicing_iDesatThresh[15:8]
{0x0F12, 0x000A}, //010A//70000BB4           //AFIT8_Demosaicing_iEnhThresh[7:0] AFIT8_Demosaicing_iGRDenoiseVal[15:8]
{0x0F12, 0x0000}, //0001//70000BB6           //AFIT8_Demosaicing_iGBDenoiseVal[7:0] AFIT8_Demosaicing_iNearGrayDesat[15:8]
{0x0F12, 0x120A}, //3C0A//70000BB8           //AFIT8_Demosaicing_iDFD_ReduceCoeff[7:0] AFIT8_Sharpening_iMSharpen[15:8]
{0x0F12, 0x1400}, //2300//70000BBA           //AFIT8_Sharpening_iMShThresh[7:0] AFIT8_Sharpening_iWSharpen[15:8]
{0x0F12, 0x0200}, //0200//70000BBC           //AFIT8_Sharpening_iWShThresh[7:0] AFIT8_Sharpening_nSharpWidth[15:8]
{0x0F12, 0xFF00}, //FF00//70000BBE           //AFIT8_Sharpening_iReduceNegative[7:0] AFIT8_Sharpening_iShDespeckle[15:8]
{0x0F12, 0x0200}, //0200//70000BC0           //AFIT8_demsharpmix1_iRGBMultiplier[7:0] AFIT8_demsharpmix1_iFilterPower[15:8]
{0x0F12, 0x1B11}, //1E10//70000BC2           //AFIT8_demsharpmix1_iBCoeff[7:0] AFIT8_demsharpmix1_iGCoeff[15:8]
{0x0F12, 0x0000}, //0000//70000BC4           //AFIT8_demsharpmix1_iWideMult[7:0] AFIT8_demsharpmix1_iNarrMult[15:8]
{0x0F12, 0x0009}, //0009//70000BC6           //AFIT8_demsharpmix1_iHystFalloff[7:0] AFIT8_demsharpmix1_iHystMinMult[15:8]
{0x0F12, 0x0406}, //0406//70000BC8           //AFIT8_demsharpmix1_iHystWidth[7:0] AFIT8_demsharpmix1_iHystFallLow[15:8]
{0x0F12, 0x0605}, //0705//70000BCA           //AFIT8_demsharpmix1_iHystFallHigh[7:0] AFIT8_demsharpmix1_iHystTune[15:8]
{0x0F12, 0x0307}, //0306//70000BCC        
{0x0F12, 0x0609}, //0509//70000BCE        
{0x0F12, 0x1C07}, //2806//70000BD0        
{0x0F12, 0x1414}, //1428//70000BD2        
{0x0F12, 0x0510}, //0518//70000BD4//[15:8]iUVNRStrengthL [7:0]iMaxThreshH        
{0x0F12, 0x8005}, //8005//70000BD6//[7:0]iUVNRStrengthH AFIT8_byr_cgras_iShadingPower[15:8]
{0x0F12, 0x0080}, //0080//70000BD8           //AFIT8_RGBGamma2_iLinearity [7:0]  AFIT8_RGBGamma2_iDarkReduce [15:8]
{0x0F12, 0x0080}, //0080//70000BDA           //AFIT8_ccm_oscar_iSaturation[7:0]   AFIT8_RGB2YUV_iYOffset [15:8]
{0x0F12, 0x0080}, //0080//70000BDC           //AFIT8_RGB2YUV_iRGBGain [7:0]   AFIT8_RGB2YUV_iSaturation [15:8]
{0x0F12, 0x0101}, //0101//70000BDE           //AFIT8_sddd8a_iClustThresh_H [7:0]  AFIT8_sddd8a_iClustThresh_C [15:8]
{0x0F12, 0x0707}, //0707//70000BE0           //AFIT8_sddd8a_iClustMulT_H [7:0]   AFIT8_sddd8a_iClustMulT_C [15:8]
{0x0F12, 0x4B01}, //4B01//70000BE2           //AFIT8_sddd8a_nClustLevel_H [7:0]   AFIT8_sddd8a_DispTH_Low [15:8]
{0x0F12, 0x184B}, //2A4B//70000BE4           //AFIT8_sddd8a_DispTH_High [7:0]   AFIT8_sddd8a_iDenThreshLow [15:8]
{0x0F12, 0x500C}, //5020//70000BE6           //AFIT8_sddd8a_iDenThreshHigh[7:0]   AFIT8_Demosaicing_iEdgeDesat [15:8]
{0x0F12, 0x0500}, //0500//70000BE8           //AFIT8_Demosaicing_iEdgeDesatThrLow [7:0]   AFIT8_Demosaicing_iEdgeDesatThrHigh [15:8]
{0x0F12, 0x1C03}, //1C03//70000BEA           //AFIT8_Demosaicing_iEdgeDesatLimit[7:0]  AFIT8_Demosaicing_iDemSharpenLow [15:8]
{0x0F12, 0x0A08}, //0D0C//70000BEC           //AFIT8_Demosaicing_iDemSharpenHigh[7:0]   AFIT8_Demosaicing_iDemSharpThresh [15:8]
{0x0F12, 0x070C}, //0823//70000BEE           //AFIT8_Demosaicing_iDemShLowLimit [7:0]   AFIT8_Demosaicing_iDespeckleForDemsharp [15:8]
{0x0F12, 0x1428}, //1428//70000BF0           //AFIT8_Demosaicing_iDemBlurLow[7:0]   AFIT8_Demosaicing_iDemBlurHigh [15:8]
{0x0F12, 0x6401}, //6401//70000BF2           //AFIT8_Demosaicing_iDemBlurRange[7:0]   AFIT8_Sharpening_iLowSharpPower [15:8]                         
{0x0F12, 0x281E}, //282D//70000BF4           //AFIT8_Sharpening_iHighSharpPower[7:0]   AFIT8_Sharpening_iLowShDenoise [15:8]
{0x0F12, 0x200F}, //2012//70000BF6           //AFIT8_Sharpening_iHighShDenoise [7:0]   AFIT8_Sharpening_iReduceEdgeMinMult [15:8]
{0x0F12, 0x0204}, //0204//70000BF8           //AFIT8_Sharpening_iReduceEdgeSlope [7:0]  AFIT8_demsharpmix1_iWideFiltReduce [15:8]
{0x0F12, 0x4603}, //3C03//70000BFA           //AFIT8_demsharpmix1_iNarrFiltReduce [7:0]  AFIT8_sddd8a_iClustThresh_H_Bin [15:8]
{0x0F12, 0x0146}, //013C//70000BFC           //AFIT8_sddd8a_iClustThresh_C_Bin [7:0]   AFIT8_sddd8a_iClustMulT_H_Bin [15:8]
{0x0F12, 0x0101}, //0101//70000BFE           //AFIT8_sddd8a_iClustMulT_C_Bin [7:0]   AFIT8_sddd8a_nClustLevel_H_Bin [15:8]
{0x0F12, 0x1C46}, //1C1E//70000C00           //AFIT8_sddd8a_DispTH_Low_Bin [7:0]   AFIT8_sddd8a_DispTH_High_Bin [15:8]
{0x0F12, 0x1819}, //1E22//70000C02           //AFIT8_sddd8a_iDenThreshLow_Bin [7:0]   AFIT8_sddd8a_iDenThreshHigh_Bin [15:8]
{0x0F12, 0x0028}, //0028//70000C04           //AFIT8_Demosaicing_iEdgeDesat_Bin[7:0]   AFIT8_Demosaicing_iEdgeDesatThrLow_Bin [15:8]
{0x0F12, 0x030A}, //030A//70000C06           //AFIT8_Demosaicing_iEdgeDesatThrHigh_Bin [7:0]  AFIT8_Demosaicing_iEdgeDesatLimit_Bin [15:8]
{0x0F12, 0x0000}, //0214//70000C08           //AFIT8_Demosaicing_iDemSharpenLow_Bin [7:0]  AFIT8_Demosaicing_iDemSharpenHigh_Bin [15:8]
{0x0F12, 0x141E}, //0E14//70000C0A           //AFIT8_Demosaicing_iDemSharpThresh_Bin [7:0]  AFIT8_Demosaicing_iDemShLowLimit_Bin [15:8]
{0x0F12, 0xFF07}, //FF06//70000C0C           //AFIT8_Demosaicing_iDespeckleForDemsharp_Bin [7:0]  AFIT8_Demosaicing_iDemBlurLow_Bin [15:8]
{0x0F12, 0x0432}, //0432//70000C0E           //AFIT8_Demosaicing_iDemBlurHigh_Bin [7:0]  AFIT8_Demosaicing_iDemBlurRange_Bin [15:8]
{0x0F12, 0x0000}, //4052//70000C10           //AFIT8_Sharpening_iLowSharpPower_Bin [7:0]  AFIT8_Sharpening_iHighSharpPower_Bin [15:8]
{0x0F12, 0x0F0F}, //150C//70000C12           //AFIT8_Sharpening_iLowShDenoise_Bin [7:0]  AFIT8_Sharpening_iHighShDenoise_Bin [15:8]
{0x0F12, 0x0440}, //0440//70000C14           //AFIT8_Sharpening_iReduceEdgeMinMult_Bin [7:0]  AFIT8_Sharpening_iReduceEdgeSlope_Bin [15:8]
{0x0F12, 0x0302}, //0302//70000C16           //AFIT8_demsharpmix1_iWideFiltReduce_Bin [7:0]  AFIT8_demsharpmix1_iNarrFiltReduce_Bin [15:8]
{0x0F12, 0x1414}, //3C3C//70000C18           //AFIT8_sddd8a_iClustThresh_H_sBin[7:0]   AFIT8_sddd8a_iClustThresh_C_sBin [15:8]
{0x0F12, 0x0101}, //0101//70000C1A           //AFIT8_sddd8a_iClustMulT_H_sBin [7:0]   AFIT8_sddd8a_iClustMulT_C_sBin [15:8]
{0x0F12, 0x1801}, //1E01//70000C1C           //AFIT8_sddd8a_nClustLevel_H_sBin [7:0]   AFIT8_sddd8a_DispTH_Low_sBin [15:8]
{0x0F12, 0x191C}, //221C//70000C1E           //AFIT8_sddd8a_DispTH_High_sBin [7:0]   AFIT8_sddd8a_iDenThreshLow_sBin [15:8]
{0x0F12, 0x2818}, //281E//70000C20           //AFIT8_sddd8a_iDenThreshHigh_sBin[7:0]   AFIT8_Demosaicing_iEdgeDesat_sBin [15:8]
{0x0F12, 0x0A00}, //0A00//70000C22           //AFIT8_Demosaicing_iEdgeDesatThrLow_sBin [7:0]  AFIT8_Demosaicing_iEdgeDesatThrHigh_sBin [15:8]
{0x0F12, 0x1403}, //1403//70000C24           //AFIT8_Demosaicing_iEdgeDesatLimit_sBin [7:0]  AFIT8_Demosaicing_iDemSharpenLow_sBin [15:8]
{0x0F12, 0x1400}, //1402//70000C26           //AFIT8_Demosaicing_iDemSharpenHigh_sBin [7:0]  AFIT8_Demosaicing_iDemSharpThresh_sBin [15:8]
{0x0F12, 0x0714}, //060E//70000C28           //AFIT8_Demosaicing_iDemShLowLimit_sBin [7:0]  AFIT8_Demosaicing_iDespeckleForDemsharp_sBin [15:8]
{0x0F12, 0x32FF}, //32FF//70000C2A           //AFIT8_Demosaicing_iDemBlurLow_sBin [7:0]  AFIT8_Demosaicing_iDemBlurHigh_sBin [15:8]
{0x0F12, 0x0004}, //5204//70000C2C           //AFIT8_Demosaicing_iDemBlurRange_sBin [7:0]  AFIT8_Sharpening_iLowSharpPower_sBin [15:8]
{0x0F12, 0x0F00}, //0C40//70000C2E           //AFIT8_Sharpening_iHighSharpPower_sBin [7:0]  AFIT8_Sharpening_iLowShDenoise_sBin [15:8]
{0x0F12, 0x400F}, //4015//70000C30           //AFIT8_Sharpening_iHighShDenoise_sBin [7:0]  AFIT8_Sharpening_iReduceEdgeMinMult_sBin [15:8]
{0x0F12, 0x0204}, //0204//70000C32           //AFIT8_Sharpening_iReduceEdgeSlope_sBin [7:0]  AFIT8_demsharpmix1_iWideFiltReduce_sBin [15:8]
{0x0F12, 0x0003}, //0003//70000C34           //AFIT8_demsharpmix1_iNarrFiltReduce_sBin [7:0]
{0x0F12, 0x0001}, //0001//70000C36        
{0x0F12, 0x0000}, //0000//0000//70000C38//AFIT16_BRIGHTNESS
{0x0F12, 0x0000}, //0000//0000//70000C3A//AFIT16_CONTRAST
{0x0F12, 0x0000}, //0000//0000//70000C3C//AFIT16_SATURATION
{0x0F12, 0x0000}, //0000//0000//70000C3E//AFIT16_SHARP_BLUR
{0x0F12, 0x0000}, //0000//0000//70000C40//AFIT16_GLAMOUR
{0x0F12, 0x00C1}, //00C1//00C1//70000C42//AFIT16_sddd8a_edge_high
{0x0F12, 0x0000}, //0000//0000//70000C44
{0x0F12, 0x03FF}, //03FF//03FF//70000C46//AFIT16_Demosaicing_iSatVal
{0x0F12, 0x009C}, //009C//0008//70000C48//AFIT16_Sharpening_iReduceEdgeThresh
{0x0F12, 0x017C}, //0251//017C//70000C4A//AFIT16_demsharpmix1_iRGBOffset
{0x0F12, 0x03FF}, //03FF//03FF//70000C4C//AFIT16_demsharpmix1_iDemClamp
{0x0F12, 0x000C}, //000C//000C//70000C4E//AFIT16_demsharpmix1_iLowThreshold
{0x0F12, 0x0010}, //0010//0010//70000C50//AFIT16_demsharpmix1_iHighThreshold
{0x0F12, 0x012C}, //0032//0032//70000C52//AFIT16_demsharpmix1_iLowBright
{0x0F12, 0x03E8}, //028A//028A//70000C54//AFIT16_demsharpmix1_iHighBright
{0x0F12, 0x0046}, //0032//0032//70000C56//AFIT16_demsharpmix1_iLowSat
{0x0F12, 0x005A}, //01F4//01F4//70000C58//AFIT16_demsharpmix1_iHighSat
{0x0F12, 0x0070}, //0070//0070//70000C5A//AFIT16_demsharpmix1_iTune
{0x0F12, 0x0001}, //0002//0002//70000C5C//AFIT16_demsharpmix1_iHystThLow
{0x0F12, 0x0000}, //0000//0000//70000C5E//AFIT16_demsharpmix1_iHystThHigh
{0x0F12, 0x0320}, //0320//0320//70000C60//AFIT16_demsharpmix1_iHystCenter
{0x0F12, 0x006E}, //0044//0070//70000C62//AFIT16_Sharpening_iLowSharpClamp
{0x0F12, 0x0014}, //0014//0014//70000C64//AFIT16_Sharpening_iHighSharpClamp
{0x0F12, 0x003C}, //0046//0046//70000C66//AFIT16_Sharpening_iLowSharpClamp_Bin
{0x0F12, 0x001E}, //0019//0019//70000C68//AFIT16_Sharpening_iHighSharpClamp_Bin
{0x0F12, 0x003C}, //0046//0046//70000C6A//AFIT16_Sharpening_iLowSharpClamp_sBin
{0x0F12, 0x001E}, //0019//0019//70000C6C//AFIT16_Sharpening_iHighSharpClamp_sBin
{0x0F12, 0x0A24}, //0A24//0A24//70000C6E//AFIT8_sddd8a_edge_low [7:0] AFIT8_sddd8a_repl_thresh [15:8]
{0x0F12, 0x1701}, //1701//1701//70000C70//AFIT8_sddd8a_repl_force [7:0] AFIT8_sddd8a_sat_level [15:8]
{0x0F12, 0x0229}, //0229//0229//70000C72//AFIT8_sddd8a_sat_thr[7:0] AFIT8_sddd8a_sat_mpl [15:8]
{0x0F12, 0x1403}, //0503//0503//70000C74//AFIT8_sddd8a_sat_noise[7:0] AFIT8_sddd8a_iMaxSlopeAllowed [15:8]
{0x0F12, 0x0004}, //080F//0101//70000C76//AFIT8_sddd8a_iHotThreshHigh[7:0] AFIT8_sddd8a_iHotThreshLow [15:8]
{0x0F12, 0x0300}, //0808//0101//70000C78//AFIT8_sddd8a_iColdThreshHigh[7:0] AFIT8_sddd8a_iColdThreshLow [15:8]
{0x0F12, 0x0000}, //0000//0000//70000C7A//AFIT8_sddd8a_AddNoisePower1[7:0] AFIT8_sddd8a_AddNoisePower2 [15:8]
{0x0F12, 0x02FF}, //00FF//02FF//70000C7C//AFIT8_sddd8a_iSatSat[7:0] AFIT8_sddd8a_iRadialTune [15:8]
{0x0F12, 0x052D}, //012D//0396//70000C7E//AFIT8_sddd8a_iRadialLimit [7:0] AFIT8_sddd8a_iRadialPower [15:8]
{0x0F12, 0x1414}, //1414//1414//70000C80//AFIT8_sddd8a_iLowMaxSlopeAllowed [7:0] AFIT8_sddd8a_iHighMaxSlopeAllowed [15:8]
{0x0F12, 0x0301}, //0301//0301//70000C82//AFIT8_sddd8a_iLowSlopeThresh[7:0] AFIT8_sddd8a_iHighSlopeThresh [15:8]
{0x0F12, 0x0007}, //0007//0007//70000C84//AFIT8_sddd8a_iSquaresRounding [7:0]                        
{0x0F12, 0x1000}, //1000//1000//70000C86        
{0x0F12, 0x2003}, //2003//2003//70000C88        
{0x0F12, 0x101A}, //1020//1020//70000C8A        
{0x0F12, 0x0010}, //0010//0010//70000C8C        
{0x0F12, 0x1E80}, //1EFF//1E80//70000C8E//AFIT8_Demosaicing_iCentGrad[7:0] AFIT8_Demosaicing_iMonochrom[15:8]
{0x0F12, 0x1E08}, //1E06//1E06//70000C90//AFIT8_Demosaicing_iDecisionThresh[7:0] AFIT8_Demosaicing_iDesatThresh[15:8]
{0x0F12, 0x000A}, //060A//030C//70000C92//AFIT8_Demosaicing_iEnhThresh[7:0] AFIT8_Demosaicing_iGRDenoiseVal[15:8]
{0x0F12, 0x0000}, //0306//0103//70000C94//AFIT8_Demosaicing_iGBDenoiseVal[7:0] AFIT8_Demosaicing_iNearGrayDesat[15:8]
{0x0F12, 0x120A}, //8B0A//5A0A//70000C96//AFIT8_Demosaicing_iDFD_ReduceCoeff[7:0] AFIT8_Sharpening_iMSharpen[15:8]
{0x0F12, 0x1400}, //2837//2D00//70000C98//AFIT8_Sharpening_iMShThresh[7:0] AFIT8_Sharpening_iWSharpen[15:8]
{0x0F12, 0x0200}, //0110//0100//70000C9A//AFIT8_Sharpening_iWShThresh[7:0] AFIT8_Sharpening_nSharpWidth[15:8]
{0x0F12, 0xFF00}, //FF00//FF00//70000C9C//AFIT8_Sharpening_iReduceNegative[7:0] AFIT8_Sharpening_iShDespeckle[15:8]
{0x0F12, 0x0200}, //0200//0200//70000C9E//AFIT8_demsharpmix1_iRGBMultiplier[7:0] AFIT8_demsharpmix1_iFilterPower[15:8]
{0x0F12, 0x1B11}, //1E10//1E10//70000CA0//AFIT8_demsharpmix1_iBCoeff[7:0] AFIT8_demsharpmix1_iGCoeff[15:8]
{0x0F12, 0x0000}, //0000//0000//70000CA2//AFIT8_demsharpmix1_iWideMult[7:0] AFIT8_demsharpmix1_iNarrMult[15:8]
{0x0F12, 0x0009}, //0009//0009//70000CA4//AFIT8_demsharpmix1_iHystFalloff[7:0] AFIT8_demsharpmix1_iHystMinMult[15:8]
{0x0F12, 0x0406}, //0406//0406//70000CA6//AFIT8_demsharpmix1_iHystWidth[7:0] AFIT8_demsharpmix1_iHystFallLow[15:8]
{0x0F12, 0x0605}, //0705//0705//70000CA8//AFIT8_demsharpmix1_iHystFallHigh[7:0] AFIT8_demsharpmix1_iHystTune[15:8]
{0x0F12, 0x0307}, //0305//0305//70000CAA        
{0x0F12, 0x0609}, //0609//0609//70000CAC        
{0x0F12, 0x1C07}, //2C07//2C07//70000CAE        
{0x0F12, 0x1414}, //142C//142C//70000CB0        
{0x0F12, 0x0510}, //0B18//0B18//70000CB2//[15:8]iUVNRStrengthL [7:0]iMaxThreshH        
{0x0F12, 0x8005}, //800B//800B//70000CB4//[7:0]iUVNRStrengthH AFIT8_byr_cgras_iShadingPower[15:8]        
{0x0F12, 0x0080}, //0080//0080//70000CB6//AFIT8_RGBGamma2_iLinearity [7:0]  AFIT8_RGBGamma2_iDarkReduce [15:8]
{0x0F12, 0x0080}, //0080//0080//70000CB8//AFIT8_ccm_oscar_iSaturation[7:0]   AFIT8_RGB2YUV_iYOffset [15:8]
{0x0F12, 0x0080}, //0080//0080//70000CBA//AFIT8_RGB2YUV_iRGBGain [7:0]   AFIT8_RGB2YUV_iSaturation [15:8]
{0x0F12, 0x0101}, //5050//0101//70000CBC//AFIT8_sddd8a_iClustThresh_H [7:0]  AFIT8_sddd8a_iClustThresh_C [15:8]
{0x0F12, 0x0707}, //0101//0A0A//70000CBE//AFIT8_sddd8a_iClustMulT_H [7:0]   AFIT8_sddd8a_iClustMulT_C [15:8]
{0x0F12, 0x4B01}, //3201//3201//70000CC0//AFIT8_sddd8a_nClustLevel_H [7:0]   AFIT8_sddd8a_DispTH_Low [15:8]
{0x0F12, 0x184B}, //1832//1428//70000CC2//AFIT8_sddd8a_DispTH_High [7:0]   AFIT8_sddd8a_iDenThreshLow [15:8]
{0x0F12, 0x500C}, //210C//100C//70000CC4//AFIT8_sddd8a_iDenThreshHigh[7:0]   AFIT8_Demosaicing_iEdgeDesat [15:8]
{0x0F12, 0x0500}, //0A00//0500//70000CC6//AFIT8_Demosaicing_iEdgeDesatThrLow [7:0]   AFIT8_Demosaicing_iEdgeDesatThrHigh [15:8]
{0x0F12, 0x1C03}, //1E04//1E02//70000CC8//AFIT8_Demosaicing_iEdgeDesatLimit[7:0]  AFIT8_Demosaicing_iDemSharpenLow [15:8]
{0x0F12, 0x0A08}, //0A08//040C//70000CCA//AFIT8_Demosaicing_iDemSharpenHigh[7:0]   AFIT8_Demosaicing_iDemSharpThresh [15:8]
{0x0F12, 0x070C}, //070C//0828//70000CCC//AFIT8_Demosaicing_iDemShLowLimit [7:0]   AFIT8_Demosaicing_iDespeckleForDemsharp [15:8]
{0x0F12, 0x1428}, //3264//5064//70000CCE//AFIT8_Demosaicing_iDemBlurLow[7:0]   AFIT8_Demosaicing_iDemBlurHigh [15:8]
{0x0F12, 0x6401}, //5A02//4605//70000CD0//AFIT8_Demosaicing_iDemBlurRange[7:0]   AFIT8_Sharpening_iLowSharpPower [15:8]
{0x0F12, 0x281E}, //1040//1E68//70000CD2//AFIT8_Sharpening_iHighSharpPower[7:0]   AFIT8_Sharpening_iLowShDenoise [15:8]
{0x0F12, 0x200F}, //4012//201E//70000CD4//AFIT8_Sharpening_iHighShDenoise [7:0]   AFIT8_Sharpening_iReduceEdgeMinMult [15:8]
{0x0F12, 0x0204}, //0604//0604//70000CD6//AFIT8_Sharpening_iReduceEdgeSlope [7:0]  AFIT8_demsharpmix1_iWideFiltReduce [15:8]
{0x0F12, 0x4603}, //4606//4606//70000CD8//AFIT8_demsharpmix1_iNarrFiltReduce [7:0]  AFIT8_sddd8a_iClustThresh_H_Bin [15:8]
{0x0F12, 0x0146}, //0146//0146//70000CDA//AFIT8_sddd8a_iClustThresh_C_Bin [7:0]   AFIT8_sddd8a_iClustMulT_H_Bin [15:8]
{0x0F12, 0x0101}, //0101//0101//70000CDC//AFIT8_sddd8a_iClustMulT_C_Bin [7:0]   AFIT8_sddd8a_nClustLevel_H_Bin [15:8]
{0x0F12, 0x1C46}, //1C18//1C18//70000CDE//AFIT8_sddd8a_DispTH_Low_Bin [7:0]   AFIT8_sddd8a_DispTH_High_Bin [15:8]
{0x0F12, 0x1819}, //1819//1819//70000CE0//AFIT8_sddd8a_iDenThreshLow_Bin [7:0]   AFIT8_sddd8a_iDenThreshHigh_Bin [15:8]
{0x0F12, 0x0028}, //0028//0028//70000CE2//AFIT8_Demosaicing_iEdgeDesat_Bin[7:0]   AFIT8_Demosaicing_iEdgeDesatThrLow_Bin [15:8]
{0x0F12, 0x030A}, //030A//030A//70000CE4//AFIT8_Demosaicing_iEdgeDesatThrHigh_Bin [7:0]  AFIT8_Demosaicing_iEdgeDesatLimit_Bin [15:8]
{0x0F12, 0x0000}, //0514//0514//70000CE6//AFIT8_Demosaicing_iDemSharpenLow_Bin [7:0]  AFIT8_Demosaicing_iDemSharpenHigh_Bin [15:8]
{0x0F12, 0x141E}, //0C14//0C14//70000CE8//AFIT8_Demosaicing_iDemSharpThresh_Bin [7:0]  AFIT8_Demosaicing_iDemShLowLimit_Bin [15:8]
{0x0F12, 0xFF07}, //FF05//FF05//70000CEA//AFIT8_Demosaicing_iDespeckleForDemsharp_Bin [7:0]  AFIT8_Demosaicing_iDemBlurLow_Bin [15:8]
{0x0F12, 0x0432}, //0432//0432//70000CEC//AFIT8_Demosaicing_iDemBlurHigh_Bin [7:0]  AFIT8_Demosaicing_iDemBlurRange_Bin [15:8]
{0x0F12, 0x0000}, //4052//4052//70000CEE//AFIT8_Sharpening_iLowSharpPower_Bin [7:0]  AFIT8_Sharpening_iHighSharpPower_Bin [15:8]
{0x0F12, 0x0F0F}, //1514//1514//70000CF0//AFIT8_Sharpening_iLowShDenoise_Bin [7:0]  AFIT8_Sharpening_iHighShDenoise_Bin [15:8]
{0x0F12, 0x0440}, //0440//0440//70000CF2//AFIT8_Sharpening_iReduceEdgeMinMult_Bin [7:0]  AFIT8_Sharpening_iReduceEdgeSlope_Bin [15:8]
{0x0F12, 0x0302}, //0302//0302//70000CF4//AFIT8_demsharpmix1_iWideFiltReduce_Bin [7:0]  AFIT8_demsharpmix1_iNarrFiltReduce_Bin [15:8]
{0x0F12, 0x1414}, //4646//4646//70000CF6//AFIT8_sddd8a_iClustThresh_H_sBin[7:0]   AFIT8_sddd8a_iClustThresh_C_sBin [15:8]
{0x0F12, 0x0101}, //0101//0101//70000CF8//AFIT8_sddd8a_iClustMulT_H_sBin [7:0]   AFIT8_sddd8a_iClustMulT_C_sBin [15:8]
{0x0F12, 0x1801}, //1801//1801//70000CFA//AFIT8_sddd8a_nClustLevel_H_sBin [7:0]   AFIT8_sddd8a_DispTH_Low_sBin [15:8]
{0x0F12, 0x191C}, //191C//191C//70000CFC//AFIT8_sddd8a_DispTH_High_sBin [7:0]   AFIT8_sddd8a_iDenThreshLow_sBin [15:8]
{0x0F12, 0x2818}, //2818//2818//70000CFE//AFIT8_sddd8a_iDenThreshHigh_sBin[7:0]   AFIT8_Demosaicing_iEdgeDesat_sBin [15:8]
{0x0F12, 0x0A00}, //0A00//0A00//70000D00//AFIT8_Demosaicing_iEdgeDesatThrLow_sBin [7:0]  AFIT8_Demosaicing_iEdgeDesatThrHigh_sBin [15:8]
{0x0F12, 0x1403}, //1403//1403//70000D02//AFIT8_Demosaicing_iEdgeDesatLimit_sBin [7:0]  AFIT8_Demosaicing_iDemSharpenLow_sBin [15:8]
{0x0F12, 0x1400}, //1405//1405//70000D04//AFIT8_Demosaicing_iDemSharpenHigh_sBin [7:0]  AFIT8_Demosaicing_iDemSharpThresh_sBin [15:8]
{0x0F12, 0x0714}, //050C//050C//70000D06//AFIT8_Demosaicing_iDemShLowLimit_sBin [7:0]  AFIT8_Demosaicing_iDespeckleForDemsharp_sBin [15:8]
{0x0F12, 0x32FF}, //32FF//32FF//70000D08//AFIT8_Demosaicing_iDemBlurLow_sBin [7:0]  AFIT8_Demosaicing_iDemBlurHigh_sBin [15:8]
{0x0F12, 0x0004}, //5204//5204//70000D0A//AFIT8_Demosaicing_iDemBlurRange_sBin [7:0]  AFIT8_Sharpening_iLowSharpPower_sBin [15:8]
{0x0F12, 0x0F00}, //1440//1440//70000D0C//AFIT8_Sharpening_iHighSharpPower_sBin [7:0]  AFIT8_Sharpening_iLowShDenoise_sBin [15:8]
{0x0F12, 0x400F}, //4015//4015//70000D0E//AFIT8_Sharpening_iHighShDenoise_sBin [7:0]  AFIT8_Sharpening_iReduceEdgeMinMult_sBin [15:8]
{0x0F12, 0x0204}, //0204//0204//70000D10//AFIT8_Sharpening_iReduceEdgeSlope_sBin [7:0]  AFIT8_demsharpmix1_iWideFiltReduce_sBin [15:8]
{0x0F12, 0x0003}, //0003//0003//70000D12//AFIT8_demsharpmix1_iNarrFiltReduce_sBin [7:0]
{0x0F12, 0x0001}, //0001//0001//70000D14
      

{0x0F12, 0xBA7A},	//70000D16
{0x0F12, 0x4FDE},	//70000D18
{0x0F12, 0x137F},	//70000D1A
{0x0F12, 0x3BDE},	//70000D1C
{0x0F12, 0xA102},	//70000D1E
{0x0F12, 0x00B5},	//70000D20

//added
//PREVIEW
{0x002A, 0x0208},
{0x0F12, 0x0000},	//REG_TC_GP_ActivePrevConfig
{0x002A, 0x0210},
{0x0F12, 0x0000},	//REG_TC_GP_ActiveCapConfig
{0x002A, 0x020C},
{0x0F12, 0x0001},	//REG_TC_GP_PrevOpenAfterChange
{0x002A, 0x01F4},
{0x0F12, 0x0001},	//REG_TC_GP_NewConfigSync
{0x002A, 0x020A},
{0x0F12, 0x0001},	//REG_TC_GP_PrevConfigChanged
{0x002A, 0x0212},
{0x0F12, 0x0001},	//REG_TC_GP_CapConfigChanged
{0x002A, 0x01E8},
{0x0F12, 0x0000},	//REG_TC_GP_EnableCapture
{0x0F12, 0x0001},	//REG_TC_GP_EnableCaptureChanged

{0xFFFF, 0x012C},	// delay p300


//added DTP 2nd start
{0xFCFC, 0xD000},
{0x0028, 0x7000},
//	param_start	afit_uNoiseIndInDoor
{0x002A, 0x085C},
{0x0F12, 0x004A},	//0049	//#afit_uNoiseIndInDoor_0_
{0x0F12, 0x004E},	//005F	//#afit_uNoiseIndInDoor_1_
{0x0F12, 0x00CB},	//00CB	//#afit_uNoiseIndInDoor_2_
{0x0F12, 0x01C0},	//01E0	//#afit_uNoiseIndInDoor_3_
{0x0F12, 0x0200},	//0220	//#afit_uNoiseIndInDoor_4_

{0x0028, 0x7000},
{0x002A, 0x0458},
{0x0F12, 0x0001},
{0x002A, 0x045C},
{0x0F12, 0x0001},	//REG_SF_USER_LeiChanged

{0x0028, 0xD000},
{0x002A, 0x4128},
{0x0F12, 0x08A3},	//GAS bypass
{0x002A, 0x6700},
{0x0F12, 0x0001},	//CCM bypass
{0x002A, 0x6800},
{0x0F12, 0x0001},	//Gamma bypass
{0x002A, 0x4700},
{0x0F12, 0x0001},	//AWB bypass
{0x002A, 0xC200},

{0xFFFF, 0x012C},	// delay p300

//added DTP 2nd end

{0x0028, 0xD000},
{0x002A, 0x4800},
{0x0F12, 0x0002},	//Colorbar pattern
{0x002A, 0x4818},
{0x0F12, 0x0C00},	//Colorbar size

{0xFFFF, 0x012C},	// delay p300

CAMACQ_MAIN_BTM_OF_DATA
}
#endif /* _CAMACQ_API_C_ */
;

GLOBAL const U16 reg_main_dtp_off[][2] 
#if defined(_CAMACQ_API_C_)
={
{0xFCFC, 0xD000},
{0x0028, 0xD000},
{0x002A, 0xB054},
{0x0F12, 0x0000},

CAMACQ_MAIN_BTM_OF_DATA
}
#endif /* _CAMACQ_API_C_ */
;


GLOBAL const U16 reg_main_init[][2]
#if defined(_CAMACQ_API_C_)
={
{0xFCFC, 0xD000},

{0x0010, 0x0001},
{0x1030, 0x0000},
{0x0014, 0x0001},

{0xFFFF, 0x000A},						//Delay 10ms

{0x0028, 0x7000},
{0x002A, 0x0150},
{0x0F12, 0xAAAA},    // ESD check

{0x0028, 0x7000},
{0x002A, 0x352C},
{0x0F12, 0xB510},
{0x0F12, 0x4A2C},
{0x0F12, 0x213F},
{0x0F12, 0x482C},
{0x0F12, 0x4B2C},
{0x0F12, 0x2400},
{0x0F12, 0x801C},
{0x0F12, 0xC004},
{0x0F12, 0x6001},
{0x0F12, 0x492B},
{0x0F12, 0x482B},
{0x0F12, 0xF000},
{0x0F12, 0xFBE9},
{0x0F12, 0x2115},
{0x0F12, 0x482A},
{0x0F12, 0x01C9},
{0x0F12, 0xF000},
{0x0F12, 0xF88E},
{0x0F12, 0x4828},
{0x0F12, 0x210B},
{0x0F12, 0x0189},
{0x0F12, 0x3018},
{0x0F12, 0xF000},
{0x0F12, 0xF888},
{0x0F12, 0x4825},
{0x0F12, 0x4926},
{0x0F12, 0x300C},
{0x0F12, 0xF000},
{0x0F12, 0xF883},
{0x0F12, 0x4823},
{0x0F12, 0x4924},
{0x0F12, 0x3010},
{0x0F12, 0xF000},
{0x0F12, 0xF87E},
{0x0F12, 0x4923},
{0x0F12, 0x4824},
{0x0F12, 0xF000},
{0x0F12, 0xFBD0},
{0x0F12, 0x4923},
{0x0F12, 0x4824},
{0x0F12, 0xF000},
{0x0F12, 0xFBCC},
{0x0F12, 0x4923},
{0x0F12, 0x4824},
{0x0F12, 0xF000},
{0x0F12, 0xFBC8},
{0x0F12, 0x4923},
{0x0F12, 0x4824},
{0x0F12, 0xF000},
{0x0F12, 0xFBC4},
{0x0F12, 0x4923},
{0x0F12, 0x4824},
{0x0F12, 0xF000},
{0x0F12, 0xFBC0},
{0x0F12, 0x4823},
{0x0F12, 0x4924},
{0x0F12, 0x6408},
{0x0F12, 0x4924},
{0x0F12, 0x4824},
{0x0F12, 0xF000},
{0x0F12, 0xFBB9},
{0x0F12, 0x4924},
{0x0F12, 0x4824},
{0x0F12, 0xF000},
{0x0F12, 0xFBB5},
{0x0F12, 0x4924},
{0x0F12, 0x4824},
{0x0F12, 0xF000},
{0x0F12, 0xFBB1},
{0x0F12, 0x4924},
{0x0F12, 0x4824},
{0x0F12, 0xF000},
{0x0F12, 0xFBAD},
{0x0F12, 0x4924},
{0x0F12, 0x4824},
{0x0F12, 0xF000},
{0x0F12, 0xFBA9},
{0x0F12, 0x4824},
{0x0F12, 0x8104},
{0x0F12, 0x4924},
{0x0F12, 0x4824},
{0x0F12, 0xF000},
{0x0F12, 0xFBA3},
{0x0F12, 0x4924},
{0x0F12, 0x4824},
{0x0F12, 0xF000},
{0x0F12, 0xFB9F},
{0x0F12, 0xBC10},
{0x0F12, 0xBC08},
{0x0F12, 0x4718},
{0x0F12, 0x00BA},
{0x0F12, 0x5CC1},
{0x0F12, 0x1C08},
{0x0F12, 0x7000},
{0x0F12, 0x3290},
{0x0F12, 0x7000},
{0x0F12, 0x368B},
{0x0F12, 0x7000},
{0x0F12, 0xD9E7},
{0x0F12, 0x0000},
{0x0F12, 0x6FC0},
{0x0F12, 0x0000},
{0x0F12, 0x0A91},
{0x0F12, 0x0000},
{0x0F12, 0x02C9},
{0x0F12, 0x0000},
{0x0F12, 0x36C3},
{0x0F12, 0x7000},
{0x0F12, 0xA607},
{0x0F12, 0x0000},
{0x0F12, 0x3733},
{0x0F12, 0x7000},
{0x0F12, 0x7C0D},
{0x0F12, 0x0000},
{0x0F12, 0x374F},
{0x0F12, 0x7000},
{0x0F12, 0x7C2B},
{0x0F12, 0x0000},
{0x0F12, 0x376B},
{0x0F12, 0x7000},
{0x0F12, 0x9E89},
{0x0F12, 0x0000},
{0x0F12, 0x394F},
{0x0F12, 0x7000},
{0x0F12, 0x395D},
{0x0F12, 0x0000},
{0x0F12, 0x39DB},
{0x0F12, 0x7000},
{0x0F12, 0x0000},
{0x0F12, 0x7000},
{0x0F12, 0x3B01},
{0x0F12, 0x7000},
{0x0F12, 0xF903},
{0x0F12, 0x0000},
{0x0F12, 0x37B3},
{0x0F12, 0x7000},
{0x0F12, 0x495F},
{0x0F12, 0x0000},
{0x0F12, 0x380D},
{0x0F12, 0x7000},
{0x0F12, 0xE421},
{0x0F12, 0x0000},
{0x0F12, 0x38BB},
{0x0F12, 0x7000},
{0x0F12, 0x216D},
{0x0F12, 0x0000},
{0x0F12, 0x392F},
{0x0F12, 0x7000},
{0x0F12, 0x0179},
{0x0F12, 0x0001},
{0x0F12, 0x3FC8},
{0x0F12, 0x7000},
{0x0F12, 0x3CE3},
{0x0F12, 0x7000},
{0x0F12, 0x04C9},
{0x0F12, 0x0000},
{0x0F12, 0x3C2F},
{0x0F12, 0x7000},
{0x0F12, 0x5027},
{0x0F12, 0x0000},
{0x0F12, 0xB570},
{0x0F12, 0x000D},
{0x0F12, 0x4CFF},
{0x0F12, 0x8821},
{0x0F12, 0xF000},
{0x0F12, 0xFB58},
{0x0F12, 0x8820},
{0x0F12, 0x4AFE},
{0x0F12, 0x0081},
{0x0F12, 0x5055},
{0x0F12, 0x1C40},
{0x0F12, 0x8020},
{0x0F12, 0xBC70},
{0x0F12, 0xBC08},
{0x0F12, 0x4718},
{0x0F12, 0x6801},
{0x0F12, 0x0409},
{0x0F12, 0x0C09},
{0x0F12, 0x6840},
{0x0F12, 0x0400},
{0x0F12, 0x0C00},
{0x0F12, 0x4AF8},
{0x0F12, 0x8992},
{0x0F12, 0x2A00},
{0x0F12, 0xD00D},
{0x0F12, 0x2300},
{0x0F12, 0x1A80},
{0x0F12, 0xD400},
{0x0F12, 0x0003},
{0x0F12, 0x0418},
{0x0F12, 0x0C00},
{0x0F12, 0x4BF4},
{0x0F12, 0x1851},
{0x0F12, 0x891B},
{0x0F12, 0x428B},
{0x0F12, 0xD300},
{0x0F12, 0x000B},
{0x0F12, 0x0419},
{0x0F12, 0x0C09},
{0x0F12, 0x4AF1},
{0x0F12, 0x8151},
{0x0F12, 0x8190},
{0x0F12, 0x4770},
{0x0F12, 0xB510},
{0x0F12, 0x48EF},
{0x0F12, 0x4CF0},
{0x0F12, 0x88C1},
{0x0F12, 0x8061},
{0x0F12, 0x2101},
{0x0F12, 0x8021},
{0x0F12, 0x8840},
{0x0F12, 0xF000},
{0x0F12, 0xFB31},
{0x0F12, 0x88E0},
{0x0F12, 0x4AEC},
{0x0F12, 0x2800},
{0x0F12, 0xD003},
{0x0F12, 0x49EC},
{0x0F12, 0x8849},
{0x0F12, 0x2900},
{0x0F12, 0xD009},
{0x0F12, 0x2001},
{0x0F12, 0x03C0},
{0x0F12, 0x8050},
{0x0F12, 0x80D0},
{0x0F12, 0x2000},
{0x0F12, 0x8090},
{0x0F12, 0x8110},
{0x0F12, 0xBC10},
{0x0F12, 0xBC08},
{0x0F12, 0x4718},
{0x0F12, 0x8050},
{0x0F12, 0x8920},
{0x0F12, 0x80D0},
{0x0F12, 0x8960},
{0x0F12, 0x0400},
{0x0F12, 0x1400},
{0x0F12, 0x8090},
{0x0F12, 0x89A1},
{0x0F12, 0x0409},
{0x0F12, 0x1409},
{0x0F12, 0x8111},
{0x0F12, 0x89E3},
{0x0F12, 0x8A24},
{0x0F12, 0x2B00},
{0x0F12, 0xD104},
{0x0F12, 0x17C3},
{0x0F12, 0x0F5B},
{0x0F12, 0x1818},
{0x0F12, 0x10C0},
{0x0F12, 0x8090},
{0x0F12, 0x2C00},
{0x0F12, 0xD1E6},
{0x0F12, 0x17C8},
{0x0F12, 0x0F40},
{0x0F12, 0x1840},
{0x0F12, 0x10C0},
{0x0F12, 0x8110},
{0x0F12, 0xE7E0},
{0x0F12, 0xB510},
{0x0F12, 0x0004},
{0x0F12, 0x49D5},
{0x0F12, 0x2204},
{0x0F12, 0x6820},
{0x0F12, 0x5E8A},
{0x0F12, 0x0140},
{0x0F12, 0x1A80},
{0x0F12, 0x0280},
{0x0F12, 0x8849},
{0x0F12, 0xF000},
{0x0F12, 0xFAFF},
{0x0F12, 0x6020},
{0x0F12, 0xE7D2},
{0x0F12, 0xB510},
{0x0F12, 0x0004},
{0x0F12, 0x49CE},
{0x0F12, 0x2208},
{0x0F12, 0x6820},
{0x0F12, 0x5E8A},
{0x0F12, 0x0140},
{0x0F12, 0x1A80},
{0x0F12, 0x0280},
{0x0F12, 0x88C9},
{0x0F12, 0xF000},
{0x0F12, 0xFAF1},
{0x0F12, 0x6020},
{0x0F12, 0xE7C4},
{0x0F12, 0xB510},
{0x0F12, 0x0004},
{0x0F12, 0x49C7},
{0x0F12, 0x48C8},
{0x0F12, 0x884A},
{0x0F12, 0x8B43},
{0x0F12, 0x435A},
{0x0F12, 0x2304},
{0x0F12, 0x5ECB},
{0x0F12, 0x0A92},
{0x0F12, 0x18D2},
{0x0F12, 0x02D2},
{0x0F12, 0x0C12},
{0x0F12, 0x88CB},
{0x0F12, 0x8B80},
{0x0F12, 0x4343},
{0x0F12, 0x0A98},
{0x0F12, 0x2308},
{0x0F12, 0x5ECB},
{0x0F12, 0x18C0},
{0x0F12, 0x02C0},
{0x0F12, 0x0C00},
{0x0F12, 0x0411},
{0x0F12, 0x0400},
{0x0F12, 0x1409},
{0x0F12, 0x1400},
{0x0F12, 0x1A08},
{0x0F12, 0x49BC},
{0x0F12, 0x3980},
{0x0F12, 0x6348},
{0x0F12, 0x0020},
{0x0F12, 0xC80F},
{0x0F12, 0xF000},
{0x0F12, 0xFAD3},
{0x0F12, 0x6020},
{0x0F12, 0xE7A0},
{0x0F12, 0xB510},
{0x0F12, 0x4CB8},
{0x0F12, 0x48B9},
{0x0F12, 0x78A1},
{0x0F12, 0x2900},
{0x0F12, 0xD101},
{0x0F12, 0x87C1},
{0x0F12, 0xE004},
{0x0F12, 0x7AE1},
{0x0F12, 0x2900},
{0x0F12, 0xD001},
{0x0F12, 0x2101},
{0x0F12, 0x87C1},
{0x0F12, 0xF000},
{0x0F12, 0xFACA},
{0x0F12, 0x49B3},
{0x0F12, 0x8B08},
{0x0F12, 0x06C2},
{0x0F12, 0xD50A},
{0x0F12, 0x7AA2},
{0x0F12, 0x0652},
{0x0F12, 0xD507},
{0x0F12, 0x2210},
{0x0F12, 0x4390},
{0x0F12, 0x8308},
{0x0F12, 0x48AF},
{0x0F12, 0x7AE1},
{0x0F12, 0x6B00},
{0x0F12, 0xF000},
{0x0F12, 0xFAC3},
{0x0F12, 0x48A2},
{0x0F12, 0x89C0},
{0x0F12, 0x2801},
{0x0F12, 0xD109},
{0x0F12, 0x78A0},
{0x0F12, 0x2800},
{0x0F12, 0xD006},
{0x0F12, 0x7AE0},
{0x0F12, 0x2800},
{0x0F12, 0xD003},
{0x0F12, 0x7AA0},
{0x0F12, 0x2140},
{0x0F12, 0x4308},
{0x0F12, 0x72A0},
{0x0F12, 0xE773},
{0x0F12, 0xB570},
{0x0F12, 0x4DA4},
{0x0F12, 0x4CA4},
{0x0F12, 0x8B28},
{0x0F12, 0x0701},
{0x0F12, 0xD507},
{0x0F12, 0x2108},
{0x0F12, 0x4388},
{0x0F12, 0x8328},
{0x0F12, 0x49A2},
{0x0F12, 0x6B20},
{0x0F12, 0x6B89},
{0x0F12, 0xF000},
{0x0F12, 0xFAAE},
{0x0F12, 0x8B28},
{0x0F12, 0x06C1},
{0x0F12, 0xD50A},
{0x0F12, 0x499A},
{0x0F12, 0x7A8A},
{0x0F12, 0x0652},
{0x0F12, 0xD406},
{0x0F12, 0x2210},
{0x0F12, 0x4390},
{0x0F12, 0x8328},
{0x0F12, 0x7AC9},
{0x0F12, 0x6B20},
{0x0F12, 0xF000},
{0x0F12, 0xFA98},
{0x0F12, 0xE71E},
{0x0F12, 0xB5F8},
{0x0F12, 0x4C98},
{0x0F12, 0x26FF},
{0x0F12, 0x8820},
{0x0F12, 0x4D98},
{0x0F12, 0x1C76},
{0x0F12, 0x2702},
{0x0F12, 0x2803},
{0x0F12, 0xD112},
{0x0F12, 0x8860},
{0x0F12, 0x2800},
{0x0F12, 0xD10F},
{0x0F12, 0x88E0},
{0x0F12, 0x2800},
{0x0F12, 0xD10C},
{0x0F12, 0xF000},
{0x0F12, 0xFA96},
{0x0F12, 0x2800},
{0x0F12, 0xD008},
{0x0F12, 0x8F28},
{0x0F12, 0x2800},
{0x0F12, 0xD001},
{0x0F12, 0x80E6},
{0x0F12, 0x80A7},
{0x0F12, 0x2001},
{0x0F12, 0x7260},
{0x0F12, 0xF000},
{0x0F12, 0xFA93},
{0x0F12, 0x8820},
{0x0F12, 0x2802},
{0x0F12, 0xD10E},
{0x0F12, 0x8860},
{0x0F12, 0x2800},
{0x0F12, 0xD10B},
{0x0F12, 0x88E0},
{0x0F12, 0x2800},
{0x0F12, 0xD108},
{0x0F12, 0x8F28},
{0x0F12, 0x2800},
{0x0F12, 0xD001},
{0x0F12, 0x80E6},
{0x0F12, 0x80A7},
{0x0F12, 0x2001},
{0x0F12, 0x7260},
{0x0F12, 0xF000},
{0x0F12, 0xFA81},
{0x0F12, 0x88E0},
{0x0F12, 0x2800},
{0x0F12, 0xD006},
{0x0F12, 0x1FC1},
{0x0F12, 0x39FD},
{0x0F12, 0xD003},
{0x0F12, 0x2001},
{0x0F12, 0xBCF8},
{0x0F12, 0xBC08},
{0x0F12, 0x4718},
{0x0F12, 0x2000},
{0x0F12, 0xE7FA},
{0x0F12, 0xB570},
{0x0F12, 0x4C7D},
{0x0F12, 0x8860},
{0x0F12, 0x2800},
{0x0F12, 0xD00C},
{0x0F12, 0x8820},
{0x0F12, 0x4D74},
{0x0F12, 0x2800},
{0x0F12, 0xD009},
{0x0F12, 0x0029},
{0x0F12, 0x31A0},
{0x0F12, 0x7AC9},
{0x0F12, 0x2900},
{0x0F12, 0xD004},
{0x0F12, 0x7AA8},
{0x0F12, 0x2180},
{0x0F12, 0x4308},
{0x0F12, 0x72A8},
{0x0F12, 0xE6D1},
{0x0F12, 0x2800},
{0x0F12, 0xD003},
{0x0F12, 0xF7FF},
{0x0F12, 0xFFAF},
{0x0F12, 0x2800},
{0x0F12, 0xD1F8},
{0x0F12, 0x2000},
{0x0F12, 0x8060},
{0x0F12, 0x8820},
{0x0F12, 0x2800},
{0x0F12, 0xD003},
{0x0F12, 0x2008},
{0x0F12, 0xF000},
{0x0F12, 0xFA5C},
{0x0F12, 0xE00B},
{0x0F12, 0x486D},
{0x0F12, 0x3020},
{0x0F12, 0x8880},
{0x0F12, 0x2800},
{0x0F12, 0xD103},
{0x0F12, 0x7AA8},
{0x0F12, 0x2101},
{0x0F12, 0x4308},
{0x0F12, 0x72A8},
{0x0F12, 0x2010},
{0x0F12, 0xF000},
{0x0F12, 0xFA4F},
{0x0F12, 0x8820},
{0x0F12, 0x2800},
{0x0F12, 0xD1E0},
{0x0F12, 0x4856},
{0x0F12, 0x89C0},
{0x0F12, 0x2801},
{0x0F12, 0xD1DC},
{0x0F12, 0x7AA8},
{0x0F12, 0x21BF},
{0x0F12, 0x4008},
{0x0F12, 0x72A8},
{0x0F12, 0xE6AA},
{0x0F12, 0x6800},
{0x0F12, 0x4961},
{0x0F12, 0x8188},
{0x0F12, 0x4861},
{0x0F12, 0x2201},
{0x0F12, 0x8981},
{0x0F12, 0x4861},
{0x0F12, 0x0252},
{0x0F12, 0x4291},
{0x0F12, 0xD902},
{0x0F12, 0x2102},
{0x0F12, 0x8181},
{0x0F12, 0x4770},
{0x0F12, 0x2101},
{0x0F12, 0x8181},
{0x0F12, 0x4770},
{0x0F12, 0xB5F1},
{0x0F12, 0x4E51},
{0x0F12, 0x8834},
{0x0F12, 0x2C00},
{0x0F12, 0xD03C},
{0x0F12, 0x2001},
{0x0F12, 0x2C08},
{0x0F12, 0xD000},
{0x0F12, 0x2000},
{0x0F12, 0x70B0},
{0x0F12, 0x4D50},
{0x0F12, 0x2700},
{0x0F12, 0x2800},
{0x0F12, 0xD009},
{0x0F12, 0xF000},
{0x0F12, 0xFA2B},
{0x0F12, 0x0028},
{0x0F12, 0x38F0},
{0x0F12, 0x6328},
{0x0F12, 0x7AB0},
{0x0F12, 0x217E},
{0x0F12, 0x4008},
{0x0F12, 0x72B0},
{0x0F12, 0xE00C},
{0x0F12, 0x484C},
{0x0F12, 0x8F00},
{0x0F12, 0x2800},
{0x0F12, 0xD003},
{0x0F12, 0xF000},
{0x0F12, 0xFA25},
{0x0F12, 0x4849},
{0x0F12, 0x8707},
{0x0F12, 0x2000},
{0x0F12, 0xF000},
{0x0F12, 0xFA28},
{0x0F12, 0x484B},
{0x0F12, 0x6328},
{0x0F12, 0x78B1},
{0x0F12, 0x0038},
{0x0F12, 0x2900},
{0x0F12, 0xD008},
{0x0F12, 0x4944},
{0x0F12, 0x3920},
{0x0F12, 0x8ACA},
{0x0F12, 0x2A00},
{0x0F12, 0xD003},
{0x0F12, 0x8B09},
{0x0F12, 0x2900},
{0x0F12, 0xD000},
{0x0F12, 0x2001},
{0x0F12, 0x7170},
{0x0F12, 0x2C02},
{0x0F12, 0xD102},
{0x0F12, 0x483A},
{0x0F12, 0x3860},
{0x0F12, 0x6328},
{0x0F12, 0x2201},
{0x0F12, 0x2C02},
{0x0F12, 0xD000},
{0x0F12, 0x2200},
{0x0F12, 0x4834},
{0x0F12, 0x2110},
{0x0F12, 0x300A},
{0x0F12, 0xF000},
{0x0F12, 0xFA12},
{0x0F12, 0x8037},
{0x0F12, 0x9900},
{0x0F12, 0x0020},
{0x0F12, 0x600C},
{0x0F12, 0xE76A},
{0x0F12, 0xB538},
{0x0F12, 0x4837},
{0x0F12, 0x4669},
{0x0F12, 0x3848},
{0x0F12, 0xF000},
{0x0F12, 0xFA0F},
{0x0F12, 0x4A32},
{0x0F12, 0x4834},
{0x0F12, 0x8F51},
{0x0F12, 0x2400},
{0x0F12, 0x3020},
{0x0F12, 0x2900},
{0x0F12, 0xD00A},
{0x0F12, 0x8754},
{0x0F12, 0x6941},
{0x0F12, 0x6451},
{0x0F12, 0x6491},
{0x0F12, 0x466B},
{0x0F12, 0x8819},
{0x0F12, 0x87D1},
{0x0F12, 0x885B},
{0x0F12, 0x0011},
{0x0F12, 0x3140},
{0x0F12, 0x800B},
{0x0F12, 0x8F91},
{0x0F12, 0x2900},
{0x0F12, 0xD002},
{0x0F12, 0x8794},
{0x0F12, 0x6940},
{0x0F12, 0x6490},
{0x0F12, 0xF000},
{0x0F12, 0xF9FD},
{0x0F12, 0xBC38},
{0x0F12, 0xBC08},
{0x0F12, 0x4718},
{0x0F12, 0xB5F8},
{0x0F12, 0x4C29},
{0x0F12, 0x89E0},
{0x0F12, 0xF000},
{0x0F12, 0xF9FD},
{0x0F12, 0x0006},
{0x0F12, 0x8A20},
{0x0F12, 0xF000},
{0x0F12, 0xFA01},
{0x0F12, 0x0007},
{0x0F12, 0x4821},
{0x0F12, 0x4D1E},
{0x0F12, 0x3020},
{0x0F12, 0x6CA9},
{0x0F12, 0x6940},
{0x0F12, 0x1809},
{0x0F12, 0x0200},
{0x0F12, 0xF000},
{0x0F12, 0xF9FF},
{0x0F12, 0x0400},
{0x0F12, 0x0C00},
{0x0F12, 0x002A},
{0x0F12, 0x326E},
{0x0F12, 0x0011},
{0x0F12, 0x390A},
{0x0F12, 0x2305},
{0x0F12, 0xF000},
{0x0F12, 0xF9FC},
{0x0F12, 0x4C14},
{0x0F12, 0x61A0},
{0x0F12, 0x8FEB},
{0x0F12, 0x0002},
{0x0F12, 0x0031},
{0x0F12, 0x0018},
{0x0F12, 0xF000},
{0x0F12, 0xF9FC},
{0x0F12, 0x466B},
{0x0F12, 0x0005},
{0x0F12, 0x8018},
{0x0F12, 0xE02D},
{0x0F12, 0x3290},
{0x0F12, 0x7000},
{0x0F12, 0x3294},
{0x0F12, 0x7000},
{0x0F12, 0x04A8},
{0x0F12, 0x7000},
{0x0F12, 0x15DC},
{0x0F12, 0x7000},
{0x0F12, 0x5000},
{0x0F12, 0xD000},
{0x0F12, 0x064C},
{0x0F12, 0x7000},
{0x0F12, 0xA000},
{0x0F12, 0xD000},
{0x0F12, 0x2468},
{0x0F12, 0x7000},
{0x0F12, 0x11DC},
{0x0F12, 0x7000},
{0x0F12, 0x2828},
{0x0F12, 0x7000},
{0x0F12, 0x1E84},
{0x0F12, 0x7000},
{0x0F12, 0x1BE4},
{0x0F12, 0x7000},
{0x0F12, 0x2EA8},
{0x0F12, 0x7000},
{0x0F12, 0x21A4},
{0x0F12, 0x7000},
{0x0F12, 0x0100},
{0x0F12, 0x7000},
{0x0F12, 0x31A0},
{0x0F12, 0x7000},
{0x0F12, 0x3F48},
{0x0F12, 0x7000},
{0x0F12, 0x01E8},
{0x0F12, 0x7000},
{0x0F12, 0xF2A0},
{0x0F12, 0xD000},
{0x0F12, 0x2A44},
{0x0F12, 0x7000},
{0x0F12, 0xF400},
{0x0F12, 0xD000},
{0x0F12, 0x2024},
{0x0F12, 0x7000},
{0x0F12, 0x1650},
{0x0F12, 0x7000},
{0x0F12, 0x4888},
{0x0F12, 0x69A2},
{0x0F12, 0x8800},
{0x0F12, 0x0039},
{0x0F12, 0xF000},
{0x0F12, 0xF9C4},
{0x0F12, 0x466B},
{0x0F12, 0x0006},
{0x0F12, 0x8058},
{0x0F12, 0x0021},
{0x0F12, 0x9800},
{0x0F12, 0x311C},
{0x0F12, 0xF000},
{0x0F12, 0xF9C4},
{0x0F12, 0x4981},
{0x0F12, 0x3140},
{0x0F12, 0x808D},
{0x0F12, 0x80CE},
{0x0F12, 0x8BA1},
{0x0F12, 0x4880},
{0x0F12, 0x8001},
{0x0F12, 0x8BE1},
{0x0F12, 0x8041},
{0x0F12, 0x8C21},
{0x0F12, 0x8081},
{0x0F12, 0xE6D7},
{0x0F12, 0xB5F8},
{0x0F12, 0x4E7B},
{0x0F12, 0x3E40},
{0x0F12, 0x6C70},
{0x0F12, 0x6CB1},
{0x0F12, 0x0200},
{0x0F12, 0xF000},
{0x0F12, 0xF99A},
{0x0F12, 0x0400},
{0x0F12, 0x0C00},
{0x0F12, 0x2401},
{0x0F12, 0x0364},
{0x0F12, 0x42A0},
{0x0F12, 0xD200},
{0x0F12, 0x0004},
{0x0F12, 0x4A74},
{0x0F12, 0x0020},
{0x0F12, 0x323E},
{0x0F12, 0x1F91},
{0x0F12, 0x2303},
{0x0F12, 0xF000},
{0x0F12, 0xF992},
{0x0F12, 0x0405},
{0x0F12, 0x0C2D},
{0x0F12, 0x4A6F},
{0x0F12, 0x0020},
{0x0F12, 0x321A},
{0x0F12, 0x0011},
{0x0F12, 0x390A},
{0x0F12, 0x2305},
{0x0F12, 0xF000},
{0x0F12, 0xF988},
{0x0F12, 0x496B},
{0x0F12, 0x3940},
{0x0F12, 0x64C8},
{0x0F12, 0x496C},
{0x0F12, 0x4E6A},
{0x0F12, 0x88C8},
{0x0F12, 0x2701},
{0x0F12, 0x3620},
{0x0F12, 0x2800},
{0x0F12, 0xD009},
{0x0F12, 0x4C69},
{0x0F12, 0x38FF},
{0x0F12, 0x1E40},
{0x0F12, 0xD00A},
{0x0F12, 0x2804},
{0x0F12, 0xD01D},
{0x0F12, 0x2806},
{0x0F12, 0xD101},
{0x0F12, 0x2000},
{0x0F12, 0x80C8},
{0x0F12, 0x82B7},
{0x0F12, 0x2001},
{0x0F12, 0xF000},
{0x0F12, 0xF988},
{0x0F12, 0xE69E},
{0x0F12, 0x000D},
{0x0F12, 0x724F},
{0x0F12, 0x2001},
{0x0F12, 0xF000},
{0x0F12, 0xF98A},
{0x0F12, 0xF000},
{0x0F12, 0xF990},
{0x0F12, 0x485B},
{0x0F12, 0x3840},
{0x0F12, 0x6C81},
{0x0F12, 0x6CC0},
{0x0F12, 0x4341},
{0x0F12, 0x0A08},
{0x0F12, 0x6160},
{0x0F12, 0x20FF},
{0x0F12, 0x1D40},
{0x0F12, 0x80E8},
{0x0F12, 0x4858},
{0x0F12, 0x3040},
{0x0F12, 0x7707},
{0x0F12, 0xE7E5},
{0x0F12, 0x4856},
{0x0F12, 0x7247},
{0x0F12, 0x21FF},
{0x0F12, 0x1DC9},
{0x0F12, 0x80C1},
{0x0F12, 0xF000},
{0x0F12, 0xF983},
{0x0F12, 0x4952},
{0x0F12, 0x3940},
{0x0F12, 0x2800},
{0x0F12, 0xD007},
{0x0F12, 0x684A},
{0x0F12, 0x0001},
{0x0F12, 0x436A},
{0x0F12, 0x0010},
{0x0F12, 0xF000},
{0x0F12, 0xF943},
{0x0F12, 0x6160},
{0x0F12, 0xE002},
{0x0F12, 0x6848},
{0x0F12, 0x4368},
{0x0F12, 0x6160},
{0x0F12, 0x8BF0},
{0x0F12, 0x2800},
{0x0F12, 0xD001},
{0x0F12, 0xF7FF},
{0x0F12, 0xFF27},
{0x0F12, 0x2000},
{0x0F12, 0xF000},
{0x0F12, 0xF95C},
{0x0F12, 0x4947},
{0x0F12, 0x20FF},
{0x0F12, 0x1DC0},
{0x0F12, 0x80C8},
{0x0F12, 0xE7C2},
{0x0F12, 0xB5F8},
{0x0F12, 0x2400},
{0x0F12, 0x4D46},
{0x0F12, 0x4846},
{0x0F12, 0x210E},
{0x0F12, 0x8041},
{0x0F12, 0x2101},
{0x0F12, 0x8001},
{0x0F12, 0xF000},
{0x0F12, 0xF965},
{0x0F12, 0x4844},
{0x0F12, 0x8840},
{0x0F12, 0xF000},
{0x0F12, 0xF89D},
{0x0F12, 0x4E3C},
{0x0F12, 0x270D},
{0x0F12, 0x073F},
{0x0F12, 0x3E40},
{0x0F12, 0x19E8},
{0x0F12, 0x8803},
{0x0F12, 0x00E2},
{0x0F12, 0x1991},
{0x0F12, 0x804B},
{0x0F12, 0x8843},
{0x0F12, 0x52B3},
{0x0F12, 0x8882},
{0x0F12, 0x80CA},
{0x0F12, 0x88C0},
{0x0F12, 0x8088},
{0x0F12, 0x3508},
{0x0F12, 0x042D},
{0x0F12, 0x0C2D},
{0x0F12, 0x1C64},
{0x0F12, 0x0424},
{0x0F12, 0x0C24},
{0x0F12, 0x2C07},
{0x0F12, 0xD3EC},
{0x0F12, 0xE640},
{0x0F12, 0xB5F0},
{0x0F12, 0xB085},
{0x0F12, 0x6801},
{0x0F12, 0x9103},
{0x0F12, 0x6881},
{0x0F12, 0x040A},
{0x0F12, 0x0C12},
{0x0F12, 0x4933},
{0x0F12, 0x8B89},
{0x0F12, 0x2900},
{0x0F12, 0xD001},
{0x0F12, 0x0011},
{0x0F12, 0xE000},
{0x0F12, 0x2100},
{0x0F12, 0x9102},
{0x0F12, 0x6840},
{0x0F12, 0x0401},
{0x0F12, 0x9803},
{0x0F12, 0x0C09},
{0x0F12, 0xF000},
{0x0F12, 0xF93C},
{0x0F12, 0x4825},
{0x0F12, 0x3040},
{0x0F12, 0x8900},
{0x0F12, 0x2800},
{0x0F12, 0xD03B},
{0x0F12, 0x2100},
{0x0F12, 0x4825},
{0x0F12, 0x4D2A},
{0x0F12, 0x30C0},
{0x0F12, 0x4684},
{0x0F12, 0x4B29},
{0x0F12, 0x4C20},
{0x0F12, 0x88DA},
{0x0F12, 0x3C40},
{0x0F12, 0x0048},
{0x0F12, 0x00D7},
{0x0F12, 0x193E},
{0x0F12, 0x197F},
{0x0F12, 0x183F},
{0x0F12, 0x5A36},
{0x0F12, 0x8AFF},
{0x0F12, 0x437E},
{0x0F12, 0x00B6},
{0x0F12, 0x0C37},
{0x0F12, 0x1906},
{0x0F12, 0x3680},
{0x0F12, 0x8177},
{0x0F12, 0x1C52},
{0x0F12, 0x00D2},
{0x0F12, 0x1914},
{0x0F12, 0x1952},
{0x0F12, 0x1812},
{0x0F12, 0x5A24},
{0x0F12, 0x8AD2},
{0x0F12, 0x4354},
{0x0F12, 0x00A2},
{0x0F12, 0x0C12},
{0x0F12, 0x8272},
{0x0F12, 0x891C},
{0x0F12, 0x895B},
{0x0F12, 0x4367},
{0x0F12, 0x435A},
{0x0F12, 0x1943},
{0x0F12, 0x3340},
{0x0F12, 0x89DB},
{0x0F12, 0x9C02},
{0x0F12, 0x18BA},
{0x0F12, 0x4363},
{0x0F12, 0x18D2},
{0x0F12, 0x0212},
{0x0F12, 0x0C12},
{0x0F12, 0x466B},
{0x0F12, 0x521A},
{0x0F12, 0x4663},
{0x0F12, 0x7DDB},
{0x0F12, 0x435A},
{0x0F12, 0x9B03},
{0x0F12, 0x0252},
{0x0F12, 0x0C12},
{0x0F12, 0x521A},
{0x0F12, 0x1C49},
{0x0F12, 0x0409},
{0x0F12, 0x0C09},
{0x0F12, 0x2904},
{0x0F12, 0xD3C8},
{0x0F12, 0xB005},
{0x0F12, 0xBCF0},
{0x0F12, 0xBC08},
{0x0F12, 0x4718},
{0x0F12, 0xB510},
{0x0F12, 0xF7FF},
{0x0F12, 0xFF7D},
{0x0F12, 0xF000},
{0x0F12, 0xF8FA},
{0x0F12, 0xE502},
{0x0F12, 0x0000},
{0x0F12, 0x3F88},
{0x0F12, 0x7000},
{0x0F12, 0x2A24},
{0x0F12, 0x7000},
{0x0F12, 0x31A0},
{0x0F12, 0x7000},
{0x0F12, 0x2A64},
{0x0F12, 0x7000},
{0x0F12, 0xA006},
{0x0F12, 0x0000},
{0x0F12, 0xA000},
{0x0F12, 0xD000},
{0x0F12, 0x064C},
{0x0F12, 0x7000},
{0x0F12, 0x07C4},
{0x0F12, 0x7000},
{0x0F12, 0x07E8},
{0x0F12, 0x7000},
{0x0F12, 0x1FA0},
{0x0F12, 0x7000},
{0x0F12, 0x4778},
{0x0F12, 0x46C0},
{0x0F12, 0xC000},
{0x0F12, 0xE59F},
{0x0F12, 0xFF1C},
{0x0F12, 0xE12F},
{0x0F12, 0x1F63},
{0x0F12, 0x0001},
{0x0F12, 0x4778},
{0x0F12, 0x46C0},
{0x0F12, 0xC000},
{0x0F12, 0xE59F},
{0x0F12, 0xFF1C},
{0x0F12, 0xE12F},
{0x0F12, 0x1EDF},
{0x0F12, 0x0001},
{0x0F12, 0x4778},
{0x0F12, 0x46C0},
{0x0F12, 0xC000},
{0x0F12, 0xE59F},
{0x0F12, 0xFF1C},
{0x0F12, 0xE12F},
{0x0F12, 0xFDAF},
{0x0F12, 0x0000},
{0x0F12, 0x4778},
{0x0F12, 0x46C0},
{0x0F12, 0xF004},
{0x0F12, 0xE51F},
{0x0F12, 0x2328},
{0x0F12, 0x0001},
{0x0F12, 0x4778},
{0x0F12, 0x46C0},
{0x0F12, 0xC000},
{0x0F12, 0xE59F},
{0x0F12, 0xFF1C},
{0x0F12, 0xE12F},
{0x0F12, 0x9E89},
{0x0F12, 0x0000},
{0x0F12, 0x4778},
{0x0F12, 0x46C0},
{0x0F12, 0xC000},
{0x0F12, 0xE59F},
{0x0F12, 0xFF1C},
{0x0F12, 0xE12F},
{0x0F12, 0x495F},
{0x0F12, 0x0000},
{0x0F12, 0x4778},
{0x0F12, 0x46C0},
{0x0F12, 0xC000},
{0x0F12, 0xE59F},
{0x0F12, 0xFF1C},
{0x0F12, 0xE12F},
{0x0F12, 0xE403},
{0x0F12, 0x0000},
{0x0F12, 0x4778},
{0x0F12, 0x46C0},
{0x0F12, 0xC000},
{0x0F12, 0xE59F},
{0x0F12, 0xFF1C},
{0x0F12, 0xE12F},
{0x0F12, 0x24B3},
{0x0F12, 0x0001},
{0x0F12, 0x4778},
{0x0F12, 0x46C0},
{0x0F12, 0xC000},
{0x0F12, 0xE59F},
{0x0F12, 0xFF1C},
{0x0F12, 0xE12F},
{0x0F12, 0xEECD},
{0x0F12, 0x0000},
{0x0F12, 0x4778},
{0x0F12, 0x46C0},
{0x0F12, 0xC000},
{0x0F12, 0xE59F},
{0x0F12, 0xFF1C},
{0x0F12, 0xE12F},
{0x0F12, 0xF049},
{0x0F12, 0x0000},
{0x0F12, 0x4778},
{0x0F12, 0x46C0},
{0x0F12, 0xC000},
{0x0F12, 0xE59F},
{0x0F12, 0xFF1C},
{0x0F12, 0xE12F},
{0x0F12, 0x12DF},
{0x0F12, 0x0000},
{0x0F12, 0x4778},
{0x0F12, 0x46C0},
{0x0F12, 0xC000},
{0x0F12, 0xE59F},
{0x0F12, 0xFF1C},
{0x0F12, 0xE12F},
{0x0F12, 0xF05B},
{0x0F12, 0x0000},
{0x0F12, 0x4778},
{0x0F12, 0x46C0},
{0x0F12, 0xC000},
{0x0F12, 0xE59F},
{0x0F12, 0xFF1C},
{0x0F12, 0xE12F},
{0x0F12, 0xF07B},
{0x0F12, 0x0000},
{0x0F12, 0x4778},
{0x0F12, 0x46C0},
{0x0F12, 0xC000},
{0x0F12, 0xE59F},
{0x0F12, 0xFF1C},
{0x0F12, 0xE12F},
{0x0F12, 0xFE6D},
{0x0F12, 0x0000},
{0x0F12, 0x4778},
{0x0F12, 0x46C0},
{0x0F12, 0xC000},
{0x0F12, 0xE59F},
{0x0F12, 0xFF1C},
{0x0F12, 0xE12F},
{0x0F12, 0x3295},
{0x0F12, 0x0000},
{0x0F12, 0x4778},
{0x0F12, 0x46C0},
{0x0F12, 0xC000},
{0x0F12, 0xE59F},
{0x0F12, 0xFF1C},
{0x0F12, 0xE12F},
{0x0F12, 0x234F},
{0x0F12, 0x0000},
{0x0F12, 0x4778},
{0x0F12, 0x46C0},
{0x0F12, 0xC000},
{0x0F12, 0xE59F},
{0x0F12, 0xFF1C},
{0x0F12, 0xE12F},
{0x0F12, 0x4521},
{0x0F12, 0x0000},
{0x0F12, 0x4778},
{0x0F12, 0x46C0},
{0x0F12, 0xC000},
{0x0F12, 0xE59F},
{0x0F12, 0xFF1C},
{0x0F12, 0xE12F},
{0x0F12, 0x7C0D},
{0x0F12, 0x0000},
{0x0F12, 0x4778},
{0x0F12, 0x46C0},
{0x0F12, 0xC000},
{0x0F12, 0xE59F},
{0x0F12, 0xFF1C},
{0x0F12, 0xE12F},
{0x0F12, 0x7C2B},
{0x0F12, 0x0000},
{0x0F12, 0x4778},
{0x0F12, 0x46C0},
{0x0F12, 0xF004},
{0x0F12, 0xE51F},
{0x0F12, 0x24C4},
{0x0F12, 0x0001},
{0x0F12, 0x4778},
{0x0F12, 0x46C0},
{0x0F12, 0xC000},
{0x0F12, 0xE59F},
{0x0F12, 0xFF1C},
{0x0F12, 0xE12F},
{0x0F12, 0x3183},
{0x0F12, 0x0000},
{0x0F12, 0x4778},
{0x0F12, 0x46C0},
{0x0F12, 0xC000},
{0x0F12, 0xE59F},
{0x0F12, 0xFF1C},
{0x0F12, 0xE12F},
{0x0F12, 0x302F},
{0x0F12, 0x0000},
{0x0F12, 0x4778},
{0x0F12, 0x46C0},
{0x0F12, 0xC000},
{0x0F12, 0xE59F},
{0x0F12, 0xFF1C},
{0x0F12, 0xE12F},
{0x0F12, 0xEF07},
{0x0F12, 0x0000},
{0x0F12, 0x4778},
{0x0F12, 0x46C0},
{0x0F12, 0xC000},
{0x0F12, 0xE59F},
{0x0F12, 0xFF1C},
{0x0F12, 0xE12F},
{0x0F12, 0x48FB},
{0x0F12, 0x0000},
{0x0F12, 0x4778},
{0x0F12, 0x46C0},
{0x0F12, 0xC000},
{0x0F12, 0xE59F},
{0x0F12, 0xFF1C},
{0x0F12, 0xE12F},
{0x0F12, 0xF0B1},
{0x0F12, 0x0000},
{0x0F12, 0x4778},
{0x0F12, 0x46C0},
{0x0F12, 0xC000},
{0x0F12, 0xE59F},
{0x0F12, 0xFF1C},
{0x0F12, 0xE12F},
{0x0F12, 0xEEDF},
{0x0F12, 0x0000},
{0x0F12, 0x4778},
{0x0F12, 0x46C0},
{0x0F12, 0xC000},
{0x0F12, 0xE59F},
{0x0F12, 0xFF1C},
{0x0F12, 0xE12F},
{0x0F12, 0xAEF1},
{0x0F12, 0x0000},
{0x0F12, 0x4778},
{0x0F12, 0x46C0},
{0x0F12, 0xC000},
{0x0F12, 0xE59F},
{0x0F12, 0xFF1C},
{0x0F12, 0xE12F},
{0x0F12, 0xFD21},
{0x0F12, 0x0000},
{0x0F12, 0x4778},
{0x0F12, 0x46C0},
{0x0F12, 0xC000},
{0x0F12, 0xE59F},
{0x0F12, 0xFF1C},
{0x0F12, 0xE12F},
{0x0F12, 0x5027},
{0x0F12, 0x0000},
{0x0F12, 0x4778},
{0x0F12, 0x46C0},
{0x0F12, 0xC000},
{0x0F12, 0xE59F},
{0x0F12, 0xFF1C},
{0x0F12, 0xE12F},
{0x0F12, 0x04C9},
{0x0F12, 0x0000},
{0x1000, 0x0001},

{0x0028, 0xD000},
{0x002A, 0x0070},
{0x0F12, 0x0007},

{0x0028, 0x7000},
{0x002A, 0x04B4},
{0x0F12, 0x0064},

{0x0028, 0x7000},
{0x002A, 0x3302},
{0x0F12, 0x0000},

{0x0F12, 0x0005},
{0x0F12, 0x0019},
{0x0F12, 0x0050},
{0x0F12, 0x0300},
{0x0F12, 0x0375},

{0x002A, 0x3F82},
{0x0F12, 0x0000},
{0x0F12, 0x0000},
{0x0F12, 0x0260},
{0x0F12, 0x0240},

{0x002A, 0x3F98},
{0x0F12, 0x0100},
{0x0F12, 0x0150},
{0x0F12, 0x0200},
{0x0F12, 0x0300},
{0x0F12, 0x0400},

{0x0F12, 0x0100},
{0x0F12, 0x00A0},
{0x0F12, 0x0080},
{0x0F12, 0x0040},
{0x0F12, 0x0020},

{0x0F12, 0x0030},
{0x0F12, 0x0040},
{0x0F12, 0x0048},
{0x0F12, 0x0050},
{0x0F12, 0x0060},

{0x0F12, 0x0100},
{0x0F12, 0x00C0},
{0x0F12, 0x0080},
{0x0F12, 0x000A},
{0x0F12, 0x0000},

{0x0F12, 0x0120},
{0x0F12, 0x0150},
{0x0F12, 0x0200},
 
{0x0F12, 0x003C},
{0x0F12, 0x003B},
{0x0F12, 0x0030},
 
{0x002A, 0x0430},
{0x0F12, 0x0002},
{0x002A, 0x3F80},
{0x0F12, 0x0000},

{0x002A, 0x165E},
{0x0F12, 0x0240},
{0x0F12, 0x0244},



{0x0028, 0xD000},
{0x002A, 0xF2AC},
{0x0F12, 0x0100},
{0x002A, 0xF400},
{0x0F12, 0x001D},
{0x0F12, 0x3F02},

{0x002A, 0xF40A},
{0x0F12, 0x0054},
{0x0F12, 0x0002},
{0x0F12, 0x0008},
{0x0F12, 0x0000},
{0x0F12, 0x00A4},

{0x002A, 0xF416},
{0x0F12, 0x0001},

{0x002A, 0xF41E},
{0x0F12, 0x0065},

{0x002A, 0xF422},
{0x0F12, 0x0005},

{0x002A, 0xF426},
{0x0F12, 0x00D4},

{0x002A, 0xF42A},
{0x0F12, 0x0001},

{0x002A, 0xF42E},
{0x0F12, 0x0406},

{0x002A, 0xF434},
{0x0F12, 0x0003},
{0x0F12, 0x0004},
{0x0F12, 0x0002},
{0x0F12, 0x0001},
{0x0F12, 0x0004},

{0x002A, 0xF446},
{0x0F12, 0x0000},

{0x002A, 0xF466},
{0x0F12, 0x0000},

{0x002A, 0x0054},
{0x0F12, 0x0028},
{0x0F12, 0x8888},

{0x002A, 0xF132},
{0x0F12, 0x0206},
{0x002A, 0xF152},
{0x0F12, 0x0206},
{0x002A, 0xF1A2},
{0x0F12, 0x0200},
{0x002A, 0xF1B2},
{0x0F12, 0x0202},


{0x002A, 0xE42E},
{0x0F12, 0x0004},


{0x0028, 0x7000},
{0x002A, 0x1014},
{0x0F12, 0x0132},
{0x0F12, 0x010A},


{0x002A, 0x01A2},
{0x0F12, 0x0003},
{0x0F12, 0x0000},
{0x0F12, 0x0000},
{0x0F12, 0x0041},
{0x0F12, 0x0000},
{0x0F12, 0x2A0C},
{0x0F12, 0x0190},


{0x002A, 0x022C},
{0x0F12, 0x0100},
{0x0F12, 0x00E3},
{0x0F12, 0x0200},
{0x0F12, 0x0238},
{0x0F12, 0x018C},
{0x0F12, 0x0166},
{0x0F12, 0x00E6},
{0x0F12, 0x0132},
{0x0F12, 0x0001},


{0x002A, 0x063A},
{0x0F12, 0x00C0},
{0x002A, 0x064A},
{0x0F12, 0x0000},
{0x002A, 0x1488},
{0x0F12, 0x0000},
{0x002A, 0x1494},
{0x0F12, 0x1002},
{0x002A, 0x149E},
{0x0F12, 0x0003},
{0x0F12, 0x0000},
{0x002A, 0x142C},
{0x0F12, 0x0602},
{0x002A, 0x14A2},
{0x0F12, 0x0000},


{0x002A, 0x1498},
{0x0F12, 0x0003},
{0x002A, 0x148A},
{0x0F12, 0x00CC},
{0x0F12, 0x00A0},


{0x002A, 0x1420},
{0x0F12, 0x0000},
{0x0F12, 0x952F},


{0x002A, 0x14B4},
{0x0F12, 0x0280},
{0x002A, 0x14C0},
{0x0F12, 0x03A0},
{0x0F12, 0x0320},
{0x002A, 0x14F4},
{0x0F12, 0x0030},
{0x002A, 0x1514},
{0x0F12, 0x0060},

{0x002A, 0x151E},
{0x0F12, 0x0003},


{0x002A, 0x1434},
{0x0F12, 0x0010},

{0x0F12, 0x0030},
{0x0F12, 0x0033},
{0x0F12, 0x0036},
{0x0F12, 0x0039},
{0x0F12, 0x003D},
{0x0F12, 0x0041},
{0x0F12, 0x0045},
{0x0F12, 0x0049},
{0x0F12, 0x004E},
{0x0F12, 0x0053},
{0x0F12, 0x0059},
{0x0F12, 0x0060},
{0x0F12, 0x0068},
{0x0F12, 0x0072},
{0x0F12, 0x007D},
{0x0F12, 0x0089},
{0x0F12, 0x0096},

{0x002A, 0x1558},
{0x0F12, 0x8000},
{0x0F12, 0x0006},
{0x0F12, 0x3FF0},
{0x0F12, 0x03E8},
{0x0F12, 0x0000},
{0x0F12, 0x0020},
{0x0F12, 0x0008},
{0x0F12, 0x0008},
{0x0F12, 0x0040},
{0x0F12, 0x0080},
{0x0F12, 0x00C0},
{0x0F12, 0x00E0},

{0x002A, 0x0224},
{0x0F12, 0x0003},


{0x002A, 0x018C},
{0x0F12, 0x0001},
{0x0F12, 0x0003},
{0x0F12, 0x0003},


{0x002A, 0x1686},
{0x0F12, 0x005C},
{0x0F12, 0x005C},
{0x0F12, 0x085C},
{0x0F12, 0x005C},
{0x0F12, 0x025A},


{0x002A, 0x1844},
{0x0F12, 0x0000},

{0x002A, 0x1680},
{0x0F12, 0x0002},


{0x002A, 0x18F8},
{0x0F12, 0x0001},
{0x002A, 0x18F6},
{0x0F12, 0x0001},
{0x002A, 0x182C},
{0x0F12, 0x0001},
{0x002A, 0x0EE4},
{0x0F12, 0x0001},
{0x002A, 0x1674},
{0x0F12, 0x0002},
{0x0F12, 0x0002},
{0x0F12, 0x0000},


{0x002A, 0x19AE},
{0x0F12, 0xEA60},
{0x0F12, 0x7530},
{0x002A, 0x19C2},
{0x0F12, 0x7530},
{0x002A, 0x0244},
{0x0F12, 0x7530},
{0x002A, 0x0336},
{0x0F12, 0x7530},


{0x002A, 0x0188},
{0x0F12, 0x6590},
{0x0F12, 0x0000},
{0x002A, 0x01B2},
{0x0F12, 0x0000},
{0x0F12, 0x0001},
{0x002A, 0x01B8},
{0x0F12, 0x0000},


{0x0F12, 0x32C8},
{0x0F12, 0x36A4},
{0x0F12, 0x3AA4},


{0x0F12, 0x1F40},
{0x0F12, 0x36A4},
{0x0F12, 0x3AA4},

{0x002A, 0x01CC},
{0x0F12, 0x0001},


{0x002A, 0x0428},

{0x0F12, 0x0000},
{0x0F12, 0x0140},
{0x0F12, 0x00F0},
{0x0F12, 0x0005},


{0x002A, 0x01F6},
{0x0F12, 0x0800},
{0x0F12, 0x0600},
{0x0F12, 0x0000},
{0x0F12, 0x0000},
{0x0F12, 0x0800},
{0x0F12, 0x0600},
{0x0F12, 0x0000},
{0x0F12, 0x0000},

{0x002A, 0x0216},
{0x0F12, 0x0001},
{0x0F12, 0x0001},

{0x002A, 0x043C},
{0x0F12, 0x0800},
{0x0F12, 0x0600},
{0x0F12, 0x0000},
{0x0F12, 0x0000},


{0x002A, 0x023E},
{0x0F12, 0x0280},
{0x0F12, 0x01E0},
{0x0F12, 0x0005},
{0x0F12, 0x3AA4},
{0x0F12, 0x36A4},

{0x002A, 0x024C},
{0x0F12, 0x0012},
{0x0F12, 0x0010},

{0x002A, 0x0254},
{0x0F12, 0x0000},
{0x0F12, 0x0000},
{0x0F12, 0x0001},
{0x0F12, 0x03E8},
{0x0F12, 0x0154},

{0x0F12, 0x0000},
{0x0F12, 0x0000},
{0x0F12, 0x0000},
{0x0F12, 0x0000},
{0x0F12, 0x0000},
{0x0F12, 0x0000},
{0x0F12, 0x0000},
{0x0F12, 0x0000},


{0x002A, 0x032E},
{0x0F12, 0x0000},

{0x0F12, 0x0800},
{0x0F12, 0x0600},
{0x0F12, 0x0009},
{0x0F12, 0x3AA4},
{0x0F12, 0x36A4},

{0x002A, 0x033E},
{0x0F12, 0x0012},
{0x0F12, 0x0010},


{0x002A, 0x0346},
{0x0F12, 0x0000},
{0x0F12, 0x0002},
{0x0F12, 0x0002},
{0x0F12, 0x0535},
{0x0F12, 0x029A},
{0x0F12, 0x0000},
{0x0F12, 0x0000},
{0x0F12, 0x0000},
{0x0F12, 0x0000},
{0x0F12, 0x0000},

{0x002A, 0x0426},
{0x0F12, 0x005F},

{0x002A, 0x0208},
{0x0F12, 0x0000},
{0x002A, 0x0210},
{0x0F12, 0x0000},
{0x002A, 0x020C},
{0x0F12, 0x0001},
{0x002A, 0x01F4},
{0x0F12, 0x0001},
{0x002A, 0x020A},
{0x0F12, 0x0001},
{0x002A, 0x0212},
{0x0F12, 0x0001},
{0x002A, 0x01E8},
{0x0F12, 0x0000},
{0x0F12, 0x0001},

{0xFFFF, 0x0064},

{0x002A, 0x0F08},
{0x0F12, 0x0001},   // changed 0x0000 -> 0x0001 for 60hz flicker 
{0x002A, 0x04A4},
{0x0F12, 0x067F},

{0x002A, 0x0D22},
{0x0F12, 0x0F00},
{0x0F12, 0x000F},
{0x0F12, 0x000F},
{0x0F12, 0x0F0F},
{0x0F12, 0x0F00},
{0x0F12, 0x000F},
{0x0F12, 0x000F},
{0x0F12, 0x000F},
{0x0F12, 0x0F00},
{0x0F12, 0x0F00},
{0x0F12, 0x0F00},
{0x0F12, 0x000F},
{0x0F12, 0x000F},
{0x0F12, 0x000F},
{0x0F12, 0x0F00},
{0x0F12, 0x0000},
{0x0F12, 0x0F00},
{0x0F12, 0x0F00},
{0x0F12, 0x0F00},
{0x0F12, 0x000F},
{0x0F12, 0x000F},
{0x0F12, 0x0F0F},
{0x0F12, 0x0F00},
{0x0F12, 0x0000},
{0x0F12, 0x000F},
{0x0F12, 0x000F},
{0x0F12, 0x0F0F},
{0x0F12, 0x0F00},
{0x0F12, 0x0F0F},
{0x0F12, 0x0F00},
{0x0F12, 0x0F0F},
{0x0F12, 0x0000},
{0x0F12, 0x000F},
{0x0F12, 0x000F},
{0x0F12, 0x000F},
{0x0F12, 0x000F},
{0x0F12, 0x0F00},
{0x0F12, 0x000F},
{0x0F12, 0x000F},
{0x0F12, 0x0F0F},
{0x0F12, 0x0F00},
{0x0F12, 0x0000},
{0x0F12, 0x000F},
{0x0F12, 0x000F},
{0x0F12, 0x0F00},
{0x0F12, 0x0F00},
{0x0F12, 0x0F00},
{0x0F12, 0x000F},
{0x0F12, 0x0F0F},
{0x0F12, 0x0000},
{0x0F12, 0x0F0F},
{0x0F12, 0x0000},
{0x0F12, 0x000F},
{0x0F12, 0x0F00},
{0x0F12, 0x0F00},
{0x0F12, 0x000F},
{0x0F12, 0x000F},
{0x0F12, 0x0F0F},
{0x0F12, 0x0F00},
{0x0F12, 0x000F},
{0x0F12, 0x0000},
{0x0F12, 0x000F},
{0x0F12, 0x0F0F},
{0x0F12, 0x0F00},
{0x0F12, 0x0F00},
{0x0F12, 0x0000},
{0x0F12, 0x0F0F},
{0x0F12, 0x000F},
{0x0F12, 0x0F0F},
{0x0F12, 0x0000},
{0x0F12, 0x000F},
{0x0F12, 0x0F0F},

{0x0F12, 0x6E49},
{0x0F12, 0xFB98},
{0x0F12, 0xF348},
{0x0F12, 0x1BD6},
{0x0F12, 0xEBEF},
{0x0F12, 0x03D3},
{0x0F12, 0xEC8D},
{0x0F12, 0xF239},
{0x0F12, 0x0E64},
{0x0F12, 0xF7EA},
{0x0F12, 0xFD3B},
{0x0F12, 0x0A7C},
{0x0F12, 0xFC9C},
{0x0F12, 0x0BD3},
{0x0F12, 0xF2E5},
{0x0F12, 0x0619},
{0x0F12, 0x0772},
{0x0F12, 0xF0B0},
{0x0F12, 0x184E},
{0x0F12, 0xF95F},
{0x0F12, 0x0B1A},
{0x0F12, 0xFC45},
{0x0F12, 0xF716},
{0x0F12, 0x0DCD},
{0x0F12, 0xEF24},
{0x0F12, 0x0221},
{0x0F12, 0xF6BD},
{0x0F12, 0x04CB},
{0x0F12, 0x00B1},
{0x0F12, 0xFEB0},
{0x0F12, 0x0268},
{0x0F12, 0x02C7},
{0x0F12, 0x010A},
{0x0F12, 0xFF93},
{0x0F12, 0x036D},
{0x0F12, 0xF859},
{0x0F12, 0x81D0},
{0x0F12, 0xFA32},
{0x0F12, 0xEFDB},
{0x0F12, 0x234D},
{0x0F12, 0xE799},
{0x0F12, 0x0337},
{0x0F12, 0xEB05},
{0x0F12, 0xE8F9},
{0x0F12, 0x152E},
{0x0F12, 0xF0D5},
{0x0F12, 0x0842},
{0x0F12, 0x043A},
{0x0F12, 0xF461},
{0x0F12, 0x0E58},
{0x0F12, 0xF658},
{0x0F12, 0x075D},
{0x0F12, 0xF78D},
{0x0F12, 0xFDE9},
{0x0F12, 0x277A},
{0x0F12, 0xFFDE},
{0x0F12, 0xFD3B},
{0x0F12, 0xFE50},
{0x0F12, 0x0AD1},
{0x0F12, 0xFE2C},
{0x0F12, 0xE90D},
{0x0F12, 0xF7B0},
{0x0F12, 0x05DB},
{0x0F12, 0x02CD},
{0x0F12, 0xF4F1},
{0x0F12, 0x02A8},
{0x0F12, 0xFDDC},
{0x0F12, 0x0B59},
{0x0F12, 0xF74E},
{0x0F12, 0x03D5},
{0x0F12, 0xFF4F},
{0x0F12, 0x00F7},
{0x0F12, 0x6A44},
{0x0F12, 0xFAD6},
{0x0F12, 0xF261},
{0x0F12, 0x1F28},
{0x0F12, 0xE691},
{0x0F12, 0x07D2},
{0x0F12, 0xEE85},
{0x0F12, 0xF426},
{0x0F12, 0x0F26},
{0x0F12, 0xF34B},
{0x0F12, 0x0036},
{0x0F12, 0x0C0F},
{0x0F12, 0xFDA9},
{0x0F12, 0x09EA},
{0x0F12, 0xF27A},
{0x0F12, 0x0CD5},
{0x0F12, 0x01E1},
{0x0F12, 0xED41},
{0x0F12, 0x1DB5},
{0x0F12, 0xFD26},
{0x0F12, 0x03F7},
{0x0F12, 0xF7BB},
{0x0F12, 0xFE81},
{0x0F12, 0x12D3},
{0x0F12, 0xE061},
{0x0F12, 0xF81C},
{0x0F12, 0x07B1},
{0x0F12, 0x0408},
{0x0F12, 0xF860},
{0x0F12, 0xFC9A},
{0x0F12, 0x0DDE},
{0x0F12, 0x0C9C},
{0x0F12, 0xF2A4},
{0x0F12, 0x02EB},
{0x0F12, 0x099B},
{0x0F12, 0xF5A6},
{0x0F12, 0x7243},
{0x0F12, 0xF74D},
{0x0F12, 0xF74B},
{0x0F12, 0x1800},
{0x0F12, 0xEF22},
{0x0F12, 0x0263},
{0x0F12, 0xEBE7},
{0x0F12, 0xF5A4},
{0x0F12, 0x09D3},
{0x0F12, 0xFAB8},
{0x0F12, 0xFDFF},
{0x0F12, 0x086B},
{0x0F12, 0x0338},
{0x0F12, 0x0514},
{0x0F12, 0xF840},
{0x0F12, 0x0768},
{0x0F12, 0xFE55},
{0x0F12, 0xF884},
{0x0F12, 0x1488},
{0x0F12, 0xFFCD},
{0x0F12, 0x035B},
{0x0F12, 0xFA4E},
{0x0F12, 0x01DB},
{0x0F12, 0x06D6},
{0x0F12, 0xEE19},
{0x0F12, 0xFEA3},
{0x0F12, 0xFE8C},
{0x0F12, 0x03A3},
{0x0F12, 0xFDDB},
{0x0F12, 0xFD9B},
{0x0F12, 0x035E},
{0x0F12, 0x03F2},
{0x0F12, 0xFCBD},
{0x0F12, 0x0300},
{0x0F12, 0xFF2E},
{0x0F12, 0xFE03},

{0x002A, 0x04A8},
{0x0F12, 0x0001},


{0x002A, 0x07E8},
{0x0F12, 0x00BC},
{0x0F12, 0x00ED},
{0x0F12, 0x0101},
{0x0F12, 0x012D},
{0x0F12, 0x0166},
{0x0F12, 0x0184},
{0x0F12, 0x01A0},
{0x002A, 0x07FE},
{0x0F12, 0x3200},
{0x0F12, 0x4000},
{0x0F12, 0x4000},
{0x0F12, 0x3C00},
{0x0F12, 0x3200},
{0x0F12, 0x4000},
{0x0F12, 0x4000},
{0x0F12, 0x3C00},
{0x0F12, 0x3200},
{0x0F12, 0x4000},
{0x0F12, 0x4000},
{0x0F12, 0x3C00},
{0x0F12, 0x3200},
{0x0F12, 0x4000},
{0x0F12, 0x4000},
{0x0F12, 0x3C00},
{0x0F12, 0x3200},
{0x0F12, 0x4000},
{0x0F12, 0x4000},
{0x0F12, 0x3C00},
{0x0F12, 0x3200},
{0x0F12, 0x4000},
{0x0F12, 0x4000},
{0x0F12, 0x3C00},
{0x0F12, 0x4000},
{0x0F12, 0x4000},
{0x0F12, 0x4000},
{0x0F12, 0x3C00},

{0x002A, 0x0836},
{0x0F12, 0x3E00},
{0x0F12, 0x4000},
{0x0F12, 0x4000},
{0x0F12, 0x4000},

{0x002A, 0x0660},
{0x0F12, 0x0000},
{0x0F12, 0x0008},
{0x0F12, 0x0015},
{0x0F12, 0x0032},
{0x0F12, 0x006C},
{0x0F12, 0x00D0},
{0x0F12, 0x0129},
{0x0F12, 0x0151},
{0x0F12, 0x0174},
{0x0F12, 0x01AA},
{0x0F12, 0x01D7},
{0x0F12, 0x01FE},
{0x0F12, 0x0221},
{0x0F12, 0x0252},
{0x0F12, 0x0281},
{0x0F12, 0x02E1},
{0x0F12, 0x0345},
{0x0F12, 0x039C},
{0x0F12, 0x03D9},
{0x0F12, 0x03FF},
{0x0F12, 0x0000},
{0x0F12, 0x0008},
{0x0F12, 0x0015},
{0x0F12, 0x0032},
{0x0F12, 0x006C},
{0x0F12, 0x00D0},
{0x0F12, 0x0129},
{0x0F12, 0x0151},
{0x0F12, 0x0174},
{0x0F12, 0x01AA},
{0x0F12, 0x01D7},
{0x0F12, 0x01FE},
{0x0F12, 0x0221},
{0x0F12, 0x0252},
{0x0F12, 0x0281},
{0x0F12, 0x02E1},
{0x0F12, 0x0345},
{0x0F12, 0x039C},
{0x0F12, 0x03D9},
{0x0F12, 0x03FF},
{0x0F12, 0x0000},
{0x0F12, 0x0008},
{0x0F12, 0x0015},
{0x0F12, 0x0032},
{0x0F12, 0x006C},
{0x0F12, 0x00D0},
{0x0F12, 0x0129},
{0x0F12, 0x0151},
{0x0F12, 0x0174},
{0x0F12, 0x01AA},
{0x0F12, 0x01D7},
{0x0F12, 0x01FE},
{0x0F12, 0x0221},
{0x0F12, 0x0252},
{0x0F12, 0x0281},
{0x0F12, 0x02E1},
{0x0F12, 0x0345},
{0x0F12, 0x039C},
{0x0F12, 0x03D9},
{0x0F12, 0x03FF},

{0x0F12, 0x0000},	
{0x0F12, 0x0008},	
{0x0F12, 0x0013},	
{0x0F12, 0x002C},	
{0x0F12, 0x005C},	
{0x0F12, 0x00BB},	
{0x0F12, 0x0109},	
{0x0F12, 0x012C},	
{0x0F12, 0x014C},	
{0x0F12, 0x0185},	
{0x0F12, 0x01B9},	
{0x0F12, 0x01E8},	
{0x0F12, 0x0210},	
{0x0F12, 0x024C},	
{0x0F12, 0x0280},	
{0x0F12, 0x02D4},	
{0x0F12, 0x0324},	
{0x0F12, 0x036C},	
{0x0F12, 0x03AC},	
{0x0F12, 0x03E6},	
{0x0F12, 0x0000},	
{0x0F12, 0x0008},	
{0x0F12, 0x0013},	
{0x0F12, 0x002C},	
{0x0F12, 0x005C},	
{0x0F12, 0x00BB},	
{0x0F12, 0x0109},	
{0x0F12, 0x012C},	
{0x0F12, 0x014C},	
{0x0F12, 0x0185},	
{0x0F12, 0x01B9},	
{0x0F12, 0x01E8},	
{0x0F12, 0x0210},	
{0x0F12, 0x024C},	
{0x0F12, 0x0280},	
{0x0F12, 0x02D4},	
{0x0F12, 0x0324},	
{0x0F12, 0x036C},	
{0x0F12, 0x03AC},	
{0x0F12, 0x03E6},	
{0x0F12, 0x0000},	
{0x0F12, 0x0008},	
{0x0F12, 0x0013},	
{0x0F12, 0x002C},	
{0x0F12, 0x005C},	
{0x0F12, 0x00BB},	
{0x0F12, 0x0109},	
{0x0F12, 0x012C},	
{0x0F12, 0x014C},	
{0x0F12, 0x0185},	
{0x0F12, 0x01B9},	
{0x0F12, 0x01E8},	
{0x0F12, 0x0210},	
{0x0F12, 0x024C},	
{0x0F12, 0x0280},	
{0x0F12, 0x02D4},	
{0x0F12, 0x0324},	
{0x0F12, 0x036C},	
{0x0F12, 0x03AC},	
{0x0F12, 0x03E6},	

{0x002A, 0x01DE},
{0x0F12, 0x0000},

{0x002A, 0x1308},
{0x0F12, 0x003E},
{0x002A, 0x130E},
{0x0F12, 0x000F},

{0x002A, 0x04EE},
{0x0F12, 0x010E},
{0x0F12, 0x00F5},

{0x002A, 0x0504},
{0x0F12, 0x3415},
{0x002A, 0x0508},
{0x0F12, 0x681F},
{0x002A, 0x050C},
{0x0F12, 0x8227},
{0x002A, 0x0510},
{0x0F12, 0xC350},

{0x002A, 0x0514},
{0x0F12, 0x3415},
{0x002A, 0x0518},
{0x0F12, 0x681F},
{0x002A, 0x051C},
{0x0F12, 0x8227},
{0x002A, 0x0520},
{0x0F12, 0xC350},

{0x002A, 0x0524},
{0x0F12, 0x01D0},
{0x0F12, 0x01D0},
{0x0F12, 0x02C0},
{0x0F12, 0x0710},

{0x0F12, 0x0100},
{0x0F12, 0x8000},

{0x0F12, 0x01D0},
{0x0F12, 0x01D0},
{0x0F12, 0x02C0},
{0x0F12, 0x0710},

{0x0F12, 0x0100},
{0x0F12, 0x8000},

{0x002A, 0x1316},
{0x0F12, 0x0000},
{0x0F12, 0x0000},
{0x0F12, 0x0000},
{0x0F12, 0x0000},
{0x0F12, 0x0101},
{0x0F12, 0x0101},
{0x0F12, 0x0101},
{0x0F12, 0x0101},
{0x0F12, 0x0101},
{0x0F12, 0x0201},
{0x0F12, 0x0102},
{0x0F12, 0x0101},
{0x0F12, 0x0101},
{0x0F12, 0x0202},
{0x0F12, 0x0202},
{0x0F12, 0x0101},
{0x0F12, 0x0101},
{0x0F12, 0x0202},
{0x0F12, 0x0202},
{0x0F12, 0x0101},
{0x0F12, 0x0201},
{0x0F12, 0x0202},
{0x0F12, 0x0202},
{0x0F12, 0x0102},
{0x0F12, 0x0201},
{0x0F12, 0x0202},
{0x0F12, 0x0202},
{0x0F12, 0x0102},
{0x0F12, 0x0101},
{0x0F12, 0x0101},
{0x0F12, 0x0101},
{0x0F12, 0x0101},

{0x002A, 0x1018},
{0x0F12, 0x02A7},
{0x0F12, 0x0343},
{0x002A, 0x0FFC},
{0x0F12, 0x036C},
{0x002A, 0x1000},
{0x0F12, 0x011D},
{0x002A, 0x1004},
{0x0F12, 0x62C1},
{0x002A, 0x1034},
{0x0F12, 0x05F0},
{0x0F12, 0x01F4},
{0x0F12, 0x006C},
{0x0F12, 0x0038},
{0x002A, 0x1020},
{0x0F12, 0x000C},
{0x0F12, 0x001E},
{0x0F12, 0x0046},
{0x002A, 0x291A},
{0x0F12, 0x0006},

{0x002A, 0x11C2},
{0x0F12, 0x0000},
{0x0F12, 0x0000},
{0x0F12, 0x0000},
{0x0F12, 0x00C2},
{0x0F12, 0x0002},
{0x0F12, 0x0001},
{0x0F12, 0x00E4},
{0x0F12, 0x053C},
{0x0F12, 0x0400},
{0x0F12, 0x055C},
{0x0F12, 0x001E},
{0x0F12, 0x0190},
{0x0F12, 0x00A0},
{0x0F12, 0x0004},
{0x0F12, 0x0001},

{0x002A, 0x0F28},
{0x0F12, 0x03C0},
{0x0F12, 0x03E2},
{0x0F12, 0x0356},
{0x0F12, 0x03FC},
{0x0F12, 0x031E},
{0x0F12, 0x03FE},
{0x0F12, 0x02F0},
{0x0F12, 0x03F0},
{0x0F12, 0x02CA},
{0x0F12, 0x03CC},
{0x0F12, 0x02A8},
{0x0F12, 0x037A},
{0x0F12, 0x0280},
{0x0F12, 0x033C},
{0x0F12, 0x0260},
{0x0F12, 0x030A},
{0x0F12, 0x0242},
{0x0F12, 0x02DC},
{0x0F12, 0x0228},
{0x0F12, 0x02B2},
{0x0F12, 0x020E},
{0x0F12, 0x0290},
{0x0F12, 0x01F8},
{0x0F12, 0x0276},
{0x0F12, 0x01E8},
{0x0F12, 0x0268},
{0x0F12, 0x01DC},
{0x0F12, 0x0256},
{0x0F12, 0x01E0},
{0x0F12, 0x0238},
{0x0F12, 0x01EC},
{0x0F12, 0x020E},
{0x0F12, 0x0000},
{0x0F12, 0x0000},
{0x0F12, 0x0000},
{0x0F12, 0x0000},
{0x0F12, 0x0000},
{0x0F12, 0x0000},
{0x0F12, 0x0000},
{0x0F12, 0x0000},

{0x0F12, 0x0005},
{0x002A, 0x0F80},
{0x0F12, 0x00E6},
{0x002A, 0x0F7C},
{0x0F12, 0x0010},

{0x002A, 0x0F84},
{0x0F12, 0x028C},
{0x0F12, 0x02A0},
{0x0F12, 0x0276},	
{0x0F12, 0x02B2},	
{0x0F12, 0x0262},
{0x0F12, 0x02BC},
{0x0F12, 0x0250},
{0x0F12, 0x02BE},
{0x0F12, 0x0240},
{0x0F12, 0x02BE},
{0x0F12, 0x0232},
{0x0F12, 0x02B6},
{0x0F12, 0x0226},
{0x0F12, 0x02A8},
{0x0F12, 0x021A},
{0x0F12, 0x029C},	
{0x0F12, 0x0214},
{0x0F12, 0x028C},
{0x0F12, 0x0216},	
{0x0F12, 0x027A},
{0x0F12, 0x0226},
{0x0F12, 0x025A},
{0x0F12, 0x0000},
{0x0F12, 0x0000},


{0x0F12, 0x0004},
{0x002A, 0x0FB8},
{0x0F12, 0x000B},
{0x002A, 0x0FBC},
{0x0F12, 0x01E2},


{0x002A, 0x0FC0},
{0x0F12, 0x03B2},
{0x0F12, 0x044E},
{0x0F12, 0x0330},
{0x0F12, 0x0454},
{0x0F12, 0x02CC},
{0x0F12, 0x0414},
{0x0F12, 0x026E},
{0x0F12, 0x03D0},
{0x0F12, 0x0226},
{0x0F12, 0x0362},
{0x0F12, 0x01F0},
{0x0F12, 0x0312},
{0x0F12, 0x01CE},
{0x0F12, 0x02CC},
{0x0F12, 0x01B2},
{0x0F12, 0x029E},
{0x0F12, 0x01AC},
{0x0F12, 0x0278},
{0x0F12, 0x01B6},
{0x0F12, 0x0248},
{0x0F12, 0x0000},
{0x0F12, 0x0000},
{0x0F12, 0x0000},
{0x0F12, 0x0000},

{0x0F12, 0x0006},
{0x002A, 0x0FF4},
{0x0F12, 0x000A},
{0x002A, 0x0FF8},
{0x0F12, 0x00C2},

{0x002A, 0x1098},
{0x0F12, 0xFE82},
{0x0F12, 0x001E},
{0x0F12, 0x09C4},
{0x0F12, 0x0122},
{0x0F12, 0x00E4},
{0x0F12, 0x0096},
{0x0F12, 0x000E},

{0x002A, 0x105C},
{0x0F12, 0x0000},
{0x0F12, 0x0000},
{0x0F12, 0x0000},
{0x0F12, 0x0000},
{0x0F12, 0x0000},
{0x0F12, 0x0000},
{0x0F12, 0x0000},
{0x0F12, 0x0000},
{0x0F12, 0x0000},
{0x0F12, 0x0000},
{0x0F12, 0x0000},
{0x0F12, 0x0000},
{0x0F12, 0x0500},
{0x0F12, 0x5555},
{0x0F12, 0x5455},
{0x0F12, 0xAA55},
{0x0F12, 0xAAAA},
{0x0F12, 0xBF54},
{0x0F12, 0xFFFF},
{0x0F12, 0x54FE},
{0x0F12, 0xFF6F},
{0x0F12, 0xFEFF},
{0x0F12, 0x1B54},
{0x0F12, 0xFFFF},
{0x0F12, 0x54FE},
{0x0F12, 0xFF06},
{0x0F12, 0xFEFF},
{0x0F12, 0x0154},
{0x0F12, 0xBFBF},
{0x0F12, 0x54BE},

{0x002A, 0x11E0},
{0x0F12, 0x0002},

{0x002A, 0x11A8},
{0x0F12, 0x02C8},
{0x0F12, 0x0325},
{0x0F12, 0x038F},

{0x0F12, 0x0F8E},
{0x0F12, 0x10B3},
{0x0F12, 0x1136},
{0x0F12, 0x1138},
{0x0F12, 0x118E},
{0x0F12, 0x1213},

{0x0F12, 0x00A7},
{0x0F12, 0x00C2},
{0x0F12, 0x00BD},
{0x0F12, 0x00AC},

{0x002A, 0x1118},
{0x0F12, 0x0050},
{0x0F12, 0x0032},
{0x0F12, 0x0032},
{0x0F12, 0xFFEC},
{0x0F12, 0xFFEC},
{0x0F12, 0x0050},
{0x0F12, 0x0050},
{0x0F12, 0x0032},
{0x0F12, 0x0032},
{0x0F12, 0xFFEC},
{0x0F12, 0xFFEC},
{0x0F12, 0x0050},
{0x0F12, 0x0050},
{0x0F12, 0x0032},
{0x0F12, 0x0032},
{0x0F12, 0xFFEC},
{0x0F12, 0xFFEC},
{0x0F12, 0x0050},
{0x0F12, 0xFF9C},
{0x0F12, 0xFFD8},
{0x0F12, 0xFFEC},
{0x0F12, 0xFF97},
{0x0F12, 0xFF97},
{0x0F12, 0xFDA8},
{0x0F12, 0xFF9C},
{0x0F12, 0xFFD8},
{0x0F12, 0xFFEC},
{0x0F12, 0xFF97},
{0x0F12, 0xFF97},
{0x0F12, 0xFDA8},
{0x0F12, 0xFF9C},
{0x0F12, 0xFFD8},
{0x0F12, 0xFFEC},
{0x0F12, 0xFF97},
{0x0F12, 0xFF97},
{0x0F12, 0xFDA8},

{0x002A, 0x1160},
{0x0F12, 0x0006},
{0x0F12, 0x0006},
{0x0F12, 0x0000},
{0x0F12, 0x0000},
{0x0F12, 0x0000},
{0x0F12, 0x0000},
{0x0F12, 0x0006},
{0x0F12, 0x0006},
{0x0F12, 0x0000},
{0x0F12, 0x0000},
{0x0F12, 0x0000},
{0x0F12, 0x0000},
{0x0F12, 0x0006},
{0x0F12, 0x0006},
{0x0F12, 0x0000},
{0x0F12, 0x0000},
{0x0F12, 0x0000},
{0x0F12, 0x0000},
{0x0F12, 0xFFDA},
{0x0F12, 0xFFDA},
{0x0F12, 0x0000},
{0x0F12, 0x0000},
{0x0F12, 0x0000},
{0x0F12, 0x0000},
{0x0F12, 0xFFDA},
{0x0F12, 0xFFDA},
{0x0F12, 0x0000},
{0x0F12, 0x0000},
{0x0F12, 0x0000},
{0x0F12, 0x0000},
{0x0F12, 0xFFDA},
{0x0F12, 0xFFDA},
{0x0F12, 0x0000},
{0x0F12, 0x0000},
{0x0F12, 0x0000},
{0x0F12, 0x0000},

{0x002A, 0x07D2},
{0x0F12, 0x00C0},
{0x0F12, 0x00E0},
{0x0F12, 0x0110},
{0x0F12, 0x0139},
{0x0F12, 0x0168},
{0x0F12, 0x019F},

{0x002A, 0x07C4},
{0x0F12, 0x4000},
{0x0F12, 0x7000},

{0x002A, 0x4000},
{0x0F12, 0x01EA},
{0x0F12, 0xFFAC},
{0x0F12, 0xFFE3},
{0x0F12, 0xFF45},
{0x0F12, 0x0140},
{0x0F12, 0xFF4F},
{0x0F12, 0xFFC3},
{0x0F12, 0xFFD5},
{0x0F12, 0x0173},
{0x0F12, 0x0137},
{0x0F12, 0x00C2},
{0x0F12, 0xFEC1},
{0x0F12, 0x00C8},
{0x0F12, 0xFF49},
{0x0F12, 0x014B},
{0x0F12, 0xFF68},
{0x0F12, 0x0109},
{0x0F12, 0x00F4},

{0x0F12, 0x01EA},
{0x0F12, 0xFFAC},
{0x0F12, 0xFFE3},
{0x0F12, 0xFF45},
{0x0F12, 0x0140},
{0x0F12, 0xFF4F},
{0x0F12, 0xFFC3},
{0x0F12, 0xFFD5},
{0x0F12, 0x0173},
{0x0F12, 0x0137},
{0x0F12, 0x00C2},
{0x0F12, 0xFEC1},
{0x0F12, 0x00C8},
{0x0F12, 0xFF49},
{0x0F12, 0x014B},
{0x0F12, 0xFF68},
{0x0F12, 0x0109},
{0x0F12, 0x00F4},

{0x0F12, 0x01EA},
{0x0F12, 0xFFAC},
{0x0F12, 0xFFE3},
{0x0F12, 0xFF45},
{0x0F12, 0x0140},
{0x0F12, 0xFF4F},
{0x0F12, 0xFFC3},
{0x0F12, 0xFFD5},
{0x0F12, 0x0173},
{0x0F12, 0x0137},
{0x0F12, 0x00C2},
{0x0F12, 0xFEC1},
{0x0F12, 0x00C8},
{0x0F12, 0xFF49},
{0x0F12, 0x014B},
{0x0F12, 0xFF68},
{0x0F12, 0x0109},
{0x0F12, 0x00F4},

{0x0F12, 0x01EA},
{0x0F12, 0xFFAC},
{0x0F12, 0xFFE3},
{0x0F12, 0xFF45},
{0x0F12, 0x0140},
{0x0F12, 0xFF4F},
{0x0F12, 0xFFC3},
{0x0F12, 0xFFD5},
{0x0F12, 0x0173},
{0x0F12, 0x0137},
{0x0F12, 0x00C2},
{0x0F12, 0xFEC1},
{0x0F12, 0x00C8},
{0x0F12, 0xFF49},
{0x0F12, 0x014B},
{0x0F12, 0xFF68},
{0x0F12, 0x0109},
{0x0F12, 0x00F4},

{0x0F12, 0x011D},
{0x0F12, 0xFFA7},
{0x0F12, 0xFFEC},
{0x0F12, 0xFF0D},
{0x0F12, 0x0193},
{0x0F12, 0xFF34},
{0x0F12, 0xFFCE},
{0x0F12, 0xFFDF},
{0x0F12, 0x015E},
{0x0F12, 0x0095},
{0x0F12, 0x0096},
{0x0F12, 0xFF0B},
{0x0F12, 0x00C3},
{0x0F12, 0xFF5C},
{0x0F12, 0x013D},
{0x0F12, 0xFF68},
{0x0F12, 0x0109},
{0x0F12, 0x00F4},

{0x0F12, 0x011D},
{0x0F12, 0xFFA7},
{0x0F12, 0xFFEC},
{0x0F12, 0xFF0D},
{0x0F12, 0x0193},
{0x0F12, 0xFF34},
{0x0F12, 0xFFCE},
{0x0F12, 0xFFDF},
{0x0F12, 0x015E},
{0x0F12, 0x0095},
{0x0F12, 0x0096},
{0x0F12, 0xFF0B},
{0x0F12, 0x00C3},
{0x0F12, 0xFF5C},
{0x0F12, 0x013D},
{0x0F12, 0xFF68},
{0x0F12, 0x0109},
{0x0F12, 0x00F4},


{0x002A, 0x07CC},
{0x0F12, 0x40D8},
{0x0F12, 0x7000},

{0x002A, 0x40D8},
{0x0F12, 0x01F6},
{0x0F12, 0xFF9F},
{0x0F12, 0xFFE5},
{0x0F12, 0xFED2},
{0x0F12, 0x0193},
{0x0F12, 0xFF23},
{0x0F12, 0xFFF7},
{0x0F12, 0x000C},
{0x0F12, 0x0211},
{0x0F12, 0x00FF},
{0x0F12, 0x00EC},
{0x0F12, 0xFF2E},
{0x0F12, 0x0220},
{0x0F12, 0xFFE7},
{0x0F12, 0x01A1},
{0x0F12, 0xFEC7},
{0x0F12, 0x016D},
{0x0F12, 0x0153},

{0x002A, 0x2A64},
{0x0F12, 0x0001},
{0x002A, 0x2A68},
{0x0F12, 0x0001},
{0x002A, 0x2A3C},
{0x0F12, 0x01DD},

{0x002A, 0x085C},
{0x0F12, 0x004A},
{0x0F12, 0x004E},
{0x0F12, 0x00CB},
{0x0F12, 0x01C0},
{0x0F12, 0x0200},

{0x002A, 0x08C0},
{0x0F12, 0x0007},
{0x0F12, 0x0000},
{0x0F12, 0x0000},
{0x0F12, 0x0002},
{0x0F12, 0x0000},
{0x0F12, 0x00C1},
{0x0F12, 0x0000},
{0x0F12, 0x03FF},
{0x0F12, 0x009C},
{0x0F12, 0x017C},
{0x0F12, 0x03FF},
{0x0F12, 0x000C},
{0x0F12, 0x0010},
{0x0F12, 0x012C},
{0x0F12, 0x03E8},
{0x0F12, 0x0046},
{0x0F12, 0x005A},
{0x0F12, 0x0070},
{0x0F12, 0x0010},
{0x0F12, 0x0010},
{0x0F12, 0x01F4},
{0x0F12, 0x006E},
{0x0F12, 0x0014},
{0x0F12, 0x003C},
{0x0F12, 0x001E},
{0x0F12, 0x003C},
{0x0F12, 0x001E},
{0x0F12, 0x0A24},
{0x0F12, 0x1701},
{0x0F12, 0x0229},
{0x0F12, 0x1403},
{0x0F12, 0x0004},
{0x0F12, 0x0300},
{0x0F12, 0x0000},
{0x0F12, 0x02FF},
{0x0F12, 0x05E8},
{0x0F12, 0x1414},
{0x0F12, 0x0301},
{0x0F12, 0x0007},
{0x0F12, 0x4000},
{0x0F12, 0x7803},
{0x0F12, 0x3C50},
{0x0F12, 0x003C},
{0x0F12, 0x1E80},
{0x0F12, 0x1E08},
{0x0F12, 0x000A},
{0x0F12, 0x0000},
{0x0F12, 0x120A},
{0x0F12, 0x1400},
{0x0F12, 0x0200},
{0x0F12, 0xFF00},
{0x0F12, 0x0200},
{0x0F12, 0x1B11},
{0x0F12, 0x0000},
{0x0F12, 0x0009},
{0x0F12, 0x0406},
{0x0F12, 0x0605},
{0x0F12, 0x0307},
{0x0F12, 0x0609},
{0x0F12, 0x2C07},
{0x0F12, 0x142C},
{0x0F12, 0x0B18},
{0x0F12, 0x800B},
{0x0F12, 0x0880},
{0x0F12, 0x0B50},
{0x0F12, 0x0080},
{0x0F12, 0x0101},
{0x0F12, 0x0707},
{0x0F12, 0x4601},
{0x0F12, 0xA444},
{0x0F12, 0x50A4},
{0x0F12, 0x0500},
{0x0F12, 0x0303},
{0x0F12, 0x1001},
{0x0F12, 0x0710},
{0x0F12, 0x1448},
{0x0F12, 0x5A03},
{0x0F12, 0x281E},
{0x0F12, 0x200F},
{0x0F12, 0x0204},
{0x0F12, 0x1403},
{0x0F12, 0x0114},
{0x0F12, 0x0101},
{0x0F12, 0x4446},
{0x0F12, 0x646E},
{0x0F12, 0x0028},
{0x0F12, 0x030A},
{0x0F12, 0x0000},
{0x0F12, 0x141E},
{0x0F12, 0xFF07},
{0x0F12, 0x0432},
{0x0F12, 0x0000},
{0x0F12, 0x0F0F},
{0x0F12, 0x0440},
{0x0F12, 0x0302},
{0x0F12, 0x1414},
{0x0F12, 0x0101},
{0x0F12, 0x4601},
{0x0F12, 0x6E44},
{0x0F12, 0x2864},
{0x0F12, 0x0A00},
{0x0F12, 0x0003},
{0x0F12, 0x1E00},
{0x0F12, 0x0714},
{0x0F12, 0x32FF},
{0x0F12, 0x0004},
{0x0F12, 0x0F00},
{0x0F12, 0x400F},
{0x0F12, 0x0204},
{0x0F12, 0x0003},
{0x0F12, 0x0001},
{0x0F12, 0x0000},
{0x0F12, 0x0000},
{0x0F12, 0x0000},
{0x0F12, 0x0002},
{0x0F12, 0x0000},
{0x0F12, 0x00C1},
{0x0F12, 0x0000},
{0x0F12, 0x03FF},
{0x0F12, 0x009C},
{0x0F12, 0x017C},
{0x0F12, 0x03FF},
{0x0F12, 0x000C},
{0x0F12, 0x0010},
{0x0F12, 0x012C},
{0x0F12, 0x03E8},
{0x0F12, 0x0046},
{0x0F12, 0x005A},
{0x0F12, 0x0070},
{0x0F12, 0x0001},
{0x0F12, 0x0000},
{0x0F12, 0x0320},
{0x0F12, 0x006E},
{0x0F12, 0x0014},
{0x0F12, 0x003C},
{0x0F12, 0x001E},
{0x0F12, 0x003C},
{0x0F12, 0x001E},
{0x0F12, 0x0A24},
{0x0F12, 0x1701},
{0x0F12, 0x0229},
{0x0F12, 0x1403},
{0x0F12, 0x0004},
{0x0F12, 0x0300},
{0x0F12, 0x0000},
{0x0F12, 0x02FF},
{0x0F12, 0x05E8},
{0x0F12, 0x1414},
{0x0F12, 0x0301},
{0x0F12, 0x0007},
{0x0F12, 0x2000},
{0x0F12, 0x5003},
{0x0F12, 0x3228},
{0x0F12, 0x0032},
{0x0F12, 0x1E80},
{0x0F12, 0x1E08},
{0x0F12, 0x000A},
{0x0F12, 0x0000},
{0x0F12, 0x120A},
{0x0F12, 0x1400},
{0x0F12, 0x0200},
{0x0F12, 0xFF00},
{0x0F12, 0x0200},
{0x0F12, 0x1B11},
{0x0F12, 0x0000},
{0x0F12, 0x0009},
{0x0F12, 0x0406},
{0x0F12, 0x0605},
{0x0F12, 0x0307},
{0x0F12, 0x0609},
{0x0F12, 0x2C07},
{0x0F12, 0x142C},
{0x0F12, 0x0518},
{0x0F12, 0x8005},
{0x0F12, 0x0580},
{0x0F12, 0x0080},
{0x0F12, 0x0080},
{0x0F12, 0x0101},
{0x0F12, 0x0707},
{0x0F12, 0x4B01},
{0x0F12, 0x494B},
{0x0F12, 0x5044},
{0x0F12, 0x0500},
{0x0F12, 0x0603},
{0x0F12, 0x0D03},
{0x0F12, 0x071E},
{0x0F12, 0x1432},
{0x0F12, 0x5A01},
{0x0F12, 0x281E},
{0x0F12, 0x200F},
{0x0F12, 0x0204},
{0x0F12, 0x1E03},
{0x0F12, 0x011E},
{0x0F12, 0x0101},
{0x0F12, 0x3A3C},
{0x0F12, 0x585A},
{0x0F12, 0x0028},
{0x0F12, 0x030A},
{0x0F12, 0x0000},
{0x0F12, 0x141E},
{0x0F12, 0xFF07},
{0x0F12, 0x0432},
{0x0F12, 0x0000},
{0x0F12, 0x0F0F},
{0x0F12, 0x0440},
{0x0F12, 0x0302},
{0x0F12, 0x1E1E},
{0x0F12, 0x0101},
{0x0F12, 0x3C01},
{0x0F12, 0x5A3A},
{0x0F12, 0x2858},
{0x0F12, 0x0A00},
{0x0F12, 0x0003},
{0x0F12, 0x1E00},
{0x0F12, 0x0714},
{0x0F12, 0x32FF},
{0x0F12, 0x0004},
{0x0F12, 0x0F00},
{0x0F12, 0x400F},
{0x0F12, 0x0204},
{0x0F12, 0x0003},
{0x0F12, 0x0001},
{0x0F12, 0x0000},
{0x0F12, 0x0000},
{0x0F12, 0x0000},
{0x0F12, 0x0000},
{0x0F12, 0x0000},
{0x0F12, 0x00C1},
{0x0F12, 0x0000},
{0x0F12, 0x03FF},
{0x0F12, 0x009E},
{0x0F12, 0x017C},
{0x0F12, 0x03FF},
{0x0F12, 0x000C},
{0x0F12, 0x0010},
{0x0F12, 0x012C},
{0x0F12, 0x03E8},
{0x0F12, 0x0046},
{0x0F12, 0x005A},
{0x0F12, 0x0070},
{0x0F12, 0x0001},
{0x0F12, 0x0000},
{0x0F12, 0x0320},
{0x0F12, 0x008C},
{0x0F12, 0x0014},
{0x0F12, 0x003C},
{0x0F12, 0x001E},
{0x0F12, 0x003C},
{0x0F12, 0x001E},
{0x0F12, 0x0A24},
{0x0F12, 0x1701},
{0x0F12, 0x0229},
{0x0F12, 0x1403},
{0x0F12, 0x0004},
{0x0F12, 0x0300},
{0x0F12, 0x0000},
{0x0F12, 0x02FF},
{0x0F12, 0x05DE},
{0x0F12, 0x1414},
{0x0F12, 0x0301},
{0x0F12, 0x0007},
{0x0F12, 0x1000},
{0x0F12, 0x2803},
{0x0F12, 0x261E},
{0x0F12, 0x0026},
{0x0F12, 0x1E80},
{0x0F12, 0x1E08},
{0x0F12, 0x010A},
{0x0F12, 0x0001},
{0x0F12, 0x3C0A},
{0x0F12, 0x2300},
{0x0F12, 0x0200},
{0x0F12, 0xFF00},
{0x0F12, 0x0200},
{0x0F12, 0x1B11},
{0x0F12, 0x0000},
{0x0F12, 0x0009},
{0x0F12, 0x0406},
{0x0F12, 0x0605},
{0x0F12, 0x0307},
{0x0F12, 0x0609},
{0x0F12, 0x1C07},
{0x0F12, 0x1014},
{0x0F12, 0x0510},
{0x0F12, 0x8005},
{0x0F12, 0x0080},
{0x0F12, 0x0080},
{0x0F12, 0x0080},
{0x0F12, 0x0101},
{0x0F12, 0x0707},
{0x0F12, 0x4B01},
{0x0F12, 0x2A4B},
{0x0F12, 0x5020},
{0x0F12, 0x0500},
{0x0F12, 0x1C03},
{0x0F12, 0x0D0C},
{0x0F12, 0x0823},
{0x0F12, 0x1428},
{0x0F12, 0x6401},
{0x0F12, 0x282D},
{0x0F12, 0x2012},
{0x0F12, 0x0204},
{0x0F12, 0x2803},
{0x0F12, 0x0128},
{0x0F12, 0x0101},
{0x0F12, 0x2224},
{0x0F12, 0x3236},
{0x0F12, 0x0028},
{0x0F12, 0x030A},
{0x0F12, 0x0410},
{0x0F12, 0x141E},
{0x0F12, 0xFF07},
{0x0F12, 0x0432},
{0x0F12, 0x4050},
{0x0F12, 0x0F0F},
{0x0F12, 0x0440},
{0x0F12, 0x0302},
{0x0F12, 0x2828},
{0x0F12, 0x0101},
{0x0F12, 0x2401},
{0x0F12, 0x3622},
{0x0F12, 0x2832},
{0x0F12, 0x0A00},
{0x0F12, 0x1003},
{0x0F12, 0x1E04},
{0x0F12, 0x0714},
{0x0F12, 0x32FF},
{0x0F12, 0x5004},
{0x0F12, 0x0F40},
{0x0F12, 0x400F},
{0x0F12, 0x0204},
{0x0F12, 0x0003},
{0x0F12, 0x0001},
{0x0F12, 0x0000},
{0x0F12, 0x0000},
{0x0F12, 0x0000},
{0x0F12, 0x0000},
{0x0F12, 0x0000},
{0x0F12, 0x00C1},
{0x0F12, 0x0000},
{0x0F12, 0x03FF},
{0x0F12, 0x009E},
{0x0F12, 0x017C},
{0x0F12, 0x03FF},
{0x0F12, 0x000C},
{0x0F12, 0x0010},
{0x0F12, 0x00C8},
{0x0F12, 0x03E8},
{0x0F12, 0x0046},
{0x0F12, 0x0050},
{0x0F12, 0x0070},
{0x0F12, 0x0001},
{0x0F12, 0x0000},
{0x0F12, 0x0320},
{0x0F12, 0x008C},
{0x0F12, 0x0014},
{0x0F12, 0x002D},
{0x0F12, 0x0019},
{0x0F12, 0x002D},
{0x0F12, 0x0019},
{0x0F12, 0x0A24},
{0x0F12, 0x1701},
{0x0F12, 0x0229},
{0x0F12, 0x1403},
{0x0F12, 0x0004},
{0x0F12, 0x0300},
{0x0F12, 0x0000},
{0x0F12, 0x02FF},
{0x0F12, 0x05DE},
{0x0F12, 0x1414},
{0x0F12, 0x0301},
{0x0F12, 0x0007},
{0x0F12, 0x1000},
{0x0F12, 0x2303},
{0x0F12, 0x231A},
{0x0F12, 0x0023},
{0x0F12, 0x1E80},
{0x0F12, 0x1E08},
{0x0F12, 0x010A},
{0x0F12, 0x0001},
{0x0F12, 0x3C0A},
{0x0F12, 0x2300},
{0x0F12, 0x0200},
{0x0F12, 0xFF00},
{0x0F12, 0x0200},
{0x0F12, 0x1E10},
{0x0F12, 0x0000},
{0x0F12, 0x0009},
{0x0F12, 0x0406},
{0x0F12, 0x0705},
{0x0F12, 0x0306},
{0x0F12, 0x0509},
{0x0F12, 0x2806},
{0x0F12, 0x1428},
{0x0F12, 0x0518},
{0x0F12, 0x8005},
{0x0F12, 0x0080},
{0x0F12, 0x0080},
{0x0F12, 0x0080},
{0x0F12, 0x0101},
{0x0F12, 0x0707},
{0x0F12, 0x4B01},
{0x0F12, 0x2A4B},
{0x0F12, 0x5020},
{0x0F12, 0x0500},
{0x0F12, 0x1C03},
{0x0F12, 0x0D0C},
{0x0F12, 0x0823},
{0x0F12, 0x1428},
{0x0F12, 0x6401},
{0x0F12, 0x282D},
{0x0F12, 0x2012},
{0x0F12, 0x0204},
{0x0F12, 0x3C03},
{0x0F12, 0x013C},
{0x0F12, 0x0101},
{0x0F12, 0x1C1E},
{0x0F12, 0x1E22},
{0x0F12, 0x0028},
{0x0F12, 0x030A},
{0x0F12, 0x0214},
{0x0F12, 0x0E14},
{0x0F12, 0xFF06},
{0x0F12, 0x0432},
{0x0F12, 0x4052},
{0x0F12, 0x150C},
{0x0F12, 0x0440},
{0x0F12, 0x0302},
{0x0F12, 0x3C3C},
{0x0F12, 0x0101},
{0x0F12, 0x1E01},
{0x0F12, 0x221C},
{0x0F12, 0x281E},
{0x0F12, 0x0A00},
{0x0F12, 0x1403},
{0x0F12, 0x1402},
{0x0F12, 0x060E},
{0x0F12, 0x32FF},
{0x0F12, 0x5204},
{0x0F12, 0x0C40},
{0x0F12, 0x4015},
{0x0F12, 0x0204},
{0x0F12, 0x0003},
{0x0F12, 0x0001},
{0x0F12, 0x0000},
{0x0F12, 0x0000},
{0x0F12, 0x0000},
{0x0F12, 0x0000},
{0x0F12, 0x0000},
{0x0F12, 0x00C1},
{0x0F12, 0x0000},
{0x0F12, 0x03FF},
{0x0F12, 0x009C},
{0x0F12, 0x0251},
{0x0F12, 0x03FF},
{0x0F12, 0x000C},
{0x0F12, 0x0010},
{0x0F12, 0x0032},
{0x0F12, 0x028A},
{0x0F12, 0x0032},
{0x0F12, 0x01F4},
{0x0F12, 0x0070},
{0x0F12, 0x0002},
{0x0F12, 0x0000},
{0x0F12, 0x0320},
{0x0F12, 0x0044},
{0x0F12, 0x0014},
{0x0F12, 0x0046},
{0x0F12, 0x0019},
{0x0F12, 0x0046},
{0x0F12, 0x0019},
{0x0F12, 0x0A24},
{0x0F12, 0x1701},
{0x0F12, 0x0229},
{0x0F12, 0x0503},
{0x0F12, 0x080F},
{0x0F12, 0x0808},
{0x0F12, 0x0000},
{0x0F12, 0x00FF},
{0x0F12, 0x012D},
{0x0F12, 0x1414},
{0x0F12, 0x0301},
{0x0F12, 0x0007},
{0x0F12, 0x1000},
{0x0F12, 0x2003},
{0x0F12, 0x1020},
{0x0F12, 0x0010},
{0x0F12, 0x1EFF},
{0x0F12, 0x1E06},
{0x0F12, 0x060A},
{0x0F12, 0x0306},
{0x0F12, 0x8B0A},
{0x0F12, 0x2837},
{0x0F12, 0x0110},
{0x0F12, 0xFF00},
{0x0F12, 0x0200},
{0x0F12, 0x1E10},
{0x0F12, 0x0000},
{0x0F12, 0x0009},
{0x0F12, 0x0406},
{0x0F12, 0x0705},
{0x0F12, 0x0305},
{0x0F12, 0x0609},
{0x0F12, 0x2C07},
{0x0F12, 0x142C},
{0x0F12, 0x0B18},
{0x0F12, 0x800B},
{0x0F12, 0x0080},
{0x0F12, 0x0080},
{0x0F12, 0x0080},
{0x0F12, 0x5050},
{0x0F12, 0x0101},
{0x0F12, 0x3201},
{0x0F12, 0x1832},
{0x0F12, 0x210C},
{0x0F12, 0x0A00},
{0x0F12, 0x1E04},
{0x0F12, 0x0A08},
{0x0F12, 0x070C},
{0x0F12, 0x3264},
{0x0F12, 0x5A02},
{0x0F12, 0x1040},
{0x0F12, 0x4012},
{0x0F12, 0x0604},
{0x0F12, 0x4606},
{0x0F12, 0x0146},
{0x0F12, 0x0101},
{0x0F12, 0x1C18},
{0x0F12, 0x1819},
{0x0F12, 0x0028},
{0x0F12, 0x030A},
{0x0F12, 0x0514},
{0x0F12, 0x0C14},
{0x0F12, 0xFF05},
{0x0F12, 0x0432},
{0x0F12, 0x4052},
{0x0F12, 0x1514},
{0x0F12, 0x0440},
{0x0F12, 0x0302},
{0x0F12, 0x4646},
{0x0F12, 0x0101},
{0x0F12, 0x1801},
{0x0F12, 0x191C},
{0x0F12, 0x2818},
{0x0F12, 0x0A00},
{0x0F12, 0x1403},
{0x0F12, 0x1405},
{0x0F12, 0x050C},
{0x0F12, 0x32FF},
{0x0F12, 0x5204},
{0x0F12, 0x1440},
{0x0F12, 0x4015},
{0x0F12, 0x0204},
{0x0F12, 0x0003},
{0x0F12, 0x0001},

{0x0F12, 0xBA7A},
{0x0F12, 0x4FDE},
{0x0F12, 0x137F},
{0x0F12, 0x3BDE},
{0x0F12, 0xA102},
{0x0F12, 0x00B5},

{0x002A, 0x1300},
{0x0F12, 0x019D},

{0x002A, 0x1306},
{0x0F12, 0x0280},

{0x0028, 0x7000},
{0x002A, 0x0ED8},
{0x0F12, 0x0000},

CAMACQ_MAIN_BTM_OF_DATA
}
#endif /* _CAMACQ_API_C_ */
;

GLOBAL const U16 reg_main_fmt_jpg[][2]
#if defined(_CAMACQ_API_C_)
={

CAMACQ_MAIN_BTM_OF_DATA
}
#endif /* _CAMACQ_API_C_ */
;

GLOBAL const U16 reg_main_fmt_yuv422[][2]
#if defined(_CAMACQ_API_C_)
={

CAMACQ_MAIN_BTM_OF_DATA
}
#endif /* _CAMACQ_API_C_ */
;

//==========================================================
// CAMERA PREVIEW(640*480 / FULL)
//==========================================================
GLOBAL const U16 reg_main_preview[][2]
#if defined(_CAMACQ_API_C_)
={
{0xFCFC, 0xD000},
{0x0028, 0x7000},

{0x002A, 0x0C7E},
{0x0F12, 0x0396},

{0x002A, 0x0CC4},
{0x0F12, 0x100C},

{0x002A, 0x0836},
{0x0F12, 0x3E00},
{0x0F12, 0x4000},
{0x0F12, 0x4000},
{0x0F12, 0x4000},

{0x002A, 0x0D1E},
{0x0F12, 0x2102},


{0x002A, 0x0208},
{0x0F12, 0x0000},	
{0x002A, 0x0210},
{0x0F12, 0x0000},	
{0x002A, 0x020C},
{0x0F12, 0x0001},	
{0x002A, 0x01F4},
{0x0F12, 0x0001},	
{0x002A, 0x020A},
{0x0F12, 0x0001},	
{0x002A, 0x0212},
{0x0F12, 0x0001},	
{0x002A, 0x01E8},
{0x0F12, 0x0000},	
{0x0F12, 0x0001},	
{0xFFFF, 0x0064},

CAMACQ_MAIN_BTM_OF_DATA
}
#endif /* _CAMACQ_API_C_ */
;

//==========================================================
// CAMCORDER ON
//==========================================================
GLOBAL const U16 reg_main_camcorder_on[][2]
#if defined(_CAMACQ_API_C_)
={

CAMACQ_MAIN_BTM_OF_DATA
}
#endif /* _CAMACQ_API_C_ */
;

//==========================================================
// SNAPSHOT
//==========================================================

GLOBAL const U16 reg_main_snapshot[][2]
#if defined(_CAMACQ_API_C_)
={
{0xFCFC, 0xD000}, 
{0x0028, 0x7000}, 
                  
{0x002A, 0x0D1E}, 
{0x0F12, 0xA102},	
                  
{0x002A, 0x0210}, 
{0x0F12, 0x0000},	
{0x002A, 0x01F4}, 
{0x0F12, 0x0001},	
{0x002A, 0x0212}, 
{0x0F12, 0x0001},	
{0x002A, 0x01E8}, 
{0x0F12, 0x0001},	
{0x0F12, 0x0001},	
                  
{0xFFFF, 0x00A0}, 

CAMACQ_MAIN_BTM_OF_DATA
}
#endif /* _CAMACQ_API_C_ */
;

//==========================================================
// MIDLIGHT SNAPSHOT =======> Flash Low light snapshot
//==========================================================
GLOBAL const U16 reg_main_midlight[][2]
#if defined(_CAMACQ_API_C_)
={

CAMACQ_MAIN_BTM_OF_DATA
}
#endif /* _CAMACQ_API_C_ */
;

//==========================================================
// LOWLIGHT SNAPSHOT
//==========================================================
GLOBAL const U16 reg_main_lowlight[][2]
#if defined(_CAMACQ_API_C_)
={
{0xFCFC, 0xD000},
{0x0028, 0x7000},
       
{0x002A, 0x0D1E},
{0x0F12, 0xA102},
                 
{0x002A, 0x0210},
{0x0F12, 0x0000},
{0x002A, 0x01F4},
{0x0F12, 0x0001},
{0x002A, 0x0212},
{0x0F12, 0x0001},
{0x002A, 0x01E8},
{0x0F12, 0x0001},
{0x0F12, 0x0001},
                 
{0xFFFF, 0x00A0},

CAMACQ_MAIN_BTM_OF_DATA
}
#endif /* _CAMACQ_API_C_ */
;

//==========================================================
// HIGHLIGHT SNAPSHOT
//==========================================================
GLOBAL const U16 reg_main_highlight[][2]
#if defined(_CAMACQ_API_C_)
={
{0xFCFC, 0xD000},
{0x0028, 0x7000},

{0x002A, 0x0C7E},
{0x0F12, 0x032D},	//70000C7E	//AFIT8_sddd8a_iRadialLimit [7:0], AFIT8_sddd8a_iRadialPower [15:8]

{0x002A, 0x0CC4},
{0x0F12, 0x210E},	//70000CC4	//AFIT8_sddd8a_iDenThreshHigh[7:0],   AFIT8_Demosaicing_iEdgeDesat [15:8]

{0x002A, 0x0836},
{0x0F12, 0x3A00},	//TVAR_ash_GASOutdoorAlpha_0_
{0x0F12, 0x4000},	//TVAR_ash_GASOutdoorAlpha_1_
{0x0F12, 0x4000},	//TVAR_ash_GASOutdoorAlpha_2_
{0x0F12, 0x4000},	//TVAR_ash_GASOutdoorAlpha_3_

{0x002A, 0x0D1E},
{0x0F12, 0xA102},	//70000D1E

{0x002A, 0x0210},
{0x0F12, 0x0000},	//REG_TC_GP_ActiveCapConfig
{0x002A, 0x01F4},
{0x0F12, 0x0001},	//REG_TC_GP_NewConfigSync
{0x002A, 0x0212},
{0x0F12, 0x0001},	//REG_TC_GP_CapConfigChanged
{0x002A, 0x01E8},
{0x0F12, 0x0001},	//REG_TC_GP_EnableCapture
{0x0F12, 0x0001},	//REG_TC_GP_EnableCaptureChanged 

{0xFFFF, 0x00A0},   //160ms
CAMACQ_MAIN_BTM_OF_DATA
}
#endif /* _CAMACQ_API_C_ */
;



/*****************************************************************/
/*********************** N I G H T  M O D E **********************/
/*****************************************************************/

//==========================================================
// CAMERA NIGHTMODE ON
//==========================================================
GLOBAL const U16 reg_main_nightshot_on[][2]
#if defined(_CAMACQ_API_C_)
={

CAMACQ_MAIN_BTM_OF_DATA
}
#endif /* _CAMACQ_API_C_ */
;

//==========================================================
// CAMERA NIGHTMODE OFF
//==========================================================
GLOBAL const U16 reg_main_nightshot_off[][2]
#if defined(_CAMACQ_API_C_)
={

CAMACQ_MAIN_BTM_OF_DATA
}
#endif /* _CAMACQ_API_C_ */
;

//==========================================================
// NIGHT-MODE SNAPSHOT
//==========================================================
GLOBAL const U16 reg_main_nightshot[][2]
#if defined(_CAMACQ_API_C_)
={
{0xFCFC, 0xD000},
{0x0028, 0x7000},
                 
{0x002A, 0x0D1E},
{0x0F12, 0xA102},
                 
{0x002A, 0x0210},
{0x0F12, 0x0000},
{0x002A, 0x01F4},
{0x0F12, 0x0001},
{0x002A, 0x0212},
{0x0F12, 0x0001},
{0x002A, 0x01E8},
{0x0F12, 0x0001},
{0x0F12, 0x0001},
                 
//{0xFFFF, 0x012C},  //moved delay after writing night i2c value to fix broken af sound in fireworks/night mode

CAMACQ_MAIN_BTM_OF_DATA
}
#endif /* _CAMACQ_API_C_ */
;

//==========================================================
// CAMERA_WB_AUTO (1/10)
//==========================================================
GLOBAL const U16 reg_main_wb_auto[][2]
#if defined(_CAMACQ_API_C_)
={
{0xFCFC, 0xD000},
{0x0028, 0x7000},
{0x002A, 0x2A62},
{0x0F12, 0x0001},		//Mon_AAIO_bAWB		AWB ON

CAMACQ_MAIN_BTM_OF_DATA
}
#endif /* _CAMACQ_API_C_ */
;

//==========================================================
// CAMERA_WB_DAYLIGHT (2/10)
//==========================================================
GLOBAL const U16 reg_main_wb_daylight[][2]
#if defined(_CAMACQ_API_C_)
={
{0xFCFC, 0xD000},
{0x0028, 0x7000},
{0x002A, 0x2A62},
{0x0F12, 0x0000},		//Mon_AAIO_bAWB		AWB OFF

{0x002A, 0x0470},
{0x0F12, 0x0620},		//REG_SF_USER_Rgain
{0x0F12, 0x0001},		//REG_SF_USER_RgainChanged
{0x0F12, 0x0400},		//REG_SF_USER_Ggain
{0x0F12, 0x0001},		//REG_SF_USER_GgainChanged
{0x0F12, 0x0540},		//REG_SF_USER_Bgain
{0x0F12, 0x0001},		//REG_SF_USER_BgainChaged

CAMACQ_MAIN_BTM_OF_DATA
}
#endif /* _CAMACQ_API_C_ */
;

//==========================================================
// CAMERA_WB_CLOUDY  (3/10)
//==========================================================
GLOBAL const U16 reg_main_wb_cloudy[][2]
#if defined(_CAMACQ_API_C_)
={
{0xFCFC, 0xD000},
{0x0028, 0x7000},
{0x002A, 0x2A62},
{0x0F12, 0x0000},	

{0x002A, 0x0470},
{0x0F12, 0x0780}, 
{0x0F12, 0x0001},	
{0x0F12, 0x0400},	
{0x0F12, 0x0001},	
{0x0F12, 0x0470}, 
{0x0F12, 0x0001},	

CAMACQ_MAIN_BTM_OF_DATA
}
#endif /* _CAMACQ_API_C_ */
;

//==========================================================
// CAMERA_WB_INCANDESCENT (4/10)
//==========================================================
GLOBAL const U16 reg_main_wb_incandescent[][2]
#if defined(_CAMACQ_API_C_)
={
{0xFCFC, 0xD000},
{0x0028, 0x7000},
{0x002A, 0x2A62},
{0x0F12, 0x0000},		//Mon_AAIO_bAWB		AWB OFF

{0x002A, 0x0470},
{0x0F12, 0x03C0},		//REG_SF_USER_Rgain
{0x0F12, 0x0001},		//REG_SF_USER_RgainChanged
{0x0F12, 0x0400},		//REG_SF_USER_Ggain
{0x0F12, 0x0001},		//REG_SF_USER_GgainChanged
{0x0F12, 0x0980},		//REG_SF_USER_Bgain
{0x0F12, 0x0001},		//REG_SF_USER_BgainChaged

CAMACQ_MAIN_BTM_OF_DATA
}
#endif /* _CAMACQ_API_C_ */
;

//==========================================================
// CAMERA_WB_FLUORESCENT (5/10)
//==========================================================
GLOBAL const U16 reg_main_wb_fluorescent[][2]
#if defined(_CAMACQ_API_C_)
={
{0xFCFC, 0xD000},
{0x0028, 0x7000},
{0x002A, 0x2A62},
{0x0F12, 0x0000},		//Mon_AAIO_bAWB		AWB OFF

{0x002A, 0x0470},
{0x0F12, 0x0560},		//REG_SF_USER_Rgain
{0x0F12, 0x0001},		//REG_SF_USER_RgainChanged
{0x0F12, 0x0400},		//REG_SF_USER_Ggain
{0x0F12, 0x0001},		//REG_SF_USER_GgainChanged
{0x0F12, 0x08A0},		//REG_SF_USER_Bgain
{0x0F12, 0x0001},		//REG_SF_USER_BgainChaged

CAMACQ_MAIN_BTM_OF_DATA
}
#endif /* _CAMACQ_API_C_ */
;

//==========================================================
// CAMERA_WB_FLUORESCENT (6/10)
//==========================================================
GLOBAL const U16 reg_main_wb_twilight[][2]
#if defined(_CAMACQ_API_C_)
={

CAMACQ_MAIN_BTM_OF_DATA
}
#endif /* _CAMACQ_API_C_ */
;

//==========================================================
// CAMERA_WB_FLUORESCENT (7/10)
//==========================================================
GLOBAL const U16 reg_main_wb_tungsten[][2]
#if defined(_CAMACQ_API_C_)
={

CAMACQ_MAIN_BTM_OF_DATA
}
#endif /* _CAMACQ_API_C_ */
;

//==========================================================
// CAMERA_WB_FLUORESCENT (8/10)
//==========================================================
GLOBAL const U16 reg_main_wb_off[][2]
#if defined(_CAMACQ_API_C_)
={

CAMACQ_MAIN_BTM_OF_DATA
}
#endif /* _CAMACQ_API_C_ */
;

//==========================================================
// CAMERA_WB_FLUORESCENT (9/10)
//==========================================================
GLOBAL const U16 reg_main_wb_horizon[][2]
#if defined(_CAMACQ_API_C_)
={

CAMACQ_MAIN_BTM_OF_DATA
}
#endif /* _CAMACQ_API_C_ */
;

//==========================================================
// CAMERA_WB_FLUORESCENT (10/10)
//==========================================================
GLOBAL const U16 reg_main_wb_shade[][2]
#if defined(_CAMACQ_API_C_)
={

CAMACQ_MAIN_BTM_OF_DATA
}
#endif /* _CAMACQ_API_C_ */
;


//==========================================================
// CAMERA_EFFECT_OFF (1/13)
//==========================================================
GLOBAL const U16 reg_main_effect_none[][2]
#if defined(_CAMACQ_API_C_)
={
{0xFCFC, 0xD000},
{0x0028, 0x7000},

{0x002A, 0x0648},
{0x0F12, 0x0001},		//skl_af_bPregmOff	Pre/Post Gamma Off (ฟ๘บน)  
  
{0x002A, 0x01E2},
{0x0F12, 0x0000},		//REG_TC_GP_SpecialEffects	00:Normal Mode

CAMACQ_MAIN_BTM_OF_DATA
}
#endif /* _CAMACQ_API_C_ */
;

//==========================================================
// CAMERA_EFFECT_MONO (2/13)
//==========================================================
GLOBAL const U16 reg_main_effect_gray[][2]
#if defined(_CAMACQ_API_C_)
={
{0xFCFC, 0xD000},
{0x0028, 0x7000},
{0x002A, 0x01E2},
{0x0F12, 0x0001},		//REG_TC_GP_SpecialEffects	01:Mono Mode
CAMACQ_MAIN_BTM_OF_DATA
}
#endif /* _CAMACQ_API_C_ */
;

//==========================================================
// CAMERA_EFFECT_NEGATIVE (3/13)
//==========================================================
GLOBAL const U16 reg_main_effect_negative[][2]
#if defined(_CAMACQ_API_C_)
={
{0xFCFC, 0xD000},
{0x0028, 0x7000},
{0x002A, 0x01E2},
{0x0F12, 0x0003},		//REG_TC_GP_SpecialEffects	03:Negative Mode

CAMACQ_MAIN_BTM_OF_DATA
}
#endif /* _CAMACQ_API_C_ */
;

//==========================================================
// CAMERA_EFFECT_SOLARIZE (4/13)
//==========================================================
GLOBAL const U16 reg_main_effect_solarize[][2]
#if defined(_CAMACQ_API_C_)
={

CAMACQ_MAIN_BTM_OF_DATA
}
#endif /* _CAMACQ_API_C_ */
;

//==========================================================
// CAMERA_EFFECT_SEPIA (5/13)
//==========================================================
GLOBAL const U16 reg_main_effect_sepia[][2]
#if defined(_CAMACQ_API_C_)
={
{0xFCFC, 0xD000},
{0x0028, 0x7000},
{0x002A, 0x01E2},
{0x0F12, 0x0004},		//REG_TC_GP_SpecialEffects	04:Sepia Mode

CAMACQ_MAIN_BTM_OF_DATA
}
#endif /* _CAMACQ_API_C_ */
;

//==========================================================
// CAMERA_EFFECT_POSTERIZE (6/13)
//==========================================================
GLOBAL const U16 reg_main_effect_posterize[][2]
#if defined(_CAMACQ_API_C_)
={

CAMACQ_MAIN_BTM_OF_DATA
}
#endif /* _CAMACQ_API_C_ */
;

//==========================================================
// CAMERA_EFFECT_WHITEBOARD (7/13)
//==========================================================
GLOBAL const U16 reg_main_effect_whiteboard[][2]
#if defined(_CAMACQ_API_C_)
={

CAMACQ_MAIN_BTM_OF_DATA
}
#endif /* _CAMACQ_API_C_ */
;

//==========================================================
// CAMERA_EFFECT_BLACKBOARD (8/13)
//==========================================================
GLOBAL const U16 reg_main_effect_blackboard[][2]
#if defined(_CAMACQ_API_C_)
={

CAMACQ_MAIN_BTM_OF_DATA
}
#endif /* _CAMACQ_API_C_ */
;

//==========================================================
// CAMERA_EFFECT_AQUA (9/13)
//==========================================================
GLOBAL const U16 reg_main_effect_aqua[][2]
#if defined(_CAMACQ_API_C_)
={

CAMACQ_MAIN_BTM_OF_DATA
}
#endif /* _CAMACQ_API_C_ */
;

//==========================================================
// CAMERA_EFFECT_SHARPEN (10/13)
//==========================================================
GLOBAL const U16 reg_main_effect_sharpen[][2]
#if defined(_CAMACQ_API_C_)
={

CAMACQ_MAIN_BTM_OF_DATA
}
#endif /* _CAMACQ_API_C_ */
;

//==========================================================
// CAMERA_EFFECT_AQUA (11/13)
//==========================================================
GLOBAL const U16 reg_main_effect_vivid[][2]
#if defined(_CAMACQ_API_C_)
={

CAMACQ_MAIN_BTM_OF_DATA
}
#endif /* _CAMACQ_API_C_ */
;

//==========================================================
// CAMERA_EFFECT_GREEN (12/13)
//==========================================================
GLOBAL const U16 reg_main_effect_green[][2]
#if defined(_CAMACQ_API_C_)
={

CAMACQ_MAIN_BTM_OF_DATA
}
#endif /* _CAMACQ_API_C_ */
;

//==========================================================
// CAMERA_EFFECT_SKETCH (13/13)
//==========================================================
GLOBAL const U16 reg_main_effect_sketch[][2]
#if defined(_CAMACQ_API_C_)
={

CAMACQ_MAIN_BTM_OF_DATA
}
#endif /* _CAMACQ_API_C_ */
;


//==========================================================
// CAMERA_AEC_MATRIX_METERING (2/2)
//==========================================================
GLOBAL const U16 reg_main_meter_matrix[][2]
#if defined(_CAMACQ_API_C_)
={
{0xFCFC, 0xD000}, 
{0x0028, 0x7000}, 
{0x002A, 0x1316},	
                  
{0x0F12, 0x0101}, 
{0x0F12, 0x0101}, 
{0x0F12, 0x0101}, 
{0x0F12, 0x0101}, 
{0x0F12, 0x0101}, 
{0x0F12, 0x0101}, 
{0x0F12, 0x0101}, 
{0x0F12, 0x0101}, 
{0x0F12, 0x0101}, 
{0x0F12, 0x0101}, 
{0x0F12, 0x0101}, 
{0x0F12, 0x0101}, 
{0x0F12, 0x0101}, 
{0x0F12, 0x0101}, 
{0x0F12, 0x0101}, 
{0x0F12, 0x0101}, 
{0x0F12, 0x0101}, 
{0x0F12, 0x0101}, 
{0x0F12, 0x0101}, 
{0x0F12, 0x0101}, 
{0x0F12, 0x0101}, 
{0x0F12, 0x0101}, 
{0x0F12, 0x0101}, 
{0x0F12, 0x0101}, 
{0x0F12, 0x0101}, 
{0x0F12, 0x0101}, 
{0x0F12, 0x0101}, 
{0x0F12, 0x0101}, 
{0x0F12, 0x0101}, 
{0x0F12, 0x0101}, 
{0x0F12, 0x0101}, 
{0x0F12, 0x0101}, 

CAMACQ_MAIN_BTM_OF_DATA
}
#endif /* _CAMACQ_API_C_ */
;

//==========================================================
// CAMERA_AEC_CENTERWEIGHTED_METERING (2/2)
//==========================================================
GLOBAL const U16 reg_main_meter_cw[][2]
#if defined(_CAMACQ_API_C_)
={
{0xFCFC, 0xD000}, 
{0x0028, 0x7000}, 
{0x002A, 0x1316},	
                  
{0x0F12, 0x0000}, 
{0x0F12, 0x0000}, 
{0x0F12, 0x0000}, 
{0x0F12, 0x0000}, 
{0x0F12, 0x0101}, 
{0x0F12, 0x0101}, 
{0x0F12, 0x0101}, 
{0x0F12, 0x0101}, 
{0x0F12, 0x0101}, 
{0x0F12, 0x0201}, 
{0x0F12, 0x0102}, 
{0x0F12, 0x0101}, 
{0x0F12, 0x0101}, 
{0x0F12, 0x0202}, 
{0x0F12, 0x0202}, 
{0x0F12, 0x0101}, 
{0x0F12, 0x0101}, 
{0x0F12, 0x0202}, 
{0x0F12, 0x0202}, 
{0x0F12, 0x0101}, 
{0x0F12, 0x0201}, 
{0x0F12, 0x0202}, 
{0x0F12, 0x0202}, 
{0x0F12, 0x0102}, 
{0x0F12, 0x0201}, 
{0x0F12, 0x0202}, 
{0x0F12, 0x0202}, 
{0x0F12, 0x0102}, 
{0x0F12, 0x0101}, 
{0x0F12, 0x0101}, 
{0x0F12, 0x0101}, 
{0x0F12, 0x0101}, 

CAMACQ_MAIN_BTM_OF_DATA
}
#endif /* _CAMACQ_API_C_ */
;

//==========================================================
// CAMERA_AEC_SPOT_METERING (1/2)
//==========================================================
GLOBAL const U16 reg_main_meter_spot[][2]
#if defined(_CAMACQ_API_C_)
={
{0xFCFC, 0xD000},                                
{0x0028, 0x7000},                                
{0x002A, 0x1316},
                                                 
{0x0F12, 0x0000},                                
{0x0F12, 0x0000},                                
{0x0F12, 0x0000},                                
{0x0F12, 0x0000},                                
{0x0F12, 0x0000},                                
{0x0F12, 0x0000},                                
{0x0F12, 0x0000},                                
{0x0F12, 0x0000},                                
{0x0F12, 0x0000},                                
{0x0F12, 0x0101},                                
{0x0F12, 0x0101},                                
{0x0F12, 0x0000},                                
{0x0F12, 0x0000},                                
{0x0F12, 0x0f01},                                
{0x0F12, 0x010f},                                
{0x0F12, 0x0000},                                
{0x0F12, 0x0000},                                
{0x0F12, 0x0f01},                                
{0x0F12, 0x010f},                                
{0x0F12, 0x0000},                                
{0x0F12, 0x0000},                                
{0x0F12, 0x0101},                                
{0x0F12, 0x0101},                                
{0x0F12, 0x0000},                                
{0x0F12, 0x0000},                                
{0x0F12, 0x0000},                                
{0x0F12, 0x0000},                                
{0x0F12, 0x0000},                                
{0x0F12, 0x0000},                                
{0x0F12, 0x0000},                                
{0x0F12, 0x0000},                                
{0x0F12, 0x0000},           

CAMACQ_MAIN_BTM_OF_DATA
}
#endif /* _CAMACQ_API_C_ */
;

//==========================================================
// CAMERA_AEC_FRAME_AVERAGE (2/2)
//==========================================================
GLOBAL const U16 reg_main_meter_frame[][2]
#if defined(_CAMACQ_API_C_)
={

CAMACQ_MAIN_BTM_OF_DATA
}
#endif /* _CAMACQ_API_C_ */
;

//==========================================================
// CAMERA_FLIP_NONE (1/4)
//==========================================================
GLOBAL const U16 reg_main_flip_none[][2]
#if defined(_CAMACQ_API_C_)
={

CAMACQ_MAIN_BTM_OF_DATA
}
#endif /* _CAMACQ_API_C_ */
;

//==========================================================
// CAMERA_FLIP_VERTICAL (2/4)
//==========================================================
GLOBAL const U16 reg_main_flip_water[][2]
#if defined(_CAMACQ_API_C_)
={

CAMACQ_MAIN_BTM_OF_DATA
}
#endif /* _CAMACQ_API_C_ */
;

//==========================================================
// CAMERA_FLIP_HORIZ (3/4)
//==========================================================
GLOBAL const U16 reg_main_flip_mirror[][2]
#if defined(_CAMACQ_API_C_)
={

CAMACQ_MAIN_BTM_OF_DATA
}
#endif /* _CAMACQ_API_C_ */
;


//==========================================================
// CAMERA_FLIP_SYMMETRIC (4/4)
//==========================================================
GLOBAL const U16 reg_main_flip_water_mirror[][2]
#if defined(_CAMACQ_API_C_)
={

CAMACQ_MAIN_BTM_OF_DATA
}
#endif /* _CAMACQ_API_C_ */
;

//==========================================================
// CAMERA_FRAMERATE_5FPS
//==========================================================
GLOBAL const U16 reg_main_fps_fixed_5[][2]
#if defined(_CAMACQ_API_C_)
={

CAMACQ_MAIN_BTM_OF_DATA
}
#endif /* _CAMACQ_API_C_ */
;


//==========================================================
// CAMERA_FRAMERATE_7FPS
//==========================================================
GLOBAL const U16 reg_main_fps_fixed_7[][2]
#if defined(_CAMACQ_API_C_)
={


CAMACQ_MAIN_BTM_OF_DATA
}
#endif /* _CAMACQ_API_C_ */
;

//==========================================================
// CAMERA_FRAMERATE_10FPS
//==========================================================
GLOBAL const U16 reg_main_fps_fixed_10[][2]
#if defined(_CAMACQ_API_C_)
={

CAMACQ_MAIN_BTM_OF_DATA
}
#endif /* _CAMACQ_API_C_ */
;

//==========================================================
// CAMERA_FRAMERATE_15FPS
//==========================================================
GLOBAL const U16 reg_main_fps_fixed_15[][2]
#if defined(_CAMACQ_API_C_)
={
{0xFCFC, 0xD000},
{0x0028, 0x7000},
{0x002A, 0x025A},
{0x0F12, 0x029A},
{0x0F12, 0x029A},
                 
{0x002A, 0x0208},
{0x0F12, 0x0000},
{0x002A, 0x020C},
{0x0F12, 0x0001},
{0x002A, 0x01F4},
{0x0F12, 0x0001},
{0x002A, 0x020A},
{0x0F12, 0x0001},
{0x002A, 0x01E8},
{0x0F12, 0x0000},
{0x0F12, 0x0001},

CAMACQ_MAIN_BTM_OF_DATA
}
#endif /* _CAMACQ_API_C_ */
;

//==========================================================
// CAMERA_FRAMERATE_20FPS
//==========================================================
GLOBAL const U16 reg_main_fps_fixed_20[][2]
#if defined(_CAMACQ_API_C_)
={


CAMACQ_MAIN_BTM_OF_DATA
}
#endif /* _CAMACQ_API_C_ */
;

//==========================================================
// CAMERA_FRAMERATE_25FPS
//==========================================================
GLOBAL const U16 reg_main_fps_fixed_25[][2]
#if defined(_CAMACQ_API_C_)
={
#if 1  //for 26fps
{0xFCFC, 0xD000},
{0x0028, 0x7000},
{0x002A, 0x025A},
{0x0F12, 0x0180},
{0x0F12, 0x0180},
                 
{0x002A, 0x0208},
{0x0F12, 0x0000},
{0x002A, 0x020C},
{0x0F12, 0x0001},
{0x002A, 0x01F4},
{0x0F12, 0x0001},
{0x002A, 0x020A},
{0x0F12, 0x0001},
{0x002A, 0x01E8},
{0x0F12, 0x0000},
{0x0F12, 0x0001},

#else
{0xFCFC, 0xD000},
{0x0028, 0x7000},
{0x002A, 0x025A},
{0x0F12, 0x0198},
{0x0F12, 0x0198},

{0x002A, 0x0208},
{0x0F12, 0x0000},
{0x002A, 0x020C},
{0x0F12, 0x0001},
{0x002A, 0x01F4},
{0x0F12, 0x0001},
{0x002A, 0x020A},
{0x0F12, 0x0001},
{0x002A, 0x01E8},
{0x0F12, 0x0000},
{0x0F12, 0x0001},
#endif
CAMACQ_MAIN_BTM_OF_DATA
}
#endif /* _CAMACQ_API_C_ */
;

//==========================================================
// CAMERA_FRAMERATE_30FPS
//==========================================================
GLOBAL const U16 reg_main_fps_fixed_30[][2]
#if defined(_CAMACQ_API_C_)
={
{0xFCFC, 0xD000},
{0x0028, 0x7000},
{0x002A, 0x025A},
{0x0F12, 0x0168},
{0x0F12, 0x0168},
                 
{0x002A, 0x0208},
{0x0F12, 0x0000},
{0x002A, 0x020C},
{0x0F12, 0x0001},
{0x002A, 0x01F4},
{0x0F12, 0x0001},
{0x002A, 0x020A},
{0x0F12, 0x0001},
{0x002A, 0x01E8},
{0x0F12, 0x0000},
{0x0F12, 0x0001},

CAMACQ_MAIN_BTM_OF_DATA
}
#endif /* _CAMACQ_API_C_ */
;

//==========================================================
// CAMERA_FRAMERATE_AUTO_MAX15(5~15fps)
//==========================================================
GLOBAL const U16 reg_main_fps_var_15[][2]
#if defined(_CAMACQ_API_C_)
={

CAMACQ_MAIN_BTM_OF_DATA
}
#endif /* _CAMACQ_API_C_ */
;


//==========================================================
// CAMERA_BRIGHTNESS_LEVEL1 (1/9) : 
//==========================================================
GLOBAL const U16 reg_main_brightness_level_0[][2]
#if defined(_CAMACQ_API_C_)
={
{0xFCFC, 0xD000},                                                                                     
{0x0028, 0x7000},                                                                                     
                                                                                                      
{0x002A, 0x01D0},                                                                                     
{0x0F12, 0xFF80},		//REG_TC_UserBrightness      

CAMACQ_MAIN_BTM_OF_DATA
}
#endif /* _CAMACQ_API_C_ */
;

//==========================================================
// CAMERA_BRIGHTNESS_LEVEL2 (2/9)
//==========================================================
GLOBAL const U16 reg_main_brightness_level_1[][2]
#if defined(_CAMACQ_API_C_)
={
{0xFCFC, 0xD000},                                                                                    
{0x0028, 0x7000},                                                                                    
                                                                                                     
{0x002A, 0x01D0},                                                                                    
{0x0F12, 0xFFBC},		//REG_TC_UserBrightness                                                          

CAMACQ_MAIN_BTM_OF_DATA
}
#endif /* _CAMACQ_API_C_ */
;

//==========================================================
// CAMERA_BRIGHTNESS_LEVEL3 (3/9)
//==========================================================
GLOBAL const U16 reg_main_brightness_level_2[][2]
#if defined(_CAMACQ_API_C_)
={
{0xFCFC, 0xD000},                                                                                     
{0x0028, 0x7000},                                                                                     
                                                                                                      
{0x002A, 0x01D0},                                                                                     
{0x0F12, 0xFFDC},		//REG_TC_UserBrightness               

CAMACQ_MAIN_BTM_OF_DATA
}
#endif /* _CAMACQ_API_C_ */
;

//==========================================================
// CAMERA_BRIGHTNESS_LEVEL4 (4/9)
//==========================================================
GLOBAL const U16 reg_main_brightness_level_3[][2]
#if defined(_CAMACQ_API_C_)
={
{0xFCFC, 0xD000},                                                                                    
{0x0028, 0x7000},                                                                                    
                                                                                                     
{0x002A, 0x01D0},                                                                                    
{0x0F12, 0xFFF2},		//REG_TC_UserBrightness                                               

CAMACQ_MAIN_BTM_OF_DATA
}
#endif /* _CAMACQ_API_C_ */
;

//==========================================================
// CAMERA_BRIGHTNESS_LEVEL5 (5/9)
//==========================================================
GLOBAL const U16 reg_main_brightness_level_4[][2]
#if defined(_CAMACQ_API_C_)
={
{0xFCFC, 0xD000},                                                                                    
{0x0028, 0x7000},                                                                                    
                                                                                                     
{0x002A, 0x01D0},                                                                                    
{0x0F12, 0x0000},		//REG_TC_UserBrightness                                                

CAMACQ_MAIN_BTM_OF_DATA
}
#endif /* _CAMACQ_API_C_ */
;

//==========================================================
// CAMERA_BRIGHTNESS_LEVEL6 (6/9)
//==========================================================
GLOBAL const U16 reg_main_brightness_level_5[][2]
#if defined(_CAMACQ_API_C_)
={
{0xFCFC, 0xD000},                                                                                    
{0x0028, 0x7000},                                                                                    
                                                                                                     
{0x002A, 0x01D0},                                                                                    
{0x0F12, 0x0020},		//REG_TC_UserBrightness 

CAMACQ_MAIN_BTM_OF_DATA
}
#endif /* _CAMACQ_API_C_ */
;

//==========================================================
// CAMERA_BRIGHTNESS_LEVEL7 (7/9)
//==========================================================
GLOBAL const U16 reg_main_brightness_level_6[][2]
#if defined(_CAMACQ_API_C_)
={
{0xFCFC, 0xD000},                                                                                    
{0x0028, 0x7000},                                                                                    
                                                                                                     
{0x002A, 0x01D0},                                                                                    
{0x0F12, 0x0040},		//REG_TC_UserBrightness  

CAMACQ_MAIN_BTM_OF_DATA
}
#endif /* _CAMACQ_API_C_ */
;

//==========================================================
// CAMERA_BRIGHTNESS_LEVEL8 (8/9)
//==========================================================
GLOBAL const U16 reg_main_brightness_level_7[][2]
#if defined(_CAMACQ_API_C_)
={
{0xFCFC, 0xD000},                                                                                    
{0x0028, 0x7000},                                                                                    
                                                                                                     
{0x002A, 0x01D0},                                                                                    
{0x0F12, 0x0060},		//REG_TC_UserBrightness

CAMACQ_MAIN_BTM_OF_DATA
}
#endif /* _CAMACQ_API_C_ */
;

//==========================================================
// CAMERA_BRIGHTNESS_LEVEL9 (9/9)
//==========================================================
GLOBAL const U16 reg_main_brightness_level_8[][2]
#if defined(_CAMACQ_API_C_)
={
{0xFCFC, 0xD000},                                                                                    
{0x0028, 0x7000},                                                                                    
                                                                                                     
{0x002A, 0x01D0},                                                                                    
{0x0F12, 0x0080},		//REG_TC_UserBrightness                           

CAMACQ_MAIN_BTM_OF_DATA
}
#endif /* _CAMACQ_API_C_ */
;

//==========================================================
// CAMERA_EXPOSURE_COMPENSATION_LEVEL1 (1/9) : 
//==========================================================
GLOBAL const U16 reg_main_expcompensation_level_0[][2]
#if defined(_CAMACQ_API_C_)
={
{0xFCFC, 0xD000},     
{0x0028, 0x7000},     
                      
{0x002A, 0x1308},     
{0x0F12, 0x001A},   

CAMACQ_MAIN_BTM_OF_DATA
}
#endif /* _CAMACQ_API_C_ */
;

//==========================================================
// CAMERA_EXPOSURE_COMPENSATION_LEVEL2 (2/9)
//==========================================================
GLOBAL const U16 reg_main_expcompensation_level_1[][2]
#if defined(_CAMACQ_API_C_)
={
{0xFCFC, 0xD000},
{0x0028, 0x7000},
                 
{0x002A, 0x1308},
{0x0F12, 0x001F},
                    
CAMACQ_MAIN_BTM_OF_DATA
}
#endif /* _CAMACQ_API_C_ */
;

//==========================================================
// CAMERA_EXPOSURE_COMPENSATION_LEVEL3 (3/9)
//==========================================================
GLOBAL const U16 reg_main_expcompensation_level_2[][2]
#if defined(_CAMACQ_API_C_)
={
{0xFCFC, 0xD000},                
{0x0028, 0x7000},                
                                 
{0x002A, 0x1308},
{0x0F12, 0x0028},                 

CAMACQ_MAIN_BTM_OF_DATA
}
#endif /* _CAMACQ_API_C_ */
;

//==========================================================
// CAMERA_EXPOSURE_COMPENSATION_LEVEL4 (4/9)
//==========================================================
GLOBAL const U16 reg_main_expcompensation_level_3[][2]
#if defined(_CAMACQ_API_C_)
={
{0xFCFC, 0xD000},         
{0x0028, 0x7000},         
                          
{0x002A, 0x1308},
{0x0F12, 0x0032},                

CAMACQ_MAIN_BTM_OF_DATA
}
#endif /* _CAMACQ_API_C_ */
;

//==========================================================
// CAMERA_EXPOSURE_COMPENSATION_LEVEL5 (5/9)
//==========================================================
GLOBAL const U16 reg_main_expcompensation_level_4[][2]
#if defined(_CAMACQ_API_C_)
={
{0xFCFC, 0xD000},               
{0x0028, 0x7000},               
                                
{0x002A, 0x1308},
{0x0F12, 0x003E}, 

CAMACQ_MAIN_BTM_OF_DATA
}
#endif /* _CAMACQ_API_C_ */
;

//==========================================================
// CAMERA_EXPOSURE_COMPENSATION_LEVEL6 (6/9)
//==========================================================
GLOBAL const U16 reg_main_expcompensation_level_5[][2]
#if defined(_CAMACQ_API_C_)
={
{0xFCFC, 0xD000},               
{0x0028, 0x7000},               
                                
{0x002A, 0x1308},
{0x0F12, 0x004A},  

CAMACQ_MAIN_BTM_OF_DATA
}
#endif /* _CAMACQ_API_C_ */
;

//==========================================================
// CAMERA_EXPOSURE_COMPENSATION_LEVEL7 (7/9)
//==========================================================
GLOBAL const U16 reg_main_expcompensation_level_6[][2]
#if defined(_CAMACQ_API_C_)
={
{0xFCFC, 0xD000},                  
{0x0028, 0x7000},                  
                                   
{0x002A, 0x1308},
{0x0F12, 0x0065},  

CAMACQ_MAIN_BTM_OF_DATA
}
#endif /* _CAMACQ_API_C_ */
;

//==========================================================
// CAMERA_EXPOSURE_COMPENSATION_LEVEL8 (8/9)
//==========================================================
GLOBAL const U16 reg_main_expcompensation_level_7[][2]
#if defined(_CAMACQ_API_C_)
={
{0xFCFC, 0xD000},                 
{0x0028, 0x7000},                 
                                  
{0x002A, 0x1308},
{0x0F12, 0x0075},

CAMACQ_MAIN_BTM_OF_DATA
}
#endif /* _CAMACQ_API_C_ */
;

//==========================================================
// CAMERA_EXPOSURE_COMPENSATION_LEVEL9 (9/9)
//==========================================================
GLOBAL const U16 reg_main_expcompensation_level_8[][2]
#if defined(_CAMACQ_API_C_)
={
{0xFCFC, 0xD000},
{0x0028, 0x7000},
                 
{0x002A, 0x1308},
{0x0F12, 0x008B},

CAMACQ_MAIN_BTM_OF_DATA
}
#endif /* _CAMACQ_API_C_ */
;

//==========================================================
// CAMERA_AF
//==========================================================
GLOBAL const U16 reg_main_reset_af [][2]
#if defined(_CAMACQ_API_C_)
={

CAMACQ_MAIN_BTM_OF_DATA
}
#endif /* _CAMACQ_API_C_ */
;

GLOBAL const U16 reg_main_cancel_macro_af [][2]
#if defined(_CAMACQ_API_C_)
={

CAMACQ_MAIN_BTM_OF_DATA
}
#endif /* _CAMACQ_API_C_ */
;

GLOBAL const U16 reg_main_cancel_manual_af [][2]
#if defined(_CAMACQ_API_C_)
={

CAMACQ_MAIN_BTM_OF_DATA
}
#endif /* _CAMACQ_API_C_ */
;

GLOBAL const U16 reg_main_set_af_nlux [][2]
#if defined(_CAMACQ_API_C_)
={

CAMACQ_MAIN_BTM_OF_DATA
}
#endif /* _CAMACQ_API_C_ */
;

GLOBAL const U16 reg_main_set_af_llux [][2]
#if defined(_CAMACQ_API_C_)
={

CAMACQ_MAIN_BTM_OF_DATA
}
#endif /* _CAMACQ_API_C_ */
;

GLOBAL const U16 reg_main_set_af[][2] // start_af
#if defined(_CAMACQ_API_C_)
={
{0xFCFC, 0xD000},
{0x0028, 0x7000},
{0x002A, 0x0224},
{0x0F12, 0x0005},

CAMACQ_MAIN_BTM_OF_DATA
}
#endif /* _CAMACQ_API_C_ */
;

GLOBAL const U16 reg_main_off_af[][2]
#if defined(_CAMACQ_API_C_)
={

CAMACQ_MAIN_BTM_OF_DATA
}
#endif /* _CAMACQ_API_C_ */
;


GLOBAL const U16 reg_main_check_af[][2]
#if defined(_CAMACQ_API_C_)
={

CAMACQ_MAIN_BTM_OF_DATA
}
#endif /* _CAMACQ_API_C_ */
;

GLOBAL const U16 reg_main_manual_af[][2]
#if defined(_CAMACQ_API_C_)
={
{0xFCFC, 0xD000}, 
{0x0028, 0x7000}, 
{0x002A, 0x0226}, 
{0x0F12, 0x0000}, 
                  
{0xdddd, 0x0064}, 
                  
{0x002A, 0x0224}, 
{0x0F12, 0x0004}, 
                  
{0xdddd, 0x0096}, 
     
{0x002A, 0x1494}, 
{0x0F12, 0x1002}, 
CAMACQ_MAIN_BTM_OF_DATA
}
#endif /* _CAMACQ_API_C_ */
;

GLOBAL const U16 reg_main_macro_af[][2]
#if defined(_CAMACQ_API_C_)
={
{0xFCFC, 0xD000},
{0x0028, 0x7000},
{0x002A, 0x0226},
{0x0F12, 0x0095},
                 
{0xdddd, 0x0064},
                 
{0x002A, 0x0224},
{0x0F12, 0x0004},
                 
{0xdddd, 0x0096},
 
{0x002A, 0x1494},
{0x0F12, 0x1042},

{0x002A, 0x1426},
{0x0F12, 0x100A},
                 
CAMACQ_MAIN_BTM_OF_DATA
}
#endif /* _CAMACQ_API_C_ */
;

GLOBAL const U16 reg_main_return_manual_af[][2]
#if defined(_CAMACQ_API_C_)
={

CAMACQ_MAIN_BTM_OF_DATA
}
#endif /* _CAMACQ_API_C_ */
;

GLOBAL const U16 reg_main_return_macro_af[][2]
#if defined(_CAMACQ_API_C_)
={

CAMACQ_MAIN_BTM_OF_DATA
}
#endif /* _CAMACQ_API_C_ */
;

GLOBAL const U16 reg_main_set_af_normal_mode_1[][2]
#if defined(_CAMACQ_API_C_)
={

CAMACQ_MAIN_BTM_OF_DATA
}
#endif /* _CAMACQ_API_C_ */
;

GLOBAL const U16 reg_main_set_af_normal_mode_2[][2]
#if defined(_CAMACQ_API_C_)
={

CAMACQ_MAIN_BTM_OF_DATA
}
#endif /* _CAMACQ_API_C_ */
;

GLOBAL const U16 reg_main_set_af_normal_mode_3[][2]
#if defined(_CAMACQ_API_C_)
={

CAMACQ_MAIN_BTM_OF_DATA
}
#endif /* _CAMACQ_API_C_ */
;

GLOBAL const U16 reg_main_set_af_macro_mode_1[][2]
#if defined(_CAMACQ_API_C_)
={

CAMACQ_MAIN_BTM_OF_DATA
}
#endif /* _CAMACQ_API_C_ */
;

GLOBAL const U16 reg_main_set_af_macro_mode_2[][2]
#if defined(_CAMACQ_API_C_)
={

CAMACQ_MAIN_BTM_OF_DATA
}
#endif /* _CAMACQ_API_C_ */
;

GLOBAL const U16 reg_main_set_af_macro_mode_3[][2]
#if defined(_CAMACQ_API_C_)
={

CAMACQ_MAIN_BTM_OF_DATA
}
#endif /* _CAMACQ_API_C_ */
;


//==========================================================
// CAMERA_ISO_AUTO
//==========================================================
GLOBAL const U16 reg_main_iso_auto[][2]
#if defined(_CAMACQ_API_C_)
={
{0xFCFC, 0xD000},

CAMACQ_MAIN_BTM_OF_DATA
}
#endif /* _CAMACQ_API_C_ */
;


//==========================================================
// CAMERA_ISO_50
//==========================================================
GLOBAL const U16 reg_main_iso_50[][2]
#if defined(_CAMACQ_API_C_)
={
{0xFCFC, 0xD000},

CAMACQ_MAIN_BTM_OF_DATA
}
#endif /* _CAMACQ_API_C_ */
;


//==========================================================
// CAMERA_ISO_100
//==========================================================
GLOBAL const U16 reg_main_iso_100[][2]
#if defined(_CAMACQ_API_C_)
={
{0xFCFC, 0xD000},

CAMACQ_MAIN_BTM_OF_DATA
}
#endif /* _CAMACQ_API_C_ */
;


//==========================================================
// CAMERA_ISO_200
//==========================================================
GLOBAL const U16 reg_main_iso_200[][2]
#if defined(_CAMACQ_API_C_)
={
{0xFCFC, 0xD000},

CAMACQ_MAIN_BTM_OF_DATA
}
#endif /* _CAMACQ_API_C_ */
;

//==========================================================
// CAMERA_ISO_400
//==========================================================
GLOBAL const U16 reg_main_iso_400[][2]
#if defined(_CAMACQ_API_C_)
={
{0xFCFC, 0xD000},

CAMACQ_MAIN_BTM_OF_DATA
}
#endif /* _CAMACQ_API_C_ */
;

//==========================================================
// CAMERA_ISO_800
//==========================================================
GLOBAL const U16 reg_main_iso_800[][2]
#if defined(_CAMACQ_API_C_)
={

CAMACQ_MAIN_BTM_OF_DATA
}
#endif /* _CAMACQ_API_C_ */
;

//==========================================================
// CAMERA_ISO_1600
//==========================================================
GLOBAL const U16 reg_main_iso_1600[][2]
#if defined(_CAMACQ_API_C_)
={

CAMACQ_MAIN_BTM_OF_DATA
}
#endif /* _CAMACQ_API_C_ */
;

//==========================================================
// CAMERA_ISO_3200
//==========================================================
GLOBAL const U16 reg_main_iso_3200[][2]
#if defined(_CAMACQ_API_C_)
={

CAMACQ_MAIN_BTM_OF_DATA
}
#endif /* _CAMACQ_API_C_ */
;

//==========================================================
// CAMERA_SCENE_AUTO (OFF)
//==========================================================
GLOBAL const U16 reg_main_scene_auto[][2]
#if defined(_CAMACQ_API_C_)
={
///////////////////add for effect off///////////////////////////////////////////	
	{0xFCFC, 0xD000},
{0x0028, 0x7000},

{0x002A, 0x0648},
{0x0F12, 0x0001},		//skl_af_bPregmOff	Pre/Post Gamma Off 
  
{0x002A, 0x01E2},
{0x0F12, 0x0000},		//REG_TC_GP_SpecialEffects	00:Normal Mode
///////////////////////////////////////////////////////////////////
{0xFCFC, 0xD000},
{0x0028, 0x7000},
{0x002A, 0x2A62},
{0x0F12, 0x0001},

{0xFCFC, 0xD000},
{0x0028, 0x7000},
{0x002A, 0x1316},

{0x0F12, 0x0000},
{0x0F12, 0x0000},
{0x0F12, 0x0000},
{0x0F12, 0x0000},
{0x0F12, 0x0101},
{0x0F12, 0x0101},
{0x0F12, 0x0101},
{0x0F12, 0x0101},
{0x0F12, 0x0101},
{0x0F12, 0x0201},
{0x0F12, 0x0102},
{0x0F12, 0x0101},
{0x0F12, 0x0101},
{0x0F12, 0x0202},
{0x0F12, 0x0202},
{0x0F12, 0x0101},
{0x0F12, 0x0101},
{0x0F12, 0x0202},
{0x0F12, 0x0202},
{0x0F12, 0x0101},
{0x0F12, 0x0201},
{0x0F12, 0x0202},
{0x0F12, 0x0202},
{0x0F12, 0x0102},
{0x0F12, 0x0201},
{0x0F12, 0x0202},
{0x0F12, 0x0202},
{0x0F12, 0x0102},
{0x0F12, 0x0101},
{0x0F12, 0x0101},
{0x0F12, 0x0101},
{0x0F12, 0x0101},

{0x0028, 0x7000},             
{0x002A, 0x1308},
{0x0F12, 0x003E},               
{0x002A, 0x01D0},
{0x0F12, 0x0000},		
{0x0F12, 0x0000},  	
{0x002A, 0x01D4},
{0x0F12, 0x0000},		
{0x0F12, 0x0000},		


{0x0028, 0x7000},
{0x002A, 0x025A},
{0x0F12, 0x03E8},
{0x0F12, 0x0154},

{0x002A, 0x034C},
{0x0F12, 0x0535},
{0x0F12, 0x029A},

{0x002A, 0x13F2},
{0x0F12, 0x0010},
{0x0F12, 0x0020},
{0x0F12, 0x0040},
{0x0F12, 0x0080},
{0x0F12, 0x0100},
{0x0F12, 0x0200},
{0x0F12, 0x0400},
{0x0F12, 0x0800},
{0x0F12, 0x2000},

{0x0F12, 0x0010},
{0x0F12, 0x0020},
{0x0F12, 0x0040},
{0x0F12, 0x0080},
{0x0F12, 0x0100},
{0x0F12, 0x0200},
{0x0F12, 0x0400},
{0x0F12, 0x0800},
{0x0F12, 0x2000},


{0x002A, 0x04EE},
{0x0F12, 0x010E},      
{0x0F12, 0x00F5},      

{0x0028, 0x7000},
{0x002A, 0x0504},
{0x0F12, 0x3415},
{0x002A, 0x0508},
{0x0F12, 0x681F},
{0x002A, 0x050C},
{0x0F12, 0x8227},
{0x002A, 0x0510},
{0x0F12, 0xC350},
{0x0F12, 0x0000},

{0x002A, 0x0514},
{0x0F12, 0x3415},
{0x002A, 0x0518},
{0x0F12, 0x681F},
{0x002A, 0x051C},
{0x0F12, 0x8227},
{0x002A, 0x0520},
{0x0F12, 0xC350},
{0x0F12, 0x0000},

{0x002A, 0x0524},
{0x0F12, 0x01D0},
{0x0F12, 0x01D0},
{0x0F12, 0x02C0},
{0x0F12, 0x0710},

{0x0F12, 0x0100},
{0x0F12, 0x8000},

{0x0F12, 0x01D0},
{0x0F12, 0x01D0},
{0x0F12, 0x02C0},
{0x0F12, 0x0710},

{0x0F12, 0x0100},
{0x0F12, 0x8000},

{0x002A, 0x08E4},
{0x0F12, 0x0010},
{0x0F12, 0x0010},
{0x002A, 0x0940},
{0x0F12, 0x0B50},

{0x002A, 0x04A4},
{0x0F12, 0x067F},
{0x002A, 0x048C},
{0x0F12, 0x0002},    //changed 0x0001 -> 0x0002 for 60hz flicker
{0x0F12, 0x0001},

{0x002A, 0x05EA},
{0x0F12, 0x0100},

{0x002A, 0x0486},
{0x0F12, 0x0000},
{0x002A, 0x048A},
{0x0F12, 0x0001},

{0x002A, 0x3302},
{0x0F12, 0x0000},

{0x002A, 0x0D1E},
{0x0F12, 0x2102},

{0x002A, 0x0208},
{0x0F12, 0x0000},
{0x002A, 0x0210},
{0x0F12, 0x0000},
{0x002A, 0x020C},
{0x0F12, 0x0001},
{0x002A, 0x01F4},
{0x0F12, 0x0001},
{0x002A, 0x020A},
{0x0F12, 0x0001},
{0x002A, 0x0212},
{0x0F12, 0x0001},	
{0x002A, 0x01E8},
{0x0F12, 0x0000},
{0x0F12, 0x0001},

{0xFFFF, 0x0064},

CAMACQ_MAIN_BTM_OF_DATA
}
#endif /* _CAMACQ_API_C_ */
;

//==========================================================
// CAMERA_SCENE_NIGHT
//==========================================================
GLOBAL const U16 reg_main_scene_night[][2]
#if defined(_CAMACQ_API_C_)
={
{0xFCFC, 0xD000},
{0x0028, 0x7000},  

{0x002A, 0x025A},
{0x0F12, 0x0682},	//6fps

{0x002A, 0x034C},
{0x0F12, 0x1388},	
{0x0F12, 0x1388},	


{0x002A, 0x0504},
{0x0F12, 0x3415},	
{0x002A, 0x0508},
{0x0F12, 0x681F},	
{0x002A, 0x050C},
{0x0F12, 0x8227},	
{0x002A, 0x0510},
{0x0F12, 0x1A80},	
{0x0F12, 0x0006},

{0x002A, 0x0514},
{0x0F12, 0x3415},	
{0x002A, 0x0518},
{0x0F12, 0x681F},	
{0x002A, 0x051C},
{0x0F12, 0x8227},	
{0x002A, 0x0520},
{0x0F12, 0x1A80},	
{0x0F12, 0x0006},

{0x002A, 0x0524},
{0x0F12, 0x01D0},	
{0x0F12, 0x01D0},	
{0x0F12, 0x02C0},	
{0x0F12, 0x0B00},	

{0x0F12, 0x0100},	
{0x0F12, 0x8000},	

{0x0F12, 0x01D0},	
{0x0F12, 0x01D0},	
{0x0F12, 0x02C0},	
{0x0F12, 0x0800},	

{0x0F12, 0x0100},	
{0x0F12, 0x8000},	

{0x002A, 0x08E4},
{0x0F12, 0x0000},	
{0x0F12, 0x0000},	
{0x002A, 0x0940},
{0x0F12, 0x1088},	

{0x002A, 0x0D1E},
{0x0F12, 0x2102},	


{0x002A, 0x0208},
{0x0F12, 0x0000},	
{0x002A, 0x0210},
{0x0F12, 0x0000},	
{0x002A, 0x020C},
{0x0F12, 0x0001},	
{0x002A, 0x01F4},
{0x0F12, 0x0001},	
{0x002A, 0x020A},
{0x0F12, 0x0001},	
{0x002A, 0x0212},
{0x0F12, 0x0001},	
{0x002A, 0x01E8},
{0x0F12, 0x0000},	
{0x0F12, 0x0001},	

CAMACQ_MAIN_BTM_OF_DATA
}
#endif /* _CAMACQ_API_C_ */
;

//==========================================================
// CAMERA_SCENE_LANDSCAPE
//==========================================================
GLOBAL const U16 reg_main_scene_landscape[][2]
#if defined(_CAMACQ_API_C_)
={
{0xFCFC, 0xD000},
{0x0028, 0x7000},
{0x002A, 0x01D0},
{0x0F12, 0x0000},
{0x002A, 0x01D4},
{0x0F12, 0x001E},
{0x0F12, 0x000A},   
       
{0x002A, 0x1316},        
                         
{0x0F12, 0x0101},        
{0x0F12, 0x0101},        
{0x0F12, 0x0101},        
{0x0F12, 0x0101},        
{0x0F12, 0x0101},        
{0x0F12, 0x0101},        
{0x0F12, 0x0101},        
{0x0F12, 0x0101},        
{0x0F12, 0x0101},        
{0x0F12, 0x0101},        
{0x0F12, 0x0101},        
{0x0F12, 0x0101},        
{0x0F12, 0x0101},        
{0x0F12, 0x0101},        
{0x0F12, 0x0101},        
{0x0F12, 0x0101},        
{0x0F12, 0x0101},             
{0x0F12, 0x0101},             
{0x0F12, 0x0101},             
{0x0F12, 0x0101},             
{0x0F12, 0x0101},             
{0x0F12, 0x0101},             
{0x0F12, 0x0101},             
{0x0F12, 0x0101},             
{0x0F12, 0x0101},             
{0x0F12, 0x0101},             
{0x0F12, 0x0101},             
{0x0F12, 0x0101},             
{0x0F12, 0x0101},        
{0x0F12, 0x0101},        
{0x0F12, 0x0101},        
{0x0F12, 0x0101},     

CAMACQ_MAIN_BTM_OF_DATA
}
#endif /* _CAMACQ_API_C_ */
;

//==========================================================
// CAMERA_SCENE_SUNSET
//==========================================================
GLOBAL const U16 reg_main_scene_sunset[][2]
#if defined(_CAMACQ_API_C_)
={
{0xFCFC, 0xD000},
{0x0028, 0x7000},
{0x002A, 0x2A62},
{0x0F12, 0x0000},
                 
{0x002A, 0x0470},
{0x0F12, 0x0600},
{0x0F12, 0x0001},
{0x0F12, 0x0400},
{0x0F12, 0x0001},
{0x0F12, 0x0526},
{0x0F12, 0x0001},

CAMACQ_MAIN_BTM_OF_DATA
}
#endif /* _CAMACQ_API_C_ */
;

//==========================================================
// CAMERA_SCENE_PORTRAIT
//==========================================================
GLOBAL const U16 reg_main_scene_portrait[][2]
#if defined(_CAMACQ_API_C_)
={
{0xFCFC, 0xD000}, 
{0x0028, 0x7000}, 
{0x002A, 0x01D0}, 
{0x0F12, 0x0000},	
{0x002A, 0x01D4}, 
{0x0F12, 0x0000},	
{0x0F12, 0xFFF6},	

CAMACQ_MAIN_BTM_OF_DATA
}
#endif /* _CAMACQ_API_C_ */
;

//==========================================================
// CAMERA_SCENE_SUNRISE //DAWN
//==========================================================
GLOBAL const U16 reg_main_scene_sunrise[][2]
#if defined(_CAMACQ_API_C_)
={
{0xFCFC, 0xD000},
{0x0028, 0x7000},
{0x002A, 0x2A62},
{0x0F12, 0x0000},
                 
{0x002A, 0x0470},
{0x0F12, 0x0560},
{0x0F12, 0x0001},
{0x0F12, 0x0400},
{0x0F12, 0x0001},
{0x0F12, 0x08A0},
{0x0F12, 0x0001},

CAMACQ_MAIN_BTM_OF_DATA
}
#endif /* _CAMACQ_API_C_ */
;

//==========================================================
// CAMERA_SCENE_INDOOR // == PARTY
//==========================================================
GLOBAL const U16 reg_main_scene_indoor[][2]
#if defined(_CAMACQ_API_C_)
={
{0xFCFC, 0xD000},
{0x0028, 0x7000},
{0x002A, 0x01D0},
{0x0F12, 0x0000},
{0x002A, 0x01D4},
{0x0F12, 0x001E},
{0x0F12, 0x0000},


{0x002A, 0x04EE},
{0x0F12, 0x0112},
{0x0F12, 0x00EE},

{0x002A, 0x052C},
{0x0F12, 0x0200},

{0x002A, 0x05EA},
{0x0F12, 0x0220},


{0x002A, 0x0486},
{0x0F12, 0x0003},
{0x0F12, 0x0300},
{0x0F12, 0x0001},


{0x002A, 0x3302},
{0x0F12, 0x0001},

{0x002A, 0x0D1E},
{0x0F12, 0x2102},


{0x002A, 0x0208},
{0x0F12, 0x0000},
{0x002A, 0x0210},
{0x0F12, 0x0000},
{0x002A, 0x020C},
{0x0F12, 0x0001},
{0x002A, 0x01F4},
{0x0F12, 0x0001},
{0x002A, 0x020A},
{0x0F12, 0x0001},
{0x002A, 0x0212},
{0x0F12, 0x0001},
{0x002A, 0x01E8},
{0x0F12, 0x0000},
{0x0F12, 0x0001},

CAMACQ_MAIN_BTM_OF_DATA
}
#endif /* _CAMACQ_API_C_ */
;

//==========================================================
// CAMERA_SCENE_PARTY // == INDOOR
//==========================================================
GLOBAL const U16 reg_main_scene_party[][2] 
#if defined(_CAMACQ_API_C_)
={
{0xFCFC, 0xD000},
{0x0028, 0x7000},
{0x002A, 0x01D0},
{0x0F12, 0x0000},	
{0x002A, 0x01D4},
{0x0F12, 0x001E},	
{0x0F12, 0x0000},	


{0x002A, 0x04EE},
{0x0F12, 0x0112},	
{0x0F12, 0x00EE},	

{0x002A, 0x052C},
{0x0F12, 0x0200},	

{0x002A, 0x05EA},
{0x0F12, 0x0220},	


{0x002A, 0x0486},
{0x0F12, 0x0003},	
{0x0F12, 0x0300},	
{0x0F12, 0x0001},	


{0x002A, 0x3302},
{0x0F12, 0x0001},	

{0x002A, 0x0D1E},
{0x0F12, 0x2102},	


{0x002A, 0x0208},
{0x0F12, 0x0000},	
{0x002A, 0x0210},
{0x0F12, 0x0000},	
{0x002A, 0x020C},
{0x0F12, 0x0001},	
{0x002A, 0x01F4},
{0x0F12, 0x0001},	
{0x002A, 0x020A},
{0x0F12, 0x0001},	
{0x002A, 0x0212},
{0x0F12, 0x0001},	
{0x002A, 0x01E8},
{0x0F12, 0x0000},	
{0x0F12, 0x0001},

CAMACQ_MAIN_BTM_OF_DATA
}
#endif /* _CAMACQ_API_C_ */
;

//==========================================================
// CAMERA_SCENE_SPORTS
//==========================================================
GLOBAL const U16 reg_main_scene_sports[][2]
#if defined(_CAMACQ_API_C_)
={
{0xFCFC, 0xD000},
{0x0028, 0x7000},

{0x002A, 0x04EE},
{0x0F12, 0x0112},	
{0x0F12, 0x00EE},	

{0x002A, 0x0504},
{0x0F12, 0x0002}, 
{0x002A, 0x0508},
{0x0F12, 0x0D05}, 
{0x002A, 0x050C},
{0x0F12, 0x1A0A}, 
{0x002A, 0x0510},
{0x0F12, 0x3415},	

{0x002A, 0x0514},
{0x0F12, 0x0002}, 
{0x002A, 0x0518},
{0x0F12, 0x0D05}, 
{0x002A, 0x051C},
{0x0F12, 0x1A0A}, 
{0x002A, 0x0520},
{0x0F12, 0x3415},	

{0x002A, 0x0524},
{0x0F12, 0x0200}, 
{0x0F12, 0x0200}, 
{0x0F12, 0x0200}, 
{0x0F12, 0x0200}, 

{0x0F12, 0x0200}, 
{0x0F12, 0x8000}, 

{0x0F12, 0x0200}, 
{0x0F12, 0x0200}, 
{0x0F12, 0x0200}, 
{0x0F12, 0x0200}, 

{0x0F12, 0x0200}, 
{0x0F12, 0x8000}, 





{0x002A, 0x3302},
{0x0F12, 0x0001},	

{0x002A, 0x0D1E},
{0x0F12, 0x2102},


{0x002A, 0x0208},
{0x0F12, 0x0000},	
{0x002A, 0x0210},
{0x0F12, 0x0000},	
{0x002A, 0x020C},
{0x0F12, 0x0001},	
{0x002A, 0x01F4},
{0x0F12, 0x0001},	
{0x002A, 0x020A},
{0x0F12, 0x0001},	
{0x002A, 0x0212},
{0x0F12, 0x0001},	
{0x002A, 0x01E8},
{0x0F12, 0x0000},	
{0x0F12, 0x0001},	

CAMACQ_MAIN_BTM_OF_DATA
}
#endif /* _CAMACQ_API_C_ */
;

//==========================================================
// CAMERA_SCENE_BEACH
//==========================================================
GLOBAL const U16 reg_main_scene_beach[][2]
#if defined(_CAMACQ_API_C_)
={
{0xFCFC, 0xD000},
{0x0028, 0x7000},
{0x002A, 0x01D0},
{0x0F12, 0x0020},	
{0x002A, 0x01D4},
{0x0F12, 0x001E},	
{0x0F12, 0x0000},	

{0x002A, 0x0504},
{0x0F12, 0xC350},	
{0x002A, 0x0508},
{0x0F12, 0xC350},	
{0x002A, 0x050C},
{0x0F12, 0xC350},	
{0x002A, 0x0510},
{0x0F12, 0xC350},	

{0x002A, 0x0514},
{0x0F12, 0xC350},	
{0x002A, 0x0518},
{0x0F12, 0xC350},	
{0x002A, 0x051C},
{0x0F12, 0xC350},	
{0x002A, 0x0520},
{0x0F12, 0xC350},	

{0x002A, 0x05EA},
{0x0F12, 0x0150},	


{0x002A, 0x0486},
{0x0F12, 0x0003},	
{0x0F12, 0x0100},	
{0x0F12, 0x0001},	

{0x002A, 0x3302},
{0x0F12, 0x0001},	
	
{0x002A, 0x0D1E},
{0x0F12, 0x2102},	

{0x002A, 0x0208},
{0x0F12, 0x0000},	
{0x002A, 0x0210},
{0x0F12, 0x0000},	
{0x002A, 0x020C},
{0x0F12, 0x0001},	
{0x002A, 0x01F4},
{0x0F12, 0x0001},	
{0x002A, 0x020A},
{0x0F12, 0x0001},	
{0x002A, 0x0212},
{0x0F12, 0x0001},	
{0x002A, 0x01E8},
{0x0F12, 0x0000},	
{0x0F12, 0x0001},

CAMACQ_MAIN_BTM_OF_DATA
}
#endif /* _CAMACQ_API_C_ */
;

//==========================================================
// CAMERA_SCENE_SNOW
//==========================================================
GLOBAL const U16 reg_main_scene_snow[][2]
#if defined(_CAMACQ_API_C_)
={

CAMACQ_MAIN_BTM_OF_DATA
}
#endif /* _CAMACQ_API_C_ */
;

//==========================================================
// CAMERA_SCENE_FALLCOLOR
//==========================================================
GLOBAL const U16 reg_main_scene_fallcolor[][2]
#if defined(_CAMACQ_API_C_)
={
{0xFCFC, 0xD000}, 
{0x0028, 0x7000}, 
{0x002A, 0x01D0}, 
{0x0F12, 0x0000},	
{0x002A, 0x01D4}, 
{0x0F12, 0x0032},	
{0x0F12, 0x0000},	

CAMACQ_MAIN_BTM_OF_DATA
}
#endif /* _CAMACQ_API_C_ */
;

//==========================================================
// CAMERA_SCENE_FIREWORKS
//==========================================================
GLOBAL const U16 reg_main_scene_fireworks[][2]
#if defined(_CAMACQ_API_C_)
={
{0xFCFC, 0xD000},
{0x0028, 0x7000},
                 
{0x002A, 0x025A},
{0x0F12, 0x09C4},	// 4fps
                 
{0x002A, 0x034C},
{0x0F12, 0x2710},
{0x0F12, 0x2710},
                 
                 
{0x002A, 0x0504},
{0x0F12, 0x3415},
{0x002A, 0x0508},
{0x0F12, 0x681F},
{0x002A, 0x050C},
{0x0F12, 0x8227},
{0x002A, 0x0510},
{0x0F12, 0x1A80},
{0x0F12, 0x0006},
                 
{0x002A, 0x0514},
{0x0F12, 0x3415},
{0x002A, 0x0518},
{0x0F12, 0x681F},
{0x002A, 0x051C},
{0x0F12, 0x8227},
{0x002A, 0x0520},
{0x0F12, 0x1A80},
{0x0F12, 0x0006},
                 
                 
{0x002A, 0x05EA},
{0x0F12, 0x0150},
                 
     
{0x002A, 0x0486},
{0x0F12, 0x0003},
{0x0F12, 0x0100},
{0x0F12, 0x0001},
                 

{0x002A, 0x3302},
{0x0F12, 0x0001},
                 
{0x002A, 0x0D1E},
{0x0F12, 0x2102},
      
      
{0x002A, 0x0208},
{0x0F12, 0x0000},                
{0x002A, 0x0210},                
{0x0F12, 0x0000},       
{0x002A, 0x020C},       
{0x0F12, 0x0001},       
{0x002A, 0x01F4},       
{0x0F12, 0x0001},       
{0x002A, 0x020A},       
{0x0F12, 0x0001},       
{0x002A, 0x0212},       
{0x0F12, 0x0001},       
{0x002A, 0x01E8},       
{0x0F12, 0x0000},       
{0x0F12, 0x0001},  

CAMACQ_MAIN_BTM_OF_DATA
}
#endif /* _CAMACQ_API_C_ */
;

//==========================================================
// CAMERA_SCENE_CANDLELIGHT
//==========================================================
GLOBAL const U16 reg_main_scene_candlelight[][2]
#if defined(_CAMACQ_API_C_)
={
{0xFCFC, 0xD000}, 
{0x0028, 0x7000}, 
{0x002A, 0x2A62}, 
{0x0F12, 0x0000},	
                  
{0x002A, 0x0470}, 
{0x0F12, 0x0620},	
{0x0F12, 0x0001},	
{0x0F12, 0x0400},	
{0x0F12, 0x0001},	
{0x0F12, 0x0540},	
{0x0F12, 0x0001},	

CAMACQ_MAIN_BTM_OF_DATA
}
#endif /* _CAMACQ_API_C_ */
;

//==========================================================
// CAMERA_SCENE_AGAINSTLIGHT (BACKLight??)
//==========================================================
GLOBAL const U16 reg_main_scene_againstlight[][2]
#if defined(_CAMACQ_API_C_)
={

CAMACQ_MAIN_BTM_OF_DATA
}
#endif /* _CAMACQ_API_C_ */
;

//==========================================================
// CAMERA_SCENE_TEXT
//==========================================================
GLOBAL const U16 reg_main_scene_text[][2]
#if defined(_CAMACQ_API_C_)
={
{0xFCFC, 0xD000},
{0x0028, 0x7000},
{0x002A, 0x01D0},
{0x0F12, 0x0000},
{0x002A, 0x01D4},
{0x0F12, 0x0000},
{0x0F12, 0x0010},

CAMACQ_MAIN_BTM_OF_DATA
}
#endif /* _CAMACQ_API_C_ */
;

//==========================================================
// CAMERA_SCENE_AQUA
//==========================================================
GLOBAL const U16 reg_main_scene_aqua[][2]
#if defined(_CAMACQ_API_C_)
={

CAMACQ_MAIN_BTM_OF_DATA
}
#endif /* _CAMACQ_API_C_ */
;

//==========================================================
// CAMERA_ADJUST_CONTRAST_M2
//==========================================================
GLOBAL const U16 reg_main_adjust_contrast_m2[][2]
#if defined(_CAMACQ_API_C_)
={
{0xFCFC, 0xD000}, 
{0x0028, 0x7000}, 
{0x002A, 0x01D2},
{0x0F12, 0xFF80},		//REG_TC_UserContrast

CAMACQ_MAIN_BTM_OF_DATA
}
#endif /* _CAMACQ_API_C_ */
;

//==========================================================
// CAMERA_ADJUST_CONTRAST_M1
//==========================================================
GLOBAL const U16 reg_main_adjust_contrast_m1[][2]
#if defined(_CAMACQ_API_C_)
={
{0xFCFC, 0xD000}, 
{0x0028, 0x7000}, 
{0x002A, 0x01D2},
{0x0F12, 0xFFC0},		//REG_TC_UserContrast

CAMACQ_MAIN_BTM_OF_DATA
}
#endif /* _CAMACQ_API_C_ */
;

//==========================================================
// CAMERA_ADJUST_CONTRAST_DEFAULT
//==========================================================
GLOBAL const U16 reg_main_adjust_contrast_default[][2]
#if defined(_CAMACQ_API_C_)
={
{0xFCFC, 0xD000}, 
{0x0028, 0x7000}, 
{0x002A, 0x01D2},
{0x0F12, 0x0000},		//REG_TC_UserContrast

CAMACQ_MAIN_BTM_OF_DATA
}
#endif /* _CAMACQ_API_C_ */
;

//==========================================================
// CAMERA_ADJUST_CONTRAST_P1
//==========================================================
GLOBAL const U16 reg_main_adjust_contrast_p1[][2]
#if defined(_CAMACQ_API_C_)
={
{0xFCFC, 0xD000}, 
{0x0028, 0x7000}, 
{0x002A, 0x01D2},
{0x0F12, 0x0040},		//REG_TC_UserContrast

CAMACQ_MAIN_BTM_OF_DATA
}
#endif /* _CAMACQ_API_C_ */
;

//==========================================================
// CAMERA_ADJUST_CONTRAST_P2
//==========================================================
GLOBAL const U16 reg_main_adjust_contrast_p2[][2]
#if defined(_CAMACQ_API_C_)
={
{0xFCFC, 0xD000}, 
{0x0028, 0x7000}, 
{0x002A, 0x01D2},
{0x0F12, 0x0080},		//REG_TC_UserContrast

CAMACQ_MAIN_BTM_OF_DATA
}
#endif /* _CAMACQ_API_C_ */
;

//==========================================================
// CAMERA_ADJUST_SHARPNESS_M2
//==========================================================
GLOBAL const U16 reg_main_adjust_sharpness_m2[][2]
#if defined(_CAMACQ_API_C_)
={
{0xFCFC, 0xD000}, 
{0x0028, 0x7000}, 
{0x002A, 0x01D6},
{0x0F12, 0xFFC0},		//REG_TC_UserContrast

CAMACQ_MAIN_BTM_OF_DATA
}
#endif /* _CAMACQ_API_C_ */
;

//==========================================================
// CAMERA_ADJUST_SHARPNESS_M1
//==========================================================
GLOBAL const U16 reg_main_adjust_sharpness_m1[][2]
#if defined(_CAMACQ_API_C_)
={
{0xFCFC, 0xD000}, 
{0x0028, 0x7000}, 
{0x002A, 0x01D6},
{0x0F12, 0xFFE0},		//REG_TC_UserContrast

CAMACQ_MAIN_BTM_OF_DATA
}
#endif /* _CAMACQ_API_C_ */
;


//==========================================================
// CAMERA_ADJUST_SHARPNESS_DEFAULT
//==========================================================
GLOBAL const U16 reg_main_adjust_sharpness_default[][2]
#if defined(_CAMACQ_API_C_)
={
{0xFCFC, 0xD000}, 
{0x0028, 0x7000}, 
{0x002A, 0x01D6},
{0x0F12, 0x0000},		//REG_TC_UserContrast

CAMACQ_MAIN_BTM_OF_DATA
}
#endif /* _CAMACQ_API_C_ */
;


//==========================================================
// CAMERA_ADJUST_SHARPNESS_P1
//==========================================================
GLOBAL const U16 reg_main_adjust_sharpness_p1[][2]
#if defined(_CAMACQ_API_C_)
={
{0xFCFC, 0xD000}, 
{0x0028, 0x7000}, 
{0x002A, 0x01D6},
{0x0F12, 0x0005},		//REG_TC_UserContrast

CAMACQ_MAIN_BTM_OF_DATA
}
#endif /* _CAMACQ_API_C_ */
;


//==========================================================
// CAMERA_ADJUST_SHARPNESS_P2
//==========================================================
GLOBAL const U16 reg_main_adjust_sharpness_p2[][2]
#if defined(_CAMACQ_API_C_)
={
{0xFCFC, 0xD000}, 
{0x0028, 0x7000}, 
{0x002A, 0x01D6},
{0x0F12, 0x0010},		//REG_TC_UserContrast

CAMACQ_MAIN_BTM_OF_DATA
}
#endif /* _CAMACQ_API_C_ */
;

//==========================================================
// CAMERA_ADJUST_SATURATION_M2
//==========================================================
GLOBAL const U16 reg_main_adjust_saturation_m2[][2]
#if defined(_CAMACQ_API_C_)
={
{0xFCFC, 0xD000}, 
{0x0028, 0x7000}, 
{0x002A, 0x01D4},
{0x0F12, 0xFF80},		//REG_TC_UserContrast

CAMACQ_MAIN_BTM_OF_DATA
}
#endif /* _CAMACQ_API_C_ */
;

//==========================================================
// CAMERA_ADJUST_SATURATION_M1
//==========================================================
GLOBAL const U16 reg_main_adjust_saturation_m1[][2]
#if defined(_CAMACQ_API_C_)
={
{0xFCFC, 0xD000}, 
{0x0028, 0x7000}, 
{0x002A, 0x01D4},
{0x0F12, 0xFFC0},		//REG_TC_UserContrast

CAMACQ_MAIN_BTM_OF_DATA
}
#endif /* _CAMACQ_API_C_ */
;

//==========================================================
// CAMERA_ADJUST_SATURATION_DEFAULT
//==========================================================
GLOBAL const U16 reg_main_adjust_saturation_default[][2]
#if defined(_CAMACQ_API_C_)
={
{0xFCFC, 0xD000}, 
{0x0028, 0x7000}, 
{0x002A, 0x01D4},
{0x0F12, 0x0000},		//REG_TC_UserContrast

CAMACQ_MAIN_BTM_OF_DATA
}
#endif /* _CAMACQ_API_C_ */
;

//==========================================================
// CAMERA_ADJUST_SATURATION_P1
//==========================================================
GLOBAL const U16 reg_main_adjust_saturation_p1[][2]
#if defined(_CAMACQ_API_C_)
={
{0xFCFC, 0xD000}, 
{0x0028, 0x7000}, 
{0x002A, 0x01D4},
{0x0F12, 0x0040},		//REG_TC_UserContrast   

CAMACQ_MAIN_BTM_OF_DATA
}
#endif /* _CAMACQ_API_C_ */
;

//==========================================================
// CAMERA_ADJUST_SATURATION_P2
//==========================================================
GLOBAL const U16 reg_main_adjust_saturation_p2[][2]
#if defined(_CAMACQ_API_C_)
={
{0xFCFC, 0xD000}, 
{0x0028, 0x7000}, 
{0x002A, 0x01D4},
{0x0F12, 0x007E},		//REG_TC_UserContrast

CAMACQ_MAIN_BTM_OF_DATA
}
#endif /* _CAMACQ_API_C_ */
;

     
//==========================================================
// CAMERA_JPEG_output_5M
//==========================================================
GLOBAL const U16 reg_main_jpeg_5m[][2]
#if defined(_CAMACQ_API_C_)
={


CAMACQ_MAIN_BTM_OF_DATA
}
#endif /* _CAMACQ_API_C_ */
;

//==========================================================
// CAMERA_JPEG_output_5M_2
//==========================================================
GLOBAL const U16 reg_main_jpeg_5m_2[][2]
#if defined(_CAMACQ_API_C_)
={


CAMACQ_MAIN_BTM_OF_DATA
}
#endif /* _CAMACQ_API_C_ */
;

//==========================================================
// CAMERA_JPEG_output_4M
//==========================================================
GLOBAL const U16 reg_main_jpeg_w4m[][2]
#if defined(_CAMACQ_API_C_)
={


CAMACQ_MAIN_BTM_OF_DATA
}
#endif /* _CAMACQ_API_C_ */
;

//==========================================================
// CAMERA_JPEG_output_3M
//==========================================================
GLOBAL const U16 reg_main_jpeg_3m[][2]
#if defined(_CAMACQ_API_C_)
={
{0xFCFC, 0xD000},
{0x0028, 0x7000},

{0x002A, 0x0330},
{0x0F12, 0x0800}, //REG_0TC_CCFG_usWidth  - 2048
{0x0F12, 0x0600}, //REG_0TC_CCFG_usHeight  - 1536

CAMACQ_MAIN_BTM_OF_DATA
}
#endif /* _CAMACQ_API_C_ */
;

//==========================================================
// CAMERA_JPEG_output_2M
//==========================================================
GLOBAL const U16 reg_main_jpeg_2m[][2]
#if defined(_CAMACQ_API_C_)
={
{0xFCFC, 0xD000},
{0x0028, 0x7000},

{0x002A, 0x0330},
{0x0F12, 0x0640}, //REG_0TC_CCFG_usWidth  - 1600
{0x0F12, 0x04B0}, //REG_0TC_CCFG_usHeight  - 1200

CAMACQ_MAIN_BTM_OF_DATA
}
#endif /* _CAMACQ_API_C_ */
;

//==========================================================
// CAMERA_JPEG_output_w1_5M
//==========================================================
GLOBAL const U16 reg_main_jpeg_w1_5m[][2]
#if defined(_CAMACQ_API_C_)
={

CAMACQ_MAIN_BTM_OF_DATA
}
#endif /* _CAMACQ_API_C_ */
;

//==========================================================
// CAMERA_JPEG_output_1M
//==========================================================
GLOBAL const U16 reg_main_jpeg_1m[][2]
#if defined(_CAMACQ_API_C_)
={
{0xFCFC, 0xD000},
{0x0028, 0x7000},

{0x002A, 0x0330},
{0x0F12, 0x0500}, //REG_0TC_CCFG_usWidth  - 1280
{0x0F12, 0x03C0}, //REG_0TC_CCFG_usHeight  - 960

CAMACQ_MAIN_BTM_OF_DATA
}
#endif /* _CAMACQ_API_C_ */
;

//==========================================================
// CAMERA_JPEG_output_VGA
//==========================================================
GLOBAL const U16 reg_main_jpeg_vga[][2]
#if defined(_CAMACQ_API_C_)
={
{0xFCFC, 0xD000},
{0x0028, 0x7000},

{0x002A, 0x0330},
{0x0F12, 0x0280}, //REG_0TC_CCFG_usWidth  - 640
{0x0F12, 0x01E0}, //REG_0TC_CCFG_usHeight  - 480

CAMACQ_MAIN_BTM_OF_DATA
}
#endif /* _CAMACQ_API_C_ */
;

//==========================================================
// CAMERA_JPEG_output_WQVGA
//==========================================================
GLOBAL const U16 reg_main_jpeg_wqvga[][2]
#if defined(_CAMACQ_API_C_)
={

CAMACQ_MAIN_BTM_OF_DATA
}
#endif /* _CAMACQ_API_C_ */
;

//==========================================================
// CAMERA_JPEG_output_QVGA
//==========================================================
GLOBAL const U16 reg_main_jpeg_qvga[][2]
#if defined(_CAMACQ_API_C_)
={
{0xFCFC, 0xD000},
{0x0028, 0x7000},

{0x002A, 0x0330},
{0x0F12, 0x0140}, //REG_0TC_CCFG_usWidth  - 320
{0x0F12, 0x00F0}, //REG_0TC_CCFG_usHeight  - 240

CAMACQ_MAIN_BTM_OF_DATA
}
#endif /* _CAMACQ_API_C_ */
;


//==========================================================
// CAMERA_output_qcif
//==========================================================
GLOBAL const U16 reg_main_qcif[][2]
#if defined(_CAMACQ_API_C_)
={
    {0xFCFC, 0xD000},
    {0x0028, 0x7000},
    {0x002A, 0x01F6},
    {0x0F12, 0x0738},   //REG_TC_GP_PrevReqInputWidth       //Sensor Crop Width 1848
    {0x0F12, 0x0600},   //REG_TC_GP_PrevReqInputHeight  //Sensor Crop Height 1536
    {0x0F12, 0x0064},   //REG_TC_GP_PrevInputWidthOfs       //Sensor HOffset 100=(2048-1848)/2
    {0x0F12, 0x0000},   //REG_TC_GP_PrevInputHeightOfs  //Sensor VOffset 0
    {0x002A, 0x0216},
    {0x0F12, 0x0001},   //REG_TC_GP_bUseReqInputInPre
    
    {0x002A, 0x043C},
    {0x0F12, 0x0738},   //REG_TC_PZOOM_ZoomInputWidth
    {0x0F12, 0x0600},   //REG_TC_PZOOM_ZoomInputHeight
    {0x0F12, 0x0000},   //REG_TC_PZOOM_ZoomInputWidthOfs
    {0x0F12, 0x0000},   //REG_TC_PZOOM_ZoomInputHeightOfs
    
    //Preview Size
    {0x002A, 0x023E},
    {0x0F12, 0x00B0},   //REG_0TC_PCFG_usWidth
    {0x0F12, 0x0090},   //REG_0TC_PCFG_usHeight
    
    {0x002A, 0x0D1E},
    {0x0F12, 0x2102},   //70000D1E


CAMACQ_MAIN_BTM_OF_DATA
}
#endif /* _CAMACQ_API_C_ */
;

//==========================================================
// CAMERA_output_qvga
//==========================================================
GLOBAL const U16 reg_main_qvga[][2]
#if defined(_CAMACQ_API_C_)
={
    {0xFCFC, 0xD000},
    {0x0028, 0x7000},
    {0x002A, 0x01F6},
    {0x0F12, 0x0800},   //REG_TC_GP_PrevReqInputWidth       //Sensor Crop Width 1848
    {0x0F12, 0x0600},   //REG_TC_GP_PrevReqInputHeight  //Sensor Crop Height 1536
    {0x0F12, 0x0000},   //REG_TC_GP_PrevInputWidthOfs       //Sensor HOffset 100=(2048-1848)/2
    {0x0F12, 0x0000},   //REG_TC_GP_PrevInputHeightOfs  //Sensor VOffset 0
    {0x002A, 0x0216},
    {0x0F12, 0x0001},   //REG_TC_GP_bUseReqInputInPre
    
    {0x002A, 0x043C},
    {0x0F12, 0x0800},   //REG_TC_PZOOM_ZoomInputWidth
    {0x0F12, 0x0600},   //REG_TC_PZOOM_ZoomInputHeight
    {0x0F12, 0x0000},   //REG_TC_PZOOM_ZoomInputWidthOfs
    {0x0F12, 0x0000},   //REG_TC_PZOOM_ZoomInputHeightOfs
    
    //Preview Size
    {0x002A, 0x023E},
    {0x0F12, 0x0140},   //REG_0TC_PCFG_usWidth
    {0x0F12, 0x00F0},   //REG_0TC_PCFG_usHeight
    
    {0x002A, 0x0D1E},
    {0x0F12, 0x2102},   //70000D1E



CAMACQ_MAIN_BTM_OF_DATA
}
#endif /* _CAMACQ_API_C_ */
;

//==========================================================
// CAMERA_output_cif
//==========================================================
GLOBAL const U16 reg_main_cif[][2]
#if defined(_CAMACQ_API_C_)
={

CAMACQ_MAIN_BTM_OF_DATA
}
#endif /* _CAMACQ_API_C_ */
;

//==========================================================
// CAMERA_output_vga
//==========================================================
GLOBAL const U16 reg_main_vga[][2]
#if defined(_CAMACQ_API_C_)
={
    {0xFCFC, 0xD000},
    {0x0028, 0x7000},
    {0x002A, 0x01F6},
    {0x0F12, 0x0800},   //REG_TC_GP_PrevReqInputWidth       //Sensor Crop Width 1848
    {0x0F12, 0x0600},   //REG_TC_GP_PrevReqInputHeight  //Sensor Crop Height 1536
    {0x0F12, 0x0000},   //REG_TC_GP_PrevInputWidthOfs       //Sensor HOffset 100=(2048-1848)/2
    {0x0F12, 0x0000},   //REG_TC_GP_PrevInputHeightOfs  //Sensor VOffset 0
    {0x002A, 0x0216},
    {0x0F12, 0x0001},   //REG_TC_GP_bUseReqInputInPre
    
    {0x002A, 0x043C},
    {0x0F12, 0x0800},   //REG_TC_PZOOM_ZoomInputWidth
    {0x0F12, 0x0600},   //REG_TC_PZOOM_ZoomInputHeight
    {0x0F12, 0x0000},   //REG_TC_PZOOM_ZoomInputWidthOfs
    {0x0F12, 0x0000},   //REG_TC_PZOOM_ZoomInputHeightOfs
    
    //Preview Size
    {0x002A, 0x023E},
    {0x0F12, 0x0280},   //REG_0TC_PCFG_usWidth
    {0x0F12, 0x01E0},   //REG_0TC_PCFG_usHeight
    
    {0x002A, 0x0D1E},
    {0x0F12, 0x2102},   //70000D1E



CAMACQ_MAIN_BTM_OF_DATA
}
#endif /* _CAMACQ_API_C_ */
;

GLOBAL const U16 reg_main_1080P[][2]
#if defined(_CAMACQ_API_C_)
={

CAMACQ_MAIN_BTM_OF_DATA
}
#endif /* _CAMACQ_API_C_ */
;

GLOBAL const U16 reg_main_720P[][2]
#if defined(_CAMACQ_API_C_)
={

CAMACQ_MAIN_BTM_OF_DATA
}
#endif /* _CAMACQ_API_C_ */
;

GLOBAL const U16 reg_main_800_480[][2]
#if defined(_CAMACQ_API_C_)
={

CAMACQ_MAIN_BTM_OF_DATA
}
#endif /* _CAMACQ_API_C_ */
;

GLOBAL const U16 reg_main_720_480[][2]
#if defined(_CAMACQ_API_C_)
={

CAMACQ_MAIN_BTM_OF_DATA
}
#endif /* _CAMACQ_API_C_ */
;

GLOBAL const U16 reg_main_not[][2]
#if defined(_CAMACQ_API_C_)
={

CAMACQ_MAIN_BTM_OF_DATA
}
#endif /* _CAMACQ_API_C_ */
;

GLOBAL const U16 reg_main_flicker_disabled[][2]
#if defined(_CAMACQ_API_C_)
={

CAMACQ_MAIN_BTM_OF_DATA
}
#endif /* _CAMACQ_API_C_ */
;

GLOBAL const U16 reg_main_flicker_50hz[][2]
#if defined(_CAMACQ_API_C_)
={

CAMACQ_MAIN_BTM_OF_DATA
}
#endif /* _CAMACQ_API_C_ */
;

GLOBAL const U16 reg_main_flicker_60hz[][2]
#if defined(_CAMACQ_API_C_)
={

CAMACQ_MAIN_BTM_OF_DATA
}
#endif /* _CAMACQ_API_C_ */
;

GLOBAL const U16 reg_main_flicker_auto[][2]
#if defined(_CAMACQ_API_C_)
={

CAMACQ_MAIN_BTM_OF_DATA
}
#endif /* _CAMACQ_API_C_ */
;

GLOBAL const U16 reg_main_jpeg_quality_superfine[][2]
#if defined(_CAMACQ_API_C_)
={
{0xFCFC, 0xD000},
{0x0028, 0x7000},
{0x002A, 0x0426},   //REG_TC_BRC_usCaptureQuality
{0x0F12, 0x005F},   // 95

CAMACQ_MAIN_BTM_OF_DATA
}
#endif /* _CAMACQ_API_C_ */
;

GLOBAL const U16 reg_main_jpeg_quality_fine[][2]
#if defined(_CAMACQ_API_C_)
={
{0xFCFC, 0xD000},
{0x0028, 0x7000},
{0x002A, 0x0426},   //REG_TC_BRC_usCaptureQuality
{0x0F12, 0x005A},   // 90

CAMACQ_MAIN_BTM_OF_DATA
}
#endif /* _CAMACQ_API_C_ */
;

GLOBAL const U16 reg_main_jpeg_quality_normal[][2]
#if defined(_CAMACQ_API_C_)
={
{0xFCFC, 0xD000},
{0x0028, 0x7000},
{0x002A, 0x0426},   //REG_TC_BRC_usCaptureQuality
{0x0F12, 0x0055},   // 85

CAMACQ_MAIN_BTM_OF_DATA
}
#endif /* _CAMACQ_API_C_ */
;

GLOBAL const U16 reg_main_ae_lock[][2]
#if defined(_CAMACQ_API_C_)
={
{0xFCFC, 0xD000},
{0x0028, 0x7000},
{0x002A, 0x2A5A},
{0x0F12, 0x0000}, //Mon_AAIO_bAE
CAMACQ_MAIN_BTM_OF_DATA
}
#endif /* _CAMACQ_API_C_ */
;

GLOBAL const U16 reg_main_ae_unlock[][2]
#if defined(_CAMACQ_API_C_)
={
{0xFCFC, 0xD000},
{0x0028, 0x7000},
{0x002A, 0x2A5A},
{0x0F12, 0x0001}, //Mon_AAIO_bAE
CAMACQ_MAIN_BTM_OF_DATA
}
#endif /* _CAMACQ_API_C_ */
;

GLOBAL const U16 reg_main_awb_lock[][2]
#if defined(_CAMACQ_API_C_)
={
{0xFCFC, 0xD000},
{0x0028, 0x7000},
{0x002A, 0x11D6},
{0x0F12, 0xFFFF}, //awbb_WpFilterMinThr
CAMACQ_MAIN_BTM_OF_DATA
}
#endif /* _CAMACQ_API_C_ */
;

GLOBAL const U16 reg_main_awb_unlock[][2]
#if defined(_CAMACQ_API_C_)
={
{0xFCFC, 0xD000},
{0x0028, 0x7000},
{0x002A, 0x11D6},
{0x0F12, 0x001E}, //awbb_WpFilterMinThr
CAMACQ_MAIN_BTM_OF_DATA
}
#endif /* _CAMACQ_API_C_ */
;

GLOBAL const U16 reg_main_pre_flash_start[][2]
#if defined(_CAMACQ_API_C_)
={
{0xFCFC, 0xD000},
{0x0028, 0x7000},	
{0x002A, 0x3F82},
{0x0F12, 0x0001},

CAMACQ_MAIN_BTM_OF_DATA

}
#endif /* _CAMACQ_API_C_ */
;

GLOBAL const U16 reg_main_pre_flash_end[][2]
#if defined(_CAMACQ_API_C_)
={
{0xFCFC, 0xD000},
{0x0028, 0x7000},	
{0x002A, 0x3F84},
{0x0F12, 0x0001},

CAMACQ_MAIN_BTM_OF_DATA
}
#endif /* _CAMACQ_API_C_ */
;

GLOBAL const U16 reg_main_main_flash_start[][2]
#if defined(_CAMACQ_API_C_)
={
{0xFCFC, 0xD000},
{0x0028, 0x7000},	
{0x002A, 0x3F80},		//TNP_Regs_FastFlashAlg
{0x0F12, 0x0001},

CAMACQ_MAIN_BTM_OF_DATA

}
#endif /* _CAMACQ_API_C_ */
;

GLOBAL const U16 reg_main_main_flash_end[][2]
#if defined(_CAMACQ_API_C_)
={


CAMACQ_MAIN_BTM_OF_DATA

}
#endif /* _CAMACQ_API_C_ */
;

GLOBAL const U16 reg_main_flash_ae_set[][2]
#if defined(_CAMACQ_API_C_)
={
{0xFCFC, 0xD000},
{0x0028, 0x7000},
{0x002A, 0x0500},
{0x0F12, 0x0000},

CAMACQ_MAIN_BTM_OF_DATA

}
#endif /* _CAMACQ_API_C_ */
;

GLOBAL const U16 reg_main_flash_ae_clear[][2]
#if defined(_CAMACQ_API_C_)
={
{0xFCFC, 0xD000},
{0x0028, 0x7000},
{0x002A, 0x0500},
{0x0F12, 0x0002},

CAMACQ_MAIN_BTM_OF_DATA

}
#endif /* _CAMACQ_API_C_ */
;

//denis_flash_2
GLOBAL  const U16  reg_main_flash_on_set[][2]
#if defined(_CAMACQ_API_C_)
={


CAMACQ_MAIN_BTM_OF_DATA
}
#endif /* _CAMACQ_API_C_ */
;


GLOBAL const U16 reg_main_priv_ctrl_returnpreview[][2]
#if defined(_CAMACQ_API_C_)
={


CAMACQ_MAIN_BTM_OF_DATA

}
#endif /* _CAMACQ_API_C_ */
;

GLOBAL const U16 reg_main_1_28x_zoom_0[][2]
#if defined(_CAMACQ_API_C_)
={
    {0xFCFC, 0xD000},
    {0x0028, 0x7000},
    {0x002A, 0x0444},
    {0x0F12, 0x0100},
    {0x002A, 0x0436},
    {0x0F12, 0x0002},

{0xFFFF, 0x0064},

CAMACQ_MAIN_BTM_OF_DATA
}
#endif /* _CAMACQ_API_C_ */
;

GLOBAL const U16 reg_main_1_28x_zoom_1[][2]
#if defined(_CAMACQ_API_C_)
={
    {0xFCFC, 0xD000},
    {0x0028, 0x7000},
    {0x002A, 0x0444},
    {0x0F12, 0x0108},
    {0x002A, 0x0436},
    {0x0F12, 0x0002},

    {0xFFFF, 0x0064},

CAMACQ_MAIN_BTM_OF_DATA
}
#endif /* _CAMACQ_API_C_ */
;

GLOBAL const U16 reg_main_1_28x_zoom_2[][2]
#if defined(_CAMACQ_API_C_)
={
    {0xFCFC, 0xD000},
    {0x0028, 0x7000},
    {0x002A, 0x0444},
    {0x0F12, 0x0111},
    {0x002A, 0x0436},
    {0x0F12, 0x0002},

{0xFFFF, 0x0064},

CAMACQ_MAIN_BTM_OF_DATA
}
#endif /* _CAMACQ_API_C_ */
;

GLOBAL const U16 reg_main_1_28x_zoom_3[][2]
#if defined(_CAMACQ_API_C_)
={
    {0xFCFC, 0xD000},
    {0x0028, 0x7000},
    {0x002A, 0x0444},
    {0x0F12, 0x011A},
    {0x002A, 0x0436},
    {0x0F12, 0x0002},

{0xFFFF, 0x0064},

CAMACQ_MAIN_BTM_OF_DATA
}
#endif /* _CAMACQ_API_C_ */
;

GLOBAL const U16 reg_main_1_28x_zoom_4[][2]
#if defined(_CAMACQ_API_C_)
={
    {0xFCFC, 0xD000},
    {0x0028, 0x7000},
    {0x002A, 0x0444},
    {0x0F12, 0x0123},
    {0x002A, 0x0436},
    {0x0F12, 0x0002},

{0xFFFF, 0x0064},

CAMACQ_MAIN_BTM_OF_DATA

}
#endif /* _CAMACQ_API_C_ */
;

GLOBAL const U16 reg_main_1_28x_zoom_5[][2]
#if defined(_CAMACQ_API_C_)
={
    {0xFCFC, 0xD000},
    {0x0028, 0x7000},
    {0x002A, 0x0444},
    {0x0F12, 0x012C},
    {0x002A, 0x0436},
    {0x0F12, 0x0002},

{0xFFFF, 0x0064},


CAMACQ_MAIN_BTM_OF_DATA

}
#endif /* _CAMACQ_API_C_ */
;

GLOBAL const U16 reg_main_1_28x_zoom_6[][2]
#if defined(_CAMACQ_API_C_)
={
    {0xFCFC, 0xD000},
    {0x0028, 0x7000},
    {0x002A, 0x0444},
    {0x0F12, 0x0134},
    {0x002A, 0x0436},
    {0x0F12, 0x0002},

{0xFFFF, 0x0064},


CAMACQ_MAIN_BTM_OF_DATA

}
#endif /* _CAMACQ_API_C_ */
;

GLOBAL const U16 reg_main_1_28x_zoom_7[][2]
#if defined(_CAMACQ_API_C_)
={
    {0xFCFC, 0xD000},
    {0x0028, 0x7000},
    {0x002A, 0x0444},
    {0x0F12, 0x013D},
    {0x002A, 0x0436},
    {0x0F12, 0x0002},

{0xFFFF, 0x0064},


CAMACQ_MAIN_BTM_OF_DATA

}
#endif /* _CAMACQ_API_C_ */
;

GLOBAL const U16 reg_main_1_28x_zoom_8[][2]
#if defined(_CAMACQ_API_C_)
={
{0xFCFC, 0xD000},
{0x0028, 0x7000},
{0x002A, 0x0444},
{0x0F12, 0x0147},
{0x002A, 0x0436},
{0x0F12, 0x0002},

{0xFFFF, 0x0064},


CAMACQ_MAIN_BTM_OF_DATA

}
#endif /* _CAMACQ_API_C_ */
;


GLOBAL const U16 reg_main_1_60x_zoom_0[][2]
#if defined(_CAMACQ_API_C_)
={
    {0xFCFC, 0xD000},
    {0x0028, 0x7000},
    {0x002A, 0x0444},
    {0x0F12, 0x0100},
    {0x002A, 0x0436},
    {0x0F12, 0x0002},

{0xFFFF, 0x0064},

CAMACQ_MAIN_BTM_OF_DATA
}
#endif /* _CAMACQ_API_C_ */
;

GLOBAL const U16 reg_main_1_60x_zoom_1[][2]
#if defined(_CAMACQ_API_C_)
={
    {0xFCFC, 0xD000},
    {0x0028, 0x7000},
    {0x002A, 0x0444},
    {0x0F12, 0x0113},
    {0x002A, 0x0436},
    {0x0F12, 0x0002},

{0xFFFF, 0x0064},

CAMACQ_MAIN_BTM_OF_DATA
}
#endif /* _CAMACQ_API_C_ */
;

GLOBAL const U16 reg_main_1_60x_zoom_2[][2]
#if defined(_CAMACQ_API_C_)
={
    {0xFCFC, 0xD000},
    {0x0028, 0x7000},
    {0x002A, 0x0444},
    {0x0F12, 0x0126},
    {0x002A, 0x0436},
    {0x0F12, 0x0002},

{0xFFFF, 0x0064},

CAMACQ_MAIN_BTM_OF_DATA
}
#endif /* _CAMACQ_API_C_ */
;

GLOBAL const U16 reg_main_1_60x_zoom_3[][2]
#if defined(_CAMACQ_API_C_)
={
    {0xFCFC, 0xD000},
    {0x0028, 0x7000},
    {0x002A, 0x0444},
    {0x0F12, 0x0139},
    {0x002A, 0x0436},
    {0x0F12, 0x0002},
        
{0xFFFF, 0x0064},

CAMACQ_MAIN_BTM_OF_DATA
}
#endif /* _CAMACQ_API_C_ */
;

GLOBAL const U16 reg_main_1_60x_zoom_4[][2]
#if defined(_CAMACQ_API_C_)
={
    {0xFCFC, 0xD000},
    {0x0028, 0x7000},
    {0x002A, 0x0444},
    {0x0F12, 0x014C},
    {0x002A, 0x0436},
    {0x0F12, 0x0002},

{0xFFFF, 0x0064},

CAMACQ_MAIN_BTM_OF_DATA

}
#endif /* _CAMACQ_API_C_ */
;

GLOBAL const U16 reg_main_1_60x_zoom_5[][2]
#if defined(_CAMACQ_API_C_)
={
    {0xFCFC, 0xD000},
    {0x0028, 0x7000},
    {0x002A, 0x0444},
    {0x0F12, 0x015F},
    {0x002A, 0x0436},
    {0x0F12, 0x0002},

{0xFFFF, 0x0064},


CAMACQ_MAIN_BTM_OF_DATA

}
#endif /* _CAMACQ_API_C_ */
;

GLOBAL const U16 reg_main_1_60x_zoom_6[][2]
#if defined(_CAMACQ_API_C_)
={
    {0xFCFC, 0xD000},
    {0x0028, 0x7000},
    {0x002A, 0x0444},
    {0x0F12, 0x0172},
    {0x002A, 0x0436},
    {0x0F12, 0x0002},

{0xFFFF, 0x0064},


CAMACQ_MAIN_BTM_OF_DATA

}
#endif /* _CAMACQ_API_C_ */
;

GLOBAL const U16 reg_main_1_60x_zoom_7[][2]
#if defined(_CAMACQ_API_C_)
={
    {0xFCFC, 0xD000},
    {0x0028, 0x7000},
    {0x002A, 0x0444},
    {0x0F12, 0x0185},
    {0x002A, 0x0436},
    {0x0F12, 0x0002},

{0xFFFF, 0x0064},


CAMACQ_MAIN_BTM_OF_DATA

}
#endif /* _CAMACQ_API_C_ */
;

GLOBAL const U16 reg_main_1_60x_zoom_8[][2]
#if defined(_CAMACQ_API_C_)
={
    {0xFCFC, 0xD000},
    {0x0028, 0x7000},
    {0x002A, 0x0444},
    {0x0F12, 0x0199},
    {0x002A, 0x0436},
    {0x0F12, 0x0002},

{0xFFFF, 0x0064},


CAMACQ_MAIN_BTM_OF_DATA

}
#endif /* _CAMACQ_API_C_ */
;


    GLOBAL const U16 reg_main_2_00x_zoom_0[][2]
#if defined(_CAMACQ_API_C_)
    ={
    {0xFCFC, 0xD000},
    {0x0028, 0x7000},
    {0x002A, 0x0444},
    {0x0F12, 0x0100},
    {0x002A, 0x0436},
    {0x0F12, 0x0002},
    
    {0xFFFF, 0x0064},
    
    CAMACQ_MAIN_BTM_OF_DATA
    }
#endif /* _CAMACQ_API_C_ */
    ;
    
    GLOBAL const U16 reg_main_2_00x_zoom_1[][2]
#if defined(_CAMACQ_API_C_)
    ={
    {0xFCFC, 0xD000},
    {0x0028, 0x7000},
    {0x002A, 0x0444},
    {0x0F12, 0x0120},
    {0x002A, 0x0436},
    {0x0F12, 0x0002},
    
    {0xFFFF, 0x0064},
    
    CAMACQ_MAIN_BTM_OF_DATA
    }
#endif /* _CAMACQ_API_C_ */
    ;
    
    GLOBAL const U16 reg_main_2_00x_zoom_2[][2]
#if defined(_CAMACQ_API_C_)
    ={
    {0xFCFC, 0xD000},
    {0x0028, 0x7000},
    {0x002A, 0x0444},
    {0x0F12, 0x0140},
    {0x002A, 0x0436},
    {0x0F12, 0x0002},

    {0xFFFF, 0x0064},
    
    CAMACQ_MAIN_BTM_OF_DATA
    }
#endif /* _CAMACQ_API_C_ */
    ;
    
    GLOBAL const U16 reg_main_2_00x_zoom_3[][2]
#if defined(_CAMACQ_API_C_)
    ={
    {0xFCFC, 0xD000},
    {0x0028, 0x7000},
    {0x002A, 0x0444},
    {0x0F12, 0x0160},
    {0x002A, 0x0436},
    {0x0F12, 0x0002},

    {0xFFFF, 0x0064},
    
    CAMACQ_MAIN_BTM_OF_DATA
    }
#endif /* _CAMACQ_API_C_ */
    ;
    
    GLOBAL const U16 reg_main_2_00x_zoom_4[][2]
#if defined(_CAMACQ_API_C_)
    ={
    {0xFCFC, 0xD000},
    {0x0028, 0x7000},
    {0x002A, 0x0444},
    {0x0F12, 0x0180},
    {0x002A, 0x0436},
    {0x0F12, 0x0002},
    
    {0xFFFF, 0x0064},
    
    CAMACQ_MAIN_BTM_OF_DATA
    
    }
#endif /* _CAMACQ_API_C_ */
    ;
    
    GLOBAL const U16 reg_main_2_00x_zoom_5[][2]
#if defined(_CAMACQ_API_C_)
    ={
    {0xFCFC, 0xD000},
    {0x0028, 0x7000},
    {0x002A, 0x0444},
    {0x0F12, 0x01A0},
    {0x002A, 0x0436},
    {0x0F12, 0x0002},
    
    {0xFFFF, 0x0064},
    
    
    CAMACQ_MAIN_BTM_OF_DATA
    
    }
#endif /* _CAMACQ_API_C_ */
    ;
    
    GLOBAL const U16 reg_main_2_00x_zoom_6[][2]
#if defined(_CAMACQ_API_C_)
    ={
    {0xFCFC, 0xD000},
    {0x0028, 0x7000},
    {0x002A, 0x0444},
    {0x0F12, 0x01C0},
    {0x002A, 0x0436},
    {0x0F12, 0x0002},
    
    {0xFFFF, 0x0064},
    
    
    CAMACQ_MAIN_BTM_OF_DATA
    
    }
#endif /* _CAMACQ_API_C_ */
    ;
    
    GLOBAL const U16 reg_main_2_00x_zoom_7[][2]
#if defined(_CAMACQ_API_C_)
    ={
    {0xFCFC, 0xD000},
    {0x0028, 0x7000},
    {0x002A, 0x0444},
    {0x0F12, 0x01E0},
    {0x002A, 0x0436},
    {0x0F12, 0x0002},
    
    {0xFFFF, 0x0064},
    
    
    CAMACQ_MAIN_BTM_OF_DATA
    
    }
#endif /* _CAMACQ_API_C_ */
    ;
    
    GLOBAL const U16 reg_main_2_00x_zoom_8[][2]
#if defined(_CAMACQ_API_C_)
    ={
    {0xFCFC, 0xD000},
    {0x0028, 0x7000},
    {0x002A, 0x0444},
    {0x0F12, 0x0200},
    {0x002A, 0x0436},
    {0x0F12, 0x0002},
    
    {0xFFFF, 0x0064},
    
    
    CAMACQ_MAIN_BTM_OF_DATA
    
    }
#endif /* _CAMACQ_API_C_ */
    ;


#undef GLOBAL

#endif /* _CAMACQ_S5K5CCGX_MIPI_H_ */
