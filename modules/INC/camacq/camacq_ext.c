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

#if !defined(_CAMACQ_EXT_C_)
#define _CAMACQ_EXT_C_

/* Include */
#include "camacq_ext.h"
#include "camacq_api.h"

#if defined(CONFIG_PMIC_D1980)
#include <linux/regulator/consumer.h>
#include <linux/d1982/pmic.h>
#endif

/* Init global variable */
_stCamacqSensorRegs g_stCamacqMainSensorRegs =
{
    pvPllRegs               : CAMACQ_MAIN_REG_PLL,
    pvInitRegs              : CAMACQ_MAIN_REG_INIT,
    pvSleepRegs             : CAMACQ_MAIN_REG_SLEEP,
    pvWakeupRegs            : CAMACQ_MAIN_REG_WAKEUP,
    pvPreviewRegs           : CAMACQ_MAIN_REG_PREVIEW,
    pvSnapshotRegs          : CAMACQ_MAIN_REG_SNAPSHOT,
    pvNightshotRegs         : CAMACQ_MAIN_REG_NIGHTSHOT,
    pvLowLightshotRegs      : CAMACQ_MAIN_REG_LOWLIGHT,
    pvHighLightshotRegs     : CAMACQ_MAIN_REG_HIGHLIGHT,
    pvCamcorderOnRegs       : CAMACQ_MAIN_REG_CAMCORDER_ON,

    pvQCIF                  : CAMACQ_MAIN_REG_QCIF,
    pvQVGA                  : CAMACQ_MAIN_REG_QVGA,
    pvVGA                   : CAMACQ_MAIN_REG_VGA,
    pv720P                  : CAMACQ_MAIN_REG_720P,
    pv1080P                 : CAMACQ_MAIN_REG_1080P,
    pv800_480               : CAMACQ_MAIN_REG_800_480,
    pv720_480               : CAMACQ_MAIN_REG_720_480,

    /* ME */
    pvMeterMatrixRegs       : CAMACQ_MAIN_REG_METER_MATRIX,
    pvMeterCWRegs           : CAMACQ_MAIN_REG_METER_CW,
    pvMeterSpotRegs         : CAMACQ_MAIN_REG_METER_SPOT,
   
    /* flip */
    pvFlipNoneRegs          : CAMACQ_MAIN_REG_FLIP_NONE,
    pvFlipWaterRegs         : CAMACQ_MAIN_REG_FLIP_WATER,
    pvFlipMirrorRegs        : CAMACQ_MAIN_REG_FLIP_MIRROR,
    pvFlipWaterMirrorRegs   : CAMACQ_MAIN_REG_FLIP_WATER_MIRROR,

    /* ISO */
    pvISOAutoRegs           : CAMACQ_MAIN_REG_ISO_AUTO,
    pvISO50Regs             : CAMACQ_MAIN_REG_ISO_50,
    pvISO100Regs            : CAMACQ_MAIN_REG_ISO_100,
    pvISO200Regs            : CAMACQ_MAIN_REG_ISO_200,
    pvISO400Regs            : CAMACQ_MAIN_REG_ISO_400,
    pvISO800Regs            : CAMACQ_MAIN_REG_ISO_800,
    pvISO1600Regs           : CAMACQ_MAIN_REG_ISO_1600,
    pvISO3200Regs           : CAMACQ_MAIN_REG_ISO_3200,

    /* Scene */
    pvSceneAutoRegs         : CAMACQ_MAIN_REG_SCENE_AUTO, 
    pvSceneNightRegs        : CAMACQ_MAIN_REG_SCENE_NIGHT, 
    pvSceneLandScapeRegs    : CAMACQ_MAIN_REG_SCENE_LANDSCAPE,
    pvSceneSunSetRegs       : CAMACQ_MAIN_REG_SCENE_SUNSET,
    pvScenePortraitRegs     : CAMACQ_MAIN_REG_SCENE_PORTRAIT,
    pvSceneSunRiseRegs      : CAMACQ_MAIN_REG_SCENE_SUNRISE,
    pvSceneIndoorRegs       : CAMACQ_MAIN_REG_SCENE_INDOOR,
    pvScenePartyRegs        : CAMACQ_MAIN_REG_SCENE_PARTY,
    pvSceneSportsRegs       : CAMACQ_MAIN_REG_SCENE_SPORTS,
    pvSceneBeachRegs        : CAMACQ_MAIN_REG_SCENE_BEACH,
    pvSceneSnowRegs         : CAMACQ_MAIN_REG_SCENE_SNOW,
    pvSceneFallColorRegs    : CAMACQ_MAIN_REG_SCENE_FALLCOLOR,
    pvSceneFireWorksRegs    : CAMACQ_MAIN_REG_SCENE_FIREWORKS,
    pvSceneCandleLightRegs  : CAMACQ_MAIN_REG_SCENE_CANDLELIGHT,
    pvSceneAgainstLightRegs : CAMACQ_MAIN_REG_SCENE_AGAINSTLIGHT,
    pvSceneTextRegs         : CAMACQ_MAIN_REG_SCENE_TEXT,
    pvSceneAquaRegs         : CAMACQ_MAIN_REG_SCENE_AQUA,

    /* Brightness */
    pvBrightness_0_Regs     : CAMACQ_MAIN_REG_BRIGHTNESS_LEVEL_0,
    pvBrightness_1_Regs     : CAMACQ_MAIN_REG_BRIGHTNESS_LEVEL_1,
    pvBrightness_2_Regs     : CAMACQ_MAIN_REG_BRIGHTNESS_LEVEL_2,
    pvBrightness_3_Regs     : CAMACQ_MAIN_REG_BRIGHTNESS_LEVEL_3,
    pvBrightness_4_Regs     : CAMACQ_MAIN_REG_BRIGHTNESS_LEVEL_4,
    pvBrightness_5_Regs     : CAMACQ_MAIN_REG_BRIGHTNESS_LEVEL_5,
    pvBrightness_6_Regs     : CAMACQ_MAIN_REG_BRIGHTNESS_LEVEL_6,
    pvBrightness_7_Regs     : CAMACQ_MAIN_REG_BRIGHTNESS_LEVEL_7,
    pvBrightness_8_Regs     : CAMACQ_MAIN_REG_BRIGHTNESS_LEVEL_8,

    /* Exposure Compensation */
    pvExpCompensation_0_Regs     : CAMACQ_MAIN_REG_EXPCOMPENSATION_LEVEL_0,
    pvExpCompensation_1_Regs     : CAMACQ_MAIN_REG_EXPCOMPENSATION_LEVEL_1,
    pvExpCompensation_2_Regs     : CAMACQ_MAIN_REG_EXPCOMPENSATION_LEVEL_2,
    pvExpCompensation_3_Regs     : CAMACQ_MAIN_REG_EXPCOMPENSATION_LEVEL_3,
    pvExpCompensation_4_Regs     : CAMACQ_MAIN_REG_EXPCOMPENSATION_LEVEL_4,
    pvExpCompensation_5_Regs     : CAMACQ_MAIN_REG_EXPCOMPENSATION_LEVEL_5,
    pvExpCompensation_6_Regs     : CAMACQ_MAIN_REG_EXPCOMPENSATION_LEVEL_6,
    pvExpCompensation_7_Regs     : CAMACQ_MAIN_REG_EXPCOMPENSATION_LEVEL_7,
    pvExpCompensation_8_Regs     : CAMACQ_MAIN_REG_EXPCOMPENSATION_LEVEL_8,

    /* AF */
    pvSetAFRegs             : CAMACQ_MAIN_REG_SET_AF,   // start af
    pvOffAFRegs             : CAMACQ_MAIN_REG_OFF_AF,
    pvCheckAFRegs           : CAMACQ_MAIN_REG_CHECK_AF,
    pvResetAFRegs           : CAMACQ_MAIN_REG_RESET_AF,
    pvCancelMacroAFRegs     : CAMACQ_MAIN_REG_CANCEL_MACRO_AF,
    pvCancelManualAFRegs    : CAMACQ_MAIN_REG_CANCEL_MANUAL_AF,
    pvManualAFReg           : CAMACQ_MAIN_REG_MANUAL_AF,
    pvMacroAFReg            : CAMACQ_MAIN_REG_MACRO_AF,
    pvReturnManualAFReg     : CAMACQ_MAIN_REG_RETURN_MANUAL_AF,
    pvReturnMacroAFReg      : CAMACQ_MAIN_REG_RETURN_MACRO_AF,
    pvSetAF_NLUXRegs        : CAMACQ_MAIN_REG_SET_AF_NLUX,
    pvSetAF_LLUXRegs        : CAMACQ_MAIN_REG_SET_AF_LLUX,

    pvSetAFNormalMode1      : CAMACQ_MAIN_REG_SET_AF_NORMAL_MODE_1,
    pvSetAFNormalMode2      : CAMACQ_MAIN_REG_SET_AF_NORMAL_MODE_2,
    pvSetAFNormalMode3      : CAMACQ_MAIN_REG_SET_AF_NORMAL_MODE_3,
    pvSetAFMacroMode1       : CAMACQ_MAIN_REG_SET_AF_MACRO_MODE_1,
    pvSetAFMacroMode2       : CAMACQ_MAIN_REG_SET_AF_MACRO_MODE_2,
    pvSetAFMacroMode3       : CAMACQ_MAIN_REG_SET_AF_MACRO_MODE_3,

    /* WB */
    pvWbAutoRegs            : CAMACQ_MAIN_REG_WB_AUTO,
    pvWbDaylightRegs        : CAMACQ_MAIN_REG_WB_DAYLIGHT,
    pvWbCloudyRegs          : CAMACQ_MAIN_REG_WB_CLOUDY,
    pvWbIncandescentRegs    : CAMACQ_MAIN_REG_WB_INCANDESCENT,
    pvWbFluorescentRegs     : CAMACQ_MAIN_REG_WB_FLUORESCENT,
    pvWbTwilightRegs        : CAMACQ_MAIN_REG_WB_TWILIGHT,
    pvWbTungstenRegs        : CAMACQ_MAIN_REG_WB_TUNGSTEN,
    pvWbOffRegs             : CAMACQ_MAIN_REG_WB_OFF,
    pvWbHorizonRegs         : CAMACQ_MAIN_REG_WB_HORIZON,
    pvWbShadeRegs           : CAMACQ_MAIN_REG_WB_SHADE,
    
    /* FMT */
    pvFmtJpegRegs           : CAMACQ_MAIN_REG_FMT_JPG,
    pvFmtYUV422Regs         : CAMACQ_MAIN_REG_FMT_YUV422,

    /* Effect */
    pvEffectNoneRegs        : CAMACQ_MAIN_REG_EFFECT_NONE,
    pvEffectGrayRegs        : CAMACQ_MAIN_REG_EFFECT_GRAY,
    pvEffectNegativeRegs    : CAMACQ_MAIN_REG_EFFECT_NEGATIVE,
    pvEffectSolarizeRegs    : CAMACQ_MAIN_REG_EFFECT_SOLARIZE,
    pvEffectSepiaRegs       : CAMACQ_MAIN_REG_EFFECT_SEPIA,
    pvEffectPosterizeRegs   : CAMACQ_MAIN_REG_EFFECT_POSTERIZE,
    pvEffectWhiteBoardRegs  : CAMACQ_MAIN_REG_EFFECT_WHITEBOARD,
    pvEffectBlackBoardRegs  : CAMACQ_MAIN_REG_EFFECT_BLACKBOARD,
    pvEffectAquaRegs        : CAMACQ_MAIN_REG_EFFECT_AQUA,
    pvEffectSharpenRegs     : CAMACQ_MAIN_REG_EFFECT_SHARPEN,
    pvEffectVividRegs       : CAMACQ_MAIN_REG_EFFECT_VIVID,
    pvEffectGreenRegs       : CAMACQ_MAIN_REG_EFFECT_GREEN,
    pvEffectSketchRegs      : CAMACQ_MAIN_REG_EFFECT_SKETCH,

    /* Adjust */
    pvAdjustContrastM2          : CAMACQ_MAIN_REG_ADJUST_CONTRAST_M2,
    pvAdjustContrastM1          : CAMACQ_MAIN_REG_ADJUST_CONTRAST_M1,
    pvAdjustContrastDefault     : CAMACQ_MAIN_REG_ADJUST_CONTRAST_DEFAULT,
    pvAdjustContrastP1          : CAMACQ_MAIN_REG_ADJUST_CONTRAST_P1,
    pvAdjustContrastP2          : CAMACQ_MAIN_REG_ADJUST_CONTRAST_P2,
        
    pvAdjustSharpnessM2         : CAMACQ_MAIN_REG_ADJUST_SHARPNESS_M2,
    pvAdjustSharpnessM1         : CAMACQ_MAIN_REG_ADJUST_SHARPNESS_M1,
    pvAdjustSharpnessDefault    : CAMACQ_MAIN_REG_ADJUST_SHARPNESS_DEFAULT,
    pvAdjustSharpnessP1         : CAMACQ_MAIN_REG_ADJUST_SHARPNESS_P1,
    pvAdjustSharpnessP2         : CAMACQ_MAIN_REG_ADJUST_SHARPNESS_P2,

    pvAdjustSaturationM2        : CAMACQ_MAIN_REG_ADJUST_SATURATION_M2,
    pvAdjustSaturationM1        : CAMACQ_MAIN_REG_ADJUST_SATURATION_M1,
    pvAdjustSaturationDefault   : CAMACQ_MAIN_REG_ADJUST_SATURATION_DEFAULT,
    pvAdjustSaturationP1        : CAMACQ_MAIN_REG_ADJUST_SATURATION_P1,
    pvAdjustSaturationP2        : CAMACQ_MAIN_REG_ADJUST_SATURATION_P2,

    /* Flicker */
    pvFlickerDisabled       : CAMACQ_MAIN_REG_FLICKER_DISABLED,
    pvFlicker50Hz           : CAMACQ_MAIN_REG_FLICKER_50HZ,
    pvFlicker60Hz           : CAMACQ_MAIN_REG_FLICKER_60HZ,
    pvFlickerAuto           : CAMACQ_MAIN_REG_FLICKER_AUTO,

    pvJpegQualitySuperfine  : CAMACQ_MAIN_REG_JPEG_QUALITY_SUPERFINE,
    pvJpegQualityFine       : CAMACQ_MAIN_REG_JPEG_QUALITY_FINE,
    pvJpegQualityNormal     : CAMACQ_MAIN_REG_JPEG_QUALITY_NORMAL,   

    /* Jpeg output size */
    pvJpegOutSize5M         : CAMACQ_MAIN_REG_JPEG_5M,
    pvJpegOutSize5M_2       : CAMACQ_MAIN_REG_JPEG_5M_2,
    pvJpegOutSize4M         : CAMACQ_MAIN_REG_JPEG_W4M,
    pvJpegOutSize3M         : CAMACQ_MAIN_REG_JPEG_3M,
    pvJpegOutSize2M         : CAMACQ_MAIN_REG_JPEG_2M,
    pvJpegOutSize1_5M       : CAMACQ_MAIN_REG_JPEG_W1_5M,
    pvJpegOutSize1M         : CAMACQ_MAIN_REG_JPEG_1M,
    pvJpegOutSizeVGA        : CAMACQ_MAIN_REG_JPEG_VGA,
    pvJpegOutSizeWQVGA      : CAMACQ_MAIN_REG_JPEG_WQVGA,
    pvJpegOutSizeQVGA       : CAMACQ_MAIN_REG_JPEG_QVGA,

    /* FPS */
    pvFpsFixed_5            : CAMACQ_MAIN_REG_FPS_FIXED_5,
    pvFpsFixed_7            : CAMACQ_MAIN_REG_FPS_FIXED_7,
    pvFpsFixed_10           : CAMACQ_MAIN_REG_FPS_FIXED_10,
    pvFpsFixed_15           : CAMACQ_MAIN_REG_FPS_FIXED_15,
    pvFpsFixed_20           : CAMACQ_MAIN_REG_FPS_FIXED_20,
    pvFpsFixed_25           : CAMACQ_MAIN_REG_FPS_FIXED_25,
    pvFpsFixed_30           : CAMACQ_MAIN_REG_FPS_FIXED_30,
    pvFpsAuto_15            : CAMACQ_MAIN_REG_FPS_VAR_15,

    /* AE/AWB LOCK, UNLOCK */
    pvAeLock                : CAMACQ_MAIN_REG_AE_LOCK,
    pvAeUnLock              : CAMACQ_MAIN_REG_AE_UNLOCK,
    pvAwbLock               : CAMACQ_MAIN_REG_AWB_LOCK,
    pvAwbUnLock             : CAMACQ_MAIN_REG_AWB_UNLOCK,

    /**/
    pvPreFlashStart         : CAMACQ_MAIN_REG_PRE_FLASH_START,
    pvPreFlashEnd           : CAMACQ_MAIN_REG_PRE_FLASH_END,
    pvMainFlashStart        : CAMACQ_MAIN_REG_MAIN_FLASH_START,
    pvMainFlashEnd          : CAMACQ_MAIN_REG_MAIN_FLASH_END,

    pvFlashAeSet            : CAMACQ_MAIN_REG_FLASH_AE_SET,
    pvFlashAeClear          : CAMACQ_MAIN_REG_FLASH_AE_CLEAR,

    pvFlashOnSet            : CAMACQ_MAIN_REG_FLASH_ON_SET,     //denis_flash_2

    pvDtpOn                 : CAMACQ_MAIN_REG_DTP_ON,
    pvDtpOff                : CAMACQ_MAIN_REG_DTP_OFF,

    /* Zoom */
    pvZoom_128x_0_Regs           : CAMACQ_MAIN_REG_1_28x_ZOOM_0,
    pvZoom_128x_1_Regs           : CAMACQ_MAIN_REG_1_28x_ZOOM_1,
    pvZoom_128x_2_Regs           : CAMACQ_MAIN_REG_1_28x_ZOOM_2,
    pvZoom_128x_3_Regs           : CAMACQ_MAIN_REG_1_28x_ZOOM_3,
    pvZoom_128x_4_Regs           : CAMACQ_MAIN_REG_1_28x_ZOOM_4,
    pvZoom_128x_5_Regs           : CAMACQ_MAIN_REG_1_28x_ZOOM_5,
    pvZoom_128x_6_Regs           : CAMACQ_MAIN_REG_1_28x_ZOOM_6,
    pvZoom_128x_7_Regs           : CAMACQ_MAIN_REG_1_28x_ZOOM_7,
    pvZoom_128x_8_Regs           : CAMACQ_MAIN_REG_1_28x_ZOOM_8,

    pvZoom_160x_0_Regs           : CAMACQ_MAIN_REG_1_60x_ZOOM_0,
    pvZoom_160x_1_Regs           : CAMACQ_MAIN_REG_1_60x_ZOOM_1,
    pvZoom_160x_2_Regs           : CAMACQ_MAIN_REG_1_60x_ZOOM_2,
    pvZoom_160x_3_Regs           : CAMACQ_MAIN_REG_1_60x_ZOOM_3,
    pvZoom_160x_4_Regs           : CAMACQ_MAIN_REG_1_60x_ZOOM_4,
    pvZoom_160x_5_Regs           : CAMACQ_MAIN_REG_1_60x_ZOOM_5,
    pvZoom_160x_6_Regs           : CAMACQ_MAIN_REG_1_60x_ZOOM_6,
    pvZoom_160x_7_Regs           : CAMACQ_MAIN_REG_1_60x_ZOOM_7,
    pvZoom_160x_8_Regs           : CAMACQ_MAIN_REG_1_60x_ZOOM_8,

    pvZoom_200x_0_Regs           : CAMACQ_MAIN_REG_2_00x_ZOOM_0,
    pvZoom_200x_1_Regs           : CAMACQ_MAIN_REG_2_00x_ZOOM_1,
    pvZoom_200x_2_Regs           : CAMACQ_MAIN_REG_2_00x_ZOOM_2,
    pvZoom_200x_3_Regs           : CAMACQ_MAIN_REG_2_00x_ZOOM_3,
    pvZoom_200x_4_Regs           : CAMACQ_MAIN_REG_2_00x_ZOOM_4,
    pvZoom_200x_5_Regs           : CAMACQ_MAIN_REG_2_00x_ZOOM_5,
    pvZoom_200x_6_Regs           : CAMACQ_MAIN_REG_2_00x_ZOOM_6,
    pvZoom_200x_7_Regs           : CAMACQ_MAIN_REG_2_00x_ZOOM_7,
    pvZoom_200x_8_Regs           : CAMACQ_MAIN_REG_2_00x_ZOOM_8,

};

#if (CAMACQ_SENSOR_MAX==2)
_stCamacqSensorRegs g_stCamacqSubSensorRegs =
{
    pvInitRegs              : CAMACQ_SUB_REG_INIT,
    pvSleepRegs             : CAMACQ_SUB_REG_SLEEP,
    pvWakeupRegs            : CAMACQ_SUB_REG_WAKEUP,
    pvPreviewRegs           : CAMACQ_SUB_REG_PREVIEW,
    pvSnapshotRegs          : CAMACQ_SUB_REG_SNAPSHOT,
    pvCamcorderOnRegs       : CAMACQ_SUB_REG_CAMCORDER_ON,

    pvQCIF                  : CAMACQ_SUB_REG_QCIF,
    pvQVGA                  : CAMACQ_SUB_REG_QVGA,
    pvVGA                   : CAMACQ_SUB_REG_VGA,
    pv720P                  : CAMACQ_SUB_REG_720P,
    pv1080P                 : CAMACQ_SUB_REG_1080P,
    pv800_480               : CAMACQ_SUB_REG_800_480,
    pv720_480               : CAMACQ_SUB_REG_720_480,

    /* ME */
    pvMeterMatrixRegs       : CAMACQ_SUB_REG_METER_MATRIX,
    pvMeterCWRegs           : CAMACQ_SUB_REG_METER_CW,
    pvMeterSpotRegs         : CAMACQ_SUB_REG_METER_SPOT,
    
     /* flip */
    pvFlipNoneRegs          : CAMACQ_SUB_REG_FLIP_NONE,
    pvFlipWaterRegs         : CAMACQ_SUB_REG_FLIP_WATER,
    pvFlipMirrorRegs        : CAMACQ_SUB_REG_FLIP_MIRROR,
    pvFlipWaterMirrorRegs   : CAMACQ_SUB_REG_FLIP_WATER_MIRROR,

    /* ISO */
    pvISOAutoRegs           : CAMACQ_SUB_REG_ISO_AUTO,
    pvISO50Regs             : CAMACQ_SUB_REG_ISO_50,
    pvISO100Regs            : CAMACQ_SUB_REG_ISO_100,
    pvISO200Regs            : CAMACQ_SUB_REG_ISO_200,
    pvISO400Regs            : CAMACQ_SUB_REG_ISO_400,
    pvISO800Regs            : CAMACQ_SUB_REG_ISO_800,
    pvISO1600Regs           : CAMACQ_SUB_REG_ISO_1600,
    pvISO3200Regs           : CAMACQ_SUB_REG_ISO_3200,

    /* Scene */
    pvSceneAutoRegs         : CAMACQ_SUB_REG_SCENE_AUTO, 
    pvSceneNightRegs        : CAMACQ_SUB_REG_SCENE_NIGHT, 
    pvSceneLandScapeRegs    : CAMACQ_SUB_REG_SCENE_LANDSCAPE,
    pvSceneSunSetRegs       : CAMACQ_SUB_REG_SCENE_SUNSET,
    pvScenePortraitRegs     : CAMACQ_SUB_REG_SCENE_PORTRAIT,
    pvSceneSunRiseRegs      : CAMACQ_SUB_REG_SCENE_SUNRISE,
    pvSceneIndoorRegs       : CAMACQ_SUB_REG_SCENE_INDOOR,
    pvScenePartyRegs        : CAMACQ_SUB_REG_SCENE_PARTY,
    pvSceneSportsRegs       : CAMACQ_SUB_REG_SCENE_SPORTS,
    pvSceneBeachRegs        : CAMACQ_SUB_REG_SCENE_BEACH,
    pvSceneSnowRegs         : CAMACQ_SUB_REG_SCENE_SNOW,
    pvSceneFallColorRegs    : CAMACQ_SUB_REG_SCENE_FALLCOLOR,
    pvSceneFireWorksRegs    : CAMACQ_SUB_REG_SCENE_FIREWORKS,
    pvSceneCandleLightRegs  : CAMACQ_SUB_REG_SCENE_CANDLELIGHT,
    pvSceneAgainstLightRegs : CAMACQ_SUB_REG_SCENE_AGAINSTLIGHT,
    pvSceneTextRegs         : CAMACQ_SUB_REG_SCENE_TEXT,
    pvSceneAquaRegs         : CAMACQ_SUB_REG_SCENE_AQUA,

    /* Brightness */
    pvBrightness_0_Regs     : CAMACQ_SUB_REG_BRIGHTNESS_LEVEL_0,
    pvBrightness_1_Regs     : CAMACQ_SUB_REG_BRIGHTNESS_LEVEL_1,
    pvBrightness_2_Regs     : CAMACQ_SUB_REG_BRIGHTNESS_LEVEL_2,
    pvBrightness_3_Regs     : CAMACQ_SUB_REG_BRIGHTNESS_LEVEL_3,
    pvBrightness_4_Regs     : CAMACQ_SUB_REG_BRIGHTNESS_LEVEL_4,
    pvBrightness_5_Regs     : CAMACQ_SUB_REG_BRIGHTNESS_LEVEL_5,
    pvBrightness_6_Regs     : CAMACQ_SUB_REG_BRIGHTNESS_LEVEL_6,
    pvBrightness_7_Regs     : CAMACQ_SUB_REG_BRIGHTNESS_LEVEL_7,
    pvBrightness_8_Regs     : CAMACQ_SUB_REG_BRIGHTNESS_LEVEL_8,

    /* AF */
    pvSetAFRegs             : CAMACQ_SUB_REG_SET_AF,   // start af
    pvOffAFRegs             : CAMACQ_SUB_REG_OFF_AF,
    pvCheckAFRegs           : CAMACQ_SUB_REG_CHECK_AF,
    pvResetAFRegs           : CAMACQ_SUB_REG_RESET_AF,
    pvCancelMacroAFRegs     : CAMACQ_SUB_REG_CANCEL_MACRO_AF,
    pvCancelManualAFRegs    : CAMACQ_SUB_REG_CANCEL_MANUAL_AF,
    pvManualAFReg           : CAMACQ_SUB_REG_MANUAL_AF,
    pvMacroAFReg            : CAMACQ_SUB_REG_MACRO_AF,
    pvReturnManualAFReg     : CAMACQ_SUB_REG_RETURN_MANUAL_AF,
    pvReturnMacroAFReg      : CAMACQ_SUB_REG_RETURN_MACRO_AF,
    pvSetAF_NLUXRegs        : CAMACQ_SUB_REG_SET_AF_NLUX,
    pvSetAF_LLUXRegs        : CAMACQ_SUB_REG_SET_AF_LLUX,

    /* WB */
    pvWbAutoRegs            : CAMACQ_SUB_REG_WB_AUTO,
    pvWbDaylightRegs        : CAMACQ_SUB_REG_WB_DAYLIGHT,
    pvWbCloudyRegs          : CAMACQ_SUB_REG_WB_CLOUDY,
    pvWbIncandescentRegs    : CAMACQ_SUB_REG_WB_INCANDESCENT,
    pvWbFluorescentRegs     : CAMACQ_SUB_REG_WB_FLUORESCENT,
    pvWbTwilightRegs        : CAMACQ_SUB_REG_WB_TWILIGHT,
    pvWbTungstenRegs        : CAMACQ_SUB_REG_WB_TUNGSTEN,
    pvWbOffRegs             : CAMACQ_SUB_REG_WB_OFF,
    pvWbHorizonRegs         : CAMACQ_SUB_REG_WB_HORIZON,
    pvWbShadeRegs           : CAMACQ_SUB_REG_WB_SHADE,

    /* FMT */
    pvFmtJpegRegs           : CAMACQ_SUB_REG_FMT_JPG,
    pvFmtYUV422Regs         : CAMACQ_SUB_REG_FMT_YUV422,

    /* Effect */
    pvEffectNoneRegs        : CAMACQ_SUB_REG_EFFECT_NONE,
    pvEffectGrayRegs        : CAMACQ_SUB_REG_EFFECT_GRAY,
    pvEffectNegativeRegs    : CAMACQ_SUB_REG_EFFECT_NEGATIVE,
    pvEffectSolarizeRegs    : CAMACQ_SUB_REG_EFFECT_SOLARIZE,
    pvEffectSepiaRegs       : CAMACQ_SUB_REG_EFFECT_SEPIA,
    pvEffectPosterizeRegs   : CAMACQ_SUB_REG_EFFECT_POSTERIZE,
    pvEffectWhiteBoardRegs  : CAMACQ_SUB_REG_EFFECT_WHITEBOARD,
    pvEffectBlackBoardRegs  : CAMACQ_SUB_REG_EFFECT_BLACKBOARD,
    pvEffectAquaRegs        : CAMACQ_SUB_REG_EFFECT_AQUA,
    pvEffectSharpenRegs     : CAMACQ_SUB_REG_EFFECT_SHARPEN,
    pvEffectVividRegs       : CAMACQ_SUB_REG_EFFECT_VIVID,
    pvEffectGreenRegs       : CAMACQ_SUB_REG_EFFECT_GREEN,
    pvEffectSketchRegs      : CAMACQ_SUB_REG_EFFECT_SKETCH,

    /* Flicker */
    pvFlickerDisabled       : CAMACQ_SUB_REG_FLICKER_DISABLED,
    pvFlicker50Hz           : CAMACQ_SUB_REG_FLICKER_50HZ,
    pvFlicker60Hz           : CAMACQ_SUB_REG_FLICKER_60HZ,
    pvFlickerAuto           : CAMACQ_SUB_REG_FLICKER_AUTO,

    pvDtpOn                 : CAMACQ_SUB_REG_DTP_ON,
    pvDtpOff                : CAMACQ_SUB_REG_DTP_OFF,
};
#endif /* CAMACQ_SENSOR_MAX==2 */

/* EXT APIs */
/* MAIN H/W */
S32 Switch_MAIN(void)
{
    S32 iRet = 0;
    CamacqTraceIN();

    CamacqTraceOUT();
    return iRet;
}

S32 Reset_MAIN(void)
{
    S32 iRet = 0;
    CamacqTraceIN();

    gpio_direction_output( mfp_to_gpio( MFP_PIN_GPIO113 ), CAMACQ_OFF ); // RF_IF1 -> MCAM_RSTN
    CamacqExtDelay(10);
    gpio_direction_output( mfp_to_gpio( MFP_PIN_GPIO113 ), CAMACQ_ON ); // RF_IF1 -> MCAM_RSTN
    
    CamacqTraceOUT();
    return iRet;
}

S32 Standby_MAIN(void)
{
    S32 iRet = 0;
    CamacqTraceIN();

    gpio_direction_output( mfp_to_gpio( MFP_PIN_GPIO114 ), CAMACQ_OFF ); // RF_IF2 -> MCAM_STBYN
    
    CamacqTraceOUT();
    return iRet;
}

S32 Wakeup_MAIN(void)
{
    S32 iRet = 0;
    CamacqTraceIN();

    gpio_direction_output( mfp_to_gpio( MFP_PIN_GPIO114 ), CAMACQ_OFF ); // RF_IF2 -> MCAM_STBYN
    
    CamacqTraceOUT();
    return iRet;
}

// TEMP
#if defined(CAMACQ_MODEL_ALKON)
#if defined(CONFIG_PMIC_D1980)
    static struct regulator *reg_cam_io         = NULL;   //LDO2
    static struct regulator *reg_cam_avdd       = NULL;   //LDO16
    static struct regulator *reg_cam_af         = NULL;   //LDO18
#else /* CONFIG_PMIC_D1980 */
    static struct regulator *reg_cam_io         = NULL;   //LDO2
    static struct regulator *reg_cam_core       = NULL;   //BUCK1
#endif /* CONFIG_PMIC_D1980 */
#elif defined(CAMACQ_MODEL_GFORCE)
    static struct regulator *reg_cam_io         = NULL;   //LDO2
#elif defined(CAMACQ_MODEL_JETTA)
#if defined(CONFIG_PMIC_D1980)
    static struct regulator *reg_cam_io         = NULL;   //LDO2
    static struct regulator *reg_cam_avdd       = NULL;   //LDO16
    static struct regulator *reg_cam_af         = NULL;   //LDO18
    static struct regulator *reg_cam_sub_core   = NULL;   //LDO19
#else   /* CONFIG_PMIC_D1980 */
#endif  /* CONFIG_PMIC_D1980 */
#else
#endif

S32 PowerOn_MAIN(void)
{
    S32 iRet = 0;
    _stCamacqSensor* pstSensor = NULL;
    CamacqTraceIN();

#if defined(CAMACQ_MODEL_ALKON)
#if defined(CONFIG_PMIC_D1980)
    {
        CamacqTraceErr("ALKON S5K5CCGX CONFIG_PMIC_D1980 !!!!!!!");
        
        CamacqExtRequestGpio( mfp_to_gpio(MFP_PIN_GPIO39), "camacq" );  // CAM_SW_EN
        CamacqExtRequestGpio( mfp_to_gpio(MFP_PIN_GPIO52), "camacq" );  // CAM_SEL
        CamacqExtRequestGpio( mfp_to_gpio(MFP_PIN_GPIO131), "camacq" ); // SWITCH_EN ---> VCAM_M_CORE_1.2V
        CamacqExtRequestGpio( mfp_to_gpio(MFP_PIN_GPIO93), "camacq" );  // CAM_MCLK
        CamacqExtRequestGpio( mfp_to_gpio(MFP_PIN_GPIO97), "camacq" );  // MCAM_STBYN
        CamacqExtRequestGpio( mfp_to_gpio(MFP_PIN_GPIO94), "camacq" );  // MCAM_RSTN

        /* MIPI Switch */ 
        gpio_direction_output( mfp_to_gpio( MFP_PIN_GPIO39 ), 0 ); // GPIO39 -> CAM_SW_EN -> LOW_ENABLE
        gpio_direction_output( mfp_to_gpio( MFP_PIN_GPIO52 ), 0 ); // GPIO52 -> CAM_SEL -> 0 is A
        CamacqExtDelay(10);

        // 0. RESET, STAND-BY ALL OFF
        gpio_direction_output( mfp_to_gpio( MFP_PIN_GPIO94 ), 0 );    // MCAM_RSTN
        gpio_direction_output( mfp_to_gpio( MFP_PIN_GPIO97 ), 0 );    // MCAM_STBYN
        CamacqExtDelay(10);
        
        // # 1. POWER UP 
        // VCAM_AVDD_2.8V
        if(reg_cam_avdd == NULL) 
            reg_cam_avdd = regulator_get(&(g_pstCamacqSensorManager->m_stCamBlock.pdev->dev), REGULATOR_CAM_AVDD);

        if(reg_cam_avdd)
            regulator_enable(reg_cam_avdd);
        else {
            CamacqTraceErr(" ### CAM_AVDD regulator is NULL ...! ");
            iRet = -1;
        }
        CamacqExtDelay(1);

        // VCAM_AF_2.8V
        if(reg_cam_af == NULL)
            reg_cam_af = regulator_get(&(g_pstCamacqSensorManager->m_stCamBlock.pdev->dev), REGULATOR_CAM_AF);
        
        if(reg_cam_af)
            regulator_enable(reg_cam_af);
        else {
            CamacqTraceErr(" ### CAM_AF regulator is NULL ...! ");
            iRet = -1;
        }
        CamacqExtDelay(1);

        // VCAM_M_CORE_1.2V
        gpio_direction_output( mfp_to_gpio( MFP_PIN_GPIO131 ), 1 );
        CamacqExtDelay(1);
        
        // VCAM_IO enable
        if(reg_cam_io == NULL)
            reg_cam_io = regulator_get(&(g_pstCamacqSensorManager->m_stCamBlock.pdev->dev), REGULATOR_CAM_IO);
        
        if(reg_cam_io)
            regulator_enable(reg_cam_io);    
        else {
            CamacqTraceErr(" ### CAM_IO regulator is NULL ...! ");
            iRet = -1;
        }
        CamacqExtDelay(10);

        // 2. MCLK
        gpio_direction_output( mfp_to_gpio( MFP_PIN_GPIO93 ), 1 );    // CAM_MCLK
        g_pstCamacqSensorManager->m_stCamBlock.CamPowerSet( 1 );
        CamacqExtDelay(10);

        // 3. STBYN
        gpio_direction_output( mfp_to_gpio( MFP_PIN_GPIO97 ), 1 );    // MCAM_STBYN
        CamacqExtDelay(10);

        // 4. RSTN
        gpio_direction_output( mfp_to_gpio( MFP_PIN_GPIO94 ), 1 );    // MCAM_RSTN
        CamacqExtDelay(100);

        CamacqExtFreeGpio( mfp_to_gpio(MFP_PIN_GPIO39) );  // CAM_SW_EN
        CamacqExtFreeGpio( mfp_to_gpio(MFP_PIN_GPIO52) );  // CAM_SEL
        CamacqExtFreeGpio( mfp_to_gpio(MFP_PIN_GPIO131) );  // SWITCH_EN ---> VCAM_M_CORE_1.2V
        CamacqExtFreeGpio( mfp_to_gpio(MFP_PIN_GPIO93) );  // CAM_MCLK
        CamacqExtFreeGpio( mfp_to_gpio(MFP_PIN_GPIO97) );  // MCAM_STBYN
        CamacqExtFreeGpio( mfp_to_gpio(MFP_PIN_GPIO94) );  // MCAM_RSTN
    }
#else /* CONFIG_PMIC_D1980 */
    {
        CamacqTraceDbg("ALKON S5K5CCGX !!!!!!!");
        
        CamacqExtRequestGpio( mfp_to_gpio(MFP_PIN_GPIO39), "camacq" );  // CAM_SW_EN
        CamacqExtRequestGpio( mfp_to_gpio(MFP_PIN_GPIO52), "camacq" );  // CAM_SEL
        CamacqExtRequestGpio( mfp_to_gpio(MFP_PIN_GPIO108), "camacq" ); // CAM_EN1 ---> VCAM_AVDD_2.8V
        CamacqExtRequestGpio( mfp_to_gpio(MFP_PIN_GPIO109), "camacq" ); // CAM_EN2 ---> VCAM_AF_2.8V
        CamacqExtRequestGpio( mfp_to_gpio(MFP_PIN_GPIO93), "camacq" );  // CAM_MCLK
        CamacqExtRequestGpio( mfp_to_gpio(MFP_PIN_GPIO97), "camacq" );  // MCAM_STBYN
        CamacqExtRequestGpio( mfp_to_gpio(MFP_PIN_GPIO94), "camacq" );  // MCAM_RSTN
        
        /* MIPI Switch */ 
        gpio_direction_output( mfp_to_gpio( MFP_PIN_GPIO39 ), 0 ); // GPIO39 -> CAM_SW_EN -> LOW_ENABLE
        gpio_direction_output( mfp_to_gpio( MFP_PIN_GPIO52 ), 0 ); // GPIO52 -> CAM_SEL -> 0 is A
        CamacqExtDelay(10);

        // 0. RESET, STAND-BY ALL OFF
        gpio_direction_output( mfp_to_gpio( MFP_PIN_GPIO94 ), 0 );    // MCAM_RSTN
        gpio_direction_output( mfp_to_gpio( MFP_PIN_GPIO97 ), 0 );    // MCAM_STBYN
        CamacqExtDelay(1);
        
        // 1. POWER UP 
        // SWBUCK1 --- >VCAM_M_CORE_1.2V
        if( reg_cam_core == NULL )
            reg_cam_core = regulator_get(&(g_pstCamacqSensorManager->m_stCamBlock.pdev->dev), "v_buck1");
        
        if( reg_cam_core ) {
            CamacqTraceDbg(" WINGI AAA==============================");
            regulator_enable(reg_cam_core);    
        }
        else
        {
            CamacqTraceErr(" v_buck1 ########################## reg is NULL ##################### ");
            iRet = -1;
        } 
        CamacqExtDelay(50);
        
         
        gpio_direction_output( mfp_to_gpio( MFP_PIN_GPIO108 ), 1 );    // CAM_EN1 ---> VCAM_AVDD_2.8V
        CamacqExtDelay(1);
        
        // VCAM_IO enable
        if(reg_cam_io == NULL)
            reg_cam_io = regulator_get(&(g_pstCamacqSensorManager->m_stCamBlock.pdev->dev), "v_cam");
        
        if(reg_cam_io) {
            iRet = regulator_enable(reg_cam_io);    
            CamacqTraceDbg( " I/O check, iRet=%x", iRet);
        }
        else {
            CamacqTraceErr(" ### CAM_IO regulator is NULL ...! ");
            iRet = -1;
        }  
        CamacqExtDelay(10);


        gpio_direction_output( mfp_to_gpio( MFP_PIN_GPIO109 ), 1 );    // CAM_EN2 ---> VCAM_AF_2.8V

        // 2. MCLK
        gpio_direction_output( mfp_to_gpio( MFP_PIN_GPIO93 ), 1 );    // CAM_MCLK
        g_pstCamacqSensorManager->m_stCamBlock.CamPowerSet( 1 );
        CamacqExtDelay(10);

        // 4. RSTN // test swap rstn, stbyn
        gpio_direction_output( mfp_to_gpio( MFP_PIN_GPIO94 ), 1 );    // MCAM_RSTN
        CamacqExtDelay(20);

        // 3. STBYN
        gpio_direction_output( mfp_to_gpio( MFP_PIN_GPIO97 ), 1 );    // MCAM_STBYN
        CamacqExtDelay(20);

        
        CamacqExtFreeGpio( mfp_to_gpio(MFP_PIN_GPIO39) );  // CAM_SW_EN
        CamacqExtFreeGpio( mfp_to_gpio(MFP_PIN_GPIO52) );  // CAM_SEL
        CamacqExtFreeGpio( mfp_to_gpio(MFP_PIN_GPIO108) );  // CAM_EN1 ---> VCAM_AVDD_2.8V
        CamacqExtFreeGpio( mfp_to_gpio(MFP_PIN_GPIO109) );  // CAM_EN2 ---> VCAM_AF_2.8V 
        CamacqExtFreeGpio( mfp_to_gpio(MFP_PIN_GPIO93) );  // CAM_MCLK
        CamacqExtFreeGpio( mfp_to_gpio(MFP_PIN_GPIO97) );  // MCAM_STBYN
        CamacqExtFreeGpio( mfp_to_gpio(MFP_PIN_GPIO94) );  // MCAM_RSTN
    }
#endif /* CONFIG_PMIC_D1980 */
#elif defined(CAMACQ_MODEL_GFORCE)
    {
        CamacqTraceDbg("GFORCE S5K5CCGX !!!!!!!");
        
        CamacqExtRequestGpio( mfp_to_gpio(MFP_PIN_GPIO39), "camacq" );  // CAM_SW_EN
        CamacqExtRequestGpio( mfp_to_gpio(MFP_PIN_GPIO52), "camacq" );  // CAM_SEL
        CamacqExtRequestGpio( mfp_to_gpio(MFP_PIN_GPIO108), "camacq" ); // CAM_EN1 ---> VCAM_AVDD_2.8V
        CamacqExtRequestGpio( mfp_to_gpio(MFP_PIN_GPIO109), "camacq" ); // CAM_EN2 ---> VCAM_AF_2.8V
        CamacqExtRequestGpio( mfp_to_gpio(MFP_PIN_GPIO131), "camacq" ); // SWITCH_EN ---> VCAM_M_CORE_1.2V
        CamacqExtRequestGpio( mfp_to_gpio(MFP_PIN_GPIO93), "camacq" );  // CAM_MCLK
        CamacqExtRequestGpio( mfp_to_gpio(MFP_PIN_GPIO97), "camacq" );  // MCAM_STBYN
        CamacqExtRequestGpio( mfp_to_gpio(MFP_PIN_GPIO94), "camacq" );  // MCAM_RSTN
        
        /* MIPI Switch */ 
        gpio_direction_output( mfp_to_gpio( MFP_PIN_GPIO39 ), 0 ); // GPIO39 -> CAM_SW_EN -> LOW_ENABLE
        gpio_direction_output( mfp_to_gpio( MFP_PIN_GPIO52 ), 0 ); // GPIO52 -> CAM_SEL -> 0 is A
        CamacqExtDelay(10);

        // 0. RESET, STAND-BY ALL OFF
        gpio_direction_output( mfp_to_gpio( MFP_PIN_GPIO94 ), 0 );    // MCAM_RSTN
        gpio_direction_output( mfp_to_gpio( MFP_PIN_GPIO97 ), 0 );    // MCAM_STBYN
        CamacqExtDelay(10);
        
        // 1. POWER UP 
        gpio_direction_output( mfp_to_gpio( MFP_PIN_GPIO108 ), 1 );    // CAM_EN1 ---> VCAM_AVDD_2.8V
        gpio_direction_output( mfp_to_gpio( MFP_PIN_GPIO109 ), 1 );    // CAM_EN2 ---> VCAM_AF_2.8V

        gpio_direction_output( mfp_to_gpio( MFP_PIN_GPIO131 ), 1 );    // SWITCH_EN ---> VCAM_M_CORE_1.2V
        
        // VCAM_IO enable
        if(reg_cam_io == NULL)
            reg_cam_io = regulator_get(&(g_pstCamacqSensorManager->m_stCamBlock.pdev->dev), "v_cam");
        
        if(reg_cam_io)
            regulator_enable(reg_cam_io);    
        else {
            CamacqTraceErr(" ### CAM_IO regulator is NULL ...! ");
            iRet = -1;
        }  
        CamacqExtDelay(10);
        
        // 2. MCLK
        gpio_direction_output( mfp_to_gpio( MFP_PIN_GPIO93 ), 1 );    // CAM_MCLK
        g_pstCamacqSensorManager->m_stCamBlock.CamPowerSet( 1 );
        CamacqExtDelay(10);

        // 3. STBYN
        gpio_direction_output( mfp_to_gpio( MFP_PIN_GPIO97 ), 1 );    // MCAM_STBYN
        CamacqExtDelay(10);

        // 4. RSTN
        gpio_direction_output( mfp_to_gpio( MFP_PIN_GPIO94 ), 1 );    // MCAM_RSTN
        CamacqExtDelay(100);

        CamacqExtFreeGpio( mfp_to_gpio(MFP_PIN_GPIO39) );  // CAM_SW_EN
        CamacqExtFreeGpio( mfp_to_gpio(MFP_PIN_GPIO52) );  // CAM_SEL
        CamacqExtFreeGpio( mfp_to_gpio(MFP_PIN_GPIO108) );  // CAM_EN1 ---> VCAM_AVDD_2.8V
        CamacqExtFreeGpio( mfp_to_gpio(MFP_PIN_GPIO109) );  // CAM_EN2 ---> VCAM_AF_2.8V 
        CamacqExtFreeGpio( mfp_to_gpio(MFP_PIN_GPIO131) );  // SWITCH_EN ---> VCAM_M_CORE_1.2V
        CamacqExtFreeGpio( mfp_to_gpio(MFP_PIN_GPIO93) );  // CAM_MCLK
        CamacqExtFreeGpio( mfp_to_gpio(MFP_PIN_GPIO97) );  // MCAM_STBYN
        CamacqExtFreeGpio( mfp_to_gpio(MFP_PIN_GPIO94) );  // MCAM_RSTN
    }
#elif defined(CAMACQ_MODEL_JETTA)
#if defined(CONFIG_PMIC_D1980)
    {
#if 1 // Dual-Power-Sequence
        CamacqTraceDbg("JETTA ISX012 DUAL-POWER-ON CONFIG_PMIC_D1980 !!!!!!!");
        CamacqExtRequestGpio(mfp_to_gpio( MFP_PIN_GPIO39 ), "camacq");
        CamacqExtRequestGpio(mfp_to_gpio( MFP_PIN_GPIO52 ), "camacq");
        CamacqExtRequestGpio(mfp_to_gpio( MFP_PIN_GPIO131 ), "camacq");
        CamacqExtRequestGpio(mfp_to_gpio( MFP_PIN_GPIO93 ), "camacq"); // MCLK
        CamacqExtRequestGpio(mfp_to_gpio( MFP_PIN_GPIO94 ), "camacq"); // MCAM_RSTN
        CamacqExtRequestGpio(mfp_to_gpio( MFP_PIN_GPIO97 ), "camacq"); // MCAM_STBYN
        CamacqExtRequestGpio(mfp_to_gpio( MFP_PIN_GPIO96 ), "camacq"); // SCAM_RSTN
        CamacqExtRequestGpio(mfp_to_gpio(MFP_PIN_GPIO95 ), "camacq");  // SCAM_STBYN
        CamacqExtRequestGpio(mfp_to_gpio(MFP_PIN_GPIO118 ), "camacq");  // RF_IF6, for REV0.4
        
        // Prepare-condition
        gpio_direction_output( mfp_to_gpio( MFP_PIN_GPIO96 ), 1 );      // SCAM_RSTN
        gpio_direction_output( mfp_to_gpio( MFP_PIN_GPIO95 ), 0 );      // SCAM_STBYN
        gpio_direction_output( mfp_to_gpio( MFP_PIN_GPIO94 ), 0 );      // MCAM_RSTN
        gpio_direction_output( mfp_to_gpio( MFP_PIN_GPIO97 ), 0 );      // MCAM_STBYN
        CamacqExtDelay(10);

        /* MIPI Switch */ 
        gpio_direction_output( mfp_to_gpio( MFP_PIN_GPIO39 ), 0 ); // GPIO39 -> CAM_SW_EN -> LOW_ENABLE
        gpio_direction_output( mfp_to_gpio( MFP_PIN_GPIO52 ), 0 ); // GPIO52 -> CAM_SEL -> 0 is A
        CamacqExtDelay(1);
        
        // VCAM_M_CORE_1.2V 
        gpio_direction_output( mfp_to_gpio( MFP_PIN_GPIO131 ), 1 ); // RF_IF19(GPIO131) -> SWITCH_EN -> VCAM_M_CORE_1.2V 
        CamacqExtDelay(1);

        // VCAM_IO_1.8V
        if(reg_cam_io == NULL)
            reg_cam_io = regulator_get(&(g_pstCamacqSensorManager->m_stCamBlock.pdev->dev), REGULATOR_CAM_IO);
        if(reg_cam_io)
            regulator_enable(reg_cam_io);    
        else {
            CamacqTraceErr(" ### CAM_IO regulator is NULL ...! ");
            iRet = -1;
        }
        CamacqExtDelay(1);

        // VCAM_AVDD_2.8V
        if(reg_cam_avdd == NULL)
            reg_cam_avdd = regulator_get(&(g_pstCamacqSensorManager->m_stCamBlock.pdev->dev), REGULATOR_CAM_AVDD);

        if(reg_cam_avdd)
            regulator_enable(reg_cam_avdd);
        else {
            CamacqTraceErr(" ### CAM_AVDD regulator is NULL ...! ");
            iRet = -1;
        }

        // VCAM_AVDD_2.8V for REV0.4
        gpio_direction_output( mfp_to_gpio( MFP_PIN_GPIO118 ), 1 );      // RF_IF6
        CamacqExtDelay(1);   

        // VT Sensor core
        if(reg_cam_sub_core == NULL)
            reg_cam_sub_core = regulator_get(&(g_pstCamacqSensorManager->m_stCamBlock.pdev->dev), REGULATOR_CAM_S_CORE);
        if(reg_cam_sub_core) {
            regulator_enable(reg_cam_sub_core);    
        } else {
            CamacqTraceErr(" ### CAM_S_CORE regulator is NULL ...! ");
            iRet = -1;
        } 
        CamacqExtDelay(10);

        // VT RESET-LOW
        gpio_direction_output( mfp_to_gpio( MFP_PIN_GPIO96 ), 0 );      // SCAM_RSTN
        CamacqExtDelay(10);

        // MCLK
        gpio_direction_output( mfp_to_gpio( MFP_PIN_GPIO93 ), 1 );
        g_pstCamacqSensorManager->m_stCamBlock.CamPowerSet( 1 );
        CamacqExtDelay(50);

        // VT RESET-UP
        gpio_direction_output( mfp_to_gpio( MFP_PIN_GPIO96 ), 1 );      // SCAM_RSTN
        CamacqExtDelay(10);
        
        // VT Workaround
        // reg_sub_workaround
        {
            const void*     pvRegs = CAMACQ_SUB_REG_WORKAROUND;

            pstSensor = g_pstCamacqSensorManager->GetSensor( g_pstCamacqSensorManager, CAMACQ_SENSOR_SUB );
            if( pstSensor->m_pI2cClient != 0 ) {
                iRet = CamacqExtWriteI2cLists( pstSensor->m_pI2cClient, pvRegs, pstSensor->m_uiResType );
                if( iRet < 0 )
                {
                    CamacqTraceErr(": 111 iRet=%d", iRet);
                    iRet = -1;
                    return iRet;
                }
            }
            CamacqExtDelay(50);
        }   
        
        // VT STAND-BY Up 
        gpio_direction_output( mfp_to_gpio( MFP_PIN_GPIO95 ), 1 );      // SCAM_STBYN
        CamacqExtDelay(1);

        // MAIN REST-UP
        gpio_direction_output( mfp_to_gpio( MFP_PIN_GPIO94 ), 1 );      // MCAM_RSTN
        CamacqExtDelay(20);

        // PLL init
        {
            pstSensor = g_pstCamacqSensorManager->GetSensor( g_pstCamacqSensorManager, CAMACQ_SENSOR_MAIN );
            if( pstSensor->m_pstExtAPIs->m_pstSensorRegs->pvPllRegs != NULL )
            {
                iRet = CamacqExtWriteI2cLists( pstSensor->m_pI2cClient, 
                            pstSensor->m_pstExtAPIs->m_pstSensorRegs->pvPllRegs, pstSensor->m_uiResType );
                if( iRet < 0 )
                {
                    CamacqTraceErr(": 222 iRet=%d", iRet);
                    iRet = -1;
                    return iRet;
                }
            }
            else
            {
                iRet = -1;
                CamacqTraceErr( "pvPllRegs is NULL" );
            }
        }
                
        gpio_direction_output( mfp_to_gpio( MFP_PIN_GPIO97 ), 1 );      // MCAM_STBYN
        CamacqExtDelay(20);

        // VCAM_AF_2.8V
        if(reg_cam_af == NULL)
            reg_cam_af = regulator_get(&(g_pstCamacqSensorManager->m_stCamBlock.pdev->dev), REGULATOR_CAM_AF);
        if(reg_cam_af) 
            regulator_enable(reg_cam_af);
        else {
            CamacqTraceErr(" ### CAM_AF regulator is NULL ...! ");
            iRet = -1;
        }
        CamacqExtDelay(10);

        CamacqExtFreeGpio(mfp_to_gpio( MFP_PIN_GPIO39 ));
        CamacqExtFreeGpio(mfp_to_gpio( MFP_PIN_GPIO52 ));
        CamacqExtFreeGpio(mfp_to_gpio( MFP_PIN_GPIO94 ));
        CamacqExtFreeGpio(mfp_to_gpio( MFP_PIN_GPIO97 ));
        CamacqExtFreeGpio(mfp_to_gpio( MFP_PIN_GPIO131 ));
        CamacqExtFreeGpio(mfp_to_gpio( MFP_PIN_GPIO93 ));
        CamacqExtFreeGpio(mfp_to_gpio( MFP_PIN_GPIO95 ));
        CamacqExtFreeGpio(mfp_to_gpio( MFP_PIN_GPIO96 ));
        CamacqExtFreeGpio(mfp_to_gpio( MFP_PIN_GPIO118 )); // for REV0.4 RF_IF6
#else
        CamacqTraceErr("JETTA ISX012 POWER-ON CONFIG_PMIC_D1980 !!!!!!!");
        CamacqExtRequestGpio(mfp_to_gpio( MFP_PIN_GPIO39 ), "camacq");
        CamacqExtRequestGpio(mfp_to_gpio( MFP_PIN_GPIO52 ), "camacq");
        CamacqExtRequestGpio(mfp_to_gpio( MFP_PIN_GPIO94 ), "camacq");
        CamacqExtRequestGpio(mfp_to_gpio( MFP_PIN_GPIO97 ), "camacq");
        CamacqExtRequestGpio(mfp_to_gpio( MFP_PIN_GPIO131 ), "camacq");
        CamacqExtRequestGpio(mfp_to_gpio( MFP_PIN_GPIO93 ), "camacq");

        /* MIPI Switch */ 
        gpio_direction_output( mfp_to_gpio( MFP_PIN_GPIO39 ), 0 ); // GPIO39 -> CAM_SW_EN -> LOW_ENABLE
        gpio_direction_output( mfp_to_gpio( MFP_PIN_GPIO52 ), 0 ); // GPIO52 -> CAM_SEL -> 0 is A
        CamacqExtDelay(10);

        // 0. reset, stdby all off 
        gpio_direction_output( mfp_to_gpio( MFP_PIN_GPIO94 ), 0 );    // MCAM_RSTN
        gpio_direction_output( mfp_to_gpio( MFP_PIN_GPIO97 ), 0 );    // MCAM_STBYN

        // 1. power-up
        // VCAM_M_CORE_1.2V 
        gpio_direction_output( mfp_to_gpio( MFP_PIN_GPIO131 ), 1 ); // RF_IF19(GPIO131) -> SWITCH_EN -> VCAM_M_CORE_1.2V 
        CamacqExtDelay(10);

        // VCAM_IO_1.8V
        if(reg_cam_io == NULL)
            reg_cam_io = regulator_get(&(g_pstCamacqSensorManager->m_stCamBlock.pdev->dev), REGULATOR_CAM_IO);

        if(reg_cam_io)
            regulator_enable(reg_cam_io);    
        else {
            CamacqTraceErr(" ### CAM_IO regulator is NULL ...! ");
            iRet = -1;
        }
        CamacqExtDelay(10);

        // VCAM_AVDD_2.8V
        if(reg_cam_avdd == NULL)
            reg_cam_avdd = regulator_get(&(g_pstCamacqSensorManager->m_stCamBlock.pdev->dev), REGULATOR_CAM_AVDD);

        if(reg_cam_avdd)
            regulator_enable(reg_cam_avdd);
        else {
            CamacqTraceErr(" ### CAM_AVDD regulator is NULL ...! ");
            iRet = -1;
        }
        CamacqExtDelay(10);

        // VCAM_AF_2.8V
        if(reg_cam_af == NULL)
            reg_cam_af = regulator_get(&(g_pstCamacqSensorManager->m_stCamBlock.pdev->dev), REGULATOR_CAM_AF);
        if(reg_cam_af) 
            regulator_enable(reg_cam_af);
        else {
            CamacqTraceErr(" ### CAM_AF regulator is NULL ...! ");
            iRet = -1;
        }
        CamacqExtDelay(10);

        // 2. MCLK
        gpio_direction_output( mfp_to_gpio( MFP_PIN_GPIO93 ), 1 );
        g_pstCamacqSensorManager->m_stCamBlock.CamPowerSet( 1 );
        CamacqExtDelay(10);

        // 3. main reset high
        gpio_direction_output( mfp_to_gpio( MFP_PIN_GPIO94 ), 1 );   // MCAM_RSTN
        CamacqExtDelay(10);  

        // 4. main stdby high
        gpio_direction_output( mfp_to_gpio( MFP_PIN_GPIO97 ), 1 ); // MCAM_STBYN
        CamacqExtDelay(100);

        CamacqExtFreeGpio(mfp_to_gpio( MFP_PIN_GPIO39 ));
        CamacqExtFreeGpio(mfp_to_gpio( MFP_PIN_GPIO52 ));
        CamacqExtFreeGpio(mfp_to_gpio( MFP_PIN_GPIO94 ));
        CamacqExtFreeGpio(mfp_to_gpio( MFP_PIN_GPIO97 ));
        CamacqExtFreeGpio(mfp_to_gpio( MFP_PIN_GPIO131 ));
        CamacqExtFreeGpio(mfp_to_gpio( MFP_PIN_GPIO93 ));
#endif  // Dual-Power-Sequence
    }
#else /* CONFIG_PMIC_D1980 */
    {
        CamacqTraceDbg("JETTA ISX012  !!!!!!!");
    }
#endif /* CONFIG_PMIC_D1980 */ 
#elif defined(CAMACQ_MODEL_GFORCE_OLD)

#if 0 
#if defined(_S5K4ECGX_EVT1_MIPI_)
    CamacqTraceDbg("_S5K4ECGX_EVT1_MIPI_ !!!!!!!");
    CamacqTraceDbg("_S5K4ECGX_EVT1_MIPI_ !!!!!!!");
    CamacqTraceDbg("_S5K4ECGX_EVT1_MIPI_ !!!!!!!");

    CamacqExtRequestGpio(mfp_to_gpio( MFP_PIN_GPIO94 ), "camacq");
    CamacqExtRequestGpio(mfp_to_gpio( MFP_PIN_GPIO97 ), "camacq");
    CamacqExtRequestGpio(mfp_to_gpio( MFP_PIN_GPIO109 ), "camacq");
    CamacqExtRequestGpio(mfp_to_gpio( MFP_PIN_GPIO3 ), "camacq");
    CamacqExtRequestGpio(mfp_to_gpio( MFP_PIN_GPIO108 ), "camacq");
    CamacqExtRequestGpio(mfp_to_gpio( MFP_PIN_GPIO93 ), "camacq");


    // 0. reset, stdby all off 
    gpio_direction_output( mfp_to_gpio( MFP_PIN_GPIO94 ), CAMACQ_MAIN_RST_OFF );    // MCAM_RSTN
    gpio_direction_output( mfp_to_gpio( MFP_PIN_GPIO97 ), CAMACQ_OFF );            // MCAM_STBYN

    // 1. power up
    gpio_direction_output( mfp_to_gpio( MFP_PIN_GPIO109 ), CAMACQ_MAIN_EN_ON ); // GPIO109 -> CAM_EN2 -> VCAM_CORE_1.2V
    CamacqExtDelay(10);
    gpio_direction_output( mfp_to_gpio( MFP_PIN_GPIO3 ), CAMACQ_MAIN_EN_ON ); // GPIO3 -> CAM_EN4 -> VCAM_AVDD_2.8V	& VCAM_AF_2.8V
    CamacqExtDelay(10);
    gpio_direction_output( mfp_to_gpio( MFP_PIN_GPIO108 ), CAMACQ_MAIN_EN_ON ); // GPIO108 -> CAM_EN -> VCAM_IO_2.8V
    CamacqExtDelay(10);

    // 2. MCLK
    gpio_direction_output( mfp_to_gpio( MFP_PIN_GPIO93 ), 1 );
    g_pstCamacqSensorManager->m_stCamBlock.CamPowerSet( CAMACQ_ON );
    CamacqExtDelay(10);

    // 3. main stdby high
    gpio_direction_output( mfp_to_gpio( MFP_PIN_GPIO97 ), CAMACQ_ON ); // RF_IF2 -> MCAM_STBYN
    CamacqExtDelay(10);

    // 4. main reset high
    gpio_direction_output( mfp_to_gpio( MFP_PIN_GPIO94 ), CAMACQ_MAIN_RST_ON );   // RF_IF1 -> MCAM_RSTN
    CamacqExtDelay(100);  

    CamacqExtFreeGpio(mfp_to_gpio( MFP_PIN_GPIO94 ));
    CamacqExtFreeGpio(mfp_to_gpio( MFP_PIN_GPIO97 ));
    CamacqExtFreeGpio(mfp_to_gpio( MFP_PIN_GPIO109 ));
    CamacqExtFreeGpio(mfp_to_gpio( MFP_PIN_GPIO3 ));
    CamacqExtFreeGpio(mfp_to_gpio( MFP_PIN_GPIO108 ));
    CamacqExtFreeGpio(mfp_to_gpio( MFP_PIN_GPIO93 ));
  

#elif defined(__ISX006_SONY__)
    // 0. reset, stdby all off 
    gpio_direction_output( mfp_to_gpio( MFP_PIN_GPIO94 ), CAMACQ_MAIN_RST_OFF );    // MCAM_RSTN
    gpio_direction_output( mfp_to_gpio( MFP_PIN_GPIO97 ), CAMACQ_OFF );            // MCAM_STBYN

    // 1. power up
    gpio_direction_output( mfp_to_gpio( MFP_PIN_GPIO3 ), CAMACQ_MAIN_EN_ON ); // GPIO3 -> CAM_EN4 -> VCAM_AVDD_2.8V	& VCAM_AF_2.8V
    CamacqExtDelay(1);
    gpio_direction_output( mfp_to_gpio( MFP_PIN_GPIO109 ), CAMACQ_MAIN_EN_ON ); // GPIO109 -> CAM_EN2 -> VCAM_CORE_1.2V
    CamacqExtDelay(1);
    gpio_direction_output( mfp_to_gpio( MFP_PIN_GPIO108 ), CAMACQ_MAIN_EN_ON ); // GPIO108 -> CAM_EN -> VCAM_IO_2.8V
    CamacqExtDelay(1);

    // 2. MCLK
    gpio_direction_output( mfp_to_gpio( MFP_PIN_GPIO93 ), 1 );
    g_pstCamacqSensorManager->m_stCamBlock.CamPowerSet( CAMACQ_ON );
    CamacqExtDelay(1);

    // 3. main reset high
    gpio_direction_output( mfp_to_gpio( MFP_PIN_GPIO94 ), CAMACQ_MAIN_RST_ON );   // RF_IF1 -> MCAM_RSTN
    CamacqExtDelay(10);

    // 4. main stdby high
    gpio_direction_output( mfp_to_gpio( MFP_PIN_GPIO97 ), CAMACQ_ON ); // RF_IF2 -> MCAM_STBYN
    CamacqExtDelay(100); 

/*
    // Switch 
    gpio_direction_output( mfp_to_gpio( MFP_PIN_GPIO101 ), 0 ); // GPIO101 -> CAM_SW_EN -> LOW_ENABLE
    gpio_direction_output( mfp_to_gpio( MFP_PIN_GPIO100 ), 0 ); // GPIO100 -> CAM_SEL -> 0 is A
    CamacqExtDelay(10);

    // 0. reset, stdby all off 
    gpio_direction_output( mfp_to_gpio( MFP_PIN_GPIO113 ), CAMACQ_MAIN_RST_OFF );   // RF_IF1 -> MCAM_RSTN
    gpio_direction_output( mfp_to_gpio( MFP_PIN_GPIO114 ), CAMACQ_OFF ); // RF_IF2 -> MCAM_STBYN
        
    // 1. power up
    gpio_direction_output( mfp_to_gpio( MFP_PIN_GPIO109 ), CAMACQ_MAIN_EN_ON ); // GPIO109 -> CAM_EN4 -> VCAM_AVDD_2.8V	& VCAM_AF_2.8V
    CamacqExtDelay(1);
    gpio_direction_output( mfp_to_gpio( MFP_PIN_GPIO107 ), CAMACQ_MAIN_EN_ON ); // GPIO107 -> CAM_EN2 -> VCAM_CORE_1.2V
    CamacqExtDelay(1);
    gpio_direction_output( mfp_to_gpio( MFP_PIN_GPIO106 ), CAMACQ_MAIN_EN_ON ); // GPIO106 -> CAM_EN -> VCAM_IO_2.8V
    CamacqExtDelay(1);
    
    // 2. MCLK
    gpio_direction_output( mfp_to_gpio( MFP_PIN_GPIO93 ), 1 );
    CamacqExtDelay(1);

    // 6. main reset high
    gpio_direction_output( mfp_to_gpio( MFP_PIN_GPIO113 ), CAMACQ_MAIN_RST_ON );   // RF_IF1 -> MCAM_RSTN
    CamacqExtDelay(10);

    // 5. main stdby high
    gpio_direction_output( mfp_to_gpio( MFP_PIN_GPIO114 ), CAMACQ_ON ); // RF_IF2 -> MCAM_STBYN
    CamacqExtDelay(100); 
*/
#else
    /* Switch */ 
    gpio_direction_output( mfp_to_gpio( MFP_PIN_GPIO101 ), 0 ); // GPIO101 -> CAM_SW_EN -> LOW_ENABLE
    gpio_direction_output( mfp_to_gpio( MFP_PIN_GPIO100 ), 0 ); // GPIO100 -> CAM_SEL -> 0 is A
    CamacqExtDelay(10);

    // 0. both reset, stdby all off 
    gpio_direction_output( mfp_to_gpio( MFP_PIN_GPIO113 ), CAMACQ_MAIN_RST_OFF );   // RF_IF1 -> MCAM_RSTN
    gpio_direction_output( mfp_to_gpio( MFP_PIN_GPIO94 ), CAMACQ_SUB_RST_OFF );     // GPIO94 -> SCAM_RSTN
    gpio_direction_output( mfp_to_gpio( MFP_PIN_GPIO114 ), CAMACQ_OFF ); // RF_IF2 -> MCAM_STBYN
    gpio_direction_output( mfp_to_gpio( MFP_PIN_GPIO97 ), CAMACQ_OFF ); // GPIO97 -> SCAM_STBYN
    
    // 1. both power up
    gpio_direction_output( mfp_to_gpio( MFP_PIN_GPIO109 ), CAMACQ_MAIN_EN_ON ); // GPIO109 -> CAM_EN4 -> VCAM_AVDD_2.8V	& VCAM_AF_2.8V
    CamacqExtDelay(1);
    gpio_direction_output( mfp_to_gpio( MFP_PIN_GPIO108 ), CAMACQ_SUB_EN_ON ); // GPIO108 -> CAM_EN3 -> VCAM_DVDD_1.5V
    CamacqExtDelay(1);
    gpio_direction_output( mfp_to_gpio( MFP_PIN_GPIO107 ), CAMACQ_MAIN_EN_ON ); // GPIO107 -> CAM_EN2 -> VCAM_CORE_1.2V
    CamacqExtDelay(1);
    gpio_direction_output( mfp_to_gpio( MFP_PIN_GPIO106 ), CAMACQ_MAIN_EN_ON ); // GPIO106 -> CAM_EN -> VCAM_IO_2.8V
    CamacqExtDelay(1);
    
    // 2. MCLK
    gpio_direction_output( mfp_to_gpio( MFP_PIN_GPIO93 ), 1 );
    CamacqExtDelay(1);

    // 3. sub stbny high
    gpio_direction_output( mfp_to_gpio( MFP_PIN_GPIO97 ), CAMACQ_ON ); // GPIO97 -> SCAM_STBYN
    CamacqExtDelay(1);

    // 4. sub reset high
    gpio_direction_output( mfp_to_gpio( MFP_PIN_GPIO94 ), CAMACQ_SUB_RST_ON );     // GPIO94 -> SCAM_RSTN
    CamacqExtDelay(1);

    // 5. sub arm go & enter stdby
    pstSensor = g_pstCamacqSensorManager->GetSensor( g_pstCamacqSensorManager, CAMACQ_SENSOR_SUB );
    if( pstSensor->m_pI2cClient != 0 ) {
        CamacqExtWriteI2cLists( pstSensor->m_pI2cClient, pstSensor->m_pstExtAPIs->m_pstSensorRegs->pvSleepRegs, pstSensor->m_uiResType );
        CamacqExtDelay(10);
    }

    // 6. main reset high
    gpio_direction_output( mfp_to_gpio( MFP_PIN_GPIO113 ), CAMACQ_MAIN_RST_ON );   // RF_IF1 -> MCAM_RSTN
    CamacqExtDelay(10);

    // 5. main stdby high
    gpio_direction_output( mfp_to_gpio( MFP_PIN_GPIO114 ), CAMACQ_ON ); // RF_IF2 -> MCAM_STBYN
    CamacqExtDelay(100); 

#endif /* __ISX006_SONY__ */ 
#endif  // #if 0

#else  

#endif /* MODEL OPTION x */

    CamacqTraceOUT();
    return iRet;
}

S32 PowerOff_MAIN(void)
{
    S32 iRet = 0;
    _stCamacqSensor* pstSensor = NULL;

    CamacqTraceIN();

    // TEMP
#if defined(CAMACQ_MODEL_ALKON)
#if defined(CONFIG_PMIC_D1980)
    {
        CamacqTraceDbg("ALKON S5K5CCGX POWER OFF CONFIG_PMIC_D1980");
        
        CamacqExtRequestGpio( mfp_to_gpio(MFP_PIN_GPIO131), "camacq" ); // SWITCH_EN ---> VCAM_M_CORE_1.2V
        CamacqExtRequestGpio( mfp_to_gpio(MFP_PIN_GPIO93), "camacq" );  // CAM_MCLK
        CamacqExtRequestGpio( mfp_to_gpio(MFP_PIN_GPIO97), "camacq" );  // MCAM_STBYN
        CamacqExtRequestGpio( mfp_to_gpio(MFP_PIN_GPIO94), "camacq" );  // MCAM_RSTN

        // 1. RSTN DOWN
        gpio_direction_output( mfp_to_gpio( MFP_PIN_GPIO94 ), 0 ); // MCAM_RSTN
        CamacqExtDelay(10);

        // 2. MCLK DOWN
        gpio_direction_output( mfp_to_gpio( MFP_PIN_GPIO93 ), 0 ); // CAM_MCLK
        g_pstCamacqSensorManager->m_stCamBlock.CamPowerSet( 0 );
        CamacqExtDelay(10);

        // 3. STAND-BY DOWN
        gpio_direction_output( mfp_to_gpio( MFP_PIN_GPIO97 ), 0 ); // MCAM_STBYN
        CamacqExtDelay(10);

        // 4. Power-OFf
        // VCAM_IO    
        if(reg_cam_io)
            regulator_disable(reg_cam_io);    
        else {
            CamacqTraceErr(" ### CAM_IO regulator is NULL ...! ");
            iRet = -1;
        }
        CamacqExtDelay(10);
        
        // VCAM_M_CORE_1.2V
        gpio_direction_output( mfp_to_gpio( MFP_PIN_GPIO131 ), 0 );

        // VCAM_AVDD_2.8V
        if(reg_cam_avdd)
            regulator_disable(reg_cam_avdd);
        else {
            CamacqTraceErr(" ### CAM_AVDD regulator is NULL ...! ");
            iRet = -1;
        }

        // VCAM_AF_2.8V
        if(reg_cam_af)
            regulator_disable(reg_cam_af);
        else {
            CamacqTraceErr(" ### CAM_AF regulator is NULL ...! ");
            iRet = -1;
        }

        CamacqExtFreeGpio( mfp_to_gpio(MFP_PIN_GPIO131) );  // SWITCH_EN ---> VCAM_M_CORE_1.2V
        CamacqExtFreeGpio( mfp_to_gpio(MFP_PIN_GPIO93) );  // CAM_MCLK
        CamacqExtFreeGpio( mfp_to_gpio(MFP_PIN_GPIO97) );  // MCAM_STBYN
        CamacqExtFreeGpio( mfp_to_gpio(MFP_PIN_GPIO94) );  // MCAM_RSTN
    }
#else   /* CONFIG_PMIC_D1980 */
    {
        CamacqTraceDbg("ALKON S5K5CCGX POWER OFF ");
        
        CamacqExtRequestGpio( mfp_to_gpio(MFP_PIN_GPIO108), "camacq" ); // CAM_EN1 ---> VCAM_AVDD_2.8V
        CamacqExtRequestGpio( mfp_to_gpio(MFP_PIN_GPIO109), "camacq" ); // CAM_EN2 ---> VCAM_AF_2.8V
        CamacqExtRequestGpio( mfp_to_gpio(MFP_PIN_GPIO93), "camacq" );  // CAM_MCLK
        CamacqExtRequestGpio( mfp_to_gpio(MFP_PIN_GPIO97), "camacq" );  // MCAM_STBYN
        CamacqExtRequestGpio( mfp_to_gpio(MFP_PIN_GPIO94), "camacq" );  // MCAM_RSTN


        // 3. STAND-BY DOWN // swap
        gpio_direction_output( mfp_to_gpio( MFP_PIN_GPIO97 ), 0 ); // MCAM_STBYN
        CamacqExtDelay(1);
        
        // 1. RSTN DOWN
        gpio_direction_output( mfp_to_gpio( MFP_PIN_GPIO94 ), 0 ); // MCAM_RSTN
        CamacqExtDelay(1);
        
        

        // 2. MCLK DOWN
        gpio_direction_output( mfp_to_gpio( MFP_PIN_GPIO93 ), 0 ); // CAM_MCLK
        g_pstCamacqSensorManager->m_stCamBlock.CamPowerSet( 0 );
        CamacqExtDelay(10);
        
        // 4. Power-OFf
        // VCAM_IO    
        if(reg_cam_io)
            regulator_disable(reg_cam_io);    
        else {
            CamacqTraceErr(" ### CAM_IO regulator is NULL ...! ");
            iRet = -1;
        }
        CamacqExtDelay(1);

        // VBUCK1 --> VCAM_M_CORE_1.2V
        if(reg_cam_core) {
            CamacqTraceDbg("BBBBBBBBBBBBBBBBBBBBBBB");
           regulator_disable(reg_cam_core);    
        }
        else {
            CamacqTraceErr(" ### CAM_CORE regulator is NULL ...! ");
            iRet = -1;
        }
        
        gpio_direction_output( mfp_to_gpio( MFP_PIN_GPIO108 ), 0 );    // CAM_EN1 ---> VCAM_AVDD_2.8V
        gpio_direction_output( mfp_to_gpio( MFP_PIN_GPIO109 ), 0 );    // CAM_EN2 ---> VCAM_AF_2.8V

        CamacqExtFreeGpio( mfp_to_gpio(MFP_PIN_GPIO108) );  // CAM_EN1 ---> VCAM_AVDD_2.8V
        CamacqExtFreeGpio( mfp_to_gpio(MFP_PIN_GPIO109) );  // CAM_EN2 ---> VCAM_AF_2.8V 
        CamacqExtFreeGpio( mfp_to_gpio(MFP_PIN_GPIO93) );  // CAM_MCLK
        CamacqExtFreeGpio( mfp_to_gpio(MFP_PIN_GPIO97) );  // MCAM_STBYN
        CamacqExtFreeGpio( mfp_to_gpio(MFP_PIN_GPIO94) );  // MCAM_RSTN
    }
#endif /* CONFIG_PMIC_D1980 */
#elif defined(CAMACQ_MODEL_GFORCE)
    {
        CamacqTraceDbg("GFORCE S5K5CCGX POWER OFF ");
        
        CamacqExtRequestGpio( mfp_to_gpio(MFP_PIN_GPIO108), "camacq" ); // CAM_EN1 ---> VCAM_AVDD_2.8V
        CamacqExtRequestGpio( mfp_to_gpio(MFP_PIN_GPIO109), "camacq" ); // CAM_EN2 ---> VCAM_AF_2.8V
        CamacqExtRequestGpio( mfp_to_gpio(MFP_PIN_GPIO93), "camacq" );  // CAM_MCLK
        CamacqExtRequestGpio( mfp_to_gpio(MFP_PIN_GPIO97), "camacq" );  // MCAM_STBYN
        CamacqExtRequestGpio( mfp_to_gpio(MFP_PIN_GPIO94), "camacq" );  // MCAM_RSTN
        CamacqExtRequestGpio( mfp_to_gpio(MFP_PIN_GPIO131), "camacq" );  // MCAM_CORE

        // 1. RSTN DOWN
        gpio_direction_output( mfp_to_gpio( MFP_PIN_GPIO94 ), 0 ); // MCAM_RSTN
        CamacqExtDelay(10);
        
        // 2. MCLK DOWN
        gpio_direction_output( mfp_to_gpio( MFP_PIN_GPIO93 ), 0 ); // CAM_MCLK
        g_pstCamacqSensorManager->m_stCamBlock.CamPowerSet( 0 );
        CamacqExtDelay(10);
        
        // 3. STAND-BY DOWN
        gpio_direction_output( mfp_to_gpio( MFP_PIN_GPIO97 ), 0 ); // MCAM_STBYN
        CamacqExtDelay(10);
        
        // 4. Power-OFf
        // VCAM_IO    
        if(reg_cam_io)
            regulator_disable(reg_cam_io);    
        else {
            CamacqTraceErr(" ### CAM_IO regulator is NULL ...! ");
            iRet = -1;
        }

        gpio_direction_output( mfp_to_gpio( MFP_PIN_GPIO131 ), 0 );    // CORE
        gpio_direction_output( mfp_to_gpio( MFP_PIN_GPIO108 ), 0 );    // CAM_EN1 ---> VCAM_AVDD_2.8V
        gpio_direction_output( mfp_to_gpio( MFP_PIN_GPIO109 ), 0 );    // CAM_EN2 ---> VCAM_AF_2.8V

        CamacqExtFreeGpio( mfp_to_gpio(MFP_PIN_GPIO108) );  // CAM_EN1 ---> VCAM_AVDD_2.8V
        CamacqExtFreeGpio( mfp_to_gpio(MFP_PIN_GPIO109) );  // CAM_EN2 ---> VCAM_AF_2.8V 
        CamacqExtFreeGpio( mfp_to_gpio(MFP_PIN_GPIO93) );  // CAM_MCLK
        CamacqExtFreeGpio( mfp_to_gpio(MFP_PIN_GPIO97) );  // MCAM_STBYN
        CamacqExtFreeGpio( mfp_to_gpio(MFP_PIN_GPIO94) );  // MCAM_RSTN
        CamacqExtFreeGpio( mfp_to_gpio(MFP_PIN_GPIO131) );  // MCAM_CORE
    }
#elif defined(CAMACQ_MODEL_JETTA)
#if defined(CONFIG_PMIC_D1980)
    {
#if 1   // Dual-Power-Sequence
        CamacqTraceDbg(" JETTA ISX012 Dual-POWER-OFF CONFIG_PMIC_D1980");

        CamacqExtRequestGpio(mfp_to_gpio( MFP_PIN_GPIO96 ), "camacq");
        CamacqExtRequestGpio(mfp_to_gpio( MFP_PIN_GPIO95 ), "camacq");
        CamacqExtRequestGpio(mfp_to_gpio( MFP_PIN_GPIO94 ), "camacq");
        CamacqExtRequestGpio(mfp_to_gpio( MFP_PIN_GPIO97 ), "camacq");
        CamacqExtRequestGpio(mfp_to_gpio( MFP_PIN_GPIO131 ), "camacq");
        CamacqExtRequestGpio(mfp_to_gpio( MFP_PIN_GPIO93 ), "camacq");
        CamacqExtRequestGpio(mfp_to_gpio( MFP_PIN_GPIO118 ), "camacq"); // RF_IF6, for REV0.4

        // 0. Standby
        gpio_direction_output( mfp_to_gpio( MFP_PIN_GPIO97 ), 0 ); // GPIO97 MCAM_STBYN
        CamacqExtDelay(1);

        // reset down
        gpio_direction_output( mfp_to_gpio( MFP_PIN_GPIO94 ), 0 ); // MCAM_RSTN
        CamacqExtDelay(1);

        // Clock
        gpio_direction_output( mfp_to_gpio( MFP_PIN_GPIO93 ), 0 ); // GPIO93 -> CAM_MCLK
        g_pstCamacqSensorManager->m_stCamBlock.CamPowerSet( 0 );
        CamacqExtDelay(10);

        // VCAM_AF_2.8V
        if(reg_cam_af == NULL) {
            reg_cam_af = regulator_get(&(g_pstCamacqSensorManager->m_stCamBlock.pdev->dev), REGULATOR_CAM_AF);
        }    
        if(reg_cam_af) {
            regulator_disable(reg_cam_af);
        } else {
            CamacqTraceErr(" ### CAM_AF regulator is NULL ...! ");
            iRet = -1;
        }
        CamacqExtDelay(1);
        
        // VCAM_AVDD_2.8V
        if(reg_cam_avdd == NULL) {
            reg_cam_avdd = regulator_get(&(g_pstCamacqSensorManager->m_stCamBlock.pdev->dev), REGULATOR_CAM_AVDD);
        }
        if(reg_cam_avdd) {
            regulator_disable(reg_cam_avdd);
        } else {
            CamacqTraceErr(" ### CAM_AVDD regulator is NULL ...! ");
            iRet = -1;
        }

        gpio_direction_output( mfp_to_gpio( MFP_PIN_GPIO118 ), 0 ); // RF_IF6(GPIO118) for REV0.4

        CamacqExtDelay(1);
        
        // VCAM_IO_1.8V
        if(reg_cam_io == NULL) {
            reg_cam_io = regulator_get(&(g_pstCamacqSensorManager->m_stCamBlock.pdev->dev), REGULATOR_CAM_IO);
        }
        if(reg_cam_io)
        {
            regulator_disable(reg_cam_io);    
        } else {
            CamacqTraceErr(" ### CAM_IO regulator is NULL ...! ");
            iRet = -1;
        }
        CamacqExtDelay(1);

        // VCAM_M_CORE_1.2V 
        gpio_direction_output( mfp_to_gpio( MFP_PIN_GPIO131 ), 0 ); // RF_IF19(GPIO131) -> SWITCH_EN -> VCAM_M_CORE_1.2V 
        CamacqExtDelay(1);

        // CAM_S_CORE
        if(reg_cam_sub_core == NULL)
            reg_cam_sub_core = regulator_get(&(g_pstCamacqSensorManager->m_stCamBlock.pdev->dev), REGULATOR_CAM_S_CORE);
        if(reg_cam_sub_core) {
            regulator_disable(reg_cam_sub_core);    
        } else {
            CamacqTraceErr(" ### CAM_S_CORE regulator is NULL ...! ");
            iRet = -1;
        } 
        CamacqExtDelay(1);


        CamacqTraceDbg("WINIG TEST 111111111111111111111111111111");
        
        // MAIN-SUB CAM RSTN, STAND BY ALL OFF
        gpio_direction_output( mfp_to_gpio( MFP_PIN_GPIO96 ), 0 );      // SCAM_RSTN
        gpio_direction_output( mfp_to_gpio( MFP_PIN_GPIO95 ), 0 );      // SCAM_STBYN
        gpio_direction_output( mfp_to_gpio( MFP_PIN_GPIO94 ), 0 );      // MCAM_RSTN
        gpio_direction_output( mfp_to_gpio( MFP_PIN_GPIO97 ), 0 );      // MCAM_STBYN

        CamacqExtFreeGpio(mfp_to_gpio( MFP_PIN_GPIO96 ));
        CamacqExtFreeGpio(mfp_to_gpio( MFP_PIN_GPIO95 ));
        CamacqExtFreeGpio(mfp_to_gpio( MFP_PIN_GPIO94 ));
        CamacqExtFreeGpio(mfp_to_gpio( MFP_PIN_GPIO97 ));
        CamacqExtFreeGpio(mfp_to_gpio( MFP_PIN_GPIO131 ));
        CamacqExtFreeGpio(mfp_to_gpio( MFP_PIN_GPIO93 ));    
        CamacqExtFreeGpio(mfp_to_gpio( MFP_PIN_GPIO118 ));    
#else
        CamacqTraceErr(" JETTA ISX012 POWER-OFF CONFIG_PMIC_D1980");

        CamacqExtRequestGpio(mfp_to_gpio( MFP_PIN_GPIO94 ), "camacq");
        CamacqExtRequestGpio(mfp_to_gpio( MFP_PIN_GPIO97 ), "camacq");
        CamacqExtRequestGpio(mfp_to_gpio( MFP_PIN_GPIO131 ), "camacq");
        CamacqExtRequestGpio(mfp_to_gpio( MFP_PIN_GPIO93 ), "camacq");

        // 0. Standby
        gpio_direction_output( mfp_to_gpio( MFP_PIN_GPIO97 ), 0 ); // GPIO97 MCAM_STBYN
        CamacqExtDelay(10);

        // reset down
        gpio_direction_output( mfp_to_gpio( MFP_PIN_GPIO94 ), 0 ); // MCAM_RSTN
        CamacqExtDelay(10);

        // Clock
        gpio_direction_output( mfp_to_gpio( MFP_PIN_GPIO93 ), 0 ); // GPIO93 -> CAM_MCLK
        g_pstCamacqSensorManager->m_stCamBlock.CamPowerSet( 0 );
        CamacqExtDelay(10);

        // VCAM_AF_2.8V
        if(reg_cam_af == NULL) {
            reg_cam_af = regulator_get(&(g_pstCamacqSensorManager->m_stCamBlock.pdev->dev), REGULATOR_CAM_AF);
        }    
        if(reg_cam_af) {
            regulator_disable(reg_cam_af);
        } else {
            CamacqTraceErr(" ### CAM_AF regulator is NULL ...! ");
            iRet = -1;
        }
        CamacqExtDelay(10);
        
        // VCAM_AVDD_2.8V
        if(reg_cam_avdd == NULL) {
            reg_cam_avdd = regulator_get(&(g_pstCamacqSensorManager->m_stCamBlock.pdev->dev), REGULATOR_CAM_AVDD);
        }
        if(reg_cam_avdd) {
            regulator_disable(reg_cam_avdd);
        } else {
            CamacqTraceErr(" ### CAM_AVDD regulator is NULL ...! ");
            iRet = -1;
        }
        CamacqExtDelay(10);
        
        // VCAM_IO_1.8V
        if(reg_cam_io == NULL) {
            reg_cam_io = regulator_get(&(g_pstCamacqSensorManager->m_stCamBlock.pdev->dev), REGULATOR_CAM_IO);
        }
        if(reg_cam_io)
        {
            regulator_disable(reg_cam_io);    
        } else {
            CamacqTraceErr(" ### CAM_IO regulator is NULL ...! ");
            iRet = -1;
        }
        CamacqExtDelay(10);

        // VCAM_M_CORE_1.2V 
        gpio_direction_output( mfp_to_gpio( MFP_PIN_GPIO131 ), 0 ); // RF_IF19(GPIO131) -> SWITCH_EN -> VCAM_M_CORE_1.2V 
        CamacqExtDelay(10);

        CamacqExtFreeGpio(mfp_to_gpio( MFP_PIN_GPIO94 ));
        CamacqExtFreeGpio(mfp_to_gpio( MFP_PIN_GPIO97 ));
        CamacqExtFreeGpio(mfp_to_gpio( MFP_PIN_GPIO131 ));
        CamacqExtFreeGpio(mfp_to_gpio( MFP_PIN_GPIO93 ));    
#endif
    }
#else /* CONFIG_PMIC_D1980 */
    {
        CamacqTraceErr(" JETTA ISX012 POWER-OFF ");
    }
#endif /* CONFIG_PMIC_D1980 */
    
#elif defined(CAMACQ_MODEL_GFORCE_OLD)

#if 0
#if defined(_S5K4ECGX_EVT1_MIPI_) /* SENSOR OPTION e */

    CamacqExtRequestGpio(mfp_to_gpio( MFP_PIN_GPIO94 ), "camacq");
    CamacqExtRequestGpio(mfp_to_gpio( MFP_PIN_GPIO97 ), "camacq");
    CamacqExtRequestGpio(mfp_to_gpio( MFP_PIN_GPIO109 ), "camacq");
    CamacqExtRequestGpio(mfp_to_gpio( MFP_PIN_GPIO3 ), "camacq");
    CamacqExtRequestGpio(mfp_to_gpio( MFP_PIN_GPIO108 ), "camacq");
    CamacqExtRequestGpio(mfp_to_gpio( MFP_PIN_GPIO93 ), "camacq");

    // reset down
    gpio_direction_output( mfp_to_gpio( MFP_PIN_GPIO94 ), CAMACQ_MAIN_RST_OFF ); // GPIO94 -> MCAM_RSTN
    CamacqExtDelay(50);

    // Clock
    gpio_direction_output( mfp_to_gpio( MFP_PIN_GPIO93 ), 0 ); // GPIO93 -> CAM_MCLK
    g_pstCamacqSensorManager->m_stCamBlock.CamPowerSet( CAMACQ_OFF );
    CamacqExtDelay(10);

    // Standby
    gpio_direction_output( mfp_to_gpio( MFP_PIN_GPIO97 ), CAMACQ_OFF ); // GPIO97 MCAM_STBYN
    CamacqExtDelay(10);

    // Power off
    gpio_direction_output( mfp_to_gpio( MFP_PIN_GPIO109 ), CAMACQ_MAIN_EN_OFF ); // GPIO109  VCAM_CORE_1.2V
    gpio_direction_output( mfp_to_gpio( MFP_PIN_GPIO108 ), CAMACQ_MAIN_EN_OFF ); // GPIO108 VCAM_IO_2.8V
    gpio_direction_output( mfp_to_gpio( MFP_PIN_GPIO3 ), CAMACQ_MAIN_EN_OFF ); // GPIO103 VCAM_AVDD_2.8V	& VCAM_AF_2.8V
    CamacqExtDelay(10);

    CamacqExtFreeGpio(mfp_to_gpio( MFP_PIN_GPIO94 ));
    CamacqExtFreeGpio(mfp_to_gpio( MFP_PIN_GPIO97 ));
    CamacqExtFreeGpio(mfp_to_gpio( MFP_PIN_GPIO109 ));
    CamacqExtFreeGpio(mfp_to_gpio( MFP_PIN_GPIO3 ));
    CamacqExtFreeGpio(mfp_to_gpio( MFP_PIN_GPIO108 ));
    CamacqExtFreeGpio(mfp_to_gpio( MFP_PIN_GPIO93 ));
   
#elif defined(__ISX006_SONY__)

    // Standby down
    gpio_direction_output( mfp_to_gpio( MFP_PIN_GPIO97 ), CAMACQ_OFF ); // GPIO97 MCAM_STBYN
    CamacqExtDelay(10);

    // reset down
    gpio_direction_output( mfp_to_gpio( MFP_PIN_GPIO94 ), CAMACQ_MAIN_RST_OFF ); // GPIO94 -> MCAM_RSTN
    CamacqExtDelay(10);

    // Clock
    gpio_direction_output( mfp_to_gpio( MFP_PIN_GPIO93 ), 0 ); // GPIO93 -> CAM_MCLK
    g_pstCamacqSensorManager->m_stCamBlock.CamPowerSet( CAMACQ_OFF );
    CamacqExtDelay(10);

    // Power off
    gpio_direction_output( mfp_to_gpio( MFP_PIN_GPIO109 ), CAMACQ_MAIN_EN_OFF ); // GPIO109  VCAM_CORE_1.2V
    gpio_direction_output( mfp_to_gpio( MFP_PIN_GPIO108 ), CAMACQ_MAIN_EN_OFF ); // GPIO108 VCAM_IO_2.8V
    gpio_direction_output( mfp_to_gpio( MFP_PIN_GPIO3 ), CAMACQ_MAIN_EN_OFF ); // GPIO103 VCAM_AVDD_2.8V	& VCAM_AF_2.8V
    CamacqExtDelay(10);
/*
    // Standby
    gpio_direction_output( mfp_to_gpio( MFP_PIN_GPIO114 ), CAMACQ_OFF ); // RF_IF2 -> MCAM_STBYN
    CamacqExtDelay(10);

    // reset down
    gpio_direction_output( mfp_to_gpio( MFP_PIN_GPIO113 ), CAMACQ_MAIN_RST_OFF ); // RF_IF1 -> MCAM_RSTN
    CamacqExtDelay(10);

    // Clock
    gpio_direction_output( mfp_to_gpio( MFP_PIN_GPIO93 ), 0 ); // GPIO93 -> CAM_MCLK
    CamacqExtDelay(10);

    // Power
    gpio_direction_output( mfp_to_gpio( MFP_PIN_GPIO107 ), CAMACQ_MAIN_EN_OFF ); // GPIO107 -> CAM_EN2 -> VCAM_CORE_1.2V
    gpio_direction_output( mfp_to_gpio( MFP_PIN_GPIO106 ), CAMACQ_MAIN_EN_OFF ); // GPIO106 -> CAM_EN -> VCAM_IO_2.8V
    gpio_direction_output( mfp_to_gpio( MFP_PIN_GPIO109 ), CAMACQ_MAIN_EN_OFF ); // GPIO109 -> CAM_EN4 -> VCAM_AVDD_2.8V	& VCAM_AF_2.8V
    CamacqExtDelay(10);
*/
#endif
#endif

#else

#endif  /* SENSOR OPTION e */

    CamacqTraceOUT();
    return iRet;
}

/* SUB H/W */
#if (CAMACQ_SENSOR_MAX==2)
S32 Switch_SUB(void)
{
    S32 iRet = 0;
    CamacqTraceIN();

   
    CamacqTraceOUT();
    return iRet;
}

S32 Reset_SUB(void)
{
    S32 iRet = 0;
    CamacqTraceIN();

    gpio_direction_output( mfp_to_gpio( MFP_PIN_GPIO94 ), CAMACQ_OFF ); // GPIO94 -> SCAM_RSTN
    CamacqExtDelay(10);
    gpio_direction_output( mfp_to_gpio( MFP_PIN_GPIO94 ), CAMACQ_ON ); //  GPIO94 -> SCAM_RSTN
    
    CamacqTraceOUT();
    return iRet;
}

S32 Standby_SUB(void)
{
    S32 iRet = 0;
    CamacqTraceIN();

       
    CamacqTraceOUT();
    return iRet;
}

S32 Wakeup_SUB(void)
{
    S32 iRet = 0;
    CamacqTraceIN();

    CamacqTraceOUT();
    return iRet;
}

S32 PowerOn_SUB(void)
{
    S32 iRet = 0;
    CamacqTraceIN();

#if defined(CAMACQ_MODEL_GFORCE_OLD)
#if 0
#if defined(CONFIG_PMIC_D1980)
    CamacqTraceDbg("ALKON MT9V114 !!!!!!!");
    CamacqTraceDbg("ALKON MT9V114 !!!!!!!");
    CamacqTraceDbg("ALKON MT9V114 !!!!!!!");

    CamacqExtRequestGpio( mfp_to_gpio(MFP_PIN_GPIO39), "camacq" );  // CAM_SW_EN
    CamacqExtRequestGpio( mfp_to_gpio(MFP_PIN_GPIO52), "camacq" );  // CAM_SEL
    CamacqExtRequestGpio( mfp_to_gpio(MFP_PIN_GPIO93), "camacq" );  // CAM_MCLK
    CamacqExtRequestGpio( mfp_to_gpio(MFP_PIN_GPIO95), "camacq" );  // SCAM_STBYN
    CamacqExtRequestGpio( mfp_to_gpio(MFP_PIN_GPIO96), "camacq" );  // SCAM_RSTN

    /* MIPI Switch */ 
    gpio_direction_output( mfp_to_gpio( MFP_PIN_GPIO39 ), 0 ); // GPIO39 -> CAM_SW_EN -> LOW_ENABLE
    gpio_direction_output( mfp_to_gpio( MFP_PIN_GPIO52 ), 1 ); // GPIO52 -> CAM_SEL -> 1 is B
    CamacqExtDelay(10);

    /* Reset-up */
    gpio_direction_output( mfp_to_gpio( MFP_PIN_GPIO96 ), 1 );   // SCAM_RSTN

    /* Power-up */
    // VCAM_IO    
    if(reg_cam_io == NULL)
        reg_cam_io = regulator_get(&(g_pstCamacqSensorManager->m_stCamBlock.pdev->dev), 
                                        REGULATOR_CAM_IO);
    if(reg_cam_io) {
        regulator_enable(reg_cam_io);    
        CamacqExtDelay(10);
    } else {
        CamacqTraceErr(" ### CAM_IO regulator is NULL ...! ");
        iRet = -1;
    }   

    // AVDD
    if(reg_cam_avdd == NULL)
        reg_cam_avdd = regulator_get(&(g_pstCamacqSensorManager->m_stCamBlock.pdev->dev), 
                                        REGULATOR_CAM_AVDD);
    if(reg_cam_avdd) {
        regulator_enable(reg_cam_avdd);    
        CamacqExtDelay(10);
    } else {
        CamacqTraceErr(" ### CAM_AVDD regulator is NULL ...! ");
        iRet = -1;
    }   

    // CORE
    if(reg_cam_sub_core == NULL)
        reg_cam_sub_core = regulator_get(&(g_pstCamacqSensorManager->m_stCamBlock.pdev->dev), 
                                        REGULATOR_CAM_S_CORE);
    if(reg_cam_sub_core) {
        regulator_enable(reg_cam_sub_core);    
        CamacqExtDelay(10);
    } else {
        CamacqTraceErr(" ### CAM_S_CORE regulator is NULL ...! ");
        iRet = -1;
    } 

    /* Reset-down */
    gpio_direction_output( mfp_to_gpio( MFP_PIN_GPIO96 ), 0 );   // SCAM_RSTN
    CamacqExtDelay(200);
    
    // MCLK
    gpio_direction_output( mfp_to_gpio( MFP_PIN_GPIO93 ), 1 );    // CAM_MCLK
    g_pstCamacqSensorManager->m_stCamBlock.CamPowerSet( 1 );
    CamacqExtDelay(200);

    /* Reset-up */
    gpio_direction_output( mfp_to_gpio( MFP_PIN_GPIO96 ), 1 );   // SCAM_RSTN
    CamacqExtDelay(200);
    

    CamacqExtFreeGpio( mfp_to_gpio(MFP_PIN_GPIO39) );  // CAM_SW_EN
    CamacqExtFreeGpio( mfp_to_gpio(MFP_PIN_GPIO52) );  // CAM_SEL
    CamacqExtFreeGpio( mfp_to_gpio(MFP_PIN_GPIO93) );  // CAM_MCLK
    CamacqExtFreeGpio( mfp_to_gpio(MFP_PIN_GPIO95) );  // SCAM_STBYN
    CamacqExtFreeGpio( mfp_to_gpio(MFP_PIN_GPIO96) );  // SCAM_RSTN
#else
    CamacqTraceDbg("ALKON MT9V114 !!!!!!!");
    CamacqTraceDbg("ALKON MT9V114 !!!!!!!");
    CamacqTraceDbg("ALKON MT9V114 !!!!!!!");

    CamacqExtRequestGpio( mfp_to_gpio(MFP_PIN_GPIO39), "camacq" );  // CAM_SW_EN
    CamacqExtRequestGpio( mfp_to_gpio(MFP_PIN_GPIO52), "camacq" );  // CAM_SEL
    CamacqExtRequestGpio( mfp_to_gpio(MFP_PIN_GPIO108), "camacq" ); // CAM_EN1 ---> VCAM_AVDD_2.8V
    CamacqExtRequestGpio( mfp_to_gpio(MFP_PIN_GPIO93), "camacq" );  // CAM_MCLK
    CamacqExtRequestGpio( mfp_to_gpio(MFP_PIN_GPIO95), "camacq" );  // SCAM_STBYN
    CamacqExtRequestGpio( mfp_to_gpio(MFP_PIN_GPIO96), "camacq" );  // SCAM_RSTN

    /* MIPI Switch */ 
    gpio_direction_output( mfp_to_gpio( MFP_PIN_GPIO39 ), 0 ); // GPIO39 -> CAM_SW_EN -> LOW_ENABLE
    gpio_direction_output( mfp_to_gpio( MFP_PIN_GPIO52 ), 1 ); // GPIO52 -> CAM_SEL -> 1 is B
    CamacqExtDelay(10);

    /* Reset-up */
    gpio_direction_output( mfp_to_gpio( MFP_PIN_GPIO96 ), 1 );   // SCAM_RSTN

    /* Power-up */
    // VCAM_IO    
    if( A == 0 )
    {
        CamacqTraceDbg("WINGI 2222");
        reg_ldo2 = regulator_get(&(g_pstCamacqSensorManager->m_stCamBlock.pdev->dev), "v_cam");
        A = 1;
    }
    if( reg_ldo2 != NULL )
    {
        // regulator_set_voltage(reg, OUTPUT_2_8V, OUTPUT_2_8V);
        regulator_enable(reg_ldo2);    
        CamacqExtDelay(10);
    }
    else
    {
        CamacqTraceErr(" v_ldo2 ########################## reg is NULL ##################### ");
        iRet = -1;
    }   

    // AVDD
    gpio_direction_output( mfp_to_gpio( MFP_PIN_GPIO108 ), 1 );   // CAM_EN1 ---> VCAM_AVDD_2.8V

    // CORE
    if( B == 0 )
    {
        CamacqTraceDbg("WINGI 2222");
        reg_buck1 = regulator_get(&(g_pstCamacqSensorManager->m_stCamBlock.pdev->dev), "v_buck1");
        B = 1;
    }
    if( reg_buck1 != NULL )
    {
        // regulator_set_voltage(reg, OUTPUT_2_8V, OUTPUT_2_8V);
        regulator_enable(reg_buck1);    
        CamacqExtDelay(10);
    }
    else
    {
        CamacqTraceErr(" v_buck1 ########################## reg is NULL ##################### ");
        iRet = -1;
    } 

    /* Reset-down */
    gpio_direction_output( mfp_to_gpio( MFP_PIN_GPIO96 ), 0 );   // SCAM_RSTN
    CamacqExtDelay(200);
    
    // MCLK
    gpio_direction_output( mfp_to_gpio( MFP_PIN_GPIO93 ), 1 );    // CAM_MCLK
    g_pstCamacqSensorManager->m_stCamBlock.CamPowerSet( 1 );
    CamacqExtDelay(200);

    /* Reset-up */
    gpio_direction_output( mfp_to_gpio( MFP_PIN_GPIO96 ), 1 );   // SCAM_RSTN
    CamacqExtDelay(200);
    

    CamacqExtFreeGpio( mfp_to_gpio(MFP_PIN_GPIO39) );  // CAM_SW_EN
    CamacqExtFreeGpio( mfp_to_gpio(MFP_PIN_GPIO52) );  // CAM_SEL
    CamacqExtFreeGpio( mfp_to_gpio(MFP_PIN_GPIO108 )); // CAM_EN1 ---> VCAM_AVDD_2.8V
    CamacqExtFreeGpio( mfp_to_gpio(MFP_PIN_GPIO93) );  // CAM_MCLK
    CamacqExtFreeGpio( mfp_to_gpio(MFP_PIN_GPIO95) );  // SCAM_STBYN
    CamacqExtFreeGpio( mfp_to_gpio(MFP_PIN_GPIO96) );  // SCAM_RSTN
#endif /* CONFIG_PMIC_D1980 */
#endif // #if 0
#elif defined(CAMACQ_MODEL_JETTA)
#if defined(CONFIG_PMIC_D1980)
    {
#if 1
        CamacqTraceErr("JETTA MT9V114 DUAL-POWER-ON CONFIG_PMIC_D1980 !!!!!!! ");
        
        CamacqExtRequestGpio( mfp_to_gpio(MFP_PIN_GPIO39), "camacq" );  // CAM_SW_EN
        CamacqExtRequestGpio( mfp_to_gpio(MFP_PIN_GPIO52), "camacq" );  // CAM_SEL
        CamacqExtRequestGpio( mfp_to_gpio(MFP_PIN_GPIO93), "camacq" );  // CAM_MCLK
        CamacqExtRequestGpio(mfp_to_gpio( MFP_PIN_GPIO94 ), "camacq"); // MCAM_RSTN
        CamacqExtRequestGpio(mfp_to_gpio( MFP_PIN_GPIO97 ), "camacq"); // MCAM_STBYN
        CamacqExtRequestGpio(mfp_to_gpio( MFP_PIN_GPIO96 ), "camacq"); // SCAM_RSTN
        CamacqExtRequestGpio(mfp_to_gpio(MFP_PIN_GPIO95 ), "camacq");  // SCAM_STBYN
        CamacqExtRequestGpio(mfp_to_gpio(MFP_PIN_GPIO131 ), "camacq");  // MCAM_CORE
        CamacqExtRequestGpio(mfp_to_gpio(MFP_PIN_GPIO118 ), "camacq");  // RF_IF6

        // Prepare-condition
        gpio_direction_output( mfp_to_gpio( MFP_PIN_GPIO96 ), 1 );      // SCAM_RSTN
        gpio_direction_output( mfp_to_gpio( MFP_PIN_GPIO95 ), 0 );      // SCAM_STBYN
        gpio_direction_output( mfp_to_gpio( MFP_PIN_GPIO94 ), 0 );      // MCAM_RSTN
        gpio_direction_output( mfp_to_gpio( MFP_PIN_GPIO97 ), 0 );      // MCAM_STBYN
        CamacqExtDelay(10);

        /* MIPI Switch */ 
        gpio_direction_output( mfp_to_gpio( MFP_PIN_GPIO39 ), 0 ); // GPIO39 -> CAM_SW_EN -> LOW_ENABLE
        gpio_direction_output( mfp_to_gpio( MFP_PIN_GPIO52 ), 1 ); // GPIO52 -> CAM_SEL -> 0 is A
        CamacqExtDelay(1);

        // VCAM_M_CORE_1.2V 
        gpio_direction_output( mfp_to_gpio( MFP_PIN_GPIO131 ), 1 ); // RF_IF19(GPIO131) -> SWITCH_EN -> VCAM_M_CORE_1.2V 
        CamacqExtDelay(1);

        // VCAM_IO_1.8V
        if(reg_cam_io == NULL)
            reg_cam_io = regulator_get(&(g_pstCamacqSensorManager->m_stCamBlock.pdev->dev), REGULATOR_CAM_IO);
        if(reg_cam_io)
            regulator_enable(reg_cam_io);    
        else {
            CamacqTraceErr(" ### CAM_IO regulator is NULL ...! ");
            iRet = -1;
        }
        CamacqExtDelay(1);

        // VCAM_AVDD_2.8V
        if(reg_cam_avdd == NULL)
            reg_cam_avdd = regulator_get(&(g_pstCamacqSensorManager->m_stCamBlock.pdev->dev), REGULATOR_CAM_AVDD);

        if(reg_cam_avdd)
            regulator_enable(reg_cam_avdd);
        else {
            CamacqTraceErr(" ### CAM_AVDD regulator is NULL ...! ");
            iRet = -1;
        }

        // VCAM_AVDD_2.8V for REV0.4
        gpio_direction_output( mfp_to_gpio( MFP_PIN_GPIO118 ), 1 );      // RF_IF6
        CamacqExtDelay(1); 

        // VT Sensor core
        if(reg_cam_sub_core == NULL)
            reg_cam_sub_core = regulator_get(&(g_pstCamacqSensorManager->m_stCamBlock.pdev->dev), REGULATOR_CAM_S_CORE);
        if(reg_cam_sub_core) {
            regulator_enable(reg_cam_sub_core);    
        } else {
            CamacqTraceErr(" ### CAM_S_CORE regulator is NULL ...! ");
            iRet = -1;
        } 
        CamacqExtDelay(10);

        

        // VT RESET-LOW
        gpio_direction_output( mfp_to_gpio( MFP_PIN_GPIO96 ), 0 );      // SCAM_RSTN
        CamacqExtDelay(10);

        // MCLK
        gpio_direction_output( mfp_to_gpio( MFP_PIN_GPIO93 ), 1 );
        g_pstCamacqSensorManager->m_stCamBlock.CamPowerSet( 1 );
        CamacqExtDelay(50);

        // MAIN RESET-HIGH
        gpio_direction_output( mfp_to_gpio( MFP_PIN_GPIO94 ), 1 );      // MCAM_RSTN
        CamacqExtDelay(1);

        // VT RESET-UP
        gpio_direction_output( mfp_to_gpio( MFP_PIN_GPIO96 ), 1 );      // SCAM_RSTN
        CamacqExtDelay(10);

        CamacqExtFreeGpio( mfp_to_gpio(MFP_PIN_GPIO39) );  // CAM_SW_EN
        CamacqExtFreeGpio( mfp_to_gpio(MFP_PIN_GPIO52) );  // CAM_SEL
        CamacqExtFreeGpio( mfp_to_gpio(MFP_PIN_GPIO93) );  // CAM_MCLK
        CamacqExtFreeGpio( mfp_to_gpio(MFP_PIN_GPIO96) );  // SCAM_RSTN
        CamacqExtFreeGpio( mfp_to_gpio(MFP_PIN_GPIO94) );  
        CamacqExtFreeGpio( mfp_to_gpio(MFP_PIN_GPIO95) );  
        CamacqExtFreeGpio( mfp_to_gpio(MFP_PIN_GPIO97) );
        CamacqExtFreeGpio( mfp_to_gpio(MFP_PIN_GPIO131) );
        CamacqExtFreeGpio( mfp_to_gpio(MFP_PIN_GPIO118) );
#else
        CamacqTraceErr("JETTA MT9V114 ON CONFIG_PMIC_D1980 !!!!!!! ");
        
        CamacqExtRequestGpio( mfp_to_gpio(MFP_PIN_GPIO39), "camacq" );  // CAM_SW_EN
        CamacqExtRequestGpio( mfp_to_gpio(MFP_PIN_GPIO52), "camacq" );  // CAM_SEL
        CamacqExtRequestGpio( mfp_to_gpio(MFP_PIN_GPIO93), "camacq" );  // CAM_MCLK
        CamacqExtRequestGpio( mfp_to_gpio(MFP_PIN_GPIO96), "camacq" );  // SCAM_RSTN

        /* MIPI Switch */ 
        gpio_direction_output( mfp_to_gpio( MFP_PIN_GPIO39 ), 0 ); // GPIO39 -> CAM_SW_EN -> LOW_ENABLE
        gpio_direction_output( mfp_to_gpio( MFP_PIN_GPIO52 ), 1 ); // GPIO52 -> CAM_SEL -> 1 is B
        CamacqExtDelay(10);

        /* Reset-up */
        gpio_direction_output( mfp_to_gpio( MFP_PIN_GPIO96 ), 1 );   // SCAM_RSTN

        // VCAM_IO    
        if(reg_cam_io == NULL)
            reg_cam_io = regulator_get(&(g_pstCamacqSensorManager->m_stCamBlock.pdev->dev), REGULATOR_CAM_IO);
        if(reg_cam_io) {
            regulator_enable(reg_cam_io);    
        } else {
            CamacqTraceErr(" ### CAM_IO regulator is NULL ...! ");
            iRet = -1;
        }   
        CamacqExtDelay(10);

        // AVDD
        if(reg_cam_avdd == NULL)
            reg_cam_avdd = regulator_get(&(g_pstCamacqSensorManager->m_stCamBlock.pdev->dev), REGULATOR_CAM_AVDD);
        if(reg_cam_avdd) {
            regulator_enable(reg_cam_avdd);    
        } else {
            CamacqTraceErr(" ### CAM_AVDD regulator is NULL ...! ");
            iRet = -1;
        }   
        CamacqExtDelay(10);    

        // CORE
        if(reg_cam_sub_core == NULL)
            reg_cam_sub_core = regulator_get(&(g_pstCamacqSensorManager->m_stCamBlock.pdev->dev), REGULATOR_CAM_S_CORE);
        if(reg_cam_sub_core) {
            regulator_enable(reg_cam_sub_core);    
        } else {
            CamacqTraceErr(" ### CAM_S_CORE regulator is NULL ...! ");
            iRet = -1;
        } 
        CamacqExtDelay(10);

        /* Reset-down */
        gpio_direction_output( mfp_to_gpio( MFP_PIN_GPIO96 ), 0 );   // SCAM_RSTN
        CamacqExtDelay(200);
        
        // MCLK
        gpio_direction_output( mfp_to_gpio( MFP_PIN_GPIO93 ), 1 );    // CAM_MCLK
        g_pstCamacqSensorManager->m_stCamBlock.CamPowerSet( 1 );
        CamacqExtDelay(200);

        /* Reset-up */
        gpio_direction_output( mfp_to_gpio( MFP_PIN_GPIO96 ), 1 );   // SCAM_RSTN
        CamacqExtDelay(200);

        CamacqExtFreeGpio( mfp_to_gpio(MFP_PIN_GPIO39) );  // CAM_SW_EN
        CamacqExtFreeGpio( mfp_to_gpio(MFP_PIN_GPIO52) );  // CAM_SEL
        CamacqExtFreeGpio( mfp_to_gpio(MFP_PIN_GPIO93) );  // CAM_MCLK
        CamacqExtFreeGpio( mfp_to_gpio(MFP_PIN_GPIO96) );  // SCAM_RSTN
#endif
    }
#else /* CONFIG_PMIC_D1980 */
    {
        CamacqTraceDbg("JETTA MT9V114 ON !!!!!!!");
    }
#endif /* CONFIG_PMIC_D1980 */
#else
#if 0
    /* Switch */
    gpio_direction_output( mfp_to_gpio( MFP_PIN_GPIO101 ), 0 ); // GPIO101 -> CAM_SW_EN, LOW_ENABLE
    gpio_direction_output( mfp_to_gpio( MFP_PIN_GPIO100 ), 1 ); // GPIO100 -> CAM_SEL, 1 is B, B is SCAM
    CamacqExtDelay(10);

    // 0. both reset, stdny all off 
    gpio_direction_output( mfp_to_gpio( MFP_PIN_GPIO113 ), CAMACQ_MAIN_RST_OFF );   // RF_IF1 -> MCAM_RSTN
    gpio_direction_output( mfp_to_gpio( MFP_PIN_GPIO94 ), CAMACQ_SUB_RST_OFF );     // GPIO94 -> SCAM_RSTN
    gpio_direction_output( mfp_to_gpio( MFP_PIN_GPIO114 ), CAMACQ_OFF ); // RF_IF2 -> MCAM_STBYN
    gpio_direction_output( mfp_to_gpio( MFP_PIN_GPIO97 ), CAMACQ_OFF ); // GPIO97 -> SCAM_STBYN
    
    // 1. both power up
    gpio_direction_output( mfp_to_gpio( MFP_PIN_GPIO109 ), CAMACQ_MAIN_EN_ON ); // GPIO109 -> CAM_EN4 -> VCAM_AVDD_2.8V	& VCAM_AF_2.8V
    CamacqExtDelay(1);
    gpio_direction_output( mfp_to_gpio( MFP_PIN_GPIO108 ), CAMACQ_SUB_EN_ON ); // GPIO108 -> CAM_EN3 -> VCAM_DVDD_1.5V
    CamacqExtDelay(1);
    gpio_direction_output( mfp_to_gpio( MFP_PIN_GPIO107 ), CAMACQ_MAIN_EN_ON ); // GPIO107 -> CAM_EN2 -> VCAM_CORE_1.2V
    CamacqExtDelay(1);
    gpio_direction_output( mfp_to_gpio( MFP_PIN_GPIO106 ), CAMACQ_MAIN_EN_ON ); // GPIO106 -> CAM_EN -> VCAM_IO_2.8V
    CamacqExtDelay(1);

    // 2. sub stdby high
    gpio_direction_output( mfp_to_gpio( MFP_PIN_GPIO97 ), CAMACQ_ON ); // GPIO97 -> SCAM_STBYN
    CamacqExtDelay(1);

    // 3. MCLK
    gpio_direction_output( mfp_to_gpio( MFP_PIN_GPIO93 ), 1 );
    CamacqExtDelay(10);

    // 4. main reset high
    gpio_direction_output( mfp_to_gpio( MFP_PIN_GPIO113 ), CAMACQ_MAIN_RST_ON );   // RF_IF1 -> MCAM_RSTN
    CamacqExtDelay(10);

    // 5. sub reset high
    gpio_direction_output( mfp_to_gpio( MFP_PIN_GPIO94 ), CAMACQ_SUB_RST_ON );     // GPIO94 -> SCAM_RSTN
    CamacqExtDelay(1);
#endif // #if 0
#endif

    CamacqTraceOUT();
    return iRet;
}

S32 PowerOff_SUB(void)
{
    S32 iRet = 0;
    CamacqTraceIN();
    
#if defined(CAMACQ_MODEL_GFORCE_OLD)
#if 0
#if defined(CONFIG_PMIC_D1980)
    CamacqExtRequestGpio( mfp_to_gpio(MFP_PIN_GPIO39), "camacq" );  // CAM_SW_EN
    CamacqExtRequestGpio( mfp_to_gpio(MFP_PIN_GPIO52), "camacq" );  // CAM_SEL
    CamacqExtRequestGpio( mfp_to_gpio(MFP_PIN_GPIO93), "camacq" );  // CAM_MCLK
    CamacqExtRequestGpio( mfp_to_gpio(MFP_PIN_GPIO95), "camacq" );  // SCAM_STBYN
    CamacqExtRequestGpio( mfp_to_gpio(MFP_PIN_GPIO96), "camacq" );  // SCAM_RSTN

    /* Reset-down */
    gpio_direction_output( mfp_to_gpio( MFP_PIN_GPIO96 ), 0 );   // SCAM_RSTN

    // MCLK-off
    gpio_direction_output( mfp_to_gpio( MFP_PIN_GPIO93 ), 0 );    // CAM_MCLK
    g_pstCamacqSensorManager->m_stCamBlock.CamPowerSet( 0 );
    CamacqExtDelay(30);

    /* Reset-up */
    gpio_direction_output( mfp_to_gpio( MFP_PIN_GPIO96 ), 1 );   // SCAM_RSTN

    // CORE
    if(reg_cam_sub_core) {
        regulator_disable(reg_cam_sub_core);    
        CamacqExtDelay(10);
    } else {
        CamacqTraceErr(" ### CAM_S_CORE regulator is NULL ...! ");
        iRet = -1;
    }  

    // AVDD
    if(reg_cam_avdd) {
        regulator_disable(reg_cam_avdd);    
        CamacqExtDelay(10);
    } else {
        CamacqTraceErr(" ### CAM_AVDD regulator is NULL ...! ");
        iRet = -1;
    }  

    // VCAM_IO
    if(reg_cam_io) {
        regulator_disable(reg_cam_io);    
        CamacqExtDelay(10);
    } else {
        CamacqTraceErr(" ### CAM_IO regulator is NULL ...! ");
        iRet = -1;
    }  

    CamacqExtFreeGpio( mfp_to_gpio(MFP_PIN_GPIO39) );  // CAM_SW_EN
    CamacqExtFreeGpio( mfp_to_gpio(MFP_PIN_GPIO52) );  // CAM_SEL
    CamacqExtFreeGpio( mfp_to_gpio(MFP_PIN_GPIO93) );  // CAM_MCLK
    CamacqExtFreeGpio( mfp_to_gpio(MFP_PIN_GPIO95) );  // SCAM_STBYN
    CamacqExtFreeGpio( mfp_to_gpio(MFP_PIN_GPIO96) );  // SCAM_RSTN
#else
    CamacqExtRequestGpio( mfp_to_gpio(MFP_PIN_GPIO39), "camacq" );  // CAM_SW_EN
    CamacqExtRequestGpio( mfp_to_gpio(MFP_PIN_GPIO52), "camacq" );  // CAM_SEL
    CamacqExtRequestGpio( mfp_to_gpio(MFP_PIN_GPIO108), "camacq" ); // CAM_EN1 ---> VCAM_AVDD_2.8V
    CamacqExtRequestGpio( mfp_to_gpio(MFP_PIN_GPIO93), "camacq" );  // CAM_MCLK
    CamacqExtRequestGpio( mfp_to_gpio(MFP_PIN_GPIO95), "camacq" );  // SCAM_STBYN
    CamacqExtRequestGpio( mfp_to_gpio(MFP_PIN_GPIO96), "camacq" );  // SCAM_RSTN


    /* Reset-down */
    gpio_direction_output( mfp_to_gpio( MFP_PIN_GPIO96 ), 0 );   // SCAM_RSTN

    // MCLK-off
    gpio_direction_output( mfp_to_gpio( MFP_PIN_GPIO93 ), 0 );    // CAM_MCLK
    g_pstCamacqSensorManager->m_stCamBlock.CamPowerSet( 0 );
    CamacqExtDelay(30);

    /* Reset-up */
    gpio_direction_output( mfp_to_gpio( MFP_PIN_GPIO96 ), 1 );   // SCAM_RSTN

    // CORE
    if( reg_buck1 != NULL )
    {
        // regulator_set_voltage(reg, OUTPUT_2_8V, OUTPUT_2_8V);
        regulator_disable(reg_buck1);    
        CamacqExtDelay(10);
    }
    else
    {
        CamacqTraceErr(" v_buck1 ########################## reg is NULL ##################### ");
        iRet = -1;
    } 

    // AVDD
    gpio_direction_output( mfp_to_gpio( MFP_PIN_GPIO108 ), 0 );   // CAM_EN1 ---> VCAM_AVDD_2.8V

    // VCAM_IO    
    // reg = regulator_get(NULL, "v_cam");
    if( reg_ldo2 != NULL )
    {
        // regulator_set_voltage(reg, OUTPUT_2_8V, OUTPUT_2_8V);
        regulator_disable(reg_ldo2);    
        CamacqExtDelay(10);
    }
    else
    {
        CamacqTraceErr(" v_ldo2 ########################## reg is NULL ##################### ");
        iRet = -1;
    }  

    CamacqExtFreeGpio( mfp_to_gpio(MFP_PIN_GPIO39) );  // CAM_SW_EN
    CamacqExtFreeGpio( mfp_to_gpio(MFP_PIN_GPIO52) );  // CAM_SEL
    CamacqExtFreeGpio( mfp_to_gpio(MFP_PIN_GPIO108 )); // CAM_EN1 ---> VCAM_AVDD_2.8V
    CamacqExtFreeGpio( mfp_to_gpio(MFP_PIN_GPIO93) );  // CAM_MCLK
    CamacqExtFreeGpio( mfp_to_gpio(MFP_PIN_GPIO95) );  // SCAM_STBYN
    CamacqExtFreeGpio( mfp_to_gpio(MFP_PIN_GPIO96) );  // SCAM_RSTN
#endif /* CONFIG_PMIC_D1980 */
#endif // #if 0
#elif defined(CAMACQ_MODEL_JETTA)
#if defined(CONFIG_PMIC_D1980)
    {
#if 1 // Dual-Power-Sequence
        CamacqTraceErr("JETTA MT9V114 DUAL-POWER-OFF CONFIG_PMIC_D1980 !!!!!!! ");
        
        CamacqExtRequestGpio( mfp_to_gpio(MFP_PIN_GPIO39), "camacq" );  // CAM_SW_EN
        CamacqExtRequestGpio( mfp_to_gpio(MFP_PIN_GPIO52), "camacq" );  // CAM_SEL
        CamacqExtRequestGpio( mfp_to_gpio(MFP_PIN_GPIO93), "camacq" );  // CAM_MCLK
        CamacqExtRequestGpio( mfp_to_gpio(MFP_PIN_GPIO94), "camacq" );
        CamacqExtRequestGpio( mfp_to_gpio(MFP_PIN_GPIO95), "camacq" );
        CamacqExtRequestGpio( mfp_to_gpio(MFP_PIN_GPIO96), "camacq" );
        CamacqExtRequestGpio( mfp_to_gpio(MFP_PIN_GPIO97), "camacq" );  // SCAM_RSTN
        CamacqExtRequestGpio( mfp_to_gpio(MFP_PIN_GPIO131), "camacq" );  // MCAM_V_CORE
        CamacqExtRequestGpio( mfp_to_gpio(MFP_PIN_GPIO118), "camacq" );  

        /* Reset-down */
        gpio_direction_output( mfp_to_gpio( MFP_PIN_GPIO96 ), 0 );   // SCAM_RSTN

        // MCLK-off
        gpio_direction_output( mfp_to_gpio( MFP_PIN_GPIO93 ), 0 );    // CAM_MCLK
        g_pstCamacqSensorManager->m_stCamBlock.CamPowerSet( 0 );
        CamacqExtDelay(30);

        /* Reset-up */
        gpio_direction_output( mfp_to_gpio( MFP_PIN_GPIO96 ), 1 );   // SCAM_RSTN

        // CORE
        if(reg_cam_sub_core == NULL)
            reg_cam_sub_core = regulator_get(&(g_pstCamacqSensorManager->m_stCamBlock.pdev->dev), REGULATOR_CAM_S_CORE);
        if(reg_cam_sub_core) {
            regulator_disable(reg_cam_sub_core);    
        } else {
            CamacqTraceErr(" ### CAM_S_CORE regulator is NULL ...! ");
            iRet = -1;
        } 
        CamacqExtDelay(10);

        // AVDD
        if(reg_cam_avdd == NULL)
            reg_cam_avdd = regulator_get(&(g_pstCamacqSensorManager->m_stCamBlock.pdev->dev), REGULATOR_CAM_AVDD);
        if(reg_cam_avdd) {
            regulator_disable(reg_cam_avdd);    
        } else {
            CamacqTraceErr(" ### CAM_AVDD regulator is NULL ...! ");
            iRet = -1;
        }   

        gpio_direction_output( mfp_to_gpio( MFP_PIN_GPIO118 ), 0 ); // RF_IF6(GPIO118) for REV0.4
         
        CamacqExtDelay(10);    
        
        // VCAM_IO    
        if(reg_cam_io == NULL)
            reg_cam_io = regulator_get(&(g_pstCamacqSensorManager->m_stCamBlock.pdev->dev), REGULATOR_CAM_IO);
        if(reg_cam_io) {
            regulator_disable(reg_cam_io);    
        } else {
            CamacqTraceErr(" ### CAM_IO regulator is NULL ...! ");
            iRet = -1;
        }   
        CamacqExtDelay(10);

        // VCAM_M_CORE_1.2V 
        gpio_direction_output( mfp_to_gpio( MFP_PIN_GPIO131 ), 0 ); // RF_IF19(GPIO131) -> SWITCH_EN -> VCAM_M_CORE_1.2V 
        CamacqExtDelay(1);

        CamacqTraceErr("WINIG TEST 2222222222222222222222");
        // MAIN-SUB CAM RSTN, STAND BY ALL OFF
        gpio_direction_output( mfp_to_gpio( MFP_PIN_GPIO96 ), 0 );      // SCAM_RSTN
        gpio_direction_output( mfp_to_gpio( MFP_PIN_GPIO95 ), 0 );      // SCAM_STBYN
        gpio_direction_output( mfp_to_gpio( MFP_PIN_GPIO94 ), 0 );      // MCAM_RSTN
        gpio_direction_output( mfp_to_gpio( MFP_PIN_GPIO97 ), 0 );      // MCAM_STBYN

        CamacqExtFreeGpio( mfp_to_gpio(MFP_PIN_GPIO39) );  // CAM_SW_EN
        CamacqExtFreeGpio( mfp_to_gpio(MFP_PIN_GPIO52) );  // CAM_SEL
        CamacqExtFreeGpio( mfp_to_gpio(MFP_PIN_GPIO93) );  // CAM_MCLK
        CamacqExtFreeGpio( mfp_to_gpio(MFP_PIN_GPIO96) );  // SCAM_RSTN
        CamacqExtFreeGpio( mfp_to_gpio(MFP_PIN_GPIO95) );  // SCAM_STBYN
        CamacqExtFreeGpio( mfp_to_gpio(MFP_PIN_GPIO94) );  // MCAM_RSTN
        CamacqExtFreeGpio( mfp_to_gpio(MFP_PIN_GPIO97) );  // MCAM_STBYN
        CamacqExtFreeGpio( mfp_to_gpio(MFP_PIN_GPIO131) );  // SCAM_RSTN
        CamacqExtFreeGpio( mfp_to_gpio(MFP_PIN_GPIO118) );
#else
        CamacqTraceErr("JETTA MT9V114 OFF CONFIG_PMIC_D1980 !!!!!!! ");
        
        CamacqExtRequestGpio( mfp_to_gpio(MFP_PIN_GPIO39), "camacq" );  // CAM_SW_EN
        CamacqExtRequestGpio( mfp_to_gpio(MFP_PIN_GPIO52), "camacq" );  // CAM_SEL
        CamacqExtRequestGpio( mfp_to_gpio(MFP_PIN_GPIO93), "camacq" );  // CAM_MCLK
        CamacqExtRequestGpio( mfp_to_gpio(MFP_PIN_GPIO96), "camacq" );  // SCAM_RSTN

        /* Reset-down */
        gpio_direction_output( mfp_to_gpio( MFP_PIN_GPIO96 ), 0 );   // SCAM_RSTN

        // MCLK-off
        gpio_direction_output( mfp_to_gpio( MFP_PIN_GPIO93 ), 0 );    // CAM_MCLK
        g_pstCamacqSensorManager->m_stCamBlock.CamPowerSet( 0 );
        CamacqExtDelay(30);

        /* Reset-up */
        gpio_direction_output( mfp_to_gpio( MFP_PIN_GPIO96 ), 1 );   // SCAM_RSTN

        // CORE
        if(reg_cam_sub_core == NULL)
            reg_cam_sub_core = regulator_get(&(g_pstCamacqSensorManager->m_stCamBlock.pdev->dev), REGULATOR_CAM_S_CORE);
        if(reg_cam_sub_core) {
            regulator_disable(reg_cam_sub_core);    
        } else {
            CamacqTraceErr(" ### CAM_S_CORE regulator is NULL ...! ");
            iRet = -1;
        } 
        CamacqExtDelay(10);

        // AVDD
        if(reg_cam_avdd == NULL)
            reg_cam_avdd = regulator_get(&(g_pstCamacqSensorManager->m_stCamBlock.pdev->dev), REGULATOR_CAM_AVDD);
        if(reg_cam_avdd) {
            regulator_disable(reg_cam_avdd);    
        } else {
            CamacqTraceErr(" ### CAM_AVDD regulator is NULL ...! ");
            iRet = -1;
        }   
        CamacqExtDelay(10);    
        
        // VCAM_IO    
        if(reg_cam_io == NULL)
            reg_cam_io = regulator_get(&(g_pstCamacqSensorManager->m_stCamBlock.pdev->dev), REGULATOR_CAM_IO);
        if(reg_cam_io) {
            regulator_disable(reg_cam_io);    
        } else {
            CamacqTraceErr(" ### CAM_IO regulator is NULL ...! ");
            iRet = -1;
        }   
        CamacqExtDelay(10);

        CamacqExtFreeGpio( mfp_to_gpio(MFP_PIN_GPIO39) );  // CAM_SW_EN
        CamacqExtFreeGpio( mfp_to_gpio(MFP_PIN_GPIO52) );  // CAM_SEL
        CamacqExtFreeGpio( mfp_to_gpio(MFP_PIN_GPIO93) );  // CAM_MCLK
        CamacqExtFreeGpio( mfp_to_gpio(MFP_PIN_GPIO96) );  // SCAM_RSTN
#endif // Dual-Power-Sequence
    }
#else /* CONFIG_PMIC_D1980  */
#endif /* CONFIG_PMIC_D1980  */
#else
#if 0
    /* reset down */
    gpio_direction_output( mfp_to_gpio( MFP_PIN_GPIO94 ), CAMACQ_SUB_RST_OFF ); // GPIO94 -> SCAM_RSTN
    CamacqExtDelay(30);

    /* Clock */
    gpio_direction_output( mfp_to_gpio( MFP_PIN_GPIO93 ), 0 ); // GPIO93 -> CAM_MCLK
    CamacqExtDelay(10);

    /* Standby */
    // camacq_ext_standby_begin( irestype );
    gpio_direction_output( mfp_to_gpio( MFP_PIN_GPIO97 ), CAMACQ_OFF ); // GPIO97 -> SCAM_STBYN
    CamacqExtDelay(10);
   
    /* Power */
    gpio_direction_output( mfp_to_gpio( MFP_PIN_GPIO108 ), CAMACQ_SUB_EN_OFF ); // GPIO108 -> CAM_EN3 -> VCAM_DVDD1.5V
    gpio_direction_output( mfp_to_gpio( MFP_PIN_GPIO106 ), CAMACQ_SUB_EN_OFF ); // GPIO106 -> CAM_EN -> VCAM_IO_2.8V
    gpio_direction_output( mfp_to_gpio( MFP_PIN_GPIO109 ), CAMACQ_SUB_EN_OFF ); // GPIO109 -> CAM_EN4 -> VCAM_AVDD_2.8V	
    CamacqExtDelay(10);
#endif // #if 0
#endif

    CamacqTraceOUT();
    return iRet;
}
#endif /* CAMACQ_SENSOR_MAX==2  */

S32 CamacqExtBoot( struct stCamacqExtAPIs_t* this )
{
    S32 iRet = 0;
    _stCamacqSensor* pSensor = this->m_pstSensor;
    CamacqTraceIN();

#if defined(__ISX012_SONY__)
    if( !strcmp(pSensor->m_pI2cClient->name, "isx012") )
    {
        // change slave address for mipi
        CamacqTraceDbg("Change to 1A to 0x3C");
        pSensor->m_pI2cClient->addr = 0x3C;

        // Wait for Mode Transition(OM)
        iRet = WaitForModeTransition_Sony( pSensor->m_pI2cClient, 0 );
        if( iRet < 0 )
        {
            CamacqTraceErr(" WaitForModeTransition_Sony error iRet=%d", iRet);
            return iRet;
        }

        // Write set to little Endian 
        {
            CamacqTraceDbg("========= e");
            CamacqTraceDbg("Write set to little Endian ");
                        
            // {0x5008,0x00,0x01},                  //ENDIAN_SEL : 0:Little Endian 
            U8 ucWriteRegs[3] = {0, };                
            ucWriteRegs[0] = 0x50;
            ucWriteRegs[1] = 0x08;
            ucWriteRegs[2] = 0x00;
                
            CamacqExtWriteI2c( pSensor->m_pI2cClient, ucWriteRegs, 3);

            CamacqTraceDbg("========= x");
        }        

        // Calibration
        iRet = Callibration_Sony( pSensor->m_pI2cClient, pSensor->m_uiResType );
        if( iRet < 0 )
        {
            CamacqTraceErr(" Callibration_Sony error iRet=%d", iRet);
            return iRet;
        }
        
    }

#endif    

    if( this->m_pstSensorRegs->pvInitRegs != NULL )
    {
        iRet = CamacqExtWriteI2cLists( pSensor->m_pI2cClient, this->m_pstSensorRegs->pvInitRegs, pSensor->m_uiResType );
        if( iRet < 0 )
        {
            CamacqTraceErr(": iRet=%d", iRet);
            iRet = -1;
            return iRet;
        }
    }
    else
    {
        iRet = -1;
        CamacqTraceErr( "pvInitRegs is NULL" );
    }
   
    CamacqTraceOUT();
    return iRet;
}

S32 CamacqExtHflip( struct stCamacqExtAPIs_t* this, _eCamacqExtMode eMode, S32 *piVal )
{
    S32 iRet = 0;
    CamacqTraceIN( ":piVal[%d]", *piVal );
    
    iRet = CamacqExtSetFlip( this, CAMACQ_EXT_HFLIP, *piVal );
    if(iRet < 0)
    {
        CamacqTraceErr( ":hflip failed" );
        iRet = -EINVAL;
    }

    CamacqTraceOUT();
    return iRet;
}

S32 CamacqExtVflip( struct stCamacqExtAPIs_t* this, _eCamacqExtMode eMode, S32 *piVal )
{
    S32 iRet = 0;
    CamacqTraceIN( ":piVal[%d]", *piVal );
    
    iRet = CamacqExtSetFlip( this, CAMACQ_EXT_VFLIP, *piVal );
    if(iRet < 0)
    {
        CamacqTraceErr(":vflip failed");
        iRet = -EINVAL;
    }

    CamacqTraceOUT();
    return iRet;
}

S32 CamacqExtSetFlip( struct stCamacqExtAPIs_t* this, U8 ucFlip, U8 ucVal )
{
    static U8 ucCurFlip = CAMACQ_EXT_NONE;
    S32 iRet = 0;
    const void* pvRegs = NULL;

    CamacqTraceIN();
    CamacqTraceDbg( " :ucFlip[%d] ucVal[%d] ", ucFlip, ucVal );

    if( ucFlip == CAMACQ_EXT_HFLIP )
    {
        ucCurFlip = (ucCurFlip & ~CAMACQ_EXT_HFLIP) + ucVal;
    }
    else
    {
        ucCurFlip = (ucCurFlip & ~CAMACQ_EXT_VFLIP) + (ucVal << 1);
    }

    switch(ucCurFlip)
    {
        case CAMACQ_EXT_NONE:
        {
            CamacqTraceDbg(" CAMACQ_EXT_NONE ");
            pvRegs = this->m_pstSensorRegs->pvFlipNoneRegs;
        }
        break;
        case CAMACQ_EXT_HFLIP:
        {
            CamacqTraceDbg(" CAMACQ_EXT_HFLIP ");
            pvRegs = this->m_pstSensorRegs->pvFlipMirrorRegs;
        }
        break;
        case CAMACQ_EXT_VFLIP:
        {
            CamacqTraceDbg(" CAMACQ_EXT_VFLIP ");
            pvRegs = this->m_pstSensorRegs->pvFlipWaterRegs;
        }
        break;
        case (CAMACQ_EXT_HFLIP + CAMACQ_EXT_VFLIP):
        {
            CamacqTraceDbg(" CAMACQ_EXT_HFLIP + CAMACQ_EXT_VFLIP ");
            pvRegs = this->m_pstSensorRegs->pvFlipWaterMirrorRegs;
        }
        break;
        default:
        {
            CamacqTraceErr( ":invalid flip mode" );
            pvRegs = NULL;
            iRet = -EINVAL; 
        }
        break;
    }

    if( pvRegs )
    {
        CamacqExtWriteI2cLists( this->m_pstSensor->m_pI2cClient, pvRegs, this->m_pstSensor->m_uiResType );
    }

    CamacqTraceOUT();
    return iRet;
}

S32 CamacqExtExposure( struct stCamacqExtAPIs_t* this, _eCamacqExtMode eMode, S32 *piVal )
{
    S32 iRet = 0;
    const void* pvRegs = NULL;

    CamacqTraceIN();
    CamacqTraceDbg( ":piVal[%d]", *piVal );
    
    switch( (enum v4l2_ctrl_metering_exposure)*piVal )
    {
        case V4L2_CTRL_MET_EXP_MATRIX:
        {
            CamacqTraceDbg(" V4L2_CTRL_MET_EXP_MATRIX ");
            pvRegs = this->m_pstSensorRegs->pvMeterMatrixRegs;
        }
        break;
        case V4L2_CTRL_MET_EXP_CENTERWEIGHTED:
        {
            CamacqTraceDbg(" V4L2_CTRL_MET_EXP_CENTERWEIGHTED ");
            pvRegs = this->m_pstSensorRegs->pvMeterCWRegs;
        }
        break;
        case V4L2_CTRL_MET_EXP_SPOT:
        {
            CamacqTraceDbg(" V4L2_CTRL_MET_EXP_SPOT ");
            pvRegs = this->m_pstSensorRegs->pvMeterSpotRegs;
        }
        break;
        default:
        {
            CamacqTraceErr( ":invalid psival[%d]", *piVal );
            pvRegs =NULL;
            iRet = -EINVAL;
        }
        break;
    }

    if( pvRegs )
    {
        CamacqExtWriteI2cLists( this->m_pstSensor->m_pI2cClient, pvRegs, this->m_pstSensor->m_uiResType );
    }

    CamacqTraceOUT();
    return iRet;
}

S32 CamacqExtSensitivity( struct stCamacqExtAPIs_t* this, _eCamacqExtMode eMode, S32 *piVal )
{
    S32 iRet = 0;
    const void* pvRegs = NULL;

    CamacqTraceIN();
    CamacqTraceDbg( ":piVal[%d]", *piVal );

#if defined(_S5K4ECGX_EVT1_MIPI_)
    /* READ 0x04E6 */
    int iRegTcDbgAutoAlgEnBits = 0;
    U8	rgucWriteRegs_1[4] = {0x00,0x2C,0x70,0x00};
    U8	rgucWriteRegs_2[4] = {0x00,0x2E,0x04,0xE6};
    U8	rgucWriteRegs_3[4] = {0x00,0x28,0x70,0x00};
    U8	rgucWriteRegs_4[4] = {0x00,0x2A,0x04,0xE6};
    U8  rgucWriteRegs[4] = {0x00, };
    U8  rgucReadData[2] = {0, };
    U16 usReadAddr = 0x0F12;

    CamacqExtWriteI2c( this->m_pstSensor->m_pI2cClient, rgucWriteRegs_1, 4 );
    CamacqExtWriteI2c( this->m_pstSensor->m_pI2cClient, rgucWriteRegs_2, 4 );
    CamacqExtReadI2c( this->m_pstSensor->m_pI2cClient, usReadAddr, 2, rgucReadData, 2 );

    iRegTcDbgAutoAlgEnBits = ( rgucReadData[0] << 8 );
    iRegTcDbgAutoAlgEnBits |= rgucReadData[1];

    CamacqTraceDbg( "iRegTcDbgAutoAlgEnBits : 0x%x, rgucReadData[0] : 0x%x, rgucReadData[1] : 0x%x",
            iRegTcDbgAutoAlgEnBits, rgucReadData[0], rgucReadData[1] );

    if( (enum v4l2_ctrl_sensitivity)*piVal == V4L2_CTRL_ISO_AUTO )
    {
        iRegTcDbgAutoAlgEnBits = iRegTcDbgAutoAlgEnBits | 0x20;
    }
    else
    {
        iRegTcDbgAutoAlgEnBits = iRegTcDbgAutoAlgEnBits & 0xFFDF;
    }

    CamacqTraceDbg("iRegTcDbgAutoAlgEnBits : 0x%X", iRegTcDbgAutoAlgEnBits);

    rgucWriteRegs[0] = 0x0F; rgucWriteRegs[1] = 0x12;
    rgucWriteRegs[2] = (U8)(iRegTcDbgAutoAlgEnBits >> 8);
    rgucWriteRegs[3] = (U8)iRegTcDbgAutoAlgEnBits;

    CamacqTraceDbg("rgucWriteRegs: 0x%02X%02X%02X%02X" , rgucWriteRegs[0] , rgucWriteRegs[1], rgucWriteRegs[2] , rgucWriteRegs[3]);
    CamacqExtWriteI2c( this->m_pstSensor->m_pI2cClient, rgucWriteRegs_3, 4);
    CamacqExtWriteI2c( this->m_pstSensor->m_pI2cClient, rgucWriteRegs_4, 4);
    CamacqExtWriteI2c( this->m_pstSensor->m_pI2cClient, rgucWriteRegs, 4 );

#endif /* _S5K4ECGX_EVT1_MIPI_ */

    
    switch( (enum v4l2_ctrl_sensitivity)*piVal )
    {
        case V4L2_CTRL_ISO_AUTO:
        {
            CamacqTraceDbg(" V4L2_CTRL_ISO_AUTO ");
            pvRegs = this->m_pstSensorRegs->pvISOAutoRegs;
        }
        break;
        case V4L2_CTRL_ISO_50:
        {
            CamacqTraceDbg(" V4L2_CTRL_ISO_50 ");
            pvRegs = this->m_pstSensorRegs->pvISO50Regs; 
        }
        break;
        case V4L2_CTRL_ISO_100:
        {
            CamacqTraceDbg(" V4L2_CTRL_ISO_100 ");
            pvRegs = this->m_pstSensorRegs->pvISO100Regs; 
        }
        break;
        case V4L2_CTRL_ISO_200:
        {
            CamacqTraceDbg(" V4L2_CTRL_ISO_200 ");
            pvRegs = this->m_pstSensorRegs->pvISO200Regs;
        }
        break;
        case V4L2_CTRL_ISO_400:
        {
            CamacqTraceDbg(" V4L2_CTRL_ISO_400 ");
            pvRegs = this->m_pstSensorRegs->pvISO400Regs;
        }
        break;
        case V4L2_CTRL_ISO_800:
        {
            CamacqTraceDbg(" V4L2_CTRL_ISO_800 ");
            pvRegs = this->m_pstSensorRegs->pvISO800Regs;

        }
        break;
        case V4L2_CTRL_ISO_1600:
        {
            CamacqTraceDbg(" V4L2_CTRL_ISO_1600 ");
            pvRegs = this->m_pstSensorRegs->pvISO1600Regs;
        }
        break;
        case V4L2_CTRL_ISO_3200:
        {
            CamacqTraceDbg(" V4L2_CTRL_ISO_3200 ");
            pvRegs = this->m_pstSensorRegs->pvISO3200Regs;
        }
        break;
        default:
        {
            CamacqTraceErr( ":invalid piVal[%d]", *piVal );
            pvRegs = NULL;
            iRet = -EINVAL;
        }
        break;
    }

    if( pvRegs )
    {
        CamacqExtWriteI2cLists( this->m_pstSensor->m_pI2cClient, pvRegs, this->m_pstSensor->m_uiResType );
    }
     
    CamacqTraceOUT();
    return iRet;
}

S32 CamacqExtScene( struct stCamacqExtAPIs_t* this, _eCamacqExtMode eMode, S32 *piVal )
{
    S32 iRet = 0;
    const void* pvRegs = NULL;
    CamacqTraceIN();
    CamacqTraceDbg( " :piVal[%d] ", *piVal );

#if defined(_S5K4ECGX_EVT1_MIPI_)
    /* READ 0x04E6 */
    int iRegTcDbgAutoAlgEnBits = 0;
    U8	rgucWriteRegs_1[4] = {0x00,0x2C,0x70,0x00};
    U8	rgucWriteRegs_2[4] = {0x00,0x2E,0x04,0xE6};
    U8	rgucWriteRegs_3[4] = {0x00,0x28,0x70,0x00};
    U8	rgucWriteRegs_4[4] = {0x00,0x2A,0x04,0xE6};
    U8  rgucWriteRegs[4] = {0x00, };
    U8  rgucReadData[2] = {0, };
    U16 usReadAddr = 0x0F12;
    bool bNeedChange = true;

    CamacqExtWriteI2c( this->m_pstSensor->m_pI2cClient, rgucWriteRegs_1, 4 );
    CamacqExtWriteI2c( this->m_pstSensor->m_pI2cClient, rgucWriteRegs_2, 4 );
    CamacqExtReadI2c( this->m_pstSensor->m_pI2cClient, usReadAddr, 2, rgucReadData, 2 );

    iRegTcDbgAutoAlgEnBits = ( rgucReadData[0] << 8 );
    iRegTcDbgAutoAlgEnBits |= rgucReadData[1];

    CamacqTraceDbg( "iRegTcDbgAutoAlgEnBits : 0x%x, rgucReadData[0] : 0x%x, rgucReadData[1] : 0x%x",
            iRegTcDbgAutoAlgEnBits, rgucReadData[0], rgucReadData[1] );

    if( ((enum v4l2_ctrl_scene)*piVal ==  V4L2_CTRL_SCENE_PARTY) || ((enum v4l2_ctrl_scene)*piVal ==  V4L2_CTRL_SCENE_INDOOR) )
    {
        iRegTcDbgAutoAlgEnBits = iRegTcDbgAutoAlgEnBits & 0xFFDF;
    }
    else if( ((enum v4l2_ctrl_scene)*piVal ==  V4L2_CTRL_SCENE_BEACH) || ((enum v4l2_ctrl_scene)*piVal ==  V4L2_CTRL_SCENE_SNOW) )
    {
        iRegTcDbgAutoAlgEnBits = iRegTcDbgAutoAlgEnBits & 0xFFDF;
    }
    else if( ((enum v4l2_ctrl_scene)*piVal ==  V4L2_CTRL_SCENE_SUNSET) )
    {
        iRegTcDbgAutoAlgEnBits = iRegTcDbgAutoAlgEnBits & 0xFFF7;
    }
    else if( ((enum v4l2_ctrl_scene)*piVal ==  V4L2_CTRL_SCENE_DUSKDAWN) )
    {
        iRegTcDbgAutoAlgEnBits = iRegTcDbgAutoAlgEnBits & 0xFFF7;
    }
    else if( ((enum v4l2_ctrl_scene)*piVal ==  V4L2_CTRL_SCENE_CANDLELIGHT) )
    {
        iRegTcDbgAutoAlgEnBits = iRegTcDbgAutoAlgEnBits & 0xFFF7;
    }
    else
    {
        bNeedChange = false;
        CamacqTraceDbg("Dont need change");
    }

    if( bNeedChange )
    {   
        CamacqTraceDbg("iRegTcDbgAutoAlgEnBits : 0x%X", iRegTcDbgAutoAlgEnBits);
        rgucWriteRegs[0] = 0x0F; rgucWriteRegs[1] = 0x12;
        rgucWriteRegs[2] = (U8)(iRegTcDbgAutoAlgEnBits >> 8);
        rgucWriteRegs[3] = (U8)iRegTcDbgAutoAlgEnBits;

        CamacqTraceDbg("rgucWriteRegs: 0x%02X%02X%02X%02X" , rgucWriteRegs[0] , rgucWriteRegs[1], rgucWriteRegs[2] , rgucWriteRegs[3]);
        CamacqExtWriteI2c( this->m_pstSensor->m_pI2cClient, rgucWriteRegs_3, 4);
        CamacqExtWriteI2c( this->m_pstSensor->m_pI2cClient, rgucWriteRegs_4, 4);
        CamacqExtWriteI2c( this->m_pstSensor->m_pI2cClient, rgucWriteRegs, 4 );
    }    
#endif /* _S5K4ECGX_EVT1_MIPI_ */


    switch( (enum v4l2_ctrl_scene)*piVal )
    {
        case V4L2_CTRL_SCENE_AUTO:
        {   
            CamacqTraceDbg(" V4L2_CTRL_SCENE_AUTO ");
            pvRegs = this->m_pstSensorRegs->pvSceneAutoRegs;
        }
        break;
        case V4L2_CTRL_SCENE_NIGHT:
        {
            CamacqTraceDbg(" V4L2_CTRL_SCENE_NIGHT ");
            pvRegs = this->m_pstSensorRegs->pvSceneNightRegs;
        }
        break;
        case V4L2_CTRL_SCENE_LANDSCAPE:
        {
            CamacqTraceDbg(" V4L2_CTRL_SCENE_LANDSCAPE ");
            pvRegs = this->m_pstSensorRegs->pvSceneLandScapeRegs;
        }
        break;
        case V4L2_CTRL_SCENE_SUNSET:
        {
            CamacqTraceDbg(" V4L2_CTRL_SCENE_SUNSET ");
            pvRegs = this->m_pstSensorRegs->pvSceneSunSetRegs;
        }
        break;
        case V4L2_CTRL_SCENE_PORTRAIT:
        {
            CamacqTraceDbg(" V4L2_CTRL_SCENE_PORTRAIT ");
            pvRegs = this->m_pstSensorRegs->pvScenePortraitRegs;
        }
        break;
        case V4L2_CTRL_SCENE_SUNRISE:
        {
            CamacqTraceDbg(" V4L2_CTRL_SCENE_SUNRISE ");
            pvRegs = this->m_pstSensorRegs->pvSceneSunRiseRegs;
        }
        break;
        case V4L2_CTRL_SCENE_INDOOR:
        {
            CamacqTraceDbg(" V4L2_CTRL_SCENE_INDOOR ");
            pvRegs = this->m_pstSensorRegs->pvSceneIndoorRegs;
        }
        break;
        case V4L2_CTRL_SCENE_PARTY:
        {
            CamacqTraceDbg(" V4L2_CTRL_SCENE_PARTY ");
            pvRegs = this->m_pstSensorRegs->pvScenePartyRegs;
        }
        break;
        case V4L2_CTRL_SCENE_SPORTS:
        {
            CamacqTraceDbg(" V4L2_CTRL_SCENE_SPORTS ");
            pvRegs = this->m_pstSensorRegs->pvSceneSportsRegs;
        }
        break;
        case V4L2_CTRL_SCENE_BEACH:
        {
            CamacqTraceDbg(" V4L2_CTRL_SCENE_BEACH ");
            pvRegs = this->m_pstSensorRegs->pvSceneBeachRegs;
        }
        break;
        case V4L2_CTRL_SCENE_SNOW:
        {
            CamacqTraceDbg(" V4L2_CTRL_SCENE_SNOW ");
            pvRegs = this->m_pstSensorRegs->pvSceneSnowRegs;
        }
        break;
        case V4L2_CTRL_SCENE_FALLCOLOR:
        {
            CamacqTraceDbg(" V4L2_CTRL_SCENE_FALLCOLOR ");
            pvRegs = this->m_pstSensorRegs->pvSceneFallColorRegs;
        }
        break;
        case V4L2_CTRL_SCENE_FIREWORKS:
        {
            CamacqTraceDbg(" V4L2_CTRL_SCENE_FIREWORKS ");
            pvRegs = this->m_pstSensorRegs->pvSceneFireWorksRegs;
        }
        break;
        case V4L2_CTRL_SCENE_CANDLELIGHT:
        {
            CamacqTraceDbg(" V4L2_CTRL_SCENE_CANDLELIGHT ");
            pvRegs = this->m_pstSensorRegs->pvSceneCandleLightRegs;
        }
        break;
        case V4L2_CTRL_SCENE_AGAINSTLIGHT:
        {
            CamacqTraceDbg(" V4L2_CTRL_SCENE_AGAINSTLIGHT ");
            pvRegs = this->m_pstSensorRegs->pvSceneAgainstLightRegs;
        }
        break;
        case V4L2_CTRL_SCENE_TEXT:
        {
            CamacqTraceDbg(" V4L2_CTRL_SCENE_TEXT ");
            pvRegs = this->m_pstSensorRegs->pvSceneTextRegs;
        }
        break;
        case V4L2_CTRL_SCENE_DUSKDAWN:
        {
            CamacqTraceDbg(" V4L2_CTRL_SCENE_DUSKDAWN ");
            pvRegs = this->m_pstSensorRegs->pvSceneSunRiseRegs; // temp
        }
        break;
        case V4L2_CTRL_SCENE_AQUA:
        {
            CamacqTraceDbg(" V4L2_CTRL_SCENE_AQUA ");
            pvRegs = this->m_pstSensorRegs->pvSceneAquaRegs;
        }
        break;
        default:
        {
            CamacqTraceErr(  ":invalid psival[%d] ", *piVal );
            pvRegs = NULL;
            iRet = -EINVAL;
        }
        break;
    }

    if( pvRegs )    
    {
        CamacqExtWriteI2cLists( this->m_pstSensor->m_pI2cClient, pvRegs, this->m_pstSensor->m_uiResType );

        // save 
        this->m_pstSensor->m_pstCamacqSettings->stV4l2CurScene = (enum v4l2_ctrl_scene)*piVal;
    }

    CamacqTraceOUT();
    return iRet;
}

S32 CamacqExtBrightness( struct stCamacqExtAPIs_t* this, _eCamacqExtMode eMode, S32 *piVal )
{
    S32 iRet = 0;
    const void* pvRegs = NULL;
    CamacqTraceIN();
    
    CamacqTraceDbg( ":piVal[%d] ", *piVal );
    switch(*piVal)
    {
        case CAMACQ_EXT_BR_LVL_0:
        {
            CamacqTraceDbg(" CAMACQ_EXT_BR_LVL_0 ");
            pvRegs = this->m_pstSensorRegs->pvBrightness_0_Regs;
        }
        break;
        case CAMACQ_EXT_BR_LVL_1:
        {
            CamacqTraceDbg(" CAMACQ_EXT_BR_LVL_1 ");
            pvRegs = this->m_pstSensorRegs->pvBrightness_1_Regs;
        }
        break;
        case CAMACQ_EXT_BR_LVL_2:
        {
            CamacqTraceDbg(" CAMACQ_EXT_BR_LVL_2 ");
            pvRegs = this->m_pstSensorRegs->pvBrightness_2_Regs;
        }
        break;
        case CAMACQ_EXT_BR_LVL_3:
        {
            CamacqTraceDbg(" CAMACQ_EXT_BR_LVL_3 ");
            pvRegs = this->m_pstSensorRegs->pvBrightness_3_Regs;
        }
        break;
        case CAMACQ_EXT_BR_LVL_4:
        {
            CamacqTraceDbg(" CAMACQ_EXT_BR_LVL_4 ");
            pvRegs = this->m_pstSensorRegs->pvBrightness_4_Regs;
        }
        break;
        case CAMACQ_EXT_BR_LVL_5:
        {
            CamacqTraceDbg(" CAMACQ_EXT_BR_LVL_5 ");
            pvRegs = this->m_pstSensorRegs->pvBrightness_5_Regs;
        }
        break;
        case CAMACQ_EXT_BR_LVL_6:
        {
            CamacqTraceDbg(" CAMACQ_EXT_BR_LVL_6 ");
            pvRegs = this->m_pstSensorRegs->pvBrightness_6_Regs;
        }
        break;
        case CAMACQ_EXT_BR_LVL_7:
        {
            CamacqTraceDbg(" CAMACQ_EXT_BR_LVL_7 ");
            pvRegs = this->m_pstSensorRegs->pvBrightness_7_Regs;
        }
        break;
        case CAMACQ_EXT_BR_LVL_8:
        {
            CamacqTraceDbg(" CAMACQ_EXT_BR_LVL_8 ");
            pvRegs = this->m_pstSensorRegs->pvBrightness_8_Regs;
        }
        break;
        default:
        {
            CamacqTraceDbg( " :invalid psival[%d] ", *piVal );
            pvRegs = NULL;
            iRet = -EINVAL;
        }
    }

    if( pvRegs )
    {
        CamacqExtWriteI2cLists( this->m_pstSensor->m_pI2cClient, pvRegs, this->m_pstSensor->m_uiResType );
    }

    CamacqTraceOUT();
    return iRet;
}

S32 CamacqExtExposureCompensation( struct stCamacqExtAPIs_t* this, _eCamacqExtMode eMode, S32 *piVal )
{
    S32 iRet = 0;
    const void* pvRegs = NULL;
    CamacqTraceIN();
    
    CamacqTraceDbg( ":piVal[%d] ", *piVal );
    switch(*piVal)
    {
        case CAMACQ_EXT_EXP_COMP_LVL_0:
        {
            CamacqTraceDbg(" CAMACQ_EXT_EXP_COMP_LVL_0 ");
            pvRegs = this->m_pstSensorRegs->pvExpCompensation_0_Regs;
        }
        break;
        case CAMACQ_EXT_EXP_COMP_LVL_1:
        {
            CamacqTraceDbg(" CAMACQ_EXT_EXP_COMP_LVL_1 ");
            pvRegs = this->m_pstSensorRegs->pvExpCompensation_1_Regs;
        }
        break;
        case CAMACQ_EXT_EXP_COMP_LVL_2:
        {
            CamacqTraceDbg(" CAMACQ_EXT_EXP_COMP_LVL_2 ");
            pvRegs = this->m_pstSensorRegs->pvExpCompensation_2_Regs;
        }
        break;
        case CAMACQ_EXT_EXP_COMP_LVL_3:
        {
            CamacqTraceDbg(" CAMACQ_EXT_EXP_COMP_LVL_3 ");
            pvRegs = this->m_pstSensorRegs->pvExpCompensation_3_Regs;
        }
        break;
        case CAMACQ_EXT_EXP_COMP_LVL_4:
        {
            CamacqTraceDbg(" CAMACQ_EXT_EXP_COMP_LVL_4 ");
            pvRegs = this->m_pstSensorRegs->pvExpCompensation_4_Regs;
        }
        break;
        case CAMACQ_EXT_EXP_COMP_LVL_5:
        {
            CamacqTraceDbg(" CAMACQ_EXT_EXP_COMP_LVL_5 ");
            pvRegs = this->m_pstSensorRegs->pvExpCompensation_5_Regs;
        }
        break;
        case CAMACQ_EXT_EXP_COMP_LVL_6:
        {
            CamacqTraceDbg(" CAMACQ_EXT_EXP_COMP_LVL_6 ");
            pvRegs = this->m_pstSensorRegs->pvExpCompensation_6_Regs;
        }
        break;
        case CAMACQ_EXT_EXP_COMP_LVL_7:
        {
            CamacqTraceDbg(" CAMACQ_EXT_EXP_COMP_LVL_7 ");
            pvRegs = this->m_pstSensorRegs->pvExpCompensation_7_Regs;
        }
        break;
        case CAMACQ_EXT_EXP_COMP_LVL_8:
        {
            CamacqTraceDbg(" CAMACQ_EXT_EXP_COMP_LVL_8 ");
            pvRegs = this->m_pstSensorRegs->pvExpCompensation_8_Regs;
        }
        break;
        default:
        {
            CamacqTraceDbg( " :invalid psival[%d] ", *piVal );
            pvRegs = NULL;
            iRet = -EINVAL;
        }
    }

    if( pvRegs )
    {
        CamacqExtWriteI2cLists( this->m_pstSensor->m_pI2cClient, pvRegs, this->m_pstSensor->m_uiResType );
    }

    CamacqTraceOUT();
    return iRet;
}

S32 CamacqExtZoom( struct stCamacqExtAPIs_t* this, _eCamacqExtMode eMode, S32 *piVal )
{
    S32 iRet = 0;
    const void* pvRegs = NULL;
    CamacqTraceIN();
    
    CamacqTraceDbg( ":piVal[%d] ", *piVal );
    switch(*piVal)
    {
        case CAMACQ_EXT_128X_ZOOM_LVL_0:
        {
            CamacqTraceDbg(" CAMACQ_EXT_ZOOM_LVL_0 ");
            pvRegs = this->m_pstSensorRegs->pvZoom_128x_0_Regs;
        }
        break;
        case CAMACQ_EXT_128X_ZOOM_LVL_1:
        {
            CamacqTraceDbg(" CAMACQ_EXT_ZOOM_LVL_1 ");
            pvRegs = this->m_pstSensorRegs->pvZoom_128x_1_Regs;
        }
        break;
        case CAMACQ_EXT_128X_ZOOM_LVL_2:
        {
            CamacqTraceDbg(" CAMACQ_EXT_ZOOM_LVL_2 ");
            pvRegs = this->m_pstSensorRegs->pvZoom_128x_2_Regs;
        }
        break;
        case CAMACQ_EXT_128X_ZOOM_LVL_3:
        {
            CamacqTraceDbg(" CAMACQ_EXT_ZOOM_LVL_3 ");
            pvRegs = this->m_pstSensorRegs->pvZoom_128x_3_Regs;
        }
        break;
        case CAMACQ_EXT_128X_ZOOM_LVL_4:
        {
            CamacqTraceDbg(" CAMACQ_EXT_ZOOM_LVL_4 ");
            pvRegs = this->m_pstSensorRegs->pvZoom_128x_4_Regs;
        }
        break;
        case CAMACQ_EXT_128X_ZOOM_LVL_5:
        {
            CamacqTraceDbg(" CAMACQ_EXT_ZOOM_LVL_5 ");
            pvRegs = this->m_pstSensorRegs->pvZoom_128x_5_Regs;
        }
        break;
        case CAMACQ_EXT_128X_ZOOM_LVL_6:
        {
            CamacqTraceDbg(" CAMACQ_EXT_ZOOM_LVL_6 ");
            pvRegs = this->m_pstSensorRegs->pvZoom_128x_6_Regs;
        }
        break;
        case CAMACQ_EXT_128X_ZOOM_LVL_7:
        {
            CamacqTraceDbg(" CAMACQ_EXT_ZOOM_LVL_7 ");
            pvRegs = this->m_pstSensorRegs->pvZoom_128x_7_Regs;
        }
        break;
        case CAMACQ_EXT_128X_ZOOM_LVL_8:
        {
            CamacqTraceDbg(" CAMACQ_EXT_ZOOM_LVL_8 ");
            pvRegs = this->m_pstSensorRegs->pvZoom_128x_8_Regs;
        }
        break;


        case CAMACQ_EXT_160X_ZOOM_LVL_0:
        {
            CamacqTraceDbg(" CAMACQ_EXT_ZOOM_LVL_0 ");
            pvRegs = this->m_pstSensorRegs->pvZoom_160x_0_Regs;
        }
        break;
        case CAMACQ_EXT_160X_ZOOM_LVL_1:
        {
            CamacqTraceDbg(" CAMACQ_EXT_ZOOM_LVL_1 ");
            pvRegs = this->m_pstSensorRegs->pvZoom_160x_1_Regs;
        }
        break;
        case CAMACQ_EXT_160X_ZOOM_LVL_2:
        {
            CamacqTraceDbg(" CAMACQ_EXT_ZOOM_LVL_2 ");
            pvRegs = this->m_pstSensorRegs->pvZoom_160x_2_Regs;
        }
        break;
        case CAMACQ_EXT_160X_ZOOM_LVL_3:
        {
            CamacqTraceDbg(" CAMACQ_EXT_ZOOM_LVL_3 ");
            pvRegs = this->m_pstSensorRegs->pvZoom_160x_3_Regs;
        }
        break;
        case CAMACQ_EXT_160X_ZOOM_LVL_4:
        {
            CamacqTraceDbg(" CAMACQ_EXT_ZOOM_LVL_4 ");
            pvRegs = this->m_pstSensorRegs->pvZoom_160x_4_Regs;
        }
        break;
        case CAMACQ_EXT_160X_ZOOM_LVL_5:
        {
            CamacqTraceDbg(" CAMACQ_EXT_ZOOM_LVL_5 ");
            pvRegs = this->m_pstSensorRegs->pvZoom_160x_5_Regs;
        }
        break;
        case CAMACQ_EXT_160X_ZOOM_LVL_6:
        {
            CamacqTraceDbg(" CAMACQ_EXT_ZOOM_LVL_6 ");
            pvRegs = this->m_pstSensorRegs->pvZoom_160x_6_Regs;
        }
        break;
        case CAMACQ_EXT_160X_ZOOM_LVL_7:
        {
            CamacqTraceDbg(" CAMACQ_EXT_ZOOM_LVL_7 ");
            pvRegs = this->m_pstSensorRegs->pvZoom_160x_7_Regs;
        }
        break;
        case CAMACQ_EXT_160X_ZOOM_LVL_8:
        {
            CamacqTraceDbg(" CAMACQ_EXT_ZOOM_LVL_8 ");
            pvRegs = this->m_pstSensorRegs->pvZoom_160x_8_Regs;
        }
        break;        



        case CAMACQ_EXT_200X_ZOOM_LVL_0:
        {
            CamacqTraceDbg(" CAMACQ_EXT_ZOOM_LVL_0 ");
            pvRegs = this->m_pstSensorRegs->pvZoom_200x_0_Regs;
        }
        break;
        case CAMACQ_EXT_200X_ZOOM_LVL_1:
        {
            CamacqTraceDbg(" CAMACQ_EXT_ZOOM_LVL_1 ");
            pvRegs = this->m_pstSensorRegs->pvZoom_200x_1_Regs;
        }
        break;
        case CAMACQ_EXT_200X_ZOOM_LVL_2:
        {
            CamacqTraceDbg(" CAMACQ_EXT_ZOOM_LVL_2 ");
            pvRegs = this->m_pstSensorRegs->pvZoom_200x_2_Regs;
        }
        break;
        case CAMACQ_EXT_200X_ZOOM_LVL_3:
        {
            CamacqTraceDbg(" CAMACQ_EXT_ZOOM_LVL_3 ");
            pvRegs = this->m_pstSensorRegs->pvZoom_200x_3_Regs;
        }
        break;
        case CAMACQ_EXT_200X_ZOOM_LVL_4:
        {
            CamacqTraceDbg(" CAMACQ_EXT_ZOOM_LVL_4 ");
            pvRegs = this->m_pstSensorRegs->pvZoom_200x_4_Regs;
        }
        break;
        case CAMACQ_EXT_200X_ZOOM_LVL_5:
        {
            CamacqTraceDbg(" CAMACQ_EXT_ZOOM_LVL_5 ");
            pvRegs = this->m_pstSensorRegs->pvZoom_200x_5_Regs;
        }
        break;
        case CAMACQ_EXT_200X_ZOOM_LVL_6:
        {
            CamacqTraceDbg(" CAMACQ_EXT_ZOOM_LVL_6 ");
            pvRegs = this->m_pstSensorRegs->pvZoom_200x_6_Regs;
        }
        break;
        case CAMACQ_EXT_200X_ZOOM_LVL_7:
        {
            CamacqTraceDbg(" CAMACQ_EXT_ZOOM_LVL_7 ");
            pvRegs = this->m_pstSensorRegs->pvZoom_200x_7_Regs;
        }
        break;
        case CAMACQ_EXT_200X_ZOOM_LVL_8:
        {
            CamacqTraceDbg(" CAMACQ_EXT_ZOOM_LVL_8 ");
            pvRegs = this->m_pstSensorRegs->pvZoom_200x_8_Regs;
        }
        break;        
        
        default:
        {
            CamacqTraceDbg( " :invalid psival[%d] ", *piVal );
            pvRegs = NULL;
            iRet = -EINVAL;
        }
    }

    if( pvRegs )
    {
        CamacqExtWriteI2cLists( this->m_pstSensor->m_pI2cClient, pvRegs, this->m_pstSensor->m_uiResType );
    }

    CamacqTraceOUT();
    return iRet;
}



S32 CamacqExtSaturation( struct stCamacqExtAPIs_t* this, _eCamacqExtMode eMode, S32 *piVal )
{
    S32 iRet = 0;
    const void* pvRegs = NULL;
    CamacqTraceIN();
    
    CamacqTraceDbg( ":piVal[%d] ", *piVal );
    switch(*piVal)
    {
        case CAMACQ_EXT_ADJUST_SATURATION_LVL_M2:
        {
            CamacqTraceDbg(" CAMACQ_EXT_ADJUST_SATURATION_LVL_M2 ");
            pvRegs = this->m_pstSensorRegs->pvAdjustSaturationM2;
        }
        break;
        case CAMACQ_EXT_ADJUST_SATURATION_LVL_M1:
        {
            CamacqTraceDbg(" CAMACQ_EXT_ADJUST_SATURATION_LVL_M1 ");
            pvRegs = this->m_pstSensorRegs->pvAdjustSaturationM1;
        }
        break;
        case CAMACQ_EXT_ADJUST_SATURATION_LVL_DEFAULT:
        {
            CamacqTraceDbg(" CAMACQ_EXT_ADJUST_SATURATION_LVL_DEFAULT ");
            pvRegs = this->m_pstSensorRegs->pvAdjustSaturationDefault;
        }
        break;
        case CAMACQ_EXT_ADJUST_SATURATION_LVL_P1:
        {
            CamacqTraceDbg(" CAMACQ_EXT_ADJUST_SATURATION_LVL_P1 ");
            pvRegs = this->m_pstSensorRegs->pvAdjustSaturationP1;
        }
        break;
        case CAMACQ_EXT_ADJUST_SATURATION_LVL_P2:
        {
            CamacqTraceDbg(" CAMACQ_EXT_ADJUST_SATURATION_LVL_P2 ");
            pvRegs = this->m_pstSensorRegs->pvAdjustSaturationP2;
        }
        break;
        
        default:
        {
            CamacqTraceDbg( " :invalid psival[%d] ", *piVal );
            pvRegs = NULL;
            iRet = -EINVAL;
        }
    }

    if( pvRegs )
    {
        CamacqExtWriteI2cLists( this->m_pstSensor->m_pI2cClient, pvRegs, this->m_pstSensor->m_uiResType );
    }


    
    CamacqTraceOUT();
    return iRet;
}

S32 CamacqExtContrast( struct stCamacqExtAPIs_t* this, _eCamacqExtMode eMode, S32 *piVal )
{
    S32 iRet = 0;
    const void* pvRegs = NULL;
    CamacqTraceIN();
    
    CamacqTraceDbg( ":piVal[%d] ", *piVal );
    
    switch(*piVal)
    {
        case CAMACQ_EXT_ADJUST_CONTRAST_LVL_M2:
        {
            CamacqTraceDbg(" CAMACQ_EXT_ADJUST_CONTRAST_LVL_M2 ");
            pvRegs = this->m_pstSensorRegs->pvAdjustContrastM2;
        }
        break;
        case CAMACQ_EXT_ADJUST_CONTRAST_LVL_M1:
        {
            CamacqTraceDbg(" CAMACQ_EXT_ADJUST_CONTRAST_LVL_M1 ");
            pvRegs = this->m_pstSensorRegs->pvAdjustContrastM1;
        }
        break;
        case CAMACQ_EXT_ADJUST_CONTRAST_LVL_DEFAULT:
        {
            CamacqTraceDbg(" CAMACQ_EXT_ADJUST_CONTRAST_LVL_DEFAULT ");
            pvRegs = this->m_pstSensorRegs->pvAdjustContrastDefault;
        }
        break;
        case CAMACQ_EXT_ADJUST_CONTRAST_LVL_P1:
        {
            CamacqTraceDbg(" CAMACQ_EXT_ADJUST_CONTRAST_LVL_P1 ");
            pvRegs = this->m_pstSensorRegs->pvAdjustContrastP1;
        }
        break;
        case CAMACQ_EXT_ADJUST_CONTRAST_LVL_P2:
        {
            CamacqTraceDbg(" CAMACQ_EXT_ADJUST_CONTRAST_LVL_P2 ");
            pvRegs = this->m_pstSensorRegs->pvAdjustContrastP2;
        }
        break;
        
        default:
        {
            CamacqTraceDbg( " :invalid psival[%d] ", *piVal );
            pvRegs = NULL;
            iRet = -EINVAL;
        }
    }

    if( pvRegs )
    {
        CamacqExtWriteI2cLists( this->m_pstSensor->m_pI2cClient, pvRegs, this->m_pstSensor->m_uiResType );
    }
   
    CamacqTraceOUT();
    return iRet;
}

S32 CamacqExtSharpness( struct stCamacqExtAPIs_t* this, _eCamacqExtMode eMode, S32 *piVal )
{
    S32 iRet = 0;
    const void* pvRegs = NULL;
    CamacqTraceIN();
    
    CamacqTraceDbg( ":piVal[%d] ", *piVal );
    switch(*piVal)
    {
        case CAMACQ_EXT_ADJUST_SHARPNESS_LVL_M2:
        {
            CamacqTraceDbg(" CAMACQ_EXT_ADJUST_SHARPNESS_LVL_M2 ");
            pvRegs = this->m_pstSensorRegs->pvAdjustSharpnessM2;
        }
        break;
        case CAMACQ_EXT_ADJUST_SHARPNESS_LVL_M1:
        {
            CamacqTraceDbg(" CAMACQ_EXT_ADJUST_SHARPNESS_LVL_M1 ");
            pvRegs = this->m_pstSensorRegs->pvAdjustSharpnessM1;
        }
        break;
        case CAMACQ_EXT_ADJUST_SHARPNESS_LVL_DEFAULT:
        {
            CamacqTraceDbg(" CAMACQ_EXT_ADJUST_SHARPNESS_LVL_DEFAULT ");
            pvRegs = this->m_pstSensorRegs->pvAdjustSharpnessDefault;
        }
        break;
        case CAMACQ_EXT_ADJUST_SHARPNESS_LVL_P1:
        {
            CamacqTraceDbg(" CAMACQ_EXT_ADJUST_SHARPNESS_LVL_P1 ");
            pvRegs = this->m_pstSensorRegs->pvAdjustSharpnessP1;
        }
        break;
        case CAMACQ_EXT_ADJUST_SHARPNESS_LVL_P2:
        {
            CamacqTraceDbg(" CAMACQ_EXT_ADJUST_SHARPNESS_LVL_P2 ");
            pvRegs = this->m_pstSensorRegs->pvAdjustSharpnessP2;
        }
        break;
        
        default:
        {
            CamacqTraceDbg( " :invalid psival[%d] ", *piVal );
            pvRegs = NULL;
            iRet = -EINVAL;
        }
    }

    if( pvRegs )
    {
        CamacqExtWriteI2cLists( this->m_pstSensor->m_pI2cClient, pvRegs, this->m_pstSensor->m_uiResType );
    }

    
    CamacqTraceOUT();
    return iRet;
}

S32 CamacqExtAntiBanding( struct stCamacqExtAPIs_t* this, _eCamacqExtMode eMode, S32 *piVal )
{
    S32 iRet = 0;
    const void* pvRegs = NULL;
    CamacqTraceIN();
    CamacqTraceDbg( " :piVal[%d] ", *piVal );

    switch(*piVal)
    {
        case V4L2_CID_POWER_LINE_FREQUENCY_DISABLED:
        {
            CamacqTraceDbg(" V4L2_CID_POWER_LINE_FREQUENCY_DISABLED ");
            pvRegs = this->m_pstSensorRegs->pvFlickerDisabled;
        }
        break;
        case V4L2_CID_POWER_LINE_FREQUENCY_50HZ:
        {
            CamacqTraceDbg(" V4L2_CID_POWER_LINE_FREQUENCY_50HZ ");
            pvRegs = this->m_pstSensorRegs->pvFlicker50Hz;
        }
        break;
        case V4L2_CID_POWER_LINE_FREQUENCY_60HZ:
        {
            CamacqTraceDbg(" V4L2_CID_POWER_LINE_FREQUENCY_60HZ ");
            pvRegs = this->m_pstSensorRegs->pvFlicker60Hz;
        }
        break;
        case V4L2_CID_POWER_LINE_FREQUENCY_AUTO:
        {
            CamacqTraceDbg(" V4L2_CID_POWER_LINE_FREQUENCY_AUTO ");
            pvRegs = this->m_pstSensorRegs->pvFlickerAuto;
        }
        break;
        
        default:
        {
            CamacqTraceErr( " :invalid psival[%d] ", *piVal );
            pvRegs = NULL;
            iRet = -EINVAL;
        }
    }

    if( pvRegs )
    {
        CamacqExtWriteI2cLists( this->m_pstSensor->m_pI2cClient, pvRegs, this->m_pstSensor->m_uiResType );
    }

    CamacqTraceOUT();
    return iRet;
}

// temp
#define CAM_FLASH_ENSET     mfp_to_gpio( MFP_PIN_GPIO1 )
#define CAM_FLASH_FLEN      mfp_to_gpio( MFP_PIN_GPIO0 )
S32 CamacqExtFlashControl( struct stCamacqExtAPIs_t* this, _eCamacqExtMode eMode, S32 *piVal )
{
    S32 iRet = 0;
    int i = 0, num = 0;
    CamacqTraceIN();

    struct v4l2_flash *pstV4l2Flash = (struct v4l2_flash*)( piVal );
    CamacqTraceErr("pstV4l2Flash->flash_operation : %d", pstV4l2Flash->flash_operation);
    CamacqTraceErr("pstV4l2Flash->flash_mode : %d", pstV4l2Flash->flash_mode);

    switch( pstV4l2Flash->flash_operation )
    {
        case V4L2_FLASH_ON:
        {
            gpio_request( CAM_FLASH_ENSET, "camacq" );
            gpio_request( CAM_FLASH_FLEN, "camacq" );

            //spin_lock(&bl_ctrl_lock);
            
            if( pstV4l2Flash->flash_mode == V4L2_PRE_FLASH ) // temp, torch mode
            {
                /* initailize flash IC */
            	gpio_direction_output( CAM_FLASH_ENSET, 0 ); 
                gpio_direction_output( CAM_FLASH_FLEN, 0 ); 
            	mdelay(1); // to enter a shutdown mode
            	
                /* set to movie mode */
            	for( i = 0; i < 3; i++ )
            	{
            		udelay(1);
            		gpio_direction_output( CAM_FLASH_ENSET, 1 ); 
                    
            		udelay(1);
            		gpio_direction_output( CAM_FLASH_ENSET, 0 ); 
            	}
            	gpio_direction_output( CAM_FLASH_ENSET, 1 ); 
            }
            else
            {
#if 0  // denis_flash
                /* initailize flash IC */
                gpio_direction_output( CAM_FLASH_ENSET, 0 ); 
                gpio_direction_output( CAM_FLASH_FLEN, 0 ); 
                mdelay(1); // to enter a shutdown mode

                CamacqTraceErr("***** Denis : Flash main light test");
                gpio_direction_output( CAM_FLASH_FLEN, 1 ); 
                mdelay(500); 
                CamacqTraceErr("***** Denis : Flash main light test, After 500ms delay");

                
#else   // adjust flash current by requested H/W
                // CamacqTraceErr("WINGI AAAAAAAAAAAAAAAAAAA");
                /* initailize flash IC */
            	gpio_direction_output( CAM_FLASH_ENSET, 0 ); 
                gpio_direction_output( CAM_FLASH_FLEN, 0 ); 
            	mdelay(1); // to enter a shutdown mode

                // FLEN high
                gpio_direction_output( CAM_FLASH_FLEN, 1 ); 
                udelay(100);
                
                /* set to movie mode */
            	for( i = 0; i < 4; i++ )
            	{
            		udelay(1);
            		gpio_direction_output( CAM_FLASH_ENSET, 1 ); 
                    
            		udelay(1);
            		gpio_direction_output( CAM_FLASH_ENSET, 0 ); 
            	}
            	gpio_direction_output( CAM_FLASH_ENSET, 1 ); 
                
                mdelay(1); 
#endif                
            }
            
            gpio_free(CAM_FLASH_ENSET);
            gpio_free(CAM_FLASH_FLEN);

            //spin_unlock(&bl_ctrl_lock);
        }
        break;

        case V4L2_FLASH_OFF:
        {
            gpio_request( CAM_FLASH_ENSET, "ledflash" );
            gpio_request( CAM_FLASH_FLEN, "ledflash" );

            /* initailize falsh IC */
            gpio_direction_output( CAM_FLASH_ENSET, 0 ); 
            gpio_direction_output( CAM_FLASH_FLEN, 0 ); 
            mdelay(1); // to enter a shutdown mode

            gpio_free(CAM_FLASH_ENSET);
            gpio_free(CAM_FLASH_FLEN);
        }
        break;

        default:
        {
            CamacqTraceErr("Invalid Zoom value, *piVal:%d", *piVal);
            iRet = -1;
        }
        break;
        
    }
    
    
    CamacqTraceOUT();
    return iRet;
}


// TEMP
static U16 ae_auto;
static U16 ersc_auto;
S32 CamacqExtCameraPrivateControl( struct stCamacqExtAPIs_t* this, _eCamacqExtMode eMode, S32 *piVal )
{
    S32 iRet = 0;
    const void* pvRegs = NULL;
    CamacqTraceIN();
    CamacqTraceDbg( " :piVal[%d] ", *piVal );

    if( eMode == CAMACQ_EXT_MODE_GET )
    {
        switch( *piVal )
        {
            default:
            {
                CamacqTraceErr( " :invalid psival[%d] ", *piVal );
                pvRegs = NULL;
                iRet = -EINVAL;
            }
        }
    }
    else if( eMode == CAMACQ_EXT_MODE_SET )
    {
        switch( *piVal )
        {
            case V4L2_PRIVATE_CTRL_PREVIEW:
            {
                CamacqTraceDbg( "V4L2_PRIVATE_CTRL_PREVIEW" );
                CamacqExtWriteI2cLists( this->m_pstSensor->m_pI2cClient, this->m_pstSensorRegs->pvPreviewRegs, this->m_pstSensor->m_uiResType );
            }
            break;

            case V4L2_PRIVATE_CTRL_SNAPSHOT:
            {
                CamacqTraceDbg( "V4L2_PRIVATE_CTRL_SNAPSHOT" );
#if defined(__ISX012_SONY__)
                {
                    if( !strcmp( this->m_pstSensor->m_szName, "isx012") )
                    {
                        int     iRoofOutCnt = 200;
                        U8      rgucWriteRegs[3] = {0, };
                        U16     usReadAddr = 0;
                        U8      rgucReadData[2] = {0, };
                        U8      ucValue = 0;

                        // INTEN(0x0010)[bit2] <- 0x1
                        rgucWriteRegs[0] = 0x00;
                        rgucWriteRegs[1] = 0x10;
                        rgucWriteRegs[2] = 0x04;
                        CamacqExtWriteI2c( this->m_pstSensor->m_pI2cClient, rgucWriteRegs, 3 ); 

                        // ISX012_Capture_Mode
                        CamacqExtWriteI2cLists( this->m_pstSensor->m_pI2cClient, this->m_pstSensorRegs->pvSnapshotRegs, this->m_pstSensor->m_uiResType );

                        // Wait For Mode Transition(CM)
                        WaitForModeTransition_Sony( this->m_pstSensor->m_pI2cClient, 1 );

                        // Wait For JPEG_UPDATE
                        WaitForModeTransition_Sony( this->m_pstSensor->m_pI2cClient, 2 );  

                        // Read JPG_STS(0x0126)  
                        usReadAddr = 0x0126;
                        do {
                            iRet = CamacqExtReadI2c( this->m_pstSensor->m_pI2cClient, usReadAddr, 2, rgucReadData, 2 );
                            if( iRet < 0  )
                            {
                                CamacqTraceErr(" : error iRet=%d", iRet);
                                return iRet;
                            }
                            iRet = 0;
                            // CamacqExtDelay(1);
                            
                            CamacqTraceDbg( " JPG_STS=0x%x, 0x%x", rgucReadData[1], rgucReadData[0] );    
                            
                            ucValue = rgucReadData[0];
                            CamacqTraceDbg( " : ucValue=0x%x", ucValue);
                            iRoofOutCnt--;
                        }while( (ucValue != 0) && iRoofOutCnt );
                        CamacqTraceDbg(" : iRoofOutCnt=%d", iRoofOutCnt);
                    }
                    else // sub-sensor
                    {
                        CamacqExtWriteI2cLists( this->m_pstSensor->m_pI2cClient, this->m_pstSensorRegs->pvSnapshotRegs, this->m_pstSensor->m_uiResType );
                    }
                }
#else
                CamacqExtWriteI2cLists( this->m_pstSensor->m_pI2cClient, this->m_pstSensorRegs->pvSnapshotRegs, this->m_pstSensor->m_uiResType );
#endif
            }
            break;

            case V4L2_PRIVATE_CTRL_AF_SNAPSHOT:
            {
                CamacqTraceDbg( "V4L2_PRIVATE_CTRL_AF_SNAPSHOT" );
#if defined(__ISX012_SONY__)
                {
                    if( !strcmp( this->m_pstSensor->m_szName, "isx012") )
                    {
                        // Move to Capture Mode
                        int     iRoofOutCnt = 200;
                        U8      rgucWriteRegs[3] = {0, };
                        U16     usReadAddr = 0;
                        U8      rgucReadData[2] = {0, };
                        U8      ucValue = 0;

                        // ISX012_Capture_Mode
                        CamacqExtWriteI2cLists( this->m_pstSensor->m_pI2cClient, this->m_pstSensorRegs->pvSnapshotRegs, this->m_pstSensor->m_uiResType );

                        // Mode Sel
                        /*  ISX012_Capture_Mode 
                        rgucWriteRegs[0] = 0x00;
                        rgucWriteRegs[1] = 0x81;
                        rgucWriteRegs[2] = 0x02;
                        iRet = CamacqExtWriteI2c( this->m_pstSensor->m_pI2cClient, rgucWriteRegs, 3 ); 
                        */
                        
                        // Read MODESEL_FIX(0x0080)==0x2
                        usReadAddr = 0x0080;
                        ucValue = 0;
                        iRoofOutCnt = 50;
                        do {
                            CamacqExtDelay(1);
                            iRet = CamacqExtReadI2c( this->m_pstSensor->m_pI2cClient, usReadAddr, 2, &ucValue, 2 );
                            if( iRet < 0  )
                            {
                                CamacqTraceErr(" : error iRet=%d", iRet);
                                return iRet;
                            }
                            iRet = 0;

                            CamacqTraceDbg( " : MODESEL_FIX ucValue=0x%x", ucValue);
                            
                            iRoofOutCnt--;
                        }while( (ucValue != 2) && iRoofOutCnt );
                        CamacqTraceDbg(" : iRoofOutCnt=%d", iRoofOutCnt);

                        CamacqExtDelay(100);

                        // Read AWBSTS(0x8A24)==0x2
                        usReadAddr = 0x8A24;
                        ucValue = 0;
                        iRoofOutCnt = 50;
                        do {
                            CamacqExtDelay(10);
                            iRet = CamacqExtReadI2c( this->m_pstSensor->m_pI2cClient, usReadAddr, 2, &ucValue, 2 );
                            if( iRet < 0  )
                            {
                                CamacqTraceErr(" : error iRet=%d", iRet);
                                return iRet;
                            }
                            iRet = 0;
                        
                            CamacqTraceDbg( " AWBSTS ucValue=0x%x", ucValue);    
                            
                            iRoofOutCnt--;

                            if( ucValue == 6 )
                            {   
                                break;
                            }
                            
                        }while( (ucValue != 2) && iRoofOutCnt );
                        CamacqTraceDbg(" : iRoofOutCnt=%d", iRoofOutCnt);   

                        // Wait For JPEG_UPDATE
                        WaitForModeTransition_Sony( this->m_pstSensor->m_pI2cClient, 2 );  

                    }
                }
#endif
            }
            break;
            
            case V4L2_PRIVATE_CTRL_HIGHLIGHT_SNAPSHOT:
            {
                CamacqTraceDbg( "V4L2_PRIVATE_CTRL_HIGHLIGHT_SNAPSHOT" );
                CamacqExtWriteI2cLists( this->m_pstSensor->m_pI2cClient, this->m_pstSensorRegs->pvHighLightshotRegs, this->m_pstSensor->m_uiResType );
            }
            break;

            case V4L2_PRIVATE_CTRL_LOWLIGHT_SNAPSHOT:
            {
                CamacqTraceDbg( "V4L2_PRIVATE_CTRL_LOWLIGHT_SNAPSHOT" );
                CamacqExtWriteI2cLists( this->m_pstSensor->m_pI2cClient, this->m_pstSensorRegs->pvLowLightshotRegs, this->m_pstSensor->m_uiResType );
            }
            break;

            case V4L2_PRIVATE_CTRL_NIGHT_SNAPSHOT:
            {
                CamacqTraceDbg( "V4L2_PRIVATE_CTRL_NIGHT_SNAPSHOT" );
                CamacqExtWriteI2cLists( this->m_pstSensor->m_pI2cClient, this->m_pstSensorRegs->pvNightshotRegs, this->m_pstSensor->m_uiResType );
                msleep(300);
            }
            break;

            case V4L2_PRIVATE_CTRL_AE_LOCK:
            {
                CamacqTraceDbg("V4L2_PRIVATE_CTRL_AE_LOCK");
                CamacqExtWriteI2cLists( this->m_pstSensor->m_pI2cClient, this->m_pstSensorRegs->pvAeLock, this->m_pstSensor->m_uiResType );
            }
            break;

            case V4L2_PRIVATE_CTRL_AE_UNLOCK:
            {
                CamacqTraceDbg("V4L2_PRIVATE_CTRL_AE_UNLOCK");
                CamacqExtWriteI2cLists( this->m_pstSensor->m_pI2cClient, this->m_pstSensorRegs->pvAeUnLock, this->m_pstSensor->m_uiResType );
            }
            break;

            case V4L2_PRIVATE_CTRL_AWB_LOCK:
            {
                CamacqTraceDbg("V4L2_PRIVATE_CTRL_AWB_LOCK");
                CamacqExtWriteI2cLists( this->m_pstSensor->m_pI2cClient, this->m_pstSensorRegs->pvAwbLock, this->m_pstSensor->m_uiResType );
            }
            break;

            case V4L2_PRIVATE_CTRL_CHECK_AE_STABLE:
            {
                CamacqTraceDbg("V4L2_PRIVATE_CTRL_CHECK_AE_STABLE");
#if defined(_S5K5CCGX_EVT1_MIPI_)
                if( !strcmp( this->m_pstSensor->m_szName, "s5k5ccgx") )
                {
                    U8	rgucWriteRegs_0[4] = {0xFC,0xFC,0xD0,0x00};
                    U8	rgucWriteRegs_1[4] = {0x00,0x2C,0x70,0x00};
                    U8	rgucWriteRegs_2[4] = {0x00,0x2E,0x1E,0x3C};
                    U8 rgucReadData[2] = {0, };
                    U16 usReadData = 0x00;
                    U16 usReadKey = 0x0F12;
                    U32 uiLoop = 0;

//                    CamacqExtDelay(200);
                    msleep(200);
                    do {
//                        CamacqExtDelay(10);
                        msleep(10);
                        
                        CamacqExtWriteI2c( this->m_pstSensor->m_pI2cClient, rgucWriteRegs_0, 4 );
                        CamacqExtWriteI2c( this->m_pstSensor->m_pI2cClient, rgucWriteRegs_1, 4 );
                        CamacqExtWriteI2c( this->m_pstSensor->m_pI2cClient, rgucWriteRegs_2, 4 );
                        CamacqExtReadI2c( this->m_pstSensor->m_pI2cClient, usReadKey, 2, rgucReadData, 2 );

                        usReadData = ( rgucReadData[0] << 8 ); usReadData |= rgucReadData[1];

                        uiLoop++;

                        CamacqTraceDbg( "usReadData = 0x%x", usReadData );
                    }while( (usReadData != 0x01) && (uiLoop < 20) );
                }
#endif
            }
            break;

            case V4L2_PRIVATE_CTRL_AWB_UNLOCK:
            {
                CamacqTraceDbg("V4L2_PRIVATE_CTRL_AWB_UNLOCK");
                CamacqExtWriteI2cLists( this->m_pstSensor->m_pI2cClient, this->m_pstSensorRegs->pvAwbUnLock, this->m_pstSensor->m_uiResType );
            }
            break;

            case V4L2_PRIVATE_CTRL_PRE_FLASH_START:
            {
                CamacqTraceDbg("V4L2_PRIVATE_CTRL_PRE_FLASH_START");
                CamacqExtWriteI2cLists( this->m_pstSensor->m_pI2cClient, this->m_pstSensorRegs->pvPreFlashStart, this->m_pstSensor->m_uiResType );
            }
            break;

            case V4L2_PRIVATE_CTRL_PRE_FLASH_END:
            {
                CamacqTraceDbg("V4L2_PRIVATE_CTRL_PRE_FLASH_END");
                CamacqExtWriteI2cLists( this->m_pstSensor->m_pI2cClient, this->m_pstSensorRegs->pvPreFlashEnd, this->m_pstSensor->m_uiResType );
            }
            break;

            case V4L2_PRIVATE_CTRL_MAIN_FLASH_START:
            {
                CamacqTraceDbg("V4L2_PRIVATE_CTRL_MAIN_FLASH_START");
#if defined(__ISX012_SONY__)
                if( !strcmp( this->m_pstSensor->m_szName, "isx012") )
                {

                    //denis_flash_2
                    CamacqTraceDbg("Denis : on set");
                    // Flash_ON_SET.ini Write for new flash sequence.
                    CamacqExtWriteI2cLists( this->m_pstSensor->m_pI2cClient, this->m_pstSensorRegs->pvFlashOnSet, this->m_pstSensor->m_uiResType );
                    //delay 60ms
                    CamacqExtDelay(60);

                    // Read AE Scale
                    U16 Temp1 =0, Temp2 = 0;
                    U16 usReadAddr = 0x01CE;

                    CamacqExtReadI2c( this->m_pstSensor->m_pI2cClient, usReadAddr, 2, &ae_auto, 2);
                    usReadAddr = 0x01CA;
                    CamacqExtReadI2c( this->m_pstSensor->m_pI2cClient, usReadAddr, 2, &ersc_auto, 2);

                    Temp1 = (ae_auto & 0x00FF);
                    Temp2 = ( (ae_auto & 0xFF00) >> 8 );
                    ae_auto = (Temp1 << 8 ) | Temp2;
                    CamacqTraceDbg( " ae_auto=0x%x", ae_auto );

                    Temp1 = 0, Temp2 = 0;
                    Temp1 = (ersc_auto & 0x00FF);
                    Temp2 = ( (ersc_auto & 0xFF00) >> 8 );
                    ersc_auto = (Temp1 << 8 ) | Temp2;
                    CamacqTraceDbg( " ersc_auto=0x%x", ersc_auto );
                    
                    // Flash_ON_SET.ini Write
                    CamacqExtWriteI2cLists( this->m_pstSensor->m_pI2cClient, this->m_pstSensorRegs->pvMainFlashStart, this->m_pstSensor->m_uiResType );
                }
#else
                CamacqExtWriteI2cLists( this->m_pstSensor->m_pI2cClient, this->m_pstSensorRegs->pvMainFlashStart, this->m_pstSensor->m_uiResType );
#endif
            }
            break;

            case V4L2_PRIVATE_CTRL_MAIN_FLASH_END:
            {
                CamacqTraceDbg("V4L2_PRIVATE_CTRL_MAIN_FLASH_END");
#if defined(__ISX012_SONY__)
                if( !strcmp( this->m_pstSensor->m_szName, "isx012") )
                {
                    // Flash_OFF_RESET.ini Write
                    CamacqExtWriteI2cLists( this->m_pstSensor->m_pI2cClient, this->m_pstSensorRegs->pvMainFlashEnd, this->m_pstSensor->m_uiResType );

                    // VPARA_TRG(0x8800)ก็0x1
                    U8  rgucWriteRegs[3] = {0, };
                    rgucWriteRegs[0] = 0x88;
                    rgucWriteRegs[1] = 0x00;
                    rgucWriteRegs[2] = 0x01;
                    CamacqExtWriteI2c( this->m_pstSensor->m_pI2cClient, rgucWriteRegs, 3 );
                }
#else
                CamacqExtWriteI2cLists( this->m_pstSensor->m_pI2cClient, this->m_pstSensorRegs->pvMainFlashEnd, this->m_pstSensor->m_uiResType );
#endif
            }
            break;

            case V4L2_PRIVATE_CTRL_FLASH_AE_SET:
            {
                CamacqTraceDbg("V4L2_PRIVATE_CTRL_FLASH_AE_SET");
                CamacqExtWriteI2cLists( this->m_pstSensor->m_pI2cClient, this->m_pstSensorRegs->pvFlashAeSet, this->m_pstSensor->m_uiResType );
            }
            break;

            case V4L2_PRIVATE_CTRL_FLASH_AE_CLEAR:
            {
                CamacqTraceDbg("V4L2_PRIVATE_CTRL_FLASH_AE_CLEAR");
                CamacqExtWriteI2cLists( this->m_pstSensor->m_pI2cClient, this->m_pstSensorRegs->pvFlashAeClear, this->m_pstSensor->m_uiResType );
            }
            break;

            case V4L2_PRIVATE_CTRL_DTP_ON:
            {
                CamacqTraceDbg("V4L2_PRIVATE_CTRL_DTP_ON");
                
#if defined(_S5K5CCGX_EVT1_MIPI_)
                if( !strcmp(this->m_pstSensor->m_szName, "s5k5ccgx") )
                {
                    CamacqTraceDbg("WINGI DTP_ON RESET");

                    CamacqExtRequestGpio( mfp_to_gpio(MFP_PIN_GPIO94), "camacq" );  // MCAM_STBYN
                    gpio_direction_output( mfp_to_gpio( MFP_PIN_GPIO94 ), 0 );    // MCAM_RSTN
                    CamacqExtDelay(10);
                    gpio_direction_output( mfp_to_gpio( MFP_PIN_GPIO94 ), 1 );    // MCAM_RSTN
                    CamacqExtDelay(20);

                    CamacqExtFreeGpio(mfp_to_gpio( MFP_PIN_GPIO94 ));
                }
#endif               
                CamacqExtWriteI2cLists( this->m_pstSensor->m_pI2cClient, this->m_pstSensorRegs->pvDtpOn, this->m_pstSensor->m_uiResType );
            }
            break;

            case V4L2_PRIVATE_CTRL_DTP_OFF:
            {
                CamacqTraceDbg("V4L2_PRIVATE_CTRL_DTP_OFF");
                
#if defined(_S5K5CCGX_EVT1_MIPI_)
                if( !strcmp(this->m_pstSensor->m_szName, "s5k5ccgx") )
                {
                    CamacqTraceDbg("WINGI DTP_ON RESET");

                    CamacqExtRequestGpio( mfp_to_gpio(MFP_PIN_GPIO94), "camacq" );  // MCAM_STBYN
                    gpio_direction_output( mfp_to_gpio( MFP_PIN_GPIO94 ), 0 );    // MCAM_RSTN
                    CamacqExtDelay(10);
                    gpio_direction_output( mfp_to_gpio( MFP_PIN_GPIO94 ), 1 );    // MCAM_RSTN
                    CamacqExtDelay(20);

                    CamacqExtFreeGpio(mfp_to_gpio( MFP_PIN_GPIO94 ));
                }
                // init 
                CamacqExtWriteI2cLists( this->m_pstSensor->m_pI2cClient, this->m_pstSensorRegs->pvInitRegs, this->m_pstSensor->m_uiResType );
#else               
                CamacqExtWriteI2cLists( this->m_pstSensor->m_pI2cClient, this->m_pstSensorRegs->pvDtpOff, this->m_pstSensor->m_uiResType );
#endif
            }
            break;

            case V4L2_PRIVATE_CTRL_AFTER_CAPTURE:
            {
                CamacqTraceDbg("V4L2_PRIVATE_CTRL_AFTER_CAPTURE");
#if defined(__ISX012_SONY__)
                if( !strcmp( this->m_pstSensor->m_szName, "isx012") )
                {
                    // INTEN(0x0010)[bit2] <- 0x00
                    U8 rgucWriteRegs[3] = {0, };
                    rgucWriteRegs[0] = 0x00;
                    rgucWriteRegs[1] = 0x10;
                    rgucWriteRegs[2] = 0x00;
                    CamacqExtWriteI2c( this->m_pstSensor->m_pI2cClient, rgucWriteRegs, 3 ); 

                    // INTCLR(0x0012)[bit2] <- 0x04 // ???
                    rgucWriteRegs[0] = 0x00;
                    rgucWriteRegs[1] = 0x12;
                    rgucWriteRegs[2] = 0x04;
                    CamacqExtWriteI2c( this->m_pstSensor->m_pI2cClient, rgucWriteRegs, 3 ); 
                }
#endif      
            }
            break;

            case V4L2_PRIVATE_CTRL_AFTER_PREVIEW:
            {
                CamacqTraceDbg("V4L2_PRIVATE_CTRL_AFTER_PREVIEW");
#if defined(__ISX012_SONY__)
                if( !strcmp( this->m_pstSensor->m_szName, "isx012") )
                {
                    // Wait For Mode Transition(CM)
                    WaitForModeTransition_Sony( this->m_pstSensor->m_pI2cClient, 1 );

                    // INTCLR(0x0012)[bit2] <- 0x04 // ???
                    U8 rgucWriteRegs[3] = {0, };
                    rgucWriteRegs[0] = 0x00;
                    rgucWriteRegs[1] = 0x12;
                    rgucWriteRegs[2] = 0x04;
                    CamacqExtWriteI2c( this->m_pstSensor->m_pI2cClient, rgucWriteRegs, 3 ); 
                }
#endif      
            }
            break;
            
            default:
            {
                CamacqTraceErr( " :invalid psival[%d] ", *piVal );
                pvRegs = NULL;
                iRet = -EINVAL;
            }
        }
    }
    else
    {
        CamacqTraceErr("mode error : %d", eMode);
        iRet = -EINVAL;
    }

    CamacqTraceOUT();
    return iRet;
}


static bool g_bCheck1st = false;
S32 CamacqExtAutofocus( struct stCamacqExtAPIs_t* this, _eCamacqExtMode eMode, S32 *piVal )
{
    S32 iRet = 0;
    _stCamacqSensor* pSensor = this->m_pstSensor;
    CamacqTraceIN( ":piVal[%d]", *piVal );

    if( eMode == CAMACQ_EXT_MODE_GET )
    {
        // Check Can Support AF
        if( pSensor->m_bCanAF == false )
        {
            *piVal = V4L2_ATFCS_STT_NOT_SUPPORT;
            goto CAMACQ_EXT_OUT;
        }

#if defined(__ISX006_SONY__) || defined(__ISX012_SONY__)
        {
            U16 usReadAddr = 0;
            U16 usReadAddr1 = 0;
            U16 usReadAddr2 = 0;        
            U8 ucAFState = 0, ucAFResultFirst = 0, ucAFResultSecond = 0;
            U8 rgucWriteRegs[3] = {0, };

            if( !strcmp(pSensor->m_szName, "isx006") ) 
            {
                usReadAddr = 0x6D76;    // AF_STATE
            }
            else if( !strcmp(pSensor->m_szName, "isx012") )
            {
                usReadAddr = 0x8B8A;   // AF_STATE
            }
            else
            {
                CamacqTraceErr(": err 1");
                *piVal = V4L2_ATFCS_STT_NOT_SUPPORT;
                goto CAMACQ_EXT_OUT;
            }

            CamacqExtDelay(10);
            CamacqExtReadI2c( pSensor->m_pI2cClient, usReadAddr, 2, &ucAFState, 1);
            CamacqTraceDbg( " AF_STATE : 0x%2x", ucAFState);
            
            if( ucAFState == 0x08 )
            {
                // Clear AF_LOCK_STS
                if( !strcmp(pSensor->m_szName, "isx006") ) 
                {
                    rgucWriteRegs[0] = 0x00;
                    rgucWriteRegs[1] = 0xFC;
                    rgucWriteRegs[2] = 0x01;
                }
                else if( !strcmp(pSensor->m_szName, "isx012") )
                {
                    rgucWriteRegs[0] = 0x00;
                    rgucWriteRegs[1] = 0x12;
                    rgucWriteRegs[2] = 0x10;
                }
                
                CamacqExtWriteI2c( pSensor->m_pI2cClient, rgucWriteRegs, 3 );
                CamacqExtDelay(1);

                if( !strcmp(pSensor->m_szName, "isx006") ) 
                {
                    usReadAddr1 = 0x6D3A;    // AF_RESULT_First
                    usReadAddr2 = 0x6D52;    // AF_RESULT_Second 

                    CamacqExtReadI2c( pSensor->m_pI2cClient, usReadAddr1, 2, &ucAFResultFirst, 1);
                    CamacqExtReadI2c( pSensor->m_pI2cClient, usReadAddr2, 2, &ucAFResultSecond, 1);
                   
                    CamacqTraceDbg( " AF_RESULT_First : 0x%2x, AF_RESULT_Second : 0x%2x", ucAFResultFirst, ucAFResultSecond);
                    if( ucAFResultFirst == 0x01 && ucAFResultSecond == 0x01 )
                    {
                        *piVal = V4L2_ATFCS_STT_FOCUSED;    // AF Success
                    }
                    else
                    {
                        *piVal = V4L2_ATFCS_STT_NOT_FOCUSED; // AF fail
                    }

                }
                else if( !strcmp(pSensor->m_szName, "isx012") )
                {
                    usReadAddr1 = 0x8B8B;    // AF_RESULT_First

                    CamacqExtReadI2c( pSensor->m_pI2cClient, usReadAddr1, 2, &ucAFResultFirst, 1);
                                       
                    CamacqTraceDbg( " AF_RESULT_First : 0x%2x", ucAFResultFirst);
                    if( ucAFResultFirst == 0x01 )
                    {
                        *piVal = V4L2_ATFCS_STT_FOCUSED;    // AF Success
                    }
                    else if( ucAFResultFirst == 0x00 )
                    {
                        *piVal = V4L2_ATFCS_STT_NOT_FOCUSED; // AF fail
                    }
                    else
                    {
                        *piVal = V4L2_ATFCS_STT_NOT_FOCUSED; // AF fail
                    }
                }

                // OFF AF
                CamacqExtWriteI2cLists( pSensor->m_pI2cClient, this->m_pstSensorRegs->pvOffAFRegs, pSensor->m_uiResType );
            }
            else
            {
                *piVal = V4L2_ATFCS_STT_SEARCHING;
            }            
        }
#elif defined(_S5K4ECGX_EVT1_MIPI_)
        {
            // 1st check status 
            U8	rgucWriteRegs_1[4] = {0x00,0x2C,0x70,0x00};
            U8	rgucWriteRegs_2[4] = {0x00, };
            U8  rgucReadData[2] = {0x00, };
            U16 usReadData = 0x00;
            U16 usReadKey = 0x0F12;
            U32 uiLoop = 0;

            if( this->m_pstSensor->m_uiFirmwareVer == 0x0011 )
            {
                rgucWriteRegs_2[0] = 0x00; rgucWriteRegs_2[1] = 0x2E; rgucWriteRegs_2[2] = 0x2E; rgucWriteRegs_2[3] = 0xEE; 
            }
            else
            {
                rgucWriteRegs_2[0] = 0x00; rgucWriteRegs_2[1] = 0x2E; rgucWriteRegs_2[2] = 0x2E; rgucWriteRegs_2[3] = 0x06; 
            }

            do 
            {
                if( pSensor->m_pstCamacqSettings->stV4l2CurScene == V4L2_CTRL_SCENE_NIGHT )
                {
                    CamacqExtDelay(250);    // 1 frame delay
                }
                else 
                {
                    CamacqExtDelay(100);    // 1 frame delay
                }   
                
                CamacqExtWriteI2c( this->m_pstSensor->m_pI2cClient, rgucWriteRegs_1, 4 );
                CamacqExtWriteI2c( this->m_pstSensor->m_pI2cClient, rgucWriteRegs_2, 4 );
                CamacqExtReadI2c( this->m_pstSensor->m_pI2cClient, usReadKey, 2, rgucReadData, 2 );

                usReadData = ( rgucReadData[0] << 8 ); usReadData |= rgucReadData[1];
                CamacqTraceDbg( " 1st check status, usReadData : 0x%x", usReadData );

                switch( usReadData & 0xFF )
                {
                    case 1:
                    CamacqTraceDbg( "1st CAM_AF_PROGRESS " );
                    *piVal = V4L2_ATFCS_STT_SEARCHING;
                    break;

                    case 2:
                    CamacqTraceDbg( "1st CAM_AF_SUCCESS " );
                    *piVal = V4L2_ATFCS_STT_FOCUSED;
                    break;

                    default:                    
                    CamacqTraceDbg("1st CAM_AF_FAIL ");
                    *piVal = V4L2_ATFCS_STT_NOT_FOCUSED;
                    break;
                }            
                uiLoop++;
                CamacqTraceDbg( " 1st uiLoop : %d", uiLoop );
            }while( (*piVal == V4L2_ATFCS_STT_SEARCHING) && (uiLoop < 100) );

            if( (*piVal == V4L2_ATFCS_STT_NOT_FOCUSED) || (uiLoop >= 100) )
            {
                 *piVal = V4L2_ATFCS_STT_NOT_FOCUSED;
                CamacqTraceErr("1st CAM_AF_FAIL, goto CAMACQ_EXT_OUT ");
                goto CAMACQ_EXT_OUT;
            }

            // 2st check status
            if( this->m_pstSensor->m_uiFirmwareVer == 0x0011 )
            {
                rgucWriteRegs_2[0] = 0x00; rgucWriteRegs_2[1] = 0x2E; rgucWriteRegs_2[2] = 0x22; rgucWriteRegs_2[3] = 0x07; 
            }
            else
            {
                rgucWriteRegs_2[0] = 0x00; rgucWriteRegs_2[1] = 0x2E; rgucWriteRegs_2[2] = 0x21; rgucWriteRegs_2[3] = 0x67; 
            }

            uiLoop = 0;
            do
            {
                if( pSensor->m_pstCamacqSettings->stV4l2CurScene == V4L2_CTRL_SCENE_NIGHT )
                {
                    CamacqExtDelay(250);    // 1 frame delay
                }
                else 
                {
                    CamacqExtDelay(100);    // 1 frame delay
                }   

                CamacqExtWriteI2c( this->m_pstSensor->m_pI2cClient, rgucWriteRegs_1, 4 );
                CamacqExtWriteI2c( this->m_pstSensor->m_pI2cClient, rgucWriteRegs_2, 4 );
                CamacqExtReadI2c( this->m_pstSensor->m_pI2cClient, usReadKey, 2, rgucReadData, 2 );

                usReadData = 0;
                usReadData = ( rgucReadData[0] << 8 ); usReadData |= rgucReadData[1];
                CamacqTraceDbg( " 2st check status, usReadData : 0x%x", usReadData );

                switch( usReadData & 0xFF )
                {
                    case 0:
                    CamacqTraceDbg( "2st CAM_AF_SUCCESS " );
                    *piVal = V4L2_ATFCS_STT_FOCUSED;
                    break;

                    default:                    
                    CamacqTraceDbg("2st CAM_AF_PROGRESS ");
                    *piVal = V4L2_ATFCS_STT_SEARCHING;
                    break;
                }    

                uiLoop++;
                CamacqTraceDbg( " 1st uiLoop : %d", uiLoop );
            } while( (*piVal == V4L2_ATFCS_STT_SEARCHING) && (uiLoop < 100) );

            if( uiLoop >= 100 )
            {
                 *piVal = V4L2_ATFCS_STT_NOT_FOCUSED;
                CamacqTraceErr("2st CAM_AF_FAIL, goto CAMACQ_EXT_OUT ");
                goto CAMACQ_EXT_OUT;
            }
        }
#elif defined(_S5K5CCGX_EVT1_MIPI_)
        {
            S32 iResult = 0;
            U8	rgucWriteRegs_0[4] = {0xFC,0xFC,0xD0,0x00};
            U8	rgucWriteRegs_1[4] = {0x00,0x2C,0x70,0x00};
            U8	rgucWriteRegs_2[4] = {0x00,0x2E,0x2D,0x12};
            U8  rgucReadData[2] = {0, };
            U16 usReadData = 0x00;
            U16 usReadKey = 0x0F12;

            // 1st check
            if( g_bCheck1st == true )
            {
                CamacqExtWriteI2c( this->m_pstSensor->m_pI2cClient, rgucWriteRegs_0, 4 );
                CamacqExtWriteI2c( this->m_pstSensor->m_pI2cClient, rgucWriteRegs_1, 4 );
                CamacqExtWriteI2c( this->m_pstSensor->m_pI2cClient, rgucWriteRegs_2, 4 );

//                CamacqExtDelay(10);
                CamacqExtReadI2c( this->m_pstSensor->m_pI2cClient, usReadKey, 2, rgucReadData, 2 );
                
                usReadData = ( rgucReadData[0] << 8 ); usReadData |= rgucReadData[1];
                CamacqTraceDbg( " 1st check status, usReadData : 0x%x", usReadData );

                switch( usReadData & 0xFF )
                {
                    case 1:
                    CamacqTraceDbg( "1st CAM_AF_PROGRESS " );
                    *piVal = V4L2_ATFCS_STT_SEARCHING;
                    break;

                    case 2:
                    CamacqTraceDbg( "1st CAM_AF_SUCCESS " );
                    *piVal = V4L2_ATFCS_STT_FOCUSED;
                    g_bCheck1st = false;
                    break;

                    default:                    
                    CamacqTraceDbg("1st CAM_AF_FAIL ");
                    *piVal = V4L2_ATFCS_STT_NOT_FOCUSED;
                    break;
                }            
            }
            else
            {
                // 2nd check status
                rgucWriteRegs_2[0] = 0x00; 
                rgucWriteRegs_2[1] = 0x2E; 
                rgucWriteRegs_2[2] = 0x1F; 
                rgucWriteRegs_2[3] = 0x2F; 

     
//                CamacqExtDelay(10);
                CamacqExtWriteI2c( this->m_pstSensor->m_pI2cClient, rgucWriteRegs_1, 4 );
                CamacqExtWriteI2c( this->m_pstSensor->m_pI2cClient, rgucWriteRegs_2, 4 );
                CamacqExtReadI2c( this->m_pstSensor->m_pI2cClient, usReadKey, 2, rgucReadData, 2 );

                usReadData = 0;
                usReadData = ( rgucReadData[0] << 8 ); usReadData |= rgucReadData[1];
                CamacqTraceDbg( " 2nd check status, usReadData : 0x%x", usReadData );

                switch( usReadData & 0xFF00 )
                {
                    case 0:
                    CamacqTraceDbg( "2nd CAM_AF_SUCCESS " );
                    *piVal = V4L2_ATFCS_STT_FOCUSED;
                    break;

                    default:                    
                    CamacqTraceDbg("2nd CAM_AF_PROGRESS ");
                    *piVal = V4L2_ATFCS_STT_SEARCHING;
                    break;
                }    
            }
        }
#else
        // lsi ????????????????? 
        {
            U16 usReadFlag = 0x0F12;
            U8  rgucReadRegs[2] = {0, };
            U16 usAFvalue = 0;

            CamacqExtWriteI2cLists( pSensor->m_pI2cClient, this->m_pstSensorRegs->pvCheckAFRegs, pSensor->m_uiResType );
            CamacqExtDelay(1);
            CamacqExtReadI2c( pSensor->m_pI2cClient, usReadFlag, 2, rgucReadRegs, 2 );

            usAFvalue = ( rgucReadRegs[1] << 8 );
            usAFvalue |= rgucReadRegs[0];

            CamacqTraceDbg( " :rgucReadRegs[0x%x%x], usAFvalue : %x ", rgucReadRegs[0], rgucReadRegs[1], usAFvalue) ;
            switch( usAFvalue )
            {
                case CAMACQ_EXT_AF_CONTINUE:
                {
                    CamacqTraceDbg(" CAMACQ_EXT_AF_CONTINUE ");
                    *piVal = V4L2_ATFCS_STT_SEARCHING;
                }
                break;
                case CAMACQ_EXT_AF_FAILD:
                {
                    CamacqTraceDbg(" CAMACQ_EXT_AF_FAILD ");
                    *piVal = V4L2_ATFCS_STT_NOT_FOCUSED;
                }
                break;
                case CAMACQ_EXT_AF_SUCCESS:
                {
                    CamacqTraceDbg(" CAMACQ_EXT_AF_SUCCESS ");
                    *piVal = V4L2_ATFCS_STT_FOCUSED;
                }
                break;
                default:
                {
                    *piVal = V4L2_ATFCS_STT_NOT_FOCUSED;
                    iRet = -EINVAL;
                    goto CAMACQ_EXT_OUT;
                }
                break;
            }
        }
#endif /* __ISX006_SONY__ */ 
    }
    else /* CAMACQ_EXT_MODE_SET */
    {
        switch( (enum v4l2_autofocus_ctrl)(*piVal) )
        {
            case V4L2_ATFCS_CTRL_SINGLE:
            {
#if defined(_S5K4ECGX_EVT1_MIPI_) || defined(_S5K5CCGX_EVT1_MIPI_)
                CamacqExtWriteI2cLists( pSensor->m_pI2cClient, this->m_pstSensorRegs->pvSetAFRegs, pSensor->m_uiResType );
                g_bCheck1st = true;
                
#elif defined(__ISX006_SONY__) || defined(__ISX012_SONY__)
                CamacqExtWriteI2cLists( pSensor->m_pI2cClient, this->m_pstSensorRegs->pvSetAF_NLUXRegs, pSensor->m_uiResType );
                CamacqExtDelay(1);
                CamacqExtWriteI2cLists( pSensor->m_pI2cClient, this->m_pstSensorRegs->pvSetAFRegs, pSensor->m_uiResType );

                CamacqExtDelay(60); // 1Frame Delay
#endif /* _S5K4ECGX_EVT1_MIPI_ */
            }
            break;

            case V4L2_ATFCS_CTRL_AF_FLASH_ON:
            {
#if defined(__ISX012_SONY__)
                if( !strcmp( pSensor->m_szName, "isx012") )
                {
                    struct v4l2_flash  stV4l2Flash;
                    U8          rgucWriteRegs[3] = {0, };
                    int         iRoofOutCnt = 200;
                    U8          ucValue = 0;
                    U16         usReadAddr = 0x0080;    //INTSTS        
                    U8          rgucReadData[2] = {0, };
                    
                    // Move to Half Rel Mode
                    // CamacqExtWriteI2cLists( pSensor->m_pI2cClient, this->m_pstSensorRegs->pvSetAFRegs, pSensor->m_uiResType );

                    // Move to Half Rel Mode is same
                    // VPARA_TRG(0x8800)ก็0x1
                    rgucWriteRegs[0] = 0x88;
                    rgucWriteRegs[1] = 0x00;
                    rgucWriteRegs[2] = 0x01;
                    CamacqExtWriteI2c( pSensor->m_pI2cClient, rgucWriteRegs, 3 );
                                      

                    // Wait 1V time
                    CamacqExtDelay(40);                    

                    // Pre LED Flash ON
                    stV4l2Flash.flash_mode = V4L2_PRE_FLASH;
                    stV4l2Flash.flash_operation = V4L2_FLASH_ON;
                    CamacqExtFlashControl( pSensor->m_pstExtAPIs, CAMACQ_EXT_MODE_SET, (S32*)&stV4l2Flash );

                    // Read MODESEL_FIX(0x0080)==0x1   
                    iRoofOutCnt = 100;
                    usReadAddr = 0x0080; 
                    do {
                        CamacqExtDelay(1);
                        CamacqExtReadI2c( pSensor->m_pI2cClient, usReadAddr, 2, rgucReadData, 2 );
                        CamacqTraceDbg( " MODESEL_FIX=0x%x, 0x%x", rgucReadData[1], rgucReadData[0] );    
                        
                        ucValue = rgucReadData[0];
                        CamacqTraceDbg( " : MODESEL_FIX ucValue=0x%x", ucValue);
                        iRoofOutCnt--;
                    }while( (ucValue != 1) && iRoofOutCnt );
                    CamacqTraceDbg( " : iRoofOutCnt=%d", iRoofOutCnt);

                    // Read HALF_MOVE_STS(0x01B0)==0x0
                    iRoofOutCnt = 100;
                    usReadAddr = 0x01B0;
                    do {
                        CamacqExtDelay(10);
                        CamacqExtReadI2c( pSensor->m_pI2cClient, usReadAddr, 2, rgucReadData, 2 );
                        CamacqTraceDbg( " HALF_MOVE_STS=0x%x, 0x%x", rgucReadData[1], rgucReadData[0] );    
                        
                        ucValue = rgucReadData[0];
                        CamacqTraceDbg( " : HALF_MOVE_STS ucValue=0x%x", ucValue);
                        iRoofOutCnt--;
                    }while( (ucValue != 0) && iRoofOutCnt );
                    CamacqTraceDbg( " : iRoofOutCnt=%d", iRoofOutCnt);

                }
#endif             
            }
            break;

            case V4L2_ATFCS_CTRL_AF_FLASH_OFF:
            {
#if defined(__ISX012_SONY__)
                if( !strcmp( pSensor->m_szName, "isx012") )
                {
                    // Read AE Scale
                    struct v4l2_flash stV4l2Flash;
                    U8 rgucWriteRegs[3] = {0, };
                    U16 aeoffset = 0;
                    U16 ersc_now = 0;
                    U16 ae_now = 0;
                    U16 Temp1 =0, Temp2 = 0;
                    U16 usReadAddr = 0;
                    
                    usReadAddr = 0x01CC;
                    CamacqExtReadI2c( this->m_pstSensor->m_pI2cClient, usReadAddr, 2, &ersc_now, 2);
                    usReadAddr = 0x01D0;
                    CamacqExtReadI2c( this->m_pstSensor->m_pI2cClient, usReadAddr, 2, &ae_now, 2);

                    Temp1 = (ersc_now & 0x00FF);
                    Temp2 = ( (ersc_now & 0xFF00) >> 8 );
                    ersc_now = (Temp1 << 8 ) | Temp2;
                    CamacqTraceDbg( " ersc_now=0x%x", ersc_now );

                    Temp1 = 0, Temp2 = 0;
                    Temp1 = (ae_now & 0x00FF);
                    Temp2 = ( (ae_now & 0xFF00) >> 8 );
                    ae_now = (Temp1 << 8 ) | Temp2;
                    CamacqTraceDbg( " ae_now=0x%x", ae_now );

                    // CAP_GAINOFFSET
                    aeoffset = calculate_AEgain_offset(ae_auto, ae_now, ersc_auto, ersc_now, pSensor->m_uiResType);
                    CamacqTraceDbg(" : aeoffset=0x%d", aeoffset); 
                    
                    rgucWriteRegs[0] = 0x01;
                    rgucWriteRegs[1] = 0x86;
                    rgucWriteRegs[2] = (aeoffset & 0xFF);
                    CamacqExtWriteI2c( this->m_pstSensor->m_pI2cClient, rgucWriteRegs, 3 );

                    rgucWriteRegs[1] = 0x87;
                    rgucWriteRegs[2] = ((aeoffset >> 8) & 0xFF);
                    CamacqExtWriteI2c( this->m_pstSensor->m_pI2cClient, rgucWriteRegs, 3 );

                    // CPUEXT(0x5000)ก็0x0A    
                    rgucWriteRegs[0] = 0x50;
                    rgucWriteRegs[1] = 0x00;
                    rgucWriteRegs[2] = 0x0A;
                    CamacqExtWriteI2c( this->m_pstSensor->m_pI2cClient, rgucWriteRegs, 3 );

                    // Pre LED Flash OFF
                    stV4l2Flash.flash_mode = V4L2_PRE_FLASH;
                    stV4l2Flash.flash_operation = V4L2_FLASH_OFF;
                    CamacqExtFlashControl( pSensor->m_pstExtAPIs, CAMACQ_EXT_MODE_SET, (S32*)&stV4l2Flash );

                }
#endif                
            }
            break;

            case V4L2_ATFCS_CTRL_AF_CANCEL_MACRO:
            {
#if defined(__ISX012_SONY__)
                if( !strcmp( pSensor->m_szName, "isx012") )
                {
                    CamacqTraceDbg(" V4L2_ATFCS_CTRL_AF_CANCEL_MACRO ");

                     // OFF AF
                    CamacqExtWriteI2cLists( pSensor->m_pI2cClient, this->m_pstSensorRegs->pvOffAFRegs, pSensor->m_uiResType );
                    
                    // 
                    CamacqExtWriteI2cLists( pSensor->m_pI2cClient, this->m_pstSensorRegs->pvCancelMacroAFRegs, pSensor->m_uiResType );
                }
#endif
            }
            break;

            case V4L2_ATFCS_CTRL_AF_CANCEL_MANUAL:
            {
#if defined(__ISX012_SONY__)
                if( !strcmp( pSensor->m_szName, "isx012") )
                {
                    CamacqTraceDbg(" V4L2_ATFCS_CTRL_AF_CANCEL_MANUAL ");

                     // OFF AF
                    CamacqExtWriteI2cLists( pSensor->m_pI2cClient, this->m_pstSensorRegs->pvOffAFRegs, pSensor->m_uiResType );
                    
                    // 
                    CamacqExtWriteI2cLists( pSensor->m_pI2cClient, this->m_pstSensorRegs->pvCancelManualAFRegs, pSensor->m_uiResType );
                }
#endif
            }
            break;

            
            default:
            {
                CamacqTraceErr( " :invalid (enum v4l2_autofocus_ctrl)(*piVal) : [%d]", 
                    (enum v4l2_autofocus_ctrl)(*piVal) );
                iRet = -EINVAL;
            }
        }
        

    }

CAMACQ_EXT_OUT:
    CamacqTraceOUT( ":piVal[%d]", *piVal );
    return iRet;
}

S32 CamacqExtAutofocusMode( struct stCamacqExtAPIs_t* this, _eCamacqExtMode eMode, S32 *piVal )
{
    S32 iRet = 0, iDelay;
    const void* pvRegs = NULL;
    _stCamacqSensor* pSensor = this->m_pstSensor;
    CamacqTraceIN( ":piVal[%d]", *piVal );
    
    switch(*piVal)
    {
        case V4L2_ATFCS_MODE_AUTO:
        case V4L2_ATFCS_MODE_NORMAL:
        case V4L2_ATFCS_MODE_INFINITY:
        case V4L2_ATFCS_MODE_FIXED:
        case V4L2_ATFCS_MODE_EDOF:
        {
            CamacqTraceDbg(" V4L2_ATFCS_MODE_AUTO(NORMAL) ");
#if defined(_S5K4ECGX_EVT1_MIPI_)
            if( pSensor->m_pstCamacqSettings->stV4l2CurScene == V4L2_CTRL_SCENE_NIGHT )
            {
                iDelay = 250;
            }
            else 
            {
                iDelay = 100;
            }
            CamacqExtWriteI2cLists( this->m_pstSensor->m_pI2cClient, this->m_pstSensorRegs->pvSetAFNormalMode1, this->m_pstSensor->m_uiResType );
            CamacqExtDelay(iDelay);
            CamacqExtWriteI2cLists( this->m_pstSensor->m_pI2cClient, this->m_pstSensorRegs->pvSetAFNormalMode2, this->m_pstSensor->m_uiResType );
            CamacqExtDelay(iDelay);
            CamacqExtWriteI2cLists( this->m_pstSensor->m_pI2cClient, this->m_pstSensorRegs->pvSetAFNormalMode3, this->m_pstSensor->m_uiResType );
            CamacqExtDelay(iDelay);
#elif defined(__ISX012_SONY__)
            if( !strcmp(pSensor->m_szName, "isx012") )
            {
                CamacqExtWriteI2cLists( this->m_pstSensor->m_pI2cClient, 
                    this->m_pstSensorRegs->pvOffAFRegs, this->m_pstSensor->m_uiResType );    
            }
            pvRegs = this->m_pstSensorRegs->pvManualAFReg;
#else
            pvRegs = this->m_pstSensorRegs->pvManualAFReg;
#endif /* _S5K4ECGX_EVT1_MIPI_ */

        }
        break;
        
        case V4L2_ATFCS_MODE_MACRO:
        {
            CamacqTraceDbg(" V4L2_ATFCS_MODE_MACRO ");
#if defined(_S5K4ECGX_EVT1_MIPI_)
            if( pSensor->m_pstCamacqSettings->stV4l2CurScene == V4L2_CTRL_SCENE_NIGHT )
            {
                iDelay = 250;
            }
            else 
            {
                iDelay = 100;
            }
            CamacqExtWriteI2cLists( this->m_pstSensor->m_pI2cClient, this->m_pstSensorRegs->pvSetAFMacroMode1, this->m_pstSensor->m_uiResType );
            CamacqExtDelay(iDelay);
            CamacqExtWriteI2cLists( this->m_pstSensor->m_pI2cClient, this->m_pstSensorRegs->pvSetAFMacroMode2, this->m_pstSensor->m_uiResType );
            CamacqExtDelay(iDelay);
            CamacqExtWriteI2cLists( this->m_pstSensor->m_pI2cClient, this->m_pstSensorRegs->pvSetAFMacroMode3, this->m_pstSensor->m_uiResType );
            CamacqExtDelay(iDelay);
            
#elif defined(__ISX012_SONY__)
            if( !strcmp(pSensor->m_szName, "isx012") )
            {
                CamacqExtWriteI2cLists( this->m_pstSensor->m_pI2cClient, 
                    this->m_pstSensorRegs->pvOffAFRegs, this->m_pstSensor->m_uiResType );    
            }
            pvRegs = this->m_pstSensorRegs->pvMacroAFReg;
#else

            pvRegs = this->m_pstSensorRegs->pvMacroAFReg;
#endif /* _S5K4ECGX_EVT1_MIPI_ */

            
        }
        break;

//denis_af_restart
        case V4L2_ATFCS_MODE_RESTART:
        {
            CamacqTraceDbg(" V4L2_ATFCS_MODE_RESTART ");
#if defined(_S5K4ECGX_EVT1_MIPI_)
            
#elif defined(__ISX012_SONY__)

            pvRegs = this->m_pstSensorRegs->pvResetAFRegs;
#else

            pvRegs = this->m_pstSensorRegs->pvResetAFRegs;
#endif /* _S5K4ECGX_EVT1_MIPI_ */

        }
        break;
        
        case V4L2_ATFCS_MODE_RET_AUTO:
        {
            CamacqTraceDbg(" V4L2_ATFCS_MODE_RET_AUTO ");
            pvRegs = this->m_pstSensorRegs->pvReturnManualAFReg;
        }
        break;
        
        case V4L2_ATFCS_MODE_RET_MACRO:
        {
            CamacqTraceDbg(" V4L2_ATFCS_MODE_RET_MACRO ");
            pvRegs = this->m_pstSensorRegs->pvReturnMacroAFReg;
        }
        break;
       
        default:
        {
            CamacqTraceErr( " :invalid psival[%d] ", *piVal );
            pvRegs = NULL;
            iRet = -EINVAL;
        }
    }

    if( pvRegs )
    {
        CamacqExtWriteI2cLists( this->m_pstSensor->m_pI2cClient, pvRegs, this->m_pstSensor->m_uiResType );
    }

    CamacqTraceOUT( ":piVal[%d]", *piVal );
    return iRet;
}

S32 CamacqExtInitCamcorder( struct stCamacqExtAPIs_t* this, _eCamacqExtMode eMode, S32 *piVal )
{
    S32 iRet = 0;
    const void* pvRegs = NULL;
    CamacqTraceIN();

    pvRegs = this->m_pstSensorRegs->pvCamcorderOnRegs;

    if( pvRegs )
    {
        CamacqExtWriteI2cLists( this->m_pstSensor->m_pI2cClient, pvRegs, this->m_pstSensor->m_uiResType );
    }
    else
    {
        CamacqTraceErr(" this->m_pstSensorRegs->pvCamcorderOnRegs is NULL");
        iRet = -1;
    }

    CamacqTraceOUT();
    return iRet;
}

S32 CamacqExtSetCamcorderSize( struct stCamacqExtAPIs_t* this, _eCamacqExtMode eMode, S32 *piVal )
{
    S32 iRet = 0;
    const void* pvRegs = NULL;
    enum v4l2_camcorder_size eSize = (enum v4l2_camcorder_size)(*piVal);

    CamacqTraceIN();

    switch( eSize )
    {
        case V4L2_CAMCORDER_176_144:
        {
            CamacqTraceDbg("V4L2_CAMCORDER_176_144");
            pvRegs = this->m_pstSensorRegs->pvQCIF;
        }
        break;

        case V4L2_CAMCORDER_320_240:
        {
            CamacqTraceDbg("V4L2_CAMCORDER_320_240");
            pvRegs = this->m_pstSensorRegs->pvQVGA;
        }
        break;

        case V4L2_CAMCORDER_640_480:
        {
            CamacqTraceDbg("V4L2_CAMCORDER_640_480");
            pvRegs = this->m_pstSensorRegs->pvVGA;
        }
        break;

        case V4L2_CAMCORDER_720_480:
        {
            CamacqTraceDbg("V4L2_CAMCORDER_720_480");
            pvRegs = this->m_pstSensorRegs->pv720_480;
        }
        break;

        case V4L2_CAMCORDER_800_480:
        {
            CamacqTraceDbg("V4L2_CAMCORDER_800_480");
            pvRegs = this->m_pstSensorRegs->pv800_480;
        }
        break;

        case V4L2_CAMCORDER_1280_720:
        {
            CamacqTraceDbg("V4L2_CAMCORDER_1280_720");
            pvRegs = this->m_pstSensorRegs->pv720P;
        }
        break;

        case V4L2_CAMCORDER_1280_1080:
        {
            CamacqTraceDbg("V4L2_CAMCORDER_1280_1080");
            pvRegs = this->m_pstSensorRegs->pv1080P;
        }
        break;

        default:
        {
            CamacqTraceErr(" NOT SUPPORTED VALUE : %d", eSize);
            pvRegs = NULL;
        }
        
    }

    if( pvRegs )
    {
        CamacqExtWriteI2cLists( this->m_pstSensor->m_pI2cClient, pvRegs, this->m_pstSensor->m_uiResType );
    }    

    CamacqTraceOUT();
    return iRet;
}

S32 CamacqExtInitVT( struct stCamacqExtAPIs_t* this, _eCamacqExtMode eMode, S32 *piVal )
{
    S32 iRet = 0;
    const void* pvRegs = NULL;
    CamacqTraceIN();

    pvRegs = this->m_pstSensorRegs->pvCamcorderOnRegs;

    if( pvRegs )
    {
        CamacqExtWriteI2cLists( this->m_pstSensor->m_pI2cClient, pvRegs, this->m_pstSensor->m_uiResType );
    }
    else
    {
        CamacqTraceErr(" this->m_pstSensorRegs->pvCamcorderOnRegs is NULL");
        iRet = -1;
    }
    
    CamacqTraceOUT();
    return iRet;
}

S32 CamacqExtSetVTSize( struct stCamacqExtAPIs_t* this, _eCamacqExtMode eMode, S32 *piVal )
{
    S32 iRet = 0;
    const void* pvRegs = NULL;
    enum v4l2_camcorder_size eSize = (enum v4l2_camcorder_size)(*piVal);

    CamacqTraceIN();

    switch( eSize )
    {
        case V4L2_CAMCORDER_176_144:
        {
            CamacqTraceDbg("V4L2_CAMCORDER_176_144");
            pvRegs = this->m_pstSensorRegs->pvQCIF;
        }
        break;

        case V4L2_CAMCORDER_320_240:
        {
            CamacqTraceDbg("V4L2_CAMCORDER_320_240");
            pvRegs = this->m_pstSensorRegs->pvQVGA;
        }
        break;

        case V4L2_CAMCORDER_640_480:
        {
            CamacqTraceDbg("V4L2_CAMCORDER_640_480");
            pvRegs = this->m_pstSensorRegs->pvVGA;
        }
        break;

        default:
        {
            CamacqTraceErr(" NOT SUPPORTED VALUE : %d", eSize);
            pvRegs = NULL;
        }
        
    }

    if( pvRegs )
    {
        CamacqExtWriteI2cLists( this->m_pstSensor->m_pI2cClient, pvRegs, this->m_pstSensor->m_uiResType );
    }    

    CamacqTraceOUT();
    return iRet;
}

S32 CamacqExtSetPreviewSize( struct stCamacqExtAPIs_t* this, _eCamacqExtMode eMode, S32 *piVal )
{
    S32 iRet = 0;
    const void* pvRegs = NULL;
    enum v4l2_camcorder_size eSize = (enum v4l2_camcorder_size)(*piVal);

    CamacqTraceIN();

    switch( eSize )
    {
        case V4L2_CAMCORDER_176_144:
        {
            CamacqTraceDbg("V4L2_CAMCORDER_176_144");
            pvRegs = this->m_pstSensorRegs->pvQCIF;
        }
        break;

        case V4L2_CAMCORDER_320_240:
        {
            CamacqTraceDbg("V4L2_CAMCORDER_320_240");
            pvRegs = this->m_pstSensorRegs->pvQVGA;
        }
        break;

        case V4L2_CAMCORDER_640_480:
        {
            CamacqTraceDbg("V4L2_CAMCORDER_640_480");
            pvRegs = this->m_pstSensorRegs->pvVGA;
        }
        break;

        default:
        {
            CamacqTraceErr(" NOT SUPPORTED VALUE : %d", eSize);
            pvRegs = NULL;
        }
        
    }

    if( pvRegs )
    {
        CamacqExtWriteI2cLists( this->m_pstSensor->m_pI2cClient, pvRegs, this->m_pstSensor->m_uiResType );
    }    

    CamacqTraceOUT();
    return iRet;
}


S32 CamacqExtSetFps( struct stCamacqExtAPIs_t* this, _eCamacqExtMode eMode, S32 *piVal )
{
    S32 iRet = 0;
    const void* pvRegs = NULL;
    enum v4l2_fps eFps = (enum v4l2_fps)(*piVal);

    CamacqTraceIN();

    switch( eFps )
    {
        case V4L2_FPS_FIXED_15:     
        {
            CamacqTraceDbg("V4L2_FPS_FIXED_15");
            pvRegs = this->m_pstSensorRegs->pvFpsFixed_15;
        }
        break;
        
        case V4L2_FPS_FIXED_25:     //denis_25fps
        {
            CamacqTraceDbg("V4L2_FPS_FIXED_25");
            pvRegs = this->m_pstSensorRegs->pvFpsFixed_25;
        }
        break;

        case V4L2_FPS_FIXED_30:
        {
            CamacqTraceDbg("V4L2_FPS_FIXED_30");
            pvRegs = this->m_pstSensorRegs->pvFpsFixed_30;
        }
        break;

        default:
        {
            CamacqTraceErr(" NOT SUPPORTED VALUE : %d", eFps);
            pvRegs = NULL;
        }
    }

    if( pvRegs )
    {
        CamacqExtWriteI2cLists( this->m_pstSensor->m_pI2cClient, pvRegs, this->m_pstSensor->m_uiResType );
    }    

    CamacqTraceOUT();
    return iRet;
}

S32 CamacqExtWhitebalance( struct stCamacqExtAPIs_t* this, _eCamacqExtMode eMode, const S8* piVal )
{
    S32 iRet = 0, iIdx;
    CamacqTraceIN();

#if 0 /* TODO : will be changed .. */
    for( iIdx = 0; iIdx < CAMACQ_EXT_WB_MAX; iIdx++ )
    {
        if( !strncmp(piVal, (S8*)&g_V4l2CamacqWBModes[iIdx], MAX_WB_NAME_LENGTH) )
        break;
    }

    CamacqTraceDbg( ":iIdx[%d]", iIdx );
    switch( iIdx )
    {
        case CAMACQ_EXT_WB_AUTO:
        {
            CamacqExtWriteI2cLists( this->m_pstSensor->m_pI2cClient, this->m_pstSensorRegs->pvWBAutoRegs, this->m_pstSensor->uiResType );
        }
        break;
        case CAMACQ_EXT_WB_DAYLIGHT:
        {
            CamacqExtWriteI2cLists( this->m_pstSensor->m_pI2cClient, this->m_pstSensorRegs->pvWBDaylightRegs, this->m_pstSensor->uiResType );
        }
        break;
        case CAMACQ_EXT_WB_TUNGSTEN:
        {
            CamacqExtWriteI2cLists( this->m_pstSensor->m_pI2cClient, this->m_pstSensorRegs->pvWBDaylightRegs, this->m_pstSensor->uiResType );
        }
        break;
        case CAMACQ_EXT_WB_FLUORESCENT:
        {
            CamacqExtWriteI2cLists( this->m_pstSensor->m_pI2cClient, this->m_pstSensorRegs->pvWBFluorescentRegs, this->m_pstSensor->uiResType );
        }
        break;
        case CAMACQ_EXT_WB_HORIZON:
        {
            CamacqExtWriteI2cLists( this->m_pstSensor->m_pI2cClient, this->m_pstSensorRegs->pvWBFluorescentRegs, this->m_pstSensor->uiResType );
        }
        break;
        case CAMACQ_EXT_WB_OFF:
        {
            CamacqExtWriteI2cLists( this->m_pstSensor->m_pI2cClient, this->m_pstSensorRegs->pvWBAutoRegs, this->m_pstSensor->uiResType );
        }
        break;
        default:
        {
            CamacqTraceErr( " :invalid value " );
            iRet = -EINVAL;
        }
        break;
    }
#endif

    CamacqTraceOUT("OUT");
    return iRet;
}



/* Ext Common API */

/* Register control API */
/**
EX) 6AA sensor read // CAMACQ_SUB_REG_DAT_SZ is 4

// read 0x7000 01DA
U8 read_address[][CAMACQ_SUB_REG_DAT_SZ]= {
0xFCFCD000,
0x002C7000,
0x002E01DA };

U16 read_flag = 0x0F12;
U8 read_data[2] = { 0, };

camacq_ext_i2c_write_reglists( pclient, read_address, CAMACQ_SENSOW_LOW );
canacq_ext_i2c_read( pclient, read_flag, read_data, 2, CAMACQ_SENSOW_LOW );

S32 camacq_ext_i2c_write( struct i2c_client* pclient, U8 * pucvalue, U8 ucnb_write )

**/
S32 CamacqExtReadI2c( struct i2c_client *pClient, U16 usAddr, U8 ucAddrSize, U8 * pucBuf, U8 ucReadSize )
{
    S32 iRet = 0;
    struct i2c_msg stMsgs[2];
    U8 ucrgAddrBuf[CAMACQ_MAIN_EXT_RD_SZ];

    // CamacqTraceDbg_v("IN");

    if( !(pClient->adapter) )
    {
        CamacqTraceErr( ":client failed" );
        iRet= -ENODEV;
        goto CAMACQ_EXT_OUT;
    }

    if( ucAddrSize == 1 )
    {
        ucrgAddrBuf[0] = (U8)(usAddr);
    }
    else if( ucAddrSize == 2 )
    {
        ucrgAddrBuf[0] = (U8)(usAddr>>8);
        ucrgAddrBuf[1] = (U8)(usAddr);
    }
    else
    {
        CamacqTraceErr( ":invalid AddrSize ucAddrSize[%d]", ucAddrSize );
    }

    stMsgs[0].addr = pClient->addr;
    stMsgs[0].flags = 0;
    stMsgs[0].len = ucAddrSize;
    stMsgs[0].buf = ucrgAddrBuf;

    stMsgs[1].addr = pClient->addr;
    stMsgs[1].flags = I2C_M_RD; /* read flag*/
    stMsgs[1].len = ucReadSize;
    stMsgs[1].buf = pucBuf;

    iRet = i2c_transfer( pClient->adapter, stMsgs, CAMACQ_EXT_MODE_R );
    if( iRet < 0 )
    {
        CamacqTraceErr( ":transfer failed" );
        iRet = -ENODEV;
    }

CAMACQ_EXT_OUT:
    // CamacqTraceDbg_v();
    return iRet;
}

S32 CamacqExtWriteI2c( struct i2c_client *pClient, U8 * pucValue, U8 ucSize )
{
    struct i2c_msg stMsg;
    S32 iRet = 0;
    // CamacqTraceDbg_v("IN");

    if( !(pClient->adapter) )
    {
        CamacqTraceErr( ":client failed" );
        iRet = -ENODEV;
        goto CAMACQ_EXT_OUT;
    }

    stMsg.addr = pClient->addr;
    stMsg.flags = I2C_M_TEN;

    stMsg.buf = pucValue;   
    stMsg.len = ucSize; 

    iRet = i2c_transfer( pClient->adapter, &stMsg, CAMACQ_EXT_MODE_W );
    if(iRet < 0)
    {
        CamacqTraceErr( ":transfer failed, iRet=%d",iRet );
    }

CAMACQ_EXT_OUT:
    // CamacqTraceDbg_v("OUT");

    return iRet;
}

S32 CamacqExtWriteI2cLists( struct i2c_client *pClient, const void *pvArg, int iResType )
{
    S32 iRet = 0;
    S32 iNext = 0;
    U8	rgucWriteRegs[4] = {0, };
    U16	usDealy;

    CamacqTraceDbg_v("IN");

#if defined(__ISX006_SONY__) || defined(__ISX012_SONY__)
    // check Sony sensor, temp code
    if( !strcmp( pClient->name, "isx006" ) || !strcmp( pClient->name, "isx012" ) )
    {
        return CamacqExtWriteI2cLists_Sony( pClient, pvArg, iResType );
    }
#endif /* __ISX006_SONY__ */
    	
	if( iResType == CAMACQ_SENSOR_MAIN )
    {
#if (CAMACQ_MAIN_BURST_MODE) 
        U8  rgucNextRegs[4] = {0, };
        U8  ucBurstCnt = 0;
        U8  rgucBurstRegs[CAMACQ_MAIN_BURST_MAX] = {0, };   
#endif /* CAMACQ_BURST_MODE */

///// init reglists variable. ///////////////////////////////////////////////////////
#if (CAMACQ_MAIN_FS_MODE)
        U8* pvRegLists = CamacqExtReadFs( pvArg, CAMACQ_MAIN_INT_MODE, iResType );
#else // CAMACQ_MAIN_FS_MODE
#if (CAMACQ_MAIN_2BYTE_SENSOR)

#if (CAMACQ_MAIN_REG_DAT_SZ==1)
        U8 (*pvRegLists)[CAMACQ_MAIN_REG_SET_SZ] = (U8(*)[CAMACQ_MAIN_REG_SET_SZ])(pvArg);
#elif (CAMACQ_MAIN_REG_DAT_SZ==2)
        U16* pvRegLists = (U16*)(pvArg);
#endif

#else // CAMACQ_MAIN_2BYTE_SENSOR

#if (CAMACQ_MAIN_REG_DAT_SZ==2)
        U16 (*pvRegLists)[CAMACQ_MAIN_REG_SET_SZ] = (U16(*)[CAMACQ_MAIN_REG_SET_SZ])(pvArg);
#elif (CAMACQ_MAIN_REG_DAT_SZ==1) 
        U8 (*pvRegLists)[CAMACQ_MAIN_REG_SET_SZ] = (U8(*)[CAMACQ_MAIN_REG_SET_SZ])(pvArg);
#elif (CAMACQ_MAIN_REG_DAT_SZ==4)
        U32* pvRegLists = (U32*)(pvArg);
#endif 
#endif /* CAMACQ_MAIN_2BYTE_SENSOR */ 
#endif /* CAMACQ_MAIN_FS_MODE */
//// init reglists valiable. ///////////////////////////////////////////////////////

        if( pvRegLists == NULL )
            return -1;

        // start!!
        CAMACQ_MAIN_EXT_REG_GET_DATA( rgucWriteRegs, pvRegLists, iNext )
        while( !CAMACQ_MAIN_EXT_REG_IS_BTM_OF_DATA(rgucWriteRegs) )
        {
            if( CAMACQ_MAIN_EXT_REG_IS_DELAY(rgucWriteRegs) )
            {
#if (CAMACQ_MAIN_2BYTE_SENSOR)
                usDealy = rgucWriteRegs[1];
                CamacqTraceDbg(": 0x%02x%02x" , rgucWriteRegs[0], rgucWriteRegs[1]);
#else
                usDealy = (rgucWriteRegs[2]<<8 | rgucWriteRegs[3]);
                CamacqTraceDbg(": 0x%02X%02X%02X%02X" , rgucWriteRegs[0] , rgucWriteRegs[1], rgucWriteRegs[2] , rgucWriteRegs[3]);
#endif /* CAMACQ_MAIN_2BYTE_SENSOR */
                CamacqExtDelay(usDealy);
                iRet = 0;
            }
            else
            {
#if (CAMACQ_MAIN_BURST_MODE)
                CAMACQ_MAIN_EXT_REG_GET_DATA(rgucNextRegs, pvRegLists, iNext+1)
                if( CAMACQ_MAIN_EXT_REG_IS_CNTS(rgucWriteRegs) && CAMACQ_MAIN_EXT_REG_IS_CNTS(rgucNextRegs) )
                {
                    memset(rgucBurstRegs, 0x00, CAMACQ_MAIN_BURST_MAX);
                    // copy first 0x0f12 data
                    ucBurstCnt = 0;
                    memcpy(rgucBurstRegs + ucBurstCnt, rgucWriteRegs, sizeof(rgucWriteRegs));
                    ucBurstCnt+=sizeof(rgucWriteRegs);

                    iNext++;
                    CAMACQ_MAIN_EXT_REG_GET_DATA(rgucWriteRegs, pvRegLists, iNext) 
                    do
                    {
                        memcpy(rgucBurstRegs+ucBurstCnt, rgucWriteRegs+2, 2);
                        ucBurstCnt+=2;
                        iNext++;
                        CAMACQ_MAIN_EXT_REG_GET_DATA(rgucWriteRegs, pvRegLists, iNext) 
                        if(ucBurstCnt == CAMACQ_MAIN_BURST_MAX)
                        break;

                    } while( CAMACQ_MAIN_EXT_REG_IS_CNTS(rgucWriteRegs) );

#if (CAMACQ_MAIN_BURST_I2C_TRACE)
                    int i;
                    for( i=2 ; i<ucBurstCnt; i+=2) {
                        CamacqTraceDbg(": 0x%02X%02X%02X%02X" , rgucBurstRegs[0], rgucBurstRegs[1], rgucBurstRegs[i] , rgucBurstRegs[i+1]);
                    }
                    CamacqTraceDbg(":burst count[%d]", ucBurstCnt );
#endif /* CAMACQ_MAIN_BURST_I2C_TRACE */
					
                    iRet = CamacqExtWriteI2c( pClient, rgucBurstRegs, ucBurstCnt );
                    if(iRet < 0)
                    {
                        CamacqTraceErr( ":write failed" );
                        break;
                    }
                    continue;
                }
                else
#endif /* CAMACQ_MAIN_BURST_MODE */
                {
#if (CAMACQ_MAIN_2BYTE_SENSOR)         
                    CamacqTraceDbg(": 0x%02x%02x" , rgucWriteRegs[0], rgucWriteRegs[1]);
#else
                    CamacqTraceDbg(": 0x%02X%02X%02X%02X" , rgucWriteRegs[0] , rgucWriteRegs[1], rgucWriteRegs[2] , rgucWriteRegs[3]);
#endif

                    iRet = CamacqExtWriteI2c( pClient, rgucWriteRegs, CAMACQ_MAIN_REG_SET_SZ*CAMACQ_MAIN_REG_DAT_SZ );
                    if(iRet < 0)
                    {
                        CamacqTraceErr( ":write failed =============== break!!!!!!!!!!!!!!" );
                        iRet = -1;
                        return iRet;
                    }
                } // else
            } // else
	                
            iNext++;
            CAMACQ_MAIN_EXT_REG_GET_DATA(rgucWriteRegs, pvRegLists, iNext)
        } // while

#if (CAMACQ_MAIN_FS_MODE)
        CamacqFree( pvRegLists );
#endif /* CAMACQ_MAIN_FS_MODE */

    }
#if (CAMACQ_SENSOR_MAX==2)
    else // irestype == CAMACQ_SENSOR_LOW
    {
#if (CAMACQ_SUB_BURST_MODE) 
        U8  rgucNextRegs[4] = {0, };
        U8  ucBurstCnt = 0;
        U8  rgucBurstRegs[CAMACQ_SUB_BURST_MAX] = {0, };   
#endif /* CAMACQ_BURST_MODE */

//// init reglists variable. ///////////////////////////////////////////////////////
#if (CAMACQ_SUB_FS_MODE)
        U8* pvRegLists = CamacqExtReadFs( pvArg, CAMACQ_SUB_INT_MODE, iResType );
#else // CAMACQ_SUB_FS_MODE
#if (CAMACQ_SUB_2BYTE_SENSOR)

#if (CAMACQ_SUB_REG_DAT_SZ==1)
        U8 (*pvRegLists)[CAMACQ_SUB_REG_SET_SZ] = (U8(*)[CAMACQ_SUB_REG_SET_SZ])(pvArg);
#elif (CAMACQ_SUB_REG_DAT_SZ==2)
        U16* pvRegLists = (U16*)(pvArg);
#endif

#else // CAMACQ_SUB_2BYTE_SENSOR

#if (CAMACQ_SUB_REG_DAT_SZ==2)
        U16 (*pvRegLists)[CAMACQ_SUB_REG_SET_SZ] = (U16(*)[CAMACQ_SUB_REG_SET_SZ])(pvArg);
#elif (CAMACQ_SUB_REG_DAT_SZ==1) 
        U8 (*pvRegLists)[CAMACQ_SUB_REG_SET_SZ] = (U8(*)[CAMACQ_SUB_REG_SET_SZ])(pvArg);
#elif (CAMACQ_SUB_REG_DAT_SZ==4)
        U32* pvRegLists = (U32*)(pvArg);
#endif 
#endif /* CAMACQ_SUB_2BYTE_SENSOR */ 
#endif /* CAMACQ_SUB_FS_MODE */
//// init reglists valiable. ///////////////////////////////////////////////////////

        if( pvRegLists == NULL )
            return -1;

        // start!!
        CAMACQ_SUB_EXT_REG_GET_DATA( rgucWriteRegs, pvRegLists, iNext )
        while( !CAMACQ_MAIN_EXT_REG_IS_BTM_OF_DATA(rgucWriteRegs) )
        {
            if( CAMACQ_MAIN_EXT_REG_IS_DELAY(rgucWriteRegs) )
            {
#if (CAMACQ_SUB_2BYTE_SENSOR)
                usDealy = rgucWriteRegs[1];
                CamacqTraceDbg(": 0x%02x%02x" , rgucWriteRegs[0], rgucWriteRegs[1]);
#else
                usDealy = (rgucWriteRegs[2]<<8 | rgucWriteRegs[3]);
                CamacqTraceDbg(": 0x%02X%02X%02X%02X" , rgucWriteRegs[0] , rgucWriteRegs[1], rgucWriteRegs[2] , rgucWriteRegs[3]);
#endif /* CAMACQ_SUB_2BYTE_SENSOR */
                CamacqExtDelay(usDealy);
                iRet = 0;
            }
            else
            {
#if (CAMACQ_SUB_BURST_MODE)
                CAMACQ_SUB_EXT_REG_GET_DATA(rgucNextRegs, pvRegLists, iNext+1)
                if( CAMACQ_SUB_EXT_REG_IS_CNTS(rgucWriteRegs) && CAMACQ_SUB_EXT_REG_IS_CNTS(rgucNextRegs) )
                {
                    memset(rgucBurstRegs, 0x00, CAMACQ_SUB_BURST_MAX);
                    // copy first 0x0f12 data
                    ucBurstCnt = 0;
                    memcpy(rgucBurstRegs + ucBurstCnt, rgucWriteRegs, sizeof(rgucWriteRegs));
                    ucBurstCnt+=sizeof(rgucWriteRegs);

                    iNext++;
                    CAMACQ_SUB_EXT_REG_GET_DATA(rgucWriteRegs, pvRegLists, iNext) 
                    do
                    {
                        memcpy(rgucBurstRegs+ucBurstCnt, rgucWriteRegs+2, 2);
                        ucBurstCnt+=2;
                        iNext++;
                        CAMACQ_SUB_EXT_REG_GET_DATA(rgucWriteRegs, pvRegLists, iNext) 
                        if(ucBurstCnt == CAMACQ_SUB_BURST_MAX)
                        break;

                    } while( CAMACQ_SUB_EXT_REG_IS_CNTS(rgucWriteRegs) );

#if (CAMACQ_SUB_BURST_I2C_TRACE)
                    int i;
                    for( i=2 ; i<ucBurstCnt; i+=2) {
                        CamacqTraceDbg(": 0x%02X%02X%02X%02X" , rgucBurstRegs[0], rgucBurstRegs[1], rgucBurstRegs[i] , rgucBurstRegs[i+1]);
                    }
                    CamacqTraceDbg(":burst count[%d]", ucBurstCnt);
#endif /* CAMACQ_SUB_BURST_I2C_TRACE */
					
                    iRet = CamacqExtWriteI2c( pClient, rgucBurstRegs, ucBurstCnt );
                    if(iRet < 0)
                    {
                        CamacqTraceErr( ":write failed" );
                        break;
                    }
                    continue;
                }
                else
#endif /* CAMACQ_SUB_BURST_MODE */
                {
#if (CAMACQ_SUB_2BYTE_SENSOR)         
                    CamacqTraceDbg(": 0x%02x%02x" , rgucWriteRegs[0], rgucWriteRegs[1]);
#else
                    CamacqTraceDbg(": 0x%02X%02X%02X%02X" , rgucWriteRegs[0] , rgucWriteRegs[1], rgucWriteRegs[2] , rgucWriteRegs[3]);
#endif

                    iRet = CamacqExtWriteI2c( pClient, rgucWriteRegs, CAMACQ_SUB_REG_SET_SZ*CAMACQ_SUB_REG_DAT_SZ );
                    if(iRet < 0)
                    {
                        CamacqTraceErr( ":write failed =============== break!!!!!!!!!!!!!!" );
                        iRet = -1;
                        return iRet;
                    }

                } // else
            } // else
	                
            iNext++;
            CAMACQ_SUB_EXT_REG_GET_DATA(rgucWriteRegs, pvRegLists, iNext)
        } // while

#if (CAMACQ_SUB_FS_MODE)
        CamacqFree( pvRegLists );
#endif /* CAMACQ_SUB_FS_MODE */

    }
#endif /* CAMACQ_SENSOR_MAX==2 */
	
    CamacqTraceDbg( ":size[%d]", iNext );
    CamacqTraceDbg_v("OUT");

    return iRet;
}

#if defined(__ISX006_SONY__) || defined(__ISX012_SONY__)
S32 CamacqExtWriteI2cLists_Sony( struct i2c_client *pClient, const void *pvArg, int iResType )
{
    S32 iRet = 0;
    U32 uiCnt = 0;
    U8 rgucWriteRegs[4] = {0, };
    _stSonyData* pstRegLists = 0;
    
    U8* pcRegLists = 0;
    U8 ucAddrLen = 0;
    S32 iIdx = 0;

    CamacqTraceDbg_v("IN");

    CamacqTraceDbg( ":IN addr : %x ", pClient->addr );

    if( iResType == CAMACQ_SENSOR_MAIN )
    {
#if (CAMACQ_MAIN_FS_MODE)
        pcRegLists = CamacqExtReadFs( (const S8*)pvArg, CAMACQ_EXT_LEN_SONY, iResType );
        if( pcRegLists == NULL )
            return -1;
        
        {   
            iIdx = 0;
            ucAddrLen = pcRegLists[ iIdx++ ];
            while( ucAddrLen != 0xFF )
            {
                if( ucAddrLen == 0x02 )
                {
                    rgucWriteRegs[0] = pcRegLists[iIdx++];
                    rgucWriteRegs[1] = pcRegLists[iIdx++];
                    rgucWriteRegs[3] = pcRegLists[iIdx++];
                    rgucWriteRegs[2] = pcRegLists[iIdx++];
                }
                else if( ucAddrLen == 0x01 || ucAddrLen == 0x03 )
                {
                    rgucWriteRegs[0] = pcRegLists[iIdx++];
                    rgucWriteRegs[1] = pcRegLists[iIdx++];
                    rgucWriteRegs[2] = pcRegLists[iIdx++];
                    rgucWriteRegs[3] = 0x00;
                }
                else
                {
                    CamacqTraceErr("Unexpected value!!");
                    return iRet;   
                }

                // 

                if( ucAddrLen != 0x03 )
                {
                    iRet = CamacqExtWriteI2c( pClient, rgucWriteRegs, 2+ucAddrLen );
                    if(iRet < 0)
                    {
                        CamacqTraceErr( ":write failed" );
                        iRet = -1;
                        return iRet;
                    }
                
                    // CamacqTraceDbg(":Length[%d] : 0x%02X%02X%02X%02X" , ucAddrLen, rgucWriteRegs[0] , 
                    //                                 rgucWriteRegs[1], rgucWriteRegs[2] , rgucWriteRegs[3]);
                }
                else
                {
                    CamacqExtDelay(rgucWriteRegs[2]);
                    CamacqTraceDbg(" setfile delay : %d", rgucWriteRegs[2]);   
                }

                ucAddrLen = pcRegLists[ iIdx++ ];
            }
        }
        CamacqFree( pcRegLists );
#else /* CAMACQ_MAIN_FS_MODE */ 
        pstRegLists = (_stSonyData*)pvArg;
        {
            if( pstRegLists[uiCnt].ucLen == 0x02 )
            {
                rgucWriteRegs[0] = (U8)((pstRegLists[uiCnt].usRegs >> 8) & 0xFF);
                rgucWriteRegs[1] = (U8)(pstRegLists[uiCnt].usRegs & 0xFF);
                rgucWriteRegs[2] = (U8)(pstRegLists[uiCnt].usData & 0xFF);
                rgucWriteRegs[3] = (U8)(pstRegLists[uiCnt].usData >> 8 & 0xFF);   
            }
            else if( pstRegLists[uiCnt].ucLen == 0x01 || pstRegLists[uiCnt].ucLen == 0x03 )
            {
                rgucWriteRegs[0] = (U8)((pstRegLists[uiCnt].usRegs >> 8) & 0xFF);
                rgucWriteRegs[1] = (U8)(pstRegLists[uiCnt].usRegs & 0xFF);
                rgucWriteRegs[2] = (U8)(pstRegLists[uiCnt].usData & 0xFF);
                rgucWriteRegs[3] = 0x00;
            }
            else if( pstRegLists[uiCnt].ucLen == 0xFF )
            {
                rgucWriteRegs[0] = (U8)((pstRegLists[uiCnt].usRegs >> 8) & 0xFF);
                rgucWriteRegs[1] = (U8)(pstRegLists[uiCnt].usRegs & 0xFF);
                rgucWriteRegs[2] = (U8)(pstRegLists[uiCnt].usData & 0xFF);
                rgucWriteRegs[3] = (U8)(pstRegLists[uiCnt].usData >> 8 & 0xFF);
            }
            else
            {
                CamacqTraceErr("Unexpected value!!");
                return iRet;
            }

            while( rgucWriteRegs[0] != 0xFF || rgucWriteRegs[1] != 0xFF || rgucWriteRegs[2] != 0xFF )
            {
                if( pstRegLists[uiCnt].ucLen != 0x03 )
                {
                    iRet = CamacqExtWriteI2c( pClient, rgucWriteRegs, 2+pstRegLists[uiCnt].ucLen );
                    if(iRet < 0)
                    {
                        CamacqTraceErr( ":write failed =============== break!!!!!!!!!!!!!!" );
                        iRet = -1;
                        return iRet;
                    }
            
                    CamacqTraceDbg(":Length[%d] : 0x%02X%02X%02X%02X" , pstRegLists[uiCnt].ucLen, rgucWriteRegs[0] , 
                                               rgucWriteRegs[1], rgucWriteRegs[2] , rgucWriteRegs[3]);
                }
                else // 0x03 is delay
                {
                    CamacqExtDelay(rgucWriteRegs[2]);
                    CamacqTraceDbg(" setfile delay : %d", rgucWriteRegs[2]);
                }        
        
                uiCnt++;
        
                if( pstRegLists[uiCnt].ucLen == 0x02 )
                {
                    rgucWriteRegs[0] = (U8)((pstRegLists[uiCnt].usRegs >> 8) & 0xFF);
                    rgucWriteRegs[1] = (U8)(pstRegLists[uiCnt].usRegs & 0xFF);

                    rgucWriteRegs[2] = (U8)(pstRegLists[uiCnt].usData & 0xFF);
                    rgucWriteRegs[3] = (U8)(pstRegLists[uiCnt].usData >> 8 & 0xFF);   
                }
                else if(pstRegLists[uiCnt].ucLen == 0x01 || pstRegLists[uiCnt].ucLen == 0x03)
                {
                    rgucWriteRegs[0] = (U8)((pstRegLists[uiCnt].usRegs >> 8) & 0xFF);
                    rgucWriteRegs[1] = (U8)(pstRegLists[uiCnt].usRegs & 0xFF);
                    rgucWriteRegs[2] = (U8)(pstRegLists[uiCnt].usData & 0xFF);
                    rgucWriteRegs[3] = 0x00;
                }
                else if( pstRegLists[uiCnt].ucLen == 0xFF )
                {
                    rgucWriteRegs[0] = (U8)((pstRegLists[uiCnt].usRegs >> 8) & 0xFF);
                    rgucWriteRegs[1] = (U8)(pstRegLists[uiCnt].usRegs & 0xFF);
                    rgucWriteRegs[2] = (U8)(pstRegLists[uiCnt].usData & 0xFF);
                    rgucWriteRegs[3] = (U8)(pstRegLists[uiCnt].usData >> 8 & 0xFF);
                }
                else
                {
                    CamacqTraceErr("Unexpected value!!");
                    return iRet;
                }
            }
            
        }
#endif
    }
#if (CAMACQ_SENSOR_MAX==2)
    else if( iResType == CAMACQ_SENSOR_SUB )
    {
#if (CAMACQ_SUB_FS_MODE)
        pcRegLists = CamacqExtReadFs( (const S8*)pvArg, CAMACQ_EXT_LEN_SONY, iResType );
        if( pcRegLists == NULL )
            return -1;
        {   
            iIdx = 0;
            ucAddrLen = pcRegLists[ iIdx++ ];
            while( ucAddrLen != 0xFF )
            {
                if( ucAddrLen == 0x02 )
                {
                    rgucWriteRegs[0] = pcRegLists[iIdx++];
                    rgucWriteRegs[1] = pcRegLists[iIdx++];
                    rgucWriteRegs[3] = pcRegLists[iIdx++];
                    rgucWriteRegs[2] = pcRegLists[iIdx++];
                }
                else if( ucAddrLen == 0x01 || ucAddrLen == 0x03 )
                {
                    rgucWriteRegs[0] = pcRegLists[iIdx++];
                    rgucWriteRegs[1] = pcRegLists[iIdx++];
                    rgucWriteRegs[2] = pcRegLists[iIdx++];
                    rgucWriteRegs[3] = 0x00;
                }
                else
                {
                    CamacqTraceErr("Unexpected value!!");
                    return iRet;   
                }

                // 

                if( ucAddrLen != 0x03 )
                {
                    iRet = CamacqExtWriteI2c( pClient, rgucWriteRegs, 2+ucAddrLen );
                    if(iRet < 0)
                    {
                        CamacqTraceErr( ":write failed" );
                        iRet = -1;
                        return iRet;
                    }
                
                    CamacqTraceDbg(":Length[%d] : 0x%02X%02X%02X%02X" , ucAddrLen, rgucWriteRegs[0] , 
                                                    rgucWriteRegs[1], rgucWriteRegs[2] , rgucWriteRegs[3]);
                }
                else
                {
                    CamacqExtDelay(rgucWriteRegs[2]);
                    CamacqTraceDbg(" setfile delay : %d", rgucWriteRegs[2]);   
                }

                ucAddrLen = pcRegLists[ iIdx++ ];
            }
        }
        CamacqFree( pcRegLists );    
#else /* CAMACQ_SUB_FS_MODE */ 
        pstRegLists = (_stSonyData*)pvArg;
        {
            if( pstRegLists[uiCnt].ucLen == 0x02 )
            {
                rgucWriteRegs[0] = (U8)((pstRegLists[uiCnt].usRegs >> 8) & 0xFF);
                rgucWriteRegs[1] = (U8)(pstRegLists[uiCnt].usRegs & 0xFF);
                rgucWriteRegs[2] = (U8)(pstRegLists[uiCnt].usData & 0xFF);
                rgucWriteRegs[3] = (U8)(pstRegLists[uiCnt].usData >> 8 & 0xFF);   
            }
            else if( pstRegLists[uiCnt].ucLen == 0x01 || pstRegLists[uiCnt].ucLen == 0x03 )
            {
                rgucWriteRegs[0] = (U8)((pstRegLists[uiCnt].usRegs >> 8) & 0xFF);
                rgucWriteRegs[1] = (U8)(pstRegLists[uiCnt].usRegs & 0xFF);
                rgucWriteRegs[2] = (U8)(pstRegLists[uiCnt].usData & 0xFF);
                rgucWriteRegs[3] = 0x00;
            }
            else if( pstRegLists[uiCnt].ucLen == 0xFF )
            {
                rgucWriteRegs[0] = (U8)((pstRegLists[uiCnt].usRegs >> 8) & 0xFF);
                rgucWriteRegs[1] = (U8)(pstRegLists[uiCnt].usRegs & 0xFF);
                rgucWriteRegs[2] = (U8)(pstRegLists[uiCnt].usData & 0xFF);
                rgucWriteRegs[3] = (U8)(pstRegLists[uiCnt].usData >> 8 & 0xFF);
            }
            else
            {
                CamacqTraceErr("Unexpected value!!");
                return iRet;
            }

            while( rgucWriteRegs[0] != 0xFF || rgucWriteRegs[1] != 0xFF || rgucWriteRegs[2] != 0xFF )
            {
                if( pstRegLists[uiCnt].ucLen != 0x03 )
                {
                    iRet = CamacqExtWriteI2c( pClient, rgucWriteRegs, 2+pstRegLists[uiCnt].ucLen );
                    if(iRet < 0)
                    {
                        CamacqTraceErr( ":write failed =============== break!!!!!!!!!!!!!!" );
                        iRet = -1;
                        return iRet;
                    }
            
                    CamacqTraceDbg(":Length[%d] : 0x%02X%02X%02X%02X" , pstRegLists[uiCnt].ucLen, rgucWriteRegs[0] , 
                                               rgucWriteRegs[1], rgucWriteRegs[2] , rgucWriteRegs[3]);
                }
                else // 0x03 is delay
                {
                    CamacqExtDelay(rgucWriteRegs[2]);
                    CamacqTraceDbg(" setfile delay : %d", rgucWriteRegs[2]);
                }        
        
                uiCnt++;
        
                if( pstRegLists[uiCnt].ucLen == 0x02 )
                {
                    rgucWriteRegs[0] = (U8)((pstRegLists[uiCnt].usRegs >> 8) & 0xFF);
                    rgucWriteRegs[1] = (U8)(pstRegLists[uiCnt].usRegs & 0xFF);

                    rgucWriteRegs[2] = (U8)(pstRegLists[uiCnt].usData & 0xFF);
                    rgucWriteRegs[3] = (U8)(pstRegLists[uiCnt].usData >> 8 & 0xFF);   
                }
                else if(pstRegLists[uiCnt].ucLen == 0x01 || pstRegLists[uiCnt].ucLen == 0x03)
                {
                    rgucWriteRegs[0] = (U8)((pstRegLists[uiCnt].usRegs >> 8) & 0xFF);
                    rgucWriteRegs[1] = (U8)(pstRegLists[uiCnt].usRegs & 0xFF);
                    rgucWriteRegs[2] = (U8)(pstRegLists[uiCnt].usData & 0xFF);
                    rgucWriteRegs[3] = 0x00;
                }
                else if( pstRegLists[uiCnt].ucLen == 0xFF )
                {
                    rgucWriteRegs[0] = (U8)((pstRegLists[uiCnt].usRegs >> 8) & 0xFF);
                    rgucWriteRegs[1] = (U8)(pstRegLists[uiCnt].usRegs & 0xFF);
                    rgucWriteRegs[2] = (U8)(pstRegLists[uiCnt].usData & 0xFF);
                    rgucWriteRegs[3] = (U8)(pstRegLists[uiCnt].usData >> 8 & 0xFF);
                }
                else
                {
                    CamacqTraceErr("Unexpected value!!");
                    return iRet;
                }
            }
            
        }
#endif /* CAMACQ_SUB_FS_MODE */
    }
#endif /* CAMACQ_SENSOR_MAX==2 */
   
    CamacqTraceDbg_v("OUT");
    return iRet;
}


S32 WaitForModeTransition_Sony( struct i2c_client *pClient, int iMode )
{
    S32 iRet = 0;
    CamacqTraceIN();
    CamacqTraceDbg(" : iMode=%d", iMode);

    if( 0 == iMode )    // Wait for Mode Transition (OM)
    {
        
    }
    else if( 1 == iMode )   // Wait for Mode Transition (CM)
    {
        int     iRoofOutCnt = 50;
        U8      ucValue = 0;
        U16     usReadAddr = 0x000E;    //INTSTS        
        U8      rgucReadData[2] = {0, };
        U8      rgucWriteRegs[3] = {0, };   //INCLR

        do {
            CamacqExtDelay(10);

            iRet = CamacqExtReadI2c( pClient, usReadAddr, 2, rgucReadData, 2 );
            if( iRet < 0  )
            {
                CamacqTraceErr(" : error iRet=%d", iRet);
                return iRet;
            }
            iRet = 0;
                        
            CamacqTraceDbg( " 111111 INTSTS=0x%x, 0x%x", rgucReadData[1], rgucReadData[0] );    
            
            ucValue = ((rgucReadData[0] & 0x02) >> 1); // bit 1
            CamacqTraceDbg( " : 111111 ucValue=0x%x", ucValue);
            iRoofOutCnt--;
        }while( (ucValue != 1) && iRoofOutCnt );
        CamacqTraceDbg( " : 111111 iRoofOutCnt=%d", iRoofOutCnt);

        iRoofOutCnt = 50;
        do {
            rgucWriteRegs[0] = 0x00;
            rgucWriteRegs[1] = 0x12;
            rgucWriteRegs[2] = 0x02;
            iRet = CamacqExtWriteI2c( pClient, rgucWriteRegs, 3 ); // 0x02 write to INCLR
            if( iRet < 0  )
            {
                CamacqTraceErr(" : error iRet=%d", iRet);
                return iRet;
            }
            iRet = 0;
            usReadAddr = 0x000E;    //INTSTS        
            rgucReadData[0] = 0, rgucReadData[1] = 0;

            CamacqExtDelay(1);
            CamacqExtReadI2c( pClient, usReadAddr, 2, rgucReadData, 2 );
            
            CamacqTraceDbg( " 22222 INTSTS=0x%x, 0x%x", rgucReadData[1], rgucReadData[0] );    
           
            ucValue = ((rgucReadData[0] & 0x02) >> 1); // bit 1
            CamacqTraceDbg( " : 22222 ucValue=0x%x", ucValue);
            iRoofOutCnt--;
        }while( (ucValue != 0) && iRoofOutCnt );

        CamacqTraceDbg( " : 222222 iRoofOutCnt=%d", iRoofOutCnt);
        
    }
    else if( 2 == iMode ) // Wait for JPEG_UPDATE
    {    
        int     iRoofOutCnt = 50;
        U8      ucValue = 0;
        U16     usReadAddr = 0x000E;    //INTSTS        
        U8      rgucReadData[2] = {0, };
        U8      rgucWriteRegs[3] = {0, };   //INCLR

        do {
            CamacqExtDelay(10);
            
            iRet = CamacqExtReadI2c( pClient, usReadAddr, 2, rgucReadData, 2 );
            if( iRet < 0  )
            {
                CamacqTraceErr(" : error iRet=%d", iRet);
                return iRet;
            }
            iRet = 0;
                        
            CamacqTraceDbg( " 33333 INTSTS=0x%x, 0x%x", rgucReadData[1], rgucReadData[0] );    
            
            ucValue = ((rgucReadData[0] & 0x04) >> 2); // bit 2
            CamacqTraceDbg( " : 33333 ucValue=0x%x", ucValue);
            iRoofOutCnt--;
        }while( (ucValue != 1) && iRoofOutCnt );

        CamacqTraceDbg( " : 33333 iRoofOutCnt=%d", iRoofOutCnt);

        iRoofOutCnt = 50;
        do {
            rgucWriteRegs[0] = 0x00;
            rgucWriteRegs[1] = 0x12;
            rgucWriteRegs[2] = 0x04;
            iRet = CamacqExtWriteI2c( pClient, rgucWriteRegs, 3 ); // 0x02 write to INCLR
            if( iRet < 0  )
            {
                CamacqTraceErr(" : error iRet=%d", iRet);
                return iRet;
            }
            iRet = 0;

            usReadAddr = 0x000E;    //INTSTS        
            rgucReadData[0] = 0, rgucReadData[1] = 0;

            CamacqExtDelay(1);
            CamacqExtReadI2c( pClient, usReadAddr, 2, rgucReadData, 2 );
            
            CamacqTraceDbg( " 44444 INTSTS=0x%x, 0x%x", rgucReadData[1], rgucReadData[0] );    
           
            ucValue = ((rgucReadData[0] & 0x04) >> 2); // bit 2
            CamacqTraceDbg( " : 44444 ucValue=0x%x", ucValue);
            iRoofOutCnt--;
        }while( (ucValue != 0) && iRoofOutCnt );

        CamacqTraceDbg( " : 44444 iRoofOutCnt=%d", iRoofOutCnt);
    }
    
    CamacqTraceOUT();
    return iRet;
}

S32 Callibration_Sony(  struct i2c_client *pClient, int iResType )
{
    S32 iRet = 0;

    U8  rgucOTPDatas[15] = {0, };
    U16 usReadAddr = 0, usStartReadAddr, usEndReadAddr, usValue = 0, usTemp1, usTemp2;        
    U8  ucReadData = 0;
    U8  rgucWriteRegs[3] = {0, };
    S32 iIndex = 0;
    const void* pvCallRegs = 0;

    CamacqTraceIN();

    // Read OTP1(0x004F)
    usReadAddr = 0x004F;
    iRet = CamacqExtReadI2c( pClient, usReadAddr, 2, &ucReadData, 1);
    if( iRet < 0 )
    {
        CamacqTraceErr("i2c read fail, usReadAddr=0x%x", usReadAddr);
        return iRet;
    }
    iRet = 0;
    CamacqTraceDbg(" : Read OTP1(0x%x) = 0x%x", usReadAddr, ucReadData );

    // take OTP_V[bit4]
    usValue = ((ucReadData & 0x10) >> 4);
    CamacqTraceDbg(" : usValue = 0x%x", usValue);
    
    if( usValue == 1 )
    {
        usStartReadAddr = 0x004F;
        usEndReadAddr = 0x005D;
    }
    else
    {
        // Read OTP0(0x0040)
        usReadAddr = 0x0040;
        CamacqExtReadI2c( pClient, usReadAddr, 2, &ucReadData, 1);
        if( iRet < 0 )
        {
            CamacqTraceErr("i2c read fail, usReadAddr=0x%x", usReadAddr);
            return iRet;
        }
        iRet = 0;
        CamacqTraceDbg(" : Read OTP0(0x%x) = 0x%x", usReadAddr, ucReadData );

        // take OTP_V[bit4]
        usValue = ((ucReadData & 0x10) >> 4);
        CamacqTraceDbg(" : usValue = 0x%x", usValue);
        if( usValue == 1 )
        {
            usStartReadAddr = 0x0040;
            usEndReadAddr = 0x004E;
        }
        else
        {
            usStartReadAddr = 0x0;
        }
    }

    // READ OTP MAP
    if( usStartReadAddr == 0 )
    {
        pvCallRegs = CAMACQ_MAIN_REG_CALLIBRATION_SHADING_TABLE_NOCAL;

        iRet = CamacqExtWriteI2cLists_Sony( pClient, pvCallRegs, iResType );
        if( iRet < 0 )
        {
            CamacqTraceErr(" : CamacqExtWriteI2cLists_Sony");
            return iRet;
        }
        iRet = 0;
    }
    else
    {
        iIndex = 0;
        for( usReadAddr = usStartReadAddr; usReadAddr <= usEndReadAddr; usReadAddr++ )
        {
            CamacqExtReadI2c( pClient, usReadAddr, 2, &(rgucOTPDatas[iIndex]), 1);
            CamacqTraceDbg(" : Read rgucOTPDatas[%d] = 0x%x", iIndex, rgucOTPDatas[iIndex] );     
            iIndex++;
        }
        
        // Read Shading Table [113:110]
        usTemp1 = 0; usTemp2 = 0; usValue = 0;
        usTemp1 = (rgucOTPDatas[14] & 0x03); // [113:112]
        usTemp2 = ((rgucOTPDatas[13] & 0xC0) >> 6); // [111:110]
        usValue = ((usTemp1 << 2) | usTemp2);
        CamacqTraceDbg(" :Read Shading Table[113:110], usValue=0x%x, usTemp1=0x%x, usTemp2=0x%x", 
                                            usValue, usTemp1, usTemp2);

        // Write ShadingTable
        if( usValue == 0 )
        {
            pvCallRegs = CAMACQ_MAIN_REG_CALLIBRATION_SHADING_TABLE_0;
        }
        else if( usValue == 1 )
        {
            pvCallRegs = CAMACQ_MAIN_REG_CALLIBRATION_SHADING_TABLE_1;
        }
        else if( usValue == 2 )
        {
            pvCallRegs = CAMACQ_MAIN_REG_CALLIBRATION_SHADING_TABLE_2;
        }
        else
        {
            CamacqTraceErr(" invaild value = %d", usValue );
            return -1;
        }

        iRet = CamacqExtWriteI2cLists_Sony( pClient, pvCallRegs, iResType );
        if( iRet < 0 )
        {
            CamacqTraceErr("CamacqExtWriteI2cLists_Sony");
            return iRet;
        }

        // Write NorR(0x6804) = OTP Data[53:40]
        usTemp1 = 0; usTemp2 = 0; usValue = 0;
        usTemp1 = (rgucOTPDatas[6] & 0x3F); // [53:48]
        usTemp2 = (rgucOTPDatas[5] & 0xFF); // [47:40]
        usValue = ((usTemp1 << 8) | usTemp2);
        CamacqTraceDbg(" :OTP Data[53:40], usValue=0x%x, usTemp1=0x%x, usTemp2=0x%x", usValue, usTemp1, usTemp2);

        rgucWriteRegs[0] = 0x68;
        rgucWriteRegs[1] = 0x04;
        rgucWriteRegs[2] = (usValue & 0xFF);
        iRet = CamacqExtWriteI2c(pClient, rgucWriteRegs, 3);
        if( iRet < 0 )
        {
            CamacqTraceErr(" : 111 i2c write fail");
            return iRet;
        }

        rgucWriteRegs[1] = 0x05;
        rgucWriteRegs[2] = ((usValue >> 8) & 0xFF);
        iRet = CamacqExtWriteI2c(pClient, rgucWriteRegs, 3);
        if( iRet < 0 )
        {
            CamacqTraceErr(" : 222 i2c write fail");
            return iRet;
        }

        // Write NorB(0x6806) = OTP Data[69:56]
        usTemp1 = 0; usTemp2 = 0; usValue = 0;
        usTemp1 = (rgucOTPDatas[8] & 0x3F); // [69:64]
        usTemp2 = (rgucOTPDatas[7] & 0xFF); // [63:56]
        usValue = ((usTemp1 << 8) | usTemp2);
        CamacqTraceDbg(" :OTP Data[69:56], usValue=0x%x, usTemp1=0x%x, usTemp2=0x%x", usValue, usTemp1, usTemp2);

        rgucWriteRegs[0] = 0x68;
        rgucWriteRegs[1] = 0x06;
        rgucWriteRegs[2] = (usValue & 0xFF);
        iRet = CamacqExtWriteI2c(pClient, rgucWriteRegs, 3);
        if( iRet < 0 )
        {
            CamacqTraceErr(" : 333 i2c write fail");
            return iRet;
        }

        rgucWriteRegs[1] = 0x07;
        rgucWriteRegs[2] = ((usValue >> 8) & 0xFF);
        iRet = CamacqExtWriteI2c(pClient, rgucWriteRegs, 3);
        if( iRet < 0 )
        {
            CamacqTraceErr(" : 444 i2c write fail");
            return iRet;
        }

        // Write PreR(0x6808) = OTP Data[99:90]
        usTemp1 = 0; usTemp2 = 0; usValue = 0;
        usTemp1 = (rgucOTPDatas[12] & 0x0F); // [99:96]
        usTemp2 = ((rgucOTPDatas[11] & 0xFC) >> 2); // [95:90]
        usValue = ((usTemp1 << 6) | usTemp2);
        CamacqTraceDbg(" :OTP Data[99:90], usValue=0x%x, usTemp1=0x%x, usTemp2=0x%x", usValue, usTemp1, usTemp2);

        rgucWriteRegs[0] = 0x68;
        rgucWriteRegs[1] = 0x08;
        rgucWriteRegs[2] = (usValue & 0xFF);
        iRet = CamacqExtWriteI2c(pClient, rgucWriteRegs, 3);
        if( iRet < 0 )
        {
            CamacqTraceErr(" : 666 i2c write fail");
            return iRet;
        }

        rgucWriteRegs[1] = 0x09;
        rgucWriteRegs[2] = ((usValue >> 8) & 0xFF);
        iRet = CamacqExtWriteI2c(pClient, rgucWriteRegs, 3);
        if( iRet < 0 )
        {
            CamacqTraceErr(" : 777 i2c write fail");
            return iRet;
        }

        // Write PreB(0x680A) = OTP Data[109:100]
        usTemp1 = 0; usTemp2 = 0; usValue = 0;
        usTemp1 = (rgucOTPDatas[13] & 0x3F); // [109:104]
        usTemp2 = ((rgucOTPDatas[12] & 0xF0) >> 4); // [103:100]
        usValue = ((usTemp1 << 4) | usTemp2);
        CamacqTraceDbg(" :OTP Data[53:40], usValue=0x%x, usTemp1=0x%x, usTemp2=0x%x", usValue, usTemp1, usTemp2);

        rgucWriteRegs[0] = 0x68;
        rgucWriteRegs[1] = 0x0A;
        rgucWriteRegs[2] = (usValue & 0xFF);
        iRet = CamacqExtWriteI2c(pClient, rgucWriteRegs, 3);
        if( iRet < 0 )
        {
            CamacqTraceErr(" : 888 i2c write fail");
            return iRet;
        }

        rgucWriteRegs[1] = 0x0B;
        rgucWriteRegs[2] = ((usValue >> 8) & 0xFF);
        iRet = CamacqExtWriteI2c(pClient, rgucWriteRegs, 3);
        if( iRet < 0 )
        {
            CamacqTraceErr(" : 999 i2c write fail");
            return iRet;
        }
                
    }

    CamacqTraceOUT();
    return iRet;
}

#define	ERRSCL_AUTO		    0x01CA
#define	USER_AESCL_AUTO		0x01CE
#define	ERRSCL_NOW		    0x01CC
#define	USER_AESCL_NOW		0x01D0

#define	CAP_GAINOFFSET		0x0186

#define	AE_OFSETVAL		    0x1B59
#define	AE_MAXDIFF		    0x1388


U16 aeoffset_table[] = {
0,
50, 
96, 
141, 
185, 
227, 
268, 
307, 
346, 
382, 
418, 
453, 
487, 
519, 
551, 
582, 
612, 
641, 
670, 
698, 
725, 
751, 
777, 
802, 
827, 
851, 
874, 
897, 
919, 
941, 
963, 
984, 
1004, 
1025, 
1044, 
1064, 
1083, 
1102, 
1120, 
1138, 
1155, 
1173, 
1190, 
1206, 
1223, 
1239, 
1255, 
1270, 
1285, 
1300, 
1315, 
1330, 
1344, 
1358, 
1372, 
1385, 
1399, 
1412, 
1425, 
1438, 
1450, 
1463, 
1475, 
1487, 
1499, 
1510, 
1522, 
1533, 
1544, 
1555, 
1566, 
1577, 
1588, 
1598, 
1608, 
1618, 
1628, 
1638, 
1648, 
1658, 
1667, 
1676, 
1686, 
1695, 
1704, 
1713, 
1721, 
1730, 
1739, 
1747, 
1755, 
1764, 
1772, 
1780, 
1788, 
1796, 
1803, 
1811, 
1818, 
1826, 
1833, 
1841, 
1848, 
1855, 
1862, 
1869, 
1876, 
1883, 
1889, 
1896, 
1903, 
1909, 
1916, 
1922, 
1928, 
1935, 
1941, 
1947, 
1953, 
1959, 
1965, 
1971, 
1976, 
1982, 
1988, 
1993, 
1999, 
2004, 
2010, 
2015, 
2020, 
2026, 
2031, 
2036, 
2041, 
2046, 
2051, 
2056, 
2061, 
2066, 
2071, 
2075, 
2080, 
2085, 
2089, 
2094, 
2098, 
2103, 
2107, 
2112, 
2116, 
2120, 
2125, 
2129, 
2133, 
2137, 
2141, 
2145, 
2149, 
2153, 
2157, 
2161, 
2165, 
2169, 
2173, 
2177, 
2180, 
2184, 
2188, 
2191, 
2195, 
2198, 
2202, 
2205, 
2209, 
2212, 
2216, 
2219, 
2222, 
2226, 
2229, 
2232, 
2236, 
2239, 
2242, 
2245, 
2248, 
2251, 
2254, 
2257, 
2260, 
2263, 
2266, 
2269, 
2272, 
2275, 
2278, 
2281, 
2283, 
2286, 
2289, 
2292, 
2294, 
2297, 
2300, 
2302, 
2305, 
2308, 
2310, 
2313, 
2315, 
2318, 
2320, 
2323, 
2325, 
2328, 
2330, 
2332, 
2335, 
2337, 
2339, 
2342, 
2344, 
2346, 
2349, 
2351, 
2353, 
2355, 
2357, 
2359, 
2362, 
2364, 
2366, 
2368, 
2370, 
2372, 
2374, 
2376, 
2378, 
2380, 
2382, 
2384, 
2386, 
2388, 
2390, 
2392, 
2394, 
2395, 
2397, 
2399, 
2401, 
2403, 
2405, 
2406, 
2408, 
2410, 
2412, 
2413, 
2415, 
2417, 
2418, 
2420, 
2422, 
2423, 
2425, 
2427, 
2428, 
2430, 
2431, 
2433, 
2434, 
2436, 
2438, 
2439, 
2441, 
2442, 
2444, 
2445, 
2446, 
2448, 
2449, 
2451, 
2452, 
2454, 
2455, 
2456, 
2458, 
2459, 
2460, 
2462, 
2463, 
2464, 
2466, 
2467, 
2468, 
2470, 
2471, 
2472, 
2473, 
2475, 
2476, 
2477, 
2478, 
2480, 
2481, 
2482, 
2483, 
2484, 
2485, 
2487, 
2488, 
2489, 
2490, 
2491, 
2492, 
2493, 
2494, 
2495, 
2497, 
2498, 
2499, 
2500, 
2501, 
2502, 
2503, 
2504, 
2505, 
2506, 
2507, 
2508, 
2509, 
2510, 
2511, 
2512, 
2513, 
2514, 
2515, 
2516, 
2516, 
2517, 
2518, 
2519, 
2520, 
2521, 
2522, 
2523, 
2524, 
2525, 
2525, 
2526, 
2527, 
2528, 
2529, 
2530, 
2530, 
2531, 
2532, 
2533, 
2534, 
2534, 
2535, 
2536, 
2537, 
2538, 
2538, 
2539, 
2540, 
2541, 
2541, 
2542, 
2543, 
2544, 
2544, 
2545, 
2546, 
2546, 
2547, 
2548, 
2548, 
2549, 
2550, 
2551, 
2551, 
2552, 
2553, 
2553, 
2554, 
2555, 
2555, 
2556, 
2556, 
2557, 
2558, 
2558, 
2559, 
2560, 
2560, 
2561, 
2561, 
2562, 
2563, 
2563, 
2564, 
2564, 
2565, 
2565, 
2566, 
2567, 
2567, 
2568, 
2568, 
2569, 
2569, 
2570, 
2570, 
2571, 
2571, 
2572, 
2572, 
2573, 
2573, 
2574, 
2575, 
2575, 
2576, 
2576, 
2576, 
2577, 
2577, 
2578, 
2578, 
2579, 
2579, 
2580, 
2580, 
2581, 
2581, 
2582, 
2582, 
2583, 
2583, 
2583, 
2584, 
2584, 
2585, 
2585, 
2586, 
2586, 
2586, 
2587, 
2587, 
2588, 
2588, 
2589, 
2589, 
2589, 
2590, 
2590, 
2590, 
2591, 
2591, 
2592, 
2592, 
2592, 
2593, 
2593, 
2594, 
2594, 
2594, 
2595, 
2595, 
2595, 
2596, 
2596, 
2596, 
2597, 
2597, 
2597, 
2598, 
2598, 
2598, 
2599, 
2599, 
2599, 
2600, 
2600, 
2600, 
2601, 
2601, 
2601, 
2602, 
2602, 
2602, 
2603, 
2603, 
2603, 
2603, 
2604, 
2604, 
2604

};


////////////////////////////////////////////////////
//// calculate the AE gain offset Sample source ////
////////////////////////////////////////////////////
#define AE_TABLE_MAX 501
U16 calculate_AEgain_offset(U16 ae_auto, U16 ae_now, U16 ersc_auto, U16 ersc_now, U32 uiResType)
{
	U16	aediff, aeoffset;
    U16 ae_maxdiff = 0, ae_ofsetval = 0;
    U16* pAeOffset_Table = 0;

    // const U16 aeoffset_table[] = 
#if CAMACQ_MAIN_FS_MODE
    U8* rgucBuf = 0;
    U32 i = 0;
    U16 rgucAeoffsetTable[AE_TABLE_MAX] = {0, };
    U16 usTemp = 0;
    U16 usValue = 0;

    // AE_MAXDIFF
    rgucBuf = CamacqExtReadFs( "ae_maxdiff", CAMACQ_EXT_LEN_2BYTE_INT, uiResType );
    if( rgucBuf )
    {
        usTemp = *((U16*)(rgucBuf));
        
        ae_maxdiff = usTemp >> 8;
        ae_maxdiff |=  usTemp << 8;
        CamacqTraceDbg(" ae_maxdiff=0x%x", ae_maxdiff );

        CamacqFree( rgucBuf );
    }
    else
    {
        CamacqTraceErr("ae_maxdiff, rgucBuf=0x%x", rgucBuf );
        ae_maxdiff = AE_MAXDIFF;
    }

    // AE_OFSETVAL
    rgucBuf = CamacqExtReadFs( "ae_ofsetval", CAMACQ_EXT_LEN_2BYTE_INT, uiResType );
    if( rgucBuf )
    {
        usTemp = *((U16*)(rgucBuf));
        
        ae_ofsetval = usTemp >> 8;
        ae_ofsetval |=  usTemp << 8;
        CamacqTraceDbg(" ae_ofsetval=0x%x", ae_ofsetval );

        CamacqFree( rgucBuf );
    }
    else
    {
        CamacqTraceErr("ae_ofsetval, rgucBuf=0x%x", rgucBuf );
        ae_ofsetval = AE_OFSETVAL;
    }

    // AEOFFSET_TABLE
    rgucBuf = CamacqExtReadFs( "aeoffset_table", CAMACQ_EXT_LEN_2BYTE_INT, uiResType );
    if( rgucBuf )
    {
        for( i = 0; i < AE_TABLE_MAX; i++ )
        {
            usTemp = *( ((U16*)(rgucBuf)) + i );
            usValue = usTemp >> 8;
            usValue |=  usTemp << 8;

            rgucAeoffsetTable[ i ] = usValue;
        }

        pAeOffset_Table = rgucAeoffsetTable;

        // temp debuf
        for( i = 0; i < AE_TABLE_MAX; i++ )
        {
            CamacqTraceErr( "pAeOffset_Table[%d]=0x%x", i, pAeOffset_Table[i]);
        }

        CamacqFree( rgucBuf );
    }
    else
    {
        CamacqTraceErr("aeoffset_table, rgucBuf=0x%x", rgucBuf );
        pAeOffset_Table = aeoffset_table;
    }   
#else
    ae_maxdiff = AE_MAXDIFF;
    ae_ofsetval = AE_OFSETVAL;
    pAeOffset_Table = aeoffset_table;
#endif
    

	//	AE_Gain_Offset = Target - ERRSCL_NOW	
	aediff = (ae_now + ersc_now) - (ae_auto + ersc_auto);
	if (aediff  < 0)	{
			aediff = 0;
	}
	if ( ersc_now < 0 ) {
		if ( aediff >= AE_MAXDIFF )
			aeoffset = -AE_OFSETVAL - ersc_now;
		else
			aeoffset = -pAeOffset_Table[aediff/10] - ersc_now;
	}
	else {
		if ( aediff >= AE_MAXDIFF )
			aeoffset = -AE_OFSETVAL;
		else
			aeoffset = -pAeOffset_Table[aediff/10];
	}

	//	Set AE Gain offset
	// ret = write_reg(CAP_GAINOFFSET, sizeof(uint16), &aeoffset);

    return aeoffset;
}
#endif /* __ISX006_SONY__ */

/*
        enum _CamacqLuxValue
        {
            CAMACQ_LUX_NORMAL = 0,
            CAMACQ_LUX_LOW,
            CAMACQ_LUX_HIGH,
        };
*/
S32 CamacqExtCheckLux( struct stCamacqExtAPIs_t* this )
{
    S32 iRet = -1;
    U8* szName =  this->m_pstSensor->m_szName;
    struct i2c_client *pClient = this->m_pstSensor->m_pI2cClient;
    CamacqTraceIN();
    CamacqTraceDbg(" : szName=%s ", szName );

    if( !strcmp( szName,  "s5k5ccgx") )
    {
        U8	rgucWriteRegs_0[4] = {0xFC,0xFC,0xD0,0x00};
        U8	rgucWriteRegs_1[4] = {0x00,0x2C,0x70,0x00};
        U8	rgucWriteRegs_2[4] = {0x00,0x2E,0x2A,0x3C};
        U16 usReadData = 0x0;
        U32 uiNB = 0x0;     //NB is JODO
        U16 usReadKey = 0x0F12;
        
        CamacqExtWriteI2c( pClient, rgucWriteRegs_0, 4 );
        CamacqExtWriteI2c( pClient, rgucWriteRegs_1, 4 );
        CamacqExtWriteI2c( pClient, rgucWriteRegs_2, 4 );
        CamacqExtReadI2c( pClient, usReadKey, 2, &usReadData, 2 );
        CamacqTraceDbg("0x7000.2A3C usReadData : %x", usReadData);
        uiNB = ((usReadData & 0x00FF) << 8) |((usReadData & 0xFF00) >> 8);


        CamacqExtReadI2c( pClient, usReadKey, 2, &usReadData, 2 );
        CamacqTraceDbg("0x7000.2A3E usReadData : %x", usReadData);
        uiNB |= ( ( ((usReadData & 0x00FF) << 8) |((usReadData & 0xFF00) >> 8) )<< 16 );

        CamacqTraceDbg("uiNB : %x", uiNB);

        if( uiNB > 0xFFFE )
        {
            iRet = 2; // High Lux
        }
        else if( uiNB < 0x20 )
        {
            iRet = 1;   // Low Lux
        }
        else
        {
            iRet = 0; // Normal Lux
        }
        CamacqTraceDbg("Lux value iRet : %x", iRet);
    }
    else if( !strcmp( szName,  "isx012") )
    {
        U16 usReadAddr = 0x01A5;        // USER_GAIN_NOW        
        U8  ucReadData = 0;
        U16 usCheckLux = 0;
        U8* rgucBuf = 0;

        CamacqExtReadI2c( pClient, usReadAddr, 2, &ucReadData, 1 );
        CamacqTraceDbg( " : ucReadData=%d", ucReadData );

#if CAMACQ_MAIN_FS_MODE
        rgucBuf = CamacqExtReadFs( "checklux", CAMACQ_EXT_LEN_2BYTE_INT, this->m_pstSensor->m_uiResType );
        if( rgucBuf )
        {
            U16 usTemp = *((U16*)(rgucBuf));
            
            usCheckLux = usTemp >> 8;
            usCheckLux |=  usTemp << 8;
            CamacqTraceDbg(" usCheckLux=0x%x", usCheckLux );

            CamacqFree( rgucBuf );
        }
        else
        {
            CamacqTraceErr(" rgucBuf=0x%x", rgucBuf );
            usCheckLux = 0x0015;
        }
#else
        usCheckLux = 0x0014;
#endif
        
        if( ucReadData >= usCheckLux )
        {
            iRet = 1; // Low Lux
        }
        else
        {
            iRet = 0; // Normal Lux
        }
    }

    CamacqTraceOUT();
    return iRet;
}

void CamacqExtDelay( S32 iDelay )
{
    mdelay( iDelay );
    CamacqTraceDbg( ": iDelay [%d]", iDelay );
}

S32 CamacqExtRequestGpio( S32 iGpio, U8* szModuleName )
{
    S32 iRet = 0;
    CamacqTraceIN();

    iRet = gpio_request( iGpio, szModuleName );
    if( iRet < 0 ) 
    {
        CamacqTraceErr( " :invalid gpio mode, iRet = [%d] ", iRet );
        iRet = -EINVAL;
        goto CAMACQ_EXT_OUT;
    }

CAMACQ_EXT_OUT:
    CamacqTraceOUT();
    return iRet;
}

void CamacqExtFreeGpio( S32 iGpio )
{
    CamacqTraceIN();
    gpio_free(iGpio);
    CamacqTraceOUT();
}

S32 CamacqExtSetGpioConfig( S32 iGpio, S32 iMux, S32 iDirection, S32 iLevel )
{
    S32 iRet = 0;
    CamacqTraceIN();

    gpio_set_value( iGpio, iMux );

    iRet = gpio_direction_output( iGpio, iLevel );
    if(iRet < 0)
    {
        CamacqTraceErr( ":invalid gpio_direction_output" );
        iRet = -EINVAL;
        goto CAMACQ_EXT_OUT;
    }

CAMACQ_EXT_OUT:
    CamacqTraceOUT();
    return iRet;
}

/* File System API */
/* Must need to release return point value by caller side */
U8* CamacqExtReadFs( const S8 * szFileName, _eCamacqSelLen eLen, int iResType )
{
    U8* pucRet;
    S8 rgcPath[ CAMACQ_EXT_MAX_PATH+1 ], *pcHex;
    U8 *pucBuff, *pucCurBuffPos, *pucMovBuffPos;
    U32 uiLen, uiIdx, uiCnt = 0;
    U8 ucIdy, ucHexLen;
    U8 ucSonyFlag = 0;  // 1 is addr, 2 is data, 3 is data legth
    U8 rgucSonyAddr[2] = {0, };
    U8 ucSonyLen = 0;
    U8 szSonyData[ 7 ] = {0, }; // "0xFFFF"

#if defined(_LINUX_)
    loff_t stpos = 0;
    struct file *fd;
#else
    S32 fd;
#endif /* _LINUX_ */

#if defined(_LINUX_)
    mm_segment_t fs = get_fs();
    set_fs(get_ds());
#endif /* _LINUX_ */

    CamacqTraceDbg( ":eLen[%d]", eLen );
    switch( eLen )
    {
        case CAMACQ_EXT_LEN_2BYTE_ARY:
        case CAMACQ_EXT_LEN_4BYTE_ARY:
            ucHexLen = strlen( CAMACQ_EXT_HEX_CHAR );
        break;
        case CAMACQ_EXT_LEN_2BYTE_INT:
            ucHexLen = strlen( CAMACQ_EXT_HEX_SHORT );
        break;        
        case CAMACQ_EXT_LEN_4BYTE_INT:
            ucHexLen = strlen( CAMACQ_EXT_HEX_INT );
        break;
        case CAMACQ_EXT_LEN_SONY:
            ucHexLen = strlen( CAMACQ_EXT_HEX_SHORT );
        break;
        default:
            ucHexLen = strlen( CAMACQ_EXT_HEX_INT );
        break;
    }
    CamacqTraceDbg( " :ucHexLen[%d] ", ucHexLen );
    
    CamacqTraceDbg( " : szFileName[%s] ", szFileName );
    memset( rgcPath, 0x00, sizeof(S8)*(CAMACQ_EXT_MAX_PATH+1) );
    if( iResType == CAMACQ_SENSOR_MAIN )
        sprintf( rgcPath, CAMACQ_MAIN_PATH_SET_FILE, szFileName );
#if (CAMACQ_SENSOR_MAX==2)
    else if( iResType == CAMACQ_SENSOR_SUB )
        sprintf( rgcPath, CAMACQ_SUB_PATH_SET_FILE, szFileName );
#endif /* CAMACQ_SENSOR_MAX==2 */

    CamacqTraceDbg( " :rgcPath[%s] ", rgcPath );

    fd = CamacqOpen( rgcPath, CAMACQ_RDONLY );
    if( IS_ERR(fd) )
    {
        CamacqTraceErr( " :invalid fd[%d] ", (S32)fd );
        return NULL;
    }

    pcHex = (S8*)CamacqMalloc( sizeof(S8)*(ucHexLen+1) );
    memset( pcHex, 0x00, sizeof(S8)*(ucHexLen+1) );

#if defined(WIN32)
    camacq_seek( fd, 0, SEEK_END );
#endif /* _LINUX_ */

    uiLen = CamacqTell( fd );
    CamacqSeek( fd, 0, SEEK_SET );

    pucBuff = (U8*)CamacqMalloc( uiLen );
    pucRet = (U8*)CamacqMalloc( uiLen/2 ); 
    memset( pucBuff, 0x00, uiLen );
    memset( pucRet, 0x00, uiLen/2 );
    
    uiLen = CamacqRead( fd, pucBuff, uiLen );
    CamacqTraceDbg( ":uiLen[%d]", uiLen );

    ucSonyFlag = 1;
    for( uiIdx = 0; uiIdx < uiLen; uiIdx++ )
    {
        pucCurBuffPos = pucBuff + uiIdx;
        if( CAMACQ_EXT_COMMENT_START(pucCurBuffPos) ) /* Comment Start '/*'  or '//'  */
        {
            CamacqTraceDbg_v( "pucCurBuffPos[0x%0x]", pucCurBuffPos );
            pucMovBuffPos = (U8*)strstr( (S8*)pucCurBuffPos, CAMACQ_EXT_COMMENT_CLOSE);
            if(pucMovBuffPos != NULL)
            {
                uiIdx += (pucMovBuffPos - pucCurBuffPos);
             }	
	    else
	     {
                CamacqTraceDbg_v( "There is no */ : Old style" );
                CamacqTraceDbg_v( "pucCurBuffPos[0x%0x]", pucCurBuffPos );
                pucCurBuffPos = pucBuff + uiIdx;
                CamacqTraceDbg_v( "pucCurBuffPos[0x%0x]", pucCurBuffPos );
                pucMovBuffPos = (U8*)strstr( (S8*)pucCurBuffPos, CAMACQ_EXT_START);
                if(pucMovBuffPos != NULL)
                {
                    uiIdx += (pucMovBuffPos - pucCurBuffPos);
                 }	
		 else
                 {
                        CamacqTraceDbg( " !! Error uiIdx : %d", uiIdx );
                  }
	     }	
        }
        else if( CAMACQ_EXT_HEXA_START(pucCurBuffPos) ) /* HexValue Start */
        {
            memcpy( pcHex, pucCurBuffPos, ucHexLen );
            // CamacqTraceDbg_v( " : uiIdx[ %d ], pcHex[ %s ]", uiIdx, pcHex );
            if( eLen <= CAMACQ_EXT_LEN_4BYTE_ARY )
            {
                pucRet[ uiCnt++ ] = CamacqExtAsc2Hex( pcHex + CAMACQ_EXT_HEX_HALF_LEN );
                uiIdx += ucHexLen;
            }
            else if( eLen == CAMACQ_EXT_LEN_SONY )
            {
                // addr
                if( ucSonyFlag == 1 )
                {
                    for( ucIdy = 0; ucIdy < ( ucHexLen-CAMACQ_EXT_HEX_HALF_LEN ); ucIdy+=2 )
                    {
                        rgucSonyAddr[ ucIdy / 2 ] = CamacqExtAsc2Hex( pcHex + CAMACQ_EXT_HEX_HALF_LEN + ucIdy );
                        CamacqTraceDbg_v( " : rgucSonyAddr[ %d ] = %x ", ucIdy / 2, rgucSonyAddr[ ucIdy / 2 ] );
                    }
                    uiIdx += ucHexLen;
                    ucSonyFlag = 2;
                }
                // data
                else if( ucSonyFlag == 2 )
                {
                    memcpy( szSonyData, pcHex, ucHexLen );
                    CamacqTraceDbg_v( " szSonyData : %s", szSonyData );
                    uiIdx += CAMACQ_EXT_HEX_HALF_LEN;
                    ucSonyFlag = 3;
                }
                // length
                else if( ucSonyFlag == 3 )
                {
                    ucSonyLen = CamacqExtAsc2Hex( pcHex + CAMACQ_EXT_HEX_HALF_LEN );

                    CamacqTraceDbg_v( " ucSonyLen : %d", ucSonyLen );

                    // 1. save length
                    pucRet[ uiCnt++ ] = ucSonyLen;

                    // 2. save addr
                    memcpy( pucRet+uiCnt, rgucSonyAddr, sizeof(rgucSonyAddr) );
                    uiCnt += sizeof(rgucSonyAddr);

                    // 3. save data
                    if( (ucSonyLen == 1 ) ||( ucSonyLen == 3 ))
                    {
                        // pucRet[ uiCnt++ ] = 0x00;   // padding
                        pucRet[ uiCnt++ ] = CamacqExtAsc2Hex( szSonyData + CAMACQ_EXT_HEX_HALF_LEN );
                    }
                    else if( ucSonyLen == 2 )
                    {
                        for( ucIdy = 0; ucIdy < ucSonyLen; ucIdy++ )
                        {
                            pucRet[ uiCnt++ ] = CamacqExtAsc2Hex( szSonyData + CAMACQ_EXT_HEX_HALF_LEN + (ucIdy * 2) );
                        }
                    }
                    else if( ucSonyLen == 0xFF )
                    {
                        break;
                    }
                    else
                    {
                        CamacqTraceErr( " !! Error ucSonyLen : %d", ucSonyLen );
                    }
                    
                    uiIdx += CAMACQ_EXT_HEX_HALF_LEN;
                    ucSonyFlag = 1;
                }
            }
            else
            {
                for( ucIdy = 0; ucIdy < ( ucHexLen-CAMACQ_EXT_HEX_HALF_LEN ); ucIdy+=2 )
                {
                    pucRet[ uiCnt++ ] = CamacqExtAsc2Hex( pcHex + CAMACQ_EXT_HEX_HALF_LEN + ucIdy );
                }
                uiIdx += ucHexLen;
            }
        }
    }

    CamacqFree( pucBuff );
    CamacqFree( pcHex );
#if defined(_LINUX_)
    set_fs( fs );
#endif /* _LINUX_ */

    return pucRet;
}

U8  CamacqExtAsc2Hex( S8 *pcAscii )
{
    U8 ucRetHex = 0, ucCurHex = 0;
    U8 ucIdx;

    for( ucIdx = 0; ucIdx < CAMACQ_EXT_HEX_HALF_LEN; ucIdx++ )
    {
        if(pcAscii[ucIdx] >= CAMACQ_EXT_a && pcAscii[ucIdx] <= CAMACQ_EXT_f)
        {
            ucCurHex = (pcAscii[ucIdx]-CAMACQ_EXT_a+10)*(CAMACQ_EXT_HEX_WIDTH);
        }
        else if(pcAscii[ucIdx] >= CAMACQ_EXT_A && pcAscii[ucIdx] <= CAMACQ_EXT_F)
        {
            ucCurHex = (pcAscii[ucIdx]-CAMACQ_EXT_A+10)*(CAMACQ_EXT_HEX_WIDTH);
        }
        else if(pcAscii[ucIdx] >= CAMACQ_EXT_0 && pcAscii[ucIdx] <= CAMACQ_EXT_9)
        {
            ucCurHex = (pcAscii[ucIdx]-CAMACQ_EXT_0)*(CAMACQ_EXT_HEX_WIDTH);
        }

        if(ucIdx != 0)
            ucCurHex /= CAMACQ_EXT_HEX_WIDTH;

        ucRetHex += ucCurHex;
    }

    return ucRetHex;
}


//denis_exif
S32 CamacqExtCheckExposureTime( struct stCamacqExtAPIs_t* this )
{
    S32 iRet = -1;
    U8* szName =  this->m_pstSensor->m_szName;
    struct i2c_client *pClient = this->m_pstSensor->m_pI2cClient;
    CamacqTraceIN();

    if( !strcmp( szName, "s5k5ccgx") )
    {
        U8	rgucWriteRegs_0[4] = {0xFC,0xFC,0xD0,0x00};
        U8	rgucWriteRegs_1[4] = {0x00,0x2C,0x70,0x00};
        U8	rgucWriteRegs_2[4] = {0x00,0x2E,0x2A,0x14};
        U16 usReadData_lsb = 0x0;
        U16 usReadData_msb = 0x0;    
        U32 uiNB = 0x0;    
        U16 usReadKey = 0x0F12;
        
        CamacqExtWriteI2c( pClient, rgucWriteRegs_0, 4 );
        CamacqExtWriteI2c( pClient, rgucWriteRegs_1, 4 );
        CamacqExtWriteI2c( pClient, rgucWriteRegs_2, 4 );
        CamacqExtReadI2c( pClient, usReadKey, 2, &usReadData_lsb, 2 );
        CamacqTraceDbg("*** Denis : Shutter speed LSB = 0x%x", usReadData_lsb);

        usReadData_lsb = ((usReadData_lsb & 0x00FF) << 8) |((usReadData_lsb & 0xFF00) >> 8);
        CamacqTraceDbg("*** Denis : swapped Shutter speed LSB = 0x%x", usReadData_lsb);

        CamacqExtReadI2c( pClient, usReadKey, 2, &usReadData_msb, 2 );
        CamacqTraceDbg("*** Denis : Shutter speed MSB = 0x%x", usReadData_msb);

        usReadData_msb = ((usReadData_msb & 0x00FF) << 8) |((usReadData_msb & 0xFF00) >> 8);
        CamacqTraceDbg("*** Denis : swapped Shutter speed MSB = 0x%x", usReadData_msb);

        uiNB = ( usReadData_msb << 16 ) | usReadData_lsb ;
        CamacqTraceDbg("*** Denis : Shutter speed total  = 0x%x", uiNB);

        iRet = uiNB;// / 400;
        CamacqTraceDbg("*** Denis : Shutter speed iRet = %d ms", iRet);	

    }
    if( !strcmp( szName, "isx012") )
    {
        U16 usReadAddr1 = 0x019C;               
        U16 usReadAddr2 = 0x019E;               
        U8  ucReadData = 0;
        U32 uiNB = 0x0;    


        CamacqExtReadI2c( pClient, usReadAddr1, 2, &ucReadData, 1 );
        CamacqTraceDbg( "Denis : SONY Shutter  ucReadData=%d", ucReadData );
        uiNB = ucReadData;

        CamacqExtReadI2c( pClient, usReadAddr2, 2, &ucReadData, 1 );
        CamacqTraceDbg( "Denis : SONY Shutter  ucReadData=%d", ucReadData );
        uiNB |= ucReadData << 16;

        CamacqTraceDbg("*** Denis : Shutter speed total  = 0x%x", uiNB);

        iRet = uiNB*400;

        CamacqTraceDbg("*** Denis : Shutter speed iRet = %d ms", iRet);    
   }

    CamacqTraceOUT();
    return iRet;
}


//denis_exif
S32 CamacqExtCheckISO( struct stCamacqExtAPIs_t* this )
{
    S32 iRet = -1;
    U8* szName =  this->m_pstSensor->m_szName;
    struct i2c_client *pClient = this->m_pstSensor->m_pI2cClient;
    CamacqTraceIN();

    if( !strcmp( szName, "s5k5ccgx") )
    {
        U8	rgucWriteRegs_0[4] = {0xFC,0xFC,0xD0,0x00};
        U8	rgucWriteRegs_1[4] = {0x00,0x2C,0x70,0x00};
        U8	rgucWriteRegs_2[4] = {0x00,0x2E,0x2A,0x18};
        U16 usReadData = 0x0;
        U32 uiNB = 0x0;    
        U16 usReadKey = 0x0F12;
        
        CamacqExtWriteI2c( pClient, rgucWriteRegs_0, 4 );
        CamacqExtWriteI2c( pClient, rgucWriteRegs_1, 4 );
        CamacqExtWriteI2c( pClient, rgucWriteRegs_2, 4 );
        CamacqExtReadI2c( pClient, usReadKey, 2, &usReadData, 2 );
        CamacqTraceDbg("*** Denis :  ISO from sensor = 0x%x", usReadData);

        usReadData = ((usReadData & 0x00FF) << 8) |((usReadData & 0xFF00) >> 8);

        CamacqTraceDbg("*** Denis :  Swapped ISO from sensor = 0x%x", usReadData);

        uiNB = usReadData *1000 / 256;

        CamacqTraceDbg("*** Denis : uiNB = %d", uiNB);


        if( uiNB >= 1000  &&  uiNB < 1499)
        {
            iRet = 50; 
        }
        else if( uiNB >= 1500  &&  uiNB < 2499)
        {
            iRet = 100;	
        }
        else if( uiNB >= 2500  &&  uiNB < 3499)
        {
            iRet = 200;	
        }
        else if( uiNB >= 3500)
        {
            iRet = 400; 
        }

        CamacqTraceDbg("*** Denis : ISO final = %d", iRet);	
    }
    else if( !strcmp( szName, "isx012") )
    {
        U16 usReadAddr = 0x019A;               
        U8  ucReadData = 0;
        
        CamacqExtReadI2c( pClient, usReadAddr, 2, &ucReadData, 1 );
        CamacqTraceDbg( "Denis : SONY ISO ucReadData=%d", ucReadData );

        switch (ucReadData)
        {
            case 1:
            {
                iRet = 25;
            }
            break;

            case 2:
            {
                iRet = 32;
            }
            break;
            
            case 3:
            {
                iRet = 40;
            }
            break;
            
            case 4:
            {
                iRet = 40;
            }
            break;

            case 5:
            {
                iRet = 64;
            }
            break;

            case 6:
            {
                iRet = 80;
            }
            break;

            case 7:
            {
                iRet = 100;
            }
            break;

            case 8:
            {
                iRet = 125;
            }
            break;

            case 9:
            {
                iRet = 160;
            }
            break;

            case 10:
            {
                iRet = 200;
            }
            break;

            case 11:
            {
                iRet = 250;
            }
            break;

            case 12:
            {
                iRet = 320;
            }
            break;

            case 13:
            {
                iRet = 400;
            }
            break;

            case 14:
            {
                iRet = 500;
            }
            break;

            case 15:
            {
                iRet = 640;
            }
            break;

            case 16:
            {
                iRet = 800;
            }
            break;

            case 17:
            {
                iRet = 1000;
            }
            break;

            case 18:
            {
                iRet = 1250;
            }
            break;

            case 19:
            {
                iRet = 1600;
            }
            break;

            default:
            {
                iRet = 100;
            }
            break;

        }
    }

    CamacqTraceOUT();

    return iRet;
}


//denis_esd
S32 CamacqExtCheckESD( struct stCamacqExtAPIs_t* this )
{
    S32 iRet = -1;
    U8* szName =  this->m_pstSensor->m_szName;
    struct i2c_client *pClient = this->m_pstSensor->m_pI2cClient;
    // CamacqTraceIN();

    if( !strcmp( szName, "s5k5ccgx") )
    {
        U8	rgucWriteRegs_0[4] = {0xFC,0xFC,0xD0,0x00};
        U8	rgucWriteRegs_1[4] = {0x00,0x2C,0x70,0x00};
        U8	rgucWriteRegs_2[4] = {0x00,0x2E,0x01,0x50};
        U16 usReadData = 0x0;
        U16 usReadKey = 0x0F12;

        CamacqExtWriteI2c( pClient, rgucWriteRegs_0, 4 );
        CamacqExtWriteI2c( pClient, rgucWriteRegs_1, 4 );
        CamacqExtWriteI2c( pClient, rgucWriteRegs_2, 4 );
        CamacqExtReadI2c( pClient, usReadKey, 2, &usReadData, 2 );
//        CamacqTraceDbg_v("*** Denis :  ESD = 0x%x", usReadData);

        if ( usReadData == 0xAAAA)
            iRet = 0;   //success
        else
        {
            iRet = 1;  //fail
            CamacqTraceErr("*** Denis :  ESD failed = 0x%x", usReadData);
        }
    }
    else if( !strcmp( szName, "isx012") )
    {
        U16 usReadAddr = 0x005E;               
        U8  ucReadData = 0;
        U16 hakno = 0;

        CamacqExtReadI2c( pClient, usReadAddr, 2, &ucReadData, 1 );
        CamacqTraceDbg_v("*** Denis : ESD total = 0x%x", ucReadData);

        if ( ucReadData == 0x0000)
            iRet = 0;   //success
        else
            iRet = 1;   //fail

        usReadAddr = 0x6A18;               
        hakno = 0;

        CamacqExtReadI2c( pClient, usReadAddr, 2, &hakno, 2 );
        CamacqTraceErr("*** wingi = 0x%x", hakno);
             
    }
    else if( !strcmp( szName, "mt9v114") )
    {
        iRet = 0;       // not implemented yet
    }

//    CamacqTraceDbg_v("*** Denis : ESD result = %d", iRet);	

    // CamacqTraceOUT();

    return iRet;
}

//denis_ae
S32 CamacqExtCheckAEStability( struct stCamacqExtAPIs_t* this )
{
    S32 iRet = -1;
    U8* szName =  this->m_pstSensor->m_szName;
    struct i2c_client *pClient = this->m_pstSensor->m_pI2cClient;
    CamacqTraceIN();

    if( !strcmp( szName, "s5k5ccgx") )
    {
        U8  rgucWriteRegs_0[4] = {0xFC,0xFC,0xD0,0x00};
        U8  rgucWriteRegs_1[4] = {0x00,0x2C,0x70,0x00};
        U8  rgucWriteRegs_2[4] = {0x00,0x2E,0x1E,0x3C};
        U8 rgucReadData[2] = {0, };
        U16 usReadData = 0x00;
        U16 usReadKey = 0x0F12;

        CamacqExtWriteI2c( this->m_pstSensor->m_pI2cClient, rgucWriteRegs_0, 4 );
        CamacqExtWriteI2c( this->m_pstSensor->m_pI2cClient, rgucWriteRegs_1, 4 );
        CamacqExtWriteI2c( this->m_pstSensor->m_pI2cClient, rgucWriteRegs_2, 4 );
        CamacqExtReadI2c( this->m_pstSensor->m_pI2cClient, usReadKey, 2, rgucReadData, 2 );

        usReadData = ( rgucReadData[0] << 8 ); usReadData |= rgucReadData[1];

        CamacqTraceDbg( "usReadData = 0x%x", usReadData );

        if (usReadData == 1)
            iRet = V4L2_AE_STABLE;
        else 
            iRet = V4L2_AE_CHECKING;
    }
    
    CamacqTraceOUT();
    return iRet;
}
#undef _CAMACQ_EXT_C_
#endif /* _CAMACQ_EXT_C_ */
