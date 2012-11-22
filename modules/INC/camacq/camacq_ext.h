#if !defined(_CAMACQ_EXT_H_)
#define _CAMACQ_EXT_H_

/* Include */
#include "camacq_type.h"

/* Global */
#undef GLOBAL

#if !defined(_CAMACQ_EXT_C_)
#define GLOBAL extern
#else
#define GLOBAL
#endif

/* Definition */
#define CAMACQ_EXT_MODE_W 1
#define CAMACQ_EXT_MODE_R 2

#define CAMACQ_EXT_NONE 0x00
#define CAMACQ_EXT_HFLIP 0x01
#define CAMACQ_EXT_VFLIP 0x02

#define CAMACQ_EXT_DEF_HFLIP 0
#define CAMACQ_EXT_DEF_VFLIP 0
#define CAMACQ_EXT_DEF_NIGHT_MODE 0
#define CAMACQ_EXT_DEF_BRIGHT 0
#define CAMACQ_EXT_DEF_EXP_COMPENSATION 0
#define CAMACQ_EXT_DEF_MET_EXP V4L2_CTRL_MET_EXP_MATRIX
#define CAMACQ_EXT_DEF_SENS V4L2_CTRL_ISO_AUTO
#define CAMACQ_EXT_DEF_SCENE V4L2_CTRL_SCENE_AUTO
#define CAMACQ_EXT_DEF_AUTO_FCS V4L2_ATFCS_CTRL_OFF

#define CAMACQ_EXT_NUM_CAPTURE_FMTS 1
#define CAMACQ_EXT_NUM_OVERLAY_FMTS 1
#define CAMACQ_EXT_NUM_WBMODE 6
#define CAMACQ_EXT_NUM_CTRLS (sizeof(g_stCamacqControls)/sizeof(g_stCamacqControls[0]))

#define CAMACQ_EXT_MAX_PATH 255
#define CAMACQ_EXT_HEX_WIDTH 0x10
#define CAMACQ_EXT_HEX_HALF_LEN 2/* Except 0x */
#define CAMACQ_EXT_HEX_CHAR "0xFF"
#define CAMACQ_EXT_HEX_SHORT "0xFFFF"
#define CAMACQ_EXT_HEX_INT "0xFFFFFFFF"
#define CAMACQ_EXT_STAR '*'
#define CAMACQ_EXT_SLASH '/'
#define CAMACQ_EXT_0 '0'
#define CAMACQ_EXT_9 '9'
#define CAMACQ_EXT_a 'a'
#define CAMACQ_EXT_f 'f'
#define CAMACQ_EXT_A 'A'
#define CAMACQ_EXT_F 'F'
#define CAMACQ_EXT_x 'x'
#define CAMACQ_EXT_X 'X'
#define CAMACQ_EXT_COMMENT_START(a) ((a[0] == CAMACQ_EXT_SLASH) &&( (a[1] == CAMACQ_EXT_STAR)||(a[1] == CAMACQ_EXT_SLASH)))
#define CAMACQ_EXT_COMMENT_CLOSE "*/"
#define CAMACQ_EXT_START "{"
//#define CAMACQ_EXT_COMMENT_CLOSE(a) (((a[0] == CAMACQ_EXT_STAR)&& (a[1] == CAMACQ_EXT_SLASH))||(a[0] == CAMACQ_EXT_START))
#define CAMACQ_EXT_HEXA_START(a) (a[0] == CAMACQ_EXT_0 && (a[1] == CAMACQ_EXT_X || a[1] == CAMACQ_EXT_x))

/* Enumeration */
typedef enum eCamacqExtMode_e
{
    CAMACQ_EXT_MODE_OFF = 0,
    CAMACQ_EXT_MODE_ON,
    CAMACQ_EXT_MODE_GET,
    CAMACQ_EXT_MODE_SET
} _eCamacqExtMode;

typedef enum eCamacqSelLen_e
{
    CAMACQ_EXT_LEN_2BYTE_ARY = 0,
    CAMACQ_EXT_LEN_4BYTE_ARY,
    CAMACQ_EXT_LEN_2BYTE_INT,
    CAMACQ_EXT_LEN_4BYTE_INT,
    CAMACQ_EXT_LEN_SONY,
    CAMACQ_EXT_LEN_LEN_MAX
} _eCamacqSelLen;

typedef enum eCamacqAF_e
{
    CAMACQ_EXT_AF_CONTINUE = 1,
    CAMACQ_EXT_AF_FAILD,
    CAMACQ_EXT_AF_SUCCESS
} _eCamacqAF;

typedef enum eCamacqBr_e
{
    CAMACQ_EXT_BR_LVL_0 = 0,
    CAMACQ_EXT_BR_LVL_1,
    CAMACQ_EXT_BR_LVL_2,
    CAMACQ_EXT_BR_LVL_3,
    CAMACQ_EXT_BR_LVL_4,
    CAMACQ_EXT_BR_LVL_5,
    CAMACQ_EXT_BR_LVL_6,
    CAMACQ_EXT_BR_LVL_7,
    CAMACQ_EXT_BR_LVL_8
} _eCamacqBR;

typedef enum eCamacqExpCompensation_e
{
    CAMACQ_EXT_EXP_COMP_LVL_0 = 0,
    CAMACQ_EXT_EXP_COMP_LVL_1,
    CAMACQ_EXT_EXP_COMP_LVL_2,
    CAMACQ_EXT_EXP_COMP_LVL_3,
    CAMACQ_EXT_EXP_COMP_LVL_4,
    CAMACQ_EXT_EXP_COMP_LVL_5,
    CAMACQ_EXT_EXP_COMP_LVL_6,
    CAMACQ_EXT_EXP_COMP_LVL_7,
    CAMACQ_EXT_EXP_COMP_LVL_8
} _eCamacqExpCompensation;

typedef enum eCamacqExpZoom_e
{
    CAMACQ_EXT_ZOOM_LVL_0 = 0,
    CAMACQ_EXT_ZOOM_LVL_1,
    CAMACQ_EXT_ZOOM_LVL_2,
    CAMACQ_EXT_ZOOM_LVL_3,
    CAMACQ_EXT_ZOOM_LVL_4,
    CAMACQ_EXT_ZOOM_LVL_5,
    CAMACQ_EXT_ZOOM_LVL_6,
    CAMACQ_EXT_ZOOM_LVL_7,
    CAMACQ_EXT_ZOOM_LVL_8
} _eCamacqExpZoom;

typedef enum eCamacqExpSensorZoom_e
{
    CAMACQ_EXT_128X_ZOOM_LVL_0 = 0,
    CAMACQ_EXT_128X_ZOOM_LVL_1,
    CAMACQ_EXT_128X_ZOOM_LVL_2,
    CAMACQ_EXT_128X_ZOOM_LVL_3,
    CAMACQ_EXT_128X_ZOOM_LVL_4,
    CAMACQ_EXT_128X_ZOOM_LVL_5,
    CAMACQ_EXT_128X_ZOOM_LVL_6,
    CAMACQ_EXT_128X_ZOOM_LVL_7,
    CAMACQ_EXT_128X_ZOOM_LVL_8,
    
    CAMACQ_EXT_160X_ZOOM_LVL_0,
    CAMACQ_EXT_160X_ZOOM_LVL_1,
    CAMACQ_EXT_160X_ZOOM_LVL_2,
    CAMACQ_EXT_160X_ZOOM_LVL_3,
    CAMACQ_EXT_160X_ZOOM_LVL_4,
    CAMACQ_EXT_160X_ZOOM_LVL_5,
    CAMACQ_EXT_160X_ZOOM_LVL_6,
    CAMACQ_EXT_160X_ZOOM_LVL_7,
    CAMACQ_EXT_160X_ZOOM_LVL_8,    
    
    CAMACQ_EXT_200X_ZOOM_LVL_0,
    CAMACQ_EXT_200X_ZOOM_LVL_1,
    CAMACQ_EXT_200X_ZOOM_LVL_2,
    CAMACQ_EXT_200X_ZOOM_LVL_3,
    CAMACQ_EXT_200X_ZOOM_LVL_4,
    CAMACQ_EXT_200X_ZOOM_LVL_5,
    CAMACQ_EXT_200X_ZOOM_LVL_6,
    CAMACQ_EXT_200X_ZOOM_LVL_7,
    CAMACQ_EXT_200X_ZOOM_LVL_8        
} _eCamacqExpSensorZoom;


typedef enum eCamacqExpAdjustContrast_e
{
    CAMACQ_EXT_ADJUST_CONTRAST_LVL_M2 = 0,
    CAMACQ_EXT_ADJUST_CONTRAST_LVL_M1,
    CAMACQ_EXT_ADJUST_CONTRAST_LVL_DEFAULT,
    CAMACQ_EXT_ADJUST_CONTRAST_LVL_P1,
    CAMACQ_EXT_ADJUST_CONTRAST_LVL_P2,
} _eCamacqExpAdjustContrast;

typedef enum eCamacqExpAdjustSharpness_e
{
    CAMACQ_EXT_ADJUST_SHARPNESS_LVL_M2 = 0,
    CAMACQ_EXT_ADJUST_SHARPNESS_LVL_M1,
    CAMACQ_EXT_ADJUST_SHARPNESS_LVL_DEFAULT,
    CAMACQ_EXT_ADJUST_SHARPNESS_LVL_P1,
    CAMACQ_EXT_ADJUST_SHARPNESS_LVL_P2,
} _eCamacqExpAdjustSharpness;

typedef enum eCamacqExpAdjustSaturation_e
{
    CAMACQ_EXT_ADJUST_SATURATION_LVL_M2 = 0,
    CAMACQ_EXT_ADJUST_SATURATION_LVL_M1,
    CAMACQ_EXT_ADJUST_SATURATION_LVL_DEFAULT,
    CAMACQ_EXT_ADJUST_SATURATION_LVL_P1,
    CAMACQ_EXT_ADJUST_SATURATION_LVL_P2,
} _eCamacqExpAdjustSaturation;


typedef enum eCamacqWb_e
{
    CAMACQ_EXT_WB_AUTO = 0,
    CAMACQ_EXT_WB_DAYLIGHT,
    CAMACQ_EXT_WB_TUNGSTEN,
    CAMACQ_EXT_WB_FLUORESCENT,
    CAMACQ_EXT_WB_HORIZON,
    CAMACQ_EXT_WB_OFF,
    CAMACQ_EXT_WB_MAX,
} _eCamacqWb;

typedef enum eCamacqJpegQuality_e
{
    CAMACQ_EXT_JPEG_QUALITY_SUPERFINE = 100,
    CAMACQ_EXT_JPEG_QUALITY_FINE = 70,
    CAMACQ_EXT_JPEG_QUALITY_NORMAL = 40,
} _eCamacqJpegQuality;


/* Type Definition */
typedef struct stCamacqSensorRegs_t 
{
    const void*     pvPllRegs;
    const void*     pvInitRegs;
    const void*     pvSleepRegs;
    const void*     pvWakeupRegs;
    const void*     pvPreviewRegs;
    const void*     pvSnapshotRegs;
    const void*     pvNightshotRegs;
    const void*     pvLowLightshotRegs;
    const void*     pvHighLightshotRegs;
    const void*     pvCamcorderOnRegs;

    /* preview size for camcorder .. or etc*/
    const void*     pvQCIF;     // 176x144
    const void*     pvQVGA;     // 320x240
    const void*     pvVGA;      // 640x480
    const void*     pv720P;     // 1280x720
    const void*     pv1080P;    // 1280x1080
    const void*     pv800_480;  // 800x480
    const void*     pv720_480;  // 720x480

    /* ME */
    const void*     pvMeterMatrixRegs;
    const void*     pvMeterCWRegs;
    const void*     pvMeterSpotRegs;

    /* flip */
    const void*     pvFlipNoneRegs;
    const void*     pvFlipWaterRegs;
    const void*     pvFlipMirrorRegs;
    const void*     pvFlipWaterMirrorRegs;

    /* ISO */
    const void*     pvISOAutoRegs;
    const void*     pvISO50Regs;
    const void*     pvISO100Regs;
    const void*     pvISO200Regs;
    const void*     pvISO400Regs;
    const void*     pvISO800Regs;
    const void*     pvISO1600Regs;
    const void*     pvISO3200Regs;

    /* Scene */
    const void*     pvSceneAutoRegs;
    const void*     pvSceneNightRegs;
    const void*     pvSceneLandScapeRegs;
    const void*     pvSceneSunSetRegs;
    const void*     pvScenePortraitRegs;
    const void*     pvSceneSunRiseRegs;
    const void*     pvSceneIndoorRegs;
    const void*     pvScenePartyRegs;
    const void*     pvSceneSportsRegs;
    const void*     pvSceneBeachRegs;
    const void*     pvSceneSnowRegs;
    const void*     pvSceneFallColorRegs;
    const void*     pvSceneFireWorksRegs;
    const void*     pvSceneCandleLightRegs;
    const void*     pvSceneAgainstLightRegs;
    const void*     pvSceneTextRegs;
    const void*     pvSceneAquaRegs;

    /* Brightness */
    const void*     pvBrightness_0_Regs;
    const void*     pvBrightness_1_Regs;
    const void*     pvBrightness_2_Regs;
    const void*     pvBrightness_3_Regs;
    const void*     pvBrightness_4_Regs;
    const void*     pvBrightness_5_Regs;
    const void*     pvBrightness_6_Regs;
    const void*     pvBrightness_7_Regs;
    const void*     pvBrightness_8_Regs;

    /* ExposureCompensation */
    const void*     pvExpCompensation_0_Regs;
    const void*     pvExpCompensation_1_Regs;
    const void*     pvExpCompensation_2_Regs;
    const void*     pvExpCompensation_3_Regs;
    const void*     pvExpCompensation_4_Regs;
    const void*     pvExpCompensation_5_Regs;
    const void*     pvExpCompensation_6_Regs;
    const void*     pvExpCompensation_7_Regs;
    const void*     pvExpCompensation_8_Regs;

    /* AF */
    const void*     pvSetAFRegs;
    const void*     pvOffAFRegs;
    const void*     pvCheckAFRegs;
    const void*     pvResetAFRegs;
    const void*     pvCancelMacroAFRegs;
    const void*     pvCancelManualAFRegs;
    const void*     pvManualAFReg;  // auto, normal
    const void*     pvMacroAFReg;
    const void*     pvReturnManualAFReg;
    const void*     pvReturnMacroAFReg;
    const void*     pvSetAF_NLUXRegs;
    const void*     pvSetAF_LLUXRegs;

    const void*     pvSetAFNormalMode1;
    const void*     pvSetAFNormalMode2;
    const void*     pvSetAFNormalMode3;
    const void*     pvSetAFMacroMode1;
    const void*     pvSetAFMacroMode2;
    const void*     pvSetAFMacroMode3;

    /* WB */
    const void*     pvWbAutoRegs;
    const void*     pvWbDaylightRegs;
    const void*     pvWbCloudyRegs;
    const void*     pvWbIncandescentRegs;
    const void*     pvWbFluorescentRegs;
    const void*     pvWbTwilightRegs;
    const void*     pvWbTungstenRegs;
    const void*     pvWbOffRegs;
    const void*     pvWbHorizonRegs;
    const void*     pvWbShadeRegs;

    /* FMT */
    const void*     pvFmtJpegRegs;
    const void*     pvFmtYUV422Regs;

    /* Effect */
    const void*     pvEffectNoneRegs;
    const void*     pvEffectGrayRegs;
    const void*     pvEffectNegativeRegs;
    const void*     pvEffectSolarizeRegs;
    const void*     pvEffectSepiaRegs;
    const void*     pvEffectPosterizeRegs;
    const void*     pvEffectWhiteBoardRegs;
    const void*     pvEffectBlackBoardRegs;
    const void*     pvEffectAquaRegs;
    const void*     pvEffectSharpenRegs;
    const void*     pvEffectVividRegs;
    const void*     pvEffectGreenRegs;
    const void*     pvEffectSketchRegs;

    const void*     pvFlickerDisabled;
    const void*     pvFlicker50Hz;
    const void*     pvFlicker60Hz;
    const void*     pvFlickerAuto;

    /* Adjust */
    const void*     pvAdjustContrastM2;
    const void*     pvAdjustContrastM1;
    const void*     pvAdjustContrastDefault;
    const void*     pvAdjustContrastP1;
    const void*     pvAdjustContrastP2;
    
    const void*     pvAdjustSharpnessM2;
    const void*     pvAdjustSharpnessM1;
    const void*     pvAdjustSharpnessDefault;
    const void*     pvAdjustSharpnessP1;
    const void*     pvAdjustSharpnessP2;

    const void*     pvAdjustSaturationM2;
    const void*     pvAdjustSaturationM1;
    const void*     pvAdjustSaturationDefault;
    const void*     pvAdjustSaturationP1;
    const void*     pvAdjustSaturationP2;

    /* Jpeg Quality */
    const void*     pvJpegQualitySuperfine;
    const void*     pvJpegQualityFine;
    const void*     pvJpegQualityNormal;    
    
    /* Jpeg output size */
    const void*    pvJpegOutSize5M;
    const void*    pvJpegOutSize5M_2;
    const void*    pvJpegOutSize4M;
    const void*    pvJpegOutSize3M;
    const void*    pvJpegOutSize2M;
    const void*    pvJpegOutSize1_5M;
    const void*    pvJpegOutSize1M;
    const void*    pvJpegOutSizeVGA;
    const void*    pvJpegOutSizeWQVGA;
    const void*    pvJpegOutSizeQVGA;

    /* FPS */
    const void*    pvFpsFixed_5;
    const void*    pvFpsFixed_7;
    const void*    pvFpsFixed_10;
    const void*    pvFpsFixed_15;
    const void*    pvFpsFixed_20;
    const void*    pvFpsFixed_25;
    const void*    pvFpsFixed_30;
    const void*    pvFpsAuto_15;

    /* AE/AWB Lock,Unlcok */
    const void*    pvAeLock;
    const void*    pvAeUnLock;
    const void*    pvAwbLock;
    const void*    pvAwbUnLock;   

    /**/
    const void*    pvPreFlashStart;
    const void*    pvPreFlashEnd;
    const void*    pvMainFlashStart;
    const void*    pvMainFlashEnd;

    const void*    pvFlashAeSet;
    const void*    pvFlashAeClear;

    const void*    pvFlashOnSet;        //denis_flash_2

    const void*    pvDtpOn;
    const void*    pvDtpOff;

    /* Zoom */
    const void*     pvZoom_128x_0_Regs;
    const void*     pvZoom_128x_1_Regs;
    const void*     pvZoom_128x_2_Regs;
    const void*     pvZoom_128x_3_Regs;
    const void*     pvZoom_128x_4_Regs;
    const void*     pvZoom_128x_5_Regs;
    const void*     pvZoom_128x_6_Regs;
    const void*     pvZoom_128x_7_Regs;
    const void*     pvZoom_128x_8_Regs;

    const void*     pvZoom_160x_0_Regs;
    const void*     pvZoom_160x_1_Regs;
    const void*     pvZoom_160x_2_Regs;
    const void*     pvZoom_160x_3_Regs;
    const void*     pvZoom_160x_4_Regs;
    const void*     pvZoom_160x_5_Regs;
    const void*     pvZoom_160x_6_Regs;
    const void*     pvZoom_160x_7_Regs;
    const void*     pvZoom_160x_8_Regs;

    const void*     pvZoom_200x_0_Regs;
    const void*     pvZoom_200x_1_Regs;
    const void*     pvZoom_200x_2_Regs;
    const void*     pvZoom_200x_3_Regs;
    const void*     pvZoom_200x_4_Regs;
    const void*     pvZoom_200x_5_Regs;
    const void*     pvZoom_200x_6_Regs;
    const void*     pvZoom_200x_7_Regs;
    const void*     pvZoom_200x_8_Regs;

    /* Private Control */
    const void*    pvPrivCtrlReturnPreview;

} _stCamacqSensorRegs;

typedef struct stCamacqExtAPIs_t
{
    struct stCamacqSensor_t* m_pstSensor;           /* owner */
    struct stCamacqSensorRegs_t *m_pstSensorRegs;   /* set file lists */

    // H/W Control
    S32 (*PowerOn)(void);
    S32 (*PowerOff)(void);
    S32 (*Switch)(void);
    S32 (*Reset)(void);
    S32 (*Standby)(void);
    S32 (*Wakeup)(void);
    
    // EXT APIs
    S32 (*Boot)( struct stCamacqExtAPIs_t* this );
    S32 (*Hflip)( struct stCamacqExtAPIs_t* this, _eCamacqExtMode eMode, S32 *piVal );
    S32 (*Vflip)( struct stCamacqExtAPIs_t* this, _eCamacqExtMode eMode, S32 *piVal );
    S32 (*SetFlip)( struct stCamacqExtAPIs_t* this, U8 ucFlip, U8 ucVal );
    S32 (*Exposure)( struct stCamacqExtAPIs_t* this, _eCamacqExtMode eMode, S32 *piVal );
    S32 (*Sensitivity)( struct stCamacqExtAPIs_t* this, _eCamacqExtMode eMode, S32 *piVal );
    S32 (*Scene)( struct stCamacqExtAPIs_t* this, _eCamacqExtMode eMode, S32 *piVal );
    S32 (*Brightness)( struct stCamacqExtAPIs_t* this, _eCamacqExtMode eMode, S32 *piVal );
    S32 (*ExposureCompensation)( struct stCamacqExtAPIs_t* this, _eCamacqExtMode eMode, S32 *piVal );
    S32 (*Zoom)( struct stCamacqExtAPIs_t* this, _eCamacqExtMode eMode, S32 *piVal );
    S32 (*Saturation)( struct stCamacqExtAPIs_t* this, _eCamacqExtMode eMode, S32 *piVal );
    S32 (*Contrast)( struct stCamacqExtAPIs_t* this, _eCamacqExtMode eMode, S32 *piVal );
    S32 (*Sharpness)( struct stCamacqExtAPIs_t* this, _eCamacqExtMode eMode, S32 *piVal );
    S32 (*Autofocus)( struct stCamacqExtAPIs_t* this, _eCamacqExtMode eMode, S32 *piVal );
    S32 (*Whitebalance)( struct stCamacqExtAPIs_t* this, _eCamacqExtMode eMode, const S8 *piVal );
    S32 (*AntiBanding)( struct stCamacqExtAPIs_t* this, _eCamacqExtMode eMode, S32 *piVal );
    S32 (*AutofocusMode)( struct stCamacqExtAPIs_t* this, _eCamacqExtMode eMode, S32 *piVal );
    S32 (*FlashControl) ( struct stCamacqExtAPIs_t* this, _eCamacqExtMode eMode, S32 *piVal );
    S32 (*CameraPrivateControl)( struct stCamacqExtAPIs_t* this, _eCamacqExtMode eMode, S32 *piVal );
    S32 (*InitCamcorder)( struct stCamacqExtAPIs_t* this, _eCamacqExtMode eMode, S32 *piVal );
    S32 (*SetCamcorderSize)( struct stCamacqExtAPIs_t* this, _eCamacqExtMode eMode, S32 *piVal );
    S32 (*InitVT)( struct stCamacqExtAPIs_t* this, _eCamacqExtMode eMode, S32 *piVal );
    S32 (*SetVTSize)( struct stCamacqExtAPIs_t* this, _eCamacqExtMode eMode, S32 *piVal );
    S32 (*SetPreviewSize)( struct stCamacqExtAPIs_t* this, _eCamacqExtMode eMode, S32 *piVal );
    S32 (*SetFps)( struct stCamacqExtAPIs_t* this, _eCamacqExtMode eMode, S32 *piVal );
    
}_stCamacqExtAPIs;

/* EXT APIs */
GLOBAL S32 PowerOn_MAIN(void);
GLOBAL S32 PowerOff_MAIN(void);
GLOBAL S32 Switch_MAIN(void);
GLOBAL S32 Reset_MAIN(void);
GLOBAL S32 Standby_MAIN(void);
GLOBAL S32 Wakeup_MAIN(void);

GLOBAL S32 PowerOn_SUB(void);
GLOBAL S32 PowerOff_SUB(void);
GLOBAL S32 Switch_SUB(void);
GLOBAL S32 Reset_SUB(void);
GLOBAL S32 Standby_SUB(void);
GLOBAL S32 Wakeup_SUB(void);

GLOBAL S32 CamacqExtBoot( struct stCamacqExtAPIs_t* this );
GLOBAL S32 CamacqExtSetFlip( struct stCamacqExtAPIs_t* this, U8 ucFlip, U8 ucVal );
GLOBAL S32 CamacqExtHflip( struct stCamacqExtAPIs_t* this, _eCamacqExtMode eMode, S32 *piVal );
GLOBAL S32 CamacqExtVflip( struct stCamacqExtAPIs_t* this, _eCamacqExtMode eMode, S32 *piVal );
GLOBAL S32 CamacqExtExposure( struct stCamacqExtAPIs_t* this, _eCamacqExtMode eMode, S32 *piVal );
GLOBAL S32 CamacqExtSensitivity( struct stCamacqExtAPIs_t* this, _eCamacqExtMode eMode, S32 *piVal );
GLOBAL S32 CamacqExtScene( struct stCamacqExtAPIs_t* this, _eCamacqExtMode eMode, S32 *piVal );
GLOBAL S32 CamacqExtBrightness( struct stCamacqExtAPIs_t* this, _eCamacqExtMode eMode, S32 *piVal );
GLOBAL S32 CamacqExtExposureCompensation( struct stCamacqExtAPIs_t* this, _eCamacqExtMode eMode, S32 *piVal );
GLOBAL S32 CamacqExtZoom( struct stCamacqExtAPIs_t* this, _eCamacqExtMode eMode, S32 *piVal );
GLOBAL S32 CamacqExtContrast( struct stCamacqExtAPIs_t* this, _eCamacqExtMode eMode, S32 *piVal );
GLOBAL S32 CamacqExtSaturation( struct stCamacqExtAPIs_t* this, _eCamacqExtMode eMode, S32 *piVal );
GLOBAL S32 CamacqExtSharpness( struct stCamacqExtAPIs_t* this, _eCamacqExtMode eMode, S32 *piVal );
GLOBAL S32 CamacqExtAutofocus( struct stCamacqExtAPIs_t* this, _eCamacqExtMode eMode, S32 *piVal );
GLOBAL S32 CamacqExtWhitebalance( struct stCamacqExtAPIs_t* this, _eCamacqExtMode eMode, const S8 *piVal );
GLOBAL S32 CamacqExtAntiBanding( struct stCamacqExtAPIs_t* this, _eCamacqExtMode eMode, S32 *piVal );
GLOBAL S32 CamacqExtAutofocusMode( struct stCamacqExtAPIs_t* this, _eCamacqExtMode eMode, S32 *piVal );
GLOBAL S32 CamacqExtFlashControl( struct stCamacqExtAPIs_t* this, _eCamacqExtMode eMode, S32 *piVal );
GLOBAL S32 CamacqExtCameraPrivateControl( struct stCamacqExtAPIs_t* this, _eCamacqExtMode eMode, S32 *piVal );
GLOBAL S32 CamacqExtInitCamcorder( struct stCamacqExtAPIs_t* this, _eCamacqExtMode eMode, S32 *piVal );
GLOBAL S32 CamacqExtSetCamcorderSize( struct stCamacqExtAPIs_t* this, _eCamacqExtMode eMode, S32 *piVal );
GLOBAL S32 CamacqExtInitVT( struct stCamacqExtAPIs_t* this, _eCamacqExtMode eMode, S32 *piVal );
GLOBAL S32 CamacqExtSetVTSize( struct stCamacqExtAPIs_t* this, _eCamacqExtMode eMode, S32 *piVal );
GLOBAL S32 CamacqExtSetPreviewSize( struct stCamacqExtAPIs_t* this, _eCamacqExtMode eMode, S32 *piVal );
GLOBAL S32 CamacqExtSetFps( struct stCamacqExtAPIs_t* this, _eCamacqExtMode eMode, S32 *piVal );

/* Functions */

/* EXT Common API */
GLOBAL void     CamacqExtDelay( S32 iDelay );
GLOBAL S32      CamacqExtRequestGpio( S32 iGpio, U8* szModuleName );
GLOBAL void     CamacqExtFreeGpio( S32 iGpio );
GLOBAL S32      CamacqExtSetGpioConfig( S32 iGpio, S32 iMux, S32 iDirection, S32 iLevel );

GLOBAL S32      CamacqExtWriteI2c( struct i2c_client *pClient, U8 * pucValue, U8 ucSize ); 
GLOBAL S32      CamacqExtReadI2c( struct i2c_client *pClient, U16 usAddr, U8 ucAddrSize, U8 * pucBuf, U8 ucReadSize );
GLOBAL S32      CamacqExtWriteI2cLists( struct i2c_client *pClient, const void *pvArg, int iResType );
GLOBAL S32      CamacqExtCheckLux( struct stCamacqExtAPIs_t* this );

/* File System API */
GLOBAL U8* CamacqExtReadFs( const S8 * szFileName, _eCamacqSelLen eLen, int iResType );
GLOBAL U8  CamacqExtAsc2Hex( S8 *pcAscii );


//denis_exif
GLOBAL S32 CamacqExtCheckExposureTime( struct stCamacqExtAPIs_t* this );
GLOBAL S32 CamacqExtCheckISO( struct stCamacqExtAPIs_t* this );

//denis_esd
GLOBAL S32 CamacqExtCheckESD( struct stCamacqExtAPIs_t* this );

//denis_ae
GLOBAL S32 CamacqExtCheckAEStability( struct stCamacqExtAPIs_t* this );

/* EXT sensor depending API */
#if defined(__ISX006_SONY__) || defined(__ISX012_SONY__)
GLOBAL S32 CamacqExtWriteI2cLists_Sony( struct i2c_client *pClient, const void *pvArg, int iResType );
GLOBAL struct stSonyData_t* CamacqExtReadFs_Sony( const S8 * szFileName, int iResType );
GLOBAL S32 WaitForModeTransition_Sony( struct i2c_client *pClient, int iMode );
GLOBAL S32 Callibration_Sony(  struct i2c_client *pClient, int iResType );
GLOBAL U16 calculate_AEgain_offset(U16 ae_auto,U16 ae_now,U16 ersc_auto,U16 ersc_now, U32 uiResType);
#endif /* __ISX006_SONY__ */

/* Global Variable */
GLOBAL _stCamacqSensorRegs      g_stCamacqMainSensorRegs;
GLOBAL _stCamacqSensorRegs      g_stCamacqSubSensorRegs;

#undef GLOBAL

#endif /* _CAMACQ_EXT_H_ */
