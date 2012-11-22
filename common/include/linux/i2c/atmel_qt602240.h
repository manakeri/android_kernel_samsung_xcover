/*
 *  Atmel Touch Screen Driver
 *
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 */

#ifndef _ATMEL_AT42QT5480_H
#define	_ATMEL_AT42QT5480_H

/* Register definitions based on AT42QT preliminary draft datasheet */
#define RESERVED_T0                               0
#define RESERVED_T1                               1
#define DEBUG_DELTAS_T2                           2
#define DEBUG_REFERENCES_T3                       3
#define DEBUG_SIGNALS_T4                          4
#define GEN_MESSAGEPROCESSOR_T5                   5
#define GEN_COMMANDPROCESSOR_T6                   6
#define GEN_POWERCONFIG_T7                        7
#define GEN_ACQUISITIONCONFIG_T8                  8
#define TOUCH_MULTITOUCHSCREEN_T9                 9
#define TOUCH_SINGLETOUCHSCREEN_T10               10
#define TOUCH_XSLIDER_T11                         11
#define TOUCH_YSLIDER_T12                         12
#define TOUCH_XWHEEL_T13                          13
#define TOUCH_YWHEEL_T14                          14
#define TOUCH_KEYARRAY_T15                        15
#define PROCG_SIGNALFILTER_T16                    16
#define PROCI_LINEARIZATIONTABLE_T17              17
#define SPT_COMCONFIG_T18                         18
#define SPT_GPIOPWM_T19                           19
#define PROCI_GRIPFACESUPPRESSION_T20             20
#define RESERVED_T21                              21
#define PROCG_NOISESUPPRESSION_T22                22
#define TOUCH_PROXIMITY_T23                           23
#define PROCI_ONETOUCHGESTUREPROCESSOR_T24        24
#define SPT_SELFTEST_T25                          25
#define DEBUG_CTERANGE_T26                        26
#define PROCI_TWOTOUCHGESTUREPROCESSOR_T27        27
#define SPT_CTECONFIG_T28                         28
#define SPT_GPI_T29                               29
#define SPT_GATE_T30                              30
#define TOUCH_KEYSET_T31                          31
#define TOUCH_XSLIDERSET_T32                      32
#define RESERVED_T33                              33
#define GEN_MESSAGEBLOCK_T34                      34
#define SPT_GENERICDATA_T35                       35
#define RESERVED_T36                              36
#define DEBUG_DIAGNOSTIC_T37                      37
#define SPT_USERDATA_T38                          38

/*! \brief Returned by get_object_address() if object is not found. */
#define OBJECT_NOT_FOUND   0

/*! Address where object table starts at touch IC memory map. */
#define OBJECT_TABLE_START_ADDRESS      7

/*! Size of one object table element in touch IC memory map. */
#define OBJECT_TABLE_ELEMENT_SIZE       6

/*! Offset to RESET register from the beginning of command processor. */
#define RESET_OFFSET                        0

/*! Offset to BACKUP register from the beginning of command processor. */
#define BACKUP_OFFSET           1

/*! Offset to CALIBRATE register from the beginning of command processor. */
#define CALIBRATE_OFFSET        2

/*! Offset to REPORTALL register from the beginning of command processor. */
#define REPORTATLL_OFFSET       3

/*! Offset to DEBUG_CTRL register from the beginning of command processor. */
#define DEBUG_CTRL_OFFSET       4

/*! Offset to DIAGNOSTIC register from the beginning of command processor. */
#define DIAGNOSTIC_OFFSET       5

#define CONNECT_OK                  1
#define CONNECT_ERROR               2

#define MESSAGE_READ_OK             1
#define MESSAGE_READ_FAILED         2

#define WRITE_MEM_OK                1
#define WRITE_MEM_FAILED            2

#define CFG_WRITE_OK                1
#define CFG_WRITE_FAILED            2

#define I2C_INIT_OK                 1
#define I2C_INIT_FAIL               2

#define CRC_CALCULATION_OK          1
#define CRC_CALCULATION_FAILED      2

#define ID_MAPPING_OK               1
#define ID_MAPPING_FAILED           2

#define ID_DATA_OK                  1
#define ID_DATA_NOT_AVAILABLE       2

/* -------------------- Type define block --------------------------------*/
typedef struct
{
	u8 reset;       /*!< Force chip reset             */
	u8 backupnv;    /*!< Force backup to eeprom/flash */
	u8 calibrate;   /*!< Force recalibration          */
	u8 reportall;   /*!< Force all objects to report  */
	u8 debugctrl;   /*!< Turn on output of debug data */
	u8 diagnostic;  /*!< Controls the diagnostic object */
} gen_commandprocessor_t6_config_t;

typedef struct
{
	u8 idleacqint;    /*!< Idle power mode sleep length in ms           */
	u8 actvacqint;    /*!< Active power mode sleep length in ms         */
	u8 actv2idleto;   /*!< Active to idle power mode delay length in ms */
} gen_powerconfig_t7_config_t;

typedef struct
{
	u8 chrgtime;          /*!< Burst charge time                      */
	u8 Reserved;  
	u8 tchdrift;          /*!< Touch drift compensation period        */
	u8 driftst;           /*!< Drift suspend time                     */
	u8 tchautocal;        /*!< Touch automatic calibration delay in ms*/
	u8 sync;              /*!< Measurement synchronisation control    */
	u8 atchcalst;
	u8 atchcalsthr;      
} gen_acquisitionconfig_t8_config_t;

typedef struct
{
	/* Screen Configuration */
	u8 ctrl;            /*!< Main configuration field           */

	/* Physical Configuration */
	u8 xorigin;         /*!< Object x start position on matrix  */
	u8 yorigin;         /*!< Object y start position on matrix  */
	u8 xsize;           /*!< Object x size (i.e. width)         */
	u8 ysize;           /*!< Object y size (i.e. height)        */

	/* Detection Configuration */
	u8 akscfg;          /*!< Adjacent key suppression config     */
	u8 blen;            /*!< Burst length for all object channels*/
	u8 tchthr;          /*!< Threshold for all object channels   */
	u8 tchdi;           /*!< Detect integration config           */

	u8 orientate;  /*!< Controls flipping and rotating of touchscreen
						 *   object */
	u8 mrgtimeout; /*!< Timeout on how long a touch might ever stay
						 *   merged - units of 0.2s, used to tradeoff power
						 *                           *   consumption against being able to detect a touch
						 *                                                   *   de-merging early */

	/* Position Filter Configuration */
	u8 movhysti;   /*!< Movement hysteresis setting used after touchdown */
	u8 movhystn;   /*!< Movement hysteresis setting used once dragging   */
	u8 movfilter;  /*!< Position filter setting controlling the rate of  */

	/* Multitouch Configuration */
	u8 numtouch;   /*!< The number of touches that the screen will attempt
						 *   to track */
	u8 mrghyst;    /*!< The hystersis applied on top of the merge threshold
						 *   to stop oscillation */
	u8 mrgthr;     /*!< The threshold for the point when two peaks are
						 *   considered one touch */

	u8 tchamphyst;          /*!< TBD */

	/* Resolution Controls */
	u16 xres;
	u16 yres;
	u8 xloclip;
	u8 xhiclip;
	u8 yloclip;
	u8 yhiclip;
	u8 xedgectrl;
	u8 xedgedist;
	u8 yedgectrl;
	u8 yedgedist;
	u8 jumplimit;

} touch_multitouchscreen_t9_config_t;

typedef struct
{
	/* Key Array Configuration */
	u8 ctrl;               /*!< Main configuration field           */

	/* Physical Configuration */
	u8 xorigin;           /*!< Object x start position on matrix  */
	u8 yorigin;           /*!< Object y start position on matrix  */
	u8 xsize;             /*!< Object x size (i.e. width)         */
	u8 ysize;             /*!< Object y size (i.e. height)        */

	/* Detection Configuration */
	u8 akscfg;             /*!< Adjacent key suppression config     */
	u8 blen;               /*!< Burst length for all object channels*/
	u8 tchthr;             /*!< Threshold for all object channels   */
	u8 tchdi;              /*!< Detect integration config           */
	u8 reserved[2];        /*!< Spare x2 */

} touch_keyarray_t15_config_t;

typedef struct
{
	u8  ctrl;
	u16 xoffset;
	u8  xsegment[16];
	u16 yoffset;
	u8  ysegment[16];

} proci_linearizationtable_t17_config_t;

typedef struct
{
	/* GPIOPWM Configuration */
	u8 ctrl;             /*!< Main configuration field           */
	u8 reportmask;       /*!< Event mask for generating messages to
							   *   the host */
	u8 dir;              /*!< Port DIR register   */
	u8 pullup;           /*!< Port pull-up per pin enable register */
	u8 out;              /*!< Port OUT register*/
	u8 wake;             /*!< Port wake on change enable register  */
	u8 pwm;              /*!< Port pwm enable register    */
	u8 per;              /*!< PWM period (min-max) percentage*/
	u8 duty[4];          /*!< PWM duty cycles percentage */

} spt_gpiopwm_t19_config_t;

typedef struct
{
	u8 ctrl;
	u8 xlogrip;
	u8 xhigrip;
	u8 ylogrip;
	u8 yhigrip;
	u8 maxtchs;
	u8 RESERVED2;
	u8 szthr1;
	u8 szthr2;
	u8 shpthr1;
	u8 shpthr2;
	u8 supextto;
} proci_gripfacesuppression_t20_config_t;

/*! ==PROCG_NOISESUPPRESSION_T22==
  The T22 NOISESUPPRESSION object provides frequency hopping acquire control,
  outlier filtering and grass cut filtering.
 */
/*! \brief 
  This structure is used to configure the NOISESUPPRESSION object and
  should be made accessible to the host controller.
 */

typedef struct
{
	u8 ctrl;                 /* LCMASK ACMASK */
	u8 reserved;
	u8 reserved1;
	u16 gcaful;               /* LCMASK */
	u16 gcafll;               /* LCMASK */
	u8 actvgcafvalid;        /* LCMASK */
	u8 noisethr;             /* LCMASK */
	u8 reserved2;
	u8 freqhopscale;
	u8 freq[5u];             /* LCMASK ACMASK */
	u8 idlegcafvalid;        /* LCMASK */
} procg_noisesuppression_t22_config_t;

/*! ==TOUCH_PROXIMITY_T23==
  The T23 Proximity is a proximity key made of upto 16 channels
 */
/*! \brief
  This structure is used to configure the prox object and should be made
  accessible to the host controller.
 */
typedef struct
{
	/* Prox Configuration */
	u8 ctrl;               /*!< ACENABLE LCENABLE Main configuration field           */

	/* Physical Configuration */
	u8 xorigin;           /*!< ACMASK LCMASK Object x start position on matrix  */
	u8 yorigin;           /*!< ACMASK LCMASK Object y start position on matrix  */
	u8 xsize;             /*!< ACMASK LCMASK Object x size (i.e. width)         */
	u8 ysize;             /*!< ACMASK LCMASK Object y size (i.e. height)        */
	u8 reserved_for_future_aks_usage;
	/* Detection Configuration */
	u8 blen;               /*!< ACMASK Burst length for all object channels*/
	u16 tchthr;             /*!< LCMASK Threshold    */
	u8 tchdi;              /*!< Detect integration config           */
	u8 average;            /*!< LCMASK Sets the filter length on the prox signal */
	u16 rate;               /*!< Sets the rate that prox signal must exceed */

} touch_proximity_t23_config_t;

typedef struct
{
	u8  ctrl;
	u8  numgest;
	u16 gesten;
	u8  pressproc;
	u8  tapto;
	u8  flickto;
	u8  dragto;
	u8  spressto;
	u8  lpressto;
	u8  rptpressto;
	u16 flickthr;
	u16 dragthr;
	u16 tapthr;
	u16 throwthr;
} proci_onetouchgestureprocessor_t24_config_t;

typedef struct
{
	u8  ctrl;
	u8  cmd;
	u16 upsiglim;
	u16 losiglim;
} spt_selftest_t25_config_t;

#ifdef MULTITOUCH_ENABLE_CHJ   // for multitouch enable
typedef struct
{
	u16 upsiglim;              /* LCMASK */
	u16 losiglim;              /* LCMASK */

} siglim_t;

/*! = Config Structure = */

typedef struct
{
	u8  ctrl;                 /* LCENABLE */
	u8  cmd;
#if(NUM_OF_TOUCH_OBJECTS)
	siglim_t siglim[NUM_OF_TOUCH_OBJECTS];   /* LCMASK */
#endif

} spt_selftest_t25_config_t;
#endif

typedef struct
{
	u8  ctrl;          /*!< Bit 0 = object enable, bit 1 = report enable */
	u8  numgest;       /*!< Runtime control for how many two touch gestures
							 *   to process */
	u8 reserved;
	u8 gesten;        /*!< Control for turning particular gestures on or
							*  off */
	u8  rotatethr;
	u16 zoomthr;

} proci_twotouchgestureprocessor_t27_config_t;

/*! ==SPT_CTECONFIG_T28==
  The T28 CTECONFIG object provides controls for the CTE.
 */

/*! \brief 
  This structure is used to configure the CTECONFIG object and
  should be made accessible to the host controller.
 */

typedef struct
{
	u8 ctrl;          /*!< Ctrl field reserved for future expansion */
	u8 cmd;           /*!< Cmd field for sending CTE commands */
	u8 mode;          /*!< LCMASK CTE mode configuration field */
	u8 idlegcafdepth; /*!< LCMASK The global gcaf number of averages when idle */
	u8 actvgcafdepth; /*!< LCMASK The global gcaf number of averages when active */
	u8 voltage;
} spt_cteconfig_t28_config_t;

typedef struct
{
	char * config_name;
	gen_powerconfig_t7_config_t power_config;
	gen_acquisitionconfig_t8_config_t acquisition_config;
	touch_multitouchscreen_t9_config_t touchscreen_config;
	touch_keyarray_t15_config_t keyarray_config;
	proci_gripfacesuppression_t20_config_t gripfacesuppression_config;
	proci_linearizationtable_t17_config_t linearization_config;
	spt_selftest_t25_config_t selftest_config;
	procg_noisesuppression_t22_config_t noise_suppression_config;
	proci_onetouchgestureprocessor_t24_config_t onetouch_gesture_config;
	proci_twotouchgestureprocessor_t27_config_t twotouch_gesture_config;
	spt_cteconfig_t28_config_t cte_config;
	touch_proximity_t23_config_t proximity_config;
} all_config_setting;

typedef struct
{
	int x;
	int y;
	int press;
} dec_input;

typedef struct
{
	u8 object_type;     /*!< Object type ID. */
	u16 i2c_address;    /*!< Start address of the obj config structure. */
	u8 size;            /*!< Byte length of the obj config structure -1.*/
	u8 instances;       /*!< Number of objects of this obj. type -1. */
	u8 num_report_ids;  /*!< The max number of touches in a screen,
							  *  max number of sliders in a slider array, etc.*/
} object_t;

typedef struct
{
	u8 family_id;            /* address 0 */
	u8 variant_id;           /* address 1 */

	u8 version;              /* address 2 */
	u8 build;                /* address 3 */

	u8 matrix_x_size;        /* address 4 */
	u8 matrix_y_size;        /* address 5 */

	u8 num_declared_objects; /* address 6 */
} info_id_t;

typedef struct
{
	/*! Info ID struct. */
	info_id_t info_id;

	/*! Pointer to an array of objects. */
	object_t *objects;

	/*! CRC field, low bytes. */
	u16 CRC;

	/*! CRC field, high byte. */
	u8 CRC_hi;
} info_block_t;

typedef struct
{
	u8 object_type;         /*!< Object type. */
	u8 instance;        /*!< Instance number. */
} report_id_map_t;

#if 0
#define RESET_BIT        0x80   /* reset bit in general_status_1 */
#define SL3_DET          0x08   /* slider 3 detect bit in general_status_1 */
#define SL4_DET          0x10   /* slider 4 detect bit in general_status_1 */
#define TS0_DET          0x01   /* touchscreen 0 detect bit in general_status_2 */
#define TS1_DET          0x02   /* touchscreen 1 detect bit in general_status_2 */
#define BACKUP_CODE      0x55   /* command code to force a backup to eeprom */
#define CALIBRATE_CODE   0x01   /* command code to force a device calibration */

#define QT5480_GENERAL_STATUS_1_SL0_DET		(0x1 < 0)
#define QT5480_GENERAL_STATUS_1_SL1_DET		(0x1 < 1)
#define QT5480_GENERAL_STATUS_1_SL2_DET		(0x1 < 2)
#define QT5480_GENERAL_STATUS_1_SL3_DET		(0x1 < 3)
#define QT5480_GENERAL_STATUS_1_SL4_DET		(0x1 < 4)
#define QT5480_GENERAL_STATUS_1_SL5_DET		(0x1 < 5)
#define QT5480_GENERAL_STATUS_1_CYCLE_OVERRUN		(0x1 < 6)
#define QT5480_GENERAL_STATUS_1_RESET		(0x1 < 7)

#define QT5480_GENERAL_STATUS_2_TS0_DET		(0x1 < 0)
#define QT5480_GENERAL_STATUS_2_TS1_DET		(0x1 < 1)
#define QT5480_GENERAL_STATUS_2_TMT		(0x1 < 4)
#define QT5480_GENERAL_STATUS_2_ERR		(0x1 < 5)
#define QT5480_GENERAL_STATUS_2_CAL		(0x1 < 6)
#define QT5480_GENERAL_STATUS_2_TSCR_DET		(0x1 < 7)

enum {	/* QT5xx0 registers */
   QT5480_CHIP_ID = 0,          QT5480_CODE_VERSION,          QT5480_CALIBRATE,
   QT5480_RESET,                QT5480_BACKUP_REQUEST,        QT5480_ADDRESS_PTR,
   QT5480_EEPROM_CHKSUM,        QT5480_KEY_STATUS_1 = 8,      QT5480_GENERAL_STATUS_1 = 14,
   QT5480_GENERAL_STATUS_2,     QT5480_TOUCHSCR_1_X,          QT5480_TOUCHSCR_1_Y = 18,
   QT5480_TOUCHSCR_2_X = 20,    QT5480_SLIDER_0 = 20,         QT5480_TOUCHSCR_2_Y = 22,
   QT5480_SLIDER_4 = 24,        QT5480_FORCE_SNS = 26,        QT5480_KEY_GATE_STATUS,
   QT5480_RESERVED_1,           QT5480_BUILD_VERSION = 37,	  QT5480_CHAN_1_DELTA = 256,    QT5480_CHAN_1_REF = 352,
   QT5480_RESERVED_2 = 448,     QT5480_KEY_CONTROL = 512,     QT5480_THRESHOLD = 560,
   QT5480_BL = 608,             QT5480_LP_MODE = 656,         QT5480_MIN_CYC_TIME,
   QT5480_AWAKE_TIMEOUT,        QT5480_TRIGGER_CONTROL,       QT5480_GUARD_KEY_ENABLE,
   QT5480_TOUCHSCR_SETUP,       QT5480_TOUCHSCR_LEN,          QT5480_SLIDER_1_LEN = 662,
   QT5480_TOUCHSCR_HYST = 668,  QT5480_SLIDER_1_HYST = 668,   QT5480_GPO_CONTROL = 674,
   QT5480_NDRIFT,               QT5480_PDRIFT,				  QT5480_NDIL,
   QT5480_SDIL,                 QT5480_NRD,                   QT5480_DHT,
   QT5480_FORCE_THRESH,         QT5480_LIN_OFFSET_X = 684,    QT5480_LIN_TABLE_X = 686,
   QT5480_LIN_OFFSET_Y = 702,   QT5480_LIN_TABLE_Y = 704,     QT5480_BURST_CONTROL = 720, QT5480_RESERVED_END =747,
   QT5480_UNFORCED = 0xffff
};

struct qt_tscrn_touch_t {
   /* touch-screen touch packet information */
   uint16_t x;
   uint16_t y;
   uint8_t size;
   uint8_t area;
};

struct qt5480_status_t {
   /* dynamic device status */
   uint8_t updated;                     /* flag used to indicate new status data is available */
   uint8_t general_status_1;            /* values extracted from status packets */   
   uint8_t general_status_2;   
   uint8_t key[6];
   uint8_t slider[6];
   struct qt_tscrn_touch_t touchscr[2];
};
#endif
#endif /* _ATMEL_AT42QT5480_H */
