/*=================================================================================================
 
    Module Name: crucialtec_oj_touch_full.c
 
    DESCRIPTION: Linux driver for Crucialtec Optical Joystick
 
==================================================================================================*/

/*==================================================================================================
                                           INCLUDE FILES
==================================================================================================*/
#include <linux/module.h>
#include <linux/device.h>
#include <linux/sysdev.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/hrtimer.h>
#include <linux/platform_device.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/clk.h>
#include <linux/irq.h>
#include <linux/sysfs.h>
#include <linux/debugfs.h>
#include <linux/hwmon-sysfs.h>
#include <linux/gpio_event.h>
#include <linux/miscdevice.h>
//#include <asm/gpio.h>
#include <mach/gpio.h>
#include <mach/mfp-pxa9xx.h>
//#include <asm/io.h>
#include <linux/io.h>
#include <asm/uaccess.h>
#include <linux/earlysuspend.h>
#ifdef CONFIG_CRUCIALTEC_OJ_ESD_RECOVERY
#include <mot/esd_poll.h>
#endif
//#include <mot/crucialtec_oj.h>
#include <linux/workqueue.h>
#include <linux/sched.h>
#include <linux/timer.h>
#include <linux/jiffies.h>
#include <plat/mfp.h>


/*==================================================================================================
                                          GLOBAL VARIABLES
==================================================================================================*/


/*==================================================================================================
                                          LOCAL CONSTANTS
==================================================================================================*/
#define CRU_MOUSE_MODE 0
#define CRU_KEY_MODE 1
#define CRU_TEN_MODE 2
#define CRU_ELEVEN_MODE 3
#define CRU_TWELVE_MODE 4

#define CRU_TEN_VAL 10
#define CRU_ELEVEN_VAL 20
#define CRU_TWELVE_VAL 15

#define KEY_ZOOM_IN 241
#define KEY_ZOOM_OUT 242

#define CENTER_X 400
#define CENTER_Y 240

/* power control */
#define ON      1
#define OFF     0

#define OTP_SENSITIVITY_X1  90//100
#define OTP_SENSITIVITY_X2  80//95
#define OTP_SENSITIVITY_X3  70//90
#define OTP_SENSITIVITY_X4  60//85
#define OTP_SENSITIVITY_X5  50//80
#define OTP_SENSITIVITY_X6  45//75
#define OTP_SENSITIVITY_X7  40//70
#define OTP_SENSITIVITY_X8  35//65
#define OTP_SENSITIVITY_X9  30//60

#define OTP_SENSITIVITY_X_DEFAULT  4

#define OTP_SENSITIVITY_Y1  90//100
#define OTP_SENSITIVITY_Y2  80//95
#define OTP_SENSITIVITY_Y3  70//90
#define OTP_SENSITIVITY_Y4  60//85
#define OTP_SENSITIVITY_Y5  50//80
#define OTP_SENSITIVITY_Y6  45//75
#define OTP_SENSITIVITY_Y7  40//70
#define OTP_SENSITIVITY_Y8  35//65
#define OTP_SENSITIVITY_Y9  30//60

#define OTP_SENSITIVITY_Y_DEFAULT  4

#define OTP_SENSITIVITY_MAX_LEVEL   9

static int gv_otp_Xsensitivity[OTP_SENSITIVITY_MAX_LEVEL]=
{
    OTP_SENSITIVITY_X1,
    OTP_SENSITIVITY_X2,
    OTP_SENSITIVITY_X3,
    OTP_SENSITIVITY_X4,
    OTP_SENSITIVITY_X5,
    OTP_SENSITIVITY_X6,
    OTP_SENSITIVITY_X7,
    OTP_SENSITIVITY_X8,
    OTP_SENSITIVITY_X9,
};

static int gv_otp_Ysensitivity[OTP_SENSITIVITY_MAX_LEVEL]=
{
    OTP_SENSITIVITY_Y1,
    OTP_SENSITIVITY_Y2,
    OTP_SENSITIVITY_Y3,
    OTP_SENSITIVITY_Y4,
    OTP_SENSITIVITY_Y5,
    OTP_SENSITIVITY_Y6,
    OTP_SENSITIVITY_Y7,
    OTP_SENSITIVITY_Y8,
    OTP_SENSITIVITY_Y9,
};


#define REVERSE_DIRECTION -1
#define OTP_SCHEDULE_DELAYED_WORK 1

// Support for Misc device
//    Enables the confiuguration of Mode, Valid Delta, and Key Press Delay via adb shell
#define CRUCIALTEC_OJ_MISC_DEVICE_SUPPORT 0     
#define CRUCIALTEC_OJ_MISC_TEMP_BUFFER_SIZE 900
#define CRUCIALTEC_OJ_MISC_DEVICE_WRITE_PARAMETER_LIMIT   16

#define CRUCIALTEC_OJ_PRODUCT_NAME    "Optical_Joystick"
#define CRUCIALTEC_VENDOR_ID   0x0100
#define CRUCIALTEC_OJ_MOTION_GPIO_IRQ_REQUEST_FLAG    IRQF_TRIGGER_LOW

#define CRUCIALTEC_OJ_DELAY_RESET_LINE_FROM_LOW_TO_HIGH            20  // in usec
#define CRUCIALTEC_OJ_DELAY_FOR_SYSTEM_TO_BE_READY                 15  // in msec
#define CRUCIALTEC_OJ_DELAY_FOR_SYSTEM_SELF_TEST                  250  // in msec
#define CRUCIALTEC_OJ_DELAY_FOR_SYSTEM_TO_BE_READY_AFTER_SUSPEND  100  // in msec
#define CRUCIALTEC_OJ_DELAY_FOR_SYSTEM_TO_BE_READY_AFTER_SOFT_RESET  100 //in msec

// If the placement of the OJ is rotated clockwise by 90 degrees then swap the x and y data
#define CRUCIALTEC_OJ_ORIENTATION_POSITION_SWAP_X_TO_Y   1
#define CRUCIALTEC_OJ_PERFORM_SELF_TEST_DURING_INIT      0

#define DPAD_UP_KEY   108 //103 // 22    // DPAD's values are from the optical_joystick.kl file
#define DPAD_DOWN_KEY  103 //108 //24 
#define DPAD_RIGHT_KEY 106 //13 
#define DPAD_LEFT_KEY  105  //33 

// For Directional Pad Mode.  The distance traveled must be greater then this to be a valid input.
#define CRUCIALTEC_OJ_MAX_DISTANCE_TRAVELED  15 
#define CRUCIALTEC_OJ_SENSITIVITY_LOW      30 
#define CRUCIALTEC_OJ_SENSITIVITY_MEDIUM   CRUCIALTEC_OJ_MAX_DISTANCE_TRAVELED 
#define CRUCIALTEC_OJ_SENSITIVITY_HIGH     5

// For Directional Pad Mode. Delay between key press (in ms)
#define CRUCIALTEC_OJ_MAX_DELAY_BETWEEN_KEY  65 

#define CRUCIALTEC_OJ_XMIN_NOMINAL  -128           // X-axis min and max values
#define CRUCIALTEC_OJ_XMAX_NOMINAL  127
#define CRUCIALTEC_OJ_YMIN_NOMINAL  -128           // Y-axis min and max values
#define CRUCIALTEC_OJ_YMAX_NOMINAL  127

#define FPD_POLL_TIMER ((1*HZ)/5)                  // Finger Present Detection Polling Timer
#define FPD_POLL_LTIME ((10*HZ))
#define FPD_START_TIMER ((10*HZ))    

#define CRUCIALTEC_OJ_VALID_SQUAL    20            // FPD's SQUAL value threshold
#define CRUCIALTEC_OJ_OUTDOOR_LIGHT_LEVEL 3        // ALS's value for outdoor lighting
#define CRUCIALTEC_OJ_JITTER_CHECK  1              // Jitter/Ghost check: 0=OFF, 1=ON

// Crucialtec's 4-way Nav Code settings
#define MS_TO_NS_FACTOR  1000000  //in ns, equals to 1ms

#define DELTA_SUM_TIME    200 // ms
#define DELTA_SUM_CP       50//35
#define SCROLL_LEFT      1
#define SCROLL_RIGHT    2
#define SCROLL_UP          3
#define SCROLL_DOWN     4

#define AUTO_SCROLL_VALUE     20
#define DELTA_XY_CP       2

#define DELTA_MIN_IN       50  
#define DELTA_MIN_OUT     100  

#define KEY_ACCEL_VALUE   ((DELTA_MIN_IN)-10)
#define ACCEL_FACTOR      5

#define CT_4_WAY_NAV_POLLING_RATE  20  // ms
#define CT_4_WAY_NUM_OF_POLL       80  // The total polling duration equals to polling rate * num of poll

// Sensitivity High
#define OJ_SENSITIVITY_MIN_IN_X_HIGH    40
#define OJ_SENSITIVITY_MIN_IN_Y_HIGH    40
#define OJ_SENSITIVITY_MIN_OUT_X_HIGH   40
#define OJ_SENSITIVITY_MIN_OUT_Y_HIGH   40
#define OJ_SENSITIVITY_DELAY_BETWEEN_KEYS_HIGH  0
#if OTP_SCHEDULE_DELAYED_WORK
#define OJ_SENSITIVITY_SUM_TIME_HIGH   200
#else
#define OJ_SENSITIVITY_SUM_TIME_HIGH   25
#endif


// Sensitivity Medium
#define OJ_SENSITIVITY_MIN_IN_X_MEDIUM    80
#define OJ_SENSITIVITY_MIN_IN_Y_MEDIUM    80
#define OJ_SENSITIVITY_MIN_OUT_X_MEDIUM   80
#define OJ_SENSITIVITY_MIN_OUT_Y_MEDIUM   80
#define OJ_SENSITIVITY_DELAY_BETWEEN_KEYS_MEDIUM 0
#if OTP_SCHEDULE_DELAYED_WORK
#define OJ_SENSITIVITY_SUM_TIME_MEDIUM   200
#else
#define OJ_SENSITIVITY_SUM_TIME_MEDIUM   25
#endif

// Sensitivity Low
#define OJ_SENSITIVITY_MIN_IN_X_LOW   120
#define OJ_SENSITIVITY_MIN_IN_Y_LOW   120
#define OJ_SENSITIVITY_MIN_OUT_X_LOW  120
#define OJ_SENSITIVITY_MIN_OUT_Y_LOW  120
#define OJ_SENSITIVITY_DELAY_BETWEEN_KEYS_LOW 0
#if OTP_SCHEDULE_DELAYED_WORK
#define OJ_SENSITIVITY_SUM_TIME_LOW   200
#else
#define OJ_SENSITIVITY_SUM_TIME_LOW   25
#endif


// For Directional Pad Mode. Delay between OJ movement and TSB keys (in ms)
#define CRUCIALTEC_OJ_MAX_DELAY_BETWEEN_OJ_AND_TSB_KEYS  300
// Delay of the DPAD directional keys after a DPAD_OK pressed (in ms)
#define CRUCIALTEC_OJ_DELAY_BETWEEN_OJ_DPAD_DIRECTIONAL_KEYS_AND_OJ_DOME_KEY   400
#define CHECK_AND_ASSIGN(a, b)  (a = (b >= 0) ? b : a)

/******************************************************************************
*                  Crucial OJ's Registers and Defines
******************************************************************************/

/* Product ID
 *      =========================================================================================
 *Bit   |     7    |     6    |     5    |     4    |     3    |     2    |     1    |     0    |
 *      =========================================================================================
 *Field |   PID7   |   PID6   |   PID5   |   PID4   |   PID3   |   PID2   |   PID1   |   PID0   |
 *      =========================================================================================
 */
#define CRUCIALTEC_OJ_REG_PRODUCT_ID              0x00


/* Revision ID
 *      =========================================================================================
 *Bit   |     7    |     6    |     5    |     4    |     3    |     2    |     1    |     0    |
 *      =========================================================================================
 *Field |   RID7   |   RID6   |   RID5   |   RID4   |   RID3   |   RID2   |   RID1   |   RID0   |
 *      =========================================================================================
 */
#define CRUCIALTEC_OJ_REG_REVISION_ID             0x01


/* Motion Register
 *       =========================================================================================
 * Bit   |     7    |     6    |     5    |     4    |     3    |     2    |     1    |     0    |
 *       =========================================================================================
 * Field |    MOT   |  PIXRDY  | PIXFIRST |    OVF   | Reserved | Reserved | Reserved |   GPIO   |
 *       =========================================================================================
 */
#define CRUCIALTEC_OJ_REG_MOTION                  0x02
#define CRUCIALTEC_OJ_REG_MOTION_VALUE_MOTION_MASK                         0x80   // Motion Detection
#define CRUCIALTEC_OJ_REG_MOTION_VALUE_MOTION_NO_MOTION                    0x00
#define CRUCIALTEC_OJ_REG_MOTION_VALUE_MOTION_OCCURRED                     0x80
#define CRUCIALTEC_OJ_REG_MOTION_VALUE_PIXRDY_MASK                         0x40   // Pixel Dump Reg is Ready
#define CRUCIALTEC_OJ_REG_MOTION_VALUE_PIXRDY_DATA_NOT_AVAILABLE           0x00
#define CRUCIALTEC_OJ_REG_MOTION_VALUE_PIXRDY_DATA_AVAILABLE               0x40
#define CRUCIALTEC_OJ_REG_MOTION_VALUE_PIXFIRST_MASK                       0x20   // Pixel Grab Reg is Ready
#define CRUCIALTEC_OJ_REG_MOTION_VALUE_PIXFIRST_NOT_FROM_PIXEL_ZERO_ZERO   0x00
#define CRUCIALTEC_OJ_REG_MOTION_VALUE_PIXFIRST_IS_FROM_PIXEL_ZERO_ZER0    0x20
#define CRUCIALTEC_OJ_REG_MOTION_VALUE_MOTION_OVERFLOW_MASK                0x10   // Motion Overflow
#define CRUCIALTEC_OJ_REG_MOTION_VALUE_MOTION_OVERFLOW_NO_OVERFLOW         0x00
#define CRUCIALTEC_OJ_REG_MOTION_VALUE_MOTION_OVERFLOW_OCCURRED            0x10
#define CRUCIALTEC_OJ_REG_MOTION_VALUE_MOTION_GPIO_STATUS_MASK             0x01   // Motion's GPIO Status
#define CRUCIALTEC_OJ_REG_MOTION_VALUE_MOTION_GPIO_STATUS_LOW              0x00
#define CRUCIALTEC_OJ_REG_MOTION_VALUE_MOTION_GPIO_STATUS_HIGH             0x01


/* X Movement is counts since last report.  Eight bit 2's complement number
 *      =========================================================================================
 *Bit   |     7    |     6    |     5    |     4    |     3    |     2    |     1    |     0    |
 *      =========================================================================================
 *Field |    X7    |    X6    |    X5    |    X4    |    X3    |    X2    |    X1    |    X0    |
 *      =========================================================================================
 */
#define CRUCIALTEC_OJ_REG_DELTA_X                 0x03


/* Y Movement is counts since last report.  Eight bit 2's complement number
 *      =========================================================================================
 *Bit   |     7    |     6    |     5    |     4    |     3    |     2    |     1    |     0    |
 *      =========================================================================================
 *Field |    Y7    |    Y6    |    Y5    |    Y4    |    Y3    |    Y2    |    Y1    |    Y0    |
 *      =========================================================================================
 */
#define CRUCIALTEC_OJ_REG_DELTA_Y                 0x04


#define CRUCIALTEC_OJ_REG_SQUAL                   0x05
#define CRUCIALTEC_OJ_REG_SHUTTER_UPPER           0x06
#define CRUCIALTEC_OJ_REG_SHUTTER_LOWER           0x07
#define CRUCIALTEC_OJ_REG_MAX_PIXEL               0x08
#define CRUCIALTEC_OJ_REG_PIXEL_SUM               0x09
#define CRUCIALTEC_OJ_REG_MIN_PIXEL               0x0A
#define CRUCIALTEC_OJ_REG_PIXEL_GRAB              0x0B


/* Self Test CRC Registers
 *      =========================================================================================
 *Bit   |     7    |     6    |     5    |     4    |     3    |     2    |     1    |     0    |
 *      =========================================================================================
 *Field |   CRC7   |   CRC6   |   CRC5   |   CRC4   |   CRC3   |   CRC2   |   CRC1   |   CRC0   |
 *      =========================================================================================
 */
#define CRUCIALTEC_OJ_REG_CRC0                    0x0C
#define CRUCIALTEC_OJ_REG_CRC0_SELF_TEST_VALID_VALUE    0x33

#define CRUCIALTEC_OJ_REG_CRC1                    0x0D
#define CRUCIALTEC_OJ_REG_CRC1_SELF_TEST_VALID_VALUE    0x8E

#define CRUCIALTEC_OJ_REG_CRC2                    0x0E
#define CRUCIALTEC_OJ_REG_CRC2_SELF_TEST_VALID_VALUE    0x24

#define CRUCIALTEC_OJ_REG_CRC3                    0x0F
#define CRUCIALTEC_OJ_REG_CRC3_SELF_TEST_VALID_VALUE    0x6C


/* Self Test Register
 *      =========================================================================================
 *Bit   |     7    |     6    |     5    |     4    |     3    |     2    |     1    |     0    |
 *      =========================================================================================
 *Field | Reserved | Reserved | Reserved | Reserved | Reserved | Reserved | Reserved |  TESTEN  |
 *      =========================================================================================
 */
#define CRUCIALTEC_OJ_REG_SELF_TEST               0x10
#define CRUCIALTEC_OJ_REG_SELF_TEST_VALUE_TEST_ENABLE_MASK    0x01    // Test Mode
#define CRUCIALTEC_OJ_REG_SELF_TEST_VALUE_TEST_ENABLE_OFF     0x00
#define CRUCIALTEC_OJ_REG_SELF_TEST_VALUE_TEST_ENABLE_ON      0x01


/* Configuration Bits Register - to set the sensor resolution
 *      =========================================================================================
 *Bit   |     7    |     6    |     5    |     4    |     3    |     2    |     1    |     0    |
 *      =========================================================================================
 *Field |Resolution| Reserved | Reserved | Reserved | Reserved | Reserved | Reserved | Reserved |
 *      =========================================================================================
 */
#define CRUCIALTEC_OJ_REG_CONFIG_BITS             0x11
#define CRUCIALTEC_OJ_REG_CONFIG_BITS_VALUE_RESOLUTION_MASK     0x80   // Sensor Resolution
#define CRUCIALTEC_OJ_REG_CONFIG_BITS_VALUE_RESOLUTION_500_CPI  0x00
#define CRUCIALTEC_OJ_REG_CONFIG_BITS_VALUE_RESOLUTION_1000_CPI 0x80


/* LED Control Register 
 *      =========================================================================================
 *Bit   |     7    |     6    |     5    |     4    |     3    |     2    |     1    |     0    |
 *      =========================================================================================
 *Field | Reserved | Reserved | Reserved | Reserved |   LED3   | Reserved | Reserved | Reserved |
 *      =========================================================================================
 */
#define CRUCIALTEC_OJ_REG_LED_CONTROL             0x1A
#define CRUCIALTEC_OJ_REG_LED_CONTROL_VALUE_LED_MASK              0x08  // LED Control Mode
#define CRUCIALTEC_OJ_REG_LED_CONTROL_VALUE_LED_NORMAL_OPERATION  0x00
#define CRUCIALTEC_OJ_REG_LED_CONTROL_VALUE_LED_CONTINUOUS_ON     0x08


/* IO Mode Register 
 *      =========================================================================================
 *Bit   |     7    |     6    |     5    |     4    |     3    |     2    |     1    |     0    |
 *      =========================================================================================
 *Field | Reserved | Reserved | Reserved |   Burst  | Reserved |    SPI   | Reserved |   TWI    |
 *      =========================================================================================
 */
#define CRUCIALTEC_OJ_REG_IO_MODE                 0x1C
#define CRUCIALTEC_OJ_REG_IO_MODE_VALUE_BURST_MODE_MASK      0x10     // Burst Mode
#define CRUCIALTEC_OJ_REG_IO_MODE_VALUE_BURST_MODE_OFF       0x00
#define CRUCIALTEC_OJ_REG_IO_MODE_VALUE_BURST_MODE_ON        0x10
#define CRUCIALTEC_OJ_REG_IO_MODE_VALUE_SPI_MODE_MASK        0x04     // SPI Mode
#define CRUCIALTEC_OJ_REG_IO_MODE_VALUE_SPI_MODE_OFF         0x00
#define CRUCIALTEC_OJ_REG_IO_MODE_VALUE_SPI_MODE_ON          0x04
#define CRUCIALTEC_OJ_REG_IO_MODE_VALUE_TWI_MODE_MASK        0x01     // TWI Mode
#define CRUCIALTEC_OJ_REG_IO_MODE_VALUE_TWI_MODE_OFF         0x00
#define CRUCIALTEC_OJ_REG_IO_MODE_VALUE_TWI_MODE_ON          0x01


/* Observation Register
 *      =========================================================================================
 *Bit   |     7    |     6    |     5    |     4    |     3    |     2    |     1    |     0    |
 *      =========================================================================================
 *Field |  MODE1   |  MODE0   | Reserved | Reserved | Reserved | Reserved | Reserved | Reserved |
 *      =========================================================================================
 */
#define CRUCIALTEC_OJ_REG_OBSERVATION             0x2E
#define CRUCIALTEC_OJ_REG_OBSERVATION_VALUE_MODE_MASK       0xC0   // Observation Mode
#define CRUCIALTEC_OJ_REG_OBSERVATION_VALUE_RUN_MODE        0x00
#define CRUCIALTEC_OJ_REG_OBSERVATION_VALUE_RESET1_MODE     0x40
#define CRUCIALTEC_OJ_REG_OBSERVATION_VALUE_RESET2_MODE     0x80
#define CRUCIALTEC_OJ_REG_OBSERVATION_VALUE_RESET3_MODE     0xC0


/* Soft Reset Register
 *      =========================================================================================
 *Bit   |     7    |     6    |     5    |     4    |     3    |     2    |     1    |     0    |
 *      =========================================================================================
 *Field |   RST    |   RST    |   RST    |   RST    |   RST    |   RST    |   RST    |   RST    |
 *      =========================================================================================
 */
#define CRUCIALTEC_OJ_REG_SOFT_RESET              0x3A
#define CRUCIALTEC_OJ_REG_SOFT_RESET_VALUE_REVERT_TO_DEFAULT  0x5A  // Soft Reset value


/* Shutter maximum open time - upper and lower 8-bit
 *      =========================================================================================
 *Bit   |     7    |     6    |     5    |     4    |     3    |     2    |     1    |     0    |
 *      =========================================================================================
 *Field |    SM7   |    SM6   |    SM5   |    SM4   |    SM3   |    SM2   |    SM1   |    SM0   |
 *      =========================================================================================
 */
#define CRUCIALTEC_OJ_REG_SHUTTER_MAX_HI          0x3B
#define CRUCIALTEC_OJ_REG_SHUTTER_MAX_LO          0x3C


/* Inverse Revision ID
 *      =========================================================================================
 *Bit   |     7    |     6    |     5    |     4    |     3    |     2    |     1    |     0    |
 *      =========================================================================================
 *Field |   NRID7  |   NRID6  |   NRID5  |   NRID4  |   NRID3  |   NRID2  |   NRID1  |   NRID0  |
 *      =========================================================================================
 */
#define CRUCIALTEC_OJ_REG_INVERSE_REVISION_ID     0x3E


/* Inverse Product ID
 *      =========================================================================================
 *Bit   |     7    |     6    |     5    |     4    |     3    |     2    |     1    |     0    |
 *      =========================================================================================
 *Field |   NPID7  |   NPID6  |   NPID5  |   NPID4  |   NPID3  |   NPID2  |   NPID1  |   NPID0  |
 *      =========================================================================================
 */
#define CRUCIALTEC_OJ_REG_INVERSE_PRODUCT_ID      0x3F


/* Engine Register
 *      =========================================================================================
 *Bit   |     7    |     6    |     5    |     4    |     3    |     2    |     1    |     0    |
 *      =========================================================================================
 *Field |  Engine  |   Speed  |  Assert/ |   XY Q   | Reserved |  Finger  | XY_Scale | Reserved |
 *      |          |          | DeAssert |          |          |          |          |          |
 *      =========================================================================================
 */
#define CRUCIALTEC_OJ_REG_ENGINE              0x60
#define CRUCIALTEC_OJ_REG_ENGINE_VALUE_ENGINE_MASK    0x80      // Engine Properties
#define CRUCIALTEC_OJ_REG_ENGINE_VALUE_ENGINE_OFF     0x00
#define CRUCIALTEC_OJ_REG_ENGINE_VALUE_ENGINE_ON      0x80
#define CRUCIALTEC_OJ_REG_ENGINE_VALUE_SPEED_MASK     0x40      // Speed Switching
#define CRUCIALTEC_OJ_REG_ENGINE_VALUE_SPEED_OFF      0x00
#define CRUCIALTEC_OJ_REG_ENGINE_VALUE_SPEED_ON       0x40
#define CRUCIALTEC_OJ_REG_ENGINE_VALUE_ASSERT_MASK    0x20      // Assert and Deassert
#define CRUCIALTEC_OJ_REG_ENGINE_VALUE_ASSERT_OFF     0x00
#define CRUCIALTEC_OJ_REG_ENGINE_VALUE_ASSERT_ON      0x20
#define CRUCIALTEC_OJ_REG_ENGINE_VALUE_XYQ_MASK       0x10      // XY Quantization
#define CRUCIALTEC_OJ_REG_ENGINE_VALUE_XYQ_OFF        0x00
#define CRUCIALTEC_OJ_REG_ENGINE_VALUE_XYQ_ON         0x10
#define CRUCIALTEC_OJ_REG_ENGINE_VALUE_FINGER_MASK    0x04      // Finger Present Detection
#define CRUCIALTEC_OJ_REG_ENGINE_VALUE_FINGER_OFF     0x00
#define CRUCIALTEC_OJ_REG_ENGINE_VALUE_FINGER_ON      0x04
#define CRUCIALTEC_OJ_REG_ENGINE_VALUE_XY_SCALE_MASK  0x02      // XY Scaling Factor
#define CRUCIALTEC_OJ_REG_ENGINE_VALUE_XY_SCALE_OFF   0x00
#define CRUCIALTEC_OJ_REG_ENGINE_VALUE_XY_SCALE_ON    0x02
#define CRUCIALTEC_OJ_REG_ENGINE_VALUE_DEFAULT         \
       (CRUCIALTEC_OJ_REG_ENGINE_VALUE_ENGINE_ON | CRUCIALTEC_OJ_REG_ENGINE_VALUE_SPEED_ON)

/* Resolution Register
 *      =========================================================================================
 *Bit   |     7    |     6    |     5    |     4    |     3    |     2    |     1    |     0    |
 *      =========================================================================================
 *Field | Reserved | Reserved | WakeRES2 | WakeRES1 | WakeRES0 |   RES2   |   RES1   |   RES0   |
 *      =========================================================================================
 */
#define CRUCIALTEC_OJ_REG_RESOLUTION          0x62
// Sets resolution when sensor wakes up from rest modes.
// Effective if speed switching is enabled.
#define CRUCIALTEC_OJ_REG_RESOLUTION_VALUE_WAKEUP_RES_MASK      0x38  // Wakeup resolution
#define CRUCIALTEC_OJ_REG_RESOLUTION_VALUE_WAKEUP_RES_250_CPI   0x08
#define CRUCIALTEC_OJ_REG_RESOLUTION_VALUE_WAKEUP_RES_500_CPI   0x10
#define CRUCIALTEC_OJ_REG_RESOLUTION_VALUE_WAKEUP_RES_750_CPI   0x18
#define CRUCIALTEC_OJ_REG_RESOLUTION_VALUE_WAKEUP_RES_1000_CPI  0x20
#define CRUCIALTEC_OJ_REG_RESOLUTION_VALUE_WAKEUP_RES_1250_CPI  0x28
// Sets resolution for sensor
#define CRUCIALTEC_OJ_REG_RESOLUTION_VALUE_RES_MASK             0x07  // Resolution of sensor
#define CRUCIALTEC_OJ_REG_RESOLUTION_VALUE_RES_250_CPI          0x01
#define CRUCIALTEC_OJ_REG_RESOLUTION_VALUE_RES_500_CPI          0x02
#define CRUCIALTEC_OJ_REG_RESOLUTION_VALUE_RES_750_CPI          0x03
#define CRUCIALTEC_OJ_REG_RESOLUTION_VALUE_RES_1000_CPI         0x04
#define CRUCIALTEC_OJ_REG_RESOLUTION_VALUE_RES_1250_CPI         0x05


/* Speed Control Register
 *      =========================================================================================
 *Bit   |     7    |     6    |     5    |     4    |     3    |     2    |     1    |     0    |
 *      =========================================================================================
 *Field | Y_scale  | X_scale  | Reserved | Reserved |SP_IntVal1|SP_IntVal0| Low_cpi  | Low_cpi  |
 *      =========================================================================================
 */
#define CRUCIALTEC_OJ_REG_SPEED_CONTROL       0x63
#define CRUCIALTEC_OJ_REG_SPEED_CONTROL_VALUE_Y_SCALE_MASK          0x80   // Y scaling factor
#define CRUCIALTEC_OJ_REG_SPEED_CONTROL_VALUE_Y_SCALE_OFF           0x00
#define CRUCIALTEC_OJ_REG_SPEED_CONTROL_VALUE_Y_SCALE_ON            0x80
#define CRUCIALTEC_OJ_REG_SPEED_CONTROL_VALUE_X_SCALE_MASK          0x40   // X scaling factor
#define CRUCIALTEC_OJ_REG_SPEED_CONTROL_VALUE_X_SCALE_OFF           0x00
#define CRUCIALTEC_OJ_REG_SPEED_CONTROL_VALUE_X_SCALE_ON            0x40
#define CRUCIALTEC_OJ_REG_SPEED_CONTROL_VALUE_SP_INT_VAL_MASK       0x0C   // Speed switching checking interval in ms
#define CRUCIALTEC_OJ_REG_SPEED_CONTROL_VALUE_SP_INT_VAL_4_MS       0x00
#define CRUCIALTEC_OJ_REG_SPEED_CONTROL_VALUE_SP_INT_VAL_8_MS       0x04
#define CRUCIALTEC_OJ_REG_SPEED_CONTROL_VALUE_SP_INT_VAL_12_MS      0x08
#define CRUCIALTEC_OJ_REG_SPEED_CONTROL_VALUE_SP_INT_VAL_16_MS      0x0C
#define CRUCIALTEC_OJ_REG_SPEED_CONTROL_VALUE_LOW_CPI_MASK          0x02   // Sets low CPI when in speed switching
#define CRUCIALTEC_OJ_REG_SPEED_CONTROL_VALUE_LOW_CPI_OFF           0x00
#define CRUCIALTEC_OJ_REG_SPEED_CONTROL_VALUE_LOW_CPI_ON_250_CPI    0x02
#define CRUCIALTEC_OJ_REG_SPEED_CONTROL_VALUE_HIGH_CPI_MASK         0x01   // Sets high CPI when in speed switching
#define CRUCIALTEC_OJ_REG_SPEED_CONTROL_VALUE_HIGH_CPI_OFF          0x00
#define CRUCIALTEC_OJ_REG_SPEED_CONTROL_VALUE_HIGH_CPI_ON_1250_CPI  0x01


/* Speed Switching Properties for the sensor
 *      =========================================================================================
 *Bit   |     7    |     6    |     5    |     4    |     3    |     2    |     1    |     0    |
 *      =========================================================================================
 *Field |    ST    |    ST    |    ST    |    ST    |    ST    |    ST    |    ST    |    ST    |
 *      =========================================================================================
 */
#define CRUCIALTEC_OJ_REG_SPEED_ST12          0x64
#define CRUCIALTEC_OJ_REG_SPEED_ST12_DEFAULT_VALUE  0x08

#define CRUCIALTEC_OJ_REG_SPEED_ST21          0x65
#define CRUCIALTEC_OJ_REG_SPEED_ST21_DEFAULT_VALUE  0x06

#define CRUCIALTEC_OJ_REG_SPEED_ST23          0x66
#define CRUCIALTEC_OJ_REG_SPEED_ST23_DEFAULT_VALUE  0x40

#define CRUCIALTEC_OJ_REG_SPEED_ST32          0x67
#define CRUCIALTEC_OJ_REG_SPEED_ST32_DEFAULT_VALUE  0x08

#define CRUCIALTEC_OJ_REG_SPEED_ST34          0x68
#define CRUCIALTEC_OJ_REG_SPEED_ST34_DEFAULT_VALUE  0x48

#define CRUCIALTEC_OJ_REG_SPEED_ST43          0x69
#define CRUCIALTEC_OJ_REG_SPEED_ST43_DEFAULT_VALUE  0x0A

#define CRUCIALTEC_OJ_REG_SPEED_ST45          0x6A
#define CRUCIALTEC_OJ_REG_SPEED_ST45_DEFAULT_VALUE  0x50

#define CRUCIALTEC_OJ_REG_SPEED_ST54          0x6B
#define CRUCIALTEC_OJ_REG_SPEED_ST54_DEFAULT_VALUE  0x48


/* Assert De-assert Control Register
 *      =========================================================================================
 *Bit   |     7    |     6    |     5    |     4    |     3    |     2    |     1    |     0    |
 *      =========================================================================================
 *Field |     1    |     1    | Reserved | Reserved | Reserved | ST_HIGH2 | ST_HIGH1 | ST_HIGH0 |
 *      =========================================================================================
 */
#define CRUCIALTEC_OJ_REG_AD_CTRL             0x6D
#define CRUCIALTEC_OJ_REG_AD_CTRL_VALUE_MASK        0x07   // Set the Assert De-Assert Control
#define CRUCIALTEC_OJ_REG_AD_CTRL_VALUE_LOWEST_CPI  0xC1
#define CRUCIALTEC_OJ_REG_AD_CTRL_VALUE_LOW_CPI     0xC2
#define CRUCIALTEC_OJ_REG_AD_CTRL_VALUE_MIDDLE_CPI  0xC3
#define CRUCIALTEC_OJ_REG_AD_CTRL_VALUE_HIGHER_CPI  0xC4
#define CRUCIALTEC_OJ_REG_AD_CTRL_VALUE_HIGHEST_CPI 0xC5


/* For setting the HIGH/LOW speed Assert shuter threshold
 *      =========================================================================================
 *Bit   |     7    |     6    |     5    |     4    |     3    |     2    |     1    |     0    |
 *      =========================================================================================
 *Field |    ATH   |    ATH   |    ATH   |    ATH   |    ATH   |    ATH   |    ATH   |    ATH   |
 *      =========================================================================================
 */
#define CRUCIALTEC_OJ_REG_AD_ATH_HIGH         0x6E
#define CRUCIALTEC_OJ_REG_AD_ATH_HIGH_DEFAULT_VALUE   0x34

#define CRUCIALTEC_OJ_REG_AD_ATH_LOW          0x70
#define CRUCIALTEC_OJ_REG_AD_ATH_LOW_DEFAULT_VALUE    0x18

/* For setting the HIGH/LOW speed De-assert shuter threshold
 *      =========================================================================================
 *Bit   |     7    |     6    |     5    |     4    |     3    |     2    |     1    |     0    |
 *      =========================================================================================
 *Field |    DTH   |    DTH   |    DTH   |    DTH   |    DTH   |    DTH   |    DTH   |    DTH   |
 *      =========================================================================================
 */
#define CRUCIALTEC_OJ_REG_AD_DTH_HIGH         0x6F
#define CRUCIALTEC_OJ_REG_AD_DTH_HIGH_DEFAULT_VALUE   0x3C

#define CRUCIALTEC_OJ_REG_AD_DTH_LOW          0x71
#define CRUCIALTEC_OJ_REG_AD_DTH_LOW_DEFAULT_VALUE    0x20

/* Quantization Control Register for X and Y
 *      =========================================================================================
 *Bit   |     7    |     6    |     5    |     4    |     3    |     2    |     1    |     0    |
 *     =========================================================================================
 *Field |  YQ_ON   | YQ_DIV2  | YQ_DIV1  | YQ_DIV0  |  XQ_ON   | XQ_DIV3  | XQ_DIV2  | XQ_DIV0  |
 *     =========================================================================================
 */
#define CRUCIALTEC_OJ_REG_QUANTIZE_CTRL       0x73
#define CRUCIALTEC_OJ_REG_QUANTIZE_CTRL_VALUE_YQ_MASK              0x80   // Y Quantization Control
#define CRUCIALTEC_OJ_REG_QUANTIZE_CTRL_VALUE_YQ_OFF               0x00
#define CRUCIALTEC_OJ_REG_QUANTIZE_CTRL_VALUE_YQ_ON                0x80
#define CRUCIALTEC_OJ_REG_QUANTIZE_CTRL_VALUE_XQ_MASK              0x08   // X Quantization Control
#define CRUCIALTEC_OJ_REG_QUANTIZE_CTRL_VALUE_XQ_OFF               0x00
#define CRUCIALTEC_OJ_REG_QUANTIZE_CTRL_VALUE_XQ_ON                0x08
#define CRUCIALTEC_OJ_REG_QUANTIZE_CTRL_VALUE_YQ_DIV_MASK          0x70   // Y Quantization Factor
#define CRUCIALTEC_OJ_REG_QUANTIZE_CTRL_VALUE_YQ_DIV_FACTOR_OF_1   0x10   // Y Quantization Factor of 1
#define CRUCIALTEC_OJ_REG_QUANTIZE_CTRL_VALUE_YQ_DIV_FACTOR_OF_2   0x20   // Y Quantization Factor of 2
#define CRUCIALTEC_OJ_REG_QUANTIZE_CTRL_VALUE_XQ_DIV_MASK          0x07   // X Quantization Factor
#define CRUCIALTEC_OJ_REG_QUANTIZE_CTRL_VALUE_XQ_DIV_FACTOR_OF_1   0x01   // Y Quantization Factor of 1
#define CRUCIALTEC_OJ_REG_QUANTIZE_CTRL_VALUE_XQ_DIV_FACTOR_OF_2   0x02   // Y Quantization Factor of 2


/* Quantization Gradient Control Register for X and Y
 *      =========================================================================================
 *Bit   |     7    |     6    |     5    |     4    |     3    |     2    |     1    |     0    |
 *     =========================================================================================
 *Field | Reserved | Reserved | Reserved | Reserved | Reserved |  XYQ_M   |  XYQ_C1  |  XYQ_C0  |
 *     =========================================================================================
 */
#define CRUCIALTEC_OJ_REG_XYQ_THRESH          0x74
#define CRUCIALTEC_OJ_REG_XYQ_THRESH_VALUE_GRADIENT_MASK    0x04   // Gradient of Linear Region
#define CRUCIALTEC_OJ_REG_XYQ_THRESH_VALUE_GRADIENT_1       0x00
#define CRUCIALTEC_OJ_REG_XYQ_THRESH_VALUE_GRADIENT_2       0x04
#define CRUCIALTEC_OJ_REG_XYQ_THRESH_VALUE_INDICATES_OFFSET_OF_LINEAR_REGION_MASK    0x03
#define CRUCIALTEC_OJ_REG_XYQ_THRESH_VALUE_INDICATES_OFFSET_OF_LINEAR_REGION_DEFAULT 0x02


/* Finger Presence Detection Control Register
 *      =========================================================================================
 *Bit   |     7    |     6    |     5    |     4    |     3    |     2    |     1    |     0    |
 *     =========================================================================================
 *Field | Reserved | FPD_POL  | FPD_TH5  | FPD_TH4  | FPD_TH3  | FPD_TH2  | FPD_TH1  | FPD_TH0  |
 *     =========================================================================================
 */
#define CRUCIALTEC_OJ_REG_FPD_CTRL            0x75
#define CRUCIALTEC_OJ_REG_FPD_CTRL_DEFAULT_VALUE            0x50
#define CRUCIALTEC_OJ_REG_FPD_CTRL_VALUE_GPIO_STATUS_MASK   0x40   // FPD GPIO Status
#define CRUCIALTEC_OJ_REG_FPD_CTRL_VALUE_GPIO_STATUS_LOW    0x00
#define CRUCIALTEC_OJ_REG_FPD_CTRL_VALUE_GPIO_STATUS_HIGH   0x40
#define CRUCIALTEC_OJ_REG_FPD_CTRL_VALUE_THRESHOLD_MASK     0x3F   // Sets FPD threshold based on shutter value.


/* Sensor Orientation Control Register
 *      =========================================================================================
 *Bit   |     7    |     6    |     5    |     4    |     3    |     2    |     1    |     0    |
 *     =========================================================================================
 *Field | XY_SWAP  |  Y_INV   |   X_INV  | Reserved | Reserved | Reserved |  ORIENT  |  ORIENT  |
 *     =========================================================================================
 */
#define CRUCIALTEC_OJ_REG_ORIENTATION_CTRL    0x77
#define CRUCIALTEC_OJ_REG_ORIENTATION_CTRL_VALUE_XY_SWAP_MASK   0x80   // XY Swap Mask
#define CRUCIALTEC_OJ_REG_ORIENTATION_CTRL_VALUE_XY_SWAP_OFF    0x00   // Normal sensor reporting of DX and DY
#define CRUCIALTEC_OJ_REG_ORIENTATION_CTRL_VALUE_XY_SWAP_ON     0x80   // Swap data of DX to DY and DY to DX

#define CRUCIALTEC_OJ_REG_ORIENTATION_CTRL_VALUE_Y_INV_MASK     0x40   // Y Invert Mask
#define CRUCIALTEC_OJ_REG_ORIENTATION_CTRL_VALUE_Y_INV_OFF      0x00   // Normal sensor reporting DY
#define CRUCIALTEC_OJ_REG_ORIENTATION_CTRL_VALUE_Y_INV_ON       0x40   // Invert data of DY only

#define CRUCIALTEC_OJ_REG_ORIENTATION_CTRL_VALUE_X_INV_MASK     0x20   // X Invert Mask
#define CRUCIALTEC_OJ_REG_ORIENTATION_CTRL_VALUE_X_INV_OFF      0x00   // Normal sensor reporting DX
#define CRUCIALTEC_OJ_REG_ORIENTATION_CTRL_VALUE_X_INV_ON       0x20   // Invert data of DX only

#define CRUCIALTEC_OJ_REG_ORIENTATION_CTRL_VALUE_ORIENT_MASK              0x03  // Orient Pin State 
#define CRUCIALTEC_OJ_REG_ORIENTATION_CTRL_VALUE_ORIENT_90_DEG_CLOCKWISE  0x00
#define CRUCIALTEC_OJ_REG_ORIENTATION_CTRL_VALUE_ORIENT_ZERO_DEG          0x01

#define CRUCIALTEC_OJ_REG_ORIENTATION_CTRL_VALUE_DEFAULT  \
               CRUCIALTEC_OJ_REG_ORIENTATION_CTRL_VALUE_XY_SWAP_ON

/* + H/W configuration for Jetta */
#define OTP_RST     mfp_to_gpio(MFP_PIN_GPIO153) //153
#define OTP_INT     mfp_to_gpio(MFP_PIN_GPIO152) //152
#define OTP_SHDN    mfp_to_gpio(MFP_PIN_GPIO151) //151 
#define OTP_I2C_SCL 144
#define OTP_I2C_SDA 145
#define OTP_MAJOR  100
/* - H/W configuration for Jetta */

/*==================================================================================================
                                  LOCAL TYPEDEFS (STRUCTURES, UNIONS, ENUMS)
==================================================================================================*/
// OJ's Operation Mode
typedef enum {
   OPERATIONAL_MODE_MIN,
   OPERATIONAL_MODE_MOUSE_MODE = OPERATIONAL_MODE_MIN,       // 0
   OPERATIONAL_MODE_DIRECTIONAL_PAD_MODE,                    // 1
   OPERATIONAL_MODE_DIRECTIONAL_PAD_STROKE_AND_HOLD,         // 2
   OPERATIONAL_MODE_JOYSTICK_MODE,                           // 3
   OPERATIONAL_MODE_DIRECTIONAL_PAD_MODE_ONLY,               // 4
   OPERATIONAL_MODE_DIRECTIONAL_PAD_STROKE_AND_HOLD_ONLY,    // 5
   OPERATIONAL_MODE_DIRECTIONAL_PAD_BLASTER_MODE,            // 6
   OPERATIONAL_MODE_ON,                                      // 7
   OPERATIONAL_MODE_OFF,                                     // 8
   OPERATIONAL_MODE_MAX
} OJ_OPERATIONAL_MODE_T;

typedef enum {
   OJ_TOUCH_NONE,
   OJ_TOUCH_PRESSED,
   OJ_TOUCH_RELEASED
} OJ_KEY_STATE_T;

// Data structure for the crucialtec OJ driver
struct crucialtec_oj_data {
   uint16_t                 addr;
   struct i2c_client       *client;
   struct input_dev        *input_dev;
   uint8_t                  dev_id;
#if OTP_SCHEDULE_DELAYED_WORK
    struct delayed_work work;
#else
    struct work_struct  work;
#endif
  const struct file_operations *cru_ops;
   int                      motion_gpio;
   int                      shutdown_gpio;
   int                      reset_gpio;
   OJ_OPERATIONAL_MODE_T    operational_mode;
   uint8_t                  initialized;
#ifdef CONFIG_CRUCIALTEC_OJ_ESD_RECOVERY
   uint8_t                  esd_polling;
#endif
   uint16_t                 valid_delta;
   uint16_t                 delay_between_key;
   uint8_t                  current_key_pressed;
   OJ_KEY_STATE_T           key_state;
   bool                     fpd_timer;
   uint8_t                  valid_squal;
   uint8_t                  jitter_check;
   bool                     als_register;
   uint8_t                  outdoor_light_level;
   bool                     outdoor_light;
   bool                     suspend_mode;
   uint8_t                  irq_count;
   int64_t                  dpad_key_release_time;
   uint16_t                 tsb_keyguard_delay;
   uint16_t                 dpad_dir_key_delay_after_dome_pressed;

   // Variables for blaster mode
   uint16_t                 delta_sum_time;
   uint8_t                  delta_sum_cp;
   uint8_t                  delta_min_in_x;
   uint8_t                  delta_min_in_y;
   uint8_t                  delta_min_out_x;
   uint8_t                  delta_min_out_y;
   uint8_t                  blaster_polling_rate;
   uint16_t                 blaster_num_of_poll;
   bool                     valid_key;
   uint8_t                  Num_cont_in;
   uint8_t                  pre_event;
   int16_t                  SumDeltaX;
   int16_t                  SumDeltaY;
   int                        Dome_state;
   int                        Dome_Cnt;//crucial_cgj
   int                        Mode_Change;
   int                        Mode_EZChange;
   int8_t                        deltaX;
   int8_t                        deltaY;

   // touch Click
   int                        Dome_Press;
   int                        Touch_Press;
   int8_t                        gClickAreaX;  //PressSumX -> gPressAreaX;
   int8_t                        gClickAreaY;  //PressSumY -> gPressAreaY

   int                            OnePress_time;
   int                            double_waittime;
   bool                         OneClick;
   bool                         Pre_Double;
   bool                         Click_cnt;
   bool                         DoubleClick;
   bool                         longPressChk;

   //Trigger
   bool                     TriggerChk;
   int                        Trigger_Cnt;
   bool                         Pre_Double_cancel;
   uint8_t                  Pre_current_key_pressed;


    int                     shortcut_mode;//crucial_yong
    int input_count; //crucial_yong touch
    int current_X;
    int current_Y;
    int8_t pre_current_X;
    int8_t pre_current_Y;    
    int sum_X;
    int sum_Y;
    int process;   
    struct early_suspend early_suspend;    
    struct mutex ops_lock;        
};


struct crucial_oj_platform_data {
   //struct device dev;
   int                      gpio_motion_irq;
   int                      gpio_shutdown;
   int                      gpio_reset;
   const char *name;
};

// Data structure for the I2c's read and write
typedef struct
{
   uint8_t reg;
   uint8_t data;
} crucialtec_oj_reg_data_t;

typedef struct
{
   uint8_t key_event;    
   //uint8_t accel_value;
} key_event_info_t;

/*==================================================================================================
                                        LOCAL VARIABLES
==================================================================================================*/

//static struct class *ojkey_class;

static int16_t gv_Sum_DeltaX;
static int16_t gv_Sum_DeltaY;
static int gv_TestSum_DeltaX;
static int gv_TestSum_DeltaY;
static int gv_MotionWork_Cnt;

static ktime_t gv_CurrTime, gv_PrevTime;
static ktime_t gv_SubTime;

// This contains the data for the Crucialtec Optical Joystick
static struct crucialtec_oj_data    oj_data;

// Register and data values for the default configuration
static crucialtec_oj_reg_data_t oj_check_regs[] =
{
   // Speed switching configuration
   {CRUCIALTEC_OJ_REG_SPEED_ST12, CRUCIALTEC_OJ_REG_SPEED_ST12_DEFAULT_VALUE},
   {CRUCIALTEC_OJ_REG_SPEED_ST21, CRUCIALTEC_OJ_REG_SPEED_ST21_DEFAULT_VALUE},
   {CRUCIALTEC_OJ_REG_SPEED_ST23, CRUCIALTEC_OJ_REG_SPEED_ST23_DEFAULT_VALUE},
   {CRUCIALTEC_OJ_REG_SPEED_ST32, CRUCIALTEC_OJ_REG_SPEED_ST32_DEFAULT_VALUE},
   {CRUCIALTEC_OJ_REG_SPEED_ST34, CRUCIALTEC_OJ_REG_SPEED_ST34_DEFAULT_VALUE},
   {CRUCIALTEC_OJ_REG_SPEED_ST43, CRUCIALTEC_OJ_REG_SPEED_ST43_DEFAULT_VALUE},
   {CRUCIALTEC_OJ_REG_SPEED_ST45, CRUCIALTEC_OJ_REG_SPEED_ST45_DEFAULT_VALUE},
   {CRUCIALTEC_OJ_REG_SPEED_ST54, CRUCIALTEC_OJ_REG_SPEED_ST54_DEFAULT_VALUE},
   
   // Assert/De-assert configuration
   {CRUCIALTEC_OJ_REG_AD_CTRL,     CRUCIALTEC_OJ_REG_AD_CTRL_VALUE_HIGHER_CPI},
   {CRUCIALTEC_OJ_REG_AD_ATH_HIGH, CRUCIALTEC_OJ_REG_AD_ATH_HIGH_DEFAULT_VALUE},
   {CRUCIALTEC_OJ_REG_AD_DTH_HIGH, CRUCIALTEC_OJ_REG_AD_DTH_HIGH_DEFAULT_VALUE},
   {CRUCIALTEC_OJ_REG_AD_ATH_LOW,  CRUCIALTEC_OJ_REG_AD_ATH_LOW_DEFAULT_VALUE},
   {CRUCIALTEC_OJ_REG_AD_DTH_LOW,  CRUCIALTEC_OJ_REG_AD_DTH_LOW_DEFAULT_VALUE},
   
   // Finger Presence Detect configuration
   {CRUCIALTEC_OJ_REG_FPD_CTRL,   CRUCIALTEC_OJ_REG_FPD_CTRL_DEFAULT_VALUE},

   // XY Quantization configuration
   {CRUCIALTEC_OJ_REG_QUANTIZE_CTRL, (CRUCIALTEC_OJ_REG_QUANTIZE_CTRL_VALUE_YQ_ON |
                                      CRUCIALTEC_OJ_REG_QUANTIZE_CTRL_VALUE_YQ_DIV_FACTOR_OF_1 |
                                      CRUCIALTEC_OJ_REG_QUANTIZE_CTRL_VALUE_XQ_ON |
                                      CRUCIALTEC_OJ_REG_QUANTIZE_CTRL_VALUE_XQ_DIV_FACTOR_OF_1 )},
   {CRUCIALTEC_OJ_REG_XYQ_THRESH, CRUCIALTEC_OJ_REG_XYQ_THRESH_VALUE_INDICATES_OFFSET_OF_LINEAR_REGION_DEFAULT},

   // LED Control configuration
   {CRUCIALTEC_OJ_REG_LED_CONTROL, CRUCIALTEC_OJ_REG_LED_CONTROL_VALUE_LED_NORMAL_OPERATION},

   {0,0}  /* Must end with 0 reg */
};

// Registers and data values for OJ's self test sequence
crucialtec_oj_reg_data_t oj_self_test_seq[]=
{
   /* Perform a soft reset */
   { CRUCIALTEC_OJ_REG_SOFT_RESET,    CRUCIALTEC_OJ_REG_SOFT_RESET_VALUE_REVERT_TO_DEFAULT },  

   { CRUCIALTEC_OJ_REG_ENGINE,        (CRUCIALTEC_OJ_REG_ENGINE_VALUE_ENGINE_ON |
                                       CRUCIALTEC_OJ_REG_ENGINE_VALUE_SPEED_ON  |
                                       CRUCIALTEC_OJ_REG_ENGINE_VALUE_ASSERT_ON |
                                       CRUCIALTEC_OJ_REG_ENGINE_VALUE_XYQ_ON    |
                                       CRUCIALTEC_OJ_REG_ENGINE_VALUE_FINGER_ON |
                                       CRUCIALTEC_OJ_REG_ENGINE_VALUE_XY_SCALE_ON) },
                                        
   { CRUCIALTEC_OJ_REG_QUANTIZE_CTRL, (CRUCIALTEC_OJ_REG_QUANTIZE_CTRL_VALUE_YQ_ON              |
                                       CRUCIALTEC_OJ_REG_QUANTIZE_CTRL_VALUE_YQ_DIV_FACTOR_OF_2 |
                                       CRUCIALTEC_OJ_REG_QUANTIZE_CTRL_VALUE_XQ_ON              |
                                       CRUCIALTEC_OJ_REG_QUANTIZE_CTRL_VALUE_XQ_DIV_FACTOR_OF_2) },

   { CRUCIALTEC_OJ_REG_SPEED_CONTROL, (CRUCIALTEC_OJ_REG_SPEED_CONTROL_VALUE_Y_SCALE_ON      |
                                       CRUCIALTEC_OJ_REG_SPEED_CONTROL_VALUE_X_SCALE_ON      |
                                       CRUCIALTEC_OJ_REG_SPEED_CONTROL_VALUE_SP_INT_VAL_8_MS |
                                       CRUCIALTEC_OJ_REG_SPEED_CONTROL_VALUE_LOW_CPI_OFF     |
                                       CRUCIALTEC_OJ_REG_SPEED_CONTROL_VALUE_HIGH_CPI_OFF)    },

   { CRUCIALTEC_OJ_REG_SELF_TEST,      CRUCIALTEC_OJ_REG_SELF_TEST_VALUE_TEST_ENABLE_ON},

   {0,0}  /* Must end with 0 reg */
};

/*==================================================================================================
                                   LOCAL FUNCTION PROTOTYPES
==================================================================================================*/
static int crucialtec_oj_read_register (struct i2c_client *client, uint8_t address, uint8_t *value);
static int crucialtec_oj_write_register (struct i2c_client *client, uint8_t address, uint8_t value);
static irqreturn_t crucialtec_oj_irq_handler (int irq, void *dev_id);
int crucialtec_oj_toggle_reset_line(void);
bool crucialtec_oj_check_registers(struct i2c_client *client);
bool crucialtec_oj_init_hardware(struct i2c_client *client);
#if OTP_SCHEDULE_DELAYED_WORK
static void otp_motion_continue_work(void);
static void otp_motion_start_work(struct work_struct *work);
#endif
static int crucialtec_oj_probe (struct i2c_client *client, const struct i2c_device_id *id);
static int crucialtec_oj_remove (struct i2c_client *client);
#ifdef CONFIG_CRUCIALTEC_OJ_ESD_RECOVERY
bool crucialtec_oj_detect_esd(void);
void crucialtec_oj_check_esd(void* arg);
void crucialtec_oj_fix_esd(void);
#endif /* CONFIG_CRUCIALTEC_OJ_ESD_RECOVERY */
#ifdef CONFIG_HAS_EARLYSUSPEND
static void crucialtec_oj_early_suspend(struct early_suspend *h);
static void crucialtec_oj_late_resume(struct early_suspend *h);
#endif
static int crucialtec_oj_suspend(struct i2c_client *client, pm_message_t msg);
static int crucialtec_oj_resume(struct i2c_client *client);

#if 0
static void fpd_poll_work(struct work_struct *work);
static void fpd_start_work(struct work_struct *work);
static DECLARE_DELAYED_WORK(fpd_poll_workqueue, fpd_poll_work);
static DECLARE_DELAYED_WORK(fpd_start_workqueue, fpd_start_work);
#endif

//static void OTP_timer_fire(unsigned long data);

void crucialtec_oj_als_cb(unsigned prev_zone, unsigned curr_zone, unsigned cookie);

static void blaster_mode_poll_work(struct work_struct *work);
static DECLARE_DELAYED_WORK(blaster_mode_poll_workqueue, blaster_mode_poll_work);

bool is_ok_for_dpad_key(void);
key_event_info_t blaster_mode_process_data(int8_t delta_x, int8_t delta_y, int64_t current_time);
/*==================================================================================================
                                          LOCAL MACROS
==================================================================================================*/

#define CRUCIALTEC_OJ_DEBUG_MSGS 0   // Debug message flag

#if CRUCIALTEC_OJ_DEBUG_MSGS
#define DEBUG_OTP_PRINTK(fmt, args...) printk(KERN_INFO fmt, ## args)
#else
#define DEBUG_OTP_PRINTK(fmt, args...)
#endif

/*==================================================================================================
                                          LOCAL FUNCTIONS
==================================================================================================*/


/*==================================================================================================

FUNCTION: crucialtec_oj_read_register

DESCRIPTION: 
   This function reads the value from the given register.

ARGUMENTS PASSED:
   i2c_client *client  - pointer to the device's data
   uint8_t     address - addresd to read from
   uint8_t    *value   - read data from the register
    
REFERENCE ARGUMENTS PASSED:
   None

RETURN VALUE:
   If the return value is less than 1 then there is an error 
   else it returns the number of byte read.

DEPENDENCIES:
   I2c driver is ready to accept command.

SIDE EFFECTS:
   None

==================================================================================================*/
static int crucialtec_oj_read_register (struct i2c_client *client, uint8_t address, uint8_t *value)
{
   uint8_t data[1];
   int rc;
   
   data[0] = address; 
   //rc = i2c_smbus_read_byte_data(client, data);
#if 1   
   if ((rc = i2c_master_send(client, data, 1)) < 0)
   {
      dev_err(&client->dev, "[OTP]  %s(%s): i2c_master_send error %d\n", __FILE__, __FUNCTION__, rc);
      
      return (rc);
   }
   
   //msleep_interruptible(1);
   
   *value = 0;
   
   if ((rc = i2c_master_recv(client, value, 1)) < 0)
   {
      dev_err(&client->dev, "[OTP]  %s(%s): i2c_master_recv error %d\n", __FILE__, __FUNCTION__, rc);
   }
#endif   
   return (rc);
}


/*==================================================================================================

FUNCTION: crucialtec_oj_write_register

DESCRIPTION: 
   This function writes the data to the given register.

ARGUMENTS PASSED:
   i2c_client *client  - pointer to the device's data
   uint8_t     address - addresd to write to 
   uint8_t     value   - value to write to the address
    
REFERENCE ARGUMENTS PASSED:
   None

RETURN VALUE:
   If the return value is less than 1 then there is an error 
   else it returns the number of byte read.

DEPENDENCIES:
   I2c driver is ready to accept command.

SIDE EFFECTS:
   None

==================================================================================================*/
static int crucialtec_oj_write_register (struct i2c_client *client, uint8_t address, uint8_t value)
{
   int rc;
   crucialtec_oj_reg_data_t msg;
   
   msg.reg = address;
   msg.data = value;
   
   if ((rc = i2c_master_send(client, (uint8_t *)&msg, sizeof(msg))) < 0)
   {
      dev_err(&client->dev, "[OTP]  %s(%s): i2c_master_send error %d\n", __FILE__, __FUNCTION__, rc);
   }
   
   return (rc);
}


/*==================================================================================================

FUNCTION: oj_disable_irq

DESCRIPTION: 
   This function disables the OJ's IRQ.

ARGUMENTS PASSED:
   None
    
REFERENCE ARGUMENTS PASSED:
   None

RETURN VALUE:
   None

DEPENDENCIES:
   I2c driver is ready to accept command.

SIDE EFFECTS:
   None

==================================================================================================*/
void oj_disable_irq(void)
{
   if ( oj_data.irq_count > 0 )
   {
      oj_data.irq_count--;
      disable_irq(oj_data.client->irq);

      DEBUG_OTP_PRINTK("[OTP] %s: Disabling irq %d (count: %d)\n", 
                   __FUNCTION__, oj_data.client->irq, oj_data.irq_count);
   }
   else
   {
      DEBUG_OTP_PRINTK("[OTP] %s : irq %d is already disabled (count: %d)\n", 
                   __FUNCTION__, oj_data.client->irq, oj_data.irq_count);
   }
   return;
}


/*==================================================================================================

FUNCTION: oj_enable_irq

DESCRIPTION: 
   This function enables the OJ's IRQ.

ARGUMENTS PASSED:
   None
    
REFERENCE ARGUMENTS PASSED:
   None

RETURN VALUE:
   None

DEPENDENCIES:
   I2c driver is ready to accept command.

SIDE EFFECTS:
   None

==================================================================================================*/
void oj_enable_irq(void)
{
   if ( oj_data.irq_count < 1 )
   {
      oj_data.irq_count++;
      enable_irq(oj_data.client->irq);

      DEBUG_OTP_PRINTK("[OTP] %s: Enabling irq %d (count: %d)\n", 
                   __FUNCTION__, oj_data.client->irq, oj_data.irq_count);
   }
   else
   {
      DEBUG_OTP_PRINTK("[OTP] %s: irq %d is already enabled (count: %d)\n", 
                   __FUNCTION__, oj_data.client->irq, oj_data.irq_count);
   }
   return;
}


/*==================================================================================================

FUNCTION: crucialtec_oj_get_dpad_release_time

DESCRIPTION: 
   This function returns the time stamp (in ns) of the last DPAD keys from the OJ.

ARGUMENTS PASSED:
   None
    
REFERENCE ARGUMENTS PASSED:
   None

RETURN VALUE:
   None

DEPENDENCIES:
   I2c driver is ready to accept command.

SIDE EFFECTS:
   None

==================================================================================================*/
int64_t crucialtec_oj_get_dpad_release_time(void)
{
   return oj_data.dpad_key_release_time;
}


/*==================================================================================================

FUNCTION: crucialtec_oj_get_key_guard_delay

DESCRIPTION: 
   This function returns the TSB keyguard delay timeout.

ARGUMENTS PASSED:
   None
    
REFERENCE ARGUMENTS PASSED:
   None

RETURN VALUE:
   None

DEPENDENCIES:
   I2c driver is ready to accept command.

SIDE EFFECTS:
   None

==================================================================================================*/
uint16_t crucialtec_oj_get_key_guard_delay(void)
{
   return oj_data.tsb_keyguard_delay;
}

/*==================================================================================================

FUNCTION: crucialtec_oj_als_cb

DESCRIPTION:
   This is the callback function for the ALS sensor.  This function sets the outdoor light boolean
   based on the ALS's current light zone.

ARGUMENTS PASSED:
   unsigned prev_zone - the previous light zone
   unsigned curr_zone - the current light zone
   unsigned cookie - cookie, pased to register

REFERENCE ARGUMENTS PASSED:
   None

RETURN VALUE:
   None

DEPENDENCIES:
   None

SIDE EFFECTS:
   None

==================================================================================================*/
void crucialtec_oj_als_cb(unsigned prev_zone, unsigned curr_zone, unsigned cookie)
{
   DEBUG_OTP_PRINTK("[OTP] ALS prev_zone = %d, curr_zone= %d\n", prev_zone, curr_zone);

   oj_data.outdoor_light = (curr_zone >= oj_data.outdoor_light_level) ? true : false;
}


/*==================================================================================================

FUNCTION: crucialtec_check_for_jitter

DESCRIPTION:
   This function checks for jitter/ghost movement by using the OJ's SQUAL and Shutter.

ARGUMENTS PASSED:
   uint16_t shutter    - the current shutter value of the OJ
   uint8_t  pixel_sum  - the current pixel sum of the OJ

REFERENCE ARGUMENTS PASSED:
   None

RETURN VALUE:
   True  - Jitter not detected
   False - Jitter detected

DEPENDENCIES:
   None

SIDE EFFECTS:
   None

==================================================================================================*/


#if 1
//2011.08.03 Application Code for SEC Jetta
#define OJ_DEGREE 5
const uint16_t oj_sht_tbl[OJ_DEGREE] = {70, 500, 1250, 2250, 2750};
const uint8_t oj_pxsum_tbl[OJ_DEGREE] = {0, 20, 30, 40, 50};
#else
//2011.05.25 Application Code for SEC Jetta
#define OJ_DEGREE 6
const uint16_t oj_sht_tbl[OJ_DEGREE] = {29, 1749, 1999, 2249, 2749, 2929};
const uint8_t oj_pxsum_tbl[OJ_DEGREE] = {0, 0, 39, 44, 49, 54};
#endif
    
bool crucialtec_check_for_jitter(uint16_t shutter, uint8_t pixel_sum)
{
   bool ret = false;   
   uint8_t i = 0;

   for(i=1; i<OJ_DEGREE; i++)
   {
      if(((oj_sht_tbl[i-1]<shutter)&&(shutter<=oj_sht_tbl[i])) && (oj_pxsum_tbl[i]<pixel_sum))
      {
         //Pass 
         ret = true;
         break;
      }
   }
   
   return (ret);
}

#if 0
static void fpd_start_work(struct work_struct *work)
{
   int8_t delta_x = 0;
   int8_t delta_y = 0;   
   int err = 0;

   // Init and configure the OJ's hardware
   if (!(crucialtec_oj_init_hardware(oj_data.client)))
   {
      DEBUG_OTP_PRINTK("[OTP] ##### 4 \n");
      dev_err(&oj_data.client->dev, "[OTP]  %s(%s): Hardware initialization error \n", __FILE__, __FUNCTION__);
      err = (-EFAULT);
      //goto probe_error_free_clientdata;
   }

   crucialtec_oj_read_register(oj_data.client, CRUCIALTEC_OJ_REG_DELTA_X, &delta_x);
   crucialtec_oj_read_register(oj_data.client, CRUCIALTEC_OJ_REG_DELTA_Y, &delta_y);

   //mod_timer(&OTP_timer, jiffies+1);  //cru_cgj
   //OTP_timer_fire(0);

   schedule_delayed_work(&fpd_poll_workqueue, FPD_POLL_TIMER);

}
#endif

/*==================================================================================================

FUNCTION: fpd_poll_work

DESCRIPTION:
   This function checks for the present of the finger on the OJ.  If the finger is on the OJ
   then send the last key pressed and released up and schedule this function again.
   If the finger is not on the OJ then clear the OJ's registers and enables the IRQ.

ARGUMENTS PASSED:
    Pointer to the optical finger navigation's work_struct structure.

REFERENCE ARGUMENTS PASSED:
   None

RETURN VALUE:
   None

DEPENDENCIES:
   None

SIDE EFFECTS:
   None

==================================================================================================*/
#define OTP_TOUCH_PRESS 1
#define OTP_TOUCH_RELEASE 0

#define OTP_TOUCH_NONE 0
#define OTP_TOUCH_ONECLICK 1
#define OTP_TOUCH_DOUBLECLICK 2
#define DCLICK_WAIT_TIME_MAX 5
#define OCLICK_CANCEL_TIME 8
#define CLICK_AREA_MAX 2

bool get_Otp_Click_Area(int touch_state)
{
        if(touch_state)
        {

            oj_data.gClickAreaX += abs(oj_data.deltaX);
            oj_data.gClickAreaY += abs(oj_data.deltaY);
            if(oj_data.gClickAreaX <= CLICK_AREA_MAX && oj_data.gClickAreaY <= CLICK_AREA_MAX)
            {
                return true;
            }
            else
            {
                oj_data.OneClick = false;
                oj_data.longPressChk = true;
                return false;
             }
        }
        else   
            oj_data.gClickAreaX = oj_data.gClickAreaY =0;

        return false;
}
  
uint8_t  get_Otp_touch(void)
{
       
        oj_data.Touch_Press = 1; //SKC gpio_get_value(S5PC1XX_GPH0(0));
    
        //Touch Press
        if (get_Otp_Click_Area(oj_data.Touch_Press) && oj_data.longPressChk == false )  //Lcd touch press
        {
        
            if( oj_data.double_waittime != 0 && oj_data.double_waittime < DCLICK_WAIT_TIME_MAX)
            {
                oj_data.OneClick = false;
                oj_data.longPressChk = true;
                oj_data.double_waittime = 0;
                return OTP_TOUCH_DOUBLECLICK;
            }
            oj_data.OneClick = true;
            if(++oj_data.OnePress_time > OCLICK_CANCEL_TIME )
            {
                oj_data.OneClick = false;
                oj_data.longPressChk = true;
            }
            oj_data.double_waittime = 0;
        }
        else if(oj_data.Touch_Press == OTP_TOUCH_RELEASE)
        {
           
            if(oj_data.OneClick == true)
            {
                if(++oj_data.double_waittime > DCLICK_WAIT_TIME_MAX)
                {
                    oj_data.OneClick = false;
                    oj_data.longPressChk = false;
                    return OTP_TOUCH_ONECLICK;
                }
            }
            oj_data.OnePress_time = 0;
            oj_data.longPressChk = false;
        }
    
        return OTP_TOUCH_NONE;
    }

void crucial_otp_work_func(struct work_struct *work)
{
   uint8_t  shutter_upper = 0;
   uint8_t  shutter_lower = 0;
   uint16_t shutter = 0;
   uint8_t pixel_sum = 0;
   
   uint8_t motion_status = 0;
   int8_t delta_x = 0;
   int8_t delta_y = 0;
   int64_t current_time = 0;
   key_event_info_t  key_info;

   static uint16_t count_me = 0;

   DEBUG_OTP_PRINTK("[OTP] crucial_otp_work_func\n");

   if (oj_data.suspend_mode == true)
   {
      DEBUG_OTP_PRINTK("[OTP] suspend mode - Done blaster mode polling\n");
   
      count_me = 0;
    
      oj_data.current_key_pressed = 0;
  
      return;
   }
      
   crucialtec_oj_read_register(oj_data.client, CRUCIALTEC_OJ_REG_MOTION, &motion_status);

   crucialtec_oj_read_register(oj_data.client, CRUCIALTEC_OJ_REG_DELTA_X, &delta_x);
   crucialtec_oj_read_register(oj_data.client, CRUCIALTEC_OJ_REG_DELTA_Y, &delta_y);

   crucialtec_oj_read_register(oj_data.client, CRUCIALTEC_OJ_REG_SHUTTER_MAX_HI, &shutter_upper);
   crucialtec_oj_read_register(oj_data.client, CRUCIALTEC_OJ_REG_SHUTTER_MAX_LO, &shutter_lower);
   crucialtec_oj_read_register(oj_data.client, CRUCIALTEC_OJ_REG_PIXEL_SUM,      &pixel_sum);

   shutter = (shutter_upper << 8) | shutter_lower;

   if (oj_data.jitter_check == 1)
   {
      if (crucialtec_check_for_jitter(shutter, pixel_sum) == false)
      {
         DEBUG_OTP_PRINTK("[OTP] Jitter detected! shutter=%d, pixel_sum=%d\n",shutter, pixel_sum );

         oj_data.current_key_pressed = 0;
         oj_data.key_state = OJ_TOUCH_RELEASED;
         oj_data.fpd_timer = false;

         schedule_delayed_work(&blaster_mode_poll_workqueue, msecs_to_jiffies(5));

         return;
      }
   }

   DEBUG_OTP_PRINTK("[OTP] Register CRUCIALTEC_OJ_REG_MOTION  0x%x\n", motion_status);
   DEBUG_OTP_PRINTK("[OTP] delta_x=%d, delta_y=%d\n",delta_x,delta_y);
   
   current_time = ktime_to_ns(ktime_get());

   if( ( delta_x != 0 ) || ( delta_y != 0 ) ) 
   {
      count_me=0;
      DEBUG_OTP_PRINTK("[OTP] count_me is cleared\n");
      key_info = blaster_mode_process_data(delta_x, delta_y, current_time);
      if ( (key_info.key_event != 0) && (is_ok_for_dpad_key() == true) )
      {
         input_report_key(oj_data.input_dev, key_info.key_event, 1);    // report key
         input_sync(oj_data.input_dev);

         msleep_interruptible(1);

         input_report_key(oj_data.input_dev, key_info.key_event, 0);    // report key
         input_sync(oj_data.input_dev);

         DEBUG_OTP_PRINTK("[OTP] sum_delta_x=%d, sum_delta_y=%d, key_event %d\n", gv_TestSum_DeltaX, gv_TestSum_DeltaY, key_info.key_event);

         oj_data.dpad_key_release_time = ktime_to_ns(ktime_get());
      }
   }
   else
   {
      count_me++;
      DEBUG_OTP_PRINTK("[OTP] count_me = %d\n", count_me);
   }

   if (count_me <= oj_data.blaster_num_of_poll)
   {
      schedule_delayed_work(&blaster_mode_poll_workqueue, msecs_to_jiffies(oj_data.blaster_polling_rate));
   }
   else
   {
      DEBUG_OTP_PRINTK("[OTP] Done blaster mode polling\n");
   
      count_me = 0;
    
      oj_data.current_key_pressed = 0;

      crucialtec_oj_read_register(oj_data.client, CRUCIALTEC_OJ_REG_DELTA_X, &delta_x);
      crucialtec_oj_read_register(oj_data.client, CRUCIALTEC_OJ_REG_DELTA_Y, &delta_y);

      // clear the motion register
      crucialtec_oj_write_register(oj_data.client, CRUCIALTEC_OJ_REG_MOTION, 0x00);

    //SKC  oj_enable_irq();
   }
}



/*==================================================================================================

FUNCTION: is_ok_for_dpad_key

DESCRIPTION:
   This function checks last OJ's dome pressed vs the current time to see if it is safe
   for the next OJ's dpad key press event.

ARGUMENTS PASSED:
   None

REFERENCE ARGUMENTS PASSED:
   None

RETURN VALUE:
   DPAD keys or zero

DEPENDENCIES:
   None

SIDE EFFECTS:
   None

==================================================================================================*/
bool is_ok_for_dpad_key(void)
{
   int64_t key_guard_delay = (oj_data.dpad_dir_key_delay_after_dome_pressed * MS_TO_NS_FACTOR);
   
   int64_t delta_time = ktime_to_ns(ktime_get());// - adp5588_get_last_oj_dome_key();

   //DEBUG_OTP_PRINTK("[OTP] key_guard_delay=%16lx, delta_time=%16lx\n", key_guard_delay, delta_time);
   
   return ( (delta_time > key_guard_delay) ? true : false);
}


/*==================================================================================================

FUNCTION: get_blaster_nav_key

DESCRIPTION:
   This function sets the current dpad key based on the sum of delta X and Y values.

ARGUMENTS PASSED:
   int16_t sum_delta_x - the sum of delta X
   int16_t sum_delta_y - the sum of delta Y
   uint8_t delta_min_x - the offset for X
   uint8_t delta_min_y - the offset for Y

REFERENCE ARGUMENTS PASSED:
   None

RETURN VALUE:
   DPAD keys or zero

DEPENDENCIES:
   None

SIDE EFFECTS:
   None

==================================================================================================*/
uint8_t get_blaster_nav_key(int16_t sum_delta_x, int16_t sum_delta_y, uint8_t delta_min_x, uint8_t delta_min_y)
{
   uint8_t key;
   uint16_t abs_sum_x = abs(sum_delta_x);
   uint16_t abs_sum_y = abs(sum_delta_y);

    DEBUG_OTP_PRINTK("[OTP] sumX=%d, sumY=%d\n", sum_delta_x, sum_delta_y);    
#if 0   
   if ( ( abs_sum_x > ( abs_sum_y + ( abs_sum_x >> 1 ) ) ) &&
        ( abs_sum_x >  delta_min_x-(delta_min_x>>(oj_data.Num_cont_in+1)) -(delta_min_x>>(oj_data.Num_cont_in+3)) ))
#else
    if ( ( abs_sum_x >  abs_sum_y ) && ( abs_sum_x >  delta_min_x ) )  
#endif        
   {

    DEBUG_OTP_PRINTK("[OTP] delta_min_x=%d\n", delta_min_x);    
    DEBUG_OTP_PRINTK("[OTP] Num_cont_in=%d, condition =%d\n", oj_data.Num_cont_in, (delta_min_x>>(oj_data.Num_cont_in+1)) + (delta_min_x>>(oj_data.Num_cont_in+3)) );        
    
      if(oj_data.Num_cont_in)
      {
         if(sum_delta_x<0)
         {//Left event
            key=oj_data.pre_event==DPAD_LEFT_KEY?DPAD_LEFT_KEY:0;
         }
         else
         {//Right event
            key=oj_data.pre_event==DPAD_RIGHT_KEY?DPAD_RIGHT_KEY:0;
         }
      }
      else
      {//First event
         key = (sum_delta_x < 0) ? DPAD_LEFT_KEY : DPAD_RIGHT_KEY;
      }
      oj_data.Num_cont_in++;
      oj_data.pre_event=(sum_delta_x < 0) ? DPAD_LEFT_KEY : DPAD_RIGHT_KEY;

    gv_TestSum_DeltaX = sum_delta_x;
    gv_TestSum_DeltaY = sum_delta_y;          
    
   }
#if 0
   else if ( ( abs_sum_y > ( abs_sum_x + ( abs_sum_y >> 1 ) ) ) &&
             ( abs_sum_y >  delta_min_y-(delta_min_y>>(oj_data.Num_cont_in+1)) -(delta_min_y>>(oj_data.Num_cont_in+3)) ))
#else
    else if ( ( abs_sum_y >  abs_sum_x ) && ( abs_sum_y >  delta_min_y) )     
#endif        
   {

    DEBUG_OTP_PRINTK("[OTP] delta_min_y=%d\n", delta_min_y);    
    DEBUG_OTP_PRINTK("[OTP] Num_cont_in=%d, condition =%d\n", oj_data.Num_cont_in, (delta_min_y>>(oj_data.Num_cont_in+1)) + (delta_min_y>>(oj_data.Num_cont_in+3)) );        
        
      if(oj_data.Num_cont_in)
      {
         if(sum_delta_y<0)
         {//Up event
            key=oj_data.pre_event==DPAD_UP_KEY?DPAD_UP_KEY:0;
         }
         else
         {//Down event
            key=oj_data.pre_event==DPAD_DOWN_KEY?DPAD_DOWN_KEY:0;
         }
      }
      else
      {//First event
        key = (sum_delta_y < 0) ? DPAD_UP_KEY : DPAD_DOWN_KEY;
      }
      oj_data.Num_cont_in++;
      oj_data.pre_event=(sum_delta_y < 0) ? DPAD_UP_KEY : DPAD_DOWN_KEY;

    gv_TestSum_DeltaX = sum_delta_x;
    gv_TestSum_DeltaY = sum_delta_y;              
   }
   else
   {
      key = 0;
   }
   
   return key;
}


/*==================================================================================================

FUNCTION: blaster_mode_process_data

DESCRIPTION:
   This function keeps track of the sum for delta X and Y, and returns the directional key codes.

ARGUMENTS PASSED:
   int8_t delta_x        - delta X
   int8_t delta_y        - delta Y
   int64_t current_time  - the current time in ns

REFERENCE ARGUMENTS PASSED:
   None

RETURN VALUE:
   The directional keycode

DEPENDENCIES:
   None

SIDE EFFECTS:
   None

==================================================================================================*/
key_event_info_t blaster_mode_process_data(int8_t delta_x, int8_t delta_y, int64_t current_time)
{
   key_event_info_t key_info;

   static int64_t   last_time = 0;
   static int16_t   sum_delta_x = 0;
   static int16_t   sum_delta_y = 0;
   
   //DEBUG_OTP_PRINTK("[OTP] CRUCIALTEC OJ: current_time=%llu, last_time=%llu, delta sum_time=%llu\n", current_time, last_time, oj_data.delta_sum_time*MS_TO_NS_FACTOR);
   //DEBUG_OTP_PRINTK("[OTP] CRUCIALTEC OJ: current_time=%16lx, last_time=%16lx, delta_sum_time=%16lx\n", current_time, last_time, (oj_data.delta_sum_time*MS_TO_NS_FACTOR));
   
   if ( (current_time-last_time) < (oj_data.delta_sum_time*MS_TO_NS_FACTOR))
   {

Label_CT_test_nav_check:

      DEBUG_OTP_PRINTK("[OTP] time is < delta sum time\n");

      sum_delta_x += delta_x;
      sum_delta_y += delta_y;

      key_info.key_event = get_blaster_nav_key(sum_delta_x,
                                               sum_delta_y,
                                               oj_data.delta_min_in_x,
                                               oj_data.delta_min_in_y);

      if (key_info.key_event != 0)
      {
         sum_delta_x = 0;
         sum_delta_y = 0;
         
         oj_data.valid_key = true;

         DEBUG_OTP_PRINTK("[OTP] clear sum delta\n");
      }
   }
   else
   {
      oj_data.Num_cont_in  =  0;
      sum_delta_x = 0;
      sum_delta_y = 0;
      oj_data.valid_key = false;
      goto Label_CT_test_nav_check;

      DEBUG_OTP_PRINTK("[OTP] time is > delta sum time\n");
      
      if (oj_data.valid_key == false)
      {
         key_info.key_event = get_blaster_nav_key(sum_delta_x,
                                                  sum_delta_y,
                                                  oj_data.delta_min_out_x,
                                                  oj_data.delta_min_out_y);
      }
      else
      {
         key_info.key_event = 0;
      }

      sum_delta_x = 0;
      sum_delta_y = 0;
      oj_data.valid_key = false;
      
      DEBUG_OTP_PRINTK("[OTP] clear sum delta\n");
   }

   if ( (delta_x != 0) || (delta_y != 0) )
   {
      DEBUG_OTP_PRINTK("[OTP] set last time as current time\n");

      last_time = current_time; 
   }

   return key_info;
}


/*==================================================================================================

FUNCTION: blaster_mode_poll_work

DESCRIPTION:
    Pointer to the optical finger navigation's work_struct structure.

ARGUMENTS PASSED:
   int8_t delta_x        - delta X
   int8_t delta_y        - delta Y
   int64_t current_time  - the current time in ns

REFERENCE ARGUMENTS PASSED:
   None

RETURN VALUE:
   The directional keycode

DEPENDENCIES:
   None

SIDE EFFECTS:
   None

==================================================================================================*/
static void blaster_mode_poll_work(struct work_struct *work)
{
   uint8_t  shutter_upper = 0;
   uint8_t  shutter_lower = 0;
   uint16_t shutter = 0;
   uint8_t pixel_sum = 0;
   
   uint8_t motion_status = 0;
   int8_t delta_x = 0;
   int8_t delta_y = 0;
   int64_t current_time = 0;
   key_event_info_t  key_info;

   static uint16_t count_me = 0;

   DEBUG_OTP_PRINTK("[OTP] blaster_mode_poll_work\n");

   if (oj_data.suspend_mode == true)
   {
      DEBUG_OTP_PRINTK("[OTP] suspend mode - Done blaster mode polling\n");
   
      count_me = 0;
    
      oj_data.current_key_pressed = 0;
  
      return;
   }
      
   crucialtec_oj_read_register(oj_data.client, CRUCIALTEC_OJ_REG_MOTION, &motion_status);

   crucialtec_oj_read_register(oj_data.client, CRUCIALTEC_OJ_REG_DELTA_X, &delta_x);
   crucialtec_oj_read_register(oj_data.client, CRUCIALTEC_OJ_REG_DELTA_Y, &delta_y);

   crucialtec_oj_read_register(oj_data.client, CRUCIALTEC_OJ_REG_SHUTTER_MAX_HI, &shutter_upper);
   crucialtec_oj_read_register(oj_data.client, CRUCIALTEC_OJ_REG_SHUTTER_MAX_LO, &shutter_lower);
   crucialtec_oj_read_register(oj_data.client, CRUCIALTEC_OJ_REG_PIXEL_SUM,      &pixel_sum);

   shutter = (shutter_upper << 8) | shutter_lower;

   if (oj_data.jitter_check == 1)
   {
      if (crucialtec_check_for_jitter(shutter, pixel_sum) == false)
      {
         DEBUG_OTP_PRINTK("[OTP] Jitter detected! shutter=%d, pixel_sum=%d (blaster)\n",shutter, pixel_sum );

         oj_data.current_key_pressed = 0;
         oj_data.key_state = OJ_TOUCH_RELEASED;
         oj_data.fpd_timer = false;

         schedule_delayed_work(&blaster_mode_poll_workqueue, msecs_to_jiffies(5));

         return;
      }
   }

   DEBUG_OTP_PRINTK("[OTP] Register CRUCIALTEC_OJ_REG_MOTION  0x%x\n", motion_status);
   
   current_time = ktime_to_ns(ktime_get());

   if( ( delta_x != 0 ) || ( delta_y != 0 ) ) 
   {
        DEBUG_OTP_PRINTK("[OTP] delta_x=%d, delta_y=%d (blaster)\n",delta_x,delta_y);
        
      count_me=0;
      DEBUG_OTP_PRINTK("[OTP] count_me is cleared\n");
      key_info = blaster_mode_process_data(delta_x, delta_y, current_time);
      if ( (key_info.key_event != 0) && (is_ok_for_dpad_key() == true) )
      {
         input_report_key(oj_data.input_dev, key_info.key_event, 1);    // report key
         input_sync(oj_data.input_dev);

         msleep_interruptible(1);

         input_report_key(oj_data.input_dev, key_info.key_event, 0);    // report key
         input_sync(oj_data.input_dev);

         DEBUG_OTP_PRINTK("[OTP] sum_delta_x=%d, sum_delta_y=%d, key_event (blaster) %d\n", gv_TestSum_DeltaX, gv_TestSum_DeltaY, key_info.key_event);
         
         oj_data.dpad_key_release_time = ktime_to_ns(ktime_get());
      }
   }
   else
   {
      count_me++;
      DEBUG_OTP_PRINTK("[OTP] count_me = %d\n", count_me);
   }

   if (count_me <= oj_data.blaster_num_of_poll)
   {
      schedule_delayed_work(&blaster_mode_poll_workqueue, msecs_to_jiffies(oj_data.blaster_polling_rate));
   }
   else
   {
      DEBUG_OTP_PRINTK("[OTP] Done blaster mode polling\n");
   
      count_me = 0;
    
      oj_data.current_key_pressed = 0;

      crucialtec_oj_read_register(oj_data.client, CRUCIALTEC_OJ_REG_DELTA_X, &delta_x);
      crucialtec_oj_read_register(oj_data.client, CRUCIALTEC_OJ_REG_DELTA_Y, &delta_y);

      // clear the motion register
      crucialtec_oj_write_register(oj_data.client, CRUCIALTEC_OJ_REG_MOTION, 0x00);

      oj_enable_irq();
   }
}


/*==================================================================================================

FUNCTION: crucialtec_oj_irq_handler

DESCRIPTION: 
   This function services the Optical Finger Nagivation's Motion GPIO line.

ARGUMENTS PASSED:
   int   irq     - The interrupt request line to service.
   void *dev_id  - The pointer to the device to service.
    
REFERENCE ARGUMENTS PASSED:
   None

RETURN VALUE:
   IRQ_HANDLED - The interrupt request has been handled.

DEPENDENCIES:
   None

SIDE EFFECTS:
   None

==================================================================================================*/
static irqreturn_t crucialtec_oj_irq_handler (int irq, void *dev_id)
{
   struct crucialtec_oj_data *oj = (struct crucialtec_oj_data *)dev_id;

   DEBUG_OTP_PRINTK("[OTP] crucialtec_oj_irq_handler\n");  
   
//    oj_disable_irq();

#if OTP_SCHEDULE_DELAYED_WORK
//    queue_work(ojkey_wq, &oj->work);
//    queue_delayed_work(ojkey_wq, &&oj->work, 2);        
    schedule_delayed_work(&oj->work, 0);         
#else
    schedule_work(&oj->work);
#endif

   return IRQ_HANDLED;
}


/*==================================================================================================

FUNCTION: crucialtec_oj_toggle_reset_line

DESCRIPTION: 
   This function toggles the reset line as part of the initialization procedure.

ARGUMENTS PASSED:
   None
    
REFERENCE ARGUMENTS PASSED:
   None

RETURN VALUE:
   

DEPENDENCIES:
   None

SIDE EFFECTS:
   None

==================================================================================================*/


int crucialtec_oj_toggle_reset_line(void)
{
   int err = 0;
   
   //s3c_gpio_cfgpin(S5PC1XX_GPC(3), 0x00);
   
   //udelay(CRUCIALTEC_OJ_DELAY_RESET_LINE_FROM_LOW_TO_HIGH);
   
   // Set the reset line to low
  // err = s3c_gpio_cfgpin(S5PC1XX_GPH0(1), 0);//gpio_direction_output(S5PC1XX_GPC(3), 0);
  err = gpio_direction_output(OTP_RST, 0);
 
   if (err)
   {
      printk(KERN_ERR "[OTP] crucialtec_oj_toggle_reset_line: crucialtec_oj_toggle_reset_line: gpio_direction_output for input %d\n", 
             oj_data.reset_gpio);
      return false;
   }
   
   // Let the OJ's reset line stay low for awhile
   udelay(CRUCIALTEC_OJ_DELAY_RESET_LINE_FROM_LOW_TO_HIGH);
    
   // Set the reset Line to high
  // err = s3c_gpio_cfgpin(S5PC1XX_GPH0(1), 1);//gpio_direction_output(S5PC1XX_GPC(3), 1);
   err = gpio_direction_output(OTP_RST, 1);
 
   if (err)
   {
      printk(KERN_ERR "[OTP] crucialtec_oj_toggle_reset_line: crucialtec_oj_toggle_reset_line: gpio_direction_output %d\n", 
             oj_data.reset_gpio);
      return false;
   }

   // Waiting for the driver to be ready
   msleep_interruptible(CRUCIALTEC_OJ_DELAY_FOR_SYSTEM_TO_BE_READY);

   return true;
}


/*==================================================================================================

FUNCTION: crucialtec_oj_check_registers

DESCRIPTION: 
   This function checks the default values in the OJ's registers.  This is part of the init process.

ARGUMENTS PASSED:
   i2c_client *client  - pointer to the device's data
    
REFERENCE ARGUMENTS PASSED:
   None

RETURN VALUE:
   true  - matches default values
   false - doesn't match default values

DEPENDENCIES:
   None

SIDE EFFECTS:
   None

==================================================================================================*/
bool crucialtec_oj_check_registers(struct i2c_client *client)
{
   int i;
   uint8_t value = 0;
   crucialtec_oj_reg_data_t *cfg;
   
   cfg = oj_check_regs;

   DEBUG_OTP_PRINTK("[OTP] Read Check Registers Configuration - start\n");
   
   for (i=0; cfg[i].reg !=0; i++)
   {
      crucialtec_oj_read_register(client, cfg[i].reg, &value);
      DEBUG_OTP_PRINTK("[OTP] Reg 0x%x - Table Value 0x%x - Read Value 0x%x\n", cfg[i].reg, cfg[i].data, value);
      
      if (cfg[i].data != value)
      {
         printk(KERN_ERR "[OTP] crucialtec_oj_check_registers: default register check failure: Reg 0x%x - default value is 0x%x - read value is - 0x%x\n", 
                cfg[i].reg, cfg[i].data, value);
         return false;
      }
   }

   DEBUG_OTP_PRINTK("[OTP] Read Check Registers Configuration - end\n");

   return true;
}


/*==================================================================================================

FUNCTION: crucialtec_oj_configure_registers

DESCRIPTION: 
   This function configures some of the main OJ's registers.

ARGUMENTS PASSED:
   i2c_client *client  - pointer to the device's data
    
REFERENCE ARGUMENTS PASSED:
   None

RETURN VALUE:
   None

DEPENDENCIES:
   None

SIDE EFFECTS:
   None

==================================================================================================*/
void crucialtec_oj_configure_registers(struct i2c_client *client)
{
   uint8_t value = 0;

   // Set the Engine register:
   //     Optical Finger Navigation Engine -- ON 
   //     Speed Switching -- ON
   crucialtec_oj_write_register(client, 
                                CRUCIALTEC_OJ_REG_ENGINE, 
                                CRUCIALTEC_OJ_REG_ENGINE_VALUE_DEFAULT);

   // Set Resolution:
   //    Wakeup Resolution from reset mode-- 500cpi
   //    Default Resolution -- 500cpi
   crucialtec_oj_write_register(client,
                                CRUCIALTEC_OJ_REG_RESOLUTION,
                                (CRUCIALTEC_OJ_REG_RESOLUTION_VALUE_WAKEUP_RES_500_CPI |
                                  CRUCIALTEC_OJ_REG_RESOLUTION_VALUE_RES_500_CPI) );

   // Set Speed control:
   //     Y Scaling factor -- OFF
   //     X Scaling factor -- OFF
   //     Speed switching checking interval -- 16ms
   //     Low cpi when in speed switching -- 250cpi
   //     Hi cpi when in speed switching -- Disable
   crucialtec_oj_write_register(client,
                                CRUCIALTEC_OJ_REG_SPEED_CONTROL,
                                (CRUCIALTEC_OJ_REG_SPEED_CONTROL_VALUE_Y_SCALE_OFF |
                                 CRUCIALTEC_OJ_REG_SPEED_CONTROL_VALUE_X_SCALE_OFF |
                                 CRUCIALTEC_OJ_REG_SPEED_CONTROL_VALUE_SP_INT_VAL_16_MS |
                                 CRUCIALTEC_OJ_REG_SPEED_CONTROL_VALUE_LOW_CPI_ON_250_CPI |
                                 CRUCIALTEC_OJ_REG_SPEED_CONTROL_VALUE_HIGH_CPI_OFF) );

   // Check OJ's registers
   // crucialtec_oj_check_registers(client);

   // Read these registers after the driver configuration
   //  CRUCIALTEC_OJ_REG_MOTION (0x02), CRUCIALTEC_OJ_REG_DELTA_X (0x03), CRUCIALTEC_OJ_REG_DELTA_Y (0x04)

   crucialtec_oj_read_register(client, CRUCIALTEC_OJ_REG_MOTION, &value);
   DEBUG_OTP_PRINTK("[OTP] CRUCIALTEC_OJ_REG_MOTION Reg 0x02 - Value 0x%x\n", value);
   
   crucialtec_oj_read_register(client, CRUCIALTEC_OJ_REG_DELTA_X, &value);
   DEBUG_OTP_PRINTK("[OTP] CRUCIALTEC_OJ_REG_DELTA_X Reg 0x03 - Value 0x%x\n", value);

   crucialtec_oj_read_register(client, CRUCIALTEC_OJ_REG_DELTA_Y, &value);
   DEBUG_OTP_PRINTK("[OTP] CRUCIALTEC_OJ_REG_DELTA_Y Reg 0x04 - Value 0x%x\n", value);
   
   crucialtec_oj_read_register(client, CRUCIALTEC_OJ_REG_ENGINE, &value);
   DEBUG_OTP_PRINTK("[OTP] CRUCIALTEC_OJ_REG_ENGINE Reg 0x60 - Value 0x%x\n", value);


   // Swap the X and Y values 
  // if (CRUCIALTEC_OJ_ORIENTATION_POSITION_SWAP_X_TO_Y)
   if (0)
   {
      crucialtec_oj_write_register(client, 
                                   CRUCIALTEC_OJ_REG_ORIENTATION_CTRL, 
                                   CRUCIALTEC_OJ_REG_ORIENTATION_CTRL_VALUE_DEFAULT);
   }
}


/*==================================================================================================

FUNCTION: crucialtec_oj_init_hardware

DESCRIPTION: 
   This function initializes and configures the OJ driver.

ARGUMENTS PASSED:
   i2c_client *client  - pointer to the device's data
    
REFERENCE ARGUMENTS PASSED:
   None

RETURN VALUE:
   

DEPENDENCIES:
   None

SIDE EFFECTS:
   None

==================================================================================================*/
bool crucialtec_oj_init_hardware(struct i2c_client *client)
   {
      int err = 0;     
      
      // Set the reset line to low
      err = gpio_direction_output(OTP_RST, 0);
    
      if (err)
      {
         printk(KERN_ERR "[OTP] crucialtec_oj_toggle_reset_line: gpio_direction_output for input %d\n", 
                oj_data.reset_gpio);
         return false;
      }
      
      // Let the OJ's reset line stay low for awhile
      udelay(CRUCIALTEC_OJ_DELAY_RESET_LINE_FROM_LOW_TO_HIGH);
       
      // Set the reset Line to high
      err = gpio_direction_output(OTP_RST, 1);
      if (err)
      {
         printk(KERN_ERR "[OTP] crucialtec_oj_toggle_reset_line: gpio_direction_output %d\n", 
                oj_data.reset_gpio);
         return false;
      }
   
      // Waiting for the driver to be ready
      mdelay(CRUCIALTEC_OJ_DELAY_FOR_SYSTEM_TO_BE_READY);
   
      return true;
   }

static int crucialtec_open(struct inode *inode, struct file *filp)
{
   DEBUG_OTP_PRINTK("[OTP] crucialtec_open \n");

   return 0;
}
static int crucialtec_release(struct inode *inode, struct file *filp)
{
   DEBUG_OTP_PRINTK("[OTP] crucialtec_release \n");

   return 0;
}

#define MOUSE_MODE "mouse"

static ssize_t crucialtec_write (struct file *file, const char __user *buf, size_t count, loff_t *ppos)
{

/*
    int mode; 
    mode = (int)buf[0];

    printk("#####crucialtec_write buf=%c, count= %d, mode = %d\n",buf,count,mode);

    if(mode==1)
        oj_data.Mode_Change = CRU_KEY_MODE;
    else if(mode == CRU_TEN_MODE)
      oj_data.Mode_Change = CRU_TEN_MODE;
    else if(mode == CRU_ELEVEN_MODE)
      oj_data.Mode_Change = CRU_ELEVEN_MODE;
    else if(mode == CRU_TWELVE_MODE)
      oj_data.Mode_Change = CRU_TWELVE_MODE;
    else
        oj_data.Mode_Change = CRU_MOUSE_MODE;
*/
    return (ssize_t)count;

}

static ssize_t crucialtec_read (struct file *file, char __user *buf, size_t count, loff_t *ppos)
{
    char temp[CRUCIALTEC_OJ_MISC_TEMP_BUFFER_SIZE];

    sprintf(temp,"%d %d %d", oj_data.deltaX,oj_data.deltaY,oj_data.Mode_Change);

    if (copy_to_user(buf, temp, strlen(temp)))
    {
        DEBUG_OTP_PRINTK("[OTP] copy_to_user error!\n");
        return -EFAULT;
    }
    DEBUG_OTP_PRINTK("[OTP] crucialtec_read =%s, len=%d\n",temp,strlen(temp));
    //file->private_data = (void *)1;

    return strlen(temp);    
}

static const struct file_operations crucialtec_fops = {
	.owner = THIS_MODULE,
	.read = crucialtec_read,
	.write = crucialtec_write,
	.open = crucialtec_open,
	.mmap = NULL, //v4l2_mmap,
	.ioctl = NULL, //v4l2_ioctl,
	.release = crucialtec_release,
	.poll = NULL, //v4l2_poll,
	.llseek = NULL, //no_llseek,
};

static struct miscdevice crucialtec_oj_miscdev = {
   .minor     = OTP_MAJOR,
   .name      = "otpmisc",
   .fops      = &crucialtec_fops,
};

static int otp_hw_reset(void)
{
    printk(KERN_INFO "[OTP] otp_hw_reset\n");
   
    //HW reset
    //Set NRST pin to Low;
    if (gpio_direction_output(OTP_RST, 0))
        printk(KERN_ERR "[OTP] otp_hw_reset Fail : reset_gpio Low %d\n", OTP_RST);

    //NRST pulse width, Minimum 20us
    udelay(20);    

    //Set NRST pin to High;
    if (gpio_direction_output(OTP_RST, 1))
        printk(KERN_ERR "[OTP] otp_hw_reset Fail : reset_gpio High %d\n", OTP_RST);

    //Motion delay after reset, Minimum 3.5ms, Maximum 23ms
    msleep(23);

    return 0;
}


static int otp_power_mode(int onoff)
{
#if 0
    unsigned long otp_shdn_mfp = 0;
    int gpio_otp_shdn = OTP_SHDN;
#endif    
    printk(KERN_INFO "[OTP] otp_power_mode (%d)\n", onoff);
      
    if(onoff)
    {
        int8_t delta_x, delta_y;
#if 0    
	/* Set correct pin status when soc enters low power mode*/
	otp_shdn_mfp = pxa3xx_mfp_read(gpio_otp_shdn);

	/*set wlan_pd pin to output low in low power
		mode to save power in low power mode */
	otp_shdn_mfp &= ~0x100;
	pxa3xx_mfp_write(gpio_otp_shdn, otp_shdn_mfp & 0xffff);
#endif    
    
        //Set SHUTDOWN pin to Low
        if (gpio_direction_output(OTP_SHDN, 0))
            printk(KERN_ERR "[OTP] otp_power_mode(%d) Fail :  shutdown_gpio %d\n", onoff, OTP_SHDN);

        //Reset wait time after stable supply voltage, Minimum 100ms
        msleep(100);

        //HW reset
        //Set NRST pin to Low;       
        if (gpio_direction_output(OTP_RST, 0))
            printk(KERN_ERR "[OTP] otp_power_mode(%d) Fail : reset_gpio Low %d\n", onoff,  OTP_RST);


        //NRST pulse width, Minimum 20us
        udelay(20);

        //Set NRST pin to High;        
        if (gpio_direction_output(OTP_RST, 1))
            printk(KERN_ERR "[OTP] otp_power_mode(%d) Fail :  reset_gpio High %d\n",onoff,  OTP_RST);

        //Motion delay after reset, Minimum 3.5ms, Maximum 23ms
        msleep(23);

        //Dummpy Read for Buffer Clear        
        crucialtec_oj_configure_registers(oj_data.client);
        crucialtec_oj_read_register(oj_data.client, CRUCIALTEC_OJ_REG_DELTA_X, &delta_x);
        crucialtec_oj_read_register(oj_data.client, CRUCIALTEC_OJ_REG_DELTA_Y, &delta_y);

        // clear the motion register
        crucialtec_oj_write_register(oj_data.client, CRUCIALTEC_OJ_REG_MOTION, 0x00);

    }
    else
    {
#if 0        
        //Set NRST pin to Low;
        if (gpio_direction_output(OTP_RST, 0))
            printk(KERN_ERR "[OTP] otp_power_mode(%d) Fail : reset_gpio Low %d\n", onoff,  OTP_RST);
        
        //Reset wait time after stable supply voltage, Minimum 100ms
        msleep(100);        
#endif        
#if 0        
	/* Set correct pin status when soc enters low power mode*/
	otp_shdn_mfp = pxa3xx_mfp_read(gpio_otp_shdn);

	/* set wlan_pd pin to output high in low power
		mode to ensure 8787 is not power off in low power mode*/
	otp_shdn_mfp |= 0x100;
	pxa3xx_mfp_write(gpio_otp_shdn, otp_shdn_mfp & 0xffff);
#endif        
        //Set Shutdown pin to High       
        if (gpio_direction_output(OTP_SHDN, 1))
            printk(KERN_ERR "[OTP] otp_power_mode(%d) Fail :  shutdown_gpio %d\n", onoff, OTP_SHDN);

        //Delay 150ms
        msleep(2);             
    }

    return 0;

}

#if OTP_SCHEDULE_DELAYED_WORK
static void otp_motion_continue_work(void)
{	
    if(!gpio_get_value(OTP_INT))
    {         
        DEBUG_OTP_PRINTK("[OTP] INT_LOW(continue) : %d\n", ++gv_MotionWork_Cnt);              

        schedule_delayed_work(&oj_data.work, 2 /*msecs_to_jiffies(oj_data.blaster_polling_rate)*/);        
	//queue_work(ojkey_wq, &oj_data.work);        
        //queue_delayed_work(ojkey_wq, &oj_data.work, 2);                  
    }
    else
    {       
        DEBUG_OTP_PRINTK("[OTP] INT_HIGH : Released!\n");                
        gv_MotionWork_Cnt=0;
        
        //oj_enable_irq();
    }
}


static void otp_motion_start_work(struct work_struct *work)
{
   uint8_t  shutter_upper = 0;
   uint8_t  shutter_lower = 0;
   uint16_t shutter = 0;
   uint8_t pixel_sum = 0;   
   uint8_t motion_status = 0;
   int8_t delta_x = 0;
   int8_t delta_y = 0;
   key_event_info_t  key_info;

   DEBUG_OTP_PRINTK("[OTP] otp_motion_start_work\n");
     
   crucialtec_oj_read_register(oj_data.client, CRUCIALTEC_OJ_REG_MOTION, &motion_status);

   crucialtec_oj_read_register(oj_data.client, CRUCIALTEC_OJ_REG_DELTA_X, &delta_x);
   crucialtec_oj_read_register(oj_data.client, CRUCIALTEC_OJ_REG_DELTA_Y, &delta_y);

   crucialtec_oj_read_register(oj_data.client, CRUCIALTEC_OJ_REG_SHUTTER_UPPER, &shutter_upper);
   crucialtec_oj_read_register(oj_data.client, CRUCIALTEC_OJ_REG_SHUTTER_LOWER, &shutter_lower);
   crucialtec_oj_read_register(oj_data.client, CRUCIALTEC_OJ_REG_PIXEL_SUM,      &pixel_sum);

   shutter = (shutter_upper << 8) | shutter_lower;

   if (oj_data.jitter_check == 1)
   {
      if (crucialtec_check_for_jitter(shutter, pixel_sum) == false)
      {
         printk(KERN_INFO "[OTP] Jitter detected! shutter=%d, pixel_sum=%d\n",shutter, pixel_sum );

         oj_data.current_key_pressed = 0;
         oj_data.key_state = OJ_TOUCH_RELEASED;
         oj_data.fpd_timer = false;

        otp_motion_continue_work();
        return;
      }
   }

   DEBUG_OTP_PRINTK("[OTP] Register CRUCIALTEC_OJ_REG_MOTION  0x%x\n", motion_status);
      
    gv_CurrTime = ktime_get();	
    gv_SubTime = ktime_sub(gv_CurrTime,gv_PrevTime);

    DEBUG_OTP_PRINTK("[OTP] delta_x=%d, delta_y=%d, gv_SubTime sec = %d, gv_SubTime msec = %d \n",delta_x, delta_y, gv_SubTime.tv.sec, (gv_SubTime.tv.nsec/MS_TO_NS_FACTOR));

    if ( (gv_SubTime.tv.sec == 0) && (gv_SubTime.tv.nsec <= (oj_data.delta_sum_time*MS_TO_NS_FACTOR)))
    {

        DEBUG_OTP_PRINTK("[OTP] time is <= delta sum time\n");        
      
        gv_Sum_DeltaX += delta_x;
        gv_Sum_DeltaY += delta_y;

        key_info.key_event = get_blaster_nav_key(gv_Sum_DeltaX, gv_Sum_DeltaY, 
            gv_otp_Xsensitivity[oj_data.delta_min_in_x], gv_otp_Ysensitivity[oj_data.delta_min_in_y]);

        if ( (key_info.key_event != 0) /*&& (is_ok_for_dpad_key() == true)*/ )
        {
            input_report_key(oj_data.input_dev, key_info.key_event, 1);    // report key
            input_sync(oj_data.input_dev);

            //msleep(1);

            input_report_key(oj_data.input_dev, key_info.key_event, 0);    // report key
            input_sync(oj_data.input_dev);

            DEBUG_OTP_PRINTK("[OTP] gv_SubTime sec = %d, gv_SubTime msec = %d \n",gv_SubTime.tv.sec, (gv_SubTime.tv.nsec/MS_TO_NS_FACTOR));
            printk(KERN_INFO "[OTP] [%2d] SUMX:[%4d] SUMY:[%4d], KEY:[%3d]\n", oj_data.delta_min_in_x, gv_Sum_DeltaX, gv_Sum_DeltaY, key_info.key_event);

            oj_data.valid_key = true;            
            gv_Sum_DeltaX = 0;
            gv_Sum_DeltaY = 0;                 
            gv_PrevTime = ktime_set(0,0);
        }

    }    
    else
    {        
        oj_data.Num_cont_in=0;                
        oj_data.valid_key = false;        
        gv_Sum_DeltaX = delta_x;
        gv_Sum_DeltaY = delta_y;
        
        DEBUG_OTP_PRINTK("[OTP] time is > delta sum time\n");                
    }      
   
   if( ( delta_x != 0 ) || ( delta_y != 0 ) ) 
   {
	gv_PrevTime = ktime_get();	
   }

    otp_motion_continue_work();

}
#endif

static ssize_t ojkey_sumXY_fs_read(struct device *dev, struct device_attribute *attr, char *buf)
{
	int count = 0;

	count = sprintf(buf,"%d,%d\n", gv_TestSum_DeltaX, gv_TestSum_DeltaY);

	return count;
}

static ssize_t ojkey_level_store(struct device *dev,struct device_attribute *attr, const char *buf, size_t count)
{    
	int rc;
	unsigned long level;

	rc = strict_strtoul(buf, 0, &level);
	if (rc)
	{
                printk(KERN_INFO "[OTP] ojkey_level_store : rc = %d\n", rc);
		return rc;
	}

	rc = -ENXIO;
	mutex_lock(&oj_data.ops_lock);

        printk(KERN_INFO "[OTP] ojkey_level_store : Level = %lu\n", level);

        if((level >= 0) && (level < OTP_SENSITIVITY_MAX_LEVEL))
        {
            oj_data.delta_min_in_x = level;
            oj_data.delta_min_in_y = level;
        }

        rc = count;
    
	mutex_unlock(&oj_data.ops_lock);

	return rc;
}


static struct device_attribute dev_attr_set_level =
	__ATTR(ojkeylevel, 0664 , NULL, ojkey_level_store);

static struct device_attribute dev_attr_get_ojkeysum =
	__ATTR(ojkeysum, 0444, ojkey_sumXY_fs_read, NULL);


static struct attribute *otp_sysfs_attrs[] = {
	&dev_attr_set_level.attr,
	&dev_attr_get_ojkeysum.attr,
	NULL
};

static struct attribute_group otp_attribute_group = {
	.attrs = otp_sysfs_attrs,
};

/*==================================================================================================

FUNCTION: crucialtec_oj_probe

DESCRIPTION: 
   This function initializes and registers the optical joystick device.

ARGUMENTS PASSED:
   i2c_client *client  - pointer to the device's data
   i2c_device_id *id   - pointer to the device's id
    
REFERENCE ARGUMENTS PASSED:
   None

RETURN VALUE:
   None

DEPENDENCIES:
   None

SIDE EFFECTS:
   None

==================================================================================================*/
static int crucialtec_oj_probe (struct i2c_client *client, const struct i2c_device_id *id)
{
   int err = 0;
   //struct crucial_oj_platform_data *pdev = NULL;
   uint8_t product_id = 0;
   uint8_t revision_id = 0;   

   printk(KERN_INFO "[OTP] crucialtec_oj_probe\n");   

   if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C))
   {
      dev_err(&client->dev, " [OTP] %s(%s): No supported I2C function \n", __FILE__, __FUNCTION__);
      err = (-ENOTSUPP);
     // goto probe_error_free_gpio;
   }

   oj_data.client = client;

   //oj_data.motion_gpio   = pdev->gpio_motion_irq;
   oj_data.shutdown_gpio = OTP_SHDN; 
   oj_data.reset_gpio    = OTP_RST;//pdev->gpio_reset;
   oj_data.Mode_Change = CRU_TWELVE_MODE;
   oj_data.Mode_EZChange= CRU_KEY_MODE;//CRU_KEY_MODE; 
   // Set the OJ's data 
   i2c_set_clientdata(client, &oj_data);
   printk(KERN_INFO  "[OTP] slave address 0x%02x\n", client->addr);

   // Configure and Register as an Input Device //
   
   // Allocate the device
   oj_data.input_dev = input_allocate_device();
   if (oj_data.input_dev == NULL)
   {
      printk(KERN_ERR "[OTP] CRUCIALTEC input add error \n"); 
      err = (-ENOMEM);
      goto probe_error_free_clientdata;
   }

    if (gpio_request(OTP_SHDN, "OTP_SHDN"))
        printk(KERN_ERR "[OTP] OTP_SHDN Request GPIO_%d failed!\n", OTP_SHDN);        
    if (gpio_direction_output(OTP_SHDN, 0))
        printk(KERN_ERR "[OTP] crucialtec_oj_probe Fail :  shutdown_gpio %d\n", OTP_SHDN);

    printk(KERN_ERR "[OTP] crucialtec_oj_probe : shutdown_gpio %d\n", OTP_SHDN);

    if (gpio_request(OTP_RST, "OTP_RST"))
	    printk(KERN_ERR "[OTP] OTP_RST Request GPIO_%d failed!\n", OTP_RST);
    if (gpio_direction_output(OTP_RST, 1))
            printk(KERN_ERR "[OTP] crucialtec_oj_probe Fail : reset_gpio Low %d\n", OTP_RST);
    
    printk(KERN_ERR "[OTP] crucialtec_oj_probe : reset_gpio %d\n", OTP_RST);

   // Get the Product ID and and Revision ID
   printk(KERN_INFO "[OTP] CRUCIALTEC Get Product ID +\n"); 
   crucialtec_oj_read_register(client, CRUCIALTEC_OJ_REG_PRODUCT_ID, &product_id);
   crucialtec_oj_read_register(client, CRUCIALTEC_OJ_REG_REVISION_ID, &revision_id);
   printk(KERN_INFO "[OTP] CRUCIALTEC Get Product ID -\n"); 

   // Set the information of the device
   oj_data.input_dev->name           = CRUCIALTEC_OJ_PRODUCT_NAME; //pdev->name;
   oj_data.input_dev->id.bustype     = BUS_I2C;
   oj_data.input_dev->phys = "ojkey/input0";   
   oj_data.input_dev->id.vendor      = CRUCIALTEC_VENDOR_ID; 
   oj_data.input_dev->id.product     = product_id;
   oj_data.input_dev->id.version     = revision_id;

   // Init the data we are tracking 
   oj_data.operational_mode  = OPERATIONAL_MODE_DIRECTIONAL_PAD_BLASTER_MODE;
   oj_data.valid_delta       = CRUCIALTEC_OJ_MAX_DISTANCE_TRAVELED; 
   oj_data.delay_between_key = OJ_SENSITIVITY_DELAY_BETWEEN_KEYS_MEDIUM;
   oj_data.current_key_pressed = 0;
   oj_data.key_state         = OJ_TOUCH_RELEASED;
   oj_data.fpd_timer         = false;
   oj_data.valid_squal       = CRUCIALTEC_OJ_VALID_SQUAL;
   oj_data.jitter_check      = CRUCIALTEC_OJ_JITTER_CHECK;
   oj_data.als_register      = false;
   oj_data.outdoor_light     = false;
   oj_data.outdoor_light_level = CRUCIALTEC_OJ_OUTDOOR_LIGHT_LEVEL;
   oj_data.suspend_mode      = false;
   oj_data.dpad_key_release_time = 0;
   oj_data.tsb_keyguard_delay = CRUCIALTEC_OJ_MAX_DELAY_BETWEEN_OJ_AND_TSB_KEYS;
   oj_data.dpad_dir_key_delay_after_dome_pressed = 
      CRUCIALTEC_OJ_DELAY_BETWEEN_OJ_DPAD_DIRECTIONAL_KEYS_AND_OJ_DOME_KEY;

   oj_data.delta_sum_time = OJ_SENSITIVITY_SUM_TIME_MEDIUM;
   oj_data.delta_sum_cp   = DELTA_SUM_CP;
   oj_data.delta_min_in_x = OTP_SENSITIVITY_X_DEFAULT;
   oj_data.delta_min_in_y = OTP_SENSITIVITY_Y_DEFAULT;
   oj_data.delta_min_out_x = OTP_SENSITIVITY_X_DEFAULT;
   oj_data.delta_min_out_y = OTP_SENSITIVITY_Y_DEFAULT;
   oj_data.blaster_polling_rate = CT_4_WAY_NAV_POLLING_RATE;
   oj_data.blaster_num_of_poll  = CT_4_WAY_NUM_OF_POLL;
   oj_data.valid_key       = false;
   oj_data.Num_cont_in     = 0;
   oj_data.pre_event       = 0;
   oj_data.SumDeltaX = 0;
   oj_data.SumDeltaY = 0;
    oj_data.current_X = CENTER_X;
    oj_data.current_Y = CENTER_Y;
   
   // Configure the properties of this device for the Input system
   set_bit(EV_KEY,         oj_data.input_dev->evbit);        // device has keys
   set_bit(EV_SW,          oj_data.input_dev->evbit); 

   oj_data.input_dev->evbit[0] = oj_data.input_dev->evbit[0] = BIT_MASK(EV_SYN) | BIT_MASK(EV_KEY) | BIT_MASK(EV_ABS);
   oj_data.input_dev->keybit[BIT_WORD(BTN_TOUCH)] = BIT_MASK(BTN_TOUCH);
   
   set_bit(KEY_UP,    oj_data.input_dev->keybit);       // keys for this device //UP
   set_bit(KEY_DOWN,  oj_data.input_dev->keybit);  //Down
   set_bit(KEY_RIGHT, oj_data.input_dev->keybit);   //Right
   set_bit(KEY_LEFT,  oj_data.input_dev->keybit);   //Left
   //set_bit(KEY_ENTER, oj_data.input_dev->keybit);  //Ok
   //set_bit(KEY_ZOOM_IN, oj_data.input_dev->keybit);  //Zoom_In
   //set_bit(KEY_ZOOM_OUT, oj_data.input_dev->keybit);  //Zoom_Out
   set_bit(EV_REL,         oj_data.input_dev->evbit);        // device has relative event
   set_bit(REL_X,          oj_data.input_dev->relbit);       // relative data for this device
   set_bit(REL_Y,          oj_data.input_dev->relbit);
   set_bit(BTN_LEFT,      oj_data.input_dev->keybit);
   set_bit(BTN_RIGHT,      oj_data.input_dev->keybit);
   //set_bit(KEY_ENTER,      oj_data.input_dev->keybit);
      set_bit(ABS_X,      oj_data.input_dev->absbit);
   set_bit(ABS_Y,      oj_data.input_dev->absbit);
   set_bit(ABS_Z,      oj_data.input_dev->absbit);   

   input_set_abs_params(oj_data.input_dev, ABS_X, 0, 800, 0, 0);
   input_set_abs_params(oj_data.input_dev, ABS_Y, 0, 480, 0, 0);
   //input_set_abs_params(oj_data.input_dev, ABS_PRESSURE, 0, 1, 0, 0);

    mutex_init(&oj_data.ops_lock);

   // Register the device 
   err = input_register_device(oj_data.input_dev);

   if (err)
   {
      dev_err(&client->dev, " %s(%s): Unable to register %s input device\n", __FILE__, __FUNCTION__, oj_data.input_dev->name);
      printk(KERN_ERR "[OTP] CRUCIALTEC input add error  11\n"); 

      goto probe_error_free_device;
   }

    gv_CurrTime = ktime_set(0,0);
    gv_PrevTime = ktime_set(0,0);

    /*sys_fs*/
    err = sysfs_create_group(&oj_data.input_dev->dev.kobj,&otp_attribute_group);
    if (err) {
        printk(KERN_ERR "[OTP] Creating sysfs attribute group failed\n");
        goto probe_error_free_device;
    }
    
#if OTP_SCHEDULE_DELAYED_WORK
    INIT_DELAYED_WORK(&oj_data.work, otp_motion_start_work);
    //INIT_WORK(&oj_data.work, otp_motion_start_work);   
#else
    INIT_WORK(&oj_data.work, crucial_otp_work_func);   
#endif

   /* Set the IRQ handler */
   if (client->irq)
   {
    
      err = request_irq(client->irq, crucialtec_oj_irq_handler, IRQF_TRIGGER_FALLING, CRUCIALTEC_OJ_PRODUCT_NAME, &oj_data);
      
      if (err == 0)
      {
         printk(KERN_INFO "[OTP] Request IRQ Success\n");
         err = set_irq_wake(client->irq, 1);
         oj_data.irq_count = 1;
      }
      else
      {
         dev_err(&client->dev, " %s(%s): Request IRQ  %d  failed\n", __FILE__, __FUNCTION__, client->irq);
         printk(KERN_ERR "[OTP] Request IRQ Fail! \n");
         goto probe_error_free_irq;         
      }
   }

#if 0
   if (CRUCIALTEC_OJ_MISC_DEVICE_SUPPORT==1)
   {
      err = misc_register (&crucialtec_oj_miscdev);
      
      if (err) 
      {
         dev_err(&client->dev, " %s(%s): Unable to register %s misc device\n",
                 __FILE__, __FUNCTION__, oj_data.input_dev->name);
   
         goto probe_error_free_irq;
      }
   }
#endif

    // sync the device information
    //  input_sync(oj_data.input_dev);
    //  schedule_delayed_work(&fpd_start_workqueue, FPD_START_TIMER);

    err = misc_register (&crucialtec_oj_miscdev);
    if (err) 
        printk(KERN_ERR "[OTP] misc_register error\n");

#ifdef CONFIG_HAS_EARLYSUSPEND
	oj_data.early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	oj_data.early_suspend.suspend = crucialtec_oj_early_suspend;
	oj_data.early_suspend.resume = crucialtec_oj_late_resume;
	register_early_suspend(&oj_data.early_suspend);
#endif

   oj_data.initialized = 1; 

#ifdef CONFIG_CRUCIALTEC_OJ_ESD_RECOVERY
   ESD_POLL_START(crucialtec_oj_check_esd, 0);

   oj_data.esd_polling = 1;
#endif

    otp_power_mode(1);

    printk(KERN_INFO "[OTP] crucialtec_oj_probe End!!\n");   

   return(err);

probe_error_free_irq:
   free_irq(client->irq, &oj_data);

probe_error_free_device:
   input_free_device(oj_data.input_dev);

probe_error_free_clientdata:
   i2c_set_clientdata(client, NULL);
   
#if 0
probe_error_free_gpio:
   gpio_free(oj_data.reset_gpio);
   gpio_free(oj_data.shutdown_gpio);
#endif   

   return (err);
}


/*==================================================================================================

FUNCTION: crucialtec_oj_suspend

DESCRIPTION: 
   This function suspends the OJ device.

ARGUMENTS PASSED:
   i2c_client *client  - pointer to the device's data
   pm_message_t msg    - power management message
       
REFERENCE ARGUMENTS PASSED:
   None

RETURN VALUE:
   None

DEPENDENCIES:
   None

SIDE EFFECTS:
   None

==================================================================================================*/

static int crucialtec_oj_suspend(struct i2c_client *client, pm_message_t msg)
{
    struct crucialtec_oj_data *oj = i2c_get_clientdata(client);

    printk(KERN_INFO "[OTP] crucialtec_oj_suspend+\n");

#ifdef CONFIG_CRUCIALTEC_OJ_ESD_RECOVERY
    if (oj_data.esd_polling == 1)
    {
    esd_poll_stop(crucialtec_oj_check_esd);
    oj_data.esd_polling = 0;
    }
#endif

    oj_disable_irq();

#if OTP_SCHEDULE_DELAYED_WORK   
    cancel_delayed_work_sync(&oj->work);
#else
    cancel_work_sync(&oj->work);
#endif

    if(oj_data.suspend_mode == false)
    {
        otp_power_mode(0);
    }

    oj_data.suspend_mode = true;

    printk(KERN_INFO "[OTP] crucialtec_oj_suspend-\n");
   
   return 0;
}


/*==================================================================================================

FUNCTION: crucialtec_oj_resume

DESCRIPTION: 
   This function resumes the OJ device.

ARGUMENTS PASSED:
   i2c_client *client  - pointer to the device's data
    
REFERENCE ARGUMENTS PASSED:
   None

RETURN VALUE:
   None

DEPENDENCIES:
   None

SIDE EFFECTS:
   None

==================================================================================================*/
static int crucialtec_oj_resume(struct i2c_client *client)
{
   //int err;

    printk(KERN_INFO "[OTP] crucialtec_oj_resume+\n");

   if (oj_data.operational_mode == OPERATIONAL_MODE_OFF)
   {
      // The OJ's mode is OFF, so do not resume
      return 0;
   }

    if(oj_data.suspend_mode == true)
    {
        otp_power_mode(1);
    }

    oj_data.suspend_mode = false;

    oj_enable_irq();

#ifdef CONFIG_CRUCIALTEC_OJ_ESD_RECOVERY
    if (oj_data.esd_polling == 0)
    {
        ESD_POLL_START(crucialtec_oj_check_esd, 0);
        oj_data.esd_polling = 1;
    }
#endif

    printk(KERN_INFO "[OTP] crucialtec_oj_resume-\n");

   return 0;
}


#ifdef CONFIG_HAS_EARLYSUSPEND
static void crucialtec_oj_early_suspend (struct early_suspend *h)
{

   DEBUG_OTP_PRINTK("[OTP] crucialtec_oj_early_suspend\n");
    crucialtec_oj_suspend (oj_data.client, PMSG_SUSPEND);
}

static void crucialtec_oj_late_resume (struct early_suspend *h)
{
    DEBUG_OTP_PRINTK("[OTP] crucialtec_oj_late_resume\n");
    crucialtec_oj_resume (oj_data.client);
}
#endif


/*==================================================================================================

FUNCTION: crucialtec_oj_remove

DESCRIPTION: 
   This function removes the OJ device from the input system.

ARGUMENTS PASSED:
   i2c_client *client  - pointer to the device's data
    
REFERENCE ARGUMENTS PASSED:
   None

RETURN VALUE:
   None

DEPENDENCIES:
   None

SIDE EFFECTS:
   None

==================================================================================================*/
static int crucialtec_oj_remove (struct i2c_client *client)
{
   struct crucialtec_oj_data *oj = i2c_get_clientdata(client);

   printk(KERN_INFO "[OTP] crucialtec_oj_remove\n");
   
   dev_dbg(&client->dev, "%s: enter. \n", __FUNCTION__);
   
   free_irq(client->irq, oj);
   
   gpio_free(oj_data.reset_gpio);
   
   gpio_free(oj_data.shutdown_gpio);
   
   input_unregister_device(oj->input_dev);

#if 1
   //if (CRUCIALTEC_OJ_MISC_DEVICE_SUPPORT==1)
   //{
      misc_deregister(&crucialtec_oj_miscdev);
   //}
#endif 
#ifdef CONFIG_CRUCIALTEC_OJ_ESD_RECOVERY
   if (oj_data.esd_polling == 1)
   {
      esd_poll_stop(crucialtec_oj_check_esd);
      oj_data.esd_polling = 0;
   }
#endif

   kfree(oj);
   
   return 0;
}



#ifdef CONFIG_CRUCIALTEC_OJ_ESD_RECOVERY
/*==================================================================================================

FUNCTION: crucialtec_oj_detect_esd

DESCRIPTION: 
   This function determines if the OJ's registers is invalid or not.

ARGUMENTS PASSED:
   None
    
REFERENCE ARGUMENTS PASSED:
   None

RETURN VALUE:
   true  - OJ's registers is invalid
   false - OJ's registers is correct

DEPENDENCIES:
   None

SIDE EFFECTS:
   None

==================================================================================================*/
bool crucialtec_oj_detect_esd(void)
{
   uint8_t engine_reg_value = 0;
   uint8_t orientation_ctrl = 0;

   crucialtec_oj_read_register(oj_data.client, 
                               CRUCIALTEC_OJ_REG_ENGINE, 
                               &engine_reg_value);

   crucialtec_oj_read_register(oj_data.client, 
                               CRUCIALTEC_OJ_REG_ORIENTATION_CTRL, 
                               &orientation_ctrl);
	
	if ((engine_reg_value != CRUCIALTEC_OJ_REG_ENGINE_VALUE_DEFAULT) ||
	    (orientation_ctrl != CRUCIALTEC_OJ_REG_ORIENTATION_CTRL_VALUE_DEFAULT))
	{
	   printk (KERN_ERR "[OTP] %s: invalid register value detected\n", __FUNCTION__);
		
		return true;
	}
	
	return false;
}


/*==================================================================================================

FUNCTION: crucialtec_oj_fix_esd

DESCRIPTION: 
   This function resets the OJ and configures the neccessary registers.

ARGUMENTS PASSED:
   None
    
REFERENCE ARGUMENTS PASSED:
   None

RETURN VALUE:
   None

DEPENDENCIES:
   None

SIDE EFFECTS:
   None

==================================================================================================*/
void crucialtec_oj_fix_esd(void)
{
	// Perform a sof reset
	crucialtec_oj_write_register(oj_data.client, 
                                CRUCIALTEC_OJ_REG_SOFT_RESET, 
                                CRUCIALTEC_OJ_REG_SOFT_RESET_VALUE_REVERT_TO_DEFAULT);

   // Waiting for the driver to be ready
   msleep_interruptible(CRUCIALTEC_OJ_DELAY_FOR_SYSTEM_TO_BE_READY_AFTER_SOFT_RESET);

   oj_enable_irq();

   // Configure the registers
	crucialtec_oj_configure_registers(oj_data.client);

}


/*==================================================================================================

FUNCTION: crucialtec_oj_fix_esd

DESCRIPTION: 
   This function checks for invalid values in the OJ's registers and correct them.

ARGUMENTS PASSED:
   None
    
REFERENCE ARGUMENTS PASSED:
   None

RETURN VALUE:
   None

DEPENDENCIES:
   None

SIDE EFFECTS:
   None

==================================================================================================*/
void crucialtec_oj_check_esd(void* arg)
{
   if(crucialtec_oj_detect_esd() == true)
	{
	    crucialtec_oj_fix_esd();
	}
}

#endif  /* CONFIG_CRUCIALTEC_OJ_ESD_RECOVERY */


static const struct i2c_device_id crucialtec_oj_id[] = {
   {"otp_device", 0},
   {},
};

static struct i2c_driver crucialtec_oj_driver = {
   .probe    = crucialtec_oj_probe,
   .remove   = crucialtec_oj_remove,
   .suspend  = crucialtec_oj_suspend,
   .resume   = crucialtec_oj_resume, 

   .id_table = crucialtec_oj_id,
   .driver   = {
      .name  = "otp_device",
   },
};

static int __devinit crucialtec_oj_init(void)
{

   printk(KERN_INFO "[OTP] crucialtec_oj_init\n");  

   return i2c_add_driver(&crucialtec_oj_driver);;
}


static void __exit crucialtec_oj_exit(void)
{

   printk(KERN_INFO "[OTP] crucialtec_oj_exit\n");      
   
   i2c_del_driver(&crucialtec_oj_driver);
}


module_init(crucialtec_oj_init);
module_exit(crucialtec_oj_exit);


MODULE_DESCRIPTION("CRUCIALTEC OPTICAL JOYSTICK DRIVER");
MODULE_AUTHOR("CRUCIALTEC");
MODULE_LICENSE("GPL");
