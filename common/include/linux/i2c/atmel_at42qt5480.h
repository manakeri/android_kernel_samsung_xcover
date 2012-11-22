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

#endif /* _ATMEL_AT42QT5480_H */
